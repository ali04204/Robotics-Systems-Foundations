# EKF Pseudocode — planar AMR

Readable, implementation-oriented pseudocode. No heavy notation.

## Symbols

|
 Symbol 
|
 Meaning 
|
|
--------
|
---------
|
|
`x`
|
 state vector (best estimate) 
|
|
`P`
|
 state covariance (uncertainty) 
|
|
`u`
|
 control / input (from commands or odom velocities) 
|
|
`z`
|
 measurement vector (from a sensor) 
|
|
`f(x, u, dt)`
|
 nonlinear process model 
|
|
`h(x)`
|
 nonlinear measurement model 
|
|
`F`
|
 Jacobian of 
`f`
 w.r.t. 
`x`
|
|
`H`
|
 Jacobian of 
`h`
 w.r.t. 
`x`
|
|
`Q`
|
 process noise covariance 
|
|
`R`
|
 measurement noise covariance 
|
|
`K`
|
 Kalman gain 
|
|
`y`
|
 innovation / residual (
`z - h(x)`
) 
|
|
`S`
|
 innovation covariance 
|

## State definition (default)

```
x = [ x_pos, y_pos, theta, v, w ]^T    # 5x1
```

A 3D alternative is `[x_pos, y_pos, theta]` when you treat odom `(v, w)` as
inputs instead of estimated states.

## Input definition

Treated as control-like input `u`:

```
u = [ v_cmd, w_cmd ]     # from wheel odometry velocities or commanded twist
```

For the 5D state, you may also skip `u` entirely and just propagate `v` and
`w` as random-walk states driven by `Q`.

## Measurement definition

Two independent measurement streams, each with its own `H` and `R`:

```
z_odom = [ v_meas, w_meas ]        # body-frame twist from wheel encoders
z_imu  = [ w_meas_imu ]            # yaw rate from IMU gyroscope
```

IMU linear acceleration is optional and noisy on mobile robots; usually
skipped for planar indoor AMRs unless you need slip detection.

## Process model (unicycle, forward Euler)

```
x_pos'   = x_pos  + v * cos(theta) * dt
y_pos'   = y_pos  + v * sin(theta) * dt
theta'   = theta  + w * dt
v'       = v            # random-walk: constant with process noise
w'       = w            # random-walk: constant with process noise
```

Assumptions:
- planar motion, flat ground
- no slip (lateral velocity = 0)
- small `dt` so Euler is acceptable
- `v` and `w` drift slowly compared to sampling rate

## Prediction step

```
x_pred = f(x, u, dt)
F      = df/dx  evaluated at (x, u, dt)
P_pred = F * P * F^T + Q
```

## Measurement update step

For each sensor independently:

```
y = z - h(x_pred)                    # innovation
H = dh/dx  evaluated at x_pred
S = H * P_pred * H^T + R             # innovation covariance
K = P_pred * H^T * S^-1              # Kalman gain
x_new = x_pred + K * y
P_new = (I - K * H) * P_pred
```

Run update for odometry, then for IMU (order does not matter mathematically
but can matter numerically).

## Covariance propagation intuition

- `Q` **grows** `P` during predict — model is never perfect.
- `R` **resists shrinking** `P` during update — sensor is never perfect.
- If `R` is small vs `P`, `K` is large → estimate jumps to measurement.
- If `R` is large vs `P`, `K` is small → estimate barely moves.

## Innovation / residual

`y = z - h(x_pred)` is the **single best diagnostic signal** in an EKF.

- Should look like zero-mean noise scaled by `sqrt(diag(S))`.
- Persistent bias in `y` = model wrong, sensor biased, or `R` mis-sized.
- Growing `|y|` = filter is diverging or sensor is stale.

## Kalman gain intuition

- `K in [0, 1]` per channel (informally).
- Think of it as: "what fraction of the innovation should I apply?"
- It is automatically computed from `P`, `H`, `R`.

## Compact EKF loop (from memory)

```
# 1. predict
x_pred = f(x, u, dt)
F      = jacobian_f(x, u, dt)
P_pred = F @ P @ F.T + Q

# 2. for each sensor z with model h, jacobian H, noise R:
y = z - h(x_pred)
S = H @ P_pred @ H.T + R
K = P_pred @ H.T @ inv(S)
x  = x_pred + K @ y
P  = (I - K @ H) @ P_pred

# 3. wrap angles, loop
```

That is the whole EKF. Everything else is choosing `f`, `h`, `Q`, `R`.