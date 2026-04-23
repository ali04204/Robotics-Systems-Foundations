# State and Models — planar AMR

## Chosen planar state (default)

```
x = [ x, y, theta, v, w ]^T
```

**Why this state:**

- `x, y, theta` are the pose — what localization actually cares about.
- `v, w` are included so the filter can smooth noisy velocity measurements
  and let `Q` handle small acceleration events implicitly.
- Matches the natural output of a unicycle / differential-drive AMR.
- Small enough (5D) to reason about and debug by hand.

## Alternative simpler state

```
x = [ x, y, theta ]^T
```

Acceptable when:

- Wheel odometry velocities are clean and you trust them as inputs, not
  measurements.
- You do not need smoothed velocity estimates for downstream control.
- You want the absolute minimum state to debug fast.

Trade-off: velocity noise feeds straight into position through the process
model, and you lose the ability to fuse IMU yaw rate as a measurement of
the state `w`.

## Process model (unicycle, forward Euler)

Continuous form:

```
x_dot     = v * cos(theta)
y_dot     = v * sin(theta)
theta_dot = w
v_dot     = 0           (random walk, driven by Q)
w_dot     = 0           (random walk, driven by Q)
```

Discrete form with step `dt`:

```
x_k+1     = x_k     + v_k * cos(theta_k) * dt
y_k+1     = y_k     + v_k * sin(theta_k) * dt
theta_k+1 = theta_k + w_k * dt
v_k+1     = v_k
w_k+1     = w_k
```

**Assumptions**

- Flat ground, no z / roll / pitch motion.
- No slip — lateral velocity is 0 in the body frame.
- `dt` small enough that Euler ≈ exact integration.
- `v`, `w` change slowly relative to the sampling rate.

## Measurement models

### Wheel odometry (body-frame twist)

Encoders + wheel kinematics give linear and angular velocity:

```
z_odom  = [ v_meas,  w_meas ]
h_odom(x) = [ v, w ]
H_odom    = [[0, 0, 0, 1, 0],
             [0, 0, 0, 0, 1]]
```

Wheel odometry can also publish pose, but that pose is just integrated
twist — feeding it as a measurement typically leaks drift into the filter.
Prefer twist.

### IMU yaw rate

Gyroscope on the z-axis:

```
z_imu  = [ w_gyro ]
h_imu(x) = [ w ]
H_imu    = [[0, 0, 0, 0, 1]]
```

### IMU linear acceleration (optional)

Rarely useful for indoor planar AMRs: gravity projection, chassis vibration,
and bias make it noisy and slow-reacting. If used, it would update `v` via a
derivative relationship, not directly — usually not worth it.

## Control-like input vs measurement

There are two valid designs; pick one and be consistent.

|
 Design 
|
`v, w`
 come from 
|
`v, w`
 role 
|
|
--------
|
------------------
|
-------------
|
|
 A (input) 
|
 commanded twist or clean odom 
|
 used inside 
`f(x, u, dt)`
 as 
`u`
|
|
 B (measured, default here) 
|
 wheel odom published as twist 
|
 used as measurement 
`z_odom`
|

Design B is preferred when odom is noisy, because the filter can model that
noise via `R_odom`. Design A is preferred when commands are very reliable
and odom is unavailable.

## Simple Jacobian descriptions

### Process Jacobian `F`

Partial derivatives of the discrete process model w.r.t. state. Only the
position equations are nonlinear (because of `cos` and `sin`).

```
F = d f / d x

      x   y   theta                 v              w
x  [  1   0  -v*sin(theta)*dt   cos(theta)*dt     0       ]
y  [  0   1   v*cos(theta)*dt   sin(theta)*dt     0       ]
th [  0   0   1                 0                 dt      ]
v  [  0   0   0                 1                 0       ]
w  [  0   0   0                 0                 1       ]
```

### Measurement Jacobians

Both are linear → Jacobian is literally the selector matrix:

```
H_odom = [[0, 0, 0, 1, 0],
          [0, 0, 0, 0, 1]]

H_imu  = [[0, 0, 0, 0, 1]]
```

Linear `h` means the EKF update reduces to a standard KF update for these
sensors — easy to debug.

## Assumptions and limitations

- Yaw is represented as a plain scalar; wrap to `[-pi, pi]` after update.
- No IMU bias state — real robots need it, we skip it to stay minimal.
- No global correction (no AMCL / GPS), so absolute drift is unbounded.
- Noise is assumed Gaussian and zero-mean — convenient, often wrong in
  practice. Sanity-check innovations to catch violations.