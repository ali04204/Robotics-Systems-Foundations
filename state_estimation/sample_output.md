# Sample output

Captured directly from running the scripts in this folder on a clean
machine with NumPy installed.

## `ekf_planar_reference.py`

```
=== ekf_planar_reference.py : one predict + update cycle ===
Prior state   : [0.  0.  0.  0.5 0.1]
Prior P shape : (5, 5)
Predicted x   : [0.05 0.   0.01 0.5  0.1 ]
Odom z        : [0.52 0.09]
Innovation y  : [ 0.02 -0.01]
Kalman gain K shape : (5, 2)
Posterior x   : [0.05189753 0.         0.00906977 0.5199241  0.09023256]
Posterior P shape : (5, 5)
Posterior diag(P) : [0.10015123 0.100225   0.05106977 0.00039848 0.00244186]
```

What each line means:

- **Prior state** — initial `[x, y, theta, v, w]`, robot at origin moving
  at 0.5 m/s and rotating at 0.1 rad/s.
- **Predicted x** — after one 0.1 s predict step: x moved by `v*dt = 0.05 m`,
  theta by `w*dt = 0.01 rad`. `v` and `w` unchanged (random-walk model).
- **Odom z** — synthetic odom reading slightly above the predicted twist.
- **Innovation y** — `z - h(x_pred) = [+0.02, -0.01]`. Small, consistent with
  a sensor that is a bit off the prediction.
- **Posterior x** — after the update, `v` pulled up to ~0.52 toward the
  measurement, `w` pulled down to ~0.09. Position barely changed because
  this step only measures velocities.
- **Posterior diag(P)** — uncertainties on `v` and `w` dropped sharply
  (0.1 → ~0.0004, ~0.0024). Position and heading still uncertain; they need
  their own kind of measurement (or time + motion coupling) to shrink.

## `ekf_demo.py`

```
=== EKF demo : synthetic planar trajectory ===
Steps=20, dt=0.1s, cmd=(v=0.50, w=0.20)

  k | true x   y   theta  |  est x    y    theta  |  odom_y      imu_y
----------------------------------------------------------------------------------------
  1 | +0.050 +0.000 +0.020 | +0.050 +0.000 +0.021 | [+0.504, +0.189]  [+0.026]
  2 | +0.100 +0.001 +0.040 | +0.100 +0.001 +0.042 | [+0.000, -0.054]  [+0.021]
  4 | +0.200 +0.006 +0.080 | +0.203 +0.006 +0.082 | [-0.072, -0.042]  [+0.028]
  6 | +0.299 +0.015 +0.120 | +0.294 +0.015 +0.119 | [+0.042, -0.021]  [+0.026]
  8 | +0.399 +0.028 +0.160 | +0.392 +0.027 +0.158 | [+0.034, -0.055]  [+0.030]
 10 | +0.497 +0.045 +0.200 | +0.496 +0.045 +0.197 | [-0.056, -0.024]  [+0.028]
 12 | +0.595 +0.066 +0.240 | +0.589 +0.065 +0.237 | [+0.045, +0.021]  [+0.001]
 14 | +0.692 +0.090 +0.280 | +0.686 +0.089 +0.279 | [+0.060, -0.114]  [+0.069]
 16 | +0.788 +0.119 +0.320 | +0.789 +0.120 +0.323 | [-0.049, +0.107]  [-0.019]
 18 | +0.882 +0.151 +0.360 | +0.887 +0.154 +0.368 | [-0.083, -0.015]  [+0.005]
 20 | +0.975 +0.188 +0.400 | +0.975 +0.189 +0.410 | [+0.058, -0.105]  [+0.024]

=== Final ===
True    x = [0.97548688 0.18760548 0.4        0.5        0.2       ]
Estimated x = [0.97464335 0.18895274 0.41009113 0.51310478 0.18382194]
Error      = [ 0.00084353 -0.00134726 -0.01009113 -0.01310478  0.01617806]
diag(P)    = [0.05332492 0.07495943 0.04013168 0.00077872 0.00035173]

Notes:
  * Innovations (odom_y, imu_y) should look like zero-mean noise.
  * Position error is small because heading is well corrected.
  * Long-term drift is NOT bounded without a global sensor.
```

What each section means:

- **Header** — configuration of the run: 20 steps of 0.1 s each, constant
  commanded twist (`v=0.5 m/s`, `w=0.2 rad/s`).
- **Per-step rows** — printed every other step. Columns:
  - `true x y theta` : ground truth pose from the simulator.
  - `est x y theta`  : filter's posterior estimate after predict + 2 updates.
  - `odom_y`         : odom innovation `z_odom - h_odom(x_pred) = [dv, dw]`.
    The first step shows a big innovation (~0.5 for v) because the filter
    started with `v = 0` while truth commanded `v = 0.5`.
  - `imu_y`          : IMU yaw-rate innovation, usually near zero after the
    first couple of steps.
- **Final block** — side-by-side of truth vs estimate, the per-state error,
  and `diag(P)` (remaining uncertainty). Position error is millimetre-scale
  over the short run; heading and velocity errors are ~1% — expected for a
  simple filter with moderate noise.

## `ekf_smoke_test.py`

```
EKF smoke test passed
```

Prints exactly one line when everything imports cleanly and all shape /
symmetry assertions hold.

## Why this is only a learning demo

- Ground truth is available because the trajectory is synthetic — a real
  robot has no such oracle.
- Sensor noise is Gaussian and zero-mean; real encoders and gyros have
  bias, temperature drift, slip, and outliers that would require extra
  states and gating.
- There is no global correction (AMCL, GPS, fiducials), so long runs would
  drift without bound — the "Error" column stays small only because the
  run is short.
- `Q` and `R` are pedagogically sane, not calibrated for any specific
  robot platform.
- Treat the filter here as a structural template, not a production
  estimator. For real robots, use `robot_localization` and apply the
  tuning guidance from `tuning_notes.md` and `robot_localization_bridge.md`.