# stateestimation

Block 2 of the AMR study guide: practical state estimation for a planar mobile
robot using wheel odometry + IMU. This folder is built for **studying and
memorising an EKF quickly**, not for production use.

## What this folder is for

- Fast revision of Extended Kalman Filter (EKF) structure before coding.
- A practical reference tying wheel odometry + IMU to a planar AMR state.
- Minimal runnable Python that mirrors what `robot_localization` does
  conceptually, without being a clone of it.

## Chosen state

Default state vector (5D):

```
x = [x, y, theta, v, w]
```

- `x, y`    : planar position in the world frame (m)
- `theta`   : heading / yaw (rad)
- `v`       : body-frame linear speed along the heading (m/s)
- `w`       : body-frame yaw rate (rad/s)

A simpler 3D state `[x, y, theta]` is fine when you trust wheel odometry
velocities directly as inputs and do not want to estimate them.

## Files

|
 File 
|
 Purpose 
|
|
------
|
---------
|
|
`ekf_pseudocode.md`
|
 Clean EKF pseudocode + 8–12 line loop from memory 
|
|
`state_and_models.md`
|
 State choice, process model, measurement model, Jacobians 
|
|
`sensor_inputs.md`
|
 Wheel odom / IMU signal mapping, good/bad qualities, failure modes 
|
|
`tuning_notes.md`
|
 Process / IMU / odom covariance in practice + diagnostic symptoms 
|
|
`failure_interpretation.md`
|
 Covariance rising, lag, stale input, overconfidence, drift 
|
|
`robot_localization_bridge.md`
|
 Mapping these concepts to ROS 2 
`robot_localization`
|
|
`sample_output.md`
|
 Captured demo output + what each section means 
|
|
`ekf_planar_reference.py`
|
 Educational reference EKF (predict / update primitives) 
|
|
`ekf_demo.py`
|
 Short planar trajectory + synthetic odom + IMU demo 
|
|
`ekf_smoke_test.py`
|
 Tiny sanity script that prints 
`EKF smoke test passed`
|

## Quick EKF intuition (6–10 lines)

1. Keep a **state estimate** `x` and a **covariance** `P` (how unsure you are).
2. **Predict**: push `x` forward through the motion model, grow `P` by `Q`.
3. **Measure**: compare sensor reading `z` to predicted reading `h(x)`.
4. That difference is the **innovation** `y = z - h(x)`.
5. The **Kalman gain** `K` decides how much to trust the sensor vs the model.
6. **Update**: `x <- x + K y`, and shrink `P` accordingly.
7. If sensor is noisy (big `R`), `K` is small → trust the model more.
8. If model is bad (big `Q`), `K` is large → trust the sensor more.

## Why wheel odometry alone is not enough

- Wheel odometry drifts: wheel slip, uneven floor, calibration error.
- Heading error accumulates fastest and is **unbounded** over time.
- It has no absolute reference; a straight path can curve in the estimate.
- IMU yaw rate corrects short-term heading; a global sensor (GPS, AMCL, lidar
  localization) is still needed to bound long-term drift.
- The EKF **fuses** them: odom gives good short-term translation, IMU gives
  good short-term rotation, covariances decide who wins.

## How to run the demo

```bash
cd stateestimation
python3 ekf_planar_reference.py   # one predict/update cycle, prints shapes
python3 ekf_demo.py               # short synthetic trajectory with fusion
python3 ekf_smoke_test.py         # prints: EKF smoke test passed
```

Only dependency: NumPy.

## What this demo proves and what it does not prove

**Proves**

- The predict / update structure is sound.
- State and covariance shapes stay consistent across steps.
- The fusion reduces noise vs raw sensor values on a known trajectory.

**Does not prove**

- That the Jacobians are the absolute best form (they are first-order and
  kept intentionally simple).
- That the tuning is correct for any real robot — `Q` and `R` are educational
  defaults.
- That the filter is robust to real-world failures (slip, bias drift, latency,
  frame errors). Treat it as a study artefact, not a library.