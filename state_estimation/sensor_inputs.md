# Sensor Inputs — what each signal actually gives you

Practical mapping for a planar AMR fusing wheel odometry + IMU.

## Quick map

|
 Source 
|
 Signal 
|
 Maps to state via 
|
 Good at 
|
 Bad at 
|
|
--------
|
--------
|
-------------------
|
---------
|
--------
|
|
 Wheel encoders 
|
 linear speed 
`v_meas`
|
 measurement of 
`v`
|
 short-term distance, cheap, high rate 
|
 slip, wheel wear, flat battery / voltage effects 
|
|
 Wheel encoders 
|
 angular speed 
`w_meas`
|
 measurement of 
`w`
|
 short-term turning when not slipping 
|
 slip during fast turns, asymmetric wheels 
|
|
 Wheel-based pose 
|
`x, y, theta`
 integrated 
|
 (avoid as measurement) 
|
 smooth trajectory for viz 
|
 unbounded drift — do not feed back as pose 
|
|
 IMU gyro z 
|
 yaw rate 
`w_gyro`
|
 measurement of 
`w`
|
 clean, high-rate, slip-immune 
|
 bias drift over minutes, temperature 
|
|
 IMU accel 
|
`a_x, a_y, a_z`
|
 indirect; derivative of 
`v`
|
 detecting bumps / slip events 
|
 gravity projection, vibration, heavy low-pass needed 
|
|
 IMU magnetometer 
|
 heading reference 
|
 (optional absolute 
`theta`
) 
|
 long-term heading anchor 
|
 indoor magnetic distortion, not trustworthy 
|

## Wheel odometry — what it actually gives

- **Primary**: body-frame linear and angular velocity `(v, w)` from wheel
  radii + wheel separation + encoder rates.
- **Secondary**: integrated pose `(x, y, theta)` in an odom frame.
- Used as a **measurement** in this EKF design (`z_odom = [v, w]`), not as
  pose, because pose is pre-integrated drift.

### Good at

- High update rate (usually ≥ 50 Hz).
- Very accurate over short distances on flat, high-friction floors.
- Gives a direct read of body-frame twist — no integration needed.

### Bad at

- Wheel slip (spin on dust, carpet, tile cracks) silently corrupts `v` and
  especially `w`.
- Systematic bias if wheel radii / baseline are miscalibrated — hard to see.
- Heading error accumulates forever; after a few minutes, pose is unusable
  alone.

## IMU angular rate (gyro z)

- **What**: instantaneous yaw rate of the chassis.
- **Role**: measurement of `w`.

### Good at

- Not affected by wheel slip — pure kinematic measurement.
- Very high rate (100–500+ Hz typical) with low short-term noise.
- Best sensor for detecting turning onset quickly.

### Bad at

- Bias drift: gyro output is non-zero when the robot is still. Needs a
  bias-estimation state in production filters (skipped here).
- Temperature and vibration sensitivity.
- Integrated yaw drifts with time — gyro is a **rate** sensor, never a
  direct pose sensor.

## IMU linear acceleration (optional)

- **What**: body-frame accelerations `a_x, a_y, a_z` including gravity.
- **Role**: could correct `v` via its derivative, but usually skipped on
  indoor planar AMRs.

### Good at

- Detecting impacts, sudden decelerations, wheel-slip events as a flag.
- Short-term cross-check against commanded acceleration.

### Bad at

- Gravity projection must be removed using pitch/roll — a planar AMR has no
  filter state for those, so errors leak directly.
- Chassis vibration dominates the signal at small speeds.
- Double integration to position is never useful — it explodes fast.

## Typical failure modes to recognise

|
 Mode 
|
 Looks like 
|
 Who causes it 
|
|
------
|
------------
|
---------------
|
|
 Drift 
|
 pose slowly wanders from truth 
|
 all dead-reckoning sensors 
|
|
 Bias 
|
 constant offset in 
`w`
 or 
`a`
|
 gyro, accelerometer 
|
|
 Noise 
|
 jittery estimate 
|
 encoders on rough floor, IMU vibration 
|
|
 Stale data 
|
 estimator updates slow or skips 
|
 transport lag, dropped frames 
|
|
 Slip 
|
 odom says turning but IMU says not 
|
 wheel on dust / carpet / obstacle 
|
|
 Vibration 
|
 accel huge, w_imu noisy 
|
 motor resonance, loose mounting 
|
|
 Time skew 
|
 innovation jumps correlate with steps 
|
 bad timestamps / clock skew 
|

## Practical signal-source rules

1. Trust gyro more than encoder `w` at high angular rates and during slip.
2. Trust encoder `v` more than accelerometer-integrated speed, always.
3. Never feed odom-integrated pose as a measurement — it double-counts
   the same drift you are trying to estimate.
4. Timestamps matter: if you cannot trust them to ~10 ms, the filter will
   look like it has noise problems even with perfect `Q` and `R`.