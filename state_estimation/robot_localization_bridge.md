# Bridge to ROS 2 `robot_localization`

This file maps the EKF concepts from the rest of this folder onto the
`robot_localization` package so the intuition transfers directly.

## Concept mapping

|
 This folder 
|
`robot_localization`
|
|
-------------
|
----------------------
|
|
 State 
`[x, y, theta, v, w]`
|
 A subset of the 15D state 
`[x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az]`
|
|
`f(x, u, dt)`
 unicycle 
|
 Built-in omnidirectional motion model (enable only the DOFs you want) 
|
|
`process_jacobian`
|
 Internal to 
`ekf_node`
|
|
`Q`
|
`process_noise_covariance`
 matrix in YAML 
|
|
`R_odom`
|
`covariance`
 field on 
`nav_msgs/Odometry`
 messages 
|
|
`R_imu`
|
`*_covariance`
 fields on 
`sensor_msgs/Imu`
 messages 
|
|
`H`
 selector matrices 
|
 Boolean fusion config, e.g. 
`odom0_config`
|
|
 Innovation 
`y`
|
 Published on 
`~/diagnostics`
 and state topics 
|

## Which covariances show up in which messages

### `nav_msgs/Odometry`

- `pose.covariance` — 6×6, for `(x, y, z, roll, pitch, yaw)`.
- `twist.covariance` — 6×6, for `(vx, vy, vz, vroll, vpitch, vyaw)`.

For a planar AMR, only the diagonal entries that you actually fuse should be
populated with real values. Set the unused rows/cols to large numbers (e.g.
`1e6`) so the filter ignores them. Do **not** leave them at `0` — zero in
ROS covariance fields often means "unknown" and will be rejected or trusted
inappropriately.

### `sensor_msgs/Imu`

- `orientation_covariance` — 3×3 for roll/pitch/yaw. If the IMU is not
  reporting orientation, set element 0 to `-1` per the message convention.
- `angular_velocity_covariance` — 3×3. For a planar AMR, the yaw entry is
  the one that actually matters.
- `linear_acceleration_covariance` — 3×3. Only populate if you actually
  fuse accelerations.

## Why frame consistency matters

- `robot_localization` expects `odom` as a smoothly drifting local frame and
  `map` as the globally anchored frame.
- The rule is: **odom must be continuous; map may jump.**
- If you mis-wire an absolute pose source into the local EKF, the local
  frame will jump — downstream control and costmaps will judder.
- TF frames in your IMU and odom messages must match what the EKF expects,
  otherwise your measurement vector is silently rotated and the filter
  produces beautifully smooth garbage.

## Why stale timestamps matter

- The EKF associates each measurement with a time so it can apply the
  correct prediction step.
- If a message arrives with an old `header.stamp`, `robot_localization`
  may:
  - reject it (default),
  - roll the filter back and replay (expensive), or
  - apply it at current time (wrong).
- Large clock skew (even 50 ms on a mobile robot during fast turns) shows
  up as unexplained noise in the estimate.
- Always use the real sensor timestamp, not the time of receipt. On micro
  ROS / embedded IMU drivers, sync the clock first.

## What I actually tune first in `robot_localization`

Order of attack, in priority:

1. **Which DOFs to fuse per sensor** (`odomN_config`, `imuN_config`)
   - Fuse wheel odom as **twist only** (`vx`, `vyaw`), not pose.
   - Fuse IMU as yaw rate only, unless you have a good orientation source.
2. **Sensor message covariances**
   - Populate only the rows that matter; inflate the rest.
   - Start with numbers close to datasheet noise, then tweak.
3. **`process_noise_covariance`**
   - Small on `x, y, yaw` diagonals; larger on `vx, vyaw` to let the filter
     track acceleration events.
4. **`initial_estimate_covariance`**
   - Leave generous so the filter converges from startup.
5. **`frequency`**
   - Match or slightly exceed the fastest sensor rate.
6. **`two_d_mode: true`**
   - Locks roll, pitch, z and their derivatives to zero — essential for a
     planar AMR, saves tuning a dozen irrelevant entries.
7. **Differential mode** (`odomN_differential`, `imuN_differential`)
   - Turn on only if the absolute pose is known to drift; otherwise leave
     off.

If the estimator is still wrong after step 3, the problem is almost
certainly **frames or timestamps**, not tuning.