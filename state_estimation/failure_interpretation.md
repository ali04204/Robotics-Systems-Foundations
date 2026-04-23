# Failure Interpretation — reading what the EKF is telling you

The EKF is usually trying to warn you before it fully diverges. This page
maps the five canonical failure signals to causes and fixes.

## 1. Covariance rising

**What it means**
- The filter's uncertainty `P` is growing without bound.
- Predict is adding `Q`; updates are not bringing `P` back down.

**What it looks like in behavior**
- Estimate drifts smoothly but never snaps to the sensor.
- Diagonal entries of `P` keep climbing over time.
- Innovation values look fine, but `K` stays tiny.

**Likely cause**
- No measurements are arriving (topic dropped, driver crashed).
- `R` is configured so large that the sensor is effectively ignored.
- Innovation gating is rejecting every measurement.

**Likely fix**
- Confirm sensor topic publishing rate with `ros2 topic hz`.
- Sanity-check `R` against the sensor datasheet.
- Relax or log the gating threshold.

## 2. Estimator lag

**What it means**
- The estimate follows the truth with a time delay.
- Filter is too conservative about responding to new measurements.

**What it looks like in behavior**
- During a turn, heading trails behind IMU yaw rate.
- During acceleration, `v` updates slower than the real motion.
- Innovations have a visible bias correlated with motion phase.

**Likely cause**
- `Q` too small on the lagging state (heading or velocities).
- `R` too large on the sensor that would correct the lag.
- Unmodelled acceleration (you model `v_dot = 0` but the robot is accelerating).

**Likely fix**
- Raise `Q[theta, theta]`, `Q[v, v]`, or `Q[w, w]` moderately.
- Drop `R` on the IMU or odom channel responsible for that state.
- If lag is structural, add acceleration to the model (only if needed).

## 3. Stale sensor input

**What it means**
- Measurements are arriving with timestamps from the past.
- The filter is applying old corrections to a newer state.

**What it looks like in behavior**
- Sudden jumps in the estimate that correlate with sensor re-delivery.
- Innovation spikes every time a lagged burst of data lands.
- Estimate oscillates around the truth instead of tracking it.

**Likely cause**
- Clock skew between sensor driver and estimator node.
- Transport lag (USB / Wi-Fi / large serialization).
- ROS 2 QoS or buffering issue.

**Likely fix**
- Check `header.stamp` vs `rclcpp::Clock` / wall time at receipt.
- Drop measurements older than a sanity threshold.
- In `robot_localization`, use `transform_time_offset` or the
  `*_differential` / latest-timestamp guards.

## 4. Overconfident covariance

**What it means**
- `P` is smaller than it should be for the actual uncertainty.
- Filter believes itself more than is warranted.

**What it looks like in behavior**
- Estimate resists new measurements — looks locked to its own trajectory.
- Innovations are large but the state barely moves per update.
- After a disturbance (kick, slip), recovery takes many seconds.

**Likely cause**
- `Q` too small for the real process noise.
- Initial `P_0` set too small and the filter never grows back.
- Sensor `R` too small, previously driving `P` down too aggressively.

**Likely fix**
- Increase `Q` — especially on the states you expect to drift.
- Start with a larger `P_0`.
- Sanity-check whether the sensor is really that good, or just seemed to be
  in the test environment.

## 5. Drift without correction

**What it means**
- The estimate is dead-reckoning and never gets pulled back to ground truth.
- All sensors in the filter are themselves relative (twist, yaw rate).

**What it looks like in behavior**
- Short-term motion looks perfect; after minutes, position is far from truth.
- Heading slowly rotates relative to the world.
- Nothing is obviously "wrong" — filter is consistent with its inputs.

**Likely cause**
- No absolute reference in the fusion (no AMCL, GPS, landmark, or map-based
  correction).
- Gyro bias not being estimated.

**Likely fix**
- Add an absolute pose source (AMCL, fiducial marker, GPS) for long-term
  anchoring — the EKF can run as local, the global correction can be a
  separate filter (this is the standard two-EKF `robot_localization` setup:
  odom→odom local, map→odom global).
- Add gyro bias as a state if you run for long periods.