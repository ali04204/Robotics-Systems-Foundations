# Tuning Notes — practical EKF tuning for a planar AMR

Tuning is mostly choosing `Q` and `R`. Everything else follows.

## What process noise `Q` means in practice

- `Q` is "how much I **don't** trust my motion model per step."
- It **grows** `P` during predict, telling the filter the estimate got
  blurrier just from time passing.
- Bigger `Q` → filter trusts measurements more → reacts faster, more jittery.
- Smaller `Q` → filter trusts the model more → smoother, but laggy and can
  diverge from truth.

Rule of thumb: set `Q` so that, after one predict step with no update, the
covariance grows by roughly the amount the real state could drift.

Per-state intuition for `[x, y, theta, v, w]`:

- `Q_xx, Q_yy` small — position only drifts because of model integration
  error; this is already captured through `v` and `theta`.
- `Q_theta` small-ish — heading drifts because of unmodelled slip.
- `Q_vv, Q_ww` larger — these states are driven by `Q` (we model them as
  near-constant), so `Q` here captures real acceleration.

## What IMU covariance `R_imu` means in practice

- `R_imu` is "how much I **don't** trust the gyro reading."
- Small `R_imu` → gyro measurement dominates `w` and `theta`.
- Large `R_imu` → gyro is ignored, filter trusts its own model / odom.

Starting value: use the gyro datasheet noise density, squared and scaled
by sample rate, as a rough floor. Increase it if vibration dominates.

## What odometry covariance `R_odom` means in practice

- `R_odom` is "how much I **don't** trust wheel velocities."
- Usually **larger than `R_imu` for `w`** because encoders are slip-prone.
- Usually smaller than `R_imu` for `v` — encoders measure linear speed well.

This asymmetry is the whole reason fusion helps: each sensor is trusted
where it is actually good.

## Parameters I would likely tune first

Priority order:

1. `R_odom[w, w]` — raise it if turning estimate is jittery or wrong during
   slip-prone moves.
2. `R_imu[w, w]` — raise it if estimate jitters during vibration.
3. `Q[theta, theta]` — raise slightly if heading lags behind truth on turns.
4. `Q[v, v]` and `Q[w, w]` — raise if velocity estimates lag behind real
   acceleration.
5. Initial `P_0` — set reasonably large so the filter does not start
   overconfident; it will converge on its own.

Leave `Q[x, x]` and `Q[y, y]` tiny. Position uncertainty should grow via
the Jacobian coupling `F @ P @ F.T`, not directly through `Q`.

## Symptoms of bad tuning

### Covariance too small (overconfident filter)

- Estimate stops moving toward measurements.
- Innovations stay large and do not shrink.
- Looks like a frozen / lagging estimate — gain `K` is too small.
- Fix: increase `Q` (or `R` for a single overtrusted sensor is *smaller*,
  not larger — confirm which side is overconfident).

### Covariance too large

- Estimate matches the latest measurement too closely — jittery, noisy.
- `P` never decreases meaningfully.
- Fix: reduce `Q`, tighten `R` if a specific sensor is being distrusted
  despite being accurate.

### Estimator lag

- Estimate trails the real trajectory during turns and accelerations.
- Innovations have a **bias** correlated with motion phase.
- Fix: raise `Q` on velocity/yaw-rate states, or include acceleration in
  the model.

### Smooth but wrong estimate

- Trajectory looks clean but drifts from reality.
- Innovations are small but non-zero on average.
- Likely cause: miscalibration or sensor bias (e.g., wheel radii, gyro
  bias), not a pure tuning issue. Calibrate first, then tune.

### Covariance rising unbounded

- Predict step keeps growing `P`, updates are not bringing it down.
- Likely cause: sensor dropped, innovation gating rejecting everything, or
  `R` so large that `K` ≈ 0.
- Fix: confirm sensor is arriving, check timestamps, lower `R` toward the
  datasheet value.

## Quick diagnostic checklist

1. Are the **innovations zero-mean**?
2. Are the innovations **the size of `sqrt(diag(S))`**? (Not 10×, not 0.1×.)
3. Does `P` **shrink on updates** and **grow on predicts**?
4. Is the estimate **between** the prediction and the measurement? (It should
   be — if it is outside that range, check signs and frames.)
5. Are **timestamps monotonic** and close to wall-clock time?

If all five are yes, your tuning is probably already in the right ballpark.