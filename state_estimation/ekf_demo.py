"""
Minimal EKF demo for a planar AMR.

What it does:
  1. Simulates a short ground-truth trajectory with a constant commanded
     twist (a gentle left curve).
  2. Generates synthetic noisy wheel-odometry (v, w) and IMU yaw-rate (w)
     measurements from that trajectory.
  3. Runs a simple EKF loop that predicts with the unicycle model and
     updates with both sensors each step.
  4. Prints a compact per-step summary and the final estimate.

Assumptions (kept explicit so it stays a study artefact):
  * Flat ground, no slip, no IMU bias.
  * Measurement noise is zero-mean Gaussian with fixed covariance.
  * IMU yaw rate is the same body-frame quantity as the state 'w'.
  * Time step is constant (dt = 0.1 s).
  * Ground truth is only available here because the trajectory is synthetic.
"""

from __future__ import annotations
import math

import numpy as np

from ekf_planar_reference import (
    EKFState,
    predict,
    update_odom,
    update_imu,
    wrap_angle,
)


# ---------------------------------------------------------------------------
# Synthetic ground truth
# ---------------------------------------------------------------------------

def simulate_step(x_true: np.ndarray, v_cmd: float, w_cmd: float,
                  dt: float) -> np.ndarray:
    """Same unicycle integration the filter uses, run on the truth."""
    px, py, theta, _, _ = x_true
    px_n = px + v_cmd * math.cos(theta) * dt
    py_n = py + v_cmd * math.sin(theta) * dt
    th_n = wrap_angle(theta + w_cmd * dt)
    return np.array([px_n, py_n, th_n, v_cmd, w_cmd])


def run_demo(n_steps: int = 20, dt: float = 0.1, seed: int = 0) -> None:
    rng = np.random.default_rng(seed)

    # Constant commanded twist: gentle left curve.
    v_cmd = 0.50  # m/s
    w_cmd = 0.20  # rad/s

    # Sensor noise (std-dev). Encoders shakier on w than gyro.
    sigma_odom_v = 0.03
    sigma_odom_w = 0.08
    sigma_imu_w = 0.02

    # Filter noise matrices (educational defaults).
    Q = np.diag([1e-4, 1e-4, 1e-3, 5e-3, 5e-3])
    R_odom = np.diag([sigma_odom_v ** 2, sigma_odom_w ** 2])
    R_imu = np.diag([sigma_imu_w ** 2])

    # Initial conditions.
    x_true = np.array([0.0, 0.0, 0.0, v_cmd, w_cmd])
    state = EKFState(
        x=np.array([0.0, 0.0, 0.0, 0.0, 0.0]),   # deliberately wrong velocities
        P=np.diag([0.05, 0.05, 0.02, 0.5, 0.5]), # generous initial uncertainty
    )

    print("=== EKF demo : synthetic planar trajectory ===")
    print(f"Steps={n_steps}, dt={dt}s, cmd=(v={v_cmd:.2f}, w={w_cmd:.2f})")
    print()
    print(
        "  k | true x   y   theta  |  est x    y    theta  |"
        "  odom_y      imu_y   "
    )
    print("-" * 88)

    for k in range(1, n_steps + 1):
        # 1) truth advances
        x_true = simulate_step(x_true, v_cmd, w_cmd, dt)

        # 2) synthetic noisy sensor readings from the truth
        z_odom = np.array([
            x_true[3] + rng.normal(0.0, sigma_odom_v),
            x_true[4] + rng.normal(0.0, sigma_odom_w),
        ])
        z_imu = np.array([x_true[4] + rng.normal(0.0, sigma_imu_w)])

        # 3) EKF predict
        state = predict(state, Q, dt)

        # 4) EKF update: odom then IMU
        state, y_odom, _ = update_odom(state, z_odom, R_odom)
        state, y_imu, _ = update_imu(state, z_imu, R_imu)

        if k % 2 == 0 or k == 1:
            print(
                f"{k:3d} | "
                f"{x_true[0]:+.3f} {x_true[1]:+.3f} {x_true[2]:+.3f} | "
                f"{state.x[0]:+.3f} {state.x[1]:+.3f} {state.x[2]:+.3f} | "
                f"[{y_odom[0]:+.3f}, {y_odom[1]:+.3f}]  "
                f"[{y_imu[0]:+.3f}]"
            )

    print()
    print("=== Final ===")
    print(f"True    x = {x_true}")
    print(f"Estimated x = {state.x}")
    err = x_true - state.x
    err[2] = wrap_angle(err[2])
    print(f"Error      = {err}")
    print(f"diag(P)    = {np.diag(state.P)}")
    print()
    print("Notes:")
    print("  * Innovations (odom_y, imu_y) should look like zero-mean noise.")
    print("  * Position error is small because heading is well corrected.")
    print("  * Long-term drift is NOT bounded without a global sensor.")


if __name__ == "__main__":
    run_demo()