"""
Educational planar-AMR EKF reference.

State (5D):
    x = [x, y, theta, v, w]^T
        x, y   : world-frame planar position (m)
        theta  : heading / yaw (rad)
        v      : body-frame linear speed (m/s)
        w      : body-frame yaw rate (rad/s)

This file is a learning artefact, not a production filter:
  * Jacobians are first-order and intentionally simple.
  * No bias states, no global correction, no outlier rejection.
  * NumPy is used only for matrix algebra (readability).

The math mirrors what robot_localization does conceptually.
"""

from __future__ import annotations
from dataclasses import dataclass, field
import math

import numpy as np


# ---------------------------------------------------------------------------
# State container
# ---------------------------------------------------------------------------

STATE_DIM = 5  # [x, y, theta, v, w]


@dataclass
class EKFState:
    """Bundle of the state vector and its covariance."""
    x: np.ndarray = field(default_factory=lambda: np.zeros(STATE_DIM))
    P: np.ndarray = field(default_factory=lambda: np.eye(STATE_DIM) * 0.1)

    def copy(self) -> "EKFState":
        return EKFState(x=self.x.copy(), P=self.P.copy())


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]. Do this after any theta update."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Process model
# ---------------------------------------------------------------------------

def process_model(x: np.ndarray, dt: float) -> np.ndarray:
    """
    Unicycle forward Euler:
        x'     = x + v*cos(theta)*dt
        y'     = y + v*sin(theta)*dt
        theta' = theta + w*dt
        v'     = v         (random walk, Q handles changes)
        w'     = w         (random walk, Q handles changes)

    Assumes planar motion, no slip, small dt.
    """
    px, py, theta, v, w = x
    out = np.empty_like(x)
    out[0] = px + v * math.cos(theta) * dt
    out[1] = py + v * math.sin(theta) * dt
    out[2] = wrap_angle(theta + w * dt)
    out[3] = v
    out[4] = w
    return out


def process_jacobian(x: np.ndarray, dt: float) -> np.ndarray:
    """
    F = d f / d x  for the discrete process model above.
    Only the position rows couple to theta and v; the rest is identity.
    """
    _, _, theta, v, _ = x
    F = np.eye(STATE_DIM)
    F[0, 2] = -v * math.sin(theta) * dt
    F[0, 3] = math.cos(theta) * dt
    F[1, 2] = v * math.cos(theta) * dt
    F[1, 3] = math.sin(theta) * dt
    F[2, 4] = dt
    # TODO: if we later add acceleration inputs, augment F[3,:] and F[4,:].
    return F


def predict(state: EKFState, Q: np.ndarray, dt: float) -> EKFState:
    """
    EKF predict step.

        x_pred = f(x, dt)
        P_pred = F P F^T + Q
    """
    F = process_jacobian(state.x, dt)
    x_pred = process_model(state.x, dt)
    P_pred = F @ state.P @ F.T + Q
    return EKFState(x=x_pred, P=P_pred)


# ---------------------------------------------------------------------------
# Measurement models
# ---------------------------------------------------------------------------
# Both sensor models here are linear in the state, so H is just a selector.
# The EKF update still works; it reduces to a standard KF update for linear h.


def measurement_model_odom(x: np.ndarray) -> np.ndarray:
    """Wheel odometry publishes body-frame twist: h(x) = [v, w]."""
    return np.array([x[3], x[4]])


H_ODOM = np.array([
    [0.0, 0.0, 0.0, 1.0, 0.0],  # v
    [0.0, 0.0, 0.0, 0.0, 1.0],  # w
])


def measurement_model_imu(x: np.ndarray) -> np.ndarray:
    """IMU gyro z-axis publishes yaw rate: h(x) = [w]."""
    return np.array([x[4]])


H_IMU = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])


# ---------------------------------------------------------------------------
# Update step
# ---------------------------------------------------------------------------

def update(state: EKFState,
           z: np.ndarray,
           h_of_x: np.ndarray,
           H: np.ndarray,
           R: np.ndarray) -> tuple[EKFState, np.ndarray, np.ndarray]:
    """
    Generic EKF measurement update for one sensor.

        y = z - h(x)                # innovation (residual)
        S = H P H^T + R             # innovation covariance
        K = P H^T S^-1              # Kalman gain
        x_new = x + K y
        P_new = (I - K H) P

    Returns the new state, the innovation y, and the gain K (handy for
    study / debugging).
    """
    y = z - h_of_x
    S = H @ state.P @ H.T + R
    # For small matrices, solve is more numerically stable than an explicit inverse.
    K = np.linalg.solve(S.T, (state.P @ H.T).T).T

    x_new = state.x + K @ y
    x_new[2] = wrap_angle(x_new[2])  # heading lives on a circle

    I = np.eye(STATE_DIM)
    P_new = (I - K @ H) @ state.P
    # Enforce symmetry; small numerical asymmetries creep in over long runs.
    P_new = 0.5 * (P_new + P_new.T)

    return EKFState(x=x_new, P=P_new), y, K


def update_odom(state: EKFState, z_odom: np.ndarray, R_odom: np.ndarray):
    """Convenience wrapper for wheel-odom (v, w) measurements."""
    return update(state, z_odom, measurement_model_odom(state.x), H_ODOM, R_odom)


def update_imu(state: EKFState, z_imu: np.ndarray, R_imu: np.ndarray):
    """Convenience wrapper for IMU yaw-rate measurement."""
    return update(state, z_imu, measurement_model_imu(state.x), H_IMU, R_imu)


# ---------------------------------------------------------------------------
# Main: one predict/update cycle so this file runs by itself.
# ---------------------------------------------------------------------------

def _default_Q() -> np.ndarray:
    # Educational defaults. Tiny on position (grows via F), larger on v, w.
    return np.diag([1e-4, 1e-4, 1e-3, 5e-3, 5e-3])


def _default_R_odom() -> np.ndarray:
    # Encoders: decent on v, shakier on w due to slip.
    return np.diag([0.02 ** 2, 0.05 ** 2])


def _default_R_imu() -> np.ndarray:
    # Gyro: low noise on w.
    return np.diag([0.01 ** 2])


def _main() -> None:
    dt = 0.1

    # Start at the origin, small initial uncertainty.
    state = EKFState(
        x=np.array([0.0, 0.0, 0.0, 0.5, 0.1]),  # moving at 0.5 m/s, 0.1 rad/s
        P=np.diag([0.1, 0.1, 0.05, 0.1, 0.1]),
    )

    print("=== ekf_planar_reference.py : one predict + update cycle ===")
    print(f"Prior state   : {state.x}")
    print(f"Prior P shape : {state.P.shape}")

    predicted = predict(state, _default_Q(), dt)
    print(f"Predicted x   : {predicted.x}")

    # Fake odom reading slightly off the predicted twist.
    z_odom = np.array([0.52, 0.09])
    posterior, y, K = update_odom(predicted, z_odom, _default_R_odom())

    print(f"Odom z        : {z_odom}")
    print(f"Innovation y  : {y}")
    print(f"Kalman gain K shape : {K.shape}")
    print(f"Posterior x   : {posterior.x}")
    print(f"Posterior P shape : {posterior.P.shape}")
    print(f"Posterior diag(P) : {np.diag(posterior.P)}")


if __name__ == "__main__":
    _main()