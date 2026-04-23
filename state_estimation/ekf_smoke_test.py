"""
Tiny smoke test for the planar EKF reference.

Checks:
  * Import works.
  * One predict + one update run without exceptions.
  * Shapes are consistent (state is 5D, P is 5x5, innovations are the
    expected size).
  * Covariance stays symmetric to a tight numerical tolerance.
"""

import numpy as np

from ekf_planar_reference import (
    EKFState,
    STATE_DIM,
    predict,
    update_odom,
    update_imu,
)


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    dt = 0.1

    state = EKFState(
        x=np.array([0.0, 0.0, 0.0, 0.3, 0.05]),
        P=np.eye(STATE_DIM) * 0.1,
    )

    Q = np.diag([1e-4, 1e-4, 1e-3, 5e-3, 5e-3])
    R_odom = np.diag([0.02 ** 2, 0.05 ** 2])
    R_imu = np.diag([0.01 ** 2])

    # Predict
    predicted = predict(state, Q, dt)
    _assert(predicted.x.shape == (STATE_DIM,), "state shape broke in predict")
    _assert(predicted.P.shape == (STATE_DIM, STATE_DIM), "P shape broke in predict")

    # Update (odom)
    z_odom = np.array([0.31, 0.06])
    post_odom, y_odom, K_odom = update_odom(predicted, z_odom, R_odom)
    _assert(post_odom.x.shape == (STATE_DIM,), "state shape broke in odom update")
    _assert(post_odom.P.shape == (STATE_DIM, STATE_DIM), "P shape broke in odom update")
    _assert(y_odom.shape == (2,), "odom innovation should be length 2")
    _assert(K_odom.shape == (STATE_DIM, 2), "odom gain should be 5x2")

    # Update (imu)
    z_imu = np.array([0.055])
    post_imu, y_imu, K_imu = update_imu(post_odom, z_imu, R_imu)
    _assert(y_imu.shape == (1,), "imu innovation should be length 1")
    _assert(K_imu.shape == (STATE_DIM, 1), "imu gain should be 5x1")

    # Covariance symmetry check
    P = post_imu.P
    _assert(np.allclose(P, P.T, atol=1e-9), "P lost symmetry after update")

    print("EKF smoke test passed")


if __name__ == "__main__":
    main()
