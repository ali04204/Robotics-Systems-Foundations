import math


def propagate_unicycle(x: float, y: float, theta: float,
                       v: float, w: float, dt: float) -> tuple[float, float, float]:
    """
    Simple unicycle forward Euler update.
    State: x, y, theta
    Inputs: v (linear velocity), w (angular velocity), dt
    """
    x_next = x + v * math.cos(theta) * dt
    y_next = y + v * math.sin(theta) * dt
    theta_next = theta + w * dt
    return x_next, y_next, theta_next


def clamp_scalar(value: float, min_value: float, max_value: float) -> float:
    """Clamp one scalar between min and max."""
    return max(min_value, min(value, max_value))


def clamp_velocities(v: float, w: float,
                     v_max: float, w_max: float) -> tuple[float, float]:
    """
    Symmetric clamp for linear and angular velocity.
    Linear velocity is clamped to [-v_max, v_max]
    Angular velocity is clamped to [-w_max, w_max]
    """
    v_clamped = clamp_scalar(v, -v_max, v_max)
    w_clamped = clamp_scalar(w, -w_max, w_max)
    return v_clamped, w_clamped


def slew_limit_scalar(target: float, current: float,
                      max_rate: float, dt: float) -> float:
    """
    Limit how fast a scalar command can change.
    max_rate is maximum allowed change per second.
    """
    max_delta = max_rate * dt
    delta = target - current
    delta_limited = clamp_scalar(delta, -max_delta, max_delta)
    return current + delta_limited


def slew_limit_velocities(v_target: float, w_target: float,
                          v_current: float, w_current: float,
                          dv_max: float, dw_max: float,
                          dt: float) -> tuple[float, float]:
    """
    Slew-rate limit linear and angular velocity commands separately.
    """
    v_out = slew_limit_scalar(v_target, v_current, dv_max, dt)
    w_out = slew_limit_scalar(w_target, w_current, dw_max, dt)
    return v_out, w_out

def almost_equal(a: float, b: float, eps: float = 1e-9) -> bool:
    return abs(a - b) < eps


# 1. Straight motion
x, y, theta = propagate_unicycle(0.0, 0.0, 0.0, v=1.0, w=0.0, dt=1.0)
assert almost_equal(x, 1.0)
assert almost_equal(y, 0.0)
assert almost_equal(theta, 0.0)

# 2. Pure rotation
x, y, theta = propagate_unicycle(0.0, 0.0, 0.0, v=0.0, w=2.0, dt=0.5)
assert almost_equal(x, 0.0)
assert almost_equal(y, 0.0)
assert almost_equal(theta, 1.0)

# 3. Clamp test
v_c, w_c = clamp_velocities(v=2.0, w=-3.0, v_max=0.5, w_max=1.0)
assert almost_equal(v_c, 0.5)
assert almost_equal(w_c, -1.0)

# 4. Slew-rate test for linear velocity
v_out, w_out = slew_limit_velocities(
    v_target=1.0, w_target=0.0,
    v_current=0.0, w_current=0.0,
    dv_max=0.5, dw_max=2.0,
    dt=0.1
)
assert almost_equal(v_out, 0.05)
assert almost_equal(w_out, 0.0)

# 5. Slew-rate test for angular velocity
v_out, w_out = slew_limit_velocities(
    v_target=0.0, w_target=3.0,
    v_current=0.0, w_current=0.0,
    dv_max=1.0, dw_max=4.0,
    dt=0.1
)
assert almost_equal(v_out, 0.0)
assert almost_equal(w_out, 0.4)

print("All checks passed.")