"""Pure path-following helpers for TP-32 and later navigation TPs."""

from dataclasses import dataclass
import math


@dataclass
class FollowerConfig:
    """Configuration shared by the local follower."""

    lookahead_distance: float
    max_speed: float
    max_accel: float
    goal_slowdown_radius: float
    z_nominal: float


def direct_position_setpoint(
    position,
    goal,
    max_step,
    z_nominal,
):
    """Generate a bounded position setpoint toward a direct waypoint.

    Horizontal motion is limited to max_step per control cycle.
    Altitude is kept at z_nominal for the 2.5D MVP.
    """
    if max_step <= 0.0:
        raise ValueError('max_step must be positive')

    px, py, _ = (float(value) for value in position)
    gx, gy, _ = (float(value) for value in goal)

    dx = gx - px
    dy = gy - py

    distance_xy = math.hypot(dx, dy)

    if distance_xy <= max_step or distance_xy == 0.0:
        target_x = gx
        target_y = gy
    else:
        scale = max_step / distance_xy
        target_x = px + scale * dx
        target_y = py + scale * dy

    return (
        target_x,
        target_y,
        float(z_nominal),
    )


def yaw_towards(position, target, current_yaw=0.0):
    """Return a yaw angle that faces the horizontal target direction."""
    px, py, _ = (float(value) for value in position)
    tx, ty, _ = (float(value) for value in target)

    dx = tx - px
    dy = ty - py

    if math.hypot(dx, dy) < 1e-9:
        return float(current_yaw)

    return math.atan2(dy, dx)


def lookahead_point(path, position, lookahead_distance):
    """Reserved for the path-based follower after the direct-route MVP."""
    raise NotImplementedError


def desired_velocity(
    position,
    target,
    speed_limit,
    distance_to_goal,
    config,
):
    """Reserved for TP-33 velocity shaping and slowdown."""
    raise NotImplementedError


class RateLimiter:
    """Velocity rate limiter reserved for TP-33."""

    def __init__(self, max_accel):
        self.max_accel = max_accel

    def apply(self, dt, velocity):
        """Apply acceleration limiting in TP-33."""
        raise NotImplementedError
