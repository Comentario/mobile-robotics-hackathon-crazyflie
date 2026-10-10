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
    """Return a point ahead of the closest point on a 2-D path.

    The vehicle position is projected onto the nearest path segment.
    The returned target is then advanced by lookahead_distance along
    the path. Remaining coordinates, such as altitude, are interpolated.
    """
    lookahead_distance = float(lookahead_distance)

    if (
        not math.isfinite(lookahead_distance)
        or lookahead_distance <= 0.0
    ):
        raise ValueError(
            'lookahead_distance must be positive and finite'
        )

    if not path:
        raise ValueError('path must not be empty')

    points = [
        tuple(float(value) for value in point)
        for point in path
    ]

    dimension = len(points[0])

    if dimension < 2:
        raise ValueError(
            'path points must contain at least x and y'
        )

    if any(
        len(point) != dimension
        for point in points
    ):
        raise ValueError(
            'all path points must have the same dimension'
        )

    if any(
        not all(math.isfinite(value) for value in point)
        for point in points
    ):
        raise ValueError(
            'path points must contain finite values'
        )

    position = tuple(
        float(value)
        for value in position
    )

    if len(position) < 2:
        raise ValueError(
            'position must contain at least x and y'
        )

    if len(points) == 1:
        return points[0]

    px = position[0]
    py = position[1]

    best_distance = math.inf
    best_segment = 0
    best_fraction = 0.0
    best_projection = points[0]

    for index, (start, end) in enumerate(
        zip(points, points[1:])
    ):
        dx = end[0] - start[0]
        dy = end[1] - start[1]

        length_squared = dx * dx + dy * dy

        if length_squared == 0.0:
            fraction = 0.0
        else:
            fraction = (
                (px - start[0]) * dx
                + (py - start[1]) * dy
            ) / length_squared

            fraction = min(
                1.0,
                max(0.0, fraction),
            )

        projection = tuple(
            start_value
            + fraction * (end_value - start_value)
            for start_value, end_value
            in zip(start, end)
        )

        distance = math.hypot(
            px - projection[0],
            py - projection[1],
        )

        if distance < best_distance:
            best_distance = distance
            best_segment = index
            best_fraction = fraction
            best_projection = projection

    remaining = lookahead_distance
    current = best_projection

    start = points[best_segment]
    end = points[best_segment + 1]

    segment_length = math.hypot(
        end[0] - start[0],
        end[1] - start[1],
    )

    available = (
        segment_length
        * (1.0 - best_fraction)
    )

    if available >= remaining and segment_length > 0.0:
        fraction = (
            best_fraction
            + remaining / segment_length
        )

        return tuple(
            start_value
            + fraction * (end_value - start_value)
            for start_value, end_value
            in zip(start, end)
        )

    remaining -= available

    for index in range(
        best_segment + 1,
        len(points) - 1,
    ):
        start = points[index]
        end = points[index + 1]

        segment_length = math.hypot(
            end[0] - start[0],
            end[1] - start[1],
        )

        if segment_length == 0.0:
            continue

        if segment_length >= remaining:
            fraction = remaining / segment_length

            return tuple(
                start_value
                + fraction * (end_value - start_value)
                for start_value, end_value
                in zip(start, end)
            )

        remaining -= segment_length
        current = end

    return points[-1]


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
