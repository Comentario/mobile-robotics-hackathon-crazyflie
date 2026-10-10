"""Reactive avoidance helpers for the local navigator."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AvoidanceConfig:
    """Configuration for reactive obstacle avoidance."""

    warning_distance: float
    critical_distance: float
    ttc_threshold: float = 0.0
    avoidance_speed: float = 0.2

    def __post_init__(self):
        if self.critical_distance <= 0.0:
            raise ValueError('critical_distance must be positive')

        if self.warning_distance <= self.critical_distance:
            raise ValueError(
                'warning_distance must be greater than critical_distance'
            )

        if (
            not math.isfinite(self.ttc_threshold)
            or self.ttc_threshold < 0.0
        ):
            raise ValueError(
                'ttc_threshold must be non-negative and finite'
            )

        if (
            not math.isfinite(self.avoidance_speed)
            or self.avoidance_speed < 0.0
        ):
            raise ValueError(
                'avoidance_speed must be non-negative and finite'
            )


def _range_value(ranges, name):
    """Read a range from either a mapping or a ROS-like object."""
    if isinstance(ranges, dict):
        return float(ranges.get(name, math.nan))

    return float(getattr(ranges, name, math.nan))


def _avoidance_strength(distance, config):
    """Return avoidance strength in [0, 1] for one range reading."""
    if math.isnan(distance):
        return 0.0

    if distance == math.inf:
        return 0.0

    if distance == -math.inf:
        return 1.0

    if distance <= config.critical_distance:
        return 1.0

    if distance >= config.warning_distance:
        return 0.0

    span = (
        config.warning_distance
        - config.critical_distance
    )

    return (
        config.warning_distance
        - distance
    ) / span


def estimate_closing_rates(
    previous_ranges,
    current_ranges,
    dt,
):
    """Estimate positive obstacle closing speeds from consecutive ranges.

    A positive value means that the measured obstacle is approaching.
    Stable, receding, invalid, or unavailable measurements return zero.
    """
    dt = float(dt)

    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError(
            'dt must be positive and finite'
        )

    result = {}

    for name in (
        'front',
        'back',
        'left',
        'right',
    ):
        previous = _range_value(
            previous_ranges,
            name,
        )

        current = _range_value(
            current_ranges,
            name,
        )

        if not (
            math.isfinite(previous)
            and math.isfinite(current)
            and previous >= 0.0
            and current >= 0.0
        ):
            result[name] = 0.0
            continue

        result[name] = max(
            0.0,
            (previous - current) / dt,
        )

    return result


def _minimum_ttc(ranges, closing_rates):
    """Return the smallest valid horizontal TTC and its sensor name."""
    best_ttc = math.inf
    best_sensor = None

    for name in (
        'front',
        'back',
        'left',
        'right',
    ):
        distance = _range_value(
            ranges,
            name,
        )

        closing_rate = _range_value(
            closing_rates,
            name,
        )

        if not (
            math.isfinite(distance)
            and distance > 0.0
            and math.isfinite(closing_rate)
            and closing_rate > 0.0
        ):
            continue

        ttc = distance / closing_rate

        if ttc < best_ttc:
            best_ttc = ttc
            best_sensor = name

    return best_ttc, best_sensor


def reactive_correction(
    ranges,
    closing_rates,
    velocity_body,
    config,
):
    """Apply reactive horizontal obstacle avoidance in the body frame.

    REP-103 body convention:
      +x: front
      -x: back
      +y: left
      -y: right

    TP-34 applies instantaneous range avoidance.
    TP-36 additionally commands horizontal hover when an approaching
    obstacle has TTC less than or equal to config.ttc_threshold.
    """
    vx, vy, vz = (
        float(value)
        for value in velocity_body
    )

    if not all(
        math.isfinite(value)
        for value in (vx, vy, vz)
    ):
        raise ValueError(
            'velocity_body must contain finite values'
        )

    front = _avoidance_strength(
        _range_value(ranges, 'front'),
        config,
    )

    back = _avoidance_strength(
        _range_value(ranges, 'back'),
        config,
    )

    left = _avoidance_strength(
        _range_value(ranges, 'left'),
        config,
    )

    right = _avoidance_strength(
        _range_value(ranges, 'right'),
        config,
    )

    original_vx = vx
    original_vy = vy

    if front > 0.0:
        if vx > 0.0:
            vx *= 1.0 - front

        vx -= (
            config.avoidance_speed
            * front
        )

    if back > 0.0:
        if vx < 0.0:
            vx *= 1.0 - back

        vx += (
            config.avoidance_speed
            * back
        )

    if left > 0.0:
        if vy > 0.0:
            vy *= 1.0 - left

        vy -= (
            config.avoidance_speed
            * left
        )

    if right > 0.0:
        if vy < 0.0:
            vy *= 1.0 - right

        vy += (
            config.avoidance_speed
            * right
        )

    epsilon = 1e-9

    blocked = (
        (
            original_vx > epsilon
            and front >= 1.0
            and left >= 1.0
            and right >= 1.0
        )
        or (
            original_vx < -epsilon
            and back >= 1.0
            and left >= 1.0
            and right >= 1.0
        )
        or (
            original_vy > epsilon
            and left >= 1.0
            and front >= 1.0
            and back >= 1.0
        )
        or (
            original_vy < -epsilon
            and right >= 1.0
            and front >= 1.0
            and back >= 1.0
        )
        or (
            abs(original_vx) <= epsilon
            and abs(original_vy) <= epsilon
            and front >= 1.0
            and back >= 1.0
            and left >= 1.0
            and right >= 1.0
        )
    )

    if (
        config.ttc_threshold > 0.0
        and closing_rates is not None
    ):
        minimum_ttc, sensor = _minimum_ttc(
            ranges,
            closing_rates,
        )

        if (
            sensor is not None
            and minimum_ttc
            <= config.ttc_threshold
        ):
            return (
                (0.0, 0.0, vz),
                True,
                f'ttc_low:{sensor}',
            )

    if blocked:
        vx = 0.0
        vy = 0.0

    reason = (
        'horizontal_blocked'
        if blocked
        else ''
    )

    return (
        (vx, vy, vz),
        blocked,
        reason,
    )


def vertical_limit(
    z_target,
    up_range,
    down_range,
    z_min,
    z_max,
    critical_distance,
    current_z=None,
):
    """Limit a vertical target using the altitude envelope and range data.

    A critical upward range blocks commands that would increase altitude.
    A critical downward range blocks commands that would decrease altitude.
    NaN and +inf are treated as unavailable or clear readings respectively,
    while -inf is treated as a critical obstacle according to REP-117.
    """
    z_target = float(z_target)
    z_min = float(z_min)
    z_max = float(z_max)
    critical_distance = float(critical_distance)

    if not all(
        math.isfinite(value)
        for value in (
            z_target,
            z_min,
            z_max,
            critical_distance,
        )
    ):
        raise ValueError(
            'vertical limits and target must be finite'
        )

    if z_min > z_max:
        raise ValueError(
            'z_min must not exceed z_max'
        )

    if critical_distance <= 0.0:
        raise ValueError(
            'critical_distance must be positive'
        )

    safe_target = min(
        z_max,
        max(z_min, z_target),
    )

    if current_z is None:
        return safe_target

    current_z = float(current_z)

    if not math.isfinite(current_z):
        raise ValueError(
            'current_z must be finite'
        )

    def is_critical(distance):
        distance = float(distance)

        if math.isnan(distance):
            return False

        if distance == math.inf:
            return False

        if distance == -math.inf:
            return True

        return distance <= critical_distance

    if (
        safe_target > current_z
        and is_critical(up_range)
    ):
        safe_target = current_z

    if (
        safe_target < current_z
        and is_critical(down_range)
    ):
        safe_target = current_z

    return safe_target
