"""Localization-driven navigation speed policy for TP-16."""

import math


GOOD = 0
DEGRADED = 1
LOST = 2


def navigation_speed(status, nominal_speed, degraded_speed):
    """Return the allowed navigation speed for a localization state.

    GOOD returns nominal_speed.
    DEGRADED returns degraded_speed.
    LOST or an unknown state disables normal navigation and returns None.
    """
    nominal_speed = float(nominal_speed)
    degraded_speed = float(degraded_speed)

    if not math.isfinite(nominal_speed) or nominal_speed <= 0.0:
        raise ValueError('nominal_speed must be positive and finite')

    if not math.isfinite(degraded_speed) or degraded_speed <= 0.0:
        raise ValueError('degraded_speed must be positive and finite')

    if degraded_speed > nominal_speed:
        raise ValueError(
            'degraded_speed must not exceed nominal_speed'
        )

    if status == GOOD:
        return nominal_speed

    if status == DEGRADED:
        return degraded_speed

    if status == LOST:
        return None

    return None
