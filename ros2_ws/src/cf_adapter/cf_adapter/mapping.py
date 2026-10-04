"""Pure conversion helpers used by the Crazyflie adapter."""

import math


def topic(robot_name, suffix):
    """Build a normalized Crazyswarm2 topic or service name."""
    robot = robot_name.strip('/')
    endpoint = suffix.strip('/')
    return f'/{robot}/{endpoint}'


def yaw_to_quaternion(yaw_rad):
    """Convert a yaw angle in radians to a quaternion."""
    half_yaw = 0.5 * yaw_rad
    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def seconds_to_duration_parts(duration_s):
    """Convert floating-point seconds to ROS sec/nanosec fields."""
    if duration_s < 0.0:
        raise ValueError('Duration must be non-negative')

    sec = int(duration_s)
    nanosec = int(round((duration_s - sec) * 1_000_000_000))

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    return sec, nanosec


def scan_to_ranges(scan_ranges):
    """Convert Crazyswarm2 range data to the project range convention.

    The exact Crazyflie Multi-ranger ordering is deliberately not implemented
    until the real sensor/logging interface is confirmed.
    """
    raise NotImplementedError(
        'Multi-ranger mapping requires the confirmed Crazyflie sensor ordering'
    )