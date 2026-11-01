"""Unit tests for safety_supervisor.geofence (pure logic, no ROS).

Run: colcon test --packages-select safety_supervisor && colcon test-result --verbose
"""
from safety_supervisor.geofence import Geofence

FENCE = Geofence(-2.0, -2.0, 0.0, 2.0, 2.0, 1.5)


def test_inside_point():
    assert FENCE.contains((0.0, 1.9, 0.5))
    assert not FENCE.contains((2.05, 0.0, 0.5))
    assert FENCE.contains((2.05, 0.0, 0.5), margin=0.1)


def test_clamp_outside_point():
    assert FENCE.clamp((3.0, -2.5, 0.5)) == (2.0, -2.0, 0.5)
    assert FENCE.clamp((1.0, 1.0, 0.5)) == (1.0, 1.0, 0.5)


def test_altitude_limits():
    assert FENCE.clamp((0.0, 0.0, 2.0))[2] == 1.5
    assert FENCE.clamp((0.0, 0.0, -0.3))[2] == 0.0
    assert not FENCE.contains((0.0, 0.0, 1.6))
