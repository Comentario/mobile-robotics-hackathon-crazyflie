"""Unit tests for safety_supervisor.watchdog (pure logic, no ROS).

Run: colcon test --packages-select safety_supervisor && colcon test-result --verbose
"""
from safety_supervisor.watchdog import StreamWatchdog


def test_never_stamped_is_stale():
    watchdog = StreamWatchdog(0.3)
    assert watchdog.stale(0.0)
    assert not watchdog.received_any()


def test_stale_after_timeout():
    watchdog = StreamWatchdog(0.3)
    watchdog.stamp(10.0)
    assert watchdog.received_any()
    assert not watchdog.stale(10.29)
    assert watchdog.stale(10.31)
