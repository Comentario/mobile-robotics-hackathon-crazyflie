"""Unit tests for safety_supervisor.supervisor (pure logic, no ROS).

Run: colcon test --packages-select safety_supervisor && colcon test-result --verbose
"""
import math

from safety_supervisor.supervisor import (
    ABORT, DEGRADED, GOOD, HOVER, LAND, LOST, NORMAL, RECOVERY_TIME, SLOW,
    SafetyInputs, SafetySupervisor, SupervisorConfig,
)


def inputs(now=0.0, **changes):
    values = dict(now=now, localization=GOOD, state_stale=False, ranges_stale=False,
                  command_stale=False, min_horizontal_range=math.inf, min_ttc=math.inf,
                  z=0.5, planner_no_path=False, outside_geofence=False)
    values.update(changes)
    return SafetyInputs(**values)


def supervisor(**config):
    return SafetySupervisor(SupervisorConfig(**config))


def test_all_good_is_normal():
    state, scale, reason = supervisor().evaluate(inputs())
    assert (state, scale, reason) == (NORMAL, 1.0, '')


def test_degraded_localization_is_slow():
    state, scale, reason = supervisor().evaluate(inputs(localization=DEGRADED))
    assert state == SLOW and scale == 0.5 and reason == 'localization_degraded'


def test_lost_localization_hovers_then_lands():
    s = supervisor(hover_timeout=3.0)
    assert s.evaluate(inputs(0.0, localization=LOST))[0] == HOVER
    assert s.evaluate(inputs(3.0, localization=LOST))[0] == HOVER
    assert s.evaluate(inputs(3.1, localization=LOST))[:3:2] == (LAND, 'localization_lost')
    # On the ground a loss never requests a landing.
    grounded = supervisor(hover_timeout=3.0)
    grounded.evaluate(inputs(0.0, localization=LOST, z=0.0))
    assert grounded.evaluate(inputs(5.0, localization=LOST, z=0.0))[0] == HOVER


def test_critical_range_hovers():
    state, _, reason = supervisor(critical_distance=0.3).evaluate(inputs(min_horizontal_range=0.2))
    assert (state, reason) == (HOVER, 'obstacle_close')


def test_stale_state_hovers():
    state, _, reason = supervisor().evaluate(inputs(state_stale=True))
    assert (state, reason) == (HOVER, 'state_timeout')


def test_stale_ranges_slow_or_hover():
    state, _, reason = supervisor(ranges_required=True).evaluate(inputs(ranges_stale=True))
    assert (state, reason) == (HOVER, 'ranges_timeout')
    state, _, _ = supervisor(ranges_required=False).evaluate(inputs(ranges_stale=True))
    assert state == NORMAL


def test_priority_abort_over_everything():
    s = supervisor()
    s.state = ABORT
    assert s.evaluate(inputs())[0] == ABORT
    assert s.evaluate(inputs(10.0, localization=LOST))[0] == ABORT
    # Among the active rules, the loss causes are reported first.
    state, _, reason = supervisor().evaluate(
        inputs(localization=LOST, min_horizontal_range=0.1, outside_geofence=True))
    assert (state, reason) == (HOVER, 'localization_lost')


def test_hysteresis_on_recovery():
    s = supervisor()
    assert s.evaluate(inputs(0.0, min_horizontal_range=0.1))[0] == HOVER
    assert s.evaluate(inputs(0.5))[0] == HOVER
    assert s.evaluate(inputs(0.5 + RECOVERY_TIME - 0.01))[0] == HOVER
    assert s.evaluate(inputs(0.5 + RECOVERY_TIME))[0] == NORMAL
