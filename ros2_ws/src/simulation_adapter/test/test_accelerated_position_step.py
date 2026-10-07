"""TP-10, paso 2: aceleración, frenado y convergencia sin ROS."""
import math

import pytest

from simulation_adapter.dynamics import DroneState, accelerated_position_step


def test_accelerates_from_rest_with_known_displacement():
    state = DroneState(z=1, flying=True)
    result = accelerated_position_step(state, (2, 0, 1), 0.5, 0.2, 0.1)
    assert result.vx == pytest.approx(0.02)
    assert result.x == pytest.approx(0.001)
    assert state.x == 0 and state.vx == 0  # El estado anterior se conserva.
    assert result.flying and result.z == 1


def test_diagonal_motion_respects_vector_speed_and_acceleration():
    state = DroneState(z=1)
    for _ in range(500):
        previous = state
        state = accelerated_position_step(state, (3, 4, 2), 0.5, 0.2, 0.1)
        assert math.hypot(state.vx, state.vy, state.vz) <= 0.5 + 1e-12
        change = (state.vx-previous.vx, state.vy-previous.vy, state.vz-previous.vz)
        assert math.hypot(*change) / 0.1 <= 0.2 + 1e-12
    assert (state.x, state.y, state.z) == pytest.approx((3, 4, 2), abs=0.001)
    assert math.hypot(state.vx, state.vy, state.vz) < 0.001


def test_brakes_and_settles_at_goal():
    state = DroneState(z=1)
    trace = []
    for _ in range(200):
        state = accelerated_position_step(state, (2, 0, 1), 0.5, 0.2, 0.1)
        trace.append(state)
    assert max(item.vx for item in trace) == pytest.approx(0.5)
    assert any(0.05 < item.vx < 0.4 and item.x > 1.5 for item in trace)
    assert state.x == pytest.approx(2, abs=0.001)
    assert abs(state.vx) < 0.001


def test_at_goal_with_existing_velocity_does_not_teleport_or_stop_instantly():
    state = DroneState(x=2, z=1, vx=0.5)
    result = accelerated_position_step(state, (2, 0, 1), 0.5, 0.2, 0.1)
    assert result.vx == pytest.approx(0.48)
    assert result.x == pytest.approx(2.049)


def test_changed_target_reverses_progressively_and_eventually_converges():
    state = DroneState(z=1, vx=0.5)
    for _ in range(300):
        previous = state
        state = accelerated_position_step(state, (-1, 0, 1), 0.5, 0.2, 0.1)
        assert abs(state.vx - previous.vx) <= 0.02 + 1e-12
    assert state.x == pytest.approx(-1, abs=0.001)
    assert abs(state.vx) < 0.001


def test_stationary_goal_preserves_yaw_height_and_flight_flag():
    state = DroneState(x=2, z=1, yaw=0.7, flying=True)
    assert accelerated_position_step(state, (2, 0, 1), 0.5, 0.2, 0.1) == state


@pytest.mark.parametrize('speed, accel, dt, response', [
    (0, 0.2, 0.1, 1), (0.5, 0, 0.1, 1), (0.5, -1, 0.1, 1),
    (0.5, 0.2, 0, 1), (0.5, 0.2, 0.1, 0), (0.5, 0.2, 2, 1),
    (0.5, math.nan, 0.1, 1), (math.inf, 0.2, 0.1, 1),
])
def test_invalid_parameters_are_rejected(speed, accel, dt, response):
    with pytest.raises(ValueError):
        accelerated_position_step(DroneState(), (2, 0, 1), speed, accel, dt, response)


def test_invalid_state_and_target_are_rejected():
    with pytest.raises(ValueError):
        accelerated_position_step(DroneState(vx=1), (2, 0, 1), 0.5, 0.2, 0.1)
    with pytest.raises(ValueError):
        accelerated_position_step(DroneState(x=math.nan), (2, 0, 1), 0.5, 0.2, 0.1)
    with pytest.raises(ValueError):
        accelerated_position_step(DroneState(), (2, 0), 0.5, 0.2, 0.1)
