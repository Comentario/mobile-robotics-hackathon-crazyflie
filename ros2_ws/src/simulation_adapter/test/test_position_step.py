"""TP-10, step 1: known results and motion limits, without ROS."""
import math

import pytest

from simulation_adapter.dynamics import position_step


def test_moves_known_distance_in_one_second():
    position, velocity = position_step((0, 0, 1), (2, 0, 1), 0.5, 1.0)
    assert position == pytest.approx((0.5, 0, 1))
    assert velocity == pytest.approx((0.5, 0, 0))


def test_diagonal_speed_is_limited_as_a_vector():
    position, velocity = position_step((0, 0, 1), (3, 4, 1), 1.0, 1.0)
    assert position == pytest.approx((0.6, 0.8, 1))
    assert math.hypot(*velocity) == pytest.approx(1.0)


def test_stops_at_nearby_goal_without_overshooting():
    position, velocity = position_step((0, 0, 1), (0.1, 0, 1), 0.5, 1.0)
    assert position == pytest.approx((0.1, 0, 1))
    assert velocity == pytest.approx((0.1, 0, 0))


def test_already_at_goal_remains_still():
    position, velocity = position_step((2, -1, 0.5), (2, -1, 0.5), 0.5, 0.1)
    assert position == (2, -1, 0.5)
    assert velocity == (0, 0, 0)


def test_repeated_steps_reach_goal_and_then_remain_still():
    position = (0, 0, 1)
    for _ in range(50):
        position, velocity = position_step(position, (2, 0, 1), 0.5, 0.1)
        assert 0 <= position[0] <= 2
        assert math.hypot(*velocity) <= 0.5 + 1e-12
    assert position == (2, 0, 1)
    assert velocity == (0, 0, 0)


@pytest.mark.parametrize('speed, dt', [(0, 1), (-1, 1), (1, 0), (1, -1), (math.inf, 1), (1, math.nan)])
def test_invalid_speed_or_time_is_rejected(speed, dt):
    with pytest.raises(ValueError):
        position_step((0, 0, 1), (2, 0, 1), speed, dt)


def test_invalid_coordinates_are_rejected():
    with pytest.raises(ValueError):
        position_step((0, 0), (2, 0, 1), 0.5, 1)
    with pytest.raises(ValueError):
        position_step((0, 0, 1), (math.nan, 0, 1), 0.5, 1)
