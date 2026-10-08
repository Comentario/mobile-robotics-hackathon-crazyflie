"""Unit tests for the TP-32 direct position follower."""

import math

import pytest

from local_navigator.follower import (
    direct_position_setpoint,
    yaw_towards,
)


def test_position_step_is_bounded():
    target = direct_position_setpoint(
        position=(0.0, 0.0, 0.5),
        goal=(1.0, 0.0, 0.5),
        max_step=0.1,
        z_nominal=0.5,
    )

    assert target == pytest.approx((0.1, 0.0, 0.5))


def test_position_reaches_nearby_goal_without_overshoot():
    target = direct_position_setpoint(
        position=(0.0, 0.0, 0.5),
        goal=(0.05, 0.0, 0.5),
        max_step=0.1,
        z_nominal=0.5,
    )

    assert target == pytest.approx((0.05, 0.0, 0.5))


def test_diagonal_step_is_bounded():
    target = direct_position_setpoint(
        position=(0.0, 0.0, 0.5),
        goal=(1.0, 1.0, 0.5),
        max_step=0.2,
        z_nominal=0.5,
    )

    distance = math.hypot(target[0], target[1])

    assert distance == pytest.approx(0.2)
    assert target[0] == pytest.approx(target[1])


def test_holds_z_nominal():
    target = direct_position_setpoint(
        position=(0.0, 0.0, 0.3),
        goal=(1.0, 0.0, 2.0),
        max_step=0.1,
        z_nominal=0.5,
    )

    assert target[2] == pytest.approx(0.5)


def test_yaw_faces_target():
    yaw = yaw_towards(
        position=(0.0, 0.0, 0.5),
        target=(0.0, 1.0, 0.5),
    )

    assert yaw == pytest.approx(math.pi / 2.0)


def test_yaw_is_preserved_at_target():
    yaw = yaw_towards(
        position=(1.0, 1.0, 0.5),
        target=(1.0, 1.0, 0.5),
        current_yaw=0.7,
    )

    assert yaw == pytest.approx(0.7)


def test_rejects_nonpositive_step():
    with pytest.raises(ValueError):
        direct_position_setpoint(
            position=(0.0, 0.0, 0.5),
            goal=(1.0, 0.0, 0.5),
            max_step=0.0,
            z_nominal=0.5,
        )
