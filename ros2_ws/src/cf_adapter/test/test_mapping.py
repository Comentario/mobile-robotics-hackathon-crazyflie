"""Unit tests for the pure Crazyflie adapter mapping helpers."""

import math

import pytest

from cf_adapter.mapping import (
    seconds_to_duration_parts,
    topic,
    yaw_to_quaternion,
)


def test_topic_is_normalized():
    assert topic('cf231', 'takeoff') == '/cf231/takeoff'
    assert topic('/cf231/', '/takeoff/') == '/cf231/takeoff'


def test_zero_yaw_quaternion():
    qx, qy, qz, qw = yaw_to_quaternion(0.0)

    assert qx == pytest.approx(0.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(0.0)
    assert qw == pytest.approx(1.0)


def test_pi_yaw_quaternion():
    _, _, qz, qw = yaw_to_quaternion(math.pi)

    assert qz == pytest.approx(1.0)
    assert qw == pytest.approx(0.0, abs=1e-7)


def test_duration_conversion():
    sec, nanosec = seconds_to_duration_parts(2.5)

    assert sec == 2
    assert nanosec == 500_000_000


def test_negative_duration_is_rejected():
    with pytest.raises(ValueError):
        seconds_to_duration_parts(-1.0)