"""Unit tests for the pure Crazyflie adapter mapping helpers."""

import math

import pytest

from cf_adapter.mapping import (
    ReferenceTranslator,
    scan_to_ranges,
    twist_transform,
    transform_twist,
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


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, 2**31])
def test_invalid_duration_is_rejected(value):
    with pytest.raises(ValueError):
        seconds_to_duration_parts(value)


def test_scan_cardinal_directions_and_rep117():
    rays = scan_to_ranges([1.0, math.inf, -math.inf, math.nan])
    assert rays['back'] == 1.0
    assert rays['right'] == math.inf
    assert rays['front'] == -math.inf
    assert math.isnan(rays['left'])
    assert scan_to_ranges([1, 2, 3, 4], 0, math.pi / 2) == {
        'front': 1, 'left': 2, 'back': 3, 'right': 4}


@pytest.mark.parametrize('ranges, start, step', [([1, 2], 0, 1), ([1]*4, 0, 0), ([1]*4, 0.2, math.pi/2)])
def test_ambiguous_scan_is_rejected(ranges, start, step):
    with pytest.raises(ValueError):
        scan_to_ranges(ranges, start, step)


def test_hover_latches_measured_position_and_yaw():
    translator = ReferenceTranslator()
    target, velocity, yaw = translator.translate(2, (0, 0, 0), (0, 0, 0), 0, (2, 3, 1), 0.8, 10)
    assert target == (2, 3, 1)
    assert velocity == (0, 0, 0)
    assert yaw == 0.8
    target, _, yaw = translator.translate(2, (0, 0, 0), (0, 0, 0), 0, (2.1, 3, 1), 0.9, 10.05)
    assert target == (2, 3, 1)
    assert yaw == 0.8


def test_velocity_integrates_from_measured_xy_and_holds_requested_z():
    translator = ReferenceTranslator()
    target, velocity, _ = translator.translate(1, (0, 0, 0.7), (0.2, -0.1, 0), 0, (2, 3, 1), 0, 10)
    assert target == pytest.approx((2.01, 2.995, 0.7))
    assert velocity == (0.2, -0.1, 0)
    target, _, _ = translator.translate(1, (0, 0, 0.7), (0.2, -0.1, 0), 0, (2, 3, 1), 0, 10.05)
    assert target == pytest.approx((2.02, 2.99, 0.7))
    with pytest.raises(ValueError):
        translator.translate(1, (0, 0, 0.7), (0.2, 0, 0), 0, (2, 3, 1), 0, 11)


@pytest.mark.parametrize('mode, velocity, yaw', [(9, (0, 0, 0), 0), (1, (0, 0, 1), 0), (0, (0, 0, 0), math.nan)])
def test_invalid_reference_is_rejected(mode, velocity, yaw):
    with pytest.raises(ValueError):
        ReferenceTranslator().translate(mode, (0, 0, 1), velocity, yaw, (0, 0, 1), 0, 10)


def test_world_velocity_and_degree_gyro_become_body_si_twist():
    covariance = [0.0]*36
    covariance[0], covariance[7], covariance[35] = 4.0, 1.0, 100.0
    matrix = twist_transform(yaw_to_quaternion(math.pi/2), 'world', 'degrees')
    values, converted = transform_twist((1, 0, 0, 0, 0, 90), covariance, matrix)
    assert values == pytest.approx((0, -1, 0, 0, 0, math.pi/2))
    assert converted[0] == pytest.approx(1)
    assert converted[7] == pytest.approx(4)
    assert converted[35] == pytest.approx(100*(math.pi/180)**2)
