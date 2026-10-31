"""Unit tests for TP-34 horizontal reactive avoidance."""

import math

import pytest

from local_navigator.avoidance import (
    AvoidanceConfig,
    estimate_closing_rates,
    reactive_correction,
    vertical_limit,
)


@pytest.fixture
def config():
    return AvoidanceConfig(
        warning_distance=0.5,
        critical_distance=0.2,
        avoidance_speed=0.2,
    )


def clear_ranges():
    return {
        'front': math.inf,
        'back': math.inf,
        'left': math.inf,
        'right': math.inf,
    }


def test_clear_path_preserves_velocity(config):
    velocity, blocked, reason = reactive_correction(
        ranges=clear_ranges(),
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert velocity == pytest.approx((0.3, 0.0, 0.0))
    assert blocked is False
    assert reason == ''


def test_front_critical_obstacle_prevents_forward_motion(config):
    ranges = clear_ranges()
    ranges['front'] = 0.15

    velocity, blocked, _ = reactive_correction(
        ranges=ranges,
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert velocity[0] <= 0.0
    assert blocked is False


def test_front_warning_obstacle_slows_forward_motion(config):
    ranges = clear_ranges()
    ranges['front'] = 0.35

    velocity, _, _ = reactive_correction(
        ranges=ranges,
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert 0.0 < velocity[0] < 0.3


def test_left_obstacle_pushes_right(config):
    ranges = clear_ranges()
    ranges['left'] = 0.15

    velocity, _, _ = reactive_correction(
        ranges=ranges,
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert velocity[1] < 0.0


def test_right_obstacle_pushes_left(config):
    ranges = clear_ranges()
    ranges['right'] = 0.15

    velocity, _, _ = reactive_correction(
        ranges=ranges,
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert velocity[1] > 0.0


def test_invalid_range_is_ignored(config):
    ranges = clear_ranges()
    ranges['front'] = math.nan

    velocity, blocked, _ = reactive_correction(
        ranges=ranges,
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert velocity == pytest.approx((0.3, 0.0, 0.0))
    assert blocked is False


def test_boxed_forward_direction_reports_blocked(config):
    ranges = clear_ranges()
    ranges['front'] = 0.15
    ranges['left'] = 0.15
    ranges['right'] = 0.15

    velocity, blocked, reason = reactive_correction(
        ranges=ranges,
        closing_rates=None,
        velocity_body=(0.3, 0.0, 0.0),
        config=config,
    )

    assert blocked is True
    assert reason == 'horizontal_blocked'
    assert velocity[:2] == pytest.approx((0.0, 0.0))


def test_low_ttc_requests_hover():
    ttc_config = AvoidanceConfig(
        warning_distance=0.5,
        critical_distance=0.2,
        ttc_threshold=1.5,
        avoidance_speed=0.2,
    )

    ranges = clear_ranges()
    ranges['front'] = 0.6

    velocity, blocked, reason = reactive_correction(
        ranges=ranges,
        closing_rates={
            'front': 1.0,
            'back': 0.0,
            'left': 0.0,
            'right': 0.0,
        },
        velocity_body=(0.3, 0.0, 0.0),
        config=ttc_config,
    )

    assert velocity[:2] == pytest.approx(
        (0.0, 0.0)
    )

    assert blocked is True
    assert reason == 'ttc_low:front'


def test_high_ttc_preserves_navigation():
    ttc_config = AvoidanceConfig(
        warning_distance=0.5,
        critical_distance=0.2,
        ttc_threshold=1.5,
        avoidance_speed=0.2,
    )

    ranges = clear_ranges()
    ranges['front'] = 2.0

    velocity, blocked, reason = reactive_correction(
        ranges=ranges,
        closing_rates={
            'front': 0.5,
            'back': 0.0,
            'left': 0.0,
            'right': 0.0,
        },
        velocity_body=(0.3, 0.0, 0.0),
        config=ttc_config,
    )

    assert velocity == pytest.approx(
        (0.3, 0.0, 0.0)
    )

    assert blocked is False
    assert reason == ''


def test_closing_rate_estimation():
    rates = estimate_closing_rates(
        previous_ranges={
            'front': 2.0,
            'back': 1.0,
            'left': 1.0,
            'right': 1.0,
        },
        current_ranges={
            'front': 1.0,
            'back': 1.2,
            'left': 1.0,
            'right': 1.0,
        },
        dt=0.5,
    )

    assert rates['front'] == pytest.approx(
        2.0
    )

    assert rates['back'] == pytest.approx(
        0.0
    )

    assert rates['left'] == pytest.approx(
        0.0
    )


def test_invalid_ranges_do_not_create_closing_rate():
    rates = estimate_closing_rates(
        previous_ranges={
            'front': math.inf,
            'back': math.nan,
            'left': 1.0,
            'right': 1.0,
        },
        current_ranges={
            'front': 1.0,
            'back': 0.5,
            'left': 1.2,
            'right': 1.0,
        },
        dt=0.1,
    )

    assert rates['front'] == pytest.approx(
        0.0
    )

    assert rates['back'] == pytest.approx(
        0.0
    )

    assert rates['left'] == pytest.approx(
        0.0
    )


def test_ceiling_blocks_ascent():
    target = vertical_limit(
        z_target=0.8,
        up_range=0.15,
        down_range=math.inf,
        z_min=0.0,
        z_max=1.5,
        critical_distance=0.2,
        current_z=0.5,
    )

    assert target == pytest.approx(0.5)


def test_floor_blocks_descent():
    target = vertical_limit(
        z_target=0.2,
        up_range=math.inf,
        down_range=0.15,
        z_min=0.0,
        z_max=1.5,
        critical_distance=0.2,
        current_z=0.5,
    )

    assert target == pytest.approx(0.5)


def test_vertical_target_respects_altitude_envelope():
    high_target = vertical_limit(
        z_target=2.0,
        up_range=math.inf,
        down_range=math.inf,
        z_min=0.1,
        z_max=1.5,
        critical_distance=0.2,
        current_z=0.5,
    )

    low_target = vertical_limit(
        z_target=-0.2,
        up_range=math.inf,
        down_range=math.inf,
        z_min=0.1,
        z_max=1.5,
        critical_distance=0.2,
        current_z=0.5,
    )

    assert high_target == pytest.approx(1.5)
    assert low_target == pytest.approx(0.1)
