"""Unit tests for simulator fault injection."""

import math

import pytest

from simulation_adapter.faults import FaultInjector


def test_range_spike_only_in_window():
    injector = FaultInjector([
        {
            'type': 'range_spike',
            'sensor': 'front',
            'value': 0.1,
            't_start': 5.0,
            't_end': 6.0,
        }
    ])

    ranges = {
        'front': 1.0,
        'back': 2.0,
    }

    before = injector.apply_to_ranges(
        4.0,
        ranges,
    )

    during = injector.apply_to_ranges(
        5.5,
        ranges,
    )

    assert before['front'] == pytest.approx(1.0)
    assert during['front'] == pytest.approx(0.1)
    assert during['back'] == pytest.approx(2.0)


def test_range_dropout_produces_nan():
    injector = FaultInjector([
        {
            'type': 'range_dropout',
            'sensor': 'front',
            't_start': 1.0,
            't_end': 2.0,
        }
    ])

    result = injector.apply_to_ranges(
        1.5,
        {'front': 1.0},
    )

    assert math.isnan(result['front'])


def test_pose_freeze_repeats_last_pose():
    injector = FaultInjector([
        {
            'type': 'pose_freeze',
            't_start': 5.0,
            't_end': 10.0,
        }
    ])

    history = [
        (
            4.0,
            {
                'x': 1.0,
                'y': 0.0,
                'z': 0.5,
                'yaw': 0.0,
            },
        ),
        (
            5.0,
            {
                'x': 2.0,
                'y': 0.0,
                'z': 0.5,
                'yaw': 0.0,
            },
        ),
        (
            6.0,
            {
                'x': 3.0,
                'y': 0.0,
                'z': 0.5,
                'yaw': 0.0,
            },
        ),
    ]

    current = history[-1][1]

    result = injector.apply_to_pose(
        6.0,
        current,
        history,
    )

    assert result['x'] == pytest.approx(2.0)
    assert result['stamp_t'] == pytest.approx(5.0)


def test_pose_dropout_publishes_nothing():
    injector = FaultInjector([
        {
            'type': 'pose_dropout',
            't_start': 5.0,
            't_end': 10.0,
        }
    ])

    pose = {
        'x': 1.0,
        'y': 2.0,
        'z': 0.5,
        'yaw': 0.0,
    }

    result = injector.apply_to_pose(
        6.0,
        pose,
        [],
    )

    assert result is None


def test_command_pause_drops_commands():
    injector = FaultInjector([
        {
            'type': 'command_pause',
            't_start': 3.0,
            't_end': 4.0,
        }
    ])

    assert not injector.drop_command(2.0)
    assert injector.drop_command(3.5)
    assert not injector.drop_command(5.0)


def test_pose_delay_keeps_sample_time():
    injector = FaultInjector([
        {
            'type': 'pose_delay',
            'delay': 0.5,
            't_start': 5.0,
            't_end': 10.0,
        }
    ])

    history = [
        (5.4, {'x': 1.0, 'y': 0.0, 'z': 0.5, 'yaw': 0.0}),
        (5.6, {'x': 2.0, 'y': 0.0, 'z': 0.5, 'yaw': 0.0}),
        (6.0, {'x': 3.0, 'y': 0.0, 'z': 0.5, 'yaw': 0.0}),
    ]

    result = injector.apply_to_pose(6.0, history[-1][1], history)

    assert result['x'] == pytest.approx(1.0)
    assert result['stamp_t'] == pytest.approx(5.4)
