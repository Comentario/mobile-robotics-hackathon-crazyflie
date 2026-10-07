"""Unit tests for the simulation world and raycasting."""

import math

import pytest

from simulation_adapter.world import World


def make_world():
    return World.from_dict({
        'bounds': [-2.0, -2.0, 2.0, 2.0],
        'ceiling': 2.0,
        'obstacles': [],
    })


def test_raycast_hits_wall_in_front():
    world = make_world()

    distance = world.raycast(
        origin=(0.0, 0.0, 0.5),
        direction=(1.0, 0.0, 0.0),
        max_range=4.0,
        t=0.0,
    )

    assert distance == pytest.approx(2.0)


def test_raycast_returns_inf_beyond_max_range():
    world = make_world()

    distance = world.raycast(
        origin=(0.0, 0.0, 0.5),
        direction=(1.0, 0.0, 0.0),
        max_range=1.0,
        t=0.0,
    )

    assert math.isinf(distance)


def test_raycast_hits_obstacle():
    world = World.from_dict({
        'bounds': [-2.0, -2.0, 2.0, 2.0],
        'ceiling': 2.0,
        'obstacles': [
            {
                'type': 'box',
                'center': [1.0, 0.0],
                'size': [0.4, 0.4],
                'height': 1.0,
            }
        ],
    })

    distance = world.raycast(
        origin=(0.0, 0.0, 0.5),
        direction=(1.0, 0.0, 0.0),
        max_range=4.0,
        t=0.0,
    )

    assert distance == pytest.approx(0.8)


def test_temporary_obstacle_only_active_in_window():
    world = World.from_dict({
        'bounds': [-2.0, -2.0, 2.0, 2.0],
        'ceiling': 2.0,
        'obstacles': [
            {
                'type': 'box',
                'center': [1.0, 0.0],
                'size': [0.4, 0.4],
                'height': 1.0,
                't_start': 5.0,
                't_end': 10.0,
            }
        ],
    })

    obstacle = world.obstacles[0]

    assert not obstacle.active(4.9)
    assert obstacle.active(5.0)
    assert obstacle.active(7.0)
    assert obstacle.active(10.0)
    assert not obstacle.active(10.1)


def test_moving_obstacle_position():
    world = World.from_dict({
        'bounds': [-2.0, -2.0, 2.0, 2.0],
        'ceiling': 2.0,
        'obstacles': [
            {
                'type': 'box',
                'center': [0.0, 0.0],
                'size': [0.2, 0.2],
                'velocity': [0.5, -0.25],
                't_start': 2.0,
            }
        ],
    })

    center = world.obstacles[0].center_at(4.0)

    assert center[0] == pytest.approx(1.0)
    assert center[1] == pytest.approx(-0.5)


def test_collision_detected():
    world = World.from_dict({
        'bounds': [-2.0, -2.0, 2.0, 2.0],
        'ceiling': 2.0,
        'obstacles': [
            {
                'type': 'box',
                'center': [0.5, 0.0],
                'size': [0.4, 0.4],
                'height': 1.0,
            }
        ],
    })

    assert world.collides(
        position=(0.5, 0.0, 0.5),
        radius=0.08,
        t=0.0,
    )