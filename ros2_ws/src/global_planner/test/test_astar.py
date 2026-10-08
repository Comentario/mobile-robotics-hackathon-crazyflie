"""Unit tests for TP-27 A* baseline."""

import math

import pytest

from global_planner.astar import astar, direct_path


def free_grid(width, height):
    return [
        [1.0] * width
        for _ in range(height)
    ]


def test_free_space_finds_path():
    grid = free_grid(5, 5)

    path = astar(
        grid,
        start=(0, 0),
        goal=(4, 4),
        connectivity=8,
    )

    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)


def test_wall_requires_detour():
    grid = free_grid(7, 5)

    for y in range(4):
        grid[y][3] = math.inf

    path = astar(
        grid,
        start=(1, 1),
        goal=(5, 1),
        connectivity=4,
    )

    assert path is not None
    assert path[0] == (1, 1)
    assert path[-1] == (5, 1)

    assert all(
        math.isfinite(grid[y][x])
        for x, y in path
    )

    assert any(
        y == 4
        for _, y in path
    )


def test_no_path_returns_none():
    grid = free_grid(5, 5)

    for y in range(5):
        grid[y][2] = math.inf

    path = astar(
        grid,
        start=(0, 2),
        goal=(4, 2),
        connectivity=4,
    )

    assert path is None


def test_blocked_goal_returns_none():
    grid = free_grid(5, 5)
    grid[4][4] = math.inf

    assert astar(
        grid,
        start=(0, 0),
        goal=(4, 4),
    ) is None


def test_eight_connectivity_is_shorter_in_open_space():
    grid = free_grid(5, 5)

    path_four = astar(
        grid,
        start=(0, 0),
        goal=(4, 4),
        connectivity=4,
    )

    path_eight = astar(
        grid,
        start=(0, 0),
        goal=(4, 4),
        connectivity=8,
    )

    assert len(path_eight) < len(path_four)


def test_diagonal_does_not_cut_blocked_corner():
    grid = free_grid(3, 3)

    grid[0][1] = math.inf
    grid[1][0] = math.inf

    path = astar(
        grid,
        start=(0, 0),
        goal=(1, 1),
        connectivity=8,
    )

    assert path is None


def test_direct_path_spacing():
    path = direct_path(
        start=(0.0, 0.0, 0.5),
        goal=(1.0, 0.0, 0.5),
        spacing=0.3,
    )

    assert path[0] == pytest.approx(
        (0.0, 0.0, 0.5)
    )

    assert path[-1] == pytest.approx(
        (1.0, 0.0, 0.5)
    )

    for first, second in zip(path, path[1:]):
        distance = math.dist(first, second)
        assert distance <= 0.3 + 1e-12


def test_unknown_policy_block_vs_penalize():
    from global_planner.astar import build_cost_map

    grid = [
        [0, -1, 0],
        [0, 0, 0],
    ]

    blocked = build_cost_map(
        grid=grid,
        unknown_policy='block',
        unknown_cost=5.0,
        proximity_weight=0.0,
    )

    penalized = build_cost_map(
        grid=grid,
        unknown_policy='penalize',
        unknown_cost=5.0,
        proximity_weight=0.0,
    )

    blocked_path = astar(
        blocked,
        start=(0, 0),
        goal=(2, 0),
        connectivity=4,
    )

    penalized_path = astar(
        penalized,
        start=(0, 0),
        goal=(2, 0),
        connectivity=4,
    )

    assert blocked_path is not None
    assert (1, 0) not in blocked_path

    assert penalized_path is not None
    assert math.isfinite(penalized[0][1])
    assert penalized[0][1] == pytest.approx(6.0)


def test_unknown_free_has_normal_cost():
    from global_planner.astar import build_cost_map

    cost_map = build_cost_map(
        grid=[[0, -1, 0]],
        unknown_policy='free',
        unknown_cost=5.0,
        proximity_weight=0.0,
    )

    assert cost_map[0][1] == pytest.approx(1.0)


def test_proximity_cost_decreases_with_clearance():
    from global_planner.astar import build_cost_map

    grid = [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [100, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
    ]

    cost_map = build_cost_map(
        grid=grid,
        unknown_policy='free',
        unknown_cost=5.0,
        proximity_weight=2.0,
    )

    near_cost = cost_map[2][1]
    far_cost = cost_map[2][4]

    assert near_cost > far_cost
    assert math.isinf(cost_map[2][0])


def test_invalid_unknown_policy_is_rejected():
    from global_planner.astar import build_cost_map

    with pytest.raises(ValueError):
        build_cost_map(
            grid=[[0]],
            unknown_policy='invalid',
            unknown_cost=5.0,
            proximity_weight=2.0,
        )


@pytest.mark.skip(reason='TP-31: planner scenario matrix')
def test_u_shape_escape():
    pass


@pytest.mark.skip(reason='TP-31: planner scenario matrix')
def test_narrow_corridor():
    pass


def test_binary_cost_map_blocks_occupied_and_unknown():
    from global_planner.astar import binary_cost_map

    cost_map = binary_cost_map(
        data=[
            0, 100,
            -1, 0,
        ],
        width=2,
        height=2,
    )

    assert cost_map[0][0] == pytest.approx(1.0)
    assert math.isinf(cost_map[0][1])
    assert math.isinf(cost_map[1][0])
    assert cost_map[1][1] == pytest.approx(1.0)


def test_world_to_cell():
    from global_planner.astar import world_to_cell

    assert world_to_cell(
        x=0.25,
        y=0.25,
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.5,
        width=4,
        height=4,
    ) == (0, 0)

    assert world_to_cell(
        x=1.25,
        y=0.75,
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.5,
        width=4,
        height=4,
    ) == (2, 1)


def test_cell_to_world_returns_cell_center():
    from global_planner.astar import cell_to_world

    from_world = cell_to_world(
        cell=(2, 1),
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.5,
    )

    assert from_world == pytest.approx(
        (1.25, 0.75)
    )
