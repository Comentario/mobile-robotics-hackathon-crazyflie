"""TP-31 reproducible planner scenario matrix."""

import math

from global_planner.astar import astar
from global_planner.replan import ReplanPolicy


def free_grid(width, height):
    """Return a rectangular unit-cost grid."""
    return [
        [1.0] * width
        for _ in range(height)
    ]


def test_matrix_free_space():
    """Planner finds the direct shortest route in free space."""
    grid = free_grid(7, 5)

    path = astar(
        grid,
        start=(0, 2),
        goal=(6, 2),
        connectivity=4,
    )

    assert path is not None
    assert path[0] == (0, 2)
    assert path[-1] == (6, 2)
    assert all(
        y == 2
        for _, y in path
    )


def test_matrix_wall_detour():
    """Planner passes through the available gap around a wall."""
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

    assert any(
        y == 4
        for _, y in path
    )

    assert all(
        math.isfinite(grid[y][x])
        for x, y in path
    )


def test_matrix_u_shape_escape():
    """Planner escapes a U-shaped obstacle through its open side."""
    grid = free_grid(9, 9)

    # Left and right arms.
    for y in range(1, 7):
        grid[y][2] = math.inf
        grid[y][6] = math.inf

    # Closed bottom of the U.
    for x in range(2, 7):
        grid[1][x] = math.inf

    path = astar(
        grid,
        start=(4, 3),
        goal=(8, 3),
        connectivity=4,
    )

    assert path is not None
    assert path[0] == (4, 3)
    assert path[-1] == (8, 3)

    # The only escape from inside the U is above both arms.
    assert any(
        y >= 7
        for _, y in path
    )

    assert all(
        math.isfinite(grid[y][x])
        for x, y in path
    )


def test_matrix_narrow_corridor():
    """Planner traverses a one-cell-wide corridor."""
    grid = [
        [math.inf] * 7
        for _ in range(7)
    ]

    for x in range(7):
        grid[3][x] = 1.0

    path = astar(
        grid,
        start=(0, 3),
        goal=(6, 3),
        connectivity=4,
    )

    assert path is not None
    assert path[0] == (0, 3)
    assert path[-1] == (6, 3)

    assert all(
        y == 3
        for _, y in path
    )


def test_matrix_temporarily_blocked_goal():
    """Planner reports no path while the goal is occupied and recovers."""
    grid = free_grid(5, 5)

    start = (0, 0)
    goal = (4, 4)

    grid[goal[1]][goal[0]] = math.inf

    blocked_path = astar(
        grid,
        start=start,
        goal=goal,
        connectivity=4,
    )

    assert blocked_path is None

    grid[goal[1]][goal[0]] = 1.0

    recovered_path = astar(
        grid,
        start=start,
        goal=goal,
        connectivity=4,
    )

    assert recovered_path is not None
    assert recovered_path[0] == start
    assert recovered_path[-1] == goal


def test_matrix_popup_obstacle_triggers_new_route():
    """A new obstacle on the route triggers replanning around it."""
    grid = free_grid(7, 5)

    start = (0, 2)
    goal = (6, 2)

    initial_path = astar(
        grid,
        start=start,
        goal=goal,
        connectivity=4,
    )

    assert initial_path is not None

    popup_cell = initial_path[
        len(initial_path) // 2
    ]

    assert popup_cell not in (
        start,
        goal,
    )

    grid[
        popup_cell[1]
    ][
        popup_cell[0]
    ] = math.inf

    policy = ReplanPolicy(
        replan_period=2.0,
        max_path_deviation=0.4,
    )

    policy.mark_planned(0.0)

    reason = policy.reason(
        now=1.0,
        path=initial_path,
        position=start,
        goal_changed=False,
        path_blocked=True,
        navigator_request=False,
    )

    assert reason == 'blocked'

    replanned_path = astar(
        grid,
        start=start,
        goal=goal,
        connectivity=4,
    )

    assert replanned_path is not None
    assert replanned_path != initial_path
    assert popup_cell not in replanned_path

    assert all(
        math.isfinite(grid[y][x])
        for x, y in replanned_path
    )
