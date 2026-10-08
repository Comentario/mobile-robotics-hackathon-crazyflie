"""Grid path-planning helpers for TP-27 and later planning TPs."""

import heapq
import math
import time


def build_cost_map(
    grid,
    unknown_policy,
    unknown_cost,
    proximity_weight,
    occupied_threshold=50,
):
    """Build a traversal-cost map from occupancy values.

    Occupancy values follow nav_msgs/OccupancyGrid conventions:
      -1       -> unknown
      0..100   -> known occupancy probability/value

    Unknown-space policies:
      block    -> unknown cells are not traversable
      free     -> unknown cells use the normal traversal cost
      penalize -> unknown cells remain traversable with extra cost

    Traversable cells have a base cost of 1.0. A configurable inverse
    distance penalty encourages paths to keep clearance from blocked cells.
    """
    if unknown_policy not in ('block', 'penalize', 'free'):
        raise ValueError(
            'unknown_policy must be "block", "penalize", or "free"'
        )

    unknown_cost = float(unknown_cost)
    proximity_weight = float(proximity_weight)
    occupied_threshold = int(occupied_threshold)

    if not math.isfinite(unknown_cost) or unknown_cost < 0.0:
        raise ValueError(
            'unknown_cost must be non-negative and finite'
        )

    if (
        not math.isfinite(proximity_weight)
        or proximity_weight < 0.0
    ):
        raise ValueError(
            'proximity_weight must be non-negative and finite'
        )

    if not 0 <= occupied_threshold <= 100:
        raise ValueError(
            'occupied_threshold must be in [0, 100]'
        )

    if not grid or not grid[0]:
        raise ValueError('grid must not be empty')

    width = len(grid[0])
    height = len(grid)

    for row in grid:
        if len(row) != width:
            raise ValueError('grid must be rectangular')

    cost_map = [
        [1.0] * width
        for _ in range(height)
    ]

    blocked_cells = []

    for y, row in enumerate(grid):
        for x, raw_value in enumerate(row):
            value = int(raw_value)

            if value < -1 or value > 100:
                raise ValueError(
                    'occupancy values must be in [-1, 100]'
                )

            if value == -1:
                if unknown_policy == 'block':
                    cost_map[y][x] = math.inf
                    blocked_cells.append((x, y))
                elif unknown_policy == 'penalize':
                    cost_map[y][x] = 1.0 + unknown_cost
                else:
                    cost_map[y][x] = 1.0

                continue

            if value >= occupied_threshold:
                cost_map[y][x] = math.inf
                blocked_cells.append((x, y))
            else:
                cost_map[y][x] = 1.0

    if proximity_weight == 0.0 or not blocked_cells:
        return cost_map

    distance_map = [
        [math.inf] * width
        for _ in range(height)
    ]

    queue = []

    for x, y in blocked_cells:
        distance_map[y][x] = 0.0
        heapq.heappush(
            queue,
            (0.0, x, y),
        )

    neighbor_steps = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    )

    while queue:
        distance, x, y = heapq.heappop(queue)

        if distance > distance_map[y][x]:
            continue

        for dx, dy, step in neighbor_steps:
            nx = x + dx
            ny = y + dy

            if not (
                0 <= nx < width
                and 0 <= ny < height
            ):
                continue

            candidate = distance + step

            if candidate >= distance_map[ny][nx]:
                continue

            distance_map[ny][nx] = candidate

            heapq.heappush(
                queue,
                (candidate, nx, ny),
            )

    for y in range(height):
        for x in range(width):
            if not math.isfinite(cost_map[y][x]):
                continue

            distance = distance_map[y][x]

            if not math.isfinite(distance) or distance <= 0.0:
                continue

            cost_map[y][x] += (
                proximity_weight
                / distance
            )

    return cost_map


def _validate_cost_map(cost_map):
    """Validate a rectangular 2-D traversal-cost map."""
    if not cost_map or not cost_map[0]:
        raise ValueError('cost_map must not be empty')

    width = len(cost_map[0])

    for row in cost_map:
        if len(row) != width:
            raise ValueError('cost_map must be rectangular')

        for value in row:
            value = float(value)

            if math.isnan(value):
                raise ValueError('cost_map must not contain NaN')

            if value < 0.0:
                raise ValueError(
                    'cost_map traversal costs must be non-negative'
                )

    return width, len(cost_map)


def _is_traversable(cost_map, cell):
    """Return True when a cell is inside the map and has finite cost."""
    x, y = cell

    if y < 0 or y >= len(cost_map):
        return False

    if x < 0 or x >= len(cost_map[0]):
        return False

    return math.isfinite(float(cost_map[y][x]))


def _heuristic(a, b, connectivity, minimum_cost):
    """Admissible A* heuristic for four- or eight-connected grids."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])

    if connectivity == 4:
        distance = dx + dy
    else:
        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal
        distance = (
            math.sqrt(2.0) * diagonal
            + straight
        )

    return minimum_cost * distance


def _neighbors(cost_map, cell, connectivity):
    """Yield valid neighboring cells and geometric step lengths."""
    x, y = cell

    cardinal = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
    )

    diagonal = (
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    )

    for dx, dy, distance in cardinal:
        candidate = (x + dx, y + dy)

        if _is_traversable(cost_map, candidate):
            yield candidate, distance

    if connectivity != 8:
        return

    for dx, dy, distance in diagonal:
        candidate = (x + dx, y + dy)

        if not _is_traversable(cost_map, candidate):
            continue

        # Do not cut diagonally through an occupied corner.
        horizontal = (x + dx, y)
        vertical = (x, y + dy)

        if not _is_traversable(cost_map, horizontal):
            continue

        if not _is_traversable(cost_map, vertical):
            continue

        yield candidate, distance


def _reconstruct_path(came_from, current):
    """Reconstruct a path from start to goal."""
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar(
    cost_map,
    start,
    goal,
    connectivity=8,
    timeout=None,
):
    """Find a path through a 2-D traversal-cost map.

    Cells are represented as ``(x, y)`` tuples and indexed as
    ``cost_map[y][x]``.

    Finite values represent traversable cells. ``math.inf`` represents
    blocked cells.

    The returned path includes both start and goal. ``None`` is returned
    when no path exists or when the optional timeout is exceeded.
    """
    width, height = _validate_cost_map(cost_map)

    if connectivity not in (4, 8):
        raise ValueError('connectivity must be 4 or 8')

    if timeout is not None:
        timeout = float(timeout)

        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('timeout must be positive and finite')

    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    if not (
        0 <= start[0] < width
        and 0 <= start[1] < height
    ):
        return None

    if not (
        0 <= goal[0] < width
        and 0 <= goal[1] < height
    ):
        return None

    if not _is_traversable(cost_map, start):
        return None

    if not _is_traversable(cost_map, goal):
        return None

    if start == goal:
        return [start]

    finite_costs = [
        float(value)
        for row in cost_map
        for value in row
        if math.isfinite(float(value))
    ]

    minimum_cost = min(finite_costs)

    start_time = time.monotonic()

    open_heap = []
    counter = 0

    g_score = {
        start: 0.0,
    }

    came_from = {}

    start_priority = _heuristic(
        start,
        goal,
        connectivity,
        minimum_cost,
    )

    heapq.heappush(
        open_heap,
        (
            start_priority,
            counter,
            start,
        ),
    )

    closed = set()

    while open_heap:
        if (
            timeout is not None
            and time.monotonic() - start_time > timeout
        ):
            return None

        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == goal:
            return _reconstruct_path(
                came_from,
                current,
            )

        closed.add(current)

        for neighbor, step_distance in _neighbors(
            cost_map,
            current,
            connectivity,
        ):
            if neighbor in closed:
                continue

            nx, ny = neighbor

            traversal_cost = float(
                cost_map[ny][nx]
            )

            tentative_g = (
                g_score[current]
                + step_distance * traversal_cost
            )

            if tentative_g >= g_score.get(
                neighbor,
                math.inf,
            ):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g

            counter += 1

            priority = (
                tentative_g
                + _heuristic(
                    neighbor,
                    goal,
                    connectivity,
                    minimum_cost,
                )
            )

            heapq.heappush(
                open_heap,
                (
                    priority,
                    counter,
                    neighbor,
                ),
            )

    return None


def direct_path(start, goal, spacing):
    """Sample a straight path from start to goal.

    The endpoints are always included and adjacent samples are separated
    by no more than ``spacing``.
    """
    spacing = float(spacing)

    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('spacing must be positive and finite')

    start = tuple(float(value) for value in start)
    goal = tuple(float(value) for value in goal)

    if len(start) != len(goal):
        raise ValueError(
            'start and goal must have the same dimension'
        )

    if not start:
        raise ValueError(
            'start and goal must not be empty'
        )

    if not all(
        math.isfinite(value)
        for value in start + goal
    ):
        raise ValueError(
            'start and goal must contain finite values'
        )

    delta = tuple(
        goal_value - start_value
        for start_value, goal_value
        in zip(start, goal)
    )

    distance = math.sqrt(
        sum(value * value for value in delta)
    )

    if distance == 0.0:
        return [start]

    segment_count = max(
        1,
        math.ceil(distance / spacing),
    )

    return [
        tuple(
            start_value
            + delta_value * index / segment_count
            for start_value, delta_value
            in zip(start, delta)
        )
        for index in range(segment_count + 1)
    ]


def binary_cost_map(data, width, height, occupied_threshold=50):
    """Convert OccupancyGrid-style data to a binary traversal-cost map.

    This is the TP-27 baseline conversion:
      known cells below occupied_threshold -> traversal cost 1.0
      occupied cells -> blocked
      unknown cells (-1) -> blocked

    TP-28 replaces this baseline with configurable unknown and cost policies.
    """
    width = int(width)
    height = int(height)

    if width <= 0 or height <= 0:
        raise ValueError('width and height must be positive')

    if len(data) != width * height:
        raise ValueError(
            'grid data length does not match width * height'
        )

    result = []

    for y in range(height):
        row = []

        for x in range(width):
            value = int(data[y * width + x])

            if value < 0 or value >= occupied_threshold:
                row.append(math.inf)
            else:
                row.append(1.0)

        result.append(row)

    return result


def world_to_cell(
    x,
    y,
    origin_x,
    origin_y,
    resolution,
    width,
    height,
):
    """Convert a world position to an OccupancyGrid cell."""
    resolution = float(resolution)

    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError('resolution must be positive and finite')

    cell_x = math.floor(
        (float(x) - float(origin_x))
        / resolution
    )

    cell_y = math.floor(
        (float(y) - float(origin_y))
        / resolution
    )

    if not (
        0 <= cell_x < int(width)
        and 0 <= cell_y < int(height)
    ):
        return None

    return int(cell_x), int(cell_y)


def cell_to_world(
    cell,
    origin_x,
    origin_y,
    resolution,
):
    """Return the world coordinates of an OccupancyGrid cell center."""
    cell_x, cell_y = cell

    return (
        float(origin_x)
        + (float(cell_x) + 0.5) * float(resolution),
        float(origin_y)
        + (float(cell_y) + 0.5) * float(resolution),
    )
