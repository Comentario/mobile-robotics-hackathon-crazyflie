"""Pure path simplification helpers for TP-30."""

import math


def _normalize_points(points):
    """Convert path points to finite tuples."""
    result = []

    for point in points:
        values = tuple(float(value) for value in point)

        if len(values) < 2:
            raise ValueError(
                'path points must have at least two dimensions'
            )

        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                'path points must contain finite values'
            )

        result.append(values)

    return result


def remove_collinear(points, tolerance=1e-3):
    """Remove intermediate points lying on the same segment."""
    tolerance = float(tolerance)

    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError(
            'tolerance must be non-negative and finite'
        )

    points = _normalize_points(points)

    if len(points) <= 2:
        return points

    result = [points[0]]

    for index in range(1, len(points) - 1):
        start = result[-1]
        middle = points[index]
        end = points[index + 1]

        if not (
            len(start) == len(middle) == len(end)
        ):
            raise ValueError(
                'all path points must have the same dimension'
            )

        segment = tuple(
            end_value - start_value
            for start_value, end_value
            in zip(start, end)
        )

        segment_norm_sq = sum(
            value * value
            for value in segment
        )

        if segment_norm_sq == 0.0:
            result.append(middle)
            continue

        relative = tuple(
            middle_value - start_value
            for start_value, middle_value
            in zip(start, middle)
        )

        projection = (
            sum(
                relative_value * segment_value
                for relative_value, segment_value
                in zip(relative, segment)
            )
            / segment_norm_sq
        )

        if not 0.0 <= projection <= 1.0:
            result.append(middle)
            continue

        closest = tuple(
            start_value
            + projection * segment_value
            for start_value, segment_value
            in zip(start, segment)
        )

        distance = math.dist(
            middle,
            closest,
        )

        if distance > tolerance:
            result.append(middle)

    result.append(points[-1])

    return result


def shortcut(points, segment_is_free):
    """Greedily join the farthest visible path points."""
    points = _normalize_points(points)

    if len(points) <= 2:
        return points

    result = [points[0]]
    current_index = 0

    while current_index < len(points) - 1:
        next_index = current_index + 1

        for candidate_index in range(
            len(points) - 1,
            current_index,
            -1,
        ):
            if segment_is_free(
                points[current_index],
                points[candidate_index],
            ):
                next_index = candidate_index
                break

        result.append(points[next_index])
        current_index = next_index

    return result


def densify(points, max_spacing):
    """Insert points so no segment exceeds max_spacing."""
    max_spacing = float(max_spacing)

    if (
        not math.isfinite(max_spacing)
        or max_spacing <= 0.0
    ):
        raise ValueError(
            'max_spacing must be positive and finite'
        )

    points = _normalize_points(points)

    if len(points) <= 1:
        return points

    result = [points[0]]

    for start, end in zip(
        points,
        points[1:],
    ):
        if len(start) != len(end):
            raise ValueError(
                'all path points must have the same dimension'
            )

        distance = math.dist(start, end)

        if distance == 0.0:
            continue

        segment_count = max(
            1,
            math.ceil(distance / max_spacing),
        )

        for index in range(1, segment_count + 1):
            fraction = index / segment_count

            result.append(
                tuple(
                    start_value
                    + fraction
                    * (end_value - start_value)
                    for start_value, end_value
                    in zip(start, end)
                )
            )

    return result


def occupancy_segment_is_free(
    start,
    end,
    data,
    width,
    height,
    origin_x,
    origin_y,
    resolution,
    occupied_threshold=50,
    unknown_is_free=False,
):
    """Check line of sight through an axis-aligned OccupancyGrid."""
    width = int(width)
    height = int(height)
    resolution = float(resolution)
    occupied_threshold = int(occupied_threshold)

    if width <= 0 or height <= 0:
        return False

    if (
        not math.isfinite(resolution)
        or resolution <= 0.0
    ):
        return False

    if len(data) != width * height:
        return False

    if not 0 <= occupied_threshold <= 100:
        raise ValueError(
            'occupied_threshold must be in [0, 100]'
        )

    start = tuple(float(value) for value in start)
    end = tuple(float(value) for value in end)

    if len(start) < 2 or len(end) < 2:
        raise ValueError(
            'start and end must contain x and y'
        )

    def world_to_cell(point):
        cell_x = math.floor(
            (point[0] - float(origin_x))
            / resolution
        )

        cell_y = math.floor(
            (point[1] - float(origin_y))
            / resolution
        )

        return int(cell_x), int(cell_y)

    def cell_is_free(cell):
        cell_x, cell_y = cell

        if not (
            0 <= cell_x < width
            and 0 <= cell_y < height
        ):
            return False

        value = int(
            data[cell_y * width + cell_x]
        )

        if value < 0:
            return bool(unknown_is_free)

        return value < occupied_threshold

    distance = math.hypot(
        end[0] - start[0],
        end[1] - start[1],
    )

    sample_spacing = resolution * 0.25

    sample_count = max(
        1,
        math.ceil(distance / sample_spacing),
    )

    previous_cell = None

    for index in range(sample_count + 1):
        fraction = index / sample_count

        point = (
            start[0]
            + fraction * (end[0] - start[0]),
            start[1]
            + fraction * (end[1] - start[1]),
        )

        cell = world_to_cell(point)

        if not cell_is_free(cell):
            return False

        if (
            previous_cell is not None
            and cell[0] != previous_cell[0]
            and cell[1] != previous_cell[1]
        ):
            # Prevent diagonal shortcuts through occupied corners.
            if not cell_is_free(
                (cell[0], previous_cell[1])
            ):
                return False

            if not cell_is_free(
                (previous_cell[0], cell[1])
            ):
                return False

        previous_cell = cell

    return True
