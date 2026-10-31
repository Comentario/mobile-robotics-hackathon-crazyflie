"""Unit tests for TP-30 path simplification."""

import math

import pytest

from path_processor.simplify import (
    densify,
    occupancy_segment_is_free,
    remove_collinear,
    shortcut,
)


def test_collinear_points_removed():
    points = [
        (0.0, 0.0, 0.5),
        (1.0, 0.0, 0.5),
        (2.0, 0.0, 0.5),
        (3.0, 0.0, 0.5),
    ]

    result = remove_collinear(points)

    assert result == [
        (0.0, 0.0, 0.5),
        (3.0, 0.0, 0.5),
    ]


def test_corner_is_preserved():
    points = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
    ]

    assert remove_collinear(points) == points


def test_shortcut_uses_farthest_visible_point():
    points = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]

    def segment_is_free(start, end):
        return math.dist(start, end) <= 2.1

    result = shortcut(
        points,
        segment_is_free,
    )

    assert result == [
        (0.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]


def test_densify_max_spacing():
    result = densify(
        [
            (0.0, 0.0, 0.5),
            (1.0, 0.0, 0.5),
        ],
        max_spacing=0.3,
    )

    assert result[0] == pytest.approx(
        (0.0, 0.0, 0.5)
    )

    assert result[-1] == pytest.approx(
        (1.0, 0.0, 0.5)
    )

    for first, second in zip(
        result,
        result[1:],
    ):
        assert math.dist(
            first,
            second,
        ) <= 0.3 + 1e-12


def test_grid_line_of_sight_is_clear():
    data = [
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
    ]

    assert occupancy_segment_is_free(
        start=(0.5, 1.5),
        end=(2.5, 1.5),
        data=data,
        width=3,
        height=3,
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
    )


def test_grid_line_of_sight_respects_obstacle():
    data = [
        0, 0, 0,
        0, 100, 0,
        0, 0, 0,
    ]

    assert not occupancy_segment_is_free(
        start=(0.5, 1.5),
        end=(2.5, 1.5),
        data=data,
        width=3,
        height=3,
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
    )


def test_unknown_is_conservative_by_default():
    data = [
        0, -1, 0,
    ]

    assert not occupancy_segment_is_free(
        start=(0.5, 0.5),
        end=(2.5, 0.5),
        data=data,
        width=3,
        height=1,
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
    )

    assert occupancy_segment_is_free(
        start=(0.5, 0.5),
        end=(2.5, 0.5),
        data=data,
        width=3,
        height=1,
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
        unknown_is_free=True,
    )
