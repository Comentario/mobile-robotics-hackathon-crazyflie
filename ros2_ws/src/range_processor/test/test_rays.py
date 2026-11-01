"""Unit tests for range_processor.rays (pure geometry, no ROS)."""
import math

import pytest

from range_processor.rays import ray_in_map


def close(a, b):
    return all(math.isclose(x, y, abs_tol=1e-9) for x, y in zip(a, b))


def test_front_ray_rotates_with_yaw_and_offset():
    origin, end, hit = ray_in_map((1.0, 2.0, .5, math.pi/2), 'front', 1.0, (.02, 0., 0.), 4.0)
    assert close(origin, (1.0, 2.02, .5))
    assert close(end, (1.0, 3.02, .5))
    assert hit


def test_no_return_is_free_space_to_range_max():
    origin, end, hit = ray_in_map((0., 0., .5, 0.), 'left', math.inf, (0., 0., 0.), 4.0)
    assert close(end, (0., 4.0, .5)) and not hit


def test_too_close_is_a_hit_at_the_sensor_and_nan_is_ignored():
    origin, end, hit = ray_in_map((0., 0., .5, 0.), 'back', -math.inf, (0., 0., 0.), 4.0)
    assert close(origin, end) and hit
    assert ray_in_map((0., 0., .5, 0.), 'back', math.nan, (0., 0., 0.), 4.0) is None


def test_invalid_inputs_are_rejected():
    with pytest.raises(ValueError):
        ray_in_map((0., 0., .5, 0.), 'diagonal', 1.0, (0., 0., 0.), 4.0)
    with pytest.raises(ValueError):
        ray_in_map((0., 0., math.nan, 0.), 'front', 1.0, (0., 0., 0.), 4.0)
