"""Tests unitarios de global_planner.astar (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select global_planner && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-27 / TP-28 / TP-29 / TP-31')

def test_free_space_finds_straight_path():
    pass

def test_wall_detour():
    pass

def test_u_shape_escape():
    pass

def test_narrow_corridor():
    pass

def test_no_path_returns_none():
    pass

def test_unknown_policy_block_vs_penalize():
    pass

def test_direct_path_spacing():
    pass

