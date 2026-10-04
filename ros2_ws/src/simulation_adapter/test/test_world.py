"""Tests unitarios de simulation_adapter.world (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select simulation_adapter && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-10 / TP-41')

def test_raycast_hits_wall_in_front():
    pass

def test_raycast_returns_inf_beyond_max_range():
    pass

def test_temporary_obstacle_only_active_in_window():
    pass

def test_moving_obstacle_position():
    pass

def test_collision_detected():
    pass

