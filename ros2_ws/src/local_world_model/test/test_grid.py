"""Tests unitarios de local_world_model.grid (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select local_world_model && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-22 / TP-23 / TP-24 / TP-25')

def test_front_wall_appears_in_correct_cells():
    pass

def test_free_space_along_ray():
    pass

def test_unobserved_is_unknown():
    pass

def test_recenter_keeps_observations():
    pass

def test_occupied_cell_decays():
    pass

def test_inflation_radius():
    pass

def test_corridor_stays_open():
    pass

