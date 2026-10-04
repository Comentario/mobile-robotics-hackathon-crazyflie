"""Tests unitarios de global_planner.replan (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select global_planner && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-27 / TP-28 / TP-29 / TP-31')

def test_new_obstacle_on_path_triggers_replan():
    pass

def test_deviation_triggers_replan():
    pass

def test_no_replan_when_path_valid():
    pass

