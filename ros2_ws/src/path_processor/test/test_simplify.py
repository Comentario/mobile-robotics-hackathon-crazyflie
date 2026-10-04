"""Tests unitarios de path_processor.simplify (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select path_processor && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-30')

def test_collinear_points_removed():
    pass

def test_shortcut_respects_obstacles():
    pass

def test_densify_max_spacing():
    pass

