"""Tests unitarios de safety_supervisor.geofence (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select safety_supervisor && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-37 / TP-38 / TP-39')

def test_inside_point():
    pass

def test_clamp_outside_point():
    pass

def test_altitude_limits():
    pass

