"""Tests unitarios de cf_adapter.mapping (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select cf_adapter && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-08 / TP-09')

def test_scan_order_matches_sensor_model():
    pass

def test_no_return_becomes_inf():
    pass

def test_velocity_rotated_to_body_frame():
    pass

