"""Tests unitarios de telemetry.metrics (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select telemetry && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-40')

def test_to_dict_is_json_serializable():
    pass

