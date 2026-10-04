"""Tests unitarios de simulation_adapter.dynamics (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select simulation_adapter && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-10 / TP-41')

def test_reaches_position_setpoint():
    pass

def test_respects_max_speed():
    pass

def test_hover_after_command_watchdog():
    pass

def test_emergency_stop_falls():
    pass

