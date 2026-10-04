"""Tests unitarios de mission_manager.mission (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select mission_manager && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-26')

def test_load_mission_yaml():
    pass

def test_preflight_requires_good_health():
    pass

def test_advances_when_within_tolerance():
    pass

def test_completes_after_last_waypoint():
    pass

def test_waypoint_timeout_aborts():
    pass

def test_safety_land_aborts_mission():
    pass

