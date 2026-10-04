"""Tests unitarios de local_navigator.avoidance (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select local_navigator && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-32 / TP-33 / TP-34 / TP-35 / TP-36')

def test_front_obstacle_stops_forward_motion():
    pass

def test_side_obstacle_pushes_away():
    pass

def test_low_ttc_requests_hover():
    pass

def test_blocked_requests_replan():
    pass

def test_ceiling_blocks_ascent():
    pass

def test_floor_blocks_descent():
    pass

