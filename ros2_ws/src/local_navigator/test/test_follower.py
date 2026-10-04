"""Tests unitarios de local_navigator.follower (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select local_navigator && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-32 / TP-33 / TP-34 / TP-35 / TP-36')

def test_lookahead_on_straight_path():
    pass

def test_slowdown_near_goal():
    pass

def test_rate_limiter_bounds_change():
    pass

def test_holds_z_nominal():
    pass

