"""Tests unitarios de state_monitor.health (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select state_monitor && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-15 / TP-16 / TP-18')

def test_fresh_pose_is_good():
    pass

def test_old_pose_is_degraded_then_lost():
    pass

def test_nan_pose_is_lost():
    pass

def test_position_jump_detected():
    pass

def test_frozen_pose_detected():
    pass

def test_speed_policy_per_status():
    pass

