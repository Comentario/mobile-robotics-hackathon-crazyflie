"""Tests unitarios de simulation_adapter.faults (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select simulation_adapter && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TP-41: inyección avanzada de fallos pendiente')

def test_range_spike_only_in_window():
    pass

def test_pose_freeze_repeats_last_pose():
    pass

def test_pose_dropout_publishes_nothing():
    pass

