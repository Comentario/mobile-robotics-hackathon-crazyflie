"""Tests unitarios de safety_supervisor.supervisor (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select safety_supervisor && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-37 / TP-38 / TP-39')

def test_all_good_is_normal():
    pass

def test_degraded_localization_is_slow():
    pass

def test_lost_localization_hovers_then_lands():
    pass

def test_critical_range_hovers():
    pass

def test_stale_state_hovers():
    pass

def test_stale_ranges_slow_or_hover():
    pass

def test_priority_abort_over_everything():
    pass

def test_hysteresis_on_recovery():
    pass

