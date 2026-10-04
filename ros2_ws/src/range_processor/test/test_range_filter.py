"""Tests unitarios de range_processor.range_filter (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select range_processor && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-19 / TP-20 / TP-21')

def test_nan_and_inf_classified():
    pass

def test_out_of_range_invalidated():
    pass

def test_single_spike_rejected():
    pass

def test_closing_rate_sign():
    pass

def test_ttc_infinite_when_receding():
    pass

