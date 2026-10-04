"""Tests unitarios de safety_supervisor.watchdog (lógica pura, sin ROS).

Ejecutar: colcon test --packages-select safety_supervisor && colcon test-result --verbose
"""
import pytest

pytestmark = pytest.mark.skip(reason='TODO: implementar junto con TP-37 / TP-38 / TP-39')

def test_never_stamped_is_stale():
    pass

def test_stale_after_timeout():
    pass

