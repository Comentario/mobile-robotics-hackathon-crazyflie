"""Unit tests for TP-16 localization-driven speed policy."""

import pytest

from local_navigator.speed_policy import navigation_speed


def test_good_uses_nominal_speed():
    assert navigation_speed(0, 0.3, 0.15) == pytest.approx(0.3)


def test_degraded_uses_reduced_speed():
    assert navigation_speed(1, 0.3, 0.15) == pytest.approx(0.15)


def test_lost_disables_normal_navigation():
    assert navigation_speed(2, 0.3, 0.15) is None


def test_unknown_status_disables_normal_navigation():
    assert navigation_speed(99, 0.3, 0.15) is None


def test_rejects_nonpositive_speed():
    with pytest.raises(ValueError):
        navigation_speed(0, 0.0, 0.15)


def test_rejects_degraded_speed_above_nominal():
    with pytest.raises(ValueError):
        navigation_speed(0, 0.3, 0.4)
