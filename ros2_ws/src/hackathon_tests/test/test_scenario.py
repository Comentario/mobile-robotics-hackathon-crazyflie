"""Unit tests for hackathon_tests.scenario (pure logic, no ROS).

Run: colcon test --packages-select hackathon_tests && colcon test-result --verbose
"""
from pathlib import Path

import pytest

from hackathon_tests.scenario import evaluate, load_scenario

SCENARIOS = Path(__file__).resolve().parents[2]/'hackathon_bringup'/'config'/'scenarios'


def test_all_scenarios_load():
    files = sorted(SCENARIOS.glob('*.yaml'))
    assert len(files) >= 16
    for path in files:
        scenario = load_scenario(path)
        assert scenario['expected']


def test_evaluate_pass():
    metrics = {'final_mission_state_name': 'COMPLETED', 'collisions': 0,
               'min_obstacle_distance': 0.3, 'safety_states_seen': ['NORMAL', 'SLOW']}
    passed, failures = evaluate(metrics, {'mission_state': 'COMPLETED', 'max_collisions': 0,
                                          'min_obstacle_distance': 0.2,
                                          'must_see_safety': ['SLOW', 'HOVER']})
    assert passed and failures == []


def test_evaluate_reports_each_failure():
    metrics = {'final_mission_state_name': 'ABORTED', 'collisions': 2,
               'min_obstacle_distance': None, 'safety_states_seen': ['NORMAL'],
               'max_abs_x': 2.4, 'max_abs_y': 0.0, 'max_z': 1.0, 'final_z': 0.5}
    passed, failures = evaluate(metrics, {'mission_state': 'COMPLETED', 'max_collisions': 0,
                                          'must_see_safety': ['HOVER'], 'max_abs_xy': 2.0,
                                          'max_z': 0.8, 'final_z_max': 0.1})
    assert not passed
    assert len(failures) == 6


def test_missing_metrics_never_pass_a_bound():
    passed, failures = evaluate({}, {'mission_state': 'COMPLETED', 'final_z_max': 0.1})
    assert not passed and len(failures) == 2


def test_invalid_scenario_is_rejected(tmp_path):
    path = tmp_path/'bad.yaml'
    path.write_text('world: w.yaml\nmission: m.yaml\ntimeout: 10\nexpected: {max_speed: 1}\n')
    with pytest.raises(ValueError):
        load_scenario(path)
