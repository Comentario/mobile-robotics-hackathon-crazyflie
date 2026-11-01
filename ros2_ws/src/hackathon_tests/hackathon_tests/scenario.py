"""Scenario loading and pass/fail evaluation.

Format of config/scenarios/*.yaml: see docs/simulations.md.
Supported `expected` keys:
  mission_state, max_collisions, min_obstacle_distance, must_see_safety, must_see_health,
  must_see_planner, max_abs_xy, max_z, final_z_max
must_see_* lists alternatives: at least one of the listed states must be observed.
"""
import math

import yaml

REQUIRED_KEYS = ('world', 'mission', 'timeout', 'expected')
EXPECTED_KEYS = ('mission_state', 'max_collisions', 'min_obstacle_distance', 'must_see_safety',
                 'must_see_health', 'must_see_planner', 'max_abs_xy', 'max_z', 'final_z_max')
SEEN_FIELDS = {'must_see_safety': 'safety_states_seen',
               'must_see_health': 'health_states_seen',
               'must_see_planner': 'planner_states_seen'}


def load_scenario(path):
    """Read and validate one scenario file; return its dictionary."""
    with open(path, encoding='utf-8') as stream:
        scenario = yaml.safe_load(stream)
    if not isinstance(scenario, dict):
        raise ValueError(f'{path}: a scenario must be a mapping')
    missing = [key for key in REQUIRED_KEYS if key not in scenario]
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    timeout = scenario['timeout']
    if not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError(f'{path}: timeout must be a positive number of seconds')
    expected = scenario['expected'] or {}
    unknown = [key for key in expected if key not in EXPECTED_KEYS]
    if unknown:
        raise ValueError(f'{path}: unsupported expected keys {unknown}')
    for key in SEEN_FIELDS:
        if key in expected and not (isinstance(expected[key], list) and expected[key]):
            raise ValueError(f'{path}: {key} must be a non-empty list')
    scenario['expected'] = expected
    return scenario


def evaluate(metrics, expected):
    """Return (passed, list of readable failures).

    metrics: dictionary published by telemetry on /hk/metrics.
    """
    failures = []

    def number(name):
        value = metrics.get(name)
        return math.inf if value is None else float(value)

    if 'mission_state' in expected:
        state = metrics.get('final_mission_state_name')
        if state != expected['mission_state']:
            failures.append(f'mission_state {state} != {expected["mission_state"]}')
    if 'max_collisions' in expected and number('collisions') > expected['max_collisions']:
        failures.append(f'collisions {int(number("collisions"))} > {expected["max_collisions"]}')
    if 'min_obstacle_distance' in expected:
        distance = number('min_obstacle_distance')
        if distance < expected['min_obstacle_distance']:
            failures.append(f'min_obstacle_distance {distance:.3f} m < '
                            f'{expected["min_obstacle_distance"]} m')
    for key, field in SEEN_FIELDS.items():
        if key in expected:
            seen = set(metrics.get(field) or [])
            if not seen & set(expected[key]):
                failures.append(f'{key}: none of {expected[key]} seen (seen {sorted(seen)})')
    if 'max_abs_xy' in expected:
        reach = max(number('max_abs_x'), number('max_abs_y'))
        if reach > expected['max_abs_xy']:
            failures.append(f'max |x|,|y| {reach:.3f} m > {expected["max_abs_xy"]} m')
    if 'max_z' in expected and number('max_z') > expected['max_z']:
        failures.append(f'max_z {number("max_z"):.3f} m > {expected["max_z"]} m')
    if 'final_z_max' in expected and number('final_z') > expected['final_z_max']:
        failures.append(f'final_z {number("final_z"):.3f} m > {expected["final_z_max"]} m')
    return not failures, failures
