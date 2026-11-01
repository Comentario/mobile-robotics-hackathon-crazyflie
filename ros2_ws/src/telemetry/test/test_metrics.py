"""Unit tests for telemetry.metrics (pure logic, no ROS)."""
import json
import math

from telemetry.metrics import DRONE_RADIUS, RunMetrics, box_distance

WALL = ((0.0, 0.0, 1.0), (0.1, 1.2, 2.0))       # T03 wall at x=0
ROOM_WALL = ((-2.0, 0.0, 1.0), (0.02, 4.0, 2.0))


def test_box_distance():
    assert box_distance((0.0, 0.0, 1.0), *WALL) == 0.0
    assert math.isclose(box_distance((0.55, 0.0, 1.0), *WALL), 0.5)


def test_clearance_and_collision_counting():
    m = RunMetrics()
    m.update_truth((-1.0, 0.0, 0.5), [WALL], [ROOM_WALL])
    assert math.isclose(m.min_obstacle_distance, 0.95 - DRONE_RADIUS)
    assert m.collisions == 0
    m.update_truth((-0.08, 0.0, 0.5), [WALL], [ROOM_WALL])   # touching the wall
    m.update_truth((-0.07, 0.0, 0.5), [WALL], [ROOM_WALL])   # same contact
    m.update_truth((-1.0, 0.0, 0.5), [WALL], [ROOM_WALL])
    m.update_truth((-1.95, 0.0, 0.5), [WALL], [ROOM_WALL])   # room wall: new contact
    assert m.collisions == 2
    assert m.min_obstacle_distance == 0.0


def test_one_stale_event_per_silence():
    m = RunMetrics()
    m.note_state_age(0.02, 0.3)          # the stream is healthy
    assert m.stale_state_events == 0
    m.note_state_age(0.5, 0.3)           # it goes quiet
    m.note_state_age(1.0, 0.3)           # still quiet: the same gap, not a new one
    m.note_state_age(4.0, 0.3)           # it never comes back
    assert m.stale_state_events == 1
    m.note_state_age(0.02, 0.3)          # it recovers
    m.note_state_age(0.9, 0.3)           # and goes quiet again: a second event
    assert m.stale_state_events == 2


def test_to_dict_is_json_serializable():
    m = RunMetrics()
    m.safety_states_seen.update({'SLOW', 'NORMAL'})
    data = m.to_dict()
    assert data['safety_states_seen'] == ['NORMAL', 'SLOW']
    assert data['min_obstacle_distance'] is None
    assert 'in_collision' not in data and 'state_stale' not in data
    json.dumps(data)
