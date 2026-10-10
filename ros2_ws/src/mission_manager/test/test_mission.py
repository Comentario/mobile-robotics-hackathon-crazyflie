"""Unit tests for the pure TP-26 mission logic."""

import yaml

from mission_manager.mission import (
    ABORTED,
    COMPLETED,
    NAVIGATE,
    PAUSED,
    PREFLIGHT,
    SAFETY_LAND,
    SAFETY_NORMAL,
    TAKEOFF,
    MissionStateMachine,
    Waypoint,
    load_mission,
)


GOOD = 0
DEGRADED = 1
PLANNER_IDLE = 0


def make_machine(waypoints=None, timeout=10.0):
    if waypoints is None:
        waypoints = [Waypoint(1.0, 0.0, 0.5)]

    return MissionStateMachine(
        waypoints=waypoints,
        tolerance=0.1,
        timeout=timeout,
        takeoff_height=0.5,
    )


def reach_navigation(machine):
    ok, _ = machine.start(0.0)
    assert ok

    action = machine.step(
        now=0.0,
        position=(0.0, 0.0, 0.0),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )
    assert action == ('takeoff',)
    assert machine.state == TAKEOFF

    action = machine.step(
        now=2.0,
        position=(0.0, 0.0, 0.5),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )
    assert action[0] == 'goal'
    assert machine.state == NAVIGATE


def test_load_mission_yaml(tmp_path):
    mission_file = tmp_path / 'mission.yaml'
    mission_file.write_text(
        yaml.safe_dump(
            {
                'frame_id': 'map',
                'waypoints': [
                    [1.0, 2.0, 0.5],
                    [-1.0, 0.0, 0.6],
                ],
            }
        ),
        encoding='utf-8',
    )

    waypoints = load_mission(mission_file)

    assert waypoints == [
        Waypoint(1.0, 2.0, 0.5),
        Waypoint(-1.0, 0.0, 0.6),
    ]


def test_preflight_requires_good_health():
    machine = make_machine()
    machine.start(0.0)

    action = machine.step(
        now=0.0,
        position=(0.0, 0.0, 0.0),
        health=DEGRADED,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )

    assert action == ('none',)
    assert machine.state == PREFLIGHT

    action = machine.step(
        now=0.1,
        position=(0.0, 0.0, 0.0),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )

    assert action == ('takeoff',)
    assert machine.state == TAKEOFF


def test_advances_when_within_tolerance():
    machine = make_machine(
        [
            Waypoint(1.0, 0.0, 0.5),
            Waypoint(2.0, 0.0, 0.5),
        ]
    )
    reach_navigation(machine)

    action = machine.step(
        now=3.0,
        position=(1.0, 0.0, 0.5),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )

    assert machine.index == 1
    assert action == ('goal', Waypoint(2.0, 0.0, 0.5))


def test_completes_after_last_waypoint():
    machine = make_machine()
    reach_navigation(machine)

    action = machine.step(
        now=3.0,
        position=(1.0, 0.0, 0.5),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )

    assert action == ('land',)

    machine.step(
        now=5.0,
        position=(1.0, 0.0, 0.0),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )

    assert machine.state == COMPLETED


def test_waypoint_timeout_aborts():
    machine = make_machine(timeout=2.0)
    reach_navigation(machine)

    action = machine.step(
        now=4.1,
        position=(0.0, 0.0, 0.5),
        health=GOOD,
        safety_state=SAFETY_NORMAL,
        planner_status=PLANNER_IDLE,
    )

    assert action == ('none',)
    assert machine.state == ABORTED
    assert machine.reason == 'waypoint_timeout'


def test_safety_land_aborts_mission():
    machine = make_machine()
    reach_navigation(machine)

    machine.step(
        now=3.0,
        position=(0.0, 0.0, 0.5),
        health=GOOD,
        safety_state=SAFETY_LAND,
        planner_status=PLANNER_IDLE,
    )

    assert machine.state == ABORTED
    assert machine.reason == 'safety_land'


def test_pause_and_resume():
    machine = make_machine()
    reach_navigation(machine)

    ok, _ = machine.pause()

    assert ok
    assert machine.state == PAUSED

    ok, _ = machine.resume(now=5.0)

    assert ok
    assert machine.state == NAVIGATE


def test_start_can_restart_terminal_mission():
    machine = make_machine()

    machine.start(0.0)
    machine.abort('operator_abort')

    ok, _ = machine.start(10.0)

    assert ok
    assert machine.state == PREFLIGHT
    assert machine.index == 0
    assert machine.reason == ''


def test_hover_suspends_waypoint_deadline():
    from mission_manager.mission import MissionStateMachine,Waypoint,SAFETY_HOVER
    m=MissionStateMachine([Waypoint(1,0,.5)],.15,3,takeoff_height=.5)
    m.start(0);m.step(0,(0,0,.5),0,0,0);m.step(1,(0,0,.5),0,0,0)
    assert m.step(2,(0,0,.5),0,SAFETY_HOVER,0)==('none',)
    m.step(10,(0,0,.5),0,SAFETY_HOVER,0)
    assert m.step(11,(0,0,.5),0,0,0)[0]=='goal'
