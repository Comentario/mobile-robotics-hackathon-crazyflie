"""Unit tests for TP-29 replanning policy."""

from global_planner.replan import ReplanPolicy


def make_policy():
    return ReplanPolicy(
        replan_period=2.0,
        max_path_deviation=0.4,
    )


def test_new_obstacle_on_path_triggers_replan():
    policy = make_policy()

    reason = policy.reason(
        now=1.0,
        path=[(0.0, 0.0), (1.0, 0.0)],
        position=(0.5, 0.0),
        goal_changed=False,
        path_blocked=True,
        navigator_request=False,
    )

    assert reason == 'blocked'


def test_deviation_triggers_replan():
    policy = make_policy()

    reason = policy.reason(
        now=1.0,
        path=[(0.0, 0.0), (1.0, 0.0)],
        position=(0.5, 1.0),
        goal_changed=False,
        path_blocked=False,
        navigator_request=False,
    )

    assert reason == 'deviation'


def test_no_replan_when_path_valid():
    policy = make_policy()
    policy.mark_planned(0.0)

    reason = policy.reason(
        now=1.0,
        path=[(0.0, 0.0), (1.0, 0.0)],
        position=(0.5, 0.1),
        goal_changed=False,
        path_blocked=False,
        navigator_request=False,
    )

    assert reason is None


def test_periodic_replan():
    policy = make_policy()
    policy.mark_planned(1.0)

    reason = policy.reason(
        now=3.1,
        path=[(0.0, 0.0), (1.0, 0.0)],
        position=(0.5, 0.0),
        goal_changed=False,
        path_blocked=False,
        navigator_request=False,
    )

    assert reason == 'periodic'


def test_goal_change_triggers_replan():
    policy = make_policy()

    reason = policy.reason(
        now=1.0,
        path=[(0.0, 0.0), (1.0, 0.0)],
        position=(0.5, 0.0),
        goal_changed=True,
        path_blocked=False,
        navigator_request=False,
    )

    assert reason == 'goal_changed'


def test_navigator_request_triggers_replan():
    policy = make_policy()

    reason = policy.reason(
        now=1.0,
        path=[(0.0, 0.0), (1.0, 0.0)],
        position=(0.5, 0.0),
        goal_changed=False,
        path_blocked=False,
        navigator_request=True,
    )

    assert reason == 'navigator'
