import pytest
from local_navigator.route_follower import RouteFollower


def test_ordered_route_does_not_jump_across_obstacle_to_nearest_later_point():
    follower = RouteFollower()
    follower.set_path([(0., 0., .8), (0., 1., .8), (1., 1., .8), (1., 0., .8)])
    assert follower.target((0., 0., .8)) == (0., 1., .8)
    assert follower.target((1., 0., .8)) == (0., 1., .8)
    assert follower.target((0., 1., .8)) == (1., 1., .8)


def test_identical_route_refresh_preserves_progress_and_new_route_resets():
    follower = RouteFollower()
    points = [(0., 0., .8), (0., 1., .8), (1., 1., .8)]
    follower.set_path(points)
    follower.target((0., 0., .8))
    follower.set_path(points)
    assert follower.index == 1
    follower.set_path([(0., 0., .8), (-1., 0., .8)])
    assert follower.index == 0
    follower.clear()
    assert follower.target((0., 0., .8)) is None


def test_invalid_route_is_rejected():
    with pytest.raises(ValueError):
        RouteFollower().set_path([(float('nan'), 0., .8)])


def test_nearby_overshoot_advances_in_order_without_far_jumps():
    follower = RouteFollower(.03, .15)
    follower.set_path([(0., 0., .8), (.05, 0., .8), (.10, 0., .8), (.15, 0., .8)])
    # A velocity-controlled vehicle passes (.05, 0) 4 cm off: it counts as reached.
    assert follower.target((.06, .04, .8)) == (.10, 0., .8)
    # Far from the current point nothing is skipped, even if past it along the route.
    follower.set_path([(0., 0., .8), (0., 1., .8), (1., 1., .8)])
    follower.target((0., 0., .8))
    assert follower.target((.5, 1.5, .8)) == (0., 1., .8)


def test_start_anchor_counts_as_reached_inside_pass_radius():
    follower = RouteFollower(.03, .15)
    follower.set_path([(-1.625, .875, .3), (-1.625, .825, .3), (-1.625, .775, .3)])
    # The vehicle stopped 4.5 cm short of the previous waypoint (leg start).
    assert follower.target((-1.58, .875, .3)) == (-1.625, .825, .3)
