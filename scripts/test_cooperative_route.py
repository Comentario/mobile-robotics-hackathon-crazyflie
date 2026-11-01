import unittest
from cooperative_route import find_leg, validate_route


class CooperativeRouteTests(unittest.TestCase):
    def test_static_route_selection_and_unknown_goal(self):
        legs = [dict(waypoint_index=0, points=[[0., 0., .8], [1., 0., .8]]),
                dict(waypoint_index=1, points=[[1., 0., .8], [1., 1., .8]])]
        result = dict(frame_id='map', modified=[1], legs=legs)
        self.assertEqual(validate_route(result), legs)
        self.assertIs(find_leg(legs, [1., 1., .8]), legs[1])
        self.assertIsNone(find_leg(legs, [2., 0., .8]))

    def test_blocked_or_synthetic_route_cannot_silently_be_enabled(self):
        with self.assertRaises(ValueError):
            validate_route(dict(frame_id='map', modified=[], legs=[]), True)
        with self.assertRaises(ValueError):
            validate_route(dict(simulation=True))


if __name__ == '__main__':
    unittest.main()
