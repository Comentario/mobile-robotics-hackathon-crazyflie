import math
import unittest

from plan_shared_map import alignment, astar, inflate, plan, transform


class SharedMapPlanningTests(unittest.TestCase):
    def snapshot(self):
        return dict(session_id='flight1', width=60, height=60, resolution=.1,
                    origin=[-3., -3.], layers=[dict(height_m=.8, tolerance_m=.045,
                                                  data=[0]*3600)])

    def test_measured_alignment_with_rotation_and_nonzero_a_origin(self):
        a = dict(session_id='flight1', initial_pose=dict(x=1., y=2., yaw_deg=0.))
        b = dict(header=dict(frame_id='world'), pose=dict(position=dict(x=3., y=4., z=0.),
                 orientation=dict(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5))))
        tf = alignment(self.snapshot(), a, b)
        point = transform([2., 2., .8], tf)
        self.assertAlmostEqual(point[0], 3.)
        self.assertAlmostEqual(point[1], 5.)
        original = transform(point, tf, inverse=True)
        self.assertAlmostEqual(original[0], 2.)
        self.assertAlmostEqual(original[1], 2.)
        a['session_id'] = 'other_flight'
        with self.assertRaisesRegex(ValueError, 'different map session'):
            alignment(self.snapshot(), a, b)

    def test_unknown_cells_and_clearance_are_blocked(self):
        data = [0]*400
        data[10*20+10] = -1
        blocked = inflate(data, 20, 20, .1, .2)
        self.assertTrue(blocked[10*20+12])
        self.assertFalse(blocked[10*20+15])
        self.assertTrue(blocked[0])

    def test_no_diagonal_corner_cutting(self):
        blocked = [False, True, True, False]
        with self.assertRaisesRegex(ValueError, 'No route'):
            astar(blocked, 2, 2, (0, 0), (1, 1))

    def test_detour_preserves_exact_goal_and_clearance(self):
        s = self.snapshot()
        data = s['layers'][0]['data']
        for y in range(20, 41):
            data[y*60+30] = 100
        mission = dict(frame_id='map', waypoints=[[1., 0., .8], [1., 1.5, .8]])
        result = plan(s, mission, [-1., 0., .8], [0., 0., 0.], .1, .15)
        self.assertEqual(result['legs'][0]['points'][-1], [1., 0., .8])
        self.assertEqual(result['modified'][-1], mission['waypoints'][-1])
        self.assertGreater(max(abs(p[1]) for p in result['legs'][0]['points']), 1.)
        for p in result['modified']:
            x, y = math.floor((p[0]+3)/.1), math.floor((p[1]+3)/.1)
            self.assertEqual(result['grid']['inflated'][y*60+x], 0)

    def test_no_partial_route_when_later_waypoint_is_blocked(self):
        s = self.snapshot()
        s['layers'][0]['data'][30*60+40] = 100
        with self.assertRaisesRegex(ValueError, 'Start or waypoint'):
            plan(s, dict(frame_id='map', waypoints=[[0., 1., .8], [1., 0., .8]]),
                 [-1., 0., .8], [0., 0., 0.], .1, .15)

    def test_wrong_height_cannot_be_reused(self):
        s = self.snapshot()
        s['layers'][0]['height_m'] = .3
        with self.assertRaisesRegex(ValueError, 'No measured map layer'):
            plan(s, dict(frame_id='map', waypoints=[[1., 0., .8]]),
                 [-1., 0., .8], [0., 0., 0.], .1, .15)

    def test_simulation_is_explicitly_restricted_to_preview(self):
        s = self.snapshot()
        s['simulation'] = True
        with self.assertRaisesRegex(ValueError, 'Simulation map'):
            plan(s, dict(frame_id='map', waypoints=[[1., 0., .8]]),
                 [-1., 0., .8], [0., 0., 0.], .1, .15)

    def test_geofence_rejects_goal_without_body_clearance(self):
        with self.assertRaisesRegex(ValueError, 'Start or waypoint'):
            plan(self.snapshot(), dict(frame_id='map', waypoints=[[1.9, 0., .8]]),
                 [-1., 0., .8], [0., 0., 0.], .1, .15)


if __name__ == '__main__':
    unittest.main()
