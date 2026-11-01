import math
import unittest

from generate_polygon_preview import OBSTACLE, POLYGON, WAYPOINTS, generate, make_map


class PolygonPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.snapshot = make_map()
        cls.result, cls.candidate = generate(cls.snapshot, margin=.15)

    def test_cf2_layer_format_and_explicit_synthetic_provenance(self):
        s = self.snapshot
        self.assertTrue(s['simulation'])
        self.assertFalse(s['map_validated_for_drone1'])
        self.assertEqual((s['width'], s['height'], s['resolution']), (160, 160, .05))
        self.assertEqual(len(s['layers'][0]['data']), s['width']*s['height'])
        self.assertEqual(s['layers'][0]['height_m'], .8)
        self.assertEqual(s['data'], s['layers'][0]['data'])
        self.assertEqual(set(s['data']), {0, 100})

    def test_original_waypoints_are_not_silently_replaced(self):
        self.assertEqual(self.result['requested_waypoints'], WAYPOINTS)
        self.assertEqual(self.result['initial'][1:], WAYPOINTS)
        self.assertEqual(self.result['modified'], [])
        self.assertEqual(self.result['legs'], [])
        self.assertIn('ORIGINAL MISSION BLOCKED', self.result['status'])
        self.assertEqual([p['waypoint_number'] for p in self.result['waypoint_changes']], [2, 3, 4])
        self.assertEqual(self.result['proposed_waypoints'][0], WAYPOINTS[0])

    def test_proposed_route_visits_all_proposed_points_in_order(self):
        self.assertEqual(len(self.candidate['legs']), 5)
        for leg, point in zip(self.candidate['legs'], self.result['proposed_waypoints']):
            self.assertEqual(leg['points'][-1], point)
        self.assertEqual(self.result['proposed_waypoints'][-1], WAYPOINTS[-1])

    def test_proposal_keeps_geometric_clearance_along_every_segment(self):
        xmin, ymin, xmax, ymax = OBSTACLE
        route = self.candidate['modified']
        for a, b in zip(route, route[1:]):
            for i in range(11):
                x, y = (a[k]+(b[k]-a[k])*i/10 for k in (0, 1))
                for (ax, ay), (bx, by) in zip(POLYGON, POLYGON[1:]+POLYGON[:1]):
                    signed_distance = ((bx-ax)*(y-ay)-(by-ay)*(x-ax))/math.hypot(bx-ax, by-ay)
                    self.assertGreaterEqual(signed_distance, .25-1e-9)
                dx, dy = max(xmin-x, 0, x-xmax), max(ymin-y, 0, y-ymax)
                self.assertGreaterEqual(math.hypot(dx, dy), .25-1e-9)

    def test_obstacle_blocks_direct_leg_two_to_three_and_proposal_goes_below(self):
        a, b = WAYPOINTS[1:3]
        center_x = (OBSTACLE[0]+OBSTACLE[2])/2
        t = (center_x-a[0])/(b[0]-a[0])
        y = a[1]+t*(b[1]-a[1])
        self.assertTrue(OBSTACLE[1] <= y <= OBSTACLE[3])
        near_obstacle = [p for p in self.candidate['legs'][2]['points']
                         if OBSTACLE[0] <= p[0] <= OBSTACLE[2]]
        self.assertTrue(near_obstacle)
        self.assertTrue(all(p[1] <= OBSTACLE[1]-.25 for p in near_obstacle))

    def test_twelve_cm_clearance_preserves_all_requested_waypoints(self):
        result, _ = generate(self.snapshot, radius=.10, margin=.02)
        self.assertEqual(result['waypoint_changes'], [])
        self.assertEqual(result['proposed_waypoints'], WAYPOINTS)
        self.assertAlmostEqual(result['clearance_m'], .12)
        self.assertEqual(len(result['legs']), 5)
        self.assertTrue(result['modified'])
        for leg, point in zip(result['legs'], WAYPOINTS):
            self.assertEqual(leg['points'][-1], point)


if __name__ == '__main__':
    unittest.main()
