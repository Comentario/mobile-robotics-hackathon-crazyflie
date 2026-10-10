import math
import unittest
from cf2_navigation import Navigation


def feed(nav, now, x=0, y=0, yaw=0, front=32766, right=32766, session='test', readings=None):
    nav.update({'log': 'cf2_pose', 'timestamp_ms': int(now*1000),
                'hovering': True, 'session_id': session,
                'data': {'stateEstimate.x': x, 'stateEstimate.y': y,
                         'stateEstimate.z': .3, 'stateEstimate.yaw': yaw}}, now)
    nav.update({'log': 'cf2_ranges', 'timestamp_ms': int(now*1000)+2,
                'hovering': True, 'session_id': session,
                'data': {'range.front': front, 'range.right': right,
                         'range.left': 32766, 'range.back': 32766,
                         'range.up': 1500, 'range.zrange': 300, **(readings or {})}}, now+.002)


class NavigationTests(unittest.TestCase):
    def test_front_warning_changes_real_adapter_velocity(self):
        nav = Navigation(timeout=10)
        feed(nav, 1, front=350)
        decision = nav.tick(1.01)
        self.assertLess(decision['velocity_body'][0], 0)
        self.assertFalse(decision['land'])

    def test_critical_obstacle_overrides_route_without_motor_cut(self):
        nav = Navigation([(.5, 0)])
        feed(nav, 1, front=159, right=84)
        decision = nav.tick(1.01)
        self.assertEqual(decision['velocity_body'], (0., 0., 0.))
        self.assertEqual(decision['state'], 'HOVER')
        self.assertFalse(decision['land'])

    def test_start_is_preserved_and_square_finishes_with_pose_feedback(self):
        nav = Navigation([(.3, 0), (.3, .3), (0, .3), (0, 0)], timeout=30)
        x, y, landed = 2., -1., False
        for i in range(1000):
            now = 1+i*.05
            feed(nav, now, x=x, y=y)
            decision = nav.tick(now+.01)
            if decision['land']:
                self.assertEqual(decision['reason'], 'mission_complete')
                landed = True
                break
            vx, vy, _ = decision['velocity_body']
            x, y = x+vx*.05, y+vy*.05
        self.assertTrue(landed)
        self.assertEqual(nav.origin, (2., -1.))
        self.assertLess(math.hypot(x-2, y+1), .08)
        self.assertGreater(nav.replans, 4)

    def test_stale_data_hovers_then_requests_controlled_landing(self):
        nav = Navigation()
        feed(nav, 1)
        nav.tick(1.01)
        self.assertEqual(nav.tick(1.6)['velocity_body'], (0., 0., 0.))
        self.assertTrue(nav.tick(3.7)['land'])

    def test_astar_routes_around_inflated_obstacle(self):
        nav = Navigation([(.9, 0)])
        feed(nav, 1, front=600)
        decision = nav.tick(1.01)
        self.assertTrue(decision['path'])
        self.assertTrue(any(abs(p[1]) >= .2 for p in decision['path']))
        self.assertGreater(decision['replans'], 0)

    def test_yaw_rotation_repels_in_body_direction(self):
        nav = Navigation(timeout=10)
        feed(nav, 1, yaw=90, front=350)
        decision = nav.tick(1.01)
        self.assertLess(decision['velocity_body'][0], 0)
        self.assertAlmostEqual(decision['velocity_body'][1], 0)

    def test_one_obstacle_inspection_and_return_in_closed_loop(self):
        nav = Navigation([(1.1, 0), (0, 0)], timeout=60)
        x = y = 0.
        minimum = math.inf
        # Circular obstacle at (0.55,0), radius 0.15 m. Four ideal range rays.
        for i in range(2400):
            now = 1+i*.05
            readings = {}
            for name, dx, dy in [('front',1,0), ('back',-1,0), ('left',0,1), ('right',0,-1)]:
                rx, ry = .55-x, -y
                along = rx*dx+ry*dy
                perpendicular = rx*dy-ry*dx
                raw = 32766
                if abs(perpendicular) <= .15:
                    distance = along-math.sqrt(.15**2-perpendicular**2)
                    if distance >= 0:
                        raw = distance*1000
                readings['range.'+name] = raw
            feed(nav, now, x=x, y=y, readings=readings)
            decision = nav.tick(now+.01)
            minimum = min(minimum, math.hypot(x-.55,y)-.15)
            if decision['land']:
                self.assertEqual(decision['reason'], 'mission_complete')
                break
            vx, vy, _ = decision['velocity_body']
            x, y = x+vx*.05, y+vy*.05
        else:
            self.fail('Obstacle inspection did not complete and return')
        self.assertGreater(minimum, .2)
        self.assertLess(math.hypot(x,y), .08)


if __name__ == '__main__':
    unittest.main()
