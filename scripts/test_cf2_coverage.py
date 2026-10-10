import math
import unittest
from cf2_navigation import Navigation
from cf2_coverage import CoverageMission
from test_cf2_navigation import feed
from mission_manager.mission import LANDING


class CoverageTests(unittest.TestCase):
    def test_blocked_target_revisited_and_failure_is_not_impossibility(self):
        mission = CoverageMission((0, 0), (5, 3), .3, margin=.9)
        mission.start(0)
        blocked = mission.active
        mission.skip(1, 'no_path_in_current_map')
        while mission.phase != 'REVISIT':
            g = mission.current_waypoint
            mission.step(2, (g.x, g.y, g.z), 0, 0, 0)
        self.assertEqual(mission.active, blocked)
        mission.skip(3, 'no_path_in_current_map')
        self.assertEqual(mission.phase, 'RETURN')
        self.assertEqual(len(mission.unresolved), 1)
        mission.step(4, (0, 0, .3), 0, 0, 0)
        self.assertEqual(mission.reason, 'survey_partial_returned')

    def test_budget_returns_instead_of_claiming_completion(self):
        mission = CoverageMission((1, 2), (5, 3), .3, budget=10)
        mission.start(0)
        mission.step(11, (1.2, 2, .3), 0, 0, 0)
        self.assertEqual(mission.phase, 'RETURN')
        self.assertTrue(mission.unresolved)
        mission.step(12, (1, 2, .3), 0, 0, 0)
        self.assertEqual(mission.state, LANDING)

    def test_five_by_three_survey_moves_replans_and_returns_with_wall_sensing(self):
        nav = Navigation(speed=.2, warning=.8, timeout=30,
                         coverage_area=(5, 3), survey_seconds=180)
        x = y = 0.
        max_distance = 0
        for i in range(2400):
            now = 1+i*.1
            readings = {'range.front': (2.5-x)*1000, 'range.back': (2.5+x)*1000,
                        'range.left': (1.5-y)*1000, 'range.right': (1.5+y)*1000}
            feed(nav, now, x=x, y=y, readings=readings)
            decision = nav.tick(now+.01)
            self.assertLessEqual(abs(x), 2.5)
            self.assertLessEqual(abs(y), 1.5)
            if decision['land']:
                self.assertEqual(decision['reason'], 'survey_route_complete')
                self.assertLess(math.hypot(x, y), .1)
                self.assertGreater(decision['coverage']['observed_percent'], 50)
                self.assertFalse(decision['coverage']['proof_of_impossibility'])
                break
            vx, vy, _ = decision['velocity_body']
            x, y = x+vx*.1, y+vy*.1
            max_distance = max(max_distance, math.hypot(x, y))
        else:
            self.fail('Survey failed to finish and return')
        self.assertGreater(max_distance, 1.5)
        self.assertGreater(nav.replans, 30)

    def test_new_obstacle_changes_route_within_large_survey_envelope(self):
        nav = Navigation(speed=.2, warning=.8, coverage_area=(5, 3))
        feed(nav, 1)
        nav.tick(1.01)
        from mission_manager.mission import Waypoint
        direct = nav._plan((0., 0., .3), Waypoint(1.8, 0, .3), 1.01)
        self.assertTrue(direct)
        feed(nav, 1.1, front=600)
        detour = nav._plan((0., 0., .3), Waypoint(1.8, 0, .3), 1.11)
        self.assertTrue(detour)
        self.assertTrue(any(abs(p[1]) >= .2 for p in detour))

    def test_repair_targets_unknown_near_observed_free_space(self):
        nav = Navigation(coverage_area=(5, 3), warning=.8)
        feed(nav, 1, front=1500, readings={'range.back': 1500})
        nav.tick(1.01)
        mission = nav.mission
        mission.pending, mission.deferred, mission.active = [], [], None
        mission.phase = 'RETURN'
        mission.prepare_repair((0, 0, .3), 2, nav.mapper.snapshot(2))
        self.assertTrue(mission.repair_used)
        self.assertEqual(mission.phase, 'REPAIR_SURVEY')
        goal = mission.current_waypoint
        self.assertLessEqual(abs(goal.x), 2.5-mission.margin)
        self.assertLessEqual(abs(goal.y), 1.5-mission.margin)

    def test_static_obstacle_survey_replans_preserves_clearance_and_returns(self):
        nav = Navigation(speed=.2, warning=.8, timeout=30,
                         coverage_area=(5, 3), survey_seconds=180)
        x = y = 0.
        minimum = math.inf
        for i in range(2400):
            now = 1+i*.1
            readings = {'range.front': (2.5-x)*1000, 'range.back': (2.5+x)*1000,
                        'range.left': (1.5-y)*1000, 'range.right': (1.5+y)*1000}
            # Ideal four-ray ToF returns against a fixed cylinder and arena walls.
            for name, dx, dy in [('front',1,0), ('back',-1,0), ('left',0,1), ('right',0,-1)]:
                rx, ry = .8-x, -.4-y
                along, perpendicular = rx*dx+ry*dy, rx*dy-ry*dx
                if abs(perpendicular) <= .2:
                    distance = along-math.sqrt(.2**2-perpendicular**2)
                    if distance >= 0:
                        readings['range.'+name] = min(readings['range.'+name], distance*1000)
            feed(nav, now, x=x, y=y, readings=readings)
            decision = nav.tick(now+.01)
            minimum = min(minimum, math.hypot(x-.8, y+.4)-.2)
            if decision['land']:
                self.assertIn(decision['reason'], ('survey_partial_returned', 'survey_route_complete'))
                self.assertLess(math.hypot(x, y), .1)
                self.assertGreater(decision['coverage']['observed_percent'], 50)
                break
            vx, vy, _ = decision['velocity_body']
            x, y = x+vx*.1, y+vy*.1
        else:
            self.fail('Obstacle survey did not finish and return')
        self.assertGreater(minimum, .2)
        self.assertGreater(nav.replans, 50)


if __name__ == '__main__':
    unittest.main()
