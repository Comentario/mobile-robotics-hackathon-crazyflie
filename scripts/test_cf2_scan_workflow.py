import math
import unittest
from cf2_navigation import Navigation
from cf2_coverage import CoverageMission
from test_cf2_navigation import feed

class ScanWorkflowTests(unittest.TestCase):
    def test_scan_freezes_goal_and_replanning_while_mapping_continues(self):
        nav=Navigation(coverage_area=(5,3),scan_yaw_rate=8)
        feed(nav,1,front=1000)
        first=nav.tick(1.01)
        self.assertEqual(first['state'],'SCAN')
        goal=nav.mission.current_waypoint
        targets=list(nav.mission.targets)
        for i in range(1,20):
            now=1+i*.1
            feed(nav,now,yaw=i*.8,front=1000)
            d=nav.tick(now+.01)
            self.assertEqual(d['state'],'SCAN')
            self.assertEqual(d['velocity_body'],(0.,0.,0.))
            self.assertEqual(nav.mission.current_waypoint,goal)
            self.assertEqual(nav.mission.targets,targets)
            self.assertEqual(nav.replans,0)
        self.assertGreater(nav.mapper.snapshot(now)['stats']['mapped_samples'],10)
        feed(nav,3,yaw=90,front=1000)
        d=nav.tick(3.01)
        self.assertNotEqual(d['state'],'SCAN')
        self.assertEqual(d['yaw_rate_deg_s'],0)
        self.assertTrue(d['local_target'])

    def test_close_obstacle_interrupts_scan_for_avoidance(self):
        nav=Navigation(coverage_area=(5,3),scan_yaw_rate=8)
        feed(nav,1)
        nav.tick(1.01)
        feed(nav,1.1,front=150)
        d=nav.tick(1.11)
        self.assertNotEqual(d['state'],'SCAN')
        self.assertEqual(d['yaw_rate_deg_s'],0)
        self.assertLess(d['velocity_body'][0],0)

    def test_target_spacing_includes_initial_approach(self):
        m=CoverageMission((1,2),(5,3),.3,spacing=.5,margin=.6)
        points=[m.origin,*m.targets]
        self.assertTrue(all(math.dist(a,b)<=.500001 for a,b in zip(points,points[1:])))

if __name__=='__main__': unittest.main()
