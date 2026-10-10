import math
import unittest
from cf2_navigation import Navigation
from cf2_coverage import CoverageMission
from test_cf2_navigation import feed

class ScanWorkflowTests(unittest.TestCase):
    def test_rotation_and_translation_are_simultaneous_and_mapping_continuous(self):
        nav=Navigation(coverage_area=(5,3),scan_yaw_rate=8)
        for i in range(20):
            now=1+i*.1
            feed(nav,now,yaw=i*.8,front=600)
            d=nav.tick(now+.01)
            self.assertNotEqual(d['state'],'SCAN')
            self.assertEqual(d['yaw_rate_deg_s'],8)
            self.assertEqual(d['workflow'],'MOVE')
            self.assertTrue(d['local_target'])
            self.assertLessEqual(math.hypot(*d['local_target'][:2]),.500001)
        self.assertGreater(math.hypot(*d['velocity_body'][:2]),.01)
        self.assertGreater(nav.mapper.snapshot(now)['stats']['mapped_samples'],10)

    def test_close_obstacle_stops_rotation_but_preserves_avoidance(self):
        nav=Navigation(coverage_area=(5,3),scan_yaw_rate=8)
        feed(nav,1,front=150)
        d=nav.tick(1.01)
        self.assertEqual(d['yaw_rate_deg_s'],0)
        self.assertLess(d['velocity_body'][0],0)

    def test_rotating_single_height_wall_survey_observes_and_returns(self):
        nav=Navigation(speed=.15,warning=.5,coverage_area=(5,3),survey_seconds=180,
                       scan_yaw_rate=12,mapping_radius=.8)
        x=y=yaw=0.; simultaneous=0
        for i in range(2400):
            now=1+i*.1; readings={}
            for name,offset in [('front',0),('left',math.pi/2),('back',math.pi),('right',-math.pi/2)]:
                dx,dy=math.cos(yaw+offset),math.sin(yaw+offset); distances=[]
                if abs(dx)>1e-8: distances.append(((2.5 if dx>0 else -2.5)-x)/dx)
                if abs(dy)>1e-8: distances.append(((1.5 if dy>0 else -1.5)-y)/dy)
                distance=min(distances)
                readings['range.'+name]=distance*1000 if distance<=4 else 32766
            feed(nav,now,x=x,y=y,yaw=math.degrees(yaw),readings=readings)
            d=nav.tick(now+.01)
            self.assertNotEqual(d['state'],'SCAN')
            if d['land']:
                self.assertEqual(d['reason'],'survey_route_complete')
                self.assertLess(math.hypot(x,y),.1)
                self.assertGreater(d['coverage']['observed_percent'],75)
                break
            vx,vy,_=d['velocity_body']
            simultaneous+=bool(d['yaw_rate_deg_s'] and math.hypot(vx,vy)>.03)
            x+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*.1
            y+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*.1
            yaw+=math.radians(d['yaw_rate_deg_s'])*.1
        else: self.fail('Single-height rotating survey did not finish and return')
        self.assertGreater(simultaneous,100)

    def test_single_pass_and_nearby_target_spacing(self):
        m=CoverageMission((1,2),(5,3),.3,spacing=.5,margin=.6)
        points=[m.origin,*m.targets]
        self.assertFalse(m.cross_pass)
        self.assertLess(len(m.targets),65)
        self.assertTrue(all(math.dist(a,b)<=.500001 for a,b in zip(points,points[1:])))

if __name__=='__main__': unittest.main()
