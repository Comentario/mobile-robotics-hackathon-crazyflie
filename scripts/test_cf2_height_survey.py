import math
import unittest
from cf2_navigation import Navigation

class HeightSurveyTests(unittest.TestCase):
    def test_rotating_two_level_survey_returns_and_changes_height_at_home(self):
        nav=Navigation(speed=.2,warning=.8,coverage_area=(2,2),survey_seconds=100,
                       survey_heights=(.3,.45),scan_yaw_rate=8)
        x=y=yaw=0.; z=.3
        changed=False; rotated=False
        for i in range(1600):
            now=1+i*.1
            readings={}
            for name,offset in [('front',0),('left',math.pi/2),('back',math.pi),('right',-math.pi/2)]:
                dx,dy=math.cos(yaw+offset),math.sin(yaw+offset)
                distances=[]
                if abs(dx)>1e-8: distances.append(((1 if dx>0 else -1)-x)/dx)
                if abs(dy)>1e-8: distances.append(((1 if dy>0 else -1)-y)/dy)
                readings['range.'+name]=min(distances)*1000
            readings.update({'range.up':2000,'range.zrange':z*1000})
            nav.update({'log':'cf2_pose','session_id':'test','timestamp_ms':int(now*1000),
                'hovering':True,'data':{'stateEstimate.x':x,'stateEstimate.y':y,
                'stateEstimate.z':z,'stateEstimate.yaw':math.degrees(yaw)}},now)
            nav.update({'log':'cf2_ranges','session_id':'test','timestamp_ms':int(now*1000)+2,
                        'hovering':True,'data':readings},now+.002)
            d=nav.tick(now+.01)
            if d['land']:
                self.assertIn(d['reason'],('survey_levels_returned','survey_levels_partial_returned'))
                self.assertEqual(len(nav.completed_layers),2)
                self.assertLess(math.hypot(x,y),.1)
                break
            vx,vy,vz=d['velocity_body']; rate=d['yaw_rate_deg_s']
            if vz:
                changed=True
                self.assertEqual((vx,vy),(0.,0.))
                self.assertEqual(rate,0)
                self.assertLess(math.hypot(x,y),.1)
                self.assertLessEqual(abs(vz),.08)
            rotated=rotated or rate>0
            x+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*.1
            y+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*.1
            z+=vz*.1; yaw+=math.radians(rate)*.1
        else: self.fail('Two-level survey did not return')
        self.assertTrue(changed)
        self.assertTrue(rotated)
        self.assertTrue(all(layer['stats']['mapped_samples']>0 for layer in nav.mapper.snapshot(now)['layers']))

if __name__=='__main__': unittest.main()
