import unittest
from cf2_live_map import LiveMap


def observe(mapper, stamp, y=0., front=500, clock=1.):
    common=dict(session_id='surface',timestamp_ms=stamp,hovering=True)
    mapper.update(dict(common,log='cf2_pose',data={'stateEstimate.x':0.,'stateEstimate.y':y,
        'stateEstimate.z':.3,'stateEstimate.yaw':0.}),now=clock)
    mapper.update(dict(common,log='cf2_ranges',data={'range.front':front}),now=clock)


class SurfaceTests(unittest.TestCase):
    def test_adjacent_hits_confirm_only_measured_cells(self):
        m=LiveMap(confirmation='spatial',retain_observations=True)
        for stamp,y in [(1000,.005),(1100,.025),(1200,.055)]:observe(m,stamp,y,clock=stamp/1000)
        s=m.snapshot(1.2)
        self.assertEqual(s['presentation_data'][m.cell(.5,.005)],100)
        self.assertEqual(s['presentation_data'][m.cell(.5,.055)],100)
        self.assertEqual(s['presentation_data'][m.cell(.5,.105)],-1)
        self.assertEqual(s['data'][m.cell(.5,.105)],-1)

    def test_support_uses_sensor_time_not_playback_speed(self):
        m=LiveMap(confirmation='spatial',retain_observations=True)
        for stamp,clock,y in [(1000,1,.005),(1100,1.01,.025),(1200,1.02,.055)]:
            observe(m,stamp,y,clock=clock)
        self.assertEqual(m.snapshot(1.02)['presentation_data'][m.cell(.5,.005)],100)

    def test_isolated_hit_does_not_confirm_and_free_evidence_can_clear(self):
        m=LiveMap(confirmation='spatial',retain_observations=True)
        observe(m,1000)
        self.assertEqual(m.snapshot(1)['presentation_data'][m.cell(.5,0)],-1)
        for i in range(4):observe(m,1100+i*100,front=1000,clock=1.1+i*.1)
        self.assertEqual(m.snapshot(1.5)['data'][m.cell(.5,0)],0)
        self.assertNotIn(m.cell(.5,0),m.hit_stamps)

    def test_confirmation_does_not_change_raw_planning_map(self):
        a,b=LiveMap(confirmation='cell'),LiveMap(confirmation='spatial')
        for stamp,y in [(1000,.005),(1100,.025),(1200,.055)]:
            for m in (a,b):observe(m,stamp,y,clock=stamp/1000)
        self.assertEqual(a.snapshot(1.2)['data'],b.snapshot(1.2)['data'])

if __name__=='__main__':unittest.main()
