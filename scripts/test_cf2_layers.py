import unittest
from cf2_layers import LayeredMap, validate_heights
from test_cf2_live_map import pose, ranges

class LayerTests(unittest.TestCase):
    def sample(self, m, z, stamp, distance=500, phase=None):
        p = pose(stamp=stamp)
        p['data']['stateEstimate.z'] = z
        if phase: p['phase'] = phase
        r = ranges(stamp=stamp+2, front=distance)
        if phase: r['phase'] = phase
        m.update(p, now=stamp/1000)
        m.update(r, now=stamp/1000+.002)

    def test_layers_share_origin_without_fusing_height_obstacles(self):
        m = LayeredMap((.3, .45))
        for stamp in (1000,1200,1400,1700): self.sample(m,.3,stamp)
        for stamp in (2000,2200,2400,2700): self.sample(m,.45,stamp,1000)
        s = m.snapshot(10000)
        c = m.cell(.5,0)
        self.assertEqual(s['layers'][0]['presentation_data'][c],100)
        self.assertEqual(s['layers'][1]['data'][c],0)
        self.assertEqual(m.maps[.3].origin,m.maps[.45].origin)
        self.assertFalse(s['map_validated_for_drone1'])
        self.assertTrue(s['composite_is_visualization_only'])

    def test_transition_is_not_mapped_but_trail_is_retained(self):
        m = LayeredMap((.3,.45))
        self.sample(m,.3,1000,phase='height_change')
        self.assertEqual(m.snapshot(1)['layers'][0]['stats']['hits'],0)
        for i in range(2101):
            p=pose(stamp=2000+i)
            m.update(p,now=2+i*.001)
        self.assertEqual(len(m.snapshot(5)['trail']),2102)

    def test_display_clips_observations_without_hiding_control_obstacles(self):
        m=LayeredMap(mapping_radius=4.)
        self.sample(m,.3,1000,distance=1500)
        m.update({'log':'cf2_navigation','session_id':'one','data':
                  {'start':[0,0],'coverage':{'area':[2,2]}}},now=1.1)
        s=m.snapshot(2)
        cell=m.cell(1.5,0)
        self.assertEqual(s['data'][cell],-1)
        self.assertEqual(m.planning_snapshot(2)['data'][cell],100)
        for i,v in enumerate(s['data']):
            if v!=-1:
                ox,oy=s['origin']
                self.assertTrue(m.inside(ox+(i%m.width+.5)*m.resolution,
                                        oy+(i//m.width+.5)*m.resolution,m.resolution/2))

    def test_invalid_levels_are_rejected(self):
        for heights in ((.3,.31),(.45,.3),(.3,float('nan')), (.9,)):
            with self.assertRaises(ValueError): validate_heights(heights)

if __name__=='__main__': unittest.main()
