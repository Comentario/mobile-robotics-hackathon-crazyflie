import math
import unittest
from cf2_live_map import LiveMap
from cf2_layers import LayeredMap
from cf2_survey_sim import SurveySimulation, ranges, sweep

class SurveySimulationTests(unittest.TestCase):
    def test_short_free_rays_do_not_clear_gap_to_distant_obstacle(self):
        mapper=LiveMap(mapping_radius=1.2,free_radius=.3,retain_observations=True)
        common=dict(session_id='short_rays',timestamp_ms=1000,hovering=True)
        mapper.update(dict(common,log='cf2_pose',data={'stateEstimate.x':0.,'stateEstimate.y':0.,
            'stateEstimate.z':.3,'stateEstimate.yaw':0.}),now=1)
        mapper.update(dict(common,log='cf2_ranges',data={'range.front':1000,'range.back':1500,
            'range.left':1500,'range.right':32766}),now=1.002)
        s=mapper.snapshot(1.01)
        self.assertEqual(s['data'][mapper.cell(.15,0)],0)
        self.assertEqual(s['data'][mapper.cell(.7,0)],-1)
        self.assertEqual(s['data'][mapper.cell(1.,0)],100)
        self.assertEqual(s['data'][mapper.cell(0.,-.2)],-1)
        for i,v in enumerate(s['data']):
            if v==0:
                p=(s['origin'][0]+(i%s['width']+.5)*s['resolution'],
                   s['origin'][1]+(i//s['width']+.5)*s['resolution'])
                self.assertLessEqual(math.hypot(*p),.3+s['resolution'])

    def test_new_session_does_not_inherit_simulation_reveal_radius(self):
        mapper=LayeredMap()
        mapper.update({'session_id':'simulation','log':'cf2_navigation',
            'data':{'free_radius_m':.3,'mapping_radius_m':1.2}})
        self.assertEqual(mapper.free_radius,.3)
        mapper.update({'session_id':'physical','log':'cf2_navigation',
            'data':{'mapping_radius_m':.8}})
        self.assertIsNone(mapper.free_radius)
        self.assertIsNone(mapper.current.free_radius)

    def test_zigzag_starts_top_left_then_down(self):
        points=sweep((5,3),.35)
        self.assertEqual(points[0],(-2.15,1.15))
        self.assertEqual(points[1],(-2.15,-1.15))
        self.assertAlmostEqual(points[2][1],points[1][1])
        self.assertGreater(points[2][0],points[1][0])
        self.assertLessEqual(points[2][0]-points[1][0],.35)
        self.assertEqual(points[3][1],1.15)

    def test_static_obstacles_only_at_sensor_height(self):
        box=(.5,-.2,.7,.2,.2)
        low=ranges(0,0,0,(5,3),[box],.15)
        high=ranges(0,0,0,(5,3),[box],.3)
        self.assertEqual(low['range.front'],500)
        self.assertEqual(high['range.front'],2500)

    def test_simulation_never_initializes_map_with_world(self):
        sim=SurveySimulation(scenario='obstacles')
        self.assertIsNone(sim.mapper.snapshot(0)['origin'])
        rows,decision=sim.step()
        self.assertTrue(decision['simulation'])
        self.assertEqual(decision['source'],'IDEAL SIMULATION')
        self.assertEqual(decision['free_radius_m'],.3)
        self.assertEqual(rows[1]['data']['stateEstimate.yaw'],0)
        self.assertEqual(sim.yaw,0)

if __name__=='__main__':unittest.main()
