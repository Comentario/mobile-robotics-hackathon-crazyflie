import math
import unittest
from cf2_navigation import Navigation
from cf2_exploration import ExplorationMission
from test_cf2_navigation import feed
from mission_manager.mission import LANDING
from cf2_live_map import LiveMap


def grid(free=False):
    return dict(origin=(-1.5,-1.5), width=30, height=30, resolution=.1,
                data=[0 if free else -1]*900, recent_observations=[])


class ExplorationTests(unittest.TestCase):
    def test_target_returns_with_explicit_reason(self):
        mission=ExplorationMission((0,0),(3,3),.3)
        mission.start(0)
        mission.refresh(1,(0,0,.3),grid(True))
        self.assertEqual(mission.end_reason,'coverage_target_reached')
        mission.step(2,(0,0,.3),0,0,0)
        self.assertEqual(mission.state,LANDING)
        self.assertFalse(mission.summary(grid(True))['inspection_complete'])

    def test_budget_and_stagnation_are_distinct(self):
        mission=ExplorationMission((0,0),(3,3),.3,budget=10)
        mission.start(0);mission.refresh(1,(0,0,.3),grid())
        mission.step(11,(.2,0,.3),0,0,0)
        self.assertEqual(mission.end_reason,'flight_budget_reached')
        unlimited=ExplorationMission((0,0),(3,3),.3,budget=None,no_progress_seconds=2000)
        unlimited.start(0);unlimited.refresh(1000,(0,0,.3),grid())
        unlimited.step(1000,(0,0,.3),0,0,0)
        self.assertNotEqual(unlimited.phase,'RETURN')
        unlimited.refresh(2001,(0,0,.3),grid())
        self.assertEqual(unlimited.end_reason,'no_information_progress')
        self.assertFalse(unlimited.summary(grid())['proof_of_impossibility'])

    def test_small_measured_gain_resets_progress_timer(self):
        mission=ExplorationMission((0,0),(3,3),.3)
        mission.start(0)
        snapshot=grid()
        mission.refresh(1,(0,0,.3),snapshot)
        snapshot['data'][15*30+15]=0
        mission.refresh(44,(0,0,.3),snapshot)
        mission.refresh(50,(0,0,.3),snapshot)
        self.assertNotEqual(mission.phase,'RETURN')
        self.assertEqual(mission.progress_at,44)

    def test_inspection_views_use_observer_at_measurement(self):
        snapshot=grid(True);snapshot['data'][15*30+15]=100
        snapshot['recent_observations']=[dict(endpoint=(.05,.05),observer=(.6,.05))]
        mission=ExplorationMission((0,0),(3,3),.3)
        mission.start(0);mission.refresh(1,(-.6,.05,.3),snapshot)
        self.assertEqual(mission.objects[0]['view_bins'],[0])
        self.assertEqual(mission.phase,'INSPECT')
        target=mission.current_waypoint
        i=math.floor((target.y+1.5)/.1)*30+math.floor((target.x+1.5)/.1)
        self.assertEqual(snapshot['data'][i],0)
        mission.refresh(2,(0,-.6,.3),snapshot)
        self.assertEqual(mission.objects[0]['view_bins'],[0])

    def test_short_unknown_probe_and_continuous_rotation(self):
        nav=Navigation(coverage_area=(5,3),coverage_mode='explore',scan_yaw_rate=12)
        for step in range(12):
            t=1+step*.05;feed(nav,t)
            decision=nav.tick(t+.01)
        self.assertIsInstance(nav.mission,ExplorationMission)
        target=nav.mission.current_waypoint
        self.assertLessEqual(math.hypot(target.x,target.y),.46)
        self.assertEqual(decision['yaw_rate_deg_s'],12)
        self.assertGreater(math.hypot(*decision['velocity_body'][:2]),0)

    def test_range_observation_keeps_measured_observer(self):
        mapper=LiveMap(mapping_radius=.8)
        mapper.update({'log':'cf2_pose','session_id':'observer','timestamp_ms':1000,
            'hovering':True,'data':{'stateEstimate.x':.2,'stateEstimate.y':.1,
                'stateEstimate.z':.3,'stateEstimate.yaw':0}}, now=1)
        mapper.update({'log':'cf2_ranges','session_id':'observer','timestamp_ms':1002,
            'hovering':True,'data':{'range.front':500}}, now=1.002)
        observation=mapper.snapshot(1.1)['recent_observations'][0]
        self.assertEqual(observation['observer'],(.2,.1))
        self.assertAlmostEqual(observation['endpoint'][0],.7)
        self.assertEqual(mapper.snapshot(3)['recent_observations'],[])

    def test_quantized_margin_can_escape_only_away_from_obstacle(self):
        from mission_manager.mission import Waypoint
        nav=Navigation(coverage_area=(3,3),coverage_mode='explore')
        feed(nav,1);nav.tick(1.01)
        layer=nav.mapper.current
        cell=layer.cell(.25,0)
        layer.scores[cell]=6;layer.updated[cell]=1
        nav.mission.phase='RETURN'
        path=nav._plan((0,0,.3),Waypoint(-.5,0,.3),1.1)
        self.assertTrue(path)
        self.assertLess(path[1][0],0)
        occupied_xy=(layer.origin[0]+(cell%layer.width+.5)*layer.resolution,
                     layer.origin[1]+(cell//layer.width+.5)*layer.resolution)
        self.assertGreater(math.dist(path[1][:2],occupied_xy),math.dist(path[0][:2],occupied_xy))
        blocked=nav._plan((*occupied_xy,.3),Waypoint(-.5,0,.3),1.2)
        self.assertEqual(blocked,[])

    def test_rejects_multiple_heights(self):
        with self.assertRaises(ValueError):
            Navigation(coverage_area=(5,3),coverage_mode='explore',survey_heights=(.3,.45))

if __name__=='__main__':unittest.main()
