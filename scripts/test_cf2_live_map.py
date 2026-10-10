import unittest
from cf2_live_map import LiveMap


def pose(stamp=100, yaw=0, airborne=True, session='one'):
    return {'log': 'cf2_pose', 'timestamp_ms': stamp, 'hovering': airborne,
            'session_id': session, 'data': {'stateEstimate.x': 0,
            'stateEstimate.y': 0, 'stateEstimate.z': 0.3, 'stateEstimate.yaw': yaw}}


def ranges(stamp=102, front=1000, session='one'):
    return {'log': 'cf2_ranges', 'timestamp_ms': stamp, 'hovering': True,
            'session_id': session, 'data': {'range.front': front,
            'range.back': 32766, 'range.left': 32766, 'range.right': 32766}}


class LiveMappingTests(unittest.TestCase):
    def test_incremental_ray_and_invalid_returns(self):
        m = LiveMap()
        m.update(pose(), now=1)
        m.update(ranges(), now=1)
        s = m.snapshot(now=1)
        self.assertEqual(s['data'][m.cell(1, 0)], 100)
        self.assertEqual(s['data'][m.cell(0.5, 0)], 0)
        self.assertEqual(s['data'][m.cell(0, 0.5)], -1)
        self.assertEqual(s['stats']['hits'], 1)
        self.assertEqual(s['stats']['invalid_ranges'], 3)

    def test_yaw_and_stale_pose(self):
        m = LiveMap()
        m.update(pose(yaw=90), now=1)
        m.update(ranges(), now=1)
        self.assertEqual(m.snapshot(now=1)['data'][m.cell(0, 1)], 100)
        m.update(ranges(stamp=200), now=1)
        self.assertEqual(m.snapshot(now=1)['stats']['unpaired'], 1)

    def test_ground_and_new_flight_are_not_fused(self):
        m = LiveMap()
        m.update(pose(airborne=False), now=1)
        m.update(ranges(), now=1)
        self.assertIsNone(m.snapshot(now=1)['origin'])
        m.update(pose(), now=2)
        m.update(ranges(), now=2)
        m.update(pose(session='two'), now=3)
        self.assertEqual(m.snapshot(now=3)['stats']['hits'], 0)
        self.assertTrue(all(v == -1 for v in m.snapshot(now=3)['data']))

    def test_moving_object_can_clear_and_observations_expire(self):
        m = LiveMap(decay_seconds=20)
        m.update(pose(), now=1)
        m.update(ranges(front=500), now=1)
        self.assertEqual(m.snapshot(now=1)['data'][m.cell(.5, 0)], 100)
        for i in range(4):
            m.update(ranges(front=1000), now=2+i)
        self.assertEqual(m.snapshot(now=5)['data'][m.cell(.5, 0)], 0)
        self.assertTrue(all(v == -1 for v in m.snapshot(now=26)['data']))


if __name__ == '__main__':
    unittest.main()
