"""Offline checks for units, observation validity and temporal alignment."""
import unittest

from cf2_build_map import build_map, ray_endpoint


def sample(yaw=0, stamp=100, height=0.3):
    return {'log': 'cf2_pose', 'timestamp_ms': stamp, 'data': {
        'stateEstimate.x': 0, 'stateEstimate.y': 0,
        'stateEstimate.z': height, 'stateEstimate.yaw': yaw}}


def ranges(front=1000, stamp=100):
    return {'log': 'cf2_ranges', 'timestamp_ms': stamp, 'data': {
        'range.front': front, 'range.back': 32766,
        'range.left': 32766, 'range.right': 32766}}


class MappingTests(unittest.TestCase):
    def test_yaw_degrees_and_body_axes(self):
        pose = sample(yaw=90)['data']
        x, y = ray_endpoint(pose, 'front', 1)
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 1)
        x, y = ray_endpoint(pose, 'right', 1)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 0)

    def test_millimetres_free_ray_and_occupied_endpoint(self):
        grid, width, _, stats = build_map([sample(), ranges()], resolution=0.1, size=4)
        self.assertEqual(grid[20 * width + 30], 100)
        self.assertEqual(grid[20 * width + 25], 0)
        self.assertEqual(grid[25 * width + 20], -1)
        self.assertEqual(stats['hits'], 1)

    def test_invalid_returns_leave_unknown_space(self):
        for value in (32766, 18, 0, float('nan')):
            grid, _, _, stats = build_map([sample(), ranges(front=value)])
            self.assertTrue(all(cell == -1 for cell in grid))
            self.assertEqual(stats['hits'], 0)

    def test_stale_pose_does_not_paint_map(self):
        grid, _, _, stats = build_map([sample(stamp=0), ranges(stamp=100)])
        self.assertEqual(stats['unpaired_samples'], 1)
        self.assertTrue(all(cell == -1 for cell in grid))

    def test_height_filter_excludes_ground_samples(self):
        _, _, _, stats = build_map([sample(stamp=0, height=0), ranges(stamp=0),
                                    sample(stamp=200), ranges(stamp=200)],
                                   min_height=0.2, max_height=0.4)
        self.assertEqual(stats['hits'], 1)
        self.assertEqual(stats['unpaired_samples'], 1)


if __name__ == '__main__':
    unittest.main()
