"""ROS transport check for the offline preview; run in an isolated ROS_DOMAIN_ID."""
import json
import subprocess
import sys
import time
import unittest

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'Source ROS before running the transport check')
class PolygonTransportTests(unittest.TestCase):
    def test_preview_topics_and_blocked_original_route(self):
        from nav_msgs.msg import Path, OccupancyGrid
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from generate_polygon_preview import generate, make_map
        from pathlib import Path as FilePath
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            result, _ = generate(make_map(), margin=.15)
            path = FilePath(directory)/'preview.json'
            path.write_text(json.dumps(result))
            script_directory = str(FilePath(__file__).resolve().parent)
            code = ('import sys,json;sys.path.insert(0,sys.argv[1]);'
                    'from plan_shared_map import publish;publish(json.load(open(sys.argv[2])))')
            process = subprocess.Popen([sys.executable, '-c', code, script_directory, str(path)],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            node = None
            try:
                rclpy.init()
                node = rclpy.create_node('test_polygon_route_topics')
                seen = {}
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                topics = [('/hk/path/initial', Path), ('/hk/path/modified', Path),
                          ('/hk/path/proposed', Path), ('/hk/shared_map/raw', OccupancyGrid),
                          ('/hk/shared_map/inflated', OccupancyGrid),
                          ('/hk/shared_map/preview_status', String)]
                subscriptions = [node.create_subscription(kind, topic,
                    lambda msg, t=topic: seen.__setitem__(t, msg), qos) for topic, kind in topics]
                deadline = time.monotonic()+15
                while len(seen) < len(topics) and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.2)
                    if process.poll() is not None:
                        self.fail(str(process.communicate()))
                self.assertEqual(len(seen), 6)
                self.assertEqual(len(seen['/hk/path/initial'].poses), 6)
                self.assertEqual(len(seen['/hk/path/modified'].poses), 0)
                self.assertEqual(len(seen['/hk/path/proposed'].poses), len(result['proposed']))
                self.assertEqual(seen['/hk/shared_map/raw'].header.frame_id, 'map')
                self.assertEqual(list(seen['/hk/shared_map/raw'].data), result['grid']['raw'])
                self.assertEqual(list(seen['/hk/shared_map/inflated'].data), result['grid']['inflated'])
                self.assertIn('ORIGINAL MISSION BLOCKED', seen['/hk/shared_map/preview_status'].data)
                published = {topic for topic, _ in
                             node.get_publisher_names_and_types_by_node('shared_map_route_preview', '/')}
                self.assertTrue({topic for topic, _ in topics}.issubset(published))
                self.assertFalse({'/hk/cmd/nav', '/hk/cmd/safe', '/cf231/cmd_full_state'} & published)
            finally:
                if node:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()


if __name__ == '__main__':
    unittest.main()
