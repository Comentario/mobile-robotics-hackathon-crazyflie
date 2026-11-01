"""Isolated-domain ROS check: fake state/mission, no adapter or radio."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

try:
    import rclpy
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'Source ROS and the project interface overlay')
class CooperativeNavigationTransportTests(unittest.TestCase):
    def test_current_leg_tracking_and_hover_for_unknown_goal(self):
        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root/'ros2_ws/src/local_navigator'))
        from local_navigator.local_navigator_node import LocalNavigator
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from hackathon_interfaces.msg import LocalizationHealth, MissionStatus, MultiRange, Setpoint
        from generate_polygon_preview import make_map, generate
        from rclpy.executors import SingleThreadedExecutor

        with tempfile.TemporaryDirectory() as directory:
            result, _ = generate(make_map())
            route = Path(directory)/'route.json'
            route.write_text(json.dumps(result))
            proc = subprocess.Popen([sys.executable, str(root/'scripts/cooperative_route.py'),
                                     '--route', str(route), '--allow-synthetic'],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            navigator = driver = executor = None
            try:
                rclpy.init(args=['--ros-args', '-p', 'use_planned_path:=true',
                                 '-p', 'z_nominal:=0.8', '-p', 'nominal_speed:=0.1',
                                 '-p', 'degraded_speed:=0.05', '-p', 'max_speed:=0.1'])
                navigator = LocalNavigator()
                driver = rclpy.create_node('fake_cooperative_test_inputs')
                executor = SingleThreadedExecutor()
                executor.add_node(navigator)
                executor.add_node(driver)
                commands = []
                driver.create_subscription(Setpoint, '/hk/cmd/nav', commands.append, 10)
                goal_pub = driver.create_publisher(PoseStamped, '/hk/goal', 10)
                state_pub = driver.create_publisher(Odometry, '/hk/state', 10)
                mission_pub = driver.create_publisher(MissionStatus, '/hk/mission_status', 10)
                ranges_pub = driver.create_publisher(MultiRange, '/hk/ranges', 10)
                health_pub = driver.create_publisher(
                    LocalizationHealth, '/hk/localization_health', 10)

                def feed(goal):
                    now = driver.get_clock().now().to_msg()
                    state = Odometry()
                    state.header.stamp, state.header.frame_id = now, 'map'
                    state.pose.pose.position.x = .03
                    state.pose.pose.position.y = -.09
                    state.pose.pose.position.z = .8
                    state.pose.pose.orientation.w = 1.
                    msg = PoseStamped()
                    msg.header.stamp, msg.header.frame_id = now, 'map'
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
                    mission = MissionStatus()
                    mission.state = MissionStatus.NAVIGATE
                    # The navigator needs fresh ranges; obstacles are out of range.
                    ranges = MultiRange()
                    ranges.header.stamp, ranges.header.frame_id = now, 'base_link'
                    for name in ('front', 'back', 'left', 'right', 'up'):
                        setattr(ranges, name, float('inf'))
                    ranges.down = .8
                    ranges_pub.publish(ranges)
                    # TP-16: the navigator only moves with good localization.
                    health = LocalizationHealth()
                    health.header.stamp = now
                    health.status = LocalizationHealth.GOOD
                    health_pub.publish(health)
                    state_pub.publish(state)
                    goal_pub.publish(msg)
                    mission_pub.publish(mission)

                def until(condition, goal):
                    deadline = time.monotonic()+8
                    while time.monotonic() < deadline:
                        feed(goal)
                        executor.spin_once(timeout_sec=.02)
                        if condition():
                            return
                        if proc.poll() is not None:
                            self.fail(str(proc.communicate()))
                    self.fail('ROS condition timeout')

                target = tuple(result['legs'][0]['points'][-1])
                until(lambda: navigator.path_stamp is not None and commands and
                      commands[-1].mode == Setpoint.MODE_VELOCITY, target)
                self.assertEqual(navigator.route.points[-1], target)
                self.assertGreater(len(navigator.route.points), 10)
                cmd = commands[-1]
                self.assertLessEqual((cmd.velocity.x**2+cmd.velocity.y**2)**.5, .10001)
                unknown = (0.5, 0.5, .8)
                commands.clear()
                until(lambda: navigator.goal == unknown and not navigator.route.points and
                      commands and commands[-1].mode == Setpoint.MODE_HOVER, unknown)
                self.assertEqual(commands[-1].velocity.x, 0.)
                self.assertEqual(commands[-1].velocity.y, 0.)
                # Independently exercise expiry and mismatched endpoint rejection.
                navigator.goal = target
                navigator.goal_stamp = navigator._now()
                navigator.route.set_path(result['legs'][0]['points'])
                navigator.path_stamp = navigator._now()-2
                navigator.state_stamp = navigator._now()
                navigator._control_tick()
                executor.spin_once(timeout_sec=.1)
                self.assertTrue(navigator.use_planned_path)
            finally:
                if executor:
                    executor.shutdown()
                for node in (navigator, driver):
                    if node:
                        node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                proc.terminate()
                try:
                    proc.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.communicate()


if __name__ == '__main__':
    unittest.main()
