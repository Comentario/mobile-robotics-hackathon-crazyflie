"""Telemetry and metrics (TP-40). Recording is done with scripts/record_bag.sh.

Subscribes:
  /hk/state, /hk/mission_status, /hk/planner_status, /hk/safety_state,
  /hk/localization_health, /hk/sim/ground_truth and /hk/sim/obstacles (simulation only)
Publishes:
  /hk/metrics               std_msgs/String   JSON with the accumulated summary
Writes:
  <output_dir>/<date>_<scenario>.json when the mission ends
Parameters:
  output_dir, scenario_name, state_timeout

Collisions and obstacle clearance use the simulator ground truth, never the
estimated pose. Logic: metrics.py.
"""
from datetime import datetime
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from hackathon_interfaces.msg import LocalizationHealth, MissionStatus, PlannerStatus, SafetyState

from .metrics import RunMetrics

# simulation_adapter marker ids: 0-3 walls, 4 ceiling, 5 floor, >= 6 obstacles.
FLOOR_ID, FIRST_OBSTACLE_ID = 5, 6
FINAL_STATES = (MissionStatus.COMPLETED, MissionStatus.ABORTED)


def state_names(message_type):
    return {getattr(message_type, name): name for name in dir(message_type)
            if name.isupper() and isinstance(getattr(message_type, name), int)}


MISSION_NAMES = state_names(MissionStatus)
SAFETY_NAMES = state_names(SafetyState)
HEALTH_NAMES = state_names(LocalizationHealth)
PLANNER_NAMES = state_names(PlannerStatus)


class Telemetry(Node):

    def __init__(self):
        super().__init__('telemetry')
        self.output_dir = Path(str(self.declare_parameter('output_dir', '/tmp/hk_runs').value))
        self.scenario_name = str(self.declare_parameter('scenario_name', 'run').value)
        self.state_timeout = float(self.declare_parameter('state_timeout', 0.3).value)
        self.metrics = RunMetrics()
        self.obstacles, self.walls = [], []
        self.last_state = None
        self.mission_start = None
        self.last_safety = None
        self.written = False

        self.metrics_pub = self.create_publisher(String, '/hk/metrics', 10)
        self.create_subscription(Odometry, '/hk/state', self._state_callback, 10)
        self.create_subscription(MissionStatus, '/hk/mission_status', self._mission_callback, 10)
        self.create_subscription(PlannerStatus, '/hk/planner_status', self._planner_callback, 10)
        self.create_subscription(SafetyState, '/hk/safety_state', self._safety_callback, 10)
        self.create_subscription(LocalizationHealth, '/hk/localization_health', self._health_callback, 10)
        self.create_subscription(Odometry, '/hk/sim/ground_truth', self._truth_callback, 10)
        self.create_subscription(MarkerArray, '/hk/sim/obstacles', self._obstacles_callback, 10)
        self.create_timer(0.5, self._publish_metrics)
        self.get_logger().info(f'Telemetry ready for scenario "{self.scenario_name}" -> /hk/metrics')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _state_callback(self, msg):
        self._check_state_age()
        self.last_state = self._now()

    def _check_state_age(self):
        """Feed the age of the last pose to the metrics. Counting logic: metrics.py."""
        if self.last_state is not None:
            self.metrics.note_state_age(self._now() - self.last_state, self.state_timeout)

    def _mission_callback(self, msg):
        m = self.metrics
        m.final_mission_state = int(msg.state)
        m.waypoints_total = int(msg.waypoint_count)
        m.waypoints_reached = (m.waypoints_total if msg.state == MissionStatus.COMPLETED
                               else min(int(msg.waypoint_index), m.waypoints_total))
        if self.mission_start is None and msg.state in (MissionStatus.TAKEOFF, MissionStatus.NAVIGATE):
            self.mission_start = self._now()
        if self.mission_start is not None and not self.written:
            m.mission_time = self._now() - self.mission_start
        if msg.state in FINAL_STATES and not self.written:
            self._write_summary()

    def _planner_callback(self, msg):
        self.metrics.planner_states_seen.add(PLANNER_NAMES.get(msg.status, str(msg.status)))
        self.metrics.replans = max(self.metrics.replans, int(msg.replan_count))

    def _safety_callback(self, msg):
        self.metrics.safety_states_seen.add(SAFETY_NAMES.get(msg.state, str(msg.state)))
        if self.last_safety == SafetyState.NORMAL and msg.state != SafetyState.NORMAL:
            self.metrics.safety_interventions += 1
        self.last_safety = msg.state

    def _health_callback(self, msg):
        self.metrics.health_states_seen.add(HEALTH_NAMES.get(msg.status, str(msg.status)))

    def _obstacles_callback(self, msg):
        obstacles, walls = [], []
        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.id == FLOOR_ID:
                continue
            p, s = marker.pose.position, marker.scale
            box = ((p.x, p.y, p.z), (s.x, s.y, s.z))
            (obstacles if marker.id >= FIRST_OBSTACLE_ID else walls).append(box)
        self.obstacles, self.walls = obstacles, walls

    def _truth_callback(self, msg):
        p = msg.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.metrics.update_truth((p.x, p.y, p.z), self.obstacles, self.walls)

    def _summary(self):
        summary = self.metrics.to_dict()
        summary['scenario'] = self.scenario_name
        summary['final_mission_state_name'] = MISSION_NAMES.get(self.metrics.final_mission_state)
        return summary

    def _publish_metrics(self):
        self._check_state_age()
        msg = String()
        msg.data = json.dumps(self._summary())
        self.metrics_pub.publish(msg)

    def _write_summary(self):
        self.written = True
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / f'{datetime.now():%Y%m%d_%H%M%S}_{self.scenario_name}.json'
            path.write_text(json.dumps(self._summary(), indent=2))
            self.get_logger().info(f'Run summary written to {path}')
        except OSError as error:
            self.get_logger().error(f'Could not write the run summary: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = Telemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
