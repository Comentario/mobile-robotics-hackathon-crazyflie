"""Scenario evaluator for the T01-T16 test matrix (TP-18, TP-25, TP-31, TP-41).

Launched with `simulation.launch.py evaluate:=true`.

Subscribes:
  /hk/metrics               std_msgs/String   (from telemetry)
  /hk/mission_status        hackathon_interfaces/MissionStatus
When the run ends (mission COMPLETED/ABORTED, or the scenario timeout):
  compares the metrics with the scenario's `expected`, prints PASS/FAIL and appends it to
  <output_dir>/results.csv. Exits with code 0 (pass) or 1 (fail).
Parameters:
  scenario_file, scenario_name, output_dir, settle_time

Logic: scenario.py.
"""
import csv
from datetime import datetime
import json
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hackathon_interfaces.msg import MissionStatus

from .scenario import evaluate, load_scenario

FINAL_STATES = (MissionStatus.COMPLETED, MissionStatus.ABORTED)


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('scenario_runner')
        self.scenario_file = str(self.declare_parameter('scenario_file', '').value)
        self.output_dir = Path(str(self.declare_parameter('output_dir', '/tmp/hk_runs').value))
        # Time to keep collecting metrics after the mission ends (e.g. final_z after landing).
        self.settle_time = float(self.declare_parameter('settle_time', 6.0).value)
        self.scenario = load_scenario(self.scenario_file)
        self.name = str(self.declare_parameter('scenario_name', Path(self.scenario_file).stem).value)
        self.metrics = {}
        self.ended_at = None
        self.passed = None
        self.start = self._now()
        self.create_subscription(String, '/hk/metrics', self._metrics_callback, 10)
        self.create_subscription(MissionStatus, '/hk/mission_status', self._mission_callback, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f'Evaluating {self.name} (timeout {self.scenario["timeout"]:.0f} s)')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _metrics_callback(self, msg):
        try:
            self.metrics = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Ignoring malformed /hk/metrics message')

    def _mission_callback(self, msg):
        if msg.state in FINAL_STATES and self.ended_at is None:
            self.ended_at = self._now()

    def _tick(self):
        if self.passed is not None:
            return
        now = self._now()
        timed_out = now - self.start > self.scenario['timeout']
        settled = self.ended_at is not None and now - self.ended_at >= self.settle_time
        if not (timed_out or settled):
            return
        passed, failures = evaluate(self.metrics, self.scenario['expected'])
        self.passed = passed
        verdict = 'PASS' if passed else 'FAIL'
        detail = 'timeout' if timed_out and self.ended_at is None else 'mission ended'
        message = f'{verdict} {self.name} ({detail})' + ('' if passed else ': ' + '; '.join(failures))
        (self.get_logger().info if passed else self.get_logger().error)(message)
        self._append_result(verdict, detail, failures)
        raise SystemExit(0 if passed else 1)

    def _append_result(self, verdict, detail, failures):
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / 'results.csv'
            new = not path.exists()
            with path.open('a', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                if new:
                    writer.writerow(['time', 'scenario', 'result', 'ended_by', 'failures',
                                     'final_state', 'collisions', 'min_obstacle_distance',
                                     'mission_time'])
                m = self.metrics
                writer.writerow([f'{datetime.now():%Y-%m-%d %H:%M:%S}', self.name, verdict, detail,
                                 ' | '.join(failures), m.get('final_mission_state_name'),
                                 m.get('collisions'), m.get('min_obstacle_distance'),
                                 m.get('mission_time')])
        except OSError as error:
            self.get_logger().error(f'Could not write results.csv: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as exit_request:
        code = int(exit_request.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
