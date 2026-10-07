"""Lightweight Crazyflie simulator used during development.

The node exposes the same project-facing ROS API as cf_adapter, allowing the
navigation and safety stack to run without Crazyflie hardware or Crazyswarm2.
"""

from collections import deque
import math
import os
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
import yaml

from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from hackathon_interfaces.msg import MultiRange, Setpoint
from hackathon_interfaces.srv import Land, Takeoff

from .dynamics import KinematicDrone
from .faults import FaultInjector
from .world import World


class SimulationAdapter(Node):
    """Expose the hardware abstraction API using a lightweight simulator."""

    def __init__(self):
        super().__init__('simulation_adapter')

        self._declare_parameters()
        self._read_parameters()
        self.scenario = self._load_scenario(self.scenario_file)
        self.world = self._load_world(self.world_file)
        self.fault_injector = FaultInjector(
          self.scenario.get('faults', [])
        )

        self.drone = KinematicDrone(
            max_speed=self.max_speed,
            max_accel=self.max_accel,
            command_watchdog=self.command_watchdog,
        )

        self._apply_initial_state()

        self.rng = np.random.default_rng(self.random_seed)

        self.pose_history = deque()

        # Simulation integration must never depend on ROS/system clock jumps.
        self.start_monotonic = time.monotonic()
        self.last_sim_monotonic = self.start_monotonic

        # ROS timers are also scheduled using a steady clock.
        self.steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        self._create_ros_interfaces()
        self._create_timers()

        self.get_logger().info(
            f'Simulation adapter ready with world "{self.world_file}"'
        )

        self.get_logger().info(
            'Project API: /hk/cmd/safe, /hk/takeoff, /hk/land, '
            '/hk/emergency, /hk/adapter/odom, /hk/adapter/ranges'
        )

    def _declare_parameters(self):
        """Declare all simulator parameters."""
        self.declare_parameter('world_file', '')
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('scenario_name', '')

        self.declare_parameter('sim_rate', 100.0)
        self.declare_parameter('state_rate', 50.0)
        self.declare_parameter('range_rate', 20.0)

        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_accel', 2.0)

        self.declare_parameter('range_noise_std', 0.01)
        self.declare_parameter('pose_noise_std', 0.005)

        self.declare_parameter('command_watchdog', 0.5)

        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 4.0)

        self.declare_parameter('drone_radius', 0.08)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('random_seed', 42)

    def _read_parameters(self):
        """Read simulator parameters."""
        self.world_file = self.get_parameter(
            'world_file'
        ).value

        self.scenario_file = self.get_parameter(
            'scenario_file'
        ).value

        self.sim_rate = float(
            self.get_parameter('sim_rate').value
        )

        self.state_rate = float(
            self.get_parameter('state_rate').value
        )

        self.range_rate = float(
            self.get_parameter('range_rate').value
        )

        self.max_speed = float(
            self.get_parameter('max_speed').value
        )

        self.max_accel = float(
            self.get_parameter('max_accel').value
        )

        self.range_noise_std = float(
            self.get_parameter('range_noise_std').value
        )

        self.pose_noise_std = float(
            self.get_parameter('pose_noise_std').value
        )

        self.command_watchdog = float(
            self.get_parameter('command_watchdog').value
        )

        self.range_min = float(
            self.get_parameter('range_min').value
        )

        self.range_max = float(
            self.get_parameter('range_max').value
        )

        self.drone_radius = float(
            self.get_parameter('drone_radius').value
        )

        self.map_frame = str(
            self.get_parameter('map_frame').value
        )

        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.random_seed = int(
            self.get_parameter('random_seed').value
        )

    def _create_ros_interfaces(self):
        """Create publishers, subscription, and services."""
        self.odom_pub = self.create_publisher(
            Odometry,
            '/hk/adapter/odom',
            10,
        )

        self.ranges_pub = self.create_publisher(
            MultiRange,
            '/hk/adapter/ranges',
            10,
        )

        self.ground_truth_pub = self.create_publisher(
            Odometry,
            '/hk/sim/ground_truth',
            10,
        )

        self.obstacles_pub = self.create_publisher(
            MarkerArray,
            '/hk/sim/obstacles',
            10,
        )

        self.command_sub = self.create_subscription(
            Setpoint,
            '/hk/cmd/safe',
            self._command_callback,
            10,
        )

        self.takeoff_srv = self.create_service(
            Takeoff,
            '/hk/takeoff',
            self._takeoff_callback,
        )

        self.land_srv = self.create_service(
            Land,
            '/hk/land',
            self._land_callback,
        )

        self.emergency_srv = self.create_service(
            Trigger,
            '/hk/emergency',
            self._emergency_callback,
        )

    def _create_timers(self):
        """Create timers using a steady clock."""
        self.sim_timer = self.create_timer(
            1.0 / self.sim_rate,
            self._simulation_tick,
            clock=self.steady_clock,
        )

        self.state_timer = self.create_timer(
            1.0 / self.state_rate,
            self._publish_state,
            clock=self.steady_clock,
        )

        self.range_timer = self.create_timer(
            1.0 / self.range_rate,
            self._publish_ranges,
            clock=self.steady_clock,
        )

    def _simulation_time(self):
        """Return elapsed monotonic simulation time."""
        return time.monotonic() - self.start_monotonic

    def _load_world(self, path):
        """Load world configuration."""
        if not path:
            self.get_logger().warn(
                'No world_file provided; using default empty room'
            )

            return World.from_dict({
                'bounds': [-2.0, -2.0, 2.0, 2.0],
                'ceiling': 2.0,
                'obstacles': [],
            })

        if not os.path.exists(path):
            raise FileNotFoundError(
                f'World file does not exist: {path}'
            )

        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}

        return World.from_dict(data)

    def _load_scenario(self, path):
        """Load the scenario YAML."""
        if not path:
            return {}

        if not os.path.exists(path):
            raise FileNotFoundError(
                f'Scenario file does not exist: {path}'
            )

        with open(path, encoding='utf-8') as stream:
            return yaml.safe_load(stream) or {}

    def _apply_initial_state(self):
        """Apply the initial pose defined by the scenario."""
        start = self.scenario.get('start')

        if start is None:
            return

        if not isinstance(start, (list, tuple)) or len(start) != 4:
            raise ValueError(
                'Scenario start must be [x, y, z, yaw]'
            )

        x, y, z, yaw = [float(value) for value in start]

        self.drone.state.x = x
        self.drone.state.y = y
        self.drone.state.z = z
        self.drone.state.yaw = yaw

        self.drone.state.vx = 0.0
        self.drone.state.vy = 0.0
        self.drone.state.vz = 0.0

        self.drone.state.flying = z > 0.0

        self.get_logger().info(
            'Initial simulated pose: '
            f'x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}'
        )

    def _simulation_tick(self):
        """Advance the kinematic simulation."""
        now = time.monotonic()

        dt = now - self.last_sim_monotonic
        self.last_sim_monotonic = now

        # Avoid large integration jumps after debugger pauses or system stalls.
        dt = min(max(dt, 0.0), 0.1)

        t = self._simulation_time()

        previous = (
            self.drone.state.x,
            self.drone.state.y,
            self.drone.state.z,
        )

        self.drone.step(
            t,
            dt,
        )

        current = (
            self.drone.state.x,
            self.drone.state.y,
            self.drone.state.z,
        )

        if (
            self.drone.state.flying
            and self.world.collides(
                current,
                self.drone_radius,
                t,
            )
        ):
            self.get_logger().error(
                'Simulated collision detected; stopping vehicle'
            )

            self.drone.state.x = previous[0]
            self.drone.state.y = previous[1]
            self.drone.state.z = previous[2]

            self.drone.state.vx = 0.0
            self.drone.state.vy = 0.0
            self.drone.state.vz = 0.0

        pose = self._state_as_pose_dict()

        self.pose_history.append(
            (t, pose)
        )

        while (
            self.pose_history
            and t - self.pose_history[0][0] > 10.0
        ):
            self.pose_history.popleft()

    def _command_callback(self, message):
        """Receive the safety-approved command."""
        t = self._simulation_time()

        if self.fault_injector.drop_command(t):
            return

        self.drone.set_command(
            t,
            message,
        )

    def _takeoff_callback(self, request, response):
        """Start simulated takeoff."""
        try:
            accepted = self.drone.takeoff(
                request.height,
                request.duration,
            )

            response.success = bool(accepted)

            if accepted:
                response.message = 'Simulated takeoff accepted'
            else:
                response.message = 'Simulated takeoff rejected'

        except ValueError as exc:
            response.success = False
            response.message = str(exc)

        return response

    def _land_callback(self, request, response):
        """Start simulated landing."""
        try:
            accepted = self.drone.land(
                request.duration,
            )

            response.success = bool(accepted)

            if accepted:
                response.message = 'Simulated landing accepted'
            else:
                response.message = 'Simulated landing rejected'

        except ValueError as exc:
            response.success = False
            response.message = str(exc)

        return response

    def _emergency_callback(self, request, response):
        """Activate simulated emergency stop."""
        del request

        self.drone.emergency_stop()

        response.success = True
        response.message = 'Simulated emergency stop activated'

        return response

    def _state_as_pose_dict(self):
        """Convert simulator state to fault-injector pose representation."""
        state = self.drone.state

        return {
            'x': state.x,
            'y': state.y,
            'z': state.z,
            'yaw': state.yaw,
        }

    @staticmethod
    def _set_yaw(quaternion, yaw):
        """Create a quaternion containing yaw rotation."""
        half_yaw = 0.5 * yaw

        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(half_yaw)
        quaternion.w = math.cos(half_yaw)

    def _make_odometry(self, pose, noisy):
        """Create an Odometry message."""
        message = Odometry()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.base_frame

        x = pose['x']
        y = pose['y']
        z = pose['z']

        if noisy and self.pose_noise_std > 0.0:
            x += self.rng.normal(
                0.0,
                self.pose_noise_std,
            )

            y += self.rng.normal(
                0.0,
                self.pose_noise_std,
            )

            z += self.rng.normal(
                0.0,
                self.pose_noise_std,
            )

        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.position.z = float(z)

        self._set_yaw(
            message.pose.pose.orientation,
            pose['yaw'],
        )

        message.twist.twist.linear.x = self.drone.state.vx
        message.twist.twist.linear.y = self.drone.state.vy
        message.twist.twist.linear.z = self.drone.state.vz

        return message

    def _publish_state(self):
        """Publish ground truth and measured odometry."""
        exact_pose = self._state_as_pose_dict()

        ground_truth = self._make_odometry(
            exact_pose,
            noisy=False,
        )

        self.ground_truth_pub.publish(
            ground_truth
        )

        t = self._simulation_time()

        measured_pose = self.fault_injector.apply_to_pose(
            t,
            exact_pose,
            list(self.pose_history),
        )

        if measured_pose is None:
            return

        measured = self._make_odometry(
            measured_pose,
            noisy=True,
        )

        self.odom_pub.publish(
            measured
        )

    def _sensor_directions(self):
        """Return ranger directions expressed in the map frame."""
        yaw = self.drone.state.yaw

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        return {
            'front': (
                cos_yaw,
                sin_yaw,
                0.0,
            ),
            'back': (
                -cos_yaw,
                -sin_yaw,
                0.0,
            ),
            'left': (
                -sin_yaw,
                cos_yaw,
                0.0,
            ),
            'right': (
                sin_yaw,
                -cos_yaw,
                0.0,
            ),
            'up': (
                0.0,
                0.0,
                1.0,
            ),
            'down': (
                0.0,
                0.0,
                -1.0,
            ),
        }

    def _simulate_ranges(self):
        """Raycast all simulated range sensors."""
        state = self.drone.state

        origin = (
            state.x,
            state.y,
            state.z,
        )

        t = self._simulation_time()

        result = {}

        for name, direction in self._sensor_directions().items():
            distance = self.world.raycast(
                origin,
                direction,
                self.range_max,
                t,
            )

            if math.isfinite(distance):
                if self.range_noise_std > 0.0:
                    distance += self.rng.normal(
                        0.0,
                        self.range_noise_std,
                    )

                if distance < self.range_min:
                    distance = -math.inf
                elif distance > self.range_max:
                    distance = math.inf

            result[name] = float(distance)

        return self.fault_injector.apply_to_ranges(
            t,
            result,
        )

    def _publish_ranges(self):
        """Publish synthetic MultiRange measurements."""
        ranges = self._simulate_ranges()

        message = MultiRange()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame

        message.front = ranges['front']
        message.back = ranges['back']
        message.left = ranges['left']
        message.right = ranges['right']
        message.up = ranges['up']
        message.down = ranges['down']

        message.range_min = self.range_min
        message.range_max = self.range_max

        self.ranges_pub.publish(
            message
        )

        self._publish_obstacles()

    def _publish_obstacles(self):
        """Publish active obstacles as visualization markers."""
        marker_array = MarkerArray()

        clear = Marker()

        clear.header.frame_id = self.map_frame
        clear.header.stamp = self.get_clock().now().to_msg()

        clear.action = Marker.DELETEALL

        marker_array.markers.append(clear)

        t = self._simulation_time()

        marker_id = 0

        for obstacle in self.world.obstacles:
            if not obstacle.active(t):
                continue

            center_x, center_y = obstacle.center_at(t)

            height = min(
                obstacle.height,
                self.world.ceiling,
            )

            marker = Marker()

            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'sim_obstacles'
            marker.id = marker_id

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = height / 2.0

            marker.pose.orientation.w = 1.0

            marker.scale.x = obstacle.size[0]
            marker.scale.y = obstacle.size[1]
            marker.scale.z = height

            marker.color.r = 0.7
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.8

            marker_array.markers.append(
                marker
            )

            marker_id += 1

        self.obstacles_pub.publish(
            marker_array
        )


def main(args=None):
    rclpy.init(args=args)

    node = SimulationAdapter()

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