"""Local navigation node for TP-32 and TP-34.

Inputs:
  /hk/state                 nav_msgs/Odometry
  /hk/goal                  geometry_msgs/PoseStamped
  /hk/ranges                hackathon_interfaces/MultiRange

Output:
  /hk/cmd/nav               hackathon_interfaces/Setpoint

TP-32 provides the bounded direct-goal position follower.
TP-34 adds reactive horizontal obstacle avoidance.

Only safety_supervisor may publish /hk/cmd/safe.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from hackathon_interfaces.msg import MultiRange, Setpoint, MissionStatus

from .avoidance import AvoidanceConfig, reactive_correction
from .follower import direct_position_setpoint
from .route_follower import RouteFollower


class LocalNavigator(Node):
    """Generate bounded navigation proposals with horizontal avoidance."""

    def __init__(self):
        super().__init__('local_navigator')

        self._declare_parameters()
        self._read_parameters()

        self.position = None
        self.current_yaw = 0.0
        self.goal = None
        self.goal_stamp = None
        self.ranges = None
        self.route = RouteFollower(self.route_point_tolerance)
        self.path_stamp = None
        self.create_subscription(Path, '/hk/path/processed', self._path_callback, 10)

        self.command_pub = self.create_publisher(
            Setpoint,
            '/hk/cmd/nav',
            10,
        )

        self.create_subscription(
            Odometry,
            '/hk/state',
            self._state_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            '/hk/goal',
            self._goal_callback,
            10,
        )

        self.create_subscription(
            MultiRange,
            '/hk/ranges',
            self._ranges_callback,
            10,
        )

        self.state_stamp = None
        self.range_stamp = None
        self.mission_active = False
        self.create_subscription(MissionStatus, '/hk/mission_status', self._mission_callback, 10)
        self.create_timer(1.0 / self.control_rate, self._control_tick)

        self.get_logger().info(
            'TP-32/34 local navigator ready: '
            '/hk/state + /hk/goal + /hk/ranges -> /hk/cmd/nav'
        )

    def _declare_parameters(self):
        """Declare local-navigation parameters."""
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('nominal_speed', 0.3)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('z_nominal', 0.5)
        self.declare_parameter('goal_timeout', 0.5)
        self.use_planned_path = self.declare_parameter('use_planned_path', False).value
        self.path_timeout = self.declare_parameter('path_timeout', 0.5).value
        self.route_point_tolerance = self.declare_parameter('route_point_tolerance', 0.03).value
        self.path_start_tolerance = self.declare_parameter('path_start_tolerance', 0.15).value
        self.state_timeout = self.declare_parameter('state_timeout', 0.3).value
        self.range_timeout = self.declare_parameter('range_timeout', 0.3).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.slowdown_radius = self.declare_parameter('goal_slowdown_radius', 0.5).value
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('warning_distance', 0.5)
        self.declare_parameter('critical_distance', 0.2)
        self.declare_parameter('avoidance_speed', 0.2)

    def _read_parameters(self):
        """Read and validate local-navigation parameters."""
        self.control_rate = float(
            self.get_parameter('control_rate').value
        )

        self.nominal_speed = float(
            self.get_parameter('nominal_speed').value
        )

        self.max_speed = float(
            self.get_parameter('max_speed').value
        )

        self.z_nominal = float(
            self.get_parameter('z_nominal').value
        )

        self.goal_timeout = float(
            self.get_parameter('goal_timeout').value
        )

        self.map_frame = str(
            self.get_parameter('map_frame').value
        )

        self.avoidance_config = AvoidanceConfig(
            warning_distance=float(
                self.get_parameter('warning_distance').value
            ),
            critical_distance=float(
                self.get_parameter('critical_distance').value
            ),
            avoidance_speed=float(
                self.get_parameter('avoidance_speed').value
            ),
        )

        if not all(math.isfinite(v) and v > 0 for v in (self.control_rate, self.nominal_speed, self.max_speed, self.goal_timeout, self.state_timeout, self.range_timeout, self.slowdown_radius)):
            raise ValueError('Rates, speeds, timeouts and slowdown radius must be finite and positive')

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        if self.nominal_speed <= 0.0:
            raise ValueError('nominal_speed must be positive')

        if self.max_speed <= 0.0:
            raise ValueError('max_speed must be positive')

        if self.goal_timeout <= 0.0:
            raise ValueError('goal_timeout must be positive')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.path_timeout, self.route_point_tolerance, self.path_start_tolerance)):
            raise ValueError('Path timeouts and tolerances must be finite and positive')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _mission_callback(self, msg):
        self.mission_active = msg.state == MissionStatus.NAVIGATE

    def _stamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _state_callback(self, msg):
        """Store the latest normalized vehicle state."""
        if msg.header.frame_id != self.map_frame or not 0 <= self._now()-self._stamp(msg) <= self.state_timeout:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            return
        self.state_stamp = self._stamp(msg)
        self.position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )

        q = msg.pose.pose.orientation

        self.current_yaw = math.atan2(
            2.0 * (
                q.w * q.z
                + q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
            ),
        )

    def _goal_callback(self, msg):
        """Store the latest direct waypoint."""
        if msg.header.frame_id != self.map_frame or not 0 <= self._now()-self._stamp(msg) <= self.goal_timeout:
            self.get_logger().warn(
                f'Ignoring goal in frame "{msg.header.frame_id}"; '
                f'expected "{self.map_frame}"'
            )
            return

        values = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn(
                'Ignoring goal with non-finite position'
            )
            return

        new_goal = tuple(
            float(value)
            for value in values
        )
        if new_goal != self.goal:
            self.route.clear()
            self.path_stamp = None
        self.goal = new_goal
        self.goal_stamp = self._stamp(msg)

    def _path_callback(self, msg):
        if not self.use_planned_path:
            return
        points = tuple((p.pose.position.x, p.pose.position.y, p.pose.position.z)
                       for p in msg.poses)
        valid = (msg.header.frame_id == self.map_frame
                 and 0 <= self._now()-self._stamp(msg) <= self.path_timeout
                 and self.goal is not None and bool(points)
                 and all(all(math.isfinite(v) for v in p) for p in points))
        if valid:
            valid = (math.dist(points[-1], self.goal) <= 1e-4
                     and all(abs(p[2]-self.z_nominal) <= 1e-4 for p in points))
        if valid and points != self.route.points:
            valid = (self.position is not None and
                     math.dist(points[0][:2], self.position[:2]) <= self.path_start_tolerance)
        if not valid:
            self.route.clear()
            self.path_stamp = None
            return
        self.route.set_path(points)
        self.path_stamp = self._stamp(msg)

    def _publish_path_wait(self):
        msg = Setpoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.mode = Setpoint.MODE_HOVER
        msg.position.x, msg.position.y, msg.position.z = self.position
        msg.yaw = self.current_yaw
        self.command_pub.publish(msg)

    def _ranges_callback(self, msg):
        """Store the latest horizontal range measurements."""
        if msg.header.frame_id != self.base_frame or not 0 <= self._now()-self._stamp(msg) <= self.range_timeout:
            return
        self.range_stamp = self._stamp(msg)
        self.ranges = {
            'front': float(msg.front),
            'back': float(msg.back),
            'left': float(msg.left),
            'right': float(msg.right),
        }

    @staticmethod
    def _map_to_body(vx_map, vy_map, yaw):
        """Rotate a horizontal vector from map to base_link."""
        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        return (
            cosine * vx_map + sine * vy_map,
            -sine * vx_map + cosine * vy_map,
        )

    @staticmethod
    def _body_to_map(vx_body, vy_body, yaw):
        """Rotate a horizontal vector from base_link to map."""
        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        return (
            cosine * vx_body - sine * vy_body,
            sine * vx_body + cosine * vy_body,
        )

    def _limit_horizontal_speed(self, vx, vy):
        """Limit horizontal speed magnitude."""
        speed = math.hypot(vx, vy)

        if speed <= self.max_speed or speed == 0.0:
            return vx, vy

        scale = self.max_speed / speed

        return (
            vx * scale,
            vy * scale,
        )

    def _control_tick(self):
        """Publish one local-navigation proposal."""
        if not self.mission_active or self.state_stamp is None or not 0 <= self._now()-self.state_stamp <= self.state_timeout:
            return
        if self.position is None or self.goal is None:
            return

        if self.goal_stamp is None:
            return

        goal_age = (
            self._now()
            - self.goal_stamp
        )

        if not 0 <= goal_age <= self.goal_timeout:
            return

        navigation_target = self.goal
        if self.use_planned_path:
            if (self.path_stamp is None or not 0 <= self._now()-self.path_stamp <= self.path_timeout
                    or not self.route.points):
                self._publish_path_wait()
                return
            navigation_target = self.route.target(self.position)

        max_step = (
            self.nominal_speed
            / self.control_rate
        )

        follower_target = direct_position_setpoint(
            position=self.position,
            goal=navigation_target,
            max_step=max_step,
            z_nominal=self.z_nominal,
        )

        vx_map = (
            follower_target[0]
            - self.position[0]
        ) * self.control_rate

        vy_map = (
            follower_target[1]
            - self.position[1]
        ) * self.control_rate

        distance = math.hypot(self.goal[0]-self.position[0], self.goal[1]-self.position[1])
        scale = min(1.0, distance / self.slowdown_radius)
        vx_map, vy_map = vx_map*scale, vy_map*scale
        if self.ranges is not None and self.range_stamp is not None and 0 <= self._now()-self.range_stamp <= self.range_timeout:
            vx_body, vy_body = self._map_to_body(
                vx_map,
                vy_map,
                self.current_yaw,
            )

            corrected_body, _, _ = reactive_correction(
                ranges=self.ranges,
                closing_rates=None,
                velocity_body=(
                    vx_body,
                    vy_body,
                    0.0,
                ),
                config=self.avoidance_config,
            )

            vx_map, vy_map = self._body_to_map(
                corrected_body[0],
                corrected_body[1],
                self.current_yaw,
            )

        vx_map, vy_map = self._limit_horizontal_speed(
            vx_map,
            vy_map,
        )

        target_x = (
            self.position[0]
            + vx_map / self.control_rate
        )

        target_y = (
            self.position[1]
            + vy_map / self.control_rate
        )

        # Mantener rumbo: un giro cambia los rayos ToF y puede aparentar una
        # velocidad de cierre que no procede de la traslación (TTC). El MVP
        # no exige orientar el morro hacia cada waypoint.
        yaw = self.current_yaw

        msg = Setpoint()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.mode = Setpoint.MODE_VELOCITY

        msg.position.x = float(target_x)
        msg.position.y = float(target_y)
        msg.position.z = float(self.goal[2])

        msg.velocity.x = float(vx_map)
        msg.velocity.y = float(vy_map)
        msg.velocity.z = 0.0

        msg.yaw = float(yaw)

        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LocalNavigator()

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
