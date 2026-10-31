"""Local navigation node for TP-32 and TP-34.

Inputs:
  /hk/state                 nav_msgs/Odometry
  /hk/goal                  geometry_msgs/PoseStamped
  /hk/ranges                hackathon_interfaces/MultiRange
  /hk/localization_health   hackathon_interfaces/LocalizationHealth

Output:
  /hk/cmd/nav               hackathon_interfaces/Setpoint

TP-32 provides the bounded direct-goal position follower.
TP-34 adds reactive horizontal obstacle avoidance.
TP-16 adapts navigation speed to localization health.

Only safety_supervisor may publish /hk/cmd/safe.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from hackathon_interfaces.msg import (
    LocalizationHealth,
    MissionStatus,
    MultiRange,
    Setpoint,
)

from .avoidance import (
    AvoidanceConfig,
    estimate_closing_rates,
    reactive_correction,
    vertical_limit,
)
from .follower import (
    FollowerConfig,
    RateLimiter,
    desired_velocity,
    direct_position_setpoint,
    lookahead_point,
    yaw_towards,
)
from .route_follower import RouteFollower
from .speed_policy import navigation_speed


class LocalNavigator(Node):
    """Generate bounded navigation proposals with horizontal avoidance."""

    def __init__(self):
        super().__init__('local_navigator')

        self._declare_parameters()
        self._read_parameters()

        self.follower_config = FollowerConfig(
            lookahead_distance=self.lookahead_distance,
            max_speed=self.max_speed,
            max_accel=self.max_accel,
            goal_slowdown_radius=self.goal_slowdown_radius,
            z_nominal=self.z_nominal,
        )

        self.rate_limiter = RateLimiter(
            max_accel=self.max_accel
        )

        self.last_control_monotonic = None

        self.position = None
        self.current_yaw = 0.0
        self.goal = None
        self.last_goal_monotonic = None
        self.processed_path = []
        self.processed_path_received = False
        self.ranges = None
        self.closing_rates = None
        self.last_ranges_monotonic = None
        self.last_replan_reason = None
        self.localization_status = None
        self.mission_active = False
        self.last_state_monotonic = None
        # CF1 cooperative route: ordered tracking of an offline-planned path.
        self.route = RouteFollower(self.route_point_tolerance, self.path_start_tolerance)
        self.path_stamp = None
        self.route_yaw = None

        self.command_pub = self.create_publisher(
            Setpoint,
            '/hk/cmd/nav',
            10,
        )
        self.create_subscription(MissionStatus, '/hk/mission_status',
                                 self._mission_callback, 10)

        self.replan_pub = self.create_publisher(
            String,
            '/hk/replan_request',
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
            Path,
            '/hk/path/processed',
            self._path_callback,
            10,
        )

        self.create_subscription(
            MultiRange,
            '/hk/ranges',
            self._ranges_callback,
            10,
        )

        self.create_subscription(
            LocalizationHealth,
            '/hk/localization_health',
            self._localization_health_callback,
            10,
        )

        self.steady_clock = Clock(
            clock_type=ClockType.STEADY_TIME
        )

        self.create_timer(
            1.0 / self.control_rate,
            self._control_tick,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            'TP-16/30/32/33/34/35/36 local navigator ready: '
            '/hk/state + /hk/goal + /hk/path/processed + /hk/ranges + '
            '/hk/localization_health -> /hk/cmd/nav'
        )

    def _declare_parameters(self):
        """Declare local-navigation parameters."""
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('lookahead_distance', 0.4)
        self.declare_parameter('nominal_speed', 0.3)
        self.declare_parameter('degraded_speed', 0.15)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_accel', 0.5)
        self.declare_parameter('goal_slowdown_radius', 0.5)
        self.declare_parameter('z_nominal', 0.5)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 1.5)
        self.declare_parameter('goal_timeout', 0.5)
        self.use_planned_path = self.declare_parameter('use_planned_path', False).value
        self.path_timeout = self.declare_parameter('path_timeout', 0.5).value
        self.route_point_tolerance = self.declare_parameter('route_point_tolerance', 0.03).value
        self.path_start_tolerance = self.declare_parameter('path_start_tolerance', 0.15).value
        self.declare_parameter('map_frame', 'map')
        self.state_timeout = float(self.declare_parameter('state_timeout', 0.3).value)
        self.range_timeout = float(self.declare_parameter('range_timeout', 0.3).value)

        self.declare_parameter('warning_distance', 0.5)
        self.declare_parameter('critical_distance', 0.2)
        self.declare_parameter('ttc_threshold', 1.5)
        self.declare_parameter('avoidance_speed', 0.2)

    def _read_parameters(self):
        """Read and validate local-navigation parameters."""
        self.control_rate = float(
            self.get_parameter('control_rate').value
        )

        self.lookahead_distance = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        self.nominal_speed = float(
            self.get_parameter('nominal_speed').value
        )

        self.degraded_speed = float(
            self.get_parameter('degraded_speed').value
        )

        self.max_speed = float(
            self.get_parameter('max_speed').value
        )

        self.max_accel = float(
            self.get_parameter('max_accel').value
        )

        self.goal_slowdown_radius = float(
            self.get_parameter(
                'goal_slowdown_radius'
            ).value
        )

        self.z_nominal = float(
            self.get_parameter('z_nominal').value
        )

        self.z_min = float(
            self.get_parameter('z_min').value
        )

        self.z_max = float(
            self.get_parameter('z_max').value
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
            ttc_threshold=float(
                self.get_parameter('ttc_threshold').value
            ),
            avoidance_speed=float(
                self.get_parameter('avoidance_speed').value
            ),
        )

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        if self.lookahead_distance <= 0.0:
            raise ValueError(
                'lookahead_distance must be positive'
            )

        if self.nominal_speed <= 0.0:
            raise ValueError('nominal_speed must be positive')

        if self.degraded_speed <= 0.0:
            raise ValueError('degraded_speed must be positive')

        if self.degraded_speed > self.nominal_speed:
            raise ValueError(
                'degraded_speed must not exceed nominal_speed'
            )

        if self.max_speed <= 0.0:
            raise ValueError('max_speed must be positive')

        if self.max_accel <= 0.0:
            raise ValueError('max_accel must be positive')

        if self.goal_slowdown_radius <= 0.0:
            raise ValueError(
                'goal_slowdown_radius must be positive'
            )

        if not (
            math.isfinite(self.z_min)
            and math.isfinite(self.z_max)
        ):
            raise ValueError(
                'z_min and z_max must be finite'
            )

        if self.z_min > self.z_max:
            raise ValueError(
                'z_min must not exceed z_max'
            )

        if not self.z_min <= self.z_nominal <= self.z_max:
            raise ValueError(
                'z_nominal must lie inside [z_min, z_max]'
            )

        if self.goal_timeout <= 0.0:
            raise ValueError('goal_timeout must be positive')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.path_timeout, self.route_point_tolerance, self.path_start_tolerance)):
            raise ValueError('Path timeouts and tolerances must be finite and positive')

    def _state_callback(self, msg):
        """Store the latest normalized vehicle state."""
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.header.frame_id != self.map_frame or not all(
            math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        ):
            return
        self.last_state_monotonic = time.monotonic()
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
        if msg.header.frame_id not in ('', self.map_frame):
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

        if (
            self.goal is not None
            and new_goal != self.goal
            and self.processed_path_received
        ):
            # Once path-based navigation is active, wait for the
            # new processed path instead of following the new goal
            # directly during the planner transition.
            self.processed_path = []

        self.goal = new_goal
        self.last_goal_monotonic = time.monotonic()

    def _path_callback(self, msg):
        """Store the latest processed global path."""
        if self.use_planned_path:
            self._route_path_callback(msg)
            return

        if msg.header.frame_id not in ('', self.map_frame):
            self.get_logger().warn(
                f'Ignoring processed path in frame '
                f'"{msg.header.frame_id}"; '
                f'expected "{self.map_frame}"'
            )
            return

        points = []

        for pose in msg.poses:
            values = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )

            if not all(
                math.isfinite(value)
                for value in values
            ):
                self.get_logger().warn(
                    'Ignoring processed path with '
                    'non-finite position'
                )
                return

            points.append(
                tuple(
                    float(value)
                    for value in values
                )
            )

        self.processed_path_received = True
        self.processed_path = points

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _route_path_callback(self, msg):
        """Accept a fresh, complete cooperative route that ends at the goal."""
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
        """Store ranges and estimate horizontal obstacle closing speeds."""
        now = time.monotonic()

        new_ranges = {
            'front': float(msg.front),
            'back': float(msg.back),
            'left': float(msg.left),
            'right': float(msg.right),
            'up': float(msg.up),
            'down': float(msg.down),
        }

        if (
            self.ranges is not None
            and self.last_ranges_monotonic is not None
        ):
            dt = (
                now
                - self.last_ranges_monotonic
            )

            if (
                math.isfinite(dt)
                and dt > 0.0
            ):
                self.closing_rates = (
                    estimate_closing_rates(
                        previous_ranges=self.ranges,
                        current_ranges=new_ranges,
                        dt=dt,
                    )
                )
            else:
                self.closing_rates = None
        else:
            self.closing_rates = None

        self.ranges = new_ranges
        self.last_ranges_monotonic = now

    def _localization_health_callback(self, msg):
        """Store the latest localization-health state."""
        self.localization_status = int(msg.status)

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

    def _request_replan(self, reason):
        """Publish one replan request per continuous blocked condition."""
        reason = str(reason or 'local_blocked')

        if reason == self.last_replan_reason:
            return

        msg = String()
        msg.data = reason

        self.replan_pub.publish(msg)

        self.last_replan_reason = reason

    def _control_tick(self):
        """Publish one smoothed local-navigation proposal."""
        if not self.mission_active or self.last_state_monotonic is None:
            return
        if time.monotonic() - self.last_state_monotonic > self.state_timeout:
            return
        if self.last_ranges_monotonic is None or time.monotonic() - self.last_ranges_monotonic > self.range_timeout:
            return
        if self.position is None:
            return

        if self.processed_path_received:
            if not self.processed_path:
                self.rate_limiter.reset()
                self.last_control_monotonic = None
                return

            navigation_target = lookahead_point(
                path=self.processed_path,
                position=self.position,
                lookahead_distance=self.lookahead_distance,
            )

            final_goal = self.processed_path[-1]

        else:
            if self.goal is None:
                self.rate_limiter.reset()
                self.last_control_monotonic = None
                return

            if self.last_goal_monotonic is None:
                return

            goal_age = (
                time.monotonic()
                - self.last_goal_monotonic
            )

            # In route mode freshness is guarded by path_timeout (hover while
            # stale). mission_manager pauses goals during a safety HOVER, so
            # requiring a fresh goal here would never let the vehicle recover.
            if goal_age > self.goal_timeout and not self.use_planned_path:
                self.rate_limiter.reset()
                self.last_control_monotonic = None
                return

            navigation_target = self.goal
            final_goal = self.goal

        active_speed = navigation_speed(
            status=self.localization_status,
            nominal_speed=self.nominal_speed,
            degraded_speed=self.degraded_speed,
        )

        if active_speed is None:
            self.rate_limiter.reset()
            self.last_control_monotonic = None
            return

        if self.use_planned_path:
            # CF1 cooperative route: hover until a fresh route is accepted,
            # then follow its points in order instead of a lookahead shortcut.
            if (self.path_stamp is None or not 0 <= self._now()-self.path_stamp <= self.path_timeout
                    or not self.route.points):
                self.rate_limiter.reset()
                self.last_control_monotonic = None
                self._publish_path_wait()
                return
            navigation_target = self.route.target(self.position)
            final_goal = self.route.points[-1]

        distance_to_goal = math.hypot(
            final_goal[0] - self.position[0],
            final_goal[1] - self.position[1],
        )

        requested_velocity = desired_velocity(
            position=self.position,
            target=navigation_target,
            speed_limit=active_speed,
            distance_to_goal=distance_to_goal,
            config=self.follower_config,
        )

        now = time.monotonic()

        if self.last_control_monotonic is None:
            dt = 1.0 / self.control_rate
        else:
            dt = now - self.last_control_monotonic

            if (
                not math.isfinite(dt)
                or dt <= 0.0
            ):
                dt = 1.0 / self.control_rate

            dt = min(
                dt,
                2.0 / self.control_rate,
            )

        self.last_control_monotonic = now

        limited_velocity = self.rate_limiter.apply(
            dt=dt,
            velocity=requested_velocity,
        )

        vx_map = limited_velocity[0]
        vy_map = limited_velocity[1]

        # Localization-health speed limits have priority over smoothing.
        active_horizontal_speed = math.hypot(
            vx_map,
            vy_map,
        )

        if (
            active_horizontal_speed > active_speed
            and active_horizontal_speed > 0.0
        ):
            scale = (
                active_speed
                / active_horizontal_speed
            )

            vx_map *= scale
            vy_map *= scale

            self.rate_limiter.reset(
                (
                    vx_map,
                    vy_map,
                    0.0,
                )
            )

        # Reactive avoidance is intentionally applied after smoothing so
        # obstacle responses are not delayed by the acceleration limiter.
        if self.ranges is not None:
            vx_body, vy_body = self._map_to_body(
                vx_map,
                vy_map,
                self.current_yaw,
            )

            corrected_body, blocked, reason = reactive_correction(
                ranges=self.ranges,
                closing_rates=self.closing_rates,
                velocity_body=(
                    vx_body,
                    vy_body,
                    0.0,
                ),
                config=self.avoidance_config,
            )

            if blocked:
                self._request_replan(reason)
            else:
                self.last_replan_reason = None

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

        if self.ranges is None:
            up_range = math.nan
            down_range = math.nan
        else:
            up_range = self.ranges.get(
                'up',
                math.nan,
            )
            down_range = self.ranges.get(
                'down',
                math.nan,
            )

        z_target = vertical_limit(
            z_target=self.z_nominal,
            up_range=up_range,
            down_range=down_range,
            z_min=self.z_min,
            z_max=self.z_max,
            critical_distance=(
                self.avoidance_config.critical_distance
            ),
            current_z=self.position[2],
        )

        if self.use_planned_path:
            # Route points are a few cm apart: facing each one makes the yaw
            # swing and fakes range closing rates. Hold the initial heading.
            if self.route_yaw is None:
                self.route_yaw = self.current_yaw
            yaw = self.route_yaw
        else:
            yaw = yaw_towards(
                position=self.position,
                target=navigation_target,
                current_yaw=self.current_yaw,
            )

        msg = Setpoint()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.mode = Setpoint.MODE_VELOCITY

        msg.position.x = float(target_x)
        msg.position.y = float(target_y)
        msg.position.z = float(z_target)

        msg.velocity.x = float(vx_map)
        msg.velocity.y = float(vy_map)
        msg.velocity.z = 0.0

        msg.yaw = float(yaw)

        self.command_pub.publish(msg)

    def _mission_callback(self, msg):
        self.mission_active = msg.state == MissionStatus.NAVIGATE
        if not self.mission_active:
            self.rate_limiter.reset()
            self.last_control_monotonic = None


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
