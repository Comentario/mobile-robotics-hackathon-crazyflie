"""ROS 2 wrapper for the TP-26 Mission Manager.

Subscribes:
  /hk/state                 nav_msgs/Odometry
  /hk/localization_health   hackathon_interfaces/LocalizationHealth
  /hk/safety_state          hackathon_interfaces/SafetyState
  /hk/planner_status        hackathon_interfaces/PlannerStatus

Publishes:
  /hk/goal                  geometry_msgs/PoseStamped
  /hk/mission_status        hackathon_interfaces/MissionStatus

Clients:
  /hk/takeoff               hackathon_interfaces/Takeoff
  /hk/land                  hackathon_interfaces/Land

Services:
  /hk/mission/start          std_srvs/Trigger
  /hk/mission/pause          std_srvs/Trigger
  /hk/mission/resume         std_srvs/Trigger
  /hk/mission/abort          std_srvs/Trigger
"""


import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from hackathon_interfaces.msg import (
    LocalizationHealth,
    MissionStatus,
    PlannerStatus,
    SafetyState,
)
from hackathon_interfaces.srv import Land, Takeoff

from .mission import (
    ABORTED,
    LANDING,
    TAKEOFF,
    MissionStateMachine,
    load_mission,
)


class MissionManager(Node):
    """Connect the pure mission state machine to the project ROS API."""

    def __init__(self):
        super().__init__('mission_manager')

        self._declare_parameters()
        self._read_parameters()

        self.machine = None

        self.input_stamps = {}
        self.position = None
        self.health = None
        self.safety_state = None
        self.planner_status = PlannerStatus.IDLE

        self._takeoff_sent = False
        self._land_sent = False
        self._force_land = False

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/hk/goal',
            10,
        )

        self.status_pub = self.create_publisher(
            MissionStatus,
            '/hk/mission_status',
            10,
        )

        self.create_subscription(
            Odometry,
            '/hk/state',
            self._state_callback,
            10,
        )

        self.create_subscription(
            LocalizationHealth,
            '/hk/localization_health',
            self._health_callback,
            10,
        )

        self.create_subscription(
            SafetyState,
            '/hk/safety_state',
            self._safety_callback,
            10,
        )

        self.create_subscription(
            PlannerStatus,
            '/hk/planner_status',
            self._planner_callback,
            10,
        )

        self.takeoff_client = self.create_client(
            Takeoff,
            '/hk/takeoff',
        )

        self.land_client = self.create_client(
            Land,
            '/hk/land',
        )

        self.create_service(
            Trigger,
            '/hk/mission/start',
            self._start_callback,
        )

        self.create_service(
            Trigger,
            '/hk/mission/pause',
            self._pause_callback,
        )

        self.create_service(
            Trigger,
            '/hk/mission/resume',
            self._resume_callback,
        )

        self.create_service(
            Trigger,
            '/hk/mission/abort',
            self._abort_callback,
        )

        self.create_timer(1.0 / self.mission_rate, self._tick)

        if self.mission_file:
            self._load_machine()

        if self.autostart and self.machine is not None:
            self.machine.start(self._now())

        self.get_logger().info(
            'Mission Manager ready: '
            '/hk/goal, /hk/mission_status, /hk/mission/*'
        )

    def _declare_parameters(self):
        """Declare Mission Manager parameters."""
        self.declare_parameter('mission_file', '')
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('waypoint_timeout', 30.0)
        self.declare_parameter('z_nominal', 0.5)
        self.declare_parameter('takeoff_duration', 2.0)
        self.declare_parameter('land_duration', 2.0)
        self.declare_parameter('autostart', False)
        self.declare_parameter('mission_rate', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.input_timeout = self.declare_parameter('state_timeout', 0.3).value

    def _read_parameters(self):
        """Read Mission Manager parameters."""
        self.mission_file = str(
            self.get_parameter('mission_file').value
        )

        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value
        )

        self.waypoint_timeout = float(
            self.get_parameter('waypoint_timeout').value
        )

        self.z_nominal = float(
            self.get_parameter('z_nominal').value
        )

        self.takeoff_duration = float(
            self.get_parameter('takeoff_duration').value
        )

        self.land_duration = float(
            self.get_parameter('land_duration').value
        )

        self.autostart = bool(
            self.get_parameter('autostart').value
        )

        self.mission_rate = float(
            self.get_parameter('mission_rate').value
        )

        self.map_frame = str(
            self.get_parameter('map_frame').value
        )

        if self.mission_rate <= 0.0:
            raise ValueError('mission_rate must be positive')

    def _load_machine(self):
        """Load the configured YAML mission and create its state machine."""
        try:
            waypoints = load_mission(self.mission_file, self.map_frame)

            self.machine = MissionStateMachine(
                waypoints=waypoints,
                tolerance=self.waypoint_tolerance,
                timeout=self.waypoint_timeout,
                takeoff_height=self.z_nominal,
            )

            self.get_logger().info(
                f'Loaded mission "{self.mission_file}" '
                f'with {len(waypoints)} waypoint(s)'
            )

            return True

        except (OSError, ValueError) as exc:
            self.machine = None
            self.get_logger().error(
                f'Cannot load mission "{self.mission_file}": {exc}'
            )
            return False

    def _state_callback(self, msg):
        """Store the latest normalized vehicle position."""
        if msg.header.frame_id != self.map_frame or not self._fresh(msg):
            return
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x,p.y,p.z)):
            return
        self.input_stamps['state'] = self._now()
        self.position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )

    def _health_callback(self, msg):
        """Store the latest localization health state."""
        if msg.header.frame_id != self.map_frame or not self._fresh(msg):
            return
        self.input_stamps['health'] = self._now()
        self.health = int(msg.status)

    def _safety_callback(self, msg):
        """Store the latest Safety Supervisor state."""
        if not self._fresh(msg):
            return
        self.input_stamps['safety'] = self._now()
        self.safety_state = int(msg.state)

    def _planner_callback(self, msg):
        """Store the latest planner state."""
        self.planner_status = int(msg.status)

    def _fresh(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        return 0 <= self._now()-stamp <= self.input_timeout

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        """Advance the mission state machine and execute its actions."""
        if self.machine is None:
            self._publish_status(
                override_reason='mission_not_configured'
            )
            return

        if self._force_land:
            self._request_land()
            self._publish_status()
            return

        if (
            self.position is None
            or self.health is None
            or self.safety_state is None
        ):
            self._publish_status(
                override_reason=self._missing_input_reason()
            )
            return

        action = self.machine.step(
            now=self._now(),
            position=self.position,
            health=self.health if self._now()-self.input_stamps.get('health',-math.inf) <= self.input_timeout else LocalizationHealth.LOST,
            safety_state=self.safety_state if self._now()-self.input_stamps.get('safety',-math.inf) <= self.input_timeout else SafetyState.HOVER,
            planner_status=self.planner_status,
        )

        if self.machine.state == ABORTED and self.machine.reason == 'waypoint_timeout':
            self._force_land = True
            self._request_land()
        if action[0] == 'goal':
            self._publish_goal(action[1])

        # Retry service submission while the state requires it.
        if self.machine.state == TAKEOFF and not self._takeoff_sent:
            self._request_takeoff()

        if self.machine.state == LANDING and not self._land_sent:
            self._request_land()

        self._publish_status()

    def _publish_goal(self, waypoint):
        """Publish the currently active waypoint."""
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = waypoint.x
        msg.pose.position.y = waypoint.y
        msg.pose.position.z = waypoint.z

        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def _publish_status(self, override_reason=None):
        """Publish current mission state and progress."""
        msg = MissionStatus()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        if self.machine is None:
            msg.state = MissionStatus.IDLE
            msg.waypoint_index = 0
            msg.waypoint_count = 0
            msg.reason = override_reason or ''
        else:
            msg.state = self.machine.state
            msg.waypoint_index = self.machine.index
            msg.waypoint_count = len(self.machine.waypoints)

            if override_reason:
                msg.reason = override_reason
            else:
                msg.reason = self.machine.reason

        self.status_pub.publish(msg)

    def _missing_input_reason(self):
        """Describe which required upstream inputs are unavailable."""
        missing = []

        if self.position is None:
            missing.append('state')

        if self.health is None:
            missing.append('localization_health')

        if self.safety_state is None:
            missing.append('safety_state')

        return 'waiting_for:' + ','.join(missing)

    def _request_takeoff(self):
        """Send one takeoff request when the adapter service is ready."""
        if self._takeoff_sent:
            return

        if not self.takeoff_client.service_is_ready():
            return

        request = Takeoff.Request()
        request.height = float(self.z_nominal)
        request.duration = float(self.takeoff_duration)

        self._takeoff_sent = True

        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self._takeoff_response)

        self.get_logger().info(
            f'Requesting takeoff to {self.z_nominal:.2f} m'
        )

    def _takeoff_response(self, future):
        """Handle the adapter takeoff response."""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Takeoff service call failed: {exc}'
            )
            self._abort_machine('takeoff_service_error')
            return

        if not response.success:
            self.get_logger().error(
                f'Takeoff rejected: {response.message}'
            )
            self._abort_machine('takeoff_rejected')
            return

        self.get_logger().info(
            f'Takeoff accepted: {response.message}'
        )

    def _request_land(self):
        """Send one landing request when the adapter service is ready."""
        if self._land_sent:
            return

        if not self.land_client.service_is_ready():
            return

        request = Land.Request()
        request.duration = float(self.land_duration)

        self._land_sent = True

        future = self.land_client.call_async(request)
        future.add_done_callback(self._land_response)

        self.get_logger().info('Requesting landing')

    def _land_response(self, future):
        """Handle the adapter landing response."""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Land service call failed: {exc}'
            )
            self._abort_machine('land_service_error')
            return

        if not response.success:
            self.get_logger().error(
                f'Landing rejected: {response.message}'
            )
            self._abort_machine('land_rejected')
            return

        self.get_logger().info(
            f'Landing accepted: {response.message}'
        )

    def _start_callback(self, request, response):
        """Start a fresh mission."""
        del request

        if not self.mission_file:
            response.success = False
            response.message = 'mission_file is not configured'
            return response

        if not self._load_machine():
            response.success = False
            response.message = 'Failed to load mission'
            return response

        success, message = self.machine.start(
            self._now()
        )

        self._takeoff_sent = False
        self._land_sent = False
        self._force_land = False

        response.success = success
        response.message = message

        return response

    def _pause_callback(self, request, response):
        """Pause the active mission state machine."""
        del request

        if self.machine is None:
            response.success = False
            response.message = 'No mission loaded'
            return response

        success, message = self.machine.pause()

        response.success = success
        response.message = message

        return response

    def _resume_callback(self, request, response):
        """Resume a paused mission."""
        del request

        if self.machine is None:
            response.success = False
            response.message = 'No mission loaded'
            return response

        success, message = self.machine.resume(
            now=self._now()
        )

        response.success = success
        response.message = message

        return response

    def _abort_callback(self, request, response):
        """Abort the mission and request a controlled landing."""
        del request

        if self.machine is None:
            response.success = False
            response.message = 'No mission loaded'
            return response

        success, message = self.machine.abort(
            'operator_abort'
        )

        if success:
            self._force_land = True
            self._land_sent = False

        response.success = success
        response.message = message

        return response

    def _abort_machine(self, reason):
        """Move the state machine to ABORTED after a service failure."""
        if self.machine is None:
            return

        if self.machine.state != ABORTED:
            self.machine.abort(reason)


def main(args=None):
    rclpy.init(args=args)

    node = MissionManager()

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
