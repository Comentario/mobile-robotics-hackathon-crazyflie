"""Safety Supervisor: the only node that publishes commands towards the adapter (TP-37 to TP-39).

States: NORMAL, SLOW, HOVER, LAND, ABORT. Takes priority over mission and navigation.

Subscribes:
  /hk/cmd/nav               hackathon_interfaces/Setpoint
  /hk/state                 nav_msgs/Odometry
  /hk/ranges                hackathon_interfaces/MultiRange
  /hk/ranges/closing_rate   hackathon_interfaces/MultiRange
  /hk/localization_health   hackathon_interfaces/LocalizationHealth
  /hk/planner_status        hackathon_interfaces/PlannerStatus
Publishes:
  /hk/cmd/safe              hackathon_interfaces/Setpoint   filtered command (or hover)
  /hk/safety_state          hackathon_interfaces/SafetyState
Service clients:
  /hk/land, /hk/emergency
Parameters:
  state_timeout, range_timeout, command_timeout, ranges_required, critical_distance,
  ttc_threshold, z_min, z_max, geofence, lost_policy, hover_timeout

Logic: supervisor.py (state machine), watchdog.py, geofence.py.

How the node works
------------------
  1. Every incoming message (state, ranges, health, commands) updates the stored
     data and "feeds" its watchdog.
  2. A timer at SAFETY_RATE Hz evaluates the state machine, publishes
     /hk/safety_state and, when ENTERING LAND or ABORT, calls /hk/land or
     /hk/emergency once.
  3. Every /hk/cmd/nav command is filtered according to the current state and
     published on /hk/cmd/safe. If the navigator stops publishing, the supervisor
     does not publish either: the adapter/firmware watchdog keeps the hover, and
     this node moves to HOVER (and to LAND after hover_timeout if airborne).

HOVER streaming (required with the real drone)
----------------------------------------------
  The real firmware cuts the motors if it stops receiving low-level setpoints
  (crazyflie-firmware 2025.02, supervisor.c: 0.5 s -> levels out and drifts; 2 s ->
  motors off). That is why, with the drone airborne and after some navigator command
  has been received, this node publishes a HOVER Setpoint itself at SAFETY_RATE Hz:
    - whenever the state is HOVER (immediate reaction, without waiting for the
      next navigator message), and
    - in NORMAL/SLOW if the navigator has been silent for more than KEEPALIVE_AGE s
      (before the firmware's 0.5 s).

TTC (TP-36)
-----------
  min_ttc = lowest TTC of the 4 horizontal sensors, using the distance from /hk/ranges
  and the d_dot from /hk/ranges/closing_rate (negative = approaching; NaN = unavailable).
  If min_ttc < ttc_threshold -> SLOW.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from hackathon_interfaces.msg import (
    LocalizationHealth, MultiRange, PlannerStatus, SafetyState, Setpoint,
)
from hackathon_interfaces.srv import Land

from .geofence import Geofence
from .supervisor import (
    AIRBORNE_Z, HOVER, LAND, ABORT, STATE_NAMES, SafetyInputs, SafetySupervisor, SupervisorConfig,
    effective_clearance,
)
from .watchdog import StreamWatchdog


def time_to_collision(distance, closing_rate):
    """Same as range_processor/range_filter.py (copied to avoid a dependency between
    packages). TTC = distance / -d_dot when approaching; inf otherwise."""
    if closing_rate is None or not math.isfinite(closing_rate) or closing_rate >= 0.0:
        return math.inf
    if math.isnan(distance) or distance == math.inf:
        return math.inf
    if distance <= 0.0:
        return 0.0
    return distance / -closing_rate


SAFETY_RATE = 20.0       # Hz at which the safety state is evaluated and published
KEEPALIVE_AGE = 0.25     # s without /hk/cmd/nav before sending HOVER (firmware: 0.5 s)
GEOFENCE_MARGIN = 0.1    # m of tolerance before the drone counts as OUTSIDE the geofence
LAND_DURATION = 2.0      # s of the landing requested by safety


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor')

        # --- Parameters (values in simulation.yaml) ----------------------------
        p = self.declare_parameter
        state_timeout = p('state_timeout', 0.3).value
        range_timeout = p('range_timeout', 0.3).value
        command_timeout = p('command_timeout', 0.5).value
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.range_timeout = range_timeout
        self.state_timeout = state_timeout
        self.command_timeout = command_timeout
        self.z_min = p('z_min', 0.2).value
        self.z_max = p('z_max', 1.5).value
        fence = list(p('geofence', [-2.0, -2.0, 0.0, 2.0, 2.0, 1.5]).value)
        config = SupervisorConfig(
            hover_timeout=p('hover_timeout', 3.0).value,
            lost_policy=p('lost_policy', 'hover_then_land').value,
            critical_distance=p('critical_distance', 0.3).value,
            ttc_threshold=p('ttc_threshold', 1.5).value,
            ranges_required=p('ranges_required', True).value,
            max_decel=p('max_decel', 0.5).value,
            reaction_time=p('reaction_time', 0.3).value,
        )

        # --- Pure logic ---------------------------------------------------------
        self.supervisor = SafetySupervisor(config)
        self.geofence = Geofence(*fence)
        self.health_wd = StreamWatchdog(state_timeout)
        self.state_wd = StreamWatchdog(state_timeout)
        self.range_wd = StreamWatchdog(range_timeout)
        self.cmd_wd = StreamWatchdog(command_timeout)

        # --- Latest received data ----------------------------------------------
        self.position = None                    # (x, y, z) of the drone
        self.localization = LocalizationHealth.LOST
        self.min_range = math.inf
        self.horizontal = {}                    # sensor -> filtered distance
        self.rates = {}                         # sensor -> d_dot (m/s) or None
        self.min_ttc = math.inf
        self.ranges_valid = False
        self.rate_stamp = None
        self.planner_no_path = False
        self._no_path_value = getattr(PlannerStatus, 'NO_PATH', None)
        self._last_state = None
        self._speed_scale = 1.0

        # --- Publishers ---------------------------------------------------------
        self.cmd_pub = self.create_publisher(Setpoint, '/hk/cmd/safe', 10)
        self.state_pub = self.create_publisher(SafetyState, '/hk/safety_state', 10)

        # --- Subscribers --------------------------------------------------------
        self.create_subscription(Setpoint, '/hk/cmd/nav', self._on_nav, 10)
        self.create_subscription(Odometry, '/hk/state', self._on_state, 10)
        self.create_subscription(MultiRange, '/hk/ranges', self._on_ranges, 10)
        self.create_subscription(
            MultiRange, '/hk/ranges/closing_rate', self._on_closing_rate, 10)
        self.create_subscription(
            LocalizationHealth, '/hk/localization_health', self._on_health, 10)
        self.create_subscription(
            PlannerStatus, '/hk/planner_status', self._on_planner, 10)

        # --- Service clients ----------------------------------------------------
        self.land_client = self.create_client(Land, '/hk/land')
        self.emergency_client = self.create_client(Trigger, '/hk/emergency')

        self.create_timer(1.0 / SAFETY_RATE, self._evaluate)

        self.get_logger().info(
            f'safety_supervisor ready: command_timeout={command_timeout} s, '
            f'hover_timeout={config.hover_timeout} s, lost_policy={config.lost_policy}, '
            f'ranges_required={config.ranges_required}, geofence={fence}, '
            f'z=[{self.z_min}, {self.z_max}]'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _fresh(self, msg, timeout):
        age = self._now() - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        return 0 <= age <= timeout

    def _on_state(self, msg):
        if msg.header.frame_id != self.map_frame or not self._fresh(msg, self.state_timeout):
            return
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        self.position = (p.x, p.y, p.z)
        self.state_wd.stamp(self._now())

    def _on_ranges(self, msg):
        if msg.header.frame_id != self.base_frame or not self._fresh(msg, self.range_timeout):
            return
        self.horizontal = {name: getattr(msg, name)
                           for name in ('front', 'back', 'left', 'right')}
        self.ranges_valid = all(not math.isnan(v) for v in self.horizontal.values())
        # Clearance = range - stopping distance towards that obstacle (supervisor.py).
        self.min_range = min(
            [effective_clearance(d, self.rates.get(name), self.supervisor.config)
             for name, d in self.horizontal.items()] or [math.inf]
        )
        self.range_wd.stamp(self._now())

    def _on_closing_rate(self, msg):
        if msg.header.frame_id != self.base_frame or not self._fresh(msg, self.range_timeout):
            return
        self.rate_stamp = self._now()
        ttcs = []
        for name, distance in self.horizontal.items():
            d_dot = getattr(msg, name)
            self.rates[name] = None if math.isnan(d_dot) else d_dot
            ttcs.append(time_to_collision(distance, self.rates[name]))
        self.min_ttc = min(ttcs) if ttcs else math.inf

    def _on_health(self, msg):
        if msg.header.frame_id == self.map_frame and self._fresh(msg, self.state_timeout):
            self.localization = msg.status
            self.health_wd.stamp(self._now())

    def _on_planner(self, msg):
        self.planner_no_path = (
            self._no_path_value is not None and msg.status == self._no_path_value
        )

    def _on_nav(self, msg):
        """Filter the navigator command according to the current state and publish it."""
        if msg.header.frame_id != self.map_frame or not self._fresh(msg, self.command_timeout):
            return
        if not all(math.isfinite(v) for v in (msg.position.x, msg.position.y, msg.position.z,
                                               msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.yaw)):
            return
        self.cmd_wd.stamp(self._now())

        setpoint = {
            'mode': msg.mode,
            'position': (msg.position.x, msg.position.y, msg.position.z),
            'velocity': (msg.velocity.x, msg.velocity.y, msg.velocity.z),
            'yaw': msg.yaw,
        }
        out = self.supervisor.filter_command(
            setpoint, self.supervisor.state, self._speed_scale, self.position,
            geofence=self.geofence, z_limits=(self.z_min, self.z_max),
        )
        if out is None:
            return

        safe = Setpoint()
        safe.header = msg.header
        safe.mode = out['mode']
        safe.position.x, safe.position.y, safe.position.z = out['position']
        safe.velocity.x, safe.velocity.y, safe.velocity.z = out['velocity']
        safe.yaw = out['yaw']
        self.cmd_pub.publish(safe)

    # ------------------------------------------------------------------
    # Periodic evaluation
    # ------------------------------------------------------------------
    def _evaluate(self):
        now = self._now()
        z = self.position[2] if self.position else 0.0

        # Drone on the ground and without commands: the previous navigator no longer
        # counts. Silence means "the mission has not started yet" again, so the drone
        # can take off again after a safety landing.
        if z <= AIRBORNE_Z and self.cmd_wd.received_any() and self.cmd_wd.stale(now):
            self.cmd_wd.last_stamp = None
        if self.rate_stamp is None or now-self.rate_stamp > self.range_timeout:
            self.rates = {}
            self.min_ttc = math.inf
        self.min_range = min([effective_clearance(d, self.rates.get(name), self.supervisor.config)
                              for name, d in self.horizontal.items()] or [math.inf])
        inputs = SafetyInputs(
            now=now,
            localization=LocalizationHealth.LOST if self.health_wd.stale(now) else self.localization,
            state_stale=self.state_wd.stale(now),
            ranges_stale=self.range_wd.stale(now) or not self.ranges_valid,
            # Commands only "expire" if they ever arrived: before the first navigator
            # command the drone is waiting, it has not lost anything.
            command_stale=self.cmd_wd.received_any() and self.cmd_wd.stale(now),
            min_horizontal_range=self.min_range if not self.range_wd.stale(now) else math.inf,
            min_ttc=self.min_ttc if not self.range_wd.stale(now) else math.inf,
            z=z,
            planner_no_path=self.planner_no_path,
            outside_geofence=(self.position is not None
                              and not self.geofence.contains(self.position,
                                                             margin=GEOFENCE_MARGIN)),
        )
        state, speed_scale, reason = self.supervisor.evaluate(inputs)
        self._speed_scale = speed_scale

        # Keep the setpoint streaming alive (see docstring).
        if (z > AIRBORNE_Z and self.position is not None and self.cmd_wd.received_any()
                and state not in (LAND, ABORT)):
            nav_silent = now - self.cmd_wd.last_stamp > KEEPALIVE_AGE
            if state == HOVER or nav_silent:
                self._publish_hover()

        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.speed_scale = float(speed_scale)
        msg.reason = reason
        self.state_pub.publish(msg)

        if state != self._last_state:
            previous = STATE_NAMES[self._last_state] if self._last_state is not None else '-'
            self.get_logger().warn(
                f'Safety: {previous} -> {STATE_NAMES[state]} (reason="{reason}")'
            )
            if state == LAND:
                self._call_land()
            elif state == ABORT:
                self._call_emergency()
            self._last_state = state

    def _publish_hover(self):
        safe = Setpoint()
        safe.header.stamp = self.get_clock().now().to_msg()
        safe.header.frame_id = self.map_frame
        safe.mode = Setpoint.MODE_HOVER
        safe.position.x, safe.position.y, safe.position.z = self.position
        self.cmd_pub.publish(safe)

    def _call_land(self):
        if not self.land_client.service_is_ready():
            self.get_logger().error('LAND requested but /hk/land is not available')
            return
        request = Land.Request()
        request.duration = LAND_DURATION
        self.land_client.call_async(request)
        self.get_logger().warn('Safety: landing requested through /hk/land')

    def _call_emergency(self):
        if not self.emergency_client.service_is_ready():
            self.get_logger().error('ABORT requested but /hk/emergency is not available')
            return
        self.emergency_client.call_async(Trigger.Request())
        self.get_logger().error('Safety: EMERGENCY requested through /hk/emergency')


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
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
