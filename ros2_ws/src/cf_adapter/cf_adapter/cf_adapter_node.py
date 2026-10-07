"""ROS 2 adapter between the Hackathon API and Crazyswarm2.

This package is the only project component allowed to depend directly on
Crazyswarm2-specific topics, services, and message types.

Estado (TP-09)
--------------
  Hecho:      equipo -> dron: /hk/cmd/safe, /hk/takeoff, /hk/land, /hk/emergency.
  Hecho:      dron -> equipo: pose en /hk/adapter/odom (ver "Fuente de la pose").
  Pendiente:  dron -> equipo: rangos en /hk/adapter/ranges (orden del scan sin
              confirmar con hardware; ver interfaces.md, "Lado Crazyswarm2").

Fuente de la pose (parámetro pose_source)
-----------------------------------------
  'topic'  Dron real: se suscribe a /<robot_name>/<pose_topic> (geometry_msgs/PoseStamped),
           que Crazyswarm2 publica si 'pose' está activado en firmware_logging de
           crazyflies.yaml.
  'tf'     Simulación de Crazyswarm2 (backend:=sim): ese backend NO publica
           /<robot>/pose (verificado en TP-08), solo el TF world -> <robot>. Se lee
           ese TF a pose_rate Hz.

  restamp_pose: si es true, header.stamp = hora de llegada a este nodo (reloj del
  laptop). Necesario con 'tf' en simulación, porque Crazyswarm2 sim usa su propio
  reloj (/clock). Con el dron real, comprobar primero si la hora de /cf231/pose ya
  es la del laptop (ver docs/frames.md y el runbook).

Semántica de Setpoint (hackathon_interfaces/msg/Setpoint.msg)
------------------------------------------------------------
  POSITION  ir a position con yaw.
  VELOCITY  seguir velocity en XY manteniendo position.z.
  HOVER     mantener la posición actual.
  cmd_full_state es siempre una orden de POSICIÓN, así que VELOCITY y HOVER se
  convierten a una posición objetivo usando la última pose conocida del dron.
"""

from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import (
    Land as CfLand,
    NotifySetpointsStop,
    Takeoff as CfTakeoff,
)

from hackathon_interfaces.msg import MultiRange, Setpoint
from hackathon_interfaces.srv import (
    Land as HkLand,
    Takeoff as HkTakeoff,
)

from .mapping import (
    seconds_to_duration_parts,
    topic,
    yaw_to_quaternion,
)

# Modo VELOCITY: la posición objetivo se coloca VELOCITY_LEAD s por delante de la
# pose actual en la dirección de la velocidad. Con setpoints a ~20 Hz, el objetivo
# avanza continuamente y el dron se mueve a la velocidad pedida.
VELOCITY_LEAD = 0.2  # s


class CrazyflieAdapter(Node):
    """Expose the project hardware API and translate it to Crazyswarm2."""

    def __init__(self):
        super().__init__('cf_adapter')

        self.declare_parameter('robot_name', 'cf231')
        self.declare_parameter('group_mask', 0)
        self.declare_parameter('land_height', 0.0)
        self.declare_parameter('streaming_valid_ms', 100)
        # Pose del dron -> /hk/adapter/odom
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_source', 'topic')    # 'topic' | 'tf'
        self.declare_parameter('pose_topic', 'pose')      # /<robot_name>/<pose_topic>
        self.declare_parameter('world_frame', 'world')    # frame padre del TF (modo 'tf')
        self.declare_parameter('pose_rate', 50.0)         # Hz (modo 'tf')
        self.declare_parameter('restamp_pose', False)

        self.robot_name = (
            self.get_parameter('robot_name')
            .get_parameter_value()
            .string_value
        )

        self.group_mask = (
            self.get_parameter('group_mask')
            .get_parameter_value()
            .integer_value
        )

        self.land_height = (
            self.get_parameter('land_height')
            .get_parameter_value()
            .double_value
        )

        self.streaming_valid_ms = (
            self.get_parameter('streaming_valid_ms')
            .get_parameter_value()
            .integer_value
        )

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pose_source = self.get_parameter('pose_source').value
        self.world_frame = self.get_parameter('world_frame').value
        self.restamp_pose = self.get_parameter('restamp_pose').value

        # Última pose conocida (x, y, z) del dron, para VELOCITY y HOVER.
        self._last_position = None
        self._hover_point = None
        self._last_tf_stamp = None

        # Project-facing publishers.
        self.odom_pub = self.create_publisher(
            Odometry,
            '/hk/adapter/odom',
            10,
        )

        # TODO(TP-09 / Dresden): rellenar desde /<robot>/scan cuando se confirme
        # el orden de los rangos en el hardware real.
        self.ranges_pub = self.create_publisher(
            MultiRange,
            '/hk/adapter/ranges',
            10,
        )

        # Project-facing command subscriber.
        self.command_sub = self.create_subscription(
            Setpoint,
            '/hk/cmd/safe',
            self._command_callback,
            10,
        )

        # Project-facing services.
        self.takeoff_srv = self.create_service(
            HkTakeoff,
            '/hk/takeoff',
            self._takeoff_callback,
        )

        self.land_srv = self.create_service(
            HkLand,
            '/hk/land',
            self._land_callback,
        )

        self.emergency_srv = self.create_service(
            Trigger,
            '/hk/emergency',
            self._emergency_callback,
        )

        # Crazyswarm2 command publisher.
        self.full_state_pub = self.create_publisher(
            FullState,
            topic(self.robot_name, 'cmd_full_state'),
            10,
        )

        # Crazyswarm2 service clients.
        self.cf_takeoff = self.create_client(
            CfTakeoff,
            topic(self.robot_name, 'takeoff'),
        )

        self.cf_land = self.create_client(
            CfLand,
            topic(self.robot_name, 'land'),
        )

        self.cf_emergency = self.create_client(
            Empty,
            topic(self.robot_name, 'emergency'),
        )

        self.cf_notify_stop = self.create_client(
            NotifySetpointsStop,
            topic(self.robot_name, 'notify_setpoints_stop'),
        )

        # Crazyswarm2 pose source -> /hk/adapter/odom.
        if self.pose_source == 'tf':
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            pose_rate = self.get_parameter('pose_rate').value
            self.create_timer(1.0 / pose_rate, self._poll_tf_pose)
            pose_desc = f'TF {self.world_frame} -> {self.robot_name} a {pose_rate} Hz'
        elif self.pose_source == 'topic':
            pose_topic = topic(self.robot_name, self.get_parameter('pose_topic').value)
            self.create_subscription(PoseStamped, pose_topic, self._pose_callback, 10)
            pose_desc = f'topic {pose_topic}'
        else:
            raise ValueError(f"pose_source debe ser 'topic' o 'tf', no '{self.pose_source}'")

        self.get_logger().info(
            f'cf_adapter ready for robot "{self.robot_name}"'
        )

        self.get_logger().info(
            'Project API: /hk/cmd/safe, /hk/takeoff, /hk/land, '
            '/hk/emergency, /hk/adapter/odom, /hk/adapter/ranges'
        )
        self.get_logger().info(
            f'Pose desde {pose_desc} (restamp_pose={self.restamp_pose})'
        )

    # ------------------------------------------------------------------
    # Dron -> equipo: pose
    # ------------------------------------------------------------------
    def _pose_callback(self, msg):
        """Dron real: PoseStamped de Crazyswarm2 -> Odometry del equipo."""
        self._publish_odom(msg.header.stamp, msg.pose.position, msg.pose.orientation)

    def _poll_tf_pose(self):
        """Simulación de Crazyswarm2: lee el TF world -> robot y lo publica como pose."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.robot_name, Time()
            )
        except TransformException:
            return    # aún no hay TF: no se publica nada (el monitor dirá LOST)

        # Solo publicar si el TF es nuevo (si Crazyswarm2 deja de publicar, no
        # inventamos poses repetidas: el monitor de salud debe notarlo).
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp == self._last_tf_stamp:
            return
        self._last_tf_stamp = stamp

        t = tf.transform.translation
        self._publish_odom(tf.header.stamp, t, tf.transform.rotation)

    def _publish_odom(self, stamp, position, orientation):
        """Construye y publica /hk/adapter/odom (frame map -> base_link)."""
        odom = Odometry()
        odom.header.stamp = (
            self.get_clock().now().to_msg() if self.restamp_pose else stamp
        )
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = position.x
        odom.pose.pose.position.y = position.y
        odom.pose.pose.position.z = position.z
        odom.pose.pose.orientation = orientation
        self.odom_pub.publish(odom)

        self._last_position = (position.x, position.y, position.z)

    # ------------------------------------------------------------------
    # Equipo -> dron: setpoints
    # ------------------------------------------------------------------
    def _command_callback(self, msg):
        """Translate a project Setpoint to a Crazyswarm2 FullState command."""
        if msg.mode not in (
            Setpoint.MODE_POSITION,
            Setpoint.MODE_VELOCITY,
            Setpoint.MODE_HOVER,
        ):
            self.get_logger().error(
                f'Rejected setpoint with unsupported mode: {msg.mode}'
            )
            return

        # VELOCITY y HOVER necesitan saber dónde está el dron.
        if msg.mode != Setpoint.MODE_POSITION and self._last_position is None:
            self.get_logger().warn(
                'Setpoint VELOCITY/HOVER rechazado: aún no se conoce la pose del dron',
                throttle_duration_sec=2.0,
            )
            return

        if msg.mode != Setpoint.MODE_HOVER:
            self._hover_point = None

        command = FullState()
        command.header = msg.header

        if msg.mode == Setpoint.MODE_POSITION:
            target = (msg.position.x, msg.position.y, msg.position.z)
            velocity = (0.0, 0.0, 0.0)
        elif msg.mode == Setpoint.MODE_VELOCITY:
            # XY: un poco por delante de la pose actual. Z: la altura pedida.
            x, y, _ = self._last_position
            target = (
                x + msg.velocity.x * VELOCITY_LEAD,
                y + msg.velocity.y * VELOCITY_LEAD,
                msg.position.z,
            )
            velocity = (msg.velocity.x, msg.velocity.y, 0.0)
        else:  # HOVER: quedarse donde estaba al recibir el primer HOVER.
            if self._hover_point is None:
                self._hover_point = self._last_position
            target = self._hover_point
            velocity = (0.0, 0.0, 0.0)

        command.pose.position.x = target[0]
        command.pose.position.y = target[1]
        command.pose.position.z = target[2]

        qx, qy, qz, qw = yaw_to_quaternion(msg.yaw)

        command.pose.orientation.x = qx
        command.pose.orientation.y = qy
        command.pose.orientation.z = qz
        command.pose.orientation.w = qw

        command.twist.linear.x = velocity[0]
        command.twist.linear.y = velocity[1]
        command.twist.linear.z = velocity[2]

        command.twist.angular.x = 0.0
        command.twist.angular.y = 0.0
        command.twist.angular.z = 0.0

        command.acc.x = 0.0
        command.acc.y = 0.0
        command.acc.z = 0.0

        self.full_state_pub.publish(command)

    def _takeoff_callback(self, request, response):
        """Forward the project takeoff request to Crazyswarm2."""
        if request.height <= 0.0:
            response.success = False
            response.message = 'Takeoff height must be positive'
            return response

        if request.duration <= 0.0:
            response.success = False
            response.message = 'Takeoff duration must be positive'
            return response

        if not self.cf_takeoff.service_is_ready():
            response.success = False
            response.message = 'Crazyflie takeoff service is not available'
            return response

        cf_request = CfTakeoff.Request()
        cf_request.group_mask = self.group_mask
        cf_request.height = request.height

        sec, nanosec = seconds_to_duration_parts(request.duration)
        cf_request.duration.sec = sec
        cf_request.duration.nanosec = nanosec

        self._queue_high_level_request(
            self.cf_takeoff,
            cf_request,
            'takeoff',
        )

        response.success = True
        response.message = 'Takeoff request forwarded to Crazyswarm2'

        return response

    def _land_callback(self, request, response):
        """Forward the project land request to Crazyswarm2."""
        if request.duration <= 0.0:
            response.success = False
            response.message = 'Landing duration must be positive'
            return response

        if not self.cf_land.service_is_ready():
            response.success = False
            response.message = 'Crazyflie land service is not available'
            return response

        cf_request = CfLand.Request()
        cf_request.group_mask = self.group_mask
        cf_request.height = self.land_height

        sec, nanosec = seconds_to_duration_parts(request.duration)
        cf_request.duration.sec = sec
        cf_request.duration.nanosec = nanosec

        self._queue_high_level_request(
            self.cf_land,
            cf_request,
            'land',
        )

        response.success = True
        response.message = 'Land request forwarded to Crazyswarm2'

        return response

    def _emergency_callback(self, request, response):
        """Forward an emergency request immediately to Crazyswarm2."""
        del request

        if not self.cf_emergency.service_is_ready():
            response.success = False
            response.message = 'Crazyflie emergency service is not available'
            return response

        future = self.cf_emergency.call_async(Empty.Request())
        future.add_done_callback(
            partial(self._service_done_callback, label='emergency')
        )

        response.success = True
        response.message = 'Emergency request forwarded to Crazyswarm2'

        return response

    def _queue_high_level_request(self, client, request, label):
        """Release streaming priority before a high-level command."""
        if not self.cf_notify_stop.service_is_ready():
            self.get_logger().warn(
                'notify_setpoints_stop is unavailable; '
                f'forwarding {label} directly'
            )
            self._send_request(client, request, label)
            return

        notify_request = NotifySetpointsStop.Request()
        notify_request.group_mask = self.group_mask
        notify_request.remain_valid_millisecs = self.streaming_valid_ms

        future = self.cf_notify_stop.call_async(notify_request)

        future.add_done_callback(
            lambda _: self._send_request(client, request, label)
        )

    def _send_request(self, client, request, label):
        """Send a Crazyswarm2 service request asynchronously."""
        future = client.call_async(request)
        future.add_done_callback(
            partial(self._service_done_callback, label=label)
        )

    def _service_done_callback(self, future, label):
        """Log completion of an asynchronous Crazyswarm2 service call."""
        try:
            future.result()
            self.get_logger().info(
                f'Crazyswarm2 {label} request completed'
            )
        except Exception as exc:
            self.get_logger().error(
                f'Crazyswarm2 {label} request failed: {exc}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = CrazyflieAdapter()

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
