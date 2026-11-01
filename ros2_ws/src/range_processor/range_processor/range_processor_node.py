"""Processing of the five Multi-ranger rays + bottom ToF (TP-19 to TP-21, TP-36).

Subscribes:
  /hk/adapter/ranges        hackathon_interfaces/MultiRange  raw ranges
  /hk/state                 nav_msgs/Odometry
Publishes:
  /hk/ranges                hackathon_interfaces/MultiRange  validated and filtered ranges
  /hk/ranges/closing_rate   hackathon_interfaces/MultiRange  d_dot per sensor (m/s, negative = approaching)
  /hk/ranges/markers        visualization_msgs/MarkerArray   rays in the map for RViz
Parameters:
  range_min, range_max, filter_window, max_valid_jump, spike_confirm_time, invalid_hold,
  sensor_offset, map_frame

Logic: range_filter.py (validation, filter, d_dot), rays.py (ray geometry).

Behaviour
---------
  /hk/ranges and /hk/ranges/closing_rate (TP-20): one output message per input
  message, with the same header (stamp and base_link frame). In closing_rate,
  NaN = d_dot not available (few samples, sensor without return or a jump that was
  just accepted).
  /hk/ranges/markers (TP-21): the filtered rays in the map frame, using the latest
  /hk/state pose; red where the sensor sees an obstacle, grey without return.
"""
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from hackathon_interfaces.msg import MultiRange

from .range_filter import SENSORS, RangeFilter, RangeFilterConfig
from .rays import SENSOR_DIRECTIONS, ray_in_map


class RangeProcessor(Node):

    def __init__(self):
        super().__init__('range_processor')

        p = self.declare_parameter
        config = RangeFilterConfig(
            range_min=p('range_min', 0.02).value,
            range_max=p('range_max', 4.0).value,
            filter_window=p('filter_window', 3).value,
            max_valid_jump=p('max_valid_jump', 1.0).value,
            spike_confirm_time=p('spike_confirm_time', 0.3).value,
            invalid_hold=p('invalid_hold', 0.3).value,
        )
        self.sensor_offset = float(p('sensor_offset', 0.02).value)
        self.map_frame = str(p('map_frame', 'map').value)
        self.config = config
        self.filter = RangeFilter(config)
        self._rejected = 0
        self.pose = None

        self.ranges_pub = self.create_publisher(MultiRange, '/hk/ranges', 10)
        self.rate_pub = self.create_publisher(MultiRange, '/hk/ranges/closing_rate', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/hk/ranges/markers', 10)

        self.create_subscription(MultiRange, '/hk/adapter/ranges', self._on_ranges, 10)
        self.create_subscription(Odometry, '/hk/state', self._on_state, 10)

        self.get_logger().info(
            f'range_processor ready: range [{config.range_min}, {config.range_max}] m, '
            f'median of {config.filter_window}, spike > {config.max_valid_jump} m '
            f'confirmed after {config.spike_confirm_time} s'
        )

    def _on_state(self, msg):
        if msg.header.frame_id != self.map_frame:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, p.z, yaw)

    def _on_ranges(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = {name: getattr(msg, name) for name in SENSORS}
        filtered = self.filter.update(t, raw)

        out = MultiRange()
        out.header = msg.header
        out.range_min = float(self.config.range_min)
        out.range_max = float(self.config.range_max)
        rate = MultiRange()
        rate.header = msg.header
        rate.range_min = out.range_min
        rate.range_max = out.range_max
        for name in SENSORS:
            setattr(out, name, float(filtered[name]))
            d_dot = self.filter.closing_rate(name)
            setattr(rate, name, float(d_dot) if d_dot is not None else math.nan)
        self.ranges_pub.publish(out)
        self.rate_pub.publish(rate)
        self._publish_markers(msg.header.stamp, filtered)

        if self.filter.rejected_spikes != self._rejected:
            self._rejected = self.filter.rejected_spikes
            self.get_logger().warn(f'Range spike discarded (total: {self._rejected})')

    def _publish_markers(self, stamp, filtered):
        if self.pose is None:
            return
        hits, misses = self._line_marker(stamp, 0, (0.9, 0.1, 0.1)), self._line_marker(stamp, 1, (0.6, 0.6, 0.6))
        for name in SENSORS:
            offset = tuple(self.sensor_offset*c for c in SENSOR_DIRECTIONS[name])
            ray = ray_in_map(self.pose, name, filtered[name], offset, float(self.config.range_max))
            if ray is None:
                continue
            origin, end, hit = ray
            target = hits if hit else misses
            target.points.extend(Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in (origin, end))
        self.markers_pub.publish(MarkerArray(markers=[hits, misses]))

    def _line_marker(self, stamp, marker_id, color):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = self.map_frame, stamp
        marker.ns, marker.id = 'ranges', marker_id
        marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RangeProcessor()
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
