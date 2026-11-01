"""Local map: rolling 2D occupancy grid (TP-22 to TP-25). Priority P1.

Subscribes:
  /hk/ranges                hackathon_interfaces/MultiRange
  /hk/state                 nav_msgs/Odometry
Publishes:
  /hk/map                   nav_msgs/OccupancyGrid  -1 unknown, 0 free, 100 occupied (inflated)
Parameters:
  map_resolution, map_width, map_height, obstacle_inflation_radius, obstacle_decay_time,
  map_rate, state_timeout, range_timeout, sensor_offset, map_frame, base_frame

Only the four horizontal Multi-ranger rays are integrated: the map is 2D at flight
height. Logic: grid.py; ray geometry: range_processor/rays.py.
"""
import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from hackathon_interfaces.msg import MultiRange
from range_processor.rays import SENSOR_DIRECTIONS, ray_in_map

from .grid import RollingGrid

HORIZONTAL_SENSORS = ('front', 'back', 'left', 'right')


class LocalWorldModel(Node):

    def __init__(self):
        super().__init__('local_world_model')
        p = self.declare_parameter
        self.resolution = float(p('map_resolution', 0.05).value)
        self.width = float(p('map_width', 6.0).value)
        self.height = float(p('map_height', 6.0).value)
        self.inflation = float(p('obstacle_inflation_radius', 0.25).value)
        self.decay_time = float(p('obstacle_decay_time', 5.0).value)
        self.map_rate = float(p('map_rate', 10.0).value)
        self.state_timeout = float(p('state_timeout', 0.3).value)
        self.range_timeout = float(p('range_timeout', 0.3).value)
        self.sensor_offset = float(p('sensor_offset', 0.02).value)
        self.map_frame = str(p('map_frame', 'map').value)
        self.base_frame = str(p('base_frame', 'base_link').value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.decay_time, self.map_rate, self.state_timeout, self.range_timeout)):
            raise ValueError('Map rate, decay time and timeouts must be positive and finite')
        if not math.isfinite(self.inflation) or self.inflation < 0:
            raise ValueError('obstacle_inflation_radius must be non-negative and finite')

        self.grid = RollingGrid(self.resolution, self.width, self.height)
        self.pose = None
        self.pose_stamp = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/hk/map', 10)
        self.create_subscription(Odometry, '/hk/state', self._state_callback, 10)
        self.create_subscription(MultiRange, '/hk/ranges', self._ranges_callback, 10)
        self.create_timer(1.0/self.map_rate, self._publish_map)
        self.get_logger().info(
            f'Local world model ready: {self.grid.columns}x{self.grid.rows} cells of '
            f'{self.resolution:.2f} m, inflation {self.inflation:.2f} m, '
            f'decay {self.decay_time:.1f} s -> /hk/map')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def _state_callback(self, msg):
        if msg.header.frame_id != self.map_frame:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, p.z, yaw)
        self.pose_stamp = self._now()
        self.grid.recenter(p.x, p.y)

    def _ranges_callback(self, msg):
        now = self._now()
        if msg.header.frame_id != self.base_frame or not 0 <= now - self._stamp(msg) <= self.range_timeout:
            return
        if self.pose is None or now - self.pose_stamp > self.state_timeout:
            return
        range_max = float(msg.range_max) if math.isfinite(msg.range_max) and msg.range_max > 0 else 4.0
        for sensor in HORIZONTAL_SENSORS:
            offset = tuple(axis*self.sensor_offset for axis in SENSOR_DIRECTIONS[sensor])
            ray = ray_in_map(self.pose, sensor, getattr(msg, sensor), offset, range_max)
            if ray is not None:
                self.grid.integrate_ray(now, *ray)

    def _publish_map(self):
        self.grid.decay(self._now(), self.decay_time)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.grid.resolution
        msg.info.width, msg.info.height = self.grid.columns, self.grid.rows
        msg.info.origin.position.x, msg.info.origin.position.y = self.grid.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.inflated(self.inflation)
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalWorldModel()
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
