"""Sensores virtuales del SIL: TF físico → pose, LaserScan y logging vertical.

No integra movimiento ni envía comandos. Conserva la API /<robot>/* de persona 2.
"""
import math
import random

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from crazyflie_interfaces.msg import LogDataGeneric
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from .dynamics import DroneState
from .sensors import sample_ranges
from .world import World


class SimSensors(Node):
    def __init__(self):
        super().__init__('sim_sensors')
        p = self.declare_parameter
        self.robot = p('robot_name', 'cf231').value
        self.world_frame = p('world_frame', 'world').value
        self.map_frame = p('map_frame', 'map').value
        self.range_min, self.range_max = p('range_min', .02).value, p('range_max', 3.49).value
        self.noise = p('range_noise_std', .01).value
        self.pose_noise = p('pose_noise_std', .005).value
        self.timeout = p('state_timeout', .3).value
        self.rng = random.Random(p('random_seed', 0).value)
        path = p('world_file', '').value
        self.world = World((-2,-2,2,2),2)
        if path:
            with open(path,encoding='utf-8') as stream:
                self.world = World.from_dict(yaml.safe_load(stream))
        self.last_tf = None
        self.first_stamp = None
        self.pose_pub = self.create_publisher(PoseStamped,f'/{self.robot}/pose',10)
        self.scan_pub = self.create_publisher(LaserScan,f'/{self.robot}/scan',10)
        self.extra_pub = self.create_publisher(LogDataGeneric,f'/{self.robot}/ranges_extra',10)
        self.truth_pub = self.create_publisher(Odometry,'/hk/sim/ground_truth',10)
        self.create_subscription(TFMessage,'/tf',self._tf_callback,10)
        self.create_timer(1/p('range_rate',20.0).value,self._ranges)

    def _tf_callback(self,msg):
        for tf in msg.transforms:
            if tf.header.frame_id != self.world_frame or tf.child_frame_id != self.robot:
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec/1e9
            if self.last_tf and stamp <= self._stamp(self.last_tf):
                continue
            v,q = tf.transform.translation,tf.transform.rotation
            if not all(math.isfinite(n) for n in (v.x,v.y,v.z,q.x,q.y,q.z,q.w)):
                continue
            if not math.isclose(math.hypot(q.x,q.y,q.z,q.w),1,abs_tol=.001):
                continue
            self.last_tf = tf
            if self.first_stamp is None:
                self.first_stamp = stamp
            pose = PoseStamped()
            pose.header = tf.header
            pose.pose.position.x = v.x+self.rng.gauss(0,self.pose_noise)
            pose.pose.position.y = v.y+self.rng.gauss(0,self.pose_noise)
            pose.pose.position.z = v.z+self.rng.gauss(0,self.pose_noise)
            pose.pose.orientation = q
            self.pose_pub.publish(pose)
            truth = Odometry()
            truth.header.stamp = tf.header.stamp
            truth.header.frame_id = self.map_frame
            truth.child_frame_id = self.robot
            truth.pose.pose.position.x,truth.pose.pose.position.y,truth.pose.pose.position.z = v.x,v.y,v.z
            truth.pose.pose.orientation = q
            # TF does not measure twist: mark it unknown rather than claiming a measurement.
            for index in (0,7,14,21,28,35):
                truth.twist.covariance[index] = 1e6
            self.truth_pub.publish(truth)

    @staticmethod
    def _stamp(tf):
        return tf.header.stamp.sec + tf.header.stamp.nanosec/1e9

    def _ranges(self):
        if self.last_tf is None:
            return
        age = self.get_clock().now().nanoseconds/1e9-self._stamp(self.last_tf)
        if not 0 <= age <= self.timeout:
            return
        v,q = self.last_tf.transform.translation,self.last_tf.transform.rotation
        state = DroneState(x=v.x,y=v.y,z=v.z)
        ranges = sample_ranges(self.world,state,self._stamp(self.last_tf)-self.first_stamp,
                               self.range_min,self.range_max,self.noise,self.rng,
                               orientation=(q.x,q.y,q.z,q.w))
        scan = LaserScan()
        scan.header.stamp = self.last_tf.header.stamp
        scan.header.frame_id = self.robot
        scan.angle_min,scan.angle_max,scan.angle_increment = -math.pi,math.pi/2,math.pi/2
        scan.range_min,scan.range_max = float(self.range_min),float(self.range_max)
        scan.ranges = [float(ranges[k]) for k in ('back','right','front','left')]
        self.scan_pub.publish(scan)
        extra = LogDataGeneric()
        extra.header = scan.header
        extra.values = [float(ranges[k]*1000) for k in ('up','down')]
        self.extra_pub.publish(extra)


def main(args=None):
    rclpy.init(args=args)
    node=SimSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
