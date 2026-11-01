"""ROS transport tests against synthetic telemetry and mock Crazyswarm2 services.

Run with Humble sourced, in an isolated ROS_DOMAIN_ID, using system Python.
No radio or physical Crazyflie is used.
"""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger
from tf2_msgs.msg import TFMessage
from crazyflie_interfaces.msg import FullState, LogDataGeneric
from crazyflie_interfaces.srv import Land as CfLand, NotifySetpointsStop, Takeoff as CfTakeoff
from hackathon_interfaces.msg import MultiRange, Setpoint
from hackathon_interfaces.srv import Land, Takeoff
from cf_adapter.cf_adapter_node import CrazyflieAdapter


@pytest.fixture
def rig():
    rclpy.init(args=['--ros-args', '-p', 'service_timeout:=0.15', '-p', 'vertical_ranges_topic:=/cf231/vertical_ranges'])
    adapter = CrazyflieAdapter()
    peer = Node('tp09_mock_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(peer)
    received = {'commands': [], 'states': [], 'ranges': [], 'calls': []}
    subscriptions = [
        peer.create_subscription(FullState, '/cf231/cmd_full_state', received['commands'].append, 10),
        peer.create_subscription(Odometry, '/hk/adapter/odom', received['states'].append, 10),
        peer.create_subscription(MultiRange, '/hk/adapter/ranges', received['ranges'].append, 10),
    ]
    pubs = {
        'state': peer.create_publisher(Odometry, '/cf231/odom', 10),
        'scan': peer.create_publisher(LaserScan, '/cf231/scan', 10),
        'vertical': peer.create_publisher(LogDataGeneric, '/cf231/vertical_ranges', 10),
        'command': peer.create_publisher(Setpoint, '/hk/cmd/safe', 10),
    }

    def spin_until(predicate, timeout=3):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'ROS operation did not finish'

    def state(x=2.0):
        msg = Odometry()
        msg.header.stamp = adapter.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'world', 'cf231'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, 3.0, 1.0
        msg.pose.pose.orientation.w = 1.0
        return msg

    def command(mode=Setpoint.MODE_HOVER):
        msg = Setpoint()
        msg.header.stamp = adapter.get_clock().now().to_msg()
        msg.header.frame_id, msg.mode = 'map', mode
        return msg

    spin_until(lambda: pubs['state'].get_subscription_count() and pubs['command'].get_subscription_count())
    yield adapter, peer, executor, received, pubs, spin_until, state, command
    executor.shutdown(timeout_sec=1)
    adapter.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()


def test_telemetry_transport_and_missing_vertical_sensors(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    pubs['state'].publish(state())
    wait(lambda: got['states'])
    assert got['states'][-1].header.frame_id == 'map'
    assert got['states'][-1].child_frame_id == 'base_link'
    scan = LaserScan()
    scan.header.stamp = adapter.get_clock().now().to_msg()
    scan.header.frame_id = 'cf231'
    scan.range_min, scan.range_max = 0.01, 3.49
    scan.angle_min, scan.angle_increment = -math.pi, math.pi / 2
    scan.ranges = [1.0, 2.0, 3.0, math.inf]
    pubs['scan'].publish(scan)
    wait(lambda: got['ranges'])
    result = got['ranges'][-1]
    assert (result.back, result.right, result.front, result.left) == (1, 2, 3, math.inf)
    assert math.isnan(result.up) and math.isnan(result.down)
    vertical = LogDataGeneric()
    vertical.header.stamp = adapter.get_clock().now().to_msg()
    vertical.values = [1200.0, 500.0]
    pubs['vertical'].publish(vertical)
    wait(lambda: adapter.vertical is not None)
    scan.header.stamp = adapter.get_clock().now().to_msg()
    pubs['scan'].publish(scan)
    wait(lambda: len(got['ranges']) == 2)
    assert got['ranges'][-1].up == pytest.approx(1.2)
    assert got['ranges'][-1].down == pytest.approx(0.5)
    adapter.vertical[0].sec -= 1
    scan.header.stamp = adapter.get_clock().now().to_msg()
    pubs['scan'].publish(scan)
    wait(lambda: len(got['ranges']) == 3)
    assert math.isnan(got['ranges'][-1].up)


def test_hover_transport_latches_pose_instead_of_origin(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    pubs['state'].publish(state())
    wait(lambda: adapter.latest_state is not None)
    pubs['command'].publish(command())
    wait(lambda: got['commands'])
    result = got['commands'][-1]
    assert result.header.frame_id == 'world'
    assert (result.pose.position.x, result.pose.position.y, result.pose.position.z) == (2, 3, 1)
    pubs['state'].publish(state(2.1))
    wait(lambda: adapter.latest_state.pose.pose.position.x == 2.1)
    pubs['command'].publish(command())
    wait(lambda: len(got['commands']) == 2)
    assert got['commands'][-1].pose.position.x == 2.0


def test_rejects_stale_wrong_frame_and_nonfinite_commands(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    adapter._odom_callback(state())
    for kind in ('stale', 'frame', 'nan', 'mode'):
        msg = command(Setpoint.MODE_POSITION)
        if kind == 'stale':
            msg.header.stamp.sec -= 1
        elif kind == 'frame':
            msg.header.frame_id = 'body'
        elif kind == 'nan':
            msg.position.x = math.nan
        else:
            msg.mode = 99
        adapter._command_callback(msg)
    adapter.latest_state.header.stamp.sec -= 1
    adapter._command_callback(command())
    assert not adapter.stream_active
    assert adapter.reference.reference is None


def test_landing_handover_acknowledgement_and_stream_lock(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    def stop(req, response):
        got['calls'].append('stop')
        assert not adapter.streaming_allowed
        adapter._command_callback(command())
        return response
    def land(req, response):
        got['calls'].append('land')
        assert req.duration.sec == 1
        return response
    services = [
        peer.create_service(NotifySetpointsStop, '/cf231/notify_setpoints_stop', stop),
        peer.create_service(CfLand, '/cf231/land', land),
        peer.create_service(CfTakeoff, '/cf231/takeoff', lambda req, response: response),
    ]
    adapter._odom_callback(state())
    adapter._command_callback(command())
    adapter_client = peer.create_client(Land, '/hk/land')
    wait(lambda: adapter_client.service_is_ready() and adapter.cf_land.service_is_ready() and adapter.cf_notify_stop.service_is_ready())
    request = Land.Request()
    request.duration = 1.0
    future = adapter_client.call_async(request)
    wait(future.done)
    assert future.result().success
    assert 'acknowledged' in future.result().message
    assert got['calls'] == ['stop', 'land']
    assert not adapter.streaming_allowed
    adapter._command_callback(command())
    assert not adapter.stream_active
    takeoff_client = peer.create_client(Takeoff, '/hk/takeoff')
    wait(lambda: takeoff_client.service_is_ready() and adapter.cf_takeoff.service_is_ready())
    request = Takeoff.Request()
    request.height, request.duration = 0.5, 0.1
    future = takeoff_client.call_async(request)
    wait(future.done)
    assert future.result().success
    assert adapter.streaming_allowed
    assert adapter.high_level_until > adapter._now()


def test_service_timeout_blocks_stream_and_reports_failure(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    pending = Future()
    async def no_response(req, response):
        await pending
        return response
    service = peer.create_service(CfLand, '/cf231/land', no_response, callback_group=ReentrantCallbackGroup())
    client = peer.create_client(Land, '/hk/land')
    wait(lambda: client.service_is_ready() and adapter.cf_land.service_is_ready())
    request = Land.Request()
    request.duration = 1.0
    future = client.call_async(request)
    try:
        wait(future.done)
        assert not future.result().success
        assert 'timed out' in future.result().message
        assert not adapter.streaming_allowed
    finally:
        pending.set_result(None)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.01)


def test_unavailable_service_and_nan_takeoff_are_rejected(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    client = peer.create_client(Takeoff, '/hk/takeoff')
    wait(client.service_is_ready)
    request = Takeoff.Request()
    request.height, request.duration = math.nan, 1.0
    future = client.call_async(request)
    wait(future.done)
    assert not future.result().success
    request.height = 0.5
    future = client.call_async(request)
    wait(future.done)
    assert not future.result().success
    assert 'unavailable' in future.result().message
    assert not adapter.streaming_allowed


def test_tf_source_converts_only_configured_robot(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    transform = TransformStamped()
    transform.header.stamp = adapter.get_clock().now().to_msg()
    transform.header.frame_id, transform.child_frame_id = 'world', 'cf5'
    transform.transform.rotation.w = 1.0
    msg = TFMessage(transforms=[transform])
    adapter._tf_callback(msg)
    assert adapter.latest_state is None
    transform.child_frame_id = 'cf231'
    transform.transform.translation.z = 0.5
    adapter._tf_callback(msg)
    assert adapter.latest_state.pose.pose.position.z == 0.5
    assert adapter.latest_state.twist.covariance[0] == 1e6


def test_missing_priority_release_does_not_send_landing(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    def land(req, response):
        got['calls'].append('land')
        return response
    service = peer.create_service(CfLand, '/cf231/land', land)
    client = peer.create_client(Land, '/hk/land')
    wait(lambda: client.service_is_ready() and adapter.cf_land.service_is_ready())
    adapter.stream_active = True
    request = Land.Request()
    request.duration = 1.0
    future = client.call_async(request)
    wait(future.done)
    assert not future.result().success
    assert 'notify_setpoints_stop' in future.result().message
    assert not got['calls']
    assert not adapter.streaming_allowed


def test_emergency_preempts_pending_priority_handover(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    pending = Future()
    async def stop(req, response):
        got['calls'].append('stop')
        await pending
        return response
    def land(req, response):
        got['calls'].append('land')
        return response
    services = [
        peer.create_service(NotifySetpointsStop, '/cf231/notify_setpoints_stop', stop, callback_group=ReentrantCallbackGroup()),
        peer.create_service(CfLand, '/cf231/land', land),
        peer.create_service(Empty, '/cf231/emergency', lambda req, response: response),
    ]
    land_client = peer.create_client(Land, '/hk/land')
    emergency_client = peer.create_client(Trigger, '/hk/emergency')
    wait(lambda: land_client.service_is_ready() and emergency_client.service_is_ready()
         and adapter.cf_land.service_is_ready() and adapter.cf_notify_stop.service_is_ready()
         and adapter.cf_emergency.service_is_ready())
    adapter.stream_active = True
    request = Land.Request()
    request.duration = 1.0
    landing = land_client.call_async(request)
    wait(lambda: 'stop' in got['calls'])
    try:
        emergency = emergency_client.call_async(Trigger.Request())
        wait(emergency.done)
        assert emergency.result().success
        pending.set_result(None)
        wait(landing.done)
        assert not landing.result().success
        assert 'preempted' in landing.result().message
        assert got['calls'] == ['stop']
        assert not adapter.streaming_allowed
    finally:
        if not pending.done():
            pending.set_result(None)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.01)


def test_flow_only_publishes_without_scan_and_preserves_missing_sensors(rig):
    adapter, peer, executor, got, pubs, wait, state, command = rig
    adapter.flow_only = True
    wait(lambda: pubs['vertical'].get_subscription_count())
    msg = LogDataGeneric()
    msg.header.stamp = adapter.get_clock().now().to_msg()
    msg.values = [200.0]
    pubs['vertical'].publish(msg)
    wait(lambda: len(got['ranges']) == 1)
    result = got['ranges'][-1]
    assert result.down == pytest.approx(.2)
    assert result.header.frame_id == 'base_link'
    assert result.header.stamp == msg.header.stamp
    assert all(math.isnan(getattr(result, name)) for name in ('front','back','left','right','up'))
    # A scan cannot claim horizontal measurements when the deck is absent.
    scan = LaserScan()
    scan.header.stamp = adapter.get_clock().now().to_msg()
    adapter._scan_callback(scan)
    assert len(got['ranges']) == 1
    # Wrong logging schema and expired samples must not be republished.
    msg.values = [1000.0, 200.0]
    adapter._vertical_callback(msg)
    msg.values = [200.0]
    msg.header.stamp.sec -= 1
    adapter._vertical_callback(msg)
    assert len(got['ranges']) == 1
    for value, expected in ((0.0,-math.inf),(5000.0,math.inf),(math.nan,math.nan)):
        msg.header.stamp = adapter.get_clock().now().to_msg()
        msg.values = [value]
        previous = len(got['ranges'])
        pubs['vertical'].publish(msg)
        wait(lambda: len(got['ranges']) > previous)
        actual = got['ranges'][-1].down
        assert math.isnan(actual) if math.isnan(expected) else actual == expected
