"""TP-10, paso 3: comando ROS → movimiento → odometría, sin hardware."""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from hackathon_interfaces.msg import Setpoint
from hackathon_interfaces.srv import Takeoff, Land
from std_srvs.srv import Trigger
from simulation_adapter.sim_node import SimulationAdapter


@pytest.fixture
def rig(request):
    ground = getattr(request, 'param', None) == 'ground'
    start = '[0.0,0.0,0.0,0.0]' if ground else '[0.0,0.0,1.0,0.0]'
    extra = []
    if getattr(request, 'param', None) == 'yaml':
        folder = request.getfixturevalue('tmp_path')
        world = folder / 'world.yaml'
        world.write_text('bounds: [-3, -2, 3, 2]\nceiling: 2.5\nobstacles:\n  - {center: [1, 0], size: [0.2, 0.2]}\n')
        scenario = folder / 'scenario.yaml'
        scenario.write_text('start: [0.0, 0.0, 0.5, 0.0]\n')
        extra = ['-p', f'world_file:={world}', '-p', f'scenario_file:={scenario}',
                 '-p', 'pose_noise_std:=0.01', '-p', 'random_seed:=4']
    rclpy.init(args=['--ros-args', '-p', f'initial_position:={start}',
                    '-p', 'max_speed:=0.5', '-p', f'max_accel:={2.0 if ground else 0.2}',
                    '-p', f'response_time:={0.2 if ground else 1.0}'] + extra)
    simulator = SimulationAdapter()
    peer = Node('tp10_motion_test')
    executor = SingleThreadedExecutor()
    executor.add_node(simulator)
    executor.add_node(peer)
    odometry = []
    subscriber = peer.create_subscription(Odometry, '/hk/adapter/odom', odometry.append, 10)
    publisher = peer.create_publisher(Setpoint, '/hk/cmd/safe', 10)

    def wait(predicate, timeout=10):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'No se observó el resultado ROS esperado'

    def command():
        msg = Setpoint()
        msg.header.stamp = peer.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.mode = Setpoint.MODE_POSITION
        msg.position.x, msg.position.z = 0.2, 1.0
        return msg

    wait(lambda: publisher.get_subscription_count() and odometry)
    yield simulator, peer, executor, publisher, odometry, wait, command
    executor.shutdown(timeout_sec=1)
    simulator.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()


def test_position_command_moves_published_odometry_and_timeout_latches_hover(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    assert odometry[-1].pose.pose.position.x == 0
    assert odometry[-1].pose.pose.position.z == 1
    timer = peer.create_timer(0.05, lambda: publisher.publish(command()))
    wait(lambda: odometry[-1].pose.pose.position.x > 0.18)
    state = odometry[-1]
    assert state.header.frame_id == 'map'
    assert state.child_frame_id == 'base_link'
    assert abs(state.pose.pose.position.z - 1) < 1e-9
    assert state.twist.twist.linear.x > 0
    assert max(abs(msg.twist.twist.linear.x) for msg in odometry) <= 0.5
    timer.cancel()
    wait(lambda: simulator.last_command_time is None and simulator.mode == Setpoint.MODE_HOVER)
    hover_target = simulator.target
    wait(lambda: abs(simulator.state.vx) < 0.005)
    assert simulator.target == hover_target
    assert abs(simulator.state.x - hover_target[0]) < 0.02
    print(f'ROS PASS: x inicial=0, x durante navegación={state.pose.pose.position.x:.3f}, '
          f'hover={simulator.state.x:.3f}, vx={simulator.state.vx:.4f}')


def test_invalid_and_out_of_order_commands_do_not_change_reference(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    original = simulator.target
    for kind in ('frame', 'stale', 'nan', 'unsupported', 'vertical'):
        msg = command()
        if kind == 'frame':
            msg.header.frame_id = 'world'
        elif kind == 'stale':
            msg.header.stamp.sec -= 1
        elif kind == 'nan':
            msg.position.x = math.nan
        elif kind == 'unsupported':
            msg.mode = 99
        else:
            msg.mode = Setpoint.MODE_VELOCITY
            msg.velocity.z = 0.5
        simulator._command_callback(msg)
        assert simulator.target == original
        assert simulator.last_command_time is None
    msg = command()
    simulator._command_callback(msg)
    accepted = simulator.target
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec - 1
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.position.x = -1.0
    simulator._command_callback(msg)
    assert simulator.target == accepted


def test_odometry_velocity_is_in_body_frame(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    simulator.sim_timer.cancel()
    simulator.state.yaw = math.pi / 2
    simulator.state.vx, simulator.state.vy = 0.2, 0.1
    wait(lambda: odometry[-1].pose.pose.orientation.z > 0.7)
    assert odometry[-1].twist.twist.linear.x == pytest.approx(0.1)
    assert odometry[-1].twist.twist.linear.y == pytest.approx(-0.2)


def test_no_command_keeps_initial_state_stationary(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    initial_count = len(odometry)
    wait(lambda: len(odometry) >= initial_count + 5)
    assert simulator.state.x == 0
    assert simulator.state.z == 1
    assert simulator.state.vx == 0


@pytest.mark.parametrize('rig', ['ground'], indirect=True)
def test_ros_takeoff_land_and_commands_blocked_during_maneuvers(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    takeoff = peer.create_client(Takeoff, '/hk/takeoff')
    land = peer.create_client(Land, '/hk/land')
    wait(lambda: takeoff.service_is_ready() and land.service_is_ready())
    publisher.publish(command())
    count = len(odometry)
    wait(lambda: len(odometry) > count + 3)
    assert simulator.state.z == 0
    request = Takeoff.Request()
    request.height, request.duration = 0.2, 0.5
    future = takeoff.call_async(request)
    wait(future.done)
    assert future.result().success
    assert simulator.drone.phase == 'TAKING_OFF'
    timer = peer.create_timer(0.05, lambda: publisher.publish(command()))
    wait(lambda: simulator.drone.phase == 'FLYING')
    timer.cancel()
    assert simulator.state.z == pytest.approx(0.2, abs=0.015)
    assert abs(simulator.state.x) < 0.01  # No aceptó navegación durante despegue.
    request = Land.Request()
    request.duration = 0.5
    future = land.call_async(request)
    wait(future.done)
    assert future.result().success
    timer = peer.create_timer(0.05, lambda: publisher.publish(command()))
    wait(lambda: simulator.drone.phase == 'GROUND')
    timer.cancel()
    wait(lambda: odometry[-1].pose.pose.position.z == 0)
    assert simulator.state.vz == 0
    assert not simulator.state.flying
    print('ROS PASS: suelo → despegue → vuelo → aterrizaje → suelo')


def test_ros_emergency_falls_and_rejects_restart_without_node_reset(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    emergency = peer.create_client(Trigger, '/hk/emergency')
    takeoff = peer.create_client(Takeoff, '/hk/takeoff')
    wait(lambda: emergency.service_is_ready() and takeoff.service_is_ready())
    future = emergency.call_async(Trigger.Request())
    wait(future.done)
    assert future.result().success
    wait(lambda: simulator.state.z == 0)
    request = Takeoff.Request()
    request.height, request.duration = 0.5, 1.0
    future = takeoff.call_async(request)
    wait(future.done)
    assert not future.result().success
    publisher.publish(command())
    count = len(odometry)
    wait(lambda: len(odometry) > count + 3)
    assert simulator.state.z == 0 and simulator.drone.phase == 'EMERGENCY'


@pytest.mark.parametrize('rig', ['ground'], indirect=True)
def test_ros_invalid_takeoff_returns_failure(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    client = peer.create_client(Takeoff, '/hk/takeoff')
    wait(client.service_is_ready)
    request = Takeoff.Request()
    request.height, request.duration = math.nan, 1.0
    future = client.call_async(request)
    wait(future.done)
    assert not future.result().success
    assert simulator.drone.phase == 'GROUND'


def test_ros_ranges_truth_markers_and_motion_are_consistent(rig):
    from hackathon_interfaces.msg import MultiRange
    from visualization_msgs.msg import MarkerArray
    from tf2_msgs.msg import TFMessage
    simulator, peer, executor, publisher, odometry, wait, command = rig
    ranges, truth, markers, transforms = [], [], [], []
    subscriptions = [peer.create_subscription(MultiRange,'/hk/adapter/ranges',ranges.append,10),
                     peer.create_subscription(Odometry,'/hk/sim/ground_truth',truth.append,10),
                     peer.create_subscription(MarkerArray,'/hk/sim/obstacles',markers.append,10),
                     peer.create_subscription(TFMessage,'/tf',transforms.append,10)]
    wait(lambda: ranges and truth and markers and transforms)
    assert ranges[-1].header.frame_id == 'base_link'
    assert ranges[-1].front == pytest.approx(1.98,abs=1e-6)
    assert ranges[-1].down == pytest.approx(.99,abs=1e-6)
    assert ranges[-1].header.stamp.sec > 0
    assert truth[-1].header.frame_id == 'map'
    assert len(markers[-1].markers) == 7
    timer = peer.create_timer(.05,lambda: publisher.publish(command()))
    wait(lambda: odometry[-1].pose.pose.position.x > .18)
    wait(lambda: ranges[-1].front < 1.80)
    assert ranges[-1].front == pytest.approx(2-simulator.state.x-.02,abs=.02)
    timer.cancel()
    print('ROS PASS: comando → movimiento → seis rangos coherentes, ground truth, TF y marcadores')


def test_ros_velocity_and_yaw(rig):
    simulator, peer, executor, publisher, odometry, wait, command = rig
    def velocity():
        msg = command()
        msg.mode = Setpoint.MODE_VELOCITY
        msg.velocity.x = .2
        msg.yaw = math.pi/2
        return msg
    timer = peer.create_timer(.05,lambda: publisher.publish(velocity()))
    wait(lambda: simulator.state.x > .2 and simulator.state.yaw > 1.5)
    wait(lambda: odometry[-1].pose.pose.orientation.z > .68)
    assert simulator.state.z == 1
    assert abs(odometry[-1].twist.twist.linear.x) < .03
    assert odometry[-1].twist.twist.linear.y == pytest.approx(-.2,abs=.03)
    timer.cancel()
    wait(lambda: simulator.mode == Setpoint.MODE_HOVER and simulator.last_command_time is None)


@pytest.mark.parametrize('rig', ['yaml'], indirect=True)
def test_ros_loads_world_scenario_and_keeps_ground_truth_without_pose_noise(rig):
    from hackathon_interfaces.msg import MultiRange
    simulator, peer, executor, publisher, odometry, wait, command = rig
    ranges, truth = [], []
    subscriptions = [peer.create_subscription(MultiRange,'/hk/adapter/ranges',ranges.append,10),
                     peer.create_subscription(Odometry,'/hk/sim/ground_truth',truth.append,10)]
    wait(lambda: ranges and truth)
    assert truth[-1].pose.pose.position.z == .5
    assert odometry[-1].pose.covariance[0] == pytest.approx(.0001)
    assert odometry[-1].pose.pose.position.z != .5
    assert ranges[-1].front == pytest.approx(.88,abs=1e-6)
    assert ranges[-1].up == pytest.approx(1.96,abs=1e-6)


@pytest.mark.parametrize('rig', ['ground'], indirect=True)
def test_ros_complete_flight_with_navigation_and_sensors(rig):
    from hackathon_interfaces.msg import MultiRange
    simulator, peer, executor, publisher, odometry, wait, command = rig
    ranges = []
    subscription = peer.create_subscription(MultiRange,'/hk/adapter/ranges',ranges.append,10)
    takeoff, land = peer.create_client(Takeoff,'/hk/takeoff'), peer.create_client(Land,'/hk/land')
    wait(lambda: takeoff.service_is_ready() and land.service_is_ready() and ranges)
    request = Takeoff.Request()
    request.height, request.duration = .3, .5
    future = takeoff.call_async(request)
    wait(future.done)
    assert future.result().success
    wait(lambda: simulator.drone.phase == 'FLYING')
    def navigate():
        msg = command()
        msg.position.z = .3
        return msg
    timer = peer.create_timer(.05,lambda: publisher.publish(navigate()))
    wait(lambda: abs(simulator.state.x-.2) < .01)
    wait(lambda: abs(ranges[-1].front-1.78) < .02)
    assert ranges[-1].down == pytest.approx(.29,abs=.02)
    timer.cancel()
    request = Land.Request()
    request.duration = .5
    future = land.call_async(request)
    wait(future.done)
    assert future.result().success
    wait(lambda: simulator.drone.phase == 'GROUND')
    wait(lambda: ranges[-1].down == -math.inf and odometry[-1].pose.pose.position.z == 0)
    print('ROS PASS: despegue → POSITION → rangos contra sala → aterrizaje')
