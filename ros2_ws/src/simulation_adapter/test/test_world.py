"""Geometry, time windows and six sensors, without ROS."""
import math
from pathlib import Path
import random
import pytest
import yaml
from simulation_adapter.world import World, Box
from simulation_adapter.dynamics import DroneState
from simulation_adapter.sensors import sample_ranges


@pytest.fixture
def room():
    return World((-2, -2, 2, 2), 2)


@pytest.mark.parametrize('direction,expected', [((1,0,0),2),((-1,0,0),2),
    ((0,1,0),2),((0,-1,0),2),((0,0,1),1),((0,0,-1),1)])
def test_room_surfaces(room, direction, expected):
    assert room.raycast((0,0,1), direction, 4, 0) == pytest.approx(expected)


def test_max_range_and_boundary(room):
    assert math.isinf(room.raycast((0,0,1),(1,0,0),1,0))
    assert room.raycast((0,0,1),(1,0,0),2,0) == 2
    assert room.raycast((2,0,1),(1,0,0),4,0) == 0
    assert room.raycast((3,0,1),(-1,0,0),4,0) == 0


def test_nearest_box_parallel_miss_and_height(room):
    room.obstacles = [Box((1,0),(0.2,0.2),0.8), Box((1.5,0),(0.2,0.2))]
    assert room.raycast((0,0,0.5),(1,0,0),4,0) == pytest.approx(0.9)
    assert room.raycast((0,0,1),(1,0,0),4,0) == pytest.approx(1.4)
    assert room.raycast((0,1,1),(1,0,0),4,0) == 2
    assert room.raycast((1,0,0.5),(1,0,0),4,0) == 0


def test_obstacle_time_and_motion(room):
    box = Box((1,0),(0.2,0.2),velocity=(-0.1,0),t_start=2,t_end=5)
    room.obstacles = [box]
    assert not box.active(1.99) and box.active(2) and not box.active(5)
    assert box.center_at(3) == pytest.approx((0.9,0))
    assert room.raycast((0,0,1),(1,0,0),4,1) == 2
    assert room.raycast((0,0,1),(1,0,0),4,3) == pytest.approx(0.8)
    assert room.raycast((0,0,1),(1,0,0),4,5) == 2


def test_sphere_collisions(room):
    room.obstacles = [Box((1,0),(0.2,0.2),0.8)]
    assert not room.collides((0,0,1),0.1,0)
    assert room.collides((1.95,0,1),0.1,0)
    assert room.collides((0,0,0.05),0.1,0)
    assert room.collides((0.8,0,0.5),0.11,0)
    assert not room.collides((1,0,1),0.1,0)


def test_six_ranges_offsets_and_yaw(room):
    state = DroneState(x=0.5,z=1)
    result = sample_ranges(room,state,0,0.02,4)
    assert result == pytest.approx(dict(front=1.48,back=2.48,left=1.98,right=1.98,up=0.96,down=0.99))
    state.yaw = math.pi/2
    rotated = sample_ranges(room,state,0,0.02,4)
    assert rotated['front'] == pytest.approx(1.98)
    assert rotated['left'] == pytest.approx(2.48)


def test_range_rep117_and_seeded_noise(room):
    state = DroneState(z=0)
    assert sample_ranges(room,state,0,0.02,1)['front'] == math.inf
    assert sample_ranges(room,state,0,0.02,4)['down'] == -math.inf
    state.z = 1
    a = sample_ranges(room,state,0,0.02,4,0.01,random.Random(3))
    b = sample_ranges(room,state,0,0.02,4,0.01,random.Random(3))
    assert a == b and a['front'] != 1.98


@pytest.mark.parametrize('data', [dict(bounds=[0,0,0,1],ceiling=2),dict(bounds=[0,0,1,1],ceiling=-1),
    dict(bounds=[0,0,1,1],ceiling=2,obstacles=[dict(center=[0,0],size=[-1,1])]),
    dict(bounds=[0,0,1,1],ceiling=2,obstacles=[dict(type='sphere')])])
def test_invalid_world(data):
    with pytest.raises(ValueError):
        World.from_dict(data)


def test_all_repository_worlds_load():
    root = Path(__file__).resolve().parents[2] / 'hackathon_bringup/config/worlds'
    files = list(root.glob('*.yaml'))
    assert len(files) >= 10
    for path in files:
        world = World.from_dict(yaml.safe_load(path.read_text()))
        assert world.ceiling > 0


def test_rangers_rotate_with_roll_and_pitch(room):
    q=(0,math.sin(math.pi/4),0,math.cos(math.pi/4))
    ranges=sample_ranges(room,DroneState(z=1),0,.02,4,orientation=q)
    assert ranges['front']==pytest.approx(.98)
    assert ranges['up']==pytest.approx(1.96)
