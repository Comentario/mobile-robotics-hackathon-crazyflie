"""Unit tests for local_world_model.grid (pure logic, no ROS).

Run: colcon test --packages-select local_world_model && colcon test-result --verbose
"""
from local_world_model.grid import FREE, OCCUPIED, UNKNOWN, RollingGrid


def grid():
    return RollingGrid(0.05, 6.0, 6.0)


def test_front_wall_appears_in_correct_cells():
    g = grid()
    g.integrate_ray(0.0, (0.0, 0.0, .5), (1.0, 0.0, .5), True)
    assert g.value(1.0, 0.0) == OCCUPIED
    assert g.value(0.95, 0.0) == FREE


def test_free_space_along_ray():
    g = grid()
    g.integrate_ray(0.0, (0.0, 0.0, .5), (0.0, 1.5, .5), True)
    assert all(g.value(0.0, y/10) == FREE for y in range(0, 14))
    assert g.value(0.0, 1.5) == OCCUPIED
    assert g.value(0.0, 1.6) == UNKNOWN


def test_ray_without_return_changes_nothing():
    """A reading with no echo is not evidence of free space (REP-117 +inf)."""
    g = grid()
    g.integrate_ray(0.0, (0.0, 0.0, .5), (1.0, 0.0, .5), True)
    g.integrate_ray(1.0, (0.0, 0.0, .5), (0.0, 2.0, .5), False)
    assert g.value(1.0, 0.0) == OCCUPIED      # the obstacle is not erased
    assert g.value(0.0, 1.0) == UNKNOWN       # and nothing is marked free


def test_unobserved_is_unknown():
    g = grid()
    assert set(g.cells) == {UNKNOWN}
    assert g.value(10.0, 0.0) is None


def test_recenter_keeps_observations():
    g = grid()
    g.integrate_ray(0.0, (0.0, 0.0, .5), (1.0, 0.0, .5), True)
    g.recenter(1.2, 0.3)
    assert g.value(1.0, 0.0) == OCCUPIED
    x, y = g.cell_to_world(g.columns//2, g.rows//2)
    assert abs(x-1.2) <= g.resolution and abs(y-0.3) <= g.resolution


def test_occupied_cell_decays():
    g = grid()
    g.integrate_ray(0.0, (0.0, 0.0, .5), (1.0, 0.0, .5), True)
    g.decay(4.0, 5.0)
    assert g.value(1.0, 0.0) == OCCUPIED
    g.decay(6.0, 5.0)
    assert g.value(1.0, 0.0) == UNKNOWN
    # A later ray that reaches past the old obstacle also clears it.
    g.integrate_ray(7.0, (0.0, 0.0, .5), (1.0, 0.0, .5), True)
    g.integrate_ray(8.0, (0.0, 0.0, .5), (2.0, 0.0, .5), True)
    assert g.value(1.0, 0.0) == FREE


def test_inflation_radius():
    g = grid()
    g.integrate_ray(0.0, (0.0, 0.0, .5), (1.0, 0.0, .5), True)
    inflated = g.inflated(0.25)
    i, j = g.world_to_cell(1.0, 0.0)
    assert inflated[j*g.columns + i + 5] == OCCUPIED
    assert inflated[j*g.columns + i + 6] != OCCUPIED
    assert g.value(1.2, 0.0) != OCCUPIED   # the raw grid is unchanged


def test_corridor_stays_open():
    g = grid()
    # Walls 0.4 m to each side; inflation 0.15 m leaves the centre line free.
    for x in range(0, 20):
        g.integrate_ray(0.0, (x/10, 0.0, .5), (x/10, 0.4, .5), True)
        g.integrate_ray(0.0, (x/10, 0.0, .5), (x/10, -0.4, .5), True)
    inflated = g.inflated(0.15)
    for x in range(0, 20):
        i, j = g.world_to_cell(x/10, 0.0)
        assert inflated[j*g.columns + i] == FREE
