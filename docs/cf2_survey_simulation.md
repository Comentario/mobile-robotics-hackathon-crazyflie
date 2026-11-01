# CF2: fixed-sweep live RViz simulation

This is an **ideal software simulation**, not a recording and not a flight.
No Crazyradio, cfclient or physical drone is needed. UI labels are English.
The physical flight profile is unchanged while this simpler workflow is evaluated.

Close previous mapper/RViz instances. From the repository root, run one command:

```bash
bash scripts/cf2_sim_rviz.sh --scenario empty
```

For static obstacles:

```bash
bash scripts/cf2_sim_rviz.sh --scenario obstacles
```

The launcher starts the live mapper, its status panel, RViz and the simulator.
The map remains visible after the simulated survey ends. Close RViz to stop.
The configured area is 5 m along X and 3 m along Y, centred at home. The default
height is 0.30 m. Static panels intersect that plane; one low box lies below it
and is deliberately absent from the lateral-sensor map at this height.
The empty scenario has boundary walls and no internal obstacles.

## Separate movement, sensing and presentation

- A fixed schedule goes from home to the interior upper-left corner, follows
  vertical zigzag lanes spaced at most 0.35 m apart, then returns home.
- Default yaw rate is zero. The simulated speed is 0.15 m/s. `--rate 4` accelerates
  playback four times; it does not change the simulated aircraft speed.
- A* is requested when observed occupied cells obstruct the direct route.
  Reactive correction uses all four current lateral ranges. A detour resumes
  the same scheduled target. A blocked target is recorded after lack of progress;
  it is not reclassified as completed or physically impossible.
- Geometry produces scalar ray distances. The controller receives those readings
  and a measured occupancy map; it never receives the world obstacles directly.
  A geometric collision terminates the simulation as a failure.

To compare continuous rotation with the same scenario:

```bash
bash scripts/cf2_sim_rviz.sh --scenario empty --yaw-rate 8
bash scripts/cf2_sim_rviz.sh --scenario obstacles --yaw-rate 8
```

This does not model Flow-deck yaw-induced drift, actuator dynamics or radio
latency. Successful simulated motion is not a hardware validation.

## Why white patches looked like displaced cake slices

Each accepted measurement clears cells along one ray from the **current measured
pose**, and those cells remain. Rotation sweeps directions while translation
moves the centre. Their accumulated footprint is therefore not necessarily a
circle centred on the current drone. The previous 0.80 m radius made that footprint
large; this profile limits each free-space ray to **0.30 m**.

The colored live beams are clipped to the same local radius. A valid obstacle
closer than that shortens its ray. Invalid/no-return measurements reveal nothing.
Four valid directions in unobstructed space form a cross with four 30 cm arms,
not a synthetic disk. `--reveal-radius 0.2` makes each arm 20 cm (40 cm total).
The supported simulation range is 0.10–0.40 m per arm.

Free-space reveal and occupied returns are deliberately separate:

- White free rays extend at most 0.30 m by default.
- Measured obstacle endpoints up to 1.20 m remain in the raw occupancy map.
- The gap between the local reveal and a distant obstacle stays unknown.
- No white cells are painted behind an obstacle.
- Blue is unknown; the full flight trail persists for the session.

Black tiles use local surface confirmation: three distinct sensor timestamps
spanning at least 0.10 s support a measured occupied cell and its immediate
neighbors, within a two-second sensor-time window. Unknown cells are never
filled by this filter. Orange tiles retain measured occupied cells that are
not yet confirmed; larger orange points show recent range endpoints. This is not a complete object contour or a certified
CF1 navigation map. Each height is only a horizontal observation plane.

## Debug without RViz

```bash
.venv-cf2/bin/python scripts/cf2_survey_sim.py --headless --scenario empty
.venv-cf2/bin/python scripts/cf2_survey_sim.py --headless --scenario obstacles
```

Each run saves `data/runs/cf2_sim_simulation_ID_map.json`, explicitly marked
`simulation=true` and `map_validated_for_drone1=false`. Budget expiry and blocked
return are partial results. This simulator is independent of adaptive obstacle
inspection, dynamic-obstacle tracking and multi-height missions.

Verified ideal fixed-heading runs completed all 28 scheduled endpoints and
returned home: 83.8% observed in the empty scenario and 82.2% with static boxes.
ROS/UDP checks verified the moving trajectory, four rays capped at 0.30 m and
the incremental map. These numbers are software results, not physical accuracy.

The same scenarios with ideal continuous yaw at 8 degrees/s also completed
28/28 endpoints and returned: 87.4% observed without internal obstacles and
83.8% with static boxes. This comparison excludes Flow drift during rotation.

## Denser static-surface demonstration

```bash
bash scripts/cf2_sim_rviz.sh --scenario obstacles --spacing 0.25 --reveal-radius 0.4 --yaw-rate 8
```

Closer lanes and 40 cm free-space rays increase measured area. Obstacle detection
remains at the previous range; reducing it would discard useful surface samples.
The support filter uses sensor timestamps, so accelerating playback does not
change its confidence requirements. This is surface evidence, not sensor
calibration, shape completion or reconstruction of an object's solid interior.
Blue inside or behind an opaque object may legitimately remain unknown.
The physical Flow-deck behavior during yaw still requires separate validation.

Surface-confirmation comparison on identical ideal static geometry: raw occupied
cells stayed at 263, while black confirmed cells increased from 31 to 263.
The denser 0.25 m lane / 0.40 m reveal / 8 degree/s yaw profile completed
38/38 endpoints, returned home and observed 91.1% of the configured area.
No unknown cells were filled as obstacles by the confirmation filter.
