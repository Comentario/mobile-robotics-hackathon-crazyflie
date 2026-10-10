# CF2: live 5 × 3 m survey

This is an online coverage mission using Flow/EKF and all four lateral
Multiranger sensors. Lighthouse is not required. The flight script owns the
radio; the mapper and RViz only receive telemetry. Work remains on
`integration/cf2-navigation-mapping`, independent of the other team's branch.

## Place the mission

Physically mark the 5 × 3 m rectangle and place CF2 in its center, in clear
space. The first airborne EKF XY pose defines (0,0); the rectangle is
X ∈ [-2.5,2.5], Y ∈ [-1.5,1.5] relative to this reference. X/Y are estimator
axes, not continuously changing body axes. The dimensions are supplied by us;
the drone does not independently identify the arena boundary.

Keep static obstacles fixed and visible at 0.3 m sensor height. Chair legs
and mesh gaps may be missed by the four narrow beams. A broad fixed box/panel
is a clearer first obstacle. Use the textured, illuminated floor already
validated for Flow. Close cfclient and previous map/RViz sessions.

## Run live

Terminal 1:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
bash scripts/cf2_view_live.sh
```

Terminal 2:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --navigate --live-map \
  --height 0.3 --speed 0.2 --warning-distance 0.5 \
  --coverage-area 5 3 --coverage-spacing 0.5 \
  --survey-heights 0.3 --scan-yaw-rate 8 \
  --survey-seconds 180 --waypoint-timeout 30
```

This commands actual horizontal travel, not reactive hover. It surveys in
parallel strips using intermediate targets at most 0.5 m apart, then performs a perpendicular pass. Sweep goals stay away
from the boundary by a margin consistent with the 0.5 m reactive warning (0.6 m from the boundary).
A* uses the current occupancy map, inflation and unknown-space penalties;
it recomputes every second, on target changes, and after a newly observed
obstacle intersects the route (at most once every 0.2 s). The four-sensor
reactive layer operates between replans. Speed remains capped at 0.2 m/s.

If a target has no route for 5 s or cannot be reached within its waypoint
budget, it is deferred. Other sectors are surveyed and deferred targets get
one further attempt with the updated map. After the sweep, a bounded repair
pass selects up to 12 unknown sectors adjacent to observed free space. At
180 s the survey stops adding work and starts returning to (0,0); the return
has its own 30 s deadline. If return is blocked, the script reports that
failure and requests the normal landing routine where it is. Localization
or telemetry failures keep their existing hover/landing behavior.

No software test establishes that this entire new mission works on hardware.
The first physical survey must be evaluated using the live status and recording.
The script does not measure battery endurance; choose the survey budget based
on the aircraft's available flight time, including return and landing.

## What RViz shows

| Element | Meaning |
|---|---|
| Gray background | Unknown space |
| White cells | Observed free space |
| Dark navy cells | Confirmed obstacle returns |
| Orange points | Recent, unconfirmed returns |
| Blue line | Actual EKF XY trajectory |
| Violet line | Current replanned route |
| Navy rectangle | Configured survey boundary |
| Gray / green targets | Pending / visited targets |
| Violet disk | Active target |
| Gold disk | Start reference |
| Blue body / red arrow | Drone position / heading |
| Four colored rays | Current usable lateral range readings |

All geometry is shown in a flat 2D plane. Distorted world-space text has been
removed; the terminals report `LIVE`, position, coverage and phase. The drone
marker turns gray and rays disappear if data becomes stale. After landing
stale data is expected, while the measured map and trajectory remain visible.

Confirmed cells require four returns spanning at least 0.6 s. They remain
throughout the session unless measured free rays overturn their occupancy evidence.
This records static obstacles across the mission without expiring them after
five seconds. Repeated evidence is not a reliable static/dynamic classifier:
a person standing still can also become confirmed. No-return values never
paint unknown space white. The map contains measured geometry only; it does
not invent walls between returns.

## Interpret completion

`SURVEY`, `CROSS_SURVEY`, `REVISIT`, `REPAIR_SURVEY`, `RETURN` identify the mission
phase. The console reports visited targets and the percentage of observed
map cells inside the rectangle separately. A completed route can still leave
unknown cells due to occlusion, beam gaps or invalid returns.

`survey_route_complete` means all scheduled targets were reached and CF2
returned. `survey_partial_returned` means targets remained unresolved when
CF2 returned. Neither means that 100% of the environment was sensed.
`return_unresolved:...` means the return failed; the aircraft requests landing
at its current position. Unreachable under the current map and time budget
is not proof of physical impossibility.

JSONL and the final `_map.json` retain the map, origin, planned path, observed
percentage, visited targets and unresolved target reasons. Coordinates remain
local and `shared_frame_aligned=false`; drone 1 needs a measured coordinate
transform before consuming them. There is no Lighthouse correction for Flow
drift in this mission.

## Software verification

Coverage tests exercise 5 × 3 m wall sensing with pose-feedback motion and
return, live obstacle replanning, bounded deferred-target retry, survey-budget
return, unknown-sector repair, and a fixed-cylinder survey. The cylinder test
returned after the survey deadline with partial coverage and maintained over
0.2 m surface clearance in the ideal sensor model. This model does not simulate
radio delay, flow drift, missed physical returns or aircraft dynamics.

ROS publication was checked with synthetic live telemetry: moving trajectory,
confirmed occupancy, boundary dimensions, active targets, and absence of the
distorted text markers. Python compilation, shell syntax and RViz YAML checks
passed. No physical flight was executed by the coding agent.

## Discrete height sweeps and persistent fog of war

Start by evaluating the single-height command above. To survey multiple levels,
replace `--survey-heights 0.3` with `--survey-heights 0.3 0.45 0.6`.
The first level must match `--height`. Levels are increasing, separated by at
least 0.10 m, with a maximum of five levels between 0.20 and 0.80 m.
`--survey-seconds` is the TOTAL survey budget, shared across levels; 180 s
with three levels initially gives roughly 60 s per level, so partial coverage
is likely. Choose the budget using measured battery endurance, reserving return
and landing time. These additional heights have software validation only.

Each level performs its own sweep/replanning/repair and returns to the same
start XY before climbing at at most 0.08 m/s. Mapping pauses during the climb
and resumes after height settles. Up/down ranging and existing supervisor
conditions apply during transitions. A level's readings are only admitted
within ±0.045 m of its configured EKF height. Heights reference this session's
EKF Z, not an externally surveyed altitude.

`--scan-yaw-rate 8` performs in-place 90-degree scans at 8 degrees/s (about 11.25 s per scan).
Allowed values are 0–15 degrees/s; 0 disables the scan pauses. Rotation is separate from travel. The first scan occurs at the start, and
another scan becomes eligible 10 s after the previous scan ends. The active target and its
progress timeout are frozen during scanning; the total survey budget continues.
Rotation pauses during close-obstacle retreat, return, stale data and altitude changes. Translational
commands remain aligned to the fixed local XY frame using measured yaw.
When rotating, changes between successive ray directions are not treated as
object closing speed; direct distance avoidance still uses all four sensors.

The light-blue rectangle is unknown space. Only valid measured rays reveal
white free cells; confirmed obstacles are black. Colored obstacle outlines
and per-level trails use a fixed palette: blue, orange, purple, green, red.
The composite view overlays all levels; the full blue trail is never trimmed
within a session. Observed cells do not expire by time by default. Measured
free evidence may clear an obstacle. A new flight session resets the view,
preventing independent EKF coordinate frames from being accidentally fused.

Presentation cells are clipped to the rectangle, while the planner retains
nearby outside measurements for collision checks. The rectangle represents
configured survey limits, not a detected physical wall. Red X marks unresolved
targets; diagnostics distinguish blocked start/goal margins, planner bounds
and searches with no resolved route. A apparently open target can still be
unavailable after obstacle inflation, progress timeout or the survey deadline.

Each final `_map.json` exports independent `layers` with configured heights,
raw occupancy, confirmed presentation occupancy and per-level traces. ROS
publishes `/cf2/presentation/layer_0`, `layer_1`, etc. The composite is explicitly
visualization-only: white at one height does not prove free space at another.
`map_validated_for_drone1=false` and `shared_frame_aligned=false` remain set.
Before CF1 navigation, validate static obstacles and Flow drift, measure the
frame transform, and use the matching height layer and vehicle clearance.
A four-ray map is sparse; unseen cells must remain unknown for CF1 planning.

The current verification run passed 37 tests, including a rotating two-level
survey that returned home before changing height, clipped presentation with
outside obstacles still visible to planning, isolated height occupancy,
persistent confirmed cells and a trajectory longer than 2,000 samples.
Live synthetic UDP reception and ROS publication of both height layers and
moving markers passed. This validates software behavior, not physical flight
at the additional heights or accuracy sufficient for CF1 collision avoidance.

## Closer survey profile

The flight CLI now defaults to a 0.50 m avoidance warning distance, reduced
from 0.80 m. The sweep margin correspondingly decreases from 0.90 to 0.60 m
for a 5 × 3 m area. This places the outer sweep targets at X=±1.90 m,
Y=±0.90 m, instead of X=±1.60 m, Y=±0.60 m. Critical clearance remains
0.25 m. Sensor acquisition and mapping still accept their original valid
returns; this changes when avoidance starts, not the physical ToF range.
An explicit `--warning-distance 0.8` overrides the new default.

## Simple live workflow

`MOVE` follows nearby intermediate sweep targets. A* checks the map and the
reactive layer can retreat. Advancing to a reached intermediate target does
not reset the velocity limiter or deliberately issue a zero command. The
purple marker is the local follower target (roughly 0.2 m ahead), and the
purple path shows only the next five path points. Gray dots are the fixed
survey schedule; they are not all immediate movement commands. The local
movement target is capped at 0.5 m from the current XY pose.

`SCAN` gently brakes translation, then turns 90 degrees. The four sensors
jointly sweep every bearing. An orange ring surrounds the robot in RViz.
The console prints `scan_settling` or `scan_quarter_turn`, angular progress,
`target frozen` and `mapping active`. No new goals, repairs or A* replans are
created during this phase. A blocked clearance interrupts scanning so normal
avoidance resumes. A scan that fails to achieve its measured angle has a
bounded timeout before movement resumes.

Mapping continues from incoming valid measurements during both phases. White
triangular tiles have no cube edges; they denote measured free space, not
obstacles. Only filled black cells denote confirmed occupied space. Light
blue remains unknown. This is an evidence-based fog of war, not a circular
visibility radius: four narrow rays cannot prove an entire disk free when
returns are unavailable. Moving without valid returns may leave the blue
trajectory surrounded by unknown space. Nothing synthesizes unseen walls or
free areas to make the display look complete.
