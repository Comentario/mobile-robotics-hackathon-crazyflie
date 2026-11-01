# TP integration and drone 2

> **Historical log** of the `integration/cf2-navigation-mapping` branch, written during the
> hackathon. Both drones are now integrated in `main`: the end-to-end emulated T01 passes,
> `local_world_model` is implemented and the T01–T16 matrix is evaluated automatically.
> See the [README](../README.md) for the current state.

Working branch: `integration/cf2-navigation-mapping`. It does not modify `main`.

## Audit of local references

Local `main` points to `a101bb9` (merge of PR #12, TP-32). TP-26 was already
integrated by PR #8 (`dc3a4f6`). After authenticating to GitHub, `origin/integration-mvp-crazyswarm`
(`61e8316`) was downloaded. Its content is integrated only into our branch, keeping the
references of `main` and of Persons 1/2.

The bringup and live map were saved in `f45cc57`, and `tp31-planner-test-matrix` was
merged through `d9e7b66`. By ancestry that branch includes all the following commits,
without repeated cherry-picks:

| TP | Commit | Component |
|---|---|---|
| 34 | ffe029c | Horizontal avoidance |
| 16 | f5444bb | Localization-based speed policy |
| 27 | 27548ba | A* |
| 28 | 56b06c8 | Costs and unknown-space policy |
| 29 | ecf7b9b | Replanning |
| 30 | 0489953 | Simplification and lookahead |
| 33 | d33367a | Velocity smoothing |
| 35 | ca6a1aa | Vertical avoidance |
| 36 | 49d6894 | Dynamic obstacles |
| 31 | 711a434 | Planner test matrix |

## Result and scope

The Person 1/2 integration provides state, health, ranges, supervisor, mission and
emulation. Person 3's A*, route processing and complete avoidance are kept. Navigation
uses velocity commands compatible with the adapter and only publishes during NAVIGATE,
with recent state and ranges.

- Full build: 14 packages, no errors.
- Component tests: 95 pass, 16 skipped because of the existing skeletons.
- Map tests: 9 pass.
- Full emulated T01 verification: **pending** at the time. The runs found gaps of
  about 55–56 s in the publications and ended by timeout. An overlapping run also found
  duplicated publishers; it was repeated in an isolated domain and the timing problem
  persisted. End-to-end navigation was not considered validated.

## Selected route for drone 2: map first

Use `cf2_first_hover.py --live-map` and `cf2_live_map.py --ros`, following
[the mapping guide](cf2_live_mapping.md). It is a single radio connection, the Flow deck
for local estimation and the Multiranger for the lateral rays. It requires neither
Lighthouse, Crazyswarm2 nor running the whole integrated mission. The browser shows the
incremental map during the flight.

The mapper publishes `/cf2/map`, `/cf2/odom`, `/cf2/ranges` (MultiRange) and the
individual ranges. It keeps its own frame per session; it does not represent an
absolute position nor SLAM. Invalid ranges stay unknown.

Flights without `--navigate` are open loop and do not consume `/hk/cmd/safe`.
The new [CF2 autonomous demo](cf2_autonomous_demo.md) connects the pure TP functions
directly to the cflib process through `--navigate`: A*, tracking, avoidance and
supervisor produce the velocity sent over the radio. It does not run the ROS mission
nodes and does not require Lighthouse. This route passes 17 map/control tests,
including obstacle, return and stale data; its physical response still requires
validation with the mentor.
At the time, `local_world_model` of the general stack was still a skeleton: for our first
goal the map was produced by the implemented CF2 mapper.
Do not start `real.launch.py` together with the radio script.

## Next stages, after the map

1. Validate the continuous map with fixed obstacles and a clear flight at 30 cm.
2. Solve the emulated test and connect map/state/ranges to the navigation pipeline with
   explicit frame parameters and a single real adapter.
3. Validate commands, height and landing before closed-loop control.
4. Only then align the local frame with the Lighthouse frame of drone 1 to share
   information. Sharing topics does not align coordinates.
