# Shared CF2 simulation result

`cf2_sim_simulation_387101d5_map.json` is an exact copy of the latest local
simulation export selected on 2026-10-11. It is **not a physical flight map**.

- Source: `data/runs/cf2_sim_simulation_387101d5_map.json`.
- SHA-256: `bca8be4008bd97e9614c94247da4244ad35a0971fffc6413f66bdff8275a503e`.
- Area: 5 m along X × 3 m along Y; height: 0.30 m.
- Resolution: 0.05 m; grid: 160 × 160, origin (-4, -4) m.
  The survey rectangle occupies part of that storage grid.
- Free-space reveal radius: 0.40 m; occupied-return mapping radius: 1.20 m.
- Result: 38/38 scheduled targets visited, 91.1% observed, `sweep_returned`.

The JSON contains occupancy grids, per-height layers, the complete trail,
range readings, mission results and the session-local coordinate frame.
Occupancy values are -1 (unknown), 0 (free) and 100 (occupied).
`presentation_data` is filtered visual evidence; the composite is visual only.
Inspect the layer data for mapping consumers rather than treating visualization
as a collision-free route. Unknown space is not established free space.

`simulation=true`, `shared_frame_aligned=false` and
`map_validated_for_drone1=false` are intentionally preserved. Drone 1 cannot
use these coordinates as Lighthouse coordinates without frame alignment and
physical validation. This final snapshot is not a replay telemetry recording.

## Before a physical test

The ideal simulator uses a fixed vertical sweep and local detours. The physical
`cf2_first_hover.py` still defaults to adaptive `--coverage-mode explore`;
its `survey` mode is an older coverage workflow. The simulator's short reveal
radius is not currently a physical-flight CLI setting. Therefore running the
previous flight command does not reproduce this simulation.

Before an equivalent full-area trial, unify the fixed-sweep workflow and its
map settings between simulation and flight, then validate a short hardware run.
The simulator does not validate Flow drift, radio latency, actuator response,
flight endurance or landing. Check upward/downward ranges on the real aircraft:
a previous physical run ended with `vertical_envelope` and an upward reading of
2 mm. Keep the existing flight protections while investigating that reading.
