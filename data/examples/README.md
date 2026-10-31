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

## Latest shared physical flight

- [Physical map](cf2_flight_8710eab2f152_map.json): exact original export.
- [Diagnostic](cf2_flight_8710eab2f152_diagnostic.json): sensor counts and stop reason.
- This flight completed 17/28 targets and returned after the 180-second survey
  budget. Measured observed area: 23.7%. All four lateral directions produced
  some usable distances; none of the samples had four simultaneously usable.
- No estimated XY pose samples crossed the survey rectangle.
- The scheduled route including home approach/return is 41.38 m: at 0.15 m/s
  its ideal lower-bound duration is 275.84 seconds, before controller delays.
  Increasing a budget requires establishing real battery endurance first.
- The map is partial, session-local and not validated for CF1. Lighthouse frame
  alignment remains required. No occupied survey cells does not establish that
  unknown cells are free. Virtual flight boundaries are not measured walls.

`data/examples/` is tracked by Git. `data/runs/` remains ignored for local
recordings; .gitignore does not delete those files. Team members can switch to
`integration/cf2-navigation-mapping` and pull to obtain both exports. The
simulation above is distinct from the physical map; do not substitute one for
the other during real navigation.
