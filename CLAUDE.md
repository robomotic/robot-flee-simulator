# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Install dependencies
pip install -r requirements.txt

# Run the simulator (required: bounding box coordinates)
python src/main.py --south 52.20 --west 0.10 --north 52.21 --east 0.12 \
                   --g1-count 2 --go1-count 2 --duration-hours 0.1667

# Save output to file instead of spawning live viewer
python src/main.py --south 52.20 --west 0.10 --north 52.21 --east 0.12 --save-to-file

# View recorded simulation
rerun rerun.dat

# Run component tests (hits live Overpass API)
python test_components.py

# Run integration test (short simulation against live OSM data)
python test_main.py
```

There is no test runner or lint setup; tests are run directly as scripts.

## Architecture

All source modules live under `src/` and are imported without a package prefix (the test scripts add `src/` to `sys.path`).

**Data flow:**

```
MapDataFetcher (map.py)
  → queries Overpass API for walkable OSM ways
  → returns list of {id, nodes: [(lat,lon)], tags}

RobotManager (robot.py)
  → converts OSM node coords to local Cartesian via GeoConverter
  → pre-processes ways into {local_coords, total_length}
  → creates Robot/RobotState instances placed on random way segments
  → update_positions(): advances each robot along its assigned segment,
    reverses at endpoints (back-and-forth patrol), drains battery
  → detect_collisions(): O(n²) AABB overlap check

RerunLogger (logger.py)
  → wraps rerun-sdk; two modes:
    - save_to_file=True  → rr.save("rerun.dat")  (offline)
    - save_to_file=False → rr.init(..., spawn=True) (live viewer)
  → logs map ways as LineStrips2D, robots as Points2D, battery as Scalars

main.py
  → CLI entry point; orchestrates the three components in a loop
```

**Coordinate system:** `GeoConverter` (utils.py) uses equirectangular projection centered on the bounding-box midpoint. Local `(x, y)` are meters east/north from that origin. This is only accurate for small areas (a few km).

**Robot model:** each `Robot` holds a `RobotState` dataclass. `segment_progress` (0–1) tracks position along the assigned OSM way; patrol direction flips at 0.0 and 1.0. Battery drain is `velocity² × 0.01 × dt`; robots stop when battery reaches 0.

**Rerun entity paths:**
- `/map/way_<i>` — road segments
- `/robots/positions` — all robot positions (batched)
- `/robots/battery` — battery scalars
- `/robots/velocity_color` — velocity-encoded color points
- `/collisions/event_<i>` — per-collision text logs
