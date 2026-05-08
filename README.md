# Robo Fleet Simulator

A simulator for fleets of Unitree robots (G1/Go1 models) patrolling within user-defined geographic areas using OpenStreetMap data. The simulator logs robot positions, velocities, and battery levels to Rerun for local visualization.

## Features

- Fetches walkable ways (streets/roads) from OpenStreetMap via Overpass API
- Simulates Unitree robots with unique serial numbers
- Robots patrol assigned road segments by moving back and forth
- Each robot has configurable velocity and battery drain
- Basic collision detection between robots (bounding box overlap)
- Logs simulation data to Rerun for 2D visualization
- Command-line interface for configuring simulation parameters

## Installation

1. Clone this repository
2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv venv OR uv venv
   source .venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   uv pip install -r requirements.txt
   ```

## Dependencies

- requests
- numpy
- rerun-sdk
- (Optional for development) pytest

## Usage

Run the simulator with the following command:

```bash
python src/main.py --south <lat> --west <lon> --north <lat> --east <lon> [options]
```

### Required Arguments

- `--south`: Southern latitude boundary of the area of interest
- `--west`: Western longitude boundary of the area of interest
- `--north`: Northern latitude boundary of the area of interest
- `--east`: Eastern longitude boundary of the area of interest

### Optional Arguments

- `--g1-count`: Number of G1 robots (default: 5)
- `--go1-count`: Number of Go1 robots (default: 5)
- `--duration-hours`: Simulation duration in hours (default: 24.0)
- `--time-step`: Simulation time step in seconds (default: 1.0)
- `--start-time`: Start time in YYYY-MM-DD HH:MM format (default: current time)

### Example

```bash
# Simulate 4 robots in a small area of Cambridge, UK for 10 minutes
python src/main.py --south 52.20 --west 0.10 --north 52.21 --east 0.12 \
                   --g1-count 2 --go1-count 2 --duration-hours 0.1667
```

```bash
# Simulate 4 robots in a small area of Cambridge, UK for 1 hour
uv run src/main.py --south 52.20 --west 0.10 --north 52.21 --east 0.12 --g1-count 2 --go1-count 2 --duration-hours 1
```

## Visualization

After running the simulator, view the results with Rerun:

```bash
rerun rerun.dat
```

This will open a web interface showing:
- Robot positions as colored points (blue for G1, orange for Go1)
- Robot size proportional to bounding box
- Velocity represented by color intensity (brighter blue = faster)
- Battery levels as scalar values
- Road segments as gray lines
- Collision events as warning logs

## Output

The simulator saves simulation data to `rerun.dat` in the current directory. This file can be loaded into the Rerun viewer for visualization and analysis.

## Implementation Details

### Robot Model

Each robot has:
- Vendor: Unitree (fixed)
- Model: Either G1 or Go1
- Unique serial number (UUID)
- Bounding box for collision detection
- Current position (latitude/longitude)
- Velocity (m/s)
- Heading (radians, 0 = north)
- Battery level (percentage)
- Assigned road segment from OpenStreetMap
- Patrol direction (+1 for forward, -1 for backward)

### Simulation Logic

1. **Map Data**: Fetches walkable ways from OpenStreetMap using Overpass API
2. **Robot Initialization**: 
   - Creates specified number of G1 and Go1 robots
   - Assigns each robot to a random road segment
   - Places robot at random position along that segment
   - Assigns random initial velocity (0.5-2.0 m/s) and battery (20-100%)
3. **Simulation Loop**:
   - Updates robot positions based on velocity and time step
   - Implements back-and-forth patrolling on assigned road segments
   - Drains battery based on velocity (simple quadratic model)
   - Detects collisions between robots (AABB overlap)
   - Logs state to Rerun at each time step
4. **Termination**: Runs for specified duration or until interrupted

## Limitations and Future Work

### Current Limitations

1. **Simple Battery Model**: Battery drain is proportional to velocity², which is a rough approximation
2. **Basic Collision Detection**: Only detects overlaps, doesn't implement collision avoidance
3. **Road Following**: Robots stay on assigned segments and don't navigate intersections
4. **Coordinate System**: Uses equirectangular projection which has distortion over large areas
5. **Static Map**: Map data is fetched once at start and doesn't update during simulation

### Possible Improvements

1. **Advanced Battery Model**: Consider acceleration, terrain, and payload effects
2. **Collision Avoidance**: Implement steering behaviors to avoid collisions
3. **Intersection Navigation**: Allow robots to navigate complex road networks
4. **Dynamic Re-routing**: Change assigned segments based on traffic or obstacles
5. **Human Simulation**: Add human agents as specified in original requirements
6. **More Realistic Patrolling**: Vary patrol patterns (random waypoints, timed intervals)
7. **3D Visualization**: Use Rerun's 3D capabilities for terrain and building data

## Contributing

Feel free to submit issues or pull requests to improve the simulator.

## License

[Specify license if desired]

## Acknowledgments

- OpenStreetMap contributors for map data
- Rerun team for visualization framework
- Unitree for robot designs
