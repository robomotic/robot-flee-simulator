"""
Robot management for the Robo Fleet Simulator.
Handles robot creation, state updates, and patrolling behavior.
"""

import uuid
import random
from typing import ClassVar, List, Dict, Any, Tuple, Optional
from dataclasses import dataclass, field
from utils import GeoPoint, GeoConverter, BoundingBox
import math

# Sensor specs sourced from Unitree datasheets:
#   G1  – Depth Camera + 3D LiDAR  → forward 120° sector, 15 m
#   Go1 – Fisheye binocular depth (150×170° per unit) → 150° sector, 10 m
SENSOR_SPECS: Dict[str, Dict[str, float]] = {
    "G1":  {"fov_degrees": 120.0, "range_m": 15.0},
    "Go1": {"fov_degrees": 150.0, "range_m": 10.0},
}


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference a − b mapped to (−π, π]."""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d

@dataclass
class BoundingBox2D:
    """2D bounding box for collision detection."""
    x: float  # center x
    y: float  # center y
    width: float
    height: float
    
    def overlaps(self, other: 'BoundingBox2D') -> bool:
        """Check if this bounding box overlaps with another."""
        return (abs(self.x - other.x) * 2 < (self.width + other.width) and
                abs(self.y - other.y) * 2 < (self.height + other.height))

@dataclass
class RobotState:
    """Current state of a robot."""
    id: str
    vendor: str
    model: str
    serial_number: str
    position: GeoPoint  # geographic position
    velocity: float     # m/s
    heading: float      # radians (0 = north, positive = clockwise)
    battery: float      # percentage (0-100)
    bounding_box: BoundingBox2D  # in local coordinates
    current_segment_id: Optional[int] = None
    segment_progress: float = 0.0  # progress along current segment (0-1)
    patrol_direction: int = 1      # 1 for forward, -1 for backward

@dataclass
class Robot:
    """Robot entity with immutable properties and mutable state."""
    # Immutable properties
    vendor: str = "Unitree"
    model: str = field(default_factory=lambda: random.choice(["G1", "Go1"]))
    serial_number: str = field(default_factory=lambda: str(uuid.uuid4()))

    # Mutable state
    state: RobotState = field(init=False)

    # Physical properties
    length: float = 0.5  # meters
    width: float = 0.3   # meters
    height: float = 0.4  # meters

    # Battery drain rate: % per second at 1 m/s (quadratic in velocity).
    # G1  (~720 Wh battery): 4.0 h at 1 m/s → 100 / (4 * 3600) ≈ 0.00694 %/s
    # Go1 (~154 Wh battery): 3.5 h at 1 m/s → 100 / (3.5 * 3600) ≈ 0.00794 %/s
    BATTERY_DRAIN_RATES: ClassVar[Dict[str, float]] = {
        "G1":  100.0 / (4.0 * 3600),
        "Go1": 100.0 / (3.5 * 3600),
    }
    battery_drain_rate: float = field(init=False)

    def __post_init__(self):
        # battery_drain_rate is set by RobotManager after model is assigned
        self.battery_drain_rate = 100.0 / (4.0 * 3600)

    def update_drain_rate(self):
        self.battery_drain_rate = self.BATTERY_DRAIN_RATES.get(self.model, 100.0 / (4.0 * 3600))

class RobotManager:
    """Manages a fleet of robots, their initialization, and updates."""
    
    def __init__(self, g1_count: int, go1_count: int, 
                 walkable_ways: List[Dict[str, Any]], 
                 bbox: Tuple[float, float, float, float]):
        """
        Initialize the robot manager.
        
        Args:
            g1_count: Number of G1 robots
            go1_count: Number of Go1 robots
            walkable_ways: List of walkable ways from map data
            bbox: Bounding box (south, west, north, east)
        """
        self.robots: List[Robot] = []
        self.walkable_ways = walkable_ways
        self.bbox = BoundingBox(*bbox)
        
        # Create reference point for coordinate conversion (center of bbox)
        center_lat = (bbox[0] + bbox[2]) / 2
        center_lon = (bbox[1] + bbox[3]) / 2
        self.geo_converter = GeoConverter(GeoPoint(center_lat, center_lon))
        
        # Pre-process ways for faster lookup
        self.processed_ways = self._process_ways(walkable_ways)
        
        # Create robots
        self._create_robots(g1_count, go1_count)
        
        # Assign initial positions and states
        self._initialize_robot_states()
    
    def _process_ways(self, ways: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Process ways to add useful precomputed data.
        
        Args:
            ways: Raw ways from map data
            
        Returns:
            Processed ways with additional fields
        """
        processed = []
        for way in ways:
            # Convert geographic coordinates to local coordinates
            local_coords = []
            for lat, lon in way['nodes']:
                geo_point = GeoPoint(lat, lon)
                x, y = self.geo_converter.geo_to_local(geo_point)
                local_coords.append((x, y))
            
            # Calculate total length of the way
            total_length = 0.0
            for i in range(1, len(local_coords)):
                x1, y1 = local_coords[i-1]
                x2, y2 = local_coords[i]
                segment_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                total_length += segment_length
            
            processed.append({
                **way,
                'local_coords': local_coords,
                'total_length': total_length
            })
        
        return processed
    
    def _create_robots(self, g1_count: int, go1_count: int):
        """Create robot instances."""
        for _ in range(g1_count):
            robot = Robot()
            robot.model = "G1"
            robot.update_drain_rate()
            self.robots.append(robot)

        for _ in range(go1_count):
            robot = Robot()
            robot.model = "Go1"
            robot.update_drain_rate()
            self.robots.append(robot)
    
    def _initialize_robot_states(self):
        """Initialize the state for each robot."""
        for robot in self.robots:
            # Assign to a random walkable way
            if not self.processed_ways:
                raise ValueError("No walkable ways available")
            
            way = random.choice(self.processed_ways)
            
            # Start at a random position along the way
            progress = random.random()  # 0 to 1
            
            # Random initial heading (will be overridden by actual direction of travel)
            heading = random.uniform(0, 2 * math.pi)
            
            # Initial velocity (random between 0.5 and 2.0 m/s for walking pace)
            velocity = random.uniform(0.5, 2.0)
            
            # Initial battery (random between 20% and 100%)
            battery = random.uniform(20.0, 100.0)
            
            # Get position at this progress along the way
            local_x, local_y = self._interpolate_position_on_way(way, progress)
            geo_point = self.geo_converter.local_to_geo(local_x, local_y)
            
            # Create bounding box (centered on position)
            bbox = BoundingBox2D(
                x=local_x,
                y=local_y,
                width=robot.width,
                height=robot.length
            )
            
            # Create initial state
            robot.state = RobotState(
                id=str(uuid.uuid4()),
                vendor=robot.vendor,
                model=robot.model,
                serial_number=robot.serial_number,
                position=geo_point,
                velocity=velocity,
                heading=heading,
                battery=battery,
                bounding_box=bbox,
                current_segment_id=way['id'],
                segment_progress=progress,
                patrol_direction=random.choice([-1, 1])  # Random initial direction
            )
    
    def _interpolate_position_on_way(self, way: Dict[str, Any], progress: float) -> Tuple[float, float]:
        """
        Interpolate a position along a way based on progress (0-1).
        
        Args:
            way: Processed way dictionary
            progress: Progress along the way (0 = start, 1 = end)
            
        Returns:
            Tuple of (x, y) local coordinates
        """
        local_coords = way['local_coords']
        total_length = way['total_length']
        
        if total_length <= 0 or len(local_coords) < 2:
            # Fallback to first point
            return local_coords[0] if local_coords else (0.0, 0.0)
        
        # Target distance along the way
        target_distance = progress * total_length
        
        # Find which segment contains this distance
        distance_traveled = 0.0
        for i in range(1, len(local_coords)):
            x1, y1 = local_coords[i-1]
            x2, y2 = local_coords[i]
            
            segment_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            
            if distance_traveled + segment_length >= target_distance:
                # Interpolate within this segment
                segment_progress = (target_distance - distance_traveled) / segment_length
                x = x1 + segment_progress * (x2 - x1)
                y = y1 + segment_progress * (y2 - y1)
                return (x, y)
            
            distance_traveled += segment_length
        
        # If we reach here, return the last point
        return local_coords[-1]
    
    def get_local_positions(self) -> List[Tuple[float, float]]:
        """Return local (x, y) coords of all active robots."""
        return [
            (r.state.bounding_box.x, r.state.bounding_box.y)
            for r in self.robots if r.state.battery > 0
        ]

    def detect_in_cone(
        self,
        pedestrian_local_positions: List[Tuple[float, float]],
    ) -> List[Dict[str, Any]]:
        """
        For each active robot, count how many pedestrians and other robots
        fall inside its forward sensor cone.  Positions are never stored.

        Returns a list of dicts: {id, model, serial_number,
                                   total_humans, total_robots}
        """
        active = [(r, r.state.bounding_box.x, r.state.bounding_box.y)
                  for r in self.robots if r.state.battery > 0]

        results = []
        for robot, rx, ry in active:
            specs = SENSOR_SPECS.get(robot.model, SENSOR_SPECS["G1"])
            half_fov = math.radians(specs["fov_degrees"] / 2)
            max_range = specs["range_m"]
            heading = robot.state.heading

            def _in_cone(ox: float, oy: float) -> bool:
                dist = math.hypot(ox - rx, oy - ry)
                if dist < 0.01 or dist > max_range:
                    return False
                return abs(_angle_diff(math.atan2(oy - ry, ox - rx), heading)) <= half_fov

            total_humans = sum(1 for px, py in pedestrian_local_positions if _in_cone(px, py))
            total_robots = sum(1 for other, ox, oy in active
                               if other.state.id != robot.state.id and _in_cone(ox, oy))

            results.append({
                "id": robot.state.id,
                "model": robot.state.model,
                "serial_number": robot.state.serial_number,
                "total_humans": total_humans,
                "total_robots": total_robots,
            })
        return results

    def get_detection_cones(self) -> List[Dict[str, Any]]:
        """
        Return the outline of each robot's sensor cone as a list of
        (lat, lon) points for map visualisation (apex → arc → apex).
        """
        arc_steps = 14
        cones = []
        for robot in self.robots:
            if robot.state.battery <= 0:
                continue
            specs = SENSOR_SPECS.get(robot.model, SENSOR_SPECS["G1"])
            half_fov = math.radians(specs["fov_degrees"] / 2)
            max_range = specs["range_m"]
            rx, ry = robot.state.bounding_box.x, robot.state.bounding_box.y
            heading = robot.state.heading

            pts = [(rx, ry)]
            for i in range(arc_steps + 1):
                angle = heading - half_fov + i * (2 * half_fov / arc_steps)
                pts.append((rx + math.cos(angle) * max_range,
                             ry + math.sin(angle) * max_range))
            pts.append((rx, ry))  # close

            lat_lons = [self.geo_converter.local_to_geo(x, y) for x, y in pts]
            cones.append({
                "robot_id": robot.state.id,
                "model": robot.state.model,
                "serial_number": robot.state.serial_number,
                "lat_lons": [(g.lat, g.lon) for g in lat_lons],
            })
        return cones

    def update_positions(
        self,
        time_step: float,
        pedestrian_local_positions: List[Tuple[float, float]] = None,
    ):
        """
        Update robot positions. If pedestrian_local_positions is provided,
        robots slow down when pedestrians are within AVOIDANCE_RADIUS metres.
        """
        nearby_threshold_stop = 2.0   # m — stop completely
        nearby_threshold_slow = 5.0   # m — halve speed

        for robot in self.robots:
            state = robot.state

            # Skip if battery is depleted
            if state.battery <= 0:
                state.velocity = 0.0
                continue

            # --- pedestrian avoidance: speed reduction ---
            effective_velocity = state.velocity
            if pedestrian_local_positions:
                rx, ry = state.bounding_box.x, state.bounding_box.y
                min_dist = min(
                    (math.hypot(rx - px, ry - py) for px, py in pedestrian_local_positions),
                    default=float('inf'),
                )
                if min_dist < nearby_threshold_stop:
                    effective_velocity = 0.0
                elif min_dist < nearby_threshold_slow:
                    effective_velocity *= 0.4

            # Quadratic drain: faster walking costs disproportionately more power.
            # Rate is calibrated so 1 m/s continuous walking matches real endurance specs.
            battery_drain = (effective_velocity ** 2) * robot.battery_drain_rate * time_step
            state.battery = max(0.0, state.battery - battery_drain)
            
            # If battery is depleted, stop
            if state.battery <= 0:
                state.velocity = 0.0
                continue

            # Calculate distance to move (uses effective_velocity so avoidance takes effect)
            distance = effective_velocity * time_step
            
            # Get current way
            current_way = None
            for way in self.processed_ways:
                if way['id'] == state.current_segment_id:
                    current_way = way
                    break
            
            if current_way is None:
                # Way no longer available, reassign
                self._reassign_robot(robot)
                current_way = None
                for way in self.processed_ways:
                    if way['id'] == state.current_segment_id:
                        current_way = way
                        break
            
            if current_way is not None:
                # Update progress along the way
                new_progress = state.segment_progress + (state.patrol_direction * distance / current_way['total_length'])
                
                # Handle reaching ends of way
                if new_progress <= 0.0:
                    # Reached start, reverse direction
                    new_progress = 0.0
                    state.patrol_direction = 1
                elif new_progress >= 1.0:
                    # Reached end, reverse direction
                    new_progress = 1.0
                    state.patrol_direction = -1
                
                state.segment_progress = new_progress
                
                # Update position
                local_x, local_y = self._interpolate_position_on_way(current_way, new_progress)
                state.position = self.geo_converter.local_to_geo(local_x, local_y)
                
                # Update bounding box position
                state.bounding_box.x = local_x
                state.bounding_box.y = local_y
                
                # Update heading based on direction of travel
                if distance > 0.001:  # Only update heading if we moved significantly
                    # Calculate heading from previous to current position would be ideal
                    # For now, approximate based on way direction at this point
                    # We'll compute the tangent direction
                    if len(current_way['local_coords']) >= 2:
                        # Find the segment we're currently on
                        local_coords = current_way['local_coords']
                        total_length = current_way['total_length']
                        target_distance = new_progress * total_length
                        
                        distance_traveled = 0.0
                        for i in range(1, len(local_coords)):
                            x1, y1 = local_coords[i-1]
                            x2, y2 = local_coords[i]
                            segment_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                            
                            if distance_traveled + segment_length >= target_distance:
                                # We're on this segment, calculate its angle
                                if segment_length > 0:
                                    angle = math.atan2(y2 - y1, x2 - x1)
                                    # Adjust for patrol direction
                                    if state.patrol_direction == -1:
                                        angle += math.pi  # Reverse direction
                                    state.heading = angle
                                break
                            
                            distance_traveled += segment_length
    
    def _reassign_robot(self, robot: Robot):
        """Reassign a robot to a new random way."""
        if not self.processed_ways:
            return
        
        way = random.choice(self.processed_ways)
        progress = random.random()
        
        local_x, local_y = self._interpolate_position_on_way(way, progress)
        geo_point = self.geo_converter.local_to_geo(local_x, local_y)
        
        # Update state
        state = robot.state
        state.current_segment_id = way['id']
        state.segment_progress = progress
        state.position = geo_point
        state.bounding_box.x = local_x
        state.bounding_box.y = local_y
        # Keep current velocity and heading, but ensure they're reasonable
        if state.velocity <= 0:
            state.velocity = random.uniform(0.5, 2.0)
        state.patrol_direction = random.choice([-1, 1])
    
    def detect_collisions(self) -> List[Tuple[str, str]]:
        """
        Detect collisions between robots.
        
        Returns:
            List of tuples (robot_id1, robot_id2) representing colliding pairs
        """
        collisions = []
        robot_states = [r.state for r in self.robots if r.state.battery > 0]
        
        for i in range(len(robot_states)):
            for j in range(i + 1, len(robot_states)):
                state1 = robot_states[i]
                state2 = robot_states[j]
                
                if state1.bounding_box.overlaps(state2.bounding_box):
                    collisions.append((state1.id, state2.id))
        
        return collisions
    
    def get_robots_state(self) -> List[Dict[str, Any]]:
        """
        Get the current state of all robots for logging.
        
        Returns:
            List of dictionaries representing robot states
        """
        states = []
        for robot in self.robots:
            state = robot.state
            if state.battery > 0:  # Only include active robots
                states.append({
                    'id': state.id,
                    'vendor': state.vendor,
                    'model': state.model,
                    'serial_number': state.serial_number,
                    'latitude': state.position.lat,
                    'longitude': state.position.lon,
                    'velocity': state.velocity,
                    'heading': state.heading,
                    'battery': state.battery,
                    'bounding_box': {
                        'x': state.bounding_box.x,
                        'y': state.bounding_box.y,
                        'width': state.bounding_box.width,
                        'height': state.bounding_box.height
                    },
                    'current_segment_id': state.current_segment_id,
                    'segment_progress': state.segment_progress,
                    'patrol_direction': state.patrol_direction
                })
        return states