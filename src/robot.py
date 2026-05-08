"""
Robot management for the Robo Fleet Simulator.
Handles robot creation, state updates, and patrolling behavior.
"""

import uuid
import random
from typing import List, Dict, Any, Tuple, Optional
from dataclasses import dataclass, field
from utils import GeoPoint, GeoConverter, BoundingBox
import math

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
    
    def __post_init__(self):
        """Initialize robot state after creation."""
        # State will be initialized by RobotManager
        pass

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
        # Create G1 robots
        for _ in range(g1_count):
            robot = Robot()
            robot.model = "G1"
            self.robots.append(robot)
        
        # Create Go1 robots
        for _ in range(go1_count):
            robot = Robot()
            robot.model = "Go1"
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
    
    def update_positions(self, time_step: float):
        """
        Update robot positions based on velocity and time step.
        
        Args:
            time_step: Time step in seconds
        """
        for robot in self.robots:
            state = robot.state
            
            # Skip if battery is depleted
            if state.battery <= 0:
                state.velocity = 0.0
                continue
            
            # Drain battery based on velocity and time
            # Simple model: drain proportional to velocity^2
            battery_drain = (state.velocity ** 2) * 0.01 * time_step  # Adjust factor as needed
            state.battery = max(0.0, state.battery - battery_drain)
            
            # If battery is depleted, stop
            if state.battery <= 0:
                state.velocity = 0.0
                continue
            
            # Calculate distance to move
            distance = state.velocity * time_step
            
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