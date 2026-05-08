"""
Rerun logger for the Robo Fleet Simulator.
Handles logging simulation data to Rerun for visualization.
"""

import rerun as rr
import rerun.blueprint as rrb
import numpy as np
from typing import List, Dict, Any, Tuple
from datetime import datetime

class RerunLogger:
    """Logs simulation data to Rerun for visualization."""
    
    def __init__(self, app_id: str = "robo_fleet_simulator"):
        """
        Initialize the Rerun logger.
        
        Args:
            app_id: Application ID for Rerun session
        """
        self.app_id = app_id
        self.is_initialized = False
        self.has_sent_blueprint = False
        
        # Color mapping for robot models
        self.model_colors = {
            "G1": [0, 150, 255],    # Blue
            "Go1": [255, 150, 0]    # Orange
        }
    
    def initialize(self, save_to_file=False):
        """Initialize Rerun logging session."""
        if not self.is_initialized:
            if save_to_file:
                rr.init(self.app_id)
                rr.save("rerun.dat")
            else:
                rr.init(self.app_id, spawn=True)
                self._send_blueprint()
            self.is_initialized = True
    
    def _send_blueprint(self):
        """Send a blueprint for proper visualization."""
        if self.has_sent_blueprint:
            return
            
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial2DView(
                    name="Map & Robots",
                    origin="/",
                    contents=[
                        "+ /map/**",
                        "+ /robots/**",
                        "+ /collisions/**"
                    ],
                    spatial_information=rrb.SpatialInformation(
                        show_origin=True,
                        show_bounds=False,
                    ),
                ),
                rrb.TextLogView(
                    name="Logs",
                    contents=[
                        "+ /logs/**"
                    ]
                ),
                rrb.TimeSeriesView(
                    name="Battery Levels",
                    origin="/robots/battery"
                ),
                rrb.TimeSeriesView(
                    name="Velocity",
                    origin="/robots/velocity_color"
                )
            ),
            rrb.TimePanel(
                state=rrb.components.PanelState.Collapsed,
                play_state=rrb.components.PlayState.Playing,
                loop_mode=rrb.components.LoopMode.All,
            ),
        )
        rr.send_blueprint(blueprint)
        self.has_sent_blueprint = True
    
    def _send_blueprint(self):
        """Send a blueprint for proper visualization."""
        if self.has_sent_blueprint:
            return
            
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial2DView(
                    name="Map & Robots",
                    origin="/",
                    contents=[
                        "+ /map/**",
                        "+ /robots/**",
                        "+ /collisions/**"
                    ],
                    spatial_information=rrb.SpatialInformation(
                        show_origin=True,
                        show_bounds=False,
                    ),
                ),
                rrb.TextLogView(
                    name="Logs",
                    contents=[
                        "+ /logs/**"
                    ]
                ),
                rrb.TimeSeriesView(
                    name="Battery Levels",
                    origin="/robots/battery"
                ),
                rrb.TimeSeriesView(
                    name="Velocity",
                    origin="/robots/velocity_color"
                )
            ),
            rrb.TimePanel(
                state=rrb.components.PanelState.Collapsed,
                play_state=rrb.components.PlayState.Playing,
                loop_mode=rrb.components.LoopMode.All,
            ),
        )
        rr.send_blueprint(blueprint)
        self.has_sent_blueprint = True
    
    def log_map_data(self, walkable_ways: List[Dict[str, Any]]):
        """
        Log map data (walkable ways) to Rerun.
        
        Args:
            walkable_ways: List of walkable ways from map data
        """
        if not self.is_initialized:
            self.initialize()
        
        # Log each way as a line strip
        for i, way in enumerate(walkable_ways):
            if 'nodes' in way and len(way['nodes']) >= 2:
                # Convert to numpy array for Rerun
                points = np.array([[node[0], node[1]] for node in way['nodes']])
                
                # Log as line strip
                rr.log(
                    f"map/way_{i}",
                    rr.LineStrips2D(
                        [points],  # Need to pass as list of strips
                        colors=[100, 100, 100],  # Gray
                        radii=2.0  # Use radii instead of widths
                    )
                )
        
        # Log map background (optional)
        rr.log("map/background", rr.TextLog("Map data loaded from OpenStreetMap", level=rr.TextLogLevel.INFO))
    
    def log_robots(self, robot_states: List[Dict[str, Any]], timestamp: datetime = None):
        """
        Log robot states to Rerun.
        
        Args:
            robot_states: List of robot state dictionaries
            timestamp: Simulation timestamp (defaults to now)
        """
        if not self.is_initialized:
            self.initialize()
        
        # Set time if provided
        if timestamp is not None:
            # Convert to duration since start (we'll use the timestamp as duration for simplicity)
            # In a real app, you'd subtract start time, but for now we'll use the timestamp directly
            rr.set_time("sim_time", duration=timestamp.timestamp())
        
        # Prepare data for batch logging
        positions = []
        colors = []
        radii = []
        velocities = []
        battery_levels = []
        
        for state in robot_states:
            # Position (latitude, longitude) - we'll use these directly for now
            positions.append([state['longitude'], state['latitude']])  # Note: lon, lat order for x,y
            
            # Color by model
            model = state['model']
            color = self.model_colors.get(model, [128, 128, 128])  # Default gray
            colors.append(color)
            
            # Size based on bounding box (use average of width and height)
            bbox = state['bounding_box']
            radius = max(bbox['width'], bbox['height']) / 2
            radii.append(radius)
            
            # Velocity magnitude
            velocities.append(state['velocity'])
            
            # Battery level
            battery_levels.append(state['battery'])
        
        if positions:  # Only log if we have robots
            positions_np = np.array(positions)
            colors_np = np.array(colors, dtype=np.uint8)
            radii_np = np.array(radii)
            velocities_np = np.array(velocities)
            battery_np = np.array(battery_levels)
            
            # Log robot positions as points
            rr.log(
                "robots/positions",
                rr.Points2D(
                    positions_np,
                    colors=colors_np,
                    radii=radii_np
                )
            )
            
            # Log velocity as colored points (speed represented by color intensity)
            if len(positions) > 0:
                # Normalize velocities for color representation (0-1 range)
                max_vel = max(velocities_np) if len(velocities_np) > 0 else 1.0
                if max_vel > 0:
                    vel_normalized = velocities_np / max_vel
                    # Blue color intensity based on velocity (0=dark blue, 1=bright blue)
                    vel_colors = np.zeros((len(positions_np), 3), dtype=np.uint8)
                    vel_colors[:, 2] = (vel_normalized * 255).astype(np.uint8)  # Blue channel
                else:
                    vel_colors = np.zeros((len(positions_np), 3), dtype=np.uint8)
                    vel_colors[:, 2] = 128  # Medium blue if no velocity
                
                # Log velocity colors
                rr.log(
                    "robots/velocity_color",
                    rr.Points2D(
                        positions_np,
                        colors=vel_colors,
                        radii=radii_np
                    )
                )
            
            # Log battery levels as scalar values
            rr.log(
                "robots/battery",
                rr.Scalars(battery_np)
            )
    
    def log_collisions(self, collisions: List[Tuple[str, str]], timestamp: datetime = None):
        """
        Log collision events to Rerun.
        
        Args:
            collisions: List of tuples (robot_id1, robot_id2) representing colliding pairs
            timestamp: Simulation timestamp (defaults to now)
        """
        if not self.is_initialized:
            self.initialize()
        
        # Set time if provided
        if timestamp is not None:
            rr.set_time("sim_time", duration=timestamp.timestamp())
        
        if collisions:
            # Log each collision as a text log
            for i, (robot_id1, robot_id2) in enumerate(collisions):
                rr.log(
                    f"collisions/event_{i}",
                    rr.TextLog(
                        f"Collision between robots {robot_id1[:8]} and {robot_id2[:8]}",
                        level=rr.TextLogLevel.WARN
                    )
                )
            
            # Also log as a single warning with count
            rr.log(
                "collisions/summary",
                rr.TextLog(
                    f"Detected {len(collisions)} collision(s)",
                    level=rr.TextLogLevel.WARN
                )
            )
    
    def flush(self):
        """Flush and save Rerun data."""
        if self.is_initialized:
            # Data is already being saved to file by the initial rr.save call
            pass

def test_logger():
    """Test function for the logger."""
    logger = RerunLogger("test_logger")
    logger.initialize()
    
    # Test logging some dummy data
    dummy_states = [
        {
            'id': 'test1',
            'vendor': 'Unitree',
            'model': 'G1',
            'serial_number': 'SN001',
            'latitude': 51.50,
            'longitude': -0.10,
            'velocity': 1.5,
            'heading': 0.0,
            'battery': 85.0,
            'bounding_box': {
                'x': 0.0,
                'y': 0.0,
                'width': 0.3,
                'height': 0.5
            }
        }
    ]
    
    logger.log_map_data([])  # Empty ways for test
    logger.log_robots(dummy_states)
    logger.flush()
    print("Logger test completed")

if __name__ == "__main__":
    test_logger()