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
        self.app_id = app_id
        self.is_initialized = False
        self.has_sent_blueprint = False

        # Color mapping for robot models (RGBA)
        self.model_colors = {
            "G1":  [0,   150, 255, 255],   # Blue
            "Go1": [255, 150,   0, 255],   # Orange
        }

    def initialize(self, save_to_file=False):
        """Initialize Rerun logging session."""
        if not self.is_initialized:
            if save_to_file:
                rr.init(self.app_id)
                rr.save("rerun.rrd")
            else:
                rr.init(self.app_id, spawn=True)
            self._send_blueprint()
            self.is_initialized = True

    def _send_blueprint(self):
        if self.has_sent_blueprint:
            return

        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.MapView(
                    origin="world",
                    name="Map & Robots",
                    zoom=15.0,
                    background=rrb.MapProvider.OpenStreetMap,
                    contents=["+ world/**"],
                ),
                rrb.Horizontal(
                    rrb.TimeSeriesView(
                        name="Battery Levels",
                        origin="/robots/battery",
                    ),
                    rrb.TimeSeriesView(
                        name="Humans Detected",
                        origin="/robots/detections/humans",
                    ),
                    rrb.TimeSeriesView(
                        name="Robots Detected",
                        origin="/robots/detections/robots",
                    ),
                    rrb.TextLogView(
                        name="Logs",
                        contents=["+ /logs/**", "+ /collisions/**"],
                    ),
                ),
                row_shares=[3, 1],
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
        """Log OSM walkable ways as geo line strings (logged once, static)."""
        if not self.is_initialized:
            self.initialize()

        # Batch all ways into a single GeoLineStrings call for efficiency
        strips = []
        for way in walkable_ways:
            nodes = way.get('nodes', [])
            if len(nodes) >= 2:
                # nodes are already (lat, lon) tuples
                strips.append(np.array(nodes, dtype=np.float64))

        if strips:
            rr.log(
                "world/map",
                rr.GeoLineStrings(
                    lat_lon=strips,
                    colors=[120, 120, 120, 180],   # semi-transparent gray
                    radii=rr.Radius.ui_points(1.5),
                ),
                static=True,
            )

    def log_robots(self, robot_states: List[Dict[str, Any]], timestamp: datetime = None):
        """Log robot positions and battery levels."""
        if not self.is_initialized:
            self.initialize()

        if timestamp is not None:
            rr.set_time("sim_time", duration=timestamp.timestamp())

        lat_lons = []
        colors = []

        for state in robot_states:
            lat_lons.append([state['latitude'], state['longitude']])
            colors.append(self.model_colors.get(state['model'], [128, 128, 128, 255]))

        if lat_lons:
            rr.log(
                "world/robots",
                rr.GeoPoints(
                    lat_lon=np.array(lat_lons, dtype=np.float64),
                    colors=np.array(colors, dtype=np.uint8),
                    radii=rr.Radius.ui_points(8.0),
                ),
            )

        # Log each robot's battery on its own path so the time-series shows per-robot lines
        for state in robot_states:
            short_id = state['serial_number'][:8]
            rr.log(
                f"robots/battery/{state['model']}_{short_id}",
                rr.Scalars(state['battery']),
            )

    def log_pedestrians(self, pedestrian_states: List[Dict[str, Any]], timestamp: datetime = None):
        """Log pedestrian positions as green GeoPoints."""
        if not self.is_initialized:
            self.initialize()

        if timestamp is not None:
            rr.set_time("sim_time", duration=timestamp.timestamp())

        if not pedestrian_states:
            return

        lat_lons = np.array(
            [[s['latitude'], s['longitude']] for s in pedestrian_states],
            dtype=np.float64,
        )
        rr.log(
            "world/pedestrians",
            rr.GeoPoints(
                lat_lon=lat_lons,
                colors=np.array([[60, 200, 80, 220]] * len(pedestrian_states), dtype=np.uint8),
                radii=rr.Radius.ui_points(5.0),
            ),
        )

    def log_detections(self, detections: List[Dict[str, Any]], timestamp: datetime = None):
        """Log per-robot cone detection counts (humans and robots seen)."""
        if not self.is_initialized:
            self.initialize()
        if timestamp is not None:
            rr.set_time("sim_time", duration=timestamp.timestamp())

        for d in detections:
            short_id = d['serial_number'][:8]
            label = f"{d['model']}_{short_id}"
            rr.log(f"robots/detections/humans/{label}", rr.Scalars(float(d['total_humans'])))
            rr.log(f"robots/detections/robots/{label}", rr.Scalars(float(d['total_robots'])))

    def log_robot_cones(self, cones: List[Dict[str, Any]], timestamp: datetime = None):
        """Draw each robot's sensor cone outline on the map."""
        if not self.is_initialized:
            self.initialize()
        if timestamp is not None:
            rr.set_time("sim_time", duration=timestamp.timestamp())

        # Model colours with low alpha so cones are visible but not distracting
        cone_colors = {
            "G1":  [0,   180, 255,  50],
            "Go1": [255, 140,   0,  50],
        }
        for cone in cones:
            color = cone_colors.get(cone['model'], [200, 200, 200, 50])
            rr.log(
                f"world/cones/{cone['serial_number'][:8]}",
                rr.GeoLineStrings(
                    lat_lon=[np.array(cone['lat_lons'], dtype=np.float64)],
                    colors=[color],
                    radii=rr.Radius.ui_points(1.0),
                ),
            )

    def log_collisions(self, collisions: List[Tuple[str, str]], timestamp: datetime = None):
        """Log collision events."""
        if not self.is_initialized:
            self.initialize()

        if timestamp is not None:
            rr.set_time("sim_time", duration=timestamp.timestamp())

        for robot_id1, robot_id2 in collisions:
            rr.log(
                "collisions/events",
                rr.TextLog(
                    f"Collision: {robot_id1[:8]} ↔ {robot_id2[:8]}",
                    level=rr.TextLogLevel.WARN,
                ),
            )

    def flush(self):
        pass
