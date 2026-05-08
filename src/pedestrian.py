"""
Pedestrian simulation for the Robo Fleet Simulator.

Pedestrians enter at map edges, walk through the area (on roads or freely),
and steer away from robots and each other when within avoidance range.
"""

import uuid
import random
import math
from typing import List, Dict, Any, Tuple, Optional
from dataclasses import dataclass, field

from utils import GeoPoint, GeoConverter, BoundingBox

# Avoidance kicks in within this radius (meters)
AVOIDANCE_RADIUS = 5.0
# How strongly the avoidance vector pushes (m/s equivalent)
AVOIDANCE_STRENGTH = 1.5


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference a − b, in (−π, π]."""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d


@dataclass
class PedestrianState:
    id: str
    position: GeoPoint
    local_x: float
    local_y: float
    velocity: float          # m/s
    heading: float           # radians (math convention: 0=east, CCW)
    active: bool = True
    mode: str = "road"       # "road" | "wander"

    # Road-follower state
    current_segment_id: Optional[int] = None
    segment_progress: float = 0.0

    # Wander-mode goal (local coords)
    goal_x: float = 0.0
    goal_y: float = 0.0


class PedestrianManager:
    """Manages a population of pedestrians entering and leaving the map."""

    def __init__(
        self,
        walkable_ways: List[Dict[str, Any]],
        bbox: Tuple[float, float, float, float],   # (south, west, north, east)
        spawn_rate: float = 0.3,                   # new pedestrians per simulated second
        max_pedestrians: int = 30,
    ):
        self.spawn_rate = spawn_rate
        self.max_pedestrians = max_pedestrians
        self._spawn_acc = 0.0
        self.pedestrians: List[PedestrianState] = []

        south, west, north, east = bbox
        center_lat = (south + north) / 2
        center_lon = (west + east) / 2
        self.geo_converter = GeoConverter(GeoPoint(center_lat, center_lon))

        # Bounding box corners in local coords
        sw = self.geo_converter.geo_to_local(GeoPoint(south, west))
        ne = self.geo_converter.geo_to_local(GeoPoint(north, east))
        self.min_x, self.min_y = sw
        self.max_x, self.max_y = ne

        self.processed_ways = self._process_ways(walkable_ways)

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _process_ways(self, ways: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        out = []
        for way in ways:
            coords = []
            for lat, lon in way['nodes']:
                coords.append(self.geo_converter.geo_to_local(GeoPoint(lat, lon)))
            total = sum(
                math.hypot(coords[i][0] - coords[i-1][0], coords[i][1] - coords[i-1][1])
                for i in range(1, len(coords))
            )
            if total > 0.1:
                out.append({**way, 'local_coords': coords, 'total_length': total})
        return out

    # ------------------------------------------------------------------
    # Spawn / despawn
    # ------------------------------------------------------------------

    def _edge_spawn_point(self) -> Tuple[float, float]:
        """Random point on one of the four map edges."""
        edge = random.randint(0, 3)
        if edge == 0:   # south
            return random.uniform(self.min_x, self.max_x), self.min_y
        elif edge == 1: # north
            return random.uniform(self.min_x, self.max_x), self.max_y
        elif edge == 2: # west
            return self.min_x, random.uniform(self.min_y, self.max_y)
        else:           # east
            return self.max_x, random.uniform(self.min_y, self.max_y)

    def _random_interior(self) -> Tuple[float, float]:
        return (
            random.uniform(self.min_x, self.max_x),
            random.uniform(self.min_y, self.max_y),
        )

    def _spawn(self):
        sx, sy = self._edge_spawn_point()
        velocity = random.uniform(0.8, 1.5)

        # 70 % road follower, 30 % free wanderer
        if self.processed_ways and random.random() < 0.7:
            mode = "road"
            way = random.choice(self.processed_ways)
            seg_id = way['id']
            progress = random.random()
            nx, ny = self._interpolate(way, progress)
            heading = math.atan2(ny - sy, nx - sx)
            gx, gy = 0.0, 0.0
        else:
            mode = "wander"
            seg_id = None
            progress = 0.0
            gx, gy = self._random_interior()
            heading = math.atan2(gy - sy, gx - sx)

        geo = self.geo_converter.local_to_geo(sx, sy)
        self.pedestrians.append(PedestrianState(
            id=str(uuid.uuid4()),
            position=geo,
            local_x=sx,
            local_y=sy,
            velocity=velocity,
            heading=heading,
            mode=mode,
            current_segment_id=seg_id,
            segment_progress=progress,
            goal_x=gx,
            goal_y=gy,
        ))

    def _outside(self, x: float, y: float) -> bool:
        margin = 30.0
        return (x < self.min_x - margin or x > self.max_x + margin or
                y < self.min_y - margin or y > self.max_y + margin)

    # ------------------------------------------------------------------
    # Movement helpers
    # ------------------------------------------------------------------

    def _interpolate(self, way: Dict[str, Any], progress: float) -> Tuple[float, float]:
        coords = way['local_coords']
        total = way['total_length']
        target = max(0.0, min(1.0, progress)) * total
        dist = 0.0
        for i in range(1, len(coords)):
            x1, y1 = coords[i - 1]
            x2, y2 = coords[i]
            seg = math.hypot(x2 - x1, y2 - y1)
            if dist + seg >= target:
                t = (target - dist) / seg if seg > 0 else 0.0
                return x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            dist += seg
        return coords[-1]

    def _avoidance_heading_offset(
        self,
        x: float, y: float,
        base_heading: float,
        nearby: List[Tuple[float, float]],
    ) -> float:
        """Return a heading offset (radians) to steer away from nearby agents."""
        ax, ay = 0.0, 0.0
        for ox, oy in nearby:
            dx, dy = x - ox, y - oy
            dist = math.hypot(dx, dy)
            if 0.0 < dist < AVOIDANCE_RADIUS:
                weight = AVOIDANCE_STRENGTH * (1.0 - dist / AVOIDANCE_RADIUS)
                ax += (dx / dist) * weight
                ay += (dy / dist) * weight
        if ax == 0.0 and ay == 0.0:
            return 0.0
        avoid_dir = math.atan2(ay, ax)
        magnitude = min(1.0, math.hypot(ax, ay) / AVOIDANCE_STRENGTH)
        return magnitude * _angle_diff(avoid_dir, base_heading) * 0.6

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update(
        self,
        time_step: float,
        robot_local_positions: List[Tuple[float, float]],
    ):
        # Spawn
        active_count = sum(1 for p in self.pedestrians if p.active)
        if active_count < self.max_pedestrians:
            self._spawn_acc += self.spawn_rate * time_step
            while self._spawn_acc >= 1.0 and active_count < self.max_pedestrians:
                self._spawn()
                self._spawn_acc -= 1.0
                active_count += 1

        # All active pedestrian positions (for mutual avoidance)
        ped_positions = [(p.local_x, p.local_y) for p in self.pedestrians if p.active]

        for ped in self.pedestrians:
            if not ped.active:
                continue

            prev_x, prev_y = ped.local_x, ped.local_y

            # --- compute base movement ---
            if ped.mode == "road" and ped.current_segment_id is not None:
                way = next(
                    (w for w in self.processed_ways if w['id'] == ped.current_segment_id),
                    None,
                )
                if way:
                    step_progress = ped.velocity * time_step / way['total_length']
                    new_progress = ped.segment_progress + step_progress
                    if new_progress >= 1.0:
                        # Reached end: pick a new random way instead of reversing
                        new_way = random.choice(self.processed_ways)
                        ped.current_segment_id = new_way['id']
                        ped.segment_progress = 0.0
                        nx, ny = self._interpolate(new_way, 0.0)
                    else:
                        ped.segment_progress = new_progress
                        nx, ny = self._interpolate(way, new_progress)
                    base_heading = math.atan2(ny - prev_y, nx - prev_x) if (nx != prev_x or ny != prev_y) else ped.heading
                else:
                    # Lost way — switch to wander
                    ped.mode = "wander"
                    ped.goal_x, ped.goal_y = self._random_interior()
                    nx, ny = prev_x, prev_y
                    base_heading = ped.heading
            else:
                # Wander: head toward goal
                dx, dy = ped.goal_x - prev_x, ped.goal_y - prev_y
                if math.hypot(dx, dy) < 10.0:
                    ped.goal_x, ped.goal_y = self._random_interior()
                    dx, dy = ped.goal_x - prev_x, ped.goal_y - prev_y
                base_heading = math.atan2(dy, dx)
                nx = prev_x + math.cos(base_heading) * ped.velocity * time_step
                ny = prev_y + math.sin(base_heading) * ped.velocity * time_step

            # --- avoidance: deflect heading away from robots and other peds ---
            nearby = robot_local_positions + [
                pos for pos in ped_positions if pos != (prev_x, prev_y)
            ]
            offset = self._avoidance_heading_offset(prev_x, prev_y, base_heading, nearby)
            if offset != 0.0:
                deflected = base_heading + offset
                dist = ped.velocity * time_step
                nx = prev_x + math.cos(deflected) * dist
                ny = prev_y + math.sin(deflected) * dist

            # --- commit ---
            ped.heading = math.atan2(ny - prev_y, nx - prev_x) if (nx != prev_x or ny != prev_y) else ped.heading
            ped.local_x = nx
            ped.local_y = ny
            ped.position = self.geo_converter.local_to_geo(nx, ny)

            if self._outside(nx, ny):
                ped.active = False

        # Prune dead entries when list grows too large
        if len(self.pedestrians) > self.max_pedestrians * 4:
            self.pedestrians = [p for p in self.pedestrians if p.active]

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_local_positions(self) -> List[Tuple[float, float]]:
        return [(p.local_x, p.local_y) for p in self.pedestrians if p.active]

    def get_state(self) -> List[Dict[str, Any]]:
        return [
            {
                'id': p.id,
                'latitude': p.position.lat,
                'longitude': p.position.lon,
                'velocity': p.velocity,
                'mode': p.mode,
            }
            for p in self.pedestrians if p.active
        ]
