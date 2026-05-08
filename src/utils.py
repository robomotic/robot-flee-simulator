"""
Utility functions for the Robo Fleet Simulator.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class GeoPoint:
    """Geographic point in latitude/longitude."""
    lat: float
    lon: float
    
    def to_tuple(self) -> Tuple[float, float]:
        return (self.lat, self.lon)

@dataclass
class BoundingBox:
    """Bounding box defined by south, west, north, east coordinates."""
    south: float
    west: float
    north: float
    east: float
    
    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.south, self.west, self.north, self.east)

class GeoConverter:
    """Converts between geographic coordinates and local Cartesian coordinates."""
    
    def __init__(self, reference_point: GeoPoint):
        """
        Initialize converter with a reference point.
        
        Args:
            reference_point: The origin point for local coordinates (lat, lon)
        """
        self.ref_lat = math.radians(reference_point.lat)
        self.ref_lon = math.radians(reference_point.lon)
        # Earth radius in meters
        self.R = 6378137.0
    
    def geo_to_local(self, geo_point: GeoPoint) -> Tuple[float, float]:
        """
        Convert geographic coordinates to local Cartesian coordinates (x, y) in meters.
        Uses equirectangular projection for small areas.
        
        Returns:
            Tuple of (x, y) where x is east, y is north in meters from reference point
        """
        lat_rad = math.radians(geo_point.lat)
        lon_rad = math.radians(geo_point.lon)
        
        # Differences from reference point
        dlat = lat_rad - self.ref_lat
        dlon = lon_rad - self.ref_lon
        
        # Convert to meters
        x = self.R * dlon * math.cos(self.ref_lat)
        y = self.R * dlat
        
        return (x, y)
    
    def local_to_geo(self, x: float, y: float) -> GeoPoint:
        """
        Convert local Cartesian coordinates to geographic coordinates.
        
        Args:
            x: East offset in meters
            y: North offset in meters
            
        Returns:
            GeoPoint with latitude and longitude
        """
        # Convert meters to radians
        dlat = y / self.R
        dlon = x / (self.R * math.cos(self.ref_lat))
        
        lat = math.degrees(self.ref_lat + dlat)
        lon = math.degrees(self.ref_lon + dlon)
        
        return GeoPoint(lat, lon)

def distance_between_points(point1: GeoPoint, point2: GeoPoint) -> float:
    """
    Calculate the great-circle distance between two geographic points.
    
    Args:
        point1: First GeoPoint
        point2: Second GeoPoint
        
    Returns:
        Distance in meters
    """
    # Haversine formula
    lat1, lon1 = math.radians(point1.lat), math.radians(point1.lon)
    lat2, lon2 = math.radians(point2.lat), math.radians(point2.lon)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    # Earth radius in meters
    R = 6371000
    
    return R * c

def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to the range [0, 2*pi).
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in radians
    """
    return angle % (2 * math.pi)