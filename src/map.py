"""
Map data fetcher for the Robo Fleet Simulator.
Handles retrieving walkable ways from OpenStreetMap via Overpass API.
"""

import requests
from typing import List, Tuple, Dict, Any
from utils import GeoPoint, BoundingBox

class MapDataFetcher:
    """Fetches and processes map data from OpenStreetMap."""
    
    # Walkable highway types (based on OpenStreetMap tags)
    WALKABLE_HIGHWAYS = {
        'residential', 'primary', 'secondary', 'tertiary', 'unclassified',
        'service', 'living_street', 'pedestrian', 'footway', 'path',
        'track', 'road'  # Added 'road' as a general type
    }
    
    def __init__(self, south: float, west: float, north: float, east: float):
        """
        Initialize the map fetcher with a bounding box.
        
        Args:
            south: Southern latitude boundary
            west: Western longitude boundary
            north: Northern latitude boundary
            east: Eastern longitude boundary
        """
        self.bbox = BoundingBox(south, west, north, east)
        self.overpass_url = "https://overpass-api.de/api/interpreter"
    
    def build_overpass_query(self) -> str:
        """
        Build an Overpass API query for walkable ways in the bounding box.
        
        Returns:
            Overpass QL query string
        """
        s, w, n, e = self.bbox.to_tuple()
        
        # Build condition for walkable highways using regex match
        highway_pattern = "|".join(sorted(self.WALKABLE_HIGHWAYS))
        
        query = f"""
        [out:json][timeout:25];
        (
          way["highway"~"({highway_pattern})"]({s},{w},{n},{e});
        );
        out geom;
        """
        return query.strip()
    
    def fetch_osm_data(self) -> Dict[str, Any]:
        """
        Fetch data from Overpass API.
        
        Returns:
            JSON response from Overpass API
            
        Raises:
            Exception: If the API request fails
        """
        query = self.build_overpass_query()
        headers = {"User-Agent": "RoboFleetSimulator/1.0"}
        
        try:
            response = requests.post(
                self.overpass_url,
                data={"data": query},
                headers=headers,
                timeout=30
            )
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            raise Exception(f"Failed to fetch OSM data: {e}")
    
    def get_walkable_ways(self) -> List[Dict[str, Any]]:
        """
        Extract and process walkable ways from OSM data.
        
        Returns:
            List of dictionaries representing walkable ways, each with:
            - id: OSM way ID
            - nodes: list of (lat, lon) tuples
            - tags: dictionary of OSM tags
        """
        data = self.fetch_osm_data()
        ways = []
        
        for element in data.get('elements', []):
            if element.get('type') == 'way':
                # Extract coordinates from geometry
                geometry = element.get('geometry', [])
                if not geometry:
                    # Fallback to nodes if geometry not provided
                    nodes = element.get('nodes', [])
                    # We would need to fetch node details separately, skip for now
                    continue
                
                # Convert geometry to list of (lat, lon)
                coords = [(point['lat'], point['lon']) for point in geometry]
                
                # Only include ways with at least 2 points
                if len(coords) >= 2:
                    ways.append({
                        'id': element.get('id'),
                        'nodes': coords,
                        'tags': element.get('tags', {}),
                        'type': 'way'
                    })
        
        return ways

def test_map_fetcher():
    """Test function for the map fetcher."""
    # Example bounding box (London area)
    fetcher = MapDataFetcher(51.50, -0.15, 51.52, -0.10)
    try:
        ways = fetcher.get_walkable_ways()
        print(f"Found {len(ways)} walkable ways")
        for i, way in enumerate(ways[:3]):  # Show first 3 ways
            print(f"Way {i}: ID={way['id']}, points={len(way['nodes'])}")
            if way['nodes']:
                print(f"  First point: {way['nodes'][0]}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_map_fetcher()