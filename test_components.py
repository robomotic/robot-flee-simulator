#!/usr/bin/env python3
"""
Test script for the Robo Fleet Simulator components.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from map import MapDataFetcher
from robot import RobotManager
from logger import RerunLogger
from utils import GeoPoint

def test_components():
    """Test each component individually."""
    print("Testing Robo Fleet Simulator components...")
    
    # Test 1: Map data fetcher (using a small area to keep response quick)
    print("\n1. Testing MapDataFetcher...")
    try:
        # Small bounding box around Cambridge, UK
        fetcher = MapDataFetcher(52.20, 0.10, 52.21, 0.12)
        ways = fetcher.get_walkable_ways()
        print(f"   Found {len(ways)} walkable ways")
        if ways:
            print(f"   First way: ID={ways[0]['id']}, {len(ways[0]['nodes'])} points")
    except Exception as e:
        print(f"   Error in MapDataFetcher: {e}")
        return False
    
    # Test 2: Robot manager (using mock ways if needed)
    print("\n2. Testing RobotManager...")
    try:
        # Use the ways we fetched, or create mock data if none found
        if len(ways) == 0:
            print("   No real ways found, using mock data for robot test")
            # Create minimal mock way data
            mock_ways = [{
                'id': 12345,
                'nodes': [(52.205, 0.11), (52.206, 0.11), (52.206, 0.115)],
                'tags': {'highway': 'residential'}
            }]
            test_ways = mock_ways
        else:
            test_ways = ways[:3]  # Use first 3 ways for testing
        
        # Create robot manager
        manager = RobotManager(
            g1_count=2,
            go1_count=2,
            walkable_ways=test_ways,
            bbox=(52.20, 0.10, 52.21, 0.12)
        )
        
        print(f"   Created {len(manager.robots)} robots")
        print(f"   G1 robots: {sum(1 for r in manager.robots if r.model == 'G1')}")
        print(f"   Go1 robots: {sum(1 for r in manager.robots if r.model == 'Go1')}")
        
        # Test updating positions
        manager.update_positions(1.0)  # 1 second time step
        print("   Updated robot positions")
        
        # Test collision detection
        collisions = manager.detect_collisions()
        print(f"   Detected {len(collisions)} collisions")
        
        # Get robot states
        states = manager.get_robots_state()
        print(f"   Retrieved states for {len(states)} robots")
        if states:
            sample = states[0]
            print(f"   Sample robot: {sample['model']} {sample['serial_number'][:8]}...")
            print(f"     Position: ({sample['latitude']:.6f}, {sample['longitude']:.6f})")
            print(f"     Velocity: {sample['velocity']:.2f} m/s")
            print(f"     Battery: {sample['battery']:.1f}%")
    
    except Exception as e:
        print(f"   Error in RobotManager: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test 3: Rerun logger
    print("\n3. Testing RerunLogger...")
    try:
        logger = RerunLogger("test_sim")
        logger.initialize()
        
        # Log some mock map data
        logger.log_map_data(test_ways[:1] if test_ways else [])
        
        # Log robot states
        states = manager.get_robots_state() if 'manager' in locals() else []
        logger.log_robots(states)
        
        # Log mock collisions
        if states and len(states) >= 2:
            mock_collisions = [(states[0]['id'], states[1]['id'])]
            logger.log_collisions(mock_collisions)
        
        logger.flush()
        print("   Logger test completed successfully")
    
    except Exception as e:
        print(f"   Error in RerunLogger: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print("\nAll component tests passed!")
    return True

if __name__ == "__main__":
    success = test_components()
    sys.exit(0 if success else 1)