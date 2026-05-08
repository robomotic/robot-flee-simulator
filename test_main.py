#!/usr/bin/env python3
"""
Quick test of the main simulation with a short duration.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from main import main
import argparse

def test_main():
    """Test the main function with a short simulation."""
    # Create a simple argument namespace
    class TestArgs:
        south = 52.20
        west = 0.10
        north = 52.21
        east = 0.12
        g1_count = 2
        go1_count = 2
        duration_hours = 0.01  # 36 seconds
        time_step = 1.0
        start_time = None
    
    # Temporarily replace sys.argv
    original_argv = sys.argv
    sys.argv = ['robosim.py']
    
    try:
        # We'll test by calling the functions directly rather than main()
        # to avoid the argument parsing
        from map import MapDataFetcher
        from robot import RobotManager
        from logger import RerunLogger
        from utils import BoundingBox
        from datetime import datetime, timedelta
        
        print("Testing main simulation components...")
        
        # Fetch map data
        print("1. Fetching map data...")
        map_fetcher = MapDataFetcher(
            south=TestArgs.south,
            west=TestArgs.west,
            north=TestArgs.north,
            east=TestArgs.east
        )
        walkable_ways = map_fetcher.get_walkable_ways()
        print(f"   Found {len(walkable_ways)} walkable ways")
        
        if not walkable_ways:
            print("   ERROR: No walkable ways found")
            return False
        
        # Initialize robot manager
        print("2. Initializing robot manager...")
        robot_manager = RobotManager(
            g1_count=TestArgs.g1_count,
            go1_count=TestArgs.go1_count,
            walkable_ways=walkable_ways,
            bbox=(TestArgs.south, TestArgs.west, TestArgs.north, TestArgs.east)
        )
        print(f"   Created {len(robot_manager.robots)} robots")
        
        # Initialize logger
        print("3. Initializing logger...")
        logger = RerunLogger("test_sim")
        logger.initialize()
        logger.log_map_data(walkable_ways)
        
        # Run short simulation
        print("4. Running short simulation...")
        start_time = datetime.now()
        end_time = start_time + timedelta(hours=TestArgs.duration_hours)
        simulation_seconds = 0
        total_steps = int(TestArgs.duration_hours * 3600 / TestArgs.time_step)
        
        print(f"   Simulating {total_steps} steps ({TestArgs.duration_hours} hours)")
        
        for step in range(min(total_steps, 10)):  # Limit to 10 steps for quick test
            # Update robot positions
            robot_manager.update_positions(TestArgs.time_step)
            
            # Detect collisions
            collisions = robot_manager.detect_collisions()
            
            # Log robot states
            current_time = start_time + timedelta(seconds=simulation_seconds)
            logger.log_robots(robot_manager.get_robots_state(), current_time)
            
            if collisions:
                logger.log_collisions(collisions, current_time)
            
            simulation_seconds += TestArgs.time_step
            
            if step % 5 == 0:
                print(f"   Step {step}/{min(total_steps, 10)}")
        
        logger.flush()
        print("   Simulation completed and data saved")
        
        print("\nAll tests passed!")
        return True
        
    except Exception as e:
        print(f"   ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        sys.argv = original_argv

if __name__ == "__main__":
    success = test_main()
    sys.exit(0 if success else 1)