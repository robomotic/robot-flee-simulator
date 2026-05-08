#!/usr/bin/env python3
"""
Robo Fleet Simulator - Main Entry Point

Simulates a fleet of Unitree robots (G1/Go1 models) patrolling within a 
user-defined geographic area using OpenStreetMap data.
"""

import argparse
import sys
import time
from datetime import datetime, timedelta

# Local imports
from map import MapDataFetcher
from robot import RobotManager
from pedestrian import PedestrianManager
from logger import RerunLogger
from utils import BoundingBox, GeoPoint

import rerun as rr

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Robo Fleet Simulator')
    
    # Bounding box arguments
    parser.add_argument('--south', type=float, required=True,
                        help='Southern latitude boundary')
    parser.add_argument('--west', type=float, required=True,
                        help='Western longitude boundary')
    parser.add_argument('--north', type=float, required=True,
                        help='Northern latitude boundary')
    parser.add_argument('--east', type=float, required=True,
                        help='Eastern longitude boundary')
    
    # Robot count arguments
    parser.add_argument('--g1-count', type=int, default=5,
                        help='Number of G1 robots (default: 5)')
    parser.add_argument('--go1-count', type=int, default=5,
                        help='Number of Go1 robots (default: 5)')
    
    # Simulation arguments
    parser.add_argument('--duration-hours', type=float, default=24.0,
                        help='Simulation duration in hours (default: 24)')
    parser.add_argument('--time-step', type=float, default=1.0,
                        help='Simulation time step in seconds (default: 1.0)')
    parser.add_argument('--start-time', type=str, default=None,
                        help='Start time in YYYY-MM-DD HH:MM format (default: now)')
    parser.add_argument('--save-to-file', action='store_true',
                        help='Save Rerun data to file (default: real-time streaming)')

    # Pedestrian arguments
    parser.add_argument('--max-pedestrians', type=int, default=30,
                        help='Maximum simultaneous pedestrians (default: 30)')
    parser.add_argument('--ped-spawn-rate', type=float, default=0.3,
                        help='Pedestrians spawned per simulated second (default: 0.3)')

    return parser.parse_args()

def main():
    """Main simulation loop."""
    args = parse_arguments()
    
    print("Starting Robo Fleet Simulator...")
    print(f"Bounding box: ({args.south}, {args.west}) to ({args.north}, {args.east})")
    print(f"Robots: {args.g1_count} G1, {args.go1_count} Go1")
    print(f"Pedestrians: max {args.max_pedestrians}, spawn rate {args.ped_spawn_rate}/s")
    print(f"Duration: {args.duration_hours} hours")
    print(f"Time step: {args.time_step} seconds")
    
    # Parse start time
    if args.start_time:
        try:
            start_time = datetime.strptime(args.start_time, '%Y-%m-%d %H:%M')
        except ValueError:
            print("Error: Start time must be in YYYY-MM-DD HH:MM format")
            sys.exit(1)
    else:
        start_time = datetime.now()
    
    end_time = start_time + timedelta(hours=args.duration_hours)
    print(f"Simulation period: {start_time} to {end_time}")
    
    # Initialize components
    print("\nInitializing components...")
    
    # 1. Fetch map data
    print("Fetching map data from OpenStreetMap...")
    map_fetcher = MapDataFetcher(
        south=args.south,
        west=args.west,
        north=args.north,
        east=args.east
    )
    walkable_ways = map_fetcher.get_walkable_ways()
    
    if not walkable_ways:
        print("Error: No walkable ways found in the specified area")
        sys.exit(1)
    
    print(f"Found {len(walkable_ways)} walkable ways")
    
    # 2. Initialize robot manager
    print("Initializing robot manager...")
    robot_manager = RobotManager(
        g1_count=args.g1_count,
        go1_count=args.go1_count,
        walkable_ways=walkable_ways,
        bbox=(args.south, args.west, args.north, args.east)
    )

    # 2b. Initialize pedestrian manager
    print("Initializing pedestrian manager...")
    pedestrian_manager = PedestrianManager(
        walkable_ways=walkable_ways,
        bbox=(args.south, args.west, args.north, args.east),
        spawn_rate=args.ped_spawn_rate,
        max_pedestrians=args.max_pedestrians,
    )
    
    # 3. Initialize logger
    print("Initializing Rerun logger...")
    logger = RerunLogger("robo_fleet_simulator")
    
    # Initialize with file saving if requested, otherwise real-time
    if args.save_to_file:
        logger.initialize(save_to_file=True)  # This will save to file
    else:
        logger.initialize(save_to_file=False)  # Real-time streaming (spawns viewer)
    
    logger.log_map_data(walkable_ways)
    logger.log_robots(robot_manager.get_robots_state())
    
    # 4. Simulation loop
    print("\nStarting simulation...")
    current_time = start_time
    simulation_seconds = 0
    total_steps = int(args.duration_hours * 3600 / args.time_step)
    
    try:
        for step in range(total_steps):
            # Update simulation time
            current_time = start_time + timedelta(seconds=simulation_seconds)
            
            # Update pedestrians first so robots can react to their positions
            pedestrian_manager.update(
                args.time_step,
                robot_local_positions=robot_manager.get_local_positions(),
            )

            # Update robot positions, passing pedestrian locations for avoidance
            robot_manager.update_positions(
                args.time_step,
                pedestrian_local_positions=pedestrian_manager.get_local_positions(),
            )

            # Detect collisions
            collisions = robot_manager.detect_collisions()
            if collisions:
                logger.log_collisions(collisions, current_time)

            # Cone detection: count humans and robots in each robot's FOV
            detections = robot_manager.detect_in_cone(
                pedestrian_local_positions=pedestrian_manager.get_local_positions(),
            )

            # Log states
            logger.log_robots(robot_manager.get_robots_state(), current_time)
            logger.log_pedestrians(pedestrian_manager.get_state(), current_time)
            logger.log_robot_cones(robot_manager.get_detection_cones(), current_time)
            logger.log_detections(detections, current_time)
            
            # Progress reporting
            if step % 100 == 0:
                progress = (step / total_steps) * 100
                print(f"Progress: {progress:.1f}% ({step}/{total_steps} steps)")
            
            simulation_seconds += args.time_step
            
            # Sleep to maintain real-time if needed (commented out for faster simulation)
            # time.sleep(args.time_step)
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"\nSimulation error: {e}")
        raise
    
    finally:
        print("\nSimulation completed")
        logger.flush()
        print("Rerun data saved. To view: rerun --web-viewer rerun.rrd")

if __name__ == "__main__":
    main()