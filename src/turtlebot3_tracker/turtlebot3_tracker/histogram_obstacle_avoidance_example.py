#!/usr/bin/env python3
"""
Example script demonstrating how to use the HistogramWindowObstacleAvoidance class.
This script shows how to create an instance of the class and use it to process
laser scan data and get steering commands or updated waypoints.
"""

import numpy as np
import math
from typing import List, Tuple

# Import the HistogramWindowObstacleAvoidance class
from trajectory_tracker_with_obstacle_avoidance_node import HistogramWindowObstacleAvoidance

def create_sample_scan_data(num_points: int = 360, range_max: float = 3.5) -> List[float]:
    """
    Create sample laser scan data for testing.
    
    Args:
        num_points: Number of scan points
        range_max: Maximum range value
        
    Returns:
        List of distance measurements
    """
    # Create a simple obstacle pattern
    scan_data = []
    for i in range(num_points):
        angle = i * 2 * math.pi / num_points
        
        # Create some obstacles at specific angles
        if 0.5 < angle < 1.0:  # Obstacle in front-right
            distance = 0.8
        elif 2.0 < angle < 2.5:  # Obstacle in front-left
            distance = 1.2
        elif 4.0 < angle < 4.5:  # Obstacle behind
            distance = 1.5
        else:
            distance = range_max
            
        scan_data.append(distance)
    
    return scan_data

def main():
    """
    Main function demonstrating the usage of HistogramWindowObstacleAvoidance class.
    """
    print("HistogramWindowObstacleAvoidance Example")
    print("=" * 50)
    
    # Create an instance of the obstacle avoidance class
    obstacle_avoidance = HistogramWindowObstacleAvoidance(
        robot_radius=0.2,
        safety_margin=0.3,
        num_sectors=36,
        scan_angle=360,
        range_max=3.5
    )
    
    # Create sample laser scan data
    scan_data = create_sample_scan_data()
    print(f"Created sample scan data with {len(scan_data)} points")
    
    # Example 1: Get steering command for a target direction
    print("\nExample 1: Getting steering command")
    print("-" * 30)
    
    target_theta = math.pi / 4  # 45 degrees # considering target theta in base_link frame
    print(f"Target direction: {np.degrees(target_theta):.1f} degrees")
    
    steering_idx, steering_angle = obstacle_avoidance.get_steering_command(scan_data, target_theta)
    print(f"Best steering index: {steering_idx}")
    print(f"Best steering angle: {np.degrees(steering_angle):.1f} degrees")
    
    # Example 2: Get updated waypoint
    print("\nExample 2: Getting updated waypoint")
    print("-" * 30)
    
    # Current robot pose (x, y, theta)
    current_pose = (0.0, 0.0, 0.0)  # At origin, facing positive x-axis
    
    # Target waypoint
    target_waypoint = (2.0, 1.0)  # 2 meters forward, 1 meter to the right
    
    print(f"Current pose: x={current_pose[0]:.1f}, y={current_pose[1]:.1f}, theta={np.degrees(current_pose[2]):.1f}°")
    print(f"Target waypoint: x={target_waypoint[0]:.1f}, y={target_waypoint[1]:.1f}")
    
    updated_waypoint = obstacle_avoidance.get_updated_waypoint(scan_data, current_pose, target_waypoint)
    print(f"Updated waypoint: x={updated_waypoint[0]:.1f}, y={updated_waypoint[1]:.1f}")
    
    # Example 3: Show histogram information
    print("\nExample 3: Histogram analysis")
    print("-" * 30)
    
    # Update occupancy grid and get histogram
    obstacle_avoidance.update_occupancy_grid(scan_data)
    histogram = obstacle_avoidance.create_polar_histogram()
    
    print(f"Histogram shape: {histogram.shape}")
    print(f"Minimum distance in histogram: {np.min(histogram):.2f} m")
    print(f"Maximum distance in histogram: {np.max(histogram):.2f} m")
    print(f"Effective radius (robot + safety): {obstacle_avoidance.effective_radius:.2f} m")
    
    # Count valid sectors
    valid_sectors = histogram >= obstacle_avoidance.effective_radius
    num_valid_sectors = np.sum(valid_sectors)
    print(f"Number of valid sectors: {num_valid_sectors}/{len(histogram)}")
    
    print("\nExample completed successfully!")

if __name__ == "__main__":
    main() 