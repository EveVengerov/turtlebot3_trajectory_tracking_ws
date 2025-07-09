#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import math


def smooth_path_spline(waypoints, num_points=100):
    """
    Smooth a path using cubic spline interpolation.
    
    Args:
        waypoints: List of 2D waypoints [(x0, y0), (x1, y1), ..., (xn, yn)]
        num_points: Number of points to generate for the smooth trajectory
    
    Returns:
        smooth_x: Array of x-coordinates for smooth trajectory
        smooth_y: Array of y-coordinates for smooth trajectory
    """
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required for smoothing")
    
    # Extract x and y coordinates
    x_coords = [wp[0] for wp in waypoints]
    y_coords = [wp[1] for wp in waypoints]
    
    # Create parameter t for spline interpolation
    # Use cumulative distance along the path as parameter
    t = [0.0]
    for i in range(1, len(waypoints)):
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        distance = math.sqrt(dx*dx + dy*dy)
        t.append(t[-1] + distance)
    
    # Create cubic splines for x and y coordinates
    cs_x = CubicSpline(t, x_coords, bc_type='natural')
    cs_y = CubicSpline(t, y_coords, bc_type='natural')
    
    # Generate smooth trajectory points
    t_smooth = np.linspace(t[0], t[-1], num_points)
    smooth_x = cs_x(t_smooth)
    smooth_y = cs_y(t_smooth)
    
    return smooth_x, smooth_y


def plot_waypoints_and_smooth_path(waypoints, smooth_x, smooth_y, title="Path Smoothing with Spline Interpolation"):
    """
    Plot original waypoints and smoothed trajectory.
    
    Args:
        waypoints: List of original 2D waypoints
        smooth_x: Array of x-coordinates for smooth trajectory
        smooth_y: Array of y-coordinates for smooth trajectory
        title: Title for the plot
    """
    # Extract waypoint coordinates
    wp_x = [wp[0] for wp in waypoints]
    wp_y = [wp[1] for wp in waypoints]
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    
    # Plot original waypoints
    plt.plot(wp_x, wp_y, 'ro-', linewidth=2, markersize=8, label='Original Waypoints', alpha=0.7)
    
    # Plot smoothed trajectory
    plt.plot(smooth_x, smooth_y, 'b-', linewidth=3, label='Smoothed Trajectory', alpha=0.8)
    
    # Add waypoint numbers
    for i, (x, y) in enumerate(zip(wp_x, wp_y)):
        plt.annotate(f'WP{i}', (x, y), xytext=(5, 5), textcoords='offset points', 
                    fontsize=10, fontweight='bold', color='red')
    
    # Customize the plot
    plt.xlabel('X Coordinate', fontsize=12)
    plt.ylabel('Y Coordinate', fontsize=12)
    plt.title(title, fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Add some padding around the plot
    plt.tight_layout()
    
    return plt


def main():
    """
    Main function demonstrating path smoothing with spline interpolation.
    """
    print('Path Smoothing with Spline Interpolation Demo')
    print('=' * 50)
    
    # Example waypoints - you can modify these or load from file
    waypoints = [
        (0.0, 0.0),      # Start point
        (2.0, 1.0),      # Waypoint 1
        (4.0, 0.5),      # Waypoint 2
        (6.0, 2.0),      # Waypoint 3
        (8.0, 1.5),      # Waypoint 4
        (10.0, 3.0),     # Waypoint 5
        (12.0, 2.0),     # End point
    ]
    
    print(f"Original waypoints: {waypoints}")
    print(f"Number of waypoints: {len(waypoints)}")
    
    try:
        # Generate smooth trajectory using spline interpolation
        smooth_x, smooth_y = smooth_path_spline(waypoints, num_points=200)
        
        print(f"Generated {len(smooth_x)} smooth trajectory points")
        
        # Plot the results
        plt = plot_waypoints_and_smooth_path(waypoints, smooth_x, smooth_y)
        
        # # Save the plot
        # plot_filename = "path_smoothing_result.png"
        # plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        # print(f"Plot saved as: {plot_filename}")
        
        # Show the plot
        plt.show()
        
        # Print some statistics
        print("\nPath Statistics:")
        print(f"Original path length: {calculate_path_length(waypoints):.3f} units")
        print(f"Smoothed path length: {calculate_path_length_from_arrays(smooth_x, smooth_y):.3f} units")
        
    except Exception as e:
        print(f"Error during path smoothing: {e}")


def calculate_path_length(waypoints):
    """Calculate the total length of a path given waypoints."""
    total_length = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i-1][0]
        dy = waypoints[i][1] - waypoints[i-1][1]
        total_length += math.sqrt(dx*dx + dy*dy)
    return total_length


def calculate_path_length_from_arrays(x_coords, y_coords):
    """Calculate the total length of a path given coordinate arrays."""
    total_length = 0.0
    for i in range(1, len(x_coords)):
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        total_length += math.sqrt(dx*dx + dy*dy)
    return total_length


if __name__ == '__main__':
    main()
