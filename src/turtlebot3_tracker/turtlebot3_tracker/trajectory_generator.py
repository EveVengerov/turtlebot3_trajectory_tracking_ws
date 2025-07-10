#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, interp1d
import math
from typing import List, Tuple, Optional, Dict, Any


class TrajectoryGenerator:
    """
    A class for generating smooth trajectories from waypoints or paths.
    
    This class provides methods to:
    1. Smooth paths using cubic spline interpolation
    2. Generate time-parameterized trajectories with different velocity profiles
    3. Calculate trajectory statistics
    4. Visualize trajectories
    """
    
    def __init__(self):
        """Initialize the trajectory generator."""
        self.waypoints = None
        self.smooth_x = None
        self.smooth_y = None
        self.time = None
        self.x_traj = None
        self.y_traj = None
        self.v_traj = None
        self.a_traj = None
        
    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """
        Set the waypoints for trajectory generation.
        
        Args:
            waypoints: List of 2D waypoints [(x0, y0), (x1, y1), ..., (xn, yn)]
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints are required")
        
        self.waypoints = waypoints
        self.get_logger().info(f"Set {len(waypoints)} waypoints")
        
    def smooth_path_spline(self, num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Smooth a path using cubic spline interpolation.
        
        Args:
            num_points: Number of points to generate for the smooth trajectory
        
        Returns:
            smooth_x: Array of x-coordinates for smooth trajectory
            smooth_y: Array of y-coordinates for smooth trajectory
        """
        if self.waypoints is None:
            raise ValueError("Waypoints must be set before smoothing")
        
        if len(self.waypoints) < 2:
            raise ValueError("At least 2 waypoints are required for smoothing")
        
        # Extract x and y coordinates
        x_coords = [wp[0] for wp in self.waypoints]
        y_coords = [wp[1] for wp in self.waypoints]
        
        # Create parameter t for spline interpolation
        # Use cumulative distance along the path as parameter
        t = [0.0]
        for i in range(1, len(self.waypoints)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            distance = math.sqrt(dx*dx + dy*dy)
            t.append(t[-1] + distance)
        
        # Create cubic splines for x and y coordinates
        cs_x = CubicSpline(t, x_coords, bc_type='natural')
        cs_y = CubicSpline(t, y_coords, bc_type='natural')
        
        # Generate smooth trajectory points
        t_smooth = np.linspace(t[0], t[-1], num_points)
        self.smooth_x = cs_x(t_smooth)
        self.smooth_y = cs_y(t_smooth)
        
        return self.smooth_x, self.smooth_y
    
    def generate_time_parameterized_trajectory(self, 
                                             velocity_profile: str = 'trapezoidal',
                                             max_velocity: float = 1.0,
                                             acceleration: float = 0.5,
                                             deceleration: float = 0.5,
                                             dt: float = 0.1) -> Dict[str, np.ndarray]:
        """
        Generate a time-parameterized trajectory from a smoothed path.
        
        Args:
            velocity_profile: Type of velocity profile ('trapezoidal', 'constant', 'sine')
            max_velocity: Maximum velocity (m/s)
            acceleration: Acceleration rate (m/s²)
            deceleration: Deceleration rate (m/s²)
            dt: Time step for trajectory sampling (s)
        
        Returns:
            Dictionary containing trajectory data:
            - 'time': Array of time points
            - 'x_traj': Array of x-coordinates at each time point
            - 'y_traj': Array of y-coordinates at each time point
            - 'v_traj': Array of velocities at each time point
            - 'a_traj': Array of accelerations at each time point
        """
        if self.smooth_x is None or self.smooth_y is None:
            raise ValueError("Path must be smoothed before generating time-parameterized trajectory")
        
        # Calculate path distances
        distances = self._calculate_path_distances(self.smooth_x, self.smooth_y)
        total_distance = distances[-1]
        
        # Generate velocity profile
        if velocity_profile == 'trapezoidal':
            time, v_traj, a_traj = self._generate_trapezoidal_velocity_profile(
                total_distance, max_velocity, acceleration, deceleration, dt)
        elif velocity_profile == 'constant':
            time, v_traj, a_traj = self._generate_constant_velocity_profile(
                total_distance, max_velocity, dt)
        elif velocity_profile == 'sine':
            time, v_traj, a_traj = self._generate_sine_velocity_profile(
                total_distance, max_velocity, dt)
        else:
            raise ValueError(f"Unknown velocity profile: {velocity_profile}")
        
        # Generate position trajectory based on velocity profile
        x_traj, y_traj = self._generate_position_trajectory(self.smooth_x, self.smooth_y, time, v_traj)
        
        # Store trajectory data
        self.time = time
        self.x_traj = x_traj
        self.y_traj = y_traj
        self.v_traj = v_traj
        self.a_traj = a_traj
        
        return {
            'time': time,
            'x_traj': x_traj,
            'y_traj': y_traj,
            'v_traj': v_traj,
            'a_traj': a_traj
        }
    
    def _calculate_path_distances(self, x_coords: np.ndarray, y_coords: np.ndarray) -> List[float]:
        """Calculate cumulative distances along the path."""
        distances = [0.0]
        for i in range(1, len(x_coords)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            distance = math.sqrt(dx*dx + dy*dy)
            distances.append(distances[-1] + distance)
        return distances
    
    def _generate_trapezoidal_velocity_profile(self, total_distance: float, max_velocity: float,
                                             acceleration: float, deceleration: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate trapezoidal velocity profile."""
        # Calculate time for acceleration and deceleration phases
        t_acc = max_velocity / acceleration
        t_dec = max_velocity / deceleration
        
        # Distance covered during acceleration and deceleration
        d_acc = 0.5 * acceleration * t_acc**2
        d_dec = 0.5 * deceleration * t_dec**2
        
        # Check if we can reach max velocity
        if d_acc + d_dec > total_distance:
            # Triangular profile - can't reach max velocity
            max_velocity = math.sqrt(total_distance / (1/(2*acceleration) + 1/(2*deceleration)))
            t_acc = max_velocity / acceleration
            t_dec = max_velocity / deceleration
            t_const = 0.0
        else:
            # Trapezoidal profile - can reach max velocity
            d_const = total_distance - d_acc - d_dec
            t_const = d_const / max_velocity
        
        # Total time
        total_time = t_acc + t_const + t_dec
        
        # Generate time array
        time = np.arange(0, total_time + dt, dt)
        
        # Generate velocity and acceleration profiles
        velocity = np.zeros_like(time)
        acceleration_profile = np.zeros_like(time)
        
        for i, t in enumerate(time):
            if t <= t_acc:
                # Acceleration phase
                velocity[i] = acceleration * t
                acceleration_profile[i] = acceleration
            elif t <= t_acc + t_const:
                # Constant velocity phase
                velocity[i] = max_velocity
                acceleration_profile[i] = 0.0
            else:
                # Deceleration phase
                t_dec_elapsed = t - (t_acc + t_const)
                velocity[i] = max_velocity - deceleration * t_dec_elapsed
                acceleration_profile[i] = -deceleration
        
        return time, velocity, acceleration_profile
    
    def _generate_constant_velocity_profile(self, total_distance: float, velocity: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate constant velocity profile."""
        total_time = total_distance / velocity
        time = np.arange(0, total_time + dt, dt)
        velocity_profile = np.full_like(time, velocity)
        acceleration_profile = np.zeros_like(time)
        
        return time, velocity_profile, acceleration_profile
    
    def _generate_sine_velocity_profile(self, total_distance: float, max_velocity: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate sine-based velocity profile for smooth start/stop."""
        # Use half sine wave for smooth acceleration and deceleration
        total_time = total_distance / (max_velocity * 2 / math.pi)  # Average velocity is 2/pi * max_velocity
        time = np.arange(0, total_time + dt, dt)
        
        velocity_profile = max_velocity * np.sin(math.pi * time / total_time)
        acceleration_profile = max_velocity * math.pi / total_time * np.cos(math.pi * time / total_time)
        
        return time, velocity_profile, acceleration_profile
    
    def _generate_position_trajectory(self, smooth_x: np.ndarray, smooth_y: np.ndarray,
                                    time: np.ndarray, velocity: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Generate position trajectory based on velocity profile."""
        # Calculate cumulative distance at each time point
        distance_traj = np.cumsum(velocity) * (time[1] - time[0])  # Approximate integration
        
        # Normalize distance to path parameter
        total_path_distance = self._calculate_path_length_from_arrays(smooth_x, smooth_y)
        path_parameter = distance_traj / total_path_distance
        
        # Ensure path_parameter doesn't exceed 1.0
        path_parameter = np.clip(path_parameter, 0.0, 1.0)
        
        # Interpolate position along the smooth path
        path_distances = self._calculate_path_distances(smooth_x, smooth_y)
        path_distances_normalized = np.array(path_distances) / path_distances[-1]
        
        # Create interpolation functions
        x_interp = interp1d(path_distances_normalized, smooth_x, kind='cubic', bounds_error=False, fill_value='extrapolate')
        y_interp = interp1d(path_distances_normalized, smooth_y, kind='cubic', bounds_error=False, fill_value='extrapolate')
        
        # Generate trajectory positions
        x_traj = x_interp(path_parameter)
        y_traj = y_interp(path_parameter)
        
        return x_traj, y_traj
    
    def _calculate_path_length_from_arrays(self, x_coords: np.ndarray, y_coords: np.ndarray) -> float:
        """Calculate the total length of a path given coordinate arrays."""
        total_length = 0.0
        for i in range(1, len(x_coords)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            total_length += math.sqrt(dx*dx + dy*dy)
        return total_length
    
    def get_trajectory_statistics(self) -> Dict[str, float]:
        """Get statistics about the generated trajectory."""
        if self.time is None or self.x_traj is None:
            raise ValueError("No trajectory has been generated yet")
        
        stats = {
            'duration': self.time[-1],
            'total_distance': self._calculate_path_length_from_arrays(self.x_traj, self.y_traj),
            'max_velocity': np.max(self.v_traj),
            'avg_velocity': np.mean(self.v_traj),
            'max_acceleration': np.max(self.a_traj),
            'min_acceleration': np.min(self.a_traj),
            'num_points': len(self.time)
        }
        
        return stats
    
    def plot_trajectory(self, show_original_path: bool = True, 
                       show_velocity_profile: bool = True,
                       show_acceleration_profile: bool = True) -> plt.Figure:
        """
        Plot the generated trajectory with various visualizations.
        
        Args:
            show_original_path: Whether to show the original waypoints and smooth path
            show_velocity_profile: Whether to show velocity profile
            show_acceleration_profile: Whether to show acceleration profile
        
        Returns:
            matplotlib Figure object
        """
        if self.time is None or self.x_traj is None:
            raise ValueError("No trajectory has been generated yet")
        
        # Determine number of subplots
        num_plots = 1  # Always show position trajectory
        if show_velocity_profile:
            num_plots += 1
        if show_acceleration_profile:
            num_plots += 1
        
        fig, axes = plt.subplots(1, num_plots, figsize=(5*num_plots, 8))
        if num_plots == 1:
            axes = [axes]
        
        plot_idx = 0
        
        # Plot 1: Position trajectory
        axes[plot_idx].plot(self.x_traj, self.y_traj, 'b-', linewidth=2, label='Time-Parameterized Trajectory')
        
        if show_original_path and self.smooth_x is not None and self.smooth_y is not None:
            axes[plot_idx].plot(self.smooth_x, self.smooth_y, 'r--', linewidth=1, alpha=0.7, label='Original Smooth Path')
        
        if self.waypoints is not None:
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            axes[plot_idx].plot(wp_x, wp_y, 'ko-', linewidth=1, markersize=6, label='Original Waypoints', alpha=0.5)
        
        # Mark start and end points
        axes[plot_idx].plot(self.x_traj[0], self.y_traj[0], 'go', markersize=10, label=f'Start (t={self.time[0]:.1f}s)')
        axes[plot_idx].plot(self.x_traj[-1], self.y_traj[-1], 'ro', markersize=10, label=f'End (t={self.time[-1]:.1f}s)')
        
        axes[plot_idx].set_xlabel('X Position (m)')
        axes[plot_idx].set_ylabel('Y Position (m)')
        axes[plot_idx].set_title('Position Trajectory')
        axes[plot_idx].legend()
        axes[plot_idx].grid(True, alpha=0.3)
        axes[plot_idx].axis('equal')
        plot_idx += 1
        
        # Plot 2: Velocity profile
        if show_velocity_profile:
            axes[plot_idx].plot(self.time, self.v_traj, 'g-', linewidth=2)
            axes[plot_idx].set_xlabel('Time (s)')
            axes[plot_idx].set_ylabel('Velocity (m/s)')
            axes[plot_idx].set_title('Velocity Profile')
            axes[plot_idx].grid(True, alpha=0.3)
            plot_idx += 1
        
        # Plot 3: Acceleration profile
        if show_acceleration_profile:
            axes[plot_idx].plot(self.time, self.a_traj, 'r-', linewidth=2)
            axes[plot_idx].set_xlabel('Time (s)')
            axes[plot_idx].set_ylabel('Acceleration (m/s²)')
            axes[plot_idx].set_title('Acceleration Profile')
            axes[plot_idx].grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def get_logger(self):
        """Get a simple logger for the trajectory generator."""
        import logging
        logger = logging.getLogger('TrajectoryGenerator')
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        return logger


def create_trajectory_from_waypoints(waypoints: List[Tuple[float, float]],
                                   velocity_profile: str = 'trapezoidal', # 'trapezoidal', 'constant', 'sine'
                                   max_velocity: float = 1.0,
                                   acceleration: float = 0.5,
                                   deceleration: float = 0.5,
                                   dt: float = 0.1,
                                   num_smooth_points: int = 100) -> TrajectoryGenerator:
    """
    Convenience function to create a complete trajectory from waypoints.
    
    Args:
        waypoints: List of 2D waypoints
        velocity_profile: Type of velocity profile
        max_velocity: Maximum velocity (m/s)
        acceleration: Acceleration rate (m/s²)
        deceleration: Deceleration rate (m/s²)
        dt: Time step for trajectory sampling (s)
        num_smooth_points: Number of points for smooth path generation
    
    Returns:
        TrajectoryGenerator instance with complete trajectory
    """
    generator = TrajectoryGenerator()
    generator.set_waypoints(waypoints)
    generator.smooth_path_spline(num_smooth_points)
    generator.generate_time_parameterized_trajectory(
        velocity_profile=velocity_profile,
        max_velocity=max_velocity,
        acceleration=acceleration,
        deceleration=deceleration,
        dt=dt
    )
    return generator


def example_usage():
    """Example usage of the TrajectoryGenerator class."""
    # Example waypoints
    waypoints = [
        (0.0, 0.0),      # Start point
        (2.0, 1.0),      # Waypoint 1
        (4.0, 0.5),      # Waypoint 2
        (6.0, 2.0),      # Waypoint 3
        (8.0, 1.5),      # Waypoint 4
        (10.0, 3.0),     # Waypoint 5
        (12.0, 2.0),     # End point
    ]
    
    # Create trajectory generator
    generator = TrajectoryGenerator()
    
    # Set waypoints and generate trajectory
    generator.set_waypoints(waypoints)
    generator.smooth_path_spline(num_points=200)
    
    # Generate time-parameterized trajectory
    trajectory_data = generator.generate_time_parameterized_trajectory(
        velocity_profile='trapezoidal', # 'trapezoidal', 'constant', 'sine'
        max_velocity=1.5, 
        acceleration=0.8,
        deceleration=0.8,
        dt=0.1
    )
    
    # Get statistics
    stats = generator.get_trajectory_statistics()
    print("Trajectory Statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value:.3f}")
    
    # Plot trajectory
    fig = generator.plot_trajectory()
    plt.show()
    
    return generator


if __name__ == '__main__':
    example_usage()
