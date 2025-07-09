#!/usr/bin/env python3

import numpy as np
import math
from typing import Tuple, Optional
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion  # Install using: pip install transforms3d or tf-transformations


class TrajectoryController:
    """
    A controller class for trajectory tracking of a differential drive robot.
    
    This class implements various control algorithms:
    - Pure Pursuit Controller
    - PID Controller
    - Stanley Controller
    """
    
    def __init__(self, controller_type: str = 'pure_pursuit', **kwargs):
        """
        Initialize the trajectory controller.
        
        Args:
            controller_type: Type of controller ('pure_pursuit', 'pid', 'stanley')
            **kwargs: Controller-specific parameters
        """
        self.controller_type = controller_type
        self.current_pose = None
        self.current_velocity = None
        self.target_pose = None
        self.target_velocity = 0.0
        
        # Path/trajectory for Pure Pursuit
        self.path_x = None
        self.path_y = None
        self.current_path_index = 0
        
        # Controller parameters
        if controller_type == 'pure_pursuit':
            self.lookahead_distance = kwargs.get('lookahead_distance', 0.5)
            self.max_linear_velocity = kwargs.get('max_linear_velocity', 1.0)
            self.max_angular_velocity = kwargs.get('max_angular_velocity', 1.0)
            self.min_linear_velocity = kwargs.get('min_linear_velocity', 0.1)
            
        elif controller_type == 'pid':
            self.kp_linear = kwargs.get('kp_linear', 1.0)
            self.ki_linear = kwargs.get('ki_linear', 0.0)
            self.kd_linear = kwargs.get('kd_linear', 0.0)
            self.kp_angular = kwargs.get('kp_angular', 1.0)
            self.ki_angular = kwargs.get('ki_angular', 0.0)
            self.kd_angular = kwargs.get('kd_angular', 0.0)
            self.max_linear_velocity = kwargs.get('max_linear_velocity', 1.0)
            self.max_angular_velocity = kwargs.get('max_angular_velocity', 1.0)
            
            # PID state variables
            self.linear_error_integral = 0.0
            self.angular_error_integral = 0.0
            self.prev_linear_error = 0.0
            self.prev_angular_error = 0.0
            self.prev_time = None
            
        elif controller_type == 'stanley':
            self.k_e = kwargs.get('k_e', 1.0)  # Cross-track error gain
            self.k_v = kwargs.get('k_v', 1.0)  # Velocity gain
            self.max_steering_angle = kwargs.get('max_steering_angle', math.pi/4)
            self.max_linear_velocity = kwargs.get('max_linear_velocity', 1.0)
            self.min_linear_velocity = kwargs.get('min_linear_velocity', 0.1)
            
        else:
            raise ValueError(f"Unknown controller type: {controller_type}")
    
    def update_current_state(self, pose: PoseStamped, velocity: Optional[Twist] = None):
        """
        Update the current robot state.
        
        Args:
            pose: Current robot pose
            velocity: Current robot velocity (optional)
        """
        self.current_pose = pose
        self.current_velocity = velocity
    
    def set_target(self, target_pose: PoseStamped, target_velocity: float = 0.0):
        """
        Set the target pose and velocity.
        
        Args:
            target_pose: Target pose
            target_velocity: Target velocity
        """
        self.target_pose = target_pose
        self.target_velocity = target_velocity
    
    def set_path(self, path_x: list, path_y: list):
        """
        Set the path for Pure Pursuit controller.
        
        Args:
            path_x: List of x-coordinates for the path
            path_y: List of y-coordinates for the path
        """
        self.path_x = path_x
        self.path_y = path_y
        self.current_path_index = 0
    
    def compute_control(self, current_time: float) -> Twist:
        """
        Compute control commands based on the current state and target.
        
        Args:
            current_time: Current time for time-based controllers
            
        Returns:
            Twist message with linear and angular velocity commands
        """
        if self.current_pose is None or self.target_pose is None:
            return Twist()
        
        if self.controller_type == 'pure_pursuit':
            return self._pure_pursuit_control()
        elif self.controller_type == 'pid':
            return self._pid_control(current_time)
        elif self.controller_type == 'stanley':
            return self._stanley_control()
        else:
            return Twist()
    
    def _pure_pursuit_control(self) -> Twist:
        """
        Pure Pursuit controller implementation.
        
        Returns:
            Twist message with control commands
        """
        # Check if we have a path
        if self.path_x is None or self.path_y is None or len(self.path_x) < 2:
            # Fallback to single target point if no path is available
            return self._pure_pursuit_single_target()
        
        # Extract current position
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Get current heading from quaternion
        orientation_q = self.current_pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, current_heading = euler_from_quaternion(quaternion)
        
        # Find the lookahead point on the path
        lookahead_x, lookahead_y, path_index = self._find_lookahead_point(
            current_x, current_y, current_heading)
        
        # Update current path index
        self.current_path_index = path_index
        
        # Calculate heading to lookahead point
        dx = lookahead_x - current_x
        dy = lookahead_y - current_y
        desired_heading = math.atan2(dy, dx)
        
        # Calculate heading error
        heading_error = self._normalize_angle(desired_heading - current_heading)
        
        # Calculate curvature using Pure Pursuit formula
        # curvature = 2 * sin(alpha) / L, where alpha is heading error and L is lookahead distance
        curvature = 2.0 * math.sin(heading_error) / self.lookahead_distance
        
        # Calculate control commands
        linear_velocity = min(self.target_velocity, self.max_linear_velocity)
        
        # Check if we're close to the end of the path
        distance_to_end = math.sqrt((current_x - self.path_x[-1])**2 + (current_y - self.path_y[-1])**2)
        if distance_to_end < 0.1:  # Close to end
            linear_velocity = self.min_linear_velocity
        
        angular_velocity = curvature * linear_velocity
        angular_velocity = np.clip(angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        return twist
    
    def _pure_pursuit_single_target(self) -> Twist:
        """
        Fallback Pure Pursuit implementation for single target point.
        
        Returns:
            Twist message with control commands
        """
        # Extract current and target positions
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y
        
        # Calculate distance to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate desired heading
        desired_heading = math.atan2(dy, dx)
        
        # Get current heading from quaternion
        orientation_q = self.current_pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, current_heading = euler_from_quaternion(quaternion)
        
        # Calculate heading error
        heading_error = self._normalize_angle(desired_heading - current_heading)
        
        # Pure Pursuit algorithm
        if distance > self.lookahead_distance:
            # Look ahead point is beyond the target, use target directly
            lookahead_x = target_x
            lookahead_y = target_y
        else:
            # Calculate lookahead point
            lookahead_x = current_x + self.lookahead_distance * math.cos(desired_heading)
            lookahead_y = current_y + self.lookahead_distance * math.sin(desired_heading)
        
        # Calculate curvature
        curvature = 2.0 * math.sin(heading_error) / self.lookahead_distance
        
        # Calculate control commands
        # Ensure positive linear velocity (robot moves forward)
        linear_velocity = max(self.min_linear_velocity, min(self.target_velocity, self.max_linear_velocity))
        if distance < 0.1:  # Close to target
            linear_velocity = self.min_linear_velocity
        
        # Calculate angular velocity based on curvature
        angular_velocity = curvature * linear_velocity
        angular_velocity = np.clip(angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Ensure the robot is moving forward by checking heading alignment
        # If heading error is too large, reduce linear velocity
        if abs(heading_error) > math.pi/4:  # More than 45 degrees
            linear_velocity = max(self.min_linear_velocity, linear_velocity * 0.5)

        print(f"heading error: {heading_error}")
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        return twist
    
    def _find_lookahead_point(self, current_x: float, current_y: float, current_heading: float) -> Tuple[float, float, int]:
        """
        Find the lookahead point on the path.
        
        Args:
            current_x: Current robot x position
            current_y: Current robot y position
            current_heading: Current robot heading
            
        Returns:
            Tuple of (lookahead_x, lookahead_y, path_index)
        """
        start_index = max(0, self.current_path_index - 1)
        best_index = None
        best_distance = float('inf')
        # First, try to find a point at least lookahead_distance away and in front
        for i in range(start_index, len(self.path_x)):
            path_x = self.path_x[i]
            path_y = self.path_y[i]
            dx = path_x - current_x
            dy = path_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            angle_to_point = math.atan2(dy, dx)
            angle_diff = self._normalize_angle(angle_to_point - current_heading)
            # Only consider points in front (within +/- 90 degrees)
            if distance >= self.lookahead_distance and abs(angle_diff) < math.pi/2:
                return path_x, path_y, i
            # Track the closest point in front (for fallback)
            if abs(angle_diff) < math.pi/2 and distance < best_distance:
                best_distance = distance
                best_index = i
        # Fallback: use the closest point in front, if any
        if best_index is not None:
            return self.path_x[best_index], self.path_y[best_index], best_index
        # If no point in front, use the closest point overall (but never behind)
        # This is a last resort and should rarely happen
        min_distance = float('inf')
        min_index = start_index
        for i in range(start_index, len(self.path_x)):
            path_x = self.path_x[i]
            path_y = self.path_y[i]
            dx = path_x - current_x
            dy = path_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < min_distance:
                min_distance = distance
                min_index = i
        # If even here, check if the closest point is in front, else just use current position as lookahead
        path_x = self.path_x[min_index]
        path_y = self.path_y[min_index]
        dx = path_x - current_x
        dy = path_y - current_y
        angle_to_point = math.atan2(dy, dx)
        angle_diff = self._normalize_angle(angle_to_point - current_heading)
        if abs(angle_diff) < math.pi/2:
            return path_x, path_y, min_index
        # As a last fallback, use a point directly in front at lookahead distance
        lookahead_x = current_x + self.lookahead_distance * math.cos(current_heading)
        lookahead_y = current_y + self.lookahead_distance * math.sin(current_heading)
        return lookahead_x, lookahead_y, self.current_path_index
    
    def _pid_control(self, current_time: float) -> Twist:
        """
        PID controller implementation.
        
        Args:
            current_time: Current time for integral calculation
            
        Returns:
            Twist message with control commands
        """
        # Extract current and target positions
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y
        
        # Calculate position error
        dx = target_x - current_x
        dy = target_y - current_y
        position_error = math.sqrt(dx*dx + dy*dy)
        
        # Calculate heading error
        desired_heading = math.atan2(dy, dx)

        # Use the class helper to get current heading from quaternion
        orientation_q = self.current_pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, current_heading = euler_from_quaternion(quaternion)
        heading_error = self._normalize_angle(desired_heading - current_heading)

        # Calculate time step for integral and derivative
        dt = 0.1  # Default time step
        if self.prev_time is not None:
            dt = current_time - self.prev_time
        
        # Update integral terms
        self.linear_error_integral += position_error * dt
        self.angular_error_integral += heading_error * dt
        
        # Calculate derivative terms
        linear_error_derivative = 0.0
        angular_error_derivative = 0.0
        if self.prev_time is not None and dt > 0:
            linear_error_derivative = (position_error - self.prev_linear_error) / dt
            angular_error_derivative = (heading_error - self.prev_angular_error) / dt
        
        # PID control law
        linear_velocity = (self.kp_linear * position_error + 
                          self.ki_linear * self.linear_error_integral + 
                          self.kd_linear * linear_error_derivative)
        
        angular_velocity = (self.kp_angular * heading_error + 
                           self.ki_angular * self.angular_error_integral + 
                           self.kd_angular * angular_error_derivative)
        
        # Apply velocity limits

        if np.abs(heading_error) > math.pi/2:
            linear_velocity = 0.05 # stop the robot to turn
        else:
            linear_velocity = np.clip(linear_velocity, 0.0, self.max_linear_velocity)
        angular_velocity = np.clip(angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Update previous values
        self.prev_linear_error = position_error
        self.prev_angular_error = heading_error
        self.prev_time = current_time
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        return twist
    
    def _stanley_control(self) -> Twist:
        """
        Stanley controller implementation.
        
        Returns:
            Twist message with control commands
        """
        # Extract current and target positions
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y
        
        # Calculate cross-track error
        dx = target_x - current_x
        dy = target_y - current_y
        cross_track_error = math.sqrt(dx*dx + dy*dy)
        
        # Determine sign of cross-track error
        orientation_q = self.current_pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, current_heading = euler_from_quaternion(quaternion)
        desired_heading = math.atan2(dy, dx)
        heading_diff = self._normalize_angle(desired_heading - current_heading)
        
        if abs(heading_diff) > math.pi/2:
            cross_track_error = -cross_track_error
        
        # Calculate heading error
        heading_error = self._normalize_angle(desired_heading - current_heading)
        
        # Get current velocity
        current_velocity = 0.0
        if self.current_velocity is not None:
            current_velocity = math.sqrt(
                self.current_velocity.linear.x**2 + 
                self.current_velocity.linear.y**2
            )
        
        # Stanley control law
        # Steering angle = heading_error + arctan(k_e * cross_track_error / (k_v + velocity))
        steering_angle = (heading_error + 
                         math.atan2(self.k_e * cross_track_error, 
                                   self.k_v + current_velocity))
        
        # Apply steering angle limits
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Calculate control commands
        linear_velocity = min(self.target_velocity, self.max_linear_velocity)

        if np.abs(heading_error) > math.pi/2:
            linear_velocity = 0.05 # stop the robot to turn

        if cross_track_error < 0.1:  # Close to path
            linear_velocity = max(linear_velocity, self.min_linear_velocity)
        
        angular_velocity = steering_angle * linear_velocity
        
        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        return twist

    
    def _normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to [-pi, pi].
        
        Args:
            angle: Input angle in radians
            
        Returns:
            Normalized angle in radians
        """

        return math.atan2(math.sin(angle), math.cos(angle))
    
    def reset(self):
        """Reset controller state."""
        if self.controller_type == 'pid':
            self.linear_error_integral = 0.0
            self.angular_error_integral = 0.0
            self.prev_linear_error = 0.0
            self.prev_angular_error = 0.0
            self.prev_time = None
        
        # Reset Pure Pursuit path index
        if self.controller_type == 'pure_pursuit':
            self.current_path_index = 0
    
    def get_controller_info(self) -> dict:
        """
        Get controller information and parameters.
        
        Returns:
            Dictionary with controller information
        """
        info = {
            'controller_type': self.controller_type,
            'current_pose': self.current_pose,
            'target_pose': self.target_pose,
            'target_velocity': self.target_velocity
        }
        
        if self.controller_type == 'pure_pursuit':
            info.update({
                'lookahead_distance': self.lookahead_distance,
                'max_linear_velocity': self.max_linear_velocity,
                'max_angular_velocity': self.max_angular_velocity,
                'min_linear_velocity': self.min_linear_velocity,
                'path_length': len(self.path_x) if self.path_x is not None else 0,
                'current_path_index': self.current_path_index
            })
        elif self.controller_type == 'pid':
            info.update({
                'kp_linear': self.kp_linear,
                'ki_linear': self.ki_linear,
                'kd_linear': self.kd_linear,
                'kp_angular': self.kp_angular,
                'ki_angular': self.ki_angular,
                'kd_angular': self.kd_angular,
                'max_linear_velocity': self.max_linear_velocity,
                'max_angular_velocity': self.max_angular_velocity
            })
        elif self.controller_type == 'stanley':
            info.update({
                'k_e': self.k_e,
                'k_v': self.k_v,
                'max_steering_angle': self.max_steering_angle,
                'max_linear_velocity': self.max_linear_velocity,
                'min_linear_velocity': self.min_linear_velocity
            })
        
        return info 