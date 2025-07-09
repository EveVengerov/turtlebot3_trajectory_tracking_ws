#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for trajectory generation
        DeclareLaunchArgument(
            'velocity_profile',
            default_value='trapezoidal',
            description='Velocity profile type: trapezoidal, constant, or sine'
        ),
        
        DeclareLaunchArgument(
            'max_velocity',
            default_value='0.3',
            description='Maximum velocity in m/s'
        ),
        
        DeclareLaunchArgument(
            'acceleration',
            default_value='0.5',
            description='Acceleration in m/s²'
        ),
        
        DeclareLaunchArgument(
            'deceleration',
            default_value='0.5',
            description='Deceleration in m/s²'
        ),
        
        DeclareLaunchArgument(
            'dt',
            default_value='0.1',
            description='Time step for trajectory sampling in seconds'
        ),
        
        DeclareLaunchArgument(
            'num_smooth_points',
            default_value='100',
            description='Number of points for smooth trajectory generation'
        ),
        
        # Launch arguments for controller
        DeclareLaunchArgument(
            'controller_type',
            default_value='pure_pursuit',
            description='Controller type: pure_pursuit, pid, or stanley'
        ),
        
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='0.2',
            description='Lookahead distance for pure pursuit controller in meters'
        ),
        
        DeclareLaunchArgument(
            'kp_linear',
            default_value='1.0',
            description='Proportional gain for linear velocity (PID controller)'
        ),
        
        DeclareLaunchArgument(
            'ki_linear',
            default_value='0.0',
            description='Integral gain for linear velocity (PID controller)'
        ),
        
        DeclareLaunchArgument(
            'kd_linear',
            default_value='0.0',
            description='Derivative gain for linear velocity (PID controller)'
        ),
        
        DeclareLaunchArgument(
            'kp_angular',
            default_value='1.0',
            description='Proportional gain for angular velocity (PID controller)'
        ),
        
        DeclareLaunchArgument(
            'ki_angular',
            default_value='0.0',
            description='Integral gain for angular velocity (PID controller)'
        ),
        
        DeclareLaunchArgument(
            'kd_angular',
            default_value='0.0',
            description='Derivative gain for angular velocity (PID controller)'
        ),
        
        DeclareLaunchArgument(
            'k_e',
            default_value='1.0',
            description='Cross-track error gain for Stanley controller'
        ),
        
        DeclareLaunchArgument(
            'k_v',
            default_value='1.0',
            description='Velocity gain for Stanley controller'
        ),
        
        # Launch arguments for control loop
        DeclareLaunchArgument(
            'control_frequency',
            default_value='50.0',
            description='Control loop frequency in Hz'
        ),
        
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.1',
            description='Goal tolerance in meters'
        ),
        
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='1.0',
            description='Maximum linear velocity for controller in m/s'
        ),
        
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='1.0',
            description='Maximum angular velocity for controller in rad/s'
        ),
        
        # Launch arguments for visualization
        DeclareLaunchArgument(
            'publish_visualization',
            default_value='true',
            description='Whether to publish trajectory visualization'
        ),
        
        DeclareLaunchArgument(
            'visualization_frequency',
            default_value='1.0',
            description='Visualization publishing frequency in Hz'
        ),
        
        # Trajectory publisher node
        Node(
            package='turtlebot3_tracker',
            executable='trajectory_publisher_node',
            name='trajectory_publisher_node',
            output='screen',
            parameters=[{
                # Trajectory generation parameters
                'velocity_profile': LaunchConfiguration('velocity_profile'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'acceleration': LaunchConfiguration('acceleration'),
                'deceleration': LaunchConfiguration('deceleration'),
                'dt': LaunchConfiguration('dt'),
                'num_smooth_points': LaunchConfiguration('num_smooth_points'),
                
                # Controller parameters
                'controller_type': LaunchConfiguration('controller_type'),
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'kp_linear': LaunchConfiguration('kp_linear'),
                'ki_linear': LaunchConfiguration('ki_linear'),
                'kd_linear': LaunchConfiguration('kd_linear'),
                'kp_angular': LaunchConfiguration('kp_angular'),
                'ki_angular': LaunchConfiguration('ki_angular'),
                'kd_angular': LaunchConfiguration('kd_angular'),
                'k_e': LaunchConfiguration('k_e'),
                'k_v': LaunchConfiguration('k_v'),
                
                # Control loop parameters
                'control_frequency': LaunchConfiguration('control_frequency'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                
                # Visualization parameters
                'publish_visualization': LaunchConfiguration('publish_visualization'),
                'visualization_frequency': LaunchConfiguration('visualization_frequency'),
                
                # Default waypoints - using a simple format that ROS2 can handle
                'default_waypoints': [0.0, 0.0, 1.0, 1.0, 2.0, 0.0, 3.0, 1.0, 4.0, 0.0],
            }]
        )
    ]) 