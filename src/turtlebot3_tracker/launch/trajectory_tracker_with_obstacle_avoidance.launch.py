#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('turtlebot3_tracker')
    
    # RViz config file path
    rviz_config_file = os.path.join(pkg_share, 'config', 'trajectory_visualization.rviz')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to launch RViz for visualization'
        ),
        
        DeclareLaunchArgument(
            'target_x',
            default_value='4.0',
            description='Target X position in meters'
        ),
        
        DeclareLaunchArgument(
            'target_y',
            default_value='0.0',
            description='Target Y position in meters'
        ),
        
        DeclareLaunchArgument(
            'debug_mode',
            default_value='False',
            description='Enable debug mode (bypass obstacle avoidance)'
        ),
        
        # Trajectory tracker with obstacle avoidance node
        Node(
            package='turtlebot3_tracker',
            executable='trajectory_tracker_with_obstacle_avoidance_node',
            name='trajectory_tracker_with_obstacle_avoidance',
            output='screen',
            parameters=[{
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'debug_mode': LaunchConfiguration('debug_mode'),
            }]
        ),
        
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output="log"
        )
    ]) 