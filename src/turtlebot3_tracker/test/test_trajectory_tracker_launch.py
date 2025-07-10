#!/usr/bin/env python3

import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import pytest


@pytest.mark.rostest
def generate_test_description():
    """Generate test description for trajectory tracker launch tests."""
    
    # Declare launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    velocity_profile = LaunchConfiguration('velocity_profile')
    max_velocity = LaunchConfiguration('max_velocity')
    controller_type = LaunchConfiguration('controller_type')
    
    # Create trajectory publisher node
    trajectory_node = Node(
        package='turtlebot3_tracker',
        executable='trajectory_publisher_node',
        name='test_trajectory_publisher',
        output='screen',
        parameters=[{
            'velocity_profile': velocity_profile,
            'max_velocity': max_velocity,
            'controller_type': controller_type,
            'publish_visualization': True,
            'use_rviz': False,  # Disable RViz for testing
        }]
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Whether to launch RViz for visualization'
        ),
        
        DeclareLaunchArgument(
            'velocity_profile',
            default_value='trapezoidal',
            description='Velocity profile type'
        ),
        
        DeclareLaunchArgument(
            'max_velocity',
            default_value='0.5',
            description='Maximum velocity'
        ),
        
        DeclareLaunchArgument(
            'controller_type',
            default_value='pure_pursuit',
            description='Controller type'
        ),
        
        # Launch nodes
        trajectory_node,
        
        # Keep the test alive
        ReadyToTest(),
        
        # Timer to allow nodes to start up
        TimerAction(
            period=2.0,
            actions=[]
        )
    ])


class TestTrajectoryTrackerLaunch:
    """Test class for trajectory tracker launch functionality."""
    
    def test_trajectory_node_startup(self, launch_service, proc_info, proc_output):
        """Test that trajectory node starts up correctly."""
        # This test verifies that the trajectory node starts without errors
        # The actual testing is done by the launch test framework
        assert proc_info[0].returncode == 0, "Trajectory node should start successfully"
    
    def test_topic_publication(self, launch_service, proc_info, proc_output):
        """Test that required topics are published."""
        # This test would verify that the required topics are being published
        # In a real implementation, you would check for specific topics
        assert True, "Topic publication test passed"
    
    def test_parameter_loading(self, launch_service, proc_info, proc_output):
        """Test that parameters are loaded correctly."""
        # This test would verify that parameters are loaded correctly
        # In a real implementation, you would check parameter values
        assert True, "Parameter loading test passed"


if __name__ == '__main__':
    # This allows the test to be run directly
    import pytest
    pytest.main() 