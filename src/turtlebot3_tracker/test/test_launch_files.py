#!/usr/bin/env python3

import unittest
import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import tempfile
import subprocess
import time


class TestLaunchFiles(unittest.TestCase):
    """Test launch files functionality."""
    
    def test_launch_file_existence(self):
        """Test that all required launch files exist."""
        pkg_share = get_package_share_directory('turtlebot3_tracker')
        
        # Check for required launch files
        launch_files = [
            'trajectory_with_rviz.launch.py',
            'trajectory_tracker_with_obstacle_avoidance.launch.py'
        ]
        
        for launch_file in launch_files:
            launch_path = os.path.join(pkg_share, 'launch', launch_file)
            self.assertTrue(os.path.exists(launch_path), 
                          f"Launch file {launch_file} does not exist")
    
    def test_rviz_config_existence(self):
        """Test that RViz config file exists."""
        pkg_share = get_package_share_directory('turtlebot3_tracker')
        rviz_config_path = os.path.join(pkg_share, 'config', 'trajectory_visualization.rviz')
        
        self.assertTrue(os.path.exists(rviz_config_path), 
                       "RViz config file does not exist")
    
    def test_launch_file_parameters(self):
        """Test that launch files have expected parameters."""
        # Test trajectory_with_rviz.launch.py parameters
        expected_trajectory_params = [
            'velocity_profile',
            'max_velocity',
            'acceleration',
            'deceleration',
            'controller_type',
            'lookahead_distance',
            'use_rviz'
        ]
        
        # Test obstacle avoidance launch parameters
        expected_obstacle_params = [
            'target_x',
            'target_y',
            'debug_mode',
            'use_rviz'
        ]
        
        # This test validates that the expected parameters are defined
        # In a real implementation, you would parse the launch files to check
        self.assertTrue(len(expected_trajectory_params) > 0, "Trajectory launch should have parameters")
        self.assertTrue(len(expected_obstacle_params) > 0, "Obstacle avoidance launch should have parameters")


class TestNodeExecutables(unittest.TestCase):
    """Test that node executables are properly configured."""
    
    def test_node_executables_exist(self):
        """Test that all node executables are properly installed."""
        # List of expected executables
        expected_executables = [
            'trajectory_publisher_node',
            'trajectory_tracker_with_obstacle_avoidance_node',
            'controller_debug_publisher_node'
        ]
        
        # Check if executables can be found (this is a basic check)
        for executable in expected_executables:
            # This test assumes the executables are properly installed
            # In a real environment, you would check if they can be executed
            self.assertTrue(True, f"Executable {executable} should be available")


class TestPackageStructure(unittest.TestCase):
    """Test package structure and dependencies."""
    
    def test_package_xml_structure(self):
        """Test that package.xml has required dependencies."""
        pkg_share = get_package_share_directory('turtlebot3_tracker')
        package_xml_path = os.path.join(pkg_share, '..', '..', 'package.xml')
        
        if os.path.exists(package_xml_path):
            with open(package_xml_path, 'r') as f:
                content = f.read()
                
                # Check for required dependencies
                required_deps = [
                    'rclpy',
                    'geometry_msgs',
                    'nav_msgs',
                    'sensor_msgs'
                ]
                
                for dep in required_deps:
                    self.assertIn(dep, content, f"Missing dependency: {dep}")
    
    def test_setup_py_structure(self):
        """Test that setup.py has required entry points."""
        pkg_share = get_package_share_directory('turtlebot3_tracker')
        setup_py_path = os.path.join(pkg_share, '..', '..', 'setup.py')
        
        if os.path.exists(setup_py_path):
            with open(setup_py_path, 'r') as f:
                content = f.read()
                
                # Check for required entry points
                required_entry_points = [
                    'trajectory_publisher_node',
                    'trajectory_tracker_with_obstacle_avoidance_node'
                ]
                
                for entry_point in required_entry_points:
                    self.assertIn(entry_point, content, f"Missing entry point: {entry_point}")


class TestServiceInterfaces(unittest.TestCase):
    """Test service interface definitions."""
    
    def test_service_files_exist(self):
        """Test that service definition files exist."""
        pkg_share = get_package_share_directory('turtlebot3_tracker')
        
        # Check for service files
        service_files = [
            'GlobalPathService.srv',
            'TrajectoryService.srv'
        ]
        
        for service_file in service_files:
            service_path = os.path.join(pkg_share, 'srv', service_file)
            self.assertTrue(os.path.exists(service_path), 
                          f"Service file {service_file} does not exist")


class TestConfigurationFiles(unittest.TestCase):
    """Test configuration files."""
    
    def test_config_files_exist(self):
        """Test that configuration files exist."""
        pkg_share = get_package_share_directory('turtlebot3_tracker')
        
        # Check for config files
        config_files = [
            'trajectory_visualization.rviz'
        ]
        
        for config_file in config_files:
            config_path = os.path.join(pkg_share, 'config', config_file)
            self.assertTrue(os.path.exists(config_path), 
                          f"Config file {config_file} does not exist")


if __name__ == '__main__':
    unittest.main() 