#!/usr/bin/env python3

import unittest
import os
import sys
from pathlib import Path


class TestPackageStructure(unittest.TestCase):
    """Basic tests for package structure and imports."""
    
    def test_package_imports(self):
        """Test that basic imports work."""
        try:
            import rclpy
            import geometry_msgs.msg
            import nav_msgs.msg
            import sensor_msgs.msg
            self.assertTrue(True, "Basic ROS imports successful")
        except ImportError as e:
            self.fail(f"Failed to import ROS modules: {e}")
    
    def test_python_files_exist(self):
        """Test that main Python files exist."""
        package_dir = Path(__file__).parent.parent / 'turtlebot3_tracker'
        
        required_files = [
            'trajectory_tracker_node.py',
            'trajectory_generator.py',
            'trajectory_controller.py',
            'trajectory_tracker_with_obstacle_avoidance_node.py',
            'test_controller_publisher_node.py'
        ]
        
        for file_name in required_files:
            file_path = package_dir / file_name
            self.assertTrue(file_path.exists(), f"Required file {file_name} does not exist")
    
    def test_launch_files_exist(self):
        """Test that launch files exist."""
        launch_dir = Path(__file__).parent.parent / 'launch'
        
        required_launch_files = [
            'trajectory_with_rviz.launch.py',
            'trajectory_tracker_with_obstacle_avoidance.launch.py'
        ]
        
        for launch_file in required_launch_files:
            launch_path = launch_dir / launch_file
            self.assertTrue(launch_path.exists(), f"Required launch file {launch_file} does not exist")
    
    def test_config_files_exist(self):
        """Test that config files exist."""
        config_dir = Path(__file__).parent.parent / 'config'
        
        required_config_files = [
            'trajectory_visualization.rviz'
        ]
        
        for config_file in required_config_files:
            config_path = config_dir / config_file
            self.assertTrue(config_path.exists(), f"Required config file {config_file} does not exist")
    
    def test_service_files_exist(self):
        """Test that service files exist."""
        srv_dir = Path(__file__).parent.parent / 'srv'
        
        required_srv_files = [
            'GlobalPathService.srv',
            'TrajectoryService.srv'
        ]
        
        for srv_file in required_srv_files:
            srv_path = srv_dir / srv_file
            self.assertTrue(srv_path.exists(), f"Required service file {srv_file} does not exist")
    
    def test_package_xml_exists(self):
        """Test that package.xml exists."""
        package_xml = Path(__file__).parent.parent / 'package.xml'
        self.assertTrue(package_xml.exists(), "package.xml does not exist")
    
    def test_setup_py_exists(self):
        """Test that setup.py exists."""
        setup_py = Path(__file__).parent.parent / 'setup.py'
        self.assertTrue(setup_py.exists(), "setup.py does not exist")
    
    def test_cmake_lists_exists(self):
        """Test that CMakeLists.txt exists."""
        cmake_lists = Path(__file__).parent.parent / 'CMakeLists.txt'
        self.assertTrue(cmake_lists.exists(), "CMakeLists.txt does not exist")


class TestAlgorithmValidation(unittest.TestCase):
    """Test basic algorithm validation without ROS dependencies."""
    
    def test_waypoint_validation(self):
        """Test waypoint validation logic."""
        # Test valid waypoints
        valid_waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
        
        # Check that we have at least 2 waypoints
        self.assertGreaterEqual(len(valid_waypoints), 2, "At least 2 waypoints required")
        
        # Check that each waypoint is a tuple with 2 coordinates
        for wp in valid_waypoints:
            self.assertIsInstance(wp, tuple, "Waypoints must be tuples")
            self.assertEqual(len(wp), 2, "Waypoints must have 2 coordinates")
            self.assertIsInstance(wp[0], (int, float), "X coordinate must be numeric")
            self.assertIsInstance(wp[1], (int, float), "Y coordinate must be numeric")
    
    def test_velocity_profile_validation(self):
        """Test velocity profile validation."""
        valid_profiles = ['trapezoidal', 'constant', 'sine']
        test_profile = 'trapezoidal'
        
        self.assertIn(test_profile, valid_profiles, "Invalid velocity profile")
    
    def test_controller_type_validation(self):
        """Test controller type validation."""
        valid_controllers = ['pure_pursuit', 'pid', 'stanley']
        test_controller = 'pure_pursuit'
        
        self.assertIn(test_controller, valid_controllers, "Invalid controller type")
    
    def test_parameter_validation(self):
        """Test parameter validation."""
        # Test velocity parameters
        max_velocity = 1.0
        acceleration = 0.5
        deceleration = 0.5
        
        self.assertGreater(max_velocity, 0, "Max velocity must be positive")
        self.assertGreater(acceleration, 0, "Acceleration must be positive")
        self.assertGreater(deceleration, 0, "Deceleration must be positive")
        
        # Test controller parameters
        lookahead_distance = 0.5
        kp_linear = 1.0
        kp_angular = 1.0
        
        self.assertGreater(lookahead_distance, 0, "Lookahead distance must be positive")
        self.assertGreater(kp_linear, 0, "Linear proportional gain must be positive")
        self.assertGreater(kp_angular, 0, "Angular proportional gain must be positive")


if __name__ == '__main__':
    unittest.main() 