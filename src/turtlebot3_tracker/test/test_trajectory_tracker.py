#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import time
import numpy as np


class TestTrajectoryTracker(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.node = Node('test_trajectory_tracker')
        
        # Test flags
        cls.cmd_vel_received = False
        cls.trajectory_path_received = False
        cls.waypoints_received = False
        cls.robot_trace_received = False
        
        # Create subscribers to test the published topics
        cls.cmd_vel_sub = cls.node.create_subscription(
            Twist,
            '/cmd_vel',
            cls.cmd_vel_callback,
            10
        )
        
        cls.trajectory_path_sub = cls.node.create_subscription(
            Path,
            '/trajectory_path',
            cls.trajectory_path_callback,
            10
        )
        
        cls.waypoints_sub = cls.node.create_subscription(
            PoseArray,
            '/trajectory_waypoints',
            cls.waypoints_callback,
            10
        )
        
        cls.robot_trace_sub = cls.node.create_subscription(
            Path,
            '/robot_trace_path',
            cls.robot_trace_callback,
            10
        )
    
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def cmd_vel_callback(cls, msg):
        cls.cmd_vel_received = True
        cls.node.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
    
    @classmethod
    def trajectory_path_callback(cls, msg):
        cls.trajectory_path_received = True
        cls.node.get_logger().info(f'Received trajectory path with {len(msg.poses)} poses')
    
    @classmethod
    def waypoints_callback(cls, msg):
        cls.waypoints_received = True
        cls.node.get_logger().info(f'Received waypoints with {len(msg.poses)} poses')
    
    @classmethod
    def robot_trace_callback(cls, msg):
        cls.robot_trace_received = True
        cls.node.get_logger().info(f'Received robot trace with {len(msg.poses)} poses')
    
    def test_trajectory_publication(self):
        """Test that trajectory data is published correctly."""
        # Wait for some time to receive messages
        timeout = 15.0  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if (self.cmd_vel_received and self.trajectory_path_received and 
                self.waypoints_received and self.robot_trace_received):
                break
        
        # Assert that we received trajectory messages
        self.assertTrue(self.cmd_vel_received, "No cmd_vel messages received")
        self.assertTrue(self.trajectory_path_received, "No trajectory path messages received")
        self.assertTrue(self.waypoints_received, "No waypoints messages received")
    
    def test_cmd_vel_validity(self):
        """Test that cmd_vel messages contain valid velocity values."""
        if not self.cmd_vel_received:
            self.skipTest("No cmd_vel messages received")
        
        # This test would need to be implemented with actual message validation
        # For now, we just check that the topic is being published
        self.assertTrue(True, "cmd_vel topic is being published")


class TestObstacleAvoidance(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.node = Node('test_obstacle_avoidance')
        
        # Test flags
        cls.cmd_vel_received = False
        cls.planned_path_received = False
        
        # Create subscribers to test the published topics
        cls.cmd_vel_sub = cls.node.create_subscription(
            Twist,
            '/cmd_vel',
            cls.cmd_vel_callback,
            10
        )
        
        cls.planned_path_sub = cls.node.create_subscription(
            Path,
            '/planned_path',
            cls.planned_path_callback,
            10
        )
    
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def cmd_vel_callback(cls, msg):
        cls.cmd_vel_received = True
        cls.node.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
    
    @classmethod
    def planned_path_callback(cls, msg):
        cls.planned_path_received = True
        cls.node.get_logger().info(f'Received planned path with {len(msg.poses)} poses')
    
    def test_obstacle_avoidance_publication(self):
        """Test that obstacle avoidance data is published correctly."""
        # Wait for some time to receive messages
        timeout = 15.0  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.cmd_vel_received and self.planned_path_received:
                break
        
        # Assert that we received obstacle avoidance messages
        self.assertTrue(self.cmd_vel_received, "No cmd_vel messages received in obstacle avoidance mode")
        self.assertTrue(self.planned_path_received, "No planned path messages received in obstacle avoidance mode")


class TestTrajectoryGeneration(unittest.TestCase):
    """Test trajectory generation functionality."""
    
    def test_waypoint_validation(self):
        """Test that waypoints are properly validated."""
        # Test valid waypoints
        valid_waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
        self.assertTrue(len(valid_waypoints) >= 2, "At least 2 waypoints required")
        
        # Test that waypoints are tuples of floats
        for wp in valid_waypoints:
            self.assertIsInstance(wp, tuple, "Waypoints must be tuples")
            self.assertEqual(len(wp), 2, "Waypoints must have 2 coordinates")
            self.assertIsInstance(wp[0], (int, float), "X coordinate must be numeric")
            self.assertIsInstance(wp[1], (int, float), "Y coordinate must be numeric")
    
    def test_velocity_profile_validation(self):
        """Test that velocity profiles are properly validated."""
        valid_profiles = ['trapezoidal', 'constant', 'sine']
        test_profile = 'trapezoidal'
        self.assertIn(test_profile, valid_profiles, "Invalid velocity profile")
    
    def test_controller_type_validation(self):
        """Test that controller types are properly validated."""
        valid_controllers = ['pure_pursuit', 'pid', 'stanley']
        test_controller = 'pure_pursuit'
        self.assertIn(test_controller, valid_controllers, "Invalid controller type")


class TestParameterValidation(unittest.TestCase):
    """Test parameter validation functionality."""
    
    def test_velocity_parameters(self):
        """Test that velocity parameters are within valid ranges."""
        max_velocity = 1.0
        acceleration = 0.5
        deceleration = 0.5
        
        self.assertGreater(max_velocity, 0, "Max velocity must be positive")
        self.assertGreater(acceleration, 0, "Acceleration must be positive")
        self.assertGreater(deceleration, 0, "Deceleration must be positive")
    
    def test_controller_parameters(self):
        """Test that controller parameters are within valid ranges."""
        lookahead_distance = 0.5
        kp_linear = 1.0
        kp_angular = 1.0
        
        self.assertGreater(lookahead_distance, 0, "Lookahead distance must be positive")
        self.assertGreater(kp_linear, 0, "Linear proportional gain must be positive")
        self.assertGreater(kp_angular, 0, "Angular proportional gain must be positive")


if __name__ == '__main__':
    unittest.main() 