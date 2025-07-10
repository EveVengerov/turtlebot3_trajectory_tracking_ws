#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import time


class TestTrajectoryPublisher(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_trajectory_publisher')
        
        # Create subscribers to test the published topics
        cls.cmd_vel_received = False
        cls.path_received = False
        
        cls.cmd_vel_sub = cls.node.create_subscription(
            Twist,
            '/cmd_vel',
            cls.cmd_vel_callback,
            10
        )
        
        cls.path_sub = cls.node.create_subscription(
            Path,
            '/trajectory_path',
            cls.path_callback,
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
    def path_callback(cls, msg):
        cls.path_received = True
        cls.node.get_logger().info(f'Received path with {len(msg.poses)} poses')
    
    def test_trajectory_publication(self):
        """Test that trajectory data is published correctly."""
        # Wait for some time to receive messages
        timeout = 10.0  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.cmd_vel_received and self.path_received:
                break
        
        # Assert that we received both types of messages
        self.assertTrue(self.cmd_vel_received, "No cmd_vel messages received")
        self.assertTrue(self.path_received, "No path messages received")


if __name__ == '__main__':
    unittest.main() 