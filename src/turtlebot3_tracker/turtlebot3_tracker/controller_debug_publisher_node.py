#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray
import math
from .trajectory_controller import TrajectoryController
from tf_transformations import euler_from_quaternion

class ControllerDebugPublisherNode(Node):
    def __init__(self):
        super().__init__('controller_debug_publisher_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # Publisher for cmd_vel (output of PID controller)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_odom = None
        self.timer = self.create_timer(0.1, self.control_and_publish)
        # Set up the PID controller
        self.controller = TrajectoryController(
            controller_type='pid',
            kp_linear=0.4,
            ki_linear=0.0,
            kd_linear=0.05,
            kp_angular=0.7,
            ki_angular=0.0,
            kd_angular=0.2,
            max_linear_velocity=0.18,
            max_angular_velocity=0.7
        )
        # Set a fixed desired pose for demonstration
        self.desired_pose = PoseStamped()
        self.desired_pose.header.frame_id = 'map'
        self.desired_pose.pose.position.x = 2.0
        self.desired_pose.pose.position.y = 2.0
        self.desired_pose.pose.position.z = 0.0
        self.desired_pose.pose.orientation.w = 1.0
        self.target_velocity = 0.15

    def odom_callback(self, msg):
        self.last_odom = msg




    def control_and_publish(self):
        if self.last_odom is None:
            return
        # Convert odometry to PoseStamped
        current_pose = PoseStamped()
        current_pose.header = self.last_odom.header
        current_pose.pose = self.last_odom.pose.pose
        # Update controller state
        self.controller.update_current_state(current_pose, self.last_odom.twist.twist)
        # Set target
        self.controller.set_target(self.desired_pose, self.target_velocity)
        # Compute control
        current_time = self.get_clock().now().nanoseconds / 1e9
        cmd_vel = self.controller.compute_control(current_time)
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerDebugPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 