#!/usr/bin/env python3

import numpy as np
import math
import time
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# Import our custom modules
from .trajectory_generator import TrajectoryGenerator
from .trajectory_controller import TrajectoryController


class TrajectoryPublisherNode(Node):
    """
    A ROS2 node that generates smooth trajectories from waypoints and implements
    trajectory tracking control for a differential drive robot.
    
    This node:
    1. Generates smooth paths from waypoints using cubic spline interpolation
    2. Creates time-parameterized trajectories with different velocity profiles
    3. Implements trajectory tracking using various control algorithms
    4. Publishes trajectory visualization and control commands
    """
    
    def __init__(self):
        super().__init__('trajectory_publisher_node')
        
        # Declare parameters
        self.declare_custom_parameters()
        
        # Initialize trajectory generator and controller
        self.trajectory_generator = TrajectoryGenerator()
        self.controller = self._create_controller()
        
        # Trajectory data
        self.waypoints = []
        self.trajectory_data = None
        self.current_trajectory_index = 0
        self.trajectory_start_time = None
        self.is_trajectory_active = False
        self.previous_target_index = 0
        
        # Robot state
        self.current_pose = None
        self.current_velocity = None
        self.robot_frame = "base_link"
        self.world_frame = "map"
        
        # Setup QoS profiles
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup timers
        self.setup_timers()
        
        # Load default waypoints if specified
        self.load_default_waypoints()
        
        self.robot_trace_path = []  # Store the actual path taken by the robot
        
        self.going_to_first_waypoint = False  # Flag for go-to-first-waypoint mode
        self.trajectory_completed = False  # Flag for completed trajectory
        
        self.get_logger().info("Trajectory Publisher Node initialized")
    
    def declare_custom_parameters(self):
        """Declare ROS parameters for the node."""
        # Trajectory generation parameters
        self.declare_parameter('velocity_profile', 'trapezoidal')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('acceleration', 0.5)
        self.declare_parameter('deceleration', 0.5)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('num_smooth_points', 100)
        
        # Controller parameters
        self.declare_parameter('controller_type', 'pid')
        self.declare_parameter('lookahead_distance', 0.2)
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.0)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.0)
        self.declare_parameter('k_e', 1.0)  # Stanley controller
        self.declare_parameter('k_v', 1.0)  # Stanley controller
        
        # Control loop parameters
        self.declare_parameter('control_frequency', 20.0)  # Hz
        self.declare_parameter('goal_tolerance', 0.1)  # meters
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        
        # Visualization parameters
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('visualization_frequency', 1.0)  # Hz
        
        # Default waypoints (can be overridden via parameters or services)
        self.declare_parameter('default_waypoints', [0.0, 0.0, 1.0, 1.0, 2.0, 0.0, 3.0, 1.0, 4.0, 0.0])
    
    def _create_controller(self) -> TrajectoryController:
        """Create and configure the trajectory controller."""
        controller_type = self.get_parameter('controller_type').value
        
        if controller_type == 'pure_pursuit':
            return TrajectoryController(
                controller_type='pure_pursuit',
                lookahead_distance= self.get_parameter('lookahead_distance').value,
                max_linear_velocity=self.get_parameter('max_linear_velocity').value,
                max_angular_velocity=self.get_parameter('max_angular_velocity').value
            )
        elif controller_type == 'pid':
            return TrajectoryController(
                controller_type='pid',
                kp_linear=self.get_parameter('kp_linear').value,
                ki_linear=self.get_parameter('ki_linear').value,
                kd_linear=self.get_parameter('kd_linear').value,
                kp_angular=self.get_parameter('kp_angular').value,
                ki_angular=self.get_parameter('ki_angular').value,
                kd_angular=self.get_parameter('kd_angular').value,
                max_linear_velocity=self.get_parameter('max_linear_velocity').value,
                max_angular_velocity=self.get_parameter('max_angular_velocity').value
            )
        elif controller_type == 'stanley':
            return TrajectoryController(
                controller_type='stanley',
                k_e=self.get_parameter('k_e').value,
                k_v=self.get_parameter('k_v').value,
                max_linear_velocity=self.get_parameter('max_linear_velocity').value,
                max_angular_velocity=self.get_parameter('max_angular_velocity').value
            )
        else:
            self.get_logger().warn(f"Unknown controller type: {controller_type}, using pure_pursuit")
            return TrajectoryController(controller_type='pure_pursuit')
    
    def setup_publishers(self):
        """Setup ROS publishers."""
        # Control command publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # Trajectory visualization publishers
        if self.get_parameter('publish_visualization').value:
            self.path_pub = self.create_publisher(
                Path, 
                '/trajectory_path', 
                10
            )
            
            self.waypoints_pub = self.create_publisher(
                PoseArray, 
                '/trajectory_waypoints', 
                10
            )
            
            self.marker_pub = self.create_publisher(
                MarkerArray, 
                '/trajectory_markers', 
                10
            )
            # Publisher for the actual robot trace path
            self.trace_path_pub = self.create_publisher(
                Path,
                '/robot_trace_path',
                10
            )
    
    def setup_subscribers(self):
        """Setup ROS subscribers."""
        # Robot pose and velocity
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Optional: Subscribe to laser scan for obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
    
    def setup_timers(self):
        """Setup ROS timers."""
        # Control loop timer
        control_frequency = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(
            1.0 / control_frequency,
            self.control_loop
        )
        
        # Visualization timer
        if self.get_parameter('publish_visualization').value:
            viz_frequency = self.get_parameter('visualization_frequency').value
            self.viz_timer = self.create_timer(
                1.0 / viz_frequency,
                self.publish_visualization
            )
    
    def load_default_waypoints(self):
        """Load default waypoints from parameters."""
        default_waypoints = self.get_parameter('default_waypoints').value
        if default_waypoints and len(default_waypoints) >= 2:
            # Convert flattened list [x1, y1, x2, y2, ...] to list of tuples [(x1, y1), (x2, y2), ...]
            self.waypoints = []
            for i in range(0, len(default_waypoints), 2):
                if i + 1 < len(default_waypoints):
                    self.waypoints.append((default_waypoints[i], default_waypoints[i + 1]))
            self.get_logger().info(f"Loaded {len(self.waypoints)} default waypoints")
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """
        Set waypoints and generate trajectory.
        
        Args:
            waypoints: List of 2D waypoints [(x0, y0), (x1, y1), ..., (xn, yn)]
        """
        if len(waypoints) < 2:
            self.get_logger().error("At least 2 waypoints are required")
            return False
        
        self.waypoints = waypoints
        self.get_logger().info(f"Set {len(waypoints)} waypoints")
        
        self.trajectory_completed = False  # Reset completed flag when new waypoints are set
        # Generate trajectory
        return self.generate_trajectory()
    
    def generate_trajectory(self) -> bool:
        """
        Generate smooth trajectory from waypoints.
        
        Returns:
            True if trajectory generation was successful
        """
        try:
            # Set waypoints in trajectory generator
            self.trajectory_generator.set_waypoints(self.waypoints)
            
            # Smooth the path
            num_points = self.get_parameter('num_smooth_points').value
            self.trajectory_generator.smooth_path_spline(num_points)
            
            # Generate time-parameterized trajectory
            velocity_profile = self.get_parameter('velocity_profile').value
            max_velocity = self.get_parameter('max_velocity').value
            acceleration = self.get_parameter('acceleration').value
            deceleration = self.get_parameter('deceleration').value
            dt = self.get_parameter('dt').value
            
            self.trajectory_data = self.trajectory_generator.generate_time_parameterized_trajectory(
                velocity_profile=velocity_profile,
                max_velocity=max_velocity,
                acceleration=acceleration,
                deceleration=deceleration,
                dt=dt
            )
            # Set the path for the controller (for pure pursuit)
            self.controller.set_path(self.trajectory_data['x_traj'], self.trajectory_data['y_traj'])
            
            self.trajectory_completed = False  # Reset completed flag when new trajectory is generated
            self.get_logger().info("Trajectory generated successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate trajectory: {str(e)}")
            return False
    
    def start_trajectory_tracking(self):
        """Start trajectory tracking."""
        if self.trajectory_completed:
            self.get_logger().info("Trajectory already completed. Not starting tracking again.")
            return False
        if self.trajectory_data is None:
            self.get_logger().error("No trajectory available. Generate trajectory first.")
            return False
        
        if self.current_pose is None:
            self.get_logger().error("No robot pose available. Cannot start tracking.")
            return False
        
        self.current_trajectory_index = 0
        self.trajectory_start_time = self.get_clock().now().nanoseconds / 1e9
        self.is_trajectory_active = True
        self.controller.reset()
        self.robot_trace_path = []  # Clear the trace when starting a new trajectory
        
        self.get_logger().info("Started trajectory tracking")
        return True
    
    def stop_trajectory_tracking(self):
        """Stop trajectory tracking."""
        self.is_trajectory_active = False
        self.publish_stop_command()
        self.trajectory_completed = True
        self.get_logger().info("Stopped trajectory tracking")
    
    def _is_at_first_waypoint(self) -> bool:
        """Check if the robot is at the first waypoint."""
        if self.current_pose is None or not self.waypoints:
            return False
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        wx, wy = self.waypoints[0]
        distance = math.sqrt((x - wx) ** 2 + (y - wy) ** 2)
        goal_tolerance = self.get_parameter('goal_tolerance').value
        return distance < goal_tolerance

    def go_to_first_waypoint(self):
        """Drive the robot to the first waypoint using a simple proportional controller."""
        if self.current_pose is None or not self.waypoints:
            return

        wx, wy = self.waypoints[0]
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y

        dx = wx - x
        dy = wy - y
        distance = math.sqrt(dx**2 + dy**2)
        goal_tolerance = self.get_parameter('goal_tolerance').value

        # Calculate angle to waypoint
        yaw = self._get_yaw_from_pose(self.current_pose.pose)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self._normalize_angle(angle_to_goal - yaw)

        # Simple proportional controller
        linear_speed = min(0.3, distance)  # Cap speed
        angular_speed = min(1.0, max(-1.0, 2.0 * angle_diff))  # Cap angular speed

        cmd = Twist()
        if distance > goal_tolerance:
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
        else:
            # Arrived at first waypoint
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.going_to_first_waypoint = False
            self.get_logger().info("Arrived at first waypoint, starting trajectory tracking...")
            

            self.start_trajectory_tracking()

        self.cmd_vel_pub.publish(cmd)

    def _get_yaw_from_pose(self, pose):
        """Extract yaw from a Pose message."""
        import math
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        # Yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        import math
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        # Update current pose
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        
        # Update current velocity
        self.current_velocity = msg.twist.twist
        
        # Update controller state
        self.controller.update_current_state(self.current_pose, self.current_velocity)
        
        # Store the robot's path for trace visualization
        if self.current_pose is not None:
            pose_copy = PoseStamped()
            pose_copy.header = self.current_pose.header
            pose_copy.pose = self.current_pose.pose
            self.robot_trace_path.append(pose_copy)
        
        # Auto-generate trajectory if we have waypoints but no trajectory
        if self.waypoints and self.trajectory_data is None:
            self.get_logger().info("Received first odometry, generating trajectory...")
            self.generate_trajectory()
        
        # If trajectory is ready, not active, and not completed, go to first waypoint if needed
        if self.trajectory_data is not None and not self.is_trajectory_active and not self.trajectory_completed:
            if not self._is_at_first_waypoint():
                if not self.going_to_first_waypoint:
                    self.get_logger().info("Robot not at first waypoint, driving to it...")
                    self.going_to_first_waypoint = True
                self.go_to_first_waypoint()

            else:
                if self.going_to_first_waypoint:
                    self.going_to_first_waypoint = False
                self.get_logger().info("Wait for few seconds to start trajectory tracking")
                time.sleep(3)

                self.get_logger().info("Robot reached first waypoint, starting trajectory tracking...")
                self.start_trajectory_tracking()
    
    def scan_callback(self, msg: LaserScan):
        """Callback for laser scan messages (for future obstacle avoidance)."""
        # TODO: Implement obstacle avoidance logic
        pass
    
    def control_loop(self):
        """Main control loop for trajectory tracking."""
        if not self.is_trajectory_active or self.trajectory_data is None:
            return
        
        if self.current_pose is None:
            return
        
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        trajectory_time = current_time - self.trajectory_start_time
        
        # Find current target point in trajectory
        target_index = self._find_target_index(trajectory_time)

        if target_index % 20 == 0 and target_index != self.previous_target_index: # Log only once every 20 target indices
            self.get_logger().info(f"Target index: {target_index}, / {len(self.trajectory_data['time'])-1}")
            self.previous_target_index = target_index

        
        if target_index >= len(self.trajectory_data['time'])-1:
            # Trajectory completed
            self.get_logger().info("Trajectory completed")
            self.stop_trajectory_tracking()
            self.trajectory_completed = True
            return
        
        # Set target for controller
        target_pose = self._create_target_pose(target_index)
        target_velocity = self.trajectory_data['v_traj'][target_index]
        
        self.controller.set_target(target_pose, target_velocity)
        
        # Compute control command
        control_cmd = self.controller.compute_control(current_time)
        
        # Publish control command
        self.cmd_vel_pub.publish(control_cmd)

        # self.get_logger().info("Controller Type: " + self.controller.controller_type)
        # self.get_logger().info(f"Linear velocity: {control_cmd.linear.x}, Angular velocity: {control_cmd.angular.z}")

        
        # Check if goal is reached
        if self._is_goal_reached(target_pose):
            self.current_trajectory_index = target_index + 1
    
    def _find_target_index(self, trajectory_time: float) -> int:
        """Find the target index in the trajectory based on time."""
        time_array = self.trajectory_data['time']
        
        # Find the closest time index
        if trajectory_time <= time_array[0]:
            return 0
        elif trajectory_time >= time_array[-1]:
            return len(time_array) - 1
        else:
            # Binary search for the closest time
            left, right = 0, len(time_array) - 1
            while left < right:
                mid = (left + right) // 2
                if time_array[mid] < trajectory_time:
                    left = mid + 1
                else:
                    right = mid
            return left
    
    def _create_target_pose(self, index: int) -> PoseStamped:
        """Create a target pose from trajectory data."""
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.world_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        target_pose.pose.position.x = self.trajectory_data['x_traj'][index]
        target_pose.pose.position.y = self.trajectory_data['y_traj'][index]
        target_pose.pose.position.z = 0.0
        
        # Calculate orientation (tangent to the path)
        if index < len(self.trajectory_data['x_traj']) - 1:
            dx = self.trajectory_data['x_traj'][index + 1] - self.trajectory_data['x_traj'][index]
            dy = self.trajectory_data['y_traj'][index + 1] - self.trajectory_data['y_traj'][index]
            yaw = math.atan2(dy, dx)
        else:
            # For the last point, use the previous orientation
            dx = self.trajectory_data['x_traj'][index] - self.trajectory_data['x_traj'][index - 1]
            dy = self.trajectory_data['y_traj'][index] - self.trajectory_data['y_traj'][index - 1]
            yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return target_pose
    
    def _is_goal_reached(self, target_pose: PoseStamped) -> bool:
        """Check if the robot has reached the goal."""
        if self.current_pose is None:
            return False
        
        # Calculate distance to target
        dx = target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        goal_tolerance = self.get_parameter('goal_tolerance').value
        return distance < goal_tolerance
    
    def publish_stop_command(self):
        """Publish stop command to the robot."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
    
    def publish_visualization(self):
        """Publish trajectory visualization."""
        if self.trajectory_data is None:
            return
        
        # Publish path
        if hasattr(self, 'path_pub'):
            path_msg = Path()
            path_msg.header.frame_id = self.world_frame
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for i in range(len(self.trajectory_data['x_traj'])):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = self.trajectory_data['x_traj'][i]
                pose.pose.position.y = self.trajectory_data['y_traj'][i]
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
        
        # Publish the actual robot trace path
        if hasattr(self, 'trace_path_pub') and self.robot_trace_path:
            trace_path_msg = Path()
            trace_path_msg.header.frame_id = self.world_frame
            trace_path_msg.header.stamp = self.get_clock().now().to_msg()
            trace_path_msg.poses = self.robot_trace_path
            self.trace_path_pub.publish(trace_path_msg)
        
        # Publish waypoints
        if hasattr(self, 'waypoints_pub') and self.waypoints:
            waypoints_msg = PoseArray()
            waypoints_msg.header.frame_id = self.world_frame
            waypoints_msg.header.stamp = self.get_clock().now().to_msg()
            
            for x, y in self.waypoints:
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                waypoints_msg.poses.append(pose.pose)
            
            self.waypoints_pub.publish(waypoints_msg)
        
        # Publish markers
        if hasattr(self, 'marker_pub'):
            marker_array = MarkerArray()
            
            # Current robot position marker
            if self.current_pose:
                robot_marker = Marker()
                robot_marker.header.frame_id = self.world_frame
                robot_marker.header.stamp = self.get_clock().now().to_msg()
                robot_marker.ns = "robot"
                robot_marker.id = 0
                robot_marker.type = Marker.CYLINDER
                robot_marker.action = Marker.ADD
                robot_marker.pose = self.current_pose.pose
                robot_marker.scale.x = 0.3
                robot_marker.scale.y = 0.3
                robot_marker.scale.z = 0.1
                robot_marker.color.r = 0.0
                robot_marker.color.g = 1.0
                robot_marker.color.b = 0.0
                robot_marker.color.a = 1.0
                marker_array.markers.append(robot_marker)
            
            # Target position marker
            if self.is_trajectory_active and self.trajectory_data:
                target_index = min(self.current_trajectory_index, len(self.trajectory_data['x_traj']) - 1)
                target_marker = Marker()
                target_marker.header.frame_id = self.world_frame
                target_marker.header.stamp = self.get_clock().now().to_msg()
                target_marker.ns = "target"
                target_marker.id = 1
                target_marker.type = Marker.SPHERE
                target_marker.action = Marker.ADD
                target_marker.pose.position.x = self.trajectory_data['x_traj'][target_index]
                target_marker.pose.position.y = self.trajectory_data['y_traj'][target_index]
                target_marker.pose.position.z = 0.0
                target_marker.scale.x = 0.2
                target_marker.scale.y = 0.2
                target_marker.scale.z = 0.2
                target_marker.color.r = 1.0
                target_marker.color.g = 0.0
                target_marker.color.b = 0.0
                target_marker.color.a = 1.0
                marker_array.markers.append(target_marker)

            # Lookahead point marker (for pure pursuit visualization)
            if self.controller.controller_type == 'pure_pursuit' and self.controller.path_x is not None and self.controller.path_y is not None:
                lookahead_index = getattr(self.controller, 'current_path_index', None)
                if lookahead_index is not None and 0 <= lookahead_index < len(self.controller.path_x):
                    lookahead_marker = Marker()
                    lookahead_marker.header.frame_id = self.world_frame
                    lookahead_marker.header.stamp = self.get_clock().now().to_msg()
                    lookahead_marker.ns = "lookahead"
                    lookahead_marker.id = 2
                    lookahead_marker.type = Marker.CUBE
                    lookahead_marker.action = Marker.ADD
                    lookahead_marker.pose.position.x = self.controller.path_x[lookahead_index]
                    lookahead_marker.pose.position.y = self.controller.path_y[lookahead_index]
                    lookahead_marker.pose.position.z = 0.0
                    lookahead_marker.scale.x = 0.18
                    lookahead_marker.scale.y = 0.18
                    lookahead_marker.scale.z = 0.18
                    lookahead_marker.color.r = 0.0
                    lookahead_marker.color.g = 0.3
                    lookahead_marker.color.b = 1.0
                    lookahead_marker.color.a = 1.0
                    marker_array.markers.append(lookahead_marker)
            
            self.marker_pub.publish(marker_array)
    
    def get_trajectory_info(self) -> dict:
        """Get information about the current trajectory."""
        info = {
            'waypoints_count': len(self.waypoints),
            'trajectory_active': self.is_trajectory_active,
            'controller_type': self.controller.controller_type,
            'current_pose': self.current_pose is not None
        }
        
        if self.trajectory_data:
            info.update({
                'trajectory_duration': self.trajectory_data['time'][-1],
                'trajectory_points': len(self.trajectory_data['time']),
                'max_velocity': np.max(self.trajectory_data['v_traj']),
                'total_distance': self._calculate_total_distance()
            })
        
        return info
    
    def _calculate_total_distance(self) -> float:
        """Calculate total distance of the trajectory."""
        if self.trajectory_data is None:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(self.trajectory_data['x_traj'])):
            dx = self.trajectory_data['x_traj'][i] - self.trajectory_data['x_traj'][i-1]
            dy = self.trajectory_data['y_traj'][i] - self.trajectory_data['y_traj'][i-1]
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        return total_distance


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = TrajectoryPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down trajectory publisher node")
    finally:
        node.stop_trajectory_tracking()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
