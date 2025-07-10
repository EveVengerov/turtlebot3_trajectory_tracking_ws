# VFH : Vector field Potential Inspired Histogram window obstacle avoidance with occupancy map to account for safety margi/robot radius passing through cluttered environment.

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from typing import Tuple, List, Optional
from tf_transformations import euler_from_quaternion

# Define a colormap (e.g., "viridis" or "plasma")
colormap = cm.get_cmap('viridis')

# Constants
SCAN_ANGLE = 360  # Angle covered by the LIDAR (in degrees)
NUM_SECTORS = 36  # Number of sectors in the histogram
MIN_DISTANCE = 1.0  # Minimum distance threshold (in meters)
MAX_VELOCITY = 0.15  # Maximum linear velocity (m/s)
RANGE_MAX = 3.5

ROBOT_RADIUS = 0.2
SAFETY_MARGIN = 0.5

class HistogramWindowObstacleAvoidance:
    """
    A class that implements histogram-based obstacle avoidance for mobile robots.
    Takes laser scan data and waypoints/trajectories and returns the best steering index/updated waypoint.
    """
    
    def __init__(self, robot_radius: float = ROBOT_RADIUS, safety_margin: float = SAFETY_MARGIN,
                 num_sectors: int = NUM_SECTORS, scan_angle: float = SCAN_ANGLE, range_max: float = RANGE_MAX,
                 logger=None):
        """
        Initialize the HistogramWindowObstacleAvoidance class.
        
        Args:
            robot_radius: Radius of the robot in meters
            safety_margin: Additional safety margin in meters
            num_sectors: Number of sectors in the histogram
            scan_angle: Total scan angle in degrees
            range_max: Maximum range for the sensor
            logger: ROS 2 logger instance for logging
        """
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.num_sectors = num_sectors
        self.scan_angle = scan_angle
        self.range_max = range_max
        self.effective_radius = robot_radius + safety_margin
        self.logger = logger
        
        # Initialize occupancy grid
        self.occupancy_grid = None
        self.grid_size = 2.0  # 2.0 meters around the robot center
        self.resolution = 0.15  # Cell size (0.15 meters)
        
        # Previous steering state for hysteresis
        self.prev_steering_idx = 0
        self.prev_steering_direction = "STRAIGHT"
    
    def _log(self, level: str, message: str):
        """Safely log messages if logger is available."""
        if self.logger is not None:
            if level == "debug":
                self.logger.debug(message)
            elif level == "info":
                self.logger.info(message)
            elif level == "warn":
                self.logger.warn(message)
    
    def update_occupancy_grid(self, scan_data: List[float]) -> None:
        """
        Update the occupancy grid from laser scan data.
        
        Args:
            scan_data: Array of distance measurements from LIDAR
        """
        # Preprocess scan data
        scan_data = [min(r, self.range_max) if r != float('inf') else self.range_max for r in scan_data]
        
        # Convert scan data to Cartesian coordinates
        angles = np.linspace(0, -self.scan_angle, len(scan_data))
        x = np.array(scan_data) * np.sin(np.deg2rad(angles))
        y = np.array(scan_data) * np.cos(np.deg2rad(angles))
        
        # Define the grid
        grid_size_cells = int(self.grid_size / self.resolution)
        grid = np.zeros((grid_size_cells, grid_size_cells))
        
        # Convert the x, y coordinates to grid indices
        for i in range(len(x)):
            # Convert (x, y) to grid cell index
            grid_x = int((x[i] + self.grid_size / 2) / self.resolution)
            grid_y = int((y[i] + self.grid_size / 2) / self.resolution)
            
            # Ensure indices are within grid bounds
            if 0 <= grid_x < grid.shape[0] and 0 <= grid_y < grid.shape[1]:
                grid[grid_x, grid_y] = 1
                # Mark neighboring cells within a radius of 1 cell
                for dx in range(-1, 2):  # -1, 0, 1
                    for dy in range(-1, 2):  # -1, 0, 1
                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < grid_size_cells and 0 <= ny < grid_size_cells:
                            grid[nx, ny] = 1
        
        # Update occupancy grid with temporal filtering
        if self.occupancy_grid is not None and np.any(self.occupancy_grid):
            for nx in range(grid_size_cells):
                for ny in range(grid_size_cells):
                    self.occupancy_grid[nx, ny] = (self.occupancy_grid[nx, ny] * 0.9 + grid[nx, ny]) / 1.9
        else:
            self.occupancy_grid = grid
    
    def create_polar_histogram(self) -> np.ndarray:
        """
        Create a polar histogram from the occupancy grid.
        
        Returns:
            Histogram array representing distances in each sector
        """
        if self.occupancy_grid is None:
            return np.ones(self.num_sectors) * self.range_max
            
        histogram = np.ones(self.num_sectors) * self.range_max
        
        # Loop through all grid cells and update histogram for occupied cells
        for occupancy_grid_x in range(self.occupancy_grid.shape[0]):
            for occupancy_grid_y in range(self.occupancy_grid.shape[1]):
                if self.occupancy_grid[occupancy_grid_x, occupancy_grid_y] > 0.25:  # Only consider occupied cells
                    # Calculate the distance from the robot (center of the occupancy_grid)
                    dist = np.sqrt((occupancy_grid_x - self.occupancy_grid.shape[0] // 2) ** 2 + 
                                 (occupancy_grid_y - self.occupancy_grid.shape[1] // 2) ** 2) * self.resolution
                    
                    # Calculate the angle for the current cell
                    angle = np.arctan2(occupancy_grid_x - self.occupancy_grid.shape[0] // 2, 
                                     occupancy_grid_y - self.occupancy_grid.shape[1] // 2)
                    
                    # Wrap angle to the range [0, 2π]
                    angle = (angle + 2 * np.pi) % (2 * np.pi)
                    
                    # Convert angle to bin index
                    angle_idx = int(angle / (2 * np.pi) * self.num_sectors)
                    
                    histogram[angle_idx] = min(histogram[angle_idx], dist - self.resolution)
        
        # flip the histogram
        histogram = histogram[::-1]
        return histogram
    
    def find_best_steering_direction(self, target_theta_base_link: float) -> Tuple[int, float, str]:
        """
        Find the best steering direction based on histogram data and target direction in base_link frame.
        
        Args:
            target_theta: Target direction in radians
            
        Returns:
            Tuple of (best_steering_index, best_steering_angle, steering_direction)
        """
        target_theta = target_theta_base_link  

        # Create polar histogram
        histogram = self.create_polar_histogram()
        # Log the histogram for debugging
        # print(f"Polar histogram: {[round(h, 2) for h in histogram]}")
        
        # Mask unsafe sectors
        valid_sectors = histogram >= self.effective_radius
        
        # Create a mask for clear sectors with at least three consecutive valid sectors
        clear_sectors = np.zeros_like(valid_sectors, dtype=bool)
        
        for i in range(len(valid_sectors)):
            # Check the current sector and its two consecutive neighbors
            if (valid_sectors[i] and 
                valid_sectors[(i - 1) % len(valid_sectors)] and 
                valid_sectors[(i + 1) % len(valid_sectors)]):
                clear_sectors[i] = True
        
        valid_sectors = clear_sectors
        
        # Warn if no valid sectors are found
        if not np.any(valid_sectors):
            print("No valid sectors available for steering.")
            return 0, 0.0, "STOP"  # Stop robot
        
        # Convert target_theta to index
        target_theta_unwrapped = (target_theta_base_link + 2 * np.pi) % (2 * np.pi)
        target_idx = int(np.rad2deg(target_theta_unwrapped) / (self.scan_angle / self.num_sectors))

        if target_idx < self.num_sectors/2:
            print(f"[turtlebot3_histogram_window_controller] Target index is {target_idx} Direction is LEFT, histogram value is {histogram[target_idx]}")
        else:
            print(f"[turtlebot3_histogram_window_controller] Target index is {target_idx} Direction is RIGHT, histogram value is {histogram[target_idx]}")
        
        best_steering_idx = target_idx
        best_steering_angle = np.deg2rad(target_idx * (self.scan_angle / self.num_sectors))
        
        # Check if the target direction is valid
        if valid_sectors[target_idx]:
            best_steering_idx = target_idx
            best_steering_angle = np.deg2rad(target_idx * (self.scan_angle / self.num_sectors))
            best_steering_angle = target_theta_base_link
            steering_direction = "TOWARD TARGET"  # Target direction is valid
        else:
            # Find closest valid direction to both sides of target
            valid_indices = np.where(valid_sectors)[0]  # all open sectors from 0 to 360 degrees 
            # find valid sectors 0 to 180 degrees, all indices upto total number of sectors/2
            n = len(histogram)
            valid_indices_left = valid_indices[valid_indices <= n/2 ]  # Higher indices = LEFT
            valid_indices_right = valid_indices[valid_indices > n/2 ]  # Lower indices = RIGHT
            
            # Calculate distances to closest valid sectors on both sides
            left_distance = float('inf')
            right_distance = float('inf')
            closest_left_idx = None
            closest_right_idx = None
            
            # Find closest valid sector to the left (higher indices)
            if len(valid_indices_left) > 0:
                closest_left_idx = valid_indices_left[np.argmin(valid_indices_left - target_idx)]
                left_distance = closest_left_idx - target_idx
            
            # Find closest valid sector to the right (lower indices)
            if len(valid_indices_right) > 0:
                closest_right_idx = valid_indices_right[np.argmin(target_idx - valid_indices_right)]
                right_distance = n - closest_right_idx - target_idx
            
            # # Print distance values for debugging
            # print(f"[turtlebot3_histogram_window_controller] Left distance: {left_distance:.2f} (closest_left_idx: {closest_left_idx})")
            # print(f"[turtlebot3_histogram_window_controller] Right distance: {right_distance:.2f} (closest_right_idx: {closest_right_idx})")
            # print(f"[turtlebot3_histogram_window_controller] N: {n}")
            # print(f"[turtlebot3_histogram_window_controller] threshold for obstacle avoidance: {self.effective_radius}")
            
            # If no sectors found on either side, wrap around
            if closest_left_idx is None and closest_right_idx is None:
                raise ValueError("No valid sectors found on either side of the target")
            else:
                # Choose the closer direction
                if left_distance <= right_distance and closest_left_idx is not None:
                    closest_idx = closest_left_idx
                    steering_direction = "LEFT"  # Higher index = LEFT
                    print(f"[turtlebot3_histogram_window_controller] Choosing LEFT: left_distance={left_distance:.2f} <= right_distance={right_distance:.2f}")
                elif closest_right_idx is not None:
                    closest_idx = closest_right_idx
                    steering_direction = "RIGHT"  # Lower index = RIGHT
                    print(f"[turtlebot3_histogram_window_controller] Choosing RIGHT: right_distance={right_distance:.2f} < left_distance={left_distance:.2f}")
                else:
                    # Fallback to left if right is not available
                    closest_idx = closest_left_idx
                    steering_direction = "LEFT"  # Higher index = LEFT
                    print(f"Fallback to LEFT: only left available")
            
            # Ensure closest_idx is not None (fallback to target_idx if needed)
            if closest_idx is None:
                closest_idx = target_idx
                steering_direction = "STRAIGHT"
            
            best_steering_idx = closest_idx
            best_steering_angle = np.deg2rad(closest_idx * (self.scan_angle / self.num_sectors))

            # Unwrap to [-pi, pi]
            if best_steering_angle > np.pi:
                best_steering_angle = best_steering_angle - 2 * np.pi
        
        return best_steering_idx, best_steering_angle, steering_direction
    
    def get_updated_waypoint(self, scan_data: List[float], current_pose: Tuple[float, float, float], 
                           target_waypoint: Tuple[float, float]) -> Tuple[float, float]:
        """
        Get an updated waypoint that avoids obstacles.
        
        Args:
            scan_data: Laser scan data
            current_pose: Current robot pose (x, y, theta)
            target_waypoint: Target waypoint (x, y)
            
        Returns:
            Updated waypoint (x, y) that avoids obstacles
        """
        # Update occupancy grid
        self.update_occupancy_grid(scan_data)
        
        # Calculate target direction
        current_x, current_y, current_theta = current_pose
        target_x, target_y = target_waypoint
        
        dx = target_x - current_x
        dy = target_y - current_y
        target_theta = math.atan2(dy, dx)
        
        # Calculate angular error
        angular_error = target_theta 
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))  # Normalize to [-pi, pi]
        
        # Find best steering direction
        best_steering_idx, best_steering_angle, steering_direction = self.find_best_steering_direction(angular_error)
        
        # Calculate updated waypoint based on best steering direction
        # Use a fixed distance for the updated waypoint
        waypoint_distance = 0.5  # meters
        
        # Calculate the updated waypoint in the best steering direction
        updated_x = current_x + waypoint_distance * math.cos(current_theta + best_steering_angle)
        updated_y = current_y + waypoint_distance * math.sin(current_theta + best_steering_angle)
        
        return updated_x, updated_y
    
    def get_steering_command(self, scan_data: List[float], target_theta: float) -> Tuple[int, float, str]:
        """
        Get steering command based on scan data and target direction.
        
        Args:
            scan_data: Laser scan data
            target_theta: Target direction in radians
            
        Returns:
            Tuple of (steering_index, steering_angle, steering_direction)
        """
        # Update occupancy grid
        self.update_occupancy_grid(scan_data)
        
        # Find best steering direction
        best_steering_idx, best_steering_angle, steering_direction = self.find_best_steering_direction(target_theta)
        
        return best_steering_idx, best_steering_angle, steering_direction

class TurtleBotHistogramWindowController(Node):
    def __init__(self):
        super().__init__('turtlebot_histogram_window_controller')

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Visualization publishers
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/current_waypoint', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/target_marker', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, '/waypoint_marker', 10)
        self.robot_pose_marker_pub = self.create_publisher(Marker, '/robot_pose_marker', 10)

        # LIDAR subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Odometry subscription
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Initialize the obstacle avoidance class
        self.obstacle_avoidance = HistogramWindowObstacleAvoidance(logger=self.get_logger())

        # Control Parameters
        self.kp_linear = 0.5  # Linear velocity proportional gain
        self.kp_angular = 2.0  # Angular velocity proportional gain

        # Target position (for P-controller) - set to a reasonable default
        self.target_x = 4.0  # 4 meters forward
        self.target_y = 0.0  # No lateral offset

        # Debug mode - bypass obstacle avoidance for testing
        self.debug_mode = False  # Set to False to enable obstacle avoidance

        # Counter for printing occupancy map
        self.print_counter = 0
        self.print_frequency = 50  # Print every 50 iterations
        self.print_occupancy_map = True  # Set to False to disable printing

        self.state = "target_following"

        # Initialize current pose
        self.current_pose = {'x': 0, 'y': 0, 'theta': 0}
        self.scan_data = None
        self.odom_data = None

        # Robot path tracking
        self.robot_path_poses = [] # Changed from self.robot_path
        self.path_counter = 0

        # Initialize interactive plotting
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 2, figsize=(12, 6))
        
        # Timer for visualization updates
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)  # 10 Hz

    def set_target_position(self, x: float, y: float):
        """Set the target position for the robot to navigate to"""
        self.target_x = x
        self.target_y = y
        self.get_logger().info(f"Target position set to: x={x:.2f}, y={y:.2f}")

    def odom_callback(self, msg):
        """Callback for odometry data"""
        # Extract robot's current position and orientation from odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Extract orientation from quaternion using TF transformations
        orientation_q = msg.pose.pose.orientation
        _, _, current_theta = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        
        # Update current pose
        self.current_pose = {'x': current_x, 'y': current_y, 'theta': current_theta}
        self.odom_data = msg

    def scan_callback(self, msg):
        try:
            self.scan_data = msg.ranges 
            self.control_loop()
        except Exception as e:
            self.get_logger().warn(f"Error in scan callback: {e}")

    def control_loop(self):
        try:
            # Check if we have both scan and odometry data
            if self.scan_data is None or self.odom_data is None:
                return

            # Get current position and orientation from odometry
            current_x = self.current_pose['x']
            current_y = self.current_pose['y']
            current_theta = self.current_pose['theta']

            # Calculate the target heading and distance
            dx = self.target_x - current_x
            dy = self.target_y - current_y
            target_distance = math.sqrt(dx**2 + dy**2)
            target_theta = math.atan2(dy, dx)

            # Calculate target angle in base_link frame
            target_theta_base_link = math.atan2(math.sin(target_theta - current_theta), math.cos(target_theta - current_theta))

            # # Debug information about target and current position
            # self.get_logger().info(
            #     f'Target: ({self.target_x:.2f}, {self.target_y:.2f}), '
            #     f'Current: ({current_x:.2f}, {current_y:.2f}), '
            #     f'Distance: {target_distance:.2f}m, '
            #     f'Target angle: {np.degrees(target_theta):.1f}°, '
            #     f'Current angle: {np.degrees(current_theta):.1f}°'
            # )

            # Apply Histogram obstacle avoidance control if LIDAR data is available
            if self.scan_data is not None and not self.debug_mode:
                best_steering_idx, best_steering_theta_base_link, steering_direction = self.obstacle_avoidance.get_steering_command(
                    self.scan_data, target_theta_base_link)
                
                best_steering_theta = best_steering_theta_base_link + current_theta
                best_steering_theta = (best_steering_theta + np.pi) % (2 * np.pi) - np.pi  # Unwrap to [-pi, pi]

                print("----------------------------------------------------------------")
                # Debug information
                self.get_logger().info(
                    f'Obstacle Avoidance: steering_idx={best_steering_idx}, '
                    f'steering_angle={np.degrees(best_steering_theta):.1f}°, '
                    f'steering_direction={steering_direction}, '
                    f'original_target={np.degrees(target_theta):.1f}°'
                )
                target_theta_difference = best_steering_theta_base_link
            else:
                # In debug mode, use the original target angle
                target_theta_difference = target_theta_base_link
                self.get_logger().info(f'Debug mode: using original target angle {np.degrees(target_theta):.1f}°')
                
                # Update occupancy grid even in debug mode for visualization
                if self.scan_data is not None:
                    self.obstacle_avoidance.update_occupancy_grid(self.scan_data)

            # Calculate control errors
            angular_error = target_theta_difference
            linear_error = target_distance

            # Apply P-control with improved logic
            # Allow turning even when close to target, but reduce linear velocity for large angular errors
            angular_error_deg = abs(180 * angular_error / np.pi)
            
            if angular_error_deg > 5:  # Large angular error - prioritize turning
                linear_velocity = 0.0
                angular_velocity = self.kp_angular * angular_error
            elif angular_error_deg > 10:  # Medium angular error - reduce linear velocity
                linear_velocity = self.kp_linear * linear_error * 0.3  # Reduce linear velocity
                angular_velocity = self.kp_angular * angular_error
            else:  # Small angular error - normal operation
                linear_velocity = self.kp_linear * linear_error
                angular_velocity = self.kp_angular * angular_error

            # Stop linear motion when very close to target
            if target_distance < 0.05:  # 5cm threshold
                linear_velocity = 0.0

            # Limit velocities for safety
            linear_velocity = max(min(linear_velocity, 0.15), -0.15)  # Increased max linear velocity
            angular_velocity = max(min(angular_velocity, 0.2), -0.2)  # Increased max angular velocity

            # Publish velocity commands
            self.publish_velocity(linear_velocity, angular_velocity) # positive angular velocity is left turn, negative is right turn

            # Log current position and velocities
            self.get_logger().info(
                f'Publishing velocity: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}, '
                f'distance={target_distance:.2f}, angle_error={angular_error:.2f} ({angular_error_deg:.1f}°)'
            )
            # Print current position and orientation
            self.get_logger().info(
                f"Current Position: x={current_x:.2f}, y={current_y:.2f}, theta={current_theta:.2f} rad"
            )

            # Print occupancy map periodically
            self.print_counter += 1
            if self.print_counter % self.print_frequency == 0 and self.print_occupancy_map:
                # Log occupancy map statistics instead of printing
                if self.obstacle_avoidance.occupancy_grid is not None:
                    occupied_cells = np.sum(self.obstacle_avoidance.occupancy_grid > 0.25)
                    total_cells = self.obstacle_avoidance.occupancy_grid.size
                    self.get_logger().info(
                        f'Occupancy Map: {occupied_cells}/{total_cells} cells occupied '
                        f'({100*occupied_cells/total_cells:.1f}%)'
                    )

            # Publish visualization data
            self.publish_waypoint(self.target_x, self.target_y)

        except Exception as e:
            self.get_logger().warn(f"Failed to calculate control: {e}")
            self.publish_velocity(0.0, 0.0)  # Stop robot if there's an issue

    def publish_velocity(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = float(linear_velocity)
        msg.angular.z = float(angular_velocity)
        self.publisher_.publish(msg)

    def stop_vehicle(self):
        """Publishes a zero-velocity message to stop the vehicle."""
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info('Published stop command: linear=0.0, angular=0.0')

    def publish_occupancy_grid(self):
        """Publish the occupancy grid for visualization."""
        if self.obstacle_avoidance.occupancy_grid is None:
            return
            
        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"  # Changed from base_link to map
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set grid metadata
        grid_msg.info.resolution = self.obstacle_avoidance.resolution
        grid_msg.info.width = self.obstacle_avoidance.occupancy_grid.shape[0]
        grid_msg.info.height = self.obstacle_avoidance.occupancy_grid.shape[1]
        
        # Set origin in map frame (center of the grid at robot's current position)
        if self.current_pose is not None:
            grid_msg.info.origin.position.x = self.current_pose['x'] - self.obstacle_avoidance.grid_size / 2
            grid_msg.info.origin.position.y = self.current_pose['y'] - self.obstacle_avoidance.grid_size / 2
        else:
            grid_msg.info.origin.position.x = -self.obstacle_avoidance.grid_size / 2
            grid_msg.info.origin.position.y = -self.obstacle_avoidance.grid_size / 2
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Convert grid to occupancy data (0-100)
        # Invert the grid so that occupied cells are 100 and free cells are 0
        grid_data = self.obstacle_avoidance.occupancy_grid.copy()
        grid_data = (grid_data * 100).astype(np.int8)
        # Ensure values are in valid range [0, 100]
        grid_data = np.clip(grid_data, 0, 100)
        
        # # Flip the y-axis by mirroring along the x-axis
        # grid_data = np.flipud(grid_data)

        # # flip the x-axis by mirroring along the y-axis
        # grid_data = np.fliplr(grid_data)
        
        grid_msg.data = grid_data.flatten().tolist()
        
        # Debug information
        occupied_cells = np.sum(grid_data > 0)
        total_cells = len(grid_msg.data)
        self.get_logger().debug(f"Occupancy Grid: {occupied_cells}/{total_cells} cells occupied, "
                               f"resolution={grid_msg.info.resolution}, "
                               f"size={grid_msg.info.width}x{grid_msg.info.height}, "
                               f"origin=({grid_msg.info.origin.position.x:.2f}, {grid_msg.info.origin.position.y:.2f})")
        
        self.occupancy_grid_pub.publish(grid_msg)
    
    def publish_waypoint_marker(self, x: float, y: float, waypoint_id: int = 0):
        """Publish waypoint position as a marker for visualization."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoint"
        marker.id = waypoint_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set scale and color (blue for waypoints)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        self.waypoint_marker_pub.publish(marker)
    
    def publish_waypoint(self, x: float, y: float):
        """Publish current waypoint for visualization."""
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.pose.position.x = x
        waypoint_msg.pose.position.y = y
        waypoint_msg.pose.position.z = 0.0
        waypoint_msg.pose.orientation.w = 1.0
        
        self.waypoint_pub.publish(waypoint_msg)
        
        # Also publish as marker
        self.publish_waypoint_marker(x, y)
    
    def publish_visualization(self):
        """Publish all visualization data."""
        # Publish occupancy grid
        if self.obstacle_avoidance.occupancy_grid is not None:
            self.publish_occupancy_grid()
            self.get_logger().debug("Published occupancy grid")
        else:
            self.get_logger().debug("No occupancy grid available for publishing")
        
        # Publish target marker
        self.publish_target_marker()
        
        # Publish robot pose marker
        self.publish_robot_pose_marker()
        
        # Publish robot path
        self.publish_robot_path()
    
    def publish_robot_path(self):
        """Publish robot path for visualization."""
        if self.current_pose is None:
            return
            
        # Add current pose to path
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = self.current_pose['x']
        pose_stamped.pose.position.y = self.current_pose['y']
        pose_stamped.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.current_pose['theta']
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Create new path with current pose
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add existing poses
        if hasattr(self, 'robot_path_poses'):
            path_msg.poses = self.robot_path_poses + [pose_stamped]
        else:
            path_msg.poses = [pose_stamped]
            self.robot_path_poses = []
        
        # Store poses for next iteration
        self.robot_path_poses = path_msg.poses
        
        # Limit path length to prevent memory issues
        if len(self.robot_path_poses) > 1000:
            self.robot_path_poses = self.robot_path_poses[-500:]
        
        self.robot_path_pub.publish(path_msg)
    
    def publish_target_marker(self):
        """Publish target position as a marker for visualization."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set scale and color
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.target_marker_pub.publish(marker)
    
    def publish_robot_pose_marker(self):
        """Publish robot pose as a marker for visualization."""
        if self.current_pose is None:
            return
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = self.current_pose['x']
        marker.pose.position.y = self.current_pose['y']
        marker.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        yaw = self.current_pose['theta']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set scale and color (red arrow pointing in robot's direction)
        marker.scale.x = 0.5  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.robot_pose_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TurtleBotHistogramWindowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Stopping the vehicle...')
        node.stop_vehicle()  # Stop the vehicle before shutting down
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        node.stop_vehicle()  # Stop the vehicle on error
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
