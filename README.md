# TurtleBot3 Trajectory Tracker

This package provides trajectory generation and publishing capabilities for TurtleBot3 robots with support for both basic trajectory tracking and obstacle avoidance.

## Demo

### Basic Trajectory Tracking
**Demo Video:** [Look Ahead Tracking Demo](assets/look_ahead_tracking.webm)

### Obstacle Avoidance  
**Demo Video:** [Obstacle Avoidance Demo](assets/Obstacle_avoidance_demo.webm)

*Note: Click the links above to view the demo videos. WebM format is supported by most modern browsers.*

## Features

- **Path Smoothing**: Uses cubic spline interpolation to create smooth trajectories from waypoints
- **Time Parameterization**: Generates time-parameterized trajectories with different velocity profiles:
  - Trapezoidal velocity profile (acceleration → constant → deceleration)
  - Constant velocity profile
  - Sine-based velocity profile for smooth start/stop
- **Multiple Controllers**: Supports various trajectory tracking controllers:
  - Pure Pursuit controller
  - PID controller
  - Stanley controller
- **Obstacle Avoidance**: Advanced obstacle avoidance using histogram-based algorithms
- **ROS 2 Integration**: Publishes trajectory data to TurtleBot3-compatible topics
- **Visualization**: RViz integration for real-time trajectory visualization

## Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for the robot
- `/trajectory_path` (nav_msgs/Path): Complete planned trajectory path
- `/trajectory_waypoints` (geometry_msgs/PoseArray): Trajectory waypoints
- `/trajectory_markers` (visualization_msgs/MarkerArray): Trajectory visualization markers
- `/robot_trace_path` (nav_msgs/Path): Actual path taken by the robot
- `/planned_path` (nav_msgs/Path): Planned trajectory path (obstacle avoidance mode)

## Subscribed Topics

- `/odom` (nav_msgs/Odometry): Robot odometry data
- `/scan` (sensor_msgs/LaserScan): Laser scan data (for obstacle avoidance)

## TF Transforms

- Publishes robot pose as TF transform from `map` to `base_link`

## Installation

1. Build the package:
```bash
cd ~/turtlebot3_trajectory_tracking_ws
colcon build --packages-select turtlebot3_tracker
source install/setup.bash
```

## Usage

### 1. Trajectory Tracking Without Obstacle Avoidance

This mode provides basic trajectory tracking functionality without obstacle avoidance.

#### Method 1: Using the launch file
```bash
ros2 launch turtlebot3_tracker trajectory_with_rviz.launch.py
```

With custom parameters:
```bash
ros2 launch turtlebot3_tracker trajectory_with_rviz.launch.py \
    velocity_profile:=sine \
    max_velocity:=0.8 \
    controller_type:=pure_pursuit \
    lookahead_distance:=0.5 \
    use_rviz:=true
```

#### Method 2: Running the node directly
```bash
ros2 run turtlebot3_tracker trajectory_tracker_node
```

#### Available Parameters for Basic Trajectory Tracking

**Trajectory Generation Parameters:**
- `velocity_profile`: Type of velocity profile (`trapezoidal`, `constant`, `sine`)
- `max_velocity`: Maximum velocity in m/s (default: 0.5)
- `acceleration`: Acceleration rate in m/s² (default: 0.5)
- `deceleration`: Deceleration rate in m/s² (default: 0.5)
- `dt`: Time step for trajectory sampling in seconds (default: 0.1)
- `num_smooth_points`: Number of points for smooth trajectory generation (default: 100)

**Controller Parameters:**
- `controller_type`: Controller type (`pure_pursuit`, `pid`, `stanley`)
- `lookahead_distance`: Lookahead distance for pure pursuit controller in meters (default: 0.2)
- `kp_linear`, `ki_linear`, `kd_linear`: PID gains for linear velocity
- `kp_angular`, `ki_angular`, `kd_angular`: PID gains for angular velocity
- `k_e`, `k_v`: Stanley controller gains

**Control Loop Parameters:**
- `control_frequency`: Control loop frequency in Hz (default: 20.0)
- `goal_tolerance`: Goal tolerance in meters (default: 0.1)
- `max_linear_velocity`: Maximum linear velocity for controller in m/s (default: 1.0)
- `max_angular_velocity`: Maximum angular velocity for controller in rad/s (default: 1.0)

### 2. Trajectory Tracking With Obstacle Avoidance

This mode includes advanced obstacle avoidance capabilities using histogram-based algorithms.

#### Method 1: Using the launch file
```bash
ros2 launch turtlebot3_tracker trajectory_tracker_with_obstacle_avoidance.launch.py
```

With custom parameters:
```bash
ros2 launch turtlebot3_tracker trajectory_tracker_with_obstacle_avoidance.launch.py \
    target_x:=5.0 \
    target_y:=3.0 \
    debug_mode:=false \
    use_rviz:=true
```

#### Method 2: Running the node directly
```bash
ros2 run turtlebot3_tracker trajectory_tracker_with_obstacle_avoidance_node
```

#### Available Parameters for Obstacle Avoidance Mode

**Target Parameters:**
- `target_x`: Target X position in meters (default: 4.0)
- `target_y`: Target Y position in meters (default: 0.0)
- `debug_mode`: Enable debug mode to bypass obstacle avoidance (default: false)

**Visualization Parameters:**
- `use_rviz`: Whether to launch RViz for visualization (default: true)

### 3. Monitoring and Visualization

#### Monitor Published Topics

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor planned trajectory path
ros2 topic echo /trajectory_path

# Monitor robot trace path (actual path taken)
ros2 topic echo /robot_trace_path

# Monitor waypoints
ros2 topic echo /trajectory_waypoints
```

#### Visualize in RViz

The launch files automatically start RViz with pre-configured displays. You can also start RViz manually:

```bash
ros2 run rviz2 rviz2
```

In RViz, add these displays:
- **Path**: Set topic to `/trajectory_path` for planned trajectory
- **Path**: Set topic to `/robot_trace_path` for actual robot path
- **PoseArray**: Set topic to `/trajectory_waypoints` for waypoints
- **MarkerArray**: Set topic to `/trajectory_markers` for trajectory markers

### 4. Testing

Run the basic test suite to validate package functionality:

```bash
# Run basic tests (recommended)
cd src/turtlebot3_tracker/test
python3 -m pytest test_basic.py -v

# Or run with colcon
colcon test --packages-select turtlebot3_tracker --pytest-args "test_basic.py -v"
```

**Test Coverage:**
- ✅ Package structure validation (8 tests)
- ✅ Algorithm validation (4 tests)
- ✅ Import verification
- ✅ File existence checks

**Expected Output:** 12 tests passed

For comprehensive testing including ROS integration tests, use:
```bash
colcon test --packages-select turtlebot3_tracker
```

## Example Waypoints

The default trajectory uses these waypoints:
- Start: (0.0, 0.0)
- Waypoint 1: (5.0, 0.0)
- Waypoint 2: (5.0, 5.0)
- End: (0.0, 5.0)

## Customizing the Trajectory

### For Basic Trajectory Tracking

To modify the trajectory, edit the waypoints in the `trajectory_tracker_node.py` file:

```python
# In the load_default_waypoints method
waypoints = [
    (0.0, 0.0),      # Start point
    (2.0, 1.0),      # Waypoint 1
    (4.0, 0.5),      # Waypoint 2
    (6.0, 2.0),      # Waypoint 3
    (8.0, 1.5),      # Waypoint 4
    (10.0, 3.0),     # Waypoint 5
    (12.0, 2.0),     # End point
]
```

### For Obstacle Avoidance Mode

The obstacle avoidance mode uses a single target point. You can modify the target by:

1. Using launch parameters:
```bash
ros2 launch turtlebot3_tracker trajectory_tracker_with_obstacle_avoidance.launch.py target_x:=8.0 target_y:=4.0
```

2. Or modifying the default values in the launch file.

## Controller Types

### Pure Pursuit Controller
- **Best for**: Smooth path following
- **Key parameter**: `lookahead_distance`
- **Usage**: Good for general trajectory tracking

### PID Controller
- **Best for**: Precise position control
- **Key parameters**: `kp_linear`, `ki_linear`, `kd_linear`, `kp_angular`, `ki_angular`, `kd_angular`
- **Usage**: When you need tight control over position and orientation

### Stanley Controller
- **Best for**: Vehicle-like path following
- **Key parameters**: `k_e` (cross-track error gain), `k_v` (velocity gain)
- **Usage**: Good for car-like robots or when you want smooth path following

## Troubleshooting

### Common Issues

1. **Robot not moving**: Check if `/cmd_vel` topic is being published
2. **Poor trajectory tracking**: Adjust controller parameters or try different controller type
3. **RViz not showing trajectory**: Ensure `use_rviz:=true` and check topic names
4. **Obstacle avoidance not working**: Verify laser scan data is available on `/scan` topic

### Debug Mode

For obstacle avoidance mode, you can enable debug mode to bypass obstacle avoidance:

```bash
ros2 launch turtlebot3_tracker trajectory_tracker_with_obstacle_avoidance.launch.py debug_mode:=true
```

## Dependencies

- ROS 2 (Humble or later)
- Python packages: numpy, scipy, matplotlib
- ROS 2 packages: rclpy, geometry_msgs, nav_msgs, tf2_ros, tf_transformations, sensor_msgs, visualization_msgs

## License

MIT License 