# TurtleBot3 Trajectory Tracker

This package provides trajectory generation and publishing capabilities for TurtleBot3 robots.

## Features

- **Path Smoothing**: Uses cubic spline interpolation to create smooth trajectories from waypoints
- **Time Parameterization**: Generates time-parameterized trajectories with different velocity profiles:
  - Trapezoidal velocity profile (acceleration → constant → deceleration)
  - Constant velocity profile
  - Sine-based velocity profile for smooth start/stop
- **ROS 2 Integration**: Publishes trajectory data to TurtleBot3-compatible topics

## Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for the robot
- `/planned_path` (nav_msgs/Path): Complete planned trajectory path
- `/trajectory_poses` (geometry_msgs/PoseArray): Trajectory poses (if needed)

## TF Transforms

- Publishes robot pose as TF transform from `map` to `base_link`

## Installation

1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_tracker
source install/setup.bash
```

## Usage

### Running the Trajectory Publisher

#### Method 1: Using the launch file
```bash
ros2 launch turtlebot3_tracker trajectory_publisher.launch.py
```

With custom parameters:
```bash
ros2 launch turtlebot3_tracker trajectory_publisher.launch.py velocity_profile:=sine max_velocity:=2.0 acceleration:=1.0 deceleration:=1.0
```

#### Method 2: Running the node directly
```bash
ros2 run turtlebot3_tracker trajectory_publisher
```

### Available Parameters

- `velocity_profile`: Type of velocity profile (`trapezoidal`, `constant`, `sine`)
- `max_velocity`: Maximum velocity in m/s (default: 1.5)
- `acceleration`: Acceleration rate in m/s² (default: 0.8)
- `deceleration`: Deceleration rate in m/s² (default: 0.8)

### Monitoring the Trajectory

You can monitor the published topics using:

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor planned path
ros2 topic echo /planned_path

# Visualize the path in RViz
ros2 run rviz2 rviz2
```

In RViz, add a Path display and set the topic to `/planned_path`.

### Testing

Run the test suite:
```bash
colcon test --packages-select turtlebot3_tracker
```

## Example Waypoints

The default trajectory uses these waypoints:
- Start: (0.0, 0.0)
- Waypoint 1: (2.0, 1.0)
- Waypoint 2: (4.0, 0.5)
- Waypoint 3: (6.0, 2.0)
- Waypoint 4: (8.0, 1.5)
- Waypoint 5: (10.0, 3.0)
- End: (12.0, 2.0)

## Customizing the Trajectory

To modify the trajectory, edit the waypoints in the `main_ros()` function in `trajectory_pub.py`:

```python
waypoints = [
    (0.0, 0.0),      # Start point
    (2.0, 1.0),      # Waypoint 1
    # Add your custom waypoints here
    (12.0, 2.0),     # End point
]
```

## Dependencies

- ROS 2 (Humble or later)
- Python packages: numpy, scipy, matplotlib
- ROS 2 packages: rclpy, geometry_msgs, nav_msgs, tf2_ros, tf_transformations

## License

MIT License 