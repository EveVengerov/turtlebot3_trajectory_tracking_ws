# TurtleBot3 Trajectory Publisher

A comprehensive ROS2 package for generating smooth trajectories from waypoints and implementing trajectory tracking control for differential drive robots.

## Features

- **Smooth Path Generation**: Uses cubic spline interpolation to create smooth paths from waypoints
- **Time-Parameterized Trajectories**: Supports multiple velocity profiles (trapezoidal, constant, sine)
- **Multiple Control Algorithms**: Implements Pure Pursuit, PID, and Stanley controllers
- **Real-time Trajectory Tracking**: Follows generated trajectories with configurable control parameters
- **Visualization**: Publishes trajectory visualization for RViz
- **Parameterized Configuration**: Highly configurable through ROS parameters

## Architecture

The package consists of three main components:

1. **TrajectoryGenerator** (`trajectory_generator.py`): Generates smooth, time-parameterized trajectories
2. **TrajectoryController** (`trajectory_controller.py`): Implements trajectory tracking control algorithms
3. **TrajectoryPublisherNode** (`trajectory_publisher_node.py`): Main ROS2 node that integrates everything

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/turtlebot3_ws/src
git clone <repository-url>
```

2. Build the workspace:
```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_tracker
```

3. Source the workspace:
```bash
source ~/turtlebot3_ws/install/setup.bash
```

## Usage

### Basic Usage

1. **Launch the trajectory publisher node**:
```bash
ros2 launch turtlebot3_tracker trajectory_publisher.launch.py
```

2. **With custom parameters**:
```bash
ros2 launch turtlebot3_tracker trajectory_publisher.launch.py \
    controller_type:=pure_pursuit \
    max_velocity:=1.5 \
    lookahead_distance:=0.8
```

### Running with TurtleBot3 Simulation

1. **Start TurtleBot3 simulation**:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **Launch the trajectory publisher**:
```bash
ros2 launch turtlebot3_tracker trajectory_publisher.launch.py
```

3. **Visualize in RViz**:
```bash
ros2 launch turtlebot3_navigation2 rviz2.launch.py
```

### Running with Real TurtleBot3

1. **Start TurtleBot3 bringup**:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py
```

2. **Launch the trajectory publisher**:
```bash
ros2 launch turtlebot3_tracker trajectory_publisher.launch.py
```

## Configuration Parameters

### Trajectory Generation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `velocity_profile` | `trapezoidal` | Velocity profile type: `trapezoidal`, `constant`, or `sine` |
| `max_velocity` | `1.0` | Maximum velocity in m/s |
| `acceleration` | `0.5` | Acceleration in m/s² |
| `deceleration` | `0.5` | Deceleration in m/s² |
| `dt` | `0.1` | Time step for trajectory sampling in seconds |
| `num_smooth_points` | `100` | Number of points for smooth trajectory generation |

### Controller Parameters

#### Pure Pursuit Controller
| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookahead_distance` | `0.5` | Lookahead distance in meters |
| `max_linear_velocity` | `1.0` | Maximum linear velocity in m/s |
| `max_angular_velocity` | `1.0` | Maximum angular velocity in rad/s |
| `min_linear_velocity` | `0.1` | Minimum linear velocity in m/s |

#### PID Controller
| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp_linear` | `1.0` | Proportional gain for linear velocity |
| `ki_linear` | `0.0` | Integral gain for linear velocity |
| `kd_linear` | `0.0` | Derivative gain for linear velocity |
| `kp_angular` | `1.0` | Proportional gain for angular velocity |
| `ki_angular` | `0.0` | Integral gain for angular velocity |
| `kd_angular` | `0.0` | Derivative gain for angular velocity |

#### Stanley Controller
| Parameter | Default | Description |
|-----------|---------|-------------|
| `k_e` | `1.0` | Cross-track error gain |
| `k_v` | `1.0` | Velocity gain |
| `max_steering_angle` | `π/4` | Maximum steering angle in radians |

### Control Loop Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_frequency` | `20.0` | Control loop frequency in Hz |
| `goal_tolerance` | `0.1` | Goal tolerance in meters |
| `max_linear_velocity` | `1.0` | Maximum linear velocity for controller in m/s |
| `max_angular_velocity` | `1.0` | Maximum angular velocity for controller in rad/s |

### Visualization Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_visualization` | `true` | Whether to publish trajectory visualization |
| `visualization_frequency` | `1.0` | Visualization publishing frequency in Hz |

## ROS Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry (position and velocity) |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data (for future obstacle avoidance) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/trajectory_path` | `nav_msgs/Path` | Generated trajectory path |
| `/trajectory_waypoints` | `geometry_msgs/PoseArray` | Original waypoints |
| `/trajectory_markers` | `visualization_msgs/MarkerArray` | Robot and target position markers |

## Control Algorithms

### 1. Pure Pursuit Controller

The Pure Pursuit controller is a geometric path tracking algorithm that:
- Uses a lookahead distance to find a target point on the path
- Calculates the curvature needed to reach that point
- Generates appropriate linear and angular velocities

**Advantages**:
- Simple and computationally efficient
- Works well for smooth paths
- Predictable behavior

**Best for**: General trajectory tracking with smooth paths

### 2. PID Controller

The PID controller uses proportional, integral, and derivative control:
- **Proportional**: Responds to current position error
- **Integral**: Accumulates position error over time
- **Derivative**: Responds to rate of change of error

**Advantages**:
- Well-understood control theory
- Can be tuned for specific performance requirements
- Good for precise positioning

**Best for**: Precise trajectory tracking with specific performance requirements

### 3. Stanley Controller

The Stanley controller is designed for path tracking:
- Combines heading error and cross-track error
- Uses velocity-dependent steering
- Provides smooth and stable tracking

**Advantages**:
- Smooth tracking behavior
- Velocity-adaptive control
- Good for high-speed tracking

**Best for**: High-speed trajectory tracking with smooth control

## Example Waypoints

The default waypoints create a zigzag pattern:
```python
default_waypoints = [
    [0.0, 0.0],   # Start
    [1.0, 1.0],   # First turn
    [2.0, 0.0],   # Second turn
    [3.0, 1.0],   # Third turn
    [4.0, 0.0]    # End
]
```

You can customize waypoints by:
1. Modifying the `default_waypoints` parameter in the launch file
2. Creating a custom waypoint service (future feature)
3. Loading waypoints from a file (future feature)

## Testing

### Run the Test Script

```bash
cd ~/turtlebot3_ws/src/turtlebot3_tracker/turtlebot3_tracker
python3 test_trajectory.py
```

This will:
- Generate a test trajectory
- Test different controllers
- Display plots showing trajectory generation and tracking performance

### Manual Testing

1. **Start the node**:
```bash
ros2 run turtlebot3_tracker trajectory_publisher_node
```

2. **Monitor topics**:
```bash
# View trajectory path
ros2 topic echo /trajectory_path

# View robot commands
ros2 topic echo /cmd_vel

# View robot position
ros2 topic echo /odom
```

3. **Visualize in RViz**:
- Add a Path display and subscribe to `/trajectory_path`
- Add a PoseArray display and subscribe to `/trajectory_waypoints`
- Add a MarkerArray display and subscribe to `/trajectory_markers`

## Troubleshooting

### Common Issues

1. **Robot not moving**:
   - Check if `/odom` topic is being published
   - Verify controller parameters are appropriate
   - Ensure waypoints are reachable

2. **Poor tracking performance**:
   - Adjust controller gains
   - Reduce maximum velocity
   - Increase control frequency

3. **Oscillatory behavior**:
   - Reduce proportional gains
   - Increase derivative gains
   - Adjust lookahead distance (for Pure Pursuit)

4. **Slow response**:
   - Increase proportional gains
   - Reduce lookahead distance (for Pure Pursuit)
   - Increase maximum velocity

### Debugging

1. **Enable debug logging**:
```bash
ros2 run turtlebot3_tracker trajectory_publisher_node --ros-args --log-level DEBUG
```

2. **Monitor control commands**:
```bash
ros2 topic echo /cmd_vel --once
```

3. **Check parameter values**:
```bash
ros2 param list
ros2 param get /trajectory_publisher_node controller_type
```

## Future Enhancements

- [ ] Obstacle avoidance integration
- [ ] Dynamic waypoint loading from files
- [ ] Multiple trajectory support
- [ ] Advanced control algorithms (MPC, LQR)
- [ ] Trajectory optimization
- [ ] Real-time trajectory modification
- [ ] Performance metrics and logging
- [ ] Web-based configuration interface

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- TurtleBot3 team for the excellent robot platform
- ROS2 community for the robust robotics framework
- Control theory community for the algorithms implemented 