# HistogramWindowObstacleAvoidance Class

## Overview

The `HistogramWindowObstacleAvoidance` class implements a histogram-based obstacle avoidance algorithm for mobile robots. This class takes laser scan data and waypoints/trajectories as input and returns the best steering index and updated waypoints that avoid obstacles.

## Features

- **Occupancy Grid Creation**: Converts laser scan data into an occupancy grid for better obstacle representation
- **Polar Histogram**: Creates a polar histogram from the occupancy grid to analyze obstacle distribution
- **Steering Direction Selection**: Finds the best steering direction based on target direction and obstacle avoidance
- **Configurable Parameters**: Robot radius, safety margin, number of sectors, and scan angle can be customized

## Class Methods

### `__init__(robot_radius, safety_margin, num_sectors, scan_angle, range_max)`

Initialize the obstacle avoidance class with configurable parameters.

**Parameters:**
- `robot_radius` (float): Radius of the robot in meters (default: 0.2)
- `safety_margin` (float): Additional safety margin in meters (default: 0.3)
- `num_sectors` (int): Number of sectors in the histogram (default: 36)
- `scan_angle` (float): Total scan angle in degrees (default: 360)
- `range_max` (float): Maximum range for the sensor (default: 3.5)

### `update_occupancy_grid(scan_data)`

Updates the occupancy grid from laser scan data.

**Parameters:**
- `scan_data` (List[float]): Array of distance measurements from LIDAR

### `create_polar_histogram() -> np.ndarray`

Creates a polar histogram from the occupancy grid.

**Returns:**
- Histogram array representing distances in each sector

### `find_best_steering_direction(target_theta) -> Tuple[int, float]`

Finds the best steering direction based on histogram data and target direction.

**Parameters:**
- `target_theta` (float): Target direction in radians

**Returns:**
- Tuple of (best_steering_index, best_steering_angle)

### `get_updated_waypoint(scan_data, current_pose, target_waypoint) -> Tuple[float, float]`

Gets an updated waypoint that avoids obstacles.

**Parameters:**
- `scan_data` (List[float]): Laser scan data
- `current_pose` (Tuple[float, float, float]): Current robot pose (x, y, theta)
- `target_waypoint` (Tuple[float, float]): Target waypoint (x, y)

**Returns:**
- Updated waypoint (x, y) that avoids obstacles

### `get_steering_command(scan_data, target_theta) -> Tuple[int, float]`

Gets steering command based on scan data and target direction.

**Parameters:**
- `scan_data` (List[float]): Laser scan data
- `target_theta` (float): Target direction in radians

**Returns:**
- Tuple of (steering_index, steering_angle)

## Usage Example

```python
from trajectory_tracker_with_obstacle_avoidance_node import HistogramWindowObstacleAvoidance

# Create an instance
obstacle_avoidance = HistogramWindowObstacleAvoidance(
    robot_radius=0.2,
    safety_margin=0.3,
    num_sectors=36,
    scan_angle=360,
    range_max=3.5
)

# Sample laser scan data
scan_data = [3.0, 2.5, 1.8, ...]  # Your laser scan data

# Get steering command
steering_idx, steering_angle = obstacle_avoidance.get_steering_command(scan_data, target_theta)

# Get updated waypoint
current_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
target_waypoint = (2.0, 1.0)    # (x, y)
updated_waypoint = obstacle_avoidance.get_updated_waypoint(scan_data, current_pose, target_waypoint)
```

## Algorithm Details

1. **Occupancy Grid Creation**: Laser scan data is converted to Cartesian coordinates and mapped to a grid
2. **Temporal Filtering**: The occupancy grid is updated with temporal filtering for stability
3. **Polar Histogram**: The occupancy grid is converted to a polar histogram showing obstacle distances in each sector
4. **Safety Validation**: Sectors are validated based on robot radius and safety margin
5. **Consecutive Sector Check**: Only sectors with at least three consecutive valid sectors are considered safe
6. **Direction Selection**: The algorithm finds the best steering direction that is closest to the target while avoiding obstacles

## Integration with ROS2

The class is designed to work seamlessly with ROS2 nodes. The main node (`TurtleBotHistogramWindowController`) uses this class to:

- Process laser scan data from the `/scan` topic
- Calculate steering commands for obstacle avoidance
- Publish velocity commands to the `/cmd_vel` topic

## Configuration

The class can be configured with different parameters depending on your robot and environment:

- **Robot Radius**: Adjust based on your robot's physical size
- **Safety Margin**: Increase for more conservative obstacle avoidance
- **Number of Sectors**: More sectors provide finer angular resolution but increase computation
- **Scan Angle**: Adjust based on your sensor's field of view
- **Range Max**: Set to your sensor's maximum range

## Dependencies

- `numpy`: For numerical operations
- `math`: For mathematical functions
- `typing`: For type hints

## Example Script

See `histogram_obstacle_avoidance_example.py` for a complete example demonstrating how to use the class with sample data. 