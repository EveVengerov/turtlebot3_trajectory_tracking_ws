cmake_minimum_required(VERSION 3.22)
project(turtlebot3_tracker)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Service interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GlobalPathService.srv"
)

# Install Python nodes
install(
  PROGRAMS
    turtlebot3_tracker/trajectory_publisher_node.py
    turtlebot3_tracker/controller_debug_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test files
install(
  DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)

# Find and install launch test files
find_package(launch_testing REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

# Add launch tests (commented out for now due to compatibility issues)
# launch_testing_ament_cmake_add_launch_test(
#   test_trajectory_tracker_launch.py
#   TARGET test_trajectory_tracker_launch
#   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )

ament_export_dependencies(rosidl_default_runtime)
ament_package()