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

ament_export_dependencies(rosidl_default_runtime)
ament_package()