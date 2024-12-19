cmake_minimum_required(VERSION 3.8)
project(ellipsoid_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ellipsoid.msg"
  "msg/EllipsoidArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Export dependencies
ament_export_dependencies(geometry_msgs std_msgs rosidl_default_runtime)

# Install
ament_package()
