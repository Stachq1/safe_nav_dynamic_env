cmake_minimum_required(VERSION 3.5)
project(dynablox_ros)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
add_compile_options(-Wall -Wextra)
add_compile_definitions(GLOG_USE_GLOG_EXPORT)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynablox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ellipsoid_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(voxblox_ros REQUIRED)

# Include directories
include_directories(include)
include_directories(${voxblox_ros_INCLUDE_DIRS})

# Add library
add_library(${PROJECT_NAME}
  src/visualization/motion_visualizer.cpp
  src/visualization/cloud_visualizer.cpp
  src/motion_detector.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  dynablox
  geometry_msgs
  tf2_ros
  ellipsoid_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  voxblox
  voxblox_msgs
  voxblox_ros
)

# Add executables
add_executable(motion_detector_node src/motion_detector_node.cpp)
target_link_libraries(motion_detector_node ${PROJECT_NAME} glog::glog)

add_executable(cloud_visualizer_node src/cloud_visualizer_node.cpp)
target_link_libraries(cloud_visualizer_node ${PROJECT_NAME} glog::glog)

# Install targets
install(TARGETS ${PROJECT_NAME}
  motion_detector_node
  cloud_visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Export package dependencies
ament_export_dependencies(rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs tf2_ros ellipsoid_msgs pcl_conversions voxblox voxblox_ros voxblox_msgs dynablox)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
