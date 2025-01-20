cmake_minimum_required(VERSION 3.5)
project(pvd_msgs)

# Set the C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# Include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${pvd_msgs_INCLUDE_DIRS}
  ${novatel_gps_msgs_INCLUDE_DIRS}
)

# Define message dependencies
set(MSG_DEPS
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Define message files
set(MSG_FILES
  msg/PVDdata.msg
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()

