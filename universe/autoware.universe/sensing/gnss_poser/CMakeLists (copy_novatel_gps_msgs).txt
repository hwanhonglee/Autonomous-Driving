cmake_minimum_required(VERSION 3.14)
project(gnss_poser)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

# HH_240308
# Add the path to novatel_gps_msgs headers
find_package(novatel_gps_msgs REQUIRED)

# HH_240308
# Include directories for gnss_poser_node
include_directories(
  include
  ${novatel_gps_msgs_INCLUDE_DIRS} 
)

# HH_240308
# Add gnss_poser_node target
ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_core.cpp
)

# Define headers for the library
set(GNSS_POSER_HEADERS
  include/gnss_poser/gnss_poser_core.hpp
)

# HH_240308
# Add headers to the library target
target_sources(gnss_poser_node PRIVATE ${GNSS_POSER_HEADERS})

# Link libraries to gnss_poser_node target
target_link_libraries(gnss_poser_node
  Geographic
)

# Register node
rclcpp_components_register_node(gnss_poser_node
  PLUGIN "gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
)

# Install
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)

