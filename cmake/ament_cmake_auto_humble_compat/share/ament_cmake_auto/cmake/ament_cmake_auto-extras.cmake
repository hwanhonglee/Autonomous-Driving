# Keep every Humble macro except ament_auto_package() unchanged.  The system
# directory is derived from the active ROS distribution instead of copying
# the whole ROS package into this repository.
find_package(ament_cmake QUIET REQUIRED)

if(NOT DEFINED ENV{ROS_DISTRO} OR "$ENV{ROS_DISTRO}" STREQUAL "")
  message(FATAL_ERROR "ROS_DISTRO is required by the ament_cmake_auto compatibility layer")
endif()

set(_ament_cmake_auto_system_dir
  "/opt/ros/$ENV{ROS_DISTRO}/share/ament_cmake_auto/cmake")
if(NOT EXISTS "${_ament_cmake_auto_system_dir}/ament_auto_add_library.cmake")
  message(FATAL_ERROR
    "System ament_cmake_auto macros not found: ${_ament_cmake_auto_system_dir}")
endif()

include("${_ament_cmake_auto_system_dir}/ament_auto_add_executable.cmake")
include("${_ament_cmake_auto_system_dir}/ament_auto_add_gmock.cmake")
include("${_ament_cmake_auto_system_dir}/ament_auto_add_gtest.cmake")
include("${_ament_cmake_auto_system_dir}/ament_auto_add_library.cmake")
include("${_ament_cmake_auto_system_dir}/ament_auto_generate_code.cmake")
include("${_ament_cmake_auto_system_dir}/ament_auto_find_build_dependencies.cmake")
include("${_ament_cmake_auto_system_dir}/ament_auto_find_test_dependencies.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/ament_auto_package.cmake")

unset(_ament_cmake_auto_system_dir)
