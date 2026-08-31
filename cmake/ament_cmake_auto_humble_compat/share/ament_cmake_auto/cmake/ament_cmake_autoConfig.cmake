# Workspace-local compatibility package for the ament_cmake_auto version
# shipped with ROS 2 Humble.  Autoware 1.9 uses the newer
# USE_SCOPED_HEADER_INSTALL_DIR option, while Humble's macro forwards that
# option to ament_package() and fails during configuration.

if(_ament_cmake_auto_CONFIG_INCLUDED)
  if(NOT DEFINED ament_cmake_auto_FOUND)
    set(ament_cmake_auto_FOUND FALSE)
  elseif(NOT ament_cmake_auto_FOUND)
    set(ament_cmake_auto_FOUND FALSE)
  endif()
  return()
endif()
set(_ament_cmake_auto_CONFIG_INCLUDED TRUE)

set(ament_cmake_auto_FOUND_AMENT_PACKAGE TRUE)
set(ament_cmake_auto_FOUND TRUE)

if(NOT ament_cmake_auto_FIND_QUIETLY)
  message(STATUS "Found workspace ament_cmake_auto Humble compatibility layer (${ament_cmake_auto_DIR})")
endif()

include("${ament_cmake_auto_DIR}/ament_cmake_auto-extras.cmake")
