# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This is Humble's ament_auto_package macro with support for the upstream
# USE_SCOPED_HEADER_INSTALL_DIR option.  Keeping the remainder identical to
# Humble minimizes the surface area of the compatibility layer.
macro(ament_auto_package)
  cmake_parse_arguments(
    _ARG_AMENT_AUTO_PACKAGE
    "INSTALL_TO_PATH;USE_SCOPED_HEADER_INSTALL_DIR"
    ""
    "INSTALL_TO_SHARE"
    ${ARGN})

  set(_run_depends
    ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_EXPORT_DEPENDS}
    ${${PROJECT_NAME}_EXEC_DEPENDS})
  foreach(_dep
      ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
      ${${PROJECT_NAME}_FOUND_BUILDTOOL_DEPENDS})
    if(_dep IN_LIST _run_depends)
      ament_export_dependencies("${_dep}")
    endif()
  endforeach()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    if(_ARG_AMENT_AUTO_PACKAGE_USE_SCOPED_HEADER_INSTALL_DIR)
      set(_include_destination "include/${PROJECT_NAME}")
    else()
      set(_include_destination "include")
    endif()
    ament_export_include_directories("${_include_destination}")
    install(DIRECTORY include/ DESTINATION "${_include_destination}")
    unset(_include_destination)
  endif()

  if(NOT ${PROJECT_NAME}_LIBRARIES STREQUAL "")
    ament_export_libraries(${${PROJECT_NAME}_LIBRARIES})
    install(
      TARGETS ${${PROJECT_NAME}_LIBRARIES}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  if(NOT ${PROJECT_NAME}_EXECUTABLES STREQUAL "")
    if(_ARG_AMENT_AUTO_PACKAGE_INSTALL_TO_PATH)
      set(_destination "bin")
    else()
      set(_destination "lib/${PROJECT_NAME}")
    endif()
    install(
      TARGETS ${${PROJECT_NAME}_EXECUTABLES}
      DESTINATION ${_destination}
    )
  endif()

  foreach(_dir ${_ARG_AMENT_AUTO_PACKAGE_INSTALL_TO_SHARE})
    install(
      DIRECTORY "${_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endforeach()

  ament_execute_extensions(ament_auto_package)
  ament_package(${_ARG_AMENT_AUTO_PACKAGE_UNPARSED_ARGUMENTS})
endmacro()
