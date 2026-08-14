cmake_minimum_required(VERSION 3.14)
project(vils_interfaces)

# HH_260810 - Generate the typed PC2 acceptance and operator-arm interfaces for the prototype VILS boundary.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AcceptedPc4Status.msg"
  "srv/ArmPc4Source.srv"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
