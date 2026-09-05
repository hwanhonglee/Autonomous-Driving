cmake_minimum_required(VERSION 3.14)
project(autoware_e2e_vad_launch)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# HH_260906 - Install the shared portable E2E package for the ROS shadow adapter.
ament_python_install_package(
  portable_e2e
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../portable_e2e"
)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/carla_actuation_sweep.py
    scripts/carla_actuation_sweep_logic.py
    scripts/carla_autoware_state_publisher.py
    scripts/carla_map_aligned_odometry.py
    scripts/portable_e2e_shadow_node.py
    scripts/vad_aeb_configurator.py
    scripts/vad_imu_acceleration_adapter.py
    scripts/vad_route_logic.py
    scripts/vad_route_manager.py
    scripts/vad_standard_route_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

set(autoware_control_config
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/launcher/autoware_launch/autoware_launch/config/control"
)
set(required_control_configs
  trajectory_follower/trajectory_follower_node.param.yaml
  trajectory_follower/lateral/mpc.param.yaml
  trajectory_follower/longitudinal/pid.param.yaml
  common/nearest_search.param.yaml
  shift_decider/shift_decider.param.yaml
  external_cmd_selector/external_cmd_selector.param.yaml
  vehicle_cmd_gate/vehicle_cmd_gate.param.yaml
)
foreach(config IN LISTS required_control_configs)
  if(NOT EXISTS "${autoware_control_config}/${config}")
    message(FATAL_ERROR "Missing pinned autoware_launch control config: ${config}")
  endif()
endforeach()
install(
  DIRECTORY "${autoware_control_config}/"
  DESTINATION share/${PROJECT_NAME}/config/control
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_vad_route_logic test/test_vad_route_logic.py)
  ament_add_pytest_test(test_vad_standard_route_adapter test/test_vad_standard_route_adapter.py)
  ament_add_pytest_test(test_route_test_full_stack test/test_route_test_full_stack.py)
  ament_add_pytest_test(test_vad_aeb_configurator test/test_vad_aeb_configurator.py)
  ament_add_pytest_test(
    test_vad_imu_acceleration_adapter
    test/test_vad_imu_acceleration_adapter.py
  )
  ament_add_pytest_test(test_validate_route_map test/test_validate_route_map.py)
  ament_add_pytest_test(test_prepare_carla_route test/test_prepare_carla_route.py)
  ament_add_pytest_test(test_fast_profile test/test_fast_profile.py)
  ament_add_pytest_test(
    test_portable_e2e_shadow
    test/test_portable_e2e_shadow.py
  )
  ament_add_pytest_test(test_smart_mpc_integration test/test_smart_mpc_integration.py)
  ament_add_pytest_test(
    test_carla_vehicle_status_contract
    test/test_carla_vehicle_status_contract.py
  )
  ament_add_pytest_test(
    test_carla_actuation_calibration
    test/test_carla_actuation_calibration.py
  )
  ament_add_pytest_test(
    test_carla_actuation_sweep_safety
    test/test_carla_actuation_sweep_safety.py
  )
  ament_add_pytest_test(
    test_carla_map_aligned_odometry
    test/test_carla_map_aligned_odometry.py
  )
endif()

ament_package()
