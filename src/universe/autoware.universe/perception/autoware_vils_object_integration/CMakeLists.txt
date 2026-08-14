cmake_minimum_required(VERSION 3.14)
project(autoware_vils_object_integration)

# HH_260810 - Use the Autoware package helper so manifest dependencies and standard warnings stay consistent.
find_package(autoware_cmake REQUIRED)
autoware_package()

# HH_260810 - Link OpenSSL explicitly for the named receiver-side ROS reserialization contract.
find_package(OpenSSL REQUIRED)

# HH_260810 - Keep contract validation and physical-main fusion independently unit-testable.
ament_auto_add_library(${PROJECT_NAME}_contract SHARED
  src/contract.cpp
)

# HH_260810 - Build one PC2 process that owns validation, arming, status, and selected output.
ament_auto_add_executable(vils_object_integration_node
  src/vils_object_integration_node.cpp
)
target_link_libraries(vils_object_integration_node
  ${PROJECT_NAME}_contract
  OpenSSL::Crypto
)

if(BUILD_TESTING)
  # HH_260810 - Verify the pure payload and fusion contract without requiring a live DDS graph.
  ament_auto_add_gtest(test_contract
    test/test_contract.cpp
  )
  target_link_libraries(test_contract
    ${PROJECT_NAME}_contract
  )

  # HH_260810 - Exercise exact writer binding, manual arm, physical-main fusion, and TTL over DDS.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_vils_shadow.test.py
    TIMEOUT 60
  )
endif()

# HH_260810 - Install the operator note as a file and the runtime resource trees under package share.
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
