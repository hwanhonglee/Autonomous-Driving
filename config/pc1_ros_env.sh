#!/usr/bin/env bash

# HH_260811 - Keep PC1 Domain 10 traffic on CycloneDDS and resolve the DDS XML portably.
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PC1_ROS_ENV_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_URI="file://${PC1_ROS_ENV_DIR}/cyclonedds_pc1.xml"
unset PC1_ROS_ENV_DIR

# HH_260811 - Preserve the historical operator commands while documenting that run_bridge is full RX/TX.
alias run_autoware='ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'
alias run_planning_universe='ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'
alias run_bridge='ros2 launch ros2_socketcan can_brdige.launch.xml'
