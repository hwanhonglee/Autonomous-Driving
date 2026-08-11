#!/usr/bin/env bash
# Compatibility entry point for terminals that still cache the former alias.
# HH_260811 - Keep the original run_autoware command path without adding guards or changing order.
set -e

source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash

exec ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$HOME/Downloads/sample-map-planning" \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
