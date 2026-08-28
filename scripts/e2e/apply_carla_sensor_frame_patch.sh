#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
universe="${root}/src/universe/autoware_universe"
launcher="${root}/src/launcher/autoware_launch"
universe_patch="${root}/patches/autoware_carla_interface_ros_sensor_frame.patch"
launcher_patch="${root}/patches/autoware_launch_carla_ros_sensor_frame.patch"

for repository in "${universe}" "${launcher}"; do
  if [[ ! -d "${repository}/.git" ]]; then
    echo "Required repository is missing: ${repository}" >&2
    exit 1
  fi
done
for patch_file in "${universe_patch}" "${launcher_patch}"; do
  if [[ ! -f "${patch_file}" ]]; then
    echo "CARLA sensor frame patch is missing: ${patch_file}" >&2
    exit 1
  fi
done

loader="${universe}/simulator/autoware_carla_interface/src/autoware_carla_interface/modules/sensor_kit_loader.py"
calibration="${launcher}/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"

if ! grep -q "_ros_baselink_to_vehicle_center_transform" "${loader}" 2>/dev/null; then
  if ! git -C "${universe}" apply --check "${universe_patch}"; then
    echo "CARLA interface ROS sensor frame patch does not apply cleanly." >&2
    exit 1
  fi
  git -C "${universe}" apply "${universe_patch}"
  echo "Applied CARLA interface ROS sensor frame patch."
else
  echo "CARLA interface ROS sensor frame patch is already applied."
fi

if ! grep -q "Frames: ROS base_link" "${calibration}" 2>/dev/null; then
  if ! git -C "${launcher}" apply --check "${launcher_patch}"; then
    echo "CARLA sensor calibration ROS frame patch does not apply cleanly." >&2
    exit 1
  fi
  git -C "${launcher}" apply "${launcher_patch}"
  echo "Applied CARLA sensor calibration ROS frame patch."
else
  echo "CARLA sensor calibration ROS frame patch is already applied."
fi
