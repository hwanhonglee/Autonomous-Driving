#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_imu_source_timestamp.patch"
carla_ros="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA IMU source timestamp patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q 'def imu(self, carla_imu_measurement, timestamp)' "${carla_ros}" && \
  grep -q 'self.imu(data\[1\], measurement_timestamp)' "${carla_ros}" && \
  grep -q 'update_sensor_timestamp("imu", timestamp)' "${carla_ros}"; then
  echo "CARLA IMU source timestamp patch is already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA IMU source timestamp patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA IMU source timestamp patch."
