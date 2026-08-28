#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_vehicle_status_contract.patch"
launch_file="${repository}/simulator/autoware_carla_interface/launch/autoware_carla_interface.launch.xml"
carla_ros="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py"
conversion_file="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/vehicle_status_conversion.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA vehicle-status patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q 'vehicle_info.param.yaml' "${launch_file}" && \
  grep -q 'fl_wheel_to_virtual_tire_angle' "${carla_ros}" && \
  grep -q '^def carla_yaw_rate_to_ros' "${conversion_file}" 2>/dev/null; then
  echo "CARLA vehicle-status patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA vehicle-status patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA vehicle-status contract patch."
