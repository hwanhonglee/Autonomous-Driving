#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_base_link_route_contract.patch"
launch_file="${repository}/simulator/autoware_carla_interface/launch/autoware_carla_interface.launch.xml"
carla_autoware="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_autoware.py"
conversion_file="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/vehicle_status_conversion.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA base_link route contract patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q 'name="spawn_point_reference"' "${launch_file}" && \
  grep -q 'self.spawn_point_reference = self.param_\["spawn_point_reference"\]' "${carla_autoware}" && \
  grep -q '^def rear_axle_lateral_velocity' "${conversion_file}"; then
  echo "CARLA base_link route contract patch is already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA base_link route contract patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA base_link route contract patch."
