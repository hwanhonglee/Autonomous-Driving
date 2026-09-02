#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_camera_qos_split.patch"
module_dir="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/modules"
publisher_manager="${module_dir}/ros_publisher_manager.py"
sensor_loader="${module_dir}/sensor_kit_loader.py"
sensor_manager="${module_dir}/sensor_manager.py"
interface_readme="${repository}/simulator/autoware_carla_interface/README.md"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA camera QoS split patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q '"best_effort_depth_1": self._create_best_effort_qos(depth=1)' \
    "${publisher_manager}" 2>/dev/null && \
  grep -q 'sensor_config.image_qos_profile' "${publisher_manager}" 2>/dev/null && \
  grep -q 'camera_info_qos_profile=ros_config.get' "${sensor_loader}" 2>/dev/null && \
  grep -q 'image_qos_profile: Optional\[str\]' "${sensor_manager}" 2>/dev/null && \
  grep -q 'falls back to the shared' "${interface_readme}" 2>/dev/null; then
  echo "CARLA camera QoS split patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA camera QoS split patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA camera QoS split patch."
