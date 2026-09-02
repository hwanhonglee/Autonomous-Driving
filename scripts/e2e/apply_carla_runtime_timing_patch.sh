#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_runtime_timing.patch"
package_dir="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface"
carla_autoware="${package_dir}/carla_autoware.py"
carla_ros="${package_dir}/carla_ros.py"
runtime_timing="${package_dir}/modules/runtime_timing.py"
sensor_worker="${package_dir}/modules/sensor_publish_worker.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA runtime timing patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q '"sensor_wait"' "${carla_autoware}" 2>/dev/null && \
  grep -q '"world_tick"' "${carla_autoware}" 2>/dev/null && \
  grep -q '"camera_bundle_total"' "${carla_ros}" 2>/dev/null && \
  grep -q '"camera_image_publish"' "${carla_ros}" 2>/dev/null && \
  grep -q '"camera_bundle_queue_wait"' "${sensor_worker}" 2>/dev/null && \
  grep -q '^class RuntimeTimingMonitor:' "${runtime_timing}" 2>/dev/null; then
  echo "CARLA runtime timing patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA runtime timing patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA runtime timing patch."
