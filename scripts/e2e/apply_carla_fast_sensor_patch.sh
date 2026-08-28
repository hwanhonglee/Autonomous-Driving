#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_camera_fast_options.patch"
carla_ros="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py"
carla_wrapper="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/modules/carla_wrapper.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA fast sensor patch is missing: ${patch_file}" >&2
  exit 1
fi

# Later CARLA contract patches intentionally edit the same functions, so a
# reverse git-apply check is no longer a reliable applied-state test. Require
# independent markers from both files before treating this patch as present.
if grep -q 'def _queue_camera_bundle' "${carla_ros}" && \
  grep -q 'def _measurement_timestamp' "${carla_ros}" && \
  grep -q 'MAX_PENDING_CAMERA_FRAMES = 8' "${carla_wrapper}" && \
  grep -q 'bp.set_attribute("enable_postprocess_effects", enabled)' "${carla_wrapper}"; then
  echo "CARLA fast sensor patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA fast sensor patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA fast sensor patch."
