#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_base_link_pose.patch"
source_file="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA base_link pose patch is missing: ${patch_file}" >&2
  exit 1
fi

if ! grep -q "base_link_transform = CoordinateTransformer.shift_carla_transform_along_local_x" "${source_file}" 2>/dev/null; then
  if ! git -C "${repository}" apply --check "${patch_file}"; then
    echo "CARLA base_link pose patch does not apply cleanly." >&2
    exit 1
  fi
  git -C "${repository}" apply "${patch_file}"
  echo "Applied CARLA base_link pose patch."
else
  echo "CARLA base_link pose patch is already applied."
fi
