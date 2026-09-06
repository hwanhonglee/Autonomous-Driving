#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_clean_shutdown.patch"
carla_ros="${repository}/simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA clean shutdown patch is missing: ${patch_file}" >&2
  exit 1
fi

# HH_260906 - Require the complete context-stop, join, and node-destroy ordering contract.
if grep -Fq 'Stop the ROS context first so the blocked executor can leave spin.' \
    "${carla_ros}" 2>/dev/null \
  && grep -Fq 'Join only after shutdown has awakened the executor wait set.' \
    "${carla_ros}" 2>/dev/null \
  && grep -Fq 'Destroy the node only after its executor has released it.' \
    "${carla_ros}" 2>/dev/null; then
  echo "CARLA clean shutdown patch is already applied."
  exit 0
fi

if git -C "${repository}" apply --reverse --check "${patch_file}" 2>/dev/null; then
  echo "CARLA clean shutdown patch is already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA clean shutdown patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA clean shutdown patch."
