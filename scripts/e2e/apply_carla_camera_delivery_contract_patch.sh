#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# HH_260906 - Persist ignored Universe changes as a reproducible root-repository patch.
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_carla_interface_camera_delivery_contract.patch"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Required repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "CARLA camera delivery contract patch is missing: ${patch_file}" >&2
  exit 1
fi

if git -C "${repository}" apply --check --reverse "${patch_file}" \
  >/dev/null 2>&1; then
  echo "CARLA camera delivery contract patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "CARLA camera delivery contract patch is partially applied or conflicts." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied CARLA camera delivery contract patch."
