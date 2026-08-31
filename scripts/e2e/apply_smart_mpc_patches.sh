#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

apply_patch_once() {
  local repository="$1"
  local patch_file="$2"
  local label="$3"

  if [[ ! -d "${repository}/.git" ]]; then
    echo "Repository is missing: ${repository}" >&2
    exit 1
  fi
  if [[ ! -f "${patch_file}" ]]; then
    echo "Patch is missing: ${patch_file}" >&2
    exit 1
  fi
  if git -C "${repository}" apply --reverse --check "${patch_file}" 2>/dev/null; then
    echo "${label} is already applied."
    return
  fi
  if ! git -C "${repository}" apply --check "${patch_file}"; then
    echo "${label} does not apply cleanly." >&2
    exit 1
  fi
  git -C "${repository}" apply "${patch_file}"
  echo "Applied ${label}."
}

"${root}/scripts/e2e/apply_autoware_launch_control_override.sh"
"${root}/scripts/e2e/apply_autoware_launch_vehicle_cmd_gate_override.sh"

apply_patch_once \
  "${root}/src/launcher/autoware_launch" \
  "${root}/patches/autoware_launch_smart_mpc_nominal_ilqr_preset.patch" \
  "Smart MPC launch preset"
apply_patch_once \
  "${root}/src/launcher/autoware_launch" \
  "${root}/patches/autoware_launch_smart_mpc_runtime_param.patch" \
  "Smart MPC runtime parameter pass-through"
apply_patch_once \
  "${root}/src/universe/autoware_universe" \
  "${root}/patches/autoware_smart_mpc_carla_runtime.patch" \
  "Smart MPC CARLA runtime compatibility patch"
