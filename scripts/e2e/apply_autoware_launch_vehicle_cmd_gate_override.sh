#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/launcher/autoware_launch"
patch_file="${root}/patches/autoware_launch_vehicle_cmd_gate_param_override.patch"
autoware_launch="${repository}/autoware_launch/launch/autoware.launch.xml"
control_component="${repository}/autoware_launch/launch/components/tier4_control_component.launch.xml"

if [[ ! -d "${repository}/.git" || ! -f "${patch_file}" ]]; then
  echo "Autoware launch repository or vehicle command gate patch is missing." >&2
  exit 1
fi
if grep -Fq 'name="vehicle_cmd_gate_param_path" value="$(var vehicle_cmd_gate_param_path)"' "${autoware_launch}" \
  && grep -Fq 'name="vehicle_cmd_gate_param_path" value="$(var vehicle_cmd_gate_param_path)"' "${control_component}"; then
  echo "Autoware vehicle command gate parameter override is already applied."
  exit 0
fi
if git -C "${repository}" apply --reverse --check "${patch_file}" 2>/dev/null; then
  echo "Autoware vehicle command gate parameter override is already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "Autoware vehicle command gate parameter override patch does not apply cleanly." >&2
  exit 1
fi
git -C "${repository}" apply "${patch_file}"
echo "Applied Autoware vehicle command gate parameter override."
