#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/launcher/autoware_launch"
patch_file="${root}/patches/autoware_launch_lateral_param_override.patch"

if [[ ! -d "${repository}/.git" || ! -f "${patch_file}" ]]; then
  echo "Autoware launch repository or control override patch is missing." >&2
  exit 1
fi
if grep -Fq 'name="use_lateral_controller_param_override"' \
    "${repository}/autoware_launch/launch/autoware.launch.xml" \
  && grep -Fq 'name="resolved_lateral_controller_param_path"' \
    "${repository}/autoware_launch/launch/components/tier4_control_component.launch.xml" \
  && grep -Fq 'name="use_longitudinal_controller_param_override"' \
    "${repository}/autoware_launch/launch/autoware.launch.xml" \
  && grep -Fq 'name="resolved_longitudinal_controller_param_path"' \
    "${repository}/autoware_launch/launch/components/tier4_control_component.launch.xml"; then
  echo "Autoware lateral and longitudinal controller parameter overrides are already applied."
  exit 0
fi
if git -C "${repository}" apply --reverse --check "${patch_file}" 2>/dev/null; then
  echo "Autoware lateral and longitudinal controller parameter overrides are already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "Autoware lateral controller parameter override patch does not apply cleanly." >&2
  exit 1
fi
git -C "${repository}" apply "${patch_file}"
echo "Applied Autoware lateral and longitudinal controller parameter overrides."
