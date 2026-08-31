#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_consumers_system_headers.patch"

if [[ ! -d "${repository}/.git" || ! -f "${patch_file}" ]]; then
  echo "Autoware Universe repository or TensorRT consumer patch is missing." >&2
  exit 1
fi
if git -C "${repository}" apply --reverse --check "${patch_file}" 2>/dev/null; then
  echo "TensorRT consumer system-header compatibility patch is already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "TensorRT consumer system-header compatibility patch does not apply cleanly." >&2
  exit 1
fi
git -C "${repository}" apply "${patch_file}"
echo "Applied TensorRT consumer system-header compatibility patch."
