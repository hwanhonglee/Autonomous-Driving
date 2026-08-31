#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_common_system_headers.patch"
cmake_file="${repository}/perception/autoware_tensorrt_common/CMakeLists.txt"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "TensorRT system-header patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q 'target_include_directories(${PROJECT_NAME} SYSTEM' "${cmake_file}" 2>/dev/null; then
  echo "TensorRT system-header compatibility patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "TensorRT system-header compatibility patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied TensorRT system-header compatibility patch."
