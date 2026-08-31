#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_plugins_local_sdk_headers.patch"
cmake_file="${repository}/perception/autoware_tensorrt_plugins/CMakeLists.txt"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "TensorRT local-SDK header patch is missing: ${patch_file}" >&2
  exit 1
fi

if grep -q 'include_directories(SYSTEM ${TENSORRT_INCLUDE_DIRS})' "${cmake_file}" 2>/dev/null; then
  echo "TensorRT local-SDK header compatibility patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "TensorRT local-SDK header compatibility patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied TensorRT local-SDK header compatibility patch."
