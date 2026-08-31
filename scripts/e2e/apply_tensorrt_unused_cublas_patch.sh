#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_plugins_remove_unused_cublas.patch"
header="${repository}/perception/autoware_tensorrt_plugins/include/autoware/multi_scale_deform_attn_ops/ms_deform_attn_kernel.hpp"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "TensorRT unused-cuBLAS patch is missing: ${patch_file}" >&2
  exit 1
fi

if ! grep -q '#include <cublas_v2.h>' "${header}" 2>/dev/null; then
  echo "TensorRT unused-cuBLAS compatibility patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "TensorRT unused-cuBLAS compatibility patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied TensorRT unused-cuBLAS compatibility patch."
