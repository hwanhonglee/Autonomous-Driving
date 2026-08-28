#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_vad_fp16_layer_norm.patch"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "VAD FP16 LayerNorm patch is missing: ${patch_file}" >&2
  exit 1
fi

policy_source="${repository}/e2e/autoware_tensorrt_vad/lib/networks/net.cpp"
policy_header="${repository}/e2e/autoware_tensorrt_vad/src/networks/net.hpp"
policy_test="${repository}/e2e/autoware_tensorrt_vad/test/test_layer_norm_precision.cpp"
if grep -q "should_force_layer_norm_fp32" "${policy_source}" 2>/dev/null && \
   grep -q "is_layer_norm_related_layer" "${policy_header}" 2>/dev/null && \
   [[ -f "${policy_test}" ]]; then
  echo "VAD FP16 LayerNorm precision patch is already applied."
  exit 0
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "VAD FP16 LayerNorm precision patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied VAD FP16 LayerNorm precision patch."
