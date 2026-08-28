#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_vad_temporal_head_mode.patch"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "VAD temporal head mode patch is missing: ${patch_file}" >&2
  exit 1
fi

package="${repository}/e2e/autoware_tensorrt_vad"
policy="${package}/src/temporal_head_policy.hpp"
model="${package}/src/vad_model.hpp"
node="${package}/src/vad_node.cpp"
test_source="${package}/test/test_temporal_head_policy.cpp"

if grep -q "class TemporalHeadPolicy" "${policy}" 2>/dev/null && \
   grep -q "skipping head engine initialization" "${model}" 2>/dev/null && \
   grep -q 'model_params.use_temporal_head", true' "${node}" 2>/dev/null && \
   grep -q "StatelessModeNeverInitializesOrAdvancesPrevBev" "${test_source}" 2>/dev/null; then
  echo "VAD runtime-selectable temporal head mode patch is already applied."
  exit 0
fi

if ! grep -q "class SynchronizedFrameBuffer" "${package}/src/synchronization_strategy.hpp" \
  2>/dev/null || ! grep -q "enum class BEVShiftMode" \
  "${package}/src/input_converter/bev_shift_converter.hpp" 2>/dev/null; then
  echo "Apply the VAD frame assembly and BEV shift mode patches first." >&2
  exit 1
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "VAD temporal head mode patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied runtime-selectable VAD temporal head mode patch."
