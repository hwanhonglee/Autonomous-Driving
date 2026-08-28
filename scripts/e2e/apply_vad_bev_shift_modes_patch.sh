#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_vad_bev_shift_modes.patch"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "VAD BEV shift modes patch is missing: ${patch_file}" >&2
  exit 1
fi

header="${repository}/e2e/autoware_tensorrt_vad/src/input_converter/bev_shift_converter.hpp"
source="${repository}/e2e/autoware_tensorrt_vad/lib/input_converter/bev_shift_converter.cpp"
node="${repository}/e2e/autoware_tensorrt_vad/src/vad_node.cpp"
test_source="${repository}/e2e/autoware_tensorrt_vad/test/test_bev_shift_contract.cpp"

if grep -q "enum class BEVShiftMode" "${header}" 2>/dev/null && \
   grep -q "BEVShiftMode::RosCorrected" "${source}" 2>/dev/null && \
   grep -q 'interface_params.bev_shift_mode", "legacy"' "${node}" 2>/dev/null && \
   grep -q "RosCorrectedMapsForwardAndLeftToOnnxXY" "${test_source}" 2>/dev/null; then
  echo "VAD runtime-selectable BEV shift modes patch is already applied."
  exit 0
fi

if ! grep -q "class SynchronizedFrameBuffer" \
  "${repository}/e2e/autoware_tensorrt_vad/src/synchronization_strategy.hpp" 2>/dev/null; then
  echo "Apply scripts/e2e/apply_vad_frame_assembly_patch.sh first." >&2
  exit 1
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "VAD BEV shift modes patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied runtime-selectable VAD BEV shift modes patch."
