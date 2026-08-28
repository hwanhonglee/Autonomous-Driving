#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_vad_causal_state_sync.patch"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "VAD causal state synchronization patch is missing: ${patch_file}" >&2
  exit 1
fi

header="${repository}/e2e/autoware_tensorrt_vad/src/synchronization_strategy.hpp"
source="${repository}/e2e/autoware_tensorrt_vad/lib/synchronization_strategy.cpp"
test_source="${repository}/e2e/autoware_tensorrt_vad/test/test_frame_assembly.cpp"
if grep -q "max_vehicle_state_samples" "${header}" 2>/dev/null && \
   grep -q "find_latest_causal_kinematic_state" "${source}" 2>/dev/null && \
   grep -q "kinematic_states_.upper_bound" "${source}" 2>/dev/null && \
   grep -q "SelectsLatestCausalVehicleStateDespiteOutOfOrderCallbacks" \
     "${test_source}" 2>/dev/null && \
   grep -q "WaitsForCausalVehicleStateAndRetainsFutureSamplesForNextFrame" \
     "${test_source}" 2>/dev/null; then
  echo "VAD causal vehicle-state synchronization patch is already applied."
  exit 0
fi

if ! grep -q "class SynchronizedFrameBuffer" "${header}" 2>/dev/null || \
   ! grep -q "ToleranceOneExactPathPreservesFramesAcrossInterleavedFutureCallbacks" \
     "${test_source}" 2>/dev/null; then
  echo "Apply scripts/e2e/apply_vad_frame_assembly_patch.sh first." >&2
  exit 1
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "VAD causal state synchronization patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied VAD causal vehicle-state synchronization patch."
