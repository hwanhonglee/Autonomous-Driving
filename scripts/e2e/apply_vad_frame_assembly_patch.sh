#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/universe/autoware_universe"
patch_file="${root}/patches/autoware_tensorrt_vad_frame_assembly.patch"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Universe repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "VAD frame assembly patch is missing: ${patch_file}" >&2
  exit 1
fi

frame_header="${repository}/e2e/autoware_tensorrt_vad/src/synchronization_strategy.hpp"
startup_header="${repository}/e2e/autoware_tensorrt_vad/src/runtime_startup.hpp"
runtime_config_header="${repository}/e2e/autoware_tensorrt_vad/src/runtime_configuration.hpp"
frame_config="${repository}/e2e/autoware_tensorrt_vad/config/vad_carla_tiny.param.yaml"
frame_test="${repository}/e2e/autoware_tensorrt_vad/test/test_frame_assembly.cpp"
startup_test="${repository}/e2e/autoware_tensorrt_vad/test/test_runtime_startup.cpp"
if grep -q "class SynchronizedFrameBuffer" "${frame_header}" 2>/dev/null && \
   grep -q "find_nearest_image" "${frame_header}" 2>/dev/null && \
   grep -q "find_newest_exact_frame" "${frame_header}" 2>/dev/null && \
   grep -q "class VadRuntimeStartup" "${startup_header}" 2>/dev/null && \
   grep -q "parse_image_reliability" "${runtime_config_header}" 2>/dev/null && \
   grep -q "image_queue_depth: 8" "${frame_config}" 2>/dev/null && \
   grep -q 'image_reliability: "best_effort"' "${frame_config}" 2>/dev/null && \
   grep -q "CombinesJitteredCamerasAroundFrontAnchor" "${frame_test}" 2>/dev/null && \
   grep -q "ToleranceOneExactPathPreservesFramesAcrossInterleavedFutureCallbacks" \
     "${frame_test}" 2>/dev/null && \
   grep -q "ParsesSupportedImageReliabilityValues" "${startup_test}" 2>/dev/null && \
   [[ -f "${startup_test}" ]]; then
  echo "VAD synchronized frame assembly patch is already applied."
  exit 0
fi

# The CMake hunk is based on the optional LayerNorm test block already present in this setup.
if ! grep -q "should_force_layer_norm_fp32" \
  "${repository}/e2e/autoware_tensorrt_vad/lib/networks/net.cpp" 2>/dev/null; then
  echo "Apply scripts/e2e/apply_vad_fp16_layer_norm_patch.sh first." >&2
  exit 1
fi

if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "VAD synchronized frame assembly patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied VAD synchronized frame assembly patch."
