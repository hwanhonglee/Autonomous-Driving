#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"

scripts/e2e/apply_vad_uuid_patch.sh
scripts/e2e/apply_vad_fp16_layer_norm_patch.sh
scripts/e2e/apply_vad_frame_assembly_patch.sh
scripts/e2e/apply_vad_causal_state_sync_patch.sh
scripts/e2e/apply_vad_bev_shift_modes_patch.sh
scripts/e2e/apply_vad_temporal_head_mode_patch.sh
scripts/e2e/apply_carla_fast_sensor_patch.sh
scripts/e2e/apply_carla_sensor_frame_patch.sh
scripts/e2e/apply_carla_vehicle_status_patch.sh
scripts/e2e/apply_carla_base_link_pose_patch.sh
scripts/e2e/apply_carla_base_link_route_contract_patch.sh
scripts/e2e/apply_carla_imu_source_timestamp_patch.sh
scripts/e2e/apply_autoware_launch_control_override.sh
scripts/e2e/setup_cuda_12_8.sh
scripts/e2e/setup_spconv.sh
scripts/e2e/setup_acados.sh

export AUTOWARE_E2E_SKIP_INSTALL=1
source scripts/e2e/env.sh

# Full Autoware has several memory-heavy CUDA/C++ packages. Build packages one
# at a time and limit parallel compilation inside each package by default.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"

selection_args=(--packages-up-to autoware_e2e_vad_launch)
if [[ "${AUTOWARE_E2E_FULL_BUILD_RESUME:-0}" == "1" ]]; then
  selection_args+=(--packages-skip-build-finished)
fi

colcon build \
  --base-paths src autoware_e2e_vad_launch \
  --symlink-install \
  --executor sequential \
  "${selection_args[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCUDA_TOOLKIT_ROOT_DIR="${AUTOWARE_E2E_CUDA_ROOT}" \
    -DCMAKE_CUDA_COMPILER="${CUDACXX}" \
    -DTENSORRT_ROOT="${TENSORRT_ROOT}" \
    -DNVINFER="${TENSORRT_ROOT}/lib64/libnvinfer.so" \
    -DNVONNXPARSER="${TENSORRT_ROOT}/lib64/libnvonnxparser.so" \
    -Dcumm_DIR="${AUTOWARE_E2E_SPCONV_ROOT}/share/cmake/cumm" \
    -Dspconv_DIR="${AUTOWARE_E2E_SPCONV_ROOT}/lib/cmake/spconv"

# Reconfigure only the project package with its focused tests enabled after all
# full-stack runtime dependencies have been installed.
unset AUTOWARE_E2E_SKIP_INSTALL
source scripts/e2e/env.sh

colcon build \
  --base-paths src autoware_e2e_vad_launch \
  --symlink-install \
  --executor sequential \
  --packages-select autoware_e2e_vad_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
