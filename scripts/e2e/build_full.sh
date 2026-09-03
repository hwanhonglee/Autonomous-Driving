#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/e2e/build_full.sh

Build the full Autoware/CARLA VAD stack. This command accepts no positional
arguments. Configure it with environment variables:
  CMAKE_BUILD_PARALLEL_LEVEL=N     compiler jobs per package (default: 2)
  COLCON_WORKERS=N                 colcon workers (default: 2)
  AUTOWARE_E2E_FULL_BUILD_RESUME=1 skip packages already built, then rebuild
                                   the patched safety-critical runtime subset
EOF
}

if (( $# > 0 )); then
  if (( $# == 1 )) && [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
    exit 0
  fi
  echo "build_full.sh accepts no arguments: $*" >&2
  usage >&2
  exit 2
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "full build"

scripts/e2e/apply_vad_uuid_patch.sh
scripts/e2e/apply_vad_fp16_layer_norm_patch.sh
scripts/e2e/apply_vad_frame_assembly_patch.sh
scripts/e2e/apply_vad_causal_state_sync_patch.sh
scripts/e2e/apply_vad_bev_shift_modes_patch.sh
scripts/e2e/apply_vad_temporal_head_mode_patch.sh
scripts/e2e/apply_vad_object_safety_patches.sh
scripts/e2e/apply_mission_planner_lane_only_patch.sh
scripts/e2e/apply_tensorrt_system_headers_patch.sh
scripts/e2e/apply_tensorrt_unused_cublas_patch.sh
scripts/e2e/apply_tensorrt_local_sdk_headers_patch.sh
scripts/e2e/apply_tensorrt_consumer_system_headers_patch.sh
scripts/e2e/apply_carla_fast_sensor_patch.sh
scripts/e2e/apply_carla_sensor_frame_patch.sh
scripts/e2e/apply_carla_vehicle_status_patch.sh
scripts/e2e/apply_carla_base_link_pose_patch.sh
scripts/e2e/apply_carla_base_link_route_contract_patch.sh
scripts/e2e/apply_carla_imu_source_timestamp_patch.sh
scripts/e2e/apply_carla_runtime_timing_patch.sh
scripts/e2e/apply_carla_camera_qos_split_patch.sh
scripts/e2e/apply_autoware_launch_control_override.sh
scripts/e2e/apply_autoware_launch_vehicle_cmd_gate_override.sh
scripts/e2e/setup_tl_expected.sh
scripts/e2e/setup_cuda_12_8.sh
scripts/e2e/setup_tensorrt_10_8.sh
scripts/e2e/setup_spconv.sh
scripts/e2e/setup_acados.sh

export AUTOWARE_E2E_SKIP_INSTALL=1
source scripts/e2e/env.sh

# Full Autoware has several memory-heavy CUDA/C++ packages. Build packages one
# at a time and limit parallel compilation inside each package by default.
# colcon-cmake ignores CMAKE_BUILD_PARALLEL_LEVEL when it injects explicit
# make arguments, but honors a MAKEFLAGS job/load limit.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
if [[ ! "${CMAKE_BUILD_PARALLEL_LEVEL}" =~ ^[1-9][0-9]*$ ]]; then
  echo "CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer." >&2
  exit 2
fi
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}"

selection_args=(--packages-up-to autoware_e2e_vad_launch)
if [[ "${AUTOWARE_E2E_FULL_BUILD_RESUME:-0}" == "1" ]]; then
  selection_args+=(--packages-skip-build-finished)
fi

colcon build \
  --base-paths src autoware_e2e_vad_launch \
  --symlink-install \
  --executor sequential \
  --parallel-workers "${COLCON_WORKERS:-2}" \
  "${selection_args[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -Dament_cmake_auto_DIR="${AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR}" \
    -DCUDA_TOOLKIT_ROOT_DIR="${AUTOWARE_E2E_CUDA_ROOT}" \
    -DCMAKE_CUDA_COMPILER="${CUDACXX}" \
    -DTENSORRT_ROOT="${TENSORRT_ROOT}" \
    -DNVINFER="${TENSORRT_ROOT}/lib64/libnvinfer.so" \
    -DNVONNXPARSER="${TENSORRT_ROOT}/lib64/libnvonnxparser.so" \
    -Dcumm_DIR="${AUTOWARE_E2E_SPCONV_ROOT}/share/cmake/cumm" \
    -Dspconv_DIR="${AUTOWARE_E2E_SPCONV_ROOT}/lib/cmake/spconv"

if [[ "${AUTOWARE_E2E_FULL_BUILD_RESUME:-0}" == "1" ]]; then
  # The global resume may skip this patched runtime. Rebuild it explicitly so
  # a stale library cannot be admitted under fresh source provenance.
  colcon build \
    --base-paths src \
    --symlink-install \
    --executor sequential \
    --packages-select \
      autoware_autonomous_emergency_braking \
      autoware_lanelet2_extension \
      autoware_route_handler \
      autoware_mission_planner_universe \
      autoware_tensorrt_vad \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -Dament_cmake_auto_DIR="${AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR}"
fi

# Reconfigure only the project package with its focused tests enabled after all
# full-stack runtime dependencies have been installed. Restrict package
# discovery to this data-only package so stale/partial exec dependency install
# markers cannot block its independent CMake build.
colcon build \
  --base-paths autoware_e2e_vad_launch \
  --symlink-install \
  --executor sequential \
  --parallel-workers "${COLCON_WORKERS:-2}" \
  --packages-select autoware_e2e_vad_launch \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
    -Dament_cmake_auto_DIR="${AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR}"

unset AUTOWARE_E2E_SKIP_INSTALL
source scripts/e2e/env.sh
python3 scripts/e2e/mission_planner_build_provenance.py capture
python3 scripts/e2e/vad_object_safety_build_provenance.py capture
