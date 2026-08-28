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
export AUTOWARE_E2E_SKIP_INSTALL=1
source scripts/e2e/env.sh

runtime_packages=(
  autoware_carla_interface
  autoware_external_cmd_selector
  autoware_perception_rviz_plugin
  autoware_planning_rviz_plugin
  autoware_raw_vehicle_cmd_converter
  autoware_shift_decider
  autoware_tensorrt_vad
  autoware_trajectory_follower_node
  autoware_twist2accel
  autoware_vehicle_cmd_gate
  autoware_vehicle_velocity_converter
  carla_sensor_kit_description
  sample_vehicle_launch
  sample_vehicle_description
  tier4_vehicle_launch
)

colcon build \
  --base-paths src autoware_e2e_vad_launch \
  --symlink-install \
  --parallel-workers "${COLCON_WORKERS:-3}" \
  --packages-up-to "${runtime_packages[@]}" \
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

# Reload through the project wrapper so ROS/colcon setup scripts are sourced
# with nounset temporarily disabled.
unset AUTOWARE_E2E_SKIP_INSTALL
source scripts/e2e/env.sh

# This package is launch/config data only. Its control YAML is copied from the
# pinned autoware_launch source without installing that package's full stack.
colcon build \
  --base-paths src autoware_e2e_vad_launch \
  --symlink-install \
  --packages-select autoware_e2e_vad_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
