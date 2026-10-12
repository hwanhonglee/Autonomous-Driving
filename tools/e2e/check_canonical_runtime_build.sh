#!/usr/bin/env bash
# HH_260906 - Compile in a fresh canonical prefix without installers, patchers, or overwriting the active runtime.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"
source tools/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "canonical runtime build verification"
out="$root/autonomy/canonical_rebuild"
[[ ! -e "$out" ]] || { echo "Fresh output required: $out" >&2; exit 2; }
export AUTOWARE_E2E_SKIP_INSTALL=1
# HH_260906 - Discard inherited vendor roots so old aliases cannot silently choose the compiler inputs.
unset AUTOWARE_E2E_TL_EXPECTED_ROOT AUTOWARE_E2E_DATA_PATH AUTOWARE_E2E_SPCONV_ROOT AUTOWARE_E2E_ACADOS_ROOT
source tools/e2e/env.sh
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS='-j2 -l2'
packages=(autoware_autonomous_emergency_braking autoware_carla_interface autoware_external_cmd_selector
  autoware_lanelet2_extension autoware_map_loader autoware_mission_planner_universe
  autoware_perception_rviz_plugin autoware_planning_rviz_plugin autoware_raw_vehicle_cmd_converter
  autoware_route_handler autoware_shift_decider autoware_tensorrt_vad autoware_trajectory_follower_node
  autoware_twist2accel autoware_vehicle_cmd_gate autoware_vehicle_velocity_converter
  carla_sensor_kit_description sample_vehicle_launch sample_vehicle_description tier4_vehicle_launch)
colcon --log-base "$out/log" build \
  --base-paths autonomy/src autonomy/autoware_e2e_vad_launch \
  --build-base "$out/build" --install-base "$out/install" --symlink-install \
  --parallel-workers 2 --packages-up-to "${packages[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -Dament_cmake_auto_DIR="$AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR" \
  -DCUDA_TOOLKIT_ROOT_DIR="$AUTOWARE_E2E_CUDA_ROOT" -DCMAKE_CUDA_COMPILER="$CUDACXX" \
  -DTENSORRT_ROOT="$TENSORRT_ROOT" -DNVINFER="$TENSORRT_ROOT/lib64/libnvinfer.so" \
  -DNVONNXPARSER="$TENSORRT_ROOT/lib64/libnvonnxparser.so" \
  -Dcumm_DIR="$AUTOWARE_E2E_SPCONV_ROOT/share/cmake/cumm" \
  -Dspconv_DIR="$AUTOWARE_E2E_SPCONV_ROOT/lib/cmake/spconv"
# HH_260906 - This is only compile evidence. Never switch the active installation or delete aliases here.
echo CANONICAL_RUNTIME_BUILD_COMPLETE_NOT_ACTIVATED
