#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

failures=0

check_file() {
  if [[ -e "$1" ]]; then
    printf '[ok] %s\n' "$1"
  else
    printf '[missing] %s\n' "$1"
    failures=$((failures + 1))
  fi
}

printf 'Autoware root: %s\nROS distro: %s\nROS domain: %s\n' \
  "${AUTOWARE_E2E_ROOT}" "${ROS_DISTRO}" "${ROS_DOMAIN_ID}"
if gpu_info="$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>&1)"; then
  printf '[ok] NVIDIA GPU: %s\n' "${gpu_info}"
else
  printf '[gpu unavailable] %s\n' "${gpu_info}"
  printf 'A driver/library version mismatch normally requires a reboot.\n'
  failures=$((failures + 1))
fi

check_file "${CARLA_ROOT}/CarlaUE4.sh"
check_file "${AUTOWARE_E2E_SPCONV_ROOT}/lib/libspconv.so"
check_file "${AUTOWARE_E2E_DATA_PATH}/vad/v0.1/vad-carla-tiny_backbone.onnx"
check_file "${AUTOWARE_E2E_DATA_PATH}/vad/v0.1/vad-carla-tiny_head.onnx"
check_file "${AUTOWARE_E2E_DATA_PATH}/vad/v0.1/vad-carla-tiny_head_no_prev.onnx"
check_file "${AUTOWARE_E2E_DATA_PATH}/vad/v0.1/vad-carla-tiny.param.json"
python3 -c 'import carla; print("[ok] CARLA Python API:", carla.__file__)'

for package in \
  autoware_carla_interface \
  autoware_perception_rviz_plugin \
  autoware_planning_rviz_plugin \
  autoware_tensorrt_vad \
  autoware_e2e_vad_launch; do
  if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    printf '[ok] ROS package: %s\n' "${package}"
  else
    printf '[not built] ROS package: %s\n' "${package}"
    failures=$((failures + 1))
  fi
done

plugin_library="${root}/install/autoware_tensorrt_plugins/share/autoware_tensorrt_plugins/plugins/libautoware_tensorrt_plugins.so"
check_file "${plugin_library}"

if [[ -f "${root}/install/autoware_tensorrt_vad/lib/libautoware_tensorrt_vad_lib.so" ]]; then
  if ldd "${root}/install/autoware_tensorrt_vad/lib/libautoware_tensorrt_vad_lib.so" | grep -q 'libnvinfer.so.10'; then
    printf '[ok] VAD links TensorRT 10\n'
  else
    printf '[wrong version] VAD does not link TensorRT 10\n'
    failures=$((failures + 1))
  fi
fi

exit "${failures}"
