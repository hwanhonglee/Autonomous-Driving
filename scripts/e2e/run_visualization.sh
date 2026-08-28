#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

fast=false
if [[ "${1:-}" == "--fast" ]]; then
  fast=true
  shift
fi

config="$(ros2 pkg prefix autoware_e2e_vad_launch)/share/autoware_e2e_vad_launch/rviz/vad_carla.rviz"
if [[ ! -f "${config}" ]]; then
  echo "RViz config is not installed. Run scripts/e2e/build.sh first." >&2
  exit 1
fi

camera_topic="/sensing/camera/all_cameras/image_raw"
if [[ "${fast}" == "true" ]]; then
  camera_topic="/sensing/camera/CAM_FRONT/image_raw"
fi

rviz2 \
  --display-title-format 'Autoware E2E VAD - {CONFIG_FILENAME}' \
  -d "${config}" \
  "$@" \
  --ros-args -p use_sim_time:=true &
rviz_pid=$!

ros2 run rqt_image_view rqt_image_view \
  --clear-config "${camera_topic}" &
camera_pid=$!

cleanup() {
  kill "${camera_pid}" "${rviz_pid}" 2>/dev/null || true
  wait "${camera_pid}" "${rviz_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

exited_pid=""
set +e
wait -n -p exited_pid "${rviz_pid}" "${camera_pid}"
status=$?
set -e

if [[ "${exited_pid:-}" == "${camera_pid}" ]]; then
  echo "rqt_image_view exited; stopping the visualization session." >&2
  if [[ ${status} -eq 0 ]]; then
    status=1
  fi
fi

exit "${status}"
