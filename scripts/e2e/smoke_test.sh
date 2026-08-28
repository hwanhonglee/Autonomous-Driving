#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

timeout_seconds="${AUTOWARE_E2E_SMOKE_TIMEOUT:-15}"
failures=0

check_message() {
  local topic="$1"
  if timeout "${timeout_seconds}" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    printf '[ok] message: %s\n' "${topic}"
  else
    printf '[missing] message: %s\n' "${topic}"
    failures=$((failures + 1))
  fi
}

camera_names=(
  CAM_FRONT
  CAM_BACK
  CAM_FRONT_LEFT
  CAM_BACK_LEFT
  CAM_FRONT_RIGHT
  CAM_BACK_RIGHT
)

for camera in "${camera_names[@]}"; do
  check_message "/sensing/camera/${camera}/image_raw/compressed"
done

topics=(
  /localization/kinematic_state
  /localization/acceleration
  /planning/trajectory
  /planning/vad/candidate_trajectories
  /perception/object_recognition/objects
  /perception/vad/map_points
  /trajectory_follower/control_cmd
  /control/command/actuation_cmd
)

for topic in "${topics[@]}"; do
  check_message "${topic}"
done

if ! python3 scripts/e2e/validate_vad_outputs.py --timeout "${timeout_seconds}"; then
  failures=$((failures + 1))
fi

exit "${failures}"
