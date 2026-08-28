#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

sensor_mapping_options=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sensor-mapping)
      if [[ $# -lt 2 ]]; then
        echo "--sensor-mapping requires a YAML file." >&2
        exit 2
      fi
      sensor_mapping_options=(--sensor-mapping "$2")
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--sensor-mapping YAML] ROUTE_JSON [ros2 launch arguments...]" >&2
      echo "Runs the verified lightweight VAD profile with nominal Smart MPC iLQR." >&2
      exit 0
      ;;
    -* )
      echo "Unknown option: $1" >&2
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

route_file="${1:-}"
if [[ -z "${route_file}" ]]; then
  echo "Usage: $0 [--sensor-mapping YAML] ROUTE_JSON [ros2 launch arguments...]" >&2
  echo "Runs the verified lightweight VAD profile with nominal Smart MPC iLQR." >&2
  exit 2
fi
shift

if ! ros2 pkg prefix autoware_smart_mpc_trajectory_follower >/dev/null 2>&1; then
  echo "Smart MPC is not installed. Run scripts/e2e/build_smart_mpc.sh first." >&2
  exit 1
fi

preset="$(ros2 pkg prefix autoware_launch)/share/autoware_launch/config/control/preset/smart_mpc_nominal_ilqr_preset.yaml"
if [[ ! -f "${preset}" ]]; then
  echo "Smart MPC control preset is missing. Run scripts/e2e/build_smart_mpc.sh first." >&2
  exit 1
fi

exec scripts/e2e/run_route_vad_fast.sh --full \
  "${sensor_mapping_options[@]}" \
  "${route_file}" \
  "$@" \
  control_module_preset:=smart_mpc_nominal_ilqr
