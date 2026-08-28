#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

map_path="${AUTOWARE_E2E_MAP_PATH:-Town01}"

exec ros2 launch autoware_e2e_vad_launch carla_vad.launch.xml \
  map_path:="${map_path}" \
  data_path:="${AUTOWARE_E2E_DATA_PATH}" \
  carla_host:="${CARLA_HOST:-localhost}" \
  carla_port:="${CARLA_PORT:-2000}" \
  spawn_point:="${CARLA_SPAWN_POINT:-None}" \
  "$@"
