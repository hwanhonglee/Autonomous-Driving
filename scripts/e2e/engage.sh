#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

state="${1:-true}"
if [[ "${state}" != "true" && "${state}" != "false" ]]; then
  echo "Usage: $0 [true|false]" >&2
  exit 2
fi

timeout 30 ros2 service call \
  /vehicle_cmd_gate/service/engage \
  tier4_external_api_msgs/srv/Engage \
  "{engage: ${state}}"
