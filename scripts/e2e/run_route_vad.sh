#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

route_file="${1:-}"
if [[ -z "${route_file}" ]]; then
  echo "Usage: $0 ROUTE_JSON [ros2 launch arguments...]" >&2
  exit 2
fi
shift
route_file="$(realpath "${route_file}")"
if [[ ! -f "${route_file}" ]]; then
  echo "Route file not found: ${route_file}" >&2
  exit 2
fi

mapfile -t route_values < <(
  python3 - "${route_file}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    route = json.load(stream)
print(route["town"])
print(route["spawn_point"])
print(route.get("spawn_point_reference", "base_link"))
PY
)
if [[ ${#route_values[@]} -ne 3 || "${route_values[2]}" != "base_link" ]]; then
  echo "Route spawn_point must use the base_link reference: ${route_file}" >&2
  exit 2
fi

export AUTOWARE_E2E_MAP_PATH="${route_values[0]}"
export CARLA_SPAWN_POINT="${route_values[1]}"
exec scripts/e2e/run_vad.sh \
  use_route_manager:=true \
  route_file:="${route_file}" \
  "$@" \
  spawn_point_reference:=base_link
