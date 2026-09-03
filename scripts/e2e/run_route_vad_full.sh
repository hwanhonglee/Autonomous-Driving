#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

source_route_file="${1:-}"
if [[ -z "${source_route_file}" ]]; then
  echo "Usage: $0 ROUTE_JSON [ros2 launch arguments...]" >&2
  exit 2
fi
shift
source_route_file="$(realpath "${source_route_file}")"
if [[ ! -f "${source_route_file}" ]]; then
  echo "Route file not found: ${source_route_file}" >&2
  exit 2
fi

for argument in "$@"; do
  case "${argument}" in
    use_carla_map_alignment:=*|carla_map_alignment_x_m:=*|carla_map_alignment_y_m:=*|carla_map_alignment_z_m:=*|carla_map_alignment_yaw_rad:=*)
      echo "Map alignment is controlled by map_bundle.json: ${argument%%:=*}" >&2
      exit 2
      ;;
    route_file:=*|map_path:=*|carla_map:=*|data_path:=*|carla_host:=*|carla_port:=*|spawn_point:=*|spawn_point_reference:=*|truth_initial_pose:=*|use_route_manager:=*)
      echo "Route/map launch argument is controlled by this wrapper: ${argument%%:=*}" >&2
      exit 2
      ;;
  esac
done

town="$({
  python3 - "${source_route_file}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    route = json.load(stream)
print(route["town"])
PY
} 2>/dev/null)"
if [[ -z "${town}" || "${town}" == *$'\n'* ]]; then
  echo "Route town metadata is invalid: ${source_route_file}" >&2
  exit 2
fi

map_path="${AUTOWARE_E2E_FULL_MAP_PATH:-${root}/data/maps/${town}_full}"
if [[ ! -f "${map_path}/lanelet2_map.osm" || \
      ! -f "${map_path}/pointcloud_map.pcd" || \
      ! -f "${map_path}/map_projector_info.yaml" || \
      ! -f "${map_path}/map_bundle.json" ]]; then
  echo "Complete Autoware map not found: ${map_path}" >&2
  echo "Required: lanelet2_map.osm, pointcloud_map.pcd, map_projector_info.yaml, map_bundle.json" >&2
  if [[ "${town}" == "Town01" ]]; then
    echo "Run the pinned packaged-map inventory/prepare procedure in docs/BEGINNER_QUICKSTART_KO.md section 9.2." >&2
  fi
  exit 2
fi

route_file="${source_route_file}"
alignment_enabled="false"
alignment_x_m="0.0"
alignment_y_m="0.0"
alignment_z_m="0.0"
alignment_yaw_rad="0.0"
map_bundle="${map_path}/map_bundle.json"
runtime_route_dir="${AUTOWARE_E2E_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/tmp/autoware_e2e_${UID}}/autoware_e2e}/aligned_routes"
alignment_output="$(
  python3 scripts/e2e/align_carla_route_to_map.py \
    "${source_route_file}" "${map_bundle}" \
    --runtime-dir "${runtime_route_dir}" \
    --shell-values
)"
mapfile -t alignment_values <<<"${alignment_output}"
if [[ ${#alignment_values[@]} -ne 6 ]]; then
  echo "Invalid map alignment output for ${map_bundle}" >&2
  exit 2
fi
route_file="${alignment_values[0]}"
alignment_enabled="${alignment_values[1]}"
alignment_x_m="${alignment_values[2]}"
alignment_y_m="${alignment_values[3]}"
alignment_z_m="${alignment_values[4]}"
alignment_yaw_rad="${alignment_values[5]}"
echo "Map alignment: route=${route_file}, x=${alignment_x_m}, y=${alignment_y_m}, z=${alignment_z_m}, yaw=${alignment_yaw_rad}" >&2

mapfile -t route_values < <(
  python3 - "${route_file}" <<'PY'
import json
import math
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    route = json.load(stream)
pose = route["start_ros_pose"]
half_yaw = float(pose["yaw"]) * 0.5
print(route["town"])
print(route["spawn_point"])
print(route.get("spawn_point_reference", "base_link"))
print(
    "[" + ",".join(
        f"{value:.9f}"
        for value in (
            float(pose["x"]),
            float(pose["y"]),
            float(pose["z"]),
            0.0,
            0.0,
            math.sin(half_yaw),
            math.cos(half_yaw),
        )
    ) + "]"
)
PY
)
if [[ ${#route_values[@]} -ne 4 || "${route_values[2]}" != "base_link" ]]; then
  echo "Route spawn_point must use the base_link reference: ${route_file}" >&2
  exit 2
fi
if [[ "${route_values[0]}" != "${town}" ]]; then
  echo "Aligned route changed town metadata: ${town} -> ${route_values[0]}" >&2
  exit 2
fi

python3 scripts/e2e/validate_route_map.py "${route_file}" "${map_path}"

exec ros2 launch autoware_e2e_vad_launch carla_vad_full.launch.xml \
  map_path:="${map_path}" \
  carla_map:="${town}" \
  data_path:="${AUTOWARE_E2E_DATA_PATH}" \
  carla_host:="${CARLA_HOST:-localhost}" \
  carla_port:="${CARLA_PORT:-2100}" \
  spawn_point:="${route_values[1]}" \
  truth_initial_pose:="${route_values[3]}" \
  use_carla_map_alignment:="${alignment_enabled}" \
  carla_map_alignment_x_m:="${alignment_x_m}" \
  carla_map_alignment_y_m:="${alignment_y_m}" \
  carla_map_alignment_z_m:="${alignment_z_m}" \
  carla_map_alignment_yaw_rad:="${alignment_yaw_rad}" \
  use_route_manager:=true \
  route_file:="${route_file}" \
  "$@" \
  spawn_point_reference:=base_link
