#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
  cat >&2 <<'EOF'
Usage: run_carla_actuation_calibration.sh [options] OUTPUT_DIR

Options:
  --host HOST          Existing CARLA server host (default: localhost)
  --port PORT          Existing CARLA server port (default: 2100)
  --map MAP            Verified calibration map; Phase 1 supports Town01 only
  --timeout SEC        Maximum sweep wall time (default: 1800)

The helper never starts or stops the CARLA server. It owns only the calibration
launch process group and the actor role `autoware_e2e_calibration` that it creates.
Phase 1 requires an existing Town01 world with no dynamic actors and restores the
server's original synchronous-mode and fixed-delta settings on exit.
EOF
}

host="localhost"
port="2100"
carla_map="Town01"
timeout_sec="1800"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      host="$2"
      shift 2
      ;;
    --port)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      port="$2"
      shift 2
      ;;
    --map)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      carla_map="$2"
      shift 2
      ;;
    --timeout)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      timeout_sec="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

[[ $# -eq 1 ]] || { usage; exit 2; }
[[ "${port}" =~ ^[1-9][0-9]*$ ]] || { echo "Port must be positive." >&2; exit 2; }
[[ "${timeout_sec}" =~ ^[1-9][0-9]*$ ]] || {
  echo "Timeout must be a positive integer." >&2
  exit 2
}
if [[ "${carla_map}" != "Town01" ]]; then
  echo "Phase 1 uses a verified Town01 reset pose; other maps are not supported yet." >&2
  exit 2
fi

output_dir="$(realpath -m -- "$1")"
if [[ -e "${output_dir}" || -L "${output_dir}" ]]; then
  echo "Output path already exists: ${output_dir}" >&2
  exit 2
fi

cd "${root}"
source scripts/e2e/env.sh

python3 - "${host}" "${port}" <<'PY'
import socket
import sys

with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=3.0):
    pass
PY

lock_host="${host//[^A-Za-z0-9_.-]/_}"
exec 9>"/tmp/autoware_e2e_carla_${lock_host}_${port}.lock"
if ! flock -n 9; then
  echo "Another project helper owns CARLA ${host}:${port}." >&2
  exit 1
fi

conflicts="$(
  ros2 node list --no-daemon 2>/dev/null | \
    grep -E '/(autoware_carla_interface|carla_ros2_interface|carla_actuation_sweep|autoware_raw_vehicle_cmd_converter|vad_route_manager|vad_carla_tiny)$' || true
)"
if [[ -n "${conflicts}" ]]; then
  echo "A project stack is already visible in ROS domain ${ROS_DOMAIN_ID}:" >&2
  echo "${conflicts}" >&2
  exit 1
fi

actor_guard() {
  local mode="$1"
  python3 - "${mode}" "${host}" "${port}" "${carla_map}" "${state_file}" <<'PY'
import json
import sys

import carla

mode, host, raw_port, requested_map, state_file = sys.argv[1:]
client = carla.Client(host, int(raw_port))
client.set_timeout(4.0)
world = client.get_world()
calibration_actors = [
    actor
    for actor in world.get_actors().filter("vehicle.*")
    if actor.attributes.get("role_name", "") == "autoware_e2e_calibration"
]
if mode == "preflight":
    dynamic_actors = []
    for pattern in ("vehicle.*", "sensor.*", "walker.pedestrian.*", "controller.ai.walker"):
        dynamic_actors.extend(world.get_actors().filter(pattern))
    if dynamic_actors:
        details = ", ".join(
            f"id={actor.id},type={actor.type_id},role={actor.attributes.get('role_name', '')}"
            for actor in dynamic_actors
        )
        raise SystemExit(f"CARLA world is already in use: {details}")
    current_map = world.get_map().name.rsplit("/", 1)[-1]
    if current_map != requested_map:
        raise SystemExit(
            f"CARLA server is on {current_map}; Phase 1 requires an existing {requested_map} world"
        )
    settings = world.get_settings()
    with open(state_file, "w", encoding="utf-8") as stream:
        json.dump(
            {
                "map": current_map,
                "synchronous_mode": settings.synchronous_mode,
                "fixed_delta_seconds": settings.fixed_delta_seconds,
            },
            stream,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")
elif mode == "cleanup":
    for actor in calibration_actors:
        actor.destroy()
    with open(state_file, encoding="utf-8") as stream:
        original = json.load(stream)
    current_map = world.get_map().name.rsplit("/", 1)[-1]
    if current_map == original["map"]:
        settings = world.get_settings()
        settings.synchronous_mode = original["synchronous_mode"]
        settings.fixed_delta_seconds = original["fixed_delta_seconds"]
        world.apply_settings(settings)
else:
    raise SystemExit(f"Unknown actor guard mode: {mode}")
PY
}

# The interface changes world timing, so preserve it and require exclusive use.
state_file="$(mktemp /tmp/autoware_e2e_carla_settings.XXXXXX.json)"
trap 'rm -f -- "${state_file}"' EXIT
if ! actor_guard preflight; then
  exit 1
fi

mkdir -p "${output_dir}"
cp -- "${state_file}" "${output_dir}/carla_original_settings.json"
raw_csv="${output_dir}/raw_samples.csv"
sweep_summary="${output_dir}/sweep_summary.json"
stack_log="${output_dir}/stack.log"
printf 'ROS_DOMAIN_ID=%s\nCARLA_HOST=%s\nCARLA_PORT=%s\nCARLA_MAP=%s\n' \
  "${ROS_DOMAIN_ID}" "${host}" "${port}" "${carla_map}" > "${output_dir}/runtime.env"

stack_pid=""
cleaned=false

stop_group() {
  local pid="$1"
  local timeout="$2"
  if [[ -z "${pid}" ]] || ! kill -0 -- "-${pid}" 2>/dev/null; then
    return
  fi
  kill -INT -- "-${pid}" 2>/dev/null || true
  local deadline=$((SECONDS + timeout))
  while kill -0 -- "-${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
  deadline=$((SECONDS + 5))
  while kill -0 -- "-${pid}" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

publish_emergency_brake() {
  if [[ -z "${stack_pid}" ]] || ! kill -0 -- "-${stack_pid}" 2>/dev/null; then
    return
  fi
  timeout 3 ros2 topic pub --once \
    /calibration/actuation/emergency_stop std_msgs/msg/Bool \
    '{data: true}' >/dev/null 2>&1 || true
  timeout 4 ros2 topic pub -r 20 -t 20 \
    /control/command/actuation_cmd tier4_vehicle_msgs/msg/ActuationCommandStamped \
    '{header: {frame_id: base_link}, actuation: {accel_cmd: 0.0, brake_cmd: 1.0, steer_cmd: 0.0}}' \
    >/dev/null 2>&1 || true
}

cleanup() {
  if [[ "${cleaned}" == "true" ]]; then
    return
  fi
  cleaned=true
  publish_emergency_brake
  stop_group "${stack_pid}" 30
  stack_pid=""
  actor_guard cleanup >/dev/null 2>&1 || true
  rm -f -- "${state_file}"
}

on_signal() {
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

setsid ros2 launch autoware_e2e_vad_launch carla_actuation_calibration.launch.xml \
  carla_host:="${host}" \
  carla_port:="${port}" \
  carla_map:="${carla_map}" \
  output_csv:="${raw_csv}" \
  summary_json:="${sweep_summary}" \
  > "${stack_log}" 2>&1 &
stack_pid=$!

deadline=$((SECONDS + timeout_sec))
while [[ ! -f "${sweep_summary}" ]]; do
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "Calibration launch exited before writing its summary." >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "Calibration sweep timed out after ${timeout_sec}s." >&2
    exit 1
  fi
  sleep 1
done

sweep_status="$(python3 - "${sweep_summary}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    print(json.load(handle).get("status", "missing"))
PY
)"

cleanup
trap - EXIT INT TERM

carla_share="$(ros2 pkg prefix autoware_carla_interface)/share/autoware_carla_interface"
original_accel="$(realpath "${carla_share}/accel_map.csv")"
original_brake="$(realpath "${carla_share}/brake_map.csv")"

analysis_args=(
  --raw-csv "${raw_csv}"
  --sweep-summary "${sweep_summary}"
  --output-dir "${output_dir}/analysis"
  --original-accel-map "${original_accel}"
  --original-brake-map "${original_brake}"
  --min-samples 30
  --min-repeats 3
)
if [[ "${sweep_status}" == "completed" ]]; then
  analysis_args+=(--candidate-dir "${output_dir}/candidate")
fi

analysis_status=0
python3 scripts/e2e/fit_validate_carla_actuation_map.py \
  "${analysis_args[@]}" || analysis_status=$?

if [[ "${sweep_status}" != "completed" ]]; then
  echo "Calibration sweep failed; see ${sweep_summary} and ${stack_log}." >&2
  exit 1
fi
if (( analysis_status != 0 )); then
  echo "Sweep completed, but response analysis failed with ${analysis_status}." >&2
  exit "${analysis_status}"
fi

echo "CARLA actuation calibration completed: ${output_dir}"
