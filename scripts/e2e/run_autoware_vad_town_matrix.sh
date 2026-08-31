#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
matrix_manifest="${root}/scripts/e2e/autoware_vad_town_matrix.yaml"
matrix_tool="${root}/scripts/e2e/autoware_vad_town_matrix.py"
catalog_tool="${root}/scripts/e2e/prepare_carla_expert_route_catalog.py"
server_tool="${root}/scripts/e2e/run_carla_map.sh"
trial_tool="${root}/scripts/e2e/run_recorded_route_trial.sh"

usage() {
  cat <<'EOF'
Usage: scripts/e2e/run_autoware_vad_town_matrix.sh OUTPUT_ROOT [OPTIONS]

Run the same full-stack Autoware VAD recommended+visualized profile on every
map admitted by autoware_vad_town_matrix.yaml. Each map is cold-started without
client.load_world and receives two separate trials: one true STRAIGHT-command
route and one true LEFT/RIGHT-command turn route. Route analysis, ROS bags,
Autoware/RViz full-screen PNG/GIF evidence, per-trial validation, and an all-map
aggregate report are retained.

Maps without a validated Lanelet2 + PCD full-map bundle are reported BLOCKED;
they are never mislabeled as executed VAD trials.

Options:
  --port PORT                 CARLA RPC port (default: 2100)
  --maps ID[,ID...]           run this subset of admitted maps
  --resume                    strictly validate and skip completed trials
  --fail-fast                 stop after the first trial failure
  --startup-timeout-sec SEC   CARLA cold-start timeout (default: 180)
  -h, --help                  show this help
EOF
}

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

output_root="$1"
shift
port=2100
resume=0
fail_fast=0
startup_timeout_sec=180
selected_csv=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      port="$2"
      shift 2
      ;;
    --maps)
      [[ $# -ge 2 ]] || { echo "--maps requires a value" >&2; exit 2; }
      selected_csv="$2"
      shift 2
      ;;
    --resume)
      resume=1
      shift
      ;;
    --fail-fast)
      fail_fast=1
      shift
      ;;
    --startup-timeout-sec)
      [[ $# -ge 2 ]] || { echo "--startup-timeout-sec requires a value" >&2; exit 2; }
      startup_timeout_sec="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      exit 2
      ;;
  esac
done

if [[ ! "${port}" =~ ^[0-9]+$ ]] || (( port < 1 || port > 65535 )); then
  echo "port must be an integer in [1, 65535]" >&2
  exit 2
fi
if [[ ! "${startup_timeout_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--startup-timeout-sec must be a positive integer" >&2
  exit 2
fi

output_root="$(
  python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().resolve())' \
    "${output_root}"
)"
mkdir -p "${output_root}"
if ! command -v flock >/dev/null 2>&1; then
  echo "flock is required to protect matrix output" >&2
  exit 2
fi
exec {matrix_lock_fd}>"${output_root}/.autoware_vad_town_matrix.lock"
if ! flock -n "${matrix_lock_fd}"; then
  echo "another VAD town matrix owns ${output_root}" >&2
  exit 2
fi

cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh

prepare_args=(
  prepare
  --manifest "${matrix_manifest}"
  --output-root "${output_root}"
)
if (( resume )); then
  prepare_args+=(--resume)
fi
python3 "${matrix_tool}" "${prepare_args[@]}"

mapfile -t runnable_rows < <(
  python3 "${matrix_tool}" list-runnable --output-root "${output_root}"
)
if [[ ${#runnable_rows[@]} -eq 0 ]]; then
  echo "No map has a validated Autoware full-map bundle; see ${output_root}/SUMMARY.md" >&2
  exit 1
fi

declare -A selected=()
if [[ -n "${selected_csv}" ]]; then
  IFS=',' read -r -a selected_ids <<<"${selected_csv}"
  if [[ ${#selected_ids[@]} -eq 0 ]]; then
    echo "--maps must select at least one map" >&2
    exit 2
  fi
  for map_id in "${selected_ids[@]}"; do
    if [[ ! "${map_id}" =~ ^[a-z0-9_]+$ || -n "${selected[${map_id}]+set}" ]]; then
      echo "unsafe or duplicate --maps id: ${map_id:-<empty>}" >&2
      exit 2
    fi
    selected["${map_id}"]=1
  done
  for map_id in "${selected_ids[@]}"; do
    found=0
    for row in "${runnable_rows[@]}"; do
      IFS=$'\t' read -r candidate _ <<<"${row}"
      [[ "${candidate}" == "${map_id}" ]] && found=1
    done
    if (( ! found )); then
      echo "--maps id is not admitted by a validated full-map bundle: ${map_id}" >&2
      exit 2
    fi
  done
fi

read -r route_seed pairs_per_seed min_distance max_distance preferred_distance \
  sampling_resolution max_endpoint_offset max_traces ready_timeout < <(
  python3 - "${output_root}/matrix_plan.json" <<'PY'
import json
import sys

plan = json.load(open(sys.argv[1], encoding="utf-8"))
route = plan["route_contract"]
profile = plan["runtime_profile"]
print(
    route["seed"], route["pairs_per_seed"], route["minimum_distance_m"],
    route["maximum_distance_m"], route["preferred_distance_m"],
    route["sampling_resolution_m"], route["maximum_endpoint_offset_m"],
    route["maximum_traces_per_scenario"], profile["ready_timeout_sec"],
)
PY
)

server_pid=""
server_pgid=""
current_map_id=""
interrupted=0

cleanup_server() {
  local cleanup_status=0
  if [[ -n "${server_pid}" && -n "${server_pgid}" ]]; then
    e2e_stop_owned_process_group "${server_pgid}" "${server_pid}" 30 10 5 || cleanup_status=$?
  fi
  server_pid=""
  server_pgid=""
  return "${cleanup_status}"
}

on_signal() {
  interrupted=1
  exit 130
}

on_exit() {
  local status="$1"
  trap - EXIT INT TERM HUP
  if (( interrupted )) && [[ -n "${current_map_id}" ]]; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${current_map_id}" \
      --status FAILED --stage interrupted \
      --reason "Matrix interrupted; only owned process groups were stopped." >/dev/null 2>&1 || true
  fi
  cleanup_server || true
  python3 "${matrix_tool}" summarize --output-root "${output_root}" >/dev/null 2>&1 || true
  exit "${status}"
}
trap on_signal INT TERM HUP
trap 'on_exit $?' EXIT

trial_attempt_dir() {
  local map_id="$1"
  local trial_id="$2"
  python3 - "${output_root}" "${map_id}" "${trial_id}" <<'PY'
from pathlib import Path
import re
import sys

base = Path(sys.argv[1]) / "maps" / sys.argv[2] / "trials" / sys.argv[3]
base.mkdir(parents=True, exist_ok=True)
numbers = []
for child in base.iterdir():
    match = re.fullmatch(r"attempt_(\d{3})", child.name)
    if match:
        numbers.append(int(match.group(1)))
print(base / f"attempt_{max(numbers, default=0) + 1:03d}")
PY
}

existing_attempt() {
  local map_id="$1"
  local trial_id="$2"
  python3 - "${output_root}/maps/${map_id}/status.json" "${trial_id}" <<'PY'
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8"))
trial = value["trials"][sys.argv[2]]
print(trial.get("attempt_directory") or "")
PY
}

route_for_trial() {
  local map_id="$1"
  local trial_id="$2"
  python3 - "${output_root}/maps/${map_id}/route_matrix.json" "${trial_id}" <<'PY'
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8"))
for trial in value["trials"]:
    if trial["trial_id"] == sys.argv[2]:
        print(trial["route_path"])
        break
else:
    raise SystemExit(f"missing route for {sys.argv[2]}")
PY
}

failure_count=0
for row in "${runnable_rows[@]}"; do
  IFS=$'\t' read -r map_id canonical_name server_profile full_map_path bundle_schema <<<"${row}"
  if [[ -n "${selected_csv}" && -z "${selected[${map_id}]+set}" ]]; then
    continue
  fi
  current_map_id="${map_id}"

  if (( resume )) && [[ -f "${output_root}/maps/${map_id}/route_matrix.json" ]]; then
    map_complete=1
    for trial_id in straight turn; do
      attempt_dir="$(existing_attempt "${map_id}" "${trial_id}")"
      if [[ -z "${attempt_dir}" ]] || ! python3 "${matrix_tool}" validate-trial \
        --output-root "${output_root}" --map-id "${map_id}" \
        --trial-id "${trial_id}" --trial-dir "${attempt_dir}" >/dev/null 2>&1; then
        map_complete=0
      fi
    done
    if (( map_complete )); then
      echo "RESUME_PASS map=${map_id} straight+turn evidence freshly validated"
      continue
    fi
  fi

  map_root="${output_root}/maps/${map_id}"
  server_log="${map_root}/carla_server.log"
  python3 "${matrix_tool}" update \
    --output-root "${output_root}" --map-id "${map_id}" \
    --status RUNNING --stage carla_cold_start \
    --reason "Cold-starting the owned CARLA server; client.load_world remains disabled."

  setsid "${server_tool}" "${canonical_name}" \
    --port "${port}" --quality Epic --startup-timeout-sec "${startup_timeout_sec}" \
    -- -RenderOffScreen -nosound >"${server_log}" 2>&1 &
  server_pid=$!
  server_pgid="${server_pid}"
  deadline=$((SECONDS + startup_timeout_sec + 15))
  server_ready=0
  while (( SECONDS < deadline )); do
    if grep -q '^CARLA_READY ' "${server_log}" 2>/dev/null; then
      server_ready=1
      break
    fi
    if ! kill -0 "${server_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if (( ! server_ready )); then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage carla_cold_start_failed \
      --reason "Owned CARLA cold start failed; see ${server_log}."
    cleanup_server || true
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi

  catalog_root="${map_root}/catalog"
  catalog_log="${map_root}/route_catalog.log"
  catalog_preflight_args=()
  if [[ "${bundle_schema}" == "custom_map" ]]; then
    # Custom-map domain transfer needs explicit entrance and isolated-turn geometry.
    # CARLA can label a sharp non-junction bend LANEFOLLOW even though the route
    # contains the requested maneuver later or span multiple junctions. Keep the
    # immutable seed/profile contract, but deterministically skip such pairs before
    # any closed-loop run.
    catalog_preflight_args+=(
      --initial-approach-distance-m 15.0
      --maximum-initial-lateral-deviation-m 1.5
      --maximum-initial-heading-change-deg 30.0
      --minimum-turn-arc-length-m 10.0
      --maximum-turn-arc-length-m 30.0
      --minimum-turn-heading-change-deg 60.0
      --maximum-turn-heading-change-deg 120.0
      --maximum-turn-heading-excess-deg 20.0
      --turn-alignment-heading-margin-deg 10.0
      --maximum-turn-command-lead-m 8.0
      --maximum-turn-command-tail-m 8.0
      --maximum-turn-p95-curvature-per-m 0.20
    )
  fi
  if ! python3 "${catalog_tool}" \
    --manifest "${root}/scripts/e2e/carla_expert_suite.yaml" \
    --map-id "${map_id}" --output-root "${catalog_root}" \
    --host 127.0.0.1 --port "${port}" --timeout 30 \
    --map-load-settle-sec 0 --active-server-profile "${server_profile}" \
    --weather ClearNoon --seeds "${route_seed}" \
    --pairs-per-seed "${pairs_per_seed}" --min-distance "${min_distance}" \
    --max-distance "${max_distance}" --preferred-distance "${preferred_distance}" \
    --sampling-resolution "${sampling_resolution}" \
    --max-endpoint-offset "${max_endpoint_offset}" --max-traces "${max_traces}" \
    "${catalog_preflight_args[@]}" \
    >"${catalog_log}" 2>&1; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage route_catalog_failed \
      --reason "Straight/turn route catalog generation failed; see ${catalog_log}."
    cleanup_server || true
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi
  if ! python3 "${matrix_tool}" select-routes \
    --output-root "${output_root}" --map-id "${map_id}" \
    --catalog "${catalog_root}/route_catalog.json"; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage route_contract_failed \
      --reason "Catalog did not contain separately verified straight and turn commands."
    cleanup_server || true
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi

  for trial_id in straight turn; do
    if (( resume )); then
      attempt_dir="$(existing_attempt "${map_id}" "${trial_id}")"
      if [[ -n "${attempt_dir}" ]] && python3 "${matrix_tool}" validate-trial \
        --output-root "${output_root}" --map-id "${map_id}" \
        --trial-id "${trial_id}" --trial-dir "${attempt_dir}" >/dev/null 2>&1; then
        python3 "${matrix_tool}" update \
          --output-root "${output_root}" --map-id "${map_id}" \
          --trial-id "${trial_id}" --trial-status PASS --stage resume_validated \
          --reason "Existing trial passed fresh strict validation." \
          --attempt-dir "${attempt_dir}" \
          --validation "${attempt_dir}/matrix_validation.json"
        echo "RESUME_PASS map=${map_id} trial=${trial_id}"
        continue
      fi
    fi

    route_path="$(route_for_trial "${map_id}" "${trial_id}")"
    attempt_dir="$(trial_attempt_dir "${map_id}" "${trial_id}")"
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --trial-id "${trial_id}" --trial-status RUNNING \
      --stage "${trial_id}_full_stack" \
      --reason "Running the fixed recommended visualized full-stack profile." \
      --attempt-dir "${attempt_dir}"

    set +e
    CARLA_HOST=127.0.0.1 CARLA_PORT="${port}" \
      AUTOWARE_E2E_FULL_MAP_PATH="${full_map_path}" \
      "${trial_tool}" --recommended --visualize --capture-desktop \
      --ready-timeout "${ready_timeout}" \
      "${attempt_dir}" "${route_path}" carla_timeout:=60
    trial_exit=$?
    set -e
    if (( trial_exit == 0 )) && python3 "${matrix_tool}" validate-trial \
      --output-root "${output_root}" --map-id "${map_id}" \
      --trial-id "${trial_id}" --trial-dir "${attempt_dir}"; then
      python3 "${matrix_tool}" update \
        --output-root "${output_root}" --map-id "${map_id}" \
        --trial-id "${trial_id}" --trial-status PASS --stage "${trial_id}_passed" \
        --reason "Full-stack route completion, analysis, and post-candidate desktop evidence passed." \
        --attempt-dir "${attempt_dir}" \
        --validation "${attempt_dir}/matrix_validation.json"
    else
      python3 "${matrix_tool}" update \
        --output-root "${output_root}" --map-id "${map_id}" \
        --trial-id "${trial_id}" --trial-status FAILED --stage "${trial_id}_failed" \
        --reason "Trial exit=${trial_exit} or strict artifact validation failed; see ${attempt_dir}." \
        --attempt-dir "${attempt_dir}"
      ((failure_count += 1))
      if (( fail_fast )); then
        break
      fi
    fi
  done
  cleanup_server || {
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage carla_cleanup_failed \
      --reason "Owned CARLA process group did not stop cleanly."
    ((failure_count += 1))
  }
  current_map_id=""
  if (( fail_fast && failure_count > 0 )); then
    break
  fi
done

python3 "${matrix_tool}" summarize --output-root "${output_root}"
trap - EXIT INT TERM HUP
cleanup_server || true
if (( failure_count > 0 )); then
  echo "VAD town matrix completed with ${failure_count} failed trial/map stages." >&2
  exit 1
fi
echo "VAD town matrix completed: ${output_root}"
