#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
matrix_manifest="${root}/scripts/e2e/autoware_vad_town_matrix.yaml"
matrix_tool="${root}/scripts/e2e/autoware_vad_town_matrix.py"
catalog_tool="${root}/scripts/e2e/prepare_carla_expert_route_catalog.py"
server_tool="${root}/scripts/e2e/run_carla_map.sh"
probe_tool="${root}/scripts/e2e/probe_carla_server.py"
trial_tool="${root}/scripts/e2e/run_recorded_route_trial.sh"

usage() {
  cat <<'EOF'
Usage: scripts/e2e/run_autoware_vad_town_matrix.sh OUTPUT_ROOT [OPTIONS]

Run one explicitly selected full-stack Autoware VAD visualized profile on every
map admitted by autoware_vad_town_matrix.yaml. Catalog, straight, and turn each
receive a separate owned CARLA cold start without client.load_world. Every
trial uses fresh read-only RPC preflight/completion checks and receives one true
STRAIGHT-command or LEFT/RIGHT-command route. Route analysis, ROS bags,
Autoware/RViz full-screen PNG/GIF evidence, per-trial validation, and an all-map
aggregate report are retained.

Maps without a validated Lanelet2 + PCD full-map bundle are reported BLOCKED;
they are never mislabeled as executed VAD trials.

Options:
  --port PORT                 CARLA RPC port (default: 2100)
  --maps ID[,ID...]           run this subset of admitted maps
  --runtime-profile PROFILE   recommended or speed_30kph (default: recommended)
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
runtime_profile="recommended"

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
    --runtime-profile)
      [[ $# -ge 2 ]] || { echo "--runtime-profile requires a value" >&2; exit 2; }
      runtime_profile="$2"
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

case "${runtime_profile}" in
  recommended|speed_30kph) ;;
  *)
    echo "--runtime-profile must be recommended or speed_30kph" >&2
    exit 2
    ;;
esac

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
source scripts/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "Autoware VAD town matrix"
scripts/e2e/apply_vad_object_safety_patches.sh
python3 scripts/e2e/vad_object_safety_build_provenance.py verify
scripts/e2e/apply_mission_planner_lane_only_patch.sh
python3 scripts/e2e/mission_planner_build_provenance.py verify

prepare_args=(
  prepare
  --manifest "${matrix_manifest}"
  --output-root "${output_root}"
  --runtime-profile "${runtime_profile}"
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

read -r ready_timeout < <(
  python3 - "${output_root}/matrix_plan.json" <<'PY'
import json
import sys

plan = json.load(open(sys.argv[1], encoding="utf-8"))
print(plan["runtime_profile"]["ready_timeout_sec"])
PY
)
route_catalog_specs_for_map() {
  local map_id="$1"
  python3 - "${output_root}/matrix_plan.json" "${map_id}" <<'PY'
import json
import sys

plan = json.load(open(sys.argv[1], encoding="utf-8"))
matches = [item for item in plan["maps"] if item.get("map_id") == sys.argv[2]]
if len(matches) != 1:
    raise SystemExit(f"map-specific route contract is missing/ambiguous: {sys.argv[2]}")
contracts = matches[0].get("route_generation_contracts")
if not isinstance(contracts, dict) or set(contracts) != {"straight", "turn"}:
    raise SystemExit(f"map-specific route contract is invalid: {sys.argv[2]}")
straight = contracts["straight"]
turn = contracts["turn"]
items = [("shared", straight)] if straight == turn else [
    ("straight", straight), ("turn", turn)
]
for catalog_id, contract in items:
    fields = [
        catalog_id,
        contract["weather"],
        ",".join(str(seed) for seed in contract["seeds"]),
        contract["pairs_per_seed"],
        contract["minimum_distance_m"],
        contract["maximum_distance_m"],
        contract["preferred_distance_m"],
        contract["sampling_resolution_m"],
        contract["maximum_endpoint_offset_m"],
        contract["maximum_traces_per_scenario"],
        contract.get("endpoint_waypoint_spacing_m", ""),
        contract.get("endpoint_junction_policy", "include"),
        contract.get("candidate_enumeration_policy", "all_pairs"),
        contract.get("straight_capacity_profile", ""),
    ]
    print("|".join(str(field) for field in fields))
PY
}
mapfile -t trial_wrapper_options < <(
  python3 - "${output_root}/matrix_plan.json" <<'PY'
import json
import sys

plan = json.load(open(sys.argv[1], encoding="utf-8"))
for option in plan["runtime_profile"]["wrapper_options"]:
    if not isinstance(option, str) or not option:
        raise SystemExit("runtime-profile wrapper options must be non-empty strings")
    print(option)
PY
)
if [[ ${#trial_wrapper_options[@]} -eq 0 ]]; then
  echo "selected runtime profile has no wrapper options" >&2
  exit 2
fi

server_pid=""
server_pgid=""
server_generation_id=""
server_expected_map=""
server_log=""
current_map_id=""
interrupted=0

current_map_is_terminal() {
  local map_id="$1"
  python3 - "${output_root}/maps/${map_id}/status.json" <<'PY'
import json
import sys

try:
    status = json.load(open(sys.argv[1], encoding="utf-8"))
    trials = status["trials"]
    terminal = (
        status.get("status") in {"PASS", "FAILED"}
        and all(
            trials[trial_id].get("status") in {"PASS", "FAILED"}
            for trial_id in ("straight", "turn")
        )
    )
except (KeyError, OSError, TypeError, ValueError):
    terminal = False
raise SystemExit(0 if terminal else 1)
PY
}

map_has_strict_pass_state() {
  local map_id="$1"
  python3 - "${output_root}/maps/${map_id}/status.json" <<'PY'
import json
import sys

try:
    status = json.load(open(sys.argv[1], encoding="utf-8"))
    trials = status["trials"]
    passed = (
        status.get("status") == "PASS"
        and all(
            trials[trial_id].get("status") == "PASS"
            for trial_id in ("straight", "turn")
        )
    )
except (KeyError, OSError, TypeError, ValueError):
    passed = False
raise SystemExit(0 if passed else 1)
PY
}

probe_server_health() {
  local output_path="$1"
  local stage="$2"
  local expected_mode="${3:-running}"
  local mode_args=()
  [[ -n "${server_pid}" && -n "${server_pgid}" ]] || {
    echo "CARLA health probe has no owned generation" >&2
    return 2
  }
  if [[ "${expected_mode}" == "stopped" ]]; then
    mode_args+=(--expect-stopped)
  elif [[ "${expected_mode}" != "running" ]]; then
    echo "invalid CARLA health mode: ${expected_mode}" >&2
    return 2
  fi
  python3 "${probe_tool}" \
    --host 127.0.0.1 --port "${port}" --timeout 3 \
    --expected-map "${server_expected_map}" --stage "${stage}" \
    --generation-id "${server_generation_id}" \
    --owner-pid "${server_pid}" --owner-pgid "${server_pgid}" \
    --server-log "${server_log}" --output "${output_path}" \
    "${mode_args[@]}"
}

start_server() {
  local generation_id="$1"
  local expected_map="$2"
  local log_path="$3"
  local deadline
  local server_ready=0

  if [[ -n "${server_pid}" || -n "${server_pgid}" ]]; then
    echo "refusing to overlap owned CARLA generations" >&2
    return 2
  fi
  server_generation_id="${generation_id}"
  server_expected_map="${expected_map}"
  server_log="${log_path}"
  mkdir -p "$(dirname "${server_log}")"
  setsid "${server_tool}" "${server_expected_map}" \
    --port "${port}" --quality Epic --startup-timeout-sec "${startup_timeout_sec}" \
    -- -RenderOffScreen -nosound >"${server_log}" 2>&1 &
  server_pid=$!
  server_pgid="${server_pid}"
  deadline=$((SECONDS + startup_timeout_sec + 15))
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
  (( server_ready ))
}

cleanup_server() {
  local health_path="${1:-}"
  local health_stage="${2:-cleanup}"
  local cleanup_status=0
  if [[ -n "${server_pid}" && -n "${server_pgid}" ]]; then
    e2e_stop_owned_process_group "${server_pgid}" "${server_pid}" 30 10 5 || cleanup_status=$?
    if [[ -n "${health_path}" ]]; then
      probe_server_health "${health_path}" "${health_stage}" stopped || cleanup_status=$?
    fi
  fi
  server_pid=""
  server_pgid=""
  server_generation_id=""
  server_expected_map=""
  server_log=""
  return "${cleanup_status}"
}

on_signal() {
  interrupted=1
  exit 130
}

on_exit() {
  local status="$1"
  local terminal_stage="unexpected_exit"
  local terminal_reason="Matrix runner exited unexpectedly with status ${status}; the active map was terminalized before owned process cleanup."
  trap - EXIT INT TERM HUP
  if (( interrupted )); then
    terminal_stage="interrupted"
    terminal_reason="Matrix interrupted; only owned process groups were stopped."
  fi
  if (( status != 0 )) && [[ -n "${current_map_id}" ]] && \
      ! current_map_is_terminal "${current_map_id}"; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${current_map_id}" \
      --status FAILED --stage "${terminal_stage}" \
      --reason "${terminal_reason}" >/dev/null 2>&1 || true
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

verify_selected_maps_complete() {
  python3 - "${output_root}" "$@" <<'PY'
import json
from pathlib import Path
import sys

output_root = Path(sys.argv[1])
selected_ids = sys.argv[2:]
if not selected_ids:
    raise SystemExit("no selected map ids reached final aggregate validation")

try:
    aggregate = json.load(open(output_root / "aggregate.json", encoding="utf-8"))
    aggregate_rows = aggregate["maps"]
except (KeyError, OSError, TypeError, ValueError) as error:
    raise SystemExit(f"cannot validate final aggregate: {error}") from error

by_id = {}
for row in aggregate_rows:
    map_id = row.get("map_id") if isinstance(row, dict) else None
    if not isinstance(map_id, str) or map_id in by_id:
        raise SystemExit("final aggregate has an invalid or duplicate map row")
    by_id[map_id] = row

errors = []
for map_id in selected_ids:
    aggregate_row = by_id.get(map_id)
    if aggregate_row is None:
        errors.append(f"{map_id}: missing from aggregate")
        continue
    try:
        status_row = json.load(
            open(output_root / "maps" / map_id / "status.json", encoding="utf-8")
        )
    except (OSError, TypeError, ValueError) as error:
        errors.append(f"{map_id}: cannot read status.json ({error})")
        continue
    for source, row in (("aggregate", aggregate_row), ("status", status_row)):
        if row.get("runnable") is not True:
            errors.append(f"{map_id}: {source} row is not runnable")
        if row.get("status") != "PASS":
            errors.append(
                f"{map_id}: {source} map status is {row.get('status')!r}, not PASS"
            )
        trials = row.get("trials")
        if not isinstance(trials, dict):
            errors.append(f"{map_id}: {source} trials object is missing")
            continue
        for trial_id in ("straight", "turn"):
            trial = trials.get(trial_id)
            trial_status = trial.get("status") if isinstance(trial, dict) else None
            if trial_status != "PASS":
                errors.append(
                    f"{map_id}/{trial_id}: {source} status is "
                    f"{trial_status!r}, not PASS"
                )

if errors:
    raise SystemExit("selected-map final-state validation failed: " + "; ".join(errors))
print(f"FINAL_STATE_PASS selected_maps={len(selected_ids)}")
PY
}

failure_count=0
selected_map_ids=()
for row in "${runnable_rows[@]}"; do
  IFS=$'\t' read -r map_id _ <<<"${row}"
  if [[ -n "${selected_csv}" && -z "${selected[${map_id}]+set}" ]]; then
    continue
  fi
  selected_map_ids+=("${map_id}")
done
for row in "${runnable_rows[@]}"; do
  IFS=$'\t' read -r map_id canonical_name server_profile full_map_path bundle_schema <<<"${row}"
  if [[ -n "${selected_csv}" && -z "${selected[${map_id}]+set}" ]]; then
    continue
  fi
  current_map_id="${map_id}"

  if (( resume )) && [[ -f "${output_root}/maps/${map_id}/route_matrix.json" ]]; then
    map_complete=1
    declare -A resume_attempts=()
    for trial_id in straight turn; do
      attempt_dir="$(existing_attempt "${map_id}" "${trial_id}")"
      if [[ -z "${attempt_dir}" ]] || ! python3 "${matrix_tool}" validate-trial \
        --output-root "${output_root}" --map-id "${map_id}" \
        --trial-id "${trial_id}" --trial-dir "${attempt_dir}" >/dev/null 2>&1; then
        map_complete=0
      else
        resume_attempts["${trial_id}"]="${attempt_dir}"
      fi
    done
    if (( map_complete )); then
      for trial_id in straight turn; do
        attempt_dir="${resume_attempts[${trial_id}]}"
        python3 "${matrix_tool}" update \
          --output-root "${output_root}" --map-id "${map_id}" \
          --trial-id "${trial_id}" --trial-status PASS --stage resume_validated \
          --reason "Existing trial passed fresh strict validation." \
          --attempt-dir "${attempt_dir}" \
          --validation "${attempt_dir}/matrix_validation.json"
      done
      if map_has_strict_pass_state "${map_id}"; then
        echo "RESUME_PASS map=${map_id} straight+turn evidence freshly validated and status recovered"
        current_map_id=""
        continue
      fi
      echo "RESUME_REPLAY map=${map_id} strict evidence did not recover a PASS status" >&2
    fi
  fi

  mapfile -t route_catalog_specs < <(route_catalog_specs_for_map "${map_id}")
  if [[ ${#route_catalog_specs[@]} -lt 1 || ${#route_catalog_specs[@]} -gt 2 ]]; then
    echo "${map_id} has an invalid map-specific route catalog plan" >&2
    exit 2
  fi

  map_root="${output_root}/maps/${map_id}"
  catalog_health_root="${map_root}/carla_lifecycle/catalog_attempt_001"
  catalog_server_log="${map_root}/carla_server_catalog_attempt_001.log"
  mkdir -p "${catalog_health_root}"
  python3 "${matrix_tool}" update \
    --output-root "${output_root}" --map-id "${map_id}" \
    --status RUNNING --stage carla_cold_start \
    --reason "Cold-starting the catalog-only owned CARLA generation; client.load_world remains disabled."

  if ! start_server \
    "${map_id}_catalog_attempt_001" "${canonical_name}" \
    "${catalog_server_log}"; then
    probe_server_health \
      "${catalog_health_root}/preflight.json" catalog_preflight running || true
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage carla_cold_start_failed \
      --reason "Owned catalog CARLA cold start failed; see ${catalog_server_log}."
    cleanup_server \
      "${catalog_health_root}/cleanup.json" catalog_cleanup || true
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi
  if ! probe_server_health \
    "${catalog_health_root}/preflight.json" catalog_preflight running; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage carla_catalog_rpc_preflight_failed \
      --reason "Owned catalog CARLA generation failed fresh RPC/map/snapshot preflight."
    cleanup_server \
      "${catalog_health_root}/cleanup.json" catalog_cleanup || true
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi

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
  catalog_failed=0
  select_catalog_args=()
  for catalog_spec in "${route_catalog_specs[@]}"; do
    IFS='|' read -r catalog_id catalog_weather catalog_seeds_csv catalog_pairs catalog_min \
      catalog_max catalog_preferred catalog_sampling catalog_endpoint \
      catalog_max_traces catalog_waypoint_spacing catalog_junction_policy \
      catalog_enumeration_policy catalog_capacity_profile <<<"${catalog_spec}"
    catalog_endpoint_source_args=()
    if [[ "${catalog_id}" == "shared" ]]; then
      catalog_root="${map_root}/catalog"
      catalog_log="${map_root}/route_catalog.log"
      select_catalog_args=(--catalog "${catalog_root}/route_catalog.json")
      catalog_scenario_args=()
    else
      catalog_root="${map_root}/catalog/${catalog_id}"
      catalog_log="${map_root}/route_catalog_${catalog_id}.log"
      select_catalog_args+=(
        "--${catalog_id}-catalog" "${catalog_root}/route_catalog.json"
      )
      case "${catalog_id}" in
        straight)
          catalog_scenario_args=(--scenarios straight)
          if [[ "${runtime_profile}" == "speed_30kph" && \
                "${bundle_schema}" == "packaged_town" ]]; then
            catalog_waypoint_spacing="${catalog_waypoint_spacing:-10.0}"
            catalog_endpoint_source_args=(
              --endpoint-waypoint-spacing-m "${catalog_waypoint_spacing}"
              --endpoint-junction-policy "${catalog_junction_policy}"
              --candidate-enumeration-policy "${catalog_enumeration_policy}"
              --physical-straight-profile speed_30kph
            )
            if [[ -n "${catalog_capacity_profile}" ]]; then
              catalog_endpoint_source_args+=(
                --straight-capacity-profile "${catalog_capacity_profile}"
              )
            fi
          fi
          ;;
        turn)
          catalog_scenario_args=(--scenarios left,right)
          if [[ "${runtime_profile}" == "speed_30kph" && \
                "${bundle_schema}" == "packaged_town" ]]; then
            catalog_endpoint_source_args=(
              --physical-turn-profile speed_30kph
            )
          fi
          ;;
        *)
          echo "invalid split catalog id: ${catalog_id}" >&2
          catalog_failed=1
          break
          ;;
      esac
    fi
    if [[ -n "${catalog_capacity_profile}" && \
          ( "${catalog_id}" != "straight" || \
            "${runtime_profile}" != "speed_30kph" || \
            "${bundle_schema}" != "packaged_town" ) ]]; then
      echo "invalid map-specific straight capacity contract for ${map_id}/${catalog_id}" >&2
      catalog_failed=1
      break
    fi
    if ! python3 "${catalog_tool}" \
      --manifest "${root}/scripts/e2e/carla_expert_suite.yaml" \
      --map-id "${map_id}" --output-root "${catalog_root}" \
      --host 127.0.0.1 --port "${port}" --timeout 30 \
      --map-load-settle-sec 0 --active-server-profile "${server_profile}" \
      --weather "${catalog_weather}" --seeds "${catalog_seeds_csv}" \
      --pairs-per-seed "${catalog_pairs}" --min-distance "${catalog_min}" \
      --max-distance "${catalog_max}" --preferred-distance "${catalog_preferred}" \
      --sampling-resolution "${catalog_sampling}" \
      --max-endpoint-offset "${catalog_endpoint}" \
      --max-traces "${catalog_max_traces}" \
      "${catalog_scenario_args[@]}" \
      "${catalog_endpoint_source_args[@]}" \
      "${catalog_preflight_args[@]}" \
      >"${catalog_log}" 2>&1; then
      python3 "${matrix_tool}" update \
        --output-root "${output_root}" --map-id "${map_id}" \
        --status FAILED --stage route_catalog_failed \
        --reason "${catalog_id} route catalog generation failed; see ${catalog_log}."
      catalog_failed=1
      break
    fi
  done
  if (( catalog_failed )); then
    cleanup_server \
      "${catalog_health_root}/cleanup.json" catalog_cleanup || true
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi
  catalog_lifecycle_failed=0
  if ! probe_server_health \
    "${catalog_health_root}/completion.json" catalog_completion running; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage carla_catalog_rpc_completion_failed \
      --reason "Owned catalog CARLA generation failed its completion RPC check."
    catalog_lifecycle_failed=1
  fi
  if ! cleanup_server \
    "${catalog_health_root}/cleanup.json" catalog_cleanup; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage carla_catalog_cleanup_failed \
      --reason "Owned catalog CARLA process group or port did not stop cleanly."
    catalog_lifecycle_failed=1
  fi
  if (( catalog_lifecycle_failed )); then
    ((failure_count += 1))
    (( fail_fast )) && break
    continue
  fi
  if ! python3 "${matrix_tool}" select-routes \
    --output-root "${output_root}" --map-id "${map_id}" \
    "${select_catalog_args[@]}"; then
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --status FAILED --stage route_contract_failed \
      --reason "Catalog selection or fresh exact-route map/PCD preflight failed."
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
    attempt_name="$(basename "${attempt_dir}")"
    mkdir -p "${attempt_dir}"
    trial_generation_id="${map_id}_${trial_id}_${attempt_name}"
    trial_server_log="${attempt_dir}/carla_server.log"
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --trial-id "${trial_id}" --trial-status RUNNING \
      --stage "${trial_id}_carla_cold_start" \
      --reason "Cold-starting the trial-only owned CARLA generation." \
      --attempt-dir "${attempt_dir}"

    if ! start_server \
      "${trial_generation_id}" "${canonical_name}" "${trial_server_log}"; then
      probe_server_health \
        "${attempt_dir}/carla_preflight_health.json" \
        trial_preflight running || true
      cleanup_server \
        "${attempt_dir}/carla_cleanup_health.json" trial_cleanup || true
      python3 "${matrix_tool}" update \
        --output-root "${output_root}" --map-id "${map_id}" \
        --trial-id "${trial_id}" --trial-status FAILED \
        --stage "${trial_id}_carla_cold_start_failed" \
        --reason "Trial-only owned CARLA cold start failed; see ${trial_server_log}." \
        --attempt-dir "${attempt_dir}"
      ((failure_count += 1))
      if (( fail_fast )); then
        break
      fi
      continue
    fi
    python3 "${matrix_tool}" update \
      --output-root "${output_root}" --map-id "${map_id}" \
      --trial-id "${trial_id}" --trial-status RUNNING \
      --stage "${trial_id}_full_stack" \
      --reason "Running the fixed ${runtime_profile} visualized full-stack profile on its isolated CARLA generation." \
      --attempt-dir "${attempt_dir}"

    set +e
    CARLA_HOST=127.0.0.1 CARLA_PORT="${port}" \
      AUTOWARE_E2E_CARLA_GENERATION_ID="${trial_generation_id}" \
      AUTOWARE_E2E_CARLA_EXPECTED_MAP="${canonical_name}" \
      AUTOWARE_E2E_CARLA_OWNER_PID="${server_pid}" \
      AUTOWARE_E2E_CARLA_OWNER_PGID="${server_pgid}" \
      AUTOWARE_E2E_CARLA_SERVER_LOG="${trial_server_log}" \
      AUTOWARE_E2E_FULL_MAP_PATH="${full_map_path}" \
      "${trial_tool}" "${trial_wrapper_options[@]}" \
      --ready-timeout "${ready_timeout}" \
      "${attempt_dir}" "${route_path}" carla_timeout:=60
    trial_exit=$?
    set -e
    cleanup_status=0
    cleanup_server \
      "${attempt_dir}/carla_cleanup_health.json" trial_cleanup || cleanup_status=$?
    if (( trial_exit == 0 && cleanup_status == 0 )) && python3 "${matrix_tool}" validate-trial \
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
        --reason "Trial exit=${trial_exit}, CARLA cleanup=${cleanup_status}, or strict artifact validation failed; see ${attempt_dir}." \
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
final_state_ok=1
if ! verify_selected_maps_complete "${selected_map_ids[@]}"; then
  final_state_ok=0
fi
trap - EXIT INT TERM HUP
cleanup_server || true
if (( failure_count > 0 || ! final_state_ok )); then
  echo "VAD town matrix failed: failure_count=${failure_count}, final_state_ok=${final_state_ok}." >&2
  exit 1
fi
echo "VAD town matrix completed: ${output_root}"
