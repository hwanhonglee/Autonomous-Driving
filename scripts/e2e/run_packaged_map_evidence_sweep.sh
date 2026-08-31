#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
manifest="${root}/scripts/e2e/carla_expert_suite.yaml"
reporter="${root}/scripts/e2e/carla_basicagent_sweep_report.py"
inventory_tool="${root}/scripts/e2e/inventory_carla_training_maps.py"
catalog_tool="${root}/scripts/e2e/prepare_carla_expert_route_catalog.py"
suite_tool="${root}/scripts/e2e/run_carla_expert_collection_suite.py"
server_tool="${root}/scripts/e2e/run_carla_map.sh"

readonly -a safe_map_ids=(
  town01
  town02_opt
  town03
  town04
  town05_opt
  town06
  town07
  town10hd_opt
  c_track_1_0_7
)
declare -Ar canonical_names=(
  [town01]=Town01
  [town02_opt]=Town02_Opt
  [town03]=Town03
  [town04]=Town04
  [town05_opt]=Town05_Opt
  [town06]=Town06
  [town07]=Town07
  [town10hd_opt]=Town10HD_Opt
  [c_track_1_0_7]=C_track_1_0_7
)

usage() {
  cat <<'EOF'
Usage: scripts/e2e/run_packaged_map_evidence_sweep.sh OUTPUT_ROOT [OPTIONS]

Cold-start each selected packaged CARLA map at Epic quality, build one
deterministic route per scenario (route seed 0 by default), and execute one
ClearNoon/collection-seed-0 lane_follow six-camera smoke with CARLA BasicAgent.
The run exports a PNG, GIF, per-map logs/status, and an aggregate report covering
all 19 manifest maps.

This is CARLA BasicAgent evidence, not Autoware VAD inference or closed-loop
control evidence. Client-side map loading is disabled throughout the sweep.

Options:
  --port PORT                 CARLA RPC port (default: 2100)
  --maps ID[,ID...]           maps to run now; the report retains the nine-map campaign
  --route-seed ID=SEED        override one selected map's route seed; repeatable
  --resume                    skip maps whose manifests, export, PNG, and GIF validate
  --fail-fast                 stop after the first selected-map failure
  --startup-timeout-sec SEC   cold-start timeout (default: 180)
  --max-duration-sec SEC      BasicAgent episode limit (default: 180)
  -h, --help                  show this help

Safe packaged map ids:
  town01,town02_opt,town03,town04,town05_opt,town06,town07,town10hd_opt,c_track_1_0_7
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
selected_csv="$(IFS=,; echo "${safe_map_ids[*]}")"
resume=0
fail_fast=0
startup_timeout_sec=180
max_duration_sec=180
route_seed_specs=()

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
    --route-seed)
      [[ $# -ge 2 ]] || { echo "--route-seed requires ID=SEED" >&2; exit 2; }
      route_seed_specs+=("$2")
      shift 2
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
    --max-duration-sec)
      [[ $# -ge 2 ]] || { echo "--max-duration-sec requires a value" >&2; exit 2; }
      max_duration_sec="$2"
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
for numeric in startup_timeout_sec max_duration_sec; do
  value="${!numeric}"
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "--${numeric//_/-} must be a positive integer" >&2
    exit 2
  fi
done

IFS=',' read -r -a selected_map_ids <<<"${selected_csv}"
if [[ ${#selected_map_ids[@]} -eq 0 ]]; then
  echo "--maps must select at least one map" >&2
  exit 2
fi
declare -A selected_seen=()
for map_id in "${selected_map_ids[@]}"; do
  if [[ -z "${map_id}" || -z "${canonical_names[${map_id}]+present}" ]]; then
    echo "unsupported safe packaged map id: ${map_id:-<empty>}" >&2
    exit 2
  fi
  if [[ -n "${selected_seen[${map_id}]+present}" ]]; then
    echo "duplicate map id: ${map_id}" >&2
    exit 2
  fi
  selected_seen["${map_id}"]=1
done
selected_csv="$(IFS=,; echo "${selected_map_ids[*]}")"
campaign_csv="$(IFS=,; echo "${safe_map_ids[*]}")"
declare -A route_seed_by_map=()
declare -A route_seed_overridden=()
for map_id in "${safe_map_ids[@]}"; do
  route_seed_by_map["${map_id}"]=0
done
for specification in "${route_seed_specs[@]}"; do
  if [[ ! "${specification}" =~ ^([a-z0-9_]+)=(0|[1-9][0-9]*)$ ]]; then
    echo "--route-seed must use a safe ID=nonnegative-integer value: ${specification}" >&2
    exit 2
  fi
  map_id="${BASH_REMATCH[1]}"
  seed="${BASH_REMATCH[2]}"
  if [[ -z "${canonical_names[${map_id}]+present}" || -z "${selected_seen[${map_id}]+present}" ]]; then
    echo "--route-seed map must be selected by --maps: ${map_id}" >&2
    exit 2
  fi
  if [[ -n "${route_seed_overridden[${map_id}]+present}" ]]; then
    echo "duplicate --route-seed override for map: ${map_id}" >&2
    exit 2
  fi
  route_seed_by_map["${map_id}"]="${seed}"
  route_seed_overridden["${map_id}"]=1
done

output_root="$(python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().resolve())' "${output_root}")"
if [[ -e "${output_root}/aggregate.json" || -e "${output_root}/SUMMARY.md" || -d "${output_root}/maps" ]]; then
  if (( ! resume )); then
    echo "sweep output already exists; pass --resume or use a new OUTPUT_ROOT: ${output_root}" >&2
    exit 2
  fi
fi
mkdir -p "${output_root}"
if ! command -v flock >/dev/null 2>&1; then
  echo "flock is required to protect the sweep output from concurrent writers" >&2
  exit 2
fi
exec {sweep_lock_fd}>"${output_root}/.packaged_map_evidence_sweep.lock"
if ! flock -n "${sweep_lock_fd}"; then
  echo "another packaged-map sweep owns this output root: ${output_root}" >&2
  exit 2
fi

cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh

server_pid=""
server_pgid=""
current_map_id=""
interrupted=0

summarize() {
  python3 "${reporter}" summarize \
    --manifest "${manifest}" \
    --output-root "${output_root}" >/dev/null 2>&1 || true
}

cleanup_owned_server() {
  local cleanup_status=0
  if [[ -n "${server_pgid}" && -n "${server_pid}" ]]; then
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
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${current_map_id}" \
      --status FAILED \
      --stage interrupted \
      --reason "The sweep was interrupted; only its owned CARLA process group was stopped." \
      --exit-code 130 >/dev/null 2>&1 || true
  fi
  cleanup_owned_server || true
  summarize
  exit "${status}"
}

trap on_signal INT TERM HUP
trap 'on_exit $?' EXIT

inventory_log="${output_root}/inventory.log"
if ! python3 "${inventory_tool}" \
  --manifest "${manifest}" \
  --output "${output_root}/inventory.json" >"${inventory_log}" 2>&1; then
  echo "static CARLA inventory failed; see ${inventory_log}" >&2
  exit 1
fi

python3 "${reporter}" initialize \
  --manifest "${manifest}" \
  --inventory "${output_root}/inventory.json" \
  --output-root "${output_root}" \
  --selected-maps "${campaign_csv}" \
  --invocation-maps "${selected_csv}" \
  --host 127.0.0.1 \
  --port "${port}" >/dev/null
summarize

failure_count=0
for map_id in "${selected_map_ids[@]}"; do
  current_map_id="${map_id}"
  route_seed="${route_seed_by_map[${map_id}]}"
  map_root="${output_root}/maps/${map_id}"
  mkdir -p "${map_root}"

  if (( resume )) && python3 "${reporter}" validate-artifacts \
    --output-root "${output_root}" \
    --map-id "${map_id}" \
    --route-seed "${route_seed}" >/dev/null 2>&1; then
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${map_id}" \
      --status SKIP_RESUME_VALIDATED \
      --stage resume_validated \
      --route-seed "${route_seed}" \
      --reason "Existing complete episode, validated export, PNG, and GIF were revalidated." >/dev/null
    summarize
    current_map_id=""
    continue
  fi

  python3 "${reporter}" update \
    --output-root "${output_root}" \
    --map-id "${map_id}" \
    --status RUNNING \
    --stage server_starting \
    --route-seed "${route_seed}" \
    --reason "Cold-starting ${canonical_names[${map_id}]} at Epic quality with deterministic route seed ${route_seed}." >/dev/null

  setsid --wait "${server_tool}" "${canonical_names[${map_id}]}" \
    --port "${port}" \
    --quality Epic \
    --startup-timeout-sec "${startup_timeout_sec}" \
    -- -RenderOffScreen -nosound >"${map_root}/server.log" 2>&1 &
  server_pid=$!
  sleep 0.2
  server_pgid="$(ps -o pgid= -p "${server_pid}" 2>/dev/null | tr -d '[:space:]')"

  map_failed=0
  failure_reason=""
  if [[ ! "${server_pgid}" =~ ^[1-9][0-9]*$ ]]; then
    map_failed=1
    failure_reason="CARLA server wrapper exited before an owned process group was recorded."
  else
    readiness_deadline=$((SECONDS + startup_timeout_sec + 15))
    while ! grep -q '^CARLA_READY ' "${map_root}/server.log" 2>/dev/null; do
      if ! e2e_process_group_exists "${server_pgid}"; then
        map_failed=1
        failure_reason="CARLA cold-start process exited before readiness."
        break
      fi
      if (( SECONDS >= readiness_deadline )); then
        map_failed=1
        failure_reason="Timed out waiting for CARLA_READY after cold-start."
        break
      fi
      sleep 1
    done
  fi

  if (( ! map_failed )); then
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${map_id}" \
      --status RUNNING \
      --stage live_inventory \
      --reason "Cold-started map is ready; collecting read-only live inventory." >/dev/null
    if ! python3 "${inventory_tool}" \
      --manifest "${manifest}" \
      --output "${map_root}/inventory.json" \
      --live \
      --host 127.0.0.1 \
      --port "${port}" \
      --timeout 30 >"${map_root}/inventory.log" 2>&1; then
      map_failed=1
      failure_reason="Read-only live CARLA inventory failed."
    fi
  fi

  if (( ! map_failed )); then
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${map_id}" \
      --status RUNNING \
      --stage route_catalog \
      --reason "Generating deterministic route-seed-${route_seed}, pair-1 catalog." >/dev/null
    if ! python3 "${catalog_tool}" \
      --manifest "${manifest}" \
      --map-id "${map_id}" \
      --output-root "${map_root}/catalog" \
      --host 127.0.0.1 \
      --port "${port}" \
      --active-server-profile packaged_0915 \
      --map-load-settle-sec 0 \
      --seeds "${route_seed}" \
      --pairs-per-seed 1 >"${map_root}/catalog.log" 2>&1; then
      map_failed=1
      failure_reason="Deterministic route catalog generation failed."
    fi
  fi

  if (( ! map_failed )); then
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${map_id}" \
      --status RUNNING \
      --stage basicagent_smoke \
      --reason "Executing one ClearNoon/seed-0 lane_follow CARLA BasicAgent smoke." >/dev/null
    if ! python3 "${suite_tool}" \
      --manifest "${manifest}" \
      --catalog "${map_root}/catalog/route_catalog.json" \
      --output-root "${map_root}/smoke" \
      --execute \
      --active-server-profile packaged_0915 \
      --host 127.0.0.1 \
      --port "${port}" \
      --weathers ClearNoon \
      --seeds 0 \
      --scenarios lane_follow \
      --goal-tolerance-m 3.0 \
      --max-duration-sec "${max_duration_sec}" \
      --map-load-settle-sec 0 >"${map_root}/suite.log" 2>&1; then
      map_failed=1
      failure_reason="CARLA BasicAgent collection/export/render suite failed."
    fi
  fi

  if (( ! map_failed )); then
    if ! python3 "${reporter}" validate-artifacts \
      --output-root "${output_root}" \
      --map-id "${map_id}" >>"${map_root}/suite.log" 2>&1; then
      map_failed=1
      failure_reason="Suite output did not pass manifest, PNG, and GIF validation."
    fi
  fi

  if ! cleanup_owned_server; then
    map_failed=1
    failure_reason="Owned CARLA process group did not terminate cleanly."
  fi

  if (( map_failed )); then
    failure_count=$((failure_count + 1))
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${map_id}" \
      --status FAILED \
      --stage failed \
      --reason "${failure_reason}" \
      --exit-code 1 >/dev/null
    echo "FAILED map=${map_id} reason=${failure_reason} logs=${map_root}" >&2
  else
    python3 "${reporter}" update \
      --output-root "${output_root}" \
      --map-id "${map_id}" \
      --status PASS \
      --stage complete \
      --reason "BasicAgent episode, export, PNG, and GIF all validated; this is not Autoware VAD evidence." >/dev/null
    echo "PASS map=${map_id} artifacts=${map_root}/smoke"
  fi
  summarize
  current_map_id=""

  if (( map_failed && fail_fast )); then
    break
  fi
done

summarize
echo "summary=${output_root}/SUMMARY.md aggregate=${output_root}/aggregate.json failures=${failure_count}"
if (( failure_count )); then
  exit 1
fi
