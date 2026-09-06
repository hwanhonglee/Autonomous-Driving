#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
  cat >&2 <<'EOF'
Usage: run_owned_carla_route_trial.sh [owner options] OUTPUT_ROOT ROUTE_JSON [trial options]

Run one exact, existing route against fresh owned CARLA generations.  A trial
that is rejected only by the pre-engagement runtime-health gate is retried on
a new CARLA generation; route/control failures are never silently retried.

Owner options:
  --port PORT                 CARLA RPC port (default: 2100)
  --startup-timeout-sec SEC   CARLA cold-start timeout (default: 180)
  --ready-timeout-sec SEC     Autoware route-readiness timeout (default: 600)
  --max-health-retries N      Fresh-generation retries after health FAIL (default: 2)
  --quality LEVEL             CARLA quality passed to run_carla_map (default: Epic)

Trial options are run_recorded_route_trial.sh flags, for example:
  --speed-30kph --camera-source-5hz --visualize --capture-desktop
  --speed-30kph --portable-shadow-10hz plus every pinned Portable input
  --portable-device cpu --portable-cpu-set 8,10,12,14
  --control-ab-pid-i40
  --control-ab-turn-preview-5m
  --control-ab-turn-preview-10m
  --control-ab-longitudinal-recovery-2p0

The helper always enables the pre-engagement runtime-health gate and passes
carla_timeout:=60.  OUTPUT_ROOT must not already exist.
EOF
}

port=2100
startup_timeout_sec=180
ready_timeout_sec=600
max_health_retries=2
quality=Epic
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      port="$2"
      shift 2
      ;;
    --startup-timeout-sec)
      [[ $# -ge 2 ]] || { echo "--startup-timeout-sec requires a value" >&2; exit 2; }
      startup_timeout_sec="$2"
      shift 2
      ;;
    --ready-timeout-sec)
      [[ $# -ge 2 ]] || { echo "--ready-timeout-sec requires a value" >&2; exit 2; }
      ready_timeout_sec="$2"
      shift 2
      ;;
    --max-health-retries)
      [[ $# -ge 2 ]] || { echo "--max-health-retries requires a value" >&2; exit 2; }
      max_health_retries="$2"
      shift 2
      ;;
    --quality)
      [[ $# -ge 2 ]] || { echo "--quality requires a value" >&2; exit 2; }
      quality="$2"
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
    -* )
      echo "Unknown owner option before OUTPUT_ROOT: $1" >&2
      usage
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi
output_root="$1"
route_file="$2"
shift 2
trial_options=("$@")

if [[ ! "${port}" =~ ^[0-9]+$ ]] || (( port < 1 || port > 65535 )); then
  echo "port must be an integer in [1, 65535]" >&2
  exit 2
fi
for value in "${startup_timeout_sec}" "${ready_timeout_sec}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "timeouts must be positive integers" >&2
    exit 2
  fi
done
if [[ ! "${max_health_retries}" =~ ^[0-9]+$ ]] || (( max_health_retries > 9 )); then
  echo "max health retries must be an integer in [0, 9]" >&2
  exit 2
fi
case "${quality}" in
  Low|Epic) ;;
  *) echo "quality must be Low or Epic" >&2; exit 2 ;;
esac

cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh
source scripts/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "exact owned CARLA route trial"

output_root="$(realpath -m -- "${output_root}")"
route_file="$(realpath -- "${route_file}")"
if [[ -e "${output_root}" || -L "${output_root}" ]]; then
  echo "output path already exists: ${output_root}" >&2
  exit 2
fi

map_name="$(
  python3 - "${route_file}" <<'PY'
import json
from pathlib import Path
import re
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
name = payload.get("town")
if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_]+", name):
    raise SystemExit("route town must contain only letters, digits, and underscore")
print(name)
PY
)"
# HH_260906 - Reject the packaged C-track Low profile before its known UE4 LOD crash.
if [[ "${map_name}" == "C_track_1_0_7" && "${quality}" == "Low" ]]; then
  echo "C_track_1_0_7 requires Epic quality because Low crashes in skeletal-mesh rendering" >&2
  exit 2
fi
map_id="$(tr '[:upper:]' '[:lower:]' <<< "${map_name}")"
full_map_manifest="${root}/scripts/e2e/autoware_vad_town_matrix.yaml"
if ! full_map_path="$(
  python3 - "${full_map_manifest}" "${map_name}" "${root}/data/maps" <<'PY'
from pathlib import Path
import sys

import yaml

manifest_path = Path(sys.argv[1]).resolve()
map_name = sys.argv[2]
allowed_root = Path(sys.argv[3]).resolve()
payload = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
bundles = payload.get("validated_full_map_bundles") if isinstance(payload, dict) else None
if not isinstance(bundles, dict):
    raise SystemExit("matrix manifest has no validated_full_map_bundles mapping")
canonical_map = f"/Game/Carla/Maps/{map_name}"
matches = [
    spec
    for spec in bundles.values()
    if isinstance(spec, dict) and spec.get("canonical_carla_map") == canonical_map
]
if len(matches) != 1:
    raise SystemExit(
        f"expected exactly one admitted full-map bundle for {canonical_map}; got {len(matches)}"
    )
raw_path = matches[0].get("path")
if not isinstance(raw_path, str) or not raw_path:
    raise SystemExit(f"admitted full-map bundle for {canonical_map} has no path")
bundle_path = (manifest_path.parent / raw_path).resolve()
if allowed_root != bundle_path and allowed_root not in bundle_path.parents:
    raise SystemExit(f"admitted full-map bundle escapes data/maps: {bundle_path}")
if not bundle_path.is_dir():
    raise SystemExit(f"admitted full-map bundle directory is missing: {bundle_path}")
print(bundle_path)
PY
)"; then
  echo "Could not resolve the admitted full-map bundle for ${map_name}" >&2
  exit 2
fi
if [[ ! -f "${full_map_path}/map_bundle.json" ]]; then
  echo "full map bundle is missing for ${map_name}: ${full_map_path}" >&2
  exit 2
fi

mkdir -p "${output_root}/attempts" "${output_root}/host_telemetry" \
  "${output_root}/carla_lifecycle"
exec > >(tee -a "${output_root}/owned_trial_console.log") 2>&1

server_pid=""
server_pgid=""
server_generation_id=""
server_log=""
vmstat_pid=""
gpu_pid=""
active_attempt=""

probe_server() {
  local output="$1"
  local stage="$2"
  local expected_mode="${3:-running}"
  local mode_args=()
  if [[ "${expected_mode}" == "stopped" ]]; then
    mode_args+=(--expect-stopped)
  fi
  python3 scripts/e2e/probe_carla_server.py \
    --host 127.0.0.1 --port "${port}" --timeout 3 \
    --expected-map "${map_name}" --stage "${stage}" \
    --generation-id "${server_generation_id}" \
    --owner-pid "${server_pid}" --owner-pgid "${server_pgid}" \
    --server-log "${server_log}" --output "${output}" \
    "${mode_args[@]}"
}

start_server() {
  local generation_id="$1"
  local log_path="$2"
  local deadline
  if [[ -n "${server_pid}" || -n "${server_pgid}" ]]; then
    echo "refusing to overlap owned CARLA generations" >&2
    return 2
  fi
  server_generation_id="${generation_id}"
  server_log="${log_path}"
  setsid scripts/e2e/run_carla_map.sh "${map_name}" \
    --port "${port}" --quality "${quality}" \
    --startup-timeout-sec "${startup_timeout_sec}" \
    -- -RenderOffScreen -nosound >"${server_log}" 2>&1 &
  server_pid=$!
  server_pgid="${server_pid}"
  deadline=$((SECONDS + startup_timeout_sec + 15))
  while (( SECONDS < deadline )); do
    if grep -q '^CARLA_READY ' "${server_log}" 2>/dev/null; then
      return 0
    fi
    if ! kill -0 "${server_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  echo "CARLA ${map_name} failed to reach READY; see ${server_log}" >&2
  return 1
}

stop_telemetry() {
  local pid
  for pid in "${vmstat_pid}" "${gpu_pid}"; do
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      e2e_stop_owned_process_group "${pid}" "${pid}" 5 3 1 || true
    fi
  done
  vmstat_pid=""
  gpu_pid=""
}

start_telemetry() {
  local prefix="$1"
  if command -v vmstat >/dev/null 2>&1; then
    setsid vmstat -t 1 >"${prefix}_vmstat.log" 2>&1 &
    vmstat_pid=$!
  fi
  if command -v nvidia-smi >/dev/null 2>&1; then
    setsid nvidia-smi dmon -s pucvmet -d 1 >"${prefix}_nvidia_dmon.log" 2>&1 &
    gpu_pid=$!
  fi
}

stop_server() {
  local output="${1:-}"
  local stage="${2:-owned_trial_cleanup}"
  local status=0
  if [[ -n "${server_pid}" && -n "${server_pgid}" ]]; then
    e2e_stop_owned_process_group "${server_pgid}" "${server_pid}" 30 10 5 || status=$?
    if [[ -n "${output}" ]]; then
      probe_server "${output}" "${stage}" stopped || status=$?
    fi
  fi
  server_pid=""
  server_pgid=""
  server_generation_id=""
  server_log=""
  return "${status}"
}

cleanup() {
  local exit_status=$?
  trap - EXIT INT TERM HUP
  stop_telemetry
  if [[ -n "${server_pid}" ]]; then
    if [[ -n "${active_attempt}" ]]; then
      emergency_lifecycle="${output_root}/carla_lifecycle/$(basename "${active_attempt}")"
      mkdir -p "${emergency_lifecycle}"
      stop_server "${emergency_lifecycle}/emergency_cleanup_health.json" \
        emergency_cleanup || true
    else
      stop_server || true
    fi
  fi
  exit "${exit_status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

attempt_rows=()
selected_attempt=""
final_status=1
total_attempts=$((max_health_retries + 1))
for ((attempt_number = 1; attempt_number <= total_attempts; attempt_number++)); do
  attempt_id="$(printf 'attempt_%03d' "${attempt_number}")"
  active_attempt="${output_root}/attempts/${attempt_id}"
  mkdir -p "${active_attempt}"
  lifecycle_dir="${output_root}/carla_lifecycle/${attempt_id}"
  mkdir -p "${lifecycle_dir}"
  server_log="${active_attempt}/carla_server.log"
  generation_id="${map_id}_${attempt_id}_$$"
  echo "OWNED_TRIAL_ATTEMPT id=${attempt_id} map=${map_name} quality=${quality}"
  if ! start_server "${generation_id}" "${server_log}"; then
    stop_server "${lifecycle_dir}/cold_start_failure_cleanup_health.json" \
      cold_start_failure_cleanup || true
    attempt_rows+=("${attempt_id}\tCARLA_START_FAILED\t1")
    final_status=1
    break
  fi
  if ! probe_server "${lifecycle_dir}/owner_preflight_health.json" \
    owner_preflight running; then
    stop_server "${lifecycle_dir}/owner_preflight_failure_cleanup_health.json" \
      owner_preflight_failure_cleanup || true
    attempt_rows+=("${attempt_id}\tCARLA_PREFLIGHT_FAILED\t1")
    final_status=1
    break
  fi

  start_telemetry "${output_root}/host_telemetry/${attempt_id}"
  set +e
  AUTOWARE_E2E_CARLA_GENERATION_ID="${generation_id}" \
  AUTOWARE_E2E_CARLA_EXPECTED_MAP="${map_name}" \
  AUTOWARE_E2E_CARLA_OWNER_PID="${server_pid}" \
  AUTOWARE_E2E_CARLA_OWNER_PGID="${server_pgid}" \
  AUTOWARE_E2E_CARLA_SERVER_LOG="${server_log}" \
  AUTOWARE_E2E_FULL_MAP_PATH="${full_map_path}" \
  CARLA_HOST=127.0.0.1 CARLA_PORT="${port}" \
    scripts/e2e/run_recorded_route_trial.sh \
      --runtime-health-gate --runtime-health-timeout 45 \
      "${trial_options[@]}" --ready-timeout "${ready_timeout_sec}" \
      "${active_attempt}" "${route_file}" carla_timeout:=60
  trial_status=$?
  set -e
  stop_telemetry
  cleanup_status=0
  stop_server "${lifecycle_dir}/owner_cleanup_health.json" \
    owner_trial_cleanup || cleanup_status=$?
  active_attempt=""

  health_status="MISSING"
  if [[ -f "${output_root}/attempts/${attempt_id}/runtime_health.json" ]]; then
    health_status="$(
      python3 - "${output_root}/attempts/${attempt_id}/runtime_health.json" <<'PY'
import json
from pathlib import Path
import sys

try:
    value = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    print(value.get("status", "INVALID"))
except (OSError, TypeError, ValueError):
    print("INVALID")
PY
    )"
  fi
  attempt_rows+=("${attempt_id}\t${health_status}\t${trial_status}")
  if (( cleanup_status != 0 )); then
    echo "Owned CARLA cleanup failed for ${attempt_id}; aborting" >&2
    final_status=1
    break
  fi
  if (( trial_status == 0 )); then
    selected_attempt="${attempt_id}"
    final_status=0
    break
  fi
  if [[ "${health_status}" != "FAIL" ]]; then
    echo "Trial failed outside the runtime-health gate; no automatic retry" >&2
    final_status="${trial_status}"
    break
  fi
  if (( attempt_number == total_attempts )); then
    echo "Runtime health remained below threshold after ${total_attempts} generations" >&2
    final_status=1
    break
  fi
  echo "RUNTIME_HEALTH_RETRY previous=${attempt_id} next=$((attempt_number + 1))"
done

rows_file="$(mktemp)"
printf '%b\n' "${attempt_rows[@]}" >"${rows_file}"
python3 - "${output_root}" "${route_file}" "${map_name}" "${quality}" \
  "${selected_attempt}" "${final_status}" "${rows_file}" \
  "${#trial_options[@]}" "${trial_options[@]}" <<'PY'
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

root, route, map_name, quality, selected, final_status, rows_path, count, *option_values = sys.argv[1:]
if len(option_values) != int(count):
    raise SystemExit("owned trial option-vector length changed")
attempts = []
for line in Path(rows_path).read_text(encoding="utf-8").splitlines():
    if not line:
        continue
    attempt_id, health, process_status = line.split("\t")
    attempts.append(
        {
            "attempt_id": attempt_id,
            "path": str((Path(root) / "attempts" / attempt_id).resolve()),
            "runtime_health_status": health,
            "process_exit_status": int(process_status),
        }
    )
portable_requested = "--portable-shadow-10hz" in option_values
portable_bindings = []
for attempt in attempts:
    binding_path = (
        Path(attempt["path"])
        / "portable_shadow_provenance"
        / "trial_binding.json"
    )
    if not binding_path.is_file() or binding_path.is_symlink():
        continue
    binding_bytes = binding_path.read_bytes()
    binding_sha256 = hashlib.sha256(binding_bytes).hexdigest()
    binding = json.loads(binding_bytes)
    route_binding = binding.get("route_binding", {})
    model = binding.get("model_provenance", {})
    shadow_launch = binding.get("shadow_launch", {})
    if not isinstance(shadow_launch, dict):
        shadow_launch = {}
    launch_recheck_path = binding_path.parent / "launch_recheck.json"
    launch_recheck_summary = None
    if launch_recheck_path.is_file() and not launch_recheck_path.is_symlink():
        launch_recheck_bytes = launch_recheck_path.read_bytes()
        launch_recheck = json.loads(launch_recheck_bytes)
        launch_recheck_source = launch_recheck.get("source_launch", {})
        if not isinstance(launch_recheck_source, dict):
            launch_recheck_source = {}
        launch_recheck_summary = {
            "path": str(launch_recheck_path.resolve()),
            "sha256": hashlib.sha256(launch_recheck_bytes).hexdigest(),
            "status": launch_recheck.get("status"),
            "check_stage": launch_recheck.get("check_stage"),
            "trial_binding_sha256": launch_recheck.get("trial_binding_sha256"),
            "matches_initial_binding": launch_recheck.get("matches_initial_binding"),
            "source_launch_sha256": launch_recheck_source.get("sha256"),
            "installed_binding": launch_recheck.get("installed_binding"),
            "runtime_shadow_launch": launch_recheck.get("runtime_shadow_launch"),
        }
    portable_bindings.append(
        {
            "attempt_id": attempt["attempt_id"],
            "path": str(binding_path.resolve()),
            "sha256": binding_sha256,
            "status": binding.get("status"),
            "execution_mode": binding.get("execution_mode"),
            "vehicle_control_approved": binding.get("vehicle_control_approved"),
            "declared_map_id": route_binding.get("declared_map_id"),
            "observed_map_id": route_binding.get("observed_map_id"),
            "aligned_route_sha256": route_binding.get("aligned_route_sha256"),
            "map_bundle_sha256": route_binding.get("map_bundle_sha256"),
            "runtime_bundle_sha256": model.get("runtime_bundle_sha256"),
            "source_checkpoint_sha256": model.get("source_checkpoint_sha256"),
            "source_launch_sha256": shadow_launch.get("sha256"),
            "installed_launch_binding": shadow_launch.get("installed_binding"),
            "launch_snapshot_binding": shadow_launch.get("snapshot_binding"),
            "launch_recheck": launch_recheck_summary,
        }
    )
if portable_requested and int(final_status) == 0:
    selected_bindings = [
        item for item in portable_bindings if item["attempt_id"] == selected
    ]
    if (
        len(selected_bindings) != 1
        or selected_bindings[0]["status"] != "PASS"
        or selected_bindings[0]["execution_mode"] != "shadow_only"
        or selected_bindings[0]["vehicle_control_approved"] is not False
        or selected_bindings[0]["declared_map_id"]
        != selected_bindings[0]["observed_map_id"]
    ):
        raise SystemExit("selected Portable shadow attempt lacks an exact safe binding")
    selected_binding = selected_bindings[0]
    selected_recheck = selected_binding.get("launch_recheck")
    if (
        not isinstance(selected_recheck, dict)
        or selected_recheck.get("status") != "PASS"
        or selected_recheck.get("check_stage") != "immediately_before_ros_launch"
        or selected_recheck.get("trial_binding_sha256")
        != selected_binding.get("sha256")
        or selected_recheck.get("matches_initial_binding") is not True
        or selected_recheck.get("source_launch_sha256")
        != selected_binding.get("source_launch_sha256")
        or selected_recheck.get("installed_binding")
        != selected_binding.get("installed_launch_binding")
    ):
        raise SystemExit("selected Portable shadow attempt lacks an exact launch recheck")
    selected_snapshot = selected_binding.get("launch_snapshot_binding")
    if (
        not isinstance(selected_snapshot, dict)
        or selected_recheck.get("runtime_shadow_launch") != selected_snapshot
        or selected_snapshot.get("direct_ros_launch") is not True
    ):
        raise SystemExit(
            "selected Portable shadow attempt lacks an exact runtime launch snapshot"
        )
payload = {
    "schema_version": 1,
    "status": "PASS" if int(final_status) == 0 else "FAIL",
    "generated_at": datetime.now(timezone.utc).isoformat(),
    "map": map_name,
    "carla_quality": quality,
    "route": str(Path(route).resolve()),
    "route_sha256": hashlib.sha256(Path(route).read_bytes()).hexdigest(),
    "trial_options": option_values,
    "retry_policy": "fresh CARLA generation only after runtime-health FAIL",
    "attempts": attempts,
    "selected_attempt": selected or None,
    "portable_shadow": {
        "requested": portable_requested,
        "execution_mode": "shadow_only" if portable_requested else None,
        "vehicle_control_approved": False if portable_requested else None,
        "attempt_bindings": portable_bindings,
    },
}
target = Path(root) / "owned_trial_summary.json"
fd, staged_name = tempfile.mkstemp(prefix=".owned_trial_summary.", dir=target.parent)
try:
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(staged_name, target)
finally:
    Path(staged_name).unlink(missing_ok=True)
PY
rm -f -- "${rows_file}"

trap - EXIT INT TERM HUP
if [[ -n "${server_pid}" ]]; then
  stop_server || true
fi
stop_telemetry
echo "OWNED_TRIAL_RESULT status=$([[ ${final_status} -eq 0 ]] && echo PASS || echo FAIL) selected=${selected_attempt:-none} root=${output_root}"
exit "${final_status}"
