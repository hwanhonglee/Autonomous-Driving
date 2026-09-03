#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
  cat >&2 <<'EOF'
Usage: scripts/e2e/run_autoware_vad_60kph_pilot.sh OUTPUT_ROOT [options]

Run one exploratory 60 km/h straight-only Autoware VAD trial on a fresh Town06
CARLA generation.  The helper creates a deterministic 430-460 m route, records
the pinned localhost-only 5 Hz Best-Effort depth-1 camera/RViz evidence and
host telemetry, and
owns every CARLA process it starts.  It never labels the result as
real-vehicle-ready calibration evidence.

Options:
  --port PORT                 CARLA RPC port (default: 2100)
  --startup-timeout-sec SEC   CARLA cold-start timeout (default: 180)
  --ready-timeout-sec SEC     Autoware readiness timeout (default: 600)
  --geometry-ab-route-corridor-0p2
                              Change only the route corridor from 0.50 m to
                              0.20 m for the geometry A/B candidate
  -h, --help                  show this help
EOF
}

if [[ $# -eq 0 ]]; then
  usage
  exit 2
fi
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

output_root="$1"
shift
port=2100
startup_timeout_sec=180
ready_timeout_sec=600
geometry_ab_route_corridor_0p2=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      port="$2"
      shift 2
      ;;
    --startup-timeout-sec)
      [[ $# -ge 2 ]] || {
        echo "--startup-timeout-sec requires a value" >&2
        exit 2
      }
      startup_timeout_sec="$2"
      shift 2
      ;;
    --ready-timeout-sec)
      [[ $# -ge 2 ]] || {
        echo "--ready-timeout-sec requires a value" >&2
        exit 2
      }
      ready_timeout_sec="$2"
      shift 2
      ;;
    --geometry-ab-route-corridor-0p2)
      geometry_ab_route_corridor_0p2=true
      shift
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
for value in "${startup_timeout_sec}" "${ready_timeout_sec}"; do
  if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
    echo "timeouts must be positive integers" >&2
    exit 2
  fi
done

output_root="$(
  python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().resolve())' \
    "${output_root}"
)"
if [[ -e "${output_root}" || -L "${output_root}" ]]; then
  echo "output path already exists: ${output_root}" >&2
  exit 2
fi

cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh
source scripts/e2e/workspace_runtime_lock.sh
e2e_acquire_workspace_runtime_lock "60 kph exploratory pilot"

for command in flock vmstat pidstat nvidia-smi; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "60 kph pilot requires ${command}" >&2
    exit 2
  fi
done
if [[ -z "${DISPLAY:-}" ]]; then
  echo "60 kph visual pilot requires DISPLAY" >&2
  exit 2
fi

scripts/e2e/apply_vad_object_safety_patches.sh
python3 scripts/e2e/vad_object_safety_build_provenance.py verify
scripts/e2e/apply_mission_planner_lane_only_patch.sh
python3 scripts/e2e/mission_planner_build_provenance.py verify

mkdir -p "${output_root}/host_telemetry"
exec > >(tee -a "${output_root}/pilot_console.log") 2>&1

server_pid=""
server_pgid=""
server_generation_id=""
server_log=""
vmstat_pid=""
gpu_pid=""
pidstat_pid=""
cleaned=false
pilot_complete=false
phase="initialization"
current_cleanup_evidence=""
current_cleanup_stage=""

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
    --expected-map Town06 --stage "${stage}" \
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
    echo "refusing to replace an owned CARLA generation before cleanup" >&2
    return 2
  fi
  server_generation_id="${generation_id}"
  server_log="${log_path}"
  mkdir -p "$(dirname "${server_log}")"
  setsid scripts/e2e/run_carla_map.sh Town06 \
    --port "${port}" --quality Epic \
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
  echo "CARLA Town06 failed to reach READY; see ${server_log}" >&2
  return 1
}

stop_server() {
  local stopped_evidence="${1:-}"
  local stopped_stage="${2:-pilot_cleanup}"
  local status=0
  if [[ -n "${server_pid}" && -n "${server_pgid}" ]]; then
    e2e_stop_owned_process_group "${server_pgid}" "${server_pid}" 30 10 5 || status=$?
    if [[ -n "${stopped_evidence}" ]]; then
      probe_server "${stopped_evidence}" "${stopped_stage}" stopped || status=$?
    fi
  fi
  if (( status == 0 )); then
    server_pid=""
    server_pgid=""
    server_generation_id=""
    server_log=""
    current_cleanup_evidence=""
    current_cleanup_stage=""
  fi
  return "${status}"
}

stop_telemetry() {
  local pid
  local status=0
  for pid in "${vmstat_pid}" "${gpu_pid}" "${pidstat_pid}"; do
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      e2e_stop_owned_process_group "${pid}" "${pid}" 5 3 1 || status=$?
    fi
  done
  if (( status == 0 )); then
    vmstat_pid=""
    gpu_pid=""
    pidstat_pid=""
  fi
  return "${status}"
}

cleanup() {
  local exit_status=$?
  local cleanup_status=0
  local telemetry_status=0
  if [[ "${cleaned}" == "true" ]]; then
    return
  fi
  cleaned=true
  if [[ -n "${server_pid}" ]]; then
    mkdir -p "$(dirname "${current_cleanup_evidence}")"
    stop_server "${current_cleanup_evidence}" \
      "${current_cleanup_stage:-${phase}_emergency_cleanup}" || cleanup_status=$?
  fi
  stop_telemetry || telemetry_status=$?
  if [[ "${pilot_complete}" != "true" && -d "${output_root}" ]]; then
    python3 - "${output_root}/pilot_failure.json" "${phase}" \
      "${exit_status}" "${cleanup_status}" "${telemetry_status}" <<'PY'
import json
import os
from datetime import datetime, timezone
from pathlib import Path
import tempfile
import sys

output, phase, process_status, cleanup_status, telemetry_status = sys.argv[1:]
payload = {
    "schema_version": 1,
    "status": "FAILED",
    "phase": phase,
    "process_exit_status": int(process_status),
    "carla_cleanup_status": int(cleanup_status),
    "telemetry_cleanup_status": int(telemetry_status),
    "recorded_at": datetime.now(timezone.utc).isoformat(),
}
target = Path(output)
descriptor, staged_name = tempfile.mkstemp(
    prefix=f".{target.name}.", suffix=".staged", dir=target.parent
)
staged = Path(staged_name)
try:
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(staged, target)
finally:
    staged.unlink(missing_ok=True)
PY
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

setsid vmstat -t 1 >"${output_root}/host_telemetry/vmstat.log" 2>&1 &
vmstat_pid=$!
setsid nvidia-smi dmon -s pucvmet -d 1 -o DT \
  >"${output_root}/host_telemetry/nvidia_smi_dmon.log" 2>&1 &
gpu_pid=$!
setsid pidstat -urd -h -p ALL 1 \
  >"${output_root}/host_telemetry/pidstat.log" 2>&1 &
pidstat_pid=$!
python3 - "${output_root}/host_telemetry/preflight.json" <<'PY'
import json
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys

output = Path(sys.argv[1])
payload = {
    "captured_at": datetime.now(timezone.utc).isoformat(),
    "nvidia_smi_query": "utilization.gpu,memory.used,memory.free,temperature.gpu",
    "nvidia_smi_csv": subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=utilization.gpu,memory.used,memory.free,temperature.gpu",
            "--format=csv,noheader,nounits",
        ],
        text=True,
    ).strip(),
}
output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
sleep 5
if ! kill -0 "${vmstat_pid}" 2>/dev/null || \
   ! kill -0 "${gpu_pid}" 2>/dev/null || \
   ! kill -0 "${pidstat_pid}" 2>/dev/null || \
   [[ ! -s "${output_root}/host_telemetry/vmstat.log" ]] || \
   [[ ! -s "${output_root}/host_telemetry/nvidia_smi_dmon.log" ]] || \
   [[ ! -s "${output_root}/host_telemetry/pidstat.log" ]]; then
  echo "host telemetry failed its five-second baseline preflight" >&2
  exit 1
fi

catalog_root="${output_root}/catalog"
catalog_lifecycle="${output_root}/carla_lifecycle/catalog"
mkdir -p "${catalog_lifecycle}"
phase="catalog_startup"
current_cleanup_evidence="${catalog_lifecycle}/cleanup.json"
current_cleanup_stage="catalog_emergency_cleanup"
start_server town06_60kph_catalog_001 "${output_root}/carla_catalog.log"
phase="catalog_generation"
probe_server "${catalog_lifecycle}/preflight.json" catalog_preflight
python3 scripts/e2e/prepare_carla_expert_route_catalog.py \
  --manifest scripts/e2e/carla_expert_suite.yaml \
  --map-id town06 --output-root "${catalog_root}" \
  --host 127.0.0.1 --port "${port}" --timeout 30 \
  --map-load-settle-sec 0 --active-server-profile packaged_0915 \
  --weather ClearNoon --seeds 0 --scenarios straight \
  --pairs-per-seed 1 --min-distance 430.0 --max-distance 460.0 \
  --preferred-distance 445.0 --sampling-resolution 1.0 \
  --endpoint-waypoint-spacing-m 0.5 --endpoint-junction-policy exclude \
  --candidate-enumeration-policy directed_topology_straight_v1 \
  --straight-capacity-profile town06_60kph_straight_pilot_v1 \
  --physical-straight-profile speed_60kph_straight_pilot \
  --max-endpoint-offset 2.0 --max-traces 20000 \
  >"${output_root}/route_catalog.log" 2>&1
probe_server "${catalog_lifecycle}/completion.json" catalog_completion
stop_server "${catalog_lifecycle}/cleanup.json" catalog_cleanup

route_path="$(python3 - "${catalog_root}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
catalog = json.loads((root / "route_catalog.json").read_text(encoding="utf-8"))
routes = catalog.get("routes")
if not isinstance(routes, list) or len(routes) != 1 or routes[0].get("status") != "ready":
    raise SystemExit("60 kph pilot catalog must contain exactly one ready route")
generation = catalog.get("generation")
if catalog.get("status") != "complete" or catalog.get("map_id") != "town06":
    raise SystemExit("60 kph pilot catalog identity is not complete Town06")
if not isinstance(generation, dict):
    raise SystemExit("60 kph pilot catalog has no generation contract")
capacity = generation.get("straight_capacity_contract")
physical = generation.get("physical_straight_contract")
if (
    not isinstance(capacity, dict)
    or capacity.get("profile_id") != "town06_60kph_straight_pilot_v1"
    or not isinstance(physical, dict)
    or physical.get("profile_id") != "speed_60kph_straight_pilot"
):
    raise SystemExit("60 kph pilot catalog profile identity changed")
relative = routes[0].get("path")
if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
    raise SystemExit("60 kph pilot route path must be relative to its catalog")
route = root / relative
try:
    resolved_route = route.resolve(strict=True)
    resolved_route.relative_to(root)
except (OSError, ValueError) as error:
    raise SystemExit(f"selected 60 kph route escapes its catalog: {error}") from error
if route.is_symlink() or not route.is_file():
    raise SystemExit(f"selected 60 kph route is missing: {route}")
actual = hashlib.sha256(route.read_bytes()).hexdigest()
if actual != routes[0].get("sha256"):
    raise SystemExit("selected 60 kph route SHA-256 mismatch")
payload = json.loads(route.read_text(encoding="utf-8"))
length = float(payload.get("route_length_m", 0.0))
if payload.get("town") != "Town06" or payload.get("scenario") != "straight":
    raise SystemExit("selected pilot route identity is not Town06/straight")
physical_preflight = payload.get("physical_straight_preflight")
if not isinstance(physical_preflight, dict) or physical_preflight.get("status") != "PASS":
    raise SystemExit("selected pilot route lacks physical-straight PASS evidence")
if not 430.0 <= length <= 460.0:
    raise SystemExit(f"selected pilot route length is outside [430, 460] m: {length}")
print(route.resolve())
PY
)"

attempt_root="${output_root}/trial/attempt_001"
mkdir -p "${attempt_root}"
phase="trial_startup"
current_cleanup_evidence="${attempt_root}/carla_cleanup_health.json"
current_cleanup_stage="trial_emergency_cleanup"
start_server town06_60kph_straight_attempt_001 "${attempt_root}/carla_server.log"
trial_pid="${server_pid}"
trial_pgid="${server_pgid}"
trial_log="${server_log}"
trial_generation="${server_generation_id}"
set +e
phase="recorded_trial"
geometry_ab_arguments=()
if [[ "${geometry_ab_route_corridor_0p2}" == "true" ]]; then
  geometry_ab_arguments+=(--geometry-ab-route-corridor-0p2)
fi
CARLA_HOST=127.0.0.1 CARLA_PORT="${port}" \
  AUTOWARE_E2E_CARLA_GENERATION_ID="${trial_generation}" \
  AUTOWARE_E2E_CARLA_EXPECTED_MAP=Town06 \
  AUTOWARE_E2E_CARLA_OWNER_PID="${trial_pid}" \
  AUTOWARE_E2E_CARLA_OWNER_PGID="${trial_pgid}" \
  AUTOWARE_E2E_CARLA_SERVER_LOG="${trial_log}" \
  AUTOWARE_E2E_FULL_MAP_PATH="${root}/data/maps/Town06_full" \
  scripts/e2e/run_recorded_route_trial.sh \
    --recommended --speed-60kph-pilot --camera-source-5hz \
    "${geometry_ab_arguments[@]}" \
    --visualize --capture-desktop --ready-timeout "${ready_timeout_sec}" \
    "${attempt_root}" "${route_path}" carla_timeout:=60
trial_status=$?
set -e
cleanup_status=0
phase="trial_cleanup"
stop_server "${attempt_root}/carla_cleanup_health.json" trial_cleanup || cleanup_status=$?
telemetry_cleanup_status=0
stop_telemetry || telemetry_cleanup_status=$?

coverage_status=0
if [[ -f "${attempt_root}/actuation_map_coverage.json" ]]; then
  python3 - "${attempt_root}/actuation_map_coverage.json" <<'PY' || coverage_status=$?
import json
import math
from pathlib import Path
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if payload.get("profile_id") != "carla_vad_60kph_straight_pilot_v1":
    raise SystemExit("actuation coverage profile identity mismatch")
if not math.isclose(
    float(payload.get("target_speed_mps", -1.0)),
    16.666666666666668,
    abs_tol=1.0e-9,
):
    raise SystemExit("actuation coverage target mismatch")
if payload.get("status") not in {"PASS", "EXPLORATORY"}:
    raise SystemExit("actuation coverage did not authorize the pilot")
boundary = payload.get("validation_boundary")
if not isinstance(boundary, dict) or boundary.get("real_vehicle_ready") is not False:
    raise SystemExit("actuation coverage lost the real-vehicle boundary")
if (
    payload.get("status") == "EXPLORATORY"
    and payload.get(
        "target_envelope_extension_authorized_for_exploratory_simulation"
    ) is not True
):
    raise SystemExit("out-of-axis target lacks explicit simulation authorization")
PY
else
  echo "pre-trial actuation coverage audit is absent" >&2
  coverage_status=2
fi

camera_integrity_status=0
phase="camera_integrity_validation"
python3 - "${attempt_root}" <<'PY' || camera_integrity_status=$?
import importlib.util
from pathlib import Path
import sys

root = Path.cwd()
module_path = root / "scripts/e2e/autoware_vad_town_matrix.py"
spec = importlib.util.spec_from_file_location("autoware_vad_town_matrix", module_path)
if spec is None or spec.loader is None:
    raise SystemExit("cannot load the camera-source evidence validator")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
trial = Path(sys.argv[1]).resolve()
runtime = module._runtime_env(trial / "runtime.env")
latency = module.read_object(
    trial / "latency/e2e_latency.json", "60 kph pilot latency"
)
evidence = module._camera_source_5hz_evidence(
    trial,
    runtime,
    {
        "camera_source_contract": dict(
            module.CAMERA_SOURCE_5HZ_BEST_EFFORT_IMAGE_DEPTH1_CONTRACT
        )
    },
    latency,
)
if not isinstance(evidence, dict) or evidence.get("status") != "PASS":
    raise SystemExit("camera-source evidence did not pass")
module.atomic_json(
    trial / "camera_source_5hz_validation.json",
    {"schema_version": 1, **evidence},
)
PY

runtime_load_status=0
phase="runtime_load_analysis"
python3 scripts/e2e/analyze_pilot_runtime_load.py \
  --trial-dir "${attempt_root}" \
  --host-telemetry-dir "${output_root}/host_telemetry" \
  --output-dir "${attempt_root}" || runtime_load_status=$?

acceptance_gate_status=0
phase="pilot_acceptance_validation"
python3 scripts/e2e/validate_60kph_pilot_contract.py \
  --attempt-dir "${attempt_root}" \
  --source-route "${route_path}" \
  --output "${attempt_root}/pilot_acceptance_gate.json" || \
  acceptance_gate_status=$?

handoff_status=0
phase="pilot_handoff_validation"
python3 - "${output_root}" "${route_path}" "${trial_status}" \
  "${cleanup_status}" "${coverage_status}" "${telemetry_cleanup_status}" \
  "${camera_integrity_status}" "${runtime_load_status}" \
  "${acceptance_gate_status}" <<'PY' || handoff_status=$?
import hashlib
import json
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import tempfile
import sys

(
    root_name,
    route_name,
    trial_status,
    cleanup_status,
    coverage_status,
    telemetry_status,
    camera_status,
    runtime_load_status,
    acceptance_gate_status,
) = sys.argv[1:]
root = Path(root_name)
route = Path(route_name)
attempt = root / "trial/attempt_001"
problems = []


def read_object(name):
    path = attempt / name
    try:
        if path.is_symlink() or not path.is_file():
            raise OSError("not a regular file")
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        problems.append(f"cannot read {name}: {error}")
        return {}
    if not isinstance(value, dict):
        problems.append(f"{name} is not a JSON object")
        return {}
    return value


result = read_object("result.json")
speed = read_object("speed_profile.json")
desktop = read_object("desktop_capture.json")
coverage = read_object("actuation_map_coverage.json")
runtime_coverage = read_object("actuation_map_runtime_coverage.json")
camera = read_object("camera_source_5hz_validation.json")
longitudinal = read_object("longitudinal_response.json")
runtime_load = read_object("runtime_load_analysis.json")
preflight = read_object("carla_preflight_health.json")
completion = read_object("carla_completion_health.json")
cleanup = read_object("carla_cleanup_health.json")
acceptance_gate = read_object("pilot_acceptance_gate.json")


def read_environment():
    values = {}
    try:
        lines = (attempt / "runtime.env").read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        problems.append(f"cannot read runtime.env for geometry provenance: {error}")
        return values
    for number, line in enumerate(lines, 1):
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            problems.append(f"runtime.env line {number} has no '='")
            continue
        key, value = line.split("=", 1)
        if not key or key in values:
            problems.append(f"runtime.env line {number} has an invalid/duplicate key")
            continue
        values[key] = value
    return values


environment = read_environment()
geometry_candidate_id = environment.get("GEOMETRY_AB_CANDIDATE_ID")
geometry_enabled = environment.get("GEOMETRY_AB_ROUTE_CORRIDOR_0P2")
try:
    geometry_route_width = float(environment["ROUTE_CORRIDOR_HALF_WIDTH_M"])
    geometry_turn_width = float(
        environment["TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M"]
    )
except (KeyError, TypeError, ValueError):
    geometry_route_width = None
    geometry_turn_width = None
geometry_variant = {
    "candidate_id": geometry_candidate_id,
    "route_corridor_0p2": (
        geometry_enabled == "true" if geometry_enabled is not None else None
    ),
    "route_corridor_half_width_m": geometry_route_width,
    "turn_outward_corridor_half_width_m": geometry_turn_width,
    "parameter_dump_sha256": (
        acceptance_gate.get("geometry_variant", {}).get("parameter_dump_sha256")
        if isinstance(acceptance_gate.get("geometry_variant"), dict)
        else None
    ),
    "trajectory_checksum_manifest_sha256": (
        acceptance_gate.get("geometry_variant", {}).get(
            "trajectory_checksum_manifest_sha256"
        )
        if isinstance(acceptance_gate.get("geometry_variant"), dict)
        else None
    ),
    "acceptance_gate_provenance_status": (
        acceptance_gate.get("geometry_variant", {}).get("provenance_status")
        if isinstance(acceptance_gate.get("geometry_variant"), dict)
        else None
    ),
}


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_bound_sources(value, label="sources"):
    if isinstance(value, dict):
        if "path" in value and "sha256" in value:
            path_value = value.get("path")
            expected_digest = value.get("sha256")
            if not isinstance(path_value, str) or not isinstance(expected_digest, str):
                problems.append(f"{label} path/SHA identity is incomplete")
            else:
                path = Path(path_value).expanduser()
                try:
                    if path.is_symlink() or not path.is_file():
                        raise OSError("not a regular file")
                    actual_digest = sha256_file(path)
                except OSError as error:
                    problems.append(f"{label} source is invalid: {error}")
                else:
                    if actual_digest != expected_digest:
                        problems.append(f"{label} source SHA changed after acceptance")
        for key, child in value.items():
            verify_bound_sources(child, f"{label}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            verify_bound_sources(child, f"{label}[{index}]")


bound_sources = acceptance_gate.get("sources")
if not isinstance(bound_sources, dict):
    problems.append("acceptance gate source bindings are missing")
else:
    verify_bound_sources(bound_sources)


def positive_integer(name):
    value = environment.get(name)
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if str(parsed) == value and parsed > 0 else None


expected_generation = environment.get("CARLA_GENERATION_ID")
expected_owner_pid = positive_integer("CARLA_OWNER_PID")
expected_owner_pgid = positive_integer("CARLA_OWNER_PGID")
expected_port = positive_integer("CARLA_PORT")
if (
    environment.get("CARLA_HOST") != "127.0.0.1"
    or environment.get("CARLA_LIFECYCLE")
    != "cold_start_owned_process_group_per_trial"
    or environment.get("CARLA_EXPECTED_MAP") != "Town06"
    or environment.get("CARLA_MATRIX_OWNED") != "true"
    or not isinstance(expected_generation, str)
    or not expected_generation
    or expected_owner_pid is None
    or expected_owner_pgid is None
    or expected_owner_pid != expected_owner_pgid
    or expected_port is None
    or expected_port > 65535
    or Path(environment.get("CARLA_SERVER_LOG", "")).expanduser().resolve()
    != (attempt / "carla_server.log").resolve()
):
    problems.append("owned CARLA runtime identity is incomplete or changed")

lifecycle_documents = {
    "preflight": (preflight, "carla_preflight_health.json", "trial_preflight", "running"),
    "completion": (completion, "carla_completion_health.json", "trial_completion", "running"),
    "cleanup": (cleanup, "carla_cleanup_health.json", "trial_cleanup", "stopped"),
}
lifecycle_summary = {}
lifecycle_sources = (
    bound_sources.get("carla_lifecycle", {})
    if isinstance(bound_sources, dict)
    else {}
)
for key, (document, name, stage, mode) in lifecycle_documents.items():
    expected_path = (attempt / name).resolve()
    source_record = (
        lifecycle_sources.get(key) if isinstance(lifecycle_sources, dict) else None
    )
    if (
        not isinstance(source_record, dict)
        or Path(source_record.get("path", "")).expanduser().resolve() != expected_path
    ):
        problems.append(f"CARLA {key} acceptance source path mismatch")
    if (
        document.get("schema_version") != 1
        or document.get("stage") != stage
        or document.get("status") != "PASS"
        or document.get("mode") != mode
        or document.get("expected_map") != "Town06"
        or document.get("generation_id") != expected_generation
        or document.get("owner_pid") != expected_owner_pid
        or document.get("owner_pgid") != expected_owner_pgid
        or document.get("port") != expected_port
        or document.get("read_only") is not True
        or document.get("error") is not None
    ):
        problems.append(f"CARLA {key} lifecycle contract mismatch")
    if key != "cleanup" and (
        document.get("active_map_name") != "Carla/Maps/Town06"
        or document.get("active_map_basename") != "Town06"
        or not isinstance(document.get("owner_process_state"), str)
        or not document.get("owner_process_state")
    ):
        problems.append(f"CARLA {key} running-state/map contract mismatch")
    if key == "cleanup" and (
        document.get("port_released") is not True
        or document.get("owner_process_state") is not None
    ):
        problems.append("CARLA cleanup did not prove owner exit and RPC port release")
    lifecycle_summary[key] = {
        "path": str(expected_path),
        "sha256": source_record.get("sha256") if isinstance(source_record, dict) else None,
        "status": document.get("status"),
        "stage": document.get("stage"),
        "mode": document.get("mode"),
        "generation_id": document.get("generation_id"),
        "owner_pid": document.get("owner_pid"),
        "owner_pgid": document.get("owner_pgid"),
        "port_released": document.get("port_released"),
    }

statuses = {
    "trial_exit_status": int(trial_status),
    "carla_cleanup_status": int(cleanup_status),
    "actuation_coverage_exit_status": int(coverage_status),
    "telemetry_cleanup_status": int(telemetry_status),
    "camera_integrity_exit_status": int(camera_status),
    "runtime_load_analysis_exit_status": int(runtime_load_status),
    "pilot_acceptance_gate_exit_status": int(acceptance_gate_status),
}
for label, value in statuses.items():
    if value != 0:
        problems.append(f"{label}={value}")
assessment = result.get("assessment")
final = result.get("final")
speed_exposure = result.get("speed_exposure")
goal_reached = isinstance(final, dict) and final.get("goal_reached") is True
route_status = final.get("route_status") if isinstance(final, dict) else None
physical_goal_confirmed = goal_reached and route_status == "goal_reached"
speed_exposure_status = (
    speed_exposure.get("status") if isinstance(speed_exposure, dict) else None
)
if result.get("execution_mode") != "full_stack":
    problems.append("route-test execution mode is not full_stack")
if not physical_goal_confirmed:
    problems.append("physical route goal was not reached")
if speed_exposure_status != "PASS":
    problems.append("60 kph speed exposure contract did not pass")
if (
    result.get("success") is False
    and physical_goal_confirmed
    and speed_exposure_status == "PASS"
):
    problems.append("full-stack route-test verdict failed for a non-speed reason")
expected_assessment = "PASS" if result.get("success") is True else "FAIL"
if (
    not isinstance(assessment, dict)
    or assessment.get("route_completion") != expected_assessment
):
    problems.append("route-test assessment is inconsistent with its overall verdict")
inputs = speed.get("inputs")
if (
    speed.get("status") != "complete"
    or not isinstance(inputs, dict)
    or inputs.get("profile_id") != "carla_vad_60kph_straight_pilot_v1"
    or not math.isclose(
        float(inputs.get("target_speed_mps", -1.0)),
        16.666666666666668,
        abs_tol=1.0e-9,
    )
):
    problems.append("speed-profile handoff identity is incomplete or changed")
view = desktop.get("rviz_view_contract")
if (
    desktop.get("candidate_observed") is not True
    or desktop.get("capture_started_after_candidate") is not True
    or not isinstance(view, dict)
    or view.get("vehicle_centered") is not True
):
    problems.append("owned RViz evidence is not candidate-backed and vehicle-centered")
for name in (
    "autoware_rviz_fullscreen.png",
    "autoware_rviz_candidate.png",
    "autoware_rviz_drive.gif",
):
    if not (attempt / name).is_file() or (attempt / name).stat().st_size <= 0:
        problems.append(f"owned RViz evidence is missing: {name}")
boundary = coverage.get("validation_boundary")
if (
    coverage.get("status") not in {"PASS", "EXPLORATORY"}
    or not isinstance(boundary, dict)
    or boundary.get("real_vehicle_ready") is not False
):
    problems.append("actuation-map validation boundary is invalid")
runtime_lookup = runtime_coverage.get("runtime_lookup_observation")
if (
    runtime_coverage.get("status") not in {"PASS", "EXPLORATORY"}
    or not isinstance(runtime_lookup, dict)
    or runtime_lookup.get("available") is not True
):
    problems.append("post-run actuation-map lookup evidence is invalid")
if camera.get("status") != "PASS":
    problems.append("six-camera 5 Hz integrity evidence did not pass")
if longitudinal.get("status") != "complete":
    problems.append("longitudinal-response evidence is incomplete")
runtime_findings = runtime_load.get("findings")
runtime_support = runtime_load.get("support")
runtime_problems = runtime_load.get("problems")
runtime_vad = runtime_load.get("vad_runtime")
runtime_classification = (
    runtime_findings.get("classification")
    if isinstance(runtime_findings, dict)
    else None
)
runtime_claims = (
    runtime_support.get("classification_claims")
    if isinstance(runtime_support, dict)
    else None
)
if (
    runtime_load.get("schema_version") != 3
    or runtime_load.get("analysis")
    != "CARLA/VAD pilot runtime-load phase reconstruction"
    or runtime_load.get("status") != "complete"
    or not isinstance(runtime_problems, list)
    or bool(runtime_problems)
    or not isinstance(runtime_vad, dict)
    or not isinstance(runtime_support, dict)
    or runtime_support.get("classification_supported") is not True
    or not isinstance(runtime_classification, str)
    or not runtime_classification
    or runtime_support.get("classification") != runtime_classification
    or not isinstance(runtime_support.get("camera_pattern_supported"), bool)
    or not isinstance(
        runtime_support.get("no_host_wide_saturation_supported"), bool
    )
    or not isinstance(runtime_claims, dict)
    or not isinstance(runtime_claims.get("camera_delivery_pattern"), bool)
    or not isinstance(
        runtime_claims.get("host_wide_saturation_excluded"), bool
    )
    or (
        runtime_claims.get("camera_delivery_pattern") is True
        and runtime_support.get("camera_pattern_supported") is not True
    )
    or (
        runtime_claims.get("host_wide_saturation_excluded") is True
        and runtime_support.get("no_host_wide_saturation_supported") is not True
    )
):
    problems.append("runtime-load evidence contract is incomplete or unsupported")
expected_goal_status = "PASS" if physical_goal_confirmed else "FAILED"
expected_speed_status = "PASS" if speed_exposure_status == "PASS" else "FAILED"
expected_route_test_status = "PASS" if result.get("success") is True else "FAILED"
if (
    acceptance_gate.get("physical_goal_completion_status") != expected_goal_status
    or acceptance_gate.get("speed_exposure_contract_status")
    != expected_speed_status
    or acceptance_gate.get("full_stack_route_test_status")
    != expected_route_test_status
):
    problems.append("acceptance gate goal/speed/overall statuses are inconsistent")
if (
    acceptance_gate.get("schema_version") != 1
    or acceptance_gate.get("gate_id")
    != "carla_vad_60kph_pilot_acceptance_v1"
    or acceptance_gate.get("evidence_integrity_status") != "PASS"
    or acceptance_gate.get("simulation_pilot_acceptance_status") != "PASS"
    or acceptance_gate.get("real_vehicle_ready") is not False
):
    problems.append("60 kph pilot acceptance contract did not pass")

metrics = result.get("metrics") if isinstance(result.get("metrics"), dict) else {}
sim_elapsed = metrics.get("sim_elapsed_sec")
wall_elapsed = metrics.get("wall_elapsed_sec")
real_time_factor = None
if isinstance(sim_elapsed, (int, float)) and isinstance(wall_elapsed, (int, float)):
    if math.isfinite(float(sim_elapsed)) and math.isfinite(float(wall_elapsed)) and wall_elapsed > 0:
        real_time_factor = float(sim_elapsed) / float(wall_elapsed)
payload = {
    "schema_version": 1,
    "status": "PASS" if not problems else "FAILED",
    "profile_id": "carla_vad_60kph_straight_pilot_v1",
    "validation_state": "exploratory_simulation_only",
    "evidence_integrity_status": acceptance_gate.get(
        "evidence_integrity_status"
    ),
    "simulation_pilot_acceptance_status": acceptance_gate.get(
        "simulation_pilot_acceptance_status"
    ),
    "physical_goal_completion_status": acceptance_gate.get(
        "physical_goal_completion_status"
    ),
    "speed_exposure_contract_status": acceptance_gate.get(
        "speed_exposure_contract_status"
    ),
    "full_stack_route_test_status": acceptance_gate.get(
        "full_stack_route_test_status"
    ),
    "real_vehicle_readiness_status": acceptance_gate.get(
        "real_vehicle_readiness_status"
    ),
    "real_vehicle_ready": False,
    "completed_at": datetime.now(timezone.utc).isoformat(),
    "route": {
        "path": str(route),
        "sha256": hashlib.sha256(route.read_bytes()).hexdigest(),
    },
    "exit_statuses": statuses,
    "carla_lifecycle": lifecycle_summary,
    "geometry_variant": geometry_variant,
    "acceptance_gate": {
        "path": str(attempt / "pilot_acceptance_gate.json"),
        "sha256": (
            hashlib.sha256(
                (attempt / "pilot_acceptance_gate.json").read_bytes()
            ).hexdigest()
            if (attempt / "pilot_acceptance_gate.json").is_file()
            else None
        ),
        "integrity_failures": acceptance_gate.get("integrity_failures"),
        "acceptance_failures": acceptance_gate.get("acceptance_failures"),
        "readiness_blockers": acceptance_gate.get("readiness_blockers"),
        "source_bindings": bound_sources,
    },
    "problems": problems,
    "summary": {
        "goal_reached": goal_reached,
        "route_status": route_status,
        "route_result_success": result.get("success"),
        "speed_exposure_status": speed_exposure_status,
        "physical_goal_completion_status": expected_goal_status,
        "speed_exposure_contract_status": expected_speed_status,
        "full_stack_route_test_status": expected_route_test_status,
        "maximum_observed_speed_mps": metrics.get("maximum_observed_speed_mps"),
        "maximum_sustained_speed_duration_sec": metrics.get(
            "maximum_sustained_speed_duration_sec"
        ),
        "maximum_absolute_cte_m": metrics.get("maximum_absolute_cte_m"),
        "maximum_lateral_acceleration_mps2": metrics.get(
            "maximum_lateral_acceleration_mps2"
        ),
        "traveled_distance_m": metrics.get("traveled_distance_m"),
        "sim_elapsed_sec": sim_elapsed,
        "wall_elapsed_sec": wall_elapsed,
        "real_time_factor": real_time_factor,
        "actuation_map_status": coverage.get("status"),
        "actuation_target_envelope_classification": coverage.get(
            "target_envelope_classification"
        ),
        "actuation_runtime_lookup_classification": runtime_lookup.get(
            "classification"
        ) if isinstance(runtime_lookup, dict) else None,
        "actuation_velocity_axis_clamping_observed": runtime_lookup.get(
            "velocity_axis_clamping_observed"
        ) if isinstance(runtime_lookup, dict) else None,
        "camera_bundle_coverage_percent": camera.get("bundle_coverage_percent"),
        "maximum_camera_stamp_gap_sec": camera.get(
            "maximum_camera_stamp_gap_sec"
        ),
        "candidate_front_acceptance_percent": camera.get(
            "candidate_front_acceptance_percent"
        ),
        "vad_published_count": (
            camera.get("vad_inference", {}).get("published_count")
            if isinstance(camera.get("vad_inference"), dict)
            else None
        ),
        "longitudinal_gate_positive_limit_time_percent": (
            longitudinal.get("saturation_and_duty", {})
            .get("gated_positive_acceleration_limit", {})
            .get("time_fraction_percent")
        ),
        "longitudinal_throttle_near_saturation_time_percent": (
            longitudinal.get("saturation_and_duty", {})
            .get("accel_command_near_saturation", {})
            .get("time_fraction_percent")
        ),
        "runtime_pattern": (
            runtime_vad.get("runtime_pattern")
            if isinstance(runtime_vad, dict)
            else None
        ),
        "runtime_load_classification": runtime_classification,
        "runtime_load_status": runtime_load.get("status"),
        "runtime_load_classification_supported": (
            runtime_support.get("classification_supported")
            if isinstance(runtime_support, dict)
            else None
        ),
    },
}
target = root / "pilot_run.json"
descriptor, staged_name = tempfile.mkstemp(
    prefix=f".{target.name}.", suffix=".staged", dir=target.parent
)
staged = Path(staged_name)
try:
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(staged, target)
finally:
    staged.unlink(missing_ok=True)
if problems:
    raise SystemExit("; ".join(problems))
PY

if (( trial_status != 0 || cleanup_status != 0 || coverage_status != 0 ||
      telemetry_cleanup_status != 0 || camera_integrity_status != 0 ||
      runtime_load_status != 0 || acceptance_gate_status != 0 ||
      handoff_status != 0 )); then
  echo "60 kph pilot did not complete every gate: trial=${trial_status} cleanup=${cleanup_status} coverage=${coverage_status} telemetry=${telemetry_cleanup_status} camera=${camera_integrity_status} runtime_load=${runtime_load_status} acceptance=${acceptance_gate_status} handoff=${handoff_status}" >&2
  exit 1
fi
pilot_complete=true
trap - EXIT INT TERM HUP
cleaned=true
echo "60 kph exploratory pilot completed: ${output_root}"
