#!/usr/bin/env bash
set -euo pipefail

# A matrix launched from the VS Code Snap inherits private GTK/GIO module
# paths. Those modules link against the Snap core runtime and can make the
# host ROS rqt_image_view fail with GLIBC_PRIVATE symbol errors. Sanitize only
# the Code-specific paths; keep GTK_IM_MODULE/QT_IM_MODULE so host IBus input
# continues to work.
vscode_snap_gui_env_sanitized=false
case "${GTK_PATH:-}:${GIO_MODULE_DIR:-}:${SNAP_NAME:-}" in
  *snap/code*|*:code)
    vscode_snap_gui_env_sanitized=true
    if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
      export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"
    else
      unset XDG_DATA_DIRS
    fi
    unset GIO_LAUNCHED_DESKTOP_FILE GIO_LAUNCHED_DESKTOP_FILE_PID GIO_MODULE_DIR
    unset GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_PATH XDG_DATA_HOME
    unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_EUID
    unset SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET SNAP_LIBRARY_PATH SNAP_NAME
    unset SNAP_REAL_HOME SNAP_REVISION SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA
    unset SNAP_VERSION
    ;;
esac

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh

usage() {
  cat >&2 <<'EOF'
Usage: run_recorded_route_trial.sh [options] OUTPUT_DIR ROUTE_JSON [launch arguments...]

Options:
  --recommended          Run the repeat-screened full-stack parameter profile
  --speed-30kph          Add the guarded 8.333 m/s screening profile
  --visualize            Start RViz (and the front-camera view outside capture mode)
  --capture-desktop      Record 1920x1080 owned-RViz PNG/GIF evidence after a VAD candidate
  --tight-corridor       Screen the recommended profile with a +/-0.20 m corridor
  --trajectory-stability Add the repeat-screened experimental HOLD candidate
  --smart-mpc            Run the nominal Smart MPC controller instead of standard MPC
  --fp16-heads           Run mixed FP16 VAD heads with LayerNorm kept in FP32
  --model-override YAML  Apply an editable VAD deployment overlay and preserve it in the artifact
  --sensor-mapping YAML  Apply a fast sensor mapping and preserve it in the artifact
  --mpc-input-delay SEC  Label the turn analysis with the applied MPC delay
  --mpc-steer-tau SEC    Label the turn analysis with the applied steering tau
  --ready-timeout SEC    Maximum wall time for the VAD route to become ready (default: 180)

CARLA must already be running at CARLA_HOST/CARLA_PORT. The helper owns only the
Autoware stack and recorder processes that it starts.
EOF
}

mpc_input_delay=""
mpc_steer_tau=""
ready_timeout=180
smart_mpc=false
fp16_heads=false
recommended=false
speed_30kph=false
visualize=false
capture_desktop=false
trajectory_stability=false
tight_corridor=false
comfortable_deceleration_mps2=""
maximum_longitudinal_acceleration_mps2=""
maximum_lateral_acceleration_mps2=""
target_speed_mps=""
minimum_sustained_speed_mps=""
minimum_sustained_speed_sec=""
maximum_observed_speed_mps=""
maximum_lateral_acceleration_limit_mps2=""
maneuver_lookahead_m=""
speed_profile_id="baseline"
speed_exposure_mode="not_requested"
model_override=""
sensor_mapping=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --recommended)
      recommended=true
      shift
      ;;
    --speed-30kph)
      speed_30kph=true
      recommended=true
      shift
      ;;
    --visualize)
      visualize=true
      shift
      ;;
    --capture-desktop)
      capture_desktop=true
      shift
      ;;
    --trajectory-stability)
      trajectory_stability=true
      recommended=true
      shift
      ;;
    --tight-corridor)
      tight_corridor=true
      shift
      ;;
    --smart-mpc)
      smart_mpc=true
      shift
      ;;
    --fp16-heads)
      fp16_heads=true
      shift
      ;;
    --model-override)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      model_override="$2"
      shift 2
      ;;
    --sensor-mapping)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      sensor_mapping="$2"
      shift 2
      ;;
    --mpc-input-delay)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      mpc_input_delay="$2"
      shift 2
      ;;
    --mpc-steer-tau)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      mpc_steer_tau="$2"
      shift 2
      ;;
    --ready-timeout)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      ready_timeout="$2"
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

if [[ "${recommended}" == "true" ]]; then
  if [[ "${smart_mpc}" == "true" || "${fp16_heads}" == "true" || \
        -n "${model_override}" || -n "${sensor_mapping}" ]]; then
    echo "--recommended cannot be combined with experimental controller, precision, model, or sensor options." >&2
    exit 2
  fi
fi
if [[ "${speed_30kph}" == "true" && \
      ( "${tight_corridor}" == "true" || "${trajectory_stability}" == "true" ) ]]; then
  echo "--speed-30kph must be screened independently of experimental corridor/filter modes." >&2
  exit 2
fi
if [[ "${tight_corridor}" == "true" && "${recommended}" != "true" ]]; then
  echo "--tight-corridor requires --recommended." >&2
  exit 2
fi
if [[ "${capture_desktop}" == "true" && "${visualize}" != "true" ]]; then
  echo "--capture-desktop requires --visualize." >&2
  exit 2
fi
if [[ "${smart_mpc}" == "true" && "${fp16_heads}" == "true" ]]; then
  echo "--smart-mpc and --fp16-heads must be screened separately." >&2
  exit 2
fi
if [[ "${fp16_heads}" == "true" && -n "${model_override}" ]]; then
  echo "--fp16-heads and --model-override are mutually exclusive." >&2
  exit 2
fi
if [[ "${smart_mpc}" == "true" && -n "${model_override}" ]]; then
  echo "--smart-mpc and --model-override must be screened separately." >&2
  exit 2
fi

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi

output_dir="$(realpath -m -- "$1")"
route_file="$2"
shift 2
launch_arguments=("$@")

if [[ "${recommended}" == "true" ]]; then
  mpc_input_delay="${mpc_input_delay:-0.12}"
  mpc_steer_tau="${mpc_steer_tau:-0.15}"
  maneuver_lookahead_m="3.0"
  speed_profile_id="recommended_9kph_v1"
  if [[ "${speed_30kph}" == "true" ]]; then
    maneuver_lookahead_m="4.0"
    speed_profile_id="carla_vad_30kph_v2"
    comfortable_deceleration_mps2="2.0"
    maximum_longitudinal_acceleration_mps2="1.5"
    maximum_lateral_acceleration_mps2="1.2"
    target_speed_mps="8.333333333333334"
    maximum_observed_speed_mps="9.0"
    maximum_lateral_acceleration_limit_mps2="1.8"
  else
    comfortable_deceleration_mps2="0.60"
  fi
fi

for argument in "${launch_arguments[@]}"; do
  case "${argument}" in
    vad_model_override_file:=*|sensor_mapping_file:=*)
      echo "Use the protected wrapper option instead of the ${argument%%:=*} launch argument." >&2
      exit 2
      ;;
    launch_conventional_perception:=true|launch_conventional_perception:=True|launch_conventional_perception:=TRUE)
      echo "Fast sensor mapping cannot run conventional perception." >&2
      exit 2
      ;;
  esac
  if [[ "${capture_desktop}" == "true" && "${argument}" == rviz_config:=* ]]; then
    echo "--capture-desktop fixes rviz_config to the centered evidence view." >&2
    exit 2
  fi
  if [[ "${recommended}" == "true" ]]; then
    # Keep this list aligned with run_route_vad_fast.sh's recommended profile.
    case "${argument}" in
      use_vad_imu_acceleration:=*|use_fast_vad:=*|vad_use_fp16_heads:=*|use_light_weight_sensor_mapping:=*|rviz:=*|launch_fast_camera_view:=*|use_lateral_controller_param_override:=*|lateral_controller_param_path:=*|use_longitudinal_controller_param_override:=*|longitudinal_controller_param_path:=*|vehicle_cmd_gate_param_path:=*|controller_stop_offset_m:=*|comfortable_deceleration_mps2:=*|maximum_longitudinal_acceleration_mps2:=*|longitudinal_velocity_source:=*|nominal_cruise_speed_mps:=*|maneuver_lookahead_m:=*|maneuver_exit_lookahead_m:=*|turn_inward_corridor_half_width_m:=*|turn_outward_corridor_half_width_m:=*|left_turn_outward_corridor_half_width_m:=*|right_turn_outward_corridor_half_width_m:=*|route_corridor_entry_distance_m:=*|trajectory_lateral_filter_gain:=*|left_turn_trajectory_lateral_filter_gain:=*|right_turn_trajectory_lateral_filter_gain:=*|trajectory_lateral_filter_activation_threshold_m:=*|trajectory_geometry_smoothing_strength:=*|maximum_lateral_acceleration_mps2:=*|curvature_speed_preview_m:=*|route_curvature_lookahead_m:=*|max_route_deviation_m:=*|max_candidate_age_sec:=*|candidate_timeout_sec:=*|maximum_speed_mps:=*|raw_vehicle_cmd_converter_config:=*)
        echo "Recommended profile argument is controlled by this wrapper: ${argument%%:=*}" >&2
        exit 2
        ;;
    esac
  fi
done

python3 - "${mpc_input_delay:-0.1}" "${mpc_steer_tau:-0.1}" "${recommended}" <<'PY'
import math
import sys

for label, raw in zip(("MPC delay", "steering tau"), sys.argv[1:3]):
    value = float(raw)
    if not math.isfinite(value) or value <= 0.0:
        raise SystemExit(f"{label} must be positive and finite")

if sys.argv[3] == "true":
    for label, raw, expected in (
        ("MPC delay", sys.argv[1], 0.12),
        ("steering tau", sys.argv[2], 0.15),
    ):
        if not math.isclose(float(raw), expected, abs_tol=1.0e-9):
            raise SystemExit(
                f"--recommended fixes {label} at {expected:.2f} s; got {float(raw):g}"
            )
PY

if [[ ! -f "${route_file}" ]]; then
  echo "Route file not found: ${route_file}" >&2
  exit 2
fi
route_file="$(realpath -- "${route_file}")"
route_scenario="$(
  python3 - "${route_file}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    value = json.load(stream).get("scenario")
print(value if isinstance(value, str) and value else "unknown")
PY
)"
if [[ "${speed_30kph}" == "true" ]]; then
  case "${route_scenario}" in
    straight)
      speed_exposure_mode="straight_target_required"
      minimum_sustained_speed_mps="7.5"
      minimum_sustained_speed_sec="1.0"
      ;;
    left|right)
      speed_exposure_mode="curvature_limited_turn"
      minimum_sustained_speed_mps="0.0"
      minimum_sustained_speed_sec="0.0"
      ;;
    *)
      echo "--speed-30kph requires a straight, left, or right route; got ${route_scenario}" >&2
      exit 2
      ;;
  esac
fi

recommended_mpc=""
speed_30kph_gate=""
speed_30kph_pid=""
if [[ "${recommended}" == "true" ]]; then
  package_share="$(ros2 pkg prefix autoware_e2e_vad_launch)/share/autoware_e2e_vad_launch"
  model_override="${package_share}/config/vad_carla_tiny_recommended.param.yaml"
  sensor_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable_imu.yaml"
  recommended_mpc="${package_share}/config/mpc_carla_recommended.param.yaml"
  required_profile_files=("${model_override}" "${sensor_mapping}" "${recommended_mpc}")
  if [[ "${speed_30kph}" == "true" ]]; then
    speed_30kph_gate="${package_share}/config/vehicle_cmd_gate_carla_30kph.param.yaml"
    speed_30kph_pid="${package_share}/config/pid_carla_vad_30kph.param.yaml"
    required_profile_files+=(
      "${speed_30kph_gate}"
      "${speed_30kph_gate}.metadata.json"
      "${speed_30kph_pid}"
      "${speed_30kph_pid}.metadata.json"
    )
  fi
  for required in "${required_profile_files[@]}"; do
    if [[ ! -f "${required}" ]]; then
      echo "Recommended profile is not installed: ${required}" >&2
      exit 1
    fi
  done
fi

if [[ -n "${model_override}" ]]; then
  if [[ ! -f "${model_override}" ]]; then
    echo "Model override file not found: ${model_override}" >&2
    exit 2
  fi
  model_override="$(realpath -- "${model_override}")"
fi

if [[ -n "${sensor_mapping}" ]]; then
  if [[ ! -f "${sensor_mapping}" ]]; then
    echo "Sensor mapping file not found: ${sensor_mapping}" >&2
    exit 2
  fi
  sensor_mapping="$(realpath -- "${sensor_mapping}")"
fi

raw_vehicle_cmd_converter_config="$(
  ros2 pkg prefix autoware_carla_interface
)/share/autoware_carla_interface/config/raw_vehicle_cmd_converter.param.yaml"
for argument in "${launch_arguments[@]}"; do
  case "${argument}" in
    raw_vehicle_cmd_converter_config:=*)
      raw_vehicle_cmd_converter_config="${argument#*:=}"
      ;;
  esac
done

if [[ -e "${output_dir}" || -L "${output_dir}" ]]; then
  if [[ -z "${AUTOWARE_E2E_CARLA_GENERATION_ID:-}" ]]; then
    echo "Output directory already exists: ${output_dir}" >&2
    exit 2
  fi
  # The matrix creates the attempt directory only so the owned CARLA process
  # can stream its generation log before this helper starts.  Admit exactly
  # that one regular file; any stale or unrelated artifact still fails closed.
  if ! python3 - "${output_dir}" "${AUTOWARE_E2E_CARLA_SERVER_LOG:-}" <<'PY'
from pathlib import Path
import sys

output = Path(sys.argv[1])
declared_log = Path(sys.argv[2]).expanduser() if sys.argv[2] else None
if output.is_symlink() or not output.is_dir():
    raise SystemExit("matrix attempt output is not a regular directory")
expected_log = output.resolve() / "carla_server.log"
if (
    declared_log is None
    or declared_log.is_symlink()
    or declared_log.resolve() != expected_log
):
    raise SystemExit("matrix CARLA server log is not bound to the attempt directory")
children = list(output.iterdir())
if children != [output / "carla_server.log"]:
    raise SystemExit(
        "matrix attempt directory contains artifacts other than carla_server.log"
    )
if expected_log.is_symlink() or not expected_log.is_file():
    raise SystemExit("matrix CARLA server log is missing or not a regular file")
PY
  then
    echo "Pre-created matrix output directory violates the CARLA lifecycle contract: ${output_dir}" >&2
    exit 2
  fi
fi
if [[ ! -f "${raw_vehicle_cmd_converter_config}" ]]; then
  echo "Raw vehicle command converter config not found: ${raw_vehicle_cmd_converter_config}" >&2
  exit 2
fi
raw_vehicle_cmd_converter_config="$(realpath -- "${raw_vehicle_cmd_converter_config}")"
if [[ ! "${ready_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ready timeout must be a positive integer" >&2
  exit 2
fi
desktop_dimensions=""
desktop_display=""
capture_output_width_px=1920
capture_output_height_px=1080
capture_output_dimensions="${capture_output_width_px}x${capture_output_height_px}"
capture_rviz_config=""
capture_rviz_config_sha256=""
if [[ "${capture_desktop}" == "true" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "--capture-desktop requires DISPLAY" >&2
    exit 2
  fi
  for command in ffmpeg ffprobe xdpyinfo xprop xwininfo; do
    if ! command -v "${command}" >/dev/null 2>&1; then
      echo "--capture-desktop requires ${command}" >&2
      exit 2
    fi
  done
  # Read xdpyinfo to EOF.  With `set -o pipefail`, exiting awk after the first
  # match can make xdpyinfo receive SIGPIPE and intermittently return 141.
  # That used to abort a second matrix trial before its output directory was
  # created even though the display itself was healthy.
  desktop_dimensions="$(
    xdpyinfo -display "${DISPLAY}" 2>/dev/null |
      awk '/dimensions:/ && !found {dimensions=$2; found=1} END {print dimensions}'
  )"
  if [[ ! "${desktop_dimensions}" =~ ^[1-9][0-9]*x[1-9][0-9]*$ ]]; then
    echo "Could not determine the full DISPLAY dimensions for ${DISPLAY}" >&2
    exit 2
  fi
  desktop_display="${DISPLAY}"
  if [[ ! "${desktop_display}" =~ \.[0-9]+$ ]]; then
    desktop_display="${desktop_display}.0"
  fi
  capture_rviz_config="${root}/autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz"
  capture_rviz_config_sha256="$(
    sha256sum -- "${capture_rviz_config}" | awk '{print $1}'
  )"
  python3 - "${capture_rviz_config}" <<'PY'
import math
from pathlib import Path
import sys

import yaml

config_path = Path(sys.argv[1])
with config_path.open(encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
view = config["Visualization Manager"]["Views"]["Current"]
expected = {
    "Class": "rviz_default_plugins/TopDownOrtho",
    "Target Frame": "base_link",
    "Angle": 0.0,
    "X": 0.0,
    "Y": 0.0,
    "Scale": 10.0,
}
for key, expected_value in expected.items():
    actual = view.get(key)
    if isinstance(expected_value, float):
        if not isinstance(actual, (int, float)) or not math.isclose(
            float(actual), expected_value, abs_tol=1.0e-9
        ):
            raise SystemExit(
                f"RViz capture view {key} must be {expected_value:g}, got {actual!r}"
            )
    elif actual != expected_value:
        raise SystemExit(
            f"RViz capture view {key} must be {expected_value!r}, got {actual!r}"
        )

required_topics = {
    "/planning/vad_route/reference_path",
    "/planning/vad_route/actual_path",
    "/planning/trajectory",
    "/planning/vad_route/selected_raw_trajectory",
    "/planning/vad/candidate_trajectories",
}
visible_topics = set()

def visit(value):
    if isinstance(value, dict):
        topic = value.get("Topic")
        topic_name = topic.get("Value") if isinstance(topic, dict) else topic
        if (
            topic_name in required_topics
            and value.get("Enabled") is True
            and value.get("Value") is True
        ):
            visible_topics.add(topic_name)
        for child in value.values():
            visit(child)
    elif isinstance(value, list):
        for child in value:
            visit(child)

visit(config["Visualization Manager"]["Displays"])
missing = sorted(required_topics - visible_topics)
if missing:
    raise SystemExit(f"RViz capture view hides required path topics: {missing}")

def named_display(value, name):
    if isinstance(value, dict):
        if value.get("Name") == name:
            return value
        for child in value.values():
            found = named_display(child, name)
            if found is not None:
                return found
    elif isinstance(value, list):
        for child in value:
            found = named_display(child, name)
            if found is not None:
                return found
    return None

odometry = named_display(config["Visualization Manager"]["Displays"], "Kinematic State")
candidates = named_display(
    config["Visualization Manager"]["Displays"], "VAD Candidate Trajectories"
)
covariance = odometry.get("Covariance", {}) if isinstance(odometry, dict) else {}
clarity = {
    "odometry_keep": odometry.get("Keep") if isinstance(odometry, dict) else None,
    "odometry_covariance": covariance.get("Value"),
    "odometry_orientation": covariance.get("Orientation", {}).get("Value"),
    "odometry_position": covariance.get("Position", {}).get("Value"),
    "candidate_path_alpha": candidates.get("View Path", {}).get("Alpha")
    if isinstance(candidates, dict)
    else None,
    "candidate_path_width": candidates.get("View Path", {}).get("Width")
    if isinstance(candidates, dict)
    else None,
}
expected_clarity = {
    "odometry_keep": 1,
    "odometry_covariance": False,
    "odometry_orientation": False,
    "odometry_position": False,
    "candidate_path_alpha": 0.22,
    "candidate_path_width": 0.04,
}
if clarity != expected_clarity:
    raise SystemExit(f"RViz capture visual-clarity contract changed: {clarity!r}")
PY
fi
carla_host="${CARLA_HOST:-localhost}"
carla_port="${CARLA_PORT:-2100}"
source_route_file="${route_file}"
route_town="$(
  python3 - "${source_route_file}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    print(json.load(stream)["town"])
PY
)"
mkdir -p "${output_dir}"
carla_generation_id="${AUTOWARE_E2E_CARLA_GENERATION_ID:-standalone_${route_town}}"
carla_expected_map="${AUTOWARE_E2E_CARLA_EXPECTED_MAP:-${route_town}}"
carla_owner_pid="${AUTOWARE_E2E_CARLA_OWNER_PID:-}"
carla_owner_pgid="${AUTOWARE_E2E_CARLA_OWNER_PGID:-}"
carla_server_log="${AUTOWARE_E2E_CARLA_SERVER_LOG:-}"
matrix_owned_carla=false
if [[ -n "${AUTOWARE_E2E_CARLA_GENERATION_ID:-}" ]]; then
  matrix_owned_carla=true
  if [[ -z "${AUTOWARE_E2E_CARLA_EXPECTED_MAP:-}" ||
        ! "${carla_owner_pid}" =~ ^[1-9][0-9]*$ ||
        ! "${carla_owner_pgid}" =~ ^[1-9][0-9]*$ ||
        -z "${carla_server_log}" ]]; then
    echo "Matrix-owned CARLA lifecycle variables are incomplete" >&2
    exit 2
  fi
fi
carla_probe_args=(
  --host "${carla_host}" --port "${carla_port}" --timeout 3
  --expected-map "${carla_expected_map}"
  --generation-id "${carla_generation_id}"
)
if [[ -n "${carla_owner_pid}" ]]; then
  carla_probe_args+=(
    --owner-pid "${carla_owner_pid}" --owner-pgid "${carla_owner_pgid}"
  )
fi
if [[ -n "${carla_server_log}" ]]; then
  carla_probe_args+=(--server-log "${carla_server_log}")
fi
if ! python3 scripts/e2e/probe_carla_server.py \
  "${carla_probe_args[@]}" --stage trial_preflight \
  --output "${output_dir}/carla_preflight_health.json"; then
  echo "CARLA failed fresh read-only RPC/map/snapshot preflight" >&2
  exit 1
fi

conflicts="$(ros2 node list --no-daemon 2>/dev/null | grep -E '/(vad_route_manager|autoware_carla_interface|vad_carla_tiny)$' || true)"
if [[ -n "${conflicts}" ]]; then
  echo "An existing project stack is visible in ROS domain ${ROS_DOMAIN_ID}:" >&2
  echo "${conflicts}" >&2
  exit 1
fi

capture_rviz_runtime_config=""
if [[ "${capture_desktop}" == "true" ]]; then
  mkdir -p "${output_dir}/rviz_capture_provenance"
  capture_rviz_runtime_config="${output_dir}/rviz_capture_provenance/autoware_vad_carla.rviz"
  cp -- "${capture_rviz_config}" "${capture_rviz_runtime_config}"
  printf '%s  %s\n' "${capture_rviz_config_sha256}" "autoware_vad_carla.rviz" > \
    "${output_dir}/rviz_capture_provenance/SHA256SUMS"
fi

# Keep the stack, evaluator, bag analysis, and renderers in one map frame. The
# CARLA spawn string remains raw inside the aligned route by contract.
full_map_path="${AUTOWARE_E2E_FULL_MAP_PATH:-${root}/data/maps/${route_town}_full}"
map_bundle="${full_map_path}/map_bundle.json"
if [[ -f "${map_bundle}" ]]; then
  cp -- "${source_route_file}" "${output_dir}/source_route.json"
  cp -- "${map_bundle}" "${output_dir}/map_bundle.json"
  python3 scripts/e2e/align_carla_route_to_map.py \
    "${source_route_file}" "${map_bundle}" \
    --output "${output_dir}/aligned_route.json" --json > \
    "${output_dir}/route_alignment.json"
  route_file="${output_dir}/aligned_route.json"
fi

python3 scripts/e2e/capture_raw_vehicle_cmd_converter_provenance.py \
  --config "${raw_vehicle_cmd_converter_config}" \
  --output-dir "${output_dir}/actuation_config_provenance"
printf '%s\n' "${launch_arguments[@]}" > "${output_dir}/launch_args.txt"
printf 'ROS_DOMAIN_ID=%s\nCARLA_HOST=%s\nCARLA_PORT=%s\n' \
  "${ROS_DOMAIN_ID}" "${carla_host}" "${carla_port}" > "${output_dir}/runtime.env"
printf 'CARLA_LIFECYCLE=cold_start_owned_process_group_per_trial\nCARLA_GENERATION_ID=%s\nCARLA_EXPECTED_MAP=%s\nCARLA_OWNER_PID=%s\nCARLA_OWNER_PGID=%s\nCARLA_SERVER_LOG=%s\nCARLA_MATRIX_OWNED=%s\n' \
  "${carla_generation_id}" "${carla_expected_map}" "${carla_owner_pid}" \
  "${carla_owner_pgid}" "${carla_server_log}" "${matrix_owned_carla}" >> \
  "${output_dir}/runtime.env"
printf 'SOURCE_ROUTE_FILE=%s\nEFFECTIVE_ROUTE_FILE=%s\nFULL_MAP_PATH=%s\n' \
  "${source_route_file}" "${route_file}" "${full_map_path}" >> "${output_dir}/runtime.env"
printf 'RECOMMENDED=%s\nVISUALIZE=%s\nCAPTURE_DESKTOP=%s\nTIGHT_CORRIDOR_CANDIDATE=%s\nTRAJECTORY_STABILITY_CANDIDATE=%s\nSMART_MPC=%s\nFP16_HEADS=%s\n' \
  "${recommended}" "${visualize}" "${capture_desktop}" "${tight_corridor}" "${trajectory_stability}" "${smart_mpc}" "${fp16_heads}" >> \
  "${output_dir}/runtime.env"
if [[ "${capture_desktop}" == "true" ]]; then
  printf 'RVIZ_CAPTURE_CAMERA_SOURCE=rviz_embedded_vad_front_camera\nRVIZ_CAPTURE_EXTERNAL_CAMERA_VIEW=false\nRVIZ_CAPTURE_SOURCE=ffmpeg_x11grab_owned_window_v1\nRVIZ_CAPTURE_ROOT=false\nRVIZ_CAPTURE_SHELL_SURFACES_EXCLUDED=true\nRVIZ_CAPTURE_SCALE_APPLIED=false\nRVIZ_CAPTURE_OCCLUSION_GUARD=owned_rviz_window_only_v1\nRVIZ_CAPTURE_OUTPUT_WIDTH_PX=%s\nRVIZ_CAPTURE_OUTPUT_HEIGHT_PX=%s\n' \
    "${capture_output_width_px}" "${capture_output_height_px}" >> \
    "${output_dir}/runtime.env"
fi
printf 'VSCODE_SNAP_GUI_ENV_SANITIZED=%s\n' \
  "${vscode_snap_gui_env_sanitized}" >> "${output_dir}/runtime.env"
printf 'SPEED_30KPH=%s\nSPEED_PROFILE_ID=%s\nROUTE_SCENARIO=%s\nSPEED_EXPOSURE_MODE=%s\n' \
  "${speed_30kph}" "${speed_profile_id}" "${route_scenario}" \
  "${speed_exposure_mode}" >> "${output_dir}/runtime.env"
if [[ -n "${maneuver_lookahead_m}" ]]; then
  printf 'MANEUVER_LOOKAHEAD_M=%s\nVAD_IMU_ACCELERATION_ENABLED=true\n' \
    "${maneuver_lookahead_m}" >> "${output_dir}/runtime.env"
fi
if [[ "${speed_30kph}" == "true" ]]; then
  printf 'TARGET_SPEED_MPS=%s\nTARGET_SPEED_KPH=30.0\nMINIMUM_SUSTAINED_SPEED_MPS=%s\nMINIMUM_SUSTAINED_SPEED_SEC=%s\nMAXIMUM_OBSERVED_SPEED_MPS=%s\nMAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2=%s\nMAXIMUM_LONGITUDINAL_ACCELERATION_MPS2=%s\nMAXIMUM_LATERAL_ACCELERATION_MPS2=%s\nMAXIMUM_SPEED_SAMPLE_GAP_SEC=0.25\nCONTROLLER_STOP_OFFSET_M=0.60\nMANEUVER_EXIT_LOOKAHEAD_M=2.5\nCURVATURE_SPEED_PREVIEW_M=3.0\nROUTE_CURVATURE_LOOKAHEAD_M=20.0\nMAX_ROUTE_DEVIATION_M=1.0\nMAX_CANDIDATE_AGE_SEC=0.5\nCANDIDATE_TIMEOUT_SEC=1.5\nLONGITUDINAL_SPEED_SOURCE=explicit_simulation_profile\nLONGITUDINAL_ACCELERATION_ROLE=trajectory_internal_curve_exit_cap\nLONGITUDINAL_PID_MAX_OUT_MPS2=1.5\nLONGITUDINAL_PID_MAX_P_EFFORT_MPS2=1.5\nCOMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2=1.5\nVAD_CRUISE_VELOCITY_EVALUATED=false\nVAD_HARD_STOP_SENTINEL_PRESERVED=true\nVAD_VELOCITY_EVALUATED=false\nVAD_GEOMETRY_EVALUATED=true\nVAD_GEOMETRY_SOURCE=true\nSPEED_LIMIT_SOURCE=explicit_simulation_profile\nREAL_VEHICLE_READY=false\n' \
    "${target_speed_mps}" "${minimum_sustained_speed_mps}" \
    "${minimum_sustained_speed_sec}" "${maximum_observed_speed_mps}" \
    "${maximum_lateral_acceleration_limit_mps2}" \
    "${maximum_longitudinal_acceleration_mps2}" \
    "${maximum_lateral_acceleration_mps2}" >> "${output_dir}/runtime.env"
fi
if [[ "${capture_desktop}" == "true" ]]; then
  printf 'RVIZ_CAPTURE_CONFIG=%s\nRVIZ_CAPTURE_CONFIG_SHA256=%s\n' \
    "${capture_rviz_runtime_config}" "${capture_rviz_config_sha256}" >> \
    "${output_dir}/runtime.env"
fi
if [[ -n "${comfortable_deceleration_mps2}" ]]; then
  printf 'COMFORTABLE_DECELERATION_MPS2=%s\n' \
    "${comfortable_deceleration_mps2}" >> "${output_dir}/runtime.env"
fi
validation_state="experimental"
if [[ "${speed_30kph}" == "true" ]]; then
  validation_state="carla_30kph_v2_screening"
elif [[ "${tight_corridor}" == "true" && "${trajectory_stability}" == "true" ]]; then
  validation_state="combined_tight_corridor_and_trajectory_stability_experimental"
elif [[ "${tight_corridor}" == "true" ]]; then
  validation_state="tight_corridor_experimental"
elif [[ "${trajectory_stability}" == "true" ]]; then
  validation_state="right_turn_repeat_screened_hold"
elif [[ "${recommended}" == "true" ]]; then
  validation_state="repeat_screened_current_logic"
fi
printf 'CLOSED_LOOP_VALIDATION_STATE=%s\n' "${validation_state}" >> \
  "${output_dir}/runtime.env"
trajectory_logic_file="${root}/autoware_e2e_vad_launch/scripts/vad_route_logic.py"
route_manager_file="${root}/autoware_e2e_vad_launch/scripts/vad_route_manager.py"
trajectory_logic_sha256="$(sha256sum -- "${trajectory_logic_file}" | awk '{print $1}')"
route_manager_sha256="$(sha256sum -- "${route_manager_file}" | awk '{print $1}')"
printf 'TRAJECTORY_LOGIC_SHA256=%s\nVAD_ROUTE_MANAGER_SHA256=%s\n' \
  "${trajectory_logic_sha256}" "${route_manager_sha256}" >> "${output_dir}/runtime.env"
mkdir -p "${output_dir}/trajectory_code_provenance"
cp -- "${trajectory_logic_file}" "${output_dir}/trajectory_code_provenance/vad_route_logic.py"
cp -- "${route_manager_file}" "${output_dir}/trajectory_code_provenance/vad_route_manager.py"
printf '%s  %s\n%s  %s\n' \
  "${trajectory_logic_sha256}" "vad_route_logic.py" \
  "${route_manager_sha256}" "vad_route_manager.py" > \
  "${output_dir}/trajectory_code_provenance/SHA256SUMS"
if [[ -n "${model_override}" ]]; then
  model_override_sha256="$(sha256sum -- "${model_override}" | awk '{print $1}')"
  printf 'VAD_MODEL_OVERRIDE_FILE=%s\nVAD_MODEL_OVERRIDE_SHA256=%s\n' \
    "${model_override}" "${model_override_sha256}" >> "${output_dir}/runtime.env"
  mkdir -p "${output_dir}/vad_model_override_provenance"
  cp -- "${model_override}" \
    "${output_dir}/vad_model_override_provenance/model_override.param.yaml"
  printf '%s  %s\n' "${model_override_sha256}" "model_override.param.yaml" > \
    "${output_dir}/vad_model_override_provenance/SHA256SUMS"
fi
if [[ -n "${sensor_mapping}" ]]; then
  sensor_mapping_sha256="$(sha256sum -- "${sensor_mapping}" | awk '{print $1}')"
  printf 'SENSOR_MAPPING_FILE=%s\nSENSOR_MAPPING_SHA256=%s\n' \
    "${sensor_mapping}" "${sensor_mapping_sha256}" >> "${output_dir}/runtime.env"
  mkdir -p "${output_dir}/sensor_mapping_provenance"
  cp -- "${sensor_mapping}" \
    "${output_dir}/sensor_mapping_provenance/sensor_mapping.yaml"
  printf '%s  %s\n' "${sensor_mapping_sha256}" "sensor_mapping.yaml" > \
    "${output_dir}/sensor_mapping_provenance/SHA256SUMS"
fi
if [[ -n "${recommended_mpc}" ]]; then
  recommended_mpc_sha256="$(sha256sum -- "${recommended_mpc}" | awk '{print $1}')"
  printf 'MPC_PARAM_FILE=%s\nMPC_PARAM_SHA256=%s\nMPC_INPUT_DELAY_SEC=%s\nMPC_STEER_TAU_SEC=%s\n' \
    "${recommended_mpc}" "${recommended_mpc_sha256}" \
    "${mpc_input_delay}" "${mpc_steer_tau}" >> "${output_dir}/runtime.env"
  cp -- "${recommended_mpc}" "${output_dir}/mpc.param.yaml"
  if [[ -f "${recommended_mpc}.metadata.json" ]]; then
    cp -- "${recommended_mpc}.metadata.json" \
      "${output_dir}/mpc.param.yaml.metadata.json"
  fi
  printf '%s  %s\n' "${recommended_mpc_sha256}" "mpc.param.yaml" > \
    "${output_dir}/MPC_SHA256SUMS"
fi
if [[ -n "${speed_30kph_gate}" ]]; then
  speed_gate_sha256="$(sha256sum -- "${speed_30kph_gate}" | awk '{print $1}')"
  speed_gate_metadata="${speed_30kph_gate}.metadata.json"
  speed_gate_metadata_sha256="$(
    sha256sum -- "${speed_gate_metadata}" | awk '{print $1}'
  )"
  printf 'VEHICLE_CMD_GATE_PARAM_FILE=%s\nVEHICLE_CMD_GATE_PARAM_SHA256=%s\nVEHICLE_CMD_GATE_METADATA_SHA256=%s\n' \
    "${speed_30kph_gate}" "${speed_gate_sha256}" \
    "${speed_gate_metadata_sha256}" >> "${output_dir}/runtime.env"
  mkdir -p "${output_dir}/speed_profile_provenance"
  cp -- "${speed_30kph_gate}" \
    "${output_dir}/speed_profile_provenance/vehicle_cmd_gate.param.yaml"
  cp -- "${speed_gate_metadata}" \
    "${output_dir}/speed_profile_provenance/vehicle_cmd_gate.param.yaml.metadata.json"
  printf '%s  %s\n%s  %s\n' \
    "${speed_gate_sha256}" "vehicle_cmd_gate.param.yaml" \
    "${speed_gate_metadata_sha256}" \
    "vehicle_cmd_gate.param.yaml.metadata.json" > \
    "${output_dir}/speed_profile_provenance/SHA256SUMS"
fi
if [[ -n "${speed_30kph_pid}" ]]; then
  speed_pid_sha256="$(sha256sum -- "${speed_30kph_pid}" | awk '{print $1}')"
  speed_pid_metadata="${speed_30kph_pid}.metadata.json"
  speed_pid_metadata_sha256="$(
    sha256sum -- "${speed_pid_metadata}" | awk '{print $1}'
  )"
  printf 'LONGITUDINAL_CONTROLLER_PARAM_FILE=%s\nLONGITUDINAL_CONTROLLER_PARAM_SHA256=%s\nLONGITUDINAL_CONTROLLER_METADATA_SHA256=%s\n' \
    "${speed_30kph_pid}" "${speed_pid_sha256}" \
    "${speed_pid_metadata_sha256}" >> "${output_dir}/runtime.env"
  mkdir -p "${output_dir}/speed_profile_provenance"
  cp -- "${speed_30kph_pid}" \
    "${output_dir}/speed_profile_provenance/longitudinal_controller.param.yaml"
  cp -- "${speed_pid_metadata}" \
    "${output_dir}/speed_profile_provenance/longitudinal_controller.param.yaml.metadata.json"
  printf '%s  %s\n%s  %s\n' \
    "${speed_pid_sha256}" "longitudinal_controller.param.yaml" \
    "${speed_pid_metadata_sha256}" \
    "longitudinal_controller.param.yaml.metadata.json" >> \
    "${output_dir}/speed_profile_provenance/SHA256SUMS"
fi
printf 'RAW_VEHICLE_CMD_CONVERTER_CONFIG=%s\n' "${raw_vehicle_cmd_converter_config}" >> \
  "${output_dir}/runtime.env"
if [[ -n "${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT:-}" ]]; then
  printf 'AUTOWARE_E2E_NVIDIA_COMPAT_ROOT=%s\n' \
    "${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT}" >> "${output_dir}/runtime.env"
fi

for argument in "${launch_arguments[@]}"; do
  case "${argument}" in
    lateral_controller_param_path:=*)
      parameter_file="${argument#*:=}"
      if [[ -f "${parameter_file}" ]]; then
        cp -- "${parameter_file}" "${output_dir}/mpc.param.yaml"
        if [[ -f "${parameter_file}.metadata.json" ]]; then
          cp -- "${parameter_file}.metadata.json" \
            "${output_dir}/mpc.param.yaml.metadata.json"
        fi
      fi
      ;;
    smart_mpc_runtime_param_path:=*)
      parameter_file="${argument#*:=}"
      if [[ -f "${parameter_file}" ]]; then
        cp -- "${parameter_file}" "${output_dir}/smart_mpc_runtime.param.yaml"
      fi
      ;;
    vehicle_cmd_gate_param_path:=*)
      parameter_file="${argument#*:=}"
      if [[ -f "${parameter_file}" ]]; then
        cp -- "${parameter_file}" "${output_dir}/vehicle_cmd_gate.param.yaml"
        if [[ -f "${parameter_file}.metadata.json" ]]; then
          cp -- "${parameter_file}.metadata.json" \
            "${output_dir}/vehicle_cmd_gate.param.yaml.metadata.json"
        fi
      fi
      ;;
  esac
done

stack_pid=""
stack_pgid=""
recorder_pid=""
recorder_pgid=""
route_test_pid=""
route_test_pgid=""
desktop_pid=""
desktop_pgid=""
capture_rviz_window_id=""
capture_rviz_window_id_decimal=""
capture_rviz_window_pid=""
capture_rviz_window_pgid=""
capture_rviz_window_width_px=""
capture_rviz_window_height_px=""
capture_rviz_input_dimensions=""
capture_pad_left_px=""
capture_pad_top_px=""
capture_pad_right_px=""
capture_pad_bottom_px=""
capture_pad_filter=""
cleaned=false

carla_owner_alive() {
  if [[ "${matrix_owned_carla}" != "true" ]]; then
    return 0
  fi
  local state=""
  local actual_pgid=""
  if ! kill -0 "${carla_owner_pid}" 2>/dev/null; then
    return 1
  fi
  state="$(ps -o stat= -p "${carla_owner_pid}" 2>/dev/null | tr -d '[:space:]')"
  if [[ -z "${state}" || "${state}" == Z* ]]; then
    return 1
  fi
  actual_pgid="$(ps -o pgid= -p "${carla_owner_pid}" 2>/dev/null | tr -d '[:space:]')"
  [[ "${actual_pgid}" == "${carla_owner_pgid}" ]]
}

require_carla_owner() {
  local stage="$1"
  if carla_owner_alive; then
    return 0
  fi
  printf 'generation=%s owner_pid=%s owner_pgid=%s stage=%s\n' \
    "${carla_generation_id}" "${carla_owner_pid}" "${carla_owner_pgid}" \
    "${stage}" > "${output_dir}/carla_owner_failure.log"
  echo "Owned CARLA generation exited or became a zombie at ${stage}" >&2
  return 1
}

matching_owned_rviz_capture_windows() {
  DISPLAY="${DISPLAY}" xwininfo -root -tree 2>/dev/null |
    awk -v needle="${capture_rviz_runtime_config}" \
      'index($0, needle) && $1 ~ /^0x[0-9a-fA-F]+$/ && !seen[$1]++ {print $1}'
}

inspect_owned_rviz_capture_window() {
  local window_id="$1"
  local properties=""
  local window_info=""
  local process_state=""

  if [[ ! "${window_id}" =~ ^0x[0-9a-fA-F]+$ ]]; then
    echo "Invalid RViz capture window ID: ${window_id}" >&2
    return 1
  fi
  if ! properties="$(
    DISPLAY="${DISPLAY}" xprop -id "${window_id}" \
      _NET_WM_NAME WM_NAME WM_CLASS _NET_WM_PID 2>/dev/null
  )"; then
    echo "Could not read the owned RViz X11 properties" >&2
    return 1
  fi
  if ! grep -Fq -- "${capture_rviz_runtime_config}" <<< "${properties}"; then
    echo "RViz capture window title no longer names the pinned config" >&2
    return 1
  fi
  if ! grep -Eiq \
    '^WM_CLASS.*=[[:space:]]*"rviz2",[[:space:]]*"rviz2"[[:space:]]*$' \
    <<< "${properties}"; then
    echo "RViz capture window WM_CLASS is not rviz2" >&2
    return 1
  fi
  observed_rviz_window_pid="$(
    awk -F'= ' '/^_NET_WM_PID/ && !found {print $2; found=1}' <<< "${properties}"
  )"
  if [[ ! "${observed_rviz_window_pid}" =~ ^[1-9][0-9]*$ ]]; then
    echo "RViz capture window has no valid _NET_WM_PID" >&2
    return 1
  fi
  process_state="$(
    ps -o stat= -p "${observed_rviz_window_pid}" 2>/dev/null | tr -d '[:space:]'
  )"
  if [[ -z "${process_state}" || "${process_state}" == Z* ]]; then
    echo "RViz capture window owner is absent or a zombie" >&2
    return 1
  fi
  observed_rviz_window_pgid="$(
    ps -o pgid= -p "${observed_rviz_window_pid}" 2>/dev/null | tr -d '[:space:]'
  )"
  if [[ ! "${observed_rviz_window_pgid}" =~ ^[1-9][0-9]*$ ]]; then
    echo "RViz capture window owner has no valid process group" >&2
    return 1
  fi
  if ! window_info="$(
    DISPLAY="${DISPLAY}" xwininfo -id "${window_id}" -stats 2>/dev/null
  )"; then
    echo "Could not inspect the owned RViz X11 window" >&2
    return 1
  fi
  observed_rviz_window_map_state="$(
    awk -F: '/^[[:space:]]*Map State:/ && !found {
      value=$2; gsub(/^[[:space:]]+|[[:space:]]+$/, "", value); print value; found=1
    }' <<< "${window_info}"
  )"
  if [[ "${observed_rviz_window_map_state}" != "IsViewable" ]]; then
    echo "RViz capture window is not IsViewable" >&2
    return 1
  fi
  observed_rviz_window_width_px="$(
    awk -F: '/^[[:space:]]*Width:/ && !found {
      value=$2; gsub(/[[:space:]]/, "", value); print value; found=1
    }' <<< "${window_info}"
  )"
  observed_rviz_window_height_px="$(
    awk -F: '/^[[:space:]]*Height:/ && !found {
      value=$2; gsub(/[[:space:]]/, "", value); print value; found=1
    }' <<< "${window_info}"
  )"
  if [[ ! "${observed_rviz_window_width_px}" =~ ^[1-9][0-9]*$ ||
        ! "${observed_rviz_window_height_px}" =~ ^[1-9][0-9]*$ ]]; then
    echo "RViz capture window has invalid geometry" >&2
    return 1
  fi
}

verify_owned_rviz_capture_window() {
  local stage="$1"
  local -a matching_windows=()
  if [[ "${capture_desktop}" != "true" ]]; then
    return 0
  fi
  mapfile -t matching_windows < <(matching_owned_rviz_capture_windows)
  if (( ${#matching_windows[@]} != 1 )) ||
     [[ "${matching_windows[0]:-}" != "${capture_rviz_window_id}" ]]; then
    echo "Owned RViz X11 window identity changed during ${stage}" >&2
    return 1
  fi
  inspect_owned_rviz_capture_window "${capture_rviz_window_id}" || return 1
  if [[ "${observed_rviz_window_pid}" != "${capture_rviz_window_pid}" ||
        "${observed_rviz_window_pgid}" != "${capture_rviz_window_pgid}" ||
        "${observed_rviz_window_pgid}" != "${stack_pgid}" ]]; then
    echo "Owned RViz PID/PGID changed or left the stack group during ${stage}" >&2
    return 1
  fi
  if [[ "${observed_rviz_window_width_px}" != "${capture_rviz_window_width_px}" ||
        "${observed_rviz_window_height_px}" != "${capture_rviz_window_height_px}" ]]; then
    echo "Owned RViz geometry changed during ${stage}" >&2
    return 1
  fi
  printf 'RVIZ_CAPTURE_WINDOW_VERIFY_%s=pass\n' "${stage^^}" >> \
    "${output_dir}/runtime.env"
}

prepare_owned_rviz_capture_window() {
  if [[ "${capture_desktop}" != "true" ]]; then
    return 0
  fi

  local window_deadline=$((SECONDS + 30))
  local geometry_deadline=""
  local geometry=""
  local previous_geometry=""
  local stable_geometry_samples=0
  local window_state=""
  local -a matching_windows=()
  while (( SECONDS < window_deadline )); do
    mapfile -t matching_windows < <(matching_owned_rviz_capture_windows)
    if (( ${#matching_windows[@]} == 1 )); then
      break
    fi
    matching_windows=()
    sleep 1
  done
  if (( ${#matching_windows[@]} != 1 )); then
    echo "Expected exactly one centered RViz window for owned-window capture" >&2
    return 1
  fi
  capture_rviz_window_id="${matching_windows[0]}"
  capture_rviz_window_id_decimal="$((capture_rviz_window_id))"
  if [[ ! "${capture_rviz_window_id_decimal}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Could not convert the owned RViz XID to a positive decimal value" >&2
    return 1
  fi

  # The owned XID excludes GNOME shell surfaces by construction. Request only
  # maximization so the application content fills the 1920x1080 evidence
  # canvas; never raise it above the user's windows or move the user's pointer.
  if ! DISPLAY="${DISPLAY}" xprop -id "${capture_rviz_window_id}" \
    -f _NET_WM_STATE 32a -set _NET_WM_STATE \
    '_NET_WM_STATE_MAXIMIZED_HORZ, _NET_WM_STATE_MAXIMIZED_VERT'; then
    echo "Failed to request a maximized owned RViz capture window" >&2
    return 1
  fi
  window_state="$(
    DISPLAY="${DISPLAY}" xprop -id "${capture_rviz_window_id}" \
      _NET_WM_STATE 2>/dev/null || true
  )"
  if ! grep -q '_NET_WM_STATE_MAXIMIZED_HORZ' <<< "${window_state}" ||
     ! grep -q '_NET_WM_STATE_MAXIMIZED_VERT' <<< "${window_state}"; then
    echo "The centered RViz window did not accept the maximize request" >&2
    return 1
  fi

  geometry_deadline=$((SECONDS + 10))
  while (( SECONDS < geometry_deadline )); do
    inspect_owned_rviz_capture_window "${capture_rviz_window_id}" || return 1
    geometry="${observed_rviz_window_width_px}x${observed_rviz_window_height_px}"
    if [[ "${geometry}" == "${previous_geometry}" ]]; then
      stable_geometry_samples=$((stable_geometry_samples + 1))
    else
      previous_geometry="${geometry}"
      stable_geometry_samples=1
    fi
    if (( stable_geometry_samples >= 3 )); then
      break
    fi
    sleep 0.25
  done
  if (( stable_geometry_samples < 3 )); then
    echo "Owned RViz window geometry did not stabilize" >&2
    return 1
  fi
  if [[ "${observed_rviz_window_pgid}" != "${stack_pgid}" ]]; then
    echo "RViz X11 owner PID is not in the owned Autoware stack PGID" >&2
    return 1
  fi
  if (( observed_rviz_window_width_px < 1280 ||
        observed_rviz_window_height_px < 720 ||
        observed_rviz_window_width_px > capture_output_width_px ||
        observed_rviz_window_height_px > capture_output_height_px )); then
    echo "Owned RViz geometry must be 1280x720..${capture_output_dimensions}; got ${geometry}" >&2
    return 1
  fi

  capture_rviz_window_pid="${observed_rviz_window_pid}"
  capture_rviz_window_pgid="${observed_rviz_window_pgid}"
  capture_rviz_window_width_px="${observed_rviz_window_width_px}"
  capture_rviz_window_height_px="${observed_rviz_window_height_px}"
  capture_rviz_input_dimensions="${capture_rviz_window_width_px}x${capture_rviz_window_height_px}"
  capture_pad_left_px=$(((capture_output_width_px - capture_rviz_window_width_px) / 2))
  capture_pad_right_px=$((capture_output_width_px - capture_rviz_window_width_px - capture_pad_left_px))
  capture_pad_top_px=$(((capture_output_height_px - capture_rviz_window_height_px) / 2))
  capture_pad_bottom_px=$((capture_output_height_px - capture_rviz_window_height_px - capture_pad_top_px))
  capture_pad_filter="pad=${capture_output_width_px}:${capture_output_height_px}:${capture_pad_left_px}:${capture_pad_top_px}:color=black,setsar=1"
  verify_owned_rviz_capture_window prepared || return 1
  printf 'RVIZ_CAPTURE_WINDOW_ID=%s\nRVIZ_CAPTURE_WINDOW_ID_DECIMAL=%s\nRVIZ_CAPTURE_WINDOW_TITLE_CONFIG_MATCH=true\nRVIZ_CAPTURE_WINDOW_CLASS=rviz2\nRVIZ_CAPTURE_WINDOW_PID=%s\nRVIZ_CAPTURE_WINDOW_PGID=%s\nRVIZ_CAPTURE_STACK_PGID=%s\nRVIZ_CAPTURE_WINDOW_MAP_STATE=IsViewable\nRVIZ_CAPTURE_INPUT_WIDTH_PX=%s\nRVIZ_CAPTURE_INPUT_HEIGHT_PX=%s\nRVIZ_CAPTURE_PAD_LEFT_PX=%s\nRVIZ_CAPTURE_PAD_TOP_PX=%s\nRVIZ_CAPTURE_PAD_RIGHT_PX=%s\nRVIZ_CAPTURE_PAD_BOTTOM_PX=%s\nRVIZ_CAPTURE_PADDING_MODE=deterministic_center_black_v1\nRVIZ_CAPTURE_SCALING=none\nRVIZ_CAPTURE_SAMPLE_ASPECT_RATIO=1\nRVIZ_CAPTURE_GEOMETRY_STABLE=true\nRVIZ_CAPTURE_MAXIMIZED_REQUESTED=true\n' \
    "${capture_rviz_window_id}" "${capture_rviz_window_id_decimal}" \
    "${capture_rviz_window_pid}" "${capture_rviz_window_pgid}" \
    "${stack_pgid}" "${capture_rviz_window_width_px}" \
    "${capture_rviz_window_height_px}" "${capture_pad_left_px}" \
    "${capture_pad_top_px}" "${capture_pad_right_px}" \
    "${capture_pad_bottom_px}" >> "${output_dir}/runtime.env"
}

desktop_recorder_alive() {
  local process_state=""
  local actual_pgid=""
  if [[ "${capture_desktop}" != "true" ]]; then
    return 0
  fi
  if [[ ! "${desktop_pid}" =~ ^[1-9][0-9]*$ ||
        ! "${desktop_pgid}" =~ ^[1-9][0-9]*$ ||
        ! -f "/proc/${desktop_pid}/stat" ]]; then
    return 1
  fi
  if ! kill -0 "${desktop_pid}" 2>/dev/null; then
    return 1
  fi
  process_state="$(
    ps -o stat= -p "${desktop_pid}" 2>/dev/null | tr -d '[:space:]'
  )"
  if [[ -z "${process_state}" || "${process_state}" == Z* ]]; then
    return 1
  fi
  actual_pgid="$(
    ps -o pgid= -p "${desktop_pid}" 2>/dev/null | tr -d '[:space:]'
  )"
  [[ "${actual_pgid}" == "${desktop_pgid}" ]]
}

require_desktop_recorder() {
  local stage="$1"
  if [[ "${capture_desktop}" != "true" ]]; then
    return 0
  fi
  if ! desktop_recorder_alive; then
    echo "Owned RViz recorder exited or left its process group during ${stage}" >&2
    return 1
  fi
  printf 'RVIZ_CAPTURE_RECORDER_VERIFY_%s=pass\n' "${stage^^}" >> \
    "${output_dir}/runtime.env"
}

cleanup() {
  if [[ "${cleaned}" == "true" ]]; then
    return
  fi
  cleaned=true

  e2e_stop_owned_process_group "${desktop_pgid}" "${desktop_pid}" 15 5 2 || true
  desktop_pid=""
  desktop_pgid=""
  e2e_stop_owned_process_group "${route_test_pgid}" "${route_test_pid}" 15 5 2 || true
  route_test_pid=""
  route_test_pgid=""
  e2e_stop_owned_process_group "${recorder_pgid}" "${recorder_pid}" 15 5 2 || true
  recorder_pid=""
  recorder_pgid=""
  e2e_stop_owned_process_group "${stack_pgid}" "${stack_pid}" 30 5 2 || true
  stack_pid=""
  stack_pgid=""
}

on_signal() {
  local exit_status="$1"
  trap '' INT TERM
  cleanup
  trap - EXIT
  exit "${exit_status}"
}
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

stack_command=(scripts/e2e/run_route_vad_fast.sh --full)
if [[ "${visualize}" == "true" ]]; then
  if [[ "${capture_desktop}" == "true" ]]; then
    stack_command+=(--rviz-only)
  else
    stack_command+=(--visualize)
  fi
fi
if [[ "${recommended}" == "true" ]]; then
  stack_command=(scripts/e2e/run_route_vad_fast.sh --recommended)
  if [[ "${speed_30kph}" == "true" ]]; then
    stack_command+=(--speed-30kph)
  fi
  if [[ "${visualize}" == "true" ]]; then
    if [[ "${capture_desktop}" == "true" ]]; then
      stack_command+=(--rviz-only)
    else
      stack_command+=(--visualize)
    fi
  fi
  if [[ "${tight_corridor}" == "true" ]]; then
    stack_command+=(--tight-corridor)
  fi
  if [[ "${trajectory_stability}" == "true" ]]; then
    stack_command+=(--trajectory-stability)
  fi
elif [[ "${smart_mpc}" == "true" ]]; then
  stack_command=(scripts/e2e/run_route_vad_smart_mpc.sh)
elif [[ "${fp16_heads}" == "true" ]]; then
  stack_command=(scripts/e2e/run_route_vad_fast.sh --full --fp16-heads)
elif [[ -n "${model_override}" ]]; then
  stack_command+=(--model-override "${model_override}")
fi
if [[ "${recommended}" != "true" && -n "${sensor_mapping}" ]]; then
  stack_command+=(--sensor-mapping "${sensor_mapping}")
fi

capture_launch_arguments=()
if [[ "${capture_desktop}" == "true" ]]; then
  capture_launch_arguments+=("rviz_config:=${capture_rviz_runtime_config}")
fi
setsid "${stack_command[@]}" "${route_file}" \
  "${launch_arguments[@]}" "${capture_launch_arguments[@]}" > \
  "${output_dir}/stack.log" 2>&1 &
stack_pid=$!
stack_pgid="${stack_pid}"

critical_stack_child_failure() {
  local stack_log="$1"
  local failure_line=""

  [[ -f "${stack_log}" ]] || return 1
  failure_line="$(
    grep -m 1 -E \
      'process has died .*exit code .*mission_planner_container' \
      "${stack_log}" || true
  )"
  [[ -n "${failure_line}" ]] || return 1
  printf '%s\n' "${failure_line}"
}

deadline=$((SECONDS + ready_timeout))
route_ready=false
while (( SECONDS < deadline )); do
  require_carla_owner route_readiness || exit 1
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "Autoware stack exited before the VAD route became ready" >&2
    exit 1
  fi
  critical_failure=""
  if critical_failure="$(
    critical_stack_child_failure "${output_dir}/stack.log"
  )"; then
    printf '%s\n' "${critical_failure}" > \
      "${output_dir}/critical_process_failure.log"
    echo "Critical Autoware mission-planner process exited before route readiness: ${critical_failure}" >&2
    exit 1
  fi
  status="$(
    timeout 3 ros2 topic echo /planning/vad_route/status std_msgs/msg/String \
      --once --no-daemon --qos-reliability reliable 2>/dev/null || true
  )"
  if grep -Eq '^data: fault:' <<< "${status}"; then
    echo "VAD route manager faulted before ready: $(grep -E '^data:' <<< "${status}")" >&2
    exit 1
  fi
  if grep -Eq '^data: ready$' <<< "${status}"; then
    route_ready=true
    break
  fi
  if grep -Eq '^data: stopping$' <<< "${status}"; then
    # A short route can enter stopping before engage. Do not accept the same status
    # while VAD is still missing; require a newly delivered model candidate.
    if timeout 5 ros2 topic echo /planning/vad/candidate_trajectories \
      autoware_internal_planning_msgs/msg/CandidateTrajectories \
      --once --no-daemon --qos-reliability reliable >/dev/null 2>&1; then
      route_ready=true
      break
    fi
  fi
  sleep 1
done
if [[ "${route_ready}" != "true" ]]; then
  echo "Timed out waiting for a ready VAD route after ${ready_timeout}s" >&2
  exit 1
fi

prepare_owned_rviz_capture_window

candidate_observed_at=""
candidate_still_captured_at=""
desktop_recording_started_at=""
if [[ "${capture_desktop}" == "true" ]]; then
  # The visual proof is deliberately gated on an actual model output. A ready
  # route/status alone is not evidence that VAD inference published a candidate.
  if ! timeout 30 ros2 topic echo /planning/vad/candidate_trajectories \
    autoware_internal_planning_msgs/msg/CandidateTrajectories \
    --once --no-daemon --qos-reliability reliable >/dev/null 2>&1; then
    echo "Timed out waiting for a VAD candidate before desktop capture" >&2
    exit 1
  fi
  candidate_observed_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
  # Allow the already-started RViz window and its embedded camera panel to
  # paint the delivered candidate before preserving the stationary context.
  sleep 2
  verify_owned_rviz_capture_window candidate_pre
  if ! ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 \
    -window_id "${capture_rviz_window_id_decimal}" \
    -video_size "${capture_rviz_input_dimensions}" -i "${desktop_display}" \
    -vf "${capture_pad_filter}" \
    -frames:v 1 "${output_dir}/autoware_rviz_candidate.png"; then
    echo "Failed to capture the initial Autoware/RViz candidate PNG" >&2
    exit 1
  fi
  verify_owned_rviz_capture_window candidate_post
  candidate_still_captured_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
  desktop_recording_started_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
  setsid ffmpeg -y -nostdin -loglevel error -f x11grab -draw_mouse 0 \
    -framerate 5 -window_id "${capture_rviz_window_id_decimal}" \
    -video_size "${capture_rviz_input_dimensions}" -i "${desktop_display}" \
    -vf "${capture_pad_filter}" \
    -c:v libx264 -preset ultrafast -crf 20 \
    -pix_fmt yuv420p \
    "${output_dir}/autoware_rviz_capture.mkv" &
  desktop_pid=$!
  desktop_pgid="${desktop_pid}"
  sleep 1
  require_desktop_recorder recording_started
  verify_owned_rviz_capture_window recording_started
fi

setsid scripts/e2e/record_turn_dynamics.sh "${output_dir}/bag" \
  > "${output_dir}/recorder.log" 2>&1 &
recorder_pid=$!
recorder_pgid="${recorder_pid}"
sleep 1
if ! kill -0 "${recorder_pid}" 2>/dev/null; then
  echo "Turn recorder failed to start" >&2
  exit 1
fi

set +e
route_evaluation_started_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
route_test_arguments=(
  --full-stack
  --route-file "${route_file}"
  --result "${output_dir}/result.json"
)
if [[ "${speed_30kph}" == "true" ]]; then
  route_test_arguments+=(
    --max-cte 1.0
    --max-observed-speed "${maximum_observed_speed_mps}"
    --max-lateral-acceleration "${maximum_lateral_acceleration_limit_mps2}"
    --max-speed-sample-gap 0.25
    --longitudinal-speed-source explicit_simulation_nominal
    --no-vad-velocity-evaluated
    --vad-geometry-evaluated
  )
  if [[ "${speed_exposure_mode}" == "straight_target_required" ]]; then
    route_test_arguments+=(
      --min-sustained-speed "${minimum_sustained_speed_mps}"
      --min-sustained-speed-sec "${minimum_sustained_speed_sec}"
    )
  fi
fi
setsid scripts/e2e/route_test.sh "${route_test_arguments[@]}" > \
  "${output_dir}/route_test.log" 2>&1 &
route_test_pid=$!
route_test_pgid="${route_test_pid}"
while kill -0 "${route_test_pid}" 2>/dev/null; do
  if ! require_carla_owner route_evaluation; then
    e2e_stop_owned_process_group \
      "${route_test_pgid}" "${route_test_pid}" 15 5 2 || true
    route_test_pid=""
    route_test_pgid=""
    exit 1
  fi
  critical_failure=""
  if critical_failure="$(
    critical_stack_child_failure "${output_dir}/stack.log"
  )"; then
    printf '%s\n' "${critical_failure}" > \
      "${output_dir}/critical_process_failure.log"
    echo "Critical Autoware mission-planner process exited during route evaluation: ${critical_failure}" >&2
    e2e_stop_owned_process_group \
      "${route_test_pgid}" "${route_test_pid}" 15 5 2 || true
    route_test_pid=""
    route_test_pgid=""
    exit 1
  fi
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "Autoware stack exited during route evaluation" >&2
    e2e_stop_owned_process_group \
      "${route_test_pgid}" "${route_test_pid}" 15 5 2 || true
    route_test_pid=""
    route_test_pgid=""
    exit 1
  fi
  if [[ "${capture_desktop}" == "true" ]] && ! desktop_recorder_alive; then
    echo "Owned RViz recorder exited during route evaluation" >&2
    e2e_stop_owned_process_group \
      "${route_test_pgid}" "${route_test_pid}" 15 5 2 || true
    route_test_pid=""
    route_test_pgid=""
    exit 1
  fi
  sleep 0.25
done
wait "${route_test_pid}"
evaluation_status=$?
route_test_pid=""
route_test_pgid=""
route_evaluation_finished_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
set -e

verify_owned_rviz_capture_window representative
require_desktop_recorder representative

require_carla_owner route_completion || exit 1
if ! python3 scripts/e2e/probe_carla_server.py \
  "${carla_probe_args[@]}" --stage trial_completion \
  --output "${output_dir}/carla_completion_health.json"; then
  echo "CARLA failed the post-route read-only RPC/map/snapshot check" >&2
  exit 1
fi

critical_failure=""
if critical_failure="$(
  critical_stack_child_failure "${output_dir}/stack.log"
)"; then
  printf '%s\n' "${critical_failure}" > \
    "${output_dir}/critical_process_failure.log"
  echo "Critical Autoware mission-planner process exited at route completion: ${critical_failure}" >&2
  exit 1
fi

ros2 param dump /vad_route_manager > "${output_dir}/vad_route_manager.params.yaml" 2> \
  "${output_dir}/vad_param_dump.err" || true
ros2 param dump /control/trajectory_follower/controller_node_exe > \
  "${output_dir}/controller.params.yaml" 2> "${output_dir}/controller_param_dump.err" || true
ros2 param dump /control/vehicle_cmd_gate > \
  "${output_dir}/vehicle_cmd_gate.params.yaml" 2> \
  "${output_dir}/vehicle_cmd_gate_param_dump.err" || true

critical_failure=""
if critical_failure="$(
  critical_stack_child_failure "${output_dir}/stack.log"
)"; then
  printf '%s\n' "${critical_failure}" > \
    "${output_dir}/critical_process_failure.log"
  echo "Critical Autoware mission-planner process exited before evidence finalization: ${critical_failure}" >&2
  exit 1
fi

cleanup
trap - EXIT INT TERM

analysis_arguments=(
  --bag "${output_dir}/bag"
  --route-file "${route_file}"
  --result-dir "${output_dir}"
  --steering-report-mode virtual
)
if [[ -n "${mpc_input_delay}" ]]; then
  analysis_arguments+=(--mpc-input-delay-sec "${mpc_input_delay}")
fi
if [[ -n "${mpc_steer_tau}" ]]; then
  analysis_arguments+=(--mpc-steer-tau-sec "${mpc_steer_tau}")
fi
if [[ -n "${maneuver_lookahead_m}" ]]; then
  analysis_arguments+=(--maneuver-lookahead-m "${maneuver_lookahead_m}")
fi

analysis_status=0
if [[ "${capture_desktop}" == "true" ]]; then
  capture_duration_sec="$(
    ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 \
      "${output_dir}/autoware_rviz_capture.mkv" 2>/dev/null || true
  )"
  representative_offset_sec=""
  if [[ -z "${capture_duration_sec}" ]]; then
    echo "Failed to determine the Autoware/RViz desktop recording duration" >&2
    analysis_status=1
  elif ! representative_offset_sec="$(
    python3 - "${desktop_recording_started_at}" \
      "${route_evaluation_started_at}" "${route_evaluation_finished_at}" \
      "${capture_duration_sec}" <<'PY'
from datetime import datetime
import math
import sys

def timestamp(raw):
    return datetime.fromisoformat(raw.replace("Z", "+00:00"))

recording_started = timestamp(sys.argv[1])
evaluation_started = timestamp(sys.argv[2])
evaluation_finished = timestamp(sys.argv[3])
duration = float(sys.argv[4])
if not math.isfinite(duration) or duration <= 0.0:
    raise SystemExit(f"invalid desktop recording duration: {duration!r}")
if not recording_started <= evaluation_started < evaluation_finished:
    raise SystemExit("invalid route-evaluation timestamps for desktop capture")
representative_at = evaluation_started + (evaluation_finished - evaluation_started) / 2
offset = (representative_at - recording_started).total_seconds()
if not 0.0 < offset < duration:
    raise SystemExit(
        f"route-evaluation midpoint {offset:.6f}s is outside recording {duration:.6f}s"
    )
print(f"{offset:.6f}")
PY
  )"; then
    echo "Failed to select a representative in-route Autoware/RViz frame" >&2
    analysis_status=1
  elif ! ffmpeg -y -loglevel error \
    -i "${output_dir}/autoware_rviz_capture.mkv" \
    -ss "${representative_offset_sec}" -frames:v 1 -an \
    "${output_dir}/autoware_rviz_fullscreen.png"; then
    echo "Failed to extract the representative Autoware/RViz full-screen PNG" >&2
    analysis_status=1
  elif ! ffmpeg -y -loglevel error -i "${output_dir}/autoware_rviz_capture.mkv" \
    -filter_complex \
    '[0:v]fps=5,scale=960:-2:flags=lanczos,split[gif_a][gif_b];[gif_a]palettegen=max_colors=128[palette];[gif_b][palette]paletteuse=dither=bayer' \
    -loop 0 "${output_dir}/autoware_rviz_drive.gif"; then
    echo "Failed to render the Autoware/RViz desktop GIF" >&2
    analysis_status=1
  elif ! python3 - "${output_dir}" "${candidate_observed_at}" \
    "${candidate_still_captured_at}" "${desktop_recording_started_at}" \
    "${route_evaluation_started_at}" "${route_evaluation_finished_at}" \
    "${DISPLAY}" "${capture_output_dimensions}" "${capture_duration_sec}" \
    "${representative_offset_sec}" "${capture_rviz_config_sha256}" \
    "${desktop_dimensions}" "${capture_rviz_input_dimensions}" \
    "${capture_pad_left_px}" "${capture_pad_top_px}" \
    "${capture_pad_right_px}" "${capture_pad_bottom_px}" \
    "${capture_rviz_window_id}" "${capture_rviz_window_id_decimal}" \
    "${capture_rviz_window_pid}" "${capture_rviz_window_pgid}" <<'PY'
import hashlib
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
import sys

from PIL import Image
import yaml

output = Path(sys.argv[1])
candidate_observed_at = sys.argv[2]
candidate_still_captured_at = sys.argv[3]
recording_started_at = sys.argv[4]
evaluation_started_at = sys.argv[5]
evaluation_finished_at = sys.argv[6]
display = sys.argv[7]
source_dimensions = [int(value) for value in sys.argv[8].split("x")]
recording_duration_sec = float(sys.argv[9])
representative_offset_sec = float(sys.argv[10])
expected_config_sha256 = sys.argv[11]
display_dimensions = [int(value) for value in sys.argv[12].split("x")]
input_dimensions = [int(value) for value in sys.argv[13].split("x")]
padding_px = {
    "left": int(sys.argv[14]),
    "top": int(sys.argv[15]),
    "right": int(sys.argv[16]),
    "bottom": int(sys.argv[17]),
}
window_id_hex = sys.argv[18]
window_id_decimal = int(sys.argv[19])
window_pid = int(sys.argv[20])
window_pgid = int(sys.argv[21])

if source_dimensions != [1920, 1080]:
    raise SystemExit(f"owned-window output canvas must be 1920x1080: {source_dimensions}")
if len(display_dimensions) != 2 or any(value <= 0 for value in display_dimensions):
    raise SystemExit(f"invalid DISPLAY dimensions: {display_dimensions}")
if len(input_dimensions) != 2 or any(value <= 0 for value in input_dimensions):
    raise SystemExit(f"invalid owned-window input dimensions: {input_dimensions}")
if any(value < 0 for value in padding_px.values()):
    raise SystemExit(f"owned-window padding cannot be negative: {padding_px}")
if (
    input_dimensions[0] + padding_px["left"] + padding_px["right"]
    != source_dimensions[0]
    or input_dimensions[1] + padding_px["top"] + padding_px["bottom"]
    != source_dimensions[1]
):
    raise SystemExit(
        "owned-window input and deterministic padding do not fill the output canvas"
    )
if not window_id_hex.startswith("0x") or int(window_id_hex, 16) != window_id_decimal:
    raise SystemExit("owned RViz hexadecimal and decimal XIDs disagree")
if window_id_decimal <= 0 or window_pid <= 0 or window_pgid <= 0:
    raise SystemExit("owned RViz XID/PID/PGID must be positive")

def timestamp(raw):
    return datetime.fromisoformat(raw.replace("Z", "+00:00"))

candidate_observed = timestamp(candidate_observed_at)
candidate_still_captured = timestamp(candidate_still_captured_at)
recording_started = timestamp(recording_started_at)
evaluation_started = timestamp(evaluation_started_at)
evaluation_finished = timestamp(evaluation_finished_at)
representative_at = recording_started + timedelta(seconds=representative_offset_sec)
now = datetime.now(timezone.utc)
if not (
    candidate_observed
    <= candidate_still_captured
    <= recording_started
    <= evaluation_started
    < representative_at
    < evaluation_finished
    <= now
):
    raise SystemExit("desktop evidence timestamps are not monotonically ordered")
if (
    not math.isfinite(recording_duration_sec)
    or not math.isfinite(representative_offset_sec)
    or not 0.0 < representative_offset_sec < recording_duration_sec
):
    raise SystemExit("representative PNG offset is outside the desktop recording")

with Image.open(output / "autoware_rviz_fullscreen.png") as image:
    png_dimensions = list(image.size)
with Image.open(output / "autoware_rviz_candidate.png") as image:
    candidate_png_dimensions = list(image.size)
with Image.open(output / "autoware_rviz_drive.gif") as image:
    gif_dimensions = list(image.size)
if png_dimensions != source_dimensions or candidate_png_dimensions != source_dimensions:
    raise SystemExit(
        "full-screen PNG dimensions do not match the captured DISPLAY: "
        f"representative={png_dimensions}, candidate={candidate_png_dimensions}, "
        f"DISPLAY={source_dimensions}"
    )
if gif_dimensions[0] != 960:
    raise SystemExit(f"GIF width must be 960, got {gif_dimensions}")

rviz_config = output / "rviz_capture_provenance/autoware_vad_carla.rviz"
config_bytes = rviz_config.read_bytes()
actual_config_sha256 = hashlib.sha256(config_bytes).hexdigest()
if actual_config_sha256 != expected_config_sha256:
    raise SystemExit("captured RViz config changed after preflight validation")
config = yaml.safe_load(config_bytes)
view = config["Visualization Manager"]["Views"]["Current"]
view_contract = {
    "controller": view.get("Class"),
    "target_frame": view.get("Target Frame"),
    "angle_rad": float(view.get("Angle")),
    "center_xy_m": [float(view.get("X")), float(view.get("Y"))],
    "scale": float(view.get("Scale")),
}
expected_view_contract = {
    "controller": "rviz_default_plugins/TopDownOrtho",
    "target_frame": "base_link",
    "angle_rad": 0.0,
    "center_xy_m": [0.0, 0.0],
    "scale": 10.0,
}
if view_contract != expected_view_contract:
    raise SystemExit(
        f"RViz centered-follow contract changed: {view_contract!r}"
    )

required_path_topics = {
    "/planning/vad_route/reference_path",
    "/planning/vad_route/actual_path",
    "/planning/trajectory",
    "/planning/vad_route/selected_raw_trajectory",
    "/planning/vad/candidate_trajectories",
}
visible_path_topics = set()

def visit(value):
    if isinstance(value, dict):
        topic = value.get("Topic")
        topic_name = topic.get("Value") if isinstance(topic, dict) else topic
        if (
            topic_name in required_path_topics
            and value.get("Enabled") is True
            and value.get("Value") is True
        ):
            visible_path_topics.add(topic_name)
        for child in value.values():
            visit(child)
    elif isinstance(value, list):
        for child in value:
            visit(child)

visit(config["Visualization Manager"]["Displays"])
if visible_path_topics != required_path_topics:
    raise SystemExit(
        "RViz centered-follow config hides required path topics: "
        f"{sorted(required_path_topics - visible_path_topics)}"
    )

def named_display(value, name):
    if isinstance(value, dict):
        if value.get("Name") == name:
            return value
        for child in value.values():
            found = named_display(child, name)
            if found is not None:
                return found
    elif isinstance(value, list):
        for child in value:
            found = named_display(child, name)
            if found is not None:
                return found
    return None

odometry = named_display(config["Visualization Manager"]["Displays"], "Kinematic State")
candidates = named_display(
    config["Visualization Manager"]["Displays"], "VAD Candidate Trajectories"
)
front_camera = named_display(
    config["Visualization Manager"]["Displays"], "VAD Front Camera"
)
front_camera_topic = (
    front_camera.get("Topic", {}).get("Value")
    if isinstance(front_camera, dict)
    else None
)
camera_view_contract = {
    "embedded_rviz_display": "VAD Front Camera",
    "embedded_rviz_enabled": front_camera.get("Enabled") is True
    and front_camera.get("Value") is True
    if isinstance(front_camera, dict)
    else False,
    "embedded_rviz_topic": front_camera_topic,
    "external_rqt_image_view_launched": False,
    "occlusion_guard": "owned_rviz_window_only_v1",
}
expected_camera_view_contract = {
    "embedded_rviz_display": "VAD Front Camera",
    "embedded_rviz_enabled": True,
    "embedded_rviz_topic": "/sensing/camera/CAM_FRONT/image_raw",
    "external_rqt_image_view_launched": False,
    "occlusion_guard": "owned_rviz_window_only_v1",
}
if camera_view_contract != expected_camera_view_contract:
    raise SystemExit(
        f"RViz embedded-camera contract changed: {camera_view_contract!r}"
    )
covariance = odometry.get("Covariance", {}) if isinstance(odometry, dict) else {}
visual_clarity = {
    "odometry_display": "Kinematic State",
    "odometry_keep": odometry.get("Keep") if isinstance(odometry, dict) else None,
    "odometry_covariance": covariance.get("Value"),
    "odometry_orientation": covariance.get("Orientation", {}).get("Value"),
    "odometry_position": covariance.get("Position", {}).get("Value"),
    "candidate_path_alpha": candidates.get("View Path", {}).get("Alpha")
    if isinstance(candidates, dict)
    else None,
    "candidate_path_width": candidates.get("View Path", {}).get("Width")
    if isinstance(candidates, dict)
    else None,
}
expected_visual_clarity = {
    "odometry_display": "Kinematic State",
    "odometry_keep": 1,
    "odometry_covariance": False,
    "odometry_orientation": False,
    "odometry_position": False,
    "candidate_path_alpha": 0.22,
    "candidate_path_width": 0.04,
}
if visual_clarity != expected_visual_clarity:
    raise SystemExit(
        f"RViz visual-clarity contract changed: {visual_clarity!r}"
    )

payload = {
    "schema_version": 1,
    "candidate_observed": True,
    "candidate_topic": "/planning/vad/candidate_trajectories",
    "capture_started_after_candidate": True,
    "candidate_observed_at": candidate_observed_at,
    "candidate_still_captured_at": candidate_still_captured_at,
    "recording_started_at": recording_started_at,
    "route_evaluation_started_at": evaluation_started_at,
    "route_evaluation_finished_at": evaluation_finished_at,
    "captured_at": representative_at.isoformat(),
    "display": display,
    "display_dimensions": display_dimensions,
    "source_dimensions": source_dimensions,
    "capture_source": {
        "method": "ffmpeg_x11grab_owned_window_v1",
        "root_capture": False,
        "shell_surfaces_excluded": True,
        "window_id_hex": window_id_hex,
        "window_id_decimal": window_id_decimal,
        "window_title_contains_config": True,
        "window_config_path": str(rviz_config),
        "wm_class": "rviz2",
        "pid": window_pid,
        "pgid": window_pgid,
        "stack_process_group_owned": True,
        "map_state": "IsViewable",
        "geometry_stable": True,
        "input_dimensions": input_dimensions,
        "output_dimensions": source_dimensions,
        "padding_px": padding_px,
        "padding_mode": "deterministic_center_black_v1",
        "scaling": "none",
        "sample_aspect_ratio": 1,
    },
    "png_dimensions": png_dimensions,
    "candidate_png_dimensions": candidate_png_dimensions,
    "gif_dimensions": gif_dimensions,
    "png_file": "autoware_rviz_fullscreen.png",
    "candidate_png_file": "autoware_rviz_candidate.png",
    "gif_file": "autoware_rviz_drive.gif",
    "recording_file": "autoware_rviz_capture.mkv",
    "desktop_overlay_check": {
        "method": "owned_window_excludes_shell_surfaces_v1",
        "root_capture": False,
        "shell_surfaces_excluded": True,
        "passed": True,
    },
    "camera_view_contract": camera_view_contract,
    "representative_frame": {
        "source": "autoware_rviz_capture.mkv",
        "selection": "route_evaluation_midpoint",
        "offset_sec": representative_offset_sec,
        "recording_duration_sec": recording_duration_sec,
        "captured_at": representative_at.isoformat(),
    },
    "rviz_view_contract": {
        **view_contract,
        "vehicle_centered": True,
        "config_file": "rviz_capture_provenance/autoware_vad_carla.rviz",
        "config_sha256": actual_config_sha256,
        "visible_path_topics": sorted(visible_path_topics),
        "visual_clarity": visual_clarity,
    },
}
(output / "desktop_capture.json").write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
PY
  then
    echo "Failed to validate the Autoware/RViz desktop capture" >&2
    analysis_status=1
  fi
fi
if [[ "${speed_30kph}" == "true" ]]; then
  speed_analysis_status=0
  python3 scripts/e2e/analyze_speed_profile.py \
    --bag "${output_dir}/bag" \
    --route-file "${route_file}" \
    --result "${output_dir}/result.json" \
    --output-dir "${output_dir}" \
    --profile-id "${speed_profile_id}" \
    --target-speed-mps "${target_speed_mps}" \
    --longitudinal-speed-source explicit_simulation_nominal > \
    "${output_dir}/speed_profile_analysis.log" 2>&1 || speed_analysis_status=$?
  if (( speed_analysis_status != 0 )); then
    echo "30 km/h speed-source analysis failed; see ${output_dir}/speed_profile_analysis.log" >&2
    analysis_status=1
  fi
fi
python3 scripts/e2e/analyze_turn_dynamics.py "${analysis_arguments[@]}" || analysis_status=$?
python3 scripts/e2e/analyze_e2e_latency.py --bag "${output_dir}/bag" \
  --output-dir "${output_dir}/latency" || analysis_status=$?
scripts/e2e/render_route_result.sh "${route_file}" "${output_dir}/result.json" \
  --output "${output_dir}/route_result.png" || analysis_status=$?
animation_arguments=(
  --bag "${output_dir}/bag"
  --route-file "${route_file}"
  --output-gif "${output_dir}/turn_path_control.gif"
)
if ! python3 scripts/e2e/render_turn_animation.py "${animation_arguments[@]}" --crop turn; then
  echo "No turn interval was rendered; retrying the animation over the motion interval." >&2
  python3 scripts/e2e/render_turn_animation.py \
    "${animation_arguments[@]}" --crop motion || analysis_status=$?
fi

if (( evaluation_status != 0 )); then
  echo "Route evaluation failed; see ${output_dir}/route_test.log" >&2
  exit "${evaluation_status}"
fi
if (( analysis_status != 0 )); then
  echo "Route passed, but post-processing failed with ${analysis_status}" >&2
  exit "${analysis_status}"
fi

echo "Recorded route trial passed: ${output_dir}"
