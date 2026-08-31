#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh
source scripts/e2e/process_group_cleanup.sh

usage() {
  cat >&2 <<'EOF'
Usage: run_recorded_route_trial.sh [options] OUTPUT_DIR ROUTE_JSON [launch arguments...]

Options:
  --recommended          Run the repeat-screened full-stack parameter profile
  --visualize            Start RViz and the front-camera view in full mode
  --capture-desktop      Record full-screen PNG/GIF evidence after a VAD candidate
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
visualize=false
capture_desktop=false
trajectory_stability=false
tight_corridor=false
comfortable_deceleration_mps2=""
model_override=""
sensor_mapping=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --recommended)
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
  comfortable_deceleration_mps2="0.60"
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
      use_fast_vad:=*|vad_use_fp16_heads:=*|use_light_weight_sensor_mapping:=*|rviz:=*|launch_fast_camera_view:=*|use_lateral_controller_param_override:=*|lateral_controller_param_path:=*|use_longitudinal_controller_param_override:=*|longitudinal_controller_param_path:=*|controller_stop_offset_m:=*|comfortable_deceleration_mps2:=*|maneuver_lookahead_m:=*|maneuver_exit_lookahead_m:=*|turn_inward_corridor_half_width_m:=*|turn_outward_corridor_half_width_m:=*|left_turn_outward_corridor_half_width_m:=*|right_turn_outward_corridor_half_width_m:=*|route_corridor_entry_distance_m:=*|trajectory_lateral_filter_gain:=*|left_turn_trajectory_lateral_filter_gain:=*|right_turn_trajectory_lateral_filter_gain:=*|trajectory_lateral_filter_activation_threshold_m:=*|trajectory_geometry_smoothing_strength:=*|maximum_lateral_acceleration_mps2:=*|maximum_speed_mps:=*|raw_vehicle_cmd_converter_config:=*)
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

recommended_mpc=""
if [[ "${recommended}" == "true" ]]; then
  package_share="$(ros2 pkg prefix autoware_e2e_vad_launch)/share/autoware_e2e_vad_launch"
  model_override="${package_share}/config/vad_carla_tiny_recommended.param.yaml"
  sensor_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable.yaml"
  recommended_mpc="${package_share}/config/mpc_carla_recommended.param.yaml"
  for required in "${model_override}" "${sensor_mapping}" "${recommended_mpc}"; do
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
  echo "Output directory already exists: ${output_dir}" >&2
  exit 2
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
python3 - "${carla_host}" "${carla_port}" <<'PY'
import socket
import sys

with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=3.0):
    pass
PY

conflicts="$(ros2 node list --no-daemon 2>/dev/null | grep -E '/(vad_route_manager|autoware_carla_interface|vad_carla_tiny)$' || true)"
if [[ -n "${conflicts}" ]]; then
  echo "An existing project stack is visible in ROS domain ${ROS_DOMAIN_ID}:" >&2
  echo "${conflicts}" >&2
  exit 1
fi

mkdir -p "${output_dir}"
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
source_route_file="${route_file}"
route_town="$(
  python3 - "${source_route_file}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    print(json.load(stream)["town"])
PY
)"
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
printf 'SOURCE_ROUTE_FILE=%s\nEFFECTIVE_ROUTE_FILE=%s\nFULL_MAP_PATH=%s\n' \
  "${source_route_file}" "${route_file}" "${full_map_path}" >> "${output_dir}/runtime.env"
printf 'RECOMMENDED=%s\nVISUALIZE=%s\nCAPTURE_DESKTOP=%s\nTIGHT_CORRIDOR_CANDIDATE=%s\nTRAJECTORY_STABILITY_CANDIDATE=%s\nSMART_MPC=%s\nFP16_HEADS=%s\n' \
  "${recommended}" "${visualize}" "${capture_desktop}" "${tight_corridor}" "${trajectory_stability}" "${smart_mpc}" "${fp16_heads}" >> \
  "${output_dir}/runtime.env"
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
if [[ "${tight_corridor}" == "true" && "${trajectory_stability}" == "true" ]]; then
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
desktop_pid=""
desktop_pgid=""
capture_rviz_window_id=""
cleaned=false

pin_rviz_capture_window() {
  if [[ "${capture_desktop}" != "true" ]]; then
    return 0
  fi

  local window_deadline=$((SECONDS + 30))
  local window_id=""
  local window_state=""
  while (( SECONDS < window_deadline )); do
    # Match the pinned runtime config in the top-level RViz title. Read the
    # complete tree so pipefail cannot turn an early awk exit into a false
    # failure from xwininfo.
    window_id="$(
      DISPLAY="${DISPLAY}" xwininfo -root -tree 2>/dev/null |
        awk -v needle="${capture_rviz_runtime_config}" \
          'index($0, needle) && !found {id=$1; found=1} END {print id}'
    )"
    if [[ "${window_id}" =~ ^0x[0-9a-fA-F]+$ ]]; then
      break
    fi
    window_id=""
    sleep 1
  done
  if [[ -z "${window_id}" ]]; then
    echo "Could not identify the centered RViz window for desktop capture" >&2
    return 1
  fi

  # A desktop capture must remain an Autoware/RViz capture even if another
  # application receives input while the route is running. Keep only this
  # short-lived RViz window above normal windows; it disappears with the
  # owned stack process group during cleanup.
  if ! DISPLAY="${DISPLAY}" xprop -id "${window_id}" \
    -f _NET_WM_STATE 32a -set _NET_WM_STATE \
    '_NET_WM_STATE_MAXIMIZED_HORZ, _NET_WM_STATE_MAXIMIZED_VERT, _NET_WM_STATE_ABOVE'; then
    echo "Failed to pin the centered RViz window above normal windows" >&2
    return 1
  fi
  window_state="$(
    DISPLAY="${DISPLAY}" xprop -id "${window_id}" _NET_WM_STATE 2>/dev/null || true
  )"
  if ! grep -q '_NET_WM_STATE_ABOVE' <<< "${window_state}"; then
    echo "The centered RViz window did not accept _NET_WM_STATE_ABOVE" >&2
    return 1
  fi
  capture_rviz_window_id="${window_id}"
  printf 'RVIZ_CAPTURE_FOREGROUND_GUARD=rviz_above\nRVIZ_CAPTURE_WINDOW_ID=%s\n' \
    "${capture_rviz_window_id}" >> "${output_dir}/runtime.env"
}

cleanup() {
  if [[ "${cleaned}" == "true" ]]; then
    return
  fi
  cleaned=true

  e2e_stop_owned_process_group "${desktop_pgid}" "${desktop_pid}" 15 5 2 || true
  desktop_pid=""
  desktop_pgid=""
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
  stack_command+=(--visualize)
fi
if [[ "${recommended}" == "true" ]]; then
  stack_command=(scripts/e2e/run_route_vad_fast.sh --recommended)
  if [[ "${visualize}" == "true" ]]; then
    stack_command+=(--visualize)
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

deadline=$((SECONDS + ready_timeout))
route_ready=false
while (( SECONDS < deadline )); do
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "Autoware stack exited before the VAD route became ready" >&2
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

pin_rviz_capture_window

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
  # Allow the already-started RViz/camera windows to paint the delivered
  # candidate before preserving the initial, stationary context frame.
  sleep 2
  if ! ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 \
    -video_size "${desktop_dimensions}" -i "${desktop_display}+0,0" \
    -frames:v 1 "${output_dir}/autoware_rviz_candidate.png"; then
    echo "Failed to capture the initial Autoware/RViz candidate PNG" >&2
    exit 1
  fi
  candidate_still_captured_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
  desktop_recording_started_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
  setsid ffmpeg -y -nostdin -loglevel error -f x11grab -draw_mouse 0 \
    -framerate 5 -video_size "${desktop_dimensions}" \
    -i "${desktop_display}+0,0" -c:v libx264 -preset ultrafast -crf 20 \
    -pix_fmt yuv420p \
    "${output_dir}/autoware_rviz_capture.mkv" &
  desktop_pid=$!
  desktop_pgid="${desktop_pid}"
  sleep 1
  if ! kill -0 "${desktop_pid}" 2>/dev/null; then
    echo "Autoware/RViz desktop recorder failed to start" >&2
    exit 1
  fi
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
scripts/e2e/route_test.sh --full-stack --route-file "${route_file}" \
  --result "${output_dir}/result.json" > "${output_dir}/route_test.log" 2>&1
evaluation_status=$?
route_evaluation_finished_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
set -e

ros2 param dump /vad_route_manager > "${output_dir}/vad_route_manager.params.yaml" 2> \
  "${output_dir}/vad_param_dump.err" || true
ros2 param dump /control/trajectory_follower/controller_node_exe > \
  "${output_dir}/controller.params.yaml" 2> "${output_dir}/controller_param_dump.err" || true

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
    "${DISPLAY}" "${desktop_dimensions}" "${capture_duration_sec}" \
    "${representative_offset_sec}" "${capture_rviz_config_sha256}" <<'PY'
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
    "source_dimensions": source_dimensions,
    "png_dimensions": png_dimensions,
    "candidate_png_dimensions": candidate_png_dimensions,
    "gif_dimensions": gif_dimensions,
    "png_file": "autoware_rviz_fullscreen.png",
    "candidate_png_file": "autoware_rviz_candidate.png",
    "gif_file": "autoware_rviz_drive.gif",
    "recording_file": "autoware_rviz_capture.mkv",
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
