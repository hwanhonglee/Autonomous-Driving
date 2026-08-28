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
route_file="$(realpath -- "$2")"
shift 2
launch_arguments=("$@")

recommended_mpc=""
if [[ "${recommended}" == "true" ]]; then
  package_share="$(ros2 pkg prefix autoware_e2e_vad_launch)/share/autoware_e2e_vad_launch"
  model_override="${package_share}/config/vad_carla_tiny_recommended.param.yaml"
  sensor_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable.yaml"
  recommended_mpc="${package_share}/config/mpc_carla_recommended.param.yaml"
  mpc_input_delay="${mpc_input_delay:-0.12}"
  mpc_steer_tau="${mpc_steer_tau:-0.15}"
  comfortable_deceleration_mps2="0.60"
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
if [[ ! -f "${route_file}" ]]; then
  echo "Route file not found: ${route_file}" >&2
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
printf 'RECOMMENDED=%s\nTIGHT_CORRIDOR_CANDIDATE=%s\nTRAJECTORY_STABILITY_CANDIDATE=%s\nSMART_MPC=%s\nFP16_HEADS=%s\n' \
  "${recommended}" "${tight_corridor}" "${trajectory_stability}" "${smart_mpc}" "${fp16_heads}" >> \
  "${output_dir}/runtime.env"
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
cleaned=false

cleanup() {
  if [[ "${cleaned}" == "true" ]]; then
    return
  fi
  cleaned=true

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
if [[ "${recommended}" == "true" ]]; then
  stack_command=(scripts/e2e/run_route_vad_fast.sh --recommended)
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

setsid "${stack_command[@]}" "${route_file}" \
  "${launch_arguments[@]}" > "${output_dir}/stack.log" 2>&1 &
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
scripts/e2e/route_test.sh --full-stack --route-file "${route_file}" \
  --result "${output_dir}/result.json" > "${output_dir}/route_test.log" 2>&1
evaluation_status=$?
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
