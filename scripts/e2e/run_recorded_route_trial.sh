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
  --speed-60kph-pilot    Add the straight-only CARLA 16.667 m/s exploratory pilot
  --camera-source-5hz    Render six CARLA cameras at 5 sim-Hz with the pinned
                         localhost-only Best-Effort KEEP_LAST depth-1 profile
  --portable-shadow-10hz Run Portable E2E at the exact six-camera 10 Hz ABI in
                         isolated shadow-only mode; requires --speed-30kph
  --portable-runtime-bundle FILE
                         Pinned non-executable Portable E2E runtime .npz bundle
  --portable-runtime-bundle-sha256 SHA256
                         Lowercase SHA-256 of the exact runtime bundle
  --portable-source-checkpoint-sha256 SHA256
                         Lowercase source checkpoint SHA-256 embedded in bundle
  --portable-model-config-sha256 SHA256
                         Lowercase canonical model configuration SHA-256
  --portable-corpus-fingerprint-sha256 SHA256
                         Lowercase training corpus fingerprint SHA-256
  --portable-rig-file JSON
                         Pinned regular Common10 camera rig file
  --portable-rig-sha256 SHA256
                         Lowercase SHA-256 of the camera rig file
  --portable-contract-file JSON
                         Pinned regular Common10 contract file
  --portable-contract-sha256 SHA256
                         Lowercase SHA-256 of the Common10 contract file
  --portable-device DEVICE
                         cpu (default) or UUID-pinned logical cuda:0
  --portable-cpu-set LIST
                         Comma-separated online CPU IDs used only by CPU shadow inference
  --control-ab-pid-i40   30 kph A/B: PID max_i_effort 0.30 -> 0.40 only
  --control-ab-turn-preview-5m
                         30 kph turn A/B: curvature preview 3 m -> 5 m only
  --control-ab-turn-preview-10m
                         30 kph turn A/B: curvature preview 3 m -> 10 m only
  --control-ab-longitudinal-recovery-2p0
                         30 kph straight A/B: post-curve planning-speed recovery
                         1.5 -> 2.0 m/s^2 only; actuator limits remain 1.5
  --geometry-ab-route-corridor-0p2
                         60 kph geometry A/B: route corridor 0.50 m -> 0.20 m only
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
  --runtime-health-gate  Require stable /clock and six-camera health before engagement
  --runtime-health-timeout SEC
                         Runtime-health wall timeout (default: 45)

CARLA must already be running at CARLA_HOST/CARLA_PORT. The helper owns only the
Autoware stack and recorder processes that it starts.
EOF
}

mpc_input_delay=""
mpc_steer_tau=""
ready_timeout=180
runtime_health_gate=false
runtime_health_gate_explicit=false
runtime_health_timeout=45
runtime_health_window_sec=8.0
runtime_health_gate_mode="disabled"
smart_mpc=false
fp16_heads=false
recommended=false
speed_30kph=false
speed_60kph_pilot=false
camera_source_5hz=false
camera_source_sensor_tick_sec=0.0
portable_shadow_10hz=false
portable_runtime_bundle=""
portable_runtime_bundle_sha256=""
portable_source_checkpoint_sha256=""
portable_model_config_sha256=""
portable_corpus_fingerprint_sha256=""
portable_rig_file=""
portable_rig_sha256=""
portable_contract_file=""
portable_contract_sha256=""
portable_shadow_device="cpu"
portable_cpu_set=""
portable_input_option_provided=false
declare -A portable_option_seen=()

claim_portable_option() {
  local option="$1"
  if [[ -n "${portable_option_seen[${option}]:-}" ]]; then
    echo "Portable E2E option may be specified only once: ${option}" >&2
    exit 2
  fi
  portable_option_seen["${option}"]=true
}

visualize=false
capture_desktop=false
trajectory_stability=false
tight_corridor=false
geometry_ab_route_corridor_0p2=false
comfortable_deceleration_mps2=""
maximum_longitudinal_acceleration_mps2=""
maximum_lateral_acceleration_mps2=""
target_speed_mps=""
minimum_sustained_speed_mps=""
minimum_sustained_speed_sec=""
maximum_observed_speed_mps=""
maximum_lateral_acceleration_limit_mps2=""
maneuver_lookahead_m=""
maneuver_exit_lookahead_m=""
curvature_speed_preview_m=""
route_curvature_lookahead_m=""
max_route_deviation_m=""
# HH_260906 - Pin and record the fail-closed trajectory-correction ceiling for recommended trials.
maximum_trajectory_correction_m=""
speed_profile_id="baseline"
speed_exposure_mode="not_requested"
model_override=""
sensor_mapping=""
control_ab_pid_i40=false
control_ab_turn_preview_5m=false
# HH_260906 - Preserve the 5 m history while recording the isolated 10 m candidate.
control_ab_turn_preview_10m=false
control_ab_longitudinal_recovery_2p0=false
control_ab_candidate_id="baseline"
geometry_ab_candidate_id="baseline_corridor_0p5"
route_corridor_half_width_m="0.50"
turn_outward_corridor_half_width_m="0.50"
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
    --speed-60kph-pilot)
      speed_60kph_pilot=true
      recommended=true
      shift
      ;;
    --camera-source-5hz)
      camera_source_5hz=true
      camera_source_sensor_tick_sec=0.2
      recommended=true
      shift
      ;;
    --portable-shadow-10hz)
      claim_portable_option "$1"
      portable_shadow_10hz=true
      camera_source_sensor_tick_sec=0.1
      recommended=true
      shift
      ;;
    --portable-runtime-bundle)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_runtime_bundle="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-runtime-bundle-sha256)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_runtime_bundle_sha256="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-source-checkpoint-sha256)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_source_checkpoint_sha256="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-model-config-sha256)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_model_config_sha256="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-corpus-fingerprint-sha256)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_corpus_fingerprint_sha256="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-rig-file)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_rig_file="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-rig-sha256)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_rig_sha256="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-contract-file)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_contract_file="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-contract-sha256)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_contract_sha256="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-device)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_shadow_device="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --portable-cpu-set)
      claim_portable_option "$1"
      [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
      portable_cpu_set="$2"
      portable_input_option_provided=true
      shift 2
      ;;
    --control-ab-pid-i40)
      control_ab_pid_i40=true
      control_ab_candidate_id="pid_i40"
      shift
      ;;
    --control-ab-turn-preview-5m)
      control_ab_turn_preview_5m=true
      control_ab_candidate_id="turn_preview_5m"
      shift
      ;;
    --control-ab-turn-preview-10m)
      control_ab_turn_preview_10m=true
      control_ab_candidate_id="turn_preview_10m"
      shift
      ;;
    --control-ab-longitudinal-recovery-2p0)
      control_ab_longitudinal_recovery_2p0=true
      control_ab_candidate_id="longitudinal_recovery_2p0"
      shift
      ;;
    --geometry-ab-route-corridor-0p2)
      geometry_ab_route_corridor_0p2=true
      geometry_ab_candidate_id="route_corridor_0p2"
      route_corridor_half_width_m="0.20"
      turn_outward_corridor_half_width_m="0.20"
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
      route_corridor_half_width_m="0.20"
      turn_outward_corridor_half_width_m="0.20"
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
    --runtime-health-gate)
      runtime_health_gate=true
      runtime_health_gate_explicit=true
      shift
      ;;
    --runtime-health-timeout)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      runtime_health_timeout="$2"
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
if [[ "${camera_source_5hz}" == "true" && -n "${sensor_mapping}" ]]; then
  echo "--camera-source-5hz and --sensor-mapping are mutually exclusive." >&2
  exit 2
fi
if [[ "${portable_shadow_10hz}" == "true" && "${camera_source_5hz}" == "true" ]]; then
  echo "--portable-shadow-10hz and --camera-source-5hz are mutually exclusive." >&2
  exit 2
fi
if [[ "${portable_shadow_10hz}" == "true" && -n "${sensor_mapping}" ]]; then
  echo "--portable-shadow-10hz and --sensor-mapping are mutually exclusive." >&2
  exit 2
fi
if [[ "${portable_shadow_10hz}" == "true" && "${speed_30kph}" != "true" ]]; then
  echo "--portable-shadow-10hz requires --speed-30kph." >&2
  exit 2
fi
if [[ "${portable_shadow_10hz}" != "true" && "${portable_input_option_provided}" == "true" ]]; then
  echo "Portable E2E pinned inputs require --portable-shadow-10hz." >&2
  exit 2
fi
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  portable_required_values=(
    "${portable_runtime_bundle}"
    "${portable_runtime_bundle_sha256}"
    "${portable_source_checkpoint_sha256}"
    "${portable_model_config_sha256}"
    "${portable_corpus_fingerprint_sha256}"
    "${portable_rig_file}"
    "${portable_rig_sha256}"
    "${portable_contract_file}"
    "${portable_contract_sha256}"
  )
  for value in "${portable_required_values[@]}"; do
    if [[ -z "${value}" ]]; then
      echo "--portable-shadow-10hz requires every pinned bundle/source/config/corpus/rig/contract input." >&2
      exit 2
    fi
  done
  case "${portable_shadow_device}" in
    cpu|cuda:0) ;;
    *) echo "--portable-device must be cpu or logical cuda:0." >&2; exit 2 ;;
  esac
  if [[ "${portable_shadow_device}" == "cuda:0" &&
        ! "${CUDA_VISIBLE_DEVICES:-}" =~ ^GPU-[^,[:space:]]+$ ]]; then
    echo "Portable E2E cuda:0 requires exactly one UUID-pinned CUDA_VISIBLE_DEVICES entry." >&2
    exit 2
  fi
  if [[ -n "${portable_cpu_set}" ]]; then
    if [[ "${portable_shadow_device}" != "cpu" ]]; then
      echo "--portable-cpu-set is valid only with --portable-device cpu." >&2
      exit 2
    fi
    if [[ ! "${portable_cpu_set}" =~ ^[0-9]+(,[0-9]+)*$ ]]; then
      echo "--portable-cpu-set must be a comma-separated list of CPU IDs." >&2
      exit 2
    fi
    # HH_260906 - Reject duplicate or unavailable CPU IDs before creating trial evidence.
    if ! python3 - "${portable_cpu_set}" <<'PY'
import os
import sys

values = [int(value) for value in sys.argv[1].split(",")]
if len(values) != len(set(values)):
    raise SystemExit("--portable-cpu-set must not contain duplicate CPU IDs")
if len(values) < 4:
    raise SystemExit("--portable-cpu-set must contain at least four CPU IDs")
available = set(os.sched_getaffinity(0))
missing = sorted(set(values) - available)
if missing:
    raise SystemExit(f"--portable-cpu-set contains unavailable CPU IDs: {missing}")
PY
    then
      exit 2
    fi
  fi
fi
if [[ "${speed_30kph}" == "true" && "${speed_60kph_pilot}" == "true" ]]; then
  echo "--speed-30kph and --speed-60kph-pilot are mutually exclusive." >&2
  exit 2
fi
control_ab_selection_count=0
[[ "${control_ab_pid_i40}" == "true" ]] && control_ab_selection_count=$((control_ab_selection_count + 1))
[[ "${control_ab_turn_preview_5m}" == "true" ]] && control_ab_selection_count=$((control_ab_selection_count + 1))
[[ "${control_ab_turn_preview_10m}" == "true" ]] && control_ab_selection_count=$((control_ab_selection_count + 1))
[[ "${control_ab_longitudinal_recovery_2p0}" == "true" ]] && control_ab_selection_count=$((control_ab_selection_count + 1))
if (( control_ab_selection_count > 1 )); then
  echo "Select exactly one isolated 30 kph control A/B candidate per trial." >&2
  exit 2
fi
if [[ ( "${control_ab_pid_i40}" == "true" || \
        "${control_ab_turn_preview_5m}" == "true" || \
        "${control_ab_turn_preview_10m}" == "true" || \
        "${control_ab_longitudinal_recovery_2p0}" == "true" ) && \
      "${speed_30kph}" != "true" ]]; then
  echo "Control A/B candidates require --speed-30kph." >&2
  exit 2
fi
if [[ "${geometry_ab_route_corridor_0p2}" == "true" && \
      "${speed_60kph_pilot}" != "true" ]]; then
  echo "--geometry-ab-route-corridor-0p2 requires --speed-60kph-pilot." >&2
  exit 2
fi
if [[ "${speed_30kph}" == "true" && \
      ( "${tight_corridor}" == "true" || "${trajectory_stability}" == "true" ) ]]; then
  echo "--speed-30kph must be screened independently of experimental corridor/filter modes." >&2
  exit 2
fi
if [[ "${speed_60kph_pilot}" == "true" && \
      ( "${tight_corridor}" == "true" || "${trajectory_stability}" == "true" ) ]]; then
  echo "--speed-60kph-pilot must be screened independently of experimental corridor/filter modes." >&2
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

if [[ ( "${speed_30kph}" == "true" || "${speed_60kph_pilot}" == "true" ) &&
      "${camera_source_5hz}" == "true" ]]; then
  runtime_health_gate=true
  runtime_health_gate_mode="automatic_speed_camera_source_5hz"
elif [[ "${portable_shadow_10hz}" == "true" ]]; then
  runtime_health_gate=true
  runtime_health_gate_mode="automatic_speed_portable_shadow_10hz"
elif [[ "${runtime_health_gate_explicit}" == "true" ]]; then
  runtime_health_gate_mode="explicit"
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
  maximum_trajectory_correction_m="15.0"
  speed_profile_id="recommended_9kph_v1"
  if [[ "${speed_30kph}" == "true" ]]; then
    maneuver_lookahead_m="4.0"
    maneuver_exit_lookahead_m="2.5"
    curvature_speed_preview_m="3.0"
    if [[ "${control_ab_turn_preview_5m}" == "true" ]]; then
      curvature_speed_preview_m="5.0"
    elif [[ "${control_ab_turn_preview_10m}" == "true" ]]; then
      curvature_speed_preview_m="10.0"
    fi
    route_curvature_lookahead_m="20.0"
    max_route_deviation_m="1.0"
    speed_profile_id="carla_vad_30kph_v2"
    comfortable_deceleration_mps2="2.0"
    maximum_longitudinal_acceleration_mps2="1.5"
    if [[ "${control_ab_longitudinal_recovery_2p0}" == "true" ]]; then
      # HH_260906 - Preserve actuator limits while increasing only planning-speed recovery after curvature caps.
      maximum_longitudinal_acceleration_mps2="2.0"
    fi
    maximum_lateral_acceleration_mps2="1.2"
    target_speed_mps="8.333333333333334"
    maximum_observed_speed_mps="9.0"
    maximum_lateral_acceleration_limit_mps2="1.8"
  elif [[ "${speed_60kph_pilot}" == "true" ]]; then
    maneuver_lookahead_m="6.0"
    maneuver_exit_lookahead_m="3.5"
    curvature_speed_preview_m="6.0"
    route_curvature_lookahead_m="40.0"
    max_route_deviation_m="1.0"
    speed_profile_id="carla_vad_60kph_straight_pilot_v1"
    comfortable_deceleration_mps2="2.0"
    maximum_longitudinal_acceleration_mps2="1.5"
    maximum_lateral_acceleration_mps2="1.0"
    target_speed_mps="16.666666666666668"
    maximum_observed_speed_mps="18.0"
    maximum_lateral_acceleration_limit_mps2="1.2"
  else
    comfortable_deceleration_mps2="0.60"
  fi
fi

for argument in "${launch_arguments[@]}"; do
  if [[ "${portable_shadow_10hz}" == "true" ]]; then
    case "${argument}" in
      -r|--remap|--ros-args|__node:=*|__ns:=*|*portable_e2e_shadow*|*/planning/portable_e2e/*|*/planning/trajectory*|*/control/command/*|*/vehicle/command/*)
        echo "Portable E2E shadow trials reject node identity, topic, and control remaps: ${argument}" >&2
        exit 2
        ;;
    esac
  fi
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
      use_vad_imu_acceleration:=*|use_fast_vad:=*|vad_use_fp16_heads:=*|use_light_weight_sensor_mapping:=*|rviz:=*|launch_fast_camera_view:=*|use_lateral_controller_param_override:=*|lateral_controller_param_path:=*|use_longitudinal_controller_param_override:=*|longitudinal_controller_param_path:=*|vehicle_cmd_gate_param_path:=*|controller_stop_offset_m:=*|comfortable_deceleration_mps2:=*|maximum_longitudinal_acceleration_mps2:=*|longitudinal_velocity_source:=*|nominal_cruise_speed_mps:=*|maneuver_lookahead_m:=*|maneuver_exit_lookahead_m:=*|route_corridor_half_width_m:=*|turn_inward_corridor_half_width_m:=*|turn_outward_corridor_half_width_m:=*|left_turn_outward_corridor_half_width_m:=*|right_turn_outward_corridor_half_width_m:=*|route_corridor_entry_distance_m:=*|trajectory_lateral_filter_gain:=*|left_turn_trajectory_lateral_filter_gain:=*|right_turn_trajectory_lateral_filter_gain:=*|trajectory_lateral_filter_activation_threshold_m:=*|trajectory_geometry_smoothing_strength:=*|maximum_lateral_acceleration_mps2:=*|curvature_speed_preview_m:=*|route_curvature_lookahead_m:=*|max_route_deviation_m:=*|maximum_trajectory_correction_m:=*|max_candidate_age_sec:=*|candidate_timeout_sec:=*|maximum_speed_mps:=*|raw_vehicle_cmd_converter_config:=*)
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
if [[ "${portable_shadow_10hz}" == "true" && -L "${route_file}" ]]; then
  echo "Portable E2E source route must not be a symlink." >&2
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
  if [[ ( "${control_ab_turn_preview_5m}" == "true" || \
          "${control_ab_turn_preview_10m}" == "true" ) && \
        "${route_scenario}" != "left" && "${route_scenario}" != "right" ]]; then
    echo "Turn-preview control A/B candidates require a left or right route." >&2
    exit 2
  fi
  if [[ "${control_ab_longitudinal_recovery_2p0}" == "true" && \
        "${route_scenario}" != "straight" ]]; then
    echo "--control-ab-longitudinal-recovery-2p0 requires a straight route." >&2
    exit 2
  fi
fi
if [[ "${speed_60kph_pilot}" == "true" ]]; then
  if [[ "${route_scenario}" != "straight" ]]; then
    echo "--speed-60kph-pilot requires a straight route; got ${route_scenario}" >&2
    exit 2
  fi
  speed_exposure_mode="straight_target_required"
  minimum_sustained_speed_mps="15.0"
  minimum_sustained_speed_sec="1.0"
fi

recommended_mpc=""
speed_gate=""
speed_pid=""
cyclonedds_config=""
if [[ "${recommended}" == "true" ]]; then
  package_share="$(ros2 pkg prefix autoware_e2e_vad_launch)/share/autoware_e2e_vad_launch"
  model_override="${package_share}/config/vad_carla_tiny_recommended.param.yaml"
  sensor_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable_imu.yaml"
  if [[ "${camera_source_5hz}" == "true" ]]; then
    sensor_mapping="${package_share}/config/sensor_mapping_vad_fast_imu_camera_source_5hz_best_effort_image_depth1.yaml"
    model_override="${package_share}/config/vad_carla_tiny_camera_source_5hz_best_effort_image_depth1.param.yaml"
    cyclonedds_config="${package_share}/config/cyclonedds_camera_depth1_localhost_v2.xml"
  elif [[ "${portable_shadow_10hz}" == "true" ]]; then
    sensor_mapping="${package_share}/config/sensor_mapping_portable_e2e_10hz.yaml"
    model_override="${package_share}/config/vad_carla_tiny_camera_source_5hz_best_effort_image_depth1.param.yaml"
    cyclonedds_config="${package_share}/config/cyclonedds_camera_depth1_localhost_v2.xml"
  fi
  recommended_mpc="${package_share}/config/mpc_carla_recommended.param.yaml"
  required_profile_files=("${model_override}" "${sensor_mapping}" "${recommended_mpc}")
  if [[ "${camera_source_5hz}" == "true" ]]; then
    required_profile_files+=(
      "${sensor_mapping}.metadata.json"
      "${model_override}.metadata.json"
      "${cyclonedds_config}"
      "${cyclonedds_config}.metadata.json"
    )
  elif [[ "${portable_shadow_10hz}" == "true" ]]; then
    required_profile_files+=(
      "${cyclonedds_config}"
      "${package_share}/launch/portable_e2e_shadow.launch.xml"
    )
  fi
  if [[ "${speed_30kph}" == "true" ]]; then
    speed_gate="${package_share}/config/vehicle_cmd_gate_carla_30kph.param.yaml"
    speed_pid="${package_share}/config/pid_carla_vad_30kph.param.yaml"
    if [[ "${control_ab_pid_i40}" == "true" ]]; then
      speed_pid="${package_share}/config/pid_carla_vad_30kph_i40_ab.param.yaml"
    fi
    required_profile_files+=(
      "${speed_gate}"
      "${speed_gate}.metadata.json"
      "${speed_pid}"
      "${speed_pid}.metadata.json"
    )
  elif [[ "${speed_60kph_pilot}" == "true" ]]; then
    speed_gate="${package_share}/config/vehicle_cmd_gate_carla_60kph_pilot.param.yaml"
    speed_pid="${package_share}/config/pid_carla_vad_60kph_pilot.param.yaml"
    required_profile_files+=(
      "${speed_gate}"
      "${speed_gate}.metadata.json"
      "${speed_pid}"
      "${speed_pid}.metadata.json"
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

if [[ -n "${cyclonedds_config}" ]]; then
  if [[ ! -f "${cyclonedds_config}" ]]; then
    echo "CycloneDDS config file not found: ${cyclonedds_config}" >&2
    exit 2
  fi
  cyclonedds_config="$(realpath -- "${cyclonedds_config}")"
fi

camera_transport_profile_id="legacy_shared_camera_qos"
camera_transport_sensor_mapping_sha256=""
camera_transport_vad_override_sha256=""
camera_transport_cyclonedds_sha256=""
if [[ "${camera_source_5hz}" == "true" ]]; then
  camera_transport_profile_id="carla_vad_camera_source_5hz_best_effort_image_v2"
  camera_transport_sensor_mapping_sha256="$(
    sha256sum -- "${sensor_mapping}" | awk '{print $1}'
  )"
  camera_transport_vad_override_sha256="$(
    sha256sum -- "${model_override}" | awk '{print $1}'
  )"
  camera_transport_cyclonedds_sha256="$(
    sha256sum -- "${cyclonedds_config}" | awk '{print $1}'
  )"
  python3 - "${sensor_mapping}" "${model_override}" \
    "${sensor_mapping}.metadata.json" "${model_override}.metadata.json" \
    "${cyclonedds_config}" "${cyclonedds_config}.metadata.json" \
    "${camera_transport_profile_id}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

import yaml

mapping_path, model_path, mapping_meta_path, model_meta_path, cyclone_path, cyclone_meta_path = map(
    Path, sys.argv[1:7]
)
profile_id = sys.argv[7]

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

mapping = yaml.safe_load(mapping_path.read_text(encoding="utf-8"))
model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
mapping_meta = json.loads(mapping_meta_path.read_text(encoding="utf-8"))
model_meta = json.loads(model_meta_path.read_text(encoding="utf-8"))
cyclone_meta = json.loads(cyclone_meta_path.read_text(encoding="utf-8"))
cameras = [
    value
    for value in mapping["sensor_mappings"].values()
    if value.get("carla_type") == "sensor.camera.rgb"
]
if len(cameras) != 6:
    raise SystemExit("camera transport profile must contain exactly six cameras")
for camera in cameras:
    ros = camera["ros_config"]
    if (
        ros.get("image_qos_profile") != "best_effort_depth_1"
        or ros.get("camera_info_qos_profile") != "reliable"
        or ros.get("frequency_hz") != 5
        or camera["parameters"].get("sensor_tick") != 0.2
    ):
        raise SystemExit("camera transport profile violates the split QoS contract")
for sensor_name in ("tamagawa/imu_link", "gnss_link"):
    if mapping["sensor_mappings"][sensor_name]["ros_config"].get("qos_profile") != "reliable":
        raise SystemExit(f"{sensor_name} must retain reliable QoS")
sync = model["/**"]["ros__parameters"]["sync_params"]
if sync.get("image_reliability") != "best_effort" or sync.get("image_queue_depth") != 1:
    raise SystemExit("VAD image subscriber must request Best-Effort KEEP_LAST depth 1")
for metadata, path in (
    (mapping_meta, mapping_path),
    (model_meta, model_path),
    (cyclone_meta, cyclone_path),
):
    if (
        metadata.get("profile_id") != profile_id
        or metadata.get("effective_file_sha256") != digest(path)
    ):
        raise SystemExit(f"transport metadata/hash mismatch for {path}")
if (
    cyclone_meta.get("network_interface") != "lo"
    or cyclone_meta.get("ros_localhost_only") is not False
    or cyclone_meta.get("rmw_implementation") != "rmw_cyclonedds_cpp"
):
    raise SystemExit("CycloneDDS localhost transport metadata mismatch")
PY
fi
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  camera_transport_profile_id="portable_e2e_exact_bundle_10hz_v2"
  camera_transport_sensor_mapping_sha256="$(
    sha256sum -- "${sensor_mapping}" | awk '{print $1}'
  )"
  camera_transport_vad_override_sha256="$(
    sha256sum -- "${model_override}" | awk '{print $1}'
  )"
  camera_transport_cyclonedds_sha256="$(
    sha256sum -- "${cyclonedds_config}" | awk '{print $1}'
  )"
fi
if [[ "${camera_source_5hz}" == "true" || "${portable_shadow_10hz}" == "true" ]]; then
  # HH_260906 - Let the pinned CycloneDDS file own the single loopback selection.
  export ROS_LOCALHOST_ONLY=0
  export AUTOWARE_E2E_PINNED_CYCLONEDDS_URI="file://${cyclonedds_config}"
  export AUTOWARE_E2E_PINNED_CYCLONEDDS_SHA256="${camera_transport_cyclonedds_sha256}"
  export CYCLONEDDS_URI="${AUTOWARE_E2E_PINNED_CYCLONEDDS_URI}"
fi

raw_vehicle_cmd_converter_config="$(
  ros2 pkg prefix autoware_carla_interface
)/share/autoware_carla_interface/config/raw_vehicle_cmd_converter.param.yaml"
# HH_260906 - Bind each route artifact to the ordered camera-bundle dispatcher source.
carla_camera_bundle_dispatch_source="${root}/src/universe/autoware_universe/simulator/autoware_carla_interface/src/autoware_carla_interface/modules/carla_wrapper.py"
carla_camera_bundle_dispatch_runtime="$(
  python3 - <<'PY'
from importlib.util import find_spec
from pathlib import Path

spec = find_spec("autoware_carla_interface.modules.carla_wrapper")
if spec is None or spec.origin is None:
    raise SystemExit("cannot resolve the installed CARLA camera-bundle dispatcher")
print(Path(spec.origin).resolve())
PY
)"
if [[ ! -f "${carla_camera_bundle_dispatch_source}" ||
      ! -f "${carla_camera_bundle_dispatch_runtime}" ]]; then
  echo "CARLA camera-bundle dispatcher source or runtime file is missing" >&2
  exit 1
fi
carla_camera_bundle_dispatch_sha256="$(
  sha256sum -- "${carla_camera_bundle_dispatch_source}" | awk '{print $1}'
)"
if [[ "$(sha256sum -- "${carla_camera_bundle_dispatch_runtime}" | awk '{print $1}')" != \
      "${carla_camera_bundle_dispatch_sha256}" ]]; then
  echo "Installed CARLA camera-bundle dispatcher does not match the source" >&2
  exit 1
fi
runtime_health_probe="${root}/scripts/e2e/probe_runtime_health.py"
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
if [[ "${runtime_health_gate}" == "true" && ! -f "${runtime_health_probe}" ]]; then
  echo "Runtime health probe not found: ${runtime_health_probe}" >&2
  exit 2
fi
if [[ ! "${ready_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ready timeout must be a positive integer" >&2
  exit 2
fi
if ! python3 - "${runtime_health_timeout}" "${runtime_health_window_sec}" <<'PY'
import math
import sys

try:
    timeout = float(sys.argv[1])
    window = float(sys.argv[2])
except ValueError:
    raise SystemExit(1)
minimum = window + 0.1 + 2.0
if not math.isfinite(timeout) or timeout < minimum:
    raise SystemExit(1)
PY
then
  echo "runtime health timeout must be finite and at least 10.1 seconds" >&2
  exit 2
fi
portable_shadow_launch=""
portable_shadow_installed_launch=""
portable_shadow_validator=""
portable_shadow_analyzer=""
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  # HH_260906 - Validate the regular source while separately binding ROS symlink-install resolution.
  portable_shadow_launch="${root}/autoware_e2e_vad_launch/launch/portable_e2e_shadow.launch.xml"
  portable_shadow_installed_launch="${package_share}/launch/portable_e2e_shadow.launch.xml"
  portable_shadow_validator="${root}/scripts/e2e/validate_portable_shadow_trial.py"
  portable_shadow_analyzer="${root}/scripts/e2e/analyze_portable_e2e_shadow.py"
  portable_file_values=(
    "${portable_runtime_bundle}"
    "${portable_rig_file}"
    "${portable_contract_file}"
    "${portable_shadow_launch}"
    "${portable_shadow_validator}"
    "${portable_shadow_analyzer}"
  )
  for file in "${portable_file_values[@]}"; do
    if [[ -L "${file}" || ! -f "${file}" ]]; then
      echo "Portable E2E inputs must be regular non-symlink files: ${file}" >&2
      exit 2
    fi
  done
  if [[ ! -e "${portable_shadow_installed_launch}" ]]; then
    echo "Installed Portable E2E shadow launch is missing: ${portable_shadow_installed_launch}" >&2
    exit 2
  fi
  portable_runtime_bundle="$(realpath -- "${portable_runtime_bundle}")"
  portable_rig_file="$(realpath -- "${portable_rig_file}")"
  portable_contract_file="$(realpath -- "${portable_contract_file}")"
  portable_hash_values=(
    "${portable_runtime_bundle_sha256}"
    "${portable_source_checkpoint_sha256}"
    "${portable_model_config_sha256}"
    "${portable_corpus_fingerprint_sha256}"
    "${portable_rig_sha256}"
    "${portable_contract_sha256}"
  )
  for digest in "${portable_hash_values[@]}"; do
    if [[ ! "${digest}" =~ ^[0-9a-f]{64}$ ]]; then
      echo "Portable E2E provenance values must be lowercase SHA-256 strings." >&2
      exit 2
    fi
  done
  if [[ "$(sha256sum -- "${portable_rig_file}" | awk '{print $1}')" != "${portable_rig_sha256}" ||
        "$(sha256sum -- "${portable_contract_file}" | awk '{print $1}')" != "${portable_contract_sha256}" ]]; then
    echo "Portable E2E rig or Common10 contract SHA-256 mismatch." >&2
    exit 2
  fi
  CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
    python3 -m portable_e2e.runtime_weight_bundle verify \
      --bundle "${portable_runtime_bundle}" \
      --bundle-sha256 "${portable_runtime_bundle_sha256}" \
      --source-checkpoint-sha256 "${portable_source_checkpoint_sha256}" \
      --model-config-sha256 "${portable_model_config_sha256}" \
      --corpus-fingerprint-sha256 "${portable_corpus_fingerprint_sha256}"
fi
desktop_dimensions=""
desktop_display=""
capture_output_width_px=1920
capture_output_height_px=1080
capture_output_dimensions="${capture_output_width_px}x${capture_output_height_px}"
capture_framerate_fps=5
capture_filter_threads=1
capture_encoder="libx264"
capture_encoder_preset="ultrafast"
capture_encoder_crf=20
capture_encoder_threads=2
capture_ffmpeg_thread_policy="bounded_ffmpeg_workers_v1"
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
  python3 - "${capture_rviz_config}" "${portable_shadow_10hz}" <<'PY'
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
if sys.argv[2] == "true":
    required_topics.update(
        {
            "/planning/portable_e2e/shadow_path",
            "/planning/portable_e2e/shadow_trajectory",
        }
    )
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
front_camera = named_display(
    config["Visualization Manager"]["Displays"], "VAD Front Camera"
)
front_camera_topic = (
    front_camera.get("Topic", {}) if isinstance(front_camera, dict) else {}
)
if (
    front_camera_topic.get("Reliability Policy") != "Best Effort"
    or front_camera_topic.get("History Policy") != "Keep Last"
    or front_camera_topic.get("Depth") != 1
):
    raise SystemExit(
        "RViz front-camera reader must request Best-Effort KEEP_LAST depth 1"
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

conflicts="$(ros2 node list --no-daemon 2>/dev/null | grep -E '/(vad_route_manager|autoware_carla_interface|vad_carla_tiny|portable_e2e_shadow)$' || true)"
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
route_aligned_this_trial=false
if [[ -f "${map_bundle}" ]]; then
  cp -- "${source_route_file}" "${output_dir}/source_route.json"
  cp -- "${map_bundle}" "${output_dir}/map_bundle.json"
  python3 scripts/e2e/align_carla_route_to_map.py \
    "${source_route_file}" "${map_bundle}" \
    --output "${output_dir}/aligned_route.json" --json > \
    "${output_dir}/route_alignment.json"
  route_file="${output_dir}/aligned_route.json"
  route_aligned_this_trial=true
fi

portable_shadow_route_sha256=""
portable_shadow_binding_sha256=""
portable_shadow_runtime_launch=""
portable_shadow_runtime_launch_sha256=""
portable_shadow_declared_map_id=""
portable_shadow_carla_probe_sha256=""
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  if [[ "${route_aligned_this_trial}" != "true" ]]; then
    echo "Portable E2E shadow requires a newly materialized full-map aligned route." >&2
    exit 2
  fi
  mkdir -p "${output_dir}/portable_shadow_provenance"
  portable_shadow_runtime_launch="${output_dir}/portable_shadow_provenance/runtime_shadow.launch.xml"
  python3 "${portable_shadow_validator}" \
    --output-dir "${output_dir}" \
    --source-route "${source_route_file}" \
    --aligned-route "${route_file}" \
    --route-alignment "${output_dir}/route_alignment.json" \
    --map-bundle "${output_dir}/map_bundle.json" \
    --carla-probe "${output_dir}/carla_preflight_health.json" \
    --expected-map "${carla_expected_map}" \
    --sensor-mapping "${sensor_mapping}" \
    --vad-model-override "${model_override}" \
    --cyclonedds-config "${cyclonedds_config}" \
    --shadow-launch "${portable_shadow_launch}" \
    --installed-shadow-launch "${portable_shadow_installed_launch}" \
    --shadow-launch-snapshot "${portable_shadow_runtime_launch}" \
    --runtime-bundle "${portable_runtime_bundle}" \
    --runtime-bundle-sha256 "${portable_runtime_bundle_sha256}" \
    --source-checkpoint-sha256 "${portable_source_checkpoint_sha256}" \
    --model-config-sha256 "${portable_model_config_sha256}" \
    --corpus-fingerprint-sha256 "${portable_corpus_fingerprint_sha256}" \
    --contract-file "${portable_contract_file}" \
    --contract-sha256 "${portable_contract_sha256}" \
    --rig-file "${portable_rig_file}" \
    --rig-sha256 "${portable_rig_sha256}" \
    --device "${portable_shadow_device}" \
    --output "${output_dir}/portable_shadow_provenance/trial_binding.json" > \
    "${output_dir}/portable_shadow_provenance/validation.log"
  portable_shadow_route_sha256="$(sha256sum -- "${route_file}" | awk '{print $1}')"
  portable_shadow_binding_sha256="$(
    sha256sum -- "${output_dir}/portable_shadow_provenance/trial_binding.json" |
      awk '{print $1}'
  )"
  portable_shadow_runtime_launch_sha256="$(
    sha256sum -- "${portable_shadow_runtime_launch}" | awk '{print $1}'
  )"
  portable_shadow_declared_map_id="$(
    python3 - "${output_dir}/portable_shadow_provenance/trial_binding.json" <<'PY'
import json
from pathlib import Path
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(payload["route_binding"]["declared_map_id"])
PY
  )"
  portable_shadow_carla_probe_sha256="$(
    sha256sum -- "${output_dir}/carla_preflight_health.json" | awk '{print $1}'
  )"
fi

python3 scripts/e2e/capture_raw_vehicle_cmd_converter_provenance.py \
  --config "${raw_vehicle_cmd_converter_config}" \
  --output-dir "${output_dir}/actuation_config_provenance"
if [[ "${speed_30kph}" == "true" || "${speed_60kph_pilot}" == "true" ]]; then
  actuation_coverage_arguments=(
    --provenance-dir "${output_dir}/actuation_config_provenance"
    --profile-id "${speed_profile_id}"
    --target-speed-mps "${target_speed_mps}"
    --output "${output_dir}/actuation_map_coverage.json"
  )
  if [[ "${speed_60kph_pilot}" == "true" ]]; then
    actuation_coverage_arguments+=(--allow-target-envelope-beyond-axis)
  fi
  python3 scripts/e2e/analyze_actuation_map_coverage.py \
    "${actuation_coverage_arguments[@]}"
fi
printf '%s\n' "${launch_arguments[@]}" > "${output_dir}/launch_args.txt"
printf 'ROS_DOMAIN_ID=%s\nCARLA_HOST=%s\nCARLA_PORT=%s\n' \
  "${ROS_DOMAIN_ID}" "${carla_host}" "${carla_port}" > "${output_dir}/runtime.env"
printf 'CARLA_LIFECYCLE=cold_start_owned_process_group_per_trial\nCARLA_GENERATION_ID=%s\nCARLA_EXPECTED_MAP=%s\nCARLA_OWNER_PID=%s\nCARLA_OWNER_PGID=%s\nCARLA_SERVER_LOG=%s\nCARLA_MATRIX_OWNED=%s\n' \
  "${carla_generation_id}" "${carla_expected_map}" "${carla_owner_pid}" \
  "${carla_owner_pgid}" "${carla_server_log}" "${matrix_owned_carla}" >> \
  "${output_dir}/runtime.env"
printf 'SOURCE_ROUTE_FILE=%s\nEFFECTIVE_ROUTE_FILE=%s\nFULL_MAP_PATH=%s\n' \
  "${source_route_file}" "${route_file}" "${full_map_path}" >> "${output_dir}/runtime.env"
printf 'CARLA_CAMERA_BUNDLE_DISPATCH_POLICY=oldest_complete_source_order_v1\nCARLA_CAMERA_BUNDLE_DISPATCH_SOURCE_FILE=%s\nCARLA_CAMERA_BUNDLE_DISPATCH_RUNTIME_FILE=%s\nCARLA_CAMERA_BUNDLE_DISPATCH_SHA256=%s\n' \
  "${carla_camera_bundle_dispatch_source}" \
  "${carla_camera_bundle_dispatch_runtime}" \
  "${carla_camera_bundle_dispatch_sha256}" >> "${output_dir}/runtime.env"
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  printf 'PORTABLE_SHADOW_ENABLED=true\nPORTABLE_SHADOW_MODE=shadow_only\nPORTABLE_SHADOW_CONTROLLING_PLANNER=autoware_vad\nPORTABLE_SHADOW_VEHICLE_CONTROL_APPROVED=false\nPORTABLE_SHADOW_CANONICAL_PUBLICATION_ALLOWED=false\nPORTABLE_SHADOW_REMAPS_ALLOWED=false\nPORTABLE_SHADOW_DEVICE=%s\nPORTABLE_SHADOW_DECLARED_MAP_ID=%s\nPORTABLE_SHADOW_OBSERVED_MAP_ID=%s\nPORTABLE_SHADOW_OBSERVED_MAP_SOURCE=carla_python_api_world_get_map\nPORTABLE_SHADOW_CARLA_MAP_PROBE_FILE=%s\nPORTABLE_SHADOW_CARLA_MAP_PROBE_SHA256=%s\nPORTABLE_SHADOW_EFFECTIVE_CAMERA_HZ=10\nPORTABLE_SHADOW_MAPPING_REQUESTED_CAP_HZ=11\nPORTABLE_SHADOW_ROUTE_FILE=%s\nPORTABLE_SHADOW_ROUTE_SHA256=%s\nPORTABLE_SHADOW_ROUTE_MATERIALIZED_THIS_TRIAL=true\nPORTABLE_SHADOW_BINDING_FILE=%s\nPORTABLE_SHADOW_BINDING_SHA256=%s\nPORTABLE_SHADOW_RUNTIME_LAUNCH_FILE=%s\nPORTABLE_SHADOW_RUNTIME_LAUNCH_SHA256=%s\nPORTABLE_SHADOW_RUNTIME_LAUNCH_DIRECT=true\nPORTABLE_SHADOW_RUNTIME_BUNDLE_FILE=%s\nPORTABLE_SHADOW_RUNTIME_BUNDLE_SHA256=%s\nPORTABLE_SHADOW_SOURCE_CHECKPOINT_SHA256=%s\nPORTABLE_SHADOW_MODEL_CONFIG_SHA256=%s\nPORTABLE_SHADOW_CORPUS_FINGERPRINT_SHA256=%s\nPORTABLE_SHADOW_RIG_FILE=%s\nPORTABLE_SHADOW_RIG_SHA256=%s\nPORTABLE_SHADOW_CONTRACT_FILE=%s\nPORTABLE_SHADOW_CONTRACT_SHA256=%s\n' \
    "${portable_shadow_device}" "${portable_shadow_declared_map_id}" \
    "${portable_shadow_declared_map_id}" \
    "${output_dir}/carla_preflight_health.json" \
    "${portable_shadow_carla_probe_sha256}" "${route_file}" \
    "${portable_shadow_route_sha256}" \
    "${output_dir}/portable_shadow_provenance/trial_binding.json" \
    "${portable_shadow_binding_sha256}" "${portable_shadow_runtime_launch}" \
    "${portable_shadow_runtime_launch_sha256}" "${portable_runtime_bundle}" \
    "${portable_runtime_bundle_sha256}" \
    "${portable_source_checkpoint_sha256}" \
    "${portable_model_config_sha256}" \
    "${portable_corpus_fingerprint_sha256}" "${portable_rig_file}" \
    "${portable_rig_sha256}" "${portable_contract_file}" \
    "${portable_contract_sha256}" >> "${output_dir}/runtime.env"
  if [[ "${portable_shadow_device}" == "cpu" ]]; then
    portable_cpu_set_label="${portable_cpu_set:-inherited}"
    portable_cpu_affinity_label="inherited"
    if [[ -n "${portable_cpu_set}" ]]; then
      portable_cpu_affinity_label="taskset_cpu_list"
    fi
    printf 'PORTABLE_SHADOW_CPU_SET=%s\nPORTABLE_SHADOW_CPU_AFFINITY=%s\nPORTABLE_SHADOW_OMP_NUM_THREADS=4\nPORTABLE_SHADOW_MKL_NUM_THREADS=4\nPORTABLE_SHADOW_OPENBLAS_NUM_THREADS=1\nPORTABLE_SHADOW_NUMEXPR_NUM_THREADS=1\n' \
      "${portable_cpu_set_label}" "${portable_cpu_affinity_label}" >> \
      "${output_dir}/runtime.env"
  else
    printf 'PORTABLE_SHADOW_CPU_SET=not_applicable\nPORTABLE_SHADOW_CPU_AFFINITY=not_applicable\nPORTABLE_SHADOW_OMP_NUM_THREADS=not_applicable\nPORTABLE_SHADOW_MKL_NUM_THREADS=not_applicable\nPORTABLE_SHADOW_OPENBLAS_NUM_THREADS=not_applicable\nPORTABLE_SHADOW_NUMEXPR_NUM_THREADS=not_applicable\n' >> \
      "${output_dir}/runtime.env"
  fi
else
  printf 'PORTABLE_SHADOW_ENABLED=false\n' >> "${output_dir}/runtime.env"
fi
printf 'VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS=1\nVAD_ROUTE_MANAGER_OMP_NUM_THREADS=1\nVAD_ROUTE_MANAGER_MKL_NUM_THREADS=1\nVAD_ROUTE_MANAGER_NUMEXPR_NUM_THREADS=1\n' >> \
  "${output_dir}/runtime.env"
printf 'RECOMMENDED=%s\nVISUALIZE=%s\nCAPTURE_DESKTOP=%s\nTIGHT_CORRIDOR_CANDIDATE=%s\nTRAJECTORY_STABILITY_CANDIDATE=%s\nSMART_MPC=%s\nFP16_HEADS=%s\n' \
  "${recommended}" "${visualize}" "${capture_desktop}" "${tight_corridor}" "${trajectory_stability}" "${smart_mpc}" "${fp16_heads}" >> \
  "${output_dir}/runtime.env"
camera_ros_publish_hz=5.0
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  camera_ros_publish_hz=10.0
fi
printf 'CAMERA_SOURCE_5HZ=%s\nCAMERA_SOURCE_SENSOR_TICK_SEC=%s\nCAMERA_ROS_PUBLISH_HZ=%s\n' \
  "${camera_source_5hz}" "${camera_source_sensor_tick_sec}" \
  "${camera_ros_publish_hz}" >> \
  "${output_dir}/runtime.env"
bounded_camera_transport=false
if [[ "${camera_source_5hz}" == "true" || "${portable_shadow_10hz}" == "true" ]]; then
  bounded_camera_transport=true
fi
printf 'CAMERA_TRANSPORT_PROFILE_ID=%s\nCAMERA_IMAGE_PUBLISH_QOS=%s\nCAMERA_IMAGE_PUBLISH_HISTORY=%s\nCAMERA_IMAGE_PUBLISH_DEPTH=%s\nCAMERA_INFO_PUBLISH_QOS=%s\nCAMERA_INFO_PUBLISH_DEPTH=%s\nVAD_IMAGE_SUBSCRIPTION_QOS=%s\nVAD_IMAGE_SUBSCRIPTION_DEPTH=%s\nRVIZ_IMAGE_SUBSCRIPTION_QOS=best_effort\nRVIZ_IMAGE_SUBSCRIPTION_DEPTH=%s\nRMW_IMPLEMENTATION=%s\nROS_LOCALHOST_ONLY=%s\nCYCLONEDDS_URI=%s\nCAMERA_TRANSPORT_SENSOR_MAPPING_SHA256=%s\nCAMERA_TRANSPORT_VAD_OVERRIDE_SHA256=%s\nCAMERA_TRANSPORT_CYCLONEDDS_SHA256=%s\n' \
  "${camera_transport_profile_id}" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf best_effort || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf keep_last || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf 1 || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf reliable || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf 1 || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf best_effort || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf 1 || printf inherited)" \
  "$([[ "${bounded_camera_transport}" == "true" ]] && printf 1 || printf inherited)" \
  "${RMW_IMPLEMENTATION:-}" "${ROS_LOCALHOST_ONLY:-}" "${CYCLONEDDS_URI:-}" \
  "${camera_transport_sensor_mapping_sha256}" \
  "${camera_transport_vad_override_sha256}" \
  "${camera_transport_cyclonedds_sha256}" >> "${output_dir}/runtime.env"
printf 'CONTROL_AB_CANDIDATE_ID=%s\nCONTROL_AB_PID_I40=%s\nCONTROL_AB_TURN_PREVIEW_5M=%s\nCONTROL_AB_TURN_PREVIEW_10M=%s\nCONTROL_AB_TURN_PREVIEW_BASELINE_M=3.0\nCONTROL_AB_TURN_PREVIEW_5M_CANDIDATE_M=5.0\nCONTROL_AB_TURN_PREVIEW_10M_CANDIDATE_M=10.0\nCONTROL_AB_LONGITUDINAL_RECOVERY_2P0=%s\nCONTROL_AB_LONGITUDINAL_RECOVERY_BASELINE_MPS2=1.5\nCONTROL_AB_LONGITUDINAL_RECOVERY_CANDIDATE_MPS2=2.0\nCONTROL_AB_ACTUATOR_ACCELERATION_LIMITS_UNCHANGED=true\nCONTROL_AB_ISOLATED_SINGLE_KNOB=true\n' \
  "${control_ab_candidate_id}" "${control_ab_pid_i40}" \
  "${control_ab_turn_preview_5m}" \
  "${control_ab_turn_preview_10m}" \
  "${control_ab_longitudinal_recovery_2p0}" >> "${output_dir}/runtime.env"
printf 'GEOMETRY_AB_CANDIDATE_ID=%s\nGEOMETRY_AB_ROUTE_CORRIDOR_0P2=%s\nGEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M=0.50\nGEOMETRY_AB_ROUTE_CORRIDOR_CANDIDATE_M=0.20\nGEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB=true\nGEOMETRY_AB_PARAMETER_CHANGE_COUNT=2\nGEOMETRY_AB_COUPLED_PARAMETER_REASON=turn_width_must_not_exceed_route_width\nGEOMETRY_AB_ROUTE_SCOPE=straight_only\nROUTE_CORRIDOR_HALF_WIDTH_M=%s\nTURN_OUTWARD_CORRIDOR_HALF_WIDTH_M=%s\n' \
  "${geometry_ab_candidate_id}" "${geometry_ab_route_corridor_0p2}" \
  "${route_corridor_half_width_m}" "${turn_outward_corridor_half_width_m}" >> \
  "${output_dir}/runtime.env"
runtime_health_probe_sha256="$(sha256sum -- "${runtime_health_probe}" | awk '{print $1}')"
printf 'RUNTIME_HEALTH_GATE_ENABLED=%s\nRUNTIME_HEALTH_GATE_MODE=%s\nRUNTIME_HEALTH_TIMEOUT_SEC=%s\nRUNTIME_HEALTH_WINDOW_SEC=%s\nRUNTIME_HEALTH_REQUIRED_CONSECUTIVE_PASSES=3\nRUNTIME_HEALTH_EVIDENCE_FILE=runtime_health.json\nRUNTIME_HEALTH_PROBE_FILE=%s\nRUNTIME_HEALTH_PROBE_SHA256=%s\nRUNTIME_HEALTH_GATE_PHASE=after_optional_rviz_recorder_before_rosbag_and_engagement\n' \
  "${runtime_health_gate}" "${runtime_health_gate_mode}" \
  "${runtime_health_timeout}" "${runtime_health_window_sec}" \
  "${runtime_health_probe}" "${runtime_health_probe_sha256}" >> \
  "${output_dir}/runtime.env"
if [[ "${capture_desktop}" == "true" ]]; then
  printf 'RVIZ_CAPTURE_CAMERA_SOURCE=rviz_embedded_vad_front_camera\nRVIZ_CAPTURE_EXTERNAL_CAMERA_VIEW=false\nRVIZ_CAPTURE_SOURCE=ffmpeg_x11grab_owned_window_v1\nRVIZ_CAPTURE_ROOT=false\nRVIZ_CAPTURE_SHELL_SURFACES_EXCLUDED=true\nRVIZ_CAPTURE_SCALE_APPLIED=false\nRVIZ_CAPTURE_OCCLUSION_GUARD=owned_rviz_window_only_v1\nRVIZ_CAPTURE_OUTPUT_WIDTH_PX=%s\nRVIZ_CAPTURE_OUTPUT_HEIGHT_PX=%s\nRVIZ_CAPTURE_FFMPEG_INPUT_FORMAT=x11grab\nRVIZ_CAPTURE_FFMPEG_FRAMERATE_FPS=%s\nRVIZ_CAPTURE_FFMPEG_FILTER_THREADS=%s\nRVIZ_CAPTURE_FFMPEG_ENCODER=%s\nRVIZ_CAPTURE_FFMPEG_PRESET=%s\nRVIZ_CAPTURE_FFMPEG_CRF=%s\nRVIZ_CAPTURE_FFMPEG_ENCODER_THREADS=%s\nRVIZ_CAPTURE_FFMPEG_PIXEL_FORMAT=yuv420p\nRVIZ_CAPTURE_FFMPEG_THREAD_POLICY=%s\n' \
    "${capture_output_width_px}" "${capture_output_height_px}" \
    "${capture_framerate_fps}" "${capture_filter_threads}" \
    "${capture_encoder}" "${capture_encoder_preset}" \
    "${capture_encoder_crf}" "${capture_encoder_threads}" \
    "${capture_ffmpeg_thread_policy}" >> \
    "${output_dir}/runtime.env"
fi
printf 'VSCODE_SNAP_GUI_ENV_SANITIZED=%s\n' \
  "${vscode_snap_gui_env_sanitized}" >> "${output_dir}/runtime.env"
printf 'SPEED_30KPH=%s\nSPEED_60KPH_PILOT=%s\nSPEED_PROFILE_ID=%s\nROUTE_SCENARIO=%s\nSPEED_EXPOSURE_MODE=%s\n' \
  "${speed_30kph}" "${speed_60kph_pilot}" "${speed_profile_id}" "${route_scenario}" \
  "${speed_exposure_mode}" >> "${output_dir}/runtime.env"
if [[ -n "${maneuver_lookahead_m}" ]]; then
  printf 'MANEUVER_LOOKAHEAD_M=%s\nVAD_IMU_ACCELERATION_ENABLED=true\n' \
    "${maneuver_lookahead_m}" >> "${output_dir}/runtime.env"
fi
if [[ "${speed_30kph}" == "true" || "${speed_60kph_pilot}" == "true" ]]; then
  if [[ "${speed_30kph}" == "true" ]]; then
    printf 'TARGET_SPEED_MPS=%s\nTARGET_SPEED_KPH=30.0\n' \
      "${target_speed_mps}" >> "${output_dir}/runtime.env"
  else
    printf 'TARGET_SPEED_MPS=%s\nTARGET_SPEED_KPH=60.0\n' \
      "${target_speed_mps}" >> "${output_dir}/runtime.env"
  fi
  printf 'MINIMUM_SUSTAINED_SPEED_MPS=%s\nMINIMUM_SUSTAINED_SPEED_SEC=%s\nMAXIMUM_OBSERVED_SPEED_MPS=%s\nMAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2=%s\nMAXIMUM_LONGITUDINAL_ACCELERATION_MPS2=%s\nMAXIMUM_LATERAL_ACCELERATION_MPS2=%s\nMAXIMUM_SPEED_SAMPLE_GAP_SEC=0.25\nCONTROLLER_STOP_OFFSET_M=0.60\nMANEUVER_EXIT_LOOKAHEAD_M=%s\nCURVATURE_SPEED_PREVIEW_M=%s\nROUTE_CURVATURE_LOOKAHEAD_M=%s\nMAX_ROUTE_DEVIATION_M=%s\nMAXIMUM_TRAJECTORY_CORRECTION_M=%s\nMAX_CANDIDATE_AGE_SEC=0.5\nCANDIDATE_TIMEOUT_SEC=1.5\nLONGITUDINAL_SPEED_SOURCE=explicit_simulation_profile\nLONGITUDINAL_ACCELERATION_ROLE=trajectory_internal_curve_exit_cap\nLONGITUDINAL_PID_MAX_OUT_MPS2=1.5\nLONGITUDINAL_PID_MAX_P_EFFORT_MPS2=1.5\nCOMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2=1.5\nVAD_CRUISE_VELOCITY_EVALUATED=false\nVAD_HARD_STOP_SENTINEL_PRESERVED=true\nVAD_VELOCITY_EVALUATED=false\nVAD_GEOMETRY_EVALUATED=true\nVAD_GEOMETRY_SOURCE=true\nSPEED_LIMIT_SOURCE=explicit_simulation_profile\nREAL_VEHICLE_READY=false\n' \
    "${minimum_sustained_speed_mps}" \
    "${minimum_sustained_speed_sec}" "${maximum_observed_speed_mps}" \
    "${maximum_lateral_acceleration_limit_mps2}" \
    "${maximum_longitudinal_acceleration_mps2}" \
    "${maximum_lateral_acceleration_mps2}" "${maneuver_exit_lookahead_m}" \
    "${curvature_speed_preview_m}" "${route_curvature_lookahead_m}" \
    "${max_route_deviation_m}" "${maximum_trajectory_correction_m}" >> \
    "${output_dir}/runtime.env"
fi
if [[ "${speed_60kph_pilot}" == "true" ]]; then
  printf 'SIMULATION_ONLY_EXPLORATORY=true\nROUTE_SCOPE=straight_only\n' >> \
    "${output_dir}/runtime.env"
fi
if [[ -f "${output_dir}/actuation_map_coverage.json" ]]; then
  python3 - "${output_dir}/actuation_map_coverage.json" <<'PY' >> \
    "${output_dir}/runtime.env"
import json
from pathlib import Path
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
fields = {
    "ACTUATION_MAP_COVERAGE_STATUS": payload["status"],
    "ACTUATION_MAP_TARGET_ENVELOPE_CLASSIFICATION": payload[
        "target_envelope_classification"
    ],
    "ACTUATION_MAP_VELOCITY_AXIS_MAXIMUM_MPS": payload[
        "map_velocity_axis_maximum_mps"
    ],
    "ACTUATION_TARGET_WITHIN_MAP_VELOCITY_AXIS": str(
        payload["target_within_map_velocity_axis"]
    ).lower(),
}
for key, value in fields.items():
    print(f"{key}={value}")
PY
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
elif [[ "${speed_60kph_pilot}" == "true" ]]; then
  if [[ "${geometry_ab_route_corridor_0p2}" == "true" ]]; then
    validation_state="carla_60kph_geometry_ab_route_corridor_0p2_exploratory"
  else
    validation_state="carla_60kph_straight_pilot_v1_exploratory"
  fi
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
if [[ "${camera_source_5hz}" == "true" ]]; then
  cyclonedds_metadata="${cyclonedds_config}.metadata.json"
  cyclonedds_metadata_sha256="$(
    sha256sum -- "${cyclonedds_metadata}" | awk '{print $1}'
  )"
  mkdir -p "${output_dir}/camera_transport_provenance"
  cp -- "${cyclonedds_config}" \
    "${output_dir}/camera_transport_provenance/cyclonedds.xml"
  cp -- "${cyclonedds_metadata}" \
    "${output_dir}/camera_transport_provenance/cyclonedds.xml.metadata.json"
  printf '%s  %s\n%s  %s\n' \
    "${camera_transport_cyclonedds_sha256}" "cyclonedds.xml" \
    "${cyclonedds_metadata_sha256}" "cyclonedds.xml.metadata.json" > \
    "${output_dir}/camera_transport_provenance/SHA256SUMS"
elif [[ "${portable_shadow_10hz}" == "true" ]]; then
  mkdir -p "${output_dir}/camera_transport_provenance"
  cp -- "${cyclonedds_config}" \
    "${output_dir}/camera_transport_provenance/cyclonedds.xml"
  printf '%s  %s\n' \
    "${camera_transport_cyclonedds_sha256}" "cyclonedds.xml" > \
    "${output_dir}/camera_transport_provenance/SHA256SUMS"
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
if [[ -n "${speed_gate}" ]]; then
  speed_gate_sha256="$(sha256sum -- "${speed_gate}" | awk '{print $1}')"
  speed_gate_metadata="${speed_gate}.metadata.json"
  speed_gate_metadata_sha256="$(
    sha256sum -- "${speed_gate_metadata}" | awk '{print $1}'
  )"
  printf 'VEHICLE_CMD_GATE_PARAM_FILE=%s\nVEHICLE_CMD_GATE_PARAM_SHA256=%s\nVEHICLE_CMD_GATE_METADATA_SHA256=%s\n' \
    "${speed_gate}" "${speed_gate_sha256}" \
    "${speed_gate_metadata_sha256}" >> "${output_dir}/runtime.env"
  mkdir -p "${output_dir}/speed_profile_provenance"
  cp -- "${speed_gate}" \
    "${output_dir}/speed_profile_provenance/vehicle_cmd_gate.param.yaml"
  cp -- "${speed_gate_metadata}" \
    "${output_dir}/speed_profile_provenance/vehicle_cmd_gate.param.yaml.metadata.json"
  printf '%s  %s\n%s  %s\n' \
    "${speed_gate_sha256}" "vehicle_cmd_gate.param.yaml" \
    "${speed_gate_metadata_sha256}" \
    "vehicle_cmd_gate.param.yaml.metadata.json" > \
    "${output_dir}/speed_profile_provenance/SHA256SUMS"
fi
if [[ -n "${speed_pid}" ]]; then
  speed_pid_sha256="$(sha256sum -- "${speed_pid}" | awk '{print $1}')"
  speed_pid_metadata="${speed_pid}.metadata.json"
  speed_pid_metadata_sha256="$(
    sha256sum -- "${speed_pid_metadata}" | awk '{print $1}'
  )"
  printf 'LONGITUDINAL_CONTROLLER_PARAM_FILE=%s\nLONGITUDINAL_CONTROLLER_PARAM_SHA256=%s\nLONGITUDINAL_CONTROLLER_METADATA_SHA256=%s\n' \
    "${speed_pid}" "${speed_pid_sha256}" \
    "${speed_pid_metadata_sha256}" >> "${output_dir}/runtime.env"
  mkdir -p "${output_dir}/speed_profile_provenance"
  cp -- "${speed_pid}" \
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
portable_shadow_pid=""
portable_shadow_pgid=""
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

portable_shadow_alive() {
  local process_state=""
  local actual_pgid=""
  if [[ "${portable_shadow_10hz}" != "true" ]]; then
    return 0
  fi
  if [[ ! "${portable_shadow_pid}" =~ ^[1-9][0-9]*$ ||
        ! "${portable_shadow_pgid}" =~ ^[1-9][0-9]*$ ||
        ! -f "/proc/${portable_shadow_pid}/stat" ||
        ! -d "/proc/${portable_shadow_pid}" ]]; then
    return 1
  fi
  if ! kill -0 "${portable_shadow_pid}" 2>/dev/null; then
    return 1
  fi
  process_state="$(
    ps -o stat= -p "${portable_shadow_pid}" 2>/dev/null | tr -d '[:space:]'
  )"
  if [[ -z "${process_state}" || "${process_state}" == Z* ]]; then
    return 1
  fi
  actual_pgid="$(
    ps -o pgid= -p "${portable_shadow_pid}" 2>/dev/null | tr -d '[:space:]'
  )"
  [[ "${actual_pgid}" == "${portable_shadow_pgid}" ]]
}

require_portable_shadow() {
  local stage="$1"
  local node_count=0
  if [[ "${portable_shadow_10hz}" != "true" ]]; then
    return 0
  fi
  if ! portable_shadow_alive; then
    printf 'pid=%s pgid=%s stage=%s\n' \
      "${portable_shadow_pid}" "${portable_shadow_pgid}" "${stage}" > \
      "${output_dir}/portable_shadow_failure.log"
    echo "Owned Portable E2E shadow process exited during ${stage}." >&2
    return 1
  fi
  if [[ "$(sha256sum -- "${route_file}" | awk '{print $1}')" != \
        "${portable_shadow_route_sha256}" ]]; then
    printf 'stage=%s expected_route_sha256=%s\n' \
      "${stage}" "${portable_shadow_route_sha256}" > \
      "${output_dir}/portable_shadow_route_mutation.log"
    echo "Portable E2E aligned route changed during ${stage}." >&2
    return 1
  fi
  node_count="$(
    ros2 node list --no-daemon 2>/dev/null |
      awk '$0 == "/portable_e2e_shadow" {count += 1} END {print count + 0}'
  )"
  if [[ "${node_count}" != "1" ]]; then
    printf 'stage=%s portable_shadow_node_count=%s\n' \
      "${stage}" "${node_count}" > \
      "${output_dir}/portable_shadow_duplicate_node.log"
    echo "Portable E2E shadow node identity is absent or duplicated during ${stage}." >&2
    return 1
  fi
  printf 'PORTABLE_SHADOW_VERIFY_%s=pass\n' "${stage^^}" >> \
    "${output_dir}/runtime.env"
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
  # HH_260906 - Stop the shadow publisher before its owned recorder on every cleanup path.
  e2e_stop_owned_process_group \
    "${portable_shadow_pgid}" "${portable_shadow_pid}" 30 5 2 || true
  portable_shadow_pid=""
  portable_shadow_pgid=""
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
  elif [[ "${speed_60kph_pilot}" == "true" ]]; then
    stack_command+=(--speed-60kph-pilot)
  fi
  if [[ "${camera_source_5hz}" == "true" ]]; then
    stack_command+=(--camera-source-5hz)
  elif [[ "${portable_shadow_10hz}" == "true" ]]; then
    stack_command+=(--portable-shadow-10hz)
  fi
  if [[ "${geometry_ab_route_corridor_0p2}" == "true" ]]; then
    stack_command+=(--geometry-ab-route-corridor-0p2)
  fi
  if [[ "${control_ab_pid_i40}" == "true" ]]; then
    stack_command+=(--control-ab-pid-i40)
  elif [[ "${control_ab_turn_preview_5m}" == "true" ]]; then
    stack_command+=(--control-ab-turn-preview-5m)
  elif [[ "${control_ab_turn_preview_10m}" == "true" ]]; then
    stack_command+=(--control-ab-turn-preview-10m)
  elif [[ "${control_ab_longitudinal_recovery_2p0}" == "true" ]]; then
    stack_command+=(--control-ab-longitudinal-recovery-2p0)
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

call_portable_shadow_status_service() {
  local service_name="$1"
  local target="$2"
  local operation="$3"
  if ! timeout --signal=INT --kill-after=2 5 \
    python3 - "${service_name}" "${target}" "${operation}" <<'PY'
import json
import os
from pathlib import Path
import re
import sys
import tempfile

import rclpy
from std_srvs.srv import Trigger
import yaml

service_name = sys.argv[1]
target = Path(sys.argv[2])
operation = sys.argv[3]
if re.fullmatch(r"/(?:[a-z0-9_]+/)+[a-z0-9_]+", service_name) is None:
    raise SystemExit("Portable E2E service name is unsafe")
if target.exists() or target.is_symlink():
    raise SystemExit(f"Portable E2E {operation} status target already exists")
rclpy.init()
node = rclpy.create_node(f"portable_e2e_shadow_{operation}_client_{os.getpid()}")
try:
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=1.0):
        raise SystemExit(f"Portable E2E {operation} service is unavailable")
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.result() is None:
        raise SystemExit(f"Portable E2E {operation} service did not respond")
    response = future.result()
    if response.success is not True:
        raise SystemExit(f"Portable E2E {operation} failed: {response.message}")
    status = json.loads(response.message)
    if not isinstance(status, dict):
        raise SystemExit(f"Portable E2E {operation} returned a non-object status")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", dir=target.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                {"data": json.dumps(status, sort_keys=True, allow_nan=False)},
                stream,
                sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
  then
    echo "Could not complete Portable E2E ${operation}." >&2
    return 1
  fi
}

# HH_260906 - Seal one owned shadow window within a strict liveness-checked deadline.
seal_portable_shadow_measurement() {
  local target="$1"
  local receipt="$2"
  if ! timeout --signal=INT --kill-after=1 5 \
    python3 - "${target}" "${receipt}" "${route_file}" \
      "${portable_shadow_route_sha256}" "${portable_shadow_pid}" \
      "${portable_shadow_pgid}" "${recorder_pid}" "${recorder_pgid}" \
      "${matrix_owned_carla}" "${carla_owner_pid}" "${carla_owner_pgid}" <<'PY'
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

import rclpy
from std_srvs.srv import Trigger
import yaml

(
    target_text,
    receipt_text,
    route_text,
    route_sha256,
    shadow_pid_text,
    shadow_pgid_text,
    recorder_pid_text,
    recorder_pgid_text,
    matrix_owned_carla_text,
    carla_pid_text,
    carla_pgid_text,
) = sys.argv[1:]
target = Path(target_text)
receipt = Path(receipt_text)
route = Path(route_text)
service_name = "/portable_e2e_shadow/seal_measurement"
pending_message = "cannot seal while a camera bundle remains pending"
retry_interval_s = 0.025
deadline_s = 3.0
started = time.monotonic()
deadline = started + deadline_s
# HH_260906 - Reserve one second for service response and durable artifact staging.
graph_discovery_deadline = min(deadline - 1.0, started + 1.5)
# HH_260906 - Bound service discovery separately so response and fsync retain time.
service_discovery_deadline = deadline - 0.5
attempt_count = 0
pending_retry_count = 0
liveness_check_count = 0
graph_discovery_complete = False

if target.exists() or target.is_symlink():
    raise SystemExit("Portable E2E final status target already exists")
if receipt.exists() or receipt.is_symlink():
    raise SystemExit("Portable E2E seal receipt target already exists")

def parse_identity(pid_text, pgid_text, label):
    if not pid_text.isascii() or not pid_text.isdigit() or int(pid_text) <= 0:
        raise SystemExit(f"Portable E2E seal has invalid {label} PID")
    if not pgid_text.isascii() or not pgid_text.isdigit() or int(pgid_text) <= 0:
        raise SystemExit(f"Portable E2E seal has invalid {label} PGID")
    return int(pid_text), int(pgid_text)

shadow_identity = parse_identity(shadow_pid_text, shadow_pgid_text, "shadow")
recorder_identity = parse_identity(recorder_pid_text, recorder_pgid_text, "recorder")
carla_identity = None
if matrix_owned_carla_text == "true":
    carla_identity = parse_identity(carla_pid_text, carla_pgid_text, "CARLA")
elif matrix_owned_carla_text != "false":
    raise SystemExit("Portable E2E seal has invalid CARLA ownership declaration")

def require_process(identity, label):
    pid, expected_pgid = identity
    stat_path = Path(f"/proc/{pid}/stat")
    try:
        os.kill(pid, 0)
        stat_text = stat_path.read_text(encoding="utf-8")
        closing_parenthesis = stat_text.rfind(")")
        if closing_parenthesis < 0:
            raise ValueError("missing process-name delimiter")
        process_state = stat_text[closing_parenthesis + 2 :].split(maxsplit=1)[0]
        actual_pgid = os.getpgid(pid)
    except (OSError, ValueError) as error:
        raise SystemExit(f"Portable E2E seal lost owned {label} process: {error}")
    if process_state == "Z" or actual_pgid != expected_pgid:
        raise SystemExit(
            f"Portable E2E seal lost owned {label} identity: "
            f"state={process_state!r} pgid={actual_pgid} expected={expected_pgid}"
        )

def full_node_name(name, namespace):
    normalized_namespace = namespace.rstrip("/")
    if normalized_namespace:
        return f"{normalized_namespace}/{name}"
    return f"/{name}"

def require_owned_inputs():
    if time.monotonic() >= deadline:
        raise SystemExit("Portable E2E seal exceeded its three-second deadline")
    require_process(shadow_identity, "shadow")
    require_process(recorder_identity, "recorder")
    if carla_identity is not None:
        require_process(carla_identity, "CARLA")
    if route.is_symlink() or not route.is_file():
        raise SystemExit("Portable E2E aligned route disappeared during seal")
    if hashlib.sha256(route.read_bytes()).hexdigest() != route_sha256:
        raise SystemExit("Portable E2E aligned route changed during seal")

def observe_graph(node, timeout_s):
    rclpy.spin_once(node, timeout_sec=timeout_s)
    graph_names = [
        full_node_name(name, namespace)
        for name, namespace in node.get_node_names_and_namespaces()
    ]
    return (
        graph_names.count("/portable_e2e_shadow"),
        graph_names.count("/rosbag2_recorder"),
    )

def reject_duplicate_graph(shadow_count, recorder_count):
    if shadow_count > 1:
        raise SystemExit("Portable E2E shadow node is duplicated during seal")
    if recorder_count > 1:
        raise SystemExit("Owned rosbag recorder is duplicated during seal")

def require_liveness(node):
    global graph_discovery_complete, liveness_check_count
    require_owned_inputs()
    shadow_count = 0
    recorder_count = 0
    if graph_discovery_complete:
        shadow_count, recorder_count = observe_graph(node, 0.0)
        reject_duplicate_graph(shadow_count, recorder_count)
        if shadow_count != 1 or recorder_count != 1:
            raise SystemExit(
                "Portable E2E seal lost an owned graph node after discovery: "
                f"shadow_count={shadow_count} recorder_count={recorder_count}"
            )
    else:
        # HH_260906 - Retry only absent graph entries while rechecking owned identities each cycle.
        while True:
            require_owned_inputs()
            remaining = graph_discovery_deadline - time.monotonic()
            if remaining <= 0.0:
                raise SystemExit(
                    "Portable E2E seal graph discovery exceeded its reserved deadline: "
                    f"shadow_count={shadow_count} recorder_count={recorder_count}"
                )
            shadow_count, recorder_count = observe_graph(
                node, min(0.05, remaining)
            )
            reject_duplicate_graph(shadow_count, recorder_count)
            if shadow_count == 1 and recorder_count == 1:
                graph_discovery_complete = True
                break
    if time.monotonic() >= deadline:
        raise SystemExit(
            "Portable E2E seal exceeded its deadline during liveness validation"
        )
    liveness_check_count += 1

def stage_bytes(path, payload):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return temporary

rclpy.init()
node = rclpy.create_node(f"portable_e2e_shadow_seal_client_{os.getpid()}")
try:
    require_liveness(node)
    client = node.create_client(Trigger, service_name)
    # HH_260906 - Retry delayed DDS service discovery without relaxing owned liveness.
    while True:
        require_liveness(node)
        remaining = service_discovery_deadline - time.monotonic()
        if remaining <= 0.0:
            raise SystemExit(
                "Portable E2E seal service discovery exceeded its reserved deadline"
            )
        if client.wait_for_service(timeout_sec=min(0.05, remaining)):
            break
    while True:
        require_liveness(node)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise SystemExit("Portable E2E seal exceeded its three-second deadline")
        attempt_count += 1
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=min(0.25, remaining)
        )
        if not future.done() or future.result() is None:
            raise SystemExit("Portable E2E seal service did not respond within deadline")
        response = future.result()
        if response.success is True:
            if time.monotonic() > deadline:
                raise SystemExit("Portable E2E seal succeeded after its deadline")
            try:
                status = json.loads(response.message)
            except (json.JSONDecodeError, TypeError) as error:
                raise SystemExit(f"Portable E2E seal returned invalid JSON: {error}")
            if not isinstance(status, dict):
                raise SystemExit("Portable E2E seal returned a non-object status")
            require_liveness(node)
            if time.monotonic() >= deadline:
                raise SystemExit("Portable E2E seal succeeded after its deadline")
            break
        if response.message != pending_message:
            raise SystemExit(f"Portable E2E seal failed: {response.message}")
        pending_retry_count += 1
        require_liveness(node)
        remaining = deadline - time.monotonic()
        if remaining <= retry_interval_s:
            raise SystemExit(
                "Portable E2E camera bundle remained pending through the seal deadline"
            )
        time.sleep(retry_interval_s)

    final_yaml = yaml.safe_dump(
        {"data": json.dumps(status, sort_keys=True, allow_nan=False)},
        sort_keys=False,
    ).encode("utf-8")
    receipt_payload = {
        "schema_id": "autoware-e2e.portable-shadow-seal-receipt.v1",
        "status": "PASS",
        "service": service_name,
        "completed_at": datetime.now(timezone.utc).isoformat(),
        "deadline_seconds": deadline_s,
        "retry_interval_seconds": retry_interval_s,
        "attempt_count": attempt_count,
        "pending_retry_count": pending_retry_count,
        "liveness_check_count": liveness_check_count,
        "route_sha256": route_sha256,
        "response_message_sha256": hashlib.sha256(
            response.message.encode("utf-8")
        ).hexdigest(),
        "elapsed_seconds": time.monotonic() - started,
    }
    receipt_json = (
        json.dumps(receipt_payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    if time.monotonic() >= deadline:
        raise SystemExit("Portable E2E seal artifact staging missed its deadline")
    final_temporary = None
    receipt_temporary = None
    final_linked = False
    receipt_linked = False
    try:
        final_temporary = stage_bytes(target, final_yaml)
        receipt_temporary = stage_bytes(receipt, receipt_json)
        if time.monotonic() >= deadline:
            raise SystemExit("Portable E2E seal artifact staging exceeded its deadline")
        os.link(final_temporary, target)
        final_linked = True
        if time.monotonic() >= deadline:
            raise SystemExit("Portable E2E final status link exceeded its deadline")
        os.link(receipt_temporary, receipt)
        receipt_linked = True
        if time.monotonic() >= deadline:
            raise SystemExit("Portable E2E seal receipt link exceeded its deadline")
    except BaseException:
        if receipt_linked:
            receipt.unlink(missing_ok=True)
        if final_linked:
            target.unlink(missing_ok=True)
        raise
    finally:
        if final_temporary is not None:
            final_temporary.unlink(missing_ok=True)
        if receipt_temporary is not None:
            receipt_temporary.unlink(missing_ok=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
  then
    echo "Could not seal the Portable E2E measurement inside its bounded deadline." >&2
    return 1
  fi
}

prove_portable_shadow_recorder_subscriptions() {
  local graph_output="${output_dir}/portable_shadow_provenance/recorder_subscriptions.json"
  if ! timeout --signal=INT --kill-after=2 25 \
    python3 - "${graph_output}" <<'PY'
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import tempfile
import time

import rclpy

topics = (
    "/planning/portable_e2e/status",
    "/planning/portable_e2e/latency_ms",
    "/planning/portable_e2e/selected_candidate",
    "/planning/portable_e2e/shadow_path",
    "/planning/portable_e2e/shadow_trajectory",
)
target = Path(sys.argv[1])
if target.exists() or target.is_symlink():
    raise SystemExit("Portable E2E recorder-subscription output already exists")

def full_name(endpoint):
    namespace = endpoint.node_namespace.rstrip("/")
    return f"{namespace}/{endpoint.node_name}" if namespace else f"/{endpoint.node_name}"

rclpy.init()
node = rclpy.create_node(f"portable_e2e_recorder_graph_probe_{os.getpid()}")
try:
    deadline = time.monotonic() + 20.0
    observations = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        candidate = []
        ready = True
        for topic in topics:
            subscriptions = node.get_subscriptions_info_by_topic(topic)
            publishers = node.get_publishers_info_by_topic(topic)
            recorder_count = sum(
                full_name(endpoint) == "/rosbag2_recorder"
                for endpoint in subscriptions
            )
            shadow_count = sum(
                full_name(endpoint) == "/portable_e2e_shadow"
                for endpoint in publishers
            )
            candidate.append(
                {
                    "topic": topic,
                    "owned_recorder_subscription_count": recorder_count,
                    "portable_shadow_publisher_count": shadow_count,
                    "subscription_nodes": sorted(
                        full_name(endpoint) for endpoint in subscriptions
                    ),
                    "publisher_nodes": sorted(
                        full_name(endpoint) for endpoint in publishers
                    ),
                }
            )
            if recorder_count != 1 or shadow_count != 1:
                ready = False
        if ready:
            observations = candidate
            break
        time.sleep(0.1)
    if observations is None:
        raise SystemExit(
            "owned recorder subscriptions were not discovered on all Portable topics"
        )
    payload = {
        "schema_id": "autoware-e2e.portable-shadow-recorder-subscriptions.v1",
        "status": "PASS",
        "validated_at": datetime.now(timezone.utc).isoformat(),
        "recorder_node": "/rosbag2_recorder",
        "shadow_node": "/portable_e2e_shadow",
        "topics": observations,
        "measurement_armed": False,
    }
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", dir=target.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
  then
    echo "Owned recorder did not subscribe before Portable E2E measurement arm." >&2
    return 1
  fi
}

capture_portable_shadow_healthy_heartbeat() {
  local target="$1"
  local boundary_name="$2"
  local deadline=$((SECONDS + 35))
  if [[ -e "${target}" || -L "${target}" ]]; then
    echo "Refusing to overwrite Portable E2E ${boundary_name} boundary." >&2
    return 1
  fi
  while (( SECONDS < deadline )); do
    require_carla_owner "portable_shadow_${boundary_name}_heartbeat" || return 1
    if ! portable_shadow_alive || ! kill -0 "${recorder_pid}" 2>/dev/null; then
      echo "Portable E2E shadow or owned recorder exited before ${boundary_name} heartbeat." >&2
      return 1
    fi
    # HH_260906 - Ask the status owner to publish and return one atomic healthy boundary.
    if call_portable_shadow_status_service \
      /portable_e2e_shadow/capture_startup_boundary \
      "${target}" "${boundary_name}_boundary" 2>/dev/null; then
      return 0
    fi
    sleep 0.25
  done
  echo "Portable E2E ${boundary_name} healthy semantic heartbeat was not observed." >&2
  return 1
}

start_portable_shadow() {
  if [[ "${portable_shadow_10hz}" != "true" ]]; then
    return 0
  fi
  if [[ -n "${portable_shadow_pid}" || -n "${portable_shadow_pgid}" ]]; then
    echo "Refusing to overlap owned Portable E2E shadow generations." >&2
    return 1
  fi
  if [[ "$(sha256sum -- "${route_file}" | awk '{print $1}')" != \
        "${portable_shadow_route_sha256}" ]]; then
    echo "Portable E2E aligned route changed before shadow launch." >&2
    return 1
  fi
  local launch_recheck_file="${output_dir}/portable_shadow_provenance/launch_recheck.json"
  local launch_recheck_log="${output_dir}/portable_shadow_provenance/launch_recheck.log"
  # HH_260906 - Recheck the pinned source, install, and runtime snapshot before direct launch.
  if ! python3 "${portable_shadow_validator}" recheck-launch \
    --binding "${output_dir}/portable_shadow_provenance/trial_binding.json" \
    --binding-sha256 "${portable_shadow_binding_sha256}" \
    --shadow-launch "${portable_shadow_launch}" \
    --installed-shadow-launch "${portable_shadow_installed_launch}" \
    --shadow-launch-snapshot "${portable_shadow_runtime_launch}" \
    --output "${launch_recheck_file}" > "${launch_recheck_log}" 2>&1; then
    echo "Portable E2E launch binding changed before ROS launch." >&2
    return 1
  fi
  local launch_recheck_sha256=""
  launch_recheck_sha256="$(sha256sum -- "${launch_recheck_file}" | awk '{print $1}')"
  printf 'PORTABLE_SHADOW_LAUNCH_RECHECK_FILE=%s\nPORTABLE_SHADOW_LAUNCH_RECHECK_SHA256=%s\nPORTABLE_SHADOW_LAUNCH_RECHECK_STATUS=PASS\nPORTABLE_SHADOW_LAUNCH_RECHECK_STAGE=immediately_before_ros_launch\n' \
    "${launch_recheck_file}" "${launch_recheck_sha256}" >> \
    "${output_dir}/runtime.env"
  portable_shadow_command=(
    ros2 launch "${portable_shadow_runtime_launch}"
    "contract_file:=${portable_contract_file}"
    "contract_sha256:=${portable_contract_sha256}"
    "runtime_bundle_file:=${portable_runtime_bundle}"
    "runtime_bundle_sha256:=${portable_runtime_bundle_sha256}"
    "source_checkpoint_sha256:=${portable_source_checkpoint_sha256}"
    "corpus_fingerprint_sha256:=${portable_corpus_fingerprint_sha256}"
    "model_config_sha256:=${portable_model_config_sha256}"
    "rig_file:=${portable_rig_file}"
    "rig_sha256:=${portable_rig_sha256}"
    "declared_map_id:=${portable_shadow_declared_map_id}"
    "route_file:=${route_file}"
    "route_sha256:=${portable_shadow_route_sha256}"
    "device:=${portable_shadow_device}"
    "input_settle_timeout_s:=0.05"
    "maximum_tf_translation_error_m:=0.005"
    "maximum_tf_rotation_error_rad:=0.005"
    "use_sim_time:=true"
    "research_acknowledged:=true"
  )
  if [[ "${portable_shadow_device}" == "cpu" ]]; then
    portable_shadow_cpu_command=(setsid)
    if [[ -n "${portable_cpu_set}" ]]; then
      portable_shadow_cpu_command+=(
        taskset --cpu-list "${portable_cpu_set}"
      )
    fi
    # HH_260906 - Pin bounded CPU libraries before Python imports the inference stack.
    portable_shadow_cpu_command+=(
      env
      CUDA_VISIBLE_DEVICES=''
      OMP_NUM_THREADS=4
      MKL_NUM_THREADS=4
      OPENBLAS_NUM_THREADS=1
      NUMEXPR_NUM_THREADS=1
    )
    "${portable_shadow_cpu_command[@]}" "${portable_shadow_command[@]}" > \
      "${output_dir}/portable_shadow.log" 2>&1 &
  else
    setsid "${portable_shadow_command[@]}" > \
      "${output_dir}/portable_shadow.log" 2>&1 &
  fi
  portable_shadow_pid=$!
  portable_shadow_pgid="${portable_shadow_pid}"

  local deadline=$((SECONDS + 45))
  local node_count=0
  while (( SECONDS < deadline )); do
    if ! portable_shadow_alive; then
      echo "Portable E2E shadow exited before exposing its disarmed identity." >&2
      return 1
    fi
    node_count="$(
      ros2 node list --no-daemon 2>/dev/null |
        awk '$0 == "/portable_e2e_shadow" {count += 1} END {print count + 0}'
    )"
    if [[ "${node_count}" == "1" ]]; then
      break
    fi
    sleep 0.25
  done
  if [[ "${node_count}" != "1" ]]; then
    echo "Portable E2E shadow did not produce one owned disarmed identity." >&2
    return 1
  fi
  if ! ros2 node info --no-daemon /portable_e2e_shadow > \
    "${output_dir}/portable_shadow_provenance/node_info.txt" 2> \
    "${output_dir}/portable_shadow_provenance/node_info.err"; then
    echo "Could not inspect the owned Portable E2E shadow ROS graph." >&2
    return 1
  fi
  prove_portable_shadow_recorder_subscriptions || return 1
  local status_file="${output_dir}/portable_shadow_provenance/armed_status.yaml"
  call_portable_shadow_status_service \
    /portable_e2e_shadow/arm_measurement "${status_file}" arm_measurement || \
    return 1
  python3 - "${status_file}" \
    "${output_dir}/portable_shadow_provenance/node_info.txt" \
    "${output_dir}/portable_shadow_provenance/trial_binding.json" \
    "${portable_shadow_binding_sha256}" "${portable_shadow_route_sha256}" \
    "${portable_shadow_declared_map_id}" "${portable_runtime_bundle_sha256}" \
    "${portable_source_checkpoint_sha256}" \
    "${portable_model_config_sha256}" \
    "${portable_corpus_fingerprint_sha256}" "${portable_rig_sha256}" \
    "${portable_contract_sha256}" "${portable_shadow_device}" \
    "${portable_cpu_set}" \
    "${portable_shadow_runtime_launch}" \
    "${portable_shadow_runtime_launch_sha256}" \
    "${launch_recheck_file}" "${launch_recheck_sha256}" \
    "${portable_shadow_pid}" "${portable_shadow_pgid}" \
    "${output_dir}/portable_shadow_provenance/startup_validation.json" <<'PY'
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile

import yaml

from portable_e2e.runtime_contract import RUNTIME_GATE_ID, RuntimeGateConfig

(
    status_path,
    node_info_path,
    binding_path,
    binding_sha256,
    route_sha256,
    declared_map_id,
    runtime_bundle_sha256,
    source_checkpoint_sha256,
    model_config_sha256,
    corpus_fingerprint_sha256,
    rig_sha256,
    contract_sha256,
    device,
    portable_cpu_set,
    runtime_shadow_launch_path,
    runtime_shadow_launch_sha256,
    launch_recheck_path,
    launch_recheck_sha256,
    process_pid,
    process_pgid,
    output_path,
) = sys.argv[1:]
documents = list(yaml.safe_load_all(Path(status_path).read_text(encoding="utf-8")))
status_data = next(
    (
        document.get("data")
        for document in documents
        if isinstance(document, dict) and isinstance(document.get("data"), str)
    ),
    None,
)
if status_data is None:
    raise SystemExit("Portable E2E startup status has no String data")
status = json.loads(status_data)
binding_bytes = Path(binding_path).read_bytes()
if hashlib.sha256(binding_bytes).hexdigest() != binding_sha256:
    raise SystemExit("Portable E2E trial binding changed before startup")
binding = json.loads(binding_bytes)
launch_recheck_bytes = Path(launch_recheck_path).read_bytes()
if hashlib.sha256(launch_recheck_bytes).hexdigest() != launch_recheck_sha256:
    raise SystemExit("Portable E2E launch recheck changed before startup validation")
launch_recheck = json.loads(launch_recheck_bytes)
runtime_shadow_launch = launch_recheck.get("runtime_shadow_launch")
binding_shadow_launch = binding.get("shadow_launch")
snapshot_binding = (
    binding_shadow_launch.get("snapshot_binding")
    if isinstance(binding_shadow_launch, dict)
    else None
)
snapshot_path = Path(runtime_shadow_launch_path)
if (
    snapshot_path.is_symlink()
    or not snapshot_path.is_file()
    or (snapshot_path.stat().st_mode & 0o777) != 0o444
    or hashlib.sha256(snapshot_path.read_bytes()).hexdigest()
    != runtime_shadow_launch_sha256
    or not isinstance(runtime_shadow_launch, dict)
    or runtime_shadow_launch != snapshot_binding
    or runtime_shadow_launch.get("file") != str(snapshot_path.resolve())
    or runtime_shadow_launch.get("sha256") != runtime_shadow_launch_sha256
    or runtime_shadow_launch.get("mode_octal") != "0444"
    or runtime_shadow_launch.get("direct_ros_launch") is not True
):
    raise SystemExit("Portable E2E runtime shadow launch changed before startup validation")
if (
    launch_recheck.get("status") != "PASS"
    or launch_recheck.get("check_stage") != "immediately_before_ros_launch"
    or launch_recheck.get("trial_binding_sha256") != binding_sha256
    or launch_recheck.get("matches_initial_binding") is not True
):
    raise SystemExit("Portable E2E launch recheck provenance mismatch")
expected_topics = {
    "/planning/portable_e2e/latency_ms",
    "/planning/portable_e2e/selected_candidate",
    "/planning/portable_e2e/shadow_path",
    "/planning/portable_e2e/shadow_trajectory",
    "/planning/portable_e2e/status",
}
if (
    status.get("schema_id") != "autoware-e2e.portable-shadow-status.v3"
    or status.get("state") != "SHADOW_WAITING"
    or status.get("stage") != "measurement"
    or status.get("failure_code") != "waiting_for_inputs"
    or status.get("measurement_armed") is not True
    or status.get("measurement_sealed") is not False
    or status.get("vehicle_control_approved") is not False
    or set(status.get("output_topics", [])) != expected_topics
):
    raise SystemExit("Portable E2E armed status violates shadow-only zero-baseline policy")
for name in (
    "anchor_attempt_count",
    "accepted_count",
    "anchor_rejected_count",
    "input_event_rejected_count",
    "input_settle_deferred_count",
    "input_settle_timeout_count",
):
    if status.get(name) != 0:
        raise SystemExit(f"Portable E2E armed status counter is not zero: {name}")
if not isinstance(status.get("rejection_counts_by_stage"), dict) or any(
    value != 0 for value in status["rejection_counts_by_stage"].values()
):
    raise SystemExit("Portable E2E armed status has nonzero rejection counters")
if not isinstance(status.get("camera_bundle_counters"), dict) or any(
    value != 0 for value in status["camera_bundle_counters"].values()
):
    raise SystemExit("Portable E2E armed status has nonzero camera-bundle counters")
provenance = status.get("provenance")
# HH_260906 - Pin the complete v8 geometry gate at the startup evidence boundary.
expected_runtime_gate = asdict(RuntimeGateConfig())
RuntimeGateConfig().validate()
expected_provenance = {
    "route_sha256": route_sha256,
    "declared_map_id": declared_map_id,
    "runtime_bundle_sha256": runtime_bundle_sha256,
    "source_checkpoint_sha256": source_checkpoint_sha256,
    "model_config_sha256": model_config_sha256,
    "corpus_fingerprint_sha256": corpus_fingerprint_sha256,
    "rig_sha256": rig_sha256,
    "contract_sha256": contract_sha256,
    "runtime_device": device,
    "published_trajectory_frame": "map",
    "model_output_frame": "base_link_at_anchor",
    "runtime_gate_id": RUNTIME_GATE_ID,
    "runtime_gate": expected_runtime_gate,
}
if not isinstance(provenance, dict) or any(
    provenance.get(name) != value for name, value in expected_provenance.items()
):
    raise SystemExit("Portable E2E startup status provenance mismatch")
runtime_policy = provenance.get("runtime_policy")
if (
    not isinstance(runtime_policy, dict)
    or runtime_policy.get("input_settle_timeout_s") != 0.05
    or runtime_policy.get("maximum_tf_translation_error_m") != 0.005
    or runtime_policy.get("maximum_tf_rotation_error_rad") != 0.005
):
    raise SystemExit("Portable E2E startup status runtime policy mismatch")
runtime_execution_policy = provenance.get("runtime_execution_policy")
if not isinstance(runtime_execution_policy, dict):
    raise SystemExit("Portable E2E startup status lacks runtime execution policy")
process_affinity = sorted(os.sched_getaffinity(int(process_pid)))
if runtime_execution_policy.get("cpu_affinity") != process_affinity:
    raise SystemExit("Portable E2E status and launch-process CPU affinity disagree")
if device == "cpu":
    expected_affinity = (
        process_affinity
        if not portable_cpu_set
        else sorted(int(value) for value in portable_cpu_set.split(","))
    )
    if (
        runtime_execution_policy.get("policy_id")
        != "portable_e2e.cpu_execution.v1"
        or runtime_execution_policy.get("torch_intraop_threads") != 4
        or runtime_execution_policy.get("torch_interop_threads") != 1
        or process_affinity != expected_affinity
    ):
        raise SystemExit("Portable E2E CPU execution policy mismatch")
elif runtime_execution_policy.get("policy_id") != "portable_e2e.cuda_execution.v1":
    raise SystemExit("Portable E2E CUDA execution policy mismatch")
if not all(
    isinstance(provenance.get(name), str) and provenance[name]
    for name in ("runtime_id", "model_id")
):
    raise SystemExit("Portable E2E startup status lacks runtime/model identity")
route_binding = binding.get("route_binding")
if (
    binding.get("status") != "PASS"
    or binding.get("execution_mode") != "shadow_only"
    or binding.get("vehicle_control_approved") is not False
    or not isinstance(route_binding, dict)
    or route_binding.get("declared_map_id") != declared_map_id
    or route_binding.get("observed_map_id") != declared_map_id
    or route_binding.get("aligned_route_sha256") != route_sha256
):
    raise SystemExit("Portable E2E observed map/route binding mismatch")

publishers = []
service_servers = []
section = None
for line in Path(node_info_path).read_text(encoding="utf-8").splitlines():
    stripped = line.strip()
    if stripped in {
        "Subscribers:",
        "Publishers:",
        "Service Servers:",
        "Service Clients:",
        "Action Servers:",
        "Action Clients:",
    }:
        section = stripped
        continue
    match = re.match(r"^\s+(/[^:]+):", line)
    if section == "Publishers:" and match:
        publishers.append(match.group(1))
    elif section == "Service Servers:" and match:
        service_servers.append(match.group(1))
allowed_publishers = expected_topics | {"/parameter_events", "/rosout"}
if set(publishers) != allowed_publishers or len(publishers) != len(allowed_publishers):
    raise SystemExit(f"Portable E2E publisher graph is not exact: {publishers!r}")
if any(
    topic == "/planning/trajectory"
    or topic.startswith("/control/")
    or topic.startswith("/vehicle/command/")
    for topic in publishers
):
    raise SystemExit("Portable E2E node publishes a canonical control topic")
required_measurement_services = {
    "/portable_e2e_shadow/arm_measurement",
    "/portable_e2e_shadow/capture_startup_boundary",
    "/portable_e2e_shadow/seal_measurement",
}
if any(service_servers.count(name) != 1 for name in required_measurement_services):
    raise SystemExit("Portable E2E measurement service is absent or duplicated")

payload = {
    "schema_version": 1,
    "status": "PASS",
    "validated_at": datetime.now(timezone.utc).isoformat(),
    "node": "/portable_e2e_shadow",
    "process": {"pid": int(process_pid), "pgid": int(process_pgid)},
    # HH_260906 - Preserve the independently observed launch affinity with startup evidence.
    "runtime_execution_policy": runtime_execution_policy,
    "runtime_gate_id": RUNTIME_GATE_ID,
    "runtime_gate": expected_runtime_gate,
    "declared_map_id": declared_map_id,
    "observed_map_id": route_binding["observed_map_id"],
    "route_sha256": route_sha256,
    "binding_sha256": binding_sha256,
    "launch_recheck_sha256": launch_recheck_sha256,
    "runtime_shadow_launch_file": str(snapshot_path.resolve()),
    "runtime_shadow_launch_sha256": runtime_shadow_launch_sha256,
    "publishers": sorted(publishers),
    "measurement_arm_service": "/portable_e2e_shadow/arm_measurement",
    "measurement_startup_boundary_service": (
        "/portable_e2e_shadow/capture_startup_boundary"
    ),
    "measurement_seal_service": "/portable_e2e_shadow/seal_measurement",
    "canonical_or_control_publishers": [],
    "vehicle_control_approved": False,
    "armed_status": status,
}
analyzer_provenance = {
    "runtime_id": provenance["runtime_id"],
    "model_id": provenance["model_id"],
    "runtime_bundle_sha256": runtime_bundle_sha256,
    "source_checkpoint_sha256": source_checkpoint_sha256,
    "model_config_sha256": model_config_sha256,
    "corpus_fingerprint_sha256": corpus_fingerprint_sha256,
    "aligned_route_sha256": route_sha256,
    "observed_map_id": route_binding["observed_map_id"],
    "map_bundle_sha256": route_binding["map_bundle_sha256"],
    "runtime_gate_id": RUNTIME_GATE_ID,
    "runtime_gate": expected_runtime_gate,
}

def write_json(target, value):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", dir=target.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, target)
    finally:
        Path(temporary_name).unlink(missing_ok=True)

target = Path(output_path)
write_json(target, payload)
write_json(target.parent / "analyzer_provenance.json", analyzer_provenance)
PY
  local startup_validation_sha256=""
  local analyzer_provenance_sha256=""
  local armed_status_sha256=""
  local recorder_subscriptions_sha256=""
  startup_validation_sha256="$(
    sha256sum -- \
      "${output_dir}/portable_shadow_provenance/startup_validation.json" |
      awk '{print $1}'
  )"
  analyzer_provenance_sha256="$(
    sha256sum -- \
      "${output_dir}/portable_shadow_provenance/analyzer_provenance.json" |
      awk '{print $1}'
  )"
  armed_status_sha256="$(
    sha256sum -- "${output_dir}/portable_shadow_provenance/armed_status.yaml" |
      awk '{print $1}'
  )"
  recorder_subscriptions_sha256="$(
    sha256sum -- \
      "${output_dir}/portable_shadow_provenance/recorder_subscriptions.json" |
      awk '{print $1}'
  )"
  printf 'PORTABLE_SHADOW_PID=%s\nPORTABLE_SHADOW_PGID=%s\nPORTABLE_SHADOW_STARTED_AFTER_VAD_ROUTE_READY=true\nPORTABLE_SHADOW_STARTED_AFTER_ALIGNED_ROUTE_HASH=true\nPORTABLE_SHADOW_RECORDER_SUBSCRIPTIONS_VERIFIED_BEFORE_ARM=true\nPORTABLE_SHADOW_RECORDER_SUBSCRIPTIONS_FILE=%s\nPORTABLE_SHADOW_RECORDER_SUBSCRIPTIONS_SHA256=%s\nPORTABLE_SHADOW_ARMED_STATUS_FILE=%s\nPORTABLE_SHADOW_ARMED_STATUS_SHA256=%s\nPORTABLE_SHADOW_STARTUP_VALIDATION_FILE=%s\nPORTABLE_SHADOW_STARTUP_VALIDATION_SHA256=%s\nPORTABLE_SHADOW_ANALYZER_PROVENANCE_FILE=%s\nPORTABLE_SHADOW_ANALYZER_PROVENANCE_SHA256=%s\n' \
    "${portable_shadow_pid}" "${portable_shadow_pgid}" \
    "${output_dir}/portable_shadow_provenance/recorder_subscriptions.json" \
    "${recorder_subscriptions_sha256}" \
    "${output_dir}/portable_shadow_provenance/armed_status.yaml" \
    "${armed_status_sha256}" \
    "${output_dir}/portable_shadow_provenance/startup_validation.json" \
    "${startup_validation_sha256}" \
    "${output_dir}/portable_shadow_provenance/analyzer_provenance.json" \
    "${analyzer_provenance_sha256}" >> "${output_dir}/runtime.env"
  require_portable_shadow armed || return 1
  capture_portable_shadow_healthy_heartbeat \
    "${output_dir}/portable_shadow_provenance/startup_status.yaml" startup || \
    return 1
}

capture_portable_shadow_window_final() {
  if [[ "${portable_shadow_10hz}" != "true" ]]; then
    return 0
  fi
  while (( SECONDS - portable_shadow_window_started_seconds < 11 )); do
    require_carla_owner portable_shadow_window_settle || return 1
    if ! portable_shadow_alive || ! kill -0 "${recorder_pid}" 2>/dev/null; then
      echo "Portable E2E shadow or owned recorder exited before final window boundary." >&2
      return 1
    fi
    sleep 0.25
  done
  require_portable_shadow window_final || return 1
  local final_status="${output_dir}/portable_shadow_provenance/final_status.yaml"
  local seal_receipt="${output_dir}/portable_shadow_provenance/seal_receipt.json"
  seal_portable_shadow_measurement "${final_status}" "${seal_receipt}" || return 1
  python3 - \
    "${output_dir}/portable_shadow_provenance/armed_status.yaml" \
    "${output_dir}/portable_shadow_provenance/startup_status.yaml" \
    "${final_status}" \
    "${output_dir}/portable_shadow_provenance/window_boundaries.json" <<'PY'
import json
import os
from pathlib import Path
import sys
import tempfile

import yaml

def decode(path):
    documents = yaml.safe_load_all(Path(path).read_text(encoding="utf-8"))
    raw = next(
        (
            document.get("data")
            for document in documents
            if isinstance(document, dict) and isinstance(document.get("data"), str)
        ),
        None,
    )
    if raw is None:
        raise SystemExit(f"Portable E2E boundary has no String data: {path}")
    return json.loads(raw)

armed = decode(sys.argv[1])
startup = decode(sys.argv[2])
final = decode(sys.argv[3])
if (
    armed.get("schema_id") != "autoware-e2e.portable-shadow-status.v3"
    or armed.get("state") != "SHADOW_WAITING"
    or armed.get("stage") != "measurement"
    or armed.get("failure_code") != "waiting_for_inputs"
    or armed.get("measurement_armed") is not True
    or armed.get("measurement_sealed") is not False
    or armed.get("vehicle_control_approved") is not False
):
    raise SystemExit("Portable E2E armed boundary violates the zero-baseline contract")
for name in (
    "anchor_attempt_count",
    "accepted_count",
    "anchor_rejected_count",
    "input_event_rejected_count",
    "input_settle_deferred_count",
    "input_settle_timeout_count",
):
    if armed.get(name) != 0:
        raise SystemExit(f"Portable E2E armed boundary counter is not zero: {name}")
if not isinstance(armed.get("rejection_counts_by_stage"), dict) or any(
    value != 0 for value in armed["rejection_counts_by_stage"].values()
):
    raise SystemExit("Portable E2E armed rejection counters are not zero")
if not isinstance(armed.get("camera_bundle_counters"), dict) or any(
    value != 0 for value in armed["camera_bundle_counters"].values()
):
    raise SystemExit("Portable E2E armed camera-bundle counters are not zero")
for label, status in (("startup", startup), ("final", final)):
    if (
        status.get("schema_id") != "autoware-e2e.portable-shadow-status.v3"
        or status.get("state") != "SHADOW_OK"
        or status.get("stage") != "inference"
        or status.get("failure_code") != "none"
        or status.get("healthy_now") is not True
        or status.get("inference_inputs_healthy_now") is not True
        or status.get("calibration_extrinsics_verified") is not True
        or status.get("tf_extrinsic_parity") != "VERIFIED"
        or status.get("measurement_armed") is not True
        or status.get("vehicle_control_approved") is not False
    ):
        raise SystemExit(
            f"Portable E2E {label} boundary is not a settled healthy heartbeat"
        )
if startup.get("measurement_sealed") is not False:
    raise SystemExit("Portable E2E startup boundary was not explicitly unsealed")
if final.get("measurement_sealed") is not True:
    raise SystemExit("Portable E2E final boundary is not measurement-sealed")
final_bundle_counters = final.get("camera_bundle_counters")
if (
    not isinstance(final_bundle_counters, dict)
    or final_bundle_counters.get("pending_bundle_count") != 0
):
    raise SystemExit("Portable E2E sealed boundary retains a pending camera bundle")
if not (armed.get("provenance") == startup.get("provenance") == final.get("provenance")):
    raise SystemExit("Portable E2E provenance changed inside the analysis window")
for name in (
    "anchor_attempt_count",
    "accepted_count",
    "anchor_rejected_count",
    "input_event_rejected_count",
):
    before = startup.get(name)
    after = final.get(name)
    if (
        isinstance(before, bool)
        or not isinstance(before, int)
        or isinstance(after, bool)
        or not isinstance(after, int)
        or before < 0
        or after < before
    ):
        raise SystemExit(f"Portable E2E boundary counter is not monotonic: {name}")
window_wall_ns = final.get("status_wall_timestamp_ns", 0) - startup.get(
    "status_wall_timestamp_ns", 0
)
if window_wall_ns < 10_000_000_000:
    raise SystemExit("Portable E2E final boundary is less than ten seconds after startup")
payload = {
    "schema_id": "autoware-e2e.portable-shadow-window-boundaries.v2",
    "armed_status": armed,
    "startup_status": startup,
    "final_status": final,
}
target = Path(sys.argv[4])
if target.exists() or target.is_symlink():
    raise SystemExit("Portable E2E window boundary output already exists")
descriptor, temporary_name = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
temporary = Path(temporary_name)
try:
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.link(temporary, target)
finally:
    temporary.unlink(missing_ok=True)
PY
  sleep 0.5
  require_portable_shadow measurement_sealed || return 1
  if ! e2e_stop_owned_process_group \
    "${portable_shadow_pgid}" "${portable_shadow_pid}" 30 5 2; then
    echo "Owned Portable E2E shadow did not stop after sealing measurement." >&2
    return 1
  fi
  portable_shadow_pid=""
  portable_shadow_pgid=""
  sleep 1
  local boundary_sha256=""
  local seal_receipt_sha256=""
  boundary_sha256="$(
    sha256sum -- \
      "${output_dir}/portable_shadow_provenance/window_boundaries.json" |
      awk '{print $1}'
  )"
  seal_receipt_sha256="$(sha256sum -- "${seal_receipt}" | awk '{print $1}')"
  printf 'PORTABLE_SHADOW_WINDOW_BOUNDARIES_FILE=%s\nPORTABLE_SHADOW_WINDOW_BOUNDARIES_SHA256=%s\nPORTABLE_SHADOW_SEAL_RECEIPT_FILE=%s\nPORTABLE_SHADOW_SEAL_RECEIPT_SHA256=%s\nPORTABLE_SHADOW_SEAL_DEADLINE_SECONDS=3.0\nPORTABLE_SHADOW_SEAL_RETRY_INTERVAL_SECONDS=0.025\nPORTABLE_SHADOW_WINDOW_MINIMUM_WALL_SECONDS=10\nPORTABLE_SHADOW_WINDOW_FINAL_CAPTURED_BEFORE_RECORDER_STOP=true\nPORTABLE_SHADOW_MEASUREMENT_SEALED=true\nPORTABLE_SHADOW_STOPPED_BEFORE_RECORDER=true\n' \
    "${output_dir}/portable_shadow_provenance/window_boundaries.json" \
    "${boundary_sha256}" "${seal_receipt}" "${seal_receipt_sha256}" >> \
    "${output_dir}/runtime.env"
}

# HH_260906 - Settle the RViz geometry before the first valid candidate arms its watchdog.
prepare_owned_rviz_capture_window

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
  desktop_recording_started_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
  candidate_still_captured_at="${desktop_recording_started_at}"
  # HH_260906 - Bound live capture workers so evidence recording cannot starve CARLA and VAD.
  setsid ffmpeg -y -nostdin -loglevel error \
    -filter_threads "${capture_filter_threads}" \
    -f x11grab -draw_mouse 0 \
    -framerate "${capture_framerate_fps}" \
    -window_id "${capture_rviz_window_id_decimal}" \
    -video_size "${capture_rviz_input_dimensions}" -i "${desktop_display}" \
    -vf "${capture_pad_filter}" \
    -c:v "${capture_encoder}" -preset "${capture_encoder_preset}" \
    -crf "${capture_encoder_crf}" -threads "${capture_encoder_threads}" \
    -pix_fmt yuv420p \
    "${output_dir}/autoware_rviz_capture.mkv" &
  desktop_pid=$!
  desktop_pgid="${desktop_pid}"
  sleep 1
  require_desktop_recorder recording_started
  verify_owned_rviz_capture_window recording_started
  verify_owned_rviz_capture_window candidate_post
fi

if [[ "${runtime_health_gate}" == "true" ]]; then
  require_carla_owner runtime_health_gate || exit 1
  runtime_health_arguments=(
    --output "${output_dir}/runtime_health.json"
    --window-sec "${runtime_health_window_sec}"
    --timeout-sec "${runtime_health_timeout}"
  )
  if [[ "${camera_source_5hz}" == "true" || \
        "${portable_shadow_10hz}" == "true" ]]; then
    runtime_health_arguments+=(
      --camera-transport-profile-id "${camera_transport_profile_id}"
      --sensor-mapping-sha256 "${camera_transport_sensor_mapping_sha256}"
      --vad-model-override-sha256 "${camera_transport_vad_override_sha256}"
      --cyclonedds-uri "${CYCLONEDDS_URI}"
      --cyclonedds-config-sha256 "${camera_transport_cyclonedds_sha256}"
    )
  fi
  if [[ "${capture_desktop}" == "true" ]]; then
    require_desktop_recorder runtime_health_start
    runtime_health_arguments+=(
      --rviz-recorder-pid "${desktop_pid}"
      --rviz-recorder-pgid "${desktop_pgid}"
    )
  fi
  runtime_health_exit_status=0
  python3 "${runtime_health_probe}" "${runtime_health_arguments[@]}" > \
    "${output_dir}/runtime_health.log" 2>&1 || runtime_health_exit_status=$?
  if [[ ! -f "${output_dir}/runtime_health.json" ||
        -L "${output_dir}/runtime_health.json" ]]; then
    echo "Runtime health gate did not produce regular JSON evidence" >&2
    exit 1
  fi
  runtime_health_evidence_sha256="$(
    sha256sum -- "${output_dir}/runtime_health.json" | awk '{print $1}'
  )"
  python3 - "${output_dir}/runtime_health.json" \
    "${runtime_health_exit_status}" "${runtime_health_probe_sha256}" \
    "${runtime_health_timeout}" "${capture_desktop}" "${desktop_pid}" \
    "${desktop_pgid}" "${runtime_health_evidence_sha256}" \
    "${camera_transport_profile_id}" \
    "${camera_transport_sensor_mapping_sha256}" \
    "${camera_transport_vad_override_sha256}" \
    "${CYCLONEDDS_URI:-}" "${camera_transport_cyclonedds_sha256}" <<'PY' >> \
    "${output_dir}/runtime.env"
import json
import math
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
exit_status = int(sys.argv[2])
expected_probe_sha256 = sys.argv[3]
expected_timeout = float(sys.argv[4])
rviz_required = sys.argv[5] == "true"
expected_rviz_pid = int(sys.argv[6]) if rviz_required else None
expected_rviz_pgid = int(sys.argv[7]) if rviz_required else None
evidence_sha256 = sys.argv[8]
expected_transport_profile = sys.argv[9]
expected_mapping_sha256 = sys.argv[10]
expected_vad_override_sha256 = sys.argv[11]
expected_cyclonedds_uri = sys.argv[12]
expected_cyclonedds_sha256 = sys.argv[13]
payload = json.loads(path.read_text(encoding="utf-8"))
expected_status = "PASS" if exit_status == 0 else "FAIL"
contract = payload.get("contract")
runtime = payload.get("runtime")
sequence = payload.get("sequence")
expected_thresholds = {
    "maximum_bundle_receipt_p95_seconds": 0.04,
    "minimum_bundle_coverage_percent": 99.0,
    "minimum_camera_wall_rate_hz": 4.0,
    "minimum_complete_bundle_count": 20,
    "minimum_rtf": 0.9,
}
if expected_transport_profile == "portable_e2e_exact_bundle_10hz_v2":
    expected_thresholds["minimum_camera_wall_rate_hz"] = 9.0
    expected_thresholds["minimum_complete_bundle_count"] = 70
if (
    payload.get("schema_version") != 1
    or payload.get("probe_id") != "pre_engagement_runtime_health_v1"
    or payload.get("status") != expected_status
    or not isinstance(contract, dict)
    or not isinstance(runtime, dict)
    or runtime.get("read_only_subscriber") is not True
    or runtime.get("publisher_qos_modified") is not False
    or runtime.get("rosbag_started") is not False
    or runtime.get("vehicle_engaged") is not False
    or runtime.get("rviz_recorder_required") is not rviz_required
    or payload.get("finished_at") is None
):
    raise SystemExit("runtime health JSON top-level contract mismatch")
if (
    not math.isclose(float(payload.get("timeout_seconds")), expected_timeout)
    or not math.isclose(float(contract.get("window_seconds")), 8.0)
    or contract.get("required_consecutive_passes") != 3
    or contract.get("topics", {}).get("clock") != "/clock"
    or len(contract.get("topics", {}).get("camera_info", [])) != 6
    or contract.get("thresholds") != expected_thresholds
    or payload.get("source", {}).get("sha256") != expected_probe_sha256
):
    raise SystemExit("runtime health JSON fixed thresholds/provenance mismatch")
transport = contract.get("camera_transport")
if expected_transport_profile == "carla_vad_camera_source_5hz_best_effort_image_v1":
    expected_transport = {
        "profile_id": expected_transport_profile,
        "camera_image_publisher_reliability": "best_effort",
        "camera_info_publisher_reliability": "reliable",
        "vad_image_subscription_reliability": "best_effort",
        "rviz_image_subscription_reliability": "best_effort",
        "sensor_mapping_sha256": expected_mapping_sha256,
        "vad_model_override_sha256": expected_vad_override_sha256,
        "probe_topics": "camera_info_only",
    }
    if transport != expected_transport:
        raise SystemExit("runtime health camera transport provenance mismatch")
elif expected_transport_profile in {
    "carla_vad_camera_source_5hz_best_effort_image_v2",
    "portable_e2e_exact_bundle_10hz_v2",
}:
    expected_transport = {
        "profile_id": expected_transport_profile,
        "camera_image_publisher_reliability": "best_effort",
        "camera_info_publisher_reliability": "reliable",
        "vad_image_subscription_reliability": "best_effort",
        "rviz_image_subscription_reliability": "best_effort",
        "sensor_mapping_sha256": expected_mapping_sha256,
        "vad_model_override_sha256": expected_vad_override_sha256,
        "probe_topics": "camera_info_plus_read_only_image_graph",
        "camera_image_endpoint_history": "keep_last",
        "camera_image_endpoint_depth": 1,
        "camera_image_endpoint_durability": "volatile",
        "exact_camera_image_graph_required": True,
        "cyclonedds_loopback_interface_required": True,
        "ros_localhost_only_expected": "0",
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "cyclonedds_uri": expected_cyclonedds_uri,
        "cyclonedds_config_sha256": expected_cyclonedds_sha256,
    }
    if expected_transport_profile == "portable_e2e_exact_bundle_10hz_v2":
        expected_transport.update(
            {
                "camera_source_sensor_tick_seconds": 0.1,
                "bridge_publish_cap_hz": 11,
                "declared_effective_camera_rate_hz": 10.0,
                "minimum_camera_wall_rate_hz": 9.0,
                "minimum_complete_bundle_count": 70,
            }
        )
    graph = payload.get("camera_image_graph")
    transport_environment = runtime.get("transport_environment")
    if (
        transport != expected_transport
        or contract.get("topics", {}).get("camera_image_graph")
        != [
            "/sensing/camera/CAM_FRONT/image_raw",
            "/sensing/camera/CAM_BACK/image_raw",
            "/sensing/camera/CAM_FRONT_LEFT/image_raw",
            "/sensing/camera/CAM_BACK_LEFT/image_raw",
            "/sensing/camera/CAM_FRONT_RIGHT/image_raw",
            "/sensing/camera/CAM_BACK_RIGHT/image_raw",
        ]
        or not isinstance(graph, dict)
        or not isinstance(transport_environment, dict)
        or transport_environment.get("status") != "PASS"
    ):
        raise SystemExit("runtime health exact-transport provenance mismatch")
    if expected_status == "PASS" and graph.get("status") != "PASS":
        raise SystemExit("runtime health PASS lacks exact camera endpoint graph")
elif transport is not None:
    raise SystemExit("unexpected runtime health camera transport provenance")
rviz_active = False
if rviz_required:
    before = runtime.get("rviz_recorder_before")
    after = runtime.get("rviz_recorder_after")
    expected_identity = (expected_rviz_pid, expected_rviz_pgid)
    if expected_status == "PASS" and not all(
        isinstance(item, dict) for item in (before, after)
    ):
        raise SystemExit("runtime health JSON lacks RViz load evidence")
    for item in (before, after):
        if item is None:
            continue
        if not isinstance(item, dict):
            raise SystemExit("runtime health JSON RViz evidence is invalid")
        if (item.get("pid"), item.get("pgid")) != expected_identity:
            raise SystemExit("runtime health JSON RViz identity mismatch")
        if not isinstance(item.get("process_state"), str):
            raise SystemExit("runtime health JSON RViz process state is invalid")
    rviz_active = all(isinstance(item, dict) for item in (before, after))
if not re.fullmatch(r"[0-9a-f]{64}", evidence_sha256):
    raise SystemExit("runtime health JSON digest is invalid")
evaluated = len(payload.get("windows", []))
maximum_consecutive = 0
winning = ""
if isinstance(sequence, dict):
    evaluated = sequence.get("evaluated_window_count", evaluated)
    maximum_consecutive = sequence.get("maximum_consecutive_passes", 0)
    winning = ",".join(str(index) for index in sequence.get("winning_window_indexes", []))
if expected_status == "PASS" and (
    not isinstance(sequence, dict)
    or sequence.get("status") != "PASS"
    or sequence.get("timed_out") is not False
    or len(sequence.get("winning_window_indexes", [])) != 3
):
    raise SystemExit("runtime health JSON PASS lacks three consecutive windows")
print(f"RUNTIME_HEALTH_GATE_STATUS={expected_status}")
print(f"RUNTIME_HEALTH_GATE_EXIT_CODE={exit_status}")
print(f"RUNTIME_HEALTH_EVIDENCE_SHA256={evidence_sha256}")
print(f"RUNTIME_HEALTH_EVALUATED_WINDOWS={evaluated}")
print(f"RUNTIME_HEALTH_MAXIMUM_CONSECUTIVE_PASSES={maximum_consecutive}")
print(f"RUNTIME_HEALTH_WINNING_WINDOW_INDEXES={winning}")
print(f"RUNTIME_HEALTH_RVIZ_RECORDER_REQUIRED={str(rviz_required).lower()}")
print(f"RUNTIME_HEALTH_RVIZ_RECORDER_ACTIVE_DURING_PROBE={str(rviz_active).lower()}")
if expected_transport_profile in {
    "carla_vad_camera_source_5hz_best_effort_image_v2",
    "portable_e2e_exact_bundle_10hz_v2",
}:
    print(f"RUNTIME_HEALTH_CAMERA_IMAGE_GRAPH_STATUS={graph.get('status')}")
    print(
        "RUNTIME_HEALTH_TRANSPORT_ENVIRONMENT_STATUS="
        f"{transport_environment.get('status')}"
    )
print(f"RUNTIME_HEALTH_FINISHED_AT={payload['finished_at']}")
PY
  if (( runtime_health_exit_status != 0 )); then
    echo "Pre-engagement runtime health gate failed; see ${output_dir}/runtime_health.json" >&2
    exit 1
  fi
  require_carla_owner runtime_health_complete || exit 1
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "Autoware stack exited during the runtime health gate" >&2
    exit 1
  fi
  if [[ "${capture_desktop}" == "true" ]]; then
    require_desktop_recorder runtime_health_complete
    verify_owned_rviz_capture_window runtime_health_complete
  fi
  critical_failure=""
  if critical_failure="$(
    critical_stack_child_failure "${output_dir}/stack.log"
  )"; then
    printf '%s\n' "${critical_failure}" > \
      "${output_dir}/critical_process_failure.log"
    echo "Critical Autoware process exited during runtime health gate: ${critical_failure}" >&2
    exit 1
  fi
fi

# HH_260906 - Require a fresh native candidate and healthy route state immediately before evidence and engagement.
require_carla_owner pre_engagement_route_recheck || exit 1
if ! kill -0 "${stack_pid}" 2>/dev/null; then
  echo "Autoware stack exited before the pre-engagement route recheck" >&2
  exit 1
fi
if ! timeout 5 ros2 topic echo /planning/vad/candidate_trajectories \
  autoware_internal_planning_msgs/msg/CandidateTrajectories \
  --once --no-daemon --qos-reliability reliable >/dev/null 2>&1; then
  echo "No fresh native VAD candidate at the pre-engagement route recheck" >&2
  exit 1
fi
pre_engagement_route_status="$(
  timeout 3 ros2 topic echo /planning/vad_route/status std_msgs/msg/String \
    --once --no-daemon --qos-reliability reliable 2>/dev/null || true
)"
if ! grep -Eq '^data: ready$' <<< "${pre_engagement_route_status}" && \
   ! grep -Eq '^data: stopping$' <<< "${pre_engagement_route_status}"; then
  echo "VAD route manager is not healthy at the pre-engagement route recheck: $(
    grep -E '^data:' <<< "${pre_engagement_route_status}" || echo 'status unavailable'
  )" >&2
  exit 1
fi
printf 'VAD_ROUTE_READY_RECHECK_PHASE=after_runtime_health_before_rosbag_and_engagement\nVAD_ROUTE_READY_RECHECK_FRESH_CANDIDATE=true\nVAD_ROUTE_READY_RECHECK_STATUS=pass\n' >> \
  "${output_dir}/runtime.env"

setsid scripts/e2e/record_turn_dynamics.sh "${output_dir}/bag" \
  > "${output_dir}/recorder.log" 2>&1 &
recorder_pid=$!
recorder_pgid="${recorder_pid}"
sleep 1
if ! kill -0 "${recorder_pid}" 2>/dev/null; then
  echo "Turn recorder failed to start" >&2
  exit 1
fi

portable_shadow_window_started_seconds=""
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  portable_recorder_ready=false
  portable_recorder_deadline=$((SECONDS + 10))
  while (( SECONDS < portable_recorder_deadline )); do
    if ! kill -0 "${recorder_pid}" 2>/dev/null; then
      break
    fi
    recorder_node_count="$(
      ros2 node list --no-daemon 2>/dev/null |
        awk '$0 == "/rosbag2_recorder" {count += 1} END {print count + 0}'
    )"
    if [[ "${recorder_node_count}" == "1" ]]; then
      portable_recorder_ready=true
      break
    fi
    sleep 0.25
  done
  if [[ "${portable_recorder_ready}" != "true" ]]; then
    echo "Owned rosbag recorder was not uniquely ready before Portable E2E launch." >&2
    exit 1
  fi
  start_portable_shadow
  portable_shadow_window_started_seconds="${SECONDS}"
  printf 'PORTABLE_SHADOW_WINDOW_RECORDER_STARTED_FIRST=true\nPORTABLE_SHADOW_WINDOW_STARTED_SECONDS=%s\n' \
    "${portable_shadow_window_started_seconds}" >> "${output_dir}/runtime.env"
fi

set +e
route_evaluation_started_at="$(date --utc +%Y-%m-%dT%H:%M:%S.%6NZ)"
route_test_arguments=(
  --full-stack
  --route-file "${route_file}"
  --result "${output_dir}/result.json"
)
if [[ "${speed_30kph}" == "true" || "${speed_60kph_pilot}" == "true" ]]; then
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
  if [[ "${portable_shadow_10hz}" == "true" ]] && ! portable_shadow_alive; then
    echo "Portable E2E shadow exited during route evaluation" >&2
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
require_portable_shadow route_completion || exit 1
if ! python3 scripts/e2e/probe_carla_server.py \
  "${carla_probe_args[@]}" --stage trial_completion \
  --output "${output_dir}/carla_completion_health.json"; then
  echo "CARLA failed the post-route read-only RPC/map/snapshot check" >&2
  exit 1
fi

capture_portable_shadow_window_final

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

if ! e2e_stop_owned_process_group \
  "${recorder_pgid}" "${recorder_pid}" 30 10 3; then
  echo "Owned turn recorder did not stop cleanly before post-processing." >&2
  exit 1
fi
recorder_pid=""
recorder_pgid=""
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
if [[ "${portable_shadow_10hz}" == "true" ]]; then
  if ! python3 "${portable_shadow_analyzer}" \
    --bag "${output_dir}/bag" \
    --trial-provenance-json \
      "${output_dir}/portable_shadow_provenance/analyzer_provenance.json" \
    --window-boundaries-json \
      "${output_dir}/portable_shadow_provenance/window_boundaries.json" \
    --require-ten-hz-pass \
    --output \
      "${output_dir}/portable_shadow_provenance/shadow_evidence_analysis.json" > \
    "${output_dir}/portable_shadow_provenance/shadow_evidence_analysis.log" 2>&1; then
    echo "Portable E2E shadow evidence analysis failed." >&2
    analysis_status=1
  else
    portable_shadow_analysis_sha256="$(
      sha256sum -- \
        "${output_dir}/portable_shadow_provenance/shadow_evidence_analysis.json" |
        awk '{print $1}'
    )"
    portable_shadow_manifest="${output_dir}/portable_shadow_provenance/SHA256SUMS"
    portable_shadow_manifest_staged="${portable_shadow_manifest}.staged.$$"
    portable_shadow_manifest_files=(
      analyzer_provenance.json
      armed_status.yaml
      final_status.yaml
      launch_recheck.json
      launch_recheck.log
      node_info.err
      node_info.txt
      runtime_shadow.launch.xml
      recorder_subscriptions.json
      seal_receipt.json
      shadow_evidence_analysis.json
      shadow_evidence_analysis.log
      startup_status.yaml
      startup_validation.json
      trial_binding.json
      validation.log
      window_boundaries.json
    )
    : > "${portable_shadow_manifest_staged}"
    for name in "${portable_shadow_manifest_files[@]}"; do
      file="${output_dir}/portable_shadow_provenance/${name}"
      if [[ -L "${file}" || ! -f "${file}" ]]; then
        rm -f -- "${portable_shadow_manifest_staged}"
        echo "Portable E2E provenance manifest input is missing: ${name}" >&2
        analysis_status=1
        break
      fi
      printf '%s  %s\n' "$(sha256sum -- "${file}" | awk '{print $1}')" \
        "${name}" >> "${portable_shadow_manifest_staged}"
    done
    if [[ "${analysis_status}" == "0" ]]; then
      mv -- "${portable_shadow_manifest_staged}" "${portable_shadow_manifest}"
    fi
    portable_shadow_manifest_sha256=""
    if [[ -f "${portable_shadow_manifest}" ]]; then
      portable_shadow_manifest_sha256="$(
        sha256sum -- "${portable_shadow_manifest}" | awk '{print $1}'
      )"
    fi
    printf 'PORTABLE_SHADOW_EVIDENCE_ANALYSIS_FILE=%s\nPORTABLE_SHADOW_EVIDENCE_ANALYSIS_SHA256=%s\nPORTABLE_SHADOW_PROVENANCE_MANIFEST_FILE=%s\nPORTABLE_SHADOW_PROVENANCE_MANIFEST_SHA256=%s\n' \
      "${output_dir}/portable_shadow_provenance/shadow_evidence_analysis.json" \
      "${portable_shadow_analysis_sha256}" "${portable_shadow_manifest}" \
      "${portable_shadow_manifest_sha256}" >> "${output_dir}/runtime.env"
  fi
fi
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
  # HH_260906 - Derive the candidate still from frame zero after all owned runtime processes stop.
  elif ! ffmpeg -y -loglevel error \
    -i "${output_dir}/autoware_rviz_capture.mkv" \
    -frames:v 1 -an "${output_dir}/autoware_rviz_candidate.png"; then
    echo "Failed to extract the initial Autoware/RViz candidate PNG" >&2
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
    "${capture_rviz_window_pid}" "${capture_rviz_window_pgid}" \
    "${portable_shadow_10hz}" "${capture_framerate_fps}" \
    "${capture_filter_threads}" "${capture_encoder}" \
    "${capture_encoder_preset}" "${capture_encoder_crf}" \
    "${capture_encoder_threads}" "${capture_ffmpeg_thread_policy}" <<'PY'
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
portable_shadow_enabled = sys.argv[22] == "true"
live_recording_policy = {
    "input_format": "x11grab",
    "framerate_fps": int(sys.argv[23]),
    "filter_threads": int(sys.argv[24]),
    "video_encoder": sys.argv[25],
    "preset": sys.argv[26],
    "crf": int(sys.argv[27]),
    "encoder_threads": int(sys.argv[28]),
    "pixel_format": "yuv420p",
    "thread_policy": sys.argv[29],
}
expected_live_recording_policy = {
    "input_format": "x11grab",
    "framerate_fps": 5,
    "filter_threads": 1,
    "video_encoder": "libx264",
    "preset": "ultrafast",
    "crf": 20,
    "encoder_threads": 2,
    "pixel_format": "yuv420p",
    "thread_policy": "bounded_ffmpeg_workers_v1",
}
if live_recording_policy != expected_live_recording_policy:
    raise SystemExit(
        f"owned RViz live-recording policy changed: {live_recording_policy!r}"
    )

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
if portable_shadow_enabled:
    required_path_topics.update(
        {
            "/planning/portable_e2e/shadow_path",
            "/planning/portable_e2e/shadow_trajectory",
        }
    )
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
    "candidate_still": {
        "source": "autoware_rviz_capture.mkv",
        "offset_sec": 0.0,
        "selection": "first_recorded_frame",
        "extracted_after_owned_runtime_cleanup": True,
    },
    "gif_file": "autoware_rviz_drive.gif",
    "recording_file": "autoware_rviz_capture.mkv",
    "live_recording_policy": live_recording_policy,
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
if [[ "${speed_30kph}" == "true" || "${speed_60kph_pilot}" == "true" ]]; then
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
    if [[ "${speed_30kph}" == "true" ]]; then
      echo "30 km/h speed-source analysis failed; see ${output_dir}/speed_profile_analysis.log" >&2
    else
      echo "60 km/h speed-source analysis failed; see ${output_dir}/speed_profile_analysis.log" >&2
    fi
    analysis_status=1
  fi

  # The command converter indexes its CSV maps by observed odometry speed, not
  # by the requested cruise target.  Preserve a separate post-run audit so an
  # out-of-axis target is never mislabeled as an observed clamped lookup.
  runtime_coverage_status=0
  observed_maximum_speed_mps="$(
    python3 - "${output_dir}/result.json" <<'PY'
import json
import math
from pathlib import Path
import sys

result = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
metrics = result.get("metrics")
if not isinstance(metrics, dict):
    raise SystemExit("route result has no metrics object")
value = metrics.get("maximum_observed_speed_mps")
if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
    raise SystemExit("route result has no finite maximum observed speed")
print(f"{float(value):.17g}")
PY
  )" || runtime_coverage_status=$?
  if (( runtime_coverage_status == 0 )); then
    runtime_coverage_arguments=(
      --provenance-dir "${output_dir}/actuation_config_provenance"
      --profile-id "${speed_profile_id}"
      --target-speed-mps "${target_speed_mps}"
      --observed-maximum-speed-mps "${observed_maximum_speed_mps}"
      --output "${output_dir}/actuation_map_runtime_coverage.json"
    )
    if [[ "${speed_60kph_pilot}" == "true" ]]; then
      runtime_coverage_arguments+=(--allow-target-envelope-beyond-axis)
    fi
    python3 scripts/e2e/analyze_actuation_map_coverage.py \
      "${runtime_coverage_arguments[@]}" || runtime_coverage_status=$?
  fi
  if (( runtime_coverage_status != 0 )); then
    echo "Post-run actuation-map lookup audit failed" >&2
    analysis_status=1
  fi

  longitudinal_analysis_status=0
  longitudinal_analysis_arguments=(
    --bag "${output_dir}/bag"
    --route-file "${route_file}"
    --result "${output_dir}/result.json"
    --output-dir "${output_dir}"
    --profile-id "${speed_profile_id}"
    --target-speed-mps "${target_speed_mps}"
    --longitudinal-speed-source explicit_simulation_nominal
  )
  if [[ -f "${output_dir}/actuation_map_runtime_coverage.json" ]]; then
    longitudinal_analysis_arguments+=(
      --actuation-map-coverage \
        "${output_dir}/actuation_map_runtime_coverage.json"
    )
  fi
  python3 scripts/e2e/analyze_longitudinal_response.py \
    "${longitudinal_analysis_arguments[@]}" > \
    "${output_dir}/longitudinal_response_analysis.log" 2>&1 || \
    longitudinal_analysis_status=$?
  if (( longitudinal_analysis_status != 0 )); then
    echo "Longitudinal response analysis failed; see ${output_dir}/longitudinal_response_analysis.log" >&2
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
# HH_260906 - Render straight routes directly without an expected turn-crop failure.
if [[ "${route_scenario}" == "straight" ]]; then
  python3 scripts/e2e/render_turn_animation.py \
    "${animation_arguments[@]}" --crop motion || analysis_status=$?
elif ! python3 scripts/e2e/render_turn_animation.py \
  "${animation_arguments[@]}" --crop turn; then
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
