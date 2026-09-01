#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

usage() {
  cat >&2 <<EOF
Usage: $0 [--full] [--visualize|--rviz-only] [--recommended] [--speed-30kph] [--tight-corridor] [--trajectory-stability] [--fp16-heads] [--model-override YAML] [--sensor-mapping YAML] ROUTE_JSON [ros2 launch arguments...]

  default       Minimal Autoware control shell and the lowest runtime load
  --full        Full Autoware shell; RViz stays off unless a visual option is set
  --visualize   Full mode only: start RViz and a front-camera rqt window
  --rviz-only   Full mode only: start RViz without the external camera window
  --recommended Run the repeat-screened full-stack parameter profile
  --speed-30kph Add the guarded 8.333 m/s screening profile to --recommended
  --tight-corridor
                With --recommended, constrain every route command to +/-0.20 m
  --trajectory-stability
                Add the repeat-screened experimental HOLD outlier filter
  --fp16-heads  Mixed TensorRT FP16 heads with LayerNorm kept in FP32
  --model-override YAML
                 Apply an editable deployment parameter overlay last
  --sensor-mapping YAML
                 Use an explicit fast-profile sensor mapping for an A/B trial
EOF
}

full=false
visualize=false
rviz_only=false
recommended=false
speed_30kph=false
trajectory_stability=false
tight_corridor=false
fp16_heads=false
model_override=""
sensor_mapping=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --full)
      full=true
      shift
      ;;
    --visualize)
      visualize=true
      shift
      ;;
    --rviz-only)
      rviz_only=true
      shift
      ;;
    --recommended)
      recommended=true
      full=true
      shift
      ;;
    --speed-30kph)
      speed_30kph=true
      recommended=true
      full=true
      shift
      ;;
    --trajectory-stability)
      trajectory_stability=true
      recommended=true
      full=true
      shift
      ;;
    --tight-corridor)
      tight_corridor=true
      shift
      ;;
    --fp16-heads)
      fp16_heads=true
      shift
      ;;
    --model-override)
      if [[ $# -lt 2 ]]; then
        echo "--model-override requires a YAML file." >&2
        exit 2
      fi
      model_override="$2"
      shift 2
      ;;
    --sensor-mapping)
      if [[ $# -lt 2 ]]; then
        echo "--sensor-mapping requires a YAML file." >&2
        exit 2
      fi
      sensor_mapping="$2"
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
  if [[ "${fp16_heads}" == "true" || -n "${model_override}" || -n "${sensor_mapping}" ]]; then
    echo "--recommended controls precision, VAD synchronization, and camera transport." >&2
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

if [[ "${visualize}" == "true" && "${full}" != "true" ]]; then
  echo "--visualize requires --full. For minimal mode, run run_visualization.sh --fast separately." >&2
  exit 2
fi
if [[ "${rviz_only}" == "true" && "${full}" != "true" ]]; then
  echo "--rviz-only requires --full." >&2
  exit 2
fi
if [[ "${visualize}" == "true" && "${rviz_only}" == "true" ]]; then
  echo "--visualize and --rviz-only are mutually exclusive." >&2
  exit 2
fi

if [[ -n "${model_override}" && "${fp16_heads}" == "true" ]]; then
  echo "Use either --fp16-heads or --model-override so the last precision setting is unambiguous." >&2
  exit 2
fi

if [[ -n "${model_override}" ]]; then
  if [[ ! -f "${model_override}" ]]; then
    echo "Model override file not found: ${model_override}" >&2
    exit 2
  fi
  model_override="$(realpath "${model_override}")"
fi

if [[ -n "${sensor_mapping}" ]]; then
  if [[ ! -f "${sensor_mapping}" ]]; then
    echo "Sensor mapping file not found: ${sensor_mapping}" >&2
    exit 2
  fi
  sensor_mapping="$(realpath "${sensor_mapping}")"
fi

route_file="${1:-}"
if [[ -z "${route_file}" ]]; then
  usage
  exit 2
fi
shift

for argument in "$@"; do
  case "${argument}" in
    use_fast_vad:=*|vad_use_fp16_heads:=*|vad_model_override_file:=*|use_light_weight_sensor_mapping:=*|sensor_mapping_file:=*|rviz:=*|launch_fast_camera_view:=*)
      echo "Fast profile argument is controlled by this wrapper: ${argument%%:=*}" >&2
      exit 2
      ;;
    launch_conventional_perception:=true|launch_conventional_perception:=True|launch_conventional_perception:=TRUE)
      echo "Fast sensor mapping cannot run conventional perception." >&2
      exit 2
      ;;
  esac
  if [[ "${recommended}" == "true" ]]; then
    case "${argument}" in
      use_vad_imu_acceleration:=*|use_lateral_controller_param_override:=*|lateral_controller_param_path:=*|use_longitudinal_controller_param_override:=*|longitudinal_controller_param_path:=*|vehicle_cmd_gate_param_path:=*|controller_stop_offset_m:=*|comfortable_deceleration_mps2:=*|maximum_longitudinal_acceleration_mps2:=*|longitudinal_velocity_source:=*|nominal_cruise_speed_mps:=*|maneuver_lookahead_m:=*|maneuver_exit_lookahead_m:=*|turn_inward_corridor_half_width_m:=*|turn_outward_corridor_half_width_m:=*|left_turn_outward_corridor_half_width_m:=*|right_turn_outward_corridor_half_width_m:=*|route_corridor_entry_distance_m:=*|trajectory_lateral_filter_gain:=*|left_turn_trajectory_lateral_filter_gain:=*|right_turn_trajectory_lateral_filter_gain:=*|trajectory_lateral_filter_activation_threshold_m:=*|trajectory_geometry_smoothing_strength:=*|maximum_lateral_acceleration_mps2:=*|curvature_speed_preview_m:=*|route_curvature_lookahead_m:=*|max_route_deviation_m:=*|max_candidate_age_sec:=*|candidate_timeout_sec:=*|maximum_speed_mps:=*|raw_vehicle_cmd_converter_config:=*)
        echo "Recommended profile argument is controlled by this wrapper: ${argument%%:=*}" >&2
        exit 2
        ;;
    esac
  fi
done

package_share="$(ros2 pkg prefix autoware_e2e_vad_launch)/share/autoware_e2e_vad_launch"
fast_mapping="${package_share}/config/sensor_mapping_vad_fast.yaml"
fast_launch="${package_share}/launch/vad_carla_tiny_fast.launch.xml"
if [[ ! -f "${fast_mapping}" || ! -f "${fast_launch}" ]]; then
  echo "Fast profile is not installed. Run scripts/e2e/build.sh first." >&2
  exit 1
fi

recommended_mpc=""
speed_30kph_gate=""
speed_30kph_pid=""
if [[ "${recommended}" == "true" ]]; then
  fast_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable_imu.yaml"
  model_override="${package_share}/config/vad_carla_tiny_recommended.param.yaml"
  recommended_mpc="${package_share}/config/mpc_carla_recommended.param.yaml"
  required_profile_files=("${fast_mapping}" "${model_override}" "${recommended_mpc}")
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

if [[ -n "${sensor_mapping}" ]]; then
  fast_mapping="${sensor_mapping}"
fi

if [[ -z "${model_override}" ]]; then
  model_override="${package_share}/config/vad_carla_tiny_fast.param.yaml"
fi

profile_arguments=(
  "use_fast_vad:=true"
  "vad_use_fp16_heads:=${fp16_heads}"
  "vad_model_override_file:=${model_override}"
  "use_light_weight_sensor_mapping:=True"
  "sensor_mapping_file:=${fast_mapping}"
)

if [[ "${recommended}" == "true" ]]; then
  profile_arguments+=(
    "use_vad_imu_acceleration:=true"
    "use_lateral_controller_param_override:=true"
    "lateral_controller_param_path:=${recommended_mpc}"
    "use_longitudinal_controller_param_override:=true"
    "turn_inward_corridor_half_width_m:=0.20"
    "trajectory_geometry_smoothing_strength:=10.0"
  )
  if [[ "${speed_30kph}" == "true" ]]; then
    profile_arguments+=(
      "controller_stop_offset_m:=0.60"
      "comfortable_deceleration_mps2:=2.0"
      "maximum_longitudinal_acceleration_mps2:=1.5"
      "longitudinal_velocity_source:=explicit_simulation_nominal"
      "nominal_cruise_speed_mps:=8.333333333333334"
      "maneuver_lookahead_m:=4.0"
      "maneuver_exit_lookahead_m:=2.5"
      "maximum_lateral_acceleration_mps2:=1.2"
      "curvature_speed_preview_m:=3.0"
      "route_curvature_lookahead_m:=20.0"
      "max_candidate_age_sec:=0.5"
      "candidate_timeout_sec:=1.5"
      "max_route_deviation_m:=1.0"
      "maximum_speed_mps:=8.333333333333334"
      "vehicle_cmd_gate_param_path:=${speed_30kph_gate}"
      "longitudinal_controller_param_path:=${speed_30kph_pid}"
    )
  else
    profile_arguments+=(
      "controller_stop_offset_m:=0.60"
      "comfortable_deceleration_mps2:=0.60"
      "maneuver_lookahead_m:=3.0"
      "maximum_speed_mps:=2.5"
    )
  fi
fi

if [[ "${tight_corridor}" == "true" ]]; then
  profile_arguments+=(
    "route_corridor_half_width_m:=0.20"
    "turn_outward_corridor_half_width_m:=0.20"
  )
fi

if [[ "${trajectory_stability}" == "true" ]]; then
  profile_arguments+=(
    "right_turn_trajectory_lateral_filter_gain:=0.75"
    "trajectory_lateral_filter_activation_threshold_m:=0.20"
  )
fi

if [[ "${trajectory_stability}" == "true" ]]; then
  echo "NOTICE: this outlier filter is HOLD after repeated closed-loop screening; it is not the recommended profile." >&2
fi

echo "VAD fast profile: 6x 640x360 raw cameras at 5 Hz; recommended=${recommended}; speed30=${speed_30kph}; tight corridor=${tight_corridor}; trajectory stability=${trajectory_stability}; mixed FP16 heads=${fp16_heads}; sensor mapping=${fast_mapping}" >&2
if [[ "${full}" == "true" ]]; then
  rviz_enabled=false
  if [[ "${visualize}" == "true" || "${rviz_only}" == "true" ]]; then
    rviz_enabled=true
  fi
  profile_arguments+=(
    "rviz:=${rviz_enabled}"
    "launch_fast_camera_view:=${visualize}"
  )
  exec scripts/e2e/run_route_vad_full.sh \
    "${route_file}" "$@" "${profile_arguments[@]}"
fi

exec scripts/e2e/run_route_vad.sh \
  "${route_file}" "$@" "${profile_arguments[@]}"
