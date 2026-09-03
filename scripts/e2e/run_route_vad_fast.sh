#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

usage() {
  cat >&2 <<EOF
Usage: $0 [--full] [--visualize|--rviz-only] [--recommended] [--speed-30kph|--speed-60kph-pilot] [--camera-source-5hz] [--control-ab-pid-i40|--control-ab-turn-preview-5m] [--geometry-ab-route-corridor-0p2] [--tight-corridor] [--trajectory-stability] [--fp16-heads] [--model-override YAML] [--sensor-mapping YAML] ROUTE_JSON [ros2 launch arguments...]

  default       Minimal Autoware control shell and the lowest runtime load
  --full        Full Autoware shell; RViz stays off unless a visual option is set
  --visualize   Full mode only: start RViz and a front-camera rqt window
  --rviz-only   Full mode only: start RViz without the external camera window
  --recommended Run the repeat-screened full-stack parameter profile
  --speed-30kph Add the guarded 8.333 m/s screening profile to --recommended
  --speed-60kph-pilot
                Add the straight-only CARLA 16.667 m/s exploratory pilot
                profile to --recommended; never use this on a real vehicle
  --camera-source-5hz
                With --recommended, render all six CARLA cameras at 5 sim-Hz
                with localhost-only, depth-1 best-effort raw images, reliable
                camera_info, and continuous IMU
  --control-ab-pid-i40
                30 kph A/B only: change PID max_i_effort from 0.30 to 0.40
  --control-ab-turn-preview-5m
                30 kph A/B only: change curvature speed preview from 3 m to 5 m
  --geometry-ab-route-corridor-0p2
                60 kph A/B only: change route corridor from +/-0.50 m to
                +/-0.20 m; keep speed, controller, gate, map, and throttle fixed
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
speed_60kph_pilot=false
camera_source_5hz=false
trajectory_stability=false
tight_corridor=false
fp16_heads=false
model_override=""
sensor_mapping=""
control_ab_pid_i40=false
control_ab_turn_preview_5m=false
geometry_ab_route_corridor_0p2=false
route_corridor_half_width_m="0.50"
turn_outward_corridor_half_width_m="0.50"
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
    --speed-60kph-pilot)
      speed_60kph_pilot=true
      recommended=true
      full=true
      shift
      ;;
    --camera-source-5hz)
      camera_source_5hz=true
      recommended=true
      full=true
      shift
      ;;
    --control-ab-pid-i40)
      control_ab_pid_i40=true
      shift
      ;;
    --control-ab-turn-preview-5m)
      control_ab_turn_preview_5m=true
      shift
      ;;
    --geometry-ab-route-corridor-0p2)
      geometry_ab_route_corridor_0p2=true
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

if [[ "${camera_source_5hz}" == "true" && -n "${sensor_mapping}" ]]; then
  echo "--camera-source-5hz and --sensor-mapping are mutually exclusive." >&2
  exit 2
fi

if [[ "${speed_30kph}" == "true" && "${speed_60kph_pilot}" == "true" ]]; then
  echo "--speed-30kph and --speed-60kph-pilot are mutually exclusive." >&2
  exit 2
fi

if [[ "${control_ab_pid_i40}" == "true" && \
      "${control_ab_turn_preview_5m}" == "true" ]]; then
  echo "Select exactly one isolated 30 kph control A/B candidate per trial." >&2
  exit 2
fi
if [[ ( "${control_ab_pid_i40}" == "true" || \
        "${control_ab_turn_preview_5m}" == "true" ) && \
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

if [[ "${speed_60kph_pilot}" == "true" ]]; then
  if [[ ! -f "${route_file}" ]]; then
    echo "Route file not found: ${route_file}" >&2
    exit 2
  fi
  route_scenario="$(
    python3 - "${route_file}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    value = json.load(stream).get("scenario")
print(value if isinstance(value, str) and value else "unknown")
PY
  )"
  if [[ "${route_scenario}" != "straight" ]]; then
    echo "--speed-60kph-pilot requires a straight route; got ${route_scenario}" >&2
    exit 2
  fi
fi

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
      use_vad_imu_acceleration:=*|use_lateral_controller_param_override:=*|lateral_controller_param_path:=*|use_longitudinal_controller_param_override:=*|longitudinal_controller_param_path:=*|vehicle_cmd_gate_param_path:=*|controller_stop_offset_m:=*|comfortable_deceleration_mps2:=*|maximum_longitudinal_acceleration_mps2:=*|longitudinal_velocity_source:=*|nominal_cruise_speed_mps:=*|maneuver_lookahead_m:=*|maneuver_exit_lookahead_m:=*|route_corridor_half_width_m:=*|turn_inward_corridor_half_width_m:=*|turn_outward_corridor_half_width_m:=*|left_turn_outward_corridor_half_width_m:=*|right_turn_outward_corridor_half_width_m:=*|route_corridor_entry_distance_m:=*|trajectory_lateral_filter_gain:=*|left_turn_trajectory_lateral_filter_gain:=*|right_turn_trajectory_lateral_filter_gain:=*|trajectory_lateral_filter_activation_threshold_m:=*|trajectory_geometry_smoothing_strength:=*|maximum_lateral_acceleration_mps2:=*|curvature_speed_preview_m:=*|route_curvature_lookahead_m:=*|max_route_deviation_m:=*|max_candidate_age_sec:=*|candidate_timeout_sec:=*|maximum_speed_mps:=*|raw_vehicle_cmd_converter_config:=*)
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

if [[ "${tight_corridor}" == "true" || \
      "${geometry_ab_route_corridor_0p2}" == "true" ]]; then
  route_corridor_half_width_m="0.20"
  turn_outward_corridor_half_width_m="0.20"
fi

recommended_mpc=""
speed_gate=""
speed_pid=""
cyclonedds_config=""
if [[ "${recommended}" == "true" ]]; then
  fast_mapping="${package_share}/config/sensor_mapping_vad_fast_reliable_imu.yaml"
  if [[ "${camera_source_5hz}" == "true" ]]; then
    fast_mapping="${package_share}/config/sensor_mapping_vad_fast_imu_camera_source_5hz_best_effort_image_depth1.yaml"
  fi
  model_override="${package_share}/config/vad_carla_tiny_recommended.param.yaml"
  if [[ "${camera_source_5hz}" == "true" ]]; then
    model_override="${package_share}/config/vad_carla_tiny_camera_source_5hz_best_effort_image_depth1.param.yaml"
    cyclonedds_config="${package_share}/config/cyclonedds_camera_depth1_localhost_v2.xml"
  fi
  recommended_mpc="${package_share}/config/mpc_carla_recommended.param.yaml"
  required_profile_files=("${fast_mapping}" "${model_override}" "${recommended_mpc}")
  if [[ "${camera_source_5hz}" == "true" ]]; then
    required_profile_files+=(
      "${fast_mapping}.metadata.json"
      "${model_override}.metadata.json"
      "${cyclonedds_config}"
      "${cyclonedds_config}.metadata.json"
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

if [[ "${camera_source_5hz}" == "true" ]]; then
  # This versioned simulation profile is intentionally isolated from LAN DDS
  # participants. env.sh clears inherited Cyclone settings before this exact
  # installed config is selected.
  # ROS_LOCALHOST_ONLY=1 makes Cyclone select loopback a second time and is
  # invalid together with the pinned <NetworkInterface name="lo"> profile.
  export ROS_LOCALHOST_ONLY=0
  export AUTOWARE_E2E_PINNED_CYCLONEDDS_URI="file://${cyclonedds_config}"
  export AUTOWARE_E2E_PINNED_CYCLONEDDS_SHA256="$(
    sha256sum -- "${cyclonedds_config}" | awk '{print $1}'
  )"
  export CYCLONEDDS_URI="${AUTOWARE_E2E_PINNED_CYCLONEDDS_URI}"
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
    "route_corridor_half_width_m:=${route_corridor_half_width_m}"
    "turn_inward_corridor_half_width_m:=0.20"
    "turn_outward_corridor_half_width_m:=${turn_outward_corridor_half_width_m}"
    "trajectory_geometry_smoothing_strength:=10.0"
  )
  if [[ "${speed_30kph}" == "true" ]]; then
    curvature_speed_preview_m=3.0
    if [[ "${control_ab_turn_preview_5m}" == "true" ]]; then
      curvature_speed_preview_m=5.0
    fi
    profile_arguments+=(
      "controller_stop_offset_m:=0.60"
      "comfortable_deceleration_mps2:=2.0"
      "maximum_longitudinal_acceleration_mps2:=1.5"
      "longitudinal_velocity_source:=explicit_simulation_nominal"
      "nominal_cruise_speed_mps:=8.333333333333334"
      "maneuver_lookahead_m:=4.0"
      "maneuver_exit_lookahead_m:=2.5"
      "maximum_lateral_acceleration_mps2:=1.2"
      "curvature_speed_preview_m:=${curvature_speed_preview_m}"
      "route_curvature_lookahead_m:=20.0"
      "max_candidate_age_sec:=0.5"
      "candidate_timeout_sec:=1.5"
      "max_route_deviation_m:=1.0"
      "maximum_speed_mps:=8.333333333333334"
      "vehicle_cmd_gate_param_path:=${speed_gate}"
      "longitudinal_controller_param_path:=${speed_pid}"
    )
  elif [[ "${speed_60kph_pilot}" == "true" ]]; then
    profile_arguments+=(
      "controller_stop_offset_m:=0.60"
      "comfortable_deceleration_mps2:=2.0"
      "maximum_longitudinal_acceleration_mps2:=1.5"
      "longitudinal_velocity_source:=explicit_simulation_nominal"
      "nominal_cruise_speed_mps:=16.666666666666668"
      "maneuver_lookahead_m:=6.0"
      "maneuver_exit_lookahead_m:=3.5"
      "maximum_lateral_acceleration_mps2:=1.0"
      "curvature_speed_preview_m:=6.0"
      "route_curvature_lookahead_m:=40.0"
      "max_candidate_age_sec:=0.5"
      "candidate_timeout_sec:=1.5"
      "max_route_deviation_m:=1.0"
      "maximum_speed_mps:=16.666666666666668"
      "vehicle_cmd_gate_param_path:=${speed_gate}"
      "longitudinal_controller_param_path:=${speed_pid}"
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

if [[ "${trajectory_stability}" == "true" ]]; then
  profile_arguments+=(
    "right_turn_trajectory_lateral_filter_gain:=0.75"
    "trajectory_lateral_filter_activation_threshold_m:=0.20"
  )
fi

if [[ "${trajectory_stability}" == "true" ]]; then
  echo "NOTICE: this outlier filter is HOLD after repeated closed-loop screening; it is not the recommended profile." >&2
fi

echo "VAD fast profile: 6x 640x360 raw cameras at 5 Hz; recommended=${recommended}; speed30=${speed_30kph}; speed60pilot=${speed_60kph_pilot}; camera source 5 sim-Hz=${camera_source_5hz}; control AB pid-i40=${control_ab_pid_i40}; control AB turn-preview-5m=${control_ab_turn_preview_5m}; geometry AB route-corridor-0p2=${geometry_ab_route_corridor_0p2}; tight corridor=${tight_corridor}; trajectory stability=${trajectory_stability}; mixed FP16 heads=${fp16_heads}; sensor mapping=${fast_mapping}; CycloneDDS=${cyclonedds_config:-inherited}" >&2
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
