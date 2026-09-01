#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${root}"
source scripts/e2e/env.sh

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 OUTPUT_BAG" >&2
  exit 2
fi

output_bag="$(realpath -m -- "$1")"
if [[ -e "${output_bag}" || -L "${output_bag}" ]]; then
  echo "Output bag already exists: ${output_bag}" >&2
  exit 2
fi

# Two rosbag2 recorders use the same graph node name and trip Autoware's
# duplicated-node safety check. Serialize this project helper per ROS domain,
# then reject recorders started outside the helper before opening the bag.
exec 9>"/tmp/autoware_e2e_turn_recorder_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo "Another turn-dynamics recorder is already starting or running." >&2
  exit 1
fi
active_nodes="$(ros2 node list --no-daemon)"
if grep -Fxq "/rosbag2_recorder" <<< "${active_nodes}"; then
  echo "A rosbag2 recorder is already present in ROS domain ${ROS_DOMAIN_ID}." >&2
  echo "Stop it cleanly before starting another recording." >&2
  exit 1
fi

topic_regex='^/('
topic_regex+='clock|'
topic_regex+='localization/kinematic_state|'
topic_regex+='localization/acceleration|'
topic_regex+='sensing/camera/CAM_FRONT/camera_info|'
topic_regex+='sensing/camera/CAM_BACK/camera_info|'
topic_regex+='sensing/camera/CAM_FRONT_LEFT/camera_info|'
topic_regex+='sensing/camera/CAM_BACK_LEFT/camera_info|'
topic_regex+='sensing/camera/CAM_FRONT_RIGHT/camera_info|'
topic_regex+='sensing/camera/CAM_BACK_RIGHT/camera_info|'
topic_regex+='planning/vad/candidate_trajectories|'
topic_regex+='planning/vad/raw_trajectory|'
topic_regex+='planning/vad_route/selected_raw_trajectory|'
topic_regex+='planning/vad_route/command|'
topic_regex+='planning/vad_route/cross_track_error|'
topic_regex+='planning/vad_route/trajectory_correction|'
topic_regex+='planning/vad_route/remaining_distance|'
topic_regex+='planning/vad_route/status|'
topic_regex+='planning/vad_route/goal_reached|'
topic_regex+='perception/vad/map_points|'
topic_regex+='perception/object_recognition/objects|'
topic_regex+='diagnostics|'
topic_regex+='control/autonomous_emergency_braking/debug/rss_distance|'
topic_regex+='control/autonomous_emergency_braking/debug/markers|'
topic_regex+='control/autonomous_emergency_braking/virtual_wall|'
topic_regex+='control/autonomous_emergency_braking/metrics|'
topic_regex+='planning/route_state|'
topic_regex+='planning/trajectory|'
topic_regex+='control/trajectory_follower/lateral/predicted_trajectory|'
topic_regex+='control/trajectory_follower/predicted_trajectory|'
topic_regex+='control/trajectory_follower/lateral/diagnostic|'
topic_regex+='control/trajectory_follower/longitudinal/diagnostic|'
topic_regex+='control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms|'
topic_regex+='control/trajectory_follower/controller_node_exe/longitudinal/debug/processing_time_ms|'
topic_regex+='debug_mpc/reference_horizon_goal_stop_guard_active|'
topic_regex+='debug_mpc/reference_horizon_goal_stop_guard_reason|'
topic_regex+='debug_mpc/reference_horizon_goal_stop_distance|'
topic_regex+='debug_mpc/reference_horizon_goal_stop_floored_points|'
topic_regex+='debug_mpc_v_des|'
topic_regex+='debug_mpc_acc_des|'
topic_regex+='debug_mpc_x_current|'
topic_regex+='debug_mpc_nominal_inputs|'
topic_regex+='debug_mpc_emergency_stop_mode|'
topic_regex+='debug_mpc_goal_stop_mode|'
topic_regex+='control/trajectory_follower/control_cmd|'
topic_regex+='trajectory_follower/control_cmd|'
topic_regex+='control/command/control_cmd|'
topic_regex+='control/command/actuation_cmd|'
topic_regex+='control/command/gear_cmd|'
topic_regex+='vehicle/status/velocity_status|'
topic_regex+='vehicle/status/steering_status|'
topic_regex+='vehicle/status/actuation_status|'
topic_regex+='vehicle/status/gear_status|'
topic_regex+='system/fail_safe/mrm_state|'
topic_regex+='api/operation_mode/state|'
topic_regex+='api/routing/state|'
topic_regex+='autoware/state'
topic_regex+=')$'

echo "Recording turn dynamics to ${output_bag}"
echo "Press Ctrl-C to stop and finalize the bag."

# Keep rosbag2 in the foreground so its SIGINT handler flushes storage and metadata.
exec ros2 bag record --output "${output_bag}" --regex "${topic_regex}"
