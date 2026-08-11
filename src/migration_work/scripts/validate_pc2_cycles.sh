#!/usr/bin/env bash

# HH_260810 - Added readiness-driven PC2 start/stop validation with exact launch-group cleanup checks.
source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash
source /home/a/ros2_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

readonly PC2_LAUNCHER="/home/a/autoware/src/migration_work/scripts/run_pc2_autoware.sh"
readonly PC2_PID_FILE="/tmp/autoware_pc2_run.pid"
readonly PC2_LOCK_FILE="/tmp/autoware_pc2_run.lock"
readonly PC2_CAMERA_ROUTE_CIDR="169.254.0.11/32"
# HH_260811 - Checked the physically proven serial-214 camera path while the launcher owns its runtime /32 route.
readonly PC2_CAMERA_IP="169.254.0.11"
readonly PC2_CAMERA_IFACE="enp1s0f3"
readonly PC2_CAMERA_HOST_IP="169.254.0.1"
readonly PC2_CAMERA_CONTRACT_PROBE="/home/a/autoware/src/migration_work/scripts/probe_camera_contract.py"
# HH_260811 - Required real YOLOX payloads from the slot consumed by the camera-LiDAR fusion nodes.
readonly PC2_YOLOX_CONTRACT_PROBE="/home/a/autoware/src/migration_work/scripts/probe_yolox_contract.py"
# HH_260810 - Matched the guarded launcher and pinned every validator subprocess to the PC3-reachable DDS NIC.
readonly PC2_DDS_IFACE="enp0s31f6"
readonly PC2_CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="enp0s31f6"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
readonly PC2_CYCLE_RUN_ID="$(date +%Y%m%d_%H%M%S)"
readonly PC2_CYCLE_LOG_DIR="/home/a/autoware/src/migration_work/test_logs/live_camera_cycles/${PC2_CYCLE_RUN_ID}"
readonly CYCLE_COUNT="${1:-5}"
readonly REQUIRE_PC3_INPUTS="${REQUIRE_PC3_INPUTS:-0}"

# HH_260810 - Rejected inherited DDS settings that would make the validator and guarded launcher inspect different graphs.
if [[ -n "${CYCLONEDDS_URI:-}" && "${CYCLONEDDS_URI}" != "${PC2_CYCLONEDDS_URI}" ]]; then
  echo "A conflicting CYCLONEDDS_URI is already set; refusing an ambiguous PC2 validation." >&2
  exit 77
fi
export CYCLONEDDS_URI="${PC2_CYCLONEDDS_URI}"

if [[ "${RMW_IMPLEMENTATION}" != "rmw_cyclonedds_cpp" ]]; then
  echo "PC2 validation requires rmw_cyclonedds_cpp for the audited DDS interface pin." >&2
  exit 77
fi
if [[ ! "${CYCLE_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Cycle count must be a positive integer." >&2
  exit 64
fi
if [[ ! "${REQUIRE_PC3_INPUTS}" =~ ^[01]$ ]]; then
  echo "REQUIRE_PC3_INPUTS must be 0 or 1." >&2
  exit 64
fi
if (( REQUIRE_PC3_INPUTS == 1 )) &&
   { [[ "${ROS_DOMAIN_ID}" != "10" ]] || [[ "${ROS_LOCALHOST_ONLY}" != "0" ]]; }; then
  echo "REQUIRE_PC3_INPUTS=1 requires ROS_DOMAIN_ID=10 and ROS_LOCALHOST_ONLY=0." >&2
  exit 78
fi

# HH_260810 - Limited cleanup checks to the outputs owned by this PC2 launch while PC3 remains online.
readonly -a PC2_OWNED_OUTPUT_TOPICS=(
  /lucid_vision/camera/image
  /lucid_vision/camera/image_compressed
  /lucid_vision/camera/camera_info
  /sensing/camera/camera1/traffic_light/image_raw
  /sensing/camera/camera1/traffic_light/image_raw/compressed
  /sensing/camera/camera1/traffic_light/camera_info
  /perception/object_recognition/detection/rois0
  /perception/traffic_light_recognition/traffic_signals
  /perception/object_recognition/objects
)

# HH_260810 - Used command publisher deltas plus launch-process evidence for the no-actuation boundary.
readonly -a PC2_SAFETY_TOPICS=(
  /control/command/control_cmd
  /control/command/actuation_cmd
  /control/command/gear_cmd
  /control/command/emergency_cmd
  /control/command/hazard_lights_cmd
  /control/command/turn_indicators_cmd
  /control/trajectory_follower/control_cmd
  /control/shift_decider/gear_cmd
  /control/control_mode_request
  /vehicle/command/actuation_cmd
  /vehicle/command/control_cmd
  /vehicle/command/manual_control_cmd
  /vehicle/command/manual_gear_command
  /pacmod/to_can_bus
  /to_can_bus
)

declare -a pc2_output_baseline_counts=()
declare -a safety_baseline_counts=()
forbidden_node_baseline=""
overall_status=0
active_wrapper_pid=""

mkdir -p "${PC2_CYCLE_LOG_DIR}"
echo "cycle_run_id=${PC2_CYCLE_RUN_ID} count=${CYCLE_COUNT} require_pc3_inputs=${REQUIRE_PC3_INPUTS} dds_iface=${PC2_DDS_IFACE} log_dir=${PC2_CYCLE_LOG_DIR}"

publisher_count()
{
  local topic="$1"
  local count=""
  local output=""
  # HH_260810 - Used direct DDS discovery so validation cannot leave a ros2cli daemon behind.
  if output="$(timeout 5s ros2 topic info "${topic}" --no-daemon 2>&1)"; then
    count="$(awk '/Publisher count:/ {print $3; exit}' <<< "${output}")"
    if [[ "${count}" =~ ^[0-9]+$ ]]; then
      echo "${count}"
      return 0
    fi
  fi
  if grep -q '^Unknown topic ' <<< "${output}"; then
    echo 0
    return 0
  fi

  # HH_260810 - Kept discovery/CLI failures distinct from a confirmed zero-publisher endpoint.
  echo -1
  return 1
}

subscription_count()
{
  local topic="$1"
  local count=""
  local output=""
  if output="$(timeout 5s ros2 topic info "${topic}" --no-daemon 2>&1)"; then
    count="$(awk '/Subscription count:/ {print $3; exit}' <<< "${output}")"
    if [[ "${count}" =~ ^[0-9]+$ ]]; then
      echo "${count}"
      return 0
    fi
  fi
  if grep -q '^Unknown topic ' <<< "${output}"; then
    echo 0
    return 0
  fi

  echo -1
  return 1
}

direct_nodes()
{
  local output=""
  if output="$(timeout 8s ros2 node list --all --no-daemon 2>&1)"; then
    printf '%s\n' "${output}"
    return 0
  fi

  # HH_260810 - Prevented a failed graph query from satisfying an empty-PC2-graph cleanup check.
  echo "__GRAPH_QUERY_ERROR__"
  return 1
}

# HH_260810 - Selected only namespaces and uniquely named nodes owned by the PC2 sensing/perception launch.
pc2_owned_nodes()
{
  grep -E '^/perception(/|$)|^/sensing/camera(/|$)|^/lucid_vision(/|$)|^/pc2_perception_pointcloud_container$|^/pointcloud_container/glog_component$' || true
}

forbidden_control_vehicle_nodes()
{
  grep -E '^/(control|vehicle)(/|$)|/(socket_can|pacmod|vehicle_cmd_gate|trajectory_follower|control_validator|operation_mode_transition_manager)(/|$|_)' || true
}

# HH_260810 - Required the four single-owner PC3 data endpoints and the two shared TF endpoint sets.
pc3_input_snapshot()
{
  pc3_before_sync_publishers="$(publisher_count /sensing/lidar/top/pointcloud_before_sync)"
  pc3_raw_ex_publishers="$(publisher_count /sensing/lidar/top/pointcloud_raw_ex)"
  pc3_vector_map_publishers="$(publisher_count /map/vector_map)"
  pc3_pointcloud_map_publishers="$(publisher_count /map/pointcloud_map)"
  pc3_tf_publishers="$(publisher_count /tf)"
  pc3_tf_static_publishers="$(publisher_count /tf_static)"
  pc3_inputs_ready="FAIL"

  if (( pc3_before_sync_publishers == 1 )) &&
     (( pc3_raw_ex_publishers == 1 )) &&
     (( pc3_vector_map_publishers == 1 )) &&
     (( pc3_pointcloud_map_publishers == 1 )) &&
     (( pc3_tf_publishers == 2 )) &&
     (( pc3_tf_static_publishers == 2 )); then
    pc3_inputs_ready="PASS"
  fi
}

wait_for_pc3_inputs()
{
  local attempt
  for attempt in {1..10}; do
    pc3_input_snapshot
    [[ "${pc3_inputs_ready}" == "PASS" ]] && return 0
    sleep 0.5
  done
  return 1
}

snapshot_pc2_output_baseline()
{
  local count
  local topic
  pc2_output_baseline_counts=()
  pc2_output_baseline_valid="PASS"
  for topic in "${PC2_OWNED_OUTPUT_TOPICS[@]}"; do
    count="$(publisher_count "${topic}")"
    pc2_output_baseline_counts+=("${count}")
    (( count >= 0 )) || pc2_output_baseline_valid="FAIL"
  done
}

pc2_output_excess_snapshot()
{
  local baseline
  local current
  local index=0
  local topic
  local -a details=()
  pc2_remaining_publisher_count=0
  pc2_output_query_valid="PASS"

  for topic in "${PC2_OWNED_OUTPUT_TOPICS[@]}"; do
    baseline="${pc2_output_baseline_counts[index]:-0}"
    current="$(publisher_count "${topic}")"
    if (( baseline < 0 || current < 0 )); then
      pc2_output_query_valid="FAIL"
      details+=("${topic}:query_error")
      index="$((index + 1))"
      continue
    fi
    if (( current > baseline )); then
      pc2_remaining_publisher_count="$((pc2_remaining_publisher_count + current - baseline))"
      details+=("${topic}:${current}>${baseline}")
    fi
    index="$((index + 1))"
  done

  pc2_remaining_publisher_detail="none"
  if (( ${#details[@]} > 0 )); then
    pc2_remaining_publisher_detail="$(IFS=,; echo "${details[*]}")"
  fi
}

snapshot_safety_publisher_baseline()
{
  local count
  local topic
  safety_baseline_counts=()
  safety_baseline_valid="PASS"
  for topic in "${PC2_SAFETY_TOPICS[@]}"; do
    count="$(publisher_count "${topic}")"
    safety_baseline_counts+=("${count}")
    (( count >= 0 )) || safety_baseline_valid="FAIL"
  done
}

safety_publisher_excess_snapshot()
{
  local baseline
  local current
  local index=0
  local topic
  local -a details=()
  safety_publisher_excess=0
  safety_publisher_query_valid="PASS"

  for topic in "${PC2_SAFETY_TOPICS[@]}"; do
    baseline="${safety_baseline_counts[index]:-0}"
    current="$(publisher_count "${topic}")"
    if (( baseline < 0 || current < 0 )); then
      safety_publisher_query_valid="FAIL"
      details+=("${topic}:query_error")
      index="$((index + 1))"
      continue
    fi
    if (( current > baseline )); then
      safety_publisher_excess="$((safety_publisher_excess + current - baseline))"
      details+=("${topic}:${current}>${baseline}")
    fi
    index="$((index + 1))"
  done

  safety_publisher_detail="none"
  if (( ${#details[@]} > 0 )); then
    safety_publisher_detail="$(IFS=,; echo "${details[*]}")"
  fi
}

snapshot_forbidden_node_baseline()
{
  local nodes
  nodes="$(direct_nodes)"
  forbidden_node_baseline_query_valid="PASS"
  if grep -qx '__GRAPH_QUERY_ERROR__' <<< "${nodes}"; then
    forbidden_node_baseline_query_valid="FAIL"
    forbidden_node_baseline=""
    return 1
  fi
  forbidden_node_baseline="$(forbidden_control_vehicle_nodes <<< "${nodes}" | sort -u)"
}

forbidden_node_excess_snapshot()
{
  local nodes="$1"
  forbidden_node_query_valid="PASS"
  new_forbidden_node_detail="none"
  new_forbidden_nodes=0

  if grep -qx '__GRAPH_QUERY_ERROR__' <<< "${nodes}"; then
    forbidden_node_query_valid="FAIL"
    new_forbidden_node_detail="graph_query_error"
    return 1
  fi

  new_forbidden_node_detail="$({
    comm -13 \
      <(printf '%s\n' "${forbidden_node_baseline}" | sed '/^$/d' | sort -u) \
      <(forbidden_control_vehicle_nodes <<< "${nodes}" | sort -u)
  } | paste -sd, -)"
  if [[ -n "${new_forbidden_node_detail}" ]]; then
    new_forbidden_nodes="$(tr ',' '\n' <<< "${new_forbidden_node_detail}" | grep -c '^/' || true)"
  else
    new_forbidden_node_detail="none"
  fi
}

# HH_260810 - Inspected only the launched process group, excluding remote PC3 graph subscriptions and nodes.
pc2_control_vehicle_process_count()
{
  local target_pgid="$1"
  ps -eo pgid=,args= |
    awk -v pgid="${target_pgid}" '
      $1 == pgid {
        $1 = ""
        if ($0 ~ /__ns:=\/(control|vehicle)(\/|[[:space:]]|$)/ ||
            $0 ~ /(^|[\/:[:space:]])(vehicle_cmd_gate|trajectory_follower|control_validator|operation_mode_transition_manager|socket_can|pacmod|ssc_interface|vehicle_interface)([^[:space:]]*)/) {
          count++
        }
      }
      END { print count + 0 }
    '
}

wait_for_launch_pid()
{
  local wrapper_pid="$1"
  local attempt
  for attempt in {1..150}; do
    if [[ -s "${PC2_PID_FILE}" ]]; then
      tr -cd '0-9' < "${PC2_PID_FILE}"
      return 0
    fi
    kill -0 "${wrapper_pid}" 2>/dev/null || return 1
    sleep 0.1
  done
  return 1
}

# HH_260811 - Always hand an interrupted validation run back to the guarded launcher
# so its process-group and non-persistent camera-route cleanup can finish.
stop_active_wrapper()
{
  local attempt

  [[ "${active_wrapper_pid}" =~ ^[0-9]+$ ]] || return 0
  if kill -0 "${active_wrapper_pid}" 2>/dev/null; then
    kill -TERM "${active_wrapper_pid}" 2>/dev/null || true
    for attempt in {1..250}; do
      kill -0 "${active_wrapper_pid}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  if kill -0 "${active_wrapper_pid}" 2>/dev/null; then
    kill -KILL "${active_wrapper_pid}" 2>/dev/null || true
  fi
  wait "${active_wrapper_pid}" 2>/dev/null || true
  active_wrapper_pid=""
}

validator_cleanup()
{
  local status=$?

  trap - EXIT INT TERM
  stop_active_wrapper
  exit "${status}"
}

trap 'exit 130' INT
trap 'exit 143' TERM
trap validator_cleanup EXIT

for ((cycle = 1; cycle <= CYCLE_COUNT; cycle++)); do
  echo "cycle=${cycle} phase=pc3_preflight"
  if (( REQUIRE_PC3_INPUTS == 1 )); then
    if ! wait_for_pc3_inputs; then
      echo "cycle=${cycle} readiness=FAIL reason=pc3_inputs_unavailable pc3_before_sync_publishers=${pc3_before_sync_publishers} pc3_raw_ex_publishers=${pc3_raw_ex_publishers} pc3_vector_map_publishers=${pc3_vector_map_publishers} pc3_pointcloud_map_publishers=${pc3_pointcloud_map_publishers} pc3_tf_publishers=${pc3_tf_publishers} pc3_tf_static_publishers=${pc3_tf_static_publishers}"
      overall_status=1
      continue
    fi
  else
    pc3_input_snapshot
  fi
  echo "cycle=${cycle} pc3_inputs=${pc3_inputs_ready} required=${REQUIRE_PC3_INPUTS} before_sync=${pc3_before_sync_publishers} raw_ex=${pc3_raw_ex_publishers} vector_map=${pc3_vector_map_publishers} pointcloud_map=${pc3_pointcloud_map_publishers} tf=${pc3_tf_publishers} tf_static=${pc3_tf_static_publishers}"

  # HH_260810 - Refused to consume a stale guard PID instead of assuming it belongs to this cycle.
  if [[ -e "${PC2_PID_FILE}" || -e "${PC2_LOCK_FILE}" ]]; then
    echo "cycle=${cycle} readiness=FAIL reason=preexisting_guard_file pid_file=$([[ -e "${PC2_PID_FILE}" ]] && echo present || echo absent) lock_file=$([[ -e "${PC2_LOCK_FILE}" ]] && echo present || echo absent)"
    overall_status=1
    continue
  fi

  # HH_260810 - Snapshotted remote-domain publisher baselines before attributing any new endpoint to PC2.
  snapshot_pc2_output_baseline
  snapshot_safety_publisher_baseline
  snapshot_forbidden_node_baseline || true
  if [[ "${pc2_output_baseline_valid}" != "PASS" ||
        "${safety_baseline_valid}" != "PASS" ||
        "${forbidden_node_baseline_query_valid}" != "PASS" ]]; then
    echo "cycle=${cycle} readiness=FAIL reason=baseline_graph_query pc2_output_baseline=${pc2_output_baseline_valid} safety_baseline=${safety_baseline_valid} forbidden_node_baseline=${forbidden_node_baseline_query_valid}"
    overall_status=1
    continue
  fi

  echo "cycle=${cycle} phase=start"
  cycle_log="${PC2_CYCLE_LOG_DIR}/cycle_${cycle}.log"
  "${PC2_LAUNCHER}" >"${cycle_log}" 2>&1 &
  wrapper_pid=$!
  active_wrapper_pid="${wrapper_pid}"

  if ! launch_pid="$(wait_for_launch_pid "${wrapper_pid}")" || [[ ! "${launch_pid}" =~ ^[0-9]+$ ]]; then
    echo "cycle=${cycle} readiness=FAIL reason=launch_pid_unavailable"
    stop_active_wrapper
    overall_status=1
    continue
  fi

  readiness="TIMEOUT"
  nodes=""
  official_publishers=0
  raw_publishers=0
  native_compressed_publishers=0
  native_info_publishers=0
  normalized_info_publishers=0
  yolox_publishers=0
  yolox_subscriptions=0
  fusion_components=0
  traffic_components=0
  for attempt in {1..90}; do
    nodes="$(direct_nodes)"
    official_publishers="$(publisher_count /perception/object_recognition/objects)"
    raw_publishers="$(publisher_count /sensing/camera/camera1/traffic_light/image_raw)"
    native_compressed_publishers="$(publisher_count /lucid_vision/camera/image_compressed)"
    native_info_publishers="$(publisher_count /lucid_vision/camera/camera_info)"
    normalized_info_publishers="$(publisher_count /sensing/camera/camera1/traffic_light/camera_info)"
    yolox_publishers="$(publisher_count /perception/object_recognition/detection/rois0)"
    yolox_subscriptions="$(subscription_count /perception/object_recognition/detection/rois0)"
    fusion_components="$(grep -Ec '^/perception/object_recognition/detection/(clustering/roi_cluster/roi_pointcloud_fusion|clustering/camera_lidar_fusion/roi_cluster_fusion|roi_detected_object_fusion)$' <<< "${nodes}" || true)"
    traffic_components="$(grep -Ec '^/perception/traffic_light_recognition/camera1/(detection/traffic_light_fine_detector|classification/car_traffic_light_classifier|classification/pedestrian_traffic_light_classifier|traffic_light_roi_visualizer)$' <<< "${nodes}" || true)"
    if grep -qx '/pc2_perception_pointcloud_container' <<< "${nodes}" &&
       (( official_publishers == 1 )) && (( raw_publishers == 1 )) &&
       (( native_compressed_publishers == 1 )) && (( native_info_publishers == 1 )) &&
       (( normalized_info_publishers == 1 )) &&
       (( yolox_publishers == 1 )) && (( yolox_subscriptions >= 3 )) &&
       (( fusion_components == 3 )) &&
       (( traffic_components == 4 )); then
      if (( REQUIRE_PC3_INPUTS == 0 )); then
        readiness="PASS"
        break
      fi
      pc3_input_snapshot
      if [[ "${pc3_inputs_ready}" == "PASS" ]]; then
        readiness="PASS"
        break
      fi
      readiness="PC3_INPUTS_WAIT"
    fi
    kill -0 "${wrapper_pid}" 2>/dev/null || {
      readiness="LAUNCH_EXITED"
      break
    }
    sleep 0.5
  done

  # HH_260810 - Refreshed the remote dependency snapshot at readiness instead of inferring availability from topic names.
  pc3_input_snapshot
  if [[ "${readiness}" == "PASS" ]] &&
     (( REQUIRE_PC3_INPUTS == 1 )) && [[ "${pc3_inputs_ready}" != "PASS" ]]; then
    readiness="PC3_INPUTS_LOST"
  fi

  pc2_nodes="$(pc2_owned_nodes <<< "${nodes}")"
  node_count="$(grep -c '^/' <<< "${pc2_nodes}" || true)"
  domain_node_count="$(grep -c '^/' <<< "${nodes}" || true)"
  duplicate_nodes="$(sort <<< "${pc2_nodes}" | uniq -d | paste -sd, -)"
  native_camera_publishers="$(publisher_count /lucid_vision/camera/image)"
  camera_info_publishers="$(publisher_count /sensing/camera/camera1/traffic_light/camera_info)"
  raw_subscriptions="$(subscription_count /sensing/camera/camera1/traffic_light/image_raw)"
  control_vehicle_processes="$(pc2_control_vehicle_process_count "${launch_pid}")"
  safety_publisher_excess_snapshot
  forbidden_node_excess_snapshot "${nodes}" || true
  control_command_publishers="$(publisher_count /control/command/control_cmd)"
  control_vehicle_safety="FAIL"
  if (( control_vehicle_processes == 0 )) && (( safety_publisher_excess == 0 )) &&
     (( new_forbidden_nodes == 0 )) &&
     [[ "${safety_publisher_query_valid}" == "PASS" &&
        "${forbidden_node_query_valid}" == "PASS" ]]; then
    control_vehicle_safety="PASS"
  fi
  if [[ "${readiness}" == "PASS" && "${control_vehicle_safety}" != "PASS" ]]; then
    readiness="SAFETY_FAIL"
  fi
  process_count="$(ps -eo pgid= | awk -v pgid="${launch_pid}" '$1 == pgid {count++} END {print count + 0}')"
  gpu_state="$(nvidia-smi --query-gpu=temperature.gpu,memory.used,utilization.gpu,power.draw --format=csv,noheader,nounits 2>/dev/null | tr -d ' ' || true)"
  camera_contract="FAIL"
  camera_contract_detail="not_run"
  if camera_contract_detail="$(timeout 20s "${PC2_CAMERA_CONTRACT_PROBE}" 2>&1)"; then
    camera_contract="PASS"
  fi
  yolox_contract="FAIL"
  yolox_contract_detail="not_run"
  if yolox_contract_detail="$(timeout 15s "${PC2_YOLOX_CONTRACT_PROBE}" 2>&1)"; then
    yolox_contract="PASS"
  fi
  # HH_260811 - Required the launcher's owned /32 to select the camera's physically proven interface.
  camera_lookup="$(ip -o -4 route get "${PC2_CAMERA_IP}" 2>/dev/null || true)"
  camera_path="FAIL"
  if [[ " ${camera_lookup} " == *" dev ${PC2_CAMERA_IFACE} "* &&
        " ${camera_lookup} " == *" src ${PC2_CAMERA_HOST_IP} "* ]]; then
    camera_path="PASS"
  fi
  exact_camera_route="$(ip -o -4 route show table main exact "${PC2_CAMERA_ROUTE_CIDR}" 2>/dev/null || true)"
  topic_tools_overlay="$(grep -c 'Using guarded topic_tools 1.1.2 overlay' "${cycle_log}" || true)"

  echo "cycle=${cycle} readiness=${readiness} launch_pid=${launch_pid} processes=${process_count} pc2_nodes=${node_count} domain_nodes=${domain_node_count} duplicate_pc2_nodes=${duplicate_nodes:-none} official_publishers=${official_publishers} raw_publishers=${raw_publishers} raw_subscriptions=${raw_subscriptions} yolox_publishers=${yolox_publishers} yolox_subscriptions=${yolox_subscriptions} fusion_components=${fusion_components}/3 traffic_components=${traffic_components}/4 native_camera_publishers=${native_camera_publishers} native_compressed_publishers=${native_compressed_publishers} native_info_publishers=${native_info_publishers} camera_info_publishers=${camera_info_publishers} camera_contract=${camera_contract} yolox_contract=${yolox_contract} camera_path=${camera_path} exact_camera_route=${exact_camera_route:-none} topic_tools_overlay=${topic_tools_overlay} pc3_inputs=${pc3_inputs_ready} pc3_publishers=${pc3_before_sync_publishers}/${pc3_raw_ex_publishers}/${pc3_vector_map_publishers}/${pc3_pointcloud_map_publishers}/${pc3_tf_publishers}/${pc3_tf_static_publishers} control_vehicle_safety=${control_vehicle_safety} control_vehicle_processes=${control_vehicle_processes} new_safety_publishers=${safety_publisher_excess} safety_publisher_query=${safety_publisher_query_valid} safety_publisher_detail=${safety_publisher_detail} new_forbidden_nodes=${new_forbidden_nodes} forbidden_node_query=${forbidden_node_query_valid} forbidden_node_detail=${new_forbidden_node_detail} control_cmd_publishers=${control_command_publishers} gpu=${gpu_state:-unavailable}"
  echo "cycle=${cycle} camera_lookup=${camera_lookup// /_}"
  echo "cycle=${cycle} camera_contract_detail=${camera_contract_detail}"
  echo "cycle=${cycle} yolox_contract_detail=${yolox_contract_detail}"

  # HH_260810 - Asked only the guarded wrapper to stop; it delivers SIGINT to its exact child process group.
  stop_started_ns="$(date +%s%N)"
  kill -TERM "${wrapper_pid}" 2>/dev/null || true
  wait "${wrapper_pid}" 2>/dev/null || true
  active_wrapper_pid=""

  group_gone="FAIL"
  for attempt in {1..150}; do
    if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
      group_gone="PASS"
      break
    fi
    sleep 0.1
  done
  stop_finished_ns="$(date +%s%N)"
  stop_duration_ms="$(( (stop_finished_ns - stop_started_ns) / 1000000 ))"

  files_removed="FAIL"
  [[ ! -e "${PC2_PID_FILE}" && ! -e "${PC2_LOCK_FILE}" ]] && files_removed="PASS"

  graph_clear="FAIL"
  remaining_domain_nodes=""
  remaining_pc2_nodes=""
  pc2_remaining_publisher_count=0
  pc2_remaining_publisher_detail="none"
  # HH_260810 - Required only PC2-owned nodes and publisher deltas to disappear; PC3 must remain visible.
  for attempt in {1..30}; do
    remaining_domain_nodes="$(direct_nodes)"
    remaining_pc2_nodes="$(pc2_owned_nodes <<< "${remaining_domain_nodes}")"
    pc2_output_excess_snapshot
    if ! grep -qx '__GRAPH_QUERY_ERROR__' <<< "${remaining_domain_nodes}" &&
       [[ -z "${remaining_pc2_nodes}" ]] &&
       (( pc2_remaining_publisher_count == 0 )) &&
       [[ "${pc2_output_query_valid}" == "PASS" ]]; then
      graph_clear="PASS"
      break
    fi
    sleep 0.5
  done

  # HH_260810 - Reported that the active PC3 dependencies survived the exact PC2 process-group shutdown.
  pc3_input_snapshot
  pc3_inputs_post_stop="${pc3_inputs_ready}"

  post_gpu_memory="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | tr -d ' ' || true)"
  route_clear="FAIL"
  [[ -z "$(ip -o -4 route show table main exact "${PC2_CAMERA_ROUTE_CIDR}" 2>/dev/null)" ]] && route_clear="PASS"
  trt_teardown_errors="$(grep -c 'Destroying a runtime before destroying deserialized engines' "${cycle_log}" || true)"
  process_died_events="$(grep -c 'process has died' "${cycle_log}" || true)"
  camera_memory_errors="$(grep -Ec 'corrupted double-linked list|segmentation fault|\[component_container-1\].*exit code -11|\[component_container-1\].*exit code -6' "${cycle_log}" || true)"
  relay_shutdown_errors="$(grep -Ec 'get information by topic for publishers.*context is invalid|\[relay-[0-9]+\].*exit code -6' "${cycle_log}" || true)"
  cycle_result="PASS"
  if [[ "${readiness}" != "PASS" ||
        "${camera_contract}" != "PASS" ||
        "${yolox_contract}" != "PASS" ||
        "${camera_path}" != "PASS" ||
        "${control_vehicle_safety}" != "PASS" ||
        "${group_gone}" != "PASS" ||
        "${files_removed}" != "PASS" ||
        "${graph_clear}" != "PASS" ||
        "${route_clear}" != "PASS" ]] ||
     (( topic_tools_overlay != 1 )) ||
     (( trt_teardown_errors != 0 )) ||
     (( process_died_events != 0 )) ||
     (( camera_memory_errors != 0 )) ||
     (( relay_shutdown_errors != 0 )); then
    cycle_result="FAIL"
  fi
  if (( REQUIRE_PC3_INPUTS == 1 )) && [[ "${pc3_inputs_post_stop}" != "PASS" ]]; then
    cycle_result="FAIL"
  fi
  [[ "${cycle_result}" == "PASS" ]] || overall_status=1

  echo "cycle=${cycle} result=${cycle_result} shutdown_group=${group_gone} cleanup_files=${files_removed} graph_clear=${graph_clear} route_clear=${route_clear} remaining_pc2_nodes=$(grep -c '^/' <<< "${remaining_pc2_nodes}" || true) remaining_pc2_publishers=${pc2_remaining_publisher_count} remaining_pc2_publisher_query=${pc2_output_query_valid} remaining_pc2_publisher_detail=${pc2_remaining_publisher_detail} remaining_domain_nodes=$(grep -c '^/' <<< "${remaining_domain_nodes}" || true) pc3_inputs_post_stop=${pc3_inputs_post_stop} stop_duration_ms=${stop_duration_ms} post_gpu_memory_mib=${post_gpu_memory:-unavailable} trt_teardown_errors=${trt_teardown_errors} process_died_events=${process_died_events} camera_memory_errors=${camera_memory_errors} relay_shutdown_errors=${relay_shutdown_errors} log=${cycle_log}"
done

# HH_260810 - Returned a failing status when any required cycle, safety, or cleanup invariant failed.
exit "${overall_status}"
