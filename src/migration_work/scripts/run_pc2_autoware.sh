#!/usr/bin/env bash

# HH_260810 - Added a guarded PC2 launcher while preserving the run_autoware alias and active launch filename.
readonly PC2_LOCK_FILE="/tmp/autoware_pc2_run.lock"
readonly PC2_PID_FILE="/tmp/autoware_pc2_run.pid"
readonly PC2_LAUNCH_PATTERN='[r]os2 launch autoware_launch autoware.launch.xml'
readonly PC2_CAMERA_LAUNCH_PATTERN='[r]os2 launch lucid_vision_driver test_node_container.launch.py'
# HH_260811 - Matched the outdoor vehicle's active ROS2 profile and the physically proven serial-214 camera interface.
readonly PC2_DDS_IFACE="enp0s31f6"
readonly PC2_DDS_CONNECTION="ROS2"
readonly PC2_DDS_HOST_CIDR="192.168.9.110/24"
readonly PC2_PC3_IP="192.168.9.7"
readonly PC2_CAMERA_IFACE="enp1s0f3"
readonly PC2_CAMERA_CONNECTION="Lucid Camera Loop Top"
readonly PC2_CAMERA_HOST_CIDR="169.254.0.1/24"
readonly PC2_CAMERA_HOST_IP="169.254.0.1"
readonly PC2_CAMERA_IP="169.254.0.11"
readonly PC2_CAMERA_ROUTE_CIDR="${PC2_CAMERA_IP}/32"
# HH_260811 - Kept serial 214 pinned to its proven NIC when two link-local camera networks are active.
readonly PC2_CAMERA_ROUTE_METRIC="42762"
readonly PC2_CAMERA_ROUTE_SPEC="${PC2_CAMERA_ROUTE_CIDR} 0.0.0.0 ${PC2_CAMERA_ROUTE_METRIC}"
# HH_260810 - Used audited topic_tools 1.1.2 plus the in-flight publish shutdown guard without modifying /opt/ros.
readonly PC2_TOPIC_TOOLS_PREFIX="/home/a/autoware/src/migration_work/vendor-build/topic_tools-1.1.2-HH_260810/install"
readonly PC2_TOPIC_TOOLS_RELAY_LIBRARY="${PC2_TOPIC_TOOLS_PREFIX}/lib/librelay_node.so"
readonly PC2_TOPIC_TOOLS_RELAY_LIBRARY_SHA256="45ad17f6e202a74cba025ad94f219f7402ef532551cd1b8162923267a16cb9e3"
# HH_260810 - Pinned CycloneDDS to the only local interface proven to discover PC3 at 192.168.9.7.
readonly PC2_CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="enp0s31f6"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'

# HH_260810 - Sourced the same ROS, Autoware, and camera overlays used by the original interactive alias.
source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash
source /home/a/ros2_ws/install/setup.bash

# HH_260810 - Enabled unset-variable checking only after the ROS setup scripts finished reading optional variables.
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# HH_260810 - Refused a cross-PC start unless it uses the Domain 10 CycloneDDS settings proven against PC3.
if [[ "${ROS_DOMAIN_ID}" != "10" || "${ROS_LOCALHOST_ONLY}" != "0" ||
      "${RMW_IMPLEMENTATION}" != "rmw_cyclonedds_cpp" ]]; then
  echo "PC2 run_autoware requires Domain 10, ROS_LOCALHOST_ONLY=0, and rmw_cyclonedds_cpp." >&2
  exit 78
fi

# HH_260810 - Kept the DDS interface selection process-local and rejected a conflicting inherited setting.
if [[ -n "${CYCLONEDDS_URI:-}" && "${CYCLONEDDS_URI}" != "${PC2_CYCLONEDDS_URI}" ]]; then
  echo "A conflicting CYCLONEDDS_URI is already set; refusing an ambiguous PC2 start." >&2
  exit 77
fi
export CYCLONEDDS_URI="${PC2_CYCLONEDDS_URI}"

# HH_260810 - Verified the vendored relay binary and forced package and library resolution to it.
configure_topic_tools_overlay() {
  local actual_sha256=""
  local resolved_relay_library=""
  local resolved_prefix=""

  if [[ ! -x "${PC2_TOPIC_TOOLS_PREFIX}/lib/topic_tools/relay" ||
        ! -r "${PC2_TOPIC_TOOLS_RELAY_LIBRARY}" ]]; then
    echo "The guarded topic_tools 1.1.2 overlay is incomplete." >&2
    return 1
  fi

  actual_sha256="$(sha256sum "${PC2_TOPIC_TOOLS_RELAY_LIBRARY}" | awk '{print $1}')" || return 1
  if [[ "${actual_sha256}" != "${PC2_TOPIC_TOOLS_RELAY_LIBRARY_SHA256}" ]]; then
    echo "The guarded topic_tools relay library failed its SHA-256 check." >&2
    return 1
  fi

  export AMENT_PREFIX_PATH="${PC2_TOPIC_TOOLS_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${PC2_TOPIC_TOOLS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  resolved_prefix="$(ros2 pkg prefix topic_tools 2>/dev/null)" || return 1
  if [[ "${resolved_prefix}" != "${PC2_TOPIC_TOOLS_PREFIX}" ]]; then
    echo "topic_tools resolved to ${resolved_prefix:-none}, not the guarded 1.1.2 overlay." >&2
    return 1
  fi

  # HH_260810 - Proved the selected relay executable will dynamically load the guarded library.
  resolved_relay_library="$(
    ldd "${PC2_TOPIC_TOOLS_PREFIX}/lib/topic_tools/relay" 2>/dev/null |
      awk '/librelay_node\.so/ {print $3; exit}'
  )" || return 1
  if [[ "${resolved_relay_library}" != "${PC2_TOPIC_TOOLS_RELAY_LIBRARY}" ]]; then
    echo "The relay resolves ${resolved_relay_library:-no library}, not the guarded library." >&2
    return 1
  fi

  echo "Using guarded topic_tools 1.1.2 overlay at ${PC2_TOPIC_TOOLS_PREFIX}." >&2
}

# HH_260810 - Refused launch unless the user-space relay overlay is intact and selected.
if ! configure_topic_tools_overlay; then
  exit 76
fi

cleanup_files() {
  rm -f -- "${PC2_PID_FILE}" "${PC2_LOCK_FILE}"
}

exec 9>"${PC2_LOCK_FILE}"
if ! flock -n 9; then
  echo "PC2 Autoware is already starting or running; refusing a duplicate run." >&2
  exit 73
fi

# HH_260810 - Removed this helper's guard files on every post-lock exit, including refused preflight checks.
trap cleanup_files EXIT

# HH_260810 - Refused a second copy of the same active launch even when it was started outside this helper.
if pgrep -af "${PC2_LAUNCH_PATTERN}" >/dev/null 2>&1; then
  echo "The active PC2 Autoware launch is already running; refusing a duplicate run." >&2
  exit 73
fi

# HH_260810 - Refused the known standalone Lucid launch even when camera construction failed before creating publishers.
if pgrep -af "${PC2_CAMERA_LAUNCH_PATTERN}" >/dev/null 2>&1; then
  echo "The standalone PC2 Lucid launch is already running; refusing a duplicate camera start." >&2
  exit 74
fi

publisher_count() {
  local topic="$1"
  local count
  # HH_260810 - Used direct DDS discovery so preflight checks do not leave a ros2cli daemon behind.
  count="$(timeout 5s ros2 topic info "${topic}" --no-daemon 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')"
  echo "${count:-0}"
}

# HH_260810 - Guarded the PC2-owned camera and official object topics against pre-existing publishers.
for topic in \
  /lucid_vision/camera/image \
  /lucid_vision/camera/image_compressed \
  /lucid_vision/camera/camera_info \
  /sensing/camera/camera1/traffic_light/camera_info \
  /sensing/camera/camera1/traffic_light/image_raw/compressed \
  /sensing/camera/camera1/traffic_light/image_raw \
  /perception/object_recognition/detection/rois0 \
  /perception/traffic_light_recognition/traffic_signals \
  /perception/object_recognition/objects
do
  if (( "$(publisher_count "${topic}")" > 0 )); then
    echo "Topic ${topic} already has a publisher; refusing a conflicting PC2 start." >&2
    exit 74
  fi
done

launch_pid=""
camera_connection_uuid=""
camera_saved_routes_before=""
camera_route_owned=0

process_group_exists() {
  [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null
}

stop_launch_group() {
  local attempt

  process_group_exists || return 0
  echo "Stopping PC2 Autoware process group ${launch_pid} with SIGINT..." >&2
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  for attempt in {1..100}; do
    process_group_exists || return 0
    sleep 0.1
  done

  echo "PC2 Autoware did not stop after SIGINT; sending SIGTERM to process group ${launch_pid}." >&2
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
  for attempt in {1..50}; do
    process_group_exists || return 0
    sleep 0.1
  done

  echo "PC2 Autoware did not stop after SIGTERM; sending final SIGKILL to process group ${launch_pid}." >&2
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
}

# HH_260810 - Accepted the audited link-scope camera address while still requiring an exact CIDR match.
interface_address_is_valid() {
  local interface="$1"
  local expected_cidr="$2"

  ip -o -4 address show dev "${interface}" 2>/dev/null |
    awk -v expected="${expected_cidr}" \
      '$4 == expected { found=1 } END { exit(found ? 0 : 1) }'
}

camera_route_get_is_valid() {
  local route_line

  route_line="$(ip -o -4 route get "${PC2_CAMERA_IP}" 2>/dev/null)" || return 1
  [[ " ${route_line} " == *" dev ${PC2_CAMERA_IFACE} "* ]] &&
    [[ " ${route_line} " == *" src ${PC2_CAMERA_HOST_IP} "* ]]
}

# HH_260811 - Removed only the exact runtime route created by this helper and proved that no profile state changed.
remove_owned_camera_route() {
  local current_saved_routes=""

  (( camera_route_owned == 1 )) || return 0
  if ! nmcli device modify "${PC2_CAMERA_IFACE}" -ipv4.routes "${PC2_CAMERA_ROUTE_SPEC}" >/dev/null; then
    echo "Failed to remove the owned runtime camera route ${PC2_CAMERA_ROUTE_SPEC}." >&2
    return 1
  fi
  camera_route_owned=0

  if [[ -n "$(ip -o -4 route show table main exact "${PC2_CAMERA_ROUTE_CIDR}" 2>/dev/null)" ]]; then
    echo "The owned runtime camera route remains after cleanup." >&2
    return 1
  fi
  current_saved_routes="$(
    nmcli -g ipv4.routes connection show uuid "${camera_connection_uuid}" 2>/dev/null
  )" || return 1
  if [[ "${current_saved_routes}" != "${camera_saved_routes_before}" ]]; then
    echo "The camera profile's saved routes changed during runtime-route cleanup." >&2
    return 1
  fi

  echo "Removed the owned runtime Lucid route; saved NetworkManager routes are unchanged." >&2
}

# HH_260811 - Added the smallest reversible route proven to open and stream serial 214 on its physical NIC.
install_runtime_camera_route() {
  local current_saved_routes=""
  local route_line=""

  camera_route_owned=1
  if ! nmcli device modify "${PC2_CAMERA_IFACE}" +ipv4.routes "${PC2_CAMERA_ROUTE_SPEC}" >/dev/null; then
    echo "Failed to add runtime camera route ${PC2_CAMERA_ROUTE_SPEC}." >&2
    return 1
  fi
  if ! camera_route_get_is_valid; then
    route_line="$(ip -o -4 route get "${PC2_CAMERA_IP}" 2>&1 || true)"
    echo "Runtime camera path validation failed: ${route_line}" >&2
    return 1
  fi
  current_saved_routes="$(
    nmcli -g ipv4.routes connection show uuid "${camera_connection_uuid}" 2>/dev/null
  )" || return 1
  if [[ "${current_saved_routes}" != "${camera_saved_routes_before}" ]]; then
    echo "Adding the runtime route unexpectedly changed the saved camera profile." >&2
    return 1
  fi
  if ! ping -c 1 -W 1 -I "${PC2_CAMERA_IFACE}" "${PC2_CAMERA_IP}" >/dev/null 2>&1; then
    echo "The Lucid camera did not answer on the proven runtime path." >&2
    return 1
  fi

  echo "Installed owned runtime Lucid route ${PC2_CAMERA_ROUTE_CIDR} on ${PC2_CAMERA_IFACE}." >&2
}

pc3_route_get_is_valid() {
  local route_line

  route_line="$(ip -o -4 route get "${PC2_PC3_IP}" 2>/dev/null)" || return 1
  [[ " ${route_line} " == *" dev ${PC2_DDS_IFACE} "* ]] &&
    [[ " ${route_line} " == *" src ${PC2_DDS_HOST_CIDR%/*} "* ]]
}

active_connection_on_interface() {
  nmcli -g GENERAL.CONNECTION device show "$1" 2>/dev/null
}

# HH_260810 - Validated the direct camera link and PC3-reachable DDS link without changing profiles or routes.
validate_network_layout() {
  local active_camera_connection=""
  local active_dds_connection=""
  local camera_saved_routes=""
  local dds_connection_uuid=""
  local dds_saved_routes=""
  local exact_camera_route=""
  local route_line=""

  active_camera_connection="$(active_connection_on_interface "${PC2_CAMERA_IFACE}")" || {
    echo "Cannot inspect NetworkManager state for ${PC2_CAMERA_IFACE}." >&2
    return 1
  }
  active_dds_connection="$(active_connection_on_interface "${PC2_DDS_IFACE}")" || {
    echo "Cannot inspect NetworkManager state for ${PC2_DDS_IFACE}." >&2
    return 1
  }

  if [[ "${active_camera_connection}" != "${PC2_CAMERA_CONNECTION}" ]]; then
    echo "Expected ${PC2_CAMERA_CONNECTION} on ${PC2_CAMERA_IFACE}; found ${active_camera_connection:-none}." >&2
    return 1
  fi
  if [[ "${active_dds_connection}" != "${PC2_DDS_CONNECTION}" ]]; then
    echo "Expected ${PC2_DDS_CONNECTION} on ${PC2_DDS_IFACE}; found ${active_dds_connection:-none}." >&2
    return 1
  fi
  if ! interface_address_is_valid "${PC2_CAMERA_IFACE}" "${PC2_CAMERA_HOST_CIDR}"; then
    echo "${PC2_CAMERA_HOST_CIDR} is not active on ${PC2_CAMERA_IFACE}." >&2
    return 1
  fi
  if ! interface_address_is_valid "${PC2_DDS_IFACE}" "${PC2_DDS_HOST_CIDR}"; then
    echo "${PC2_DDS_HOST_CIDR} is not active on ${PC2_DDS_IFACE}." >&2
    return 1
  fi

  camera_connection_uuid="$(
    nmcli -g GENERAL.CON-UUID device show "${PC2_CAMERA_IFACE}" 2>/dev/null
  )" || return 1
  dds_connection_uuid="$(
    nmcli -g GENERAL.CON-UUID device show "${PC2_DDS_IFACE}" 2>/dev/null
  )" || return 1
  if [[ -z "${camera_connection_uuid}" || "${camera_connection_uuid}" == "--" ]]; then
    echo "No active NetworkManager UUID found for ${PC2_CAMERA_IFACE}." >&2
    return 1
  fi
  if [[ -z "${dds_connection_uuid}" || "${dds_connection_uuid}" == "--" ]]; then
    echo "No active NetworkManager UUID found for ${PC2_DDS_IFACE}." >&2
    return 1
  fi

  camera_saved_routes="$(
    nmcli -g ipv4.routes connection show uuid "${camera_connection_uuid}" 2>/dev/null
  )" || {
    echo "Cannot snapshot saved routes for ${camera_connection_uuid}." >&2
    return 1
  }
  dds_saved_routes="$(
    nmcli -g ipv4.routes connection show uuid "${dds_connection_uuid}" 2>/dev/null
  )" || {
    echo "Cannot snapshot saved routes for ${dds_connection_uuid}." >&2
    return 1
  }
  if [[ -n "${camera_saved_routes}" || -n "${dds_saved_routes}" ]]; then
    echo "Unexpected saved routes exist on the audited camera or DDS profile." >&2
    return 1
  fi
  camera_saved_routes_before="${camera_saved_routes}"

  exact_camera_route="$(
    ip -o -4 route show table main exact "${PC2_CAMERA_ROUTE_CIDR}" 2>/dev/null
  )" || return 1
  if [[ -n "${exact_camera_route}" ]]; then
    echo "Unexpected ${PC2_CAMERA_ROUTE_CIDR} exists; refusing to take ownership of another route." >&2
    return 1
  fi
  if ! pc3_route_get_is_valid; then
    route_line="$(ip -o -4 route get "${PC2_PC3_IP}" 2>&1 || true)"
    echo "PC3 DDS path validation failed: ${route_line}" >&2
    return 1
  fi

  echo "Validated physical Lucid interface ${PC2_CAMERA_HOST_CIDR} on ${PC2_CAMERA_IFACE}." >&2
  echo "Validated PC3 DDS path ${PC2_DDS_HOST_CIDR} -> ${PC2_PC3_IP} on ${PC2_DDS_IFACE}." >&2
}

cleanup_all() {
  local status=$?

  trap - EXIT INT TERM
  stop_launch_group
  if ! remove_owned_camera_route; then
    status=75
  fi
  cleanup_files
  exit "${status}"
}

on_signal() {
  local status="$1"
  trap - INT TERM
  stop_launch_group
  exit "${status}"
}

trap 'on_signal 130' INT
trap 'on_signal 143' TERM

# HH_260810 - Kept exact process-group and guard-file cleanup on every exit path.
trap cleanup_all EXIT

# HH_260810 - Refused launch unless the audited direct camera and PC3 DDS links are already active.
if ! validate_network_layout; then
  exit 75
fi

# HH_260811 - Installed the non-persistent camera route only after all ownership and profile checks passed.
if ! install_runtime_camera_route; then
  exit 75
fi

# HH_260810 - Started the unchanged active launch in its own process group for exact Ctrl+C cleanup.
setsid ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/a/Autoware_Map/C_track \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  "$@" \
  launch_vehicle:=false \
  launch_system:=false \
  launch_map:=false \
  launch_localization:=false \
  launch_planning:=false \
  launch_control:=false \
  launch_api:=false \
  launch_vehicle_interface:=false &
launch_pid=$!
echo "${launch_pid}" > "${PC2_PID_FILE}"

wait "${launch_pid}"
status=$?
stop_launch_group
exit "${status}"
