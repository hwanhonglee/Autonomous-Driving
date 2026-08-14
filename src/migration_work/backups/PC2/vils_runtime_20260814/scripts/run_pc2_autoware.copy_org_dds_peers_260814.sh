#!/usr/bin/env bash

# HH_260810 - Added a guarded PC2 launcher while preserving the run_autoware alias and active launch filename.
readonly PC2_LOCK_FILE="/tmp/autoware_pc2_run.lock"
readonly PC2_PID_FILE="/tmp/autoware_pc2_run.pid"
readonly PC2_LAUNCH_PATTERN='[r]os2 launch autoware_launch autoware.launch.xml'
readonly PC2_CAMERA_LAUNCH_PATTERN='[r]os2 launch lucid_vision_driver'
# HH_260812 - Accept either audited wired profile without switching it; both must stay on the vehicle DDS /24.
readonly PC2_DDS_IFACE="enp0s31f6"
readonly -a PC2_DDS_CONNECTIONS=("Ethernet" "ROS2")
readonly PC2_DDS_SUBNET_PREFIX="192.168.9."
readonly PC2_DDS_PREFIX_LENGTH="24"
readonly PC2_PC1_IP="192.168.9.2"
readonly PC2_PC3_IP="192.168.9.7"
# HH_260811 - Kept the physically proven serial-214 loop-top camera on its dedicated link.
readonly PC2_CAMERA_IFACE="enp1s0f3"
readonly PC2_CAMERA_CONNECTION="Lucid Camera Loop Top"
readonly PC2_CAMERA_HOST_CIDR="169.254.0.1/24"
readonly PC2_CAMERA_HOST_IP="169.254.0.1"
readonly PC2_CAMERA_IP="169.254.0.11"
readonly PC2_CAMERA_ROUTE_CIDR="${PC2_CAMERA_IP}/32"
# HH_260811 - Kept serial 214 pinned to its proven NIC when two link-local camera networks are active.
readonly PC2_CAMERA_ROUTE_METRIC="42762"
readonly PC2_CAMERA_ROUTE_SPEC="${PC2_CAMERA_ROUTE_CIDR} 0.0.0.0 ${PC2_CAMERA_ROUTE_METRIC}"
# HH_260812 - Added the physically discovered serial-222 windshield camera on its own routed subnet.
readonly PC2_WINDSHIELD_CAMERA_IFACE="enp1s0f0"
readonly PC2_WINDSHIELD_CAMERA_CONNECTION="Lucid Camera Wind Shield"
readonly PC2_WINDSHIELD_CAMERA_HOST_CIDR="192.168.41.1/24"
readonly PC2_WINDSHIELD_CAMERA_HOST_IP="192.168.41.1"
readonly PC2_WINDSHIELD_CAMERA_IP="192.168.41.2"
# HH_260810 - Used audited topic_tools 1.1.2 plus the in-flight publish shutdown guard without modifying /opt/ros.
readonly PC2_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PC2_MIGRATION_ROOT="$(cd -- "${PC2_SCRIPT_DIR}/.." && pwd)"
readonly PC2_TOPIC_TOOLS_BUILD_SCRIPT="${PC2_SCRIPT_DIR}/build_topic_tools_overlay.sh"
readonly PC2_TOPIC_TOOLS_PREFIX="${PC2_MIGRATION_ROOT}/vendor-build/topic_tools-1.1.2-HH_260810/install"
readonly PC2_TOPIC_TOOLS_RELAY_LIBRARY="${PC2_TOPIC_TOOLS_PREFIX}/lib/librelay_node.so"
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
  local resolved_relay_library=""
  local resolved_prefix=""

  if [[ ! -x "${PC2_TOPIC_TOOLS_PREFIX}/lib/topic_tools/relay" ||
        ! -r "${PC2_TOPIC_TOOLS_RELAY_LIBRARY}" ]]; then
    if [[ ! -x "${PC2_TOPIC_TOOLS_BUILD_SCRIPT}" ]]; then
      echo "The guarded topic_tools overlay and its build script are both unavailable." >&2
      return 1
    fi
    echo "Building the guarded topic_tools overlay from vendored source." >&2
    "${PC2_TOPIC_TOOLS_BUILD_SCRIPT}" >&2 || return 1
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
  local count=""
  local output=""
  # HH_260810 - Used direct DDS discovery so preflight checks do not leave a ros2cli daemon behind.
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

  echo -1
  return 1
}

# HH_260812 - Query all duplicate guards concurrently, bounded by one 5-second window, and fail closed.
guard_owned_topic_publishers() {
  local count=""
  local index=0
  local query_failed=0
  local snapshot_dir=""
  local topic=""
  local -a jobs=()
  local -a result_files=()
  local -a topics=(
    /lucid_vision/camera/image
    /lucid_vision/camera/image_compressed
    /lucid_vision/camera/camera_info
    /lucid_vision/windshield/image
    /lucid_vision/windshield/image_compressed
    /lucid_vision/windshield/camera_info
    /sensing/camera/camera0/image_raw
    /sensing/camera/camera0/image_raw/compressed
    /sensing/camera/camera0/camera_info
    /sensing/camera/camera1/traffic_light/camera_info
    /sensing/camera/camera1/traffic_light/image_raw/compressed
    /sensing/camera/camera1/traffic_light/image_raw
    /perception/object_recognition/detection/rois0
    /perception/traffic_light_recognition/traffic_signals
    /perception/object_recognition/objects
  )

  snapshot_dir="$(mktemp -d /tmp/pc2-publisher-guard.XXXXXX)" || return 1
  for index in "${!topics[@]}"; do
    result_files[index]="${snapshot_dir}/${index}"
    publisher_count "${topics[index]}" > "${result_files[index]}" &
    jobs[index]=$!
  done

  for index in "${!topics[@]}"; do
    if ! wait "${jobs[index]}"; then
      query_failed=1
    fi
    count="$(<"${result_files[index]}")"
    if [[ ! "${count}" =~ ^-?[0-9]+$ ]] || (( count < 0 )); then
      echo "Cannot prove publisher ownership for ${topics[index]}; refusing launch." >&2
      query_failed=1
    elif (( count > 0 )); then
      echo "Topic ${topics[index]} already has a publisher; refusing a conflicting PC2 start." >&2
      query_failed=1
    fi
  done

  rm -f -- "${result_files[@]}"
  rmdir -- "${snapshot_dir}" 2>/dev/null || true
  (( query_failed == 0 ))
}

if ! guard_owned_topic_publishers; then
  exit 74
fi

launch_pid=""
camera_connection_uuid=""
camera_saved_routes_before=""
camera_route_owned=0
dds_host_cidr=""
dds_host_ip=""

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

windshield_camera_route_get_is_valid() {
  local route_line

  route_line="$(ip -o -4 route get "${PC2_WINDSHIELD_CAMERA_IP}" 2>/dev/null)" || return 1
  [[ " ${route_line} " == *" dev ${PC2_WINDSHIELD_CAMERA_IFACE} "* ]] &&
    [[ " ${route_line} " == *" src ${PC2_WINDSHIELD_CAMERA_HOST_IP} "* ]]
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

dds_route_get_is_valid() {
  local target_ip="$1"
  local route_line

  [[ -n "${dds_host_ip}" ]] || return 1
  route_line="$(ip -o -4 route get "${target_ip}" 2>/dev/null)" || return 1
  [[ " ${route_line} " == *" dev ${PC2_DDS_IFACE} "* ]] &&
    [[ " ${route_line} " == *" src ${dds_host_ip} "* ]]
}

active_connection_on_interface() {
  nmcli -g GENERAL.CONNECTION device show "$1" 2>/dev/null
}

dds_connection_is_allowed() {
  local active_connection="$1"
  local allowed_connection

  for allowed_connection in "${PC2_DDS_CONNECTIONS[@]}"; do
    [[ "${active_connection}" == "${allowed_connection}" ]] && return 0
  done
  return 1
}

# HH_260812 - Derived the active DDS source instead of forcing .110; reject any address outside 192.168.9.0/24.
find_dds_host_cidr() {
  local cidr=""
  local host_ip=""
  local octet=""

  while read -r cidr; do
    [[ "${cidr}" == "${PC2_DDS_SUBNET_PREFIX}"*"/${PC2_DDS_PREFIX_LENGTH}" ]] || continue
    host_ip="${cidr%/*}"
    octet="${host_ip##*.}"
    if [[ "${octet}" =~ ^[0-9]+$ ]] && (( octet >= 1 && octet <= 254 )); then
      printf '%s\n' "${cidr}"
      return 0
    fi
  done < <(ip -o -4 address show dev "${PC2_DDS_IFACE}" scope global 2>/dev/null | awk '{print $4}')
  return 1
}

# HH_260810 - Validated the direct camera link and PC3-reachable DDS link without changing profiles or routes.
validate_network_layout() {
  local active_camera_connection=""
  local active_dds_connection=""
  local active_windshield_camera_connection=""
  local camera_saved_routes=""
  local dds_connection_uuid=""
  local dds_saved_routes=""
  local exact_camera_route=""
  local route_line=""
  local windshield_camera_connection_uuid=""
  local windshield_camera_saved_routes=""

  active_camera_connection="$(active_connection_on_interface "${PC2_CAMERA_IFACE}")" || {
    echo "Cannot inspect NetworkManager state for ${PC2_CAMERA_IFACE}." >&2
    return 1
  }
  active_dds_connection="$(active_connection_on_interface "${PC2_DDS_IFACE}")" || {
    echo "Cannot inspect NetworkManager state for ${PC2_DDS_IFACE}." >&2
    return 1
  }
  active_windshield_camera_connection="$(
    active_connection_on_interface "${PC2_WINDSHIELD_CAMERA_IFACE}"
  )" || {
    echo "Cannot inspect NetworkManager state for ${PC2_WINDSHIELD_CAMERA_IFACE}." >&2
    return 1
  }

  if [[ "${active_camera_connection}" != "${PC2_CAMERA_CONNECTION}" ]]; then
    echo "Expected ${PC2_CAMERA_CONNECTION} on ${PC2_CAMERA_IFACE}; found ${active_camera_connection:-none}." >&2
    return 1
  fi
  if ! dds_connection_is_allowed "${active_dds_connection}"; then
    echo "Expected Ethernet or ROS2 on ${PC2_DDS_IFACE}; found ${active_dds_connection:-none}." >&2
    return 1
  fi
  if [[ "${active_windshield_camera_connection}" != "${PC2_WINDSHIELD_CAMERA_CONNECTION}" ]]; then
    echo "Expected ${PC2_WINDSHIELD_CAMERA_CONNECTION} on ${PC2_WINDSHIELD_CAMERA_IFACE}; found ${active_windshield_camera_connection:-none}." >&2
    return 1
  fi
  if ! interface_address_is_valid "${PC2_CAMERA_IFACE}" "${PC2_CAMERA_HOST_CIDR}"; then
    echo "${PC2_CAMERA_HOST_CIDR} is not active on ${PC2_CAMERA_IFACE}." >&2
    return 1
  fi
  if ! interface_address_is_valid "${PC2_WINDSHIELD_CAMERA_IFACE}" "${PC2_WINDSHIELD_CAMERA_HOST_CIDR}"; then
    echo "${PC2_WINDSHIELD_CAMERA_HOST_CIDR} is not active on ${PC2_WINDSHIELD_CAMERA_IFACE}." >&2
    return 1
  fi
  dds_host_cidr="$(find_dds_host_cidr)" || {
    echo "No usable ${PC2_DDS_SUBNET_PREFIX}x/${PC2_DDS_PREFIX_LENGTH} address is active on ${PC2_DDS_IFACE}." >&2
    return 1
  }
  dds_host_ip="${dds_host_cidr%/*}"

  camera_connection_uuid="$(
    nmcli -g GENERAL.CON-UUID device show "${PC2_CAMERA_IFACE}" 2>/dev/null
  )" || return 1
  dds_connection_uuid="$(
    nmcli -g GENERAL.CON-UUID device show "${PC2_DDS_IFACE}" 2>/dev/null
  )" || return 1
  windshield_camera_connection_uuid="$(
    nmcli -g GENERAL.CON-UUID device show "${PC2_WINDSHIELD_CAMERA_IFACE}" 2>/dev/null
  )" || return 1
  if [[ -z "${camera_connection_uuid}" || "${camera_connection_uuid}" == "--" ]]; then
    echo "No active NetworkManager UUID found for ${PC2_CAMERA_IFACE}." >&2
    return 1
  fi
  if [[ -z "${dds_connection_uuid}" || "${dds_connection_uuid}" == "--" ]]; then
    echo "No active NetworkManager UUID found for ${PC2_DDS_IFACE}." >&2
    return 1
  fi
  if [[ -z "${windshield_camera_connection_uuid}" || "${windshield_camera_connection_uuid}" == "--" ]]; then
    echo "No active NetworkManager UUID found for ${PC2_WINDSHIELD_CAMERA_IFACE}." >&2
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
  windshield_camera_saved_routes="$(
    nmcli -g ipv4.routes connection show uuid "${windshield_camera_connection_uuid}" 2>/dev/null
  )" || {
    echo "Cannot snapshot saved routes for ${windshield_camera_connection_uuid}." >&2
    return 1
  }
  if [[ -n "${camera_saved_routes}" || -n "${windshield_camera_saved_routes}" ||
        -n "${dds_saved_routes}" ]]; then
    echo "Unexpected saved routes exist on an audited camera or DDS profile." >&2
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
  if ! dds_route_get_is_valid "${PC2_PC1_IP}"; then
    route_line="$(ip -o -4 route get "${PC2_PC1_IP}" 2>&1 || true)"
    echo "PC1 DDS path validation failed: ${route_line}" >&2
    return 1
  fi
  if ! dds_route_get_is_valid "${PC2_PC3_IP}"; then
    route_line="$(ip -o -4 route get "${PC2_PC3_IP}" 2>&1 || true)"
    echo "PC3 DDS path validation failed: ${route_line}" >&2
    return 1
  fi
  if ! windshield_camera_route_get_is_valid; then
    route_line="$(ip -o -4 route get "${PC2_WINDSHIELD_CAMERA_IP}" 2>&1 || true)"
    echo "Windshield Lucid path validation failed: ${route_line}" >&2
    return 1
  fi
  if ! ping -c 1 -W 1 -I "${PC2_WINDSHIELD_CAMERA_IFACE}" \
      "${PC2_WINDSHIELD_CAMERA_IP}" >/dev/null 2>&1; then
    echo "The windshield Lucid camera did not answer on ${PC2_WINDSHIELD_CAMERA_IFACE}." >&2
    return 1
  fi

  echo "Validated loop-top Lucid interface ${PC2_CAMERA_HOST_CIDR} on ${PC2_CAMERA_IFACE}." >&2
  echo "Validated windshield Lucid path ${PC2_WINDSHIELD_CAMERA_HOST_IP} -> ${PC2_WINDSHIELD_CAMERA_IP} on ${PC2_WINDSHIELD_CAMERA_IFACE}." >&2
  echo "Validated PC1/PC3 DDS paths from ${dds_host_cidr} on ${PC2_DDS_IFACE} (${active_dds_connection})." >&2
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
