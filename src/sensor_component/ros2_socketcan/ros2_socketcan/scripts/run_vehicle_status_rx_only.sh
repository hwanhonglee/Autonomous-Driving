#!/usr/bin/env bash

set -Eeuo pipefail
set +m
umask 077

readonly vils_script_name="run_vehicle_status_rx_only"
readonly vils_primary_can="can0"
readonly -a vils_inactive_can_interfaces=(can1 can2 can3)
readonly vils_vehicle_nic="enp0s31f6"
readonly vils_vehicle_address="192.168.9.2/24"
readonly vils_expected_prefix="/home/a/autoware/install/ros2_socketcan"
readonly vils_approved_cyclonedds="/home/a/autoware/src/migration_work/config/cyclonedds_pc1.xml"
readonly vils_approved_cyclonedds_sha256="0e4f30b76e868f5477dc8252736d66502466efbd5fd6bb7d1cde8cddbd49f880"
readonly vils_source_package_root="/home/a/autoware/src/sensor_component/ros2_socketcan/ros2_socketcan"
readonly vils_source_rx_only_launch="${vils_source_package_root}/launch/vehicle_status_rx_only.launch.xml"
readonly vils_source_receiver_launch="${vils_source_package_root}/launch/socket_can_receiver.launch.py"
readonly vils_approved_rx_only_launch_sha256="154ab6c93e0857dd23d5bb850166698085daa941a83cb1442634013a12c8e4bf"
readonly vils_approved_receiver_launch_sha256="aa43a8bcf25a7450b38d6f003b42da04d80c55987dc84202b9f3f4a78e4aefee"
readonly vils_approved_receiver_binary_sha256="f37b0a7d9a4b2d15da80090f0ce22e234415f12fbcef2a2c198f94a396758fb6"
readonly vils_approved_receiver_component_sha256="466b3b397bffdbaa758170a0f2bf3951e7167069f4eb56e4a6535139d0c1e734"
readonly vils_approved_receiver_core_sha256="f477db3055f0b96a98371c4ad3ae98f5cc1d26eb42040764ce8075a7bd2a57a6"
readonly vils_expected_receiver_node_name="socket_can_receiver_can0"
readonly vils_expected_receiver_node_namespace="/"
readonly vils_hard_max_last_offset_ms="10.0"
readonly vils_hard_max_rms_offset_ms="10.0"
readonly vils_hard_max_root_dispersion_ms="50.0"
readonly vils_default_evidence_root="/home/a/vils_pc1_evidence"
readonly vils_lock_path="/run/user/$(id -u)/vils-pc1-vehicle-status-rx-only.lock"

vils_evidence_root="${VILS_PC1_EVIDENCE_ROOT:-${vils_default_evidence_root}}"
vils_expected_time_source="${VILS_PC1_EXPECTED_TIME_SOURCE:-}"
vils_max_last_offset_ms="${VILS_PC1_MAX_LAST_OFFSET_MS:-}"
vils_max_rms_offset_ms="${VILS_PC1_MAX_RMS_OFFSET_MS:-}"
vils_max_root_dispersion_ms="${VILS_PC1_MAX_ROOT_DISPERSION_MS:-}"
vils_check_only=false
vils_duration_seconds_raw=0
vils_duration_seconds=0
vils_launch_start_seconds=0
vils_deadline_seconds=0
vils_launch_pid=""
vils_timer_pid=""
vils_run_dir=""
vils_abort_reason_file=""
vils_normal_stop_file=""
vils_cyclonedds_path=""
vils_receiver_binary=""
vils_receiver_component_library=""
vils_receiver_core_library=""
vils_installed_launch=""
vils_installed_receiver_launch=""
vils_installed_runner=""
vils_requested_stop=""
vils_pending_signal=""
vils_registering_child=false
vils_launch_group_ready=false
vils_ready=false
vils_cleaning=false
vils_parent_starttime=""
vils_launch_starttime=""
vils_timer_starttime=""
vils_receiver_pid=""
vils_receiver_starttime=""
vils_unregistered_child_status=124
declare -A vils_initial_ifindex=()
declare -A vils_initial_tx_packets=()
declare -A vils_expected_publisher_gid=()

fail()
{
  printf '[%s] BLOCKED: %s\n' "${vils_script_name}" "$*" >&2
  exit 1
}

process_starttime()
{
  local process_id="$1"
  local stat_record
  local fields_record
  local -a stat_fields

  [[ ${process_id} =~ ^[1-9][0-9]*$ && -r /proc/${process_id}/stat ]] || return 1
  stat_record=$(</proc/"${process_id}"/stat) || return 1
  # /proc/PID/stat field 2 is parenthesized and may itself contain spaces or ')'.
  fields_record=${stat_record##*) }
  read -r -a stat_fields <<<"${fields_record}"
  # After removing fields 1 and 2, array index 19 is the original field 22 (starttime).
  (( ${#stat_fields[@]} > 19 )) || return 1
  [[ ${stat_fields[19]} =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${stat_fields[19]}"
}

process_state_for_identity()
{
  local process_id="$1"
  local expected_starttime="$2"
  local expected_pgid="${3:-}"
  local expected_session="${4:-}"
  local stat_record
  local fields_record
  local -a stat_fields

  if [[ ! -e /proc/${process_id} ]]; then
    printf 'gone\n'
    return 0
  fi
  stat_record=$(</proc/"${process_id}"/stat) || {
    printf 'unknown\n'
    return 0
  }
  fields_record=${stat_record##*) }
  read -r -a stat_fields <<<"${fields_record}"
  if (( ${#stat_fields[@]} <= 19 )) || [[ ! ${stat_fields[19]} =~ ^[0-9]+$ ]]; then
    printf 'unknown\n'
  elif [[ ${stat_fields[19]} != "${expected_starttime}" ]]; then
    printf 'mismatch\n'
  elif [[ ${stat_fields[0]} == Z ]]; then
    printf 'zombie\n'
  elif [[ -n ${expected_pgid} && ${stat_fields[2]} != "${expected_pgid}" ]]; then
    printf 'wrong-group\n'
  elif [[ -n ${expected_session} && ${stat_fields[3]} != "${expected_session}" ]]; then
    printf 'wrong-group\n'
  else
    printf 'alive\n'
  fi
}

usage()
{
  cat <<'USAGE'
Usage: run_vehicle_status_rx_only.sh [options]

Options:
  --check-only             Run every preflight gate without starting the receiver.
  --duration-seconds SEC   Stop after SEC (30..86400); 0 waits for Ctrl+C.
  --evidence-root PATH     Store a new non-overwriting evidence directory below PATH.
  -h, --help               Show this help.

Required environment:
  VILS_PC1_EXPECTED_TIME_SOURCE  Operator-approved Chrony source selected on all four PCs.
  VILS_PC1_MAX_LAST_OFFSET_MS    Approved absolute last-offset limit in milliseconds.
  VILS_PC1_MAX_RMS_OFFSET_MS     Approved RMS-offset limit in milliseconds.
  VILS_PC1_MAX_ROOT_DISPERSION_MS  Approved root-dispersion limit in milliseconds.

This is the only approved entry point for the parked PC1 VILS RX-only profile.
It requires can0 controller-level LISTEN-ONLY and can1..can3 administratively DOWN.
It never starts socket_can_sender_node_exe or twistController2VCU2EPS2ACC_node.
USAGE
}

while (( $# > 0 )); do
  case "$1" in
    --check-only)
      vils_check_only=true
      shift
      ;;
    --duration-seconds)
      (( $# >= 2 )) || fail "--duration-seconds requires a value"
      vils_duration_seconds_raw="$2"
      shift 2
      ;;
    --evidence-root)
      (( $# >= 2 )) || fail "--evidence-root requires a path"
      vils_evidence_root="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unsupported option: $1"
      ;;
  esac
done

[[ ${vils_duration_seconds_raw} =~ ^[0-9]+$ ]] || fail "duration must be a non-negative decimal integer"
(( ${#vils_duration_seconds_raw} <= 5 )) || fail "duration is outside the supported decimal range"
vils_duration_seconds=$((10#${vils_duration_seconds_raw}))
if (( vils_duration_seconds != 0 && (vils_duration_seconds < 30 || vils_duration_seconds > 86400) )); then
  fail "duration must be 0 or between 30 and 86400 seconds"
fi

readonly vils_parent_pid="$$"
vils_parent_starttime=$(process_starttime "${vils_parent_pid}") ||
  fail "cannot capture the RX-only supervisor process identity"
readonly vils_parent_starttime

# HH_260814 - Keep Conda from selecting an incompatible Python ABI for ROS Humble tooling.
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
unset PYTHONHOME
set +u
source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash
set -u

for vils_command in awk chronyc find flock git grep ip mktemp pgrep pkill ps python3 readlink \
  realpath ros2 setsid sha256sum sort stat timeout tr xargs; do
  command -v "${vils_command}" >/dev/null 2>&1 || fail "required command is missing: ${vils_command}"
done

[[ ${ROS_DOMAIN_ID:-10} == 10 ]] || fail "ROS_DOMAIN_ID must be 10 for the vehicle-domain profile"
[[ ${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} == rmw_cyclonedds_cpp ]] ||
  fail "RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp"
[[ ${ROS_LOCALHOST_ONLY:-0} == 0 ]] || fail "ROS_LOCALHOST_ONLY must be 0 for the four-PC profile"
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

[[ ${CYCLONEDDS_URI:-} == file://* ]] ||
  fail "CYCLONEDDS_URI must name the approved PC1 file:// profile"
vils_cyclonedds_path=${CYCLONEDDS_URI#file://}
cyclonedds_profile_guard_reason()
{
  local parsed

  [[ -r ${vils_cyclonedds_path} ]] || {
    printf 'CycloneDDS profile is not readable: %s' "${vils_cyclonedds_path}"
    return 1
  }
  [[ $(realpath "${vils_cyclonedds_path}") == $(realpath "${vils_approved_cyclonedds}") ]] || {
    printf 'CycloneDDS profile is not the reviewed PC1 profile: %s' \
      "${vils_approved_cyclonedds}"
    return 1
  }
  [[ $(sha256sum "${vils_cyclonedds_path}" | awk '{print $1}') == \
    "${vils_approved_cyclonedds_sha256}" ]] || {
    printf 'CycloneDDS profile hash differs from the reviewed PC1 profile'
    return 1
  }
  parsed=$(python3 - "${vils_cyclonedds_path}" "${vils_vehicle_nic}" <<'PY'
import sys
import xml.etree.ElementTree as ET

path, expected_interface = sys.argv[1:]
root = ET.parse(path).getroot()
interfaces = [
    element.attrib.get("name")
    for element in root.iter()
    if element.tag.rsplit("}", 1)[-1] == "NetworkInterface"
]
if interfaces != [expected_interface]:
    raise SystemExit(f"expected exactly NetworkInterface {expected_interface!r}, got {interfaces!r}")
PY
  ) || {
    printf 'CycloneDDS XML does not pin exactly %s: %s' "${vils_vehicle_nic}" "${parsed}"
    return 1
  }
  return 0
}
vils_profile_reason=$(cyclonedds_profile_guard_reason) || fail "${vils_profile_reason}"

[[ -n ${vils_expected_time_source} ]] ||
  fail "VILS_PC1_EXPECTED_TIME_SOURCE is unset; the four-PC clock source is not approved"
if ! python3 - "${vils_max_last_offset_ms}" "${vils_max_rms_offset_ms}" \
  "${vils_max_root_dispersion_ms}" "${vils_hard_max_last_offset_ms}" \
  "${vils_hard_max_rms_offset_ms}" "${vils_hard_max_root_dispersion_ms}" <<'PY'
import math
import sys

names = ("MAX_LAST_OFFSET_MS", "MAX_RMS_OFFSET_MS", "MAX_ROOT_DISPERSION_MS")
values = []
for name, raw_value in zip(names, sys.argv[1:4]):
    try:
        value = float(raw_value)
    except ValueError:
        raise SystemExit(f"VILS_PC1_{name} must be a positive finite number")
    if not math.isfinite(value) or value <= 0.0:
        raise SystemExit(f"VILS_PC1_{name} must be a positive finite number")
    values.append(value)
hard_limits = [float(value) for value in sys.argv[4:7]]
for name, value, hard_limit in zip(names, values, hard_limits):
    if value > hard_limit:
        raise SystemExit(
            f"VILS_PC1_{name}={value} exceeds the reviewed ceiling {hard_limit}"
        )
PY
then
  fail "the four-PC Chrony offset/dispersion limits are unset or invalid"
fi

readonly vils_package_prefix="$(ros2 pkg prefix ros2_socketcan)"
[[ $(realpath "${vils_package_prefix}") == $(realpath "${vils_expected_prefix}") ]] ||
  fail "active ros2_socketcan prefix is not the reviewed Autoware install: ${vils_package_prefix}"
vils_receiver_binary="${vils_package_prefix}/lib/ros2_socketcan/socket_can_receiver_node_exe"
vils_receiver_component_library="${vils_package_prefix}/lib/libsocket_can_receiver_node.so"
vils_receiver_core_library="${vils_package_prefix}/lib/libros2_socketcan.so"
vils_installed_launch="${vils_package_prefix}/share/ros2_socketcan/launch/vehicle_status_rx_only.launch.xml"
vils_installed_receiver_launch="${vils_package_prefix}/share/ros2_socketcan/launch/socket_can_receiver.launch.py"
vils_installed_runner="${vils_package_prefix}/lib/ros2_socketcan/run_vehicle_status_rx_only.sh"
[[ -x ${vils_receiver_binary} ]] || fail "receiver executable is missing: ${vils_receiver_binary}"
[[ -r ${vils_receiver_component_library} ]] ||
  fail "receiver component library is missing: ${vils_receiver_component_library}"
[[ -r ${vils_receiver_core_library} ]] ||
  fail "receiver core library is missing: ${vils_receiver_core_library}"
[[ -r ${vils_installed_launch} ]] || fail "RX-only launch is missing: ${vils_installed_launch}"
[[ -r ${vils_installed_receiver_launch} ]] ||
  fail "receiver child launch is missing: ${vils_installed_receiver_launch}"
[[ -x ${vils_installed_runner} ]] || fail "RX-only runner is missing: ${vils_installed_runner}"
[[ $(realpath "$0") == $(realpath "${vils_installed_runner}") ]] ||
  fail "the invoked runner is not the reviewed installed runner"
[[ $(realpath "${vils_installed_launch}") == $(realpath "${vils_source_rx_only_launch}") ]] ||
  fail "installed RX-only launch does not resolve to the reviewed source launch"
[[ $(realpath "${vils_installed_receiver_launch}") == $(realpath "${vils_source_receiver_launch}") ]] ||
  fail "installed receiver child launch does not resolve to the reviewed source launch"
[[ $(sha256sum "${vils_installed_receiver_launch}" | awk '{print $1}') == \
  "${vils_approved_receiver_launch_sha256}" ]] ||
  fail "receiver child launch hash differs from the reviewed composition"
readonly vils_expected_receiver_exe="$(realpath "${vils_receiver_binary}")"
readonly vils_expected_receiver_component="$(realpath "${vils_receiver_component_library}")"
readonly vils_expected_receiver_core="$(realpath "${vils_receiver_core_library}")"

launch_provenance_guard_reason()
{
  local guard_reason
  local observed_hash

  guard_reason=$(cyclonedds_profile_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  [[ -x ${vils_receiver_binary} && -r ${vils_receiver_component_library} && \
    -r ${vils_receiver_core_library} && -r ${vils_installed_launch} && \
    -r ${vils_installed_receiver_launch} && -x ${vils_installed_runner} ]] || {
    printf 'installed RX-only composition is incomplete'
    return 1
  }
  [[ $(realpath "${vils_receiver_binary}") == "${vils_expected_receiver_exe}" ]] || {
    printf 'receiver executable no longer resolves to the reviewed binary'
    return 1
  }
  [[ $(realpath "${vils_receiver_component_library}") == \
    "${vils_expected_receiver_component}" ]] || {
    printf 'receiver component library no longer resolves to the reviewed binary'
    return 1
  }
  [[ $(realpath "${vils_receiver_core_library}") == "${vils_expected_receiver_core}" ]] || {
    printf 'receiver core library no longer resolves to the reviewed binary'
    return 1
  }
  [[ $(realpath "${vils_installed_launch}") == $(realpath "${vils_source_rx_only_launch}") ]] || {
    printf 'installed RX-only wrapper no longer resolves to its reviewed source'
    return 1
  }
  [[ $(realpath "${vils_installed_receiver_launch}") == \
    $(realpath "${vils_source_receiver_launch}") ]] || {
    printf 'installed receiver child no longer resolves to its reviewed source'
    return 1
  }
  [[ $(realpath "${vils_installed_runner}") == $(realpath "$0") ]] || {
    printf 'installed RX-only runner no longer resolves to the active runner'
    return 1
  }
  observed_hash=$(sha256sum "${vils_installed_launch}" | awk '{print $1}') || return 1
  [[ ${observed_hash} == "${vils_approved_rx_only_launch_sha256}" ]] || {
    printf 'RX-only wrapper hash changed from the reviewed composition'
    return 1
  }
  observed_hash=$(sha256sum "${vils_installed_receiver_launch}" | awk '{print $1}') || return 1
  [[ ${observed_hash} == "${vils_approved_receiver_launch_sha256}" ]] || {
    printf 'receiver child launch hash changed from the reviewed composition'
    return 1
  }
  observed_hash=$(sha256sum "${vils_expected_receiver_exe}" | awk '{print $1}') || return 1
  [[ ${observed_hash} == "${vils_approved_receiver_binary_sha256}" ]] || {
    printf 'receiver executable hash changed from the reviewed binary'
    return 1
  }
  observed_hash=$(sha256sum "${vils_expected_receiver_component}" | awk '{print $1}') || return 1
  [[ ${observed_hash} == "${vils_approved_receiver_component_sha256}" ]] || {
    printf 'receiver component library hash changed from the reviewed binary'
    return 1
  }
  observed_hash=$(sha256sum "${vils_expected_receiver_core}" | awk '{print $1}') || return 1
  [[ ${observed_hash} == "${vils_approved_receiver_core_sha256}" ]] || {
    printf 'receiver core library hash changed from the reviewed binary'
    return 1
  }
  return 0
}

exec 9<>"${vils_lock_path}"
flock -n 9 || fail "another RX-only runner holds ${vils_lock_path}"

can_tx_packets()
{
  local interface="$1"
  local statistics_path="/sys/class/net/${interface}/statistics/tx_packets"
  local value

  [[ -r ${statistics_path} ]] || return 1
  value=$(tr -d '[:space:]' <"${statistics_path}") || return 1
  [[ ${value} =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${value}"
}

primary_can_signature()
{
  local raw_json
  local parsed

  raw_json=$(timeout 2 ip -details -json link show dev "${vils_primary_can}" 2>&1) || {
    printf 'cannot inspect %s: %s' "${vils_primary_can}" "${raw_json}"
    return 1
  }
  parsed=$(python3 -c '
import json, sys

def reject(message):
    print(message)
    raise SystemExit(1)

def strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            yield from strings(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from strings(item)

items = json.load(sys.stdin)
if not isinstance(items, list) or len(items) != 1:
    reject(f"expected one link record, got {items!r}")
link = items[0]
if "UP" not in link.get("flags", []):
    reject("can0 is not administratively UP")
link_type = link.get("link_type")
if link_type != "can":
    reject(f"can0 link_type is {link_type!r}, not can")
linkinfo = link.get("linkinfo", {})
info_kind = linkinfo.get("info_kind")
if info_kind != "can":
    reject(f"can0 info_kind is {info_kind!r}, not can")
info = linkinfo.get("info_data", {})
state = info.get("state")
if str(state or "").upper() != "ERROR-ACTIVE":
    reject(f"can0 state is {state!r}, not ERROR-ACTIVE")
bittiming = info.get("bittiming", {})
bitrate = bittiming.get("bitrate") if isinstance(bittiming, dict) else None
if bitrate != 500000:
    reject(f"can0 bitrate is {bitrate!r}, not 500000")
ctrlmode_strings = {token.upper() for token in strings(info.get("ctrlmode", []))}
if "LISTEN-ONLY" not in ctrlmode_strings:
    reject(f"can0 ctrlmode lacks LISTEN-ONLY: {sorted(ctrlmode_strings)!r}")
ifindex = link.get("ifindex")
if not isinstance(ifindex, int) or ifindex <= 0:
    reject(f"invalid can0 ifindex: {ifindex!r}")
print(ifindex)
' <<<"${raw_json}" 2>&1) || {
    printf '%s' "${parsed}"
    return 1
  }
  printf '%s\n' "${parsed}"
}

inactive_can_signature()
{
  local interface="$1"
  local raw_json
  local parsed

  raw_json=$(timeout 2 ip -details -json link show dev "${interface}" 2>&1) || {
    printf 'cannot inspect %s: %s' "${interface}" "${raw_json}"
    return 1
  }
  parsed=$(python3 -c '
import json, sys

interface = sys.argv[1]
items = json.load(sys.stdin)
if not isinstance(items, list) or len(items) != 1:
    print(f"expected one {interface} link record, got {items!r}")
    raise SystemExit(1)
link = items[0]
if "UP" in link.get("flags", []):
    print(f"{interface} must remain administratively DOWN during the RX-only window")
    raise SystemExit(1)
if link.get("link_type") != "can" or link.get("linkinfo", {}).get("info_kind") != "can":
    print(f"{interface} is not a CAN interface")
    raise SystemExit(1)
ifindex = link.get("ifindex")
if not isinstance(ifindex, int) or ifindex <= 0:
    print(f"invalid {interface} ifindex: {ifindex!r}")
    raise SystemExit(1)
print(ifindex)
' "${interface}" <<<"${raw_json}" 2>&1) || {
    printf '%s' "${parsed}"
    return 1
  }
  printf '%s\n' "${parsed}"
}

network_guard_reason()
{
  local nic_line
  local nic_flags

  [[ -e "/sys/class/net/${vils_vehicle_nic}" ]] || {
    printf 'vehicle NIC %s is missing' "${vils_vehicle_nic}"
    return 1
  }
  nic_line=$(timeout 2 ip -o link show dev "${vils_vehicle_nic}" 2>&1) || {
    printf 'cannot inspect vehicle NIC %s: %s' "${vils_vehicle_nic}" "${nic_line}"
    return 1
  }
  nic_flags=${nic_line#*<}
  nic_flags=${nic_flags%%>*}
  case ",${nic_flags}," in
    *,UP,*) ;;
    *)
      printf 'vehicle NIC %s is not administratively UP' "${vils_vehicle_nic}"
      return 1
      ;;
  esac
  timeout 2 ip -4 -o address show dev "${vils_vehicle_nic}" | awk '{print $4}' |
    grep -Fxq "${vils_vehicle_address}" || {
      printf 'vehicle NIC %s does not own %s' "${vils_vehicle_nic}" "${vils_vehicle_address}"
      return 1
    }
  return 0
}

time_guard_reason()
{
  local selected_sources
  local tracking
  local threshold_result

  tracking=$(timeout 2 chronyc tracking 2>&1) || {
    printf 'chronyc tracking failed: %s' "${tracking}"
    return 1
  }
  grep -Eq '^Leap status[[:space:]]*:[[:space:]]*Normal$' <<<"${tracking}" || {
    printf 'Chrony leap status is not Normal'
    return 1
  }
  selected_sources=$(timeout 2 chronyc sources -n 2>&1) || {
    printf 'chronyc sources failed: %s' "${selected_sources}"
    return 1
  }
  awk '$1 == "^*" {print $2}' <<<"${selected_sources}" |
    grep -Fxq "${vils_expected_time_source}" || {
      printf 'selected Chrony source is not the approved source %s' "${vils_expected_time_source}"
      return 1
    }
  threshold_result=$(python3 -c '
import re
import sys

limits = {
    "Last offset": float(sys.argv[1]),
    "RMS offset": float(sys.argv[2]),
    "Root dispersion": float(sys.argv[3]),
}
tracking = sys.argv[4]
observed = {}
pattern = re.compile(r"^([^:]+):\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s+seconds")
for line in tracking.splitlines():
    match = pattern.match(line.strip())
    if match and match.group(1).strip() in limits:
        observed[match.group(1).strip()] = abs(float(match.group(2))) * 1000.0
missing = sorted(set(limits) - set(observed))
if missing:
    raise SystemExit(f"missing Chrony metrics: {missing!r}")
violations = [
    f"{name}={observed[name]:.6f}ms>{limit:.6f}ms"
    for name, limit in limits.items()
    if observed[name] > limit
]
if violations:
    raise SystemExit("Chrony threshold exceeded: " + ", ".join(violations))
' "${vils_max_last_offset_ms}" "${vils_max_rms_offset_ms}" \
    "${vils_max_root_dispersion_ms}" "${tracking}" 2>&1) || {
      printf '%s' "${threshold_result}"
      return 1
    }
  return 0
}

capture_can_baseline()
{
  local interface
  local signature
  local tx_packets

  signature=$(primary_can_signature) || fail "${signature}; do not start a VILS run"
  vils_initial_ifindex["${vils_primary_can}"]=${signature}
  tx_packets=$(can_tx_packets "${vils_primary_can}") ||
    fail "cannot read ${vils_primary_can} TX counter"
  vils_initial_tx_packets["${vils_primary_can}"]=${tx_packets}

  for interface in "${vils_inactive_can_interfaces[@]}"; do
    signature=$(inactive_can_signature "${interface}") || fail "${signature}; do not start a VILS run"
    vils_initial_ifindex["${interface}"]=${signature}
    tx_packets=$(can_tx_packets "${interface}") || fail "cannot read ${interface} TX counter"
    vils_initial_tx_packets["${interface}"]=${tx_packets}
  done
}

can_fleet_guard_reason()
{
  local interface
  local signature
  local tx_packets

  signature=$(primary_can_signature) || {
    printf '%s' "${signature}"
    return 1
  }
  [[ ${signature} == "${vils_initial_ifindex[${vils_primary_can}]}" ]] || {
    printf '%s ifindex changed from %s to %s' "${vils_primary_can}" \
      "${vils_initial_ifindex[${vils_primary_can}]}" "${signature}"
    return 1
  }

  for interface in "${vils_inactive_can_interfaces[@]}"; do
    signature=$(inactive_can_signature "${interface}") || {
      printf '%s' "${signature}"
      return 1
    }
    [[ ${signature} == "${vils_initial_ifindex[${interface}]}" ]] || {
      printf '%s ifindex changed from %s to %s' "${interface}" \
        "${vils_initial_ifindex[${interface}]}" "${signature}"
      return 1
    }
  done

  for interface in "${vils_primary_can}" "${vils_inactive_can_interfaces[@]}"; do
    tx_packets=$(can_tx_packets "${interface}") || {
      printf 'cannot read %s TX counter' "${interface}"
      return 1
    }
    [[ ${tx_packets} == "${vils_initial_tx_packets[${interface}]}" ]] || {
      printf '%s TX changed from %s to %s' "${interface}" \
        "${vils_initial_tx_packets[${interface}]}" "${tx_packets}"
      return 1
    }
  done
  return 0
}

topic_endpoint_total()
{
  local topic_name="$1"
  local topic_list
  local topic_info
  local publisher_count
  local subscription_count

  topic_list=$(timeout 3 ros2 topic list --no-daemon --spin-time 0.5 2>&1) || {
    printf 'ros2 topic list failed while checking %s: %s' "${topic_name}" "${topic_list}"
    return 1
  }
  if ! grep -Fxq "${topic_name}" <<<"${topic_list}"; then
    printf '0\n'
    return 0
  fi
  topic_info=$(timeout 3 ros2 topic info --no-daemon --spin-time 0.5 "${topic_name}" 2>&1) || {
    printf 'ros2 topic info failed for %s: %s' "${topic_name}" "${topic_info}"
    return 1
  }
  publisher_count=$(awk -F: '/^Publisher count:/{gsub(/[[:space:]]/, "", $2); print $2}' <<<"${topic_info}")
  subscription_count=$(awk -F: '/^Subscription count:/{gsub(/[[:space:]]/, "", $2); print $2}' <<<"${topic_info}")
  [[ ${publisher_count} =~ ^[0-9]+$ && ${subscription_count} =~ ^[0-9]+$ ]] || {
    printf 'unparsable endpoint counts for %s: %s' "${topic_name}" "${topic_info}"
    return 1
  }
  printf '%d\n' "$((10#${publisher_count} + 10#${subscription_count}))"
}

expected_topic_type()
{
  case "$1" in
    /from_can_bus) printf 'can_msgs/msg/Frame\n' ;;
    /vehicle/status/velocity_status) printf 'autoware_vehicle_msgs/msg/VelocityReport\n' ;;
    /vehicle/status/steering_status) printf 'autoware_vehicle_msgs/msg/SteeringReport\n' ;;
    /vehicle/status/gear_status) printf 'autoware_vehicle_msgs/msg/GearReport\n' ;;
    /vehicle/status/control_mode) printf 'autoware_vehicle_msgs/msg/ControlModeReport\n' ;;
    *) return 1 ;;
  esac
}

topic_publisher_record()
{
  local topic_name="$1"
  local expected_type="$2"
  local topic_info
  local parsed

  topic_info=$(timeout 4 ros2 topic info --no-daemon --spin-time 0.5 -v "${topic_name}" 2>&1) || {
    printf 'cannot inspect publisher provenance for %s: %s' "${topic_name}" "${topic_info}"
    return 1
  }
  parsed=$(python3 -c '
import re
import sys

expected_name, expected_namespace, expected_type = sys.argv[1:4]
text = sys.stdin.read()
count_match = re.search(r"^Publisher count:\s*(\d+)\s*$", text, re.MULTILINE)
type_match = re.search(r"^Type:\s*(.*?)\s*$", text, re.MULTILINE)
if type_match is None or type_match.group(1) != expected_type:
    actual_type = None if type_match is None else type_match.group(1)
    raise SystemExit(f"top-level topic type is {actual_type!r}, expected {expected_type!r}")
if count_match is None:
    raise SystemExit("Publisher count is missing")
if int(count_match.group(1)) != 1:
    raise SystemExit(f"expected exactly one publisher, found {count_match.group(1)}")

publishers = []
for block in re.split(r"(?=^\s*Node name:\s*)", text, flags=re.MULTILINE):
    if not re.search(r"^\s*Endpoint type:\s*PUBLISHER\s*$", block, re.MULTILINE):
        continue
    fields = {}
    for key in ("Node name", "Node namespace", "Topic type", "GID"):
        match = re.search(rf"^\s*{re.escape(key)}:\s*(.*?)\s*$", block, re.MULTILINE)
        if match:
            fields[key] = match.group(1)
    publishers.append(fields)
if len(publishers) != 1:
    raise SystemExit(f"expected one verbose publisher record, found {len(publishers)}")
publisher = publishers[0]
required = ("Node name", "Node namespace", "Topic type", "GID")
missing = [key for key in required if not publisher.get(key)]
if missing:
    raise SystemExit(f"publisher record lacks {missing!r}")
node_name = publisher["Node name"]
node_namespace = publisher["Node namespace"]
topic_type = publisher["Topic type"]
publisher_gid = publisher["GID"]
if node_name != expected_name:
    raise SystemExit(
        f"publisher node is {node_name!r}, expected {expected_name!r}"
    )
if node_namespace != expected_namespace:
    raise SystemExit(
        f"publisher namespace is {node_namespace!r}, "
        f"expected {expected_namespace!r}"
    )
if topic_type != expected_type:
    raise SystemExit(
        f"publisher type is {topic_type!r}, expected {expected_type!r}"
    )
print(publisher_gid)
' "${vils_expected_receiver_node_name}" "${vils_expected_receiver_node_namespace}" \
    "${expected_type}" <<<"${topic_info}" 2>&1) || {
      printf '%s publisher provenance failed: %s' "${topic_name}" "${parsed}"
      return 1
    }
  printf '%s\n' "${parsed}"
}

capture_publisher_baseline()
{
  local topic_name
  local expected_type
  local publisher_gid
  local -A candidate_gids=()

  for topic_name in /from_can_bus /vehicle/status/velocity_status /vehicle/status/steering_status \
    /vehicle/status/gear_status /vehicle/status/control_mode; do
    expected_type=$(expected_topic_type "${topic_name}") || return 1
    publisher_gid=$(topic_publisher_record "${topic_name}" "${expected_type}") || return 1
    candidate_gids["${topic_name}"]=${publisher_gid}
    check_fast_safety_invariants || return 1
  done
  for topic_name in "${!candidate_gids[@]}"; do
    vils_expected_publisher_gid["${topic_name}"]=${candidate_gids["${topic_name}"]}
  done
}

publisher_topic_guard_reason()
{
  local topic_name="$1"
  local expected_type
  local publisher_gid

  [[ -n ${vils_expected_publisher_gid[${topic_name}]:-} ]] || {
    printf 'publisher baseline is missing for %s' "${topic_name}"
    return 1
  }
  expected_type=$(expected_topic_type "${topic_name}") || return 1
  publisher_gid=$(topic_publisher_record "${topic_name}" "${expected_type}") || {
    printf '%s' "${publisher_gid}"
    return 1
  }
  check_fast_safety_invariants || return 1
  [[ ${publisher_gid} == "${vils_expected_publisher_gid[${topic_name}]}" ]] || {
    printf '%s publisher GID changed from %s to %s' "${topic_name}" \
      "${vils_expected_publisher_gid[${topic_name}]}" "${publisher_gid}"
    return 1
  }
  return 0
}

publisher_guard_reason()
{
  local topic_name
  local guard_reason

  for topic_name in /from_can_bus /vehicle/status/velocity_status /vehicle/status/steering_status \
    /vehicle/status/gear_status /vehicle/status/control_mode; do
    guard_reason=$(publisher_topic_guard_reason "${topic_name}") || {
      printf '%s' "${guard_reason}"
      return 1
    }
  done
  return 0
}

write_publisher_provenance()
{
  local topic_name
  local expected_type

  : >"${vils_run_dir}/publisher_provenance.txt"
  for topic_name in /from_can_bus /vehicle/status/velocity_status /vehicle/status/steering_status \
    /vehicle/status/gear_status /vehicle/status/control_mode; do
    expected_type=$(expected_topic_type "${topic_name}") || return 1
    printf '%s\ttype=%s\tnode=%s\tnamespace=%s\tgid=%s\n' \
      "${topic_name}" "${expected_type}" "${vils_expected_receiver_node_name}" \
      "${vils_expected_receiver_node_namespace}" \
      "${vils_expected_publisher_gid[${topic_name}]:-MISSING}" \
      >>"${vils_run_dir}/publisher_provenance.txt"
  done
}

tx_endpoint_guard_reason()
{
  local topic_name
  local endpoint_total

  for topic_name in /to_can_bus /to_can_bus_fd; do
    endpoint_total=$(topic_endpoint_total "${topic_name}") || {
      printf '%s' "${endpoint_total}"
      return 1
    }
    (( endpoint_total == 0 )) || {
      printf '%s has %d publisher/subscriber endpoints' "${topic_name}" "${endpoint_total}"
      return 1
    }
    if [[ ${vils_launch_group_ready} == true ]]; then
      check_fast_safety_invariants || return 1
    fi
  done
  return 0
}

stable_tx_endpoint_guard_reason()
{
  local observation
  local guard_reason

  # A fresh DirectNode has a deliberately short discovery window.  Require three independent clean
  # observations before launch so a single late-discovery window cannot be accepted as stable zero.
  for observation in 1 2 3; do
    guard_reason=$(tx_endpoint_guard_reason) || {
      printf 'CAN TX endpoint observation %d/3 failed: %s' "${observation}" "${guard_reason}"
      return 1
    }
    if [[ ${vils_launch_group_ready} == true ]]; then
      check_fast_safety_invariants || return 1
    else
      guard_reason=$(prelaunch_fast_guard_reason) || {
        printf 'pre-launch safety state changed during endpoint discovery: %s' "${guard_reason}"
        return 1
      }
    fi
    (( observation == 3 )) || sleep 0.5
  done
  return 0
}

forbidden_processes()
{
  local command_file
  local process_id
  local -a command_argv
  local command_text
  local argument
  local argument_base
  local unsafe

  for command_file in /proc/[0-9]*/cmdline; do
    process_id=${command_file#/proc/}
    process_id=${process_id%/cmdline}
    [[ ${process_id} == "$$" ]] && continue
    command_argv=()
    mapfile -d '' -t command_argv <"${command_file}" 2>/dev/null || continue
    (( ${#command_argv[@]} > 0 )) || continue
    unsafe=false
    for argument in "${command_argv[@]}"; do
      argument_base=${argument##*/}
      case "${argument_base}" in
        socket_can_sender_node_exe|socket_can_sender.launch.py|twistController2VCU2EPS2ACC_node|twistController2vcu_node|twistController2EPS2ACC_node|cansend|cangen|canplayer|canfdtest|isotpsend)
          unsafe=true
          break
          ;;
        can_brdige.launch.xml|socket_can_bridge.launch.xml)
          unsafe=true
          break
          ;;
      esac
      if [[ ${argument} == /home/a/scripts/start.sh ]]; then
        unsafe=true
        break
      fi
    done
    if [[ ${unsafe} == true ]]; then
      command_text="${command_argv[*]}"
      printf '%s:%s\n' "${process_id}" "${command_text}"
    fi
  done
}

receiver_process_records()
{
  local command_file
  local process_id
  local -a command_argv

  for command_file in /proc/[0-9]*/cmdline; do
    process_id=${command_file#/proc/}
    process_id=${process_id%/cmdline}
    command_argv=()
    mapfile -d '' -t command_argv <"${command_file}" 2>/dev/null || continue
    (( ${#command_argv[@]} > 0 )) || continue
    if [[ ${command_argv[0]##*/} == socket_can_receiver_node_exe ]]; then
      printf '%s\n' "${process_id}"
    fi
  done
}

receiver_mapped_library_guard_reason()
{
  local process_id="$1"
  local mapped_path
  local mapped_component=false
  local mapped_core=false

  [[ -r /proc/${process_id}/maps ]] || {
    printf 'cannot inspect mapped libraries for receiver PID %s' "${process_id}"
    return 1
  }
  while IFS= read -r mapped_path; do
    case "${mapped_path##*/}" in
      libsocket_can_receiver_node.so)
        [[ ${mapped_path} == "${vils_expected_receiver_component}" ]] || {
          printf 'receiver mapped unapproved component library %s' "${mapped_path}"
          return 1
        }
        mapped_component=true
        ;;
      libros2_socketcan.so)
        [[ ${mapped_path} == "${vils_expected_receiver_core}" ]] || {
          printf 'receiver mapped unapproved core library %s' "${mapped_path}"
          return 1
        }
        mapped_core=true
        ;;
    esac
  done < <(awk '$NF ~ /^\// {print $NF}' "/proc/${process_id}/maps" | sort -u)
  [[ ${mapped_component} == true && ${mapped_core} == true ]] || {
    printf 'receiver PID %s lacks one or both reviewed SocketCAN libraries' "${process_id}"
    return 1
  }
  return 0
}

receiver_guard_reason()
{
  local -a receiver_pids
  local receiver_pid
  local receiver_exe
  local receiver_pgid
  local receiver_environment
  local lifecycle_state
  local parameter_value

  mapfile -t receiver_pids < <(receiver_process_records)
  (( ${#receiver_pids[@]} == 1 )) || {
    printf 'expected exactly one SocketCAN receiver, found %d: %s' \
      "${#receiver_pids[@]}" "${receiver_pids[*]:-none}"
    return 1
  }
  receiver_pid=${receiver_pids[0]}
  if [[ -n ${vils_receiver_pid} ]]; then
    [[ ${receiver_pid} == "${vils_receiver_pid}" ]] || {
      printf 'approved receiver PID changed from %s to %s' "${vils_receiver_pid}" "${receiver_pid}"
      return 1
    }
    [[ $(receiver_identity_state) == alive ]] || {
      printf 'approved receiver PID %s no longer has its captured process identity' "${receiver_pid}"
      return 1
    }
  fi
  receiver_exe=$(readlink "/proc/${receiver_pid}/exe" 2>&1) || {
    printf 'cannot resolve receiver executable for PID %s: %s' "${receiver_pid}" "${receiver_exe}"
    return 1
  }
  [[ ${receiver_exe} == "${vils_expected_receiver_exe}" ]] || {
    printf 'receiver PID %s executable is unapproved: %s' "${receiver_pid}" "${receiver_exe}"
    return 1
  }
  receiver_pgid=$(timeout 2 ps -o pgid= -p "${receiver_pid}" | tr -d '[:space:]') || {
    printf 'cannot resolve receiver process group for PID %s' "${receiver_pid}"
    return 1
  }
  [[ ${receiver_pgid} == "${vils_launch_pid}" ]] || {
    printf 'receiver PID %s is outside approved launch group %s: PGID=%s' \
      "${receiver_pid}" "${vils_launch_pid}" "${receiver_pgid}"
    return 1
  }
  receiver_environment=$(tr '\0' '\n' <"/proc/${receiver_pid}/environ" 2>&1) || {
    printf 'cannot inspect receiver environment for PID %s: %s' \
      "${receiver_pid}" "${receiver_environment}"
    return 1
  }
  for parameter_value in \
    "ROS_DOMAIN_ID=10" \
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    "ROS_LOCALHOST_ONLY=0" \
    "CYCLONEDDS_URI=${CYCLONEDDS_URI}"; do
    grep -Fxq "${parameter_value}" <<<"${receiver_environment}" || {
      printf 'receiver PID %s lacks approved environment entry %s' \
        "${receiver_pid}" "${parameter_value%%=*}"
      return 1
    }
  done
  receiver_mapped_library_guard_reason "${receiver_pid}" || return 1
  lifecycle_state=$(timeout 4 ros2 lifecycle get --no-daemon --spin-time 0.5 \
    "/${vils_expected_receiver_node_name}" 2>&1) || {
      printf 'cannot inspect receiver lifecycle state: %s' "${lifecycle_state}"
      return 1
    }
  [[ ${lifecycle_state} == 'active [3]' ]] || {
    printf 'receiver lifecycle state is %s, not active [3]' "${lifecycle_state}"
    return 1
  }
  check_fast_safety_invariants || return 1
  for parameter_value in \
    'interface|can0' \
    'enable_can_fd|False' \
    'use_bus_time|False' \
    'filters|0:0'; do
    local parameter_name=${parameter_value%%|*}
    local expected_value=${parameter_value#*|}
    local observed_value
    observed_value=$(timeout 4 ros2 param get --no-daemon --spin-time 0.5 --hide-type \
      "/${vils_expected_receiver_node_name}" "${parameter_name}" 2>&1) || {
        printf 'cannot inspect receiver parameter %s: %s' "${parameter_name}" "${observed_value}"
        return 1
      }
    [[ ${observed_value} == "${expected_value}" ]] || {
      printf 'receiver parameter %s is %s, expected %s' \
        "${parameter_name}" "${observed_value}" "${expected_value}"
      return 1
    }
    check_fast_safety_invariants || return 1
  done
  [[ $(receiver_identity_state) == alive ]] || {
    printf 'approved receiver identity changed while its lifecycle and parameters were inspected'
    return 1
  }
  receiver_exe=$(readlink "/proc/${receiver_pid}/exe" 2>&1) || {
    printf 'cannot re-resolve receiver executable for PID %s: %s' \
      "${receiver_pid}" "${receiver_exe}"
    return 1
  }
  [[ ${receiver_exe} == "${vils_expected_receiver_exe}" ]] || {
    printf 'receiver executable changed during inspection: %s' "${receiver_exe}"
    return 1
  }
  receiver_mapped_library_guard_reason "${receiver_pid}" || return 1
  return 0
}

capture_receiver_identity()
{
  local -a receiver_pids
  local candidate_pid
  local candidate_starttime

  mapfile -t receiver_pids < <(receiver_process_records)
  (( ${#receiver_pids[@]} == 1 )) || return 1
  candidate_pid=${receiver_pids[0]}
  candidate_starttime=$(process_starttime "${candidate_pid}") || return 1
  [[ $(process_state_for_identity "${candidate_pid}" "${candidate_starttime}" \
    "${vils_launch_pid}" "${vils_launch_pid}") == alive ]] || return 1
  vils_receiver_pid=${candidate_pid}
  vils_receiver_starttime=${candidate_starttime}
}

receiver_fast_guard_reason()
{
  local -a receiver_pids
  local receiver_exe

  [[ -n ${vils_receiver_pid} && -n ${vils_receiver_starttime} ]] || {
    printf 'approved receiver identity has not been captured'
    return 1
  }
  mapfile -t receiver_pids < <(receiver_process_records)
  (( ${#receiver_pids[@]} == 1 )) || {
    printf 'expected exactly one SocketCAN receiver, found %d: %s' \
      "${#receiver_pids[@]}" "${receiver_pids[*]:-none}"
    return 1
  }
  [[ ${receiver_pids[0]} == "${vils_receiver_pid}" && $(receiver_identity_state) == alive ]] || {
    printf 'approved receiver process identity is no longer unique and alive'
    return 1
  }
  receiver_exe=$(readlink "/proc/${vils_receiver_pid}/exe" 2>&1) || {
    printf 'cannot resolve approved receiver executable: %s' "${receiver_exe}"
    return 1
  }
  [[ ${receiver_exe} == "${vils_expected_receiver_exe}" ]] || {
    printf 'approved receiver executable changed to %s' "${receiver_exe}"
    return 1
  }
  receiver_mapped_library_guard_reason "${vils_receiver_pid}" || return 1
  return 0
}

prelaunch_fast_guard_reason()
{
  local conflicts
  local guard_reason
  local receiver_processes

  guard_reason=$(can_fleet_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  conflicts=$(forbidden_processes)
  [[ -z ${conflicts} ]] || {
    printf 'actuator-capable process appeared: %s' "${conflicts//$'\n'/; }"
    return 1
  }
  receiver_processes=$(receiver_process_records)
  [[ -z ${receiver_processes} ]] || {
    printf 'SocketCAN receiver appeared before approved launch: %s' \
      "${receiver_processes//$'\n'/; }"
    return 1
  }
  return 0
}

final_prelaunch_guard_reason()
{
  local guard_reason

  # Repeat mutable provenance, network, clock, endpoint, and physical guards after the multi-window
  # discovery gate.  The final hash/physical checks are intentionally adjacent to process creation.
  guard_reason=$(launch_provenance_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(network_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(time_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(tx_endpoint_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(network_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(time_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(tx_endpoint_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(prelaunch_fast_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(launch_provenance_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  return 0
}

preflight()
{
  local conflicts
  local receiver_processes
  local guard_reason

  capture_can_baseline
  guard_reason=$(launch_provenance_guard_reason) || fail "${guard_reason}"
  guard_reason=$(network_guard_reason) || fail "${guard_reason}"
  guard_reason=$(time_guard_reason) || fail "${guard_reason}"
  conflicts=$(forbidden_processes)
  [[ -z ${conflicts} ]] || fail "actuator-capable process detected: ${conflicts//$'\n'/; }"
  receiver_processes=$(receiver_process_records)
  [[ -z ${receiver_processes} ]] ||
    fail "another SocketCAN receiver is already running: ${receiver_processes//$'\n'/; }"
  guard_reason=$(stable_tx_endpoint_guard_reason) || fail "${guard_reason}"
  guard_reason=$(final_prelaunch_guard_reason) || fail "${guard_reason}"
}

prelaunch_guard_reason()
{
  local conflicts
  local guard_reason
  local receiver_processes

  guard_reason=$(launch_provenance_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(can_fleet_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(network_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(time_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  conflicts=$(forbidden_processes)
  [[ -z ${conflicts} ]] || {
    printf 'actuator-capable process appeared before launch: %s' "${conflicts//$'\n'/; }"
    return 1
  }
  receiver_processes=$(receiver_process_records)
  [[ -z ${receiver_processes} ]] || {
    printf 'another SocketCAN receiver appeared before launch: %s' \
      "${receiver_processes//$'\n'/; }"
    return 1
  }
  guard_reason=$(stable_tx_endpoint_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  guard_reason=$(final_prelaunch_guard_reason) || {
    printf '%s' "${guard_reason}"
    return 1
  }
  return 0
}

record_abort()
{
  local reason="$*"
  [[ -n ${vils_abort_reason_file} ]] || return 0
  printf '%s\n' "${reason}" >>"${vils_abort_reason_file}"
}

abort_is_latched()
{
  [[ -n ${vils_abort_reason_file} && -s ${vils_abort_reason_file} ]]
}

snapshot_checkpoint()
{
  local snapshot_name="$1"
  local checkpoint_label="$2"
  local error_file="$3"

  [[ ${snapshot_name} == during ]] || return 0
  if ! check_fast_safety_invariants; then
    printf 'safety invariant failed after %s\n' "${checkpoint_label}" >>"${error_file}"
    return 1
  fi
  if ! readiness_deadline_guard_reason >/dev/null; then
    record_abort "required during-snapshot evidence crossed the duration deadline"
    printf 'duration deadline crossed after %s\n' "${checkpoint_label}" >>"${error_file}"
    return 1
  fi
  return 0
}

snapshot_state()
{
  local snapshot_name="$1"
  local snapshot_dir="${vils_run_dir}/${snapshot_name}"
  local error_file="${snapshot_dir}/snapshot_errors.txt"
  local failed=0
  local interface
  local topic_name
  local topic_file

  mkdir -p "${snapshot_dir}"
  : >"${error_file}"
  date --utc --iso-8601=ns >"${snapshot_dir}/utc.txt" 2>&1 || {
    printf 'date failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" date "${error_file}" || return 1
  for interface in "${vils_primary_can}" "${vils_inactive_can_interfaces[@]}"; do
    timeout 3 ip -details -statistics -json link show dev "${interface}" \
      >"${snapshot_dir}/${interface}_link.json" 2>&1 || {
        printf '%s link snapshot failed\n' "${interface}" >>"${error_file}"
        failed=1
      }
    can_tx_packets "${interface}" >"${snapshot_dir}/${interface}_tx_packets.txt" 2>&1 || {
      printf '%s TX counter snapshot failed\n' "${interface}" >>"${error_file}"
        failed=1
      }
    snapshot_checkpoint "${snapshot_name}" "${interface} state" "${error_file}" || return 1
  done
  timeout 3 ps -eo pid,ppid,pgid,lstart,args --sort=pid >"${snapshot_dir}/processes.txt" 2>&1 || {
    printf 'process snapshot failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" processes "${error_file}" || return 1
  timeout 5 ros2 node list --no-daemon --spin-time 0.5 >"${snapshot_dir}/ros_nodes.txt" 2>&1 || {
    printf 'ROS node graph snapshot failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" 'ROS node graph' "${error_file}" || return 1
  timeout 5 ros2 topic info --no-daemon --spin-time 0.5 -v /to_can_bus \
    >"${snapshot_dir}/to_can_bus.txt" 2>&1 || true
  timeout 5 ros2 topic info --no-daemon --spin-time 0.5 -v /to_can_bus_fd \
    >"${snapshot_dir}/to_can_bus_fd.txt" 2>&1 || true
  snapshot_checkpoint "${snapshot_name}" 'CAN TX topics' "${error_file}" || return 1
  for topic_name in /from_can_bus /vehicle/status/velocity_status /vehicle/status/steering_status \
    /vehicle/status/gear_status /vehicle/status/control_mode; do
    topic_file=${topic_name#/}
    topic_file=${topic_file//\//_}
    timeout 5 ros2 topic info --no-daemon --spin-time 0.5 -v "${topic_name}" \
      >"${snapshot_dir}/${topic_file}.txt" 2>&1 || {
        if [[ ${snapshot_name} == during ]]; then
          printf '%s metadata snapshot failed\n' "${topic_name}" >>"${error_file}"
          failed=1
        fi
      }
    snapshot_checkpoint "${snapshot_name}" "${topic_name} metadata" "${error_file}" || return 1
  done
  timeout 3 ip -brief address >"${snapshot_dir}/ip_address.txt" 2>&1 || {
    printf 'IP address snapshot failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" 'IP address' "${error_file}" || return 1
  timeout 3 ip route show table all >"${snapshot_dir}/ip_route.txt" 2>&1 || {
    printf 'IP route snapshot failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" routes "${error_file}" || return 1
  timeout 3 chronyc tracking >"${snapshot_dir}/chrony_tracking.txt" 2>&1 || {
    printf 'Chrony tracking snapshot failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" 'Chrony tracking' "${error_file}" || return 1
  timeout 3 chronyc sources -n -v >"${snapshot_dir}/chrony_sources.txt" 2>&1 || {
    printf 'Chrony sources snapshot failed\n' >>"${error_file}"
    failed=1
  }
  snapshot_checkpoint "${snapshot_name}" 'Chrony sources' "${error_file}" || return 1
  if [[ ${snapshot_name} == during ]]; then
    check_invariants || {
      printf 'full invariant check failed after snapshot collection\n' >>"${error_file}"
      return 1
    }
  fi
  return "${failed}"
}

capture_required_samples()
{
  local topic_name
  local topic_file
  local guard_reason

  check_invariants || return 1
  for topic_name in /from_can_bus /vehicle/status/velocity_status /vehicle/status/steering_status \
    /vehicle/status/gear_status /vehicle/status/control_mode; do
    check_fast_safety_invariants || return 1
    guard_reason=$(publisher_topic_guard_reason "${topic_name}") || {
      record_abort "required sample pre-check failed: ${guard_reason}"
      return 1
    }
    topic_file=${topic_name#/}
    topic_file=${topic_file//\//_}
    timeout 5 ros2 topic echo --no-daemon --spin-time 0.5 "${topic_name}" --once \
      >"${vils_run_dir}/required_sample_${topic_file}.yaml" 2>&1 || return 1
    guard_reason=$(publisher_topic_guard_reason "${topic_name}") || {
      record_abort "required sample post-check failed: ${guard_reason}"
      return 1
    }
    check_fast_safety_invariants || return 1
  done
  check_invariants || return 1
}

validate_evidence()
{
  local interface
  local snapshot_name
  local topic_name
  local topic_file
  local required_file

  for required_file in \
    git_status.txt git_commit.txt approved_environment.txt ros2_socketcan_prefix.txt \
    receiver_binary.sha256 receiver_component_library.sha256 receiver_core_library.sha256 \
    rx_only_launch.sha256 receiver_child_launch.sha256 \
    rx_only_runner.sha256 cyclonedds_profile.sha256 publisher_provenance.txt \
    receiver_launch.log receiver_launch_exit_status.txt; do
    [[ -f "${vils_run_dir}/${required_file}" ]] ||
      record_abort "required evidence file is missing: ${required_file}"
  done
  for snapshot_name in before during after; do
    [[ -d "${vils_run_dir}/${snapshot_name}" ]] || {
      record_abort "required evidence snapshot is missing: ${snapshot_name}"
      continue
    }
    [[ -f "${vils_run_dir}/${snapshot_name}/snapshot_errors.txt" ]] ||
      record_abort "snapshot error manifest is missing: ${snapshot_name}"
    if [[ -s "${vils_run_dir}/${snapshot_name}/snapshot_errors.txt" ]]; then
      record_abort "${snapshot_name} snapshot command failure: $(tr '\n' ';' <"${vils_run_dir}/${snapshot_name}/snapshot_errors.txt")"
    fi
    for required_file in utc.txt processes.txt ros_nodes.txt ip_address.txt ip_route.txt \
      chrony_tracking.txt chrony_sources.txt; do
      [[ -f "${vils_run_dir}/${snapshot_name}/${required_file}" ]] ||
        record_abort "required ${snapshot_name} evidence file is missing: ${required_file}"
    done
    for interface in "${vils_primary_can}" "${vils_inactive_can_interfaces[@]}"; do
      [[ -s "${vils_run_dir}/${snapshot_name}/${interface}_link.json" ]] ||
        record_abort "required ${snapshot_name} CAN evidence is missing: ${interface}_link.json"
      [[ -s "${vils_run_dir}/${snapshot_name}/${interface}_tx_packets.txt" ]] ||
        record_abort "required ${snapshot_name} CAN evidence is missing: ${interface}_tx_packets.txt"
    done
  done
  for topic_name in /from_can_bus /vehicle/status/velocity_status /vehicle/status/steering_status \
    /vehicle/status/gear_status /vehicle/status/control_mode; do
    topic_file=${topic_name#/}
    topic_file=${topic_file//\//_}
    [[ -s "${vils_run_dir}/required_sample_${topic_file}.yaml" ]] ||
      record_abort "required live sample is missing or empty: ${topic_name}"
  done
}

process_group_state()
{
  local process_group_id="$1"
  local process_table

  [[ ${process_group_id} =~ ^[1-9][0-9]*$ ]] || {
    printf 'unknown\n'
    return 0
  }
  process_table=$(timeout 2 ps -eo pgid=,stat= 2>/dev/null) || {
    printf 'unknown\n'
    return 0
  }
  awk -v expected_pgid="${process_group_id}" '
    $1 == expected_pgid && $2 !~ /^Z/ { alive = 1 }
    END { print(alive ? "alive" : "dead") }
  ' <<<"${process_table}"
}

dedicated_session_group_state()
{
  local process_group_id="$1"
  local process_table

  [[ ${process_group_id} =~ ^[1-9][0-9]*$ ]] || {
    printf 'unknown\n'
    return 0
  }
  process_table=$(timeout 2 ps -eo pgid=,sid=,stat= 2>/dev/null) || {
    printf 'unknown\n'
    return 0
  }
  awk -v expected_id="${process_group_id}" '
    $1 == expected_id && $3 !~ /^Z/ {
      alive = 1
      if ($2 != expected_id) foreign = 1
    }
    END {
      if (foreign) print "foreign"
      else if (alive) print "alive"
      else print "dead"
    }
  ' <<<"${process_table}"
}

process_group_alive()
{
  [[ $(process_group_state "$1") == alive ]]
}

launch_leader_state()
{
  [[ -n ${vils_launch_pid} && -n ${vils_launch_starttime} ]] || {
    printf 'unregistered\n'
    return 0
  }
  process_state_for_identity "${vils_launch_pid}" "${vils_launch_starttime}" \
    "${vils_launch_pid}" "${vils_launch_pid}"
}

timer_leader_state()
{
  [[ -n ${vils_timer_pid} && -n ${vils_timer_starttime} ]] || {
    printf 'unregistered\n'
    return 0
  }
  process_state_for_identity "${vils_timer_pid}" "${vils_timer_starttime}" \
    "${vils_timer_pid}" "${vils_timer_pid}"
}

receiver_identity_state()
{
  [[ -n ${vils_receiver_pid} && -n ${vils_receiver_starttime} ]] || {
    printf 'unregistered\n'
    return 0
  }
  process_state_for_identity "${vils_receiver_pid}" "${vils_receiver_starttime}" \
    "${vils_launch_pid}" "${vils_launch_pid}"
}

launch_group_alive()
{
  [[ $(launch_leader_state) == alive && $(process_group_state "${vils_launch_pid}") == alive ]]
}

wait_for_launch_group()
{
  local group_deadline=$((SECONDS + 3))

  while (( SECONDS < group_deadline )); do
    launch_group_alive && return 0
    kill -0 "${vils_launch_pid}" 2>/dev/null || {
      record_abort "receiver-only launch process exited before creating its dedicated process group"
      return 1
    }
    sleep 0.05
  done
  record_abort "receiver-only launch did not create its dedicated process group within 3 seconds"
  return 1
}

capture_direct_child_starttime()
{
  local process_id="$1"
  local stat_record
  local fields_record
  local -a stat_fields

  [[ -r /proc/${process_id}/stat ]] || return 1
  stat_record=$(</proc/"${process_id}"/stat) || return 1
  fields_record=${stat_record##*) }
  read -r -a stat_fields <<<"${fields_record}"
  (( ${#stat_fields[@]} > 19 )) || return 1
  [[ ${stat_fields[1]} == "${vils_parent_pid}" && ${stat_fields[19]} =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${stat_fields[19]}"
}

wait_for_suspended_child_identity()
{
  local process_id="$1"
  local handshake_deadline=$((SECONDS + 2))
  local stat_record
  local fields_record
  local -a stat_fields

  while (( SECONDS < handshake_deadline )); do
    if [[ ! -e /proc/${process_id} ]]; then
      return 1
    fi
    stat_record=$(</proc/"${process_id}"/stat) || {
      sleep 0.02
      continue
    }
    fields_record=${stat_record##*) }
    read -r -a stat_fields <<<"${fields_record}"
    if (( ${#stat_fields[@]} > 19 )) && \
      [[ ${stat_fields[0]} =~ ^[Tt]$ && ${stat_fields[1]} == "${vils_parent_pid}" && \
        ${stat_fields[2]} == "${process_id}" && ${stat_fields[3]} == "${process_id}" && \
        ${stat_fields[19]} =~ ^[0-9]+$ ]]; then
      printf '%s\n' "${stat_fields[19]}"
      return 0
    fi
    sleep 0.02
  done
  return 1
}

stop_owned_process_group()
{
  local process_id="$1"
  local expected_starttime="$2"
  local label="$3"
  shift 3
  local signal_name
  local attempt
  local identity_state
  local bare_identity_state
  local group_state
  local session_group_state

  [[ -n ${process_id} && -n ${expected_starttime} ]] || return 0
  for signal_name in "$@"; do
    identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}" \
      "${process_id}" "${process_id}")
    bare_identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}")
    session_group_state=$(dedicated_session_group_state "${process_id}")
    if [[ ${identity_state} == gone && ${session_group_state} == alive ]]; then
      # The setsid leader may have exited before its descendants.  Its numeric SID/PGID remains
      # reserved while those descendants exist, so this is still the captured dedicated session.
      kill -"${signal_name}" -- "-${process_id}" 2>/dev/null || true
    elif [[ ${identity_state} == gone || ${identity_state} == mismatch ]]; then
      break
    elif [[ ${identity_state} == alive || ${identity_state} == zombie ]]; then
      kill -"${signal_name}" "${process_id}" 2>/dev/null || true
      kill -"${signal_name}" -- "-${process_id}" 2>/dev/null || true
    elif [[ ${identity_state} == wrong-group && ${bare_identity_state} == alive ]]; then
      record_abort "${label} PID ${process_id} left its captured process group; signalling PID only"
      kill -"${signal_name}" "${process_id}" 2>/dev/null || true
    elif [[ ${identity_state} == unknown ]]; then
      record_abort "${label} PID ${process_id} identity is unreadable; it is not considered stopped"
    else
      record_abort "${label} PID ${process_id} was reused; refusing to signal an unowned process group"
      return 1
    fi
    for attempt in {1..20}; do
      identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}" \
        "${process_id}" "${process_id}")
      if [[ ${identity_state} == gone || ${identity_state} == zombie || \
        ${identity_state} == mismatch ]]; then
        break
      fi
      sleep 0.1
    done
  done

  identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}" \
    "${process_id}" "${process_id}")
  group_state=$(process_group_state "${process_id}")
  if [[ ${identity_state} == alive || ${identity_state} == unknown || \
    ${identity_state} == wrong-group || ${group_state} == alive || ${group_state} == unknown ]]; then
    record_abort "${label} PID/PGID ${process_id} could not be proven stopped after bounded signals (pid=${identity_state}, group=${group_state})"
    return 1
  fi
  return 0
}

reap_direct_child_if_terminal()
{
  local process_id="$1"
  local expected_starttime="$2"
  local label="$3"
  local identity_state

  if [[ -n ${expected_starttime} ]]; then
    identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}")
  elif [[ ! -e /proc/${process_id} ]]; then
    identity_state=gone
  else
    identity_state=unknown
  fi
  if [[ ${identity_state} == gone || ${identity_state} == zombie ]]; then
    set +e
    wait "${process_id}" 2>/dev/null
    vils_unregistered_child_status=$?
    set -e
    return 0
  fi
  vils_unregistered_child_status=124
  record_abort "${label} PID ${process_id} was not reaped because its identity state is ${identity_state}"
  return 1
}

direct_child_job_is_active()
{
  local process_id="$1"
  local job_pid

  # `jobs -p` is the shell's own child registry.  Unlike a bare kill(0) probe, an exact match here
  # cannot be satisfied by an unrelated process that later reused the numeric PID.
  while IFS= read -r job_pid; do
    [[ ${job_pid} == "${process_id}" ]] && return 0
  done < <(jobs -p)
  return 1
}

stop_unregistered_direct_child()
{
  local process_id="$1"
  local label="$2"
  local signal_name
  local attempt

  [[ ${process_id} =~ ^[1-9][0-9]*$ ]] || return 0
  if ! direct_child_job_is_active "${process_id}"; then
    if [[ ! -e /proc/${process_id} ]]; then
      set +e
      wait "${process_id}" 2>/dev/null
      vils_unregistered_child_status=$?
      set -e
      return 0
    fi
    record_abort "${label} PID ${process_id} is not an active direct child; refusing an unowned signal"
    vils_unregistered_child_status=124
    return 1
  fi

  # This path is used only before the suspended child is resumed.  Signal both the dedicated
  # session/process group and the direct child, then CONT so a pending TERM is deliverable.
  for signal_name in TERM KILL; do
    kill -"${signal_name}" -- "-${process_id}" 2>/dev/null || true
    kill -"${signal_name}" "${process_id}" 2>/dev/null || true
    kill -CONT -- "-${process_id}" 2>/dev/null || true
    kill -CONT "${process_id}" 2>/dev/null || true
    for attempt in {1..20}; do
      if ! direct_child_job_is_active "${process_id}"; then
        set +e
        wait "${process_id}" 2>/dev/null
        vils_unregistered_child_status=$?
        set -e
        return 0
      fi
      sleep 0.1
    done
  done

  vils_unregistered_child_status=124
  record_abort "${label} direct child PID ${process_id} did not terminate after bounded signals"
  return 1
}

signal_owned_launch()
{
  local signal_name="$1"
  local launch_group_state
  local receiver_group_state
  local launch_bare_state
  local receiver_bare_state
  local session_group_state

  launch_group_state=$(launch_leader_state)
  receiver_group_state=$(receiver_identity_state)
  launch_bare_state=$(process_state_for_identity "${vils_launch_pid}" "${vils_launch_starttime}")
  session_group_state=$(dedicated_session_group_state "${vils_launch_pid}")
  if [[ -n ${vils_receiver_pid} && -n ${vils_receiver_starttime} ]]; then
    receiver_bare_state=$(process_state_for_identity \
      "${vils_receiver_pid}" "${vils_receiver_starttime}")
  else
    receiver_bare_state=unregistered
  fi

  if [[ ${launch_group_state} == alive || ${launch_group_state} == zombie || \
    ${receiver_group_state} == alive || ${receiver_group_state} == zombie || \
    (${launch_bare_state} == gone && ${session_group_state} == alive) ]]; then
    kill -"${signal_name}" -- "-${vils_launch_pid}" 2>/dev/null || true
  else
    if [[ ${session_group_state} == foreign || ${session_group_state} == unknown ]]; then
      record_abort "launch session ownership is ${session_group_state}; refusing an unverified group signal"
    fi
    [[ ${launch_bare_state} == alive ]] &&
      kill -"${signal_name}" "${vils_launch_pid}" 2>/dev/null || true
    [[ ${receiver_bare_state} == alive ]] &&
      kill -"${signal_name}" "${vils_receiver_pid}" 2>/dev/null || true
  fi
}

stop_owned_process()
{
  local process_id="$1"
  local expected_starttime="$2"
  local label="$3"
  local signal_name
  local attempt
  local identity_state

  [[ -n ${process_id} && -n ${expected_starttime} ]] || return 0
  for signal_name in TERM KILL; do
    identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}")
    case "${identity_state}" in
      gone|zombie) return 0 ;;
      mismatch)
        record_abort "${label} PID ${process_id} was reused; refusing to signal it"
        return 1
        ;;
      unknown)
        record_abort "${label} PID ${process_id} identity is unreadable; refusing an unowned signal"
        return 1
        ;;
    esac
    kill -"${signal_name}" "${process_id}" 2>/dev/null || true
    for attempt in {1..20}; do
      identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}")
      [[ ${identity_state} == gone || ${identity_state} == zombie ]] && return 0
      [[ ${identity_state} == mismatch ]] && return 1
      sleep 0.1
    done
  done
  identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}")
  record_abort "${label} PID ${process_id} could not be proven stopped (pid=${identity_state})"
  return 1
}

stop_launch_group()
{
  local signal_name
  local attempt
  local launch_status
  local launch_identity_state
  local receiver_identity_status
  local group_state
  local recovered_starttime

  [[ -n ${vils_launch_pid} ]] || return 0
  if [[ -z ${vils_launch_starttime} ]]; then
    recovered_starttime=$(capture_direct_child_starttime "${vils_launch_pid}") || true
    if [[ -n ${recovered_starttime} ]]; then
      vils_launch_starttime=${recovered_starttime}
    else
      if stop_unregistered_direct_child "${vils_launch_pid}" \
        "receiver-only launch identity-recovery cleanup"; then
        printf '%s\n' "${vils_unregistered_child_status}" \
          >"${vils_run_dir}/receiver_launch_exit_status.txt"
        vils_launch_pid=""
        return 0
      fi
      printf '124\n' >"${vils_run_dir}/receiver_launch_exit_status.txt"
      return 1
    fi
  fi
  for signal_name in INT TERM KILL; do
    signal_owned_launch "${signal_name}"
    for attempt in {1..20}; do
      launch_identity_state=$(process_state_for_identity \
        "${vils_launch_pid}" "${vils_launch_starttime}")
      if [[ -n ${vils_receiver_pid} && -n ${vils_receiver_starttime} ]]; then
        receiver_identity_status=$(process_state_for_identity \
          "${vils_receiver_pid}" "${vils_receiver_starttime}")
      else
        receiver_identity_status=gone
      fi
      group_state=$(process_group_state "${vils_launch_pid}")
      if [[ ${launch_identity_state} =~ ^(gone|zombie|mismatch)$ && \
        ${receiver_identity_status} =~ ^(gone|zombie|mismatch)$ && ${group_state} == dead ]]; then
        break 2
      fi
      sleep 0.1
    done
  done
  stop_owned_process "${vils_receiver_pid}" "${vils_receiver_starttime}" \
    "approved receiver" || true
  group_state=$(process_group_state "${vils_launch_pid}")
  if [[ ${group_state} == alive || ${group_state} == unknown ]]; then
    record_abort "approved launch PGID ${vils_launch_pid} could not be proven stopped (group=${group_state})"
  fi
  launch_identity_state=$(process_state_for_identity "${vils_launch_pid}" "${vils_launch_starttime}")
  if [[ ${launch_identity_state} == gone || ${launch_identity_state} == zombie ]]; then
    set +e
    wait "${vils_launch_pid}" 2>/dev/null
    launch_status=$?
    set -e
  else
    launch_status=124
    record_abort "launch child was not reaped because its state is ${launch_identity_state}"
  fi
  printf '%s\n' "${launch_status}" >"${vils_run_dir}/receiver_launch_exit_status.txt"
}

stop_background_group()
{
  local process_id="$1"
  local expected_starttime="$2"
  local label="$3"
  local identity_state
  local recovered_starttime

  [[ -n ${process_id} ]] || return 0
  if [[ -z ${expected_starttime} ]]; then
    recovered_starttime=$(capture_direct_child_starttime "${process_id}") || true
    if [[ -n ${recovered_starttime} ]]; then
      expected_starttime=${recovered_starttime}
      vils_timer_starttime=${recovered_starttime}
    else
      if stop_unregistered_direct_child "${process_id}" "${label} identity-recovery cleanup"; then
        vils_timer_pid=""
        return 0
      fi
      return 1
    fi
  fi
  stop_owned_process_group "${process_id}" "${expected_starttime}" \
    "${label}" TERM KILL || true
  identity_state=$(process_state_for_identity "${process_id}" "${expected_starttime}")
  if [[ ${identity_state} == gone || ${identity_state} == zombie ]]; then
    wait "${process_id}" 2>/dev/null || true
  else
    record_abort "${label} child was not reaped because its state is ${identity_state}"
  fi
}

duration_watchdog_guard_reason()
{
  if (( vils_duration_seconds == 0 )); then
    return 0
  fi
  if (( SECONDS + 1 >= vils_deadline_seconds )); then
    return 0
  fi
  if [[ $(timer_leader_state) == alive ]]; then
    return 0
  fi
  printf 'independent duration watchdog exited before its deadline'
  return 1
}

readiness_deadline_guard_reason()
{
  if (( vils_deadline_seconds > 0 && SECONDS >= vils_deadline_seconds )); then
    printf 'required readiness evidence crossed the duration deadline'
    return 1
  fi
  return 0
}

check_fast_safety_invariants()
{
  local conflicts
  local guard_reason
  local launch_state

  if abort_is_latched; then
    return 1
  fi
  guard_reason=$(duration_watchdog_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  launch_state=$(launch_leader_state)
  [[ ${launch_state} == alive ]] || {
    record_abort "receiver-only launch identity is not healthy: ${launch_state}"
    return 1
  }
  guard_reason=$(can_fleet_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  conflicts=$(forbidden_processes)
  if [[ -n ${conflicts} ]]; then
    record_abort "forbidden process appeared: ${conflicts//$'\n'/; }"
    return 1
  fi
  if [[ -n ${vils_receiver_pid} ]]; then
    guard_reason=$(receiver_fast_guard_reason) || {
      record_abort "${guard_reason}"
      return 1
    }
  fi
  return 0
}

check_invariants()
{
  local conflicts
  local guard_reason

  check_fast_safety_invariants || return 1
  guard_reason=$(launch_provenance_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  guard_reason=$(can_fleet_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  guard_reason=$(network_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  guard_reason=$(time_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  conflicts=$(forbidden_processes)
  if [[ -n ${conflicts} ]]; then
    record_abort "forbidden process appeared: ${conflicts//$'\n'/; }"
    return 1
  fi
  guard_reason=$(receiver_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  guard_reason=$(publisher_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  guard_reason=$(tx_endpoint_guard_reason) || {
    record_abort "${guard_reason}"
    return 1
  }
  return 0
}

wait_for_receiver()
{
  local readiness_deadline=$((SECONDS + 10))
  local conflicts
  local guard_reason
  local last_receiver_reason="receiver process was not discovered"
  local -a receiver_pids

  while (( SECONDS < readiness_deadline )); do
    abort_is_latched && return 1
    guard_reason=$(duration_watchdog_guard_reason) || {
      record_abort "${guard_reason}"
      return 1
    }
    launch_group_alive || {
      record_abort "receiver-only launch group exited during startup"
      return 1
    }
    guard_reason=$(can_fleet_guard_reason) || {
      record_abort "${guard_reason}"
      return 1
    }
    guard_reason=$(tx_endpoint_guard_reason) || {
      record_abort "${guard_reason}"
      return 1
    }
    conflicts=$(forbidden_processes)
    if [[ -n ${conflicts} ]]; then
      record_abort "forbidden process appeared during startup: ${conflicts//$'\n'/; }"
      return 1
    fi
    mapfile -t receiver_pids < <(receiver_process_records)
    if (( ${#receiver_pids[@]} == 1 )); then
      capture_receiver_identity || {
        last_receiver_reason="receiver identity could not be captured atomically"
        sleep 0.2
        continue
      }
      guard_reason=$(receiver_guard_reason) || {
        last_receiver_reason=${guard_reason}
        vils_receiver_pid=""
        vils_receiver_starttime=""
        sleep 0.2
        continue
      }
      return 0
    fi
    if (( ${#receiver_pids[@]} > 1 )); then
      record_abort "multiple SocketCAN receivers appeared during startup: ${receiver_pids[*]}"
      return 1
    fi
    sleep 0.2
  done
  record_abort "receiver did not become ready within 10 seconds: ${last_receiver_reason}"
  return 1
}

wait_for_publisher_baseline()
{
  local readiness_deadline=$((SECONDS + 10))
  local error_file="${vils_run_dir}/publisher_readiness_error.txt"
  local last_reason="publisher endpoints were not discovered"
  local guard_reason

  while (( SECONDS < readiness_deadline )); do
    abort_is_latched && return 1
    guard_reason=$(can_fleet_guard_reason) || {
      record_abort "${guard_reason}"
      return 1
    }
    guard_reason=$(tx_endpoint_guard_reason) || {
      record_abort "${guard_reason}"
      return 1
    }
    if capture_publisher_baseline >"${error_file}" 2>&1; then
      rm -f "${error_file}"
      guard_reason=$(publisher_guard_reason) || {
        record_abort "${guard_reason}"
        return 1
      }
      return 0
    fi
    last_reason=$(<"${error_file}")
    sleep 0.2
  done
  rm -f "${error_file}"
  record_abort "required publisher provenance did not become ready within 10 seconds: ${last_reason}"
  return 1
}

verify_zero_residue()
{
  local conflicts
  local guard_reason
  local receiver_processes

  receiver_processes=$(receiver_process_records)
  [[ -z ${receiver_processes} ]] ||
    record_abort "SocketCAN receiver residue remains after shutdown: ${receiver_processes//$'\n'/; }"
  conflicts=$(forbidden_processes)
  [[ -z ${conflicts} ]] ||
    record_abort "actuator-capable process exists after shutdown: ${conflicts//$'\n'/; }"
  guard_reason=$(tx_endpoint_guard_reason) || record_abort "post-shutdown ${guard_reason}"
  guard_reason=$(can_fleet_guard_reason) || record_abort "post-shutdown ${guard_reason}"
}

cleanup()
{
  local original_status=$?
  local final_status=${original_status}

  [[ ${vils_cleaning} == false ]] || return
  vils_cleaning=true
  trap - EXIT
  trap '' HUP INT QUIT TERM TSTP USR1

  stop_background_group "${vils_timer_pid}" "${vils_timer_starttime}" \
    "duration watchdog" || record_abort "duration watchdog cleanup did not complete"
  stop_launch_group || record_abort "receiver-only launch cleanup did not complete"
  # Post-shutdown graph checks are passive residue checks; do not re-enter running-state guards after
  # the owned watchdog and launch group have intentionally been stopped.
  vils_launch_group_ready=false

  if [[ -n ${vils_run_dir} && -d ${vils_run_dir} ]]; then
    verify_zero_residue
    snapshot_state after || record_abort "one or more required after-snapshot commands failed"
    if [[ -n ${vils_requested_stop} && ${vils_ready} != true ]]; then
      record_abort "stop was requested before receiver readiness and required samples were proven"
    fi
    if [[ -z ${vils_requested_stop} && ! -s ${vils_abort_reason_file} ]]; then
      record_abort "run ended without an operator or duration stop request"
    fi
    validate_evidence
    if [[ -s ${vils_abort_reason_file} ]]; then
      final_status=1
    elif [[ -n ${vils_requested_stop} ]]; then
      date --utc --iso-8601=ns >"${vils_normal_stop_file}"
      printf '%s\n' "${vils_requested_stop}" >>"${vils_normal_stop_file}"
      final_status=0
    fi
    (
      cd "${vils_run_dir}"
      find . -type f ! -name checksums.txt -print0 | sort -z | xargs -0 sha256sum >checksums.txt
    ) || final_status=1
    if [[ -s ${vils_abort_reason_file} ]]; then
      printf '[%s] ABORTED:\n%s' "${vils_script_name}" "$(<"${vils_abort_reason_file}")" >&2
    fi
    printf '[%s] evidence: %s\n' "${vils_script_name}" "${vils_run_dir}"
  fi
  exit "${final_status}"
}

on_stop_signal()
{
  local signal_name="$1"

  if [[ ${vils_registering_child} == true || ${vils_launch_group_ready} != true ]]; then
    vils_pending_signal=${signal_name}
    return 0
  fi
  vils_requested_stop="operator-${signal_name}"
  exit 0
}

on_duration_signal()
{
  if [[ ${vils_registering_child} == true || ${vils_launch_group_ready} != true ]]; then
    vils_pending_signal="DURATION"
    return 0
  fi
  vils_requested_stop="duration-complete"
  exit 0
}

# HH_260814 - Refuse to start unless all vehicle-domain, clock, CAN, process, and ROS guards pass.
preflight
if [[ ${vils_check_only} == true ]]; then
  printf '[%s] PASS: preflight passed; no SocketCAN receiver or launch was started.\n' "${vils_script_name}"
  exit 0
fi

vils_resolved_evidence_root=$(realpath -m -- "${vils_evidence_root}") ||
  fail "cannot resolve evidence root: ${vils_evidence_root}"
vils_resolved_default_root=$(realpath -m -- "${vils_default_evidence_root}") ||
  fail "cannot resolve default evidence root"
case "${vils_resolved_evidence_root}" in
  "${vils_resolved_default_root}"|"${vils_resolved_default_root}"/*) ;;
  *) fail "evidence root must be the reviewed root or one of its children: ${vils_default_evidence_root}" ;;
esac
[[ ${vils_evidence_root} != *$'\n'* && ${vils_evidence_root} != *$'\r'* ]] ||
  fail "evidence root contains a control character"
[[ ! -L ${vils_evidence_root} ]] || fail "evidence root must not be a symlink: ${vils_evidence_root}"
vils_evidence_root=${vils_resolved_evidence_root}
if [[ -e ${vils_evidence_root} ]]; then
  [[ -d ${vils_evidence_root} && $(stat -c '%u' "${vils_evidence_root}") == "$(id -u)" ]] ||
    fail "existing evidence root must be an owned directory: ${vils_evidence_root}"
  [[ $(stat -c '%a' "${vils_evidence_root}") == 700 ]] ||
    fail "existing evidence root must already have mode 700: ${vils_evidence_root}"
else
  mkdir -p -- "${vils_evidence_root}" || fail "cannot create evidence root: ${vils_evidence_root}"
  chmod 700 -- "${vils_evidence_root}" || fail "cannot protect new evidence root: ${vils_evidence_root}"
fi
vils_run_dir=$(mktemp -d "${vils_evidence_root}/$(date --utc +%Y%m%dT%H%M%SZ)-pc1-rx-only.XXXXXX")
vils_abort_reason_file="${vils_run_dir}/abort_reason.txt"
vils_normal_stop_file="${vils_run_dir}/normal_stop.txt"
: >"${vils_abort_reason_file}"
: >"${vils_normal_stop_file}"

trap cleanup EXIT
trap 'on_stop_signal HUP' HUP
trap 'on_stop_signal INT' INT
trap 'on_stop_signal QUIT' QUIT
trap 'on_stop_signal TERM' TERM
trap '' TSTP
trap on_duration_signal USR1

git -C /home/a/autoware status --short --branch >"${vils_run_dir}/git_status.txt"
git -C /home/a/autoware rev-parse HEAD >"${vils_run_dir}/git_commit.txt"
printf 'ROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\nROS_LOCALHOST_ONLY=%s\nCYCLONEDDS_URI=%s\nEXPECTED_TIME_SOURCE=%s\nMAX_LAST_OFFSET_MS=%s\nMAX_RMS_OFFSET_MS=%s\nMAX_ROOT_DISPERSION_MS=%s\n' \
  "${ROS_DOMAIN_ID}" "${RMW_IMPLEMENTATION}" "${ROS_LOCALHOST_ONLY}" "${CYCLONEDDS_URI}" \
  "${vils_expected_time_source}" "${vils_max_last_offset_ms}" "${vils_max_rms_offset_ms}" \
  "${vils_max_root_dispersion_ms}" >"${vils_run_dir}/approved_environment.txt"
printf '%s\n' "${vils_package_prefix}" >"${vils_run_dir}/ros2_socketcan_prefix.txt"
sha256sum "${vils_expected_receiver_exe}" >"${vils_run_dir}/receiver_binary.sha256"
sha256sum "${vils_expected_receiver_component}" \
  >"${vils_run_dir}/receiver_component_library.sha256"
sha256sum "${vils_expected_receiver_core}" >"${vils_run_dir}/receiver_core_library.sha256"
sha256sum "${vils_installed_launch}" >"${vils_run_dir}/rx_only_launch.sha256"
sha256sum "${vils_installed_receiver_launch}" >"${vils_run_dir}/receiver_child_launch.sha256"
sha256sum "$(realpath "${vils_installed_runner}")" >"${vils_run_dir}/rx_only_runner.sha256"
sha256sum "${vils_cyclonedds_path}" >"${vils_run_dir}/cyclonedds_profile.sha256"
snapshot_state before || fail "one or more required before-snapshot commands failed"

# HH_260814 - Isolate the launch in its own process group so every descendant can be stopped and reaped.
if [[ -n ${vils_pending_signal} ]]; then
  exit 1
fi
vils_prelaunch_reason=$(prelaunch_guard_reason) || {
  record_abort "${vils_prelaunch_reason}"
  exit 1
}
vils_registering_child=true
if [[ -n ${vils_pending_signal} ]]; then
  vils_registering_child=false
  exit 1
fi
setsid bash -c '
  exec 9>&-
  kill -STOP "$BASHPID"
  exec ros2 launch ros2_socketcan vehicle_status_rx_only.launch.xml interface:="$1"
' _ "${vils_primary_can}" \
  >"${vils_run_dir}/receiver_launch.log" 2>&1 &
vils_launch_pid=$!
vils_launch_starttime=$(wait_for_suspended_child_identity "${vils_launch_pid}") || {
  vils_fallback_starttime=$(capture_direct_child_starttime "${vils_launch_pid}") || true
  if [[ -n ${vils_fallback_starttime} ]]; then
    vils_launch_starttime=${vils_fallback_starttime}
    stop_owned_process_group "${vils_launch_pid}" "${vils_fallback_starttime}" \
      "receiver-only launch handshake" TERM KILL || true
    if reap_direct_child_if_terminal "${vils_launch_pid}" "${vils_fallback_starttime}" \
      "receiver-only launch handshake"; then
      vils_launch_pid=""
      vils_launch_starttime=""
    fi
  else
    record_abort "cannot identify receiver-only launch child after handshake failure"
    if stop_unregistered_direct_child "${vils_launch_pid}" \
      "receiver-only launch handshake"; then
      vils_launch_pid=""
      vils_launch_starttime=""
    fi
  fi
  printf '%s\n' "${vils_unregistered_child_status}" >"${vils_run_dir}/receiver_launch_exit_status.txt"
  vils_registering_child=false
  record_abort "receiver-only launch failed its suspended identity handshake"
  exit 1
}
vils_prelaunch_reason=$(final_prelaunch_guard_reason) || {
  record_abort "final suspended-launch gate failed: ${vils_prelaunch_reason}"
  vils_registering_child=false
  exit 1
}
if [[ -n ${vils_pending_signal} ]]; then
  stop_owned_process_group "${vils_launch_pid}" "${vils_launch_starttime}" \
    "receiver-only launch" TERM KILL || true
  if reap_direct_child_if_terminal "${vils_launch_pid}" "${vils_launch_starttime}" \
    "receiver-only launch"; then
    vils_launch_pid=""
    vils_launch_starttime=""
  fi
  printf '%s\n' "${vils_unregistered_child_status}" >"${vils_run_dir}/receiver_launch_exit_status.txt"
  vils_registering_child=false
  exit 1
fi
kill -CONT -- "-${vils_launch_pid}" 2>/dev/null || {
  stop_owned_process_group "${vils_launch_pid}" "${vils_launch_starttime}" \
    "receiver-only launch" TERM KILL || true
  vils_registering_child=false
  record_abort "could not resume the registered receiver-only launch child"
  exit 1
}
vils_registering_child=false
if [[ -n ${vils_pending_signal} ]]; then
  record_abort "signal ${vils_pending_signal} arrived before receiver readiness"
  exit 1
fi
readonly vils_launch_start_seconds=${SECONDS}
if (( vils_duration_seconds > 0 )); then
  readonly vils_deadline_seconds=$((vils_launch_start_seconds + vils_duration_seconds))
else
  readonly vils_deadline_seconds=0
fi
wait_for_launch_group || exit 1
vils_launch_group_ready=true
if [[ -n ${vils_pending_signal} ]]; then
  if [[ ${vils_pending_signal} == DURATION ]]; then
    on_duration_signal
  else
    on_stop_signal "${vils_pending_signal}"
  fi
fi
if (( vils_duration_seconds > 0 )); then
  vils_timer_delay_seconds=$((vils_deadline_seconds - SECONDS))
  (( vils_timer_delay_seconds > 0 )) || vils_timer_delay_seconds=1
  vils_registering_child=true
  if [[ -n ${vils_pending_signal} ]]; then
    vils_registering_child=false
    if [[ ${vils_pending_signal} == DURATION ]]; then
      on_duration_signal
    else
      on_stop_signal "${vils_pending_signal}"
    fi
  fi
  setsid bash -c '
    exec 9>&-
    identity_matches()
    {
      local process_id="$1"
      local expected_starttime="$2"
      local expected_pgid="${3:-}"
      local expected_session="${4:-}"
      local stat_record
      local fields_record
      local -a stat_fields

      [[ -r /proc/${process_id}/stat ]] || return 1
      stat_record=$(</proc/"${process_id}"/stat) || return 1
      fields_record=${stat_record##*) }
      read -r -a stat_fields <<<"${fields_record}"
      (( ${#stat_fields[@]} > 19 )) || return 1
      [[ ${stat_fields[19]} == "${expected_starttime}" && ${stat_fields[0]} != Z ]] || return 1
      [[ -z ${expected_pgid} || ${stat_fields[2]} == "${expected_pgid}" ]] || return 1
      [[ -z ${expected_session} || ${stat_fields[3]} == "${expected_session}" ]] || return 1
    }

    kill -STOP "$BASHPID"
    sleep "$1"
    if identity_matches "$2" "$3" "$2" "$2"; then
      kill -INT -- "-$2" 2>/dev/null || true
    fi
    if identity_matches "$4" "$5"; then
      kill -USR1 "$4" 2>/dev/null || true
    fi
  ' _ "${vils_timer_delay_seconds}" \
    "${vils_launch_pid}" "${vils_launch_starttime}" \
    "${vils_parent_pid}" "${vils_parent_starttime}" &
  vils_timer_pid=$!
  vils_timer_starttime=$(wait_for_suspended_child_identity "${vils_timer_pid}") || {
    vils_fallback_starttime=$(capture_direct_child_starttime "${vils_timer_pid}") || true
    if [[ -n ${vils_fallback_starttime} ]]; then
      vils_timer_starttime=${vils_fallback_starttime}
      stop_owned_process_group "${vils_timer_pid}" "${vils_fallback_starttime}" \
        "duration watchdog handshake" TERM KILL || true
      if reap_direct_child_if_terminal "${vils_timer_pid}" "${vils_fallback_starttime}" \
        "duration watchdog handshake"; then
        vils_timer_pid=""
        vils_timer_starttime=""
      fi
    else
      record_abort "cannot identify duration watchdog child after handshake failure"
      if stop_unregistered_direct_child "${vils_timer_pid}" \
        "duration watchdog handshake"; then
        vils_timer_pid=""
        vils_timer_starttime=""
      fi
    fi
    vils_registering_child=false
    record_abort "duration watchdog failed its suspended identity handshake"
    exit 1
  }
  if [[ -n ${vils_pending_signal} ]]; then
    stop_owned_process_group "${vils_timer_pid}" "${vils_timer_starttime}" \
      "duration watchdog" TERM KILL || true
    if reap_direct_child_if_terminal "${vils_timer_pid}" "${vils_timer_starttime}" \
      "duration watchdog"; then
      vils_timer_pid=""
      vils_timer_starttime=""
    fi
    vils_registering_child=false
    if [[ ${vils_pending_signal} == DURATION ]]; then
      on_duration_signal
    else
      on_stop_signal "${vils_pending_signal}"
    fi
  fi
  kill -CONT -- "-${vils_timer_pid}" 2>/dev/null || {
    stop_owned_process_group "${vils_timer_pid}" "${vils_timer_starttime}" \
      "duration watchdog" TERM KILL || true
    vils_registering_child=false
    record_abort "could not resume the registered duration watchdog child"
    exit 1
  }
  vils_registering_child=false
  if [[ -n ${vils_pending_signal} ]]; then
    if [[ ${vils_pending_signal} == DURATION ]]; then
      on_duration_signal
    else
      on_stop_signal "${vils_pending_signal}"
    fi
  fi
fi

wait_for_receiver || exit 1
wait_for_publisher_baseline || exit 1
write_publisher_provenance || {
  record_abort "could not write required publisher provenance evidence"
  exit 1
}

# Require actual raw CAN and decoded vehicle-status samples under the independent duration watchdog.
if ! capture_required_samples; then
  record_abort "one or more required raw CAN/vehicle-status samples were not received"
  exit 1
fi
vils_readiness_reason=$(readiness_deadline_guard_reason) || {
  record_abort "${vils_readiness_reason}"
  exit 1
}

snapshot_state during || {
  record_abort "one or more required during-snapshot commands failed"
  exit 1
}
vils_readiness_reason=$(readiness_deadline_guard_reason) || {
  record_abort "${vils_readiness_reason}"
  exit 1
}
vils_ready=true
printf '[%s] running guarded receiver-only evidence collection; PID=%s; press Ctrl+C to stop.\n' \
  "${vils_script_name}" "${vils_launch_pid}"

vils_next_full_check_seconds=$((SECONDS + 30))
while launch_group_alive; do
  check_fast_safety_invariants || exit 1
  if (( SECONDS >= vils_next_full_check_seconds )); then
    check_invariants || exit 1
    vils_next_full_check_seconds=$((SECONDS + 30))
  fi
  if (( vils_deadline_seconds > 0 && SECONDS >= vils_deadline_seconds )); then
    vils_requested_stop="duration-complete"
    exit 0
  fi
  sleep 1
done

if (( vils_deadline_seconds > 0 && SECONDS >= vils_deadline_seconds )); then
  vils_requested_stop="duration-complete"
  exit 0
fi
record_abort "receiver-only launch group exited without a requested stop"
exit 1
