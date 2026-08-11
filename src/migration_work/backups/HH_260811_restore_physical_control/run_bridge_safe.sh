#!/usr/bin/env bash

set -euo pipefail

fail()
{
  printf '[run_bridge_safe.sh] BLOCKED: %s\n' "$*" >&2
  exit 1
}

if (( $# > 1 )) || (( $# == 1 )) && [[ $1 != "--check-only" ]]; then
  fail "the only supported option is --check-only"
fi

readonly bridge_lock="/run/user/$(id -u)/pc1-run-bridge.lock"
exec 9<>"${bridge_lock}"
flock -n 9 || fail "another run_bridge instance holds ${bridge_lock}"

# HH_260810 - Require the requested physical receive interfaces while keeping unused CAN links down.
for can_device in can0 can1; do
  [[ -e "/sys/class/net/${can_device}" ]] || fail "required physical interface ${can_device} is missing"
  can_line=$(ip -o link show dev "${can_device}")
  can_flags=${can_line#*<}
  can_flags=${can_flags%%>*}
  case ",${can_flags}," in
    *,UP,*) ;;
    *) fail "${can_device} is not administratively UP; run 'cd ~/scripts && ./start.sh' first" ;;
  esac
done
for can_device in can2 can3; do
  [[ -e "/sys/class/net/${can_device}" ]] || fail "required physical interface ${can_device} is missing"
  can_state=$(ip -brief link show dev "${can_device}" | awk '{print $2}')
  [[ ${can_state} == "DOWN" ]] || fail "unused interface ${can_device} is ${can_state}, expected DOWN"
done

# HH_260810 - Refuse duplicate bridges, actuator adapters, CAN utilities, and planning simulation; allow the receive-only bridge beside run_autoware.
unsafe_processes=()
for command_file in /proc/[0-9]*/cmdline; do
  process_id=${command_file#/proc/}
  process_id=${process_id%/cmdline}
  [[ ${process_id} == "$$" ]] && continue
  command_argv=()
  mapfile -d '' -t command_argv <"${command_file}" 2>/dev/null || continue
  (( ${#command_argv[@]} > 0 )) || continue
  command_text="${command_argv[*]}"
  process_is_unsafe=false
  for (( argument_index = 0; argument_index < ${#command_argv[@]}; argument_index++ )); do
    argument_base=${command_argv[argument_index]##*/}
    case "${argument_base}" in
      socket_can_sender_node_exe|socket_can_receiver_node_exe|twistController2VCU2EPS2ACC_node|twistController2vcu_node|twistController2EPS2ACC_node|cansend|cangen|canplayer|canfdtest|isotpsend)
        process_is_unsafe=true
        break
        ;;
      ros2)
        next_argument=${command_argv[argument_index + 1]:-}
        package_argument=${command_argv[argument_index + 2]:-}
        launch_argument=${command_argv[argument_index + 3]:-}
        if [[ ${next_argument} == "launch" && ${package_argument} == "ros2_socketcan" ]] ||
          [[ ${next_argument} == "run" && ${package_argument} == "ros2_socketcan" ]] ||
          [[ ${next_argument} == "launch" && ${package_argument} == "autoware_launch" && ${launch_argument} == "planning_simulator.launch.xml" ]]; then
          process_is_unsafe=true
          break
        fi
        ;;
    esac
  done
  if [[ ${process_is_unsafe} == true ]]; then
    unsafe_processes+=("${process_id}:${command_text}")
  fi
done

if (( ${#unsafe_processes[@]} > 0 )); then
  printf '[run_bridge_safe.sh] conflicting processes:\n' >&2
  printf '  %s\n' "${unsafe_processes[@]}" >&2
  fail "stop the listed process(es) before starting run_bridge"
fi

if (( $# == 1 )); then
  printf '[run_bridge_safe.sh] PASS: can0/can1 are UP, can2/can3 are DOWN, and no conflicting process is running; no launch was started.\n'
  exit 0
fi

set +u
source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash
set -u
export PATH="/usr/bin:${PATH}"
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# HH_260810 - Preserve run_bridge on physical can0 in receive-only mode; no CAN writer or control adapter is started.
exec ros2 launch ros2_socketcan can_brdige.launch.xml \
  interface:=can0 \
  enable_socketcan_bridge:=true \
  enable_socketcan_receiver:=true \
  enable_socketcan_sender:=false \
  enable_control_adapter:=false \
  from_can_bus_topic:=/pc1/can/from_can_bus \
  from_can_bus_fd_topic:=/pc1/can/from_can_bus_fd \
  to_can_bus_topic:=/pc1/can/to_can_bus \
  to_can_bus_fd_topic:=/pc1/can/to_can_bus_fd \
  receiver_node_name:=pc1_socket_can_receiver_can0 \
  sender_node_name:=pc1_socket_can_sender_can0
