#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

# HH_260810 - Disabled CAN link configuration and activation on PC1 so stationary tests cannot enable a physical actuator transport.
# sudo ip link set can0 type can bitrate 500000
# sudo ip link set can1 type can bitrate 500000
# sudo ip link set up can0
# sudo ip link set up can1

# HH_260810 - Prepared an isolated virtual CAN interface used by run_bridge without changing can0 through can3.
sudo modprobe vcan
if ! ip link show dev vcan0 >/dev/null 2>&1; then
  sudo ip link add dev vcan0 type vcan
fi
sudo ip link set dev vcan0 up

# HH_260810 - Kept the historical device-permission setup but skipped device classes that are not present.
shopt -s nullglob
tty_devices=(/dev/tty*)
video_devices=(/dev/video*)
if (( ${#tty_devices[@]} > 0 )); then
  sudo chmod 777 "${tty_devices[@]}"
fi
if (( ${#video_devices[@]} > 0 )); then
  sudo chmod 777 "${video_devices[@]}"
fi
if [[ -S /var/run/docker.sock ]]; then
  sudo chmod 777 /var/run/docker.sock
fi

printf 'vcan0 ready; physical can0-can3 were not enabled.\n'
