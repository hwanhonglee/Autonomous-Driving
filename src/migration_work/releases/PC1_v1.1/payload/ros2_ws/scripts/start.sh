#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

# HH_260810 - Made the legacy physical CAN setup repeatable for the full RX/TX control bridge.
sudo ip link set dev can0 down 2>/dev/null || true
sudo ip link set dev can1 down 2>/dev/null || true
sudo ip link set dev can0 type can bitrate 500000
sudo ip link set dev can1 type can bitrate 500000
sudo ip link set dev can0 up
sudo ip link set dev can1 up

# HH_260810 - Kept the historical device-permission setup but skipped absent device classes.
# Release note: the retained chmod 777 policy is legacy behavior and remains a security issue.
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

printf 'can0 and can1 ready at 500000 bit/s.\n'
