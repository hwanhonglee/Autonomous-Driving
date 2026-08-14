#!/usr/bin/env bash
set -u

readonly vils_out="${VILS_OUTPUT_DIR:-/home/a/vils_data/pc1_20260814_181555}"
readonly vils_vehicle_interface="${VILS_VEHICLE_INTERFACE:-enp0s31f6}"
readonly vils_wifi_interface="${VILS_WIFI_INTERFACE:-wlan0}"
readonly vils_can_interface="${VILS_CAN_INTERFACE:-can0}"
readonly vils_system_csv="${vils_out}/PC1_SYSTEM_METRICS.csv"
readonly vils_ping_csv="${vils_out}/PC1_PEER_PING.csv"
readonly vils_gpu_csv="${vils_out}/PC1_GPU_METRICS.csv"
readonly vils_process_log="${vils_out}/PC1_PROCESS_METRICS.log"
readonly vils_chrony_log="${vils_out}/PC1_CHRONY.log"
readonly vils_logger_log="${vils_out}/PC1_METRICS_LOGGER.log"

vils_net_stat() {
  local vils_interface="$1"
  local vils_field="$2"
  local vils_path="/sys/class/net/${vils_interface}/statistics/${vils_field}"
  if [[ -r "${vils_path}" ]]; then
    tr -d '\n' < "${vils_path}"
  else
    printf '%s' '-1'
  fi
}

if [[ ! -e "${vils_system_csv}" ]]; then
  printf '%s\n' 'iso_time,epoch_ns,uptime_s,load1,load5,load15,mem_available_kb,disk_available_bytes,bag_bytes,vehicle_rx_packets,vehicle_tx_packets,vehicle_rx_errors,vehicle_tx_errors,vehicle_rx_dropped,vehicle_tx_dropped,wifi_rx_packets,wifi_tx_packets,can0_rx_packets,can0_tx_packets,can0_rx_errors,can0_tx_errors,can0_rx_dropped,can0_tx_dropped' > "${vils_system_csv}"
fi
if [[ ! -e "${vils_ping_csv}" ]]; then
  printf '%s\n' 'iso_time,peer,return_code,rtt_ms' > "${vils_ping_csv}"
fi
if [[ ! -e "${vils_gpu_csv}" ]]; then
  printf '%s\n' 'iso_time,gpu_name,utilization_pct,memory_used_mib,memory_total_mib,temperature_c,power_w' > "${vils_gpu_csv}"
fi

printf '[%s] PC1 metrics logger started pid=%s\n' "$(date --iso-8601=seconds)" "$$" >> "${vils_logger_log}"
trap 'printf "[%s] PC1 metrics logger stopped pid=%s\n" "$(date --iso-8601=seconds)" "$$" >> "${vils_logger_log}"' EXIT

vils_iteration=0
while true; do
  vils_iso_time="$(date --iso-8601=ns)"
  vils_epoch_ns="$(date +%s%N)"
  vils_uptime_s="$(cut -d' ' -f1 /proc/uptime)"
  read -r vils_load1 vils_load5 vils_load15 _ < /proc/loadavg
  vils_mem_available_kb="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
  vils_disk_available_bytes="$(df --output=avail -B1 /home/a | tail -n 1 | tr -d ' ')"
  vils_bag_bytes="$(du -sb "${vils_out}" | cut -f1)"

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${vils_iso_time}" "${vils_epoch_ns}" "${vils_uptime_s}" \
    "${vils_load1}" "${vils_load5}" "${vils_load15}" "${vils_mem_available_kb}" \
    "${vils_disk_available_bytes}" "${vils_bag_bytes}" \
    "$(vils_net_stat "${vils_vehicle_interface}" rx_packets)" "$(vils_net_stat "${vils_vehicle_interface}" tx_packets)" \
    "$(vils_net_stat "${vils_vehicle_interface}" rx_errors)" "$(vils_net_stat "${vils_vehicle_interface}" tx_errors)" \
    "$(vils_net_stat "${vils_vehicle_interface}" rx_dropped)" "$(vils_net_stat "${vils_vehicle_interface}" tx_dropped)" \
    "$(vils_net_stat "${vils_wifi_interface}" rx_packets)" "$(vils_net_stat "${vils_wifi_interface}" tx_packets)" \
    "$(vils_net_stat "${vils_can_interface}" rx_packets)" "$(vils_net_stat "${vils_can_interface}" tx_packets)" \
    "$(vils_net_stat "${vils_can_interface}" rx_errors)" "$(vils_net_stat "${vils_can_interface}" tx_errors)" \
    "$(vils_net_stat "${vils_can_interface}" rx_dropped)" "$(vils_net_stat "${vils_can_interface}" tx_dropped)" >> "${vils_system_csv}"

  if (( vils_iteration % 5 == 0 )); then
    {
      printf '\n[%s] chronyc tracking\n' "${vils_iso_time}"
      timeout -k 1 3 chronyc -n tracking || printf 'chronyc tracking failed rc=%s\n' "$?"
      printf '[%s] chronyc sources\n' "${vils_iso_time}"
      timeout -k 1 3 chronyc -n sources || printf 'chronyc sources failed rc=%s\n' "$?"
      printf '[%s] chronyc sourcestats\n' "${vils_iso_time}"
      timeout -k 1 3 chronyc -n sourcestats || printf 'chronyc sourcestats failed rc=%s\n' "$?"
    } >> "${vils_chrony_log}" 2>&1

    for vils_peer in 192.168.9.7 192.168.9.110 192.168.9.12; do
      vils_ping_output="$(timeout -k 1 2 ping -n -c 1 -W 1 "${vils_peer}" 2>&1)"
      vils_ping_rc=$?
      vils_ping_rtt="$(printf '%s\n' "${vils_ping_output}" | sed -n 's/.*time=\([0-9.]*\) ms.*/\1/p' | head -n 1)"
      [[ -n "${vils_ping_rtt}" ]] || vils_ping_rtt='NA'
      printf '%s,%s,%s,%s\n' "${vils_iso_time}" "${vils_peer}" "${vils_ping_rc}" "${vils_ping_rtt}" >> "${vils_ping_csv}"
    done

    if command -v nvidia-smi >/dev/null 2>&1; then
      vils_gpu_line="$(timeout -k 1 3 nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw --format=csv,noheader,nounits 2>/dev/null | head -n 1)"
      if [[ -n "${vils_gpu_line}" ]]; then
        printf '%s,%s\n' "${vils_iso_time}" "${vils_gpu_line}" >> "${vils_gpu_csv}"
      fi
    fi

    {
      printf '\n[%s]\n' "${vils_iso_time}"
      ps -eo pid,ppid,pcpu,pmem,rss,etimes,comm,args --no-headers | \
        grep -E 'ros2 bag record|rosbag2_recorder|autoware.launch|can_brdige|socket_can_receiver|socket_can_sender|twistController' | \
        grep -v 'grep -E' || true
    } >> "${vils_process_log}"
  fi

  vils_iteration=$((vils_iteration + 1))
  sleep 1
done
