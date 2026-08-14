#!/usr/bin/env bash
# cspell:ignore VILS vils
# HH_260814 - Activate a dedicated PC3 VILS DDS profile without changing normal shell defaults.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This file must be sourced so its environment remains in the current shell." >&2
  exit 2
fi

_pc3_vils_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
_pc3_vils_dds="${_pc3_vils_dir}/config/cyclonedds_vehicle_domain.xml"

# HH_260814 - Fail closed when the versioned DDS file or the approved vehicle-LAN address is absent.
if [[ ! -f "${_pc3_vils_dds}" || -L "${_pc3_vils_dds}" ]]; then
  echo "PC3 VILS CycloneDDS configuration is missing or is a symbolic link." >&2
  return 2
fi
if ! ip -o -4 address show | awk '{print $4}' | grep -Fxq '192.168.9.7/24'; then
  echo "The approved PC3 vehicle-LAN address 192.168.9.7/24 is not active." >&2
  return 2
fi

# HH_260814 - Keep simulator clock and localhost-only discovery out of the vehicle-domain profile.
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://${_pc3_vils_dds}"
export PC3_VILS_CONTRACT="${_pc3_vils_dir}/pc3_vils_contract.yaml"
export PC3_VILS_BAG_TOPICS="${_pc3_vils_dir}/pc3_vils_bag_topics.txt"

printf 'PC3 VILS environment active: domain=%s rmw=%s dds=%s\n' \
  "${ROS_DOMAIN_ID}" "${RMW_IMPLEMENTATION}" "${CYCLONEDDS_URI}"

unset _pc3_vils_dir _pc3_vils_dds
