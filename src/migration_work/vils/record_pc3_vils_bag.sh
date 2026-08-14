#!/usr/bin/env bash
# cspell:ignore VILS vils
# HH_260814 - Record only reviewed PC3 evidence topics into a new compressed bag.

set -euo pipefail
umask 077

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <existing-run-directory> <duration-seconds>" >&2
  exit 2
fi

run_dir="$(realpath -e -- "$1")"
duration="$2"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
topic_file="${PC3_VILS_BAG_TOPICS:-${script_dir}/pc3_vils_bag_topics.txt}"

# HH_260814 - Refuse unsafe directories, unsupported domains, and unbounded recording durations.
if [[ ! -d "${run_dir}/rosbag" || -L "${run_dir}" ]]; then
  echo "The run directory was not initialized by the PC3 evidence collector." >&2
  exit 2
fi
if [[ -e "${run_dir}/manifest/finalization.json" || -e "${run_dir}/checksums.txt" ]]; then
  echo "The evidence run is finalized and immutable." >&2
  exit 2
fi
if [[ "$(stat -c '%u' "${run_dir}")" != "$(id -u)" ]] || (( 8#$(stat -c '%a' "${run_dir}") & 8#077 )); then
  echo "The run directory must be owner-only and owned by the current user." >&2
  exit 2
fi
if [[ ! "${duration}" =~ ^[0-9]+$ ]] || (( duration < 1 || duration > 3600 )); then
  echo "Duration must be an integer from 1 through 3600 seconds." >&2
  exit 2
fi
if [[ "${ROS_DOMAIN_ID:-}" != "10" || "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]]; then
  echo "Source activate_pc3_vils_environment.sh before recording." >&2
  exit 2
fi
if [[ "${ROS_LOCALHOST_ONLY:-}" != "0" || "${CYCLONEDDS_URI:-}" != file://* ]]; then
  echo "The reviewed vehicle-domain DDS profile is not active." >&2
  exit 2
fi
if [[ ! -f "${topic_file}" || -L "${topic_file}" ]]; then
  echo "The reviewed bag topic file is missing or is a symbolic link." >&2
  exit 2
fi

mapfile -t topics < <(sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' "${topic_file}")
if (( ${#topics[@]} == 0 )); then
  echo "The reviewed bag topic list is empty." >&2
  exit 2
fi

timestamp="$(date --utc +%Y%m%dT%H%M%SZ)"
output="${run_dir}/rosbag/pc3_${timestamp}"
if [[ -e "${output}" ]]; then
  echo "Refusing to overwrite an existing rosbag path: ${output}" >&2
  exit 2
fi

# HH_260814 - Use a bounded SIGINT shutdown so rosbag metadata is closed without starting any publisher.
set +e
timeout --signal=INT --kill-after=20s "${duration}s" \
  ros2 bag record \
  --output "${output}" \
  --storage sqlite3 \
  --compression-mode file \
  --compression-format zstd \
  --compression-threads 1 \
  --max-cache-size 104857600 \
  "${topics[@]}"
record_status=$?
set -e

if [[ ${record_status} -eq 124 && -f "${output}/metadata.yaml" ]]; then
  record_status=0
fi
if [[ ${record_status} -ne 0 ]]; then
  echo "ros2 bag record failed with status ${record_status}; preserve the partial output." >&2
  exit "${record_status}"
fi

printf '%s\n' "${output}"
