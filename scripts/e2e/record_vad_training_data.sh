#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
default_profile="${root}/autoware_e2e_vad_launch/config/vad_real_data_collection.yaml"
contract_tool="${root}/scripts/e2e/vad_training_data_contract.py"
validator="${root}/scripts/e2e/validate_vad_training_bag.py"
calibration_renderer="${root}/scripts/e2e/render_vad_calibration_preview.py"

profile="${default_profile}"
duration_s=""
preflight_timeout_s="15"
skip_preflight=false
dry_run=false

usage() {
  cat <<'EOF'
Usage: record_vad_training_data.sh [OPTIONS] OUTPUT_SESSION

Options:
  --profile PATH             Collection profile (default: project real profile)
  --duration SEC             Stop and finalize after SEC; otherwise use Ctrl-C
  --preflight-timeout SEC    Wait for required topics (default: 15)
  --skip-preflight           Record without live required-topic verification
  --dry-run                  Validate and print the rosbag command only
  -h, --help                 Show this help
EOF
}

positionals=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || { echo "--profile requires a path" >&2; exit 2; }
      profile="$2"
      shift 2
      ;;
    --duration)
      [[ $# -ge 2 ]] || { echo "--duration requires seconds" >&2; exit 2; }
      duration_s="$2"
      shift 2
      ;;
    --preflight-timeout)
      [[ $# -ge 2 ]] || { echo "--preflight-timeout requires seconds" >&2; exit 2; }
      preflight_timeout_s="$2"
      shift 2
      ;;
    --skip-preflight)
      skip_preflight=true
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      positionals+=("$@")
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      positionals+=("$1")
      shift
      ;;
  esac
done

if [[ ${#positionals[@]} -ne 1 ]]; then
  usage >&2
  exit 2
fi

profile="$(realpath -- "${profile}")"
output="$(realpath -m -- "${positionals[0]}")"

if [[ -n "${duration_s}" ]] && ! [[ "${duration_s}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--duration must be a positive number" >&2
  exit 2
fi
if [[ -n "${duration_s}" ]] && ! awk -v value="${duration_s}" 'BEGIN { exit !(value > 0) }'; then
  echo "--duration must be a positive number" >&2
  exit 2
fi
if ! [[ "${preflight_timeout_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
   ! awk -v value="${preflight_timeout_s}" 'BEGIN { exit !(value > 0) }'; then
  echo "--preflight-timeout must be a positive number" >&2
  exit 2
fi

# Capture does not require CUDA, CARLA, or the NVIDIA driver. Avoid env.sh so a
# GPU mismatch cannot prevent read-only sensor logging.
restore_nounset=false
if [[ $- == *u* ]]; then
  restore_nounset=true
  set +u
fi
source /opt/ros/humble/setup.bash
if [[ -f "${root}/install/local_setup.bash" ]]; then
  source "${root}/install/local_setup.bash"
fi
if [[ "${restore_nounset}" == true ]]; then
  set -u
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${AUTOWARE_E2E_ROS_DOMAIN_ID:-42}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
ros2_command="${AUTOWARE_E2E_ROS2_BIN:-ros2}"

python3 "${contract_tool}" check --capture-ready "${profile}"
calibration="$(python3 "${contract_tool}" calibration-path "${profile}")"
commissioning="$(python3 "${contract_tool}" provenance-path "${profile}" commissioning_file)"
rectification_config="$(python3 "${contract_tool}" provenance-path "${profile}" rectification_config_file)"
mapfile -t commissioning_evidence < <(
  python3 "${contract_tool}" commissioning-evidence-paths "${profile}"
)
mapfile -t topics < <(python3 "${contract_tool}" topics "${profile}")
if [[ ${#topics[@]} -eq 0 ]]; then
  echo "Profile produced no topics: ${profile}" >&2
  exit 2
fi

record_command=(
  "${ros2_command}" bag record
  --storage sqlite3
  --output "${output}/bag"
  --max-bag-size 4294967296
  --max-cache-size 536870912
  --compression-mode file
  --compression-format zstd
  --compression-threads 2
  "${topics[@]}"
)

if [[ "${dry_run}" == true ]]; then
  printf 'ROS_DOMAIN_ID=%q ' "${ROS_DOMAIN_ID}"
  printf '%q ' "${record_command[@]}"
  printf '\n'
  exit 0
fi

if [[ -e "${output}" || -L "${output}" ]]; then
  echo "Output session already exists: ${output}" >&2
  exit 2
fi

exec 9>"/tmp/autoware_e2e_vad_dataset_recorder_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo "Another VAD dataset recorder is already starting or running in domain ${ROS_DOMAIN_ID}." >&2
  exit 1
fi
active_nodes="$("${ros2_command}" node list --no-daemon 2>/dev/null || true)"
if grep -Fxq "/rosbag2_recorder" <<< "${active_nodes}"; then
  echo "A rosbag2 recorder is already present in ROS domain ${ROS_DOMAIN_ID}." >&2
  echo "Stop it cleanly before starting this capture." >&2
  exit 1
fi

if [[ "${skip_preflight}" != true ]]; then
  echo "Waiting for the required six-camera and ego-state topics..."
  python3 "${contract_tool}" preflight "${profile}" --timeout "${preflight_timeout_s}"
else
  echo "WARNING: live topic preflight was explicitly skipped." >&2
fi

mkdir -p "${output}"
cp -- "${profile}" "${output}/collection_profile.yaml"
cp -- "${calibration}" "${output}/calibration.yaml"
cp -- "${rectification_config}" "${output}/rectification_config.yaml"
mkdir -p "${output}/commissioning_evidence"
cp -- "${commissioning}" "${output}/commissioning_evidence/commissioning.yaml"
for evidence_path in "${commissioning_evidence[@]}"; do
  cp --reflink=auto -- "${evidence_path}" \
    "${output}/commissioning_evidence/$(basename -- "${evidence_path}")"
done
profile_snapshot="${output}/collection_profile.yaml"
calibration_snapshot="${output}/calibration.yaml"
commissioning_snapshot="${output}/commissioning_evidence/commissioning.yaml"
rectification_snapshot="${output}/rectification_config.yaml"
python3 "${contract_tool}" manifest "${profile_snapshot}" "${output}" --status recording \
  --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
  --rectification-config "${rectification_snapshot}"

echo "Recording VAD training data to ${output}/bag"
if [[ -n "${duration_s}" ]]; then
  echo "The recorder will stop after ${duration_s} seconds."
else
  echo "Press Ctrl-C once to stop and finalize the bag."
fi

recorder_pid=""
timer_pid=""
signal_received=""
duration_elapsed=false
duration_marker="${output}/.duration_elapsed"

stop_recorder() {
  signal_received="${1:-INT}"
  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    # Async children inherit SIGINT as ignored from non-interactive Bash. ROS 2
    # handles SIGTERM through the same shutdown path and flushes rosbag storage.
    kill -TERM "${recorder_pid}" 2>/dev/null || true
  fi
}
trap 'stop_recorder INT' INT
trap 'stop_recorder TERM' TERM

set +e
"${record_command[@]}" &
recorder_pid=$!
if [[ -n "${duration_s}" ]]; then
  (
    sleep "${duration_s}"
    : > "${duration_marker}"
    kill -TERM "${recorder_pid}" 2>/dev/null || true
  ) &
  timer_pid=$!
fi
record_status=0
while true; do
  wait "${recorder_pid}"
  wait_status=$?
  if ! kill -0 "${recorder_pid}" 2>/dev/null; then
    record_status=${wait_status}
    break
  fi
done
set -e

if [[ -n "${timer_pid}" ]]; then
  kill "${timer_pid}" 2>/dev/null || true
  wait "${timer_pid}" 2>/dev/null || true
fi
if [[ -f "${duration_marker}" ]]; then
  duration_elapsed=true
  rm -f -- "${duration_marker}"
fi
trap - INT TERM

expected_stop_status=false
if [[ ${record_status} -eq 0 ]]; then
  expected_stop_status=true
elif [[ ${record_status} -eq 130 || ${record_status} -eq 143 ]] && \
     { [[ -n "${signal_received}" ]] || [[ "${duration_elapsed}" == true ]]; }; then
  expected_stop_status=true
fi

if [[ -f "${output}/bag/metadata.yaml" ]] && [[ "${expected_stop_status}" == true ]]; then
  echo "Hashing finalized bag files for dataset provenance..."
  python3 "${contract_tool}" manifest \
    "${profile_snapshot}" "${output}" --status recorded_unvalidated --bag "${output}/bag" \
    --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
    --rectification-config "${rectification_snapshot}" --hash-bag-files
  echo "Bag finalized. Validating capture contract..."
  set +e
  python3 "${validator}" "${output}/bag" --profile "${profile_snapshot}" \
    --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
    --rectification-config "${rectification_snapshot}" \
    --output "${output}/training_data_report.json"
  validation_status=$?
  set -e
  if [[ ${validation_status} -ne 0 ]]; then
    python3 "${contract_tool}" manifest \
      "${profile_snapshot}" "${output}" --status validation_failed --bag "${output}/bag" \
      --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
      --rectification-config "${rectification_snapshot}"
    echo "Recording completed, but the training-data contract failed." >&2
    exit ${validation_status}
  fi
  if ! python3 "${calibration_renderer}" "${output}/bag" --profile "${profile_snapshot}" \
    --calibration "${calibration_snapshot}" --output "${output}/calibration_preview.png"; then
    python3 "${contract_tool}" manifest \
      "${profile_snapshot}" "${output}" --status preview_failed --bag "${output}/bag" \
      --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
      --rectification-config "${rectification_snapshot}"
    echo "Capture passed, but the calibration preview needs manual investigation." >&2
    exit 1
  fi
  if ! python3 "${contract_tool}" model-artifacts "${profile_snapshot}"; then
    python3 "${contract_tool}" manifest \
      "${profile_snapshot}" "${output}" --status model_artifact_failed --bag "${output}/bag" \
      --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
      --rectification-config "${rectification_snapshot}"
    echo "Capture passed, but a deployment model artifact changed during recording." >&2
    exit 1
  fi
  python3 "${contract_tool}" manifest \
    "${profile_snapshot}" "${output}" --status validated --bag "${output}/bag" \
    --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
    --rectification-config "${rectification_snapshot}"
  echo "Capture passed: ${output}/training_data_report.json"
  echo "Calibration preview: ${output}/calibration_preview.png"
  exit 0
fi

python3 "${contract_tool}" manifest "${profile_snapshot}" "${output}" --status failed \
  --calibration "${calibration_snapshot}" --commissioning "${commissioning_snapshot}" \
  --rectification-config "${rectification_snapshot}"
echo "rosbag2 did not finalize metadata (exit ${record_status}, signal ${signal_received:-none})." >&2
if [[ ${record_status} -eq 0 ]]; then
  exit 1
fi
exit "${record_status}"
