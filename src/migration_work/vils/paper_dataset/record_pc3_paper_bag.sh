#!/usr/bin/env bash
# cspell:ignore CREAT ISLNK ISREG WRONLY fastwrite fdopen fileno getuid isinstance rglob vils
# HH_260814 - Record a bounded, profile-locked PC3 paper bag without publishing or actuating anything.

set -euo pipefail
umask 077

if [[ $# -ne 3 ]]; then
  echo "Usage: $0 <existing-paper-run-directory> <core|lidar> <duration-seconds>" >&2
  exit 2
fi

run_dir="$(realpath -e -- "$1")"
profile="$2"
duration="$3"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
vils_dir="$(realpath -e -- "${script_dir}/..")"

# HH_260814 - Bind recording to the immutable topic and QoS copies frozen by prepare_pc3_paper_run.py.
case "${profile}" in
  core)
    topic_file="${run_dir}/manifest/pc3_paper_core_topics.txt"
    qos_file="${run_dir}/manifest/pc3_paper_core_qos.yaml"
    max_duration=3600
    cache_size=67108864
    required_bytes=$((5 * 1024 * 1024 * 1024))
    ;;
  lidar)
    topic_file="${run_dir}/manifest/pc3_paper_lidar_topics.txt"
    qos_file="${run_dir}/manifest/pc3_paper_lidar_qos.yaml"
    max_duration=600
    cache_size=268435456
    required_bytes=0
    ;;
  *)
    echo "Profile must be core or lidar." >&2
    exit 2
    ;;
esac

if [[ ! "${duration}" =~ ^[0-9]+$ ]] || (( duration < 1 || duration > max_duration )); then
  echo "Duration must be an integer from 1 through ${max_duration} seconds for ${profile}." >&2
  exit 2
fi
if [[ "${profile}" == "lidar" ]]; then
  required_bytes=$((5 * 1024 * 1024 * 1024 + duration * 20000000))
fi

# HH_260814 - Refuse reused, finalized, aborted, foreign-owned, or world-readable evidence directories.
if [[ ! -f "${run_dir}/manifest/run.json" || ! -f "${run_dir}/manifest/paper_trial.json" || ! -d "${run_dir}/rosbag" ]]; then
  echo "The directory was not prepared as a PC3 paper run." >&2
  exit 2
fi
if [[ -e "${run_dir}/manifest/abort.json" || -e "${run_dir}/manifest/finalization.json" || -e "${run_dir}/checksums.txt" ]]; then
  echo "The paper run is aborted or finalized and cannot accept another recording." >&2
  exit 2
fi
if [[ -L "${run_dir}" || "$(stat -c '%u' "${run_dir}")" != "$(id -u)" ]] || (( 8#$(stat -c '%a' "${run_dir}") & 8#077 )); then
  echo "The run directory must be owner-only, non-symlinked, and owned by the current user." >&2
  exit 2
fi

# HH_260814 - Require the exact vehicle-domain DDS file and prevent Wi-Fi or LiDAR-LAN auto-selection.
expected_dds="$(realpath -e -- "${vils_dir}/config/cyclonedds_vehicle_domain.xml")"
if [[ "${ROS_DOMAIN_ID:-}" != "10" || "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" || "${ROS_LOCALHOST_ONLY:-}" != "0" ]]; then
  echo "Source activate_pc3_vils_environment.sh before recording." >&2
  exit 2
fi
if [[ "${CYCLONEDDS_URI:-}" != file://* ]]; then
  echo "CYCLONEDDS_URI must name the reviewed file profile." >&2
  exit 2
fi
active_dds="$(realpath -e -- "${CYCLONEDDS_URI#file://}")"
if [[ "${active_dds}" != "${expected_dds}" ]]; then
  echo "The active DDS file is not the reviewed PC3 vehicle-domain profile." >&2
  exit 2
fi

for frozen in "${topic_file}" "${qos_file}"; do
  if [[ ! -f "${frozen}" || -L "${frozen}" ]]; then
    echo "Frozen recorder input is missing or unsafe: ${frozen}" >&2
    exit 2
  fi
done

source_topics="${script_dir}/pc3_paper_${profile}_topics.txt"
source_qos="${script_dir}/pc3_paper_${profile}_qos.yaml"

# HH_260814 - Fail closed when the two manifests do not describe this exact run and frozen profile.
python3 - "${run_dir}" "${profile}" "${duration}" "${topic_file}" "${qos_file}" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys


SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")
run_dir = Path(sys.argv[1]).resolve(strict=True)
profile = sys.argv[2]
requested_duration = int(sys.argv[3])
topic_file = Path(sys.argv[4])
qos_file = Path(sys.argv[5])


def fail(message: str) -> None:
    raise SystemExit(f"Manifest validation failed: {message}")


def load_regular_json(path: Path) -> dict:
    try:
        status = path.lstat()
    except OSError as error:
        fail(f"cannot inspect {path}: {error}")
    if stat.S_ISLNK(status.st_mode) or not stat.S_ISREG(status.st_mode):
        fail(f"manifest is not a regular non-symlink file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        fail(f"cannot parse {path}: {error}")
    if not isinstance(value, dict):
        fail(f"manifest root must be an object: {path}")
    return value


def require_safe_id(label: str, value: object) -> str:
    if not isinstance(value, str) or not SAFE_ID.fullmatch(value):
        fail(f"{label} is missing or unsafe")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


run = load_regular_json(run_dir / "manifest" / "run.json")
trial = load_regular_json(run_dir / "manifest" / "paper_trial.json")
if run.get("schema_version") != 1 or trial.get("schema_version") != 1:
    fail("unsupported run.json or paper_trial.json schema_version")

run_id = require_safe_id("run_id", run.get("run_id"))
if require_safe_id("paper_trial.run_id", trial.get("run_id")) != run_id:
    fail("run_id differs between run.json and paper_trial.json")
require_safe_id("scenario_id", trial.get("scenario_id"))
require_safe_id("condition_id", trial.get("condition_id"))
replicate_id = trial.get("replicate_id")
if isinstance(replicate_id, bool) or not isinstance(replicate_id, int) or replicate_id < 1:
    fail("replicate_id must be a positive integer")
if trial.get("status") != "PREPARED_NOT_STARTED":
    fail("paper_trial status is not PREPARED_NOT_STARTED")
if trial.get("paper_sample_eligible") != "pending":
    fail("paper_sample_eligible is not pending")

try:
    recorded_run_dir = Path(run["run_directory"]).resolve(strict=True)
except (KeyError, OSError, TypeError) as error:
    fail(f"run_directory is invalid: {error}")
if recorded_run_dir != run_dir:
    fail("run_directory does not match the requested directory")
if run.get("uid") != os.getuid():
    fail("run.json uid does not match the recorder user")
if not isinstance(run.get("host"), str) or trial.get("host") != run.get("host"):
    fail("host identity differs between manifests")
if trial.get("host_role") != "PC3_MAP_LOCALIZATION_PHYSICAL_LIDAR":
    fail("paper_trial host_role is not PC3")

planned_duration = trial.get("planned_duration_sec")
if (
    isinstance(planned_duration, bool)
    or not isinstance(planned_duration, int)
    or planned_duration < requested_duration
):
    fail("requested duration exceeds the positive planned_duration_sec")

expected_topic_relative = f"manifest/pc3_paper_{profile}_topics.txt"
profiles = trial.get("recorder_profiles")
if not isinstance(profiles, dict) or profiles.get(profile) != expected_topic_relative:
    fail(f"recorder_profiles.{profile} does not name {expected_topic_relative}")
for label, path, expected_relative in (
    ("topic", topic_file, expected_topic_relative),
    ("QoS", qos_file, f"manifest/pc3_paper_{profile}_qos.yaml"),
):
    try:
        expected_path = (run_dir / expected_relative).resolve(strict=True)
        actual_path = path.resolve(strict=True)
    except OSError as error:
        fail(f"cannot resolve frozen {label} profile: {error}")
    if actual_path != expected_path:
        fail(f"frozen {label} profile path escaped the run manifest")

# HH_260814 - Verify the nested hash records emitted by the paper-run preparer when present.
profile_hashes = trial.get("profile_hashes")
if profile_hashes is not None:
    if not isinstance(profile_hashes, dict) or not isinstance(profile_hashes.get(profile), dict):
        fail(f"profile_hashes.{profile} must be an object")
    sealed_profile = profile_hashes[profile]
    for key, path in (("topics", topic_file), ("qos", qos_file)):
        record = sealed_profile.get(key)
        relative = str(path.relative_to(run_dir))
        if not isinstance(record, dict) or record.get("path") != relative:
            fail(f"profile_hashes.{profile}.{key} path differs from {relative}")
        expected = record.get("sha256")
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            fail(f"profile_hashes.{profile}.{key} has no valid SHA-256")
        if sha256(path) != expected:
            fail(f"profile_hashes.{profile}.{key} hash differs")

# HH_260814 - Also accept a flat hash seal from compatible future or imported manifests.
for owner_name, owner in (("run.json", run), ("paper_trial.json", trial)):
    hashes = owner.get("frozen_profile_sha256")
    if hashes is None:
        continue
    if not isinstance(hashes, dict):
        fail(f"{owner_name} frozen_profile_sha256 must be an object")
    for path in (topic_file, qos_file):
        relative = str(path.relative_to(run_dir))
        expected = hashes.get(relative)
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            fail(f"{owner_name} lacks a valid hash for {relative}")
        if sha256(path) != expected:
            fail(f"{owner_name} hash differs for {relative}")
PY

if ! cmp --silent -- "${source_topics}" "${topic_file}" || ! cmp --silent -- "${source_qos}" "${qos_file}"; then
  echo "Recorder source and frozen run profile differ; create a new run instead of mutating evidence." >&2
  exit 2
fi

mapfile -t topics < <(sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' "${topic_file}")
if (( ${#topics[@]} == 0 )); then
  echo "The frozen topic allow-list is empty." >&2
  exit 2
fi
for topic in "${topics[@]}"; do
  if [[ ! "${topic}" =~ ^/[A-Za-z0-9_/]+$ ]]; then
    echo "Invalid topic in frozen allow-list: ${topic}" >&2
    exit 2
  fi
done

available_bytes="$(df --output=avail -B1 "${run_dir}" | awk 'NR==2 {print $1}')"
if [[ ! "${available_bytes}" =~ ^[0-9]+$ ]] || (( available_bytes < required_bytes )); then
  echo "Insufficient free space for ${profile}: available=${available_bytes:-unknown}, required=${required_bytes}." >&2
  exit 2
fi

# HH_260814 - Allow at most one recorder for each profile and split files for power-loss containment.
lock_file="${run_dir}/events/recording_locks/pc3_${profile}.lock"
exec 9>"${lock_file}"
if ! flock -n 9; then
  echo "A ${profile} recorder already owns this run." >&2
  exit 2
fi

# HH_260814 - One run may contain core and LiDAR once each, but never a retry or concatenation.
completion_marker="${run_dir}/events/pc3_${profile}_recording_complete.json"
existing_output="$(find "${run_dir}/rosbag" -mindepth 1 -maxdepth 1 -name "pc3_${profile}_*" -print -quit)"
if [[ -e "${completion_marker}" || -L "${completion_marker}" || -n "${existing_output}" ]]; then
  echo "The ${profile} profile already has a completion marker or output; start a new run_id." >&2
  exit 2
fi

timestamp="$(date --utc +%Y%m%dT%H%M%SZ)"
output="${run_dir}/rosbag/pc3_${profile}_${timestamp}"
if [[ -e "${output}" ]]; then
  echo "Refusing to overwrite an existing output: ${output}" >&2
  exit 2
fi

start_utc="$(date --utc --iso-8601=ns)"
printf 'profile=%s duration=%s available_bytes=%s output=%s\n' \
  "${profile}" "${duration}" "${available_bytes}" "${output}"

set +e
timeout --signal=INT --kill-after=120s "${duration}s" \
  ros2 bag record \
  --output "${output}" \
  --storage mcap \
  --storage-preset-profile fastwrite \
  --max-bag-duration 60 \
  --max-bag-size 2147483648 \
  --max-cache-size "${cache_size}" \
  --compression-mode none \
  --qos-profile-overrides-path "${qos_file}" \
  "${topics[@]}"
record_status=$?
set -e

end_utc="$(date --utc --iso-8601=ns)"
raw_record_status=${record_status}
if [[ ${record_status} -eq 124 && -f "${output}/metadata.yaml" ]]; then
  record_status=0
fi
if [[ ${record_status} -ne 0 ]]; then
  echo "ros2 bag record failed with status ${record_status}; preserve the partial output." >&2
  exit "${record_status}"
fi
if [[ ! -f "${output}/metadata.yaml" || -L "${output}/metadata.yaml" ]]; then
  echo "ros2 bag record returned success without a safe metadata.yaml; preserve the partial output." >&2
  exit 1
fi

# HH_260814 - Reject empty, substantially short, or discontinuous evidence before marking it complete.
python3 - "${output}" "${profile}" "${duration}" <<'PY'
import json
import math
import os
from pathlib import Path
import sys

import yaml


output = Path(sys.argv[1])
profile = sys.argv[2]
requested_duration = int(sys.argv[3])
metadata_path = output / "metadata.yaml"


def fail(message: str) -> None:
    raise SystemExit(f"Bag validation failed: {message}")


try:
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, yaml.YAMLError) as error:
    fail(f"cannot parse metadata.yaml: {error}")
if not isinstance(document, dict):
    fail("metadata.yaml root is not an object")
information = document.get("rosbag2_bagfile_information", document)
if not isinstance(information, dict):
    fail("rosbag2_bagfile_information is not an object")
if information.get("storage_identifier") != "mcap":
    fail("storage_identifier is not mcap")

duration_field = information.get("duration")
if isinstance(duration_field, dict):
    duration_ns = duration_field.get("nanoseconds")
else:
    duration_ns = duration_field
if isinstance(duration_ns, bool) or not isinstance(duration_ns, int) or duration_ns < 0:
    fail("duration.nanoseconds is missing or invalid")
actual_duration_sec = duration_ns / 1_000_000_000
allowed_shortfall_sec = max(1.0, requested_duration * 0.10)
minimum_duration_sec = max(0.10, requested_duration - allowed_shortfall_sec)
if actual_duration_sec < minimum_duration_sec:
    fail(
        f"recorded duration {actual_duration_sec:.3f}s is below "
        f"the {minimum_duration_sec:.3f}s minimum for {requested_duration}s requested"
    )

message_count = information.get("message_count")
if isinstance(message_count, bool) or not isinstance(message_count, int) or message_count <= 0:
    fail("message_count is missing or zero")
topic_counts: dict[str, int] = {}
topics = information.get("topics_with_message_count")
if not isinstance(topics, list):
    fail("topics_with_message_count is missing")
for entry in topics:
    if not isinstance(entry, dict):
        fail("topic count entry is not an object")
    metadata = entry.get("topic_metadata")
    count = entry.get("message_count")
    if not isinstance(metadata, dict) or not isinstance(metadata.get("name"), str):
        fail("topic metadata name is missing")
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        fail(f"message count is invalid for {metadata.get('name')}")
    topic_counts[metadata["name"]] = topic_counts.get(metadata["name"], 0) + count
if sum(topic_counts.values()) != message_count:
    fail("message_count differs from the sum of per-topic counts")

minimum_rates_hz = {
    "core": {
        "/localization/kinematic_state": 1.0,
        "/tf": 1.0,
    },
    "lidar": {
        "/sensing/lidar/concatenated/pointcloud": 5.0,
    },
}[profile]
required_counts = {
    topic: max(1, math.ceil(minimum_duration_sec * rate))
    for topic, rate in minimum_rates_hz.items()
}
for topic, required in required_counts.items():
    observed = topic_counts.get(topic, 0)
    if observed < required:
        fail(f"continuous topic {topic} has {observed} messages; at least {required} are required")

payload = {
    "schema_version": 1,
    "status": "VALIDATED_NOT_YET_MARKED_COMPLETE",
    "profile": profile,
    "requested_duration_sec": requested_duration,
    "minimum_duration_sec": minimum_duration_sec,
    "actual_duration_sec": actual_duration_sec,
    "message_count": message_count,
    "required_topic_minimum_counts": required_counts,
    "observed_required_topic_counts": {
        topic: topic_counts[topic] for topic in required_counts
    },
}
target = output / "recording_validation.json"
descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, indent=2, sort_keys=True)
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
PY

if ! ros2 bag info "${output}" > "${output}/bag_info.txt"; then
  echo "ros2 bag info failed; preserve the validated but incomplete output." >&2
  exit 1
fi

python3 - "${run_dir}" "${output}" "${profile}" "${duration}" "${start_utc}" "${end_utc}" "${raw_record_status}" <<'PY'
# HH_260814 - Store output-local and run-level exclusive completion records only after validation.
import json
import os
from pathlib import Path
import sys

run_dir = Path(sys.argv[1])
output = Path(sys.argv[2])
validation = json.loads((output / "recording_validation.json").read_text(encoding="utf-8"))
payload = {
    "schema_version": 1,
    "status": "RECORDED_CLOSED",
    "profile": sys.argv[3],
    "requested_duration_sec": int(sys.argv[4]),
    "actual_duration_sec": validation["actual_duration_sec"],
    "message_count": validation["message_count"],
    "required_topic_minimum_counts": validation["required_topic_minimum_counts"],
    "observed_required_topic_counts": validation["observed_required_topic_counts"],
    "start_utc": sys.argv[5],
    "end_utc": sys.argv[6],
    "ros2_record_exit_status": int(sys.argv[7]),
    "output_relative": str(output.relative_to(run_dir)),
    "size_bytes": sum(path.stat().st_size for path in output.rglob("*") if path.is_file()),
}


def exclusive_json(target: Path) -> None:
    descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


exclusive_json(output / "recording_complete.json")
exclusive_json(run_dir / "events" / f"pc3_{sys.argv[3]}_recording_complete.json")
PY

printf '%s\n' "${output}"
