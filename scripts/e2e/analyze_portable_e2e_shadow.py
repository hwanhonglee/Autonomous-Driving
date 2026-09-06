#!/usr/bin/env python3
# HH_260906 - Analyze Portable E2E shadow recordings without granting control authority.
"""Fail-closed analyzer for Portable E2E shadow trial evidence."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
from typing import Any, Mapping, Sequence

import yaml

from portable_e2e.runtime_contract import RUNTIME_GATE_ID, RuntimeGateConfig

try:
    from validate_vad_training_bag import percentile, sequence_timing
except ModuleNotFoundError:
    from scripts.e2e.validate_vad_training_bag import percentile, sequence_timing


NORMALIZED_SCHEMA_ID = "autoware-e2e.portable-shadow-evidence-normalized.v1"
REPORT_SCHEMA_ID = "autoware-e2e.portable-shadow-evidence-analysis.v2"
STATUS_SCHEMA_ID = "autoware-e2e.portable-shadow-status.v3"
ADAPTER_ID = "autoware-e2e.portable-shadow-node.v3"
WINDOW_BOUNDARY_SCHEMA_ID = (
    "autoware-e2e.portable-shadow-window-boundaries.v2"
)
STATUS_TOPIC = "/planning/portable_e2e/status"
LATENCY_TOPIC = "/planning/portable_e2e/latency_ms"
CANDIDATE_TOPIC = "/planning/portable_e2e/selected_candidate"
TRAJECTORY_TOPIC = "/planning/portable_e2e/shadow_trajectory"
PATH_TOPIC = "/planning/portable_e2e/shadow_path"
REQUIRED_TOPICS = (
    STATUS_TOPIC,
    LATENCY_TOPIC,
    CANDIDATE_TOPIC,
    TRAJECTORY_TOPIC,
    PATH_TOPIC,
)
EXPECTED_OUTPUT_TOPICS = tuple(sorted(REQUIRED_TOPICS))
REJECTION_STAGES = (
    "image",
    "camera_info",
    "odometry",
    "acceleration",
    "steering",
    "inference",
)
CORE_COUNTER_FIELDS = (
    "anchor_attempt_count",
    "accepted_count",
    "anchor_rejected_count",
    "input_event_rejected_count",
)
SETTLE_COUNTER_FIELDS = (
    "input_settle_deferred_count",
    "input_settle_timeout_count",
)
CAMERA_BUNDLE_COUNTER_FIELDS = (
    "pending_bundle_count",
    "dropped_stale_count",
    "expired_pending_count",
    "evicted_capacity_count",
)
CAMERA_BUNDLE_CUMULATIVE_FIELDS = tuple(
    field for field in CAMERA_BUNDLE_COUNTER_FIELDS if field != "pending_bundle_count"
)
STATUS_BOUND_PROVENANCE_FIELDS = (
    "runtime_id",
    "model_id",
    "runtime_bundle_sha256",
    "source_checkpoint_sha256",
    "model_config_sha256",
    "corpus_fingerprint_sha256",
    "declared_map_id",
    "route_sha256",
)
TRIAL_PROVENANCE_FIELDS = (
    *tuple(
        field
        for field in STATUS_BOUND_PROVENANCE_FIELDS
        if field not in ("declared_map_id", "route_sha256")
    ),
    "aligned_route_sha256",
    "observed_map_id",
    "map_bundle_sha256",
    "runtime_gate_id",
    "runtime_gate",
)
SHA256_FIELDS = frozenset(
    (
        "runtime_bundle_sha256",
        "source_checkpoint_sha256",
        "model_config_sha256",
        "corpus_fingerprint_sha256",
        "route_sha256",
        "aligned_route_sha256",
        "map_bundle_sha256",
    )
)
SHA256_HEX = frozenset("0123456789abcdef")
TEN_HZ_MINIMUM_DURATION_S = 10.0
TEN_HZ_MINIMUM_ACCEPTED_COUNT = 101
TEN_HZ_MINIMUM_RATE_HZ = 9.5
TEN_HZ_MAXIMUM_RATE_HZ = 10.5
TEN_HZ_EXPECTED_ANCHOR_PERIOD_NS = 100_000_000
TEN_HZ_ANCHOR_PERIOD_TOLERANCE_NS = 5_000
LATENCY_DEADLINE_MS = 100.0
ACCEPTED_STATUS_MAXIMUM_GAP_S = 0.20
CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CAMPAIGN_RUNTIME_POLICY = {
    "maximum_tf_translation_error_m": 0.005,
    "maximum_tf_rotation_error_rad": 0.005,
    "input_settle_timeout_s": 0.05,
}
# HH_260906 - Pin evidence analysis to the complete current runtime gate contract.
EXPECTED_RUNTIME_GATE = asdict(RuntimeGateConfig())
STATUS_FIELDS = {
    "schema_id",
    "state",
    "stage",
    "failure_code",
    "healthy_now",
    "inference_inputs_healthy_now",
    "calibration_extrinsics_verified",
    "tf_extrinsic_parity",
    "health_scope",
    "anchor_timestamp_ns",
    "status_timestamp_ns",
    "status_wall_timestamp_ns",
    *CORE_COUNTER_FIELDS,
    *SETTLE_COUNTER_FIELDS,
    "measurement_armed",
    "measurement_sealed",
    "rejection_counts_by_stage",
    "last_complete_bundle_wall_age_ms",
    "camera_bundle_counters",
    "vehicle_control_approved",
    "output_topics",
    "provenance",
    "detail",
}


class EvidenceError(RuntimeError):
    """Raised when evidence cannot support an exact shadow-trial analysis."""


def _duplicate_rejecting_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise EvidenceError(f"duplicate JSON key: {key}")
        value[key] = item
    return value


def strict_json_loads(text: str, label: str) -> Any:
    """Decode JSON while rejecting duplicate keys and non-standard numbers."""

    def reject_constant(value: str) -> None:
        raise EvidenceError(f"{label} contains non-finite JSON number {value}")

    def parse_finite_float(value: str) -> float:
        parsed = float(value)
        if not math.isfinite(parsed):
            reject_constant(value)
        return parsed

    try:
        return json.loads(
            text,
            object_pairs_hook=_duplicate_rejecting_object,
            parse_constant=reject_constant,
            parse_float=parse_finite_float,
        )
    except EvidenceError:
        raise
    except json.JSONDecodeError as error:
        raise EvidenceError(f"cannot decode {label}: {error}") from error


def _read_strict_json(path: Path, label: str) -> Any:
    source = path.expanduser().absolute()
    if source.is_symlink() or not source.is_file():
        raise EvidenceError(f"{label} must be a regular non-symlink file: {source}")
    try:
        text = source.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise EvidenceError(f"cannot read {label} {source}: {error}") from error
    return strict_json_loads(text, label)


def _canonical_sha256(value: Any, label: str) -> str:
    try:
        payload = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise EvidenceError(f"cannot canonicalize {label}: {error}") from error
    return hashlib.sha256(payload).hexdigest()


def _validate_finite_tree(value: Any, label: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise EvidenceError(f"{label} contains NaN or Inf")
    if isinstance(value, Mapping):
        for key, item in value.items():
            if not isinstance(key, str):
                raise EvidenceError(f"{label} contains a non-string object key")
            _validate_finite_tree(item, f"{label}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _validate_finite_tree(item, f"{label}[{index}]")


def _exact_keys(value: Mapping[str, Any], expected: set[str], label: str) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise EvidenceError(
            f"{label} keys changed; missing={missing}, unexpected={unexpected}"
        )


def _nonnegative_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise EvidenceError(f"{label} must be a non-negative integer")
    return value


def _positive_integer(value: Any, label: str) -> int:
    result = _nonnegative_integer(value, label)
    if result == 0:
        raise EvidenceError(f"{label} must be positive")
    return result


def _finite_number(value: Any, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f"{label} must be numeric")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise EvidenceError(f"{label} must be finite") from error
    if not math.isfinite(result):
        raise EvidenceError(f"{label} must be finite")
    if positive and result <= 0.0:
        raise EvidenceError(f"{label} must be positive")
    return result


def _nonempty_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise EvidenceError(f"{label} must be a non-empty string")
    return value


def _sha256(value: Any, label: str) -> str:
    text = _nonempty_text(value, label)
    if len(text) != 64 or any(character not in SHA256_HEX for character in text):
        raise EvidenceError(f"{label} must be a lowercase SHA-256")
    return text


def _map_id(value: Any, label: str) -> str:
    text = _nonempty_text(value, label)
    if len(text) > 64 or any(
        character not in "abcdefghijklmnopqrstuvwxyz0123456789_"
        for character in text
    ):
        raise EvidenceError(f"{label} must be normalized lowercase ASCII")
    return text


def _validate_runtime_gate_provenance(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise EvidenceError(f"{label} must be an object")
    _exact_keys(value, set(EXPECTED_RUNTIME_GATE), label)
    for field, required in EXPECTED_RUNTIME_GATE.items():
        observed = value[field]
        if type(observed) is not type(required) or observed != required:
            raise EvidenceError(f"{label}.{field} is not contract-pinned")
    return dict(value)


def validate_trial_provenance(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise EvidenceError("trial_provenance must be an object")
    _exact_keys(value, set(TRIAL_PROVENANCE_FIELDS), "trial_provenance")
    result: dict[str, Any] = {}
    for field in TRIAL_PROVENANCE_FIELDS:
        if field == "runtime_gate_id":
            if value[field] != RUNTIME_GATE_ID:
                raise EvidenceError("trial_provenance runtime gate identity changed")
            result[field] = value[field]
        elif field == "runtime_gate":
            # HH_260906 - Bind the independent trial record to the complete current gate.
            result[field] = _validate_runtime_gate_provenance(
                value[field], "trial_provenance.runtime_gate"
            )
        elif field in SHA256_FIELDS:
            result[field] = _sha256(value[field], f"trial_provenance.{field}")
        elif field == "observed_map_id":
            result[field] = _map_id(value[field], f"trial_provenance.{field}")
        else:
            result[field] = _nonempty_text(
                value[field], f"trial_provenance.{field}"
            )
    return result


def _record_timestamp(record: Mapping[str, Any], label: str) -> int:
    return _positive_integer(record.get("bag_timestamp_ns"), f"{label}.bag_timestamp_ns")


def _validate_topic_records(topics: Any) -> dict[str, list[dict[str, Any]]]:
    if not isinstance(topics, Mapping):
        raise EvidenceError("topics must be an object")
    _exact_keys(topics, set(REQUIRED_TOPICS), "topics")
    normalized: dict[str, list[dict[str, Any]]] = {}
    record_keys = {
        STATUS_TOPIC: {"bag_timestamp_ns", "payload"},
        LATENCY_TOPIC: {"bag_timestamp_ns", "value"},
        CANDIDATE_TOPIC: {"bag_timestamp_ns", "value"},
        TRAJECTORY_TOPIC: {"bag_timestamp_ns", "header_timestamp_ns"},
        PATH_TOPIC: {"bag_timestamp_ns", "header_timestamp_ns"},
    }
    for topic in REQUIRED_TOPICS:
        records = topics[topic]
        if not isinstance(records, list) or not records:
            raise EvidenceError(f"required topic has no records: {topic}")
        copied: list[dict[str, Any]] = []
        previous_timestamp = -1
        for index, record in enumerate(records):
            label = f"topics[{topic!r}][{index}]"
            if not isinstance(record, Mapping):
                raise EvidenceError(f"{label} must be an object")
            _exact_keys(record, record_keys[topic], label)
            copied_record = dict(record)
            timestamp = _record_timestamp(copied_record, label)
            if timestamp <= previous_timestamp:
                raise EvidenceError(f"{topic} bag timestamps must be strictly increasing")
            previous_timestamp = timestamp
            if topic in (TRAJECTORY_TOPIC, PATH_TOPIC):
                _positive_integer(
                    copied_record["header_timestamp_ns"],
                    f"{label}.header_timestamp_ns",
                )
            copied.append(copied_record)
        normalized[topic] = copied
    return normalized


def validate_normalized_document(value: Any) -> dict[str, Any]:
    _validate_finite_tree(value, "normalized evidence")
    if not isinstance(value, Mapping):
        raise EvidenceError("normalized evidence must be an object")
    _exact_keys(
        value,
        {"schema_id", "trial_provenance", "window_boundaries", "topics"},
        "evidence",
    )
    if value["schema_id"] != NORMALIZED_SCHEMA_ID:
        raise EvidenceError("normalized evidence schema_id changed")
    boundaries = value["window_boundaries"]
    if not isinstance(boundaries, Mapping):
        raise EvidenceError("window_boundaries must be an object")
    _exact_keys(
        boundaries,
        {"schema_id", "armed_status", "startup_status", "final_status"},
        "window_boundaries",
    )
    if boundaries["schema_id"] != WINDOW_BOUNDARY_SCHEMA_ID:
        raise EvidenceError("window boundary schema_id changed")
    return {
        "schema_id": NORMALIZED_SCHEMA_ID,
        "trial_provenance": validate_trial_provenance(value["trial_provenance"]),
        "window_boundaries": {
            "schema_id": WINDOW_BOUNDARY_SCHEMA_ID,
            "armed_status": _status_payload(
                {"payload": boundaries["armed_status"]}, -3
            ),
            "startup_status": _status_payload(
                {"payload": boundaries["startup_status"]}, -2
            ),
            "final_status": _status_payload(
                {"payload": boundaries["final_status"]}, -1
            ),
        },
        "topics": _validate_topic_records(value["topics"]),
    }


def _status_payload(record: Mapping[str, Any], index: int) -> dict[str, Any]:
    payload = record["payload"]
    if isinstance(payload, str):
        payload = strict_json_loads(payload, f"status payload {index}")
    _validate_finite_tree(payload, f"status payload {index}")
    if not isinstance(payload, Mapping):
        raise EvidenceError(f"status payload {index} must be an object")
    return dict(payload)


def _resolve_status_window(
    topics: Mapping[str, Sequence[Mapping[str, Any]]],
    boundaries: Mapping[str, Any],
) -> tuple[int, int, dict[str, Any]]:
    status_records = topics[STATUS_TOPIC]
    status_hashes = [
        _canonical_sha256(_status_payload(record, index), f"status[{index}]")
        for index, record in enumerate(status_records)
    ]
    boundary_hashes = {
        label: _canonical_sha256(boundaries[f"{label}_status"], f"{label} status boundary")
        for label in ("armed", "startup", "final")
    }
    boundary_matches = {
        label: [
            index
            for index, digest in enumerate(status_hashes)
            if digest == boundary_hashes[label]
        ]
        for label in boundary_hashes
    }
    if any(len(matches) != 1 for matches in boundary_matches.values()):
        raise EvidenceError(
            "armed, startup, and final boundaries must each occur exactly once in the bag"
        )
    armed_index = boundary_matches["armed"][0]
    startup_index = boundary_matches["startup"][0]
    final_index = boundary_matches["final"][0]
    if not armed_index < startup_index < final_index:
        raise EvidenceError("status window boundaries are empty or reversed")
    if startup_index == 0:
        raise EvidenceError("startup boundary must follow a healthy semantic heartbeat")
    payloads = [
        _status_payload(record, index) for index, record in enumerate(status_records)
    ]
    fingerprints = [_status_semantic_fingerprint(payload) for payload in payloads]
    if final_index != len(status_records) - 1:
        raise EvidenceError("sealed final boundary must be the terminal status record")
    if any(payload.get("measurement_sealed") is not False for payload in payloads[:final_index]):
        raise EvidenceError("measurement was sealed before the final boundary")
    for label, index in (("startup", startup_index), ("final", final_index)):
        payload = payloads[index]
        previous = payloads[index - 1]
        if not _is_healthy_status(payload):
            raise EvidenceError(f"{label} boundary is not healthy SHADOW_OK evidence")
        if not _is_healthy_status(previous):
            raise EvidenceError(f"{label} boundary does not follow healthy SHADOW_OK")
        previous_wall_ns = _positive_integer(
            previous.get("status_wall_timestamp_ns"),
            f"{label} boundary preceding status_wall_timestamp_ns",
        )
        current_wall_ns = _positive_integer(
            payload.get("status_wall_timestamp_ns"),
            f"{label} boundary status_wall_timestamp_ns",
        )
        if current_wall_ns <= previous_wall_ns:
            raise EvidenceError(f"{label} semantic heartbeat wall time did not advance")
    if fingerprints[startup_index] != fingerprints[startup_index - 1]:
        raise EvidenceError("startup boundary does not repeat its prior outcome")
    if payloads[startup_index].get("measurement_sealed") is not False:
        raise EvidenceError("startup boundary must precede measurement sealing")
    if payloads[final_index].get("measurement_sealed") is not True:
        raise EvidenceError("final boundary must be the sealed terminal status")
    if _camera_bundle_counters(payloads[final_index], final_index)[
        "pending_bundle_count"
    ] != 0:
        raise EvidenceError("sealed final boundary has pending camera bundles")
    if _sealed_final_outcome_fingerprint(
        payloads[final_index]
    ) != _sealed_final_outcome_fingerprint(payloads[final_index - 1]):
        raise EvidenceError("final boundary does not repeat its prior outcome")
    startup_accepted_count = _positive_integer(
        payloads[startup_index].get("accepted_count"),
        "startup boundary accepted_count",
    )
    final_accepted_count = _positive_integer(
        payloads[final_index].get("accepted_count"),
        "final boundary accepted_count",
    )
    if final_accepted_count <= startup_accepted_count:
        raise EvidenceError("accepted lifetime ordinal window is empty")
    startup_bag_ns = status_records[startup_index]["bag_timestamp_ns"]
    final_bag_ns = status_records[final_index]["bag_timestamp_ns"]
    startup_wall_ns = payloads[startup_index]["status_wall_timestamp_ns"]
    final_wall_ns = payloads[final_index]["status_wall_timestamp_ns"]
    wall_duration_s = (final_wall_ns - startup_wall_ns) * 1.0e-9
    if wall_duration_s < TEN_HZ_MINIMUM_DURATION_S:
        raise EvidenceError("status boundary wall duration is shorter than 10 seconds")
    return (
        startup_index,
        final_index,
        {
            "schema_id": WINDOW_BOUNDARY_SCHEMA_ID,
            "selection_mode": "accepted_lifetime_ordinal",
            "armed_status_sha256": boundary_hashes["armed"],
            "startup_status_sha256": boundary_hashes["startup"],
            "final_status_sha256": boundary_hashes["final"],
            "armed_status_record_index": armed_index,
            "startup_status_record_index": startup_index,
            "final_status_record_index": final_index,
            "startup_status_bag_timestamp_ns": startup_bag_ns,
            "final_status_bag_timestamp_ns": final_bag_ns,
            "startup_status_wall_timestamp_ns": startup_wall_ns,
            "final_status_wall_timestamp_ns": final_wall_ns,
            "status_wall_duration_s": wall_duration_s,
            "startup_accepted_lifetime_ordinal": startup_accepted_count,
            "final_accepted_lifetime_ordinal": final_accepted_count,
            "selected_accepted_count": (
                final_accepted_count - startup_accepted_count
            ),
            "cross_topic_receipt_time_slicing_used": False,
            "healthy_startup_heartbeat_and_terminal_seal": True,
            "exact_unique_boundary_matches": True,
        },
    )


def _status_counters(payload: Mapping[str, Any], index: int) -> dict[str, int]:
    counters = {
        field: _nonnegative_integer(payload.get(field), f"status[{index}].{field}")
        for field in (*CORE_COUNTER_FIELDS, *SETTLE_COUNTER_FIELDS)
    }
    if counters["anchor_attempt_count"] != (
        counters["accepted_count"] + counters["anchor_rejected_count"]
    ):
        raise EvidenceError(f"status[{index}] violates the exact anchor invariant")
    if counters["input_settle_timeout_count"] > counters[
        "input_settle_deferred_count"
    ]:
        raise EvidenceError(f"status[{index}] settle timeout exceeds deferred count")
    return counters


def _stage_counters(payload: Mapping[str, Any], index: int) -> dict[str, int]:
    value = payload.get("rejection_counts_by_stage")
    if not isinstance(value, Mapping):
        raise EvidenceError(f"status[{index}].rejection_counts_by_stage must be an object")
    _exact_keys(value, set(REJECTION_STAGES), f"status[{index}].rejection_counts_by_stage")
    return {
        stage: _nonnegative_integer(value[stage], f"status[{index}].stage.{stage}")
        for stage in REJECTION_STAGES
    }


def _camera_bundle_counters(payload: Mapping[str, Any], index: int) -> dict[str, int]:
    value = payload.get("camera_bundle_counters")
    if not isinstance(value, Mapping):
        raise EvidenceError(f"status[{index}].camera_bundle_counters must be an object")
    _exact_keys(
        value,
        set(CAMERA_BUNDLE_COUNTER_FIELDS),
        f"status[{index}].camera_bundle_counters",
    )
    return {
        field: _nonnegative_integer(
            value[field], f"status[{index}].camera_bundle_counters.{field}"
        )
        for field in CAMERA_BUNDLE_COUNTER_FIELDS
    }


def _validate_status_health(payload: Mapping[str, Any], index: int) -> None:
    state = payload.get("state")
    if state not in ("SHADOW_WAITING", "SHADOW_OK", "SHADOW_REJECTED"):
        raise EvidenceError(f"status[{index}].state is invalid")
    inference_healthy = payload.get("inference_inputs_healthy_now")
    extrinsics_verified = payload.get("calibration_extrinsics_verified")
    full_healthy = payload.get("healthy_now")
    parity = payload.get("tf_extrinsic_parity")
    if not all(
        isinstance(value, bool)
        for value in (inference_healthy, extrinsics_verified, full_healthy)
    ):
        raise EvidenceError(f"status[{index}] health fields must be booleans")
    if inference_healthy != (state == "SHADOW_OK"):
        raise EvidenceError(f"status[{index}] inference-input health contradicts state")
    if parity not in ("UNVERIFIED_REQUIRED", "VERIFIED", "MISMATCH_REJECTED"):
        raise EvidenceError(f"status[{index}] has invalid TF extrinsic parity")
    if extrinsics_verified != (parity == "VERIFIED"):
        raise EvidenceError(f"status[{index}] extrinsic health contradicts parity")
    if state == "SHADOW_OK" and parity != "VERIFIED":
        raise EvidenceError(f"status[{index}] accepted without verified extrinsics")
    if full_healthy != (inference_healthy and extrinsics_verified):
        raise EvidenceError(f"status[{index}] full health is not fail-closed")
    measurement_sealed = payload.get("measurement_sealed")
    if not isinstance(measurement_sealed, bool):
        raise EvidenceError(f"status[{index}].measurement_sealed must be boolean")
    measurement_armed = payload.get("measurement_armed")
    if not isinstance(measurement_armed, bool):
        raise EvidenceError(f"status[{index}].measurement_armed must be boolean")
    if state == "SHADOW_OK" and not measurement_armed:
        raise EvidenceError(f"status[{index}] accepted while measurement was disarmed")
    if measurement_sealed and not full_healthy:
        raise EvidenceError(f"status[{index}] sealed an unhealthy measurement")
    if measurement_sealed and not measurement_armed:
        raise EvidenceError(f"status[{index}] sealed a disarmed measurement")
    if payload.get("vehicle_control_approved") is not False:
        raise EvidenceError(f"status[{index}] attempted to grant vehicle control")
    if payload.get("output_topics") != list(EXPECTED_OUTPUT_TOPICS):
        raise EvidenceError(f"status[{index}] shadow output-topic allowlist changed")
    if payload.get("health_scope") != "full_runtime_requires_camera_extrinsic_parity":
        raise EvidenceError(f"status[{index}] health scope changed")
    bundle_age_ms = payload.get("last_complete_bundle_wall_age_ms")
    if bundle_age_ms is not None:
        parsed_bundle_age_ms = _finite_number(
            bundle_age_ms, f"status[{index}].last_complete_bundle_wall_age_ms"
        )
        if parsed_bundle_age_ms < 0.0:
            raise EvidenceError(f"status[{index}] bundle wall age is negative")
    elif state == "SHADOW_OK":
        raise EvidenceError(f"status[{index}] healthy state lacks bundle wall age")
    _nonnegative_integer(
        payload.get("status_timestamp_ns"), f"status[{index}].status_timestamp_ns"
    )
    _positive_integer(
        payload.get("status_wall_timestamp_ns"),
        f"status[{index}].status_wall_timestamp_ns",
    )


def _is_healthy_status(payload: Mapping[str, Any]) -> bool:
    return (
        payload.get("state") == "SHADOW_OK"
        and payload.get("stage") == "inference"
        and payload.get("failure_code") == "none"
        and payload.get("healthy_now") is True
        and payload.get("inference_inputs_healthy_now") is True
        and payload.get("calibration_extrinsics_verified") is True
        and payload.get("tf_extrinsic_parity") == "VERIFIED"
        and payload.get("vehicle_control_approved") is False
    )


def _validate_status_provenance(
    payload: Mapping[str, Any],
    index: int,
    expected: Mapping[str, str],
) -> dict[str, Any]:
    value = payload.get("provenance")
    if not isinstance(value, Mapping):
        raise EvidenceError(f"status[{index}].provenance must be an object")
    for field in STATUS_BOUND_PROVENANCE_FIELDS:
        observed = value.get(field)
        if field in SHA256_FIELDS:
            observed = _sha256(observed, f"status[{index}].provenance.{field}")
        elif field == "declared_map_id":
            observed = _map_id(
                observed, f"status[{index}].provenance.{field}"
            )
        else:
            observed = _nonempty_text(
                observed, f"status[{index}].provenance.{field}"
            )
        expected_field = {
            "declared_map_id": "observed_map_id",
            "route_sha256": "aligned_route_sha256",
        }.get(field, field)
        if observed != expected[expected_field]:
            raise EvidenceError(f"status[{index}] {field} does not match trial provenance")
    if value.get("tf_extrinsic_policy") != "live_base_link_to_optical_at_image_anchor":
        raise EvidenceError(f"status[{index}] TF extrinsic policy changed")
    if value.get("adapter_id") != ADAPTER_ID:
        raise EvidenceError(f"status[{index}] shadow adapter identity changed")
    if value.get("health_scope") != "full_runtime_requires_camera_extrinsics":
        raise EvidenceError(f"status[{index}] provenance health scope changed")
    if value.get("runtime_gate_id") != RUNTIME_GATE_ID:
        raise EvidenceError(f"status[{index}] runtime gate identity changed")
    _validate_runtime_gate_provenance(
        value.get("runtime_gate"),
        f"status[{index}].provenance.runtime_gate",
    )
    runtime_policy = value.get("runtime_policy")
    if not isinstance(runtime_policy, Mapping):
        raise EvidenceError(f"status[{index}] runtime policy is missing")
    for field, required in CAMPAIGN_RUNTIME_POLICY.items():
        observed = _finite_number(
            runtime_policy.get(field),
            f"status[{index}].provenance.runtime_policy.{field}",
            positive=True,
        )
        if observed != required:
            raise EvidenceError(
                f"status[{index}] runtime policy {field} is not campaign-pinned"
            )
    execution = value.get("runtime_execution_policy")
    expected_execution_fields = {
        "policy_id",
        "torch_intraop_threads",
        "torch_interop_threads",
        "cpu_affinity",
    }
    if not isinstance(execution, Mapping) or set(execution) != expected_execution_fields:
        raise EvidenceError(f"status[{index}] runtime execution policy is malformed")
    affinity = execution.get("cpu_affinity")
    device = value.get("runtime_device")
    if device not in ("cpu", "cuda:0"):
        raise EvidenceError(f"status[{index}] runtime device is invalid")
    expected_policy_id = (
        "portable_e2e.cpu_execution.v1"
        if device == "cpu"
        else "portable_e2e.cuda_execution.v1"
    )
    if (
        execution.get("policy_id") != expected_policy_id
        or type(execution.get("torch_intraop_threads")) is not int
        or execution["torch_intraop_threads"] <= 0
        or type(execution.get("torch_interop_threads")) is not int
        or execution["torch_interop_threads"] <= 0
        or not isinstance(affinity, list)
        or not affinity
        or any(type(cpu) is not int or cpu < 0 for cpu in affinity)
        or affinity != sorted(set(affinity))
    ):
        raise EvidenceError(f"status[{index}] runtime execution policy changed")
    if device == "cpu" and (
        execution["torch_intraop_threads"] != 4
        or execution["torch_interop_threads"] != 1
    ):
        raise EvidenceError(f"status[{index}] runtime CPU thread policy changed")
    return dict(value)


def _status_semantic_fingerprint(payload: Mapping[str, Any]) -> str:
    value = dict(payload)
    value.pop("status_timestamp_ns", None)
    value.pop("status_wall_timestamp_ns", None)
    value.pop("last_complete_bundle_wall_age_ms", None)
    value.pop("measurement_sealed", None)
    return _canonical_sha256(value, "status heartbeat")


def _sealed_final_outcome_fingerprint(payload: Mapping[str, Any]) -> str:
    value = dict(payload)
    for field in (
        "status_timestamp_ns",
        "status_wall_timestamp_ns",
        "last_complete_bundle_wall_age_ms",
        "measurement_sealed",
        "camera_bundle_counters",
        "input_settle_deferred_count",
        "input_settle_timeout_count",
    ):
        value.pop(field, None)
    return _canonical_sha256(value, "sealed final stable outcome")


def _accepted_tf_extrinsic_errors(
    detail: Mapping[str, Any],
    index: int,
    runtime_policy: Mapping[str, Any],
) -> tuple[float, float]:
    value = detail.get("tf_extrinsic_errors")
    if not isinstance(value, Mapping):
        raise EvidenceError(
            f"status[{index}].detail.tf_extrinsic_errors must be an object"
        )
    _exact_keys(
        value,
        set(CAMERA_ORDER),
        f"status[{index}].detail.tf_extrinsic_errors",
    )
    maximum_translation = 0.0
    maximum_rotation = 0.0
    for camera in CAMERA_ORDER:
        errors = value[camera]
        label = f"status[{index}].detail.tf_extrinsic_errors.{camera}"
        if not isinstance(errors, Mapping):
            raise EvidenceError(f"{label} must be an object")
        _exact_keys(
            errors,
            {"translation_error_m", "rotation_error_rad"},
            label,
        )
        translation = _finite_number(
            errors["translation_error_m"], f"{label}.translation_error_m"
        )
        rotation = _finite_number(
            errors["rotation_error_rad"], f"{label}.rotation_error_rad"
        )
        if translation < 0.0 or rotation < 0.0:
            raise EvidenceError(f"{label} errors must be non-negative")
        if translation > runtime_policy["maximum_tf_translation_error_m"]:
            raise EvidenceError(f"{label} translation error exceeds runtime policy")
        if rotation > runtime_policy["maximum_tf_rotation_error_rad"]:
            raise EvidenceError(f"{label} rotation error exceeds runtime policy")
        maximum_translation = max(maximum_translation, translation)
        maximum_rotation = max(maximum_rotation, rotation)
    return maximum_translation, maximum_rotation


def analyze_status_records(
    records: Sequence[Mapping[str, Any]],
    trial_provenance: Mapping[str, str],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    payloads = [_status_payload(record, index) for index, record in enumerate(records)]
    counters = [_status_counters(payload, index) for index, payload in enumerate(payloads)]
    stages = [_stage_counters(payload, index) for index, payload in enumerate(payloads)]
    bundle_counters = [
        _camera_bundle_counters(payload, index)
        for index, payload in enumerate(payloads)
    ]
    provenances: list[dict[str, Any]] = []
    fingerprints: list[str] = []
    for index, payload in enumerate(payloads):
        _exact_keys(payload, STATUS_FIELDS, f"status[{index}]")
        if payload.get("schema_id") != STATUS_SCHEMA_ID:
            raise EvidenceError(f"status[{index}] schema_id changed")
        _validate_status_health(payload, index)
        provenances.append(
            _validate_status_provenance(payload, index, trial_provenance)
        )
        fingerprints.append(_status_semantic_fingerprint(payload))
    canonical_provenance = _canonical_sha256(provenances[0], "status provenance")
    if any(
        _canonical_sha256(value, "status provenance") != canonical_provenance
        for value in provenances[1:]
    ):
        raise EvidenceError("status provenance changed during the recording")
    baseline = counters[0]
    baseline_stages = stages[0]
    baseline_bundle_counters = bundle_counters[0]

    accepted_events: list[dict[str, Any]] = []
    rejection_reasons: Counter[str] = Counter()
    anchor_rejection_reasons: Counter[str] = Counter()
    input_rejection_reasons: Counter[str] = Counter()
    heartbeat_count = 0
    state_transition_count = 0
    baseline_anchor = payloads[0].get("anchor_timestamp_ns")
    previous_anchor_ns = (
        _positive_integer(baseline_anchor, "startup boundary anchor_timestamp_ns")
        if baseline_anchor is not None
        else -1
    )
    for index in range(1, len(payloads)):
        counter_delta = {
            field: counters[index][field] - counters[index - 1][field]
            for field in counters[index]
        }
        stage_delta = {
            stage: stages[index][stage] - stages[index - 1][stage]
            for stage in REJECTION_STAGES
        }
        bundle_delta = {
            field: bundle_counters[index][field] - bundle_counters[index - 1][field]
            for field in CAMERA_BUNDLE_COUNTER_FIELDS
        }
        if any(value < 0 for value in (*counter_delta.values(), *stage_delta.values())):
            raise EvidenceError(f"status counters decreased at record {index}")
        if any(
            bundle_delta[field] < 0
            for field in CAMERA_BUNDLE_CUMULATIVE_FIELDS
        ):
            raise EvidenceError(f"camera-bundle counters decreased at record {index}")
        if any(value > 1 for value in counter_delta.values()) or sum(stage_delta.values()) > 1:
            raise EvidenceError(f"status transition {index} skipped evidence events")
        if counter_delta["anchor_attempt_count"] != (
            counter_delta["accepted_count"] + counter_delta["anchor_rejected_count"]
        ):
            raise EvidenceError(f"status transition {index} violates anchor accounting")
        event_count = (
            counter_delta["accepted_count"]
            + counter_delta["anchor_rejected_count"]
            + counter_delta["input_event_rejected_count"]
        )
        if event_count > 1:
            raise EvidenceError(f"status transition {index} combines multiple outcomes")
        stage_event_count = sum(stage_delta.values())
        rejected_delta = (
            counter_delta["anchor_rejected_count"]
            + counter_delta["input_event_rejected_count"]
        )
        if stage_event_count != rejected_delta:
            raise EvidenceError(f"status transition {index} loses a rejection stage")
        payload = payloads[index]
        if event_count == 0:
            if fingerprints[index] == fingerprints[index - 1]:
                heartbeat_count += 1
            else:
                state_transition_count += 1
            continue
        state = payload["state"]
        anchor = payload.get("anchor_timestamp_ns")
        if counter_delta["accepted_count"] == 1:
            if state != "SHADOW_OK":
                raise EvidenceError(f"accepted transition {index} is not SHADOW_OK")
            anchor_ns = _positive_integer(anchor, f"status[{index}].anchor_timestamp_ns")
            if anchor_ns <= previous_anchor_ns:
                raise EvidenceError("accepted/rejected anchor timestamps must increase")
            previous_anchor_ns = anchor_ns
            detail = payload.get("detail")
            if not isinstance(detail, Mapping):
                raise EvidenceError(f"status[{index}].detail must be an object")
            if detail.get("failure_code") != "none":
                raise EvidenceError(f"accepted status[{index}] has a failure code")
            if detail.get("tf_extrinsic_parity") != "VERIFIED":
                raise EvidenceError(
                    f"accepted status[{index}] detail lacks verified TF parity"
                )
            if payload.get("failure_code") != "none" or payload.get("stage") != "inference":
                raise EvidenceError(f"accepted status[{index}] top-level outcome changed")
            candidate = _nonnegative_integer(
                detail.get("candidate"), f"status[{index}].detail.candidate"
            )
            if candidate > 5:
                raise EvidenceError(f"status[{index}] candidate index exceeds Common10 ABI")
            latency_ms = _finite_number(
                detail.get("latency_ms"),
                f"status[{index}].detail.latency_ms",
                positive=True,
            )
            if detail.get("trajectory_point_count") != 64:
                raise EvidenceError(f"status[{index}] trajectory point count changed")
            measurement_anchor_floor_ns = _nonnegative_integer(
                detail.get("measurement_anchor_floor_ns"),
                f"status[{index}].detail.measurement_anchor_floor_ns",
            )
            if anchor_ns <= measurement_anchor_floor_ns:
                raise EvidenceError(
                    f"status[{index}] anchor does not follow the measurement floor"
                )
            preboundary_filtered_input_count = _nonnegative_integer(
                detail.get("preboundary_filtered_input_count"),
                f"status[{index}].detail.preboundary_filtered_input_count",
            )
            maximum_tf_translation_error_m, maximum_tf_rotation_error_rad = (
                _accepted_tf_extrinsic_errors(
                    detail,
                    index,
                    provenances[index]["runtime_policy"],
                )
            )
            accepted_events.append(
                {
                    "anchor_timestamp_ns": anchor_ns,
                    "candidate": candidate,
                    "latency_ms": latency_ms,
                    "status_bag_timestamp_ns": records[index]["bag_timestamp_ns"],
                    "status_wall_timestamp_ns": payload[
                        "status_wall_timestamp_ns"
                    ],
                    "inference_inputs_healthy": payload[
                        "inference_inputs_healthy_now"
                    ],
                    "full_healthy": payload["healthy_now"],
                    "extrinsics_verified": payload[
                        "calibration_extrinsics_verified"
                    ],
                    "maximum_tf_translation_error_m": (
                        maximum_tf_translation_error_m
                    ),
                    "maximum_tf_rotation_error_rad": maximum_tf_rotation_error_rad,
                    "measurement_anchor_floor_ns": measurement_anchor_floor_ns,
                    "preboundary_filtered_input_count": (
                        preboundary_filtered_input_count
                    ),
                }
            )
        else:
            if state != "SHADOW_REJECTED":
                raise EvidenceError(f"rejection transition {index} is not SHADOW_REJECTED")
            if counter_delta["anchor_rejected_count"] == 1:
                anchor_ns = _positive_integer(
                    anchor, f"status[{index}].anchor_timestamp_ns"
                )
                if anchor_ns <= previous_anchor_ns:
                    raise EvidenceError("accepted/rejected anchor timestamps must increase")
                previous_anchor_ns = anchor_ns
            elif anchor is not None:
                raise EvidenceError(f"input-event rejection {index} must not claim an anchor")
            failure_code = _nonempty_text(
                payload.get("failure_code"), f"status[{index}].failure_code"
            )
            if failure_code == "none":
                raise EvidenceError(f"rejection status[{index}] has no failure reason")
            stage = payload.get("stage")
            changed_stages = [name for name, delta in stage_delta.items() if delta == 1]
            if changed_stages != [stage]:
                raise EvidenceError(f"rejection status[{index}] stage counter disagrees")
            detail = payload.get("detail")
            if not isinstance(detail, Mapping):
                raise EvidenceError(f"rejection status[{index}] lacks reason detail")
            if detail.get("failure_code") != failure_code or detail.get("stage") != stage:
                raise EvidenceError(f"rejection status[{index}] detail contradicts outcome")
            _nonempty_text(detail.get("reason"), f"status[{index}].detail.reason")
            if counter_delta["anchor_rejected_count"] == 1 and stage != "inference":
                raise EvidenceError(f"anchor rejection status[{index}] must be inference stage")
            if counter_delta["input_event_rejected_count"] == 1 and stage == "inference":
                raise EvidenceError(f"input rejection status[{index}] cannot be inference stage")
            rejection_reasons[failure_code] += 1
            if counter_delta["anchor_rejected_count"] == 1:
                anchor_rejection_reasons[failure_code] += 1
            else:
                input_rejection_reasons[failure_code] += 1

    final = counters[-1]
    final_stages = stages[-1]
    window_denominators = {
        field: final[field] - baseline[field] for field in CORE_COUNTER_FIELDS
    }
    window_stages = {
        stage: final_stages[stage] - baseline_stages[stage]
        for stage in REJECTION_STAGES
    }
    if sum(window_stages.values()) != (
        window_denominators["anchor_rejected_count"]
        + window_denominators["input_event_rejected_count"]
    ):
        raise EvidenceError("bounded rejection-stage denominator is incomplete")
    if len(accepted_events) != window_denominators["accepted_count"]:
        raise EvidenceError("accepted status events do not equal the bounded denominator")
    if sum(rejection_reasons.values()) != (
        window_denominators["anchor_rejected_count"]
        + window_denominators["input_event_rejected_count"]
    ):
        raise EvidenceError("rejection reasons do not equal the bounded denominator")
    terminal_heartbeat = len(payloads) >= 2 and fingerprints[-1] == fingerprints[-2]
    if not accepted_events:
        raise EvidenceError("bounded window contains no accepted shadow inference")
    map_runtime_status_bound = all(
        provenance.get("declared_map_id") == trial_provenance["observed_map_id"]
        and provenance.get("route_sha256")
        == trial_provenance["aligned_route_sha256"]
        for provenance in provenances
    )
    anchor_attempts = window_denominators["anchor_attempt_count"]
    return (
        {
            "status_message_count": len(payloads),
            "heartbeat_count": heartbeat_count,
            "state_only_transition_count": state_transition_count,
            "terminal_exact_heartbeat_observed": terminal_heartbeat,
            "exact_anchor_invariant": True,
            "complete_increment_sequence": True,
            "denominators": window_denominators,
            "lifetime_counters_at_start": {
                field: baseline[field] for field in CORE_COUNTER_FIELDS
            },
            "lifetime_counters_at_end": {
                field: final[field] for field in CORE_COUNTER_FIELDS
            },
            "input_settle": {
                field: final[field] - baseline[field]
                for field in SETTLE_COUNTER_FIELDS
            },
            "anchor_outcomes": {
                "attempted_count": anchor_attempts,
                "accepted_count": window_denominators["accepted_count"],
                "rejected_count": window_denominators["anchor_rejected_count"],
                "accepted_percent": (
                    100.0 * window_denominators["accepted_count"] / anchor_attempts
                    if anchor_attempts
                    else 0.0
                ),
                "rejected_percent": (
                    100.0
                    * window_denominators["anchor_rejected_count"]
                    / anchor_attempts
                    if anchor_attempts
                    else 0.0
                ),
            },
            "rejection_counts_by_stage": window_stages,
            "rejection_reason_histogram": dict(sorted(rejection_reasons.items())),
            "anchor_rejection_reason_histogram": dict(
                sorted(anchor_rejection_reasons.items())
            ),
            "input_event_rejection_reason_histogram": dict(
                sorted(input_rejection_reasons.items())
            ),
            "map_runtime_status_bound": map_runtime_status_bound,
            "camera_bundle_cumulative_delta": {
                field: bundle_counters[-1][field]
                - baseline_bundle_counters[field]
                for field in CAMERA_BUNDLE_CUMULATIVE_FIELDS
            },
            "pending_bundle_net_change": (
                bundle_counters[-1]["pending_bundle_count"]
                - baseline_bundle_counters["pending_bundle_count"]
            ),
            "maximum_observed_pending_bundle_count": max(
                value["pending_bundle_count"] for value in bundle_counters
            ),
            "camera_bundle_counters_at_start": dict(baseline_bundle_counters),
            "camera_bundle_counters_at_end": dict(bundle_counters[-1]),
            "state_histogram": dict(
                sorted(Counter(payload["state"] for payload in payloads).items())
            ),
            "tf_extrinsic_parity_histogram": dict(
                sorted(
                    Counter(
                        payload["tf_extrinsic_parity"] for payload in payloads
                    ).items()
                )
            ),
            "health_snapshot_counts": {
                "total": len(payloads),
                "healthy_shadow_ok_verified": sum(
                    _is_healthy_status(payload) for payload in payloads
                ),
                "nonhealthy_or_non_ok": sum(
                    not _is_healthy_status(payload) for payload in payloads
                ),
                "waiting": sum(
                    payload["state"] == "SHADOW_WAITING" for payload in payloads
                ),
                "rejected": sum(
                    payload["state"] == "SHADOW_REJECTED" for payload in payloads
                ),
                "camera_bundle_timeout": sum(
                    payload["failure_code"] == "camera_bundle_timeout"
                    for payload in payloads
                ),
                "inference_inputs_healthy": sum(
                    payload["inference_inputs_healthy_now"] for payload in payloads
                ),
                "full_runtime_healthy": sum(
                    payload["healthy_now"] for payload in payloads
                ),
                "calibration_extrinsics_verified": sum(
                    payload["calibration_extrinsics_verified"]
                    for payload in payloads
                ),
                "extrinsic_blocked_after_inference_inputs": sum(
                    payload["inference_inputs_healthy_now"]
                    and not payload["healthy_now"]
                    for payload in payloads
                ),
                "all_statuses_healthy_shadow_ok_verified": all(
                    _is_healthy_status(payload) for payload in payloads
                ),
            },
        },
        accepted_events,
    )


def _require_complete_lifetime_prefix(
    records: Sequence[Mapping[str, Any]],
    lifetime_status: Mapping[str, Any],
    expected_arm_index: int,
    startup_index: int,
) -> dict[str, Any]:
    payloads = [_status_payload(record, index) for index, record in enumerate(records)]
    armed_transitions = [
        index
        for index, payload in enumerate(payloads)
        if payload.get("measurement_armed") is True
        and (
            index == 0
            or payloads[index - 1].get("measurement_armed") is False
        )
    ]
    if len(armed_transitions) != 1:
        raise EvidenceError("recording must contain one exact measurement-arm transition")
    arm_index = armed_transitions[0]
    if arm_index != expected_arm_index:
        raise EvidenceError("armed boundary is not the measurement-arm transition")
    if any(
        payload.get("measurement_armed") is not False
        for payload in payloads[:arm_index]
    ) or any(
        payload.get("measurement_armed") is not True
        for payload in payloads[arm_index:]
    ):
        raise EvidenceError("measurement armed state is not monotonic")
    armed = payloads[arm_index]
    if armed.get("state") != "SHADOW_WAITING" or armed.get("anchor_timestamp_ns") is not None:
        raise EvidenceError("measurement-arm status is not a waiting zero baseline")
    if (
        armed.get("stage") != "measurement"
        or armed.get("failure_code") != "waiting_for_inputs"
    ):
        raise EvidenceError("measurement-arm status outcome identity changed")
    armed_detail = armed.get("detail")
    if not isinstance(armed_detail, Mapping) or (
        armed_detail.get("stage") != "measurement"
        or armed_detail.get("failure_code") != "waiting_for_inputs"
    ):
        raise EvidenceError("measurement-arm status detail identity changed")
    _nonempty_text(
        armed_detail.get("reason"), "measurement-arm status detail reason"
    )
    measurement_anchor_floor_ns = _nonnegative_integer(
        armed_detail.get("measurement_anchor_floor_ns"),
        "measurement-arm status detail measurement_anchor_floor_ns",
    )
    if armed_detail.get("measurement_anchor_policy") != (
        "strictly_after_arm_source_time"
    ):
        raise EvidenceError("measurement-arm source-time policy changed")
    if measurement_anchor_floor_ns > _nonnegative_integer(
        armed.get("status_timestamp_ns"),
        "measurement-arm status timestamp",
    ):
        raise EvidenceError("measurement-arm floor is later than its status timestamp")
    # HH_260906 - Prove every measured camera anchor follows the atomic arm-time floor.
    for index, payload in enumerate(payloads[arm_index + 1:], start=arm_index + 1):
        anchor_ns = payload.get("anchor_timestamp_ns")
        if anchor_ns is not None and _positive_integer(
            anchor_ns,
            f"status[{index}].anchor_timestamp_ns",
        ) <= measurement_anchor_floor_ns:
            raise EvidenceError("measured anchor does not follow the arm-time floor")
    counters = _status_counters(armed, arm_index)
    if any(
        counters[field] != 0
        for field in (*CORE_COUNTER_FIELDS, *SETTLE_COUNTER_FIELDS)
    ):
        raise EvidenceError("measurement-arm status lacks the zero-count lifetime prefix")
    if any(value != 0 for value in _stage_counters(armed, arm_index).values()):
        raise EvidenceError("measurement-arm status lacks zero rejection stages")
    if any(value != 0 for value in _camera_bundle_counters(armed, arm_index).values()):
        raise EvidenceError("measurement-arm status lacks zero camera-bundle counters")
    if armed.get("measurement_sealed") is not False:
        raise EvidenceError("measurement was sealed at its arm boundary")
    for index, payload in enumerate(payloads[:arm_index]):
        if (
            payload.get("state") != "SHADOW_WAITING"
            or payload.get("anchor_timestamp_ns") is not None
            or payload.get("measurement_sealed") is not False
            or any(value != 0 for value in _status_counters(payload, index).values())
            or any(value != 0 for value in _stage_counters(payload, index).values())
            or any(
                value != 0
                for value in _camera_bundle_counters(payload, index).values()
            )
        ):
            raise EvidenceError("status outcomes occurred before measurement arm")
    pre_startup_waiting_count = 0
    pre_startup_disallowed_waiting_count = 0
    pre_startup_healthy_count = 0
    pre_startup_rejected_count = 0
    healthy_seen = False
    armed_waiting_fingerprint = _sealed_final_outcome_fingerprint(armed)
    for index, payload in enumerate(
        payloads[arm_index + 1:startup_index], start=arm_index + 1
    ):
        if payload.get("state") == "SHADOW_WAITING":
            if not healthy_seen and (
                _sealed_final_outcome_fingerprint(payload)
                == armed_waiting_fingerprint
            ):
                pre_startup_waiting_count += 1
            else:
                pre_startup_disallowed_waiting_count += 1
        elif _is_healthy_status(payload):
            healthy_seen = True
            pre_startup_healthy_count += 1
        else:
            pre_startup_rejected_count += 1
    startup = payloads[startup_index]
    startup_counters = _status_counters(startup, startup_index)
    startup_stages = _stage_counters(startup, startup_index)
    startup_bundle_counters = _camera_bundle_counters(startup, startup_index)
    pre_startup_rejection_count = (
        startup_counters["anchor_rejected_count"]
        + startup_counters["input_event_rejected_count"]
    )
    pre_startup_drop_count = sum(
        startup_bundle_counters[field]
        for field in CAMERA_BUNDLE_CUMULATIVE_FIELDS
    )
    pre_startup_timeout_count = startup_counters["input_settle_timeout_count"]
    pre_startup_clean = (
        pre_startup_disallowed_waiting_count == 0
        and pre_startup_rejected_count == 0
        and pre_startup_rejection_count == 0
        and pre_startup_drop_count == 0
        and pre_startup_timeout_count == 0
        and not any(startup_stages.values())
    )
    if lifetime_status["lifetime_counters_at_start"] != {
        field: 0 for field in CORE_COUNTER_FIELDS
    }:
        raise EvidenceError("status lifetime prefix is not complete")
    return {
        "measurement_arm_status_record_index": arm_index,
        "measurement_arm_status_sha256": _canonical_sha256(
            armed, "measurement-arm status"
        ),
        "measurement_arm_zero_baseline_verified": True,
        "measurement_anchor_floor_ns": measurement_anchor_floor_ns,
        "all_measured_anchors_follow_arm_floor": True,
        "pre_startup_initial_waiting_heartbeat_count": (
            pre_startup_waiting_count
        ),
        "pre_startup_disallowed_waiting_status_count": (
            pre_startup_disallowed_waiting_count
        ),
        "pre_startup_healthy_status_count": pre_startup_healthy_count,
        "pre_startup_rejected_status_count": pre_startup_rejected_count,
        "pre_startup_rejection_count": pre_startup_rejection_count,
        "pre_startup_drop_count": pre_startup_drop_count,
        "pre_startup_timeout_count": pre_startup_timeout_count,
        "pre_startup_only_initial_waiting_and_zero_losses": pre_startup_clean,
    }


def _select_accepted_ordinal_outputs(
    topics: Mapping[str, Sequence[Mapping[str, Any]]],
    accepted_events: Sequence[Mapping[str, Any]],
    startup_accepted_count: int,
    final_accepted_count: int,
) -> tuple[dict[str, list[dict[str, Any]]], dict[str, Any]]:
    if len(accepted_events) != final_accepted_count:
        raise EvidenceError("accepted status lifetime prefix is incomplete")
    count_by_topic = {
        topic: len(topics[topic])
        for topic in REQUIRED_TOPICS
        if topic != STATUS_TOPIC
    }
    if any(count != final_accepted_count for count in count_by_topic.values()):
        raise EvidenceError(
            "accepted lifetime denominator does not equal complete output-topic prefixes"
        )

    latencies = [
        _finite_number(record["value"], f"lifetime latency[{index}]", positive=True)
        for index, record in enumerate(topics[LATENCY_TOPIC])
    ]
    candidates = [
        _nonnegative_integer(record["value"], f"lifetime candidate[{index}]")
        for index, record in enumerate(topics[CANDIDATE_TOPIC])
    ]
    if any(candidate > 5 for candidate in candidates):
        raise EvidenceError("selected candidate index exceeds the Common10 ABI")
    accepted_anchors = [event["anchor_timestamp_ns"] for event in accepted_events]
    trajectory_anchors = [
        record["header_timestamp_ns"] for record in topics[TRAJECTORY_TOPIC]
    ]
    path_anchors = [record["header_timestamp_ns"] for record in topics[PATH_TOPIC]]
    if trajectory_anchors != accepted_anchors or path_anchors != accepted_anchors:
        raise EvidenceError(
            "complete trajectory/path prefixes do not match accepted status ordinals"
        )
    for index, event in enumerate(accepted_events):
        if candidates[index] != event["candidate"]:
            raise EvidenceError(
                f"candidate topic disagrees with accepted lifetime ordinal {index + 1}"
            )
        if not math.isclose(
            latencies[index],
            event["latency_ms"],
            rel_tol=1.0e-6,
            abs_tol=1.0e-3,
        ):
            raise EvidenceError(
                f"latency topic disagrees with accepted lifetime ordinal {index + 1}"
            )

    selected: dict[str, list[dict[str, Any]]] = {}
    for topic in REQUIRED_TOPICS:
        if topic == STATUS_TOPIC:
            continue
        selected[topic] = [
            dict(record)
            for record in topics[topic][startup_accepted_count:final_accepted_count]
        ]
        if len(selected[topic]) != final_accepted_count - startup_accepted_count:
            raise EvidenceError(f"accepted ordinal selection is partial for {topic}")
    return (
        selected,
        {
            "complete_lifetime_prefix_verified": True,
            "accepted_status_event_count": len(accepted_events),
            "output_topic_lifetime_counts": count_by_topic,
            "publisher_order_scalar_status_match": True,
            "publisher_order_header_anchor_match": True,
            "rejected_missing_dropped_or_extra_records": True,
        },
    )


def _latency_summary(values: Sequence[float]) -> dict[str, Any]:
    if not values:
        raise EvidenceError("accepted latency denominator is empty")
    return {
        "count": len(values),
        "p50_ms": percentile(values, 0.50),
        "p95_ms": percentile(values, 0.95),
        "p99_ms": percentile(values, 0.99),
        "maximum_ms": max(values),
        "deadline_ms": LATENCY_DEADLINE_MS,
        "count_over_100ms": sum(value > LATENCY_DEADLINE_MS for value in values),
    }


def _message_timing(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    result = sequence_timing([record["bag_timestamp_ns"] for record in records])
    return {
        "count": result["count"],
        "duration_s": result["duration_s"],
        "rate_hz": result["rate_hz"],
        "p95_gap_ms": result["p95_gap_ms"],
        "p99_gap_ms": result["p99_gap_ms"],
        "maximum_gap_ms": result["maximum_gap_ms"],
    }


def _accepted_status_wall_timing(
    accepted_events: Sequence[Mapping[str, Any]],
    startup_wall_timestamp_ns: int,
    final_wall_timestamp_ns: int,
) -> dict[str, Any]:
    accepted_wall_timestamps_ns = [
        event["status_wall_timestamp_ns"] for event in accepted_events
    ]
    continuity_timestamps_ns = [
        startup_wall_timestamp_ns,
        *accepted_wall_timestamps_ns,
        final_wall_timestamp_ns,
    ]
    result = sequence_timing(continuity_timestamps_ns)
    if result["duplicate_or_nonmonotonic_count"] != 0:
        raise EvidenceError(
            "startup, accepted, and sealed-final status wall timestamps are not "
            "strictly increasing"
        )
    accepted_result = sequence_timing(accepted_wall_timestamps_ns)
    maximum_gap_ms = result["maximum_gap_ms"]
    return {
        "accepted_count": len(accepted_wall_timestamps_ns),
        "boundary_inclusive_timestamp_count": result["count"],
        "boundary_inclusive_gap_count": result["count"] - 1,
        "boundary_inclusive_duration_s": result["duration_s"],
        "accepted_outcome_duration_s": accepted_result["duration_s"],
        "accepted_outcome_rate_hz": accepted_result["rate_hz"],
        "startup_to_first_accepted_gap_s": (
            accepted_wall_timestamps_ns[0] - startup_wall_timestamp_ns
        )
        * 1.0e-9,
        "last_accepted_to_sealed_final_gap_s": (
            final_wall_timestamp_ns - accepted_wall_timestamps_ns[-1]
        )
        * 1.0e-9,
        "p95_gap_s": (
            result["p95_gap_ms"] * 1.0e-3
            if result["p95_gap_ms"] is not None
            else None
        ),
        "p99_gap_s": (
            result["p99_gap_ms"] * 1.0e-3
            if result["p99_gap_ms"] is not None
            else None
        ),
        "maximum_gap_s": (
            maximum_gap_ms * 1.0e-3 if maximum_gap_ms is not None else None
        ),
        "maximum_allowed_gap_s": ACCEPTED_STATUS_MAXIMUM_GAP_S,
        "includes_startup_and_sealed_final_boundaries": True,
    }


def _rate_in_10hz_window(value: Any) -> bool:
    return isinstance(value, (int, float)) and (
        TEN_HZ_MINIMUM_RATE_HZ <= float(value) <= TEN_HZ_MAXIMUM_RATE_HZ
    )


def _ten_hz_claim(
    accepted_events: Sequence[Mapping[str, Any]],
    trajectory_timing: Mapping[str, Any],
    path_timing: Mapping[str, Any],
    latency: Mapping[str, Any],
    accepted_status_wall_timing: Mapping[str, Any],
    status: Mapping[str, Any],
    arm_to_seal_status: Mapping[str, Any],
    lifecycle_integrity: Mapping[str, Any],
    map_runtime_status_bound: bool,
) -> dict[str, Any]:
    anchors = [event["anchor_timestamp_ns"] for event in accepted_events]
    anchor_duration_s = (anchors[-1] - anchors[0]) * 1.0e-9 if len(anchors) > 1 else 0.0
    anchor_rate_hz = (
        (len(anchors) - 1) / anchor_duration_s if anchor_duration_s > 0.0 else 0.0
    )
    anchor_periods_ns = [
        current - previous for previous, current in zip(anchors, anchors[1:])
    ]
    anchor_period_lower_ns = (
        TEN_HZ_EXPECTED_ANCHOR_PERIOD_NS - TEN_HZ_ANCHOR_PERIOD_TOLERANCE_NS
    )
    anchor_period_upper_ns = (
        TEN_HZ_EXPECTED_ANCHOR_PERIOD_NS + TEN_HZ_ANCHOR_PERIOD_TOLERANCE_NS
    )
    anchor_period_violation_count = sum(
        not anchor_period_lower_ns <= period_ns <= anchor_period_upper_ns
        for period_ns in anchor_periods_ns
    )
    requirements = {
        "armed_zero_prefix_and_sealed_boundaries": {
            "observed": True,
            "met": True,
        },
        "pre_startup_only_initial_waiting_and_zero_losses": {
            "observed_initial_waiting_heartbeat_count": lifecycle_integrity[
                "pre_startup_initial_waiting_heartbeat_count"
            ],
            "observed_disallowed_waiting_status_count": lifecycle_integrity[
                "pre_startup_disallowed_waiting_status_count"
            ],
            "observed_rejected_status_count": lifecycle_integrity[
                "pre_startup_rejected_status_count"
            ],
            "observed_rejection_count": lifecycle_integrity[
                "pre_startup_rejection_count"
            ],
            "observed_drop_count": lifecycle_integrity[
                "pre_startup_drop_count"
            ],
            "observed_timeout_count": lifecycle_integrity[
                "pre_startup_timeout_count"
            ],
            "met": lifecycle_integrity[
                "pre_startup_only_initial_waiting_and_zero_losses"
            ],
        },
        "complete_accepted_ordinal_output_correlation": {
            "observed": True,
            "met": True,
        },
        "minimum_duration_s": {
            "required": TEN_HZ_MINIMUM_DURATION_S,
            "observed": anchor_duration_s,
            "met": anchor_duration_s >= TEN_HZ_MINIMUM_DURATION_S,
        },
        "minimum_accepted_count": {
            "required": TEN_HZ_MINIMUM_ACCEPTED_COUNT,
            "observed": len(accepted_events),
            "met": len(accepted_events) >= TEN_HZ_MINIMUM_ACCEPTED_COUNT,
        },
        "anchor_rate_hz": {
            "required_range": [TEN_HZ_MINIMUM_RATE_HZ, TEN_HZ_MAXIMUM_RATE_HZ],
            "observed": anchor_rate_hz,
            "met": _rate_in_10hz_window(anchor_rate_hz),
        },
        "continuous_source_anchor_period": {
            "required_period_ns": TEN_HZ_EXPECTED_ANCHOR_PERIOD_NS,
            "allowed_absolute_tolerance_ns": TEN_HZ_ANCHOR_PERIOD_TOLERANCE_NS,
            "observed_gap_count": len(anchor_periods_ns),
            "observed_minimum_period_ns": (
                min(anchor_periods_ns) if anchor_periods_ns else None
            ),
            "observed_maximum_period_ns": (
                max(anchor_periods_ns) if anchor_periods_ns else None
            ),
            "observed_violation_count": anchor_period_violation_count,
            "met": bool(anchor_periods_ns)
            and anchor_period_violation_count == 0,
        },
        "trajectory_receipt_rate_hz": {
            "required_range": [TEN_HZ_MINIMUM_RATE_HZ, TEN_HZ_MAXIMUM_RATE_HZ],
            "observed": trajectory_timing["rate_hz"],
            "met": _rate_in_10hz_window(trajectory_timing["rate_hz"]),
        },
        "path_receipt_rate_hz": {
            "required_range": [TEN_HZ_MINIMUM_RATE_HZ, TEN_HZ_MAXIMUM_RATE_HZ],
            "observed": path_timing["rate_hz"],
            "met": _rate_in_10hz_window(path_timing["rate_hz"]),
        },
        "latency_deadline": {
            "required_p99_maximum_ms": LATENCY_DEADLINE_MS,
            "observed_p99_ms": latency["p99_ms"],
            "observed_count_over_100ms": latency["count_over_100ms"],
            "met": latency["p99_ms"] <= LATENCY_DEADLINE_MS
            and latency["count_over_100ms"] == 0,
        },
        "maximum_accepted_status_wall_gap_s": {
            "required_maximum_s": ACCEPTED_STATUS_MAXIMUM_GAP_S,
            "observed": accepted_status_wall_timing["maximum_gap_s"],
            "scope": (
                "startup boundary through every accepted outcome to sealed final"
            ),
            "startup_to_first_accepted_gap_s": accepted_status_wall_timing[
                "startup_to_first_accepted_gap_s"
            ],
            "last_accepted_to_sealed_final_gap_s": accepted_status_wall_timing[
                "last_accepted_to_sealed_final_gap_s"
            ],
            "met": accepted_status_wall_timing["maximum_gap_s"] is not None
            and accepted_status_wall_timing["maximum_gap_s"]
            <= ACCEPTED_STATUS_MAXIMUM_GAP_S,
        },
        "every_measured_status_fully_healthy": {
            "observed_status_count": status["health_snapshot_counts"]["total"],
            "observed_healthy_shadow_ok_verified_count": status[
                "health_snapshot_counts"
            ]["healthy_shadow_ok_verified"],
            "observed_waiting_count": status["health_snapshot_counts"]["waiting"],
            "observed_rejected_count": status["health_snapshot_counts"][
                "rejected"
            ],
            "observed_camera_bundle_timeout_count": status[
                "health_snapshot_counts"
            ]["camera_bundle_timeout"],
            "met": status["health_snapshot_counts"][
                "all_statuses_healthy_shadow_ok_verified"
            ]
            and status["health_snapshot_counts"]["waiting"] == 0
            and status["health_snapshot_counts"]["rejected"] == 0
            and status["health_snapshot_counts"]["camera_bundle_timeout"] == 0,
        },
        "zero_rejections_arm_to_seal": {
            "observed_anchor_rejected_count": arm_to_seal_status["denominators"][
                "anchor_rejected_count"
            ],
            "observed_input_event_rejected_count": arm_to_seal_status[
                "denominators"
            ][
                "input_event_rejected_count"
            ],
            "observed_rejection_stage_count": sum(
                arm_to_seal_status["rejection_counts_by_stage"].values()
            ),
            "met": arm_to_seal_status["denominators"][
                "anchor_rejected_count"
            ] == 0
            and arm_to_seal_status["denominators"][
                "input_event_rejected_count"
            ] == 0
            and not any(
                arm_to_seal_status["rejection_counts_by_stage"].values()
            ),
        },
        "zero_silent_runtime_losses_arm_to_seal": {
            "observed_camera_bundle_cumulative_delta": arm_to_seal_status[
                "camera_bundle_cumulative_delta"
            ],
            "observed_terminal_pending_bundle_count": arm_to_seal_status[
                "camera_bundle_counters_at_end"
            ]["pending_bundle_count"],
            "observed_input_settle_deferred_count": arm_to_seal_status[
                "input_settle"
            ]["input_settle_deferred_count"],
            "observed_input_settle_timeout_count": arm_to_seal_status[
                "input_settle"
            ]["input_settle_timeout_count"],
            "deferred_count_is_reporting_only": True,
            "met": not any(
                arm_to_seal_status["camera_bundle_cumulative_delta"].values()
            )
            and arm_to_seal_status["camera_bundle_counters_at_end"][
                "pending_bundle_count"
            ] == 0
            and arm_to_seal_status["input_settle"][
                "input_settle_timeout_count"
            ] == 0,
        },
        "full_health_for_every_accept": {
            "observed_full_healthy_count": sum(
                bool(event["full_healthy"]) for event in accepted_events
            ),
            "required_count": len(accepted_events),
            "met": all(bool(event["full_healthy"]) for event in accepted_events),
        },
        "map_provenance_runtime_status_bound": {
            "observed": map_runtime_status_bound,
            "met": map_runtime_status_bound,
        },
        "exact_status_heartbeat": {
            "observed_count": status["heartbeat_count"],
            "startup_boundary_observed": status[
                "startup_healthy_semantic_heartbeat_observed"
            ],
            "met": status["startup_healthy_semantic_heartbeat_observed"],
        },
    }
    established = all(item["met"] for item in requirements.values())
    return {
        "status": (
            "ESTABLISHED_FOR_SHADOW_ONLY" if established else "NOT_ESTABLISHED"
        ),
        "pass": established,
        "scope": "shadow inference and isolated shadow publications only",
        "requirements": requirements,
    }


def analyze_normalized(value: Any) -> dict[str, Any]:
    document = validate_normalized_document(value)
    source_normalized_sha256 = _canonical_sha256(
        document, "normalized shadow evidence"
    )
    topics = document["topics"]
    startup_index, final_index, window = _resolve_status_window(
        document["topics"], document["window_boundaries"]
    )
    trial_provenance = document["trial_provenance"]
    lifetime_status, lifetime_accepted_events = analyze_status_records(
        topics[STATUS_TOPIC], trial_provenance
    )
    arming_integrity = _require_complete_lifetime_prefix(
        topics[STATUS_TOPIC],
        lifetime_status,
        window["armed_status_record_index"],
        startup_index,
    )
    startup_accepted_count = window["startup_accepted_lifetime_ordinal"]
    final_accepted_count = window["final_accepted_lifetime_ordinal"]
    selected_topics, lifetime_output_integrity = _select_accepted_ordinal_outputs(
        topics,
        lifetime_accepted_events,
        startup_accepted_count,
        final_accepted_count,
    )
    lifetime_output_integrity.update(arming_integrity)
    window_status_records = topics[STATUS_TOPIC][startup_index:final_index + 1]
    status, accepted_events = analyze_status_records(
        window_status_records, trial_provenance
    )
    status["explicit_startup_boundary_observed"] = True
    status["explicit_final_boundary_observed"] = True
    status["startup_healthy_semantic_heartbeat_observed"] = True
    status["terminal_healthy_measurement_seal_observed"] = True
    accepted_count = status["denominators"]["accepted_count"]
    if accepted_count != final_accepted_count - startup_accepted_count:
        raise EvidenceError("status window disagrees with accepted lifetime ordinals")
    expected_window_events = lifetime_accepted_events[
        startup_accepted_count:final_accepted_count
    ]
    if accepted_events != expected_window_events:
        raise EvidenceError("bounded accepted statuses disagree with lifetime ordinals")

    latencies = [
        _finite_number(record["value"], f"latency[{index}]", positive=True)
        for index, record in enumerate(selected_topics[LATENCY_TOPIC])
    ]
    candidates = [
        _nonnegative_integer(record["value"], f"candidate[{index}]")
        for index, record in enumerate(selected_topics[CANDIDATE_TOPIC])
    ]
    if any(candidate > 5 for candidate in candidates):
        raise EvidenceError("selected candidate index exceeds the Common10 ABI")
    count_by_topic = {
        topic: len(selected_topics[topic])
        for topic in REQUIRED_TOPICS
        if topic != STATUS_TOPIC
    }
    if any(count != accepted_count for count in count_by_topic.values()):
        raise EvidenceError(
            "accepted denominator does not equal latency/candidate/trajectory/path counts"
        )
    accepted_anchors = [event["anchor_timestamp_ns"] for event in accepted_events]
    trajectory_anchors = [
        record["header_timestamp_ns"] for record in selected_topics[TRAJECTORY_TOPIC]
    ]
    path_anchors = [
        record["header_timestamp_ns"] for record in selected_topics[PATH_TOPIC]
    ]
    if trajectory_anchors != accepted_anchors or path_anchors != accepted_anchors:
        raise EvidenceError("trajectory/path header anchors do not match accepted statuses")
    for index, event in enumerate(accepted_events):
        if candidates[index] != event["candidate"]:
            raise EvidenceError(f"candidate topic disagrees with accepted status {index}")
        if not math.isclose(
            latencies[index],
            event["latency_ms"],
            rel_tol=1.0e-6,
            abs_tol=1.0e-3,
        ):
            raise EvidenceError(f"latency topic disagrees with accepted status {index}")

    latency = _latency_summary(latencies)
    accepted_status_wall_timing = _accepted_status_wall_timing(
        accepted_events,
        window["startup_status_wall_timestamp_ns"],
        window["final_status_wall_timestamp_ns"],
    )
    trajectory_timing = _message_timing(selected_topics[TRAJECTORY_TOPIC])
    path_timing = _message_timing(selected_topics[PATH_TOPIC])
    selected_histogram = {
        str(candidate): count
        for candidate, count in sorted(Counter(candidates).items())
    }
    accepted_inference_healthy = sum(
        bool(event["inference_inputs_healthy"])
        for event in accepted_events
    )
    full_healthy_count = sum(bool(event["full_healthy"]) for event in accepted_events)
    extrinsics_verified_count = sum(
        bool(event["extrinsics_verified"]) for event in accepted_events
    )
    map_runtime_status_bound = status["map_runtime_status_bound"]
    route_map_binding = _canonical_sha256(
        {
            "observed_map_id": trial_provenance["observed_map_id"],
            "map_bundle_sha256": trial_provenance["map_bundle_sha256"],
            "aligned_route_sha256": trial_provenance[
                "aligned_route_sha256"
            ],
        },
        "route-map binding",
    )
    ten_hz = _ten_hz_claim(
        accepted_events,
        trajectory_timing,
        path_timing,
        latency,
        accepted_status_wall_timing,
        status,
        lifetime_status,
        lifetime_output_integrity,
        map_runtime_status_bound,
    )
    return {
        "schema_id": REPORT_SCHEMA_ID,
        "analysis_status": "EVIDENCE_VALID",
        "source_schema_id": NORMALIZED_SCHEMA_ID,
        "source_normalized_sha256": source_normalized_sha256,
        "analysis_window": window,
        "lifetime_evidence_integrity": lifetime_output_integrity,
        "arm_to_seal_status_accounting": lifetime_status,
        "status_accounting": status,
        "health": {
            "accepted_inference_input_healthy_count": accepted_inference_healthy,
            "accepted_count": accepted_count,
            "accepted_full_healthy_count": full_healthy_count,
            "accepted_extrinsics_verified_count": extrinsics_verified_count,
            "inference_inputs_complete": accepted_inference_healthy == accepted_count,
            "full_health_complete": full_healthy_count == accepted_count,
            "tf_extrinsic_parity": (
                "VERIFIED"
                if extrinsics_verified_count == accepted_count
                else "UNVERIFIED_REQUIRED"
            ),
            "maximum_observed_tf_translation_error_m": max(
                event["maximum_tf_translation_error_m"]
                for event in accepted_events
            ),
            "maximum_observed_tf_rotation_error_rad": max(
                event["maximum_tf_rotation_error_rad"] for event in accepted_events
            ),
            "campaign_runtime_policy": dict(CAMPAIGN_RUNTIME_POLICY),
            "window_status_tf_extrinsic_parity_histogram": status[
                "tf_extrinsic_parity_histogram"
            ],
        },
        "provenance": {
            "status_provenance_consistent": True,
            "trial_provenance_matches_status": True,
            "status_bound": {
                field: {
                    "declared_map_id": trial_provenance["observed_map_id"],
                    "route_sha256": trial_provenance["aligned_route_sha256"],
                }.get(field, trial_provenance.get(field))
                for field in STATUS_BOUND_PROVENANCE_FIELDS
            },
            "map": {
                "declared_map_id": trial_provenance["observed_map_id"],
                "observed_map_id": trial_provenance["observed_map_id"],
                "map_bundle_sha256": trial_provenance["map_bundle_sha256"],
                "aligned_route_sha256": trial_provenance[
                    "aligned_route_sha256"
                ],
                "route_map_binding_sha256": route_map_binding,
                "consistent_with_trial_metadata": True,
                "declared_matches_independent_observation": True,
                "runtime_status_bound": map_runtime_status_bound,
                "map_runtime_binding": map_runtime_status_bound,
                "scope": (
                    "independent observed map identity and aligned route digest; "
                    "map bundle digest remains trial metadata"
                ),
            },
        },
        "selected_candidate_histogram": selected_histogram,
        "accepted_latency": latency,
        "accepted_status_wall_timing": accepted_status_wall_timing,
        "shadow_publications": {
            "trajectory": trajectory_timing,
            "path": path_timing,
            "counts_match_accepted_denominator": True,
            "header_anchors_match_accepted_statuses": True,
        },
        "claims": {
            "vehicle_control": {
                "approved": False,
                "status": "NOT_APPROVED",
                "reason": "shadow topics have no vehicle-control authority",
            },
            "ten_hz": ten_hz,
        },
    }


def _storage_identifier(bag: Path) -> str:
    metadata_path = bag / "metadata.yaml"
    if metadata_path.is_symlink() or not metadata_path.is_file():
        raise EvidenceError(f"rosbag metadata is missing or unsafe: {metadata_path}")
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise EvidenceError(f"cannot read rosbag metadata: {error}") from error
    if not isinstance(metadata, Mapping):
        raise EvidenceError("rosbag metadata must be an object")
    information = metadata.get("rosbag2_bagfile_information")
    if not isinstance(information, Mapping):
        raise EvidenceError("rosbag metadata lacks bagfile information")
    return _nonempty_text(information.get("storage_identifier"), "storage_identifier")


def _message_header_stamp_ns(message: Any, topic: str) -> int:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    seconds_value = _nonnegative_integer(seconds, f"{topic} header stamp seconds")
    nanoseconds_value = _nonnegative_integer(
        nanoseconds, f"{topic} header stamp nanoseconds"
    )
    if nanoseconds_value >= 1_000_000_000:
        raise EvidenceError(f"{topic} header stamp nanoseconds are invalid")
    return _positive_integer(
        seconds_value * 1_000_000_000 + nanoseconds_value,
        f"{topic} header timestamp",
    )


def normalize_rosbag(
    bag: Path,
    trial_provenance: Mapping[str, Any],
    window_boundaries: Mapping[str, Any],
) -> dict[str, Any]:
    source = bag.expanduser().absolute()
    if source.is_symlink() or not source.is_dir():
        raise EvidenceError(f"rosbag must be a regular non-symlink directory: {source}")
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise EvidenceError(
            "ROS 2 Python modules are unavailable; source the ROS environment"
        ) from error

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(source), storage_id=_storage_identifier(source)
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
    except RuntimeError as error:
        raise EvidenceError(f"cannot open rosbag: {error}") from error
    try:
        topic_metadata = reader.get_all_topics_and_types()
    except RuntimeError as error:
        raise EvidenceError(f"cannot inspect rosbag topics: {error}") from error
    topic_types: dict[str, str] = {}
    for item in topic_metadata:
        if item.name in topic_types:
            raise EvidenceError(f"rosbag contains duplicate topic metadata: {item.name}")
        topic_types[item.name] = item.type
    expected_types = {
        STATUS_TOPIC: "std_msgs/msg/String",
        LATENCY_TOPIC: "std_msgs/msg/Float32",
        CANDIDATE_TOPIC: "std_msgs/msg/Int8",
        TRAJECTORY_TOPIC: "autoware_planning_msgs/msg/Trajectory",
        PATH_TOPIC: "nav_msgs/msg/Path",
    }
    for topic, expected_type in expected_types.items():
        if topic_types.get(topic) != expected_type:
            raise EvidenceError(
                f"required topic is missing or has the wrong type: {topic}"
            )
    if hasattr(reader, "set_filter"):
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(REQUIRED_TOPICS)))
    try:
        message_types = {
            topic: get_message(topic_types[topic]) for topic in REQUIRED_TOPICS
        }
    except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as error:
        raise EvidenceError(f"cannot resolve rosbag message types: {error}") from error
    records: dict[str, list[dict[str, Any]]] = {topic: [] for topic in REQUIRED_TOPICS}
    while True:
        try:
            if not reader.has_next():
                break
            topic, serialized, bag_timestamp_ns = reader.read_next()
        except RuntimeError as error:
            raise EvidenceError(f"cannot read rosbag record: {error}") from error
        if topic not in message_types:
            continue
        try:
            message = deserialize_message(serialized, message_types[topic])
        except RuntimeError as error:
            raise EvidenceError(f"cannot deserialize {topic}: {error}") from error
        record: dict[str, Any] = {
            "bag_timestamp_ns": _positive_integer(
                int(bag_timestamp_ns), f"{topic} bag timestamp"
            )
        }
        if topic == STATUS_TOPIC:
            record["payload"] = strict_json_loads(message.data, "shadow status")
        elif topic == LATENCY_TOPIC:
            record["value"] = _finite_number(
                message.data, "shadow latency", positive=True
            )
        elif topic == CANDIDATE_TOPIC:
            record["value"] = _nonnegative_integer(
                message.data, "selected candidate"
            )
        else:
            record["header_timestamp_ns"] = _message_header_stamp_ns(message, topic)
        records[topic].append(record)
    return validate_normalized_document(
        {
            "schema_id": NORMALIZED_SCHEMA_ID,
            "trial_provenance": dict(trial_provenance),
            "window_boundaries": dict(window_boundaries),
            "topics": records,
        }
    )


def atomic_new_json(path: Path, value: Mapping[str, Any]) -> Path:
    output = path.expanduser().absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink() or not output.parent.is_dir():
        raise EvidenceError("output parent must be a regular directory")
    directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        directory_descriptor = os.open(output.parent, directory_flags)
    except OSError as error:
        raise EvidenceError(f"cannot open output directory: {error}") from error
    temporary_name = f".{output.name}.tmp.{os.getpid()}.{secrets.token_hex(8)}"
    descriptor = -1
    linked = False
    completed = False
    try:
        try:
            os.stat(output.name, dir_fd=directory_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise EvidenceError(f"output already exists: {output}")
        payload = (
            json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
        ).encode("utf-8")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(
            temporary_name, flags, 0o600, dir_fd=directory_descriptor
        )
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(descriptor)
        result = os.fstat(descriptor)
        if not stat.S_ISREG(result.st_mode) or result.st_size != len(payload):
            raise EvidenceError("generated report is not a complete regular file")
        # HH_260906 - Publish by hard link so concurrent output creation cannot overwrite data.
        os.link(
            temporary_name,
            output.name,
            src_dir_fd=directory_descriptor,
            dst_dir_fd=directory_descriptor,
            follow_symlinks=False,
        )
        linked = True
        os.fsync(directory_descriptor)
        os.unlink(temporary_name, dir_fd=directory_descriptor)
        temporary_name = ""
        os.fsync(directory_descriptor)
        completed = True
        return output
    except FileExistsError as error:
        raise EvidenceError(f"output already exists: {output}") from error
    except EvidenceError:
        raise
    except (OSError, TypeError, ValueError) as error:
        raise EvidenceError(f"cannot atomically write report: {error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_name:
            try:
                os.unlink(temporary_name, dir_fd=directory_descriptor)
            except FileNotFoundError:
                pass
        if linked and not completed:
            try:
                os.unlink(output.name, dir_fd=directory_descriptor)
            except FileNotFoundError:
                pass
        try:
            os.fsync(directory_descriptor)
        except OSError:
            pass
        os.close(directory_descriptor)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze fail-closed Portable E2E shadow trial evidence"
    )
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--bag", type=Path, help="recorded ROS 2 bag directory")
    inputs.add_argument(
        "--normalized-json", type=Path, help="normalized no-ROS evidence fixture"
    )
    parser.add_argument(
        "--trial-provenance-json",
        type=Path,
        help="exact trial provenance object required with --bag",
    )
    parser.add_argument(
        "--window-boundaries-json",
        type=Path,
        help="exact armed/startup/final status boundary object required with --bag",
    )
    parser.add_argument(
        "--require-ten-hz-pass",
        action="store_true",
        help="return status 3 after writing evidence when the shadow-only 10 Hz claim fails",
    )
    parser.add_argument("--output", type=Path, required=True, help="new report JSON")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.bag is not None:
            if args.trial_provenance_json is None:
                raise EvidenceError("--trial-provenance-json is required with --bag")
            if args.window_boundaries_json is None:
                raise EvidenceError("--window-boundaries-json is required with --bag")
            provenance = validate_trial_provenance(
                _read_strict_json(args.trial_provenance_json, "trial provenance")
            )
            boundaries = _read_strict_json(
                args.window_boundaries_json, "window boundaries"
            )
            document = normalize_rosbag(args.bag, provenance, boundaries)
        else:
            if (
                args.trial_provenance_json is not None
                or args.window_boundaries_json is not None
            ):
                raise EvidenceError(
                    "provenance and window boundaries are embedded in --normalized-json"
                )
            document = validate_normalized_document(
                _read_strict_json(args.normalized_json, "normalized evidence")
            )
        report = analyze_normalized(document)
        output = atomic_new_json(args.output, report)
    except EvidenceError as error:
        parser.exit(2, f"error: {error}\n")
    print(output)
    # HH_260906 - Let campaign wrappers fail closed while retaining the diagnostic report.
    if args.require_ten_hz_pass and report["claims"]["ten_hz"]["pass"] is not True:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
