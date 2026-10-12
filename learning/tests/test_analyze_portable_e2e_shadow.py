# HH_260906 - Verify Portable E2E shadow evidence analysis remains exact and fail-closed.

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

import pytest

from scripts.e2e import analyze_portable_e2e_shadow as analyzer


def _trial_provenance() -> dict:
    return {
        "runtime_id": "portable_e2e.pytorch_shadow_runtime.v1",
        "model_id": "portable_e2e.perspective_trajectory_physical.v1",
        "runtime_bundle_sha256": "a" * 64,
        "source_checkpoint_sha256": "b" * 64,
        "model_config_sha256": "c" * 64,
        "corpus_fingerprint_sha256": "d" * 64,
        "aligned_route_sha256": "e" * 64,
        "observed_map_id": "town07",
        "map_bundle_sha256": "f" * 64,
        "runtime_gate_id": analyzer.RUNTIME_GATE_ID,
        "runtime_gate": dict(analyzer.EXPECTED_RUNTIME_GATE),
    }


def _runtime_provenance() -> dict:
    trial = _trial_provenance()
    return {
        "runtime_id": trial["runtime_id"],
        "model_id": trial["model_id"],
        "runtime_bundle_sha256": trial["runtime_bundle_sha256"],
        "source_checkpoint_sha256": trial["source_checkpoint_sha256"],
        "model_config_sha256": trial["model_config_sha256"],
        "corpus_fingerprint_sha256": trial["corpus_fingerprint_sha256"],
        "declared_map_id": trial["observed_map_id"],
        "route_sha256": trial["aligned_route_sha256"],
        "adapter_id": analyzer.ADAPTER_ID,
        "tf_extrinsic_policy": "live_base_link_to_optical_at_image_anchor",
        "health_scope": "full_runtime_requires_camera_extrinsics",
        "runtime_device": "cpu",
        "runtime_gate_id": analyzer.RUNTIME_GATE_ID,
        "runtime_gate": dict(analyzer.EXPECTED_RUNTIME_GATE),
        "runtime_policy": {
            "maximum_tf_translation_error_m": 0.005,
            "maximum_tf_rotation_error_rad": 0.005,
            "input_settle_timeout_s": 0.05,
        },
        "runtime_execution_policy": {
            "policy_id": "portable_e2e.cpu_execution.v1",
            "torch_intraop_threads": 4,
            "torch_interop_threads": 1,
            "cpu_affinity": [8, 10, 12, 14],
        },
    }


def _tf_extrinsic_errors() -> dict[str, dict[str, float]]:
    return {
        camera: {
            "translation_error_m": 0.001,
            "rotation_error_rad": 0.002,
        }
        for camera in analyzer.CAMERA_ORDER
    }


def _status(
    *,
    state: str,
    attempts: int,
    accepted: int,
    anchor_rejected: int,
    input_rejected: int,
    stages: dict[str, int],
    anchor_ns: int | None,
    status_ns: int,
    detail: dict,
    extrinsics_verified: bool = False,
    measurement_armed: bool = False,
    measurement_sealed: bool = False,
    wall_ns: int | None = None,
) -> dict:
    inference_healthy = state == "SHADOW_OK"
    detail = dict(detail)
    if measurement_armed and inference_healthy:
        detail.setdefault("measurement_anchor_floor_ns", 1_500_000_000)
        detail.setdefault("preboundary_filtered_input_count", 0)
    return {
        "schema_id": analyzer.STATUS_SCHEMA_ID,
        "state": state,
        "stage": detail.get("stage", "none"),
        "failure_code": detail.get("failure_code", "none"),
        "healthy_now": inference_healthy and extrinsics_verified,
        "inference_inputs_healthy_now": inference_healthy,
        "calibration_extrinsics_verified": extrinsics_verified,
        "tf_extrinsic_parity": (
            "VERIFIED" if extrinsics_verified else "UNVERIFIED_REQUIRED"
        ),
        "measurement_armed": measurement_armed,
        "measurement_sealed": measurement_sealed,
        "health_scope": "full_runtime_requires_camera_extrinsic_parity",
        "anchor_timestamp_ns": anchor_ns,
        "status_timestamp_ns": status_ns,
        "status_wall_timestamp_ns": status_ns + 100 if wall_ns is None else wall_ns,
        "anchor_attempt_count": attempts,
        "accepted_count": accepted,
        "anchor_rejected_count": anchor_rejected,
        "input_event_rejected_count": input_rejected,
        "input_settle_deferred_count": 0,
        "input_settle_timeout_count": 0,
        "rejection_counts_by_stage": dict(stages),
        "last_complete_bundle_wall_age_ms": 1.0,
        "camera_bundle_counters": {
            "pending_bundle_count": 0,
            "dropped_stale_count": 0,
            "expired_pending_count": 0,
            "evicted_capacity_count": 0,
        },
        "vehicle_control_approved": False,
        "output_topics": list(analyzer.EXPECTED_OUTPUT_TOPICS),
        "provenance": _runtime_provenance(),
        "detail": detail,
    }


def _accepted_fixture(
    count: int = 2,
    *,
    period_ns: int = 100_000_000,
) -> dict:
    stages = {stage: 0 for stage in analyzer.REJECTION_STAGES}
    startup_wall_ns = 100_000_000_000
    status_records = [
        {
            "bag_timestamp_ns": 1_000_000_000,
            "payload": _status(
                state="SHADOW_WAITING",
                attempts=0,
                accepted=0,
                anchor_rejected=0,
                input_rejected=0,
                stages=stages,
                anchor_ns=None,
                status_ns=1_000_000_000,
                wall_ns=startup_wall_ns - 3_000_000_000,
                detail={"failure_code": "waiting_for_inputs"},
                extrinsics_verified=False,
            ),
        }
    ]
    armed_status = {
        "bag_timestamp_ns": 1_500_000_000,
        "payload": _status(
            state="SHADOW_WAITING",
            attempts=0,
            accepted=0,
            anchor_rejected=0,
            input_rejected=0,
            stages=stages,
            anchor_ns=None,
            status_ns=1_500_000_000,
            wall_ns=startup_wall_ns - 2_000_000_000,
            detail={
                "stage": "measurement",
                "failure_code": "waiting_for_inputs",
                "reason": "armed_waiting_for_exact_bundle_and_camera_info",
                "measurement_anchor_floor_ns": 1_500_000_000,
                "measurement_anchor_policy": "strictly_after_arm_source_time",
            },
            measurement_armed=True,
        ),
    }
    status_records.append(armed_status)
    latency_records = []
    candidate_records = []
    trajectory_records = []
    path_records = []
    warm_anchor_ns = 9_900_000_000
    warm_latency_ms = 19.0
    warm_candidate = 5
    warm_detail = {
        "stage": "inference",
        "failure_code": "none",
        "candidate": warm_candidate,
        "latency_ms": warm_latency_ms,
        "tf_extrinsic_parity": "VERIFIED",
        "tf_extrinsic_errors": _tf_extrinsic_errors(),
        "trajectory_point_count": 64,
    }
    status_records.append(
        {
            "bag_timestamp_ns": 2_040_000_000,
            "payload": _status(
                state="SHADOW_OK",
                attempts=1,
                accepted=1,
                anchor_rejected=0,
                input_rejected=0,
                stages=stages,
                anchor_ns=warm_anchor_ns,
                status_ns=warm_anchor_ns,
                wall_ns=startup_wall_ns - 1_000_000_000,
                detail=warm_detail,
                extrinsics_verified=True,
                measurement_armed=True,
            ),
        }
    )
    startup = deepcopy(status_records[-1])
    startup["bag_timestamp_ns"] = 2_090_000_000
    startup["payload"]["status_timestamp_ns"] += 50_000_000
    startup["payload"]["status_wall_timestamp_ns"] = startup_wall_ns
    startup["payload"]["last_complete_bundle_wall_age_ms"] += 50.0
    status_records.append(startup)
    latency_records.append(
        {"bag_timestamp_ns": 1_910_000_000, "value": warm_latency_ms}
    )
    candidate_records.append(
        {"bag_timestamp_ns": 1_920_000_000, "value": warm_candidate}
    )
    trajectory_records.append(
        {
            "bag_timestamp_ns": 1_900_000_000,
            "header_timestamp_ns": warm_anchor_ns,
        }
    )
    path_records.append(
        {
            "bag_timestamp_ns": 1_905_000_000,
            "header_timestamp_ns": warm_anchor_ns,
        }
    )
    for index in range(count):
        anchor_ns = 10_000_000_000 + index * period_ns
        receipt_ns = 2_100_000_000 + index * period_ns
        candidate = index % 6
        latency_ms = 20.0 + index % 3
        status_records.append(
            {
                "bag_timestamp_ns": receipt_ns + 40_000_000,
                "payload": _status(
                    state="SHADOW_OK",
                    attempts=index + 2,
                    accepted=index + 2,
                    anchor_rejected=0,
                    input_rejected=0,
                    stages=stages,
                    anchor_ns=anchor_ns,
                    status_ns=anchor_ns,
                    wall_ns=startup_wall_ns + (index + 1) * period_ns,
                    detail={
                        "stage": "inference",
                        "failure_code": "none",
                        "candidate": candidate,
                        "latency_ms": latency_ms,
                        "tf_extrinsic_parity": "VERIFIED",
                        "tf_extrinsic_errors": _tf_extrinsic_errors(),
                        "trajectory_point_count": 64,
                    },
                    extrinsics_verified=True,
                    measurement_armed=True,
                ),
            }
        )
        latency_records.append(
            {"bag_timestamp_ns": receipt_ns + 10_000_000, "value": latency_ms}
        )
        candidate_records.append(
            {"bag_timestamp_ns": receipt_ns + 20_000_000, "value": candidate}
        )
        trajectory_records.append(
            {
                "bag_timestamp_ns": receipt_ns,
                "header_timestamp_ns": anchor_ns,
            }
        )
        path_records.append(
            {
                "bag_timestamp_ns": receipt_ns + 5_000_000,
                "header_timestamp_ns": anchor_ns,
            }
        )
    terminal = deepcopy(status_records[-1])
    terminal["bag_timestamp_ns"] += period_ns
    terminal["payload"]["status_timestamp_ns"] += period_ns
    terminal["payload"]["status_wall_timestamp_ns"] = max(
        startup_wall_ns + 10_000_000_000,
        terminal["payload"]["status_wall_timestamp_ns"] + period_ns,
    )
    terminal["payload"]["last_complete_bundle_wall_age_ms"] += 100.0
    terminal["payload"]["measurement_sealed"] = True
    status_records.append(terminal)
    return {
        "schema_id": analyzer.NORMALIZED_SCHEMA_ID,
        "trial_provenance": _trial_provenance(),
        "window_boundaries": {
            "schema_id": analyzer.WINDOW_BOUNDARY_SCHEMA_ID,
            "armed_status": armed_status["payload"],
            "startup_status": startup["payload"],
            "final_status": status_records[-1]["payload"],
        },
        "topics": {
            analyzer.STATUS_TOPIC: status_records,
            analyzer.LATENCY_TOPIC: latency_records,
            analyzer.CANDIDATE_TOPIC: candidate_records,
            analyzer.TRAJECTORY_TOPIC: trajectory_records,
            analyzer.PATH_TOPIC: path_records,
        },
    }


def test_valid_window_reports_exact_denominators_and_never_control() -> None:
    report = analyzer.analyze_normalized(_accepted_fixture())

    assert report["analysis_status"] == "EVIDENCE_VALID"
    assert report["status_accounting"]["denominators"] == {
        "anchor_attempt_count": 2,
        "accepted_count": 2,
        "anchor_rejected_count": 0,
        "input_event_rejected_count": 0,
    }
    assert report["status_accounting"]["heartbeat_count"] == 1
    assert report["selected_candidate_histogram"] == {"0": 1, "1": 1}
    assert report["accepted_latency"]["p50_ms"] == pytest.approx(20.5)
    assert report["accepted_latency"]["count_over_100ms"] == 0
    assert report["shadow_publications"]["trajectory"]["count"] == 2
    assert report["health"]["inference_inputs_complete"] is True
    assert report["health"]["full_health_complete"] is True
    assert report["health"]["tf_extrinsic_parity"] == "VERIFIED"
    assert report["claims"]["vehicle_control"]["approved"] is False
    assert report["claims"]["ten_hz"]["pass"] is False


def test_cross_topic_receipt_reordering_uses_accepted_ordinals_not_time() -> None:
    evidence = _accepted_fixture(count=2)
    startup_bag_ns = evidence["topics"][analyzer.STATUS_TOPIC][3][
        "bag_timestamp_ns"
    ]
    final_bag_ns = evidence["topics"][analyzer.STATUS_TOPIC][-1][
        "bag_timestamp_ns"
    ]
    for offset, topic in enumerate(
        (
            analyzer.LATENCY_TOPIC,
            analyzer.CANDIDATE_TOPIC,
            analyzer.TRAJECTORY_TOPIC,
            analyzer.PATH_TOPIC,
        )
    ):
        evidence["topics"][topic][1]["bag_timestamp_ns"] = (
            startup_bag_ns - 20_000_000 + offset * 1_000_000
        )
        evidence["topics"][topic][2]["bag_timestamp_ns"] = (
            final_bag_ns + 20_000_000 + offset * 1_000_000
        )

    report = analyzer.analyze_normalized(evidence)

    assert report["analysis_window"]["selection_mode"] == (
        "accepted_lifetime_ordinal"
    )
    assert report["analysis_window"]["cross_topic_receipt_time_slicing_used"] is False
    assert report["status_accounting"]["denominators"]["accepted_count"] == 2
    assert report["shadow_publications"]["trajectory"]["count"] == 2


def test_nonzero_measurement_arm_baseline_is_rejected_as_partial_evidence() -> None:
    evidence = _accepted_fixture()
    stage_baseline = {stage: 0 for stage in analyzer.REJECTION_STAGES}
    stage_baseline["image"] = 3
    stage_baseline["inference"] = 10
    for record in evidence["topics"][analyzer.STATUS_TOPIC]:
        payload = record["payload"]
        payload["anchor_attempt_count"] += 50
        payload["accepted_count"] += 40
        payload["anchor_rejected_count"] += 10
        payload["input_event_rejected_count"] += 3
        payload["input_settle_deferred_count"] += 7
        payload["input_settle_timeout_count"] += 2
        payload["rejection_counts_by_stage"] = {
            stage: payload["rejection_counts_by_stage"][stage]
            + stage_baseline[stage]
            for stage in analyzer.REJECTION_STAGES
        }

    with pytest.raises(analyzer.EvidenceError, match="zero-count lifetime prefix"):
        analyzer.analyze_normalized(evidence)


def test_measured_anchor_must_follow_atomic_arm_source_time_floor() -> None:
    # HH_260906 - Reject evidence that reuses an input epoch from the arm boundary.
    evidence = _accepted_fixture()
    first_measured = evidence["topics"][analyzer.STATUS_TOPIC][4]["payload"]
    first_measured["detail"]["measurement_anchor_floor_ns"] = first_measured[
        "anchor_timestamp_ns"
    ]

    with pytest.raises(analyzer.EvidenceError, match="measurement floor"):
        analyzer.analyze_normalized(evidence)


def test_final_boundary_must_be_a_terminal_sealed_repeated_outcome() -> None:
    evidence = _accepted_fixture()
    evidence["topics"][analyzer.STATUS_TOPIC].pop()
    evidence["window_boundaries"]["final_status"] = evidence["topics"][
        analyzer.STATUS_TOPIC
    ][-1]["payload"]

    with pytest.raises(analyzer.EvidenceError, match="final boundary"):
        analyzer.analyze_normalized(evidence)


def test_startup_heartbeat_final_wall_duration_and_terminal_seal_are_required() -> None:
    startup_changed = _accepted_fixture()
    startup_changed["topics"][analyzer.STATUS_TOPIC][3]["payload"]["detail"][
        "latency_ms"
    ] += 1.0
    with pytest.raises(analyzer.EvidenceError, match="startup boundary does not repeat"):
        analyzer.analyze_normalized(startup_changed)

    final_changed = _accepted_fixture()
    final_changed["topics"][analyzer.STATUS_TOPIC][-1]["payload"]["detail"][
        "candidate"
    ] = 4
    with pytest.raises(analyzer.EvidenceError, match="final boundary does not repeat"):
        analyzer.analyze_normalized(final_changed)

    too_short = _accepted_fixture()
    too_short["topics"][analyzer.STATUS_TOPIC][-1]["payload"][
        "status_wall_timestamp_ns"
    ] = 109_000_000_000
    with pytest.raises(analyzer.EvidenceError, match="shorter than 10 seconds"):
        analyzer.analyze_normalized(too_short)

    suffix = _accepted_fixture()
    extra = deepcopy(suffix["topics"][analyzer.STATUS_TOPIC][-1])
    extra["bag_timestamp_ns"] += 1
    extra["payload"]["status_timestamp_ns"] += 1
    extra["payload"]["status_wall_timestamp_ns"] += 1
    suffix["topics"][analyzer.STATUS_TOPIC].append(extra)
    with pytest.raises(analyzer.EvidenceError, match="terminal status record"):
        analyzer.analyze_normalized(suffix)

    pending = _accepted_fixture()
    pending["topics"][analyzer.STATUS_TOPIC][-1]["payload"][
        "camera_bundle_counters"
    ]["pending_bundle_count"] = 1
    with pytest.raises(analyzer.EvidenceError, match="pending camera bundles"):
        analyzer.analyze_normalized(pending)


def test_pending_bundle_gauge_may_fall_but_cumulative_counters_may_not() -> None:
    evidence = _accepted_fixture()
    statuses = evidence["topics"][analyzer.STATUS_TOPIC]
    statuses[2]["payload"]["camera_bundle_counters"]["pending_bundle_count"] = 2
    statuses[3]["payload"]["camera_bundle_counters"]["pending_bundle_count"] = 2
    statuses[4]["payload"]["camera_bundle_counters"]["pending_bundle_count"] = 1

    report = analyzer.analyze_normalized(evidence)

    assert report["status_accounting"]["pending_bundle_net_change"] == -2

    invalid = _accepted_fixture()
    invalid_statuses = invalid["topics"][analyzer.STATUS_TOPIC]
    invalid_statuses[4]["payload"]["camera_bundle_counters"][
        "dropped_stale_count"
    ] = 1
    with pytest.raises(analyzer.EvidenceError, match="camera-bundle counters decreased"):
        analyzer.analyze_normalized(invalid)


def test_rejections_have_separate_denominators_and_exact_reason_histogram() -> None:
    evidence = _accepted_fixture(count=1)
    stages_zero = {stage: 0 for stage in analyzer.REJECTION_STAGES}
    input_stages = dict(stages_zero, image=1)
    rejected_stages = dict(input_stages, inference=1)
    records = evidence["topics"][analyzer.STATUS_TOPIC]
    accepted_record = records[4]
    accepted_record["payload"]["anchor_attempt_count"] = 3
    accepted_record["payload"]["anchor_rejected_count"] = 1
    accepted_record["payload"]["input_event_rejected_count"] = 1
    accepted_record["payload"]["rejection_counts_by_stage"] = rejected_stages
    input_record = {
        "bag_timestamp_ns": 2_100_000_000,
        "payload": _status(
            state="SHADOW_REJECTED",
            attempts=1,
            accepted=1,
            anchor_rejected=0,
            input_rejected=1,
            stages=input_stages,
            anchor_ns=None,
            status_ns=9_925_000_000,
            wall_ns=100_025_000_000,
            detail={
                "stage": "image",
                "failure_code": "image_contract",
                "reason": "invalid image",
            },
            measurement_armed=True,
        ),
    }
    rejection_record = {
        "bag_timestamp_ns": 2_120_000_000,
        "payload": _status(
            state="SHADOW_REJECTED",
            attempts=2,
            accepted=1,
            anchor_rejected=1,
            input_rejected=1,
            stages=rejected_stages,
            anchor_ns=9_950_000_000,
            status_ns=9_950_000_000,
            wall_ns=100_050_000_000,
            detail={
                "stage": "inference",
                "failure_code": "trajectory_gate",
                "reason": "trajectory rejected",
            },
            measurement_armed=True,
        ),
    }
    terminal = deepcopy(accepted_record)
    terminal["bag_timestamp_ns"] = 2_240_000_000
    terminal["payload"]["status_timestamp_ns"] += 100_000_000
    terminal["payload"]["status_wall_timestamp_ns"] = 111_000_000_000
    terminal["payload"]["measurement_sealed"] = True
    evidence["topics"][analyzer.STATUS_TOPIC] = [
        records[0],
        records[1],
        records[2],
        records[3],
        input_record,
        rejection_record,
        accepted_record,
        terminal,
    ]
    evidence["window_boundaries"]["armed_status"] = records[1]["payload"]
    evidence["window_boundaries"]["startup_status"] = records[3]["payload"]
    evidence["window_boundaries"]["final_status"] = terminal["payload"]

    report = analyzer.analyze_normalized(evidence)

    accounting = report["status_accounting"]
    assert accounting["denominators"] == {
        "anchor_attempt_count": 2,
        "accepted_count": 1,
        "anchor_rejected_count": 1,
        "input_event_rejected_count": 1,
    }
    assert accounting["rejection_reason_histogram"] == {
        "image_contract": 1,
        "trajectory_gate": 1,
    }
    assert accounting["rejection_counts_by_stage"]["image"] == 1
    assert accounting["rejection_counts_by_stage"]["inference"] == 1
    assert report["claims"]["ten_hz"]["requirements"][
        "every_measured_status_fully_healthy"
    ]["met"] is False


def test_latency_percentiles_and_deadline_count_use_accepted_denominator() -> None:
    evidence = _accepted_fixture(count=4)
    values = [10.0, 20.0, 100.0, 100.001]
    for index, value in enumerate(values):
        evidence["topics"][analyzer.LATENCY_TOPIC][index + 1]["value"] = value
        evidence["topics"][analyzer.STATUS_TOPIC][index + 4]["payload"]["detail"][
            "latency_ms"
        ] = value
    evidence["topics"][analyzer.STATUS_TOPIC][-1]["payload"]["detail"][
        "latency_ms"
    ] = values[-1]

    latency = analyzer.analyze_normalized(evidence)["accepted_latency"]

    assert latency["count"] == 4
    assert latency["p50_ms"] == pytest.approx(60.0)
    assert latency["p95_ms"] == pytest.approx(100.00085)
    assert latency["p99_ms"] == pytest.approx(100.00097)
    assert latency["count_over_100ms"] == 1


def test_ten_hz_is_claimed_only_when_every_shadow_requirement_is_met() -> None:
    report = analyzer.analyze_normalized(
        _accepted_fixture(count=101)
    )

    claim = report["claims"]["ten_hz"]
    assert claim["requirements"]["minimum_duration_s"]["met"] is True
    assert claim["requirements"]["minimum_accepted_count"]["met"] is True
    assert claim["requirements"]["anchor_rate_hz"]["met"] is True
    assert claim["requirements"]["continuous_source_anchor_period"] == {
        "required_period_ns": 100_000_000,
        "allowed_absolute_tolerance_ns": 5_000,
        "observed_gap_count": 100,
        "observed_minimum_period_ns": 100_000_000,
        "observed_maximum_period_ns": 100_000_000,
        "observed_violation_count": 0,
        "met": True,
    }
    assert claim["requirements"]["maximum_accepted_status_wall_gap_s"][
        "met"
    ] is True
    assert claim["requirements"]["every_measured_status_fully_healthy"][
        "met"
    ] is True
    assert claim["requirements"]["full_health_for_every_accept"]["met"] is True
    assert claim["requirements"]["map_provenance_runtime_status_bound"]["met"] is True
    assert claim["status"] == "ESTABLISHED_FOR_SHADOW_ONLY"
    assert claim["pass"] is True
    assert report["claims"]["vehicle_control"]["approved"] is False


def test_source_anchor_gap_blocks_ten_hz_even_when_aggregate_rates_pass() -> None:
    # HH_260906 - Reject one silently coalesced source frame despite a passing mean rate.
    evidence = _accepted_fixture(count=101)
    status_records = evidence["topics"][analyzer.STATUS_TOPIC]
    for record in status_records[54:]:
        if record["payload"]["anchor_timestamp_ns"] is not None:
            record["payload"]["anchor_timestamp_ns"] += 100_000_000
    evidence["window_boundaries"]["final_status"] = status_records[-1]["payload"]
    for topic in (analyzer.TRAJECTORY_TOPIC, analyzer.PATH_TOPIC):
        for record in evidence["topics"][topic][51:]:
            record["header_timestamp_ns"] += 100_000_000

    report = analyzer.analyze_normalized(evidence)

    claim = report["claims"]["ten_hz"]
    continuity = claim["requirements"]["continuous_source_anchor_period"]
    assert claim["requirements"]["anchor_rate_hz"]["observed"] == pytest.approx(
        100.0 / 10.1
    )
    assert claim["requirements"]["anchor_rate_hz"]["met"] is True
    assert claim["requirements"]["maximum_accepted_status_wall_gap_s"]["met"] is True
    assert claim["requirements"]["trajectory_receipt_rate_hz"]["met"] is True
    assert claim["requirements"]["path_receipt_rate_hz"]["met"] is True
    assert continuity["observed_maximum_period_ns"] == 200_000_000
    assert continuity["observed_violation_count"] == 1
    assert continuity["met"] is False
    assert claim["pass"] is False


def test_identical_pre_startup_waiting_heartbeat_is_reported_and_allowed() -> None:
    evidence = _accepted_fixture(count=101)
    records = evidence["topics"][analyzer.STATUS_TOPIC]
    waiting = deepcopy(records[1])
    waiting["bag_timestamp_ns"] = 1_700_000_000
    waiting["payload"]["status_timestamp_ns"] = 1_700_000_000
    waiting["payload"]["status_wall_timestamp_ns"] = 98_500_000_000
    records.insert(2, waiting)

    report = analyzer.analyze_normalized(evidence)

    integrity = report["lifetime_evidence_integrity"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "pre_startup_only_initial_waiting_and_zero_losses"
    ]
    assert integrity["pre_startup_initial_waiting_heartbeat_count"] == 1
    assert integrity["pre_startup_disallowed_waiting_status_count"] == 0
    assert requirement["met"] is True
    assert report["claims"]["ten_hz"]["pass"] is True


def test_pre_startup_noninitial_waiting_is_reported_and_blocks_ten_hz() -> None:
    evidence = _accepted_fixture(count=101)
    records = evidence["topics"][analyzer.STATUS_TOPIC]
    waiting = {
        "bag_timestamp_ns": 1_700_000_000,
        "payload": _status(
            state="SHADOW_WAITING",
            attempts=0,
            accepted=0,
            anchor_rejected=0,
            input_rejected=0,
            stages={stage: 0 for stage in analyzer.REJECTION_STAGES},
            anchor_ns=None,
            status_ns=1_700_000_000,
            wall_ns=98_500_000_000,
            detail={
                "stage": "camera_bundle",
                "failure_code": "camera_bundle_timeout",
                "reason": "pre-startup bundle was incomplete",
            },
            measurement_armed=True,
        ),
    }
    records.insert(2, waiting)

    report = analyzer.analyze_normalized(evidence)

    integrity = report["lifetime_evidence_integrity"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "pre_startup_only_initial_waiting_and_zero_losses"
    ]
    assert integrity["pre_startup_disallowed_waiting_status_count"] == 1
    assert requirement["met"] is False
    assert report["claims"]["ten_hz"]["pass"] is False


def test_measured_waiting_timeout_is_reported_and_blocks_ten_hz() -> None:
    evidence = _accepted_fixture(count=101)
    records = evidence["topics"][analyzer.STATUS_TOPIC]
    waiting = {
        "bag_timestamp_ns": 2_190_000_000,
        "payload": _status(
            state="SHADOW_WAITING",
            attempts=2,
            accepted=2,
            anchor_rejected=0,
            input_rejected=0,
            stages={stage: 0 for stage in analyzer.REJECTION_STAGES},
            anchor_ns=None,
            status_ns=10_050_000_000,
            wall_ns=100_150_000_000,
            detail={
                "stage": "camera_bundle",
                "failure_code": "camera_bundle_timeout",
                "reason": "measured bundle timed out",
            },
            measurement_armed=True,
        ),
    }
    records.insert(5, waiting)

    report = analyzer.analyze_normalized(evidence)

    health = report["status_accounting"]["health_snapshot_counts"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "every_measured_status_fully_healthy"
    ]
    assert report["analysis_status"] == "EVIDENCE_VALID"
    assert health["waiting"] == 1
    assert health["camera_bundle_timeout"] == 1
    assert requirement["met"] is False
    assert report["claims"]["ten_hz"]["pass"] is False


@pytest.mark.parametrize("gap_location", ("startup", "interior", "final"))
def test_boundary_inclusive_accepted_wall_stall_blocks_ten_hz(
    gap_location: str,
) -> None:
    evidence = _accepted_fixture(count=101)
    statuses = evidence["topics"][analyzer.STATUS_TOPIC]
    if gap_location == "startup":
        shifted = statuses[4:]
    elif gap_location == "interior":
        shifted = statuses[54:]
    else:
        shifted = statuses[-1:]
    for record in shifted:
        record["payload"]["status_wall_timestamp_ns"] += 150_000_000

    report = analyzer.analyze_normalized(evidence)

    timing = report["accepted_status_wall_timing"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "maximum_accepted_status_wall_gap_s"
    ]
    assert timing["includes_startup_and_sealed_final_boundaries"] is True
    assert timing["maximum_gap_s"] == pytest.approx(0.25)
    assert requirement["met"] is False
    assert report["claims"]["ten_hz"]["pass"] is False


def test_boundary_inclusive_wall_gap_accepts_the_exact_fixed_limit() -> None:
    evidence = _accepted_fixture(count=101)
    statuses = evidence["topics"][analyzer.STATUS_TOPIC]
    for record in statuses[54:]:
        record["payload"]["status_wall_timestamp_ns"] += 100_000_000

    report = analyzer.analyze_normalized(evidence)

    timing = report["accepted_status_wall_timing"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "maximum_accepted_status_wall_gap_s"
    ]
    assert timing["maximum_gap_s"] == pytest.approx(0.20)
    assert requirement["required_maximum_s"] == 0.20
    assert requirement["met"] is True
    assert report["claims"]["ten_hz"]["pass"] is True


def test_pre_startup_rejection_blocks_ten_hz_claim_for_arm_to_seal_lifetime() -> None:
    evidence = _accepted_fixture(count=101)
    records = evidence["topics"][analyzer.STATUS_TOPIC]
    rejected_stages = {stage: 0 for stage in analyzer.REJECTION_STAGES}
    rejected_stages["image"] = 1
    rejection = {
        "bag_timestamp_ns": 1_700_000_000,
        "payload": _status(
            state="SHADOW_REJECTED",
            attempts=0,
            accepted=0,
            anchor_rejected=0,
            input_rejected=1,
            stages=rejected_stages,
            anchor_ns=None,
            status_ns=1_700_000_000,
            wall_ns=98_500_000_000,
            detail={
                "stage": "image",
                "failure_code": "image_contract",
                "reason": "pre-startup rejected input",
            },
            measurement_armed=True,
        ),
    }
    records.insert(2, rejection)
    for record in records[3:]:
        record["payload"]["input_event_rejected_count"] += 1
        record["payload"]["rejection_counts_by_stage"]["image"] += 1

    report = analyzer.analyze_normalized(evidence)

    arm_to_seal = report["arm_to_seal_status_accounting"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "zero_rejections_arm_to_seal"
    ]
    assert arm_to_seal["denominators"]["input_event_rejected_count"] == 1
    assert requirement["met"] is False
    assert report["claims"]["ten_hz"]["requirements"][
        "pre_startup_only_initial_waiting_and_zero_losses"
    ]["met"] is False
    assert report["claims"]["ten_hz"]["pass"] is False


@pytest.mark.parametrize("loss_kind", ("camera_drop", "settle_timeout"))
def test_pre_startup_silent_loss_is_reported_and_blocks_ten_hz(
    loss_kind: str,
) -> None:
    evidence = _accepted_fixture(count=101)
    records = evidence["topics"][analyzer.STATUS_TOPIC]
    for record in records[2:]:
        if loss_kind == "camera_drop":
            record["payload"]["camera_bundle_counters"][
                "dropped_stale_count"
            ] = 1
        else:
            record["payload"]["input_settle_deferred_count"] = 1
            record["payload"]["input_settle_timeout_count"] = 1

    report = analyzer.analyze_normalized(evidence)

    integrity = report["lifetime_evidence_integrity"]
    requirement = report["claims"]["ten_hz"]["requirements"][
        "pre_startup_only_initial_waiting_and_zero_losses"
    ]
    observed_key = (
        "pre_startup_drop_count"
        if loss_kind == "camera_drop"
        else "pre_startup_timeout_count"
    )
    assert integrity[observed_key] == 1
    assert requirement["met"] is False
    assert report["claims"]["ten_hz"]["pass"] is False


def test_silent_runtime_losses_block_pass_but_normal_deferred_count_is_allowed() -> None:
    bundle_loss = _accepted_fixture(count=101)
    bundle_loss["topics"][analyzer.STATUS_TOPIC][-1]["payload"][
        "camera_bundle_counters"
    ]["dropped_stale_count"] = 1

    bundle_report = analyzer.analyze_normalized(bundle_loss)

    bundle_requirement = bundle_report["claims"]["ten_hz"]["requirements"][
        "zero_silent_runtime_losses_arm_to_seal"
    ]
    assert bundle_requirement["met"] is False
    assert bundle_report["claims"]["ten_hz"]["pass"] is False

    settle_timeout = _accepted_fixture(count=101)
    settle_timeout["topics"][analyzer.STATUS_TOPIC][-1]["payload"][
        "input_settle_deferred_count"
    ] = 1
    settle_timeout["topics"][analyzer.STATUS_TOPIC][-1]["payload"][
        "input_settle_timeout_count"
    ] = 1

    timeout_report = analyzer.analyze_normalized(settle_timeout)

    timeout_requirement = timeout_report["claims"]["ten_hz"]["requirements"][
        "zero_silent_runtime_losses_arm_to_seal"
    ]
    assert timeout_requirement["met"] is False
    assert timeout_report["claims"]["ten_hz"]["pass"] is False

    deferred = _accepted_fixture(count=101)
    deferred["topics"][analyzer.STATUS_TOPIC][-1]["payload"][
        "input_settle_deferred_count"
    ] = 1

    deferred_report = analyzer.analyze_normalized(deferred)

    deferred_requirement = deferred_report["claims"]["ten_hz"]["requirements"][
        "zero_silent_runtime_losses_arm_to_seal"
    ]
    assert deferred_requirement["observed_input_settle_deferred_count"] == 1
    assert deferred_requirement["met"] is True
    assert deferred_report["claims"]["ten_hz"]["pass"] is True


def test_duplicate_json_keys_and_nonfinite_values_are_rejected() -> None:
    with pytest.raises(analyzer.EvidenceError, match="duplicate JSON key"):
        analyzer.strict_json_loads('{"schema_id":"one","schema_id":"two"}', "fixture")
    with pytest.raises(analyzer.EvidenceError, match="non-finite"):
        analyzer.strict_json_loads('{"value":NaN}', "fixture")
    with pytest.raises(analyzer.EvidenceError, match="non-finite"):
        analyzer.strict_json_loads('{"value":1e400}', "fixture")

    evidence = _accepted_fixture()
    evidence["topics"][analyzer.LATENCY_TOPIC][0]["value"] = float("inf")
    with pytest.raises(analyzer.EvidenceError, match="NaN or Inf"):
        analyzer.analyze_normalized(evidence)

    overflowing = _accepted_fixture()
    overflowing["topics"][analyzer.LATENCY_TOPIC][0]["value"] = 10**400
    with pytest.raises(analyzer.EvidenceError, match="must be finite"):
        analyzer.analyze_normalized(overflowing)


@pytest.mark.parametrize("observed_map_id", ("Town07", "town-07", "", "a" * 65))
def test_independent_observed_map_id_must_use_the_runtime_normal_form(
    observed_map_id: str,
) -> None:
    evidence = _accepted_fixture()
    evidence["trial_provenance"]["observed_map_id"] = observed_map_id

    with pytest.raises(analyzer.EvidenceError, match="observed_map_id"):
        analyzer.analyze_normalized(evidence)


def test_missing_declared_map_or_settle_denominator_is_rejected() -> None:
    missing_map = _accepted_fixture()
    for record in missing_map["topics"][analyzer.STATUS_TOPIC]:
        del record["payload"]["provenance"]["declared_map_id"]
    with pytest.raises(analyzer.EvidenceError, match="declared_map_id"):
        analyzer.analyze_normalized(missing_map)

    missing_settle = _accepted_fixture()
    del missing_settle["topics"][analyzer.STATUS_TOPIC][0]["payload"][
        "input_settle_deferred_count"
    ]
    with pytest.raises(analyzer.EvidenceError, match="input_settle_deferred_count"):
        analyzer.analyze_normalized(missing_settle)


def test_accepted_status_requires_current_verified_extrinsic_parity() -> None:
    evidence = _accepted_fixture()
    accepted = evidence["topics"][analyzer.STATUS_TOPIC][4]["payload"]
    accepted["tf_extrinsic_parity"] = "UNVERIFIED_REQUIRED"
    accepted["calibration_extrinsics_verified"] = False
    accepted["healthy_now"] = False

    with pytest.raises(analyzer.EvidenceError, match="accepted without verified"):
        analyzer.analyze_normalized(evidence)


def test_every_accepted_status_requires_exact_finite_in_policy_six_camera_errors() -> None:
    missing = _accepted_fixture()
    del missing["topics"][analyzer.STATUS_TOPIC][4]["payload"]["detail"][
        "tf_extrinsic_errors"
    ]
    with pytest.raises(analyzer.EvidenceError, match="tf_extrinsic_errors"):
        analyzer.analyze_normalized(missing)

    nonfinite = _accepted_fixture()
    nonfinite["topics"][analyzer.STATUS_TOPIC][4]["payload"]["detail"][
        "tf_extrinsic_errors"
    ]["CAM_FRONT"]["translation_error_m"] = float("nan")
    with pytest.raises(analyzer.EvidenceError, match="NaN or Inf"):
        analyzer.analyze_normalized(nonfinite)

    over_limit = _accepted_fixture()
    over_limit["topics"][analyzer.STATUS_TOPIC][4]["payload"]["detail"][
        "tf_extrinsic_errors"
    ]["CAM_BACK"]["rotation_error_rad"] = 0.005001
    with pytest.raises(analyzer.EvidenceError, match="exceeds runtime policy"):
        analyzer.analyze_normalized(over_limit)

    missing_camera = _accepted_fixture()
    del missing_camera["topics"][analyzer.STATUS_TOPIC][4]["payload"]["detail"][
        "tf_extrinsic_errors"
    ]["CAM_BACK_RIGHT"]
    with pytest.raises(analyzer.EvidenceError, match="keys changed"):
        analyzer.analyze_normalized(missing_camera)


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("maximum_tf_translation_error_m", 0.006),
        ("maximum_tf_rotation_error_rad", 0.006),
        ("input_settle_timeout_s", 0.051),
    ),
)
def test_campaign_runtime_policy_is_pinned_exactly(field: str, value: float) -> None:
    evidence = _accepted_fixture()
    for record in evidence["topics"][analyzer.STATUS_TOPIC]:
        record["payload"]["provenance"]["runtime_policy"][field] = value

    with pytest.raises(analyzer.EvidenceError, match="not campaign-pinned"):
        analyzer.analyze_normalized(evidence)


def test_provenance_change_or_expected_mismatch_is_rejected() -> None:
    changed = _accepted_fixture()
    changed["topics"][analyzer.STATUS_TOPIC][4]["payload"]["provenance"][
        "runtime_gate_id"
    ] = "changed"
    with pytest.raises(analyzer.EvidenceError, match="runtime gate identity changed"):
        analyzer.analyze_normalized(changed)

    mismatch = _accepted_fixture()
    mismatch["topics"][analyzer.STATUS_TOPIC][1]["payload"]["provenance"][
        "route_sha256"
    ] = "0" * 64
    with pytest.raises(analyzer.EvidenceError, match="does not match"):
        analyzer.analyze_normalized(mismatch)

    map_mismatch = _accepted_fixture()
    map_mismatch["topics"][analyzer.STATUS_TOPIC][1]["payload"]["provenance"][
        "declared_map_id"
    ] = "town03"
    with pytest.raises(analyzer.EvidenceError, match="does not match"):
        analyzer.analyze_normalized(map_mismatch)


def test_uniform_runtime_gate_identity_tampering_is_rejected() -> None:
    evidence = _accepted_fixture()
    for record in evidence["topics"][analyzer.STATUS_TOPIC]:
        record["payload"]["provenance"]["runtime_gate_id"] = (
            "portable_e2e.runtime_geometry_gate.v6"
        )

    with pytest.raises(analyzer.EvidenceError, match="runtime gate identity changed"):
        analyzer.analyze_normalized(evidence)


def test_uniform_complete_runtime_gate_tampering_is_rejected() -> None:
    evidence = _accepted_fixture()
    for record in evidence["topics"][analyzer.STATUS_TOPIC]:
        record["payload"]["provenance"]["runtime_gate"][
            "maximum_speed_mps"
        ] = 30.0

    with pytest.raises(analyzer.EvidenceError, match="not contract-pinned"):
        analyzer.analyze_normalized(evidence)


def test_trial_runtime_gate_identity_and_values_are_independently_pinned() -> None:
    # HH_260906 - Reject a trial manifest that disagrees with otherwise valid status records.
    wrong_identity = _accepted_fixture()
    wrong_identity["trial_provenance"]["runtime_gate_id"] = (
        "portable_e2e.runtime_geometry_gate.v6"
    )
    with pytest.raises(analyzer.EvidenceError, match="gate identity changed"):
        analyzer.analyze_normalized(wrong_identity)

    wrong_gate = _accepted_fixture()
    wrong_gate["trial_provenance"]["runtime_gate"][
        "current_speed_reverse_jitter_tolerance_mps"
    ] = 0.04
    with pytest.raises(analyzer.EvidenceError, match="not contract-pinned"):
        analyzer.analyze_normalized(wrong_gate)


def test_missing_topic_denominator_or_window_boundary_is_rejected() -> None:
    missing = _accepted_fixture()
    del missing["topics"][analyzer.PATH_TOPIC]
    with pytest.raises(analyzer.EvidenceError, match="topics keys changed"):
        analyzer.analyze_normalized(missing)

    denominator = _accepted_fixture()
    denominator["topics"][analyzer.CANDIDATE_TOPIC].pop()
    with pytest.raises(analyzer.EvidenceError, match="accepted lifetime denominator"):
        analyzer.analyze_normalized(denominator)

    partial = _accepted_fixture()
    partial["topics"][analyzer.STATUS_TOPIC].pop()
    with pytest.raises(analyzer.EvidenceError, match="boundaries must each occur"):
        analyzer.analyze_normalized(partial)


def test_exact_armed_zero_boundary_and_complete_output_prefix_are_required() -> None:
    missing_arm = _accepted_fixture()
    missing_arm["topics"][analyzer.STATUS_TOPIC][1]["payload"][
        "measurement_armed"
    ] = False
    with pytest.raises(analyzer.EvidenceError, match="armed boundary"):
        analyzer.analyze_normalized(missing_arm)

    extra_output = _accepted_fixture()
    extra = deepcopy(extra_output["topics"][analyzer.CANDIDATE_TOPIC][-1])
    extra["bag_timestamp_ns"] += 1
    extra_output["topics"][analyzer.CANDIDATE_TOPIC].append(extra)
    with pytest.raises(analyzer.EvidenceError, match="complete output-topic prefixes"):
        analyzer.analyze_normalized(extra_output)

    missing_prefix = _accepted_fixture()
    candidates = missing_prefix["topics"][analyzer.CANDIDATE_TOPIC]
    candidates[0]["value"] = candidates[1]["value"]
    with pytest.raises(analyzer.EvidenceError, match="lifetime ordinal 1"):
        analyzer.analyze_normalized(missing_prefix)


def test_skipped_status_transition_and_anchor_mismatch_are_rejected() -> None:
    skipped = _accepted_fixture(count=3)
    del skipped["topics"][analyzer.STATUS_TOPIC][4]
    with pytest.raises(analyzer.EvidenceError, match="skipped evidence events"):
        analyzer.analyze_normalized(skipped)

    mismatched = _accepted_fixture()
    mismatched["topics"][analyzer.PATH_TOPIC][0]["header_timestamp_ns"] += 1
    with pytest.raises(analyzer.EvidenceError, match="prefixes do not match"):
        analyzer.analyze_normalized(mismatched)


def test_atomic_output_is_new_only_and_contains_no_temporary_residue(
    tmp_path: Path,
) -> None:
    report = analyzer.analyze_normalized(_accepted_fixture())
    output = tmp_path / "shadow-analysis.json"

    assert analyzer.atomic_new_json(output, report) == output
    assert json.loads(output.read_text(encoding="utf-8"))["schema_id"] == (
        analyzer.REPORT_SCHEMA_ID
    )
    with pytest.raises(analyzer.EvidenceError, match="already exists"):
        analyzer.atomic_new_json(output, report)
    assert list(tmp_path.glob(".*.tmp.*")) == []


def test_cli_writes_report_from_normalized_json_without_ros(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    source = tmp_path / "fixture.json"
    source.write_text(json.dumps(_accepted_fixture(), allow_nan=False), encoding="utf-8")
    output = tmp_path / "report.json"

    assert analyzer.main(
        ["--normalized-json", str(source), "--output", str(output)]
    ) == 0
    assert capsys.readouterr().out.strip() == str(output)
    assert json.loads(output.read_text(encoding="utf-8"))["analysis_status"] == (
        "EVIDENCE_VALID"
    )


def test_cli_required_ten_hz_claim_returns_three_after_writing_diagnostics(
    tmp_path: Path,
) -> None:
    # HH_260906 - Preserve a complete failure report while making campaign selection fail closed.
    source = tmp_path / "fixture.json"
    source.write_text(json.dumps(_accepted_fixture(), allow_nan=False), encoding="utf-8")
    output = tmp_path / "report.json"

    assert analyzer.main(
        [
            "--normalized-json",
            str(source),
            "--require-ten-hz-pass",
            "--output",
            str(output),
        ]
    ) == 3
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["analysis_status"] == "EVIDENCE_VALID"
    assert report["claims"]["ten_hz"]["pass"] is False


def test_cli_required_ten_hz_claim_returns_zero_for_established_fixture(
    tmp_path: Path,
) -> None:
    # HH_260906 - Keep the campaign gate successful only for the complete 10 Hz contract.
    source = tmp_path / "fixture.json"
    source.write_text(
        json.dumps(_accepted_fixture(count=101), allow_nan=False),
        encoding="utf-8",
    )
    output = tmp_path / "report.json"

    assert analyzer.main(
        [
            "--normalized-json",
            str(source),
            "--require-ten-hz-pass",
            "--output",
            str(output),
        ]
    ) == 0
    assert json.loads(output.read_text(encoding="utf-8"))["claims"]["ten_hz"][
        "pass"
    ] is True
