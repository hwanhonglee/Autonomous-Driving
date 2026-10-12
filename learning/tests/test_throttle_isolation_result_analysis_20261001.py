"""HH_261001 - Bounded CPU checks for independent censored-result analysis."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import analyze_town07_throttle_isolation_v3_20261001 as audit


def synthetic_records(ticks=60, abort=False):
    """HH_261001 - Synthetic native records exercise corruption and censoring logic."""
    points = [{"x": 0.0, "y": 0.0, "distance_m": 0.0},
              {"x": 100.0, "y": 0.0, "distance_m": 100.0}]
    spec = {"trial_id": "site16_B1" if abort else "site16_A1", "site_progress_m": 16,
            "throttle": 0.2 if abort else 0.01, "arm": "B" if abort else "A"}
    rows, receipts = [], []

    def row(frame, phase, x, vx, throttle, brake, tick=None):
        control = dict(throttle=throttle, steer=0.0, brake=brake, hand_brake=False,
                       reverse=False, gear=1, manual_gear_shift=False)
        sequence = len(receipts) + 1
        receipts.append(dict(sequence=sequence, status="ACKNOWLEDGED", server_accepted=True,
                             actor_id=114, response_actor_id=114, before_frame=frame-1,
                             after_ack_frame=frame-1, requested_control=copy.deepcopy(control), reason=phase))
        alignment = dict(expected_receipt_sequence=sequence, status="PASS", observed_frame=frame,
                         control_read_frame_before=frame, control_read_frame_after=frame)
        return dict(frame=frame, timestamp=frame*0.05, phase=phase, intervention_tick=tick,
                    route_progress_m=x, route_cte_m=0.0, current_control=control, control_alignment=alignment,
                    x=x, y=0.0, z=0.0, yaw=0.0, vx=vx, vy=0.0, ax=0.0, ay=0.0, yaw_rate=0.0,
                    actor_snapshot_transform_carla=dict(x=x+1.425, y=0.0, z=0.0, yaw=0.0, pitch=0.0),
                    world_velocity_carla=[vx, 0.0, 0.0], world_acceleration_carla=[0.0, 0.0, 0.0],
                    world_angular_velocity_carla_deg_s=[0.0, 0.0, 0.0], collision_count=0, lane_invasion_count=0,
                    traffic_light_hazard=dict(hazard=False), observed_free_disk_bytes=1000, safety_failures=[])

    bootstrap = row(100, "bootstrap", 0.318, 0.0, 0.0, 1.0)
    bootstrap["alignment"] = bootstrap.pop("control_alignment")
    for tick in range(1, 71):
        item = row(100+tick, "stationary_warmup", 0.318, 0.0, 0.0, 1.0)
        item["warmup_tick"] = tick
        rows.append(item)
    rows.append(row(171, "prefix", 15.9, 2.45, 0.05, 0.0))
    rows.append(row(172, "branch", 16.02, 2.45, 0.05, 0.0))
    for tick in range(1, ticks+1):
        vx = 2.703 if abort and tick == ticks else 2.45
        item = row(172+tick, "intervention", 16.02+0.1*tick, vx, spec["throttle"], 0.0, tick)
        if abort and tick == ticks:
            item["safety_failures"] = ["maximum_speed"]
        rows.append(item)
    return rows, receipts, bootstrap, spec, points


def test_full_native_window_recounts_cadence_ack_and_source_formula():
    rows, receipts, bootstrap, spec, points = synthetic_records()
    result, plot_rows = audit.audit_records(rows, receipts, bootstrap, spec, points, 100)
    assert result["integrity_issue_counts"] == {}
    assert result["observed_window"]["observed_ticks"] == 60
    assert result["observed_window"]["net_route_progress_m"] == pytest.approx(6.0)
    assert result["observed_window"]["signed_rate_below_minus_8_mps2_count"] == 0
    assert len(plot_rows) == 61


def test_censored_safety_window_is_measured_without_inventing_three_seconds():
    rows, receipts, bootstrap, spec, points = synthetic_records(ticks=20, abort=True)
    result, plot_rows = audit.audit_records(rows, receipts, bootstrap, spec, points, 100)
    assert result["integrity_issue_counts"] == {}
    assert result["observed_window"]["observed_ticks"] == 20
    assert result["observed_window"]["observed_seconds"] == pytest.approx(1.0)
    assert result["recomputed_guard_rows"][-1]["failures"] == ["maximum_speed"]
    assert result["exploratory_common_1s"] == result["observed_window"]
    assert len(plot_rows) == 21


@pytest.mark.parametrize("corruption,issue", [
    ("frame", "native_frame_cadence"), ("timestamp", "native_timestamp_cadence"),
    ("pedal", "pedal_readback"), ("vx", "raw_state_reconstruction"),
    ("safety", "stored_safety_guard"),
])
def test_corruption_is_not_hidden_by_stored_pass_flags(corruption, issue):
    rows, receipts, bootstrap, spec, points = synthetic_records()
    target = rows[-1]
    if corruption == "frame":
        target["frame"] += 2
    elif corruption == "timestamp":
        target["timestamp"] += 0.01
    elif corruption == "pedal":
        target["current_control"]["brake"] = 0.01
    elif corruption == "vx":
        target["vx"] += 0.1
    elif corruption == "safety":
        target["safety_failures"] = ["maximum_speed"]
    result, _ = audit.audit_records(rows, receipts, bootstrap, spec, points, 100)
    assert result["integrity_issue_counts"][issue] > 0


def test_raw_reconstruction_uses_yaw_only_velocity_even_with_pitch():
    row = dict(actor_snapshot_transform_carla=dict(x=5.0, y=4.0, z=3.0, yaw=90.0, pitch=30.0),
               world_velocity_carla=[1.0, 2.0, 9.0], world_acceleration_carla=[3.0, 4.0, 8.0],
               world_angular_velocity_carla_deg_s=[0.0, 0.0, 10.0])
    reconstructed = audit.reconstruct_state(row)
    assert reconstructed["vx"] == pytest.approx(2.0)
    assert reconstructed["ax"] == pytest.approx(4.0)
    assert reconstructed["z"] == pytest.approx(3.0-1.425*0.5)
    assert reconstructed["vy"] == pytest.approx(1.0+1.425*math.radians(10.0))


def test_strict_shock_threshold_and_absolute_near_stop_are_preserved():
    branch = dict(frame=1, timestamp=0.0, route_progress_m=16.0, vx=0.5)
    rows = [dict(frame=2, timestamp=0.05, route_progress_m=16.02, vx=0.1),
            dict(frame=3, timestamp=0.1, route_progress_m=16.02, vx=-0.3),
            dict(frame=4, timestamp=0.15, route_progress_m=16.02, vx=0.0)]
    result = audit.metrics(branch, rows)
    assert result["signed_rate_below_minus_8_mps2_count"] == 0
    assert result["first_speed_at_or_below_0p05_s"] == 0.15
    rows[1]["vx"] = -0.301
    assert audit.metrics(branch, rows)["signed_rate_below_minus_8_mps2_count"] == 1


def test_automatic_gear_is_observed_but_not_a_false_readback_failure():
    rows, receipts, bootstrap, spec, points = synthetic_records()
    rows[-1]["current_control"]["gear"] = 2
    result, _ = audit.audit_records(rows, receipts, bootstrap, spec, points, 100)
    assert result["integrity_issue_counts"] == {}
    assert result["observed_intervention_gear_counts"] == {1: 59, 2: 1}


def test_existing_artifacts_are_never_overwritten(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    evidence = output / "analysis.json"
    evidence.write_text("retained")
    with pytest.raises(ValueError, match="create-only"):
        audit.write_artifacts(output, {}, {}, False)
    assert evidence.read_text() == "retained"


def test_missing_terminal_trials_fail_before_creating_output(tmp_path):
    plan_path = tmp_path / "plan.json"
    ids = [f"site{site:02d}_{name}" for site in (16, 25) for name in ("A1", "B1", "B2", "A2")]
    specs = [dict(trial_id=ids[index], index=index, arm=arm, throttle=0.01 if arm == "A" else 0.20,
                  site_progress_m=16 if index < 4 else 25) for index, arm in enumerate("ABBAABBA")]
    plan_path.write_text(json.dumps(dict(trial_order=specs)))
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="terminal trials"):
        audit.main(["--plan", str(plan_path), "--campaign", str(tmp_path / "campaign"), "--output", str(output)])
    assert not output.exists()


def retained_trial(tmp_path, censored):
    """HH_261001 - Real temporary JSON/archives test summary and identity integration."""
    rows, receipts, bootstrap, spec, points = synthetic_records(ticks=20 if censored else 60, abort=censored)
    run = tmp_path / "run"
    episode = run / "episode.partial"
    episode.mkdir(parents=True)
    spec.update(index=0, output=str(run))
    source = tmp_path / "source.py"
    source.write_text("# HH_261001 - synthetic pinned source\n")
    archive = run / "source/source.py"
    archive.parent.mkdir()
    archive.write_bytes(source.read_bytes())
    map_file = episode / "active_map_opendrive.xodr"
    map_file.write_text("synthetic map")
    plan = dict(source_sha256={"source.py": audit.sha256(source)}, external_agent_sha256={},
                active_opendrive_sha256=audit.sha256(map_file), resource_guard=dict(running_free_bytes=100))
    plan_path = tmp_path / "plan.json"
    plan_path.write_text(json.dumps(plan))
    (run / "declared_plan.json").write_bytes(plan_path.read_bytes())
    status = "FAILED_RETAINED" if censored else "FINISHED_DIAGNOSTIC_ONLY"
    cleanup = copy.deepcopy(receipts[-1])
    cleanup.update(sequence=len(receipts)+1, before_frame=rows[-1]["frame"], after_ack_frame=rows[-1]["frame"],
                   reason="failure_cleanup_full_brake" if censored else "post_intervention_cleanup_brake")
    cleanup["requested_control"].update(brake=1.0, throttle=0.0)
    receipts.append(cleanup)
    summary = None if censored else audit.metrics(rows[71], rows[72:])
    if summary:
        summary["trial_id"] = spec["trial_id"]
    manifest = dict(bootstrap_observation=bootstrap, trial=spec, fixed_plan_sha256=audit.sha256(plan_path),
                    native_states=len(rows), status=status, summary=summary,
                    actor_body_geometry=dict(actor_id=114), physics=dict(applied={}),
                    failure="CollectionError: fixed safety abort: maximum_speed" if censored else None)
    frozen = dict(summary=summary, status=status, worker_identity_matches=not censored, source_hashes_match=True)
    owner = dict(cleanup_errors=[], ports_released=True, collector_exit_code=1 if censored else 0)
    for path, value in ((episode / "manifest.json", manifest), (run / "throttle_result.json", frozen),
                        (run / "owner_result.json", owner), (run / "owner_plan.json", dict(source_sha256=plan["source_sha256"]))):
        path.write_text(json.dumps(value))
    (episode / "states.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
    (episode / "control_receipts.jsonl").write_text("".join(json.dumps(receipt)+"\n" for receipt in receipts))
    return run, spec, plan, plan_path, points


@pytest.mark.parametrize("censored", [False, True])
def test_trial_integration_preserves_censored_identity_and_null_summary(tmp_path, censored):
    run, spec, plan, plan_path, points = retained_trial(tmp_path, censored)
    result, _ = audit.analyze_trial(run, spec, plan, plan_path, points, tmp_path)
    assert result["integrity_issue_counts"] == {}
    assert result["raw_plan_trial_identity_matches"] is True
    assert result["frozen_wrapper_worker_identity_matches"] is (not censored)
    assert (result["summary_3s"] is None) is censored
    assert result["completed_valid_3s_window"] is (not censored)
    assert result["retained_abort_guard_matches_failure"] is (True if censored else None)


def test_archive_hash_failure_invalidates_completed_summary(tmp_path):
    run, spec, plan, plan_path, points = retained_trial(tmp_path, False)
    (run / "source/source.py").write_text("corrupted source archive")
    result, _ = audit.analyze_trial(run, spec, plan, plan_path, points, tmp_path)
    assert result["source_archive_audit"]["matches"] is False
    assert result["summary_3s"] is None
    assert result["completed_valid_3s_window"] is False


@pytest.mark.parametrize("field", ["timestamp", "route_progress_m"])
def test_nonfinite_interior_native_values_fail_closed(field):
    rows, receipts, bootstrap, spec, points = synthetic_records()
    rows[80][field] = float("nan")
    with pytest.raises(ValueError, match="nonfinite"):
        audit.audit_records(rows, receipts, bootstrap, spec, points, 100)


def test_nonfinite_requested_pedal_cannot_pass_readback():
    rows, receipts, bootstrap, spec, points = synthetic_records()
    receipts[-1]["requested_control"]["throttle"] = float("nan")
    result, _ = audit.audit_records(rows, receipts, bootstrap, spec, points, 100)
    assert result["integrity_issue_counts"]["pedal_readback"] == 1


def test_campaign_asset_failure_suppresses_three_second_comparisons(tmp_path, monkeypatch):
    ids = [f"site{site:02d}_{name}" for site in (16, 25) for name in ("A1", "B1", "B2", "A2")]
    specs = [dict(trial_id=ids[index], index=index, arm=arm, throttle=0.01 if arm == "A" else 0.20,
                  site_progress_m=16 if index < 4 else 25) for index, arm in enumerate("ABBAABBA")]
    route_path = tmp_path / "route.json"
    route_path.write_text(json.dumps(dict(route=[dict(x=0, y=0, distance_m=0), dict(x=100, y=0, distance_m=100)])))
    plan_path = tmp_path / "plan.json"
    plan_path.write_text(json.dumps(dict(trial_order=specs, asset_sha256={str(route_path): "wrong_hash"})))
    campaign = tmp_path / "campaign"
    for spec in specs:
        run = campaign / spec["trial_id"]
        (run / "episode.partial").mkdir(parents=True)
        for relative in ("owner_result.json", "throttle_result.json", "episode.partial/manifest.json"):
            (run / relative).write_text(json.dumps(dict(status="FINISHED_DIAGNOSTIC_ONLY")))
    def synthetic_trial(run, spec, *args):
        return dict(trial_id=spec["trial_id"], arm=spec["arm"], site_progress_m=spec["site_progress_m"],
                    integrity_issue_counts={}, completed_valid_3s_window=True, summary_3s=dict(observed_ticks=60),
                    branch_start_state=None, exploratory_common_1s=None, censored_before_3s=False), []
    monkeypatch.setattr(audit, "analyze_trial", synthetic_trial)
    result, _ = audit.analyze_campaign(campaign, plan_path, tmp_path)
    assert result["all_raw_integrity_checks_pass"] is False
    assert result["completed_3s_trial_count"] == 0
    assert all(trial["summary_3s"] is None for trial in result["trials"])
    assert result["all_predeclared_3s_ab_comparisons_available"] is False
