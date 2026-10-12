"""HH_260906 - Verify partial-campaign accounting, collision retention, dwell, and create-only publication offline."""

import copy
from pathlib import Path

import pytest

from scripts.e2e import summarize_carla_low_speed_response_v3 as audit

BOUNDS = {"physical_decoder": {"maximum_acceleration_mps2": 2.9, "maximum_deceleration_mps2": 2.9},
          "runtime_speed_rate_gate": {"maximum_acceleration_mps2": 3., "maximum_deceleration_mps2": 6.}}


def manifest_fixture():
    """HH_260906 - Keep every planned case visible even when three have no payload."""
    cases = audit.matrix()
    return {"matrix_id": "low_speed_v3", "status": "failed", "matrix": cases,
            "case_ledger": [{"case_id": case["case_id"], "status": status} for case, status in zip(cases,
                ["complete", "complete", "failed"] + ["not_run_after_failure"] * 3)],
            "completed_cases": [{"case_id": case["case_id"], "status": "complete"} for case in cases[:2]]}


def test_six_case_denominator_accepts_only_two_complete_one_failed_three_not_run():
    manifest = manifest_fixture()
    audit.verify_denominator(manifest, [case["case_id"] for case in manifest["matrix"][:3]])


@pytest.mark.parametrize("mutation", ["three_planned", "three_complete", "hide_failed", "replace_not_run", "invent_directory", "no_failed_directory", "changed_rate", "campaign_success"])
def test_partial_denominator_cannot_be_shrunk_or_promoted(mutation):
    manifest = manifest_fixture()
    directories = [case["case_id"] for case in manifest["matrix"][:3]]
    if mutation == "three_planned": manifest["matrix"] = manifest["matrix"][:3]
    if mutation == "three_complete": manifest["completed_cases"].append({"case_id": directories[2], "status": "complete"})
    if mutation == "hide_failed": manifest["case_ledger"].pop(2)
    if mutation == "replace_not_run": manifest["case_ledger"][3]["status"] = "complete"
    if mutation == "invent_directory": directories.append(manifest["matrix"][3]["case_id"])
    if mutation == "no_failed_directory": directories.pop()
    if mutation == "changed_rate": manifest["matrix"][2]["ramp_rate_per_second"] = .1
    if mutation == "campaign_success": manifest["status"] = "complete"
    with pytest.raises(audit.base.EvidenceError):
        audit.verify_denominator(manifest, directories)


def collision_case_fixture():
    """HH_260906 - A failing tick is in raw data but absent from successful-command metadata."""
    case = audit.matrix()[2]
    rows = []
    phases = ["settle"] * 70 + ["launch_prepare"] + ["post_handoff_ramp"] * 3
    speeds = [0.] * 70 + [.5, 1., 2., 1.]
    collision = {"frame": 173, "other_actor_type": "static.vegetation"}
    for index, (phase, speed) in enumerate(zip(phases, speeds)):
        throttle = 0. if phase == "settle" else .15 if phase == "launch_prepare" else .15 + .05 * (index - 70) / 20.
        control = dict(throttle=throttle, brake=1. if phase == "settle" else 0., steer=0.,
                       hand_brake=False, reverse=False, manual_gear_shift=False, gear=1)
        rows.append(dict(frame=100 + index, timestamp=index * .05, phase=phase, vx=speed, vy=0., x=index * .01,
                         y=0., travel_m=index * .01, route_progress_m=index * .01, route_cte_m=0.,
                         requested_control=control.copy(), applied_control=control.copy(),
                         collision=[collision] if index == 73 else []))
    report = {"schema": "carla.low_speed_response_calibration.v1", "case": case, "status": "failed", "matrix_id": "low_speed_v3",
              "training_data": False, "cleanup": {"completed": True, "errors": []}, "state_count": len(rows),
              "phase_counts": {phase: phases.count(phase) for phase in audit.phase_names(rows)}, "collision_events": [collision],
              "post_despawn_empty_world_frame": 174, "maximum_speed_mps": 2., "final_speed_mps": 1.,
              "safety_failure": "collision", "error": "CalibrationError: collision", "motion_analysis": {"measurements": audit.rates(rows, BOUNDS)},
              "two_stage": {"status": "post_handoff_ramp", "handoff_frame": 170, "handoff_speed_mps": .5,
                            "launch_ticks": 1, "post_handoff_ticks": 2, "last_frame": 172, "last_speed_mps": 2., "reached_measured_speed_stop": False}}
    return case, report, rows


def test_collision_tick_and_all_phase_boundaries_remain_in_summary():
    case, report, rows = collision_case_fixture()
    result = audit.verify_case(case, report, rows, BOUNDS, "failed")
    assert result["state_count"] == 74 and result["collision_observation"]["frame"] == 173
    assert result["collision_observation"]["native_speed_rate_mps2"] == pytest.approx(-20.)
    assert result["whole_case_training_quality_pass"] is False
    for cadence in result["measurements"].values():
        assert cadence["phases"]["post_handoff_ramp"]["phase_boundary_intervals"]


@pytest.mark.parametrize("mutation", ["remove_collision_tick", "hide_collision_event", "pretend_last_success", "changed_steer", "changed_rate_summary", "deleted_boundary"])
def test_collision_or_command_evidence_cannot_be_filtered(mutation):
    case, report, rows = collision_case_fixture()
    if mutation == "remove_collision_tick": rows.pop()
    if mutation == "hide_collision_event": rows[-1]["collision"] = []
    if mutation == "pretend_last_success": report["two_stage"]["post_handoff_ticks"] = 3
    if mutation == "changed_steer": rows[71]["applied_control"]["steer"] = .1
    if mutation == "changed_rate_summary": report["motion_analysis"]["measurements"]["native_20hz"]["phases"]["all"]["minimum_speed_rate_mps2"] = 0.
    if mutation == "deleted_boundary": report["motion_analysis"]["measurements"]["native_20hz"]["phases"]["launch_prepare"]["phase_boundary_intervals"] = []
    with pytest.raises(audit.base.EvidenceError):
        audit.verify_case(case, report, rows, BOUNDS, "failed")


def test_both_derived_offsets_are_preserved_even_when_one_does_not_sample_final_collision():
    _, _, rows = collision_case_fixture()
    output = audit.rates(rows, BOUNDS)
    assert output["derived_10hz_offset_0"]["sample_count"] == 37
    assert output["derived_10hz_offset_1"]["sample_count"] == 37
    native = output["native_20hz"]["phases"]["all"]["physical_decoder"]["violation_intervals"]
    assert native[-1]["to_frame"] == 173
    offset0 = output["derived_10hz_offset_0"]["phases"]["all"]["physical_decoder"]["violation_intervals"]
    assert all(item["to_frame"] != 173 for item in offset0)


@pytest.mark.parametrize("change", [{"timestamp": 10.}, {"frame": 1000}, {"vx": float("nan")}])
def test_rates_fail_closed_for_gaps_or_nonfinite_velocity(change):
    _, _, rows = collision_case_fixture()
    rows[30].update(change)
    with pytest.raises(audit.base.EvidenceError):
        audit.rates(rows, BOUNDS)


def test_coast_dwell_restarts_after_speed_recrossing_threshold():
    rows = [dict(frame=index, timestamp=index * .05, vx=speed, vy=0., phase="prepare" if index == 0 else "coast_hold",
                 travel_m=index * .01, route_progress_m=index * .01)
            for index, speed in enumerate([3.] + [.09] * 30 + [.11] + [.09] * 41)]
    recorded = {"maximum_speed_mps": .1, "required_continuous_seconds": 2., "sample_count": len(rows) - 1,
                "measurement_valid": True, "first_below_threshold_frame": 1, "first_verified_dwell_frame": 72,
                "verified_dwell_observed": True, "longest_observed_dwell_seconds": rows[72]["timestamp"] - rows[32]["timestamp"],
                "first_verified_dwell_timestamp": rows[72]["timestamp"], "first_verified_dwell_travel_m": rows[72]["travel_m"],
                "first_verified_dwell_route_progress_m": rows[72]["route_progress_m"]}
    result = audit.coast_observations(rows, recorded)
    assert result["first_verified_continuous_2s_dwell"]["frame"] == 72 and result["goal_position_tested"] is False
    recorded["first_verified_dwell_frame"] = 41
    with pytest.raises(audit.base.EvidenceError):
        audit.coast_observations(rows, recorded)


@pytest.mark.parametrize("kind", ["directory", "file", "dangling_symlink"])
def test_publisher_refuses_existing_target_before_reading_private_data(tmp_path, monkeypatch, kind):
    output = tmp_path / "category"
    if kind == "directory": output.mkdir()
    if kind == "file": output.write_text("existing")
    if kind == "dangling_symlink": output.symlink_to(tmp_path / "missing")
    monkeypatch.setattr(audit, "verify_trial", lambda _: pytest.fail("must not read evidence for an existing target"))
    with pytest.raises(audit.base.EvidenceError, match="already exists"):
        audit.publish(tmp_path / "raw", output)


def test_publication_redacts_account_paths_and_network_endpoints_without_altering_hashes():
    expected_sha = "a" * 64
    result = audit.sanitize({"source": "/home/alice/autoware_e2e/artifacts/report.json", "other": "/home/bob/data",
                             "endpoint": "220.90.5.14", "raw_sha256": expected_sha})
    assert "/home/" not in str(result) and "220.90.5.14" not in str(result)
    assert result["source"] == "${REPO_ROOT}/artifacts/report.json" and result["raw_sha256"] == expected_sha
