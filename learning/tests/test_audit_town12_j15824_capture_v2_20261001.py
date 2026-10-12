"""HH_261001 - Approved9kmh plan support cannot erase unrelated failed gates."""

from scripts.e2e import audit_town12_j15824_capture_v2_20261001 as audit


def test_new_plan_recognition_preserves_body_event_failure(monkeypatch, tmp_path):
    monkeypatch.setattr(audit, "ORIGINAL_FROZEN_CONTRACT", lambda *a: {"checks": {
        "twenty_kmh": False, "no_collision_or_lane_invasion_events": False, "source_xodr": True}})
    argv = audit.inherited.capture.collector_arguments(tmp_path, tmp_path / "route.json")
    for key, value in (("--target-speed-kmh", "9"), ("--max-duration-sec", "180"), ("--goal-stop-profile", "comfortable_v2")):
        argv[argv.index(key) + 1] = value
    plan = {"signal_expert_trial": {"target_speed_kmh": 9, "collector_arguments": argv,
        "combined_protocol_revision": {"variant_id": "town12_j15824_expert9_comfortable_v2_180s_v1", "single_factor_causal_ab": False}}}
    manifest = {"capture_contract": {"target_speed_kmh": 9, "maximum_total_duration_sec": 180,
        "goal_stop_profile": {"profile_id": "comfortable_v2", "desired_deceleration_mps2": 1.,
            "normal_brake_cap": .15, "normal_throttle_cap": .20}}}
    result = audit.revised_contract(plan, manifest, {}, {})
    assert result["checks"]["target_speed_as_planned"] and result["checks"]["approved_combined_revision"]
    assert result["status"] == "FAIL" and result["failed_checks"] == ["no_collision_or_lane_invasion_events"]
    manifest["capture_contract"]["goal_stop_profile"]["normal_brake_cap"] = .3
    assert "exact_revised_normal_stop_parameters" in audit.revised_contract(plan, manifest, {}, {})["failed_checks"]


def test_scoped_extension_restores_frozen_original():
    previous = audit.inherited.frozen_contract
    with audit.configured_auditor():
        assert audit.inherited.frozen_contract is audit.revised_contract
    assert audit.inherited.frozen_contract is previous
