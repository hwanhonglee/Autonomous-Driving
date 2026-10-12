"""HH_261002 - Synthetic publication identity/teacher-only scope tests; no real archives or model/simulator execution."""
import json

import pytest

from learning.tests import test_publish_temporal_route_joint_town03_right_20261002 as fixture
from scripts.e2e import publish_temporal_route_joint_town03_right_v2_20261002 as revision

p = revision.previous


@pytest.fixture
def archive(tmp_path, monkeypatch):
    value = fixture.archive.__wrapped__(tmp_path, monkeypatch)
    monkeypatch.setattr(revision, "publication_sources", lambda: set())
    return value


def revised_caller(archive):
    path, _, caller, run = archive
    caller["schema"] = revision.CALLER_SCHEMA
    caller["output"] = str(p.output_for(run))
    caller["run_files_sha256"] = {name: p.prior.digest(run / name) for name in p.inventory(run)}
    return path, fixture.write(path, caller), caller, run


def move_to_v3(archive):
    path, _, caller, old_run = archive; run = old_run.with_name(old_run.name[:-1] + "3"); old_run.rename(run)
    plan = json.loads((run / "declaration.json").read_bytes()); plan.update(output=str(run), schema=p.OWNER_SCHEMA[:-1] + "3")
    digest = fixture.write(run / "declaration.json", plan)
    owner = json.loads((run / "owner_result.json").read_bytes()); owner["declaration_sha256"] = digest
    fixture.write(run / "owner_result.json", owner)
    diagnostic = json.loads((run / "diagnostic_report.json").read_bytes())
    diagnostic.update(schema=plan["schema"] + ".diagnostic_report", original_owner=owner,
        raw_source_sha256={str(f): p.prior.digest(f) for f in (run / "episode").rglob("*") if f.is_file()})
    fixture.write(run / "diagnostic_report.json", diagnostic); caller["run_root"] = str(run)
    return revised_caller((path, None, caller, run))


@pytest.mark.parametrize("version", [1, 2, 3])
def test_three_exact_roles_read_original_schema_and_failures_without_NN_invention(archive, version):
    with revision.activated():
        if version == 1: value = revised_caller(fixture.absent_preNN_fixture(archive))
        elif version == 3: value = move_to_v3(archive)
        else: value = revised_caller(archive)
        _, bundle, pins = p.load_archive(*value[:2]); pins.recheck()
        assert bundle["diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
        assert bundle["output"].name == revision.RUN_OUTPUTS[value[3].name]
        if version == 1: assert bundle["control_census"]["observed_prior_learned_native_effect_count"] == 0
        else: assert bundle["control_census"]["observed_prior_learned_native_effect_count"] == 4


def test_full_93_4_second_teacher_only_interval_is_not_a_startup_or_NN_success():
    states, cameras, _, receipts, geometry = fixture.native_records(1868)
    for row in receipts[1:]: row["reason"] = "next_driving_control"
    by_frame = p.verify_records(states, cameras, [], receipts, geometry, unlogged_preNN=True)
    bundle = dict(episode=p.ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v2/episode",
        states=states, cameras=cameras, receipts=receipts, commands=[], by_frame=by_frame, unlogged_preNN=True,
        diagnostic=dict(joint_turn_motion_body_policy_wall100ms_passed=False, diagnostic_error="RightTurnObserver missing"))
    bundle["control_census"] = p.control_census(bundle)
    with revision.activated():
        title, _ = p.outcome_text(bundle); text = p.markdown(bundle, p.png_indices(934))
    assert "TEACHER-ONLY" in title and "INTEGRATION FAILURE" in title and "startup" not in title
    assert "전체 주행은 교사 제어" in text and "NN 우회전 성공으로 세지 않습니다" in text
    assert "RightTurnObserver missing" in text
    assert sum(p.playback(bundle, p.camera_indices(934))) == 93400
    assert bundle["control_census"]["acknowledged_learned_command_count"] == 0


@pytest.mark.parametrize("field", ["ignore_traffic_lights", "frozen", "all_original_hazard_detectors_and_emergency_returns_unchanged"])
def test_v3_cannot_bypass_old_v2_green_hazard_disclosure_guard(archive, field):
    with revision.activated():
        path, _, caller, run = move_to_v3(archive)
        plan = json.loads((run / "declaration.json").read_bytes()); plan["owned_green_signal_fixture"][field] = not plan["owned_green_signal_fixture"][field]
        digest = fixture.write(run / "declaration.json", plan)
        owner = json.loads((run / "owner_result.json").read_bytes()); owner["declaration_sha256"] = digest; fixture.write(run / "owner_result.json", owner)
        diagnostic = json.loads((run / "diagnostic_report.json").read_bytes()); diagnostic["original_owner"] = owner
        fixture.write(run / "diagnostic_report.json", diagnostic)
        with pytest.raises(Exception): p.load_archive(*revised_caller((path, None, caller, run))[:2])


def test_unknown_v4_and_cross_role_outputs_are_not_future_wildcards():
    run = p.ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v4"
    with revision.activated(), pytest.raises(Exception): p.output_for(run)


def test_source_closure_includes_wrapper_and_all_unchanged_old_eleven_providers():
    old = p.publication_sources(); new = revision.publication_sources()
    assert old <= new and set(new) - set(old) == {p.Path(revision.__file__)}
    assert p.prior.digest(revision.FROZEN_PATH) == revision.FROZEN_SHA


def test_scope_restores_every_provider_after_exception_and_rejects_nesting():
    names = ("SCHEMA", "CALLER_SCHEMA", "RUN_OUTPUTS", "publication_sources", "outcome_text", "markdown", "load_archive")
    saved = {name: getattr(p, name) for name in names}
    numerical = (p.verify_records, p.playback, p.render, p.renderer_call, p.control_census)
    with pytest.raises(RuntimeError):
        with revision.activated():
            assert p.SCHEMA == revision.SCHEMA and (p.verify_records, p.playback, p.render, p.renderer_call, p.control_census) == numerical
            with pytest.raises(Exception):
                with revision.activated(): pass
            raise RuntimeError("synthetic publication failure")
    assert all(getattr(p, name) is value for name, value in saved.items()) and revision._active is False


def test_unknown_provider_cannot_be_silently_replaced(monkeypatch):
    monkeypatch.setattr(p, "load_archive", lambda *args: None)
    with pytest.raises(Exception):
        with revision.activated(): pass


def test_v3_possible_actual_NN_path_keeps_original_verdict_and_prior_selector(archive):
    selector = p.prior.observed_prior_ack
    with revision.activated():
        value = move_to_v3(archive); _, bundle, _ = p.load_archive(*value[:2])
        assert "FAIL / all failed tail retained" == p.outcome_text(bundle)[0]
        receipt = p.renderer_call(p.core.displayed_receipt, bundle, 2)
        assert receipt["prior_learned_plan_drawn"] is True and receipt["observed_frame"] == 1004
    assert p.prior.observed_prior_ack is selector
