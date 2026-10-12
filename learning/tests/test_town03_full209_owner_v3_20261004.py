"""HH_261004 - Verify fresh owner identity and mandatory inherited failure conjunction with synthetic CPU fixtures."""

from contextlib import contextmanager
import copy
import json
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import research_temporal_route_joint_town03_straight_full209_v3_20261004 as m
from test_temporal_route_joint_town03_straight_v3_20261004 import pins, fake_model_and_patches, entered
from test_town03_full209_collector_v3_20261004 import flow_fixture, digest
from test_two_phase_owned_signal_restoration_v1_20261004 import prepared


@contextmanager
def scope():
    """HH_261004 - Omit only asset/model inspection in this CPU scope; retain every actual private owner alias."""
    original = m.previous.previous
    threads = original.straight.runtime_owner.previous.previous.threads
    with mock.patch.object(original.numeric, "verify_exports"), mock.patch.object(threads, "evidence_assets", return_value={}):
        with m.activated(pins()):
            yield


def test_exact_frozen_V2_owner_and_collector_sources():
    assert digest(m.previous.__file__) == m.PARENT_SHA256
    m.live.verify_frozen()
    assert m.SCHEMA == m.live.OWNER_SCHEMA and m.SCHEMA.endswith(".v3")
    assert m.live.SCHEMA.endswith(".v1")


def test_full_interval_source_guards_new_interventions_and_media_rules_are_prospective():
    old, original = m.previous, m.previous.previous
    names = ("paths", "capture_argv", "fixed_contract", "runtime_sources", "diagnostic_report", "live", "MODULE", "SCHEMA")
    before = {name:getattr(old,name) for name in names}
    original_before = {name:getattr(original,name) for name in names}
    with scope():
        plan, argv = m.fixed_contract("v3", True), m.capture_argv("v3", True, "a"*64)
        assert m.base.live is original.live is old.live is m.live
        assert m.base.paths("v3") == m.paths("v3")
        # HH_261004 - Dummy model identities intentionally omit the old asset manifest, not the fresh imported-source closure.
        with mock.patch.object(m, "_sources", return_value={}):
            source_pins = m.runtime_sources()
    assert {name:getattr(old,name) for name in names} == before
    assert {name:getattr(original,name) for name in names} == original_before
    assert plan["schema"] == m.SCHEMA and plan["capture_raw_ABI_schema"] == m.live.SCHEMA
    assert plan["run_id"] == "v3" and plan["output"] == str(m.paths("v3")[0])
    assert plan["total_capture_seconds"] == 110
    assert plan["maximum_native_state_count"] == 2200 and plan["maximum_camera_bundle_count"] == 1100
    assert plan["actual_callback_timing"]["maximum_ACK"] == 2204
    assert plan["actual_callback_timing"]["maximum_callbacks"] == 24000
    assert plan["actual_callback_timing"]["maximum_native_stage_markers"] == 17600
    assert plan["actual_denominator_is_not_prospective_maximum"] is True
    assert plan["all_actual_rows_until_original_catalog_goal_or_original_safety_abort_required"] is True
    assert plan["model_speed_cap_mps"] == 2.5 and plan["native_actor_planar_speed_maximum_mps"] == 2.7
    assert plan["controller_target_cap_mps"] == 2.4 and plan["speed_preview_s"] == .5
    assert plan["original_catalog_goal_tolerance_m"] == 2.5 and plan["policy_deadline_seconds"] == .1
    assert plan["body_clearance_screen"]["lane_inset_m"] == .1
    assert plan["precursor_motion_screen"]["cte_p95_maximum_m"] == .35
    assert plan["precursor_motion_screen"]["cte_absolute_maximum_m"] == .75
    assert plan["attempts"] == 1 and plan["automatic_retry"] is False
    assert argv[argv.index("--max-duration-sec")+1] == "110"
    assert argv[argv.index("--stationary-tail-sec")+1] == "0"
    assert argv[argv.index("--port")+1] == "2440"
    worker = plan["worker_command"]
    assert worker[worker.index("-m")+1] == m.MODULE and worker[worker.index("--run-id")+1] == "v3"
    assert plan["synchronous_parallel_JPEG_after_ACK"]["worker_count"] == 3
    assert plan["synchronous_parallel_JPEG_after_ACK"]["source_BGRA_and_selected_callback_SHA_required"] is True
    assert plan["synchronous_parallel_JPEG_after_ACK"]["actual_closed_loop_latency_gain_not_yet_proven"] is True
    assert plan["two_phase_signal_restoration"]["native_restoration_ticks_outside_capture_and_NN_denominator"] == 2
    assert plan["owned_green_signal_fixture"]["post_vehicle_restoration_native_ticks"] == 2
    assert plan["prospective_visual_publication"]["gallery"].endswith("13_town03_full_route209_v3")
    assert plan["prospective_visual_publication"]["complete_actual_interval_including_failures_required"] is True
    assert plan["prospective_visual_publication"]["outcome_or_failure_dependent_frame_selection"] is False
    for name in ("independent_validation", "full_RTE01_approved", "model_promoted", "learned_STOP_claim"):
        assert plan[name] is False
    assert plan["official_completed_main_domains"] == plan["official_completed_subfunctions"] == 0
    for name in ("tools/e2e/research_temporal_route_joint_town03_straight_full209_v3_20261004.py",
                 "tools/e2e/collect_temporal_route_joint_town03_straight_full209_v3_20261004.py",
                 "tools/e2e/parallel_exact_jpeg_after_ack_v1_20261004.py",
                 "tools/e2e/two_phase_owned_signal_restoration_v1_20261004.py",
                 "learning/tests/test_town03_full209_collector_v3_20261004.py",
                 "learning/tests/test_town03_full209_owner_v3_20261004.py"):
        assert source_pins[name] == digest(m.ROOT / name)


@pytest.mark.parametrize("run_id", ["v1", "v2", "v4", "../v3", None])
def test_unknown_owner_identity_rejects(run_id):
    with pytest.raises(RuntimeError): m.paths(run_id)


def test_standalone_capture_denied_before_owner():
    with pytest.raises(RuntimeError, match="exclusive V3 owner"):
        m.capture_argv("v3", True)


def test_nested_owner_rejects_and_restores_all_three_layers():
    old = m.previous
    original = old.previous
    before = (old.paths, original.paths, m.base.paths)
    with pytest.raises(RuntimeError, match="fresh V3 owner cannot nest"):
        with scope():
            with m.activated(pins()): pass
    assert not m._active and not old._active and not original._active
    assert (old.paths, original.paths, m.base.paths) == before


def test_inner_exception_restores_frozen_owner_aliases_without_suppressing_error():
    marker = RuntimeError("synthetic fresh V3 owner body failed")
    before = (m.previous.fixed_contract, m.previous.diagnostic_report, m.previous.previous.live, m.base.live)
    with pytest.raises(RuntimeError) as caught:
        with scope(): raise marker
    assert caught.value is marker
    assert (m.previous.fixed_contract, m.previous.diagnostic_report, m.previous.previous.live, m.base.live) == before
    assert not m._active and not m.previous._active and not m.previous.previous._active


@pytest.mark.parametrize("failure", [False, True])
def test_real_inherited_capture_factory_owns_new_recorder_and_late_bound_fixture_without_capture(tmp_path, failure):
    # HH_261004 - Enter the authentic V1/V2/V3 collector chain; replace only model identity/introspection and BasicAgent imports.
    original = m.live.previous.previous
    straight_recording = original.v2
    fixture = original.fixture
    model, patches = fake_model_and_patches()
    before = (original.collector._save_jpeg, original.collector.AcknowledgedControlTransport,
        straight_recording.ExactJPEGAfterACK, fixture.GreenSignalFixture, fixture.OWNER_SCHEMA,
        original.callback.CallbackLedger, original.callback.callback_capture_factory)
    marker = RuntimeError("synthetic genuine factory scope failure before capture")
    recorder = None
    def enter():
        nonlocal recorder
        with scope(), entered(patches):
            args = m.live.frozen.parse_args(m.capture_argv("v3", True, "a"*64))
            args.output = tmp_path / "episode"
            with m.live.activated(model, args):
                recorder = straight_recording._recorder()
                assert type(recorder) is m.live.jpeg.ParallelExactJPEGAfterACK
                assert m.live.jpeg.assert_runtime_recorder(original.collector) is recorder
                assert straight_recording.ExactJPEGAfterACK is m.live.jpeg.ParallelExactJPEGAfterACK
                assert fixture.GreenSignalFixture is m.live.signals.TwoPhaseGreenSignalFixture
                assert fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"] == m.live.OWNER_SCHEMA
                assert fixture.GreenSignalFixture.__init__.__globals__["OWNER_SCHEMA"] == m.live.OWNER_SCHEMA
                factory = original.full_capture_factory()
                function = __import__("inspect").getclosurevars(factory).nonlocals["function"]
                for name in ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
                             "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"):
                    assert function.__globals__[name] is getattr(original.collector, name)
                assert function.__globals__["_save_jpeg"].__self__ is recorder
                assert function.__globals__["full_goal_control_reason"] is original.full_goal_control_reason
                assert original.callback._ledger.capture_started is False
                if failure: raise marker
    if failure:
        with pytest.raises(RuntimeError) as caught: enter()
        assert caught.value is marker
    else: enter()
    assert recorder is not None and recorder.closed
    assert before == (original.collector._save_jpeg, original.collector.AcknowledgedControlTransport,
        straight_recording.ExactJPEGAfterACK, fixture.GreenSignalFixture, fixture.OWNER_SCHEMA,
        original.callback.CallbackLedger, original.callback.callback_capture_factory)
    assert not (tmp_path / "episode").exists() and not (tmp_path / "episode.partial").exists()


def diagnostic_fixture(tmp_path):
    """HH_261004 - Use the real new scalar verifiers and frozen readers on synthetic records, not fake success flags."""
    values = flow_fixture(tmp_path)
    episode, output = values["episode"], tmp_path
    with prepared(False) as (fixture, owner_guard):
        fixture.restore()
        signal_record = copy.deepcopy(fixture.record)
    (episode / "owned_green_signal_fixture.json").write_text(json.dumps(signal_record))
    for name, rows in (("recording_after_ack.jsonl",values["recordings"]),
                       ("camera_frames.jsonl",values["cameras"]),
                       ("owned_camera_source.jsonl",values["owned_sources"]),
                       ("wall_timing.jsonl",values["timings"]), ("states.jsonl",values["states"])):
        (episode / name).write_text("".join(json.dumps(row)+"\n" for row in rows))
    callback = values["callback"]
    callback.update(schema=m.live.SCHEMA+".callback_receipts", clock="time.perf_counter_ns_monotonic",
        callback_count=len(callback["callbacks"]), camera_bindings=[dict(role=role) for role in m.live.jpeg.CAMERA_ORDER],
        policy_stage_marks=[], errors=[])
    (episode / "callback_timing_receipts.json").write_text(json.dumps(callback))
    raw = dict(values["raw_pins"])
    for path in episode.iterdir():
        if path.is_file(): raw[str(path)] = digest(path)
    return output, episode, raw


@pytest.mark.parametrize("failure", ["none", "inherited", "signal", "parallel", "closure", "raw_before", "raw_after"])
def test_new_checks_never_overwrite_inherited_or_independent_failure_and_keep_channels(tmp_path, failure):
    output, episode, raw = diagnostic_fixture(tmp_path)
    original = m.previous.previous
    if failure in ("signal", "parallel"):
        path = episode / ("owned_green_signal_fixture.json" if failure == "signal" else "recording_after_ack.jsonl")
        if failure == "signal":
            value = json.loads(path.read_text())
            value["frozen_state_restore_readback"][0]["state"] = "Unknown"
            path.write_text(json.dumps(value))
        else:
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rows[-1]["written_camera_count"] = 5
            path.write_text("".join(json.dumps(row)+"\n" for row in rows))
        raw[str(path)] = digest(path)
    if failure == "raw_before": raw[str(episode / "states.jsonl")] = "0"*64
    inherited = dict(learned_driving_screen_passed=failure != "inherited", raw_source_sha256=raw,
        retained_prior_failure="SYNTHETIC_INHERITED_FAILURE" if failure == "inherited" else None)
    def closure(output, plan):
        if failure == "closure": raise RuntimeError("synthetic closure changed")
        if failure == "raw_after": (episode / "states.jsonl").write_text("changed during independent audit\n")
    # HH_261004 - Only the prior result/declared closure are scalar inputs; both fresh verifiers and every raw read/hash are real.
    with mock.patch.object(m, "_diagnostic", return_value=copy.deepcopy(inherited)), \
            mock.patch.object(original.right, "verify_closure", side_effect=closure):
        report = m.diagnostic_report(output, {}, {})
    assert report["schema"] == m.SCHEMA+".diagnostic_report"
    assert report["status"] == ("PASS_KNOWN_DEV_FULL209_RESEARCH" if failure == "none" else "FAIL_KNOWN_DEV_FULL209_RESEARCH")
    assert report["learned_driving_screen_passed"] is (failure == "none")
    assert report["full209_research_motion_body_goal_callback100ms_passed"] is (failure == "none")
    assert report["inherited_full209_research_screen_passed_before_new_conjunction"] is (failure != "inherited")
    if failure in ("none", "inherited"):
        if failure == "inherited": assert report["retained_prior_failure"] == "SYNTHETIC_INHERITED_FAILURE"
        assert report["two_phase_signal_restoration_passed"] is True
        assert report["synchronous_parallel_after_ACK_recording_passed"] is True
        assert report["fresh_intervention_full_raw_and_source_closure_passed"] is True
    elif failure == "signal":
        assert report["two_phase_signal_restoration_passed"] is False
        assert report["synchronous_parallel_after_ACK_recording_passed"] is True
        assert report["two_phase_signal_restoration_diagnostic_error"]
    elif failure == "parallel":
        assert report["two_phase_signal_restoration_passed"] is True
        assert report["synchronous_parallel_after_ACK_recording_passed"] is False
        assert report["parallel_after_ACK_recording_diagnostic_error"]
    else:
        assert report["fresh_intervention_full_raw_and_source_closure_passed"] is False
        assert report["fresh_intervention_diagnostic_error"]
    assert report["official_completed_main_domains"] == report["official_completed_subfunctions"] == 0
    for key in ("independent_validation", "full_RTE01_approved", "model_promoted", "learned_STOP_claim",
                "actual_closed_loop_gain_claimed", "vehicle_control_approved"):
        assert report[key] is False
