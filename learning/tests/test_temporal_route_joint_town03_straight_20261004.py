"""HH_261004 - Prove the fresh Town03 straight wrapper keeps the fixed45 GOOD6000 contract on CPU only."""

from contextlib import contextmanager
import copy
import inspect
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_straight_20261004 as live
from scripts.e2e import research_temporal_route_joint_town03_straight_20261004 as owner


def pins():
    return {name + "_sha256": str(index) * 64
            for index, name in enumerate(("weights", "metadata", "checkpoint", "declaration"), 1)}


def good6000_pins():
    return dict(
        weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
        metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
        checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
        declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")


@contextmanager
def owner_scope(values=None):
    with mock.patch.object(owner.numeric, "verify_exports"), \
         mock.patch.object(owner.runtime_owner.previous.previous.threads, "evidence_assets", return_value={}):
        with owner.activated(pins() if values is None else values):
            yield


def arguments():
    with owner_scope():
        return live.frozen.parse_args(owner.base.capture_argv("v1", True, "a" * 64))


def route():
    return live.collector.load_route(owner.ROUTE)


def test_exact_contract_is_one_known_dev_centered_fixed45_without_turn_or_promotion():
    with owner_scope():
        contract = owner.fixed_contract("v1", True)
    assert contract["schema"] == live.OWNER_SCHEMA
    assert contract["town"] == "Town03" and contract["scenario"] == "straight"
    assert contract["dataset_split"] == "val" and contract["known_DEV_site_reused"] is True
    assert contract["independent_validation"] is False and contract["attempts"] == 1
    assert contract["total_capture_seconds"] == 45 and contract["sensor_and_ACK_screen"]["native_state_count"] == 900
    assert contract["sensor_and_ACK_screen"]["camera_bundle_count"] == 450
    assert contract["precursor_motion_screen"]["net_learned_progress_minimum_m"] == 90.0
    assert contract["precursor_motion_screen"]["cte_p95_maximum_m"] == 0.35
    assert contract["precursor_motion_screen"]["cte_absolute_maximum_m"] == 0.75
    assert contract["model_speed_cap_mps"] == 2.5 and contract["native_actor_planar_speed_maximum_mps"] == 2.7
    assert contract["body_clearance_screen"]["lane_inset_m"] == 0.1
    assert contract["observed_wall_latency_screen"]["fresh_learned_bundle_delivery_to_ACK_maximum_s"] == 0.1
    assert contract["additive_native_speed_rate_guard_mps2"] == dict(
        minimum=-6.0, maximum=3.0, warmup_to_first_driving_interval_included=True)
    assert contract["controller_target_cap_mps"] == 2.4 and contract["speed_preview_s"] == 0.5
    assert contract["cpu_thread_numerical_preset"]["intra_op_threads"] == 4
    assert contract["cpu_thread_numerical_preset"]["inter_op_threads"] == 1
    assert contract["centered_lateral_offset_ros_m"] == contract["centered_yaw_offset_ros_deg"] == 0.0
    assert contract["original_navigation_sha256"] == live.ROUTE_SHA and contract["full_original_navigation_points"] == 211
    assert contract["fixed45_full_failed_budget_tail_required"] and contract["early_metric_success_forbidden"]
    assert not contract["full_RTE01_approved"] and not contract["model_promoted"]
    assert "full_RTE03_approved" not in contract and "event" not in contract
    assert "110s" not in json.dumps(contract) and "basic_turn" not in json.dumps(contract)
    assert "known_v1_hazard_subtype_proven" not in json.dumps(contract)
    assert contract["shared_runtime_lease"].endswith("storage/assets/locks/autoware_e2e_runtime.lock")
    assert contract["port_triplet"] == [2440, 2441, 2442]


def test_public_plan_and_preflight_reach_validator_without_starting_simulator():
    imported = SimpleNamespace(stdout=str(owner.base.EGG) + "/carla.so\n"
        + str(owner.CARLA_ROOT / "PythonAPI/carla/agents/__init__.py") + "\n1.8.0\n")
    with owner_scope():
        with mock.patch.object(owner.base, "digest", return_value="a" * 64), \
             mock.patch.object(owner.base, "runtime_sources", return_value={"code": "a" * 64}), \
             mock.patch.object(owner.base, "external_sources", return_value={"API": "a" * 64}), \
             mock.patch.object(owner.base, "ASSETS", {path: "a" * 64 for path in owner.base.ASSETS}):
            plan = owner.base.build_plan("v1", True)
        with mock.patch.object(owner.base, "verify_plan", return_value=plan), \
             mock.patch.object(owner.base.lifecycle, "_assert_no_other_carla"), \
             mock.patch.object(owner.base, "ports_free"), \
             mock.patch.object(owner.base, "disk_check", return_value={}), \
             mock.patch.object(owner.base.subprocess, "run", return_value=imported), \
             mock.patch.object(owner.base, "load_npz", return_value=SimpleNamespace(
                 _research_temporal_checkpoint_sha256=pins()["checkpoint_sha256"])):
            _, result = owner.base.preflight("v1", "a" * 64)
    assert result["status"] == "PREFLIGHT_PASS_NO_SIMULATOR"
    assert result["learned_control_executed"] is False


@pytest.mark.parametrize("change", [None, "seed", "duration", "offset", "town", "scenario", "points", "length", "route"])
def test_exact_capture_validator_rejects_every_undeclared_site_or_runtime_change(change, tmp_path):
    args = arguments()
    actual_route = copy.deepcopy(route())
    if change == "seed":
        args.seed += 1
    elif change == "duration":
        args.max_duration_sec = 44
    elif change == "offset":
        args.spawn_z_offset_m = 0.6
    elif change == "town":
        actual_route["town"] = "Town07"
    elif change == "scenario":
        actual_route["scenario"] = "right"
    elif change == "points":
        actual_route["route"].pop()
    elif change == "length":
        actual_route["route_length_m"] += 1
    elif change == "route":
        replacement = tmp_path / "route.json"
        replacement.write_text(json.dumps(actual_route))
        args.route_file = replacement
    if change is None:
        live.validate_capture(args, actual_route)
        assert live.profile(args.seed)["case_id"].endswith("centered_fixed45")
    else:
        with pytest.raises(Exception):
            live.validate_capture(args, actual_route)


def interval_fixture():
    states = [dict(frame=100 + index, capture_phase="stationary_warmup" if index < 24 else "driving")
              for index in range(900)]
    cameras = [dict(frame=row["frame"]) for row in states[::2]]
    manifest = dict(status="failed",
        error="CollectionError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail",
        result=dict(state_count=900, camera_anchor_count=450),
        capture_contract=dict(capture_phase_schedule=dict(stationary_tail=dict(scheduled_ticks=0))))
    return manifest, states, cameras


@pytest.mark.parametrize("corruption", [None, "gap", "camera", "warmup", "error", "goal_false", "goal_true", "count", "short"])
def test_only_complete_fixed45_failed_budget_denominator_is_accepted(corruption):
    manifest, states, cameras = interval_fixture()
    if corruption == "gap":
        states[100]["frame"] += 1
    elif corruption == "camera":
        cameras.pop()
    elif corruption == "warmup":
        states[23]["capture_phase"] = "driving"
    elif corruption == "error":
        manifest["error"] = "favorable early stop"
    elif corruption == "goal_false":
        manifest["result"]["goal_reached"] = False
    elif corruption == "goal_true":
        manifest["result"]["goal_reached"] = True
    elif corruption == "count":
        manifest["result"]["state_count"] = 899
    elif corruption == "short":
        states.pop()
    assert owner.complete_interval(manifest, states, cameras) is (corruption is None)


def test_straight_observer_never_stops_capture_early_and_recomputed_replay_is_denominator_only():
    original = route()
    observer = live.StraightIntervalObserver(original)
    commands, states = [], []
    for index, point in enumerate(original["route"][:4]):
        frame = 200 + index
        state = dict(frame=frame, timestamp=index * 0.05, capture_phase="driving",
            x=point["x"], y=point["y"], yaw=point["yaw"], collision=False, lane_invasion=False,
            control_transport=dict(expected_receipt_sequence=index + 1, status="PASS"))
        states.append(state)
        commands.append(dict(command_frame=frame, ack_receipt_sequence=index + 1,
            control_source="learned_trajectory_tracker", application=dict(learned_control_applied=True)))
        assert observer.observe(state, learned_effect=True, outgoing_safe=True) is False
    assert observer.completed is False and observer.summary()["completed"] is False
    assert observer.summary()["no_early_success_break"] is True
    observer.records.extend({} for _ in range(872))
    assert observer.summary()["completed"] is True and observer.summary()["full_fixed_interval_observed"] is True
    summary, records = owner.recomputed_straight_interval(original, states, commands)
    assert summary["completed"] and summary["full_fixed_interval_observed"]
    assert summary["completion_is_denominator_only_not_performance"] and len(records) == len(states)


def test_new_capture_AST_has_straight_provenance_and_no_right_result_key():
    old = live.parent._observer
    live.parent._observer = live.StraightIntervalObserver(route())
    try:
        capture = live.cloned_collect_episode()
        function = inspect.getclosurevars(capture).nonlocals["function"]
        constants = repr(function.__code__.co_consts)
        assert "known_dev_straight_fixed45_interval" in constants
        assert "known_dev_right_turn_event" not in constants
    finally:
        live.parent._observer = old


@pytest.mark.parametrize("failure", [False, True])
def test_actual_val_policy_camera_ACK_hooks_and_all_aliases_restore(failure):
    parent = live.parent
    recovery = parent.signed1.previous
    fast45 = recovery.previous
    args = arguments()
    basic = SimpleNamespace(BasicAgent=type("FakeBasicAgent", (), {}))
    fake = SimpleNamespace(model_id=recovery.MODEL_ID, modules=lambda: [], state_dict=lambda: {},
        _research_speed_preview_calibration=live.PREVIEW_SCHEMA,
        _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(purpose="owned_carla_research_only", simulation_only=True,
            owner_plan_sha256="a" * 64))
    before = (live.collector.collect_episode, live.collector.exact_camera_bundle,
        live.collector.AcknowledgedControlTransport, parent.generic.TemporalLiveResearchSessionV2,
        parent.generic.TemporalSimulationResearchPolicyV2)
    with mock.patch.object(parent.previous.previous.previous, "validate_model_binding"), \
         mock.patch.object(recovery, "RouteRecoveryTemporalModelV3", SimpleNamespace), \
         mock.patch.object(fast45, "CPUFiniteCachedTemporalModelV2", SimpleNamespace), \
         mock.patch.object(parent.previous.previous.torch, "get_num_threads", return_value=4), \
         mock.patch.object(parent.previous.previous.torch, "get_num_interop_threads", return_value=1), \
         mock.patch.dict(sys.modules, {"agents": SimpleNamespace(),
             "agents.navigation": SimpleNamespace(basic_agent=basic), "agents.navigation.basic_agent": basic}):
        def enter():
            with live.activated(fake, args):
                assert parent.generic.TemporalLiveResearchSessionV2.__name__ == "KnownDevStraightObservedSession"
                assert parent.generic.TemporalSimulationResearchPolicyV2 is recovery.RecoverySingleCopyPolicyV3
                assert any(cls.__name__ == "ResearchTransport" for cls in live.collector.AcknowledgedControlTransport.__mro__)
                assert live.collector.exact_camera_bundle is not before[1]
                assert live.collector.collect_episode is not before[0]
                if failure:
                    raise RuntimeError("scope failure")
        if failure:
            with pytest.raises(RuntimeError, match="scope failure"):
                enter()
        else:
            enter()
    assert before == (live.collector.collect_episode, live.collector.exact_camera_bundle,
        live.collector.AcknowledgedControlTransport, parent.generic.TemporalLiveResearchSessionV2,
        parent.generic.TemporalSimulationResearchPolicyV2)


@pytest.mark.parametrize("learned", [False, True])
def test_diagnostic_renames_inherited_generic_conjunction_without_turn_claim(learned):
    inherited = dict(independent_event=dict(schema=live.SCHEMA + ".fixed45_interval", completed=True),
        joint_turn_motion_body_policy_wall100ms_passed=True, learned_driving_screen_passed=learned,
        bounded_capture_complete=True, full_RTE03_approved=False, model_promoted=False)
    with mock.patch.object(owner, "_right_diagnostic", return_value=inherited):
        result = owner.diagnostic_report(Path("/unused"), {}, {})
    assert "independent_event" not in result and "joint_turn_motion_body_policy_wall100ms_passed" not in result
    assert result["recomputed_straight_interval"]["completed"] is True
    assert "full_RTE03_approved" not in result and not result["full_RTE01_approved"]
    assert result["straight_motion_body_policy_wall100ms_passed"]
    assert result["learned_driving_screen_passed"] is learned
    assert result["status"] == ("PASS_KNOWN_DEV_STRAIGHT_PRECURSOR" if learned else "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR")
    assert not result["independent_validation"] and not result["model_promoted"]


def test_owner_scope_restores_every_frozen_module_binding_on_error():
    selected = {
        owner.previous: ("paths", "fixed_contract", "live", "ROUTE", "SCHEMA"),
        owner.fixture_owner: ("paths", "fixed_contract", "live", "SCHEMA"),
        owner.runtime_owner: ("paths", "fixed_contract", "live", "SCHEMA"),
    }
    before = {module: {name: getattr(module, name) for name in names} for module, names in selected.items()}
    with mock.patch.object(owner.numeric, "verify_exports"), \
         mock.patch.object(owner.runtime_owner.previous.previous.threads, "evidence_assets", return_value={}):
        with pytest.raises(RuntimeError, match="original"):
            with owner.activated(pins()):
                raise RuntimeError("original")
    assert all(getattr(module, name) is value
        for module, values in before.items() for name, value in values.items())
    assert owner._active is False and live._active is False


def test_owner_scope_keeps_straight_complete_interval_after_right_v3_activation():
    with owner_scope():
        assert owner.previous.complete_interval is owner.complete_interval


def test_runtime_source_closure_includes_both_new_files_with_current_hashes():
    with owner_scope(good6000_pins()):
        sources = owner.runtime_sources()
    for path in (Path(live.__file__).resolve(), Path(owner.__file__).resolve()):
        relative = str(path.relative_to(owner.ROOT))
        assert sources[relative] == owner.digest(path)


def test_paths_are_fresh_create_only_oct4_and_unknown_versions_fail():
    output, declaration = owner.paths("v1")
    assert "2026-10-04" in str(output) and output.name.endswith("known_dev_straight_v1")
    assert declaration.name.endswith("known_dev_straight_v1_plan.json")
    assert output != owner.runtime_owner.paths("v3")
    for value in ("v0", "v2", "../v1", True):
        with pytest.raises(Exception):
            owner.paths(value)
