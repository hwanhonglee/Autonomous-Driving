"""HH_261002 - Source-only tests of one normal terminal-steering intervention; no physical capture, future admission, or TRAIN claim."""

import ast
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
from types import FunctionType, SimpleNamespace

import pytest

from scripts.e2e import town07_natural_goal_stop_brakefree_v7_20261002 as task
from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v7_20261002 as collector
from scripts.e2e import run_owned_town07_natural_goal_stop_brakefree_v7_20261002 as owner
from scripts.e2e import town07_natural_goal_stop_brakefree_v6_20261002 as v6task
from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v6_20261002 as v6collector


# HH_261002 - Reuse frozen synthetic fixture code in an explicit NEW provider namespace, never invoke its old owner or a simulator.
_fixture_path = Path(__file__).with_name("test_natural_goal_stop_teacher_brakefree_v6_20261002.py")
_spec = importlib.util.spec_from_file_location("_v7_frozen_synthetic_fixtures", _fixture_path)
_fixtures = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_fixtures)
_namespace = dict(vars(_fixtures), task=task, collector=collector, owner=owner)
for _name in ("configuration", "governor", "measured", "control", "normal", "mission_fixture", "construct_with_real_original_caps"):
    _function = getattr(_fixtures, _name)
    _namespace[_name] = FunctionType(_function.__code__, _namespace, _name, _function.__defaults__, _function.__closure__)
    _namespace[_name].__kwdefaults__ = _function.__kwdefaults__
configuration, governor, measured, control, normal, mission_fixture = (_namespace[name] for name in
    ("configuration", "governor", "measured", "control", "normal", "mission_fixture"))


def rebased_fixture(name):
    # HH_261002 - Identity-only adaptation of a frozen constructor/scope fixture; retain all API assertions and source bodies.
    original = next(node for node in ast.parse(_fixture_path.read_bytes()).body if isinstance(node, ast.FunctionDef) and node.name == name)
    node = deepcopy(original)
    replacements = {"v6": "v7", "low_speed_hold_v6": "low_speed_hold_v7", "brakefree_v6_20261002": "brakefree_v7_20261002"}
    changes = []
    for item in ast.walk(node):
        if isinstance(item, ast.Constant) and type(item.value) is str and item.value in replacements:
            changes.append((item, item.value)); item.value = replacements[item.value]
    revised = deepcopy(node)
    for item, value in changes: item.value = value
    assert ast.dump(node, include_attributes=False) == ast.dump(original, include_attributes=False)
    namespace = dict(_namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised], type_ignores=[])), str(__file__) + ":synthetic_fixture", "exec"), namespace)
    return namespace[name]


def test_public_contract_keeps_clock_pedals_source_route_and_one_attempt(capsys, monkeypatch):
    assert owner.main(["--mode", "contract"]) == 0
    plan = json.loads(capsys.readouterr().out)
    prior = v6task.fixed_contract("v6", "pilot_00")
    for key in ("fixed_arguments", "seed", "attempts", "automatic_retry", "route_sha256", "route_length_m",
                "stationary_tail_native_ticks", "required_future_support_ns", "minimum_selected_span_ns", "episode_quota_bytes",
                "closing_reserve_bytes", "free_disk_floor_bytes", "formal_feature_denominators", "native_signed_speed_rate_bounds_mps2",
                "native_absolute_speed_rate_bound_mps2", "selected_first_driving_relative_native_frame"):
        assert plan[key] == prior[key]
    assert plan["schema"] == task.SCHEMA and owner.SCHEMA.endswith(".v7.owned_pilot")
    assert task.SEED == task.FIXED_ARGS["seed"] == 2026102400 and task.FIXED_ARGS == v6task.FIXED_ARGS
    assert task.CRUISE == 1.5 and task.COAST_DISTANCE == 11.5 and task.COAST_ENTRY_BAND == (1.48, 1.58)
    assert task.COAST_TIMEOUT == 45. and configuration().stopped_speed_mps == .05
    assert plan["straight_terminal_precondition"]["whole_route_collinearity_claim"] is False
    assert plan["straight_terminal_precondition"]["original_route_never_rewritten"] is True
    assert plan["controller"]["normal_COAST_HOLD_steer"] == 0.
    assert task.mission_contract()["original_lateral_planner_and_downstream_BasicAgent_hazard_unchanged"] is False
    assert task.mission_contract()["original_GO_lateral_and_all_downstream_BasicAgent_hazards_unchanged"] is True
    assert task.mission_contract()["training_data_approved"] is False and "training_data_approved" not in task.pending_scope()
    monkeypatch.setattr(owner, "runtime_sources", lambda: {"new/source": "a" * 64})
    monkeypatch.setattr(owner.lifecycle, "external_sources", lambda: {"external/source": "b" * 64})
    with owner.activated(): inherited = owner.previous.build_plan("v7", "pilot_00")
    assert inherited["seed"] == inherited["fixed_arguments"]["seed"] == task.SEED
    assert inherited["source_sha256"] == {"new/source": "a" * 64}


@pytest.mark.parametrize("run,case", [("v6", "pilot_00"), ("v7", "train_00"), ("../v7", "pilot_00"), ("v8", "pilot_00")])
def test_only_fresh_namespace_without_old_failure_reclassification(run, case):
    with pytest.raises(Exception): task.paths(run, case)


def test_governor_physical_values_exactly_equal_v6_before_and_after_latches():
    new = governor()
    old = v6task.CoastToGoalGovernor(v6task.goal_configuration(SimpleNamespace(**v6task.FIXED_ARGS)), 1.5, 20.)
    for frame, remaining, speed in ((1, 20., 0.), (2, 20., .5), (3, 20., 1.5), (4, 11.5, 1.5),
                                    (5, 11.6, 1.51), (6, .75, .04), (7, .75, .02)):
        kwargs = dict(count_hold=True, source_frame=frame, source_timestamp=frame * .05, source_state=dict(vx=speed, vy=0.))
        previous = old.update(remaining, remaining, speed, -remaining, **kwargs)
        actual = new.update(remaining, remaining, speed, -remaining, **kwargs)
        assert actual["profile_id"] == task.PROFILE and previous["profile_id"] == v6task.PROFILE
        assert {key: value for key, value in actual.items() if key != "profile_id"} == {key: value for key, value in previous.items() if key != "profile_id"}
    assert actual["terminal_mission_phase"] == "HOLD" and actual["complete"] is False


@pytest.mark.parametrize("steer", [-.7, -.005152, 0., .005152, .7])
def test_only_normal_terminal_steering_changes_all_other_control_fields_exact(steer):
    oldg = v6task.CoastToGoalGovernor(v6task.goal_configuration(SimpleNamespace(**v6task.FIXED_ARGS)), 1.5, 20.)
    newg = governor(); oldpolicy = v6collector.BrakeFreeLaunchCoastHoldControl(); newpolicy = collector.BrakeFreeLaunchCoastHoldControl()
    for frame, remaining, speed in ((1, 20., .5), (2, 20., 1.5), (3, 11.5, 1.5), (4, .75, .0391658858)):
        kwargs = dict(count_hold=True, source_frame=frame, source_timestamp=frame * .05, source_state=dict(vx=speed, vy=0.))
        previous = oldg.update(remaining, remaining, speed, -remaining, **kwargs)
        current = newg.update(remaining, remaining, speed, -remaining, **kwargs)
        first, second = control(steer=steer), control(steer=steer)
        values = dict(speed=speed, frame=frame, timestamp=frame * .05, target_speed_mps=current["target_speed_mps"])
        oldpolicy.apply(first, goal_record=previous, normal_proposal=collector.base.control_dict(first), **values)
        newpolicy.apply(second, goal_record=current, normal_proposal=collector.base.control_dict(second), **values)
        before, after = collector.base.control_dict(first), collector.base.control_dict(second)
        assert {key: value for key, value in before.items() if key != "steer"} == {key: value for key, value in after.items() if key != "steer"}
        terminal = current["terminal_mission_phase"] in ("BRAKE_TO_STOP", "HOLD")
        assert after["steer"] == (0. if terminal else before["steer"])
        assert newpolicy.last["normal_terminal_steering_zero_applied"] is terminal
        assert newpolicy.last["raw_normal_planner_proposal"]["steer"] == collector.native_float32(steer)
        assert newpolicy.last["downstream_BasicAgent_hazard_override_untouched"] is True
        if terminal: assert after["throttle"] == 0. and after["brake"] == (1. if current["terminal_mission_phase"] == "HOLD" else 0.)


def test_original_stopped_steering_guard_is_idempotent_for_new_normal_hold_request():
    g = governor(); measured(g, 1, remaining=11.5)
    record = measured(g, 2, remaining=.75, speed=.0391658858)
    result = normal(collector.BrakeFreeLaunchCoastHoldControl(), record, steer=.005152)
    exact = collector.base.control_dict(result)
    assert result.steer == 0. and result.brake == 1.
    assert collector.base.suppress_stopped_brake_steering(result, .0391658858, True) is True
    assert collector.base.control_dict(result) == exact


def straight_route():
    return dict(route=[dict(x=float(i), y=0., distance_m=float(i), vad_command=0) for i in range(213)])


def test_geometry_scope_is_terminal_not_whole_route_and_original_route_unchanged():
    route = task.read(task.ROUTE); before = deepcopy(route)
    proof = task.straight_terminal_geometry(route)
    assert route == before and len(route["route"]) == 213
    assert proof["maximum_terminal_perpendicular_error_m"] < .001
    assert proof["maximum_terminal_segment_heading_error_rad"] < .001
    assert proof["whole_route_collinearity_claim"] is False
    synthetic = straight_route(); synthetic["route"][20]["y"] = 3.
    assert task.straight_terminal_geometry(synthetic)["maximum_terminal_perpendicular_error_m"] == 0.
    assert task.sha(task.ROUTE) == task.ROUTE_SHA


@pytest.mark.parametrize("mutation", ["bend", "heading", "reverse", "short", "count", "bool", "nan", "inf"])
def test_terminal_precondition_rejects_bends_invalid_source_or_missing_real_span(mutation):
    route = straight_route()
    if mutation == "bend": route["route"][-7]["y"] = .01
    elif mutation == "heading":
        route["route"][-7].update(x=205.5, y=.0009)
    elif mutation == "reverse": route["route"][-7]["x"] -= 2.
    elif mutation == "short":
        for row in route["route"]: row["x"] /= 100.
    elif mutation == "count": route["route"].pop()
    else: route["route"][3]["x"] = {"bool": True, "nan": float("nan"), "inf": float("inf")}[mutation]
    with pytest.raises(Exception): task.straight_terminal_geometry(route)


def test_exact_adjacent_duplicate_source_points_are_not_rewritten_or_future_filtered():
    route = straight_route(); route["route"][-4] = deepcopy(route["route"][-5]); saved = deepcopy(route)
    assert task.straight_terminal_geometry(route)["maximum_terminal_perpendicular_error_m"] == 0.
    assert route == saved and len(route["route"]) == 213


def test_source_pins_and_real_namespaces_include_all_old_closure_plus_new_three():
    task.verify_frozen()
    pins = owner.runtime_sources()
    assert len(pins) == 104 and len(owner.frozen_v6_owner.runtime_sources()) == 101
    assert all(pins[name] == expected for name, expected in task.V6_SOURCES.items())
    assert set(task.FROZEN).issubset(pins)
    assert owner.worker_module is collector and collector.task is task and owner.task is task
    assert owner.MODULE == "scripts.e2e.run_owned_town07_natural_goal_stop_brakefree_v7_20261002"
    assert collector.revised_scope.__globals__ is vars(collector)
    assert task.activated.__wrapped__.__globals__ is vars(task) and owner.activated.__wrapped__.__globals__ is vars(owner)
    assert task.CoastToGoalGovernor.update.__globals__ is vars(task)
    assert collector.BrakeFreeLaunchCoastHoldControl.apply.__globals__ is vars(collector)
    assert set(task.V7_literal_replacement_counts.values()) == {1}
    assert collector.V7_import_replacement_counts == {"town07_natural_goal_stop_brakefree_v6_20261002": 1}
    assert owner.V7_import_replacement_counts == {"town07_natural_goal_stop_brakefree_v6_20261002": 1,
        "collect_town07_natural_goal_stop_brakefree_v6_20261002": 1}


def test_frozen_source_mismatch_and_unrelated_ast_mutation_cannot_clone(monkeypatch):
    with pytest.raises(Exception): task.clone_frozen_provider(v6task, "0" * 64, dict(__file__=__file__), literals={"v6": "v7"})
    def change(tree):
        assignment = next(node for node in tree.body if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "CRUISE" for t in node.targets))
        assignment.value = ast.Constant(value=2.4)
        return lambda: None
    with pytest.raises(Exception): task.clone_frozen_provider(v6task, task.V6_SOURCES["tools/e2e/town07_natural_goal_stop_brakefree_v6_20261002.py"],
        dict(__file__=__file__), literals={"v6": "v7"}, intervention=change)
    monkeypatch.setattr(task, "sha", lambda path: "0" * 64)
    with pytest.raises(Exception): task.straight_terminal_precondition()


def test_scoped_providers_restore_real_namespace_flags_and_all_original_values():
    for module, parent, names in ((task, task.previous, ("SCHEMA", "PROFILE", "SEED", "QUOTA", "FIXED_ARGS", "paths", "pending_scope", "goal_configuration", "fixed_contract", "MovingTask")),
        (owner, owner.previous, ("task", "worker_module", "MODULE", "SCHEMA", "CASE_IDS", "ASSETS", "capture_argv", "execution_contract", "runtime_sources", "disk_check", "diagnostic_report"))):
        saved = {name: getattr(parent, name) for name in names}
        with pytest.raises(RuntimeError):
            with module.activated():
                assert module._active is True
                with pytest.raises(Exception):
                    with module.activated(): pass
                raise RuntimeError("synthetic scope failure")
        assert module._active is False and all(getattr(parent, name) is value for name, value in saved.items())


def test_real_parse_and_private_ownership_constructor_order_without_simulator(monkeypatch):
    monkeypatch.setattr(task, "configuration_from_args", lambda args: dict(run=args.natural_goal_run_id))
    args = collector.parse_args(owner.capture_argv("v7", "pilot_00"))
    assert args.natural_goal_run_id == "v7" and args.seed == task.SEED and args.max_duration_sec == 200.
    assert args.agent_initialization == "after_bootstrap" and args.stationary_tail_sec == 6.5
    argv = owner.execution_contract("v7", "pilot_00")["worker_command"]
    assert owner.MODULE in argv and argv[argv.index("--run-id") + 1] == "v7"
    node = next(n for n in ast.parse(Path(owner.previous.__file__).read_bytes()).body if isinstance(n, ast.FunctionDef) and n.name == "worker")
    segment = ast.get_source_segment(Path(owner.previous.__file__).read_text(), node)
    assert segment.index("assert_owned_worker") < segment.index("with worker_module.scope") < segment.index("base.run")


def test_real_constructor_two_caps_phase_hook_and_original_hazard_not_replaced(monkeypatch, tmp_path):
    rebased_fixture("test_real_guarded_collector_scope_and_hazard_identity_restore")(monkeypatch, tmp_path)


def test_actual_original_constructor_caps_then_GO_steering_terminal_zero_and_untouched_hazard(monkeypatch, tmp_path):
    # HH_261002 - Execute original construct_agent's complete two-cap order; the interfaces and ACK here are strictly synthetic.
    context, built, planner, g, native, state, manifest, original = _namespace["construct_with_real_original_caps"](monkeypatch, tmp_path)
    try:
        assert planner._vehicle_controller.max_brake == 0. and planner._vehicle_controller.max_throt == .20
        assert planner._hh_brakefree_phase_cap_binding["after_original_outer_cap"] is True
        assert manifest["capture_contract"]["teacher_controller"]["phase_hook_after_original_outer_cap"] is True
        assert manifest["capture_contract"]["goal_stop_profile"]["effective_control"]["phase_hook_rebound_after_original_outer_cap"] is True
        for frame, (remaining, speed, phase, brake) in enumerate(((20., 1.5, "GO", 0.), (11.5, 1.5, "BRAKE_TO_STOP", 0.),
                                                               (.75, .0391658858, "HOLD", 1.)), 100):
            native.frame = frame; native.timestamp.elapsed_seconds = frame * .05
            state.update(frame=frame, timestamp=frame * .05, vx=speed)
            collector.base.acknowledged_snapshot_state(native)
            record = g.update(remaining, remaining, speed, -remaining, count_hold=True)
            built.set_target_speed(record["target_speed_mps"] * 3.6)
            result = built.run_step(); observed = collector.base.control_dict(result)
            assert record["terminal_mission_phase"] == phase and result.brake == brake
            assert result.steer == (collector.native_float32(.1) if phase == "GO" else 0.)
            assert result.manual_gear_shift is True and result.gear == 1
            if phase != "GO": assert result.throttle == 0.
            assert set(observed) == {"throttle", "steer", "brake", "hand_brake", "reverse", "gear", "manual_gear_shift"}
            assert observed == dict(requested_control=deepcopy(observed))["requested_control"]
        hazard_input = control(steer=.37, brake=0., throttle=.2)
        hazard_result = built.add_emergency_stop(hazard_input)
        assert hazard_result is hazard_input and hazard_result.steer == collector.native_float32(.37)
        assert hazard_result.brake == 1. and hazard_result.throttle == 0.
    finally: context.__exit__(None, None, None)
    assert collector.base.install_normal_brake_cap is original and collector._active is False


def test_full_native_mission_replay_retains_all_184_states_185_updates_and_52_normal_controls():
    inputs = mission_fixture(); report = collector.audit_mission(*inputs)
    assert report["recorded_native_count"] == 184 and report["all_native_updates"] == 185
    assert report["normal_policy_count"] == 52 and report["low_speed_service_hold_normal_command_count"] == 40
    assert report["future64_QA_or_TRAIN_authorization"] is False
    for command in inputs[3]:
        row = command["normal_policy"]
        if row is not None and row["terminal_mission_phase"] != "GO":
            assert row["profile_normal_control"]["steer"] == command["final_BasicAgent_or_brake_proposal"]["steer"] == 0.


@pytest.mark.parametrize("mutation", ["normal_steer", "final_steer", "ACK_steer", "missing_flag", "missing_update", "hazard"])
def test_all_seven_fields_and_full_failure_denominator_remain_strict(mutation):
    inputs = list(mission_fixture()); command = inputs[3][14]
    if mutation == "normal_steer": command["normal_policy"]["profile_normal_control"]["steer"] = .005
    elif mutation == "final_steer": command["final_BasicAgent_or_brake_proposal"]["steer"] = .005
    elif mutation == "ACK_steer": inputs[2][14]["requested_control"]["steer"] = .005
    elif mutation == "missing_flag": command["normal_policy"].pop("normal_terminal_steering_zero_applied")
    elif mutation == "missing_update": inputs[4].pop(20)
    else: inputs[-1] = [dict(original_return_unchanged=True)]
    with pytest.raises(Exception): collector.audit_mission(*inputs)


@pytest.mark.parametrize("native_bounds_pass,mission_pass", [(True, True), (False, True), (True, False)])
def test_original_physical_or_ack_failure_cannot_pass_new_owner(monkeypatch, tmp_path, native_bounds_pass, mission_pass):
    rebased_fixture("test_compiled_owner_diagnostic_preserves_all_checks_and_requires_both_new_proofs")(
        monkeypatch, tmp_path, native_bounds_pass, mission_pass)


def test_old_v6_policy_and_frozen_hashes_remain_unchanged_and_still_express_old_hold_steering():
    old = v6task.CoastToGoalGovernor(v6task.goal_configuration(SimpleNamespace(**v6task.FIXED_ARGS)), 1.5, 20.)
    policy = v6collector.BrakeFreeLaunchCoastHoldControl()
    first = old.update(20., 20., 1.5, -20., count_hold=True, source_frame=1, source_timestamp=.05, source_state=dict(vx=1.5, vy=0.))
    initial = control(steer=.005152)
    policy.apply(initial, goal_record=first, speed=1.5, frame=1, timestamp=.05,
        target_speed_mps=first["target_speed_mps"], normal_proposal=collector.base.control_dict(initial))
    old.update(11.5, 11.5, 1.5, -11.5, count_hold=True, source_frame=2, source_timestamp=.1, source_state=dict(vx=1.5, vy=0.))
    record = old.update(.75, .75, .039, -.75, count_hold=True, source_frame=3, source_timestamp=.15, source_state=dict(vx=.039, vy=0.))
    c = control(steer=.005152)
    policy.apply(c, goal_record=record, speed=.039, frame=3, timestamp=.15,
        target_speed_mps=0., normal_proposal=collector.base.control_dict(c))
    assert c.steer == collector.native_float32(.005152) and c.brake == 1.
    assert all(task.sha(task.ROOT / name) == expected for name, expected in task.V6_SOURCES.items())
