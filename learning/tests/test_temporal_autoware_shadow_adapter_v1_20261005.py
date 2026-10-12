"""HH_261005 - Exercise the prepared shadow interface with scalar fixtures, never ROS or NN.

Original observed-frame validation, calibration parsing and trajectory conversion
are compiled from their frozen scalar source. Prefill/forward decisions below
are explicit synthetic providers: these tests DO NOT prove runtime parity,
pixel validity, real TF admission, latency or a driving capability.
"""

import ast
from collections import deque
from dataclasses import asdict, dataclass, replace
import hashlib
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "learning/portable_e2e/temporal_autoware_shadow_adapter_v1_20261005.py"
POLICY = ROOT / "learning/portable_e2e/temporal_simulation_research_v2_20261001.py"
RUNTIME = ROOT / "learning/portable_e2e/runtime.py"
CONTRACT = ROOT / "learning/portable_e2e/runtime_contract.py"


def load_core():
    spec = importlib.util.spec_from_file_location("synthetic_temporal_shadow_core_v1", SOURCE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def original_node(path, name, parent=None):
    tree = ast.parse(path.read_text())
    body = tree.body if parent is None else next(n.body for n in tree.body if isinstance(n, ast.ClassDef) and n.name == parent)
    return next(n for n in body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name)


def compile_scalar(node, scope):
    # HH_261005 - Postponed annotations avoid importing the original Torch/NumPy modules.
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    tree = ast.fix_missing_locations(ast.Module(body=[future, node], type_ignores=[]))
    exec(compile(tree, "frozen_scalar_fixture", "exec"), scope)
    return scope[node.name]


def need(condition, message):
    if not condition:
        raise ValueError(message)


def genuine_scalar_types():
    scope = dict(dataclass=dataclass, math=math, ContractError=ValueError, _require=need,
        CAMERA_ORDER=tuple(range(6)))
    for name in ("RuntimeCameraFrame", "RuntimeInputs"):
        compile_scalar(original_node(RUNTIME, name), scope)
    compile_scalar(original_node(POLICY, "ObservedTemporalFrameV2"), scope)
    for name in ("SelectedTrajectory", "_finite_number", "validate_calibration_features", "transform_base_trajectory_to_map"):
        compile_scalar(original_node(CONTRACT, name), scope)
    scope["validate_frame"] = compile_scalar(original_node(POLICY, "_validate_frame", "TemporalSimulationResearchPolicyV2"), scope)
    return SimpleNamespace(**scope)


@pytest.fixture
def prepared(monkeypatch):
    module = load_core()
    scalar = genuine_scalar_types()
    calibration = tuple(tuple(float(index + camera) for index in range(16)) for camera in range(6))
    identity = ("offline_observed_research_episode", "val", "pinned_rig", "a" * 64)

    class FixtureModel:
        _research_temporal_checkpoint_sha256 = module.GOOD_CHECKPOINT_SHA256
        _research_temporal_export_npz_sha256 = module.GOOD_NPZ_SHA256
        _research_temporal_metadata_sha256 = module.GOOD_METADATA_SHA256
        _research_temporal_export_metadata = {"checkpoint_sha256": module.GOOD_CHECKPOINT_SHA256}
        _research_recovery_numeric_loader_schema = "synthetic_loader_scalar"
        _research_recovery_owner_binding = dict(purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False)
        _research_recovery_simulator_control_authorized = False

        def state_dict(self):
            return {str(index): 0 for index in range(101)}

    class FixturePolicy:
        """HH_261005 - Synthetic forwarding only; not a replacement production runtime."""
        _validate_frame = scalar.validate_frame

        def __init__(self, model, controller, **kwargs):
            self.arguments = kwargs
            self.identity = tuple(kwargs[key] for key in ("episode_id", "split", "rig_id", "source_sha256"))
            self._prefill_count = 0
            self.calls = []
            self.last_stamp = None
            self.fail_step = None
            self.total_seconds = .02

        def reset(self, reason):
            self.calls.append(("reset", reason))
            self._prefill_count = 0
            self.last_stamp = None

        def guard(self, frame, kwargs):
            self._validate_frame(frame)
            stamps = [frame.timestamp_ns, frame.pose_timestamp_ns, kwargs["ego_stamp_ns"], kwargs["route_stamp_ns"]]
            need(all(type(value) is int for value in [kwargs["now_ns"], *stamps]), "integer stamps")
            need(all(0 <= kwargs["now_ns"] - value <= 150_000_000 for value in stamps)
                 and max(stamps) - min(stamps) <= 50_000_000, "stale future skew")
            if self.last_stamp is not None:
                need(abs(frame.timestamp_ns - self.last_stamp - 100_000_000) <= 1000, "cadence")
            past = kwargs["past_frames"]
            need(type(past) is tuple and len(past) <= 9, "bounded history")
            previous = -1
            for old in past:
                self._validate_frame(old)
                need(previous < old.timestamp_ns < frame.timestamp_ns, "causal ordered history")
                need(old.inputs.calibration == frame.inputs.calibration, "historical calibration")
                previous = old.timestamp_ns
            # HH_261005 - These deliberately scalar fault markers test delegation, not image or tensor validity.
            need(len(frame.inputs.ego_history) == 10 and len(frame.inputs.ego_history_mask) == 10, "ego history ABI")
            need(all(type(value) is bool for value in frame.inputs.ego_history_mask), "ego mask ABI")
            need(len(frame.inputs.route_xy_base_m) >= 2 and all(len(point) == 2 and all(math.isfinite(x) for x in point)
                 for point in frame.inputs.route_xy_base_m), "route ABI")
            self.last_stamp = frame.timestamp_ns

        def prefill(self, frame, **kwargs):
            self.guard(frame, kwargs)
            self.calls.append(("prefill", frame, kwargs))
            self._prefill_count += 1
            return dict(schema="synthetic.prefill", total_seconds=.2, prefill_count=self._prefill_count,
                cold_timing_retained=True, control_proposed=False, actuator_acknowledged=False)

        def step(self, frame, **kwargs):
            self.guard(frame, kwargs)
            self.calls.append(("step", frame, kwargs))
            xy = tuple((float(index + 1) / 10, 0.) for index in range(64))
            selected = asdict(scalar.SelectedTrajectory(0, xy, (1.,) * 64, .1, None, 6.4, (0.,) * 64))
            return dict(schema="synthetic.decision", total_seconds=self.total_seconds,
                learned_control_proposed=self.fail_step is None, selected=selected,
                candidate_gates=[dict(index=0, passed=True, reason=None)], emergency_reason=self.fail_step,
                raw_output={name: [0.] for name in ("curvature_knots", "preferred_speed_knots", "intent_logits", "stop_distance_raw", "acceleration_prediction")},
                decoded_output=dict(candidate_xy=[xy], candidate_speed=[[1.] * 64], intent=0),
                controller=dict(speed_cap_mps=2.5), control=dict(throttle=.5, brake=0., steer=.1),
                longitudinal_pi_research=dict(proposed_normal_tracker_control={"throttle": .7}))

    loader = SimpleNamespace(SCHEMA="synthetic_loader_scalar", verify_scope=lambda *args: None,
                             verify_owner=lambda *args: None)
    bindings = SimpleNamespace(loader=loader, Model=FixtureModel, Frame=scalar.ObservedTemporalFrameV2,
        Policy=FixturePolicy, Selected=scalar.SelectedTrajectory, validate_calibration=scalar.validate_calibration_features)
    monkeypatch.setattr(module, "_bindings", lambda: bindings)
    model = FixtureModel()
    constructor = dict(episode_id=identity[0], split=identity[1], rig_id=identity[2], source_sha256=identity[3], expected_calibration=calibration)
    core = module.TemporalAutowareShadowPreparationV1(model, SimpleNamespace(speed_cap_mps=2.5), **constructor)

    def frame(index=0):
        stamp = 1_000_000_000 + index * 100_000_000
        cameras = tuple(scalar.RuntimeCameraFrame(stamp, b"synthetic-no-pixels", "jpeg") for _ in range(6))
        inputs = scalar.RuntimeInputs(cameras, calibration, ((0.,) * 13,) * 10, (True,) * 10, ((0., 0.), (20., 0.)))
        return scalar.ObservedTemporalFrameV2(stamp, inputs, (0., 0., 0.), (0., 0., 0., 1.), stamp, *identity)

    def accept(value):
        return core.accept(value, now_ns=value.timestamp_ns, ego_stamp_ns=value.timestamp_ns, route_stamp_ns=value.timestamp_ns)

    return SimpleNamespace(m=module, scalar=scalar, core=core, model=model, constructor=constructor, frame=frame, accept=accept)


def test_import_and_original_dependency_pins_are_source_only():
    module = load_core()
    assert module.verify_sources() == module.SOURCE_PINS
    tree = ast.parse(SOURCE.read_text())
    top = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert not any(any(word in ast.unparse(node) for word in ("torch", "numpy", "rclpy", "carla")) for node in top)
    text = SOURCE.read_text()
    assert "UNIMPLEMENTED / UNVERIFIED" in text and "OFFLINE" in text


def test_direct_original_calls_and_no_model_or_actuator_bypass():
    tree = ast.parse(SOURCE.read_text())
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert {"self._policy._validate_frame", "self._policy.prefill", "self._policy.step", "self._policy.reset"} <= calls
    assert not any(word in name for name in calls for word in ("forward_cached", "apply_control", "publish", "VehicleControl"))
    build = original_node(POLICY, "build_inputs", "TemporalSimulationResearchPolicyV2")
    original_calls = {ast.unparse(node.func) for node in ast.walk(build) if isinstance(node, ast.Call)}
    assert {"build_runtime_tensors", "self._camera_digest", "_relative_pose"} <= original_calls


def test_real_prefill_before_trajectory_and_original_converter_interface(prepared):
    p = prepared
    first, second, third = (p.accept(p.frame(index)) for index in range(3))
    assert (first.status, second.status, third.status) == ("PREFILL", "PREFILL", "TRAJECTORY")
    assert first.selected is second.selected is None and first.diagnostics["total_seconds"] == .2
    assert first.diagnostics["cold_timing_retained"] is True
    assert p.core._policy.arguments["deadline_s"] == .1 and p.core._policy.arguments["longitudinal_pi"] is False
    mapped = p.scalar.transform_base_trajectory_to_map(third.selected, ego_x_m=10., ego_y_m=20., ego_yaw_rad=0.)
    assert len(mapped) == 64 and mapped[0] == (10.1, 20., 0., 1.)
    assert third.selected.logit_margin is None
    assert third.actuator_command_emitted is third.vehicle_control_approved is third.model_promoted is third.learned_STOP_claim is third.P3_approved is False
    assert third.ROS_bindings_implemented is third.runtime_numerical_parity_verified is False
    assert {"control", "controller", "longitudinal_pi_research", "learned_control_proposed"}.isdisjoint(third.diagnostics)


def test_full_history_bounded_and_every_past_input_is_genuine(prepared):
    p = prepared
    for index in range(25):
        assert p.accept(p.frame(index)).status == ("PREFILL" if index < 2 else "TRAJECTORY")
    steps = [call for call in p.core._policy.calls if call[0] == "step"]
    assert len(p.core._history) == 10 and len(steps[-1][2]["past_frames"]) == 9
    for _, current, kwargs in steps:
        stamps = [frame.timestamp_ns for frame in kwargs["past_frames"]]
        assert stamps == sorted(set(stamps)) and all(value < current.timestamp_ns for value in stamps)


@pytest.mark.parametrize("change", [
    {"position_map_m": (math.nan, 0., 0.)}, {"orientation_xyzw": (0., 0., 0., 0.)},
    {"pose_timestamp_ns": 1_000_000_001}, {"pose_timestamp_ns": 999_998_999},
    {"rig_id": "another_TF_rig"}, {"source_sha256": "b" * 64}, {"episode_id": "other"}, {"split": "test"},
    {"timestamp_ns": True},
])
def test_genuine_observed_frame_guards_fail_closed(prepared, change):
    p = prepared
    result = p.core.accept(replace(p.frame(), **change), now_ns=1_000_000_000, ego_stamp_ns=1_000_000_000, route_stamp_ns=1_000_000_000)
    assert result.status == "REJECTED" and result.selected is None and p.core.rejection_count == 1
    assert len(p.core._history) == p.core._policy._prefill_count == 0


@pytest.mark.parametrize("kwargs", [dict(now_ns=1_150_000_001), dict(now_ns=999_999_999),
    dict(ego_stamp_ns=949_999_999), dict(route_stamp_ns=1_000_000_001), dict(now_ns=True)])
def test_stale_future_skew_integer_delegate(prepared, kwargs):
    options = dict(now_ns=1_000_000_000, ego_stamp_ns=1_000_000_000, route_stamp_ns=1_000_000_000)
    options.update(kwargs)
    result = prepared.core.accept(prepared.frame(), **options)
    assert result.status == "REJECTED" and result.selected is None


@pytest.mark.parametrize("key,value", [
    ("calibration", ((0.,) * 16,) * 6), ("calibration", ((math.nan,) * 16,) * 6),
    ("ego_history", ((0.,) * 13,) * 9), ("ego_history_mask", (1,) * 10),
    ("route_xy_base_m", ((0., 0.),)), ("route_xy_base_m", ((0., 0.), (math.inf, 0.))),
])
def test_calibration_history_route_fault_delegation(prepared, key, value):
    frame = prepared.frame()
    result = prepared.accept(replace(frame, inputs=replace(frame.inputs, **{key: value})))
    assert result.status == "REJECTED" and result.selected is None


def test_wrong_camera_count_stamp_arbitrary_dict_and_cadence(prepared):
    p = prepared
    frame = p.frame()
    for cameras in (frame.inputs.camera_frames[:5], (replace(frame.inputs.camera_frames[0], timestamp_ns=1),) + frame.inputs.camera_frames[1:]):
        assert p.accept(replace(frame, inputs=replace(frame.inputs, camera_frames=cameras))).status == "REJECTED"
    assert p.core.accept(asdict(frame), now_ns=1, ego_stamp_ns=1, route_stamp_ns=1).status == "REJECTED"
    assert p.accept(p.frame()).status == "PREFILL"
    assert p.accept(p.frame()).status == "REJECTED"
    assert p.accept(p.frame(1)).status == "PREFILL"
    assert p.accept(p.frame(3)).status == "REJECTED"


@pytest.mark.parametrize("fault", ["ordinary_decoder_refused_short_support", "deadline"])
def test_rejection_keeps_failure_and_requires_new_real_prefill(prepared, fault):
    p = prepared
    for index in range(2):
        p.accept(p.frame(index))
    if fault == "deadline":
        p.core._policy.total_seconds = .100000001
    else:
        p.core._policy.fail_step = fault
    result = p.accept(p.frame(2))
    assert result.status == "REJECTED" and result.selected is None
    assert "decoded_output" in result.diagnostics and result.diagnostics["original_policy_gate_passed"] is False
    assert p.core.attempt_count == 3 and p.core.rejection_count == 1 and p.core.trajectory_count == 0
    assert p.accept(p.frame(3)).status == "PREFILL" and p.accept(p.frame(4)).status == "PREFILL"


def test_malformed_delegated_selection_does_not_count_success(prepared, monkeypatch):
    # HH_261005 - A failed converter-result attachment is retained as rejection, not a successful trajectory.
    p = prepared
    p.accept(p.frame(0))
    p.accept(p.frame(1))
    genuine_fixture_step = p.core._policy.step

    def malformed(frame, **kwargs):
        original = genuine_fixture_step(frame, **kwargs)
        original["selected"]["unexpected_scalar_fixture_key"] = 1
        return original

    monkeypatch.setattr(p.core._policy, "step", malformed)
    result = p.accept(p.frame(2))
    assert result.status == "REJECTED" and result.selected is None
    assert p.core.attempt_count == 3 and p.core.rejection_count == 1 and p.core.trajectory_count == 0
    assert len(p.core._history) == p.core._policy._prefill_count == 0


@pytest.mark.parametrize("attribute,value", [
    ("_research_temporal_export_npz_sha256", "0" * 64), ("_research_temporal_metadata_sha256", "0" * 64),
    ("_research_temporal_checkpoint_sha256", "0" * 64), ("_research_recovery_numeric_loader_schema", "old_vad"),
    ("_research_recovery_simulator_control_authorized", True),
    ("_research_recovery_owner_binding", dict(purpose="owned_carla_research_only", simulator_control_authorized=True)),
])
def test_wrong_model_provenance_and_live_authority_denied(prepared, attribute, value):
    setattr(prepared.model, attribute, value)
    with pytest.raises(prepared.m.ShadowInterfaceError):
        prepared.m.TemporalAutowareShadowPreparationV1(prepared.model, SimpleNamespace(), **prepared.constructor)


def test_shadow_topic_allowlist_never_allows_remap_or_control(prepared):
    module = prepared.m
    for topic in module.SHADOW_TOPICS:
        assert module.require_shadow_topic(topic, topic) == topic
        with pytest.raises(module.ShadowInterfaceError):
            module.require_shadow_topic(topic, "/planning/scenario_planning/trajectory")
    for topic in ("/control/command/control_cmd", "/vehicle/command", "shadow_trajectory", ""):
        with pytest.raises(module.ShadowInterfaceError):
            module.require_shadow_topic(topic, topic)
    with pytest.raises(module.ShadowInterfaceError):
        module.require_shadow_topic("/planning/portable_e2e/shadow_path", "/planning/portable_e2e/shadow_trajectory")


def test_serial_reset_and_input_refusal_preserve_inflight_policy(prepared):
    p = prepared
    p.accept(p.frame())
    p.core._lock.acquire()
    try:
        with pytest.raises(p.m.ShadowInterfaceError):
            p.accept(p.frame(1))
        with pytest.raises(p.m.ShadowInterfaceError):
            p.core.reset()
        assert p.core._policy._prefill_count == 1 and len(p.core._history) == 1
    finally:
        p.core._lock.release()
    p.core.reset()
    assert len(p.core._history) == p.core._policy._prefill_count == 0


def test_changed_or_symlinked_source_rejected(tmp_path):
    module = load_core()
    for name in module.SOURCE_PINS:
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / name).read_bytes())
    assert module.verify_sources(tmp_path) == module.SOURCE_PINS
    target = tmp_path / next(iter(module.SOURCE_PINS))
    target.write_bytes(target.read_bytes() + b"\n# synthetic mutation\n")
    with pytest.raises(module.ShadowInterfaceError):
        module.verify_sources(tmp_path)
    target.unlink()
    target.symlink_to(ROOT / next(iter(module.SOURCE_PINS)))
    with pytest.raises(module.ShadowInterfaceError):
        module.verify_sources(tmp_path)
