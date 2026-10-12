"""HH_261004 - Check fresh full-route seams and honest dynamic denominators without simulator actuation."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_straight_full209_v1_20261004 as m
from test_temporal_route_joint_town03_straight_v3_20261004 import fake_model_and_patches, entered, owner_scope, arguments


def test_dependencies_are_frozen():
    m.verify_frozen()


def test_original_duration_inverse_and_only110_allowed():
    original = next(n for n in ast.parse(Path(m.straight.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "validate_capture")
    transformed = __import__("copy").deepcopy(original)
    undo = m._duration_change(transformed)
    assert "args.max_duration_sec == 110" in ast.unparse(transformed)
    undo(transformed)
    assert ast.dump(transformed) == ast.dump(original)


def test_full_capture_AST_preserves_original_native_and_abort_math():
    original = next(n for n in ast.parse(Path(m.collector.__file__).read_bytes()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "collect_episode")
    # HH_261004 - Reproduce the predecessor's two observation-only additions before its callback transform.
    class Observation(ast.NodeTransformer):
        def visit_Assign(self, node):
            node = self.generic_visit(node)
            if ast.unparse(node) == "(goal_status, basic_agent_done, stop_reason) = tick_and_record('driving')":
                return [node, ast.parse("if stop_reason is None:\n    observed_after_tick(state_records)").body[0]]
            return node
    changed = m.instrument_fullroute_AST(Observation().visit(original))
    text = ast.unparse(changed)
    assert "full_goal_control_reason(alignment, phase, stop_reason, goal_status, event_recorder)" in text
    assert "if stop_reason is None:\n            observed_after_tick" not in text
    assert "agent.run_step() if phase == 'driving' and stop_reason is None else _blank_brake_control(carla)" in text
    assert "if alignment is not None and alignment['status'] != 'PASS':" in text
    assert "driving_end_brake" in text and "exception_cleanup_abort" in text


@pytest.mark.parametrize("native,camera,goal", [(900,450,False),(1755,878,True),(2199,1100,True),(2200,1100,False)])
def test_dynamic_callback_audits_compile_with_exact_unchanged_thresholds(native, camera, goal):
    receipts = m._dynamic_audit(m.callback.audit_receipts, native_count=native, camera_count=camera, goal_complete=goal)
    markers = m._dynamic_audit(m.callback.audit_native_markers, native_count=native, camera_count=camera, goal_complete=goal)
    assert receipts.__globals__["native_count"] == native
    assert markers.__globals__["camera_count"] == camera
    assert markers.__globals__["goal_complete"] is goal
    assert receipts.__globals__["SCHEMA"] == m.SCHEMA


def test_full_ledger_last_native_and_changed_math_disclosure(tmp_path):
    model = object()
    ledger = m.FullLedger(model,tmp_path,clock=lambda: 10)
    ledger.native_mark(2199,"driving",100,"world_tick_returned")
    assert ledger.native_marks[-1]["tick_index"] == 2199
    with pytest.raises(m.collector.CollectionError):
        ledger.native_mark(2200,"driving",100,"world_tick_returned")
    ledger.mark(SimpleNamespace(model=model),SimpleNamespace(timestamp_ns=500),"input_ready")
    assert "original_policy_math_changed" not in ledger.plans[-1]
    assert ledger.plans[-1]["original_policy_step_statements_changed"] is False
    assert ledger.plans[-1]["navigation_terminal_decoder_intervention_declared"] is True


def test_full_ledger_declares_prospective24000_scalar_limit_only_after_cleanup(tmp_path):
    ledger = m.FullLedger(object(),tmp_path)
    assert ledger.finish() is None
    ledger.capture_started = True
    value = ledger.finish()
    assert value["schema"] == m.SCHEMA + ".callback_receipts"
    assert value["scalar_receipt_limit"] == 24000
    assert value["physics_or_control_wait_added"] is False
    assert value["receipt_close_after_original_capture_cleanup"] is True


def test_goal_context_only_original_genuine_arrival():
    recorder = SimpleNamespace(snapshot=lambda: ({},{}))
    assert m.full_goal_control_reason({"status":"FAIL"},"driving","catalog_goal_tolerance",SimpleNamespace(reached=True),recorder) == "alignment_failure_abort"
    assert m._goal_context is None
    assert m.full_goal_control_reason(None,"driving",None,SimpleNamespace(reached=False),recorder) == "next_driving_control"
    assert m._goal_context is None
    try:
        assert m.full_goal_control_reason(None,"driving","catalog_goal_tolerance",SimpleNamespace(reached=True),recorder) == m.goal_guard.GOAL_REASON
        assert m._goal_context[1] == ({},{})
        with pytest.raises(m.collector.CollectionError):
            m.full_goal_control_reason(None,"driving",None,SimpleNamespace(reached=False),recorder)
    finally:
        m._goal_context = None


def test_short_interval_never_relabels_as_completed():
    route = dict(town="Town03",scenario="straight",route=[dict(x=0.,y=0.,distance_m=0.,vad_command=0),dict(x=209.,y=0.,distance_m=209.,vad_command=0)])
    observer = m.FullRouteObserver(route)
    observer.session = SimpleNamespace(_goal_cleanup_pending=None)
    summary = observer.summary()
    assert summary["completed"] is False
    assert summary["observation_count"] == 0
    assert summary["maximum_total_seconds"] == 110
    assert summary["learned_STOP_claim"] is False
    assert summary["independent_validation"] is False


@pytest.mark.parametrize("failure", [False,True])
def test_full_activation_compiles_actual_factory_and_restores_every_delegate(tmp_path, failure):
    args = arguments()
    args.max_duration_sec = 110
    args.output = tmp_path / "episode"
    model, patches = fake_model_and_patches()
    before = (m.callback.SCHEMA,m.callback.CallbackLedger,m.callback.callback_capture_factory,
        m.straight.validate_capture,m.straight.run_straight_capture,m.generic.TemporalSimulationResearchPolicyV2,
        m.collector.AcknowledgedControlTransport,m.callback.policy_runtime.TemporalSimulationResearchPolicyV2.step)
    def enter():
        with owner_scope(), entered(patches), m.activated(model,args):
            assert isinstance(m.callback._ledger,m.FullLedger)
            assert m.callback.previous._actual_fixture_owner_schema() == m.OWNER_SCHEMA
            assert m.generic.TemporalSimulationResearchPolicyV2.__name__ == "ActualNavigationTerminalPolicy"
            causal = [kind.__dict__["propose"] for kind in m.generic.TemporalLiveResearchSessionV2.__mro__
                if "propose" in kind.__dict__ and "TemporalSimulationResearchPolicyV2" in kind.__dict__["propose"].__globals__]
            assert len(causal) == 1
            assert causal[0].__globals__["TemporalSimulationResearchPolicyV2"] is m.generic.TemporalSimulationResearchPolicyV2
            assert m.collector.AcknowledgedControlTransport.__name__ == "GoalCleanupTransport"
            factory = m.full_capture_factory()
            function = __import__("inspect").getclosurevars(factory).nonlocals["function"]
            assert function.__globals__["full_goal_control_reason"] is m.full_goal_control_reason
            assert callable(factory)
            assert m.callback._ledger.capture_started is False
            if failure:
                raise RuntimeError("private CPU scope failure")
    if failure:
        with pytest.raises(RuntimeError,match="private CPU scope failure"):
            enter()
    else:
        enter()
    assert before == (m.callback.SCHEMA,m.callback.CallbackLedger,m.callback.callback_capture_factory,
        m.straight.validate_capture,m.straight.run_straight_capture,m.generic.TemporalSimulationResearchPolicyV2,
        m.collector.AcknowledgedControlTransport,m.callback.policy_runtime.TemporalSimulationResearchPolicyV2.step)
    assert not args.output.exists()


def synthetic_native_binding():
    # HH_261004 - Clearly synthetic scalar fixture, not an actual endpoint capture or performance measurement.
    stamp, yaw, row = 1_000_000_000,.25,tuple(float(i) for i in range(13))
    state = dict(frame=27,timestamp=1.,x=2.,y=3.,z=.1,yaw=yaw,
        actor_snapshot_transform_carla=dict(synthetic=True), world_velocity_carla=[2.4,0.,0.],
        world_acceleration_carla=[0.,0.,0.], world_angular_velocity_carla_deg_s=[0.,0.,0.])
    frame = SimpleNamespace(timestamp_ns=stamp,pose_timestamp_ns=stamp,
        position_map_m=(2.,3.,.1),orientation_xyzw=(0.,0.,math.sin(yaw/2),math.cos(yaw/2)),
        inputs=SimpleNamespace(ego_history=(row,)),split="val",episode_id="SYNTHETIC_SCALAR_ONLY",
        rig_id="b"*64,source_sha256="a"*64)
    session = SimpleNamespace(states=[state],bundle_frame=27,observed_frames=[frame],history=[(27,row,stamp)],source_sha256="a"*64)
    return frame,session,state


def test_terminal_source_binding_reads_exact_actual_current_source_not_outgoing_control(monkeypatch):
    frame,session,state = synthetic_native_binding()
    monkeypatch.setattr(m.parent,"_observer",SimpleNamespace(session=session))
    pose,motion = m.actual_source_binding(frame,{},None)
    assert pose["position_map_m"] == list(frame.position_map_m)
    assert motion["actual_base_link_motion_row"] == list(session.history[-1][1])
    assert pose["native_frame"] == motion["native_frame"] == 27
    assert pose["counterfactual_not_observed"] is False
    state["next_control"] = dict(throttle=1.,brake=0.)
    assert m.actual_source_binding(frame,{},None) == (pose,motion)


@pytest.mark.parametrize("fault",["frame","stamp","pose","motion","split","source"])
def test_terminal_source_binding_rejects_mismatched_current_inputs(monkeypatch,fault):
    frame,session,state = synthetic_native_binding()
    monkeypatch.setattr(m.parent,"_observer",SimpleNamespace(session=session))
    if fault == "frame": session.bundle_frame += 1
    elif fault == "stamp": state["timestamp"] += .1
    elif fault == "pose": frame.position_map_m = (0.,0.,0.)
    elif fault == "motion": frame.inputs.ego_history = ((0.,)*13,)
    elif fault == "split": frame.split = "train"
    else: frame.source_sha256 = "c"*64
    with pytest.raises(m.collector.CollectionError,match="actual current owned native source"):
        m.actual_source_binding(frame,{},None)


def synthetic_terminal_replay_fixture(monkeypatch):
    # HH_261004 - Synthetic endpoint geometry/scalars test only; no actual NN, camera capture or driving claim.
    from dataclasses import asdict
    import hashlib
    import json
    import torch
    from portable_e2e.temporal_owned_camera_history_v2_20261001 import normalized_tensor_hashes
    from test_temporal_navigation_terminal_policy_v2_20261004 import bridge_fixture, fixture_helpers
    bridge,frame,policy,raw_args,_ = bridge_fixture()
    config = arguments()
    specs = m.collector.load_camera_specs(config.mapping,config.calibration,config.wheelbase_m)
    manifest = dict(cameras=[asdict(spec) for spec in specs])
    rig = hashlib.sha256(json.dumps(m.generic.original.calibration_from_specs(specs),allow_nan=False).encode()).hexdigest()
    frame.episode_id,frame.split,frame.rig_id,frame.source_sha256 = "temporal_v2_SYNTHETIC_CPU_ONLY","val",rig,"a"*64
    x,y,z = frame.position_map_m
    q = frame.orientation_xyzw
    yaw = 2*math.atan2(q[2],q[3])
    states = [dict(frame=i,timestamp=(i+2)*.05,capture_phase="driving",x=x,y=y,z=z,yaw=yaw,
        vx=2.4,vy=0.,ax=0.,ay=0.,yaw_rate=0.,steering_tire_angle_rad=0.,command=0,
        world_velocity_carla=[2.4,0.,0.],world_acceleration_carla=[0.,0.,0.],
        world_angular_velocity_carla_deg_s=[0.,0.,0.],actor_snapshot_transform_carla=dict(synthetic_CPU_only=True)) for i in range(19)]
    frame.position_map_m = tuple(float(states[-1][name]) for name in ("x","y","z"))
    frame.orientation_xyzw = (0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    history = [(row["frame"],m.generic.original.state_features(row),int(round(row["timestamp"]*1.e9))) for row in states[::2]]
    frame.inputs.ego_history = tuple(row[1] for row in history)
    ego = torch.tensor(frame.inputs.ego_history,dtype=torch.float32)[None]
    session = SimpleNamespace(states=states,bundle_frame=18,observed_frames=[frame],history=history,source_sha256="a"*64)
    monkeypatch.setattr(m.parent,"_observer",SimpleNamespace(session=session))
    bridge.source_binding = m.actual_source_binding
    owned = policy._last_owned_input_audit
    # HH_261004 - Synthetic replay retains the exact original all-ten-valid mask as well as motion tensor SHA.
    owned["normalized_input_tensor_sha256"].update(normalized_tensor_hashes(
        dict(ego_history=ego,ego_history_mask=torch.ones((1,10),dtype=torch.bool))))
    window = bridge.current[2]
    owned["source_camera_frames"] = [None]*4+[dict(timestamp_ns=frame.timestamp_ns,camera_sha256=list(window.frames[-1].camera_sha256))]
    bridge.bind_current(frame,dict(ego_history=ego),window,now_ns=frame.timestamp_ns,
        ego_stamp_ns=frame.timestamp_ns,route_stamp_ns=frame.timestamp_ns)
    decoded = bridge(*raw_args)
    command = dict(learned_control_proposed=True,held_between_camera_frames=False,inference_frame=18,command_frame=18,
        measured_current_speed_mps=float(raw_args[2][0]),causal_navigation_support_m=10.,
        owned_camera_history=owned,navigation_terminal_research=bridge.last_receipt,
        raw_output=dict(curvature_knots=raw_args[0][0].tolist(),preferred_speed_knots=raw_args[1][0].tolist()),
        decoded_output=dict(candidate_xy=decoded["xy"][0].tolist(),candidate_speed=decoded["speed"][0].tolist(),
            instantaneous_speed=decoded["instantaneous_speed"][0].tolist(),arc_distance=decoded["arc_distance"][0].tolist()))
    payload = fixture_helpers.read_canonical_regular(m.ROOT / fixture_helpers.ROUTE_REL)
    return payload,states,[command],manifest


def test_independent_actual_terminal_source_witness_and_decoder_replay_with_synthetic_inputs(monkeypatch):
    payload,states,commands,manifest = synthetic_terminal_replay_fixture(monkeypatch)
    result = m.audit_terminal_navigation(payload,states,commands,manifest=manifest,
        stream_source_sha256="a"*64,episode_id="temporal_v2_SYNTHETIC_CPU_ONLY")
    assert result["passed"] is True
    assert result["actual_terminal_inference_count"] == 1
    assert result["model_forward_executed"] is result["camera_payload_read"] is False
    assert result["learned_STOP_claim"] is False


@pytest.mark.parametrize("fault",["pose","stamp","motion","rig","ego_SHA","mission","provided_arc","NN","decoder"])
def test_independent_terminal_replay_denies_actual_source_or_numeric_mutation(monkeypatch,fault):
    payload,states,commands,manifest = synthetic_terminal_replay_fixture(monkeypatch)
    receipt = commands[0]["navigation_terminal_research"]
    if fault == "pose": states[-1]["x"] += .1
    elif fault == "stamp": states[-1]["timestamp"] += .1
    elif fault == "motion": states[-1]["world_velocity_carla"][0] += .1
    # HH_261004 - Mutate the actual intrinsic consumed by rig hashing, not redundant CARLA FOV metadata.
    elif fault == "rig": manifest["cameras"][0]["intrinsic"]["fx"] += 1.
    elif fault == "ego_SHA": commands[0]["owned_camera_history"]["normalized_input_tensor_sha256"]["ego_history"] = "f"*64
    elif fault == "mission": receipt["mission_document"]["source_available_timestamp_ns"] += 1
    elif fault == "provided_arc": receipt["navigation_owned_D_m"] += .01
    elif fault == "NN": commands[0]["raw_output"]["curvature_knots"][0] += .01
    else: commands[0]["decoded_output"]["candidate_xy"][0][0][0] += .01
    with pytest.raises(m.collector.CollectionError):
        m.audit_terminal_navigation(payload,states,commands,manifest=manifest,
            stream_source_sha256="a"*64,episode_id="temporal_v2_SYNTHETIC_CPU_ONLY")


def synthetic_quota_receipts():
    row = dict(sequence=1,native_frame=2,timestamp_simulation_ns=100_000_000,capture_phase="driving",
        status="PASS",complete_walk=True,source_payload_read=False,cached_or_incremental_byte_count_used=False,
        entries=3,regular_files=2,regular_file_bytes=30,returned_regular_file_bytes=30,
        walk_started_monotonic_ns=10,walk_finished_monotonic_ns=20)
    value = dict(schema=m.SCHEMA+".quota_walk_receipts",clock="time.perf_counter_ns_monotonic",rows=[row],
        actual_walk_count=1,maximum_walk_count=1100,maximum_count_exceeded=False,
        every_attempt_including_failures_retained=True,complete_walk_every_call=True,incremental_cache_used=False,
        original_quota_free_floor_and_reserves_unchanged=True,actual_sensor_latency_gain_proven_by_walk_timing=False)
    cameras = [dict(frame=2,timestamp=.1,capture_phase="driving")]
    timings = [dict(command_frame=2,observed_bundle=dict(native_frame=2,exact_bundle_delivery_monotonic_ns=30))]
    return value,cameras,timings


def test_quota_walk_scalar_postcheck_keeps_unresolved_disk_dispatch_and_no_causal_claim():
    value,cameras,timings = synthetic_quota_receipts()
    result = m.audit_quota_walks(value,cameras,timings)
    assert result["passed"] is True and result["actual_walk_count"] == 1
    assert result["disk_check_dispatch_and_scheduling_interval_separately_unresolved"] is True
    assert result["actual_sensor_latency_gain_proven_by_walk_timing"] is False


@pytest.mark.parametrize("fault",["missing","cache","count","frame","clock","delivery","failure"])
def test_quota_walk_scalar_postcheck_rejects_hidden_or_reordered_failed_rows(fault):
    value,cameras,timings = synthetic_quota_receipts()
    if fault == "missing": value["rows"] = []
    elif fault == "cache": value["rows"][0]["cached_or_incremental_byte_count_used"] = True
    elif fault == "count": value["rows"][0]["returned_regular_file_bytes"] = 31
    elif fault == "frame": value["rows"][0]["native_frame"] = 3
    elif fault == "clock": value["rows"][0]["walk_started_monotonic_ns"] = 21
    elif fault == "delivery": timings[0]["observed_bundle"]["exact_bundle_delivery_monotonic_ns"] = 19
    else: value["rows"][0]["status"] = "FAIL"
    with pytest.raises(m.collector.CollectionError):
        m.audit_quota_walks(value,cameras,timings)
