"""HH_261001 - Verify measured temporal collector history, startup handoff, ACK ownership and restoration."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from portable_e2e.simulation_research import ControllerConfig
from scripts.e2e import collect_temporal_v2_simulation_research_20261001 as collect


CALIBRATION = ((.7, 1.2, .5, .5, 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.),) * 6


class FakeControl:
    def __init__(self, throttle=0., brake=0., steer=0., reverse=False, hand_brake=False):
        self.throttle, self.brake, self.steer = throttle, brake, steer
        self.reverse, self.hand_brake = reverse, hand_brake


class FakePolicy:
    def __init__(self, model, config, **kwargs):
        self.model, self.controller, self.identity = model, config, kwargs
        self._prefill_count, self.prefill_records, self.calls = 0, [], []
        self.reject = False

    def prefill(self, frame, **kwargs):
        self.calls.append(("prefill", frame, kwargs))
        self._prefill_count += 1
        result = dict(prefill_count=self._prefill_count, control_proposed=False, actuator_acknowledged=False)
        self.prefill_records.append(result)
        return dict(result)

    def step(self, frame, **kwargs):
        self.calls.append(("step", frame, kwargs))
        return dict(schema="new_temporal_v2", source_checkpoint_sha256="1" * 64,
            control_source="emergency_brake" if self.reject else "learned_trajectory_tracker",
            learned_control_proposed=not self.reject, emergency_reason="rejected" if self.reject else None,
            control=dict(throttle=0. if self.reject else .1, brake=1. if self.reject else 0.,
                         steer=.02, reverse=False, hand_brake=False))

    def reset(self, reason):
        self._prefill_count = 0
        self.calls.append(("reset", reason))


def args(tmp_path):
    return SimpleNamespace(output=tmp_path / "attempt/episode", research_source_sha256="a" * 64,
        research_longitudinal_pi=False, research_max_cte_m=1., research_speed_cap_mps=2.5,
        host="127.0.0.1", port=2440, allow_map_load=False, control_transport="acknowledged_batch",
        goal_stop_profile="disabled", physics_hz=20., capture_hz=10., stationary_warmup_sec=1.2,
        max_duration_sec=20., target_speed_kmh=9., weather="ClearNoon", seed=0, spawn_z_offset_m=.5)


def session_fixture(monkeypatch, tmp_path):
    monkeypatch.setattr(collect.original, "calibration_from_specs", lambda _specs: CALIBRATION)
    monkeypatch.setattr(collect.original, "actor_controller_config", lambda *_args: ControllerConfig(2.7, .6))
    monkeypatch.setattr(collect, "TemporalSimulationResearchPolicyV2", FakePolicy)
    route = dict(town="Town07", scenario="straight", route=[dict(x=0., y=0.), dict(x=200., y=0.)])
    states = []
    model = SimpleNamespace(_research_temporal_checkpoint_sha256="1" * 64)
    session = collect.TemporalLiveResearchSessionV2(model, args(tmp_path), route, (), states, tmp_path)
    transport = SimpleNamespace(carla=SimpleNamespace(VehicleControl=FakeControl),
                                ego=SimpleNamespace(get_physics_control=lambda: None))
    return session, transport


def observe(session, index, *, vx=1., cte=0.):
    native_frame = 100 + index * 2
    timestamp = 1. + index * .1
    state = dict(frame=native_frame, timestamp=timestamp, x=index * .1, y=0., z=0., yaw=0.,
                 command=3, vx=vx, vy=0., ax=0., ay=0., yaw_rate=0., steering_tire_angle_rad=0.,
                 route_cte_m=cte, world_velocity_carla=(vx, 0., 0.))
    session.states.append(state)
    pixels = np.zeros((360, 640, 4), dtype=np.uint8)
    pixels[..., 0] = index
    bundle = {name: SimpleNamespace(timestamp=timestamp, frame=native_frame, raw_data=pixels.tobytes())
              for name in collect.collector.MODEL_CAMERA_ORDER}
    session.observe_bundle(bundle, native_frame)
    return state, bundle


def startup(session, transport):
    for index in range(10): observe(session, index)
    teacher = FakeControl(throttle=.2)
    chosen, first = session.propose(transport, teacher, "next_driving_control")
    assert chosen is teacher and first["control_source"] == "expert_startup"
    observe(session, 10)
    chosen, second = session.propose(transport, teacher, "next_driving_control")
    assert chosen is teacher and second["control_source"] == "expert_startup"
    assert session.policy._prefill_count == 2
    return teacher


def test_current_pose_pixels_and_history_use_only_same_native_observations(monkeypatch, tmp_path):
    session, _ = session_fixture(monkeypatch, tmp_path)
    for index in range(4): observe(session, index)
    value = session.observed_frames[-1]
    assert value.position_map_m == pytest.approx((.3, 0., 0.))
    assert value.orientation_xyzw == (0., 0., 0., 1.)
    assert value.timestamp_ns == value.pose_timestamp_ns == 1_300_000_000
    assert all(camera.timestamp_ns == value.timestamp_ns for camera in value.inputs.camera_frames)
    assert value.inputs.ego_history_mask == (False,) * 6 + (True,) * 4
    assert value.inputs.ego_history[:6] == ((0.,) * 13,) * 6
    assert value.inputs.ego_history[-1][1] == 1.
    assert all(camera.payload[0, 0, 0] == 3 for camera in value.inputs.camera_frames)
    assert "route_cte_m" not in vars(value) and "world_velocity_carla" not in vars(value)


def test_three_fresh_frames_two_real_prefills_then_learned_handoff(monkeypatch, tmp_path):
    session, transport = session_fixture(monkeypatch, tmp_path)
    teacher = startup(session, transport)
    assert [call[0] for call in session.policy.calls] == ["prefill", "prefill"]
    first, second = session.policy.calls
    assert second[1].timestamp_ns - first[1].timestamp_ns == 100_000_000
    assert all(frame.timestamp_ns < first[1].timestamp_ns for frame in first[2]["past_frames"])
    observe(session, 11)
    chosen, decision = session.propose(transport, teacher, "next_driving_control")
    assert chosen is not teacher and chosen.throttle == .1
    assert session.handed_off is True and session.counts["inference_count"] == 1
    assert decision["control_source"] == "learned_trajectory_tracker" and decision["candidate_count"] == 1
    assert decision["runtime_intent"] == "explicit_cruise_GO_not_head"
    assert session.policy.calls[-1][0] == "step"


def test_native_intermediate_tick_holds_previous_decision_without_second_integrator_update(monkeypatch, tmp_path):
    session, transport = session_fixture(monkeypatch, tmp_path)
    teacher = startup(session, transport)
    observe(session, 11)
    session.propose(transport, teacher, "next_driving_control")
    before = len(session.policy.calls)
    intermediate = dict(session.states[-1], frame=session.states[-1]["frame"] + 1,
                        timestamp=session.states[-1]["timestamp"] + .05)
    session.states.append(intermediate)
    chosen, result = session.propose(transport, teacher, "next_driving_control")
    assert chosen.throttle == .1 and result["held_between_camera_frames"] is True
    assert len(session.policy.calls) == before


@pytest.mark.parametrize("reason", ["initial_stationary_control", "next_stationary_warmup_control",
    "next_stationary_tail_control", "exception_cleanup_brake"])
def test_original_setup_tail_and_cleanup_control_authority_is_not_learned(monkeypatch, tmp_path, reason):
    session, transport = session_fixture(monkeypatch, tmp_path)
    teacher = FakeControl(brake=1.)
    chosen, result = session.propose(transport, teacher, reason)
    assert chosen is teacher and result is None and session.policy is None


@pytest.mark.parametrize("kind", ["state_frame", "camera_frame", "camera_stamp", "gap", "duplicate"])
def test_noncausal_or_misaligned_camera_state_stream_is_rejected(monkeypatch, tmp_path, kind):
    session, _ = session_fixture(monkeypatch, tmp_path)
    state, bundle = observe(session, 0)
    expected_frame = state["frame"]
    if kind == "state_frame": session.states[-1] = dict(state, frame=state["frame"] + 1)
    if kind == "camera_frame": next(iter(bundle.values())).frame += 1
    if kind == "camera_stamp": next(iter(bundle.values())).timestamp += .001
    if kind == "gap":
        state, bundle = observe(session, 1)
        session.states[-1] = dict(state, frame=state["frame"] + 4, timestamp=state["timestamp"] + .2)
        for image in bundle.values(): image.frame += 4; image.timestamp += .2
        expected_frame = session.states[-1]["frame"]
    with pytest.raises(collect.collector.CollectionError):
        session.observe_bundle(bundle, expected_frame)


def test_policy_rejection_is_latched_and_never_replaced_with_teacher(monkeypatch, tmp_path):
    session, transport = session_fixture(monkeypatch, tmp_path)
    teacher = startup(session, transport)
    session.policy.reject = True
    observe(session, 11)
    first, decision = session.propose(transport, teacher, "next_driving_control")
    assert first.brake == 1. and decision["learned_control_proposed"] is False
    assert session.latched_emergency.startswith("temporal learned proposal rejected")
    session.policy.reject = False
    observe(session, 12)
    second, result = session.propose(transport, teacher, "next_driving_control")
    assert second.brake == 1. and result["control_source"] == "emergency_brake"
    assert [x[0] for x in session.policy.calls].count("step") == 1


@pytest.mark.parametrize("kind", ["cte", "teacher_brake", "camera_age"])
def test_original_external_safety_remains_authoritative_after_handoff(monkeypatch, tmp_path, kind):
    session, transport = session_fixture(monkeypatch, tmp_path)
    teacher = startup(session, transport)
    observe(session, 11)
    session.propose(transport, teacher, "next_driving_control")
    observe(session, 12, cte=1.01 if kind == "cte" else 0.)
    if kind == "teacher_brake": teacher.brake = .051
    if kind == "camera_age": session.states[-1]["timestamp"] += .151
    chosen, result = session.propose(transport, teacher, "next_driving_control")
    assert chosen.brake == 1. and chosen.throttle == 0.
    assert result["control_source"] == "emergency_brake" and session.latched_emergency is not None


def test_activation_hooks_restore_even_on_exception(monkeypatch, tmp_path):
    session, _ = session_fixture(monkeypatch, tmp_path)
    original = collect.collector.collect_episode, collect.collector.exact_camera_bundle, collect.collector.AcknowledgedControlTransport
    events = collect.collector.EventRecorder.on_collision, collect.collector.EventRecorder.on_lane_invasion
    with pytest.raises(RuntimeError):
        with collect.activated(session.model, session.args):
            assert collect.collector.collect_episode is not original[0]
            raise RuntimeError("synthetic owner interruption")
    assert (collect.collector.collect_episode, collect.collector.exact_camera_bundle,
            collect.collector.AcknowledgedControlTransport) == original
    assert (collect.collector.EventRecorder.on_collision, collect.collector.EventRecorder.on_lane_invasion) == events


def test_fixed_twenty_second_capture_predeclared_and_no_fullroute_claim(monkeypatch, tmp_path):
    session, _ = session_fixture(monkeypatch, tmp_path)
    route = dict(town="Town07", scenario="straight")
    collect.validate_capture(session.args, route)
    session.args.max_duration_sec = 21
    with pytest.raises(collect.collector.CollectionError, match="contract"):
        collect.validate_capture(session.args, route)


def test_wall_receipt_binds_actual_ACK_to_monotonic_delivery_without_mixing_simulation_exposure(monkeypatch, tmp_path):
    session, _ = session_fixture(monkeypatch, tmp_path)
    observe(session, 0)
    session.bundle_wall_timing[100] = dict(native_frame=100,
        source_image_timestamp_simulation_ns=1_000_000_000,
        exact_bundle_delivery_monotonic_ns=9_000_000_000)
    decision = dict(inference_frame=100, learned_control_proposed=True,
                    held_between_camera_frames=False, control_source="learned_trajectory_tracker")
    row = collect.wall_timing_receipt(session, decision, dict(sequence=8, server_accepted=True),
        command_frame=100, reason="next_driving_control", proposal_started_ns=9_010_000_000,
        proposal_finished_ns=9_090_000_000, ack_started_ns=9_091_000_000, ack_finished_ns=9_094_000_000)
    assert row["exact_bundle_delivery_to_ACK_seconds"] == pytest.approx(.094)
    assert row["exact_bundle_delivery_to_proposal_seconds"] == pytest.approx(.09)
    assert row["propose_seconds"] == pytest.approx(.08)
    assert row["ACK_send_seconds"] == pytest.approx(.003)
    assert row["ACK_sequence"] == 8 and row["ACK_server_accepted"] is True
    assert row["stream_source_sha256"] == session.source_sha256
    assert row["observed_bundle"]["source_image_timestamp_simulation_ns"] == 1_000_000_000
    assert row["sensor_callback_entry_observed"] is False
    assert row["sensor_callback_entry_to_ACK_seconds"] is None
    assert row["exposure_to_ACK_wall_latency_proven"] is False and row["policy_deadline_relaxed"] is False


def test_wall_summary_preserves_all_receipts_but_excludes_held_and_nonlearned_from_fresh_denominator(monkeypatch, tmp_path):
    session, _ = session_fixture(monkeypatch, tmp_path)
    observe(session, 0)
    session.bundle_wall_timing[100] = dict(exact_bundle_delivery_monotonic_ns=9_000_000_000)
    base = dict(inference_frame=100, learned_control_proposed=True,
                held_between_camera_frames=False, control_source="learned_trajectory_tracker")
    rows = []
    for index, decision in enumerate((base, dict(base, held_between_camera_frames=True), None)):
        rows.append(collect.wall_timing_receipt(session, decision, dict(sequence=index, server_accepted=True),
            command_frame=100+index, reason="next_driving_control", proposal_started_ns=9_010_000_000,
            proposal_finished_ns=9_090_000_000, ack_started_ns=9_091_000_000,
            ack_finished_ns=9_094_000_000 + index * 50_000_000))
    summary = collect.wall_timing_summary(rows)
    assert summary["record_count"] == 3 and summary["fresh_learned_proposal_ACK_count"] == 1
    assert summary["exact_bundle_delivery_to_ACK_seconds"] == dict(count=1, p95=.094, maximum=.094)
    assert summary["ten_hz_end_to_end_proven"] is False


def test_missing_camera_delivery_receipt_is_explicitly_unmeasured_not_fabricated(monkeypatch, tmp_path):
    session, _ = session_fixture(monkeypatch, tmp_path)
    row = collect.wall_timing_receipt(session, None, dict(sequence=1, server_accepted=True),
        command_frame=None, reason="bootstrap", proposal_started_ns=10, proposal_finished_ns=20,
        ack_started_ns=21, ack_finished_ns=24)
    assert row["observed_bundle"] is None
    assert row["exact_bundle_delivery_to_ACK_seconds"] is None
    assert row["exact_bundle_delivery_to_proposal_seconds"] is None
    assert collect.wall_timing_summary([row])["exact_bundle_delivery_to_ACK_seconds"]["count"] == 0
