"""HH_261001 - Check the isolated learned handoff without connecting to CARLA."""

from types import SimpleNamespace as NS
import json

import numpy as np
import pytest

from scripts.e2e import collect_ah_simulation_research as trial


def physics():
    return NS(wheels=[NS(position=NS(x=x, y=y, z=0.), max_steer_angle=angle)
                      for x, y, angle in ((140., -70., 35.), (140., 70., 35.),
                                          (-140., -70., 0.), (-140., 70., 0.))])


def state(frame=1, vx=0.):
    return dict(frame=frame, timestamp=frame / 10., command=3, vx=vx, vy=0., ax=0., ay=0.,
                yaw_rate=0., steering_tire_angle_rad=0., x=1., y=0., z=0., yaw=0., route_cte_m=0.)


def specs():
    return [NS(name=name, width=640, height=360, enable_postprocess_effects=False,
               intrinsic=dict(fx=457., fy=457., cx=320., cy=180.),
               ros_extrinsic=dict(x=1., y=0., z=1.6, roll=0., pitch=0., yaw=0.))
            for name in trial.collector.MODEL_CAMERA_ORDER]


def test_measured_geometry_and_exact_feature_commands():
    cfg = trial.actor_controller_config(physics(), 2.5)
    assert cfg.wheelbase_m == pytest.approx(2.8)
    assert cfg.maximum_steer_angle_rad == pytest.approx(np.deg2rad(35.))
    assert trial.state_features(state())[7:] == (0., 0., 0., 1., 0., 0.)
    with pytest.raises(ValueError, match="four-wheel"):
        trial.actor_controller_config(NS(wheels=[]), 2.5)
    rows = trial.calibration_from_specs(specs())
    assert len(rows) == 6 and all(len(row) == 16 for row in rows)
    assert rows[0][4:8] == (0., 0., 1., 1.)


def test_startup_then_one_way_learned_handoff_and_camera_hold(tmp_path, monkeypatch):
    calls = []
    class Policy:
        def __init__(self, model, cfg):
            self.controller = cfg
        def step(self, inputs, **kwargs):
            calls.append((inputs, kwargs))
            return dict(control_source="learned_trajectory_tracker", learned_control_proposed=True,
                emergency_reason=None, control=dict(throttle=0.1, brake=0., steer=-0.1,
                                                     reverse=False, hand_brake=False))
    monkeypatch.setattr(trial, "SimulationResearchPolicy", Policy)
    states = [state()]
    args = NS(research_speed_cap_mps=2.5, research_max_cte_m=1.)
    session = trial.LiveResearchSession(object(), args,
        dict(route=[dict(x=0., y=0.), dict(x=100., y=0.)]), specs(), states, tmp_path)
    transport = NS(ego=NS(get_physics_control=physics),
                   carla=NS(VehicleControl=lambda **kwargs: NS(**kwargs)))
    teacher = NS(throttle=0.2, brake=0., steer=0.)
    chosen, decision = session.propose(transport, teacher, "next_driving_control")
    assert chosen is teacher and decision["control_source"] == "expert_startup"
    for frame in range(1, 11):
        states.append(state(frame, vx=0.6))
        image = NS(timestamp=frame / 10., raw_data=bytes(360 * 640 * 4))
        session.observe_bundle({name: image for name in trial.collector.MODEL_CAMERA_ORDER}, frame)
    states.append(dict(state(11, vx=0.6), timestamp=1.05))
    _, waiting = session.propose(transport, teacher, "next_driving_control")
    assert waiting["control_source"] == "expert_startup" and not session.handed_off
    states.pop()
    chosen, decision = session.propose(transport, teacher, "next_driving_control")
    assert session.handed_off and chosen.steer == -0.1 and decision["learned_control_proposed"]
    assert len(calls) == 1 and calls[0][1]["ego_stamp_ns"] == 1_000_000_000
    states.append(dict(state(11, vx=0.6), timestamp=1.05))
    _, held = session.propose(transport, teacher, "next_driving_control")
    assert held["held_between_camera_frames"] and len(calls) == 1
    states.append(dict(state(12, vx=0.), timestamp=1.2))
    chosen, stale = session.propose(transport, teacher, "next_driving_control")
    assert chosen.brake == 1. and not stale["learned_control_proposed"]
    assert session.handed_off


def test_collision_before_handoff_is_never_expert_startup(tmp_path, monkeypatch):
    class Policy:
        def __init__(self, model, cfg):
            self.controller = cfg
    monkeypatch.setattr(trial, "SimulationResearchPolicy", Policy)
    session = trial.LiveResearchSession(object(), NS(research_speed_cap_mps=2.5, research_max_cte_m=1.),
        dict(route=[dict(x=0., y=0.), dict(x=100., y=0.)]), specs(), [state()], tmp_path)
    session.latched_emergency = "collision event"
    transport = NS(ego=NS(get_physics_control=physics), carla=NS(VehicleControl=lambda **k: NS(**k)))
    chosen, decision = session.propose(transport, NS(brake=0.), "next_driving_control")
    assert chosen.brake == 1. and decision["control_source"] == "emergency_brake"
    assert not decision["learned_control_proposed"]


def test_transport_applies_and_records_learned_command_then_restores_hooks(tmp_path, monkeypatch):
    class Policy:
        def __init__(self, model, cfg):
            self.controller = cfg
        def step(self, inputs, **kwargs):
            return dict(control_source="learned_trajectory_tracker", learned_control_proposed=True,
                control=dict(throttle=0.11, brake=0., steer=-0.12, reverse=False, hand_brake=False))
    monkeypatch.setattr(trial, "SimulationResearchPolicy", Policy)
    frame = [0]
    sent = []
    def vehicle_control(**kwargs):
        return NS(**dict(dict(throttle=0., brake=0., steer=0., reverse=False,
                              hand_brake=False, manual_gear_shift=False, gear=0), **kwargs))
    def apply(commands, do_tick):
        sent.append(commands[0])
        return [NS(has_error=lambda: False, error="", actor_id=1)]
    api = NS(VehicleControl=vehicle_control,
             command=NS(ApplyVehicleControl=lambda actor_id, control: control))
    states = []
    manifest = dict(capture_contract=dict(capture_phase_schedule=dict(driving=dict(control_source="carla.BasicAgent"))))
    def original_collect(*args, **kwargs):
        transport = trial.collector.AcknowledgedControlTransport(
            NS(apply_batch_sync=apply), api, NS(id=1, get_physics_control=physics), lambda: frame[0], lambda r: None)
        for number in range(1, 11):
            frame[0] = number
            states.append(state(number, vx=0.6))
            trial.collector.exact_camera_bundle({}, number)
            transport.send(vehicle_control(throttle=0.3), "next_driving_control", number)
    def original_bundle(queues, number):
        image = NS(timestamp=number / 10., raw_data=bytes(360 * 640 * 4))
        return {name: image for name in trial.collector.MODEL_CAMERA_ORDER}
    args = NS(research_speed_cap_mps=2.5, research_max_cte_m=1.)
    monkeypatch.setattr(trial.collector, "collect_episode", original_collect)
    monkeypatch.setattr(trial.collector, "exact_camera_bundle", original_bundle)
    monkeypatch.setattr(trial.collector, "run", lambda args: trial.collector.collect_episode(args,
        dict(route=[dict(x=0., y=0.), dict(x=100., y=0.)]), specs(), tmp_path, states, [], manifest))
    original_transport = trial.collector.AcknowledgedControlTransport
    trial.run(object(), args)
    assert sent[-1].throttle == 0.11 and sent[-1].steer == -0.12
    assert states[-1]["expert_proposed_next_control"]["throttle"] == 0.3
    assert states[-1]["next_control"]["throttle"] == 0.11
    journal = [json.loads(row) for row in (tmp_path / "learned_control.jsonl").read_text().splitlines()]
    assert journal[-1]["application"]["learned_control_applied"]
    assert not journal[0]["application"]["learned_control_applied"]
    assert manifest["result"]["learned_research_counts"]["learned_commands_acknowledged"] == 1
    assert trial.collector.AcknowledgedControlTransport is original_transport
    assert trial.collector.collect_episode is original_collect
