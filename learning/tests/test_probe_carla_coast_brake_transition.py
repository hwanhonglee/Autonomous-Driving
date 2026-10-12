"""HH_260906 - Test coast-to-brake timing with existing in-memory fixtures; never start or contact CARLA."""

import json
import signal
from types import SimpleNamespace as NS

import pytest

from scripts.e2e import probe_carla_coast_brake_transition as probe
from test_probe_carla_low_speed_coast import coast_world, load_result  # noqa: F401
from test_probe_carla_stationary_motion import fake_world, ROUTE  # noqa: F401


def args_for(tmp_path, *extra):
    return [str(tmp_path / "brake"), str(ROUTE), "--arm", "coast", *extra]


def test_fixed_cli_and_complete_source_closure(tmp_path):
    args = probe.parse_args(args_for(tmp_path))
    sources, inputs = probe.fingerprints(args)
    assert args.arm == "coast" and args.host == "127.0.0.1" and args.port == 2100 and args.allow_map_load is False
    assert len(sources) == 9 and set(sources) == {"scripts/e2e/" + name for name in probe.SOURCE_NAMES}
    assert inputs["route_file"] == probe.ROUTE_SHA and not args.output.exists()
    assert (probe.SETTLE_TICKS, probe.MAX_LAUNCH_TICKS, probe.COAST_TICKS, probe.TAIL_TICKS, probe.MAXIMUM_TICKS) == (70,160,600,300,1130)


@pytest.mark.parametrize("extra", [["--arm", "parking"], ["--host", "remote"], ["--port", "80"],
    ["--allow-map-load"], ["--tail-ticks", "2"], ["--coast-ticks", "900"], ["--target-speed-kmh", "1"],
    ["--launch-throttle", ".14"], ["--hand-brake"], ["--ar", "service"]])
def test_cli_rejects_unreviewed_arm_controls_and_timing(tmp_path, extra):
    with pytest.raises(SystemExit): probe.parse_args(args_for(tmp_path, *extra))


def test_arm_is_required_and_outputs_are_create_only(tmp_path):
    with pytest.raises(SystemExit): probe.parse_args([str(tmp_path / "brake"), str(ROUTE)])
    (tmp_path / "brake.partial").mkdir()
    with pytest.raises(SystemExit): probe.parse_args(args_for(tmp_path))


@pytest.mark.parametrize("arm", ["coast", "service"])
def test_controls_differ_only_in_tail_service_brake(arm):
    api = NS(VehicleControl=lambda **kw: NS(**kw))
    for phase in ("settle", "launch", "coast", "tail"):
        value = vars(probe.phase_control(api, phase, arm))
        assert value == dict(throttle=.13 if phase == "launch" else 0., steer=0.,
            brake=1. if phase == "settle" or (phase == "tail" and arm == "service") else 0.,
            hand_brake=False, reverse=False, manual_gear_shift=False, gear=0)
    with pytest.raises(probe.capture.CollectionError): probe.phase_control(api, "other", arm)
    with pytest.raises(probe.capture.CollectionError): probe.phase_control(api, "tail", "other")


def test_fixed_phase_boundaries_have_no_extra_tick_or_early_tail_completion():
    row = lambda phase,count,speed: dict(phase=phase,phase_tick=count,planar_speed_mps=speed)
    assert probe.following_phase(row("settle",69,0.)) == "settle"
    assert probe.following_phase(row("settle",70,0.)) == "launch"
    assert probe.following_phase(row("launch",1,.49999)) == "launch"
    assert probe.following_phase(row("launch",1,.5)) == "coast"
    assert probe.following_phase(row("launch",160,.5)) == "coast"
    with pytest.raises(probe.capture.CollectionError, match="launch_timeout"):
        probe.following_phase(row("launch",160,.49999))
    assert probe.following_phase(row("coast",599,0.)) == "coast"
    assert probe.following_phase(row("coast",600,.1)) == "tail"
    assert probe.following_phase(row("tail",299,0.)) == "tail"
    assert probe.following_phase(row("tail",300,0.)) == "COMPLETE"


@pytest.mark.parametrize("speed", [.10000001, .2, float("nan"), float("inf"), True, "0.1"])
def test_common_coast_endpoint_check_is_strict(speed):
    with pytest.raises(probe.capture.CollectionError, match="coast_endpoint_speed"):
        probe.following_phase(dict(phase="coast",phase_tick=600,planar_speed_mps=speed))


@pytest.fixture
def brake_world(coast_world):
    """HH_260906 - Reuse the frozen map/camera/ACK fixture and add only the new endpoint/control failure injections."""
    config, args, world = coast_world
    args = probe.parse_args([str(args.output), str(ROUTE), "--arm", "coast"])
    config["brake_failure"] = None
    spawn = world.try_spawn_actor
    def spawn_actor(blueprint, transform):
        actor = spawn(blueprint, transform)
        old_velocity, old_control = actor.get_velocity, actor.get_control
        def velocity():
            at_boundary = config["tick"] == 70 + config["launch_ticks"] + 600
            if at_boundary and config["brake_failure"] in ("endpoint_fast", "endpoint_exact"):
                return NS(x=.101 if config["brake_failure"] == "endpoint_fast" else .1, y=0., z=0.)
            return old_velocity()
        def control():
            value = old_control()
            if config["brake_failure"] == "tail_control_lag" and config["tick"] == 70 + config["launch_ticks"] + 601:
                value.brake = 0.
            return value
        actor.get_velocity, actor.get_control = velocity, control
        return actor
    world.try_spawn_actor = spawn_actor
    return config, args, world


@pytest.mark.parametrize("arm", ["coast", "service"])
@pytest.mark.parametrize("mode,launch_count", [(None,5),("max_launch",160)])
def test_full_run_records_three_handoffs_and_all300_tail_ticks(brake_world, arm, mode, launch_count):
    config, args, world = brake_world
    args.arm, config["coast_failure"] = arm, mode
    handlers = {number:signal.getsignal(number) for number in (signal.SIGINT,signal.SIGTERM)}
    probe.run(args)
    manifest, states, receipts, cameras = load_result(args)
    total, last_coast_tick = 70+launch_count+900, 70+launch_count+600
    assert manifest["status"] == "COMPLETE" and manifest["arm"] == arm
    assert len(states) == len(receipts) == config["tick"] == total
    assert manifest["phase_counts"] == dict(settle=70,launch=launch_count,coast=600,tail=300)
    assert len(cameras) == total//2 and len(config["saved"]) == total//2*6
    assert [row["tick"] for row in cameras] == list(range(2,total+1,2))
    assert [(r["from_phase"],r["to_phase"],r["effective_tick"]) for r in manifest["phase_transitions"]] == [
        ("settle","launch",71),("launch","coast",71+launch_count),("coast","tail",last_coast_tick+1)]
    assert manifest["launch_handoff"] == manifest["phase_transitions"][1]
    assert manifest["tail_handoff"] == manifest["phase_transitions"][2]
    endpoint, first_tail = states[last_coast_tick-1], states[last_coast_tick]
    assert endpoint["phase"] == "coast" and endpoint["phase_tick"] == 600 and endpoint["requested_control"]["brake"] == 0.
    assert first_tail["phase"] == "tail" and first_tail["phase_tick"] == 1 and first_tail["frame"] == endpoint["frame"]+1
    assert manifest["tail_entry_check"]["source_frame"] == endpoint["frame"] and manifest["tail_entry_check"]["status"] == "PASS"
    assert manifest["tail_entry_check"]["applies_to_both_arms"] is True
    tail = states[last_coast_tick:]
    assert len(tail) == 300 and [r["phase_tick"] for r in tail] == list(range(1,301))
    assert all(r["requested_control"]["brake"] == r["reported_control"]["brake"] == (1. if arm=="service" else 0.) for r in tail)
    assert all(r["requested_control"]["throttle"] == r["requested_control"]["steer"] == 0. for r in tail)
    assert all(r["status"] == "ACKNOWLEDGED" and r["before_frame"] == r["after_ack_frame"] == s["frame"]-1 for r,s in zip(receipts,states))
    assert all(s["measurement_complete"] and s["control_alignment"]["status"] == "PASS" for s in states)
    assert manifest["completion_reason"] == "full_600_tick_coast_and_300_tick_tail_observed"
    assert manifest["source_sha256_before"] == manifest["source_sha256_after"]
    assert len(manifest["source_sha256_before"]) == 9 and manifest["input_sha256_before"] == manifest["input_sha256_after"]
    assert manifest["world_settings_before"] == manifest["world_settings_restored"]
    assert manifest["weather_before"] == manifest["weather_restored"] and manifest["weather_requested"] == manifest["weather_actual"]
    assert not config["actors"] and len(config["destroyed"]) == 8 and manifest["cleanup_errors"] == []
    assert all(signal.getsignal(number)==handler for number,handler in handlers.items())


@pytest.mark.parametrize("arm", ["coast", "service"])
def test_common_fast_endpoint_aborts_after_recording_before_any_tail_command(brake_world, arm):
    config, args, _world = brake_world
    args.arm, config["brake_failure"] = arm, "endpoint_fast"
    with pytest.raises(probe.capture.CollectionError, match="coast_endpoint_speed"):
        probe.run(args)
    manifest, states, receipts, cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and len(states) == len(receipts) == config["tick"] == 675
    assert states[-1]["measurement_complete"] and states[-1]["phase_tick"] == 600 and states[-1]["phase"] == "coast"
    assert states[-1]["planar_speed_mps"] == .101 and states[-1]["requested_control"]["brake"] == 0.
    assert manifest["tail_entry_check"]["status"] == "FAIL" and manifest["phase_counts"]["tail"] == 0
    assert len(manifest["phase_transitions"]) == 2 and "tail_handoff" not in manifest
    assert len(cameras) == 337 and not config["actors"] and not args.output.exists()


@pytest.mark.parametrize("arm", ["coast", "service"])
def test_exact_point_one_endpoint_is_allowed_without_threshold_relaxation(brake_world, arm):
    config, args, _world = brake_world
    args.arm, config["brake_failure"] = arm, "endpoint_exact"
    probe.run(args)
    manifest, states, _receipts, _cameras = load_result(args)
    assert manifest["tail_entry_check"]["source_planar_speed_mps"] == .1 and len(states) == 975


def test_first_tail_service_control_mismatch_preserves_offending_state_and_images(brake_world):
    config, args, _world = brake_world
    args.arm, config["brake_failure"] = "service", "tail_control_lag"
    with pytest.raises(probe.capture.CollectionError, match="alignment"):
        probe.run(args)
    manifest, states, receipts, cameras = load_result(args, True)
    assert len(states) == len(receipts) == 676 and len(cameras) == 338
    assert states[-1]["phase"] == "tail" and states[-1]["requested_control"]["brake"] == 1.
    assert states[-1]["reported_control"]["brake"] == 0. and not states[-1]["measurement_complete"]
    assert cameras[-1]["complete"] and manifest["status"] == "FAIL" and not config["actors"]


@pytest.mark.parametrize("mode,ticks", [("launch_timeout",230),("overspeed",71),("lane_missing",0),("ray_hit",0),("ray_nan",0)])
def test_frozen_coast_preflight_and_launch_failures_remain_closed(brake_world, mode, ticks):
    config, args, world = brake_world
    config["coast_failure"] = mode
    with pytest.raises(probe.capture.CollectionError): probe.run(args)
    manifest, states, _receipts, _cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and len(states) == config["tick"] == ticks and not config["actors"]
    assert vars(world.get_settings()) == vars(config["initial"]) and config["weather"] is config["original_weather"]


@pytest.mark.parametrize("failure,ticks", [("speed",70),("travel",70),("nonfinite",10),("camera_nan",9),
    ("collision",10),("signal",8),("encoding",2),("settings",0),("extra_settings",0),("ack",7)])
def test_frozen_settle_protocol_and_cleanup_failures_remain_closed(brake_world, failure, ticks):
    config, args, world = brake_world
    config["failure"] = failure
    with pytest.raises(probe.capture.CollectionError): probe.run(args)
    manifest, states, _receipts, _cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and len(states) == config["tick"] == ticks and not config["actors"]
    assert vars(world.get_settings()) == vars(config["initial"]) and config["weather"] is config["original_weather"]


def test_new_worker_source_tamper_prevents_completion(brake_world, monkeypatch):
    config, args, _world = brake_world
    original, calls = probe.fingerprints, []
    def changed(arguments):
        sources, inputs = original(arguments)
        calls.append(True)
        if len(calls) == 2: sources["scripts/e2e/probe_carla_coast_brake_transition.py"] = "0"*64
        return sources, inputs
    monkeypatch.setattr(probe, "fingerprints", changed)
    with pytest.raises(probe.capture.CollectionError, match="final checks"): probe.run(args)
    manifest, _states, _receipts, _cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and not config["actors"]
