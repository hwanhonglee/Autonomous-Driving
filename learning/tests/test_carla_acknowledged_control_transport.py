"""HH_260906 - Exercise acknowledged commands and immutable observation binding without CARLA or a GPU."""

import ast
import copy
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from scripts.e2e import collect_carla_vad_expert as capture


def command(**changes):
    values = dict(throttle=0.15, brake=0.0, steer=0.0, hand_brake=False, reverse=False,
                  manual_gear_shift=False, gear=0)
    values.update(changes)
    return SimpleNamespace(**values)


class Client:
    def __init__(self):
        self.calls = []
        self.responses = [SimpleNamespace(actor_id=42, error="", has_error=lambda: False)]

    def apply_batch_sync(self, commands, do_tick):
        self.calls.append((commands, do_tick))
        if isinstance(self.responses, Exception):
            raise self.responses
        return self.responses


@pytest.fixture
def transport():
    client, journal, frames = Client(), [], [100]
    carla = SimpleNamespace(command=SimpleNamespace(ApplyVehicleControl=lambda actor_id, control: (actor_id, control)))
    ego = SimpleNamespace(id=42, apply_control=lambda *_: pytest.fail("async fallback must never be called"))
    worker = capture.AcknowledgedControlTransport(client, carla, ego, lambda: frames[0], lambda row: journal.append(copy.deepcopy(row)))
    return worker, client, journal, frames


def test_single_acknowledged_batch_does_not_tick_and_persists_before_caller_can_tick(transport):
    worker, client, journal, frames = transport
    control = command()
    receipt = worker.send(control, "test_next_control")
    assert client.calls == [([(42, control)], False)]
    assert frames == [100]
    assert journal == [receipt]
    assert receipt["status"] == "ACKNOWLEDGED" and receipt["server_accepted"] is True
    assert receipt["before_frame"] == receipt["after_ack_frame"] == 100
    assert receipt["physical_actuation_proven"] is False
    control.throttle = 0.9
    assert receipt["requested_control"]["throttle"] == 0.15
    frames[0] += 1
    second = worker.send(command(brake=1.0, throttle=0.0), "abort")
    assert second["sequence"] == 2 and len(journal) == 2


@pytest.mark.parametrize("bad", [[], [None, None], None,
    [SimpleNamespace(actor_id=99, error="", has_error=lambda: False)],
    [SimpleNamespace(actor_id=True, error="", has_error=lambda: False)],
    [SimpleNamespace(actor_id=42, error="actor rejected", has_error=lambda: True)],
    [SimpleNamespace(actor_id=42, error="", has_error=lambda: 0)],
    [SimpleNamespace(actor_id=42, error="unexpected", has_error=lambda: False)], RuntimeError("timeout")])
def test_rpc_failure_or_malformed_response_is_journaled_and_never_falls_back(transport, bad):
    worker, client, journal, frames = transport
    client.responses = bad
    with pytest.raises(capture.CollectionError, match="no async fallback"):
        worker.send(command(), "failing_command")
    assert len(client.calls) == len(journal) == 1
    assert journal[0]["status"] == "FAILED" and "error" in journal[0]
    assert frames == [100]


@pytest.mark.parametrize("changes", [{"throttle": float("nan")}, {"brake": float("inf")}, {"steer": True},
    {"throttle": -0.1}, {"brake": 1.01}, {"steer": 1.01}, {"hand_brake": 0}, {"gear": True}])
def test_invalid_command_is_rejected_before_rpc(transport, changes):
    worker, client, journal, _ = transport
    with pytest.raises(capture.CollectionError):
        worker.send(command(**changes), "invalid")
    assert client.calls == journal == []


def test_acknowledged_command_must_not_advance_a_frame(transport):
    worker, _, journal, _ = transport
    frames = iter((100, 101))
    worker.frame_reader = lambda: next(frames)
    with pytest.raises(capture.CollectionError, match="world frame changed"):
        worker.send(command(), "unexpected_tick")
    assert journal[0]["server_accepted"] is True and journal[0]["status"] == "FAILED"


def test_command_source_frame_cannot_change_during_camera_or_agent_processing(transport):
    worker, client, journal, _ = transport
    with pytest.raises(capture.CollectionError, match="source frame changed"):
        worker.send(command(), "next_row_command", expected_before_frame=99)
    assert client.calls == [] and journal[0]["status"] == "FAILED"
    assert journal[0]["before_frame"] == 100 and journal[0]["expected_before_frame"] == 99


def test_failed_journal_write_prevents_success_return_and_any_caller_tick(transport):
    worker, client, _, _ = transport
    worker.persist_receipt = lambda _: (_ for _ in ()).throw(OSError("disk full"))
    with pytest.raises(OSError, match="disk full"):
        worker.send(command(), "unrecordable")
    assert len(client.calls) == 1


def test_exact_post_tick_observation_allows_automatic_gear_but_not_control_shift(transport):
    worker, _, _, _ = transport
    expected = worker.send(command(), "next")
    result = worker.compare_observation(command(gear=1), expected, 101, 101, 101)
    assert result["status"] == "PASS" and result["expected_receipt_sequence"] == 1
    assert result["physical_actuation_proven"] is False
    assert result["lookback_relabeling_allowed"] is False
    result = worker.compare_observation(command(throttle=0.14), expected, 101, 101, 101)
    assert result["status"] == "FAIL" and result["mismatched_fields"] == ["throttle"]
    assert worker.alignment_failures == 1


@pytest.mark.parametrize("frames", [(102, 102, 102), (101, 100, 101), (101, 101, 102)])
def test_observation_cannot_bind_a_control_to_another_frame(transport, frames):
    worker, _, _, _ = transport
    expected = worker.send(command(), "next")
    result = worker.compare_observation(command(), expected, *frames)
    assert result["status"] == "FAIL" and result["frame_binding_pass"] is False


@pytest.mark.parametrize("changes,field", [({"hand_brake": True}, "hand_brake"), ({"reverse": True}, "reverse"),
                                         ({"manual_gear_shift": True}, "manual_gear_shift")])
def test_control_boolean_flags_are_exact_not_numeric_tolerances(transport, changes, field):
    worker, _, _, _ = transport
    expected = worker.send(command(), "next")
    assert field in worker.compare_observation(command(**changes), expected, 101, 101, 101)["mismatched_fields"]


def test_manually_commanded_gear_is_compared(transport):
    worker, _, _, _ = transport
    expected = worker.send(command(manual_gear_shift=True, gear=2), "manual")
    result = worker.compare_observation(command(manual_gear_shift=True, gear=1), expected, 101, 101, 101)
    assert result["mismatched_fields"] == ["gear"]


def test_snapshot_kinematics_and_raw_pitch_roll_share_one_immutable_actor():
    vector = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
    transform = SimpleNamespace(location=vector(10, 20, 1), rotation=SimpleNamespace(roll=2, pitch=3, yaw=4))
    actor = SimpleNamespace(id=42, get_transform=lambda: transform, get_velocity=lambda: vector(5, 6, 7),
                            get_acceleration=lambda: vector(8, 9, 10), get_angular_velocity=lambda: vector(11, 12, 13))
    snapshot = SimpleNamespace(find=lambda actor_id: actor if actor_id == 42 else None)
    ego = SimpleNamespace(id=42, get_transform=lambda: pytest.fail("live actor getter must not be used"))
    state, raw = capture.acknowledged_snapshot_state(snapshot, ego, 2.85)
    assert raw["actor_snapshot_transform_carla"]["pitch"] == 3
    assert raw["actor_snapshot_transform_carla"]["roll"] == 2
    assert raw["world_velocity_carla"] == (5, 6, 7)
    assert raw["world_acceleration_carla"] == (8, 9, 10)
    assert raw["world_angular_velocity_carla_deg_s"] == (11, 12, 13)
    assert state == capture.base_link_state(raw["actor_snapshot_transform_carla"], (5, 6, 7), (8, 9, 10), 13, 2.85)
    actor.get_velocity = lambda: vector(float("nan"), 0, 0)
    with pytest.raises(capture.CollectionError, match="nonfinite"):
        capture.acknowledged_snapshot_state(snapshot, ego, 2.85)
    with pytest.raises(capture.CollectionError, match="missing"):
        capture.acknowledged_snapshot_state(SimpleNamespace(find=lambda _: None), ego, 2.85)


def test_cli_default_and_all_owned_control_sites_preserve_one_transport_dispatch():
    assert capture.parse_args(["output", "route"]).control_transport == "legacy_async"
    assert capture.parse_args(["output", "route", "--control-transport", "acknowledged_batch"]).control_transport == "acknowledged_batch"
    with pytest.raises(SystemExit):
        capture.parse_args(["output", "route", "--control-transport", "ack"])
    tree = ast.parse(Path(capture.__file__).read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "collect_episode")
    calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)]
    raw_apply = [node for node in calls if isinstance(node.func, ast.Attribute) and node.func.attr == "apply_control"]
    assert len(raw_apply) == 1
    dispatcher = next(node for node in function.body if isinstance(node, ast.FunctionDef) and node.name == "send_control")
    assert raw_apply[0] in list(ast.walk(dispatcher))
    reasons = {arg.value for node in calls if isinstance(node.func, ast.Name) and node.func.id == "send_control"
               for arg in node.args[1:] if isinstance(arg, ast.Constant)}
    assert {"pre_capture_bootstrap", "stationary_warmup_start", "initial_governor_failure_abort",
            "initial_drive_control", "driving_end_brake", "exception_cleanup_abort"} <= reasons


@pytest.mark.parametrize("mode", ["legacy_async", "acknowledged_batch"])
def test_run_preserves_legacy_manifest_or_explicitly_creates_new_failed_receipt_journal(tmp_path, monkeypatch, mode):
    route = capture.ROOT / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"
    output = tmp_path / mode
    def failed(_args, _route, _specs, partial, states, _cameras, manifest):
        if mode == "legacy_async":
            assert "control_transport" not in manifest["capture_contract"]
            assert "control_receipts" not in manifest["files"]
            assert not (partial / "control_receipts.jsonl").exists()
        else:
            assert manifest["capture_contract"]["control_transport"]["physical_actuation_proven"] is False
            assert (partial / "control_receipts.jsonl").read_text() == ""
            states.append({"frame": 42, "control_transport": {"status": "FAIL"}})
        raise capture.CollectionError("synthetic failure; no simulator")
    monkeypatch.setattr(capture, "collect_episode", failed)
    with pytest.raises(capture.CollectionError, match="synthetic failure"):
        capture.run(capture.parse_args([str(output), str(route), "--control-transport", mode]))
    partial = Path(str(output) + ".partial")
    manifest = json.loads((partial / "manifest.json").read_text())
    assert manifest["status"] == "failed" and not output.exists()
    if mode == "acknowledged_batch":
        assert json.loads((partial / "states.jsonl").read_text())["control_transport"]["status"] == "FAIL"


@pytest.mark.parametrize("journal_read_failure", [False, True])
def test_full_fake_collector_retains_mismatch_camera_then_acknowledges_brake_and_cleans_up(tmp_path, monkeypatch, journal_read_failure):
    # HH_260906 - A synthetic world deliberately reports a stale command; no CARLA client or real sensor is created.
    states, cameras, events, frame, receipts_seen_at_tick, sent = [], [], [], [0], [], []
    partial = tmp_path / "episode.partial"
    partial.mkdir()
    if journal_read_failure:
        # HH_260906 - Evidence read failure must not prevent restoration after the original retained-row failure.
        original_sha = capture.sha256_file
        def broken_journal_sha(path):
            if Path(path).name == "control_receipts.jsonl":
                raise OSError("synthetic journal read failure")
            return original_sha(path)
        monkeypatch.setattr(capture, "sha256_file", broken_journal_sha)
    (partial / "control_receipts.jsonl").write_text("")
    vector = lambda x=0, y=0, z=0: SimpleNamespace(x=x, y=y, z=z)
    transform = lambda: SimpleNamespace(location=vector(1.425, 0, 0), rotation=SimpleNamespace(roll=0, pitch=0, yaw=0))
    actor_snapshot = SimpleNamespace(id=42, get_transform=transform, get_velocity=vector,
        get_acceleration=vector, get_angular_velocity=vector)
    ego = SimpleNamespace(id=42, type_id="vehicle.toyota.prius", attributes={},
        get_control=lambda: command(throttle=0.0, brake=1.0 if frame[0] == 1 else 0.0),
        destroy=lambda: events.append("destroy_ego") or True,
        apply_control=lambda *_: pytest.fail("opt-in must not send an async command"),
        get_transform=lambda: pytest.fail("opt-in must not read live kinematics"),
        get_velocity=lambda: pytest.fail("opt-in must not read live velocity"))
    blueprint = SimpleNamespace(has_attribute=lambda _: False)
    sensors = []
    def spawn(*_args, **_kwargs):
        sensor = SimpleNamespace(id=50 + len(sensors), type_id="sensor.fake", listen=lambda _: None,
            stop=lambda: None, destroy=lambda: events.append("destroy_sensor") or True)
        sensors.append(sensor)
        return sensor
    def tick(_timeout):
        journal = [json.loads(line) for line in (partial / "control_receipts.jsonl").read_text().splitlines()]
        assert journal[-1]["status"] == "ACKNOWLEDGED"
        receipts_seen_at_tick.append(journal[-1]["sequence"])
        frame[0] += 1
        events.append("tick")
        return frame[0]
    settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=0.05, no_rendering_mode=False,
                               substepping=True, max_substep_delta_time=0.01, max_substeps=5)
    world = SimpleNamespace(get_actors=lambda: SimpleNamespace(filter=lambda _: []),
        get_map=lambda: SimpleNamespace(name="Town07"), get_settings=lambda: copy.copy(settings),
        get_weather=lambda: "original", apply_settings=lambda _: events.append("settings"),
        set_weather=lambda _: events.append("weather"), get_blueprint_library=lambda: SimpleNamespace(find=lambda _: blueprint),
        try_spawn_actor=lambda *_: ego, spawn_actor=spawn, tick=tick,
        get_snapshot=lambda: SimpleNamespace(frame=frame[0], timestamp=SimpleNamespace(elapsed_seconds=frame[0] * 0.05),
                                            find=lambda actor_id: actor_snapshot if actor_id == 42 else None))
    client = Client()
    client.get_world, client.set_timeout = lambda: world, lambda _: None
    client.get_server_version = client.get_client_version = lambda: "synthetic_not_CARLA"
    original_batch = client.apply_batch_sync
    def batch(commands, do_tick):
        sent.append(capture.control_dict(commands[0][1]))
        if len(sent) == 3:
            assert len(states) == len(cameras) == 1
            assert states[0]["control_transport"]["status"] == "FAIL"
            assert sent[-1]["brake"] == 1.0 and sent[-1]["throttle"] == 0.0
            events.append("abort_after_camera")
        return original_batch(commands, do_tick)
    client.apply_batch_sync = batch
    carla = ModuleType("carla")
    carla.Client, carla.VehicleControl, carla.Transform = lambda *_: client, lambda **kw: command(**kw), transform
    carla.WeatherParameters = SimpleNamespace(ClearNoon="clear")
    carla.AttachmentType = SimpleNamespace(Rigid="rigid")
    carla.command = SimpleNamespace(ApplyVehicleControl=lambda actor_id, control: (actor_id, control))
    monkeypatch.setitem(sys.modules, "carla", carla)
    for name in ("agents", "agents.navigation", "agents.navigation.basic_agent", "agents.navigation.global_route_planner"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    local_planner = SimpleNamespace(_vehicle_controller=object())
    agent = SimpleNamespace(set_global_plan=lambda _: None, get_local_planner=lambda: local_planner,
                            done=lambda: False, run_step=lambda: command())
    sys.modules["agents.navigation.basic_agent"].BasicAgent = lambda *_a, **_k: agent
    sys.modules["agents.navigation.global_route_planner"].GlobalRoutePlanner = lambda *_: SimpleNamespace(trace_route=lambda *_: [1, 2])
    monkeypatch.setattr(capture, "_python_class_source_provenance", lambda _: {"synthetic": True})
    monkeypatch.setattr(capture, "_carla_transform", lambda *_: transform())
    monkeypatch.setattr(capture, "_carla_location", lambda *_: vector())
    monkeypatch.setattr(capture, "_configure_camera_blueprint", lambda *_: None)
    monkeypatch.setattr(capture, "measured_vehicle_fields", lambda *_: {})
    monkeypatch.setattr(capture, "_validate_image_geometry", lambda *_: None)
    monkeypatch.setattr(capture, "_save_jpeg", lambda *_: events.append("camera"))
    monkeypatch.setattr(capture, "exact_camera_bundle", lambda _q, f, _t: {
        name: SimpleNamespace(timestamp=f * 0.05) for name in capture.MODEL_CAMERA_ORDER})
    route = {"town": "Town07", "start_carla_transform": {"x": 0, "y": 0, "z": 0},
        "goal_carla_transform": {"x": 10, "y": 0, "z": 0},
        "route": [{"x": 0, "y": 0, "distance_m": 0, "vad_command": 3},
                  {"x": 10, "y": 0, "distance_m": 10, "vad_command": 3}]}
    specs = [SimpleNamespace(name=name, carla_type="camera", carla_extrinsic={}) for name in capture.MODEL_CAMERA_ORDER]
    args = capture.parse_args([str(partial), "unused", "--control-transport", "acknowledged_batch"])
    manifest = {"capture_contract": {"control_transport": {}}}
    with pytest.raises(capture.CollectionError, match="mismatched row retained"):
        capture.collect_episode(args, route, specs, partial, states, cameras, manifest)
    assert frame == [2] and receipts_seen_at_tick == [1, 2]
    assert len(states) == len(cameras) == 1 and states[0]["frame"] == cameras[0]["frame"] == 2
    assert events.count("camera") == 6 and "abort_after_camera" in events
    assert events.count("destroy_sensor") == 8 and events.count("destroy_ego") == 1
    assert manifest["cleanup"]["completed"] is (not journal_read_failure)
    assert events.count("settings") == 2 and events.count("weather") == 2
    if journal_read_failure:
        assert manifest["result"]["control_transport"]["receipt_journal_sha256"] is None
        assert "synthetic journal read failure" in manifest["result"]["control_transport"]["receipt_journal_error"]
    journal = [json.loads(line) for line in (partial / "control_receipts.jsonl").read_text().splitlines()]
    assert [row["reason"] for row in journal] == ["pre_capture_bootstrap", "initial_drive_control",
                                                  "alignment_failure_abort", "exception_cleanup_abort"]
    assert all(row["status"] == "ACKNOWLEDGED" and not row["do_tick"] for row in journal)
    assert manifest["result"]["control_transport"]["control_alignment_failure_count"] == 1
