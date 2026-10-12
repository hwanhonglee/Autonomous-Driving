"""HH_261005 - Test owned stationary capture guards and complete failed tails using only scalar fakes; never connect to CARLA."""

import ast
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest

from scripts.e2e import collect_owned_town03_stationary_wheel_rpc_v1_20261005 as capture
from scripts.e2e import run_owned_town03_stationary_wheel_rpc_v1_20261005 as owner


def state(frame=10):
    return dict(frame=frame, world_id=4, actor_id=42, type_id="vehicle.toyota.prius",
                pose=[0., 0., .5, 0., 0., 0.], velocity=[0.]*3, angular_velocity=[0.]*3,
                elapsed_simulation_seconds=frame*.05)


def measure(*, methods=None, observe=None, advance=None, budget=lambda: None, speed=30.):
    current = [10]
    calls, saved = [], []
    def tick():
        current[0] += 1
        calls.append(("tick", current[0]))
        return current[0]
    def read(variant):
        def call(frame):
            calls.append((variant, frame))
            return (1., 1.5), {"synthetic_only": True, "variant": variant}
        return call
    result = capture.measure_frames(state(), observe=observe or (lambda: state(current[0])),
        advance=advance or tick, methods=methods or {name: read(name) for name in "AB"},
        speed_limit=lambda: speed, conversions=capture.original_functions(), budget=budget,
        persist=lambda kind, value: saved.append((kind, copy.deepcopy(value))))
    return result, calls, saved


def test_original_scalar_functions_are_exact_AST_not_collector_import():
    functions = capture.original_functions()
    original = ast.parse(capture.ORIGINAL.read_text())
    expected = {node.name: node for node in original.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                and node.name in functions}
    assert set(expected) == {"CollectionError", "front_steering_measurement", "speed_limit_measurement", "_settings_dict"}
    assert all(functions[name].__code__.co_filename == str(capture.ORIGINAL)
               for name in expected if name != "CollectionError")
    assert functions["front_steering_measurement"](1., 1.5)["front_left_wheel_steer_angle_carla_deg"] == 1.
    assert functions["speed_limit_measurement"](36.)["speed_limit_mps"] == 10.
    assert "torch" not in sys.modules and "carla" not in sys.modules


def test_fixed_thirty_cold_pairs_are_AB_BA_same_frame_and_native_contiguous():
    result, calls, saved = measure()
    assert result["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY"
    assert len(result["actual_native_states"]) == 30
    rows = result["all_declared_variant_attempts"]
    assert len(rows) == 60 and all(row["status"] == "PASS" and row["same_frame_parity_passed"] for row in rows)
    assert [row["variant"] for row in rows] == list("ABBA"*15)
    assert [row["actual_frame"] for row in rows] == [frame for frame in range(10, 40) for _ in range(2)]
    assert calls[:2] == [("A", 10), ("B", 10)] and len([row for row in calls if row[0] == "tick"]) == 29
    assert len(saved) == 90 and saved[0][0] == "native"
    assert result["main_functions_approved"] == result["subfunctions_approved"] == result["retries"] == 0
    assert not result["actual_driving_or_latency_gain_proven"]
    assert rows[0]["before"] == rows[0]["after"] == state()


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 90., -90., None])
def test_original_physical_denials_keep_failed_raw_and_all_unexecuted_tail(bad):
    methods = {"A": lambda frame: ((bad, 1.), {"synthetic_only": True}),
               "B": lambda frame: pytest.fail("no retry or next variant after failure")}
    result, _, saved = measure(methods=methods)
    rows = result["all_declared_variant_attempts"]
    assert result["status"] == "FAIL_RETAINED" and rows[0]["status"] == "FAIL"
    assert len(rows) == 60 and all(row["status"] == "UNEXECUTED" for row in rows[1:])
    assert rows[0]["raw_front_left_right_degrees"][0] == capture.scalar_receipt(bad)
    json.dumps(result, allow_nan=False)
    json.dumps(saved, allow_nan=False)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.])
def test_speed_limit_original_denial_keeps_raw_value_without_fake_conversion(bad):
    result, _, _ = measure(speed=bad)
    first = result["all_declared_variant_attempts"][0]
    assert result["status"] == "FAIL_RETAINED" and first["status"] == "FAIL"
    assert first["cached_speed_limit_raw_kmh"] == capture.scalar_receipt(bad)
    assert "measured_fields" not in first
    json.dumps(result, allow_nan=False)


def test_same_frame_pair_mismatch_remains_global_failure_not_read_success_approval():
    result, _, _ = measure(methods={"A": lambda frame: ((1., 1.5), {}),
                                  "B": lambda frame: ((2., 2.5), {})})
    rows = result["all_declared_variant_attempts"]
    assert result["status"] == "FAIL_RETAINED"
    assert [row["status"] for row in rows[:2]] == ["PASS", "PASS"]
    assert all(row["same_frame_parity_passed"] is False for row in rows[:2])
    assert all(row["status"] == "UNEXECUTED" for row in rows[2:])


@pytest.mark.parametrize("field", ["frame", "world_id", "actor_id", "type_id", "pose", "velocity", "angular_velocity"])
def test_stationarity_and_exact_actual_identity_do_not_allow_tolerance(field):
    bad = state()
    if field == "type_id": bad[field] = "vehicle.other"
    elif isinstance(bad[field], list): bad[field][0] += 1e-12
    else: bad[field] += 1
    with pytest.raises(RuntimeError, match="stationary"):
        capture.require_stationary(bad, state(), 10)


def test_pre_read_guard_failure_is_failed_attempt_no_wheel_call_and_tail_retained():
    count = [0]
    def observe():
        count[0] += 1
        bad = state()
        if count[0] == 2: bad["pose"][0] = 1e-12
        return bad
    result, calls, saved = measure(observe=observe)
    first = result["all_declared_variant_attempts"][0]
    assert first["status"] == "FAIL" and first["wheel_read_attempted"] is False
    assert "started_monotonic_ns" not in first and not calls
    assert saved[-1][0] == "variant"


def test_late_native_tick_never_becomes_a_fabricated_consecutive_observation():
    result, _, _ = measure(advance=lambda: 12)
    assert result["status"] == "FAIL_RETAINED" and len(result["actual_native_states"]) == 1
    assert len(result["all_declared_variant_attempts"]) == 60
    assert all(row["status"] == "UNEXECUTED" for row in result["all_declared_variant_attempts"][2:])


def test_budget_abort_preserves_all_sixty_unexecuted_rows():
    def exhausted(): raise RuntimeError("synthetic unchanged floor reached")
    result, calls, _ = measure(budget=exhausted)
    assert result["status"] == "FAIL_RETAINED" and not calls and not result["actual_native_states"]
    assert all(row["status"] == "UNEXECUTED" for row in result["all_declared_variant_attempts"])


def test_sequential_FR_failure_retains_completed_FL_and_role_clocks():
    class Ego:
        def get_wheel_steer_angle(self, role):
            if role == 2: raise RuntimeError("synthetic FR failure")
            return 1.25
    carla = NS(VehicleWheelLocation=NS(FL_Wheel=1, FR_Wheel=2))
    with pytest.raises(RuntimeError) as caught:
        capture.sequential_read(Ego(), carla)
    assert caught.value.raw_wheel_prefix == [1.25]
    assert [row["status"] for row in caught.value.wheel_receipts] == ["PASS", "FAIL"]
    assert all(row["finished_monotonic_ns"] >= row["started_monotonic_ns"] for row in caught.value.wheel_receipts)


@pytest.fixture
def owned_scalar_world(tmp_path, monkeypatch):
    # HH_261005 - These fakes exercise lifecycle/control flow only: no real actor, connection, asset payload or worker is created.
    expected, launch = "a"*64, "b"*64
    output = tmp_path / "owned_output"
    output.mkdir()
    plan = dict(capture.fixed_contract(), output=str(output), port_triplet=[2440, 2441, 2442],
                worker_command=["/usr/bin/python3", "synthetic_capture", owner.PLAN_TOKEN, owner.AUTH_TOKEN],
                asset_sha256={str(capture.wheels.CARLA_EGG): capture.wheels.CARLA_EGG_SHA256})
    identity = dict(pid=123, ppid=456, pgid=123, start_ticks=999,
                    argv=owner.substituted(plan["worker_command"], expected, launch))
    receipt = {key: dict(pid=456) for key in ("owner", "server", "socket_owner")}
    calls = []
    monkeypatch.setattr(owner, "OUTPUT", output)
    monkeypatch.setattr(owner, "verify_plan", lambda value: plan)
    monkeypatch.setattr(owner, "verify_owner_receipt", lambda *args: receipt)
    monkeypatch.setattr(owner, "verify_launch_authority", lambda *args: calls.append("launch_guard"))
    monkeypatch.setattr(owner, "verify_archived_sources", lambda *args: calls.append("archive_guard"))
    monkeypatch.setattr(owner, "check_budget", lambda *args: calls.append("budget"))
    monkeypatch.setattr(owner, "digest", lambda path: owner.EGG_SHA256 if Path(path) == owner.EGG else "d"*64)
    monkeypatch.setattr(capture.wheels, "digest", lambda path: "d"*64 if Path(path) == capture.wheels.AUTHORITY else __import__("hashlib").sha256(Path(path).read_bytes()).hexdigest())
    monkeypatch.setattr(capture.wheels, "process_record", lambda pid: identity)
    monkeypatch.setattr(capture.os, "getpid", lambda: 123)
    monkeypatch.setattr(capture.os, "getppid", lambda: 456)
    monkeypatch.setattr(capture.os, "getsid", lambda pid: 123)
    authority = tmp_path / "synthetic_endpoint_authority.json"
    authority.write_text("{}")
    monkeypatch.setattr(capture.wheels, "AUTHORITY", authority)
    original = NS(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False,
                  substepping=True, max_substep_delta_time=.01, max_substeps=10)
    world = NS(id=4, frame=9, ego=None, settings=copy.deepcopy(original))
    def transform(): return NS(location=NS(x=0., y=0., z=0.), rotation=NS(roll=0., pitch=0., yaw=0.))
    pose = transform()
    pose.location.z = .5
    ego = NS(id=42, type_id="vehicle.toyota.prius", get_transform=lambda: pose,
             get_velocity=lambda: NS(x=0., y=0., z=0.), get_angular_velocity=lambda: NS(x=0., y=0., z=0.),
             get_speed_limit=lambda: 30., get_wheel_steer_angle=lambda role: (1., 1.5)[role],
             set_simulate_physics=lambda value: calls.append(("physics", value)))
    ego.destroy = lambda: calls.append("destroy_owned") or setattr(world, "ego", None) or True
    world.get_snapshot = lambda: NS(frame=world.frame, timestamp=NS(elapsed_seconds=world.frame*.05),
                                   find=lambda ident: ego if world.ego is not None else None)
    world.get_actor = lambda ident: world.ego
    world.get_settings = lambda: copy.deepcopy(world.settings)
    def settings(value):
        calls.append("apply_settings")
        world.settings = copy.deepcopy(value)
    world.apply_settings = settings
    world.get_actors = lambda: NS(filter=lambda pattern: [] if world.ego is None else [ego])
    world.get_map = lambda: NS(name="Town03", get_spawn_points=lambda: [transform()])
    world.get_blueprint_library = lambda: NS(find=lambda name: name)
    def spawn(blueprint, location):
        assert blueprint == "vehicle.toyota.prius" and location.location.z == .5
        calls.append("spawn_owned")
        world.ego = ego
        return ego
    world.spawn_actor = spawn
    def tick():
        world.frame += 1
        calls.append(("tick", world.frame))
        return world.frame
    world.tick = tick
    class Reader:
        def __init__(self, proof, **kwargs):
            calls.append("reader_ready")
            self.rows, self.closed = [], False
            self.first = proof["first_frame"]
        def read(self, frame, callback):
            assert frame == self.first + len(self.rows) and callback() == (frame, 4, 42, "vehicle.toyota.prius")
            self.rows.append(dict(status="PASS", frame=frame, synthetic_only=True))
            return 1., 1.5
        def close(self):
            self.closed = True
            calls.append("workers_joined")
        def receipt(self): return dict(all_owned_children_joined=self.closed, cleanup_errors=[], synthetic_only=True)
    monkeypatch.setattr(capture.wheels, "OwnedWheelReader", Reader)
    def client(host, port):
        assert host == "127.0.0.1" and port == 2440
        calls.append("Client")
        return NS(set_timeout=lambda seconds: None, get_world=lambda: world)
    module = NS(__file__=str(owner.EGG)+"/carla/__init__.py", Client=client,
                VehicleWheelLocation=NS(FL_Wheel=0, FR_Wheel=1))
    monkeypatch.setitem(sys.modules, "carla", module)
    before_path = list(sys.path)
    yield NS(plan=plan, expected=expected, launch=launch, output=output, identity=identity,
             receipt=receipt, calls=calls, world=world, original=original)
    sys.path[:] = before_path


def run(value):
    return capture.run_capture(value.plan, value.expected, launch_authority_sha256=value.launch)


def test_full_fake_owned_capture_guard_order_cold_denominator_and_cleanup(owned_scalar_world):
    value = owned_scalar_world
    report = run(value)
    assert report["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY" and report["cleanup_verified"]
    assert value.calls.index("archive_guard") < value.calls.index("Client")
    assert value.calls.index("workers_joined") < value.calls.index("destroy_owned")
    assert len([item for item in value.calls if isinstance(item, tuple) and item[0] == "tick"]) == 30
    assert len(report["measurements"]["all_declared_variant_attempts"]) == 60
    assert value.world.settings.__dict__ == value.original.__dict__
    assert json.loads((value.output / "capture_result.json").read_text())["status"] == report["status"]
    assert len(list((value.output / "capture_events").iterdir())) == 90
    assert not report["equivalent_to_moving_full209"]


@pytest.mark.parametrize("fault", ["argv", "PPID", "PGID", "session"])
def test_exact_capture_identity_denies_before_client_or_actor(owned_scalar_world, monkeypatch, fault):
    value = owned_scalar_world
    if fault == "argv": value.identity["argv"] += ["unapproved"]
    elif fault == "PPID": value.identity["ppid"] = 457
    elif fault == "PGID": value.identity["pgid"] = 999
    else: monkeypatch.setattr(capture.os, "getsid", lambda pid: 999)
    with pytest.raises(RuntimeError, match="owner child/argv/session"):
        run(value)
    assert "Client" not in value.calls and not (value.output / "capture_result.json").exists()


@pytest.mark.parametrize("stage", ["plan", "launch", "archive", "original_functions", "client"])
def test_authentic_owner_early_failure_retains_fixed_unexecuted_denominator(owned_scalar_world, monkeypatch, stage):
    value = owned_scalar_world
    def denied(*args, **kwargs): raise RuntimeError("synthetic " + stage + " denial")
    if stage == "plan": monkeypatch.setattr(owner, "verify_plan", denied)
    elif stage == "launch": monkeypatch.setattr(owner, "verify_launch_authority", denied)
    elif stage == "archive": monkeypatch.setattr(owner, "verify_archived_sources", denied)
    elif stage == "original_functions": monkeypatch.setattr(capture, "original_functions", denied)
    else: monkeypatch.setattr(sys.modules["carla"], "Client", denied)
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and stage in report["error"]
    assert report["cleanup_verified"] is False and report["cleanup_attempted"] is False
    assert len(report["measurements"]["all_declared_variant_attempts"]) == 60
    assert all(row["status"] == "UNEXECUTED" for row in report["measurements"]["all_declared_variant_attempts"])
    assert (value.output / "capture_result.json").is_file() and "spawn_owned" not in value.calls


def test_startup_worker_error_receipt_retained_and_unjoined_blocks_mutation(owned_scalar_world, monkeypatch):
    value = owned_scalar_world
    receipt = dict(all_owned_children_joined=False, cleanup_errors=["synthetic startup failure"],
                   all_requests_including_failures=[], owned_children={"FL": {"synthetic_only": True}})
    def fail(*args, **kwargs): raise capture.wheels.WheelReadError("synthetic startup failure", receipt)
    monkeypatch.setattr(capture.wheels, "OwnedWheelReader", fail)
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and report["wheel_worker_receipt"] == receipt
    assert "destroy_owned" not in value.calls and report["cleanup_verified"] is False
    assert any("unjoined" in row for row in report["cleanup_errors"])
    assert all(row["status"] == "UNEXECUTED" for row in report["measurements"]["all_declared_variant_attempts"])


def test_result_is_create_only_and_never_replaces_prior_attempt(owned_scalar_world):
    value = owned_scalar_world
    path = value.output / "capture_result.json"
    path.write_bytes(b"preserved prior attempt")
    with pytest.raises(FileExistsError): run(value)
    assert path.read_bytes() == b"preserved prior attempt"


def test_prepared_source_has_no_teacher_NN_camera_or_map_reload():
    source = Path(capture.__file__).read_text()
    tree = ast.parse(source)
    forbidden = {"load_world", "reload_world", "apply_control", "apply_batch", "apply_batch_sync",
                 "set_autopilot", "set_weather", "get_image", "backward", "forward"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                   and node.func.attr in forbidden for node in ast.walk(tree))
    assert not any(isinstance(node, ast.Import) and any(alias.name == "torch" for alias in node.names)
                   for node in ast.walk(tree))
    assert capture.fixed_contract()["bootstrap_native_ticks"] == 1
    assert capture.fixed_contract()["vehicle_blueprint"] == "vehicle.toyota.prius"
