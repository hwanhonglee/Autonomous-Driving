"""HH_261005 - Test fresh V4 failure-prefix snapshot retention and owned map initialization and unchanged stationary capture guards with scalar fakes only; never connect to CARLA."""

import ast
import copy
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest

from scripts.e2e import collect_owned_town03_stationary_wheel_rpc_v4_20261005 as capture
from scripts.e2e import run_owned_town03_stationary_wheel_rpc_v4_20261005 as owner


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
                asset_sha256={str(capture.wheels.CARLA_EGG): capture.wheels.CARLA_EGG_SHA256},
                native_runtime_input={"cache_path": str(output / "synthetic_native_cache"),
                                      "native_path": str(output / "synthetic_native_cache/libcarla.so")})
    identity = dict(pid=123, ppid=456, pgid=123, start_ticks=999,
                    argv=owner.substituted(plan["worker_command"], expected, launch))
    receipt = {key: dict(pid=456) for key in ("owner", "server", "socket_owner")}
    calls = []
    monkeypatch.setattr(owner, "OUTPUT", output)
    monkeypatch.setattr(owner, "verify_plan", lambda value: plan)
    monkeypatch.setattr(owner, "verify_owner_receipt", lambda *args: receipt)
    monkeypatch.setattr(owner, "verify_launch_authority", lambda *args: calls.append("launch_guard"))
    monkeypatch.setattr(owner, "verify_archived_sources", lambda *args: calls.append("archive_guard"))
    monkeypatch.setattr(owner, "verify_native_runtime_input", lambda *args: calls.append("native_guard") or {"synthetic_only": True})
    monkeypatch.setenv("PYTHON_EGG_CACHE", plan["native_runtime_input"]["cache_path"])
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
    # HH_261005 - Tiny explicit known-map geometry is a synthetic flow fixture, never an actual Town03 or reserved TEST payload.
    world.map_name, world.map_xodr = "Town03", "synthetic Town03 UTF8 \u03bb"
    monkeypatch.setattr(owner, "TOWN03_XODR_SIZE", len(world.map_xodr.encode("utf-8")))
    monkeypatch.setattr(owner, "TOWN03_XODR_SHA256", hashlib.sha256(world.map_xodr.encode("utf-8")).hexdigest())
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
    def opendrive():
        calls.append("Town03_ODR")
        assert world.map_name in owner.MAP_IDENTIFIERS, "no initial reserved geometry access"
        return world.map_xodr
    world.get_map = lambda: NS(name=world.map_name, to_opendrive=opendrive, get_spawn_points=lambda: [transform()])
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
            self.world_id = proof["world_id"]
        def read(self, frame, callback):
            assert frame == self.first + len(self.rows) and callback() == (frame, self.world_id, 42, "vehicle.toyota.prius")
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
        def load(name):
            calls.append(("load_world", name))
            raise RuntimeError("synthetic load failure before any actor or worker")
        return NS(set_timeout=lambda seconds: calls.append(("timeout", seconds)), get_world=lambda: world, load_world=load)
    module = NS(__file__=str(owner.EGG)+"/carla/__init__.py", Client=client,
                VehicleWheelLocation=NS(FL_Wheel=0, FR_Wheel=1))
    monkeypatch.setitem(sys.modules, "carla", module)
    monkeypatch.setitem(sys.modules, "carla.libcarla", NS(__file__=plan["native_runtime_input"]["native_path"]))
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


def test_prepared_source_has_only_one_explicit_pre_actor_owned_map_initialization():
    source = Path(capture.__file__).read_text()
    tree = ast.parse(source)
    forbidden = {"reload_world", "apply_control", "apply_batch", "apply_batch_sync",
                 "set_autopilot", "set_weather", "get_image", "backward", "forward"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                   and node.func.attr in forbidden for node in ast.walk(tree))
    assert not any(isinstance(node, ast.Import) and any(alias.name == "torch" for alias in node.names)
                   for node in ast.walk(tree))
    assert capture.fixed_contract()["bootstrap_native_ticks"] == 1
    assert capture.fixed_contract()["vehicle_blueprint"] == "vehicle.toyota.prius"
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == "load_world"]
    assert len(calls) == 1 and ast.unparse(calls[0]) == "client.load_world('Town03')"
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_initialize_owned_map")
    assert calls[0] in list(ast.walk(function))
    assert capture.fixed_contract()["initialize_owned_map_if_needed"] == "Town03"
    assert capture.fixed_contract()["maximum_load_world_calls"] == 1
    assert capture.fixed_contract()["before_any_actor_or_worker"] is True


def test_exact_unmodified_scalar_and_measurement_functions_match_V1_AST():
    # HH_261005 - Actual public functions retain the original sixty-attempt math and failure tails, not a stand-in implementation.
    original = ast.parse((capture.ROOT / "tools/e2e/collect_owned_town03_stationary_wheel_rpc_v1_20261005.py").read_text())
    revised = ast.parse(Path(capture.__file__).read_text())
    bodies = lambda tree: {node.name: ast.dump(node, include_attributes=False) for node in tree.body if isinstance(node, ast.FunctionDef)}
    old, new = bodies(original), bodies(revised)
    for name in set(old) - {"fixed_contract", "_run_capture", "run_capture", "main"}:
        assert old[name] == new[name]
    assert capture.measure_frames.__globals__ is capture.__dict__
    assert capture._run_capture.__globals__["original_functions"] is capture.original_functions


def test_full_public_capture_native_input_precedes_client_and_after_seal_is_saved(owned_scalar_world):
    value = owned_scalar_world
    report = run(value)
    assert report["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY"
    assert value.calls.index("native_guard") < value.calls.index("Client")
    assert report["native_runtime_input_before"] == report["native_runtime_input_after"] == {"synthetic_only": True}
    assert report["actual_native_module_path"] == value.plan["native_runtime_input"]["native_path"]
    assert report["actual_map_identity"] == "Town03"


@pytest.mark.parametrize("fault", ["missing_environment", "wrong_environment", "native_input", "missing_native_module", "wrong_native_origin"])
def test_native_input_denied_with_complete_fixed_ledger_before_client(owned_scalar_world, monkeypatch, fault):
    value = owned_scalar_world
    if fault == "missing_environment": monkeypatch.delenv("PYTHON_EGG_CACHE")
    elif fault == "wrong_environment": monkeypatch.setenv("PYTHON_EGG_CACHE", "/synthetic/unsealed")
    elif fault == "missing_native_module": monkeypatch.delitem(sys.modules, "carla.libcarla")
    elif fault == "wrong_native_origin": monkeypatch.setattr(sys.modules["carla.libcarla"], "__file__", "/synthetic/unsealed_native.so")
    else:
        def deny(*args): raise RuntimeError("synthetic truncated native input")
        monkeypatch.setattr(owner, "verify_native_runtime_input", deny)
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and "Client" not in value.calls
    rows = report["measurements"]["all_declared_variant_attempts"]
    assert len(rows) == 60 and all(row["status"] == "UNEXECUTED" for row in rows)


def test_post_capture_native_change_preserves_actual_measurements_but_global_failure(owned_scalar_world, monkeypatch):
    value = owned_scalar_world
    count = [0]
    def verify(*args):
        count[0] += 1
        if count[0] == 2: raise RuntimeError("synthetic native post-seal differs")
        return {"synthetic_only": True}
    monkeypatch.setattr(owner, "verify_native_runtime_input", verify)
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and report["cleanup_verified"]
    assert len(report["measurements"]["actual_native_states"]) == 30
    assert all(row["status"] == "PASS" for row in report["measurements"]["all_declared_variant_attempts"])
    assert any("native post-seal differs" in row for row in report["cleanup_errors"])


@pytest.fixture
def map_probe():
    # HH_261005 - Initialization is tested with name/count metadata and tiny known-map strings, never reserved map geometry or a real endpoint.
    calls, worlds = [], []
    raw = "synthetic Town03 UTF8 \u03bb"
    authority = NS(MAP_IDENTIFIERS=owner.MAP_IDENTIFIERS,
                   TOWN03_XODR_SIZE=len(raw.encode("utf-8")),
                   TOWN03_XODR_SHA256=hashlib.sha256(raw.encode("utf-8")).hexdigest())
    def verified(*args):
        calls.append("owner_guard")
        return {}
    authority.verify_owner_receipt = verified
    authority.verify_launch_authority = lambda *args: calls.append("ROOT_launch_guard")
    def world(name, epoch, *, geometry=raw, counts=None):
        item = NS(id=epoch, name=name, geometry=geometry,
                  counts=dict(vehicles=0, walkers=0, sensors=0) if counts is None else counts)
        def odr():
            calls.append(("ODR", item.id))
            assert item.name in authority.MAP_IDENTIFIERS, "reserved/unknown initial geometry forbidden"
            return item.geometry
        def forbidden(): pytest.fail("no snapshot, state, geometry or spawn inspection during metadata-only map initialization")
        item.get_map = lambda: NS(name=item.name, to_opendrive=odr, get_spawn_points=forbidden)
        def actors():
            calls.append(("actor_counts", item.id))
            return NS(filter=lambda pattern: [object()] * item.counts[{"vehicle.*": "vehicles", "walker.*": "walkers", "sensor.*": "sensors"}[pattern]])
        item.get_actors = actors
        item.get_snapshot = item.get_settings = item.spawn_actor = forbidden
        worlds.append(item)
        return item
    initial, target = world("Town10HD_Opt", 4), world("Carla/Maps/Town03", 5)
    active = [initial]
    def load(name):
        calls.append(("load_world", name))
        active[0] = target
        return target
    client = NS(get_world=lambda: active[0], load_world=load,
                set_timeout=lambda seconds: calls.append(("timeout", seconds)))
    return NS(calls=calls, owner=authority, make_world=world, initial=initial, target=target,
              active=active, client=client, plan=capture.fixed_contract(), report={}, expected="a"*64, launch="b"*64)


def initialize(value):
    return capture._initialize_owned_map(value.client, value.initial, value.owner, value.plan,
                                         value.expected, value.launch, value.report)


@pytest.mark.parametrize("name", ["Town03", "/Game/Carla/Maps/Town03", "Carla/Maps/Town03"])
def test_each_exact_known_map_name_records_XODR_without_load_or_reserved_payload(map_probe, name):
    value = map_probe
    value.initial.name = name
    assert initialize(value) is value.initial
    report = value.report
    assert report["actual_map_identity"] == name and report["actual_map_identity_repr"] == repr(name)
    assert report["actual_Town03_map"]["OpenDrive_sha256"] == value.owner.TOWN03_XODR_SHA256
    assert report["actual_Town03_map"]["OpenDrive_size_bytes"] == value.owner.TOWN03_XODR_SIZE
    assert report["map_initialization"]["completed"] and report["map_initialization"]["load_world_calls"] == 0
    assert not any(isinstance(row, tuple) and row[0] == "load_world" for row in value.calls)


def test_nonTown03_reads_only_metadata_until_one_authorized_load_new_epoch_and_exact_target(map_probe):
    value = map_probe
    assert initialize(value) is value.target
    report = value.report["map_initialization"]
    assert report["initial_world"] == dict(world_id=4, map_name="Town10HD_Opt", map_name_repr=repr("Town10HD_Opt"),
                                            active_actor_counts=dict(vehicles=0, walkers=0, sensors=0))
    assert report["completed"] and report["load_world_calls"] == 1
    assert report["no_initial_non_Town03_geometry_state_images_labels_read"] is True
    assert [row for row in value.calls if isinstance(row, tuple) and row[0] == "ODR"] == [("ODR", 5)]
    load = value.calls.index(("load_world", "Town03"))
    assert value.calls.index("ROOT_launch_guard") < load < value.calls.index(("ODR", 5))
    assert value.calls[:load].count("owner_guard") == 2 and value.calls[-1] == "owner_guard"
    assert value.calls[load-1:load+2] == [("timeout", 30.), ("load_world", "Town03"), ("timeout", .5)]
    assert report["normal_stationary_RPC_timeout_restored"] is True


@pytest.mark.parametrize("fault", ["wrong_size", "same_size_wrong_SHA", "bytes_not_str"])
def test_wrong_known_Town03_XODR_is_recorded_and_denied_without_repair_load(map_probe, fault):
    value = map_probe
    value.initial.name = "Town03"
    if fault == "wrong_size": value.initial.geometry += "x"
    elif fault == "same_size_wrong_SHA": value.initial.geometry = "X" + value.initial.geometry[1:]
    else: value.initial.geometry = b"synthetic invalid native ABI"
    with pytest.raises(RuntimeError, match="OpenDrive"):
        initialize(value)
    assert value.report["actual_map_identity"] == "Town03"
    assert value.report["map_initialization"]["load_world_calls"] == 0
    assert not value.report["map_initialization"]["completed"]
    assert not any(isinstance(row, tuple) and row[0] == "load_world" for row in value.calls)
    if fault != "bytes_not_str":
        document = value.report["actual_Town03_map"]
        assert document["OpenDrive_sha256"] == hashlib.sha256(value.initial.geometry.encode("utf-8")).hexdigest()


@pytest.mark.parametrize("role", ["vehicles", "walkers", "sensors"])
def test_any_foreign_initial_actor_denies_load_and_all_geometry_reads(map_probe, role):
    value = map_probe
    value.initial.counts[role] = 1
    with pytest.raises(RuntimeError, match="active actors"): initialize(value)
    assert value.report["map_initialization"]["initial_world"]["active_actor_counts"][role] == 1
    assert not any(isinstance(row, tuple) and row[0] in ("ODR", "load_world") for row in value.calls)


@pytest.mark.parametrize("fault", ["ROOT_authority", "initial_owner", "owner_reproof", "original_epoch", "late_foreign_actor"])
def test_failed_authority_or_ownership_reproof_never_loads_or_reads_initial_reserved_geometry(map_probe, fault):
    value = map_probe
    def denied(*args): raise RuntimeError("synthetic refused authority or ownership")
    if fault == "ROOT_authority": value.owner.verify_launch_authority = denied
    elif fault == "initial_owner": value.owner.verify_owner_receipt = denied
    elif fault == "owner_reproof":
        count = [0]
        def reproof(*args):
            count[0] += 1
            if count[0] == 2: denied()
            return {}
        value.owner.verify_owner_receipt = reproof
    elif fault == "original_epoch": value.client.get_world = lambda: NS(id=99)
    else:
        def late_counts():
            value.initial.counts["vehicles"] = 1
            return value.initial
        value.client.get_world = late_counts
    with pytest.raises(RuntimeError): initialize(value)
    assert not any(isinstance(row, tuple) and row[0] in ("load_world", "ODR") for row in value.calls)


@pytest.mark.parametrize("fault", ["count_bool", "count_zero", "count_two", "wrong_target", "after_actor_scope",
                                   "initialization_timeout_bool", "initialization_timeout_wrong", "stationary_timeout_wrong", "worker_definition"])
def test_initialization_requires_exact_prospective_scope_not_truthy_count(map_probe, fault):
    value = map_probe
    if fault == "count_bool": value.plan["maximum_load_world_calls"] = True
    elif fault == "count_zero": value.plan["maximum_load_world_calls"] = 0
    elif fault == "count_two": value.plan["maximum_load_world_calls"] = 2
    elif fault == "wrong_target": value.plan["initialize_owned_map_if_needed"] = "Town07"
    elif fault == "after_actor_scope": value.plan["before_any_actor_or_worker"] = False
    elif fault == "initialization_timeout_bool": value.plan["initialization_RPC_timeout_seconds"] = True
    elif fault == "initialization_timeout_wrong": value.plan["initialization_RPC_timeout_seconds"] = 31.
    elif fault == "stationary_timeout_wrong": value.plan["normal_stationary_RPC_timeout_seconds"] = 1.
    else: value.plan["worker_definition"] = "no capture setup process exists"
    with pytest.raises(RuntimeError, match="single declared"): initialize(value)
    assert value.report["map_initialization"]["load_world_calls"] == 0
    assert not any(isinstance(row, tuple) and row[0] in ("load_world", "ODR") for row in value.calls)


@pytest.mark.parametrize("fault", ["load_error", "no_world", "same_epoch", "wrong_target_name", "target_XODR", "target_inventory", "postload_epoch"])
def test_failed_one_load_retains_unknown_initial_and_target_prefix_without_retry(map_probe, fault):
    value = map_probe
    if fault == "load_error":
        def failed(name):
            value.calls.append(("load_world", name))
            raise RuntimeError("synthetic load failure")
        value.client.load_world = failed
    elif fault == "no_world":
        value.client.load_world = lambda name: value.calls.append(("load_world", name))
    elif fault == "same_epoch": value.target.id = value.initial.id
    elif fault == "wrong_target_name": value.target.name = "Town03_extra"
    elif fault == "target_XODR": value.target.geometry += "wrong"
    elif fault == "target_inventory": value.target.counts["sensors"] = 1
    else:
        value.client.get_world = lambda: value.initial if value.active[0] is value.initial else NS(id=99)
    with pytest.raises(RuntimeError): initialize(value)
    assert value.report["map_initialization"]["load_world_calls"] == 1
    assert value.report["map_initialization"]["completed"] is False
    assert value.report["map_initialization"]["initial_world"]["map_name"] == "Town10HD_Opt"
    assert [row for row in value.calls if isinstance(row, tuple) and row[0] == "load_world"] == [("load_world", "Town03")]
    assert [row for row in value.calls if isinstance(row, tuple) and row[0] == "timeout"] == [("timeout", 30.), ("timeout", .5)]
    assert value.report["map_initialization"]["normal_stationary_RPC_timeout_restored"] is True
    if fault in ("load_error", "no_world", "same_epoch", "wrong_target_name"):
        assert not any(isinstance(row, tuple) and row[0] == "ODR" for row in value.calls)
    if fault == "wrong_target_name":
        assert value.report["actual_map_identity"] == "Town03_extra"
        assert value.report["actual_map_identity_repr"] == repr("Town03_extra")


@pytest.mark.parametrize("name", ["Town03", "/Game/Carla/Maps/Town03", "Carla/Maps/Town03"])
def test_full_scalar_capture_each_exact_name_keeps_original_sixty_denominator(owned_scalar_world, name):
    value = owned_scalar_world
    value.world.map_name = name
    report = run(value)
    assert report["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY"
    assert report["map_initialization"]["completed"] and report["map_initialization"]["load_world_calls"] == 0
    assert len(report["measurements"]["actual_native_states"]) == 30
    assert len(report["measurements"]["all_declared_variant_attempts"]) == 60


@pytest.mark.parametrize("fault", ["unknown_initial", "wrong_known_XODR"])
def test_full_failed_startup_keeps_exact_initial_name_and_sixty_unexecuted_tail(owned_scalar_world, fault):
    value = owned_scalar_world
    if fault == "unknown_initial": value.world.map_name = "Town10HD_Opt"
    else: value.world.map_xodr += "wrong"
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and not report["cleanup_attempted"] and not report["cleanup_verified"]
    assert report["actual_map_identity"] == value.world.map_name
    assert report["actual_map_identity_repr"] == repr(value.world.map_name)
    assert report["map_initialization"]["completed"] is False
    rows = report["measurements"]["all_declared_variant_attempts"]
    assert len(rows) == 60 and all(row["status"] == "UNEXECUTED" for row in rows)
    assert not report["measurements"]["actual_native_states"]
    assert "spawn_owned" not in value.calls and "reader_ready" not in value.calls and "apply_settings" not in value.calls
    assert len([row for row in value.calls if isinstance(row, tuple) and row[0] == "load_world"]) == (fault == "unknown_initial")


def test_full_public_capture_after_one_owned_load_uses_new_epoch_and_restores_new_world(owned_scalar_world, monkeypatch):
    # HH_261005 - Exercise the real public capture path after initialization; a live World proxy may itself expose the newly loaded epoch.
    value = owned_scalar_world
    value.world.map_name = "Town10HD_Opt"
    original_client = sys.modules["carla"].Client
    def client(host, port):
        actual = original_client(host, port)
        def load(name):
            assert name == "Town03"
            value.calls.append(("load_world", name))
            value.world.id, value.world.map_name = 5, "Town03"
            return value.world
        actual.load_world = load
        return actual
    monkeypatch.setattr(sys.modules["carla"], "Client", client)
    report = run(value)
    assert report["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY" and report["cleanup_verified"]
    initialization = report["map_initialization"]
    assert initialization["initial_world"]["world_id"] == 4 and initialization["initial_world"]["map_name"] == "Town10HD_Opt"
    assert initialization["load_world_calls"] == 1 and initialization["completed"] is True
    assert report["actual_Town03_map"]["world_id"] == report["world_id"] == 5
    assert [row for row in value.calls if isinstance(row, tuple) and row[0] == "load_world"] == [("load_world", "Town03")]
    assert value.calls.index(("load_world", "Town03")) < value.calls.index("Town03_ODR") < value.calls.index("spawn_owned")
    assert [row for row in value.calls if isinstance(row, tuple) and row[0] == "timeout"] == [("timeout", .5), ("timeout", 30.), ("timeout", .5)]
    assert initialization["normal_stationary_RPC_timeout_restored"] is True
    assert value.calls.index(("timeout", 30.)) < value.calls.index(("load_world", "Town03")) < max(index for index, row in enumerate(value.calls) if row == ("timeout", .5)) < value.calls.index("spawn_owned")
    measurements = report["measurements"]
    assert len(measurements["actual_native_states"]) == 30 and len(measurements["all_declared_variant_attempts"]) == 60
    assert all(row["world_id"] == 5 for row in measurements["actual_native_states"])
    assert all(row["status"] == "PASS" for row in measurements["all_declared_variant_attempts"])
    assert report["original_settings"] == report["restored_settings"] and value.world.settings.__dict__ == value.original.__dict__
    assert report["owned_actor_destroyed"] and value.world.ego is None
    assert value.calls.index("workers_joined") < value.calls.index("destroy_owned")
    assert len(list((value.output / "capture_events").iterdir())) == 90


@pytest.mark.parametrize("channel,component,amount", [("velocity", "x", 1e-12), ("velocity", "y", -.25),
                                                     ("angular_velocity", "z", 1e-12), ("angular_velocity", "x", -.25)])
def test_first_finite_nonzero_snapshot_is_retained_before_exact_stationarity_denial(owned_scalar_world, monkeypatch, channel, component, amount):
    # HH_261005 - Keep a genuine observed scalar snapshot before its unchanged zero-motion guard; never clear velocity, advance extra ticks or weaken thresholds.
    value = owned_scalar_world
    spawn = value.world.spawn_actor
    def changed(blueprint, pose):
        ego = spawn(blueprint, pose)
        vector = dict(x=0., y=0., z=0.)
        vector[component] = amount
        setattr(ego, "get_" + channel, lambda: NS(**vector))
        return ego
    monkeypatch.setattr(value.world, "spawn_actor", changed)
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and "stationary" in report["error"]
    first = report["admission_snapshot"]
    assert first["frame"] == 10 and first["world_id"] == 4 and first["actor_id"] == 42
    expected = [0., 0., 0.]
    expected[("x", "y", "z").index(component)] = amount
    assert first[channel] == expected
    assert first["velocity" if channel == "angular_velocity" else "angular_velocity"] == [0., 0., 0.]
    rows = report["measurements"]["all_declared_variant_attempts"]
    assert len(rows) == 60 and all(row["status"] == "UNEXECUTED" for row in rows)
    assert report["measurements"]["actual_native_states"] == []
    assert not (value.output / "capture_ready.json").exists() and "reader_ready" not in value.calls
    assert [row for row in value.calls if isinstance(row, tuple) and row[0] == "tick"] == [("tick", 10)]
    assert report["cleanup_verified"] and report["owned_actor_destroyed"]
    assert report["original_settings"] == report["restored_settings"]
    retained = json.loads((value.output / "capture_result.json").read_text())
    assert retained["admission_snapshot"] == first and retained["status"] == "FAIL_RETAINED"


def test_actual_snapshot_is_retained_before_wrong_admission_tick_frame_denial(owned_scalar_world, monkeypatch):
    value = owned_scalar_world
    tick = value.world.tick
    monkeypatch.setattr(value.world, "tick", lambda: tick() + 1)
    report = run(value)
    assert report["status"] == "FAIL_RETAINED" and "admission snapshot/frame mismatch" in report["error"]
    assert report["admission_snapshot"]["frame"] == 10 and report["admission_snapshot"]["velocity"] == [0., 0., 0.]
    assert not report["measurements"]["actual_native_states"]
    assert len(report["measurements"]["all_declared_variant_attempts"]) == 60
    assert all(row["status"] == "UNEXECUTED" for row in report["measurements"]["all_declared_variant_attempts"])
    assert not (value.output / "capture_ready.json").exists() and report["cleanup_verified"]


def test_success_ready_keeps_authentic_map_initialization_and_first_snapshot_documents(owned_scalar_world):
    value = owned_scalar_world
    report = run(value)
    ready = json.loads((value.output / "capture_ready.json").read_text())
    assert report["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY"
    assert ready["actual_Town03_map"] == report["actual_Town03_map"]
    assert ready["map_initialization"] == report["map_initialization"]
    assert ready["actual_snapshot"] == report["admission_snapshot"] == report["measurements"]["actual_native_states"][0]
    assert ready["actual_Town03_map"]["map_name"] == "Town03" and ready["actual_Town03_map"]["world_id"] == ready["world_id"] == 4
    assert ready["actual_Town03_map"]["OpenDrive_sha256"] == owner.TOWN03_XODR_SHA256
    assert ready["map_initialization"]["load_world_calls"] == 0 and ready["map_initialization"]["completed"] is True
    assert capture.fixed_contract()["admission_snapshot_before_stationary_guard"] is True
    assert capture.fixed_contract()["physical_setup_unchanged_from_V3"] is True
    assert capture.fixed_contract()["velocity_reset_calls"] == 0
