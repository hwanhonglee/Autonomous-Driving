"""HH_261005 - Try one real owned angular-zero API setup before physics disable and prove its effect only from unchanged actual admission guards.

Actual execution requires a separately sealed owner, ROOT launch authority and
ROOT V2 endpoint authority. Importing this module never imports CARLA or Torch.
Only this experiment's actor/settings may change; failed and unexecuted tails
remain explicit. Disabled actor physics is not moving-vehicle equivalence.
"""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any

from . import owned_parallel_wheel_reads_v2_20261005 as wheels


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.owned_town03_stationary_wheel_rpc.20261005.v6"
ORIGINAL = ROOT / "tools/e2e/collect_carla_vad_expert.py"
ORIGINAL_SHA256 = "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"
WHEEL_SOURCE_SHA256 = "06dea19fea1f2dab00b5813f7631ca7d7bd76f6e08caa677c08f30cd3104c71f"
FRAMES = 30
require = wheels.require


def fixed_contract():
    return dict(measured_frames=30, physics_hz=20, fixed_delta_seconds=.05,
                variant_calls=60, wheel_rpc_calls=120, vehicle_blueprint="vehicle.toyota.prius",
                spawn_index=0, spawn_z_offset_m=.5, actorphysics_disabled=True,
                authority_wait_s=120, bootstrap_native_ticks=1, initialize_owned_map_if_needed="Town03",
                maximum_load_world_calls=1, before_any_actor_or_worker=True, world_reload=True,
                worker_definition="wheel_read_worker; capture_setup_process_exists",
                initialization_RPC_timeout_seconds=30., normal_stationary_RPC_timeout_seconds=.5,
                admission_snapshot_before_stationary_guard=True, physical_setup_unchanged_from_V3=False,
                velocity_reset_calls=1, angular_velocity_reset_calls=1, linear_velocity_reset_calls=0,
                angular_velocity_setup_before_physics_disable=True, target_angular_velocity_degrees_per_second=[0., 0., 0.])


def original_functions():
    """HH_261005 - Compile only authentic scalar AST functions/error class; never import the original collector."""
    require(wheels.digest(ORIGINAL) == ORIGINAL_SHA256, "original scalar measurement source changed")
    require(wheels.digest(Path(wheels.__file__)) == WHEEL_SOURCE_SHA256, "frozen reviewed wheel V2 adapter changed")
    tree = ast.parse(ORIGINAL.read_text(encoding="utf-8"))
    names = {"front_steering_measurement", "speed_limit_measurement", "_settings_dict", "CollectionError"}
    nodes = [copy.deepcopy(node) for node in tree.body
             if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    require(len(nodes) == 4 and {node.name for node in nodes} == names, "exact original scalar functions required")
    namespace = dict(math=math, Any=Any)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(ORIGINAL), "exec"), namespace)
    return {name: namespace[name] for name in names}


def observed_state(world, ego, epoch):
    require(world.id == epoch and ego.id > 0, "owned world/ego epoch changed")
    snapshot = world.get_snapshot()
    actor = snapshot.find(ego.id)
    current = world.get_actor(ego.id)
    require(actor is not None and actor.id == ego.id and current is not None
            and current.id == ego.id and current.type_id == ego.type_id, "owned snapshot actor/type missing or changed")
    transform, velocity, angular = actor.get_transform(), actor.get_velocity(), actor.get_angular_velocity()
    pose = [float(getattr(transform.location, key)) for key in ("x", "y", "z")]
    pose += [float(getattr(transform.rotation, key)) for key in ("roll", "pitch", "yaw")]
    linear = [float(getattr(velocity, key)) for key in ("x", "y", "z")]
    angular_values = [float(getattr(angular, key)) for key in ("x", "y", "z")]
    require(type(snapshot.frame) is int and all(math.isfinite(value) for value in
            (*pose, *linear, *angular_values, snapshot.timestamp.elapsed_seconds)), "finite actual native snapshot required")
    require(world.id == epoch and world.get_snapshot().frame == snapshot.frame, "snapshot changed during native observation")
    return dict(frame=snapshot.frame, world_id=epoch, actor_id=ego.id, type_id=ego.type_id,
                pose=pose, velocity=linear, angular_velocity=angular_values,
                elapsed_simulation_seconds=snapshot.timestamp.elapsed_seconds)


def require_stationary(state, reference, frame):
    require(state["frame"] == frame and all(state[key] == reference[key] for key in
            ("world_id", "actor_id", "type_id", "pose")) and all(value == 0. for value in
            (*state["velocity"], *state["angular_velocity"])), "exact actual stationary pose/velocity/frame/ego required")


def sequential_read(ego, carla, clock=time.perf_counter_ns):
    values, rows = [], []
    for role in wheels.ROLES:
        row = dict(role=role, started_monotonic_ns=clock(), status="RUNNING")
        rows.append(row)
        try:
            values.append(float(ego.get_wheel_steer_angle(getattr(carla.VehicleWheelLocation, role + "_Wheel"))))
            row.update(status="PASS", raw_angle_degrees=values[-1])
        except BaseException as error:
            row.update(status="FAIL", error=type(error).__name__ + ": " + str(error))
            error.wheel_receipts, error.raw_wheel_prefix = rows, list(values)
            raise
        finally:
            row["finished_monotonic_ns"] = clock()
    return tuple(values), rows


def declared_ledger(first_frame=None):
    return [dict(frame_ordinal=index, expected_frame=None if first_frame is None else first_frame + index,
                 actual_frame=None, variant=variant, position=position, status="UNEXECUTED", error=None)
            for index in range(FRAMES) for position, variant in enumerate("AB" if index % 2 == 0 else "BA")]


def scalar_receipt(value):
    # HH_261005 - Preserve invalid physical values as explicit diagnostic tags, never JSON NaN or fabricated measurements.
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_nonfinite_float": repr(value), "physical_measurement_valid": False}
    if isinstance(value, dict): return {key: scalar_receipt(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [scalar_receipt(item) for item in value]
    return value


def empty_capture_report(expected):
    return dict(schema=SCHEMA + ".capture", status="FAIL_RETAINED", declaration_sha256=expected,
                original_settings=None, cleanup_errors=[], cleanup_attempted=False,
                cleanup_verified=False, CARLA_connection_attempted=False,
                actorphysics_disabled=True, equivalent_to_moving_full209=False,
                measurements=dict(status="FAIL_UNEXECUTED_SETUP", all_declared_variant_attempts=declared_ledger(),
                    expected_native_frames=30, expected_variant_calls=60, expected_wheel_RPCs=120, actual_native_states=[]))


def measure_frames(first, *, observe, advance, methods, speed_limit, conversions, budget=lambda: None,
                   persist=lambda kind, value: None, clock=time.perf_counter_ns):
    """HH_261005 - Keep a fixed sixty-entry attempt ledger; abort once and explicitly retain every unexecuted tail."""
    require(set(methods) == {"A", "B"}, "exact sequential/parallel variants required")
    ledger = declared_ledger(first["frame"])
    states, failure = [], None
    try:
        for index in range(FRAMES):
            budget()
            if index:
                require(advance() == first["frame"] + index, "native tick skipped or changed")
            state = observe()
            require_stationary(state, first, first["frame"] + index)
            states.append(state)
            persist("native", state)
            for row in ledger[index*2:index*2+2]:
                row["actual_frame"] = state["frame"]
                row["status"] = "RUNNING"
                row["wheel_read_attempted"] = False
                try:
                    budget()
                    row["before"] = observe()
                    require_stationary(row["before"], first, state["frame"])
                    row["wheel_read_attempted"] = True
                    row["started_monotonic_ns"] = clock()
                    try:
                        raw, read_receipts = methods[row["variant"]](state["frame"])
                        row["raw_front_left_right_degrees"] = list(raw)
                        row["read_receipts"] = read_receipts
                    finally:
                        row["finished_monotonic_ns"] = clock()
                        row["elapsed_ns"] = row["finished_monotonic_ns"] - row["started_monotonic_ns"]
                    row["after"] = observe()
                    require_stationary(row["after"], first, state["frame"])
                    row["cached_speed_limit_raw_kmh"] = float(speed_limit())
                    row["measured_fields"] = {**conversions["front_steering_measurement"](*raw),
                        **conversions["speed_limit_measurement"](row["cached_speed_limit_raw_kmh"])}
                    row["status"] = "PASS"
                except BaseException as error:
                    row.update(status="FAIL", error=type(error).__name__ + ": " + str(error))
                    if hasattr(error, "wheel_receipts"): row["read_receipts"] = error.wheel_receipts
                    if hasattr(error, "raw_wheel_prefix"): row["raw_front_left_right_prefix_degrees"] = error.raw_wheel_prefix
                    raise
                finally:
                    persist("variant", scalar_receipt(row))
            pair = ledger[index*2:index*2+2]
            parity = pair[0]["raw_front_left_right_degrees"] == pair[1]["raw_front_left_right_degrees"] and pair[0]["measured_fields"] == pair[1]["measured_fields"]
            for row in pair: row["same_frame_parity_passed"] = parity
            require(parity, "same-frame raw/converted physical measurements differ")
    except BaseException as error:
        failure = type(error).__name__ + ": " + str(error)
        running = next((row for row in ledger if row["status"] == "RUNNING"), None)
        if running is not None:
            running.update(status="FAIL", error=failure)
    return scalar_receipt(dict(schema=SCHEMA + ".measurements", status="PASS_STATIONARY_RPC_PARITY_ONLY" if failure is None else "FAIL_RETAINED",
                error=failure, expected_native_frames=30, expected_variant_calls=60, expected_wheel_RPCs=120,
                actual_native_states=states, all_declared_variant_attempts=ledger,
                first_calls_and_failed_or_unexecuted_tails_retained=True, retries=0,
                actual_driving_or_latency_gain_proven=False, main_functions_approved=0, subfunctions_approved=0))


def require_capture_identity(owner, plan, expected, launch_sha, receipt):
    capture = wheels.process_record(os.getpid())
    require(Path(plan["output"]) == owner.OUTPUT and capture and capture["ppid"] == receipt["owner"]["pid"]
            and os.getppid() == receipt["owner"]["pid"] and capture["pgid"] == capture["pid"]
            and os.getsid(capture["pid"]) == capture["pid"]
            and capture["argv"] == owner.substituted(plan["worker_command"], expected, launch_sha),
            "capture must be the exact declared owner child/argv/session")
    return capture


def _active_actor_counts(world):
    actors = world.get_actors()
    return {key: len(list(actors.filter(pattern))) for key, pattern in
            (("vehicles", "vehicle.*"), ("walkers", "walker.*"), ("sensors", "sensor.*"))}


def _record_Town03_xodr(map_object, document):
    # HH_261005 - Read geometry only after its exact known Town03 identifier; retain mismatches before denying any actor creation.
    raw = map_object.to_opendrive()
    require(type(raw) is str, "authentic Town03 OpenDrive must be a UTF8 string")
    encoded = raw.encode("utf-8")
    document.update(OpenDrive_size_bytes=len(encoded), OpenDrive_sha256=hashlib.sha256(encoded).hexdigest())


def _require_Town03_map(owner, world, report, key):
    map_object = world.get_map()
    name = map_object.name
    document = dict(world_id=world.id, map_name=name, map_name_repr=repr(name))
    report[key] = document
    report["actual_map_identity"], report["actual_map_identity_repr"] = name, repr(name)
    require(type(name) is str and name in owner.MAP_IDENTIFIERS,
            "fresh owned Town03 map required; reload forbidden after initialization; observed=" + repr(name))
    _record_Town03_xodr(map_object, document)
    require(document["OpenDrive_size_bytes"] == owner.TOWN03_XODR_SIZE
            and document["OpenDrive_sha256"] == owner.TOWN03_XODR_SHA256,
            "actual Town03 full OpenDrive size/SHA differs; no repair reload permitted")
    return document


def _initialize_owned_map(client, world, owner, plan, expected, launch_sha, report):
    # HH_261005 - Other initial maps expose only world/name/count metadata, never reserved geometry, states, images, labels or inference.
    owner.verify_owner_receipt(plan, expected)
    name = world.get_map().name
    initial = dict(world_id=world.id, map_name=name, map_name_repr=repr(name), active_actor_counts=_active_actor_counts(world))
    report["map_initialization"] = dict(initial_world=initial, load_world_calls=0,
        no_initial_non_Town03_geometry_state_images_labels_read=True, completed=False,
        initialization_RPC_timeout_seconds=plan["initialization_RPC_timeout_seconds"],
        normal_stationary_RPC_timeout_seconds=plan["normal_stationary_RPC_timeout_seconds"],
        normal_stationary_RPC_timeout_restored=None)
    report["actual_map_identity"], report["actual_map_identity_repr"] = name, repr(name)
    require(all(count == 0 for count in initial["active_actor_counts"].values()),
            "fresh owned startup has active actors; initialization and capture forbidden")
    if type(name) is str and name in owner.MAP_IDENTIFIERS:
        _require_Town03_map(owner, world, report, "actual_Town03_map")
    else:
        require(plan["initialize_owned_map_if_needed"] == "Town03"
                and type(plan["maximum_load_world_calls"]) is int and plan["maximum_load_world_calls"] == 1
                and plan["before_any_actor_or_worker"] is True
                and plan["worker_definition"] == "wheel_read_worker; capture_setup_process_exists"
                and type(plan["initialization_RPC_timeout_seconds"]) is float and plan["initialization_RPC_timeout_seconds"] == 30.
                and type(plan["normal_stationary_RPC_timeout_seconds"]) is float and plan["normal_stationary_RPC_timeout_seconds"] == .5,
                "single declared pre-actor initialization required")
        owner.verify_launch_authority(plan, expected, launch_sha)
        owner.verify_owner_receipt(plan, expected)
        require(client.get_world().id == world.id == initial["world_id"] and all(count == 0 for count in _active_actor_counts(world).values()),
                "original fresh owned world epoch/inventory changed before initialization")
        # HH_261005 - Only the one pre-actor/pre-wheel-worker map RPC receives thirty seconds; ordinary and wheel deadlines are not widened.
        report["map_initialization"]["normal_stationary_RPC_timeout_restored"] = False
        try:
            client.set_timeout(plan["initialization_RPC_timeout_seconds"])
            report["map_initialization"]["load_world_calls"] = 1
            loaded = client.load_world("Town03")
        finally:
            client.set_timeout(plan["normal_stationary_RPC_timeout_seconds"])
            report["map_initialization"]["normal_stationary_RPC_timeout_restored"] = True
        require(loaded is not None and loaded.id != initial["world_id"], "new initialized owned world epoch required")
        world = loaded
        _require_Town03_map(owner, world, report, "actual_Town03_map")
        counts = _active_actor_counts(world)
        report["map_initialization"]["initialized_active_actor_counts"] = counts
        require(all(count == 0 for count in counts.values()), "initialized owned world has active actors")
        owner.verify_owner_receipt(plan, expected)
        require(client.get_world().id == world.id, "initialized owned world epoch changed before capture")
    report["map_initialization"]["completed"] = True
    return world


def _run_capture(owner, plan, expected, launch_authority_sha256, report):
    # HH_261005 - A genuine owner child and separate ROOT launch approval are prerequisites even before the first CARLA import.
    require(plan == owner.verify_plan(expected), "exact fresh capture declaration required")
    require(all(plan.get(key) == value for key, value in fixed_contract().items()), "fixed stationary scope changed")
    owner.verify_launch_authority(plan, expected, launch_authority_sha256)
    receipt = owner.verify_owner_receipt(plan, expected)
    capture = require_capture_identity(owner, plan, expected, launch_authority_sha256, receipt)
    report["capture"] = capture
    owner.verify_archived_sources(plan, expected, launch_authority_sha256)
    require(owner.EGG == wheels.CARLA_EGG and owner.EGG_SHA256 == wheels.CARLA_EGG_SHA256
            and plan["asset_sha256"].get(str(owner.EGG)) == owner.EGG_SHA256 and owner.digest(owner.EGG) == owner.EGG_SHA256,
            "exact reviewed packaged CARLA egg required")
    conversions = original_functions()
    output = Path(plan["output"])
    owner.check_budget(output)
    require(os.environ.get("PYTHON_EGG_CACHE") == plan["native_runtime_input"]["cache_path"],
            "exact verified private cache environment required; implicit extraction forbidden")
    report["native_runtime_input_before"] = owner.verify_native_runtime_input(plan)
    sys.path.insert(0, str(owner.EGG))
    import carla
    require(str(carla.__file__).startswith(str(owner.EGG)+"/"), "unsealed CARLA module origin")
    native = sys.modules.get("carla.libcarla")
    require(native is not None and str(native.__file__) == plan["native_runtime_input"]["native_path"],
            "actual native module must originate in the sealed full private input")
    report["actual_native_module_path"] = str(native.__file__)
    owner.verify_owner_receipt(plan, expected)
    report["CARLA_connection_attempted"] = True
    client = carla.Client("127.0.0.1", plan["port_triplet"][0]); client.set_timeout(plan["normal_stationary_RPC_timeout_seconds"])
    world = client.get_world()
    world = _initialize_owned_map(client, world, owner, plan, expected, launch_authority_sha256, report)
    require(not list(world.get_actors().filter("vehicle.*")) and not list(world.get_actors().filter("walker.*"))
            and not list(world.get_actors().filter("sensor.*")), "exclusive owned world has foreign active actors")
    epoch, ego, reader = world.id, None, None
    original_settings = world.get_settings()
    original_document = conversions["_settings_dict"](original_settings)
    report["original_settings"] = original_document
    events = output / "capture_events"
    events.mkdir(exist_ok=False)
    reader_receipt = None
    old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    def interrupted(sig, frame): raise RuntimeError("owned capture interrupted by signal " + str(sig))
    for sig in old_handlers: signal.signal(sig, interrupted)
    try:
        owner.verify_owner_receipt(plan, expected)
        settings = world.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = .05
        world.apply_settings(settings)
        actual_settings = conversions["_settings_dict"](world.get_settings())
        require(actual_settings == {**original_document, "synchronous_mode": True, "fixed_delta_seconds": .05}, "original physics/render/substep settings changed")
        spawn = world.get_map().get_spawn_points()[0]; spawn.location.z += .5
        report["spawn_transform"] = dict(location=[spawn.location.x, spawn.location.y, spawn.location.z],
            rotation=[spawn.rotation.roll, spawn.rotation.pitch, spawn.rotation.yaw])
        ego = world.spawn_actor(world.get_blueprint_library().find("vehicle.toyota.prius"), spawn)
        require(ego is not None and ego.type_id == "vehicle.toyota.prius", "exact single owned vehicle spawn required")
        # HH_261005 - This is one genuine owned-actor setup command, not a measured zero or a synchronous ACK; the original first native snapshot decides success.
        setup = report["angular_velocity_setup"] = dict(actor_id=ego.id, type_id=ego.type_id, world_id=epoch,
            API="carla.Actor.set_target_angular_velocity", requested_degrees_per_second=[0., 0., 0.],
            command_before_physics_disable=True, native_ticks_before_command=0,
            pre_command_snapshot_unavailable=True, synchronous_server_ACK_claimed=False,
            status="ATTEMPTED", started_monotonic_ns=time.perf_counter_ns())
        try:
            returned = ego.set_target_angular_velocity(carla.Vector3D(0., 0., 0.))
            setup["native_return_repr"] = repr(returned)
            require(returned is None, "original angular setter void API ABI changed")
            setup["status"] = "RETURNED_WITHOUT_SYNCHRONOUS_ACK"
        except BaseException as error:
            setup.update(status="FAIL_RETAINED", error=type(error).__name__ + ": " + str(error))
            raise
        finally:
            setup["finished_monotonic_ns"] = time.perf_counter_ns()
        ego.set_simulate_physics(False)
        # HH_261005 - One declared admission tick admits the freshly spawned actor into an actual snapshot; no wheel warmup is performed.
        admitted_frame = world.tick()
        first = observed_state(world, ego, epoch)
        # HH_261005 - Preserve the real admission observation even when its original exact frame/zero-velocity guard rejects setup.
        report.update(owned_actor_id=ego.id, world_id=epoch, admission_snapshot=copy.deepcopy(first), original_wheel_warmup_calls=0)
        require(first["frame"] == admitted_frame, "admission snapshot/frame mismatch")
        require_stationary(first, first, first["frame"])
        owner.write_new(output / "capture_ready.json", dict(schema=SCHEMA + ".capture_ready", declaration_sha256=expected,
            capture=capture, world_id=epoch, actor_id=ego.id, type_id=ego.type_id, first_frame=first["frame"], actual_snapshot=first,
            actual_Town03_map=copy.deepcopy(report["actual_Town03_map"]),
            map_initialization=copy.deepcopy(report["map_initialization"])))
        deadline = time.monotonic() + 120
        while not wheels.AUTHORITY.exists():
            require(time.monotonic() < deadline, "ROOT endpoint authority readiness deadline")
            owner.check_budget(output); owner.verify_owner_receipt(plan, expected); time.sleep(.05)
        authority_sha = wheels.digest(wheels.AUTHORITY)
        proof = dict(capture=capture, **{key: receipt[key] for key in ("owner", "server", "socket_owner")},
            host="127.0.0.1", port=plan["port_triplet"][0], actor_id=ego.id, type_id=ego.type_id,
            world_id=epoch, first_frame=first["frame"], synthetic_only=False,
            declaration_path=str(owner.PLAN), declaration_sha256=expected,
            owner_receipt_path=str(output / "owner_started.json"), owner_receipt_sha256=owner.digest(output / "owner_started.json"),
            authority_path=str(wheels.AUTHORITY), authority_sha256=authority_sha,
            carla_egg_path=str(owner.EGG), carla_egg_sha256=owner.EGG_SHA256)
        reader = wheels.OwnedWheelReader(proof, synthetic=False)
        def observe(): return observed_state(world, ego, epoch)
        def identity():
            state = observe(); return state["frame"], state["world_id"], state["actor_id"], state["type_id"]
        def parallel(frame):
            try:
                values = reader.read(frame, identity)
                return values, json.loads(json.dumps(reader.rows[-1], allow_nan=False))
            except BaseException as error:
                error.wheel_receipts = json.loads(json.dumps(reader.rows[-1], allow_nan=False))
                raise
        def persist(kind, value):
            number = len(report.setdefault("events", []))
            report["events"].append(dict(kind=kind, value=copy.deepcopy(value)))
            owner.check_budget(output)
            owner.write_new(events / (str(number).zfill(4) + "_" + kind + ".json"), value)
        report["measurements"] = measure_frames(first, observe=observe, advance=world.tick,
            methods=dict(A=lambda frame: sequential_read(ego, carla), B=parallel), speed_limit=ego.get_speed_limit,
            conversions=conversions, budget=lambda: owner.check_budget(output), persist=persist)
        report["status"] = report["measurements"]["status"]
    except BaseException as error:
        report["error"] = type(error).__name__ + ": " + str(error)
        if isinstance(error, wheels.WheelReadError): reader_receipt = error.receipt
    finally:
        report["cleanup_attempted"] = True
        if reader is not None:
            try: reader.close()
            except BaseException as error: report["cleanup_errors"].append(type(error).__name__ + ": " + str(error))
            reader_receipt = reader.receipt()
        report["wheel_worker_receipt"] = reader_receipt
        joined = reader_receipt is None or reader_receipt["all_owned_children_joined"]
        if reader_receipt is not None and reader_receipt["cleanup_errors"]: report["cleanup_errors"].append("wheel worker cleanup errors retained")
        if not joined: report["cleanup_errors"].append("unjoined workers: no actor destruction/settings mutation; owner must stop its own server")
        if joined:
            try:
                owner.verify_owner_receipt(plan, expected)
                require(client.get_world().id == world.id == epoch, "cleanup world epoch changed")
                if ego is not None:
                    actor = world.get_actor(ego.id)
                    require(actor is not None and actor.type_id == "vehicle.toyota.prius" and actor.id == ego.id, "cleanup owned actor identity changed")
                    require(ego.destroy() is True, "owned actor destruction failed")
                    report["owned_actor_destroyed"] = True
                world.apply_settings(original_settings)
                report["restored_settings"] = conversions["_settings_dict"](world.get_settings())
                require(report["restored_settings"] == original_document, "original world settings restoration failed")
                report["cleanup_verified"] = True
            except BaseException as error: report["cleanup_errors"].append(type(error).__name__ + ": " + str(error))
        for sig, handler in old_handlers.items(): signal.signal(sig, handler)
        if report["cleanup_errors"] or report.get("error"): report["status"] = "FAIL_RETAINED"
    return report


def run_capture(plan, expected, *, launch_authority_sha256=None):
    # HH_261005 - Retain the fixed denominator even for pre-import/setup failures; only an authenticated owner child may write its failure receipt.
    from . import run_owned_town03_stationary_wheel_rpc_v6_20261005 as owner
    report = empty_capture_report(expected)
    try:
        _run_capture(owner, plan, expected, launch_authority_sha256, report)
    except BaseException as error:
        report.update(status="FAIL_RETAINED", error=type(error).__name__ + ": " + str(error))
        if isinstance(error, wheels.WheelReadError): report["wheel_worker_receipt"] = error.receipt
    receipt = owner.verify_owner_receipt(plan, expected)
    require_capture_identity(owner, plan, expected, launch_authority_sha256, receipt)
    try:
        owner.verify_archived_sources(plan, expected, launch_authority_sha256)
        report["native_runtime_input_after"] = owner.verify_native_runtime_input(plan)
    except BaseException as error:
        report["cleanup_errors"].append("final source seals: " + type(error).__name__ + ": " + str(error))
        report["status"] = "FAIL_RETAINED"
    report = scalar_receipt(report)
    owner.write_new(owner.OUTPUT / "capture_result.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--launch-authority-sha256", required=True)
    args = parser.parse_args(argv)
    from . import run_owned_town03_stationary_wheel_rpc_v6_20261005 as owner
    report = run_capture(owner.verify_plan(args.plan_sha256), args.plan_sha256,
                         launch_authority_sha256=args.launch_authority_sha256)
    print(json.dumps(dict(status=report["status"], actual_driving_gain_proven=False)))
    return 0 if report["status"] == "PASS_STATIONARY_RPC_PARITY_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
