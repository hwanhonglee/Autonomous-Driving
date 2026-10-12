"""HH_261001 - Own one fixed exploratory W1 brake-only attempt; no automatic launch or retry."""

import argparse
from contextlib import contextmanager
from io import BytesIO
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np
import torch

from scripts.e2e import town07_moving_brake_ab_contract_v2_20261001 as protocol
from scripts.e2e import town07_moving_brake_ab_control_v2_20261001 as control_ab
from scripts.e2e import research_warmup_duration_w2_replay_20261001 as prior
from scripts.e2e import research_warmup_supervision_candidate_20261001 as warmup
from scripts.e2e import capture_exact_simulation_inputs_20261001 as exact

research, short, w1 = prior.research, prior.short, prior.w1
require, digest, read_json, write_new = protocol.require, protocol.digest, protocol.read_json, protocol.write_new
MODULE = "scripts.e2e.research_town07_w1_moving_brake_ab_v2_20261001"


def capture_argv(index):
    return [str(protocol.output(index) / "episode"), str(protocol.ROUTE),
        "--host", "127.0.0.1", "--port", "2440", "--control-transport", "acknowledged_batch",
        "--stationary-warmup-sec", "1.2", "--max-duration-sec", "20", "--target-speed-kmh", "9",
        "--seed", "0", "--weather", "ClearNoon", "--mapping", str(protocol.MAPPING),
        "--calibration", str(protocol.CALIBRATION), "--spawn-z-offset-m", "0.5",
        "--expected-live-xodr-sha256", protocol.assets()[str(protocol.XODR)],
        "--goal-stop-profile", "disabled"]


def declaration(expected):
    value = read_json(protocol.PLAN)
    protocol.verify_plan(value, expected_sha256=expected)
    return value


def validate_capture(args, route, index):
    require(args.host == "127.0.0.1" and args.port == 2440 and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.physics_hz == 20 and args.capture_hz == 10 and args.stationary_warmup_sec == 1.2
        and args.max_duration_sec == 20 and args.research_speed_cap_mps == 2.5
        and args.target_speed_kmh == 9 and args.research_max_cte_m == 1.
        and args.weather == "ClearNoon" and args.seed == 0 and args.spawn_z_offset_m == .5
        and route.get("town") == "Town07" and route.get("scenario") == "straight"
        and args.route_file.resolve() == protocol.ROUTE and digest(args.route_file) == protocol.assets()[str(protocol.ROUTE)]
        and args.output.resolve() == protocol.output(index) / "episode"
        and args.mapping.resolve() == protocol.MAPPING and args.calibration.resolve() == protocol.CALIBRATION,
        "fixed W1 exploratory twenty-second capture contract differs")


def load_w1():
    require(w1.CHECKPOINT == protocol.CHECKPOINT and w1.WEIGHTS_SHA == protocol.WEIGHTS_SHA
            and w1.METADATA_SHA == protocol.METADATA_SHA, "existing W1 helper identity differs")
    with warmup.identity(protocol.CHECKPOINT, short):
        model = research.load_ah_weights(protocol.WEIGHTS, protocol.METADATA,
            weights_sha256=protocol.WEIGHTS_SHA, metadata_sha256=protocol.METADATA_SHA)
        require(prior.tensor_digest(model.state_dict()) == protocol.MODEL_STATE,
                "actual W1 export tensor identity differs")
        return model


def directory_bytes(directory):
    total = 0
    for path in Path(directory).iterdir():
        require(path.is_file() and not path.is_symlink(), "regular sampled input archive files required")
        total += path.stat().st_size
    return total


@contextmanager
def bounded_sampled_inputs(run):
    """Keep the existing every-tenth policy-call sampler with a pre-write 128 MiB allowance."""
    directory = Path(run) / "sampled_exact_runtime_inputs"
    original_save = exact.save_input_snapshot

    def bounded_save(folder, sequence, tensors, timing, decision, capture_seconds):
        require(Path(folder) == directory, "fixed sample directory required")
        encoded = BytesIO()
        np.savez(encoded, **tensors)
        encoded_bytes = encoded.tell()
        # HH_261001 - Reserve the complete journal row before writing the original data-only NPZ.
        used = directory_bytes(directory)
        if used + encoded_bytes + 4096 > protocol.SAMPLE_LIMIT:
            write_new(Path(run) / "sample_allowance_abort.json", dict(
                schema="portable_e2e.moving_brake_sample_allowance_abort.v1",
                sequence=sequence, retained_sample_bytes=used, proposed_npz_bytes=encoded_bytes,
                allowance_bytes=protocol.SAMPLE_LIMIT, timing=timing, rejected_decision=decision,
                control_sent=False, training_data_approved=False))
            raise short.collector.CollectionError("fixed exact-input sample allowance reached")
        row = original_save(folder, sequence, tensors, timing, decision, capture_seconds)
        require((directory / row["tensor_file"]).stat().st_size == encoded_bytes,
                "original NPZ encoding size differs from prospective sample accounting")
        row["sample_directory_allowance_bytes"] = protocol.SAMPLE_LIMIT
        return row

    exact.save_input_snapshot = bounded_save
    try:
        with w1.sampled_inputs(directory):
            yield
        require(directory_bytes(directory) <= protocol.SAMPLE_LIMIT, "sample directory exceeded fixed allowance")
    finally:
        exact.save_input_snapshot = original_save


@contextmanager
def disk_guard():
    original = short.collector.exact_camera_bundle

    def guarded(*args, **kwargs):
        if shutil.disk_usage(protocol.ROOT).free < protocol.FLOOR + protocol.CLOSING_RESERVE:
            raise short.collector.CollectionError("fixed moving-brake disk reserve floor reached")
        return original(*args, **kwargs)

    short.collector.exact_camera_bundle = guarded
    try:
        yield
    finally:
        short.collector.exact_camera_bundle = original


@contextmanager
def native_guards(run):
    """Apply equal native storage/speed guards; preserve original emergency commands and ACKs."""
    original = short.LiveResearchSession.propose
    capture_reasons = {"initial_drive_control", "next_driving_control",
                       "stationary_warmup_start", "next_stationary_warmup_control",
                       "next_stationary_tail_control"}
    journal = (Path(run) / "native_guard.jsonl").open("x")

    def propose(session, transport, teacher, reason):
        # HH_261001 - Setup and exception cleanup retain their original full-brake authority.
        if reason not in capture_reasons:
            return original(session, transport, teacher, reason)
        state = session.states[-1] if session.states else None
        free = shutil.disk_usage(protocol.ROOT).free
        world_velocity = state.get("world_velocity_carla") if state else None
        speed = math.hypot(float(world_velocity[0]), float(world_velocity[1])) if world_velocity else None
        exceeded = speed is not None and (not math.isfinite(speed) or speed > 2.7)
        row = dict(command_reason=reason, source_frame=state["frame"] if state else None,
            measured_actor_planar_speed_mps=speed, maximum_actor_planar_speed_mps=2.7,
            free_disk_bytes=free, required_running_free_bytes=protocol.FLOOR + protocol.CLOSING_RESERVE,
            speed_guard_exceeded=exceeded,
            speed_guard_source="immutable native actor snapshot world XY velocity norm")
        if exceeded or free < protocol.FLOOR + protocol.CLOSING_RESERVE:
            row["rejected_native_observation"] = state
        journal.write(json.dumps(research._json_finite(row), sort_keys=True, allow_nan=False) + "\n")
        journal.flush()
        if free < protocol.FLOOR + protocol.CLOSING_RESERVE:
            raise short.collector.CollectionError("fixed native-control disk reserve floor reached")
        if exceeded:
            raise short.collector.CollectionError("fixed measured actor planar speed guard exceeded 2.7 m/s")
        chosen, decision = original(session, transport, teacher, reason)
        if decision is not None:
            decision = dict(decision)
            decision["native_safety_guard"] = row
        return chosen, decision

    short.LiveResearchSession.propose = propose
    try:
        yield
    finally:
        short.LiveResearchSession.propose = original
        journal.close()


def prior_attempt_ready(expected):
    folder = protocol.output(0)
    result = read_json(folder / "owner_diagnostic_result.json")
    plan = read_json(folder / "owner_plan.json")
    require(result.get("port_triplet_released") is True and result.get("source_hashes_match") is True
        and result.get("external_source_hashes_match") is True
        and result.get("archived_declaration_sha256") == expected
        and not result.get("owner_cleanup_errors") and plan.get("schema") == protocol.SCHEMA
        and plan.get("declaration_sha256") == expected and plan.get("trial") == protocol.trial(0)
        and result.get("trial") == protocol.trial(0)
        and plan.get("source_checkpoint_sha256") == result.get("source_checkpoint_sha256") == protocol.CHECKPOINT,
        "first fixed exploratory attempt must retain safe ownership/source cleanup; motion outcome is not selected")
    for name, pinned in plan["source_sha256"].items():
        require(digest(protocol.ROOT / name) == pinned == digest(folder / "source" / name),
                "first attempt current/archive source changed")
    require(digest(folder / "declared_plan.json") == expected, "first archived declaration differs")
    declared = read_json(folder / "declared_plan.json")
    require(external_source_match(folder, declared), "first external current/archive source changed")
    return result


def worker(args, remaining):
    declaration(args.plan_sha256)
    spec = protocol.trial(args.trial)
    run = protocol.output(args.trial)
    require(remaining == capture_argv(args.trial), "worker accepts only the complete fixed capture argument list")
    capture = short.collector.parse_args(remaining)
    capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
    validate_capture(capture, read_json(capture.route_file), args.trial)
    protocol.disk_check(2 - args.trial)
    require(run.is_dir() and not run.is_symlink(), "fresh owner-created worker directory required")
    write_new(run / "worker_endpoint_identity.json", dict(trial=spec,
        source_checkpoint_sha256=protocol.CHECKPOINT, model_state_sha256=protocol.MODEL_STATE,
        declaration_sha256=args.plan_sha256, optimizer_updates=0,
        exploratory=True, ABBA=False, vehicle_control_approved=False))
    torch.set_num_threads(2)
    model = load_w1()
    try:
        with warmup.identity(protocol.CHECKPOINT, short), control_ab.tracker_scope(research, spec["arm"]), \
                disk_guard(), native_guards(run), bounded_sampled_inputs(run):
            short.run(model, capture)
        return 0
    finally:
        declaration(args.plan_sha256)


def archive_sources(run, value, declared, expected):
    """Archive all declared local/agent source bytes before the original owner starts its server."""
    for name, pinned in declared["source_sha256"].items():
        source = protocol.ROOT / name
        target = run / "source" / name
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as read, target.open("xb") as write:
                shutil.copyfileobj(read, write)
        require(digest(source) == pinned == digest(target), "declared local source archive differs: " + name)
        value["source_sha256"][name] = pinned
    for name, pinned in declared["external_agent_sha256"].items():
        source = Path(name)
        target = run / "source_external" / source.relative_to(protocol.CARLA_ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as read, target.open("xb") as write:
            shutil.copyfileobj(read, write)
        require(digest(source) == pinned == digest(target), "declared external source archive differs")
    with protocol.PLAN.open("rb") as read, (run / "declared_plan.json").open("xb") as write:
        shutil.copyfileobj(read, write)
    require(digest(run / "declared_plan.json") == expected, "archived declaration bytes differ")


def external_source_match(run, declared):
    return all(digest(name) == pinned == digest(run / "source_external" / Path(name).relative_to(protocol.CARLA_ROOT))
               for name, pinned in declared["external_agent_sha256"].items())


def pair_receipt(expected):
    declaration(expected)
    results = {}
    for index in (0, 1):
        run = protocol.output(index)
        plan, result = read_json(run / "owner_plan.json"), read_json(run / "owner_diagnostic_result.json")
        require(plan.get("schema") == protocol.SCHEMA and plan.get("trial") == protocol.trial(index)
            and result.get("trial") == protocol.trial(index)
            and plan.get("declaration_sha256") == result.get("declaration_sha256") == expected
            and plan.get("source_checkpoint_sha256") == result.get("source_checkpoint_sha256") == protocol.CHECKPOINT,
            "both actual fixed exploratory attempt identities required")
        results[protocol.trial(index)["trial_id"]] = dict(status=result["status"],
            state_count=result["state_count"], camera_count=result["camera_count"],
            cleanup_ok=result["port_triplet_released"] and result["source_hashes_match"]
                and result["external_source_hashes_match"] and not result["owner_cleanup_errors"],
            owner_plan_sha256=digest(run / "owner_plan.json"),
            owner_diagnostic_result_sha256=digest(run / "owner_diagnostic_result.json"))
    value = dict(schema=protocol.SCHEMA + ".attempts", status="TWO_EXPLORATORY_ATTEMPTS_RETAINED",
        declaration_sha256=expected, checkpoint_sha256=protocol.CHECKPOINT, trials=results,
        trial_count=2, attempts_per_arm=1, exploratory=True, ABBA=False,
        two_full_twenty_second_captures=all(row["state_count"] == 400 and row["camera_count"] == 200
                                          for row in results.values()),
        all_ownership_and_sources_clean=all(row["cleanup_ok"] for row in results.values()),
        model_optimizer_updates=0, learned_driving_success_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    write_new(protocol.output(1) / "paired_attempt_receipt.json", value)
    return value


def trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcome
    declared = declaration(args.plan_sha256)
    spec, run = protocol.trial(args.trial), protocol.output(args.trial)
    reserve = protocol.disk_check(2 - args.trial)
    if args.trial == 1:
        prior_attempt_ready(args.plan_sha256)
    require(not run.exists() and not run.is_symlink(), "fresh fixed attempt output required; retries forbidden")
    saved = (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
             owner.write_json, owner.lifecycle._workspace_lock)
    old_weights, old_metadata = owner.WEIGHTS, owner.METADATA
    old_command, old_write, old_lock = owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock
    owner.OUTPUT, owner.WEIGHTS, owner.METADATA = run, protocol.WEIGHTS, protocol.METADATA
    owner.PINS = {p: sha for p, sha in owner.PINS.items() if p not in (old_weights, old_metadata)}
    owner.PINS.update({Path(name): sha for name, sha in dict(
        declared["asset_sha256"], **declared["external_agent_sha256"]).items()})
    owner.PINS.update({protocol.ROOT / name: sha for name, sha in declared["source_sha256"].items()})
    owner.PINS[protocol.PLAN] = args.plan_sha256

    def command():
        argv = old_command()
        module = argv.index("scripts.e2e.collect_ah_simulation_research")
        rest = argv[argv.index(str(run / "episode")):]
        require(rest == capture_argv(args.trial), "inherited owner capture arguments changed")
        return argv[:module] + [MODULE, "--", "--mode", "worker", "--trial", str(args.trial),
                               "--plan-sha256", args.plan_sha256] + rest

    def archive_write(path, value):
        if path == run / "owner_plan.json":
            archive_sources(run, value, declared, args.plan_sha256)
        old_write(path, value)

    @contextmanager
    def reserved_lock():
        with old_lock():
            protocol.disk_check(2 - args.trial)
            declaration(args.plan_sha256)
            if args.trial == 1:
                prior_attempt_ready(args.plan_sha256)
            yield

    owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock = command, archive_write, reserved_lock
    try:
        torch.set_num_threads(2)
        load_w1()
        with warmup.identity(protocol.CHECKPOINT, short):
            plan = owner.preflight()
            argv = command()
            capture = short.collector.parse_args(argv[argv.index(str(run / "episode")):])
            capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
            validate_capture(capture, read_json(protocol.ROUTE), args.trial)
            plan.update(schema=protocol.SCHEMA, trial=spec,
                source_checkpoint_sha256=protocol.CHECKPOINT, source_checkpoint_schema=protocol.CHECKPOINT_SCHEMA,
                model_state_sha256=protocol.MODEL_STATE, declaration_sha256=args.plan_sha256,
                disk_reservation=reserve, exact_input_sample_allowance_bytes=protocol.SAMPLE_LIMIT,
                fixed_trial_order=list(protocol.trial_specs()), exploratory=True, ABBA=False,
                attempts_per_arm=1, model_optimizer_updates=0, original_emergency_authority_unchanged=True,
                this_single_attempt_is_not_a_pair=True, all_exact_inference_inputs_archived=False)
            if args.mode == "preflight":
                return plan
            owner.execute(plan)
            result = outcome.final_report(run)
            result.update(trial=spec, source_checkpoint_sha256=protocol.CHECKPOINT,
                declaration_sha256=args.plan_sha256, exploratory=True, ABBA=False, attempts_per_arm=1,
                model_optimizer_updates=0, this_single_attempt_is_not_a_pair=True,
                external_source_hashes_match=external_source_match(run, declared),
                archived_declaration_sha256=digest(run / "declared_plan.json"),
                all_exact_inference_inputs_archived=False)
            if not result["external_source_hashes_match"]:
                result["status"] = "OWNER_FAILURE"
            owner.write_json(run / "owner_diagnostic_result.json", result)
            if args.trial == 1:
                pair_receipt(args.plan_sha256)
            return result
    finally:
        (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
         owner.write_json, owner.lifecycle._workspace_lock) = saved


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", required=True, choices=("contract", "declare", "preflight", "execute", "worker"))
    parser.add_argument("--trial", type=int)
    parser.add_argument("--plan-sha256")
    args, remaining = parser.parse_known_args(argv)
    if args.mode == "worker":
        require(args.trial is not None and args.plan_sha256, "worker requires fixed trial and declaration SHA")
        return worker(args, remaining)
    require(not remaining, "unexpected runtime overrides forbidden")
    if args.mode in ("contract", "declare"):
        require(args.trial is None and args.plan_sha256 is None, "contract/declaration takes no trial overrides")
        result = protocol.fixed_contract()
        if args.mode == "contract":
            result.update(status="PREPARED_NOT_DECLARED_NOT_EXECUTED",
                current_free_bytes=shutil.disk_usage(protocol.ROOT).free,
                required_initial_pair_bytes=protocol.FLOOR + 2*protocol.ARM_RESERVE + protocol.SHARED_RESERVE,
                required_four_attempt_bytes=protocol.FLOOR + 4*protocol.ARM_RESERVE + protocol.SHARED_RESERVE)
        else:
            protocol.disk_check(2)
            result = protocol.build_plan()
            write_new(protocol.PLAN, result)
            result = dict(status="DECLARED_NO_SIMULATOR", plan=str(protocol.PLAN),
                          plan_sha256=digest(protocol.PLAN), fixed_trial_order=list(protocol.trial_specs()))
    else:
        require(args.trial is not None and args.plan_sha256, "trial requires fixed index and exact declaration SHA")
        result = trial(args)
    print(json.dumps(result, sort_keys=True, indent=2, allow_nan=False))
    return 0 if args.mode != "execute" or result["status"] in (
        "DIAGNOSTIC_BUDGET_REACHED", "DIAGNOSTIC_CAPTURE_COMPLETE") else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError, RuntimeError, short.collector.CollectionError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        raise SystemExit(1)
