"""HH_261001 - Own one predeclared W1 PI cruise repetition; no physical action without execute."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import errno
import hashlib
from io import BytesIO
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import torch

from scripts.e2e import town07_w1_pi_cruise_repetition_contract_v3_20261001 as contract


require, digest, read_json, write_new = contract.require, contract.digest, contract.read_json, contract.write_new
assets = contract.assets_v2
BUDGET_ERROR = ("CollectionError: BasicAgent did not finish within the driving budget reserved by "
                "the maximum total duration and stationary tail")


def declaration(expected):
    return contract.verify_plan(read_json(contract.PLAN), expected)


def wait_for_original_bind_checks(port, probe, *, timeout_s=75., pause=None, clock=None):
    """HH_261001 - Retry only EADDRINUSE; hold no socket and never set SO_REUSEADDR/PORT."""
    pause, clock = pause or time.sleep, clock or time.monotonic
    require(timeout_s == 75., "fixed maximum natural-release wait required")
    started, attempts = clock(), []
    while True:
        try:
            probe(port)
        except OSError as error:
            elapsed = clock() - started
            attempts.append(dict(elapsed_s=elapsed, passed=False, errno=error.errno, error=str(error)))
            if error.errno != errno.EADDRINUSE or elapsed >= timeout_s:
                raise
            pause(min(.2, timeout_s - elapsed))
        else:
            attempts.append(dict(elapsed_s=clock() - started, passed=True))
            return dict(port_triplet=[port, port+1, port+2], maximum_wait_s=timeout_s,
                original_all_tcp_udp_bind_check=True, sockets_forced_or_reused=False, attempts=attempts)


def capture_argv(index):
    return [str(contract.output(index) / "episode"), str(assets.ROUTE),
        "--host", "127.0.0.1", "--port", str(contract.PORT), "--control-transport", "acknowledged_batch",
        "--stationary-warmup-sec", "1.2", "--max-duration-sec", "45", "--target-speed-kmh", "9",
        "--seed", "0", "--weather", "ClearNoon", "--mapping", str(assets.MAPPING),
        "--calibration", str(assets.CALIBRATION), "--spawn-z-offset-m", "0.5",
        "--expected-live-xodr-sha256", assets.assets()[str(assets.XODR)], "--goal-stop-profile", "disabled"]


def worker_command(index, expected):
    return [sys.executable, str(contract.ROOT / "tools/workspace_python.py"), "-m", contract.MODULE, "--",
        "--mode", "worker", "--trial", str(index), "--plan-sha256", expected, *capture_argv(index)]


def validate_capture(args, index):
    require(args.host == "127.0.0.1" and args.port == contract.PORT and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.physics_hz == 20 and args.capture_hz == 10 and args.stationary_warmup_sec == 1.2
        and args.max_duration_sec == 45 and args.research_speed_cap_mps == 2.5
        and args.target_speed_kmh == 9 and args.research_max_cte_m == 1.
        and args.weather == "ClearNoon" and args.seed == 0 and args.spawn_z_offset_m == .5
        and args.route_file.resolve() == assets.ROUTE and digest(args.route_file) == assets.assets()[str(assets.ROUTE)]
        and args.output.resolve() == contract.output(index) / "episode"
        and args.mapping.resolve() == assets.MAPPING and args.calibration.resolve() == assets.CALIBRATION,
        "fixed NEW 45-second W1 PI cruise contract differs")
    route = read_json(args.route_file)
    require(route["town"] == "Town07" and route["scenario"] == "straight", "fixed TRAIN route identity required")


@contextmanager
def w1_identity(research, short):
    """HH_261001 - Scope only export identity, exactly as previous W1 workers; never mutate weights."""
    saved = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA, short.AH_CHECKPOINT_SHA256
    try:
        research.AH_CHECKPOINT_SHA256 = short.AH_CHECKPOINT_SHA256 = assets.CHECKPOINT
        research.CHECKPOINT_SCHEMA = assets.CHECKPOINT_SCHEMA
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA, short.AH_CHECKPOINT_SHA256 = saved


def tensor_digest(values):
    result = hashlib.sha256()
    for name, tensor in sorted(values.items()):
        value = tensor.detach().cpu().contiguous()
        header = json.dumps([name, str(value.dtype), list(value.shape)], sort_keys=True,
                            separators=(",", ":"), allow_nan=False).encode()
        result.update(len(header).to_bytes(8, "big"))
        result.update(header)
        result.update(value.numpy().tobytes())
    return result.hexdigest()


def load_w1(research, short):
    with w1_identity(research, short):
        model = research.load_ah_weights(assets.WEIGHTS, assets.METADATA,
            weights_sha256=assets.WEIGHTS_SHA, metadata_sha256=assets.METADATA_SHA)
        require(tensor_digest(model.state_dict()) == assets.MODEL_STATE, "actual frozen W1 state digest differs")
        return model


def directory_bytes(directory):
    result = 0
    for path in Path(directory).iterdir():
        require(path.is_file() and not path.is_symlink(), "sample archive must contain regular files only")
        result += path.stat().st_size
    return result


@contextmanager
def observed_runtime_inputs(run, research, exact, collector):
    """HH_261001 - One non-mutating observer hashes every actual build and archives every twentieth input."""
    directory = Path(run) / "sampled_exact_runtime_inputs"
    directory.mkdir(parents=True, exist_ok=False)
    original_step, original_builder = research.SimulationResearchPolicy.step, research.build_runtime_tensors
    sequence, sample_sequence = 0, 0
    stream = (Path(run) / "all_inference_input_hashes.jsonl").open("x")
    sample_stream = (directory / "index.jsonl").open("x")

    def step(policy, inputs, **kwargs):
        nonlocal sequence, sample_sequence
        captured = []
        sampled = sequence % contract.SAMPLE_EVERY == 0

        def builder(*args, **options):
            tensors = original_builder(*args, **options)
            started = time.perf_counter()
            pixels = []
            for camera_index, frame in enumerate(inputs.camera_frames):
                require(frame.encoding == "bgra8" and isinstance(frame.payload, np.ndarray)
                        and frame.payload.shape == (360, 640, 4) and frame.payload.dtype == np.uint8,
                        "unchanged native BGRA camera ABI required for exact pixel hash")
                pixels.append(dict(camera_index=camera_index, timestamp_ns=frame.timestamp_ns, encoding=frame.encoding,
                                   shape=list(frame.payload.shape), dtype=str(frame.payload.dtype),
                                   pixels_sha256=exact.array_digest(frame.payload)))
            values = {name: exact.array_digest(tensors[name].detach().cpu().numpy()) for name in research.FIELDS}
            captured.append(dict(native_pixels=pixels, built_tensors_sha256=values,
                observer_hash_seconds=time.perf_counter()-started,
                arrays={name: tensors[name].detach().cpu().numpy().copy() for name in research.FIELDS} if sampled else None,
                observer_hash_and_copy_seconds=time.perf_counter()-started))
            return tensors

        research.build_runtime_tensors = builder
        try:
            decision = original_step(policy, inputs, **kwargs)
        finally:
            research.build_runtime_tensors = original_builder
        require(len(captured) <= 1, "one original tensor build per camera policy call required")
        saved = None
        if captured and sampled:
            arrays = captured[0]["arrays"]
            encoded = BytesIO()
            np.savez(encoded, **arrays)
            size, used = encoded.tell(), directory_bytes(directory)
            if sample_sequence >= 23 or used+size+8192 > contract.SAMPLE_LIMIT:
                write_new(Path(run) / "sample_allowance_abort.json", dict(schema=contract.SCHEMA + ".sample_abort",
                    policy_call_index=sequence, sample_sequence=sample_sequence,
                    maximum_samples=23, retained_bytes=used, proposed_npz_bytes=size,
                    allowance_bytes=contract.SAMPLE_LIMIT, proposed_decision_sent=False,
                    original_unmodified_unacknowledged_decision=decision,
                    output_or_control_modified=False, training_data_approved=False))
                raise collector.CollectionError("fixed NEW every-twentieth exact sample allowance reached")
            timing = dict(now_ns=kwargs["now_ns"], ego_stamp_ns=kwargs["ego_stamp_ns"],
                          route_stamp_ns=kwargs["route_stamp_ns"],
                          camera_stamps_ns=[frame.timestamp_ns for frame in inputs.camera_frames])
            started = time.perf_counter()
            saved = exact.save_input_snapshot(directory, sample_sequence, arrays, timing, decision,
                                             captured[0]["observer_hash_and_copy_seconds"])
            require((directory / saved["tensor_file"]).stat().st_size == size, "prospective exact NPZ byte count differs")
            saved.update(policy_call_index=sequence, every_nth_policy_call=20,
                sample_directory_allowance_bytes=contract.SAMPLE_LIMIT,
                archive_wall_seconds_outside_policy=time.perf_counter()-started)
            sample_stream.write(json.dumps(saved, sort_keys=True, allow_nan=False)+"\n")
            sample_stream.flush()
            sample_sequence += 1
        if captured:
            # HH_261001 - Array bytes are saved separately, never serialized into the all-inference hash journal.
            captured[0].pop("arrays")
        row = dict(schema=contract.SCHEMA + ".inference_input_hash", sequence=sequence,
            now_ns=kwargs["now_ns"], ego_stamp_ns=kwargs["ego_stamp_ns"], route_stamp_ns=kwargs["route_stamp_ns"],
            camera_stamps_ns=[frame.timestamp_ns for frame in inputs.camera_frames],
            actual_tensor_build=bool(captured), capture=captured[0] if captured else None,
            source_checkpoint_sha256=decision["source_checkpoint_sha256"],
            raw_output_sha256=exact.output_digest(decision["raw_output"]),
            unchanged_policy_total_seconds=decision["total_seconds"],
            sampled_exact_tensor_file=saved["tensor_file"] if saved else None,
            observer_cost_in_original_deadline=True, inputs_outputs_controls_modified=False,
            journal_write_outside_policy_deadline=True, end_to_end_fps_approved=False)
        stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
        stream.flush()
        sequence += 1
        return decision

    research.SimulationResearchPolicy.step = step
    try:
        yield
    finally:
        research.SimulationResearchPolicy.step = original_step
        research.build_runtime_tensors = original_builder
        stream.close()
        sample_stream.close()
        require(directory_bytes(directory) <= contract.SAMPLE_LIMIT, "exact sample archive exceeded 128 MiB")


@contextmanager
def native_guards(run, short, research):
    """HH_261001 - Keep native 2.7 m/s and storage floor guards before every captured command, including held commands."""
    original = short.LiveResearchSession.propose
    reasons = {"initial_drive_control", "next_driving_control", "stationary_warmup_start",
               "next_stationary_warmup_control", "next_stationary_tail_control"}
    stream = (Path(run) / "native_guard.jsonl").open("x")

    def propose(session, transport, teacher, reason):
        if reason not in reasons:
            return original(session, transport, teacher, reason)
        state = session.states[-1] if session.states else None
        velocity = state.get("world_velocity_carla") if state else None
        speed = math.hypot(float(velocity[0]), float(velocity[1])) if velocity else None
        free = shutil.disk_usage(contract.ROOT).free
        exceeded = speed is not None and (not math.isfinite(speed) or speed > 2.7)
        row = dict(command_reason=reason, source_frame=state["frame"] if state else None,
            measured_actor_planar_speed_mps=speed, maximum_actor_planar_speed_mps=2.7,
            free_disk_bytes=free, required_running_free_bytes=contract.FLOOR+contract.CLOSING_RESERVE,
            speed_guard_exceeded=exceeded, speed_guard_source="immutable native actor snapshot world XY norm")
        if exceeded or free < contract.FLOOR+contract.CLOSING_RESERVE:
            row["rejected_native_observation"] = state
        stream.write(json.dumps(research._json_finite(row), sort_keys=True, allow_nan=False) + "\n")
        stream.flush()
        if exceeded or free < contract.FLOOR+contract.CLOSING_RESERVE:
            raise short.collector.CollectionError("fixed W1 PI cruise native speed or storage guard exceeded")
        return original(session, transport, teacher, reason)

    short.LiveResearchSession.propose = propose
    try:
        yield
    finally:
        short.LiveResearchSession.propose = original
        stream.close()


def worker(args, remaining):
    from portable_e2e import simulation_research as research
    from scripts.e2e import collect_ah_simulation_research as short
    from scripts.e2e import town07_w1_pi_control_20261001 as control
    from scripts.e2e import capture_exact_simulation_inputs_20261001 as exact
    declaration(args.plan_sha256)
    run, spec = contract.output(args.trial), contract.trial(args.trial)
    require(remaining == capture_argv(args.trial), "complete fixed worker arguments required; no overrides")
    capture = short.collector.parse_args(remaining)
    capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
    validate_capture(capture, args.trial)
    contract.disk_check(3-args.trial)
    require(run.is_dir() and not run.is_symlink(), "owner-created fresh worker output required")
    write_new(run / "worker_endpoint_identity.json", dict(schema=contract.SCHEMA + ".worker",
        trial=spec, source_checkpoint_sha256=assets.CHECKPOINT, model_state_sha256=assets.MODEL_STATE,
        declaration_sha256=args.plan_sha256, optimizer_updates=0,
        maximum_capture_seconds=45., simulation_only=True, model_promoted=False,
        training_data_approved=False, vehicle_control_approved=False))
    torch.set_num_threads(2)
    model = load_w1(research, short)
    try:
        with w1_identity(research, short), control.tracker_scope(research, "B_pi_only"), \
                native_guards(run, short, research), observed_runtime_inputs(run, research, exact, short.collector):
            # HH_261001 - This new prospective 45-second owner calls the frozen session directly; old short CLI limits stay intact.
            return short.run(model, capture)
    finally:
        declaration(args.plan_sha256)


def archive_sources(run, declared, expected):
    for name, sha in declared["source_sha256"].items():
        target = run / "source" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        with (contract.ROOT / name).open("rb") as src, target.open("xb") as dest:
            shutil.copyfileobj(src, dest)
        require(digest(contract.ROOT / name) == sha == digest(target), "archived dependency differs: " + name)
    for name, sha in declared["external_agent_sha256"].items():
        source = Path(name)
        target = run / "source_external" / source.relative_to(assets.CARLA_ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as src, target.open("xb") as dest:
            shutil.copyfileobj(src, dest)
        require(digest(source) == sha == digest(target), "archived external agent differs")
    with contract.PLAN.open("rb") as src, (run / "declared_plan.json").open("xb") as dest:
        shutil.copyfileobj(src, dest)
    require(digest(run / "declared_plan.json") == expected, "archived declaration differs")


def source_checks(run, declared):
    local = all(digest(contract.ROOT / name) == sha == digest(run / "source" / name)
                for name, sha in declared["source_sha256"].items())
    external = all(digest(name) == sha == digest(run / "source_external" / Path(name).relative_to(assets.CARLA_ROOT))
                   for name, sha in declared["external_agent_sha256"].items())
    pinned = all(digest(name) == sha for name, sha in declared["asset_sha256"].items())
    return local, external, pinned


def previous_ownership_ready(index, expected):
    # HH_261001 - Continue through motion failures, never through unsafe ownership/source cleanup.
    for prior_index in range(index):
        run = contract.output(prior_index)
        result, plan = read_json(run / "owner_result.json"), read_json(run / "owner_plan.json")
        require(result.get("trial") == plan.get("trial") == contract.trial(prior_index)
                and result.get("declaration_sha256") == plan.get("declaration_sha256") == expected
                and result.get("ports_released") is True and not result.get("cleanup_errors")
                and result.get("source_hashes_match") is True and result.get("external_source_hashes_match") is True
                and result.get("asset_hashes_match") is True and digest(run / "declared_plan.json") == expected,
                "prior attempt must retain safe cleanup and exact declaration; motion PASS is not selected")


def preflight(index, expected):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from portable_e2e import simulation_research as research
    from scripts.e2e import collect_ah_simulation_research as short
    declared = declaration(expected)
    require(sys.version_info[:2] == (3, 10), "packaged CARLA API requires Python 3.10")
    require(not contract.output(index).exists() and not contract.output(index).is_symlink(),
            "fresh fixed attempt required; retries and overwrites forbidden")
    previous_ownership_ready(index, expected)
    reserve = contract.disk_check(3-index)
    require(os.access(assets.CARLA_ROOT / "CarlaUE4.sh", os.X_OK), "packaged server executable missing")
    check = subprocess.run([sys.executable, "-c", "import carla,agents,torch,numpy,PIL,cv2,shapely; print(carla.__file__); print(torch.__version__)"],
        cwd=contract.ROOT, env=owner.environment(), text=True, capture_output=True, check=True, timeout=30)
    require(check.stdout.startswith(str(assets.EGG) + "/"), "CARLA import is not from pinned packaged egg")
    capture = short.collector.parse_args(capture_argv(index))
    capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
    validate_capture(capture, index)
    torch.set_num_threads(2)
    load_w1(research, short)
    return dict(schema=contract.SCHEMA + ".owner_plan", simulator_started=False,
        trial=contract.trial(index), declaration_sha256=expected,
        source_checkpoint_sha256=assets.CHECKPOINT, model_state_sha256=assets.MODEL_STATE,
        source_sha256=declared["source_sha256"], source_pins=declared["asset_sha256"],
        external_agent_sha256=declared["external_agent_sha256"], disk_reservation=reserve,
        collector_command=worker_command(index, expected),
        server_command=["bash", "tools/e2e/run_carla_map.sh", "Town07", "--port", str(contract.PORT),
                        "--quality", "Epic", "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        capture_budget_seconds=45., wall_timeout_seconds=300., model_optimizer_updates=0,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
        all_three_declared=list(contract.trial_specs()), all_exact_inputs_archived=False,
        full_feature_approval=False, full_route_goal_success_claimed=False, end_to_end_fps_approved=False)


def execute(index, expected, plan):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    declared, run = declaration(expected), contract.output(index)
    require(plan["trial"] == contract.trial(index) and plan["declaration_sha256"] == expected,
            "preflight identity differs")
    with owner.lifecycle._workspace_lock():
        declaration(expected)
        previous_ownership_ready(index, expected)
        contract.disk_check(3-index)
        require(not run.exists() and not run.is_symlink(), "fresh attempt output required")
        run.mkdir(parents=True)
        archive_sources(run, declared, expected)
        write_new(run / "owner_plan.json", plan)
        server = worker_process = None
        identities, cleanup_errors, bind_checks, previous_handlers = {}, [], [], {}
        error, worker_code, released = None, None, False
        local = external = pinned = False

        def interrupted(signum, frame):
            raise RuntimeError("owner interrupted by signal " + str(signum))

        try:
            owner.lifecycle._assert_no_other_carla()
            bind_checks.append(dict(stage="before_launch", **wait_for_original_bind_checks(contract.PORT, owner.ports_free)))
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous_handlers[signum] = signal.signal(signum, interrupted)
            with (run / "server.log").open("xb") as log:
                server = subprocess.Popen(plan["server_command"], cwd=contract.ROOT, env=owner.environment(),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            identities["server"] = owner.process_record(server.pid)
            require(identities["server"] is not None, "started server identity unavailable")
            deadline, socket_owner = time.monotonic()+130., None
            while time.monotonic() < deadline:
                require(server.poll() is None, "owned packaged server exited before readiness")
                if b"CARLA_READY " in (run / "server.log").read_bytes():
                    descendants = owner.descendants(server.pid, owner.all_processes())
                    matching = [row for row in descendants if row["pgid"] == server.pid
                        and any("CarlaUE4-Linux-Shipping" in arg for arg in row["argv"])
                        and owner.listening_socket_owned(row["pid"], contract.PORT)]
                    require(len(matching) == 1, "RPC socket must belong uniquely to started shipping server")
                    socket_owner = matching[0]
                    break
                time.sleep(.5)
            require(socket_owner is not None, "owned packaged server readiness timeout")
            write_new(run / "owner_started.json", dict(server=identities["server"], socket_owner=socket_owner))
            with (run / "collector.log").open("xb") as log:
                worker_process = subprocess.Popen(plan["collector_command"], cwd=contract.ROOT, env=owner.environment(),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                identities["worker"] = owner.process_record(worker_process.pid)
                require(identities["worker"] is not None, "started worker identity unavailable")
                worker_code = worker_process.wait(timeout=plan["wall_timeout_seconds"])
            if worker_code:
                error = "collector returned " + str(worker_code) + "; original noncompletion or failure retained"
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            for process, name in ((worker_process, "worker"), (server, "server")):
                try:
                    if process is not None:
                        now = owner.process_record(process.pid)
                        require(not now or now["start_ticks"] == identities[name]["start_ticks"],
                                "owned PID generation changed; refusing signal")
                    require(owner.lifecycle._stop_owned_group(process), "owned process group did not stop")
                except Exception as exc:
                    cleanup_errors.append(f"{name}: {exc}")
            try:
                bind_checks.append(dict(stage="after_owned_cleanup", **wait_for_original_bind_checks(contract.PORT, owner.ports_free)))
                released = True
            except OSError as exc:
                cleanup_errors.append("natural original bind check: " + str(exc))
            try:
                local, external, pinned = source_checks(run, declared)
                require(local and external and pinned, "declared dependency, archive or external asset changed")
            except Exception as exc:
                cleanup_errors.append("source closure: " + str(exc))
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
            result = dict(schema=contract.SCHEMA + ".owner_result", trial=contract.trial(index),
                declaration_sha256=expected, source_checkpoint_sha256=assets.CHECKPOINT,
                status="OWNER_FAILURE" if cleanup_errors else "EXECUTION_FAILED_RETAINED" if error else "FINISHED_RESEARCH_ONLY",
                collector_exit_code=worker_code, error=error, cleanup_errors=cleanup_errors,
                ports_released=released, source_hashes_match=local, external_source_hashes_match=external,
                asset_hashes_match=pinned, physical_attempt_started=server is not None,
                other_jobs_terminated=False, sockets_reused_or_forced=False,
                model_optimizer_updates=0, training_data_approved=False,
                vehicle_control_approved=False, model_promoted=False,
                full_feature_approval=False, full_route_goal_success_claimed=False)
            write_new(run / "natural_port_release_receipt.json", dict(schema=contract.SCHEMA + ".natural_release",
                checks=bind_checks, all_required_original_binds_passed=released,
                sockets_reused_or_forced=False, other_jobs_terminated=False))
            write_new(run / "owner_result.json", result)
        return result


def rows(path, maximum=1000):
    require(digest(path) and Path(path).stat().st_size <= 128*1024**2, "bounded regular native journal required")
    result = []
    with Path(path).open() as stream:
        for line in stream:
            result.append(json.loads(line))
            require(len(result) <= maximum, "native journal exceeded declared maximum rows")
    return result


def native_learned_interval(states, decisions):
    """HH_261001 - Associate observed motion with the prior accepted learned command, including every held tick."""
    lookup = {}
    for decision in decisions:
        sequence = decision.get("ack_receipt_sequence")
        require(type(sequence) is int and sequence not in lookup, "unique native learned decision receipt required")
        lookup[sequence] = decision
    first = next((index for index, state in enumerate(states)
        if state.get("capture_phase") == "driving"
        and lookup.get(state.get("control_transport", {}).get("expected_receipt_sequence"), {}).get(
            "application", {}).get("learned_control_applied") is True), None)
    if first is None:
        return [], [], ["no_observed_learned_ack_interval"]
    selected = states[first:]
    witnesses, failures = [], []
    for state in selected:
        decision = lookup.get(state.get("control_transport", {}).get("expected_receipt_sequence"))
        witnesses.append(decision)
        if not (state.get("capture_phase") == "driving" and decision
                and decision.get("control_source") == "learned_trajectory_tracker"
                and decision.get("learned_control_proposed") is True
                and decision.get("application", {}).get("acknowledged") is True
                and decision.get("application", {}).get("learned_control_applied") is True
                and decision.get("emergency_reason") is None):
            failures.append(f"nonlearned_or_rejected_native_tick:{state['frame']}")
    return selected, witnesses, failures


def motion_numbers(states):
    require(bool(states), "nonempty learned observation interval required")
    cte = sorted(abs(float(state["route_cte_m"])) for state in states)
    speeds = [math.hypot(*map(float, state["world_velocity_carla"][:2])) for state in states]
    require(all(math.isfinite(x) for x in cte+speeds), "finite actual native motion values required")
    run, maximum = 0, 0
    for speed in speeds:
        run = run+1 if speed <= .05 else 0
        maximum = max(maximum, run)
    return dict(learned_native_observations=len(states), first_frame=states[0]["frame"],
        final_frame=states[-1]["frame"], observed_span_seconds=states[-1]["timestamp"]-states[0]["timestamp"],
        recorded_base_progress_m=states[-1]["route_progress_m"]-states[0]["route_progress_m"],
        actor_speed_min_mps=min(speeds), actor_speed_max_mps=max(speeds), actor_speed_mean_mps=sum(speeds)/len(speeds),
        longest_unintended_stationary_dwell_seconds=maximum*.05,
        stationary_dwell_definition="consecutive native <=0.05 m/s observations times fixed 0.05 s; includes terminal observed tick",
        cte_p95_m=cte[math.ceil(.95*len(cte))-1], cte_abs_maximum_m=cte[-1],
        cte_quantile_definition="nearest-rank absolute CTE over entire observed learned interval",
        collision_record_count=sum(len(state["collision"]) for state in states),
        lane_invasion_record_count=sum(len(state["lane_invasion"]) for state in states))


def geometry_rows(manifest, route, states, xml, body, carla, accepted_actor_id):
    """HH_261001 - Use frozen whole-body/snapshot-linear swept bounds against intended lanes inset by 10 cm."""
    live = manifest["capture_contract"]["active_map_opendrive"]
    require(hashlib.sha256(xml).hexdigest() == live["sha256"] == assets.assets()[str(assets.XODR)]
            and len(xml) == live["size_bytes"], "archived active map identity differs")
    record = manifest["capture_contract"]["actor_body_geometry"]
    require(record["status"] == "RECORDED" and record["source"] == "carla.Vehicle.bounding_box",
            "recorded actual actor body geometry required")
    require(record["actor_id"] == accepted_actor_id, "actual box actor differs from unique acknowledged actor")
    construction = manifest["capture_contract"].get("agent_initialization", {}).get("construction")
    if construction is not None:
        require(construction["status"] == "PASS" and all(construction[side]["actor_id"] == record["actor_id"]
                for side in ("before", "after")), "optional recorded agent construction actor identity mismatch")
    points = []
    for point in route["route"]:
        if not points or point["distance_m"] != points[-1]["distance_m"]:
            points.append(point)
        else:
            require(all(point[key] == points[-1][key] for key in ("x", "y", "z", "road_id", "section_id", "lane_id")),
                    "duplicate route arc with conflicting intended lane/point")
    ids = {body._triple(point) for point in points}
    carla_map = carla.Map("archived-w1-pi-cruise-repetition-research", xml.decode())
    corridors = body.lane_corridors(carla_map, carla, points, xml, ids, set(range(len(points))))
    box, radius = body._body_box(manifest, carla)
    cache, output, prior = {}, [], None

    def allowed(lo, hi):
        lanes = frozenset(body._route_lanes_for_arc_interval(points, lo, hi))
        if lanes not in cache:
            cache[lanes] = body.authorized_lane_region(corridors, lanes, .1)
        return cache[lanes], sorted(lanes)

    for state in states:
        pose = state["actor_snapshot_transform_carla"]
        arc, segment, cte = body._route_projection((pose["x"], pose["y"]), points)
        footprint = body.body_footprint(box, pose, carla)
        region, lanes = allowed(arc-radius, arc+radius)
        outside = footprint.difference(region).area
        swept_outside = None
        if prior:
            previous_pose, previous_arc, previous_footprint, previous_segment = prior
            require(-.25 <= arc-previous_arc <= body.MAX_FRAME_PROGRESS_M
                    and 0 <= segment-previous_segment <= 3, "native arc/segment progress exceeds frozen sweep bound")
            swept = body.swept_body(previous_footprint, footprint, previous_pose, pose, radius)
            swept_region, _ = allowed(previous_arc-radius, arc+radius)
            swept_outside = swept.difference(swept_region).area
        output.append(dict(frame=state["frame"], independent_actor_progress_m=arc,
            independent_actor_route_distance_m=cte, intended_lanes=lanes,
            body_outside_inset_m2=outside, prior_sweep_outside_inset_m2=swept_outside,
            body_polygon_carla_xy=list(footprint.exterior.coords),
            body_bound=outside <= 1e-6, sweep_bound=swept_outside is None or swept_outside <= 1e-6))
        prior = pose, arc, footprint, segment
    return output


def decision_checks(decisions):
    """HH_261001 - Count every post-handoff command, not just selected or successful inference frames."""
    first = next((i for i, value in enumerate(decisions)
        if value.get("application", {}).get("learned_control_applied") is True), None)
    if first is None:
        return dict(nonempty=False, post_handoff_commands=0, emergency_commands=0,
                    unexpected_stop_commands=0, control_invariant_failures=["no_handoff"], max_policy_seconds=None)
    interval, failures, stops, emergencies, latencies = decisions[first:], [], 0, 0, []
    unchanged = ("selected_speed_or_path_changed", "steering_changed", "stop_branch_changed",
                 "emergency_authority_changed", "control_cadence_changed", "feedforward_added", "throttle_floor_added")
    for decision in interval:
        sequence = decision["ack_receipt_sequence"]
        if decision.get("control_source") == "emergency_brake" or decision.get("emergency_reason"):
            emergencies += 1
        raw = decision.get("raw_output")
        selected, control, audit = decision.get("selected"), decision.get("control"), decision.get("longitudinal_pi_research")
        if not (decision.get("source_checkpoint_sha256") == assets.CHECKPOINT
                and decision.get("control_source") == "learned_trajectory_tracker"
                and decision.get("application", {}).get("learned_control_applied") is True
                and isinstance(audit, dict) and audit.get("arm") == "B_pi_only"
                and all(audit.get(name) is False for name in unchanged)
                and audit.get("integral_added") is True and audit.get("normal_control_returned") is True
                and isinstance(selected, dict) and isinstance(control, dict) and isinstance(raw, dict)):
            failures.append(f"normal_control_or_invariant:{sequence}")
            continue
        logits = raw.get("candidate_logits", [])
        require(len(logits) == len(raw.get("candidate_xy", [])) == len(raw.get("candidate_speed", [])) == 12,
                "all twelve unchanged candidate outputs required")
        require(all(len(xy) == len(speed) == 64 and all(len(point) == 2 for point in xy)
                for xy, speed in zip(raw["candidate_xy"], raw["candidate_speed"])),
                "all original twelve-candidate sixty-four-step outputs required")
        winner = max(range(12), key=lambda i: logits[i])
        if selected["candidate_index"] != winner:
            failures.append(f"raw_argmax_changed:{sequence}")
        if selected.get("xy_base_m") != raw["candidate_xy"][winner] or selected.get("speed_mps") != raw["candidate_speed"][winner]:
            failures.append(f"raw_winning_trajectory_changed:{sequence}")
        desired = float(control["desired_speed_mps"])
        if winner >= 6 or selected["candidate_index"] >= 6 or desired <= .05:
            stops += 1
        original, proposed = audit.get("original_normal_tracker_control"), audit.get("proposed_normal_tracker_control")
        if not (isinstance(original, dict) and isinstance(proposed, dict) and control == proposed
                and 0 <= desired <= 2.5
                and all(control.get(key) == original.get(key) for key in original if key not in ("throttle", "brake"))
                and 0 <= control["throttle"] <= .5 and 0 <= control["brake"] <= 1.):
            failures.append(f"expanded_controller_intervention:{sequence}")
        latency = float(decision["total_seconds"])
        require(math.isfinite(latency) and latency >= 0., "finite original inference/control latency required")
        latencies.append(latency)
    return dict(nonempty=True, post_handoff_commands=len(interval), emergency_commands=emergencies,
        unexpected_stop_commands=stops, control_invariant_failures=failures,
        max_policy_seconds=max(latencies) if latencies else None)


def acknowledged_actuator_fields_match(requested, proposed):
    """HH_261001 - Respect the frozen 1e-6 CARLA float32 control conversion tolerance, never rewrite either value."""
    if not isinstance(requested, dict) or not isinstance(proposed, dict):
        return False
    for name in ("throttle", "brake", "steer"):
        left, right = requested.get(name), proposed.get(name)
        if (type(left) not in (int, float) or type(right) not in (int, float)
                or not math.isfinite(left) or not math.isfinite(right) or abs(left-right) > 1e-6):
            return False
    return all(type(requested.get(name)) is bool and requested.get(name) is proposed.get(name)
               for name in ("reverse", "hand_brake"))


def assess(index, expected):
    """HH_261001 - Read raw evidence only; keep false goal/PENDING QA and every failed attempt unchanged."""
    from scripts.e2e import audit_carla_acknowledged_control as ack
    from scripts.e2e import audit_future_actor_lane_containment as body
    from scripts.e2e import capture_exact_simulation_inputs_20261001 as exact
    declared, run = declaration(expected), contract.output(index)
    owner = read_json(run / "owner_result.json")
    result = dict(schema=contract.SCHEMA + ".motion_screen", trial=contract.trial(index),
        declaration_sha256=expected, source_checkpoint_sha256=assets.CHECKPOINT,
        retained_in_three_attempt_denominator=True, status="FAIL_RETAINED", failures=[],
        gate_limits=declared["prospective_motion_screen"], full_feature_approval=False,
        full_RTE_01_approved=False, full_route_goal_success_claimed=False,
        generalization_approved=False, end_to_end_realtime_fps_approved=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    try:
        local, external, pinned = source_checks(run, declared)
        require(local and external and pinned and owner["ports_released"] is True
                and not owner["cleanup_errors"] and owner["source_hashes_match"] is True
                and owner["external_source_hashes_match"] is True and owner["asset_hashes_match"] is True,
                "ownership, original port release and source cleanup must all pass")
        episode = run / "episode.partial" if (run / "episode.partial").exists() else run / "episode"
        manifest = read_json(episode / "manifest.json")
        states, cameras, receipts, decisions = [rows(episode / name) for name in
            ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl", "learned_control.jsonl")]
        hashes = rows(run / "all_inference_input_hashes.jsonl")
        samples = rows(run / "sampled_exact_runtime_inputs/index.jsonl")
        learned, witnesses, binding_failures = native_learned_interval(states, decisions)
        summary = read_json(episode / "learned_summary.json")
        transport = ack.analyze_transport(states, receipts,
            manifest["capture_contract"]["control_transport"]["bootstrap_observation"],
            wheelbase=manifest["coordinate_contract"]["wheelbase_m"])
        camera_alignment = ack.analyze_camera_alignment(states, cameras)
        numbers, decisions_result = motion_numbers(learned), decision_checks(decisions)
        require(sys.version_info[:2] == (3, 10), "pinned CARLA offline map geometry requires Python 3.10")
        sys.path.insert(0, str(assets.EGG))
        import carla
        require(str(carla.__file__).startswith(str(assets.EGG)+"/"), "offline geometry imported foreign CARLA API")
        route = read_json(episode / "route.json")
        require(digest(episode / "route.json") == assets.assets()[str(assets.ROUTE)], "recorded route bytes changed")
        geometry = geometry_rows(manifest, route, states,
            (episode / "active_map_opendrive.xodr").read_bytes(), body, carla, receipts[0]["actor_id"])
        geometry_lookup = {row["frame"]: row for row in geometry}
        learned_geometry = [geometry_lookup[state["frame"]] for state in learned]
        numbers.update(independent_actor_net_progress_m=learned_geometry[-1]["independent_actor_progress_m"]
                       -learned_geometry[0]["independent_actor_progress_m"],
            body_clearance_failures=sum(not row["body_bound"] for row in learned_geometry),
            swept_clearance_failures=sum(not row["sweep_bound"] for row in learned_geometry),
            body_and_sweep_lane_inset_m=.1,
            body_geometry_scope="actual box with recorded rotation/pose; bounded snapshot-linear sweep, not continuous physical collision proof")
        receipt_by_sequence = {row["sequence"]: row for row in receipts}
        decision_ack_issues = []
        for decision in decisions:
            receipt = receipt_by_sequence.get(decision["ack_receipt_sequence"])
            source_frame = decision.get("command_frame")
            chosen = decision.get("control", decision.get("teacher_proposal", {}))
            if not (receipt and receipt["server_accepted"] is True and receipt["status"] == "ACKNOWLEDGED"
                    and receipt["reason"] == decision["command_reason"]
                    and (source_frame is None or receipt["before_frame"] == source_frame)
                    and decision["application"]["acknowledged"] is True
                    and acknowledged_actuator_fields_match(receipt["requested_control"], chosen)):
                decision_ack_issues.append(decision["ack_receipt_sequence"])
        inferences = [row for row in decisions if row.get("raw_output") is not None
                      and row.get("held_between_camera_frames") is False]
        input_hash_pass = len(hashes) == summary["counts"]["inference_count"] == len(inferences)
        state_by_frame = {state["frame"]: state for state in states}
        camera_by_frame = {camera["frame"]: camera for camera in cameras}
        if input_hash_pass:
            for i, (row, decision) in enumerate(zip(hashes, inferences)):
                captured = row["capture"]
                native_state = state_by_frame[decision["inference_frame"]]
                camera = camera_by_frame[decision["inference_frame"]]
                native_stamp = int(round(native_state["timestamp"]*1e9))
                camera_stamps = [int(round(camera["source_timestamps"][name]*1e9)) for name in camera["camera_order"]]
                input_hash_pass = input_hash_pass and (row["sequence"] == i and row["actual_tensor_build"] is True
                    and row["now_ns"] == row["ego_stamp_ns"] == row["route_stamp_ns"] == native_stamp
                    and row["camera_stamps_ns"] == camera_stamps and native_state["frame"] == camera["frame"]
                    and row["raw_output_sha256"] == hashlib.sha256(json.dumps(decision["raw_output"], sort_keys=True,
                        separators=(",", ":"), allow_nan=False).encode()).hexdigest()
                    and row["source_checkpoint_sha256"] == assets.CHECKPOINT
                    and isinstance(captured, dict) and len(captured["native_pixels"]) == 6
                    and len(captured["built_tensors_sha256"]) == 6
                    and all(pixel["camera_index"] == camera_index and pixel["encoding"] == "bgra8"
                        and pixel["shape"] == [360, 640, 4] and pixel["dtype"] == "uint8"
                        and pixel["timestamp_ns"] == camera_stamps[camera_index]
                        and isinstance(pixel["pixels_sha256"], str) and len(pixel["pixels_sha256"]) == 64
                        for camera_index, pixel in enumerate(captured["native_pixels"]))
                    and row["observer_cost_in_original_deadline"] is True
                    and row["inputs_outputs_controls_modified"] is False)
        sample_pass = [row["policy_call_index"] for row in samples] == list(range(0, len(hashes), 20))
        sample_pass = sample_pass and len(samples) <= 23 and directory_bytes(run / "sampled_exact_runtime_inputs") <= contract.SAMPLE_LIMIT
        for row in samples:
            sample_pass = sample_pass and digest(run / "sampled_exact_runtime_inputs" / row["tensor_file"]) == row["tensor_file_sha256"]
            matching = hashes[row["policy_call_index"]]
            require(row["timing"]["now_ns"] == matching["now_ns"], "sparse NPZ timing differs from original inference")
            with np.load(run / "sampled_exact_runtime_inputs" / row["tensor_file"], allow_pickle=False) as values:
                require(set(values.files) == set(row["tensor_sha256"]) == set(matching["capture"]["built_tensors_sha256"]),
                        "sparse NPZ tensor ABI keys differ")
                for name in values.files:
                    actual = exact.array_digest(values[name])
                    sample_pass = sample_pass and actual == row["tensor_sha256"][name] == matching["capture"]["built_tensors_sha256"][name]
        jpeg_pins = {}
        for camera in cameras:
            require(len(camera["images"]) == 6, "six original JPEGs required in every native camera bundle")
            for name, relative in camera["images"].items():
                path = episode / relative
                require(path.resolve().is_relative_to((episode / "images").resolve()), "camera JPEG escaped episode images")
                jpeg_pins[relative] = digest(path)
        capture = manifest["capture_contract"]
        capture_complete = (len(states) == 900 and len(cameras) == 450 and len(jpeg_pins) == 2700
            and capture["physics_hz"] == 20. and capture["camera_hz"] == 10.
            and capture["maximum_total_duration_sec"] == 45.
            and manifest["error"] == BUDGET_ERROR and manifest["status"] == "failed"
            and manifest["cleanup"]["completed"] is True and not manifest["cleanup"]["errors"])
        checks = dict(exact_fixed_45_second_capture=capture_complete,
            all_native_transport_snapshot_and_held_ack_binding=transport["status"] == "PASS"
                and not binding_failures and not decision_ack_issues,
            camera_native_alignment=camera_alignment["status"] == "PASS",
            original_all_inference_pixels_tensors_and_raw_outputs_bound=bool(input_hash_pass),
            every_twentieth_exact_inputs_retained_within_128_MiB=bool(sample_pass),
            actual_learned_progress_at_least_90_m=numbers["independent_actor_net_progress_m"] >= 90.,
            unintended_stationary_dwell_below_one_second=numbers["longest_unintended_stationary_dwell_seconds"] < 1.,
            cte_p95_at_most_035_m=numbers["cte_p95_m"] <= .35,
            cte_max_at_most_075_m=numbers["cte_abs_maximum_m"] <= .75,
            actor_speed_at_most_27_mps=numbers["actor_speed_max_mps"] <= 2.7,
            all_body_and_sweeps_have_10_cm_intended_lane_clearance=numbers["body_clearance_failures"]
                == numbers["swept_clearance_failures"] == 0,
            no_collision_or_lane_events=numbers["collision_record_count"] == numbers["lane_invasion_record_count"] == 0,
            no_rejected_or_emergency_commands=decisions_result["emergency_commands"] == 0
                and not decisions_result["control_invariant_failures"] and summary["latched_emergency"] is None,
            no_explicit_or_unexpected_model_stop=decisions_result["unexpected_stop_commands"] == 0,
            unchanged_point_one_second_inference_control_deadline=decisions_result["max_policy_seconds"] is not None
                and decisions_result["max_policy_seconds"] <= .1)
        result.update(checks=checks, failures=[name for name, passed in checks.items() if not passed],
            metrics=numbers, decisions=decisions_result, native_transport=transport,
            camera_alignment=camera_alignment, learned_binding_failures=binding_failures,
            decision_ack_failures=decision_ack_issues, original_manifest_status=manifest["status"],
            original_manifest_error=manifest["error"], original_goal_reached=manifest["result"].get("goal_reached", False),
            original_goal_and_training_flags_not_rewritten=True, learned_summary=summary,
            all_original_JPEG_sha256=jpeg_pins,
            raw_evidence_sha256={name: digest(episode / name) for name in (
                "manifest.json", "states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl",
                "learned_control.jsonl", "learned_summary.json", "route.json", "active_map_opendrive.xodr")},
            all_inference_input_hashes_sha256=digest(run / "all_inference_input_hashes.jsonl"),
            exact_sample_index_sha256=digest(run / "sampled_exact_runtime_inputs/index.jsonl"),
            geometry=geometry)
        if all(checks.values()):
            result["status"] = "PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY"
    except (ValueError, OSError, KeyError, TypeError, IndexError, AssertionError, RuntimeError) as error:
        result["failures"].append(f"assessment_error:{type(error).__name__}:{error}")
    write_new(run / "cruise_motion_screen.json", result)
    return result


def repetition_summary(expected):
    declaration(expected)
    results = []
    for index in range(3):
        run = contract.output(index)
        result, owner = read_json(run / "cruise_motion_screen.json"), read_json(run / "owner_result.json")
        require(result["trial"] == owner["trial"] == contract.trial(index)
                and result["declaration_sha256"] == owner["declaration_sha256"] == expected
                and result["retained_in_three_attempt_denominator"] is True,
                "all three fixed results and failures required; no denominator substitution")
        results.append(dict(trial=contract.trial(index), status=result["status"], failures=result["failures"],
            metrics=result.get("metrics"), motion_screen_sha256=digest(run / "cruise_motion_screen.json"),
            owner_result_sha256=digest(run / "owner_result.json")))
    passed = sum(row["status"] == "PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY" for row in results)
    value = dict(schema=contract.SCHEMA + ".repetition_summary", declaration_sha256=expected,
        status="THREE_RESEARCH_CRUISE_SCREENS_PASS" if passed == 3 else "REPETITION_SCREEN_FAILED_RETAINED",
        passed_attempts=passed, total_predeclared_attempts=3, failed_attempts=3-passed, attempts=results,
        failed_attempts_excluded=0, outcome_selected_retry=False,
        motion_reproducibility_research_screen_passed=passed == 3,
        all_attempts_same_TRAIN_route=True, new_or_holdout_route_used=False,
        full_feature_approval=False, full_RTE_01_approved=False, full_route_goal_success_claimed=False,
        generalization_approved=False, end_to_end_realtime_fps_approved=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    write_new(contract.RUN_ROOT / "repetition_summary.json", value)
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", required=True,
        choices=("contract", "declare", "preflight", "execute", "worker", "assess", "summary"))
    parser.add_argument("--trial", type=int)
    parser.add_argument("--plan-sha256")
    args, remaining = parser.parse_known_args(argv)
    if args.mode == "worker":
        require(args.trial is not None and args.plan_sha256, "worker requires exact fixed trial/declaration")
        return worker(args, remaining)
    require(not remaining, "runtime parameter overrides forbidden")
    if args.mode in ("contract", "declare"):
        require(args.trial is None and args.plan_sha256 is None, "contract/declaration accepts no trial overrides")
        if args.mode == "contract":
            result = dict(contract.fixed_contract(), status="PREPARED_NOT_DECLARED_NOT_EXECUTED")
        else:
            contract.disk_check(3)
            result = contract.build_plan()
            contract.PLAN.parent.mkdir(parents=True, exist_ok=True)
            write_new(contract.PLAN, result)
            result = dict(status="THREE_ATTEMPTS_DECLARED_NO_SIMULATOR", plan=str(contract.PLAN),
                          plan_sha256=digest(contract.PLAN), trial_order=list(contract.trial_specs()))
    elif args.mode == "summary":
        require(args.trial is None and args.plan_sha256, "summary requires declaration SHA and no selected trial")
        result = repetition_summary(args.plan_sha256)
    else:
        require(args.trial is not None and args.plan_sha256, "fixed trial and declaration SHA required")
        contract.trial(args.trial)
        if args.mode == "assess":
            result = assess(args.trial, args.plan_sha256)
        else:
            result = preflight(args.trial, args.plan_sha256)
            if args.mode == "execute":
                execute(args.trial, args.plan_sha256, result)
                result = assess(args.trial, args.plan_sha256)
    printed = {key: value for key, value in result.items() if key not in (
        "geometry", "all_original_JPEG_sha256", "raw_evidence_sha256", "native_transport")}
    print(json.dumps(printed, sort_keys=True, indent=2, allow_nan=False))
    return 1 if args.mode in ("execute", "assess") and result["status"] != "PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY" else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError, RuntimeError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        raise SystemExit(1)
