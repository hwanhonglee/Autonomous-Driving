"""HH_261001 - Explicitly own eleven prospectively fixed actual teacher recovery captures, never learned claims."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import gzip
import json
import math
import re
from pathlib import Path
import shutil
import subprocess
import sys

from scripts.e2e import research_temporal_v2_owned_camera_town07_20261001 as lifecycle
from scripts.e2e import collect_town07_signed_recovery_20261001 as worker_module
from scripts.e2e import town07_signed_recovery_task_20261001 as task
from scripts.e2e import audit_simulation_research_body_lane_20261001 as geometry

ROOT, CARLA_ROOT, EGG, PORT = lifecycle.ROOT, lifecycle.CARLA_ROOT, lifecycle.EGG, lifecycle.PORT
MAPPING, CALIBRATION, XODR = lifecycle.MAPPING, lifecycle.CALIBRATION, lifecycle.XODR
MODULE = "scripts.e2e.run_owned_town07_signed_recovery_20261001"
SCHEMA = task.SCHEMA+".owned_batch"
ASSETS = {path: value for path, value in lifecycle.ASSETS.items() if path not in (lifecycle.WEIGHTS, lifecycle.METADATA)}
CASE_IDS = tuple(row["case_id"] for row in task.CASES)
ARM_RESERVE, SHARED_RESERVE = 768*1024**2, 64*1024**2
require, digest, read_json, write_new, jsonl = lifecycle.require, lifecycle.digest, lifecycle.read_json, lifecycle.write_new, lifecycle.jsonl
_case = None
_active = False


def paths(run_id): return task.paths(run_id, _case)


def capture_argv(run_id, case_id, plan_sha=lifecycle.PLAN_TOKEN):
    output, plan = task.paths(run_id, case_id)
    return ["--recovery-run-id", run_id, "--recovery-case-id", case_id,
        str(output/"episode"), str(task.ROUTE), "--host", "127.0.0.1", "--port", "2440",
        "--control-transport", "acknowledged_batch", "--agent-initialization", "after_bootstrap",
        "--stationary-warmup-sec", "1.2", "--stationary-tail-sec", "6.5", "--max-duration-sec", "50",
        "--target-speed-kmh", "9", "--seed", str(task.case(case_id)["seed"]), "--weather", "ClearNoon",
        "--mapping", str(MAPPING), "--calibration", str(CALIBRATION), "--spawn-z-offset-m", ".5",
        "--wheelbase-m", "2.85", "--expected-live-xodr-sha256", ASSETS[XODR], "--goal-stop-profile", "disabled",
        "--goal-tolerance-m", "1", "--basic-agent-base-min-distance-m", "2", "--basic-agent-distance-ratio", ".2",
        "--moving-task-plan", str(plan), "--moving-task-plan-sha256", plan_sha]


def execution_contract(run_id, case_id):
    output, _ = task.paths(run_id, case_id)
    return dict(owner_output=str(output), server_command=["bash", "tools/e2e/run_carla_map.sh", "Town07", "--port", "2440",
        "--quality", "Epic", "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        worker_command=[sys.executable, "-B", str(ROOT/"tools/workspace_python.py"), "-m", MODULE, "--", "--mode", "worker",
            "--run-id", run_id, "--case-id", case_id, "--case-plan-sha256", lifecycle.PLAN_TOKEN], owner_wall_timeout_s=300.)


def runtime_sources():
    # HH_261001 - Imported-only closure: unused existing models are dependencies of the frozen owner, not loaded weights.
    sources = lifecycle.imported_source_closure((Path(__file__).resolve(), ROOT/"tools/workspace_python.py", ROOT/"tools/canonical_workspace.py"))
    sources.update((ROOT/"tools/e2e/run_carla_map.sh", ROOT/"tools/e2e/env.sh", lifecycle.DEFAULT_CONTRACT_PATH, lifecycle.INHERITED_FREEZE))
    pins = {str(path.relative_to(ROOT)): digest(path) for path in sorted(sources)}
    require(digest(lifecycle.INHERITED_FREEZE) == lifecycle.INHERITED_FREEZE_SHA, "inherited source freeze differs")
    inherited = read_json(lifecycle.INHERITED_FREEZE)["source_sha256"]
    for name, sha in pins.items():
        if name in inherited: require(sha == inherited[name], "frozen inherited source changed: "+name)
    for name, sha in lifecycle.FROZEN_GENERIC.items(): require(pins.get(name) == sha, "frozen reused owner dependency changed")
    return pins


def disk_check(free=None):
    free = shutil.disk_usage(ROOT).free if free is None else free
    required = task.FLOOR+len(task.CASES)*ARM_RESERVE+SHARED_RESERVE
    require(type(free) is int and free >= required, "all eleven captures plus closing/disk reserve required")
    return dict(free_bytes=free, required_bytes=required, prospective_cases=len(task.CASES), reserved_case_bytes=ARM_RESERVE)


def build_plan(run_id, case_id):
    inputs = dict(route_file=task.ROUTE, mapping=MAPPING, calibration=CALIBRATION)
    return dict(task.fixed_contract(run_id, case_id), **execution_contract(run_id, case_id),
        declared_at_utc=datetime.now(timezone.utc).isoformat(), input_paths={key: str(value) for key, value in inputs.items()},
        input_sha256={key: digest(value) for key, value in inputs.items()}, source_sha256=runtime_sources(),
        external_source_sha256=lifecycle.external_sources(), asset_sha256={str(path): sha for path, sha in ASSETS.items()})


def verify_case(run_id, case_id, expected):
    _, path = task.paths(run_id, case_id)
    require(isinstance(expected, str) and digest(path) == expected, "exact declared recovery case SHA required")
    plan = read_json(path)
    fixed = dict(task.fixed_contract(run_id, case_id), **execution_contract(run_id, case_id))
    require(set(plan) == set(fixed) | {"declared_at_utc", "input_paths", "input_sha256", "source_sha256", "external_source_sha256", "asset_sha256"}, "case fields changed")
    require(all(plan[key] == value for key, value in fixed.items()), "fixed recovery case changed")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0 and stamp <= datetime.now(timezone.utc), "prior UTC declaration required")
    require(plan["source_sha256"] == runtime_sources() and plan["external_source_sha256"] == lifecycle.external_sources()
        and plan["asset_sha256"] == {str(path): sha for path, sha in ASSETS.items()}, "case source/API/asset closure differs")
    require(all(digest(name) == sha for name, sha in plan["asset_sha256"].items()), "pinned recovery asset changed")
    inputs = dict(route_file=task.ROUTE, mapping=MAPPING, calibration=CALIBRATION)
    require(plan["input_paths"] == {key: str(value) for key, value in inputs.items()}
        and plan["input_sha256"] == {key: ASSETS[value] for key, value in inputs.items()}, "input source identity differs")
    return plan


def preflight(run_id, expected):
    plan = verify_case(run_id, _case, expected)
    output, _ = paths(run_id)
    require(sys.version_info[:2] == (3, 10), "pinned CARLA API requires Python3.10")
    require(not output.exists() and not output.is_symlink(), "fresh recovery case output required")
    lifecycle.lifecycle._assert_no_other_carla(); lifecycle.ports_free(PORT)
    disk = disk_check()
    check = subprocess.run([sys.executable, "-B", "-c", "import carla,agents,numpy,PIL; print(carla.__file__); print(agents.__file__)"],
        cwd=ROOT, env=lifecycle.environment(), text=True, capture_output=True, timeout=30, check=True)
    lines = check.stdout.strip().splitlines()
    require(len(lines) == 2 and lines[0].startswith(str(EGG)+"/")
        and lines[1] == str(CARLA_ROOT/"PythonAPI/carla/agents/__init__.py"), "foreign CARLA API/agent source")
    worker_module.parse_args(capture_argv(run_id, _case, expected))
    return plan, dict(status="PREFLIGHT_PASS_NO_SIMULATOR", case_id=_case, disk=disk, declaration_sha256=expected,
        learned_control=False, simulator_started=False, heldout_training_allowed=False)


@contextmanager
def scoped_case(case_id):
    global _case, _active
    require(not _active and case_id in CASE_IDS, "non-nested explicit recovery case required")
    saved_case = _case
    changes = dict(paths=paths, preflight=preflight, diagnostic_report=diagnostic_report, SCHEMA=SCHEMA,
        CHECKPOINT_SHA=None, ASSETS=ASSETS, MODULE=MODULE)
    saved = {name: getattr(lifecycle, name) for name in changes}
    _case, _active = case_id, True
    try:
        for name, value in changes.items(): setattr(lifecycle, name, value)
        yield
    finally:
        for name, value in saved.items(): setattr(lifecycle, name, value)
        _case, _active = saved_case, False


def worker(run_id, case_id, expected):
    plan = verify_case(run_id, case_id, expected)
    lifecycle.assert_owned_worker(dict(plan, output=plan["owner_output"]), expected)
    args = worker_module.parse_args(capture_argv(run_id, case_id, expected))
    with worker_module.scope(args): result = worker_module.base.run(args)
    print(result)
    return 0


def closure_check(output, plan):
    for name, sha in plan["source_sha256"].items(): require(digest(ROOT/name) == sha == digest(output/"source"/name), "runtime/archive source changed")
    for section in ("external_source_sha256", "asset_sha256"):
        for name, sha in plan[section].items():
            path = Path(name)
            target = output/("source" if path.is_relative_to(ROOT) else "source_external")/path.relative_to(ROOT if path.is_relative_to(ROOT) else CARLA_ROOT)
            require(digest(path) == sha == digest(target), "source/API/asset archive changed")


def pixel_audit(episode, cameras):
    """HH_261001 - Require every original BGRA hash and source-time-bound saved JPEG byte identity, not flags alone."""
    records = jsonl(episode/"actual_camera_pixels.jsonl")
    keys = [(row["frame"], row["camera"]) for row in records]
    expected = {(row["frame"], name) for row in cameras for name in worker_module.base.MODEL_CAMERA_ORDER}
    require(len(records) == len(expected) and len(set(keys)) == len(keys) and set(keys) == expected, "camera source pixel denominator differs")
    by_key = dict(zip(keys, records))
    for bundle in cameras:
        require(set(bundle["original_BGRA_source_sha256"]) == set(bundle["saved_JPEG_sha256"]) == set(worker_module.base.MODEL_CAMERA_ORDER), "every source/JPEG hash required")
        for name in worker_module.base.MODEL_CAMERA_ORDER:
            row = by_key[(bundle["frame"], name)]
            require(re.fullmatch(r"[0-9a-f]{64}", row["original_BGRA_sha256"]) is not None
                and row["original_BGRA_sha256"] == bundle["original_BGRA_source_sha256"][name]
                and row["saved_JPEG_sha256"] == bundle["saved_JPEG_sha256"][name]
                and row["saved_JPEG_relative_path"] == bundle["images"][name]
                and row["timestamp"] == bundle["source_timestamps"][name] and row["encoding"] == "original_carla_bgra8"
                and row["JPEG_encoder_unchanged"] is True and row["pose_or_image_post_transformed"] is False,
                "actual camera source identity differs")
            relative = Path(row["saved_JPEG_relative_path"])
            require(not relative.is_absolute() and ".." not in relative.parts and digest(episode/relative) == row["saved_JPEG_sha256"], "saved actual JPEG bytes changed")
    return dict(status="PASS", actual_camera_source_count=len(records), complete_bundle_count=len(cameras),
        journal_sha256=digest(episode/"actual_camera_pixels.jsonl"), exact_BGRA_payload_archived=False,
        original_BGRA_SHA_recorded_before_encoding=True, actual_saved_JPEG_bytes_verified=True)


def body_report(output, plan, episode, states, receipts, manifest):
    require(bool(states) and digest(episode/"active_map_opendrive.xodr") == ASSETS[XODR]
        and digest(episode/"route.json") == ASSETS[task.ROUTE], "body source geometry differs")
    inputs = {str(episode/name): digest(episode/name) for name in ("manifest.json", "states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl", "route.json", "active_map_opendrive.xodr", "actual_teacher_control.jsonl", "actual_camera_pixels.jsonl")}
    actors = {row["actor_id"] for row in receipts}; require(len(actors) == 1, "one exact ACK actor required")
    saved = list(sys.path)
    try:
        sys.path.insert(0, str(EGG)); import carla
        require(str(carla.__file__).startswith(str(EGG)+"/"), "foreign offline map API")
        rows, census = geometry.body_census(manifest, read_json(episode/"route.json"), states,
            (episode/"active_map_opendrive.xodr").read_bytes(), next(iter(actors)))
    finally: sys.path[:] = saved
    require(len(rows) == len(states) and len(census) == 4, "all captured native boxes/sweeps required")
    passed = all(row["status"] == "PASS" and row["assessed_count"] == len(states)-(1 if name.startswith("previous_sweep") else 0)
        for name, row in census.items())
    require(all(digest(name) == sha for name, sha in inputs.items()), "raw body source changed during audit")
    closure_check(output, plan)
    derived = output/"derived_body"; derived.mkdir(exist_ok=False)
    with gzip.open(derived/"all_native_bodies_and_sweeps.jsonl.gz", "xt") as stream:
        for row in rows: stream.write(json.dumps(row, allow_nan=False)+"\n")
    report = dict(status="PASS" if passed else "FAIL", all_native_body_and_swept_clearance_passed=passed,
        full_body_denominator=len(states), full_sweep_denominator=len(states)-1, census=census, input_sha256=inputs,
        ten_centimeter_inset_preserved=True, physical_continuous_time_proven=False, training_data_approved=False)
    write_new(derived/"summary.json", report)
    return report


def diagnostic_report(output, plan, owner):
    episode = output/"episode"
    if not episode.is_dir(): episode = output/"episode.partial"
    report = dict(schema=SCHEMA+".teacher_outcome", case_id=plan["case_id"], split=plan["split"], original_owner=owner,
        status="FAIL_RETAINED", actual_teacher_capture_passed=False, ownership_passed=False, bounded_capture_complete=False, training_data_approved=False,
        learned_control=False, model_promoted=False, vehicle_control_approved=False, future_labels_qualified=False,
        holdout_training_or_checkpoint_selection_allowed=False)
    try:
        manifest = read_json(episode/"manifest.json")
        states, cameras, receipts = (jsonl(episode/name) for name in ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl"))
        report.update(native_status=manifest.get("status"), native_error=manifest.get("error"), native_state_count=len(states), camera_bundle_count=len(cameras))
        ownership = not owner["cleanup_errors"] and owner["source_hashes_match"] and owner["external_source_hashes_match"]
        audit = lifecycle.independent_audit
        transport = audit.analyze_transport(states, receipts, manifest["capture_contract"]["control_transport"]["bootstrap_observation"], wheelbase=2.85)
        camera = audit.analyze_camera_alignment(states, cameras)
        native_chain = bool(states) and all(b["frame"] == a["frame"]+1 and abs(b["timestamp"]-a["timestamp"]-.05) <= 1e-6 for a, b in zip(states, states[1:]))
        camera_chain = bool(cameras) and all(b["frame"] == a["frame"]+2 and abs(b["timestamp"]-a["timestamp"]-.1) <= 1e-6 for a, b in zip(cameras, cameras[1:]))
        route = read_json(episode/"route.json")
        pixels = pixel_audit(episode, cameras)
        replay = task.reconstruct(plan, states, cameras, route["route"])
        tail = sum(row["capture_phase"] == "stationary_tail" for row in states)
        moving = replay.result(tail_states=tail, goal_reached=manifest.get("result", {}).get("goal_reached", False))
        require(moving == manifest["result"]["moving_task"], "pure fixed membership result differs from original")
        body = body_report(output, plan, episode, states, receipts, manifest) if ownership and states else dict(status="NOT_PROVEN", all_native_body_and_swept_clearance_passed=False)
        spawn = manifest["capture_contract"]["actual_signed_spawn"]
        expected_center, expected_rear = task.perturb_actor_center(spawn["nominal_actor_center_carla"], wheelbase_m=2.85,
            signed_lateral_offset_ros_m=plan["signed_lateral_offset_ros_m"], signed_yaw_offset_ros_deg=plan["signed_yaw_offset_ros_deg"])
        expected_actual = dict(expected_center, z=expected_center["z"]+.5)
        spawn_correct = spawn["signed_lateral_offset_ros_m"] == plan["signed_lateral_offset_ros_m"] \
            and spawn["signed_yaw_offset_ros_deg"] == plan["signed_yaw_offset_ros_deg"] \
            and spawn["applied_once_before_sensors_and_ticks"] is True and spawn["pixels_or_observed_poses_post_transformed"] is False \
            and manifest["result"]["actual_spawn_perturbation_call_count"] == 1 \
            and spawn["perturbed_actor_center_carla"] == expected_center and spawn["perturbed_rear_reference_carla"] == expected_rear \
            and spawn["actual_requested_spawn_carla"] == expected_actual
        passed = ownership and manifest.get("status") == "complete" and moving["task_complete"] and transport["status"] == camera["status"] == "PASS" \
            and native_chain and camera_chain and len(states) == 903 and len(cameras) == 452 \
            and spawn_correct and body["all_native_body_and_swept_clearance_passed"]
        report.update(status="PASS_ACTUAL_TEACHER_CAPTURE_PENDING_FUTURE_QA" if passed else "FAIL_RETAINED", actual_teacher_capture_passed=passed,
            ownership_passed=ownership, actual_spawn_identity_passed=spawn_correct, independent_transport_audit=transport,
            bounded_capture_complete=manifest.get("status") == "complete" and moving["task_complete"] and len(states) == 903 and len(cameras) == 452,
            independent_camera_alignment_audit=camera, native_chain_passed=native_chain, camera_chain_passed=camera_chain,
            fixed_membership=moving, full_capture_body_clearance=body, actual_labels_not_route_substitution=True,
            actual_camera_source_pixel_audit=pixels,
            heldout_case_reserved=plan["split"] != "train", failed_tail_relabelled_as_good=False)
        closure_check(output, plan)
    except Exception as error:
        report.update(status="FAIL_RETAINED", actual_teacher_capture_passed=False, error=type(error).__name__+": "+str(error))
    return report


def declare_batch(run_id):
    output, path = task.paths(run_id)
    require(not output.exists() and not path.exists(), "fresh create-only recovery batch required")
    disk = disk_check(); output.mkdir(parents=True, exist_ok=False)
    cases = []
    for case_id in CASE_IDS:
        plan = build_plan(run_id, case_id)
        _, case_path = task.paths(run_id, case_id)
        write_new(case_path, plan)
        cases.append(dict(case_id=case_id, split=plan["split"], declaration_path=str(case_path), declaration_sha256=digest(case_path)))
    batch = dict(schema=SCHEMA, run_id=run_id, declared_at_utc=datetime.now(timezone.utc).isoformat(), cases=cases,
        train_case_count=9, holdout_case_count=2, full_signed_offset_yaw_factorial=True, actual_images_consistent_with_actual_pose=True,
        all_cases_declared_before_first_launch=True, attempts_per_case=1, automatic_retry=False, source_snapshot_before_server_required=True,
        dataset_split_unseen_seed_or_initialization_not_independent_map=True, holdout_training_or_checkpoint_selection_allowed=False,
        fixed_selection_not_after_results=True, failed_attempts_retained=True, preflight_disk=disk, training_data_approved=False)
    write_new(path, batch)
    return dict(batch_plan=str(path), batch_plan_sha256=digest(path), cases=cases, simulator_started=False)


def verify_batch(run_id, expected):
    _, path = task.paths(run_id)
    require(isinstance(expected, str) and digest(path) == expected, "exact all-case batch SHA required")
    batch = read_json(path)
    require(batch["schema"] == SCHEMA and batch["run_id"] == run_id and batch["train_case_count"] == 9 and batch["holdout_case_count"] == 2
        and [row["case_id"] for row in batch["cases"]] == list(CASE_IDS) and batch["automatic_retry"] is False
        and batch["holdout_training_or_checkpoint_selection_allowed"] is False, "batch contract differs")
    for row in batch["cases"]: verify_case(run_id, row["case_id"], row["declaration_sha256"])
    return batch


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "declare", "preflight", "execute", "worker", "summary"), default="contract")
    parser.add_argument("--run-id", default="v1")
    parser.add_argument("--case-id", choices=CASE_IDS, default="train_04")
    parser.add_argument("--plan-sha256")
    parser.add_argument("--case-plan-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare": result = declare_batch(args.run_id)
    elif args.mode == "contract": result = task.fixed_contract(args.run_id, args.case_id)
    elif args.mode == "worker": return worker(args.run_id, args.case_id, args.case_plan_sha256)
    else:
        batch = verify_batch(args.run_id, args.plan_sha256)
        if args.mode == "summary":
            results = []
            for spec in batch["cases"]:
                output, _ = task.paths(args.run_id, spec["case_id"])
                result_path = output/"diagnostic_report.json"
                results.append(read_json(result_path) if result_path.is_file() else dict(case_id=spec["case_id"], status="NOT_EXECUTED", actual_teacher_capture_passed=False))
            result = dict(schema=SCHEMA+".summary", all_eleven_attempts=results, pass_count=sum(row["actual_teacher_capture_passed"] for row in results),
                failed_or_unexecuted_count=sum(not row["actual_teacher_capture_passed"] for row in results), training_data_approved=False,
                no_best_case_selection=True, all_failures_retained=True, holdout_training_allowed=False)
        else:
            case_sha = next(row["declaration_sha256"] for row in batch["cases"] if row["case_id"] == args.case_id)
            with scoped_case(args.case_id):
                if args.mode == "execute": return lifecycle.execute(args.run_id, case_sha)
                _, result = preflight(args.run_id, case_sha)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__": raise SystemExit(main())
