"""HH_261001 - Prospectively own one fixed final-3000 temporal Town07 screen; execution is explicit and isolated."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
from datetime import datetime, timezone
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import torch

from . import collect_temporal_v2_owned_camera_research_20261001 as live
from . import run_owned_town13_route_catalog as lifecycle
from .run_owned_ctrack_isolated_probe_20261001 import (
    all_processes, descendants, listening_socket_owned, ports_free, process_record)
from .run_owned_ah_town07_research_v2_20261001 import classify_bounded_capture
from . import audit_carla_acknowledged_control as independent_audit
from portable_e2e.contract import DEFAULT_CONTRACT_PATH
from portable_e2e.temporal_simulation_research_v2_20261001 import load_npz


ROOT = Path(__file__).resolve().parents[2]
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
XODR = CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town07.xodr"
ROUTE = ROOT / "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json"
MAPPING = ROOT / "autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml"
CALIBRATION = ROOT / "autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
EXPORT = ROOT / "storage/experiments/training/2026-10-01/temporal_v2_low_speed_joint_v2/exports/step_003000"
WEIGHTS, METADATA = EXPORT / "model_state.npz", EXPORT / "metadata.json"
CHECKPOINT_SHA = "7902216449dd8e7a601d63128a38773a5f3bf59c1660a7684b7f2d79791659b6"
WEIGHTS_SHA = "768c10aefef21b46587d34da79896dcf1d6acf0a7373eab27045d6c4e2460c90"
METADATA_SHA = "eb74e50c5150b374aaa21bced41fd703507ed16c9fa863fd4a06e9a5896590b7"
MODULE = "scripts.e2e.research_temporal_v2_owned_camera_town07_20261001"
SCHEMA = "portable_e2e.owned_temporal_v2_town07_prospective_screen.20261001.v1"
PORT, FLOOR, CLOSING_RESERVE = 2440, 5*1024**3, 16*1024**2
EPISODE_QUOTA, SAMPLE_QUOTA, ARM_RESERVE, SHARED_RESERVE = 512*1024**2, 128*1024**2, 768*1024**2, 64*1024**2
PLAN_TOKEN = "EXACT_DECLARATION_SHA256_AT_EXECUTION"
FROZEN_GENERIC = {
    "learning/portable_e2e/temporal_simulation_research_v2_20261001.py": "772275dd9e452c70bd9d5364bbacb32d8bf08438ce8bf3526f7ebcf3d64884dc",
    "tools/e2e/collect_temporal_v2_simulation_research_20261001.py": "e1978a21f77b5f4c7f9bdcadfbaf8157ab1baad07c88cc83c1aecef6ea8356a0",
}
INHERITED_FREEZE = ROOT / "docs/assets/validation/2026-10-01/selector_phase_ab/DIRECTIONAL_REPLAY_SOURCE_FREEZE.json"
INHERITED_FREEZE_SHA = "d2a8fae7537b45a18d8bc4e29c6a8ae7da9f2d9d61dc535b35a3480b2d071ed2"
AGENT_FILES = tuple(CARLA_ROOT / "PythonAPI/carla/agents" / name for name in (
    "__init__.py", "navigation/__init__.py", "tools/__init__.py", "navigation/basic_agent.py",
    "navigation/local_planner.py", "navigation/controller.py", "navigation/global_route_planner.py", "tools/misc.py"))
ASSETS = {
    WEIGHTS: WEIGHTS_SHA, METADATA: METADATA_SHA,
    ROUTE: "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9",
    MAPPING: "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
    CALIBRATION: "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea",
    EGG: "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a",
    XODR: "be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf",
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular non-symlink file required: " + str(path))
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    digest(path)
    require(Path(path).stat().st_size <= 4*1024**2, "bounded JSON required")
    return json.loads(Path(path).read_text())


def write_new(path, value):
    require(not Path(path).is_symlink(), "create-only JSON required")
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def paths(run_id):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "explicit safe version run ID required")
    output = ROOT / ("storage/experiments/validation/2026-10-01/temporal_v2_owned_camera_town07_screen_" + run_id)
    return output, output.with_name(output.name + "_plan.json")


def environment():
    env = os.environ.copy()
    env.update(CARLA_ROOT=str(CARLA_ROOT), CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="1",
               OPENBLAS_NUM_THREADS="1", PYTHONPATH=os.pathsep.join(map(str,
                   (ROOT / "learning", ROOT, EGG, CARLA_ROOT / "PythonAPI", CARLA_ROOT / "PythonAPI/carla"))))
    return env


def capture_argv(run_id, longitudinal_pi, declaration_sha=PLAN_TOKEN):
    output, _ = paths(run_id)
    result = ["--research-weights", str(WEIGHTS), "--research-metadata", str(METADATA),
        "--research-weights-sha256", WEIGHTS_SHA, "--research-metadata-sha256", METADATA_SHA,
        "--research-source-sha256", declaration_sha]
    if longitudinal_pi: result.append("--research-longitudinal-pi")
    return result + [str(output / "episode"), str(ROUTE), "--host", "127.0.0.1", "--port", str(PORT),
        "--control-transport", "acknowledged_batch", "--stationary-warmup-sec", "1.2",
        "--max-duration-sec", "20", "--target-speed-kmh", "9", "--seed", "0", "--weather", "ClearNoon",
        "--mapping", str(MAPPING), "--calibration", str(CALIBRATION), "--spawn-z-offset-m", "0.5",
        "--wheelbase-m", "2.85",
        "--expected-live-xodr-sha256", ASSETS[XODR], "--goal-stop-profile", "disabled"]


def fixed_contract(run_id, longitudinal_pi):
    output, plan_path = paths(run_id)
    return dict(schema=SCHEMA, run_id=run_id, output=str(output), declaration_path=str(plan_path),
        source_checkpoint_sha256=CHECKPOINT_SHA, optimizer_updates=3000, known_total_optimizer_updates=3200,
        checkpoint_selection="prospectively fixed final3000 only; no observed-best selection",
        parent_checkpoint_updates=200, one_episode_specialized_research=True, independent_validation=False,
        candidate_count=1, runtime_intent="explicit_cruise_GO_not_head", stop_intent_trained=False,
        immutable_camera_history=True, normalized_tensor_SHA_at_every_inference=True,
        exact_normalized_tensor_archive_retained=False, exact_tensor_sampling="none; sample allowance unused",
        original_BGRA_camera_SHA_preserved=True, all_JPEG_native_ACK_raw_predictions_retained=True,
        longitudinal_pi=longitudinal_pi, feedforward_added=False, throttle_floor_added=False,
        original_decoder_gate_tracker_math_unchanged=True, model_speed_cap_mps=2.5,
        native_actor_planar_speed_maximum_mps=2.7, route_CTE_emergency_limit_m=1.,
        policy_deadline_seconds=.1, policy_deadline_relaxed=False, no_teacher_fallback_after_handoff=True,
        startup="two adjacent real100ms parity prefills, then fresh-frame handoff after vx>.5 and10 observed ego rows",
        source_byte_ownership_timing="outside policy100ms inside observed bundle-delivery to ACK",
        source_and_normalized_tensor_SHA_timing="inside unchanged policy100ms",
        town="Town07", scenario="straight", weather="ClearNoon", vehicle="vehicle.toyota.prius", seed=0,
        port_triplet=[2440,2441,2442], physics_hz=20, capture_hz=10,
        total_capture_seconds=20, stationary_warmup_seconds=1.2, teacher_target_kph=9,
        spawn_z_offset_m=.5, recorded_base_link_wheelbase_m=2.85,
        control_transport="acknowledged_batch", goal_stop_profile="disabled",
        simulator_load_world_permitted=False, attempts=1, automatic_retry=False,
        output_episode_quota_bytes=EPISODE_QUOTA, sampled_exact_input_quota_bytes=SAMPLE_QUOTA,
        closing_reserve_bytes=CLOSING_RESERVE, free_disk_floor_bytes=FLOOR,
        reserved_attempt_bytes=ARM_RESERVE, shared_safety_reserve_bytes=SHARED_RESERVE,
        shared_runtime_lease=str(lifecycle.LOCK), natural_original_TCP_UDP_bind_release_wait_maximum_s=75.,
        original_port_bind_checks_unchanged=True, no_SO_REUSE_or_foreign_process_termination=True,
        capture_argv=capture_argv(run_id, longitudinal_pi),
        server_command=["bash", "tools/e2e/run_carla_map.sh", "Town07", "--port", "2440", "--quality", "Epic",
                        "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        worker_command=[sys.executable, "-B", str(ROOT / "tools/workspace_python.py"), "-m", MODULE, "--",
                        "--mode", "worker", "--run-id", run_id, "--plan-sha256", PLAN_TOKEN],
        owner_wall_timeout_s=300.,
        precursor_motion_screen=dict(net_learned_progress_minimum_m=30., unintended_stationary_dwell_strictly_below_s=1.,
            stationary_threshold_mps=.05, cte_p95_maximum_m=.35, cte_absolute_maximum_m=.75,
            collisions_maximum=0, lane_invasions_maximum=0, rejected_learned_proposals_maximum=0,
            interval="ACK-matched learned-controlled observations after disclosed startup, before cleanup",
            body_and_swept_lane_clearance="separate independently evaluated >=0.1m required before screen success"),
        sensor_and_ACK_screen=dict(native_state_count=400, camera_bundle_count=200,
            native_frame_stride=1, camera_frame_stride=2, native_delta_s=.05, camera_delta_s=.1,
            exact_six_camera_and_native_timestamp_match=True, failed_ACK_count_maximum=0,
            ACK_matched_next_native_control_evidence_required=True,
            terminal_ACK_without_next_native_observation_not_counted_as_effect=True),
        observed_wall_latency_screen=dict(fresh_learned_bundle_delivery_to_ACK_p95_maximum_s=.1,
            fresh_learned_bundle_delivery_to_ACK_maximum_s=.1,
            sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False,
            simulation_Hz_is_not_wall_throughput=True,
            throughput="first-to-last observed camera monotonic delivery span vs same simulation timestamp span"),
        full_route_or_normal_stop_approved=False, full_RTE01_approved=False,
        model_promoted=False, vehicle_control_approved=False, training_data_approved=False,
        failed_attempts_retained=True)


def module_path(name):
    prefixes={"scripts":ROOT/"tools","portable_e2e":ROOT/"learning/portable_e2e","tools":ROOT/"tools"}
    first,*rest=name.split(".")
    if first not in prefixes: return None
    base=prefixes[first].joinpath(*rest)
    for path in (base.with_suffix(".py"),base/"__init__.py"):
        if path.is_file(): return path
    return None


def module_name(path):
    relative=path.relative_to(ROOT)
    if relative.parts[0]=="learning": parts=relative.parts[1:]
    elif relative.parts[0]=="tools": parts=("scripts",*relative.parts[1:])
    else: return None
    names=list(parts);names[-1]=Path(names[-1]).stem
    return ".".join(names[:-1] if names[-1]=="__init__" else names)


def imported_source_closure(seeds):
    """HH_261001 - Follow explicit Python imports, including function-local imports, without sys.modules or a glob."""
    visited=set();pending=list(seeds)
    while pending:
        source=pending.pop()
        if source in visited: continue
        require(source.resolve().is_relative_to(ROOT) and not source.is_symlink(),"canonical imported source required")
        visited.add(source)
        if source.suffix!=".py": continue
        name=module_name(source)
        package=name if source.name=="__init__.py" else name.rpartition(".")[0]
        for node in ast.walk(ast.parse(source.read_text(),filename=str(source))):
            imports=[]
            if isinstance(node,ast.Import): imports=[item.name for item in node.names]
            elif isinstance(node,ast.ImportFrom):
                if node.level:
                    parts=package.split(".")
                    prefix=".".join(parts[:len(parts)-node.level+1])
                    target=".".join(part for part in (prefix,node.module) if part)
                else: target=node.module or ""
                imports=[target]+[target+"."+item.name for item in node.names if item.name!="*"]
            for imported in imports:
                parts=imported.split(".")
                for length in range(1,len(parts)+1):
                    candidate=module_path(".".join(parts[:length]))
                    if candidate is not None and candidate not in visited: pending.append(candidate)
    return visited


def runtime_sources():
    """HH_261001 - Pin deterministic imported closure plus immutable training dependencies, never a source directory glob."""
    imported = imported_source_closure((Path(__file__).resolve(),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"))
    imported.update((ROOT/"tools/e2e/run_carla_map.sh",ROOT/"tools/e2e/env.sh",DEFAULT_CONTRACT_PATH,INHERITED_FREEZE))
    metadata = read_json(METADATA)
    require(digest(METADATA) == METADATA_SHA and metadata["checkpoint_sha256"] == CHECKPOINT_SHA
            and metadata["optimizer_updates"] == 3000, "final3000 frozen metadata differs")
    for name, expected in metadata["source_sha256"].items():
        source = ROOT / name
        require(source.resolve().is_relative_to(ROOT) and digest(source) == expected,
                "fixed training dependency changed: " + name)
        imported.add(source)
    pins={str(path.relative_to(ROOT)):digest(path) for path in sorted(imported)}
    require(all(pins.get(name)==sha for name,sha in FROZEN_GENERIC.items()),"frozen generic V2 source changed")
    require(digest(INHERITED_FREEZE)==INHERITED_FREEZE_SHA,"inherited source freeze changed")
    inherited=read_json(INHERITED_FREEZE)["source_sha256"]
    for name,sha in pins.items():
        if name in inherited: require(sha==inherited[name],"inherited frozen dependency changed: "+name)
    return pins


def external_sources():
    files = AGENT_FILES + (CARLA_ROOT / "CarlaUE4.sh", CARLA_ROOT / "CarlaUE4/Config/DefaultEngine.ini",
                           CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/Town07.umap")
    return {str(path): digest(path) for path in files}


def build_plan(run_id, longitudinal_pi):
    output, plan_path = paths(run_id)
    require(type(longitudinal_pi) is bool and not output.exists() and not output.is_symlink()
            and not plan_path.exists() and not plan_path.is_symlink(), "fresh prospective version required")
    require(all(digest(path) == expected for path, expected in ASSETS.items()), "fixed assets changed")
    return dict(fixed_contract(run_id, longitudinal_pi), declared_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=runtime_sources(), external_source_sha256=external_sources(),
        asset_sha256={str(path): sha for path, sha in ASSETS.items()},
        source_closure_policy="recursive explicit Python imports including lazy imports plus fixed metadata pins; no directory glob")


def verify_plan(run_id, expected):
    _, path = paths(run_id)
    require(isinstance(expected,str) and re.fullmatch(r"[0-9a-f]{64}",expected), "exact declaration SHA required")
    require(digest(path) == expected, "create-only declaration bytes differ")
    plan = read_json(path)
    fixed = fixed_contract(run_id, plan.get("longitudinal_pi"))
    require(type(plan.get("longitudinal_pi")) is bool and set(plan) == set(fixed) | {
        "declared_at_utc","source_sha256","external_source_sha256","asset_sha256","source_closure_policy"},
        "declaration shape differs")
    require(all(plan.get(key) == value for key,value in fixed.items()), "prospective fixed contract differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0
            and stamp <= datetime.now(timezone.utc), "prior UTC declaration required")
    require(plan["source_sha256"] == runtime_sources() and plan["external_source_sha256"] == external_sources()
            and plan["asset_sha256"] == {str(path):sha for path,sha in ASSETS.items()}, "source closure changed")
    require(all(digest(name) == sha for name,sha in plan["asset_sha256"].items()), "pinned asset changed")
    return plan


def disk_check(free=None):
    value = shutil.disk_usage(ROOT).free if free is None else free
    require(type(value) is int and value >= FLOOR+ARM_RESERVE+SHARED_RESERVE, "projected disk reserve insufficient")
    return dict(free_bytes=value, required_bytes=FLOOR+ARM_RESERVE+SHARED_RESERVE,
                episode_quota_bytes=EPISODE_QUOTA, sampled_exact_input_quota_bytes=SAMPLE_QUOTA)


def preflight(run_id, expected):
    plan = verify_plan(run_id, expected)
    require(sys.version_info[:2] == (3,10), "source-pinned CARLA API needs Python3.10")
    output,_ = paths(run_id)
    require(not output.exists() and not output.is_symlink(), "fresh execution output required")
    lifecycle._assert_no_other_carla(); ports_free(PORT)
    disk = disk_check()
    check = subprocess.run([sys.executable,"-B","-c",
        "import carla,agents,torch,numpy,PIL; print(carla.__file__); print(agents.__file__); print(torch.__version__)"],
        cwd=ROOT, env=environment(), text=True, capture_output=True, timeout=30, check=True)
    require(check.stdout.startswith(str(EGG)+"/"), "foreign CARLA API import")
    imported=check.stdout.strip().splitlines()
    require(len(imported)==3 and imported[1]==str(CARLA_ROOT/"PythonAPI/carla/agents/__init__.py"),
            "foreign CARLA agent import")
    args = live.frozen.parse_args(capture_argv(run_id,plan["longitudinal_pi"],expected))
    live.frozen.validate_capture(args,live.frozen.collector.load_route(ROUTE))
    torch.set_num_threads(1)
    model = load_npz(WEIGHTS,METADATA,weights_sha256=WEIGHTS_SHA,metadata_sha256=METADATA_SHA)
    require(model._research_temporal_checkpoint_sha256 == CHECKPOINT_SHA, "loaded checkpoint differs")
    return plan,dict(status="PREFLIGHT_PASS_NO_SIMULATOR",declaration_sha256=expected,disk=disk,
        imported_API_and_torch=check.stdout.strip().splitlines(), learned_control_executed=False)


def wait_for_original_bind_release(*, timeout_s=75., pause=time.sleep, clock=time.monotonic, probe=ports_free):
    """HH_261001 - Wait naturally for TCP quiescence while retaining original exclusive TCP/UDP checks."""
    require(timeout_s == 75., "fixed75s release wait required")
    began = clock(); failures = []
    while True:
        try:
            probe(PORT)
            return dict(original_TCP_UDP_bind_checks_passed=True,wait_seconds=clock()-began,
                        bind_failures_retained=failures, SO_REUSE_used=False)
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
            failures.append(str(error))
            if clock()-began >= timeout_s:
                raise RuntimeError("original TCP/UDP bind release failed after75s: "+str(error))
            pause(min(.25, timeout_s-(clock()-began)))


def directory_bytes(directory):
    root = Path(directory)
    if not root.exists(): return 0
    require(root.is_dir() and not root.is_symlink(),"owned episode directory required")
    total = 0
    for path in root.rglob("*"):
        require(not path.is_symlink(),"symlink forbidden in episode quota")
        if path.is_file(): total += path.stat().st_size
    return total


@contextmanager
def episode_quota(args):
    """HH_261001 - Reserve bounded next-camera writes and cleanup before each bundle; retain failures unchanged."""
    collector = live.frozen.collector
    original = collector.exact_camera_bundle
    partial = args.output.with_name(args.output.name+".partial")
    def guarded(*positional,**kwargs):
        used = directory_bytes(partial)
        free = shutil.disk_usage(ROOT).free
        require(used+12*1024**2+CLOSING_RESERVE <= EPISODE_QUOTA,
                "fixed512MiB episode quota closing reserve reached")
        require(free >= FLOOR+CLOSING_RESERVE,"fixed disk floor/closing reserve reached")
        return original(*positional,**kwargs)
    collector.exact_camera_bundle = guarded
    try: yield
    finally: collector.exact_camera_bundle = original


def assert_owned_worker(plan, expected):
    """HH_261001 - A private worker cannot attach to a foreign simulator or start without the owning parent."""
    output = Path(plan["output"])
    started = read_json(output/"owner_started.json")
    require(started["declaration_sha256"] == expected and started["owner"]["pid"] == os.getppid(),
            "worker lacks exact owning parent/declaration")
    for key in ("owner","server","socket_owner"):
        actual = process_record(started[key]["pid"])
        require(actual and actual["start_ticks"] == started[key]["start_ticks"],"owner process generation differs")
    shipping = started["socket_owner"]
    require(shipping["pgid"] == started["server"]["pid"] and listening_socket_owned(shipping["pid"],PORT),
            "RPC socket not held by exact owned shipping group")


def worker(run_id, expected):
    plan = verify_plan(run_id,expected)
    assert_owned_worker(plan,expected)
    args = live.frozen.parse_args(capture_argv(run_id,plan["longitudinal_pi"],expected))
    torch.set_num_threads(1)
    model = load_npz(WEIGHTS,METADATA,weights_sha256=WEIGHTS_SHA,metadata_sha256=METADATA_SHA)
    # HH_261001 - Quota sits inside the actual activated observation hooks, not around an unrelated collector.
    with live.activated(model,args),episode_quota(args):
        result = live.frozen.collector.run(args)
    print(result)
    return 0


def archive_sources(output, plan):
    for name,sha in plan["source_sha256"].items():
        source = ROOT/name
        require(digest(source) == sha,"pre-archive source changed")
        target = output/"source"/name
        target.parent.mkdir(parents=True,exist_ok=True)
        with source.open("rb") as src,target.open("xb") as dest: shutil.copyfileobj(src,dest)
        require(digest(target) == sha,"archived source differs")
    for name,sha in plan["external_source_sha256"].items():
        source=Path(name)
        require(source.is_relative_to(CARLA_ROOT) and digest(source)==sha,"external pre-archive source differs")
        target=output/"source_external"/source.relative_to(CARLA_ROOT)
        target.parent.mkdir(parents=True,exist_ok=True)
        with source.open("rb") as src,target.open("xb") as dest: shutil.copyfileobj(src,dest)
        require(digest(target)==sha,"archived external source differs")
    # HH_261001 - Archive declared route/rig/export assets too; offline native projection never reads unarchived geometry.
    for name,sha in plan["asset_sha256"].items():
        source=Path(name)
        if source.is_relative_to(ROOT): target=output/"source"/source.relative_to(ROOT)
        else:
            require(source.is_relative_to(CARLA_ROOT),"declared asset outside fixed ownership roots")
            target=output/"source_external"/source.relative_to(CARLA_ROOT)
        require(digest(source)==sha,"asset changed before archive")
        if target.exists(): require(digest(target)==sha,"existing source/asset archive conflicts")
        else:
            target.parent.mkdir(parents=True,exist_ok=True)
            with source.open("rb") as src,target.open("xb") as dest: shutil.copyfileobj(src,dest)
        require(digest(target)==sha,"archived asset differs")


def execute(run_id, expected):
    output,plan_path = paths(run_id)
    with lifecycle._workspace_lock():
        plan,_ = preflight(run_id,expected)
        output.mkdir(parents=True,exist_ok=False)
        archive_sources(output,plan)
        shutil.copyfile(plan_path,output/"declaration.json")
        require(digest(output/"declaration.json") == expected,"archived declaration differs")
        server=worker_process=None
        identities={}; error=None; cleanup_errors=[]; worker_code=None; release=None; handlers={}
        began=time.monotonic()
        def interrupted(signum,_frame): raise RuntimeError("owner interrupted by signal "+str(signum))
        try:
            for signum in (signal.SIGINT,signal.SIGTERM): handlers[signum]=signal.signal(signum,interrupted)
            with (output/"server.log").open("xb") as log:
                server=subprocess.Popen(plan["server_command"],cwd=ROOT,env=environment(),stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
            identities["server"]=process_record(server.pid)
            deadline=time.monotonic()+130
            socket_owner=None
            while time.monotonic()<deadline:
                require(server.poll() is None,"owned packaged server exited before readiness")
                if b"CARLA_READY " in (output/"server.log").read_bytes():
                    owners=[row for row in descendants(server.pid,all_processes()) if row["pgid"]==server.pid
                        and any("CarlaUE4-Linux-Shipping" in item for item in row["argv"])
                        and listening_socket_owned(row["pid"],PORT)]
                    require(len(owners)==1,"RPC not uniquely owned by started shipping process")
                    socket_owner=owners[0]; break
                time.sleep(1.)
            require(socket_owner is not None,"owned server readiness timed out")
            write_new(output/"owner_started.json",dict(owner=process_record(os.getpid()),
                server=identities["server"],socket_owner=socket_owner,declaration_sha256=expected))
            command=[expected if item==PLAN_TOKEN else item for item in plan["worker_command"]]
            with (output/"collector.log").open("xb") as log:
                worker_process=subprocess.Popen(command,cwd=ROOT,env=environment(),stdout=log,
                    stderr=subprocess.STDOUT,start_new_session=True)
                identities["worker"]=process_record(worker_process.pid)
                worker_code=worker_process.wait(timeout=plan["owner_wall_timeout_s"])
            if worker_code: error="collector returned "+str(worker_code)+"; raw capture failure retained"
        except BaseException as exc: error=type(exc).__name__+": "+str(exc)
        finally:
            for signum,handler in handlers.items(): signal.signal(signum,handler)
            for key,process in (("worker",worker_process),("server",server)):
                try:
                    if process is not None:
                        actual=process_record(process.pid)
                        require(not actual or actual["start_ticks"]==identities[key]["start_ticks"],
                                "PID generation changed; refusing signal")
                    require(lifecycle._stop_owned_group(process),"owned process group did not stop")
                except Exception as exc: cleanup_errors.append(str(exc))
            try: release=wait_for_original_bind_release()
            except Exception as exc: cleanup_errors.append(str(exc))
            source_match=True; external_match=True
            try:
                source_match=all(digest(ROOT/name)==sha==digest(output/"source"/name)
                                 for name,sha in plan["source_sha256"].items())
                external_match=all(digest(name)==sha==digest(output/"source_external"/Path(name).relative_to(CARLA_ROOT))
                                   for name,sha in plan["external_source_sha256"].items()) \
                    and all(digest(name)==sha==digest(output/("source" if Path(name).is_relative_to(ROOT) else "source_external")/
                            Path(name).relative_to(ROOT if Path(name).is_relative_to(ROOT) else CARLA_ROOT))
                            for name,sha in plan["asset_sha256"].items())
                require(source_match and external_match,"local/archive/external source changed")
                require(digest(output/"declaration.json")==expected==digest(plan_path),"declaration changed")
            except Exception as exc: cleanup_errors.append(str(exc))
            result=dict(schema=SCHEMA+".owner_result",status="OWNER_FAILURE" if cleanup_errors else
                "CAPTURE_RETURNED_NONZERO_RETAINED" if error else "RESEARCH_CAPTURE_COMPLETE",
                declaration_sha256=expected,source_checkpoint_sha256=CHECKPOINT_SHA,
                worker_exit_code=worker_code,error=error,cleanup_errors=cleanup_errors,
                port_release=release,source_hashes_match=source_match,external_source_hashes_match=external_match,
                owner_elapsed_wall_seconds=time.monotonic()-began,
                learned_driving_screen_passed=False,full_RTE01_approved=False,
                vehicle_control_approved=False,model_promoted=False)
            write_new(output/"owner_result.json",result)
        report=diagnostic_report(output,plan,result)
        write_new(output/"diagnostic_report.json",report)
        print(json.dumps(report,indent=2,allow_nan=False))
        return 0 if report["ownership_passed"] and report["bounded_capture_complete"] else 1


def jsonl(path):
    return [json.loads(row) for row in Path(path).read_text().splitlines() if row] if Path(path).is_file() else []


def diagnostic_report(output,plan,owner):
    """HH_261001 - Separate capture/ownership/sensor/ACK/wall metrics from unapproved route/driving features."""
    episode=output/"episode.partial"
    if not episode.is_dir(): episode=output/"episode"
    manifest=read_json(episode/"manifest.json") if (episode/"manifest.json").is_file() else {}
    states,cameras,commands=(jsonl(episode/name) for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl"))
    timing=jsonl(episode/"wall_timing.jsonl")
    summary=live.frozen.wall_timing_summary(timing)
    fresh=[row for row in timing if row["learned_control_proposed"] and row["held_between_camera_frames"] is False]
    bundles=[row["observed_bundle"] for row in fresh if row["observed_bundle"]]
    unique={row["native_frame"]:row for row in bundles}
    observed=sorted(unique.values(),key=lambda row:row["native_frame"])
    wall_span=(observed[-1]["exact_bundle_delivery_monotonic_ns"]-observed[0]["exact_bundle_delivery_monotonic_ns"])/1.e9 if len(observed)>1 else None
    simulated_span=(observed[-1]["source_image_timestamp_simulation_ns"]-observed[0]["source_image_timestamp_simulation_ns"])/1.e9 if len(observed)>1 else None
    values=[row["exact_bundle_delivery_to_ACK_seconds"] for row in fresh if row["exact_bundle_delivery_to_ACK_seconds"] is not None]
    ownership=not owner["cleanup_errors"] and owner["source_hashes_match"] and owner["external_source_hashes_match"]
    native_chain=bool(states) and all(b["frame"]-a["frame"]==1 and abs(b["timestamp"]-a["timestamp"]-.05)<=1.e-6 for a,b in zip(states,states[1:]))
    camera_chain=bool(cameras) and all(b["frame"]-a["frame"]==2 and abs(b["timestamp"]-a["timestamp"]-.1)<=1.e-6
                                     for a,b in zip(cameras,cameras[1:]))
    native_by_frame={row["frame"]:row for row in states}
    exact_camera_match=bool(cameras) and all(row["frame"] in native_by_frame
        and abs(row["timestamp"]-native_by_frame[row["frame"]]["timestamp"])<=1.e-6
        and set(row.get("source_timestamps",{}))==set(live.frozen.collector.MODEL_CAMERA_ORDER)
        and all(abs(value-row["timestamp"])<=1.e-6 for value in row["source_timestamps"].values()) for row in cameras)
    receipts=jsonl(episode/"control_receipts.jsonl")
    try:
        wheelbase=manifest["coordinate_contract"]["wheelbase_m"]
        require(wheelbase==plan["recorded_base_link_wheelbase_m"],"recorded wheelbase differs from predeclared command")
        transport_audit=independent_audit.analyze_transport(states,receipts,
            manifest.get("capture_contract",{}).get("control_transport",{}).get("bootstrap_observation"),wheelbase=wheelbase)
    except Exception as error:
        transport_audit=dict(status="FAIL",error=type(error).__name__+": "+str(error))
    try: camera_audit=independent_audit.analyze_camera_alignment(states,cameras)
    except Exception as error: camera_audit=dict(status="FAIL",error=type(error).__name__+": "+str(error))
    independent_pass=transport_audit["status"]==camera_audit["status"]=="PASS"
    bounded=(classify_bounded_capture(manifest,len(states),len(cameras)) or manifest.get("status")=="complete") \
        and len(states)==400 and len(cameras)==200
    learned_ACK_sequences={row["ack_receipt_sequence"] for row in commands
                           if row.get("application",{}).get("learned_control_applied",False)}
    expected_fresh=[row for row in commands if row.get("learned_control_proposed",False)
                    and row.get("held_between_camera_frames") is False]
    fresh_timing_complete=bool(expected_fresh) and len(fresh)==len(expected_fresh) and len(values)==len(fresh)
    for command in expected_fresh:
        matching=[row for row in fresh if row["ACK_sequence"]==command["ack_receipt_sequence"]
                  and row["inference_frame"]==command["inference_frame"] and row["command_frame"]==command["command_frame"]]
        fresh_timing_complete &= len(matching)==1 and matching[0]["ACK_server_accepted"] is True \
            and matching[0]["source_checkpoint_sha256"]==CHECKPOINT_SHA \
            and matching[0]["stream_source_sha256"]==owner["declaration_sha256"]
    latency_pass=ownership and bounded and native_chain and camera_chain and exact_camera_match and independent_pass \
        and fresh_timing_complete and float(np.percentile(values,95))<=.1 and max(values)<=.1
    first_effect=next((index for index,row in enumerate(states)
        if row.get("control_transport",{}).get("expected_receipt_sequence") in learned_ACK_sequences),None)
    # HH_261001 - Retain every subsequent driving tick, including missing/nonlearned/emergency ticks, not selected successes.
    effects=states[first_effect:] if first_effect is not None else []
    complete_effect_interval=bool(effects) and all(row.get("capture_phase")=="driving"
        and row.get("control_transport",{}).get("status")=="PASS"
        and row["control_transport"]["expected_receipt_sequence"] in learned_ACK_sequences for row in effects)
    effect_sequences={row.get("control_transport",{}).get("expected_receipt_sequence") for row in effects}-{None}
    rejected=sum(row.get("control_source")=="emergency_brake" for row in commands)
    collector_progress=effects[-1]["route_progress_m"]-effects[0]["route_progress_m"] if len(effects)>1 else None
    progress=None;cte=[];projection_error=None
    try:
        archived_route=output/"source"/ROUTE.relative_to(ROOT)
        require(digest(archived_route)==ASSETS[ROUTE],"archived route geometry pin differs")
        projector=live.frozen.collector.RouteProjector(read_json(archived_route))
        native_projections=[];previous=0.
        for row in effects:
            pose=row["actor_snapshot_transform_carla"]
            projection=projector.project(float(pose["x"]),-float(pose["y"]),previous)
            previous=projection.progress_m
            native_projections.append(projection)
        progress=native_projections[-1].progress_m-native_projections[0].progress_m if len(native_projections)>1 else None
        cte=[abs(item.cross_track_error_m) for item in native_projections]
    except Exception as error:
        projection_error=type(error).__name__+": "+str(error)
    dwell=longest_dwell=0.
    for row in effects:
        velocity=row["world_velocity_carla"]
        if float(np.hypot(velocity[0],velocity[1]))<=.05: dwell+=.05
        else: dwell=0.
        longest_dwell=max(longest_dwell,dwell)
    collisions=sum(bool(row.get("collision")) for row in states)
    invasions=sum(bool(row.get("lane_invasion")) for row in states)
    motion_core=ownership and bounded and independent_pass and complete_effect_interval \
        and progress is not None and progress>=30. and longest_dwell<1. \
        and float(np.percentile(cte,95))<=.35 and max(cte)<=.75 and collisions==invasions==rejected==0
    return dict(schema=SCHEMA+".diagnostic_report",ownership_passed=ownership,bounded_capture_complete=bounded,
        original_owner=owner,original_capture_status=manifest.get("status"),original_capture_error=manifest.get("error"),
        source_checkpoint_sha256=CHECKPOINT_SHA,checkpoint_selection=plan["checkpoint_selection"],candidate_count=1,
        native_state_count=len(states),camera_bundle_count=len(cameras),native_state_chain_passed=native_chain,
        camera_frame_chain_passed=camera_chain,exact_camera_native_timestamp_match=exact_camera_match,
        actual_ACK_receipt_count=len(receipts),
        independent_transport_audit=transport_audit,independent_camera_alignment_audit=camera_audit,
        failed_ACK_receipt_count=sum(row.get("status")!="ACKNOWLEDGED" for row in receipts),
        observed_next_native_control_PASS_count=sum(row.get("control_transport",{}).get("status")=="PASS" for row in states),
        learned_proposal_count=sum(row.get("learned_control_proposed",False) for row in commands),
        learned_ACK_count=sum(row.get("application",{}).get("learned_control_applied",False) for row in commands),
        rejected_learned_or_emergency_commands=rejected,
        learned_ACK_matched_native_effect_count=sum(row.get("control_transport",{}).get("status")=="PASS"
            and row["control_transport"]["expected_receipt_sequence"] in learned_ACK_sequences for row in effects) if independent_pass else 0,
        post_handoff_native_effect_interval_count=len(effects),
        complete_post_handoff_native_effect_interval=complete_effect_interval,
        learned_ACKs_without_observed_native_effect_sequences=sorted(learned_ACK_sequences-effect_sequences),
        learned_effect_progress_m=progress,learned_effect_CTE_p95_m=float(np.percentile(cte,95)) if cte else None,
        learned_effect_progress_reference="native CARLA actor API reference independently projected on archived ROS route; not verified COM",
        native_progress_projection_error=projection_error,
        collector_rear_base_route_progress_delta_m=collector_progress,
        learned_effect_CTE_absolute_maximum_m=max(cte) if cte else None,
        learned_effect_maximum_stationary_dwell_s=longest_dwell,
        stationary_speed_reference="native actor snapshot world XY norm; model base-v0 unchanged",
        native_observations_with_collision_events=collisions,native_observations_with_lane_invasion_events=invasions,
        predeclared_motion_core_screen_passed_before_body_swept_clearance=motion_core,
        wall_timing=summary,complete_fresh_learned_wall_receipt_denominator=fresh_timing_complete,
        predeclared_observed_bundle_to_ACK_wall_latency_screen_passed=latency_pass,
        observed_learned_camera_wall_span_seconds=wall_span,matching_simulation_span_seconds=simulated_span,
        measured_simulated_seconds_per_wall_second=simulated_span/wall_span if wall_span and simulated_span else None,
        sensor_callback_entry_observed=False,exposure_to_ACK_wall_latency_proven=False,ten_hz_end_to_end_proven=False,
        exact_normalized_tensor_archive_retained=False,episode_output_bytes=directory_bytes(episode),
        precursor_motion_screen="core computed from ACK-matched native effects; independent body/swept clearance still required",
        learned_driving_screen_passed=False,full_route_or_stop_approved=False,full_RTE01_approved=False,
        vehicle_control_approved=False,model_promoted=False,training_data_approved=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker"),default="contract")
    parser.add_argument("--run-id",default="v1")
    parser.add_argument("--longitudinal-pi",action="store_true")
    parser.add_argument("--plan-sha256")
    args=parser.parse_args(argv)
    if args.mode=="contract":
        print(json.dumps(fixed_contract(args.run_id,args.longitudinal_pi),indent=2));return 0
    if args.mode=="declare":
        plan=build_plan(args.run_id,args.longitudinal_pi);disk_check()
        _,path=paths(args.run_id);path.parent.mkdir(parents=True,exist_ok=True)
        write_new(path,plan)
        print(json.dumps(dict(declaration_path=str(path),declaration_sha256=digest(path),simulator_started=False)));return 0
    if args.mode=="preflight":
        _,result=preflight(args.run_id,args.plan_sha256);print(json.dumps(result,indent=2));return 0
    if args.mode=="execute": return execute(args.run_id,args.plan_sha256)
    return worker(args.run_id,args.plan_sha256)


if __name__=="__main__":
    raise SystemExit(main())
