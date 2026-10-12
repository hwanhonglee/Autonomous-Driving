"""HH_261007 - Seal and admit a new owned, CPU-only recorded-JPEG DDS parity protocol.

The frozen V1 hard DENY is never modified or called. ROOT must separately seal
the prospective plan and an external execution authority. No training, CARLA,
actuator, original-BGRA, V5 terminal-policy, or feature approval is implied.
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
import hashlib
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import resource
import signal
import stat
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.owned_recorded_ROS_shadow.20261007.v5"
HERE = Path(__file__).resolve()
NODE = ROOT / "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v5.py"
OWNER = HERE.with_name("run_owned_temporal_recorded_ros_shadow_20261007_v5.py")
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-07/temporal_recorded_ros_shadow_v5"
PLAN, AUTHORITY = PLAN_DIR / "ROOT_plan.json", PLAN_DIR / "ROOT_execution_authority.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v5"
EPISODE = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1/episode"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
BRIDGE_REPORT = ROOT / "storage/experiments/diagnostics/2026-10-07/owned_dds_info_bridge_v1/result.json"
FLOOR, SHARED, QUOTA, CLOSING = 2*1024**3, 64*1024**2, 128*1024**2, 16*1024**2
FILE_LIMIT, JSON_LIMIT, PLAN_LIMIT = 32*1024**2, 1024**2, 4*1024**2
FROZEN = {
    "learning/portable_e2e/temporal_recorded_ros_bundle_v1_20261005.py": "6fa21ab9447bcf73bdd2fd79b0ab72ddd94b531d3a11459334f754fdfbad7d82",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py": "e0e1b001c3684dfdf1ea41274c4c39b7f9a286d008d37d942aefbe35a3591e7e",
    "learning/portable_e2e/temporal_autoware_shadow_adapter_v1_20261005.py": "7d3cf7b6f4f131c2647921a00b62f08e4b41e30cd6b98f9583ad425267e46a0e",
}
_bundle = None
V1_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v1.py":"7ae85b64b743e461e4723fbfb04fae429522710c84ed9b002a68a6f77ec67f4f",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v1.py":"d38fa56ff49d947f5beb88353c650052ea5d6e98f630cf6f94afb5814bbac404",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v1.py":"d0306f5f7cb6159a4fbea1d64b484e9cd227c3e4233dc6911d5435516bfffc17",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v1.py":"d1803f2ef0b5a474b298b023e766810067eb37d73785437176fe5dcf10c30547",
}
V2_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v2.py":"289099aab00135267a63b927341756fca96cc62f66a164c7fda4b969edbeddef",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v2.py":"caba2e19055be8bbc191a5ff5cc17799dbaa0c707303c26019cb20fcca75066b",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v2.py":"ca92f6ed88ab5c4ae6f0929a92e06fae77ecbf6e051a880874f0dbe76e7342de",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v2.py":"1202fa1f8d3011fe28c47d901a161f0eda0a3bbc184c8b185915f71acea76fb2",
}
V3_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v3.py":"5bbd9d07a9e8eb096caecf47531376d607dec8852b3b7fceea52f1f0858bd0ff",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v3.py":"7babb71d007fb2636e1a6d19ac6e606ec11f4206218087d70f41f185fde30178",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v3.py":"a86012a2d7e6e63086feec2eca8d33366951b8db9005e889bd8cd48cc890e853",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v3.py":"7923aa05f74accf0a9487965f8fa02094fee3535193d80e5a6161906235ee917",
}
V4_PINS={
    "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v4.py":"710cab6280c06d9ad4ccf43acbe688b20e017e7cf6caff340306d460ac1d91b5",
    "tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v4.py":"b0ff7ad6320de3f917cc8c56e4574e0e8b92cb6305cc1073e1d74bbfcd27edcc",
    "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v4.py":"0d8eff86dca6a9a9292c82a4150e986ebac12c7dfee597edbc4d8d05756de49b",
    "learning/tests/test_owned_recorded_ros_shadow_20261007_v4.py":"211d23d56788af1ff7dac05adcb0e24e2b9a5683fdae73bd68bbc5d2e2a897ec",
}


def need(value, message):
    if not value: raise ValueError(message)


def encode(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n").encode()


def digest(path):
    path = Path(path)
    need(path.is_absolute() and path.resolve(strict=True) == path and path.is_file(), "canonical regular input required")
    before = path.stat(); value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""): value.update(block)
    after = path.stat()
    need(all(getattr(after, key) == getattr(before, key) for key in ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")), "input changed while hashing")
    return value.hexdigest()


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def frozen_bundle():
    global _bundle
    if _bundle is not None: return _bundle
    for name, pin in FROZEN.items(): need(digest(ROOT/name) == pin, "frozen prototype changed")
    _bundle=load_module(ROOT/next(iter(FROZEN)), "_hh_frozen_recorded_bundle_261007")
    return _bundle


def source_pins():
    bundle = frozen_bundle()
    names = bundle.project_source_closure()
    names |= {str(p.relative_to(ROOT)) for p in (HERE, OWNER, NODE)}
    names |= {"tools/e2e/prepare_carla_common10_dataset.py", "tools/workspace_python.py", "tools/canonical_workspace.py",
              "learning/tests/test_owned_recorded_ros_shadow_20261007_v5.py",
              "tools/e2e/owned_dds_info_bridge_20261007_v1.c",
              "tools/e2e/probe_owned_dds_info_bridge_20261007_v1.py",
              "learning/tests/test_owned_dds_info_bridge_20261007_v1.py"}
    for name,pin in V1_PINS.items():need(digest(ROOT/name)==pin,"failed V1 source changed")
    names |= set(V1_PINS)
    for name,pin in V2_PINS.items():need(digest(ROOT/name)==pin,"failed V2 source changed")
    names |= set(V2_PINS)
    for name,pin in V3_PINS.items():need(digest(ROOT/name)==pin,"failed V3 source changed")
    names |= set(V3_PINS)
    for name,pin in V4_PINS.items():need(digest(ROOT/name)==pin,"actual V4 source changed")
    names |= set(V4_PINS)
    return {name: digest(ROOT/name) for name in sorted(names)}


def bounded_json(path, pin=None, limit=JSON_LIMIT):
    path = Path(path); need(path.resolve(strict=True) == path and path.is_file() and path.stat().st_size <= limit, "bounded canonical JSON required")
    raw = path.read_bytes()
    if pin is not None: need(hashlib.sha256(raw).hexdigest() == pin, "raw JSON SHA differs")
    return frozen_bundle().decode(raw, limit)


def new_json(path, value, limit=JSON_LIMIT):
    raw = encode(value); need(len(raw) <= limit <= PLAN_LIMIT, "bounded new JSON required")
    with Path(path).open("xb") as stream: stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    return hashlib.sha256(raw).hexdigest()


def budget(startup=False):
    need(shutil.disk_usage(ROOT).free >= FLOOR+SHARED+(QUOTA if startup else CLOSING), "separate CPU protocol reserve denied; GPU reserve is unchanged")
    total = 0
    for directory in (OUTPUT, PLAN_DIR):
        if not directory.exists(): continue
        for path in directory.rglob("*"):
            mode = path.lstat().st_mode
            need(stat.S_ISREG(mode) or stat.S_ISDIR(mode), "special/symlink output denied")
            if stat.S_ISREG(mode):
                need(path.stat().st_size <= FILE_LIMIT, "complete raw file exceeds declared quota")
                total += path.stat().st_size
    need(total <= QUOTA-CLOSING, "all-output closing reserve exhausted")
    return total


def append(path, value):
    budget(); raw = encode(value); need(len(raw) <= 512*1024, "complete scalar row too large")
    path = Path(path)
    need(path.parent.resolve(strict=True) == path.parent and path.is_relative_to(OUTPUT), "owned output path required")
    need(not path.is_symlink() and (not path.exists() or path.stat().st_size+len(raw) <= FILE_LIMIT), "raw row quota denied")
    with path.open("ab" if path.exists() else "xb") as stream: stream.write(raw); stream.flush()


def process_identity(pid):
    base = Path("/proc")/str(pid); raw = (base/"stat").read_text(); fields = raw[raw.rfind(")")+2:].split()
    return dict(pid=pid, starttime=int(fields[19]), ppid=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]),
        uid=base.stat().st_uid, argv=[x.decode() for x in (base/"cmdline").read_bytes().split(b"\0") if x])


def same_generation(expected):
    actual = process_identity(expected["pid"])
    need(all(actual[k] == expected[k] for k in ("pid", "starttime", "uid", "ppid", "pgid", "session", "argv")), "owned PID generation/argv changed")
    return actual


def live_owner():
    receipt=bounded_json(OUTPUT/"owner_started.json");same_generation(receipt["identity"])
    need(os.getppid()==receipt["identity"]["pid"],"owned parent died/reparented")
    raw_fd=os.environ.get("HH_OWNED_RUNTIME_LOCK_FD","")
    need(raw_fd.isdecimal(),"exact inherited runtime lease descriptor required")
    fd=int(raw_fd);actual=os.fstat(fd);declared=receipt["lock_identity"]
    need(fd==receipt["lock_fd"] and (actual.st_dev,actual.st_ino)==(declared["dev"],declared["ino"])
        and Path("/proc/self/fd"+"/"+str(fd)).resolve(strict=True)==LOCK,"inherited lease FD/inode differs")
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    return receipt


def verify_project_modules(view, modules, extra=()):
    for module in (*tuple(modules), *extra):
        name, filename = getattr(module, "__name__", ""), getattr(module, "__file__", None)
        if name == "_hh_owned_dds_info_261007":
            # HH_261007 - Admit only the actual privately built, probe-verified binary, never an unsealed local extension.
            bridge = view["native_dds_bridge"]
            need(filename == bridge["module_path"] and digest(filename) == bridge["module_sha256"], "private native DDS binary/source origin differs")
            continue
        if name in ("scripts","scripts.e2e") and filename is None:
            # HH_261007 - The actual canonical bootstrap legitimately creates these TWO exact namespaces, never generic nil modules.
            expected=ROOT/"tools" if name=="scripts" else ROOT/"tools/e2e"
            need(list(module.__path__) == [str(expected)], "foreign virtual canonical namespace: "+name)
            continue
        if name.startswith(("autoware_planning_msgs", "autoware_vehicle_msgs")):
            need(type(filename) is str, "actual ROS generated module origin required")
            path=Path(filename)
            package=name.split(".")[0]
            need(path.is_relative_to(ROOT/"autonomy/install"/package) or path.is_relative_to(ROOT/"autonomy/build"/package), "foreign Autoware overlay denied")
            actual=path.resolve(strict=True)
            need(str(actual) in view["generated_ROS_sources"] and digest(actual)==view["generated_ROS_sources"][str(actual)], "generated ROS source/ABI seal differs")
            continue
        if not filename:
            need(not (name in ("portable_e2e","canonical_workspace","workspace_python") or name.startswith(("portable_e2e.","scripts."))), "unsealed virtual project module: "+name)
            continue
        path = Path(filename)
        if not path.is_absolute() or not path.is_relative_to(ROOT): continue
        relative = str(path.relative_to(ROOT))
        need(relative in view["source_sha256"] and digest(path) == view["source_sha256"][relative], "unsealed imported project source")


def authorize(plan_sha, authority_sha):
    plan = bounded_json(PLAN, plan_sha, PLAN_LIMIT)
    grant = bounded_json(AUTHORITY, authority_sha)
    need(plan["schema"] == SCHEMA and plan["fixed"] == fixed_contract(), "exact fresh recorded protocol required")
    need(grant == dict(schema=SCHEMA+".ROOT_authority", plan_path=str(PLAN), plan_sha256=plan_sha,
        source_sha256=plan["source_sha256"], recorded_CPU_ROS_only=True, training_authorized=False,
        GPU_authorized=False, CARLA_authorized=False, actuation_authorized=False, TEST_authorized=False, model_promoted=False),
        "externally SHA-pinned ROOT authority required; no self-admission flag")
    need(plan["source_sha256"] == source_pins(), "whole project source closure drift")
    for name, pin in plan["source_sha256"].items():
        need(digest(PLAN_DIR/"sources"/name) == pin, "archived/current source closure differs")
    for path, pin in plan["input_sha256"].items(): need(digest(path) == pin, "whole recorded/model/rig input changed")
    for path, pin in plan["setup_sha256"].items(): need(digest(path) == pin, "clean canonical setup source changed")
    for path, pin in plan["generated_ROS_sources"].items(): need(digest(path)==pin,"canonical generated ROS message/ABI input changed")
    need(os.environ.get("CUDA_VISIBLE_DEVICES") == "" and os.environ.get("PYTHONNOUSERSITE") == "1", "clean CPU-only environment required")
    need(os.environ.get("ROS_LOG_DIR")==str(OUTPUT/"ros_logs") and os.environ.get("OMP_NUM_THREADS")=="4"
        and os.environ.get("MKL_NUM_THREADS")=="4" and os.environ.get("OPENBLAS_NUM_THREADS")=="1", "bounded clean ROS log/CPU thread environment required")
    bundle = frozen_bundle(); bundle.verify_ros_environment(plan)
    view = deepcopy(plan); view["admitted_authority_sha256"] = authority_sha
    view["plan_sha256"] = plan_sha
    view["source_sha256"] = plan["source_sha256"]
    return view


def fixed_contract():
    return dict(camera_anchors=918, native_states=1836, direct_then_DDS_no_simultaneous_NN=True,
        ROS_DOMAIN_ID=93, ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
        CPU_threads=4, interop_threads=1, physical_GPU_used=False, policy_deadline_seconds=.1,
        TF_limits=dict(translation_m=.005, rotation_rad=.005), raw_atol=1e-6, raw_rtol=1e-6,
        whole_input_tensor_hashes=True, matching_valid_contexts_only_raw_and_trajectory=True,
        all_failure_prefill_reset_and_noncomparable_rows_retained=True, original_V5_terminal_equivalence=False,
        positive_published_trajectory_context_required_for_overall_parity=True,
        native_message_publisher_GID_required=True, private_executor_only_owned_subscriptions=True,
        installed_ROS_sources_modified=False, project_native_bridge_actuation_approved=False,
        actual_publication_enter_and_finish_rechecked_100ms=True, actual_ACK_postpublish_completion_retained=True,
        original_BGRA_equivalence=False, wall_10Hz_or_P3_approved=False, automatic_retry=False,
        floor_bytes=FLOOR, shared_bytes=SHARED, quota_bytes=QUOTA, closing_bytes=CLOSING,
        file_limit_bytes=FILE_LIMIT, total_deadline_seconds=1800, per_role_deadline_seconds=900,
        startup_seconds=60, receipt_seconds=5, graph_empty_settle_seconds=.5,
        ROS_LOG_DIR=str(OUTPUT/"ros_logs"), inherited_runtime_lease_required=True, Linux_parent_death_SIGTERM=True,
        runtime_lock=str(LOCK), output=str(OUTPUT), CPU_policy_changed_from_historical_V5_claim=False)


def worker_command(mode, plan_sha, authority_sha):
    return ["/usr/bin/python3", "-B", str(ROOT/"tools/workspace_python.py"), "-m", "scripts.e2e.temporal_recorded_ros_shadow_worker_20261007_v5", "--",
        "--mode", mode, "--plan-sha256", plan_sha, "--authority-sha256", authority_sha]


def pure_proxy(view):
    old = frozen_bundle()
    result = SimpleNamespace(**vars(old))
    result.verify_imported_project_modules = verify_project_modules
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("direct", "subscriber", "publisher"), required=True)
    parser.add_argument("--plan-sha256", required=True); parser.add_argument("--authority-sha256", required=True)
    args = parser.parse_args(argv)
    resource.setrlimit(resource.RLIMIT_FSIZE,(FILE_LIMIT,FILE_LIMIT));resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    import ctypes
    need(ctypes.CDLL(None,use_errno=True).prctl(1,signal.SIGTERM,0,0,0)==0,"Linux parent-death guard required")
    def interrupted(number,frame):raise SystemExit("owned worker interrupted: "+str(number))
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGHUP,interrupted)
    result = dict(schema=SCHEMA+".role_result", mode=args.mode, status="FAILED_RETAINED", expected=918, completed=0, UNEXECUTED=918,
        failing_bundle_may_have_executed=True, error=None, P3_approved=False)
    try:
        view = authorize(args.plan_sha256, args.authority_sha256); budget()
        owner = live_owner()
        actual = process_identity(os.getpid())
        need(actual["argv"] == worker_command(args.mode, args.plan_sha256, args.authority_sha256)
            and actual["ppid"] == owner["identity"]["pid"] and actual["pgid"] == actual["session"] == actual["pid"], "exact owned child session/argv required")
        new_json(OUTPUT/(args.mode+"_started.json"), dict(identity=actual, plan_sha256=args.plan_sha256,
            authority_sha256=args.authority_sha256, mode=args.mode))
        # HH_261007 - Only this admitted branch imports the actual ROS/model implementation.
        implementation = load_module(NODE, "_hh_owned_recorded_ros_runtime_261007")
        result = implementation.run(args.mode, view)
    except BaseException as error:
        result["error"] = repr(error)
        path = OUTPUT/(args.mode+"_anchors.jsonl")
        if path.exists():
            result["completed"] = sum(1 for _ in path.open("rb")); result["UNEXECUTED"] = 918-result["completed"]
    new_json(OUTPUT/(args.mode+"_result.json"), result)
    return 0 if result["status"] == "COMPLETE_RECORDED_ROLE_NOT_FEATURE_APPROVAL" else 1


if __name__ == "__main__": raise SystemExit(main())
