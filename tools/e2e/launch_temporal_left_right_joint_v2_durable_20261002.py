"""HH_261002 - Launch exactly one authorized private balanced V2 driving fit with detached, create-only regular-file logs.

This source-only helper never connects over SSH, installs, retries, resumes,
selects GPU1 or kills any process. A launch receipt is not evidence of updates.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

NAME = "temporal_left_right_joint_20261002_v2"
SCHEMA = "portable_e2e.temporal_left_right_detached_launch.20261002.v2"
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE = WORKSPACE / "source_worktrees" / NAME
VENV = WORKSPACE / "venvs/py312"
PYTHON = VENV / "bin/python"
TRAINER_REL = "tools/e2e/train_temporal_left_right_joint_v2_20261002.py"
TRAINER_SHA = "cd991d5ba46b30eb23b5115a2dffb75130f2b182fb81701c13991977c8269a11"
TRAINER = SOURCE / TRAINER_REL
TRAIN_AUTHORITY = WORKSPACE / "plans/training" / NAME / "ROOT_fixed20k_balanced_driving_authorization.json"
HELPER = WORKSPACE / "plans/launches" / NAME / Path(__file__).name
LAUNCH_AUTHORITY = HELPER.parent / "ROOT_detached_launch_authorization.json"
LOGS = WORKSPACE / "runs/launches" / NAME
OUTPUT = WORKSPACE / "runs/training" / NAME
PERSONAL = WORKSPACE.parent
CPU_PREFLIGHT = TRAIN_AUTHORITY.parent / "ROOT_all20_TRAIN_CPU_preflight.json"
MAP_COMPLETION = TRAIN_AUTHORITY.parent / "ROOT_previous_MAPV3_completed.json"
MAP_OUTPUT = WORKSPACE / "runs/diagnostics/map_native_fusion_pair_20261002_v3"
MAP_PID, MAP_START_TICKS = 3472102, 140240638
ARGUMENT_NAMES = ("parent-run", "warm-parent-weights", "warm-parent-metadata", "backbone-parent-weights", "backbone-parent-metadata",
    "recovery-train", "recovery-conversion", "recovery-holdout", "holdout-conversion", "left-train", "right-train")
LEASE = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
ENVIRONMENT = dict(CUDA_VISIBLE_DEVICES=GPU_UUID, CUBLAS_WORKSPACE_CONFIG=":4096:8",
    PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", OMP_NUM_THREADS="4", MKL_NUM_THREADS="4")


def require(value, message):
    if not value: raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(v in "0123456789abcdef" for v in value)


def canonical(path, boundary=None):
    boundary = WORKSPACE if boundary is None else boundary
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path.resolve() == path
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical unaliased path required")
    require(path.is_relative_to(boundary), "path escapes private portable workspace")
    return path


def digest(path, boundary=None):
    canonical(path, boundary); require(path.is_file(), "regular pinned file required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_pinned(path, expected):
    # HH_261002 - Parse precisely the same first-read buffer whose SHA matches ROOT; never reopen authority bytes.
    canonical(path); require(path.is_file(), "regular pinned authority required")
    raw = path.read_bytes()
    require(sha(expected) and hashlib.sha256(raw).hexdigest() == expected, "external bytes differ")
    return json.loads(raw, object_pairs_hook=unique_object, parse_constant=lambda _: require(False, "nonfinite JSON denied"))


def contract():
    return dict(schema=SCHEMA, study_id=NAME, source=str(SOURCE), trainer=str(TRAINER),
        trainer_authority=str(TRAIN_AUTHORITY), helper=str(HELPER), launch_authority=str(LAUNCH_AUTHORITY),
        logs=str(LOGS), output=str(OUTPUT), existing_private_venv=str(VENV), physical_GPU_index=0, GPU_UUID=GPU_UUID,
        environment=dict(ENVIRONMENT), loader_workers=2, attempts=1,
        fixed20000_B5_100000_exposures=True, source_count=31, original_GOOD6000_parent_only=True,
        immediate_only=True, predecessor_MAP_output=str(MAP_OUTPUT), predecessor_PID=MAP_PID, predecessor_start_ticks=MAP_START_TICKS,
        predecessor_fixed_both20000_full350_all27_required=True, cpu_preflight=str(CPU_PREFLIGHT), runtime_and_asset_pins_required=True,
        resume=False, automatic_retry=False,
        detached_session=True, stdin="DEVNULL", stdout="create_only_regular_file", stderr="create_only_regular_file",
        original_GPU0_training_lease_required=True, trainer_rechecks_lease_and_GPU=True,
        main_capability_approved=False, model_promoted=False, vehicle_control_approved=False)


def write_new(path, value):
    canonical(path)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")


def process_generation(pid):
    # HH_261002 - Record kernel start ticks rather than treating a reused PID as the owned child.
    text = Path(f"/proc/{pid}/stat").read_text()
    return int(text[text.rfind(")") + 2:].split()[19])


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key denied"); result[key] = value
    return result


def trainer_revision():
    # HH_261002 - Only authenticate the full canonical source tree before configuring its Python namespace.
    import importlib.util
    spec = importlib.util.spec_from_file_location("hh_balanced_driving_workspace", SOURCE / "tools/workspace_python.py")
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); module.configure(SOURCE)
    from scripts.e2e import train_temporal_left_right_joint_v2_20261002 as revision
    require(Path(revision.__file__) == TRAINER and revision.ROOT == SOURCE, "trainer escaped sealed source")
    return revision


def runtime_identity(revision):
    # HH_261002 - Bind physical executable and numerical binary providers without CUDA initialization.
    require(not revision.torch.cuda.is_initialized(), "no CUDA before GPU lease")
    import importlib
    numpy_C = importlib.import_module("numpy._core._multiarray_umath" if hasattr(revision.np, "_core") else "numpy.core._multiarray_umath")
    torch_root = Path(revision.torch.__file__).resolve().parent
    providers = dict(python=Path(sys.executable).resolve(), torch_C=Path(revision.torch._C.__file__).resolve(),
        numpy_C=Path(numpy_C.__file__).resolve(), libtorch_cpu=torch_root / "lib/libtorch_cpu.so",
        libtorch_cuda=torch_root / "lib/libtorch_cuda.so")
    require(all(path.is_file() and path.resolve() == path for path in providers.values())
        and providers["numpy_C"].suffix == providers["torch_C"].suffix == ".so", "actual physical numerical binary files required")
    return dict(python_version=sys.version, torch_version=revision.torch.__version__, numpy_version=revision.np.__version__,
        providers={key: dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for key, path in providers.items()})


def verify_sources(pins):
    require(type(pins) is dict and len(pins) == 31 and pins.get(TRAINER_REL) == TRAINER_SHA
        and all(type(name) is str and not Path(name).is_absolute() and ".." not in Path(name).parts and sha(pin)
            for name, pin in pins.items()), "full thirty-one externally pinned canonical sources required")
    require(all(digest(SOURCE / name) == pin for name, pin in pins.items()), "source changed before import/fork")


def verify_assets(pins):
    require(type(pins) is dict and pins and all(type(path) is str and sha(pin) for path, pin in pins.items()), "complete CPU preflight asset pins required")
    require(all(digest(Path(path), PERSONAL) == pin for path, pin in pins.items()), "one private original training/model/receipt asset changed")


def predecessor_complete(value):
    # HH_261002 - ROOT witnesses numeric endpoint/journal verification; full350/27 metadata may FAIL but must be complete.
    proof = read_pinned(MAP_COMPLETION, value["MAP_completion_sha256"])
    require(proof.get("schema") == SCHEMA + ".MAP_completion" and proof.get("PID") == MAP_PID
        and type(proof.get("PID")) is int and proof.get("process_start_ticks") == MAP_START_TICKS
        and type(proof.get("process_start_ticks")) is int and type(proof.get("boot_id")) is str
        and proof["boot_id"] == Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        and proof.get("source_asset_numeric_endpoints_and_full_journals_verified") is True
        and proof.get("model_promoted") is False and proof.get("vehicle_control_approved") is False,
        "ROOT must verify actual preceding MAP endpoints and complete evaluation")
    summary = read_pinned(MAP_OUTPUT / "summary.json", proof["summary_sha256"])
    evaluation = read_pinned(MAP_OUTPUT / "known_evaluation.json", proof["known_evaluation_sha256"])
    fit = read_pinned(MAP_OUTPUT / "fit_complete.json", proof["fit_complete_sha256"])
    require(summary.get("schema") == "portable_e2e.map_native_fusion_pair.20261002.v3"
        and summary.get("status") == "FIXED20K_ARCHITECTURE_PAIR_DIAGNOSTIC_NOT_PROMOTED"
        and summary.get("source_and_inputs_unchanged") is True
        and summary.get("model_promoted") is False and summary.get("vehicle_control_approved") is False
        and summary.get("known_evaluation") == evaluation
        and fit.get("status") == "BOTH_FIXED_FINAL20000_NUMERIC_ENDPOINTS_VERIFIED_BEFORE_KNOWN_EVALUATION"
        and fit.get("known_payload_opened") is False and fit.get("arms") == summary.get("arms_result")
        and fit.get("actual_full_journal_evidence") == summary.get("actual_boundary"), "actual complete MAP metadata required")
    arms = summary["arms_result"]; boundary = summary["actual_boundary"]
    require(set(arms) == set(boundary) == {"control", "candidate"}, "both completed MAP arms required")
    for arm in ("control", "candidate"):
        require(type(arms[arm].get("steps")) is int and arms[arm]["steps"] == 20000
            and type(arms[arm].get("examples_seen")) is int and arms[arm]["examples_seen"] == 160000
            and type(boundary[arm].get("steps")) is int and boundary[arm]["steps"] == 20000
            and boundary[arm].get("examples_seen") == 160000 and arms[arm].get("model_promoted") is False
            and arms[arm].get("vehicle_control_approved") is False, "fixed both20k and complete160k per arm required")
    require(set(evaluation) == {"dev_town01", "holdout_town02_p02"}, "entire known350 evaluation required")
    for split, count in (("dev_town01", 118), ("holdout_town02_p02", 232)):
        require(set(evaluation[split]) == {"parent", "control", "candidate"}, "all three MAP evaluations required")
        for metrics in evaluation[split].values():
            require(type(metrics.get("evaluated_frames")) is int and metrics["evaluated_frames"] == count
                and type(metrics.get("count")) is int and metrics["count"] == count and metrics.get("full_FOV") is True,
                "MAP evaluation cannot crop the declared350 denominator")
    checks = summary["exploratory_screen_result"]["checks"]
    require(type(checks) is dict and len(checks) == 27 and all(type(v) is bool for v in checks.values()), "retain all27 including failed screens")
    proc = Path(f"/proc/{MAP_PID}/stat")
    require(not proc.exists() or process_generation(MAP_PID) != MAP_START_TICKS, "owned predecessor still running; no automatic waiting or kill")
    return proof


def source_and_authority(value):
    verify_sources(value.get("source_sha256"))
    revision = trainer_revision(); revision.verify_frozen()
    authority = read_pinned(TRAIN_AUTHORITY, value["trainer_authorization_sha256"])
    proposed = revision.contract(authority["contract"]["new_LEFT_conversion_sha256"])
    revision.validate_authorization(TRAIN_AUTHORITY, value["trainer_authorization_sha256"], proposed)
    require(revision.source_snapshot() == value["source_sha256"], "actual full31 source contract differs")
    cpu = read_pinned(CPU_PREFLIGHT, value["cpu_preflight_sha256"])
    require(all(type(cpu.get(key)) is int and cpu[key] == count for key, count in
        (("training_episode_count", 20), ("effective_batch_size", 5), ("trainable_state_tensor_count", 54), ("frozen_state_tensor_count", 47))),
        "typed actual CPU training/model census required")
    require(cpu.get("status") == "VALIDATED_ALL20_TRAIN_CPU_NO_CUDA_NO_OUTPUT" and cpu.get("schema") == revision.SCHEMA
        and json.dumps(cpu.get("protocol"), sort_keys=True, allow_nan=False) == json.dumps(proposed, sort_keys=True, allow_nan=False)
        and cpu.get("source_sha256") == value["source_sha256"]
        and cpu.get("immutable_input_sha256") == value["asset_sha256"]
        and cpu.get("authorization_sha256") == value["trainer_authorization_sha256"]
        and cpu.get("training_episode_count") == 20 and cpu.get("effective_batch_size") == 5
        and cpu.get("trainable_state_tensor_count") == 54 and cpu.get("frozen_state_tensor_count") == 47
        and cpu.get("parent_history_sha256") == revision.PARENT_HISTORY_SHA and cpu.get("holdout_payload_loaded") is False,
        "actual authorized all20 TRAIN CPU preflight required")
    require(len(cpu["fixed_sample_ids"]) == 20000 and all(len(row) == 5 for row in cpu["fixed_sample_ids"])
        and revision.json_sha(cpu["fixed_sample_ids"]) == cpu["fixed_sample_ids_sha256"], "full20k CPU B5 chronology required")
    verify_assets(value["asset_sha256"])
    require(value.get("runtime_identity") == runtime_identity(revision), "current private binary providers differ")
    arguments = value.get("asset_arguments")
    require(type(arguments) is dict and set(arguments) == set(ARGUMENT_NAMES), "all exact fit asset arguments required")
    for path in arguments.values(): canonical(Path(path), PERSONAL); require(Path(path).exists(), "declared fit input missing")
    require(arguments["right-train"] == str(PERSONAL / "datasets/prepared" / revision.RIGHT_BASENAME)
        and arguments["left-train"] == str(PERSONAL / "dataset/prepared" / revision.LEFT_BASENAME)
        and all(str(Path(arguments[name])) in value["asset_sha256"] for name in
            ("warm-parent-weights", "warm-parent-metadata", "backbone-parent-weights", "backbone-parent-metadata", "recovery-conversion")),
        "canonical RIGHT plural, frozen LEFT singular and exact parent/conversion assets required")
    return proposed


def fit_command(value, proposed):
    command = [str(PYTHON), "-B", "-u", str(SOURCE / "tools/workspace_python.py"), "-m",
        "scripts.e2e.train_temporal_left_right_joint_v2_20261002", "--", "--mode", "fit"]
    for name in ARGUMENT_NAMES: command.extend(["--" + name, value["asset_arguments"][name]])
    return command + ["--left-conversion-sha256", proposed["new_LEFT_conversion_sha256"],
        "--right-conversion-sha256", proposed["new_RIGHT_conversion_sha256"], "--parent-history-sha256", proposed["actual_parent_history_sha256"],
        "--output", str(OUTPUT), "--authorization", str(TRAIN_AUTHORITY),
        "--authorization-sha256", value["trainer_authorization_sha256"], "--loader-workers", "2"]


def lease_and_GPU_idle():
    canonical(LEASE); require(LEASE.is_file(), "existing original GPU0 lease file required")
    descriptor = os.open(LEASE, os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rows = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-gpu=index,uuid,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits"], text=True, timeout=20).strip().splitlines()
        apps = subprocess.check_output(["nvidia-smi", "-i", "0", "--query-compute-apps=pid,gpu_uuid,used_memory",
            "--format=csv,noheader,nounits"], text=True, timeout=20).strip()
        fields = [v.strip() for v in rows[0].split(",")] if len(rows) == 1 else []
        require(len(fields) == 4 and fields[:2] == ["0", GPU_UUID] and fields[2].isdecimal()
            and 0 <= int(fields[2]) <= 16 and fields[3] == "0" and not apps,
            "GPU0 must have exact UUID, no compute processes/utilization, and only driver residue")
        return dict(physical_index=0, uuid=GPU_UUID, memory_used_MiB=int(fields[2]), utilization=0, compute_processes=0)
    finally:
        os.close(descriptor)


def environment_check():
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == VENV and Path.cwd() == SOURCE
        and sys.flags.optimize == 0 and Path(__file__).absolute() == HELPER,
        "exact remote host, existing private venv, source cwd, and pinned private helper required")
    for path in (WORKSPACE, SOURCE, VENV, HELPER, TRAINER, TRAIN_AUTHORITY, LAUNCH_AUTHORITY, CPU_PREFLIGHT, MAP_COMPLETION, MAP_OUTPUT, LOGS.parent, OUTPUT.parent):
        canonical(path); require(path.exists(), "required private path missing")
    canonical(LOGS); canonical(OUTPUT)
    require(not LOGS.exists() and not OUTPUT.exists(), "fresh logs and trainer output only; no retry/resume/overwrite")
    require(all(os.environ.get(k) == v for k, v in ENVIRONMENT.items())
        and not any(os.environ.get(k) for k in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP")),
        "strict approved inherited environment required")


def launch(args):
    environment_check()
    require(sha(args.helper_sha256) and digest(HELPER) == args.helper_sha256, "ROOT-pinned launcher source changed")
    value = read_pinned(LAUNCH_AUTHORITY, args.authorization_sha256)
    require(type(value) is dict and value.get("schema") == SCHEMA + ".authorization"
        and json.dumps(value.get("contract"), sort_keys=True, allow_nan=False) == json.dumps(contract(), sort_keys=True, allow_nan=False)
        and value.get("helper_sha256") == args.helper_sha256
        and sha(value.get("trainer_authorization_sha256")) and value.get("detached_launch_authorized") is True
        and type(value.get("reviewer")) is str and len(value["reviewer"].strip()) >= 3
        and value.get("automatic_model_promotion") is False and value.get("vehicle_control_approved") is False,
        "exact external ROOT launch scope required")
    proposed = source_and_authority(value); source = value["source_sha256"]
    predecessor = predecessor_complete(value)
    gpu = lease_and_GPU_idle()
    require(digest(HELPER) == args.helper_sha256 and digest(LAUNCH_AUTHORITY) == args.authorization_sha256
        and digest(TRAIN_AUTHORITY) == value["trainer_authorization_sha256"], "launch authority changed before fork")
    verify_sources(source); verify_assets(value["asset_sha256"]); predecessor_complete(value)
    require(digest(CPU_PREFLIGHT) == value["cpu_preflight_sha256"], "CPU evidence changed before fork")
    require(runtime_identity(trainer_revision()) == value["runtime_identity"], "binary providers changed before fork")
    command = fit_command(value, proposed)
    if args.validate_only:
        return dict(status="DETACHED_LAUNCH_PREFLIGHT_ONLY_NO_PROCESS_NO_TRAIN_NO_OUTPUT", contract=contract(),
            source_sha256=source, GPU_idle=gpu, MAP_predecessor=predecessor, command=command)
    LOGS.mkdir(mode=0o700)
    write_new(LOGS / "launch_plan.json", dict(contract=contract(), source_sha256=source,
        helper_sha256=args.helper_sha256, launch_authorization_sha256=args.authorization_sha256,
        trainer_authorization_sha256=value["trainer_authorization_sha256"], command=command, GPU_idle=gpu,
        CPU_preflight_sha256=value["cpu_preflight_sha256"], MAP_completion=predecessor,
        created_at_utc=datetime.now(timezone.utc).isoformat(), actual_optimizer_updates_claimed=0))
    # HH_261002 - Real local regular files outlive SSH stdout; never pass PIPE, terminal or inherited stdin to the fit.
    with (LOGS / "stdout.log").open("xb", buffering=0) as stdout, (LOGS / "stderr.log").open("xb", buffering=0) as stderr:
        require(stat.S_ISREG(os.fstat(stdout.fileno()).st_mode) and stat.S_ISREG(os.fstat(stderr.fileno()).st_mode),
            "durable output descriptors must be regular files")
        process = subprocess.Popen(command, cwd=SOURCE, env=dict(os.environ), stdin=subprocess.DEVNULL,
            stdout=stdout, stderr=stderr, start_new_session=True, close_fds=True)
    # HH_261002 - Even an immediate child exit retains launch/log evidence; missing generation never proves a running owned fit.
    generation, generation_error = None, None
    try:
        generation = dict(process_start_ticks=process_generation(process.pid),
            boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip())
    except (OSError, ValueError) as error:
        generation_error = dict(error_type=type(error).__name__, error=str(error))
    value = dict(status="DETACHED_PROCESS_STARTED_NOT_YET_TRAINING_PROOF", pid=process.pid,
        process_start_ticks=None if generation is None else generation["process_start_ticks"],
        boot_id=None if generation is None else generation["boot_id"],
        owned_process_generation_confirmed=generation is not None, generation_read_error=generation_error,
        created_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=source, command=command, stdin="DEVNULL", stdout=str(LOGS / "stdout.log"),
        stderr=str(LOGS / "stderr.log"), session_detached=True, actual_optimizer_updates_claimed=0)
    write_new(LOGS / "launch_receipt.json", value)
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "launch"), default="contract")
    parser.add_argument("--authorization-sha256"); parser.add_argument("--helper-sha256")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    if args.mode == "contract":
        require(args.authorization_sha256 is None and args.helper_sha256 is None and not args.validate_only,
            "query cannot accept execution authority")
        result = contract()
    else: result = launch(args)
    print(json.dumps(result, indent=2, allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
