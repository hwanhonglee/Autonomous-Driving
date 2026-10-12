#!/usr/bin/env python3
"""HH_260906 - Compare four fresh, unchanged one-epoch fits with zero versus two CPU DataLoader workers."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.util
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


SCHEMA = "portable_e2e.dataloader_abba_diagnostic.v1"
PLAN_SCHEMA = "portable_e2e.dataloader_abba_plan.v1"
WORKSPACE = Path.home() / "personal/hwanhong/portable_e2e"
HELPER_PATH = WORKSPACE / "benchmarks/training_profile_v1/profile_portable_training.py"
HELPER_SHA = "a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436"
SOURCE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
ARMS = (("A1_workers0", 0), ("B1_workers2", 2), ("B2_workers2", 2), ("A2_workers0", 0))
STEPS, SAMPLES = 287, 1147
FIT_TIMEOUT, TOTAL_TIMEOUT, FINAL_RESERVE = 240, 1140, 90
WORKER_PROOF_ENV = "PORTABLE_E2E_ABBA_WORKER_PROOF_DIR"
FIT_PID_ENV = "PORTABLE_E2E_ABBA_FIT_PID"
DENIALS = dict(vehicle_control_approved=False, data_admission=False, model_promotion=False,
    test_neural_network_use=False, production_source_changes=False, automatic_performance_change=False)


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def worker_cpu_guard():
    # HH_260906 - Spawn reimports this stdlib-only main before unpickling the CPU dataset; hide CUDA in those children only.
    require("torch" not in sys.modules, "spawn CPU guard must precede importing torch")
    require(os.environ.get(FIT_PID_ENV) == str(os.getppid()), "unrecognized spawned worker parent")
    directory = Path(os.environ[WORKER_PROOF_ENV]).absolute()
    require(directory.is_dir() and directory == directory.resolve() and directory.name == "worker_proofs",
            "unsafe CPU-worker witness directory")
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    proof = dict(schema=SCHEMA, pid=os.getpid(), parent_pid=os.getppid(), process_group=os.getpgrp(),
        cuda_visible_devices="", torch_imported_before_guard=False, start_method=mp.get_start_method(),
        source_sha256=sha(Path(__file__).read_bytes()),
        notice="Pre-import CPU isolation witness, not an observation of CUDA kernel utilization.")
    require(proof["start_method"] == "spawn", "spawn required in data workers")
    with (directory / (str(os.getpid()) + ".json")).open("x") as stream:
        json.dump(proof, stream, sort_keys=True); stream.write("\n")


if __name__ == "__mp_main__":
    worker_cpu_guard()


def load_helper(path=None):
    path = Path(path or HELPER_PATH).absolute()
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), "unsafe frozen helper")
    raw = path.read_bytes()
    require(sha(raw) == HELPER_SHA, "frozen profiler helper SHA differs")
    spec = importlib.util.spec_from_file_location("_frozen_abba_profiler_helper", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    require(sha(path.read_bytes()) == HELPER_SHA, "helper changed during import")
    return module


def train_config(base, workers):
    require(type(workers) is int and workers in (0, 2), "only declared worker counts allowed")
    return dict(base.TRAIN_CONFIG, max_steps=STEPS, checkpoint_interval=STEPS, num_workers=workers)


def plan_contract(base):
    return dict(schema=PLAN_SCHEMA, diagnostic_id="hh260909-dataloader-abba-one-epoch-v1",
        source_commit=SOURCE_COMMIT, gpu_uuid=base.GPU_UUID, dataset=base.DATASET,
        dataset_manifest_sha256=base.MANIFEST_SHA, corpus_fingerprint_sha256=base.CORPUS_SHA,
        train_fingerprint_sha256=base.TRAIN_SHA, model_config=base.MODEL_CONFIG, model_config_sha256=base.MODEL_SHA,
        expected_train_samples=SAMPLES, expected_train_episodes=3,
        train_config=train_config(base, 0), loss_config=base.LOSS_CONFIG,
        arms=[dict(id=arm, num_workers=workers) for arm, workers in ARMS],
        expected_steps_per_fit=STEPS, expected_sample_exposures_per_fit=SAMPLES,
        expected_metric_rows=STEPS * 4, expected_total_sample_exposures=SAMPLES * 4,
        checkpoint_ignored_fields=["created_at_utc", "train_config.num_workers"],
        multiprocessing_start_method="spawn", worker_cuda_visible_devices="", fresh_interpreter_per_fit=True,
        frozen_helper_sha256=HELPER_SHA, fit_wall_timeout_seconds=FIT_TIMEOUT,
        internal_wall_timeout_seconds=TOTAL_TIMEOUT, external_wall_timeout_seconds=1200,
        safety_reserve_seconds=FINAL_RESERVE, finish_before_utc="2026-09-09T01:00:00Z", approval=DENIALS)


def validate_plan(plan, base):
    expected = plan_contract(base)
    require(set(plan) == set(expected) | {"declared_at_utc", "source_sha256", "worker_source_sha256"}, "unexpected plan keys")
    for key, value in expected.items():
        require(json.dumps(plan[key], sort_keys=True) == json.dumps(value, sort_keys=True), "unreviewed plan: " + key)
    require(set(plan["source_sha256"]) == set(base.SOURCE_PATHS), "training source inventory differs")
    for value in [*plan["source_sha256"].values(), plan["worker_source_sha256"]]:
        require(isinstance(value, str) and re.fullmatch("[0-9a-f]{64}", value), "invalid source SHA")
    require(base.timestamp(plan["declared_at_utc"]) < base.timestamp(plan["finish_before_utc"]), "declaration after deadline")
    return plan


def check_budget(plan, base, required, now=None):
    now = now or datetime.now(timezone.utc)
    require(base.timestamp(plan["declared_at_utc"]) <= now, "prospective declaration is in the future")
    require((base.timestamp(plan["finish_before_utc"]) - now).total_seconds() >= required,
            "insufficient time for the whole bounded operation and cleanup")


def source_identity(plan, base, repo):
    # HH_260906 - The reused helper is external and separately pinned; every actual trainer dependency still matches offline b478.
    adapted = dict(plan, profiler_source_sha256=HELPER_SHA)
    result = base.source_identity(adapted, repo, HELPER_PATH)
    result["frozen_helper.py"] = result.pop("independent_profiler.py")
    result["independent_worker.py"] = base.digest(Path(__file__))
    require(result["independent_worker.py"] == plan["worker_source_sha256"], "ABBA worker bytes differ")
    env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    def git(*args):
        return subprocess.check_output(["git", "-c", "protocol.allow=never", *args], cwd=repo, env=env, timeout=10, text=True).strip()
    require(git("rev-parse", "HEAD") == SOURCE_COMMIT and not git("status", "--porcelain"), "clean unchanged b478 repository required")
    return result


def configure_spawn():
    require("torch" not in sys.modules, "spawn selection must precede torch")
    require(mp.get_start_method(allow_none=True) in (None, "spawn"), "pre-existing non-spawn context forbidden")
    mp.set_start_method("spawn", force=False) if mp.get_start_method(allow_none=True) is None else None
    require(mp.get_start_method() == "spawn", "spawn context did not apply")


@contextmanager
def lease_fd(base):
    path = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
    require(path.parent.resolve() == path.parent and path.parent.is_dir() and not path.is_symlink(), "unsafe lease")
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield fd
    finally:
        os.close(fd)


def verify_inherited_lease(fd):
    require(type(fd) is int and fd >= 3, "supervisor lease FD required")
    expected = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
    require(Path(os.readlink("/proc/self/fd/" + str(fd))) == expected, "inherited lease target differs")
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)


def read_metrics(path, base):
    raw = base.regular(path).read_bytes()
    require(raw.endswith(b"\n"), "truncated metrics retained, not complete")
    rows = [json.loads(line, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite metrics"))) for line in raw.splitlines()]
    require(len(rows) == STEPS, "all 287 updates must be retained")
    for i, row in enumerate(rows, 1):
        require(type(row.get("global_step")) is int and row["global_step"] == i
            and type(row.get("samples_seen")) is int and row["samples_seen"] == min(i * 4, SAMPLES)
            and type(row.get("epoch")) is int and row["epoch"] == 0, "metric step, epoch, or exposure count differs")
        require(row.get("domain_samples_seen") == {"carla": min(i * 4, SAMPLES)}
            and row.get("batch_domain_sample_counts") == {"carla": 3 if i == STEPS else 4}, "batch/domain denominator differs")
        for key, value in row.items():
            if isinstance(value, float): require(math.isfinite(value), "nonfinite metric " + key)
        for key in ("loss", "gradient_norm", "regression_loss", "candidate_score_loss"):
            require(type(row.get(key)) in (int, float) and math.isfinite(row[key]), "missing finite metric " + key)
    return rows


def checkpoint_view(payload, workers, base):
    # HH_260906 - Validate before ignoring the one intended configuration difference; all optimizer/RNG/ABI fields remain exact.
    require(json.dumps(payload.get("train_config"), sort_keys=True) == json.dumps(train_config(base, workers), sort_keys=True),
            "checkpoint schedule differs")
    require(payload.get("dataset_fingerprint_sha256") == base.TRAIN_SHA
        and payload.get("corpus_fingerprint_sha256") == base.CORPUS_SHA and payload.get("training_split") == "train"
        and payload.get("loss_config") == base.LOSS_CONFIG, "checkpoint dataset/split/loss differs")
    state = payload.get("state", {})
    require(state.get("global_step") == STEPS and state.get("samples_seen") == SAMPLES
        and state.get("epoch") == 0 and state.get("next_batch_index") == STEPS,
        "checkpoint must be saved before the unchanged trainer's final epoch increment")
    view = dict(payload)
    view["train_config"] = dict(payload["train_config"], num_workers=0)
    return view


def compare_outputs(output, base, torch, trainer, report):
    rows = [read_metrics(output / arm / "fit/metrics.jsonl", base) for arm, _ in ARMS]
    report["metrics_parity"] = []
    for index in range(1, 4):
        issue = base.exact_difference(rows[0], rows[index], torch)
        report["metrics_parity"].append(dict(reference=ARMS[0][0], candidate=ARMS[index][0], exact=issue is None,
            first_difference=issue, retained_rows_per_arm=STEPS))
    report["checkpoint_parity"] = []
    reference = None
    for arm, workers in ARMS:
        payload, digest = trainer._read_checkpoint_file(output / arm / "fit/checkpoints/latest.pt", torch.device("cpu"))
        view = checkpoint_view(payload, workers, base)
        if reference is None: reference = view
        issue = base.exact_difference(reference, view, torch, ignore_created=True)
        report["checkpoint_parity"].append(dict(arm=arm, checkpoint_sha256=digest, exact=issue is None,
            first_difference=issue, weights_only=True, ignored_fields=["created_at_utc", "train_config.num_workers"]))
    require(all(row["exact"] for row in report["metrics_parity"] + report["checkpoint_parity"]),
            "exact parity failed; no performance equivalence or automatic change")


def validate_worker_proofs(directory, expected, pid, group, worker_sha, base):
    paths = sorted(directory.iterdir())
    require(len(paths) == expected, "wrong number of spawned CPU isolation witnesses")
    rows = [base.json_read(path) for path in paths]
    for path, row in zip(paths, rows):
        require(type(row.get("pid")) is int and row["pid"] > 1 and path.name == str(row["pid"]) + ".json"
            and row.get("parent_pid") == pid and row.get("process_group") == group
            and row.get("source_sha256") == worker_sha and row.get("start_method") == "spawn"
            and row.get("cuda_visible_devices") == "" and row.get("torch_imported_before_guard") is False,
            "CPU worker witness differs from the owned fit")
    return rows


def fit_one(args, base, plan, repo):
    base.check_environment(WORKSPACE, repo)
    configure_spawn(); verify_inherited_lease(args.lease_fd)
    require(os.getpid() == os.getpgrp(), "fit must own its new process session")
    arm = args.fit_arm; workers = dict(ARMS)[arm]
    output = args.output_dir.absolute(); arm_dir = output / arm
    require(output == output.resolve() and output.is_relative_to(WORKSPACE / "runs/diagnostics")
        and arm_dir.is_dir() and {p.name for p in arm_dir.iterdir()} == {"console.log"},
        "fresh supervisor-created arm directory required")
    base.regular(arm_dir / "console.log")
    require(base.digest(output / "plan.json") == base.digest(args.plan), "supervisor plan copy differs")
    before = source_identity(plan, base, repo)
    dataset_path = (WORKSPACE / base.DATASET).resolve(strict=True)
    require(dataset_path.is_relative_to((WORKSPACE.parent / "dataset").resolve()), "dataset escaped personal root")
    report = dict(schema=SCHEMA, arm=arm, num_workers=workers, status="STARTED", pid=os.getpid(), process_group=os.getpgrp(),
        started_at_utc=base.utc(), source_sha256=before, approval=DENIALS, postcheck_errors=[])
    (arm_dir / "worker_proofs").mkdir()
    os.environ[WORKER_PROOF_ENV] = str(arm_dir / "worker_proofs"); os.environ[FIT_PID_ENV] = str(os.getpid())
    loaded = None; loader_function = None
    try:
        check_budget(plan, base, FIT_TIMEOUT + FINAL_RESERVE)
        sys.path.insert(0, str(repo))
        import torch
        from portable_e2e import train as trainer
        from portable_e2e.dataset import load_training_examples
        from portable_e2e.model import ModelConfig
        from portable_e2e.losses import TrajectoryLossConfig
        from portable_e2e.torch_dataset import Common10TorchDataset
        loader_function = load_training_examples
        require(Path(trainer.__file__).resolve() == repo / "portable_e2e/train.py", "unexpected trainer module")
        require(torch.cuda.device_count() == 1 and torch.get_num_threads() == 16, "GPU visibility or CPU threads differ")
        report["torch_device_uuid"] = base.verify_torch_uuid(torch.cuda.get_device_properties(0))
        started = time.perf_counter()
        loaded = load_training_examples(dataset_path, split="train", mode="planning", check_image_hashes=True)
        cfg = ModelConfig.from_mapping(base.json_read(repo / base.MODEL_CONFIG))
        dataset = Common10TorchDataset(loaded.examples, cfg, verify_image_sha256=True, split="train")
        report["initial_full_corpus_validation_wall_seconds"] = time.perf_counter() - started
        require(len(dataset) == SAMPLES and len({e.episode_id for e in dataset.examples}) == 3
            and dataset.fingerprint_sha256 == base.TRAIN_SHA
            and loaded.validation_report["dataset_fingerprint_sha256"] == base.CORPUS_SHA, "train/corpus identity mismatch")
        settings = dict(train_config(base, workers), domain_ratios=())
        torch.cuda.synchronize(0); started = time.perf_counter()
        result = trainer.train_model(dataset=dataset, run_dir=arm_dir / "fit", dataset_fingerprint_sha256=base.TRAIN_SHA,
            corpus_fingerprint_sha256=base.CORPUS_SHA, model_config=cfg, train_config=trainer.TrainConfig(**settings),
            loss_config=TrajectoryLossConfig(**base.LOSS_CONFIG), device_name="cuda:0", training_split="train", resume=False)
        torch.cuda.synchronize(0)
        report["train_call_wall_seconds_including_final_sync"] = time.perf_counter() - started
        require(result.get("status") == "TRAINING_TARGET_REACHED" and result["state"]["global_step"] == STEPS
            and result["state"]["samples_seen"] == SAMPLES and result["state"]["epoch"] == 1
            and result["state"]["next_batch_index"] == 0, "full fresh epoch did not finish")
        read_metrics(arm_dir / "fit/metrics.jsonl", base)
        report.update(status="FIT_COMPLETE_NOT_PROMOTED", state=result["state"],
            worker_proofs=validate_worker_proofs(arm_dir / "worker_proofs", workers, os.getpid(), os.getpgrp(), plan["worker_source_sha256"], base))
    except BaseException as error:
        report.update(status="FAILED_FIT_NOT_PROMOTED", error_type=type(error).__name__, error=str(error))
    finally:
        with base.finalization_signals():
            # HH_260906 - Only this fit's multiprocessing children are stopped; the supervisor separately owns the complete session.
            report["child_cleanup"] = []
            for child in mp.active_children():
                row = dict(pid=child.pid, was_alive=child.is_alive())
                if child.is_alive(): child.terminate()
                child.join(3)
                if child.is_alive(): child.kill(); child.join(3)
                row["still_alive"] = child.is_alive(); report["child_cleanup"].append(row)
            checks = {"source": lambda: source_identity(plan, base, repo) == before,
                "plan": lambda: base.digest(args.plan) == base.digest(output / "plan.json"),
                "dataset_manifest": lambda: base.digest(dataset_path / "dataset.json") == base.MANIFEST_SHA}
            if loader_function is not None:
                def corpus():
                    started = time.perf_counter()
                    after = loader_function(dataset_path, split="train", mode="planning", check_image_hashes=True)
                    report["final_full_corpus_validation_wall_seconds"] = time.perf_counter() - started
                    return after.validation_report["dataset_fingerprint_sha256"] == base.CORPUS_SHA and (loaded is None or after.fingerprint_sha256 == loaded.fingerprint_sha256)
                checks["full_corpus"] = corpus
            for name, operation in checks.items():
                try: require(operation(), name + " changed")
                except BaseException as error: report["postcheck_errors"].append(dict(check=name, error=str(error)))
            report["completed_at_utc"] = base.utc()
            if report["postcheck_errors"] or any(r["still_alive"] for r in report["child_cleanup"]): report["status"] = "FAILED_FIT_NOT_PROMOTED"
            base.write_json(arm_dir / "fit_report.json", report)
    return 0 if report["status"] == "FIT_COMPLETE_NOT_PROMOTED" else 2


def cleanup_session(process):
    # HH_260906 - start_new_session=True makes only this new Popen PID a permitted signal target; never use name-based process kills.
    result = dict(pid=process.pid, process_group=process.pid, signals=[])
    for sig, seconds in ((signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try: os.killpg(process.pid, 0)
        except ProcessLookupError: break
        os.killpg(process.pid, sig); result["signals"].append(signal.Signals(sig).name)
        try: process.wait(timeout=seconds)
        except subprocess.TimeoutExpired: pass
    try: os.killpg(process.pid, 0); result["group_absent"] = False
    except ProcessLookupError: result["group_absent"] = True
    return result


@contextmanager
def intermediate_cleanup_signals():
    # HH_260906 - Unlike terminal finalization, an inter-fit cleanup must preserve the original whole-study alarm deadline.
    signals = (signal.SIGTERM, signal.SIGINT, signal.SIGALRM)
    previous = {sig: signal.getsignal(sig) for sig in signals}
    remaining, interval = signal.getitimer(signal.ITIMER_REAL); started = time.monotonic()
    signal.setitimer(signal.ITIMER_REAL, 0)
    try:
        for sig in signals: signal.signal(sig, signal.SIG_IGN)
        yield
    finally:
        for sig, handler in previous.items(): signal.signal(sig, handler)
        if remaining > 0:
            left = remaining - (time.monotonic() - started)
            require(left > 0, "whole-study wall budget expired during owned cleanup")
            signal.setitimer(signal.ITIMER_REAL, left, interval)


def launch_fit(args, plan, arm, fd, base, report):
    arm_dir = args.output_dir / arm; arm_dir.mkdir()
    item = dict(arm=arm, num_workers=dict(ARMS)[arm], status="STARTED", started_at_utc=base.utc())
    report["arms"].append(item)
    command = [sys.executable, str(Path(__file__).absolute()), "--plan", str(args.plan), "--output-dir", str(args.output_dir),
        "--fit-arm", arm, "--lease-fd", str(fd)]
    process = None; started = time.perf_counter()
    try:
        with (arm_dir / "console.log").open("xb") as log:
            process = subprocess.Popen(command, cwd=WORKSPACE / "autoware_e2e", stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True, pass_fds=(fd,), env=dict(os.environ))
            item["pid"] = process.pid
            item["exit_code"] = process.wait(timeout=FIT_TIMEOUT)
        item["process_wall_seconds_including_python_startup_and_integrity_checks"] = time.perf_counter() - started
    finally:
        with intermediate_cleanup_signals():
            if process is not None: item["cleanup"] = cleanup_session(process)
            item["completed_at_utc"] = base.utc()
    require(item.get("exit_code") == 0 and item.get("cleanup", {}).get("group_absent") is True, "fit failed or owned session remained")
    result = base.json_read(arm_dir / "fit_report.json")
    require(result.get("status") == "FIT_COMPLETE_NOT_PROMOTED" and result.get("pid") == item["pid"]
        and result.get("arm") == arm and result.get("num_workers") == dict(ARMS)[arm], "fit result binding differs")
    item.update(status="FIT_COMPLETE_NOT_PROMOTED", fit_report_sha256=base.digest(arm_dir / "fit_report.json"),
        train_call_wall_seconds_including_final_sync=result["train_call_wall_seconds_including_final_sync"])


def finalize(output, report, checks, base, plan):
    report["postcheck_errors"] = []
    for name, operation in checks.items():
        try: require(operation(), name + " changed")
        except BaseException as error: report["postcheck_errors"].append(dict(check=name, error=str(error)))
    inventory = {}
    for path in sorted(output.rglob("*")):
        if path.is_dir() and not path.is_symlink(): continue
        try: inventory[str(path.relative_to(output))] = dict(sha256=base.digest(path), bytes=path.stat().st_size)
        except BaseException as error: report["postcheck_errors"].append(dict(check="output_inventory", path=str(path.relative_to(output)), error=str(error)))
    report.update(files=inventory, completed_at_utc=base.utc(),
        unperformed_arms=[arm for arm, _ in ARMS if arm not in {r["arm"] for r in report["arms"]}],
        deadline_met=datetime.now(timezone.utc) < base.timestamp(plan["finish_before_utc"]))
    if report["postcheck_errors"] or not report["deadline_met"]: report["status"] = "FAILED_DIAGNOSTIC_NOT_PROMOTED"
    base.write_json(output / "report.json", report)
    hashes = {name: item["sha256"] for name, item in inventory.items()}
    hashes["report.json"] = base.digest(output / "report.json")
    with (output / "SHA256SUMS").open("x") as stream:
        for name, value in sorted(hashes.items()): stream.write(value + "  " + name + "\n")


def run(args, base):
    args.plan = base.regular(args.plan); args.output_dir = args.output_dir.absolute()
    require(args.output_dir == args.output_dir.resolve(), "canonical output path required")
    plan = validate_plan(base.json_read(args.plan), base); plan_sha = base.digest(args.plan)
    repo = WORKSPACE / "autoware_e2e"
    if args.fit_arm is not None:
        with base.bounded_signals(FIT_TIMEOUT - 30):
            return fit_one(args, base, plan, repo)
    require(args.lease_fd is None, "supervisor cannot accept a foreign lease descriptor")
    base.check_environment(WORKSPACE, repo); configure_spawn()
    check_budget(plan, base, 1200 + FINAL_RESERVE)
    dataset_path = (WORKSPACE / base.DATASET).resolve(strict=True)
    output = base.checked_output(args.output_dir, dataset_path, WORKSPACE)
    before = source_identity(plan, base, repo)
    report = dict(schema=SCHEMA, status="STARTED", started_at_utc=base.utc(), plan_sha256=plan_sha,
        source_sha256=before, arms=[], approval=DENIALS,
        notice="Four diagnostic one-epoch fits, not full training campaigns. ABBA has two fixed-order repetitions, not statistical speed evidence. Fresh Python/CUDA startup, worker spawn, disk/page cache, serialization, integrity scans and final checkpoint I/O remain included as labelled. No validation/test neural inference; unchanged full-corpus integrity scans may read test metadata/JPEG. No Torch profiler traces or automatic production change.")
    with lease_fd(base) as fd, base.bounded_signals(TOTAL_TIMEOUT):
        report["gpu_preflight"] = base.gpu_idle(repo)
        output.mkdir(parents=True, exist_ok=False)
        try:
            with (output / "plan.json").open("xb") as stream: stream.write(args.plan.read_bytes())
            for name, value in before.items():
                source = HELPER_PATH if name == "frozen_helper.py" else Path(__file__) if name == "independent_worker.py" else repo / name
                destination = output / "source" / name; destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open("xb") as stream: stream.write(source.read_bytes())
                require(base.digest(destination) == value, "source archive differs")
            for index, (arm, _) in enumerate(ARMS):
                check_budget(plan, base, (4-index) * FIT_TIMEOUT + FINAL_RESERVE)
                source_identity(plan, base, repo)
                base.gpu_idle(repo)
                launch_fit(args, plan, arm, fd, base, report)
            # HH_260906 - All owned sessions exited before the supervisor imports Torch for CPU-only secure checkpoint comparison.
            require(all(r["cleanup"]["group_absent"] for r in report["arms"]), "owned children remain")
            os.environ["CUDA_VISIBLE_DEVICES"] = ""
            sys.path.insert(0, str(repo))
            import torch
            from portable_e2e import train as trainer
            require(not torch.cuda.is_initialized(), "supervisor comparison must not initialize CUDA")
            compare_outputs(output, base, torch, trainer, report)
            require(not torch.cuda.is_initialized(), "CPU comparison unexpectedly initialized CUDA")
            report["status"] = "ABBA_COMPLETE_EXACT_PARITY_NOT_PROMOTED"
        except BaseException as error:
            report.update(status="FAILED_DIAGNOSTIC_NOT_PROMOTED", error_type=type(error).__name__, error=str(error))
        finally:
            def final_corpus():
                # HH_260906 - The final study-wide validation is additional to each fit's unchanged pre/post checks.
                require(all(item.get("cleanup", {}).get("group_absent") is True for item in report["arms"]),
                        "cannot import CPU validation while owned fit sessions remain")
                os.environ["CUDA_VISIBLE_DEVICES"] = ""
                if str(repo) not in sys.path: sys.path.insert(0, str(repo))
                from portable_e2e.dataset import load_training_examples
                started = time.perf_counter()
                loaded = load_training_examples(dataset_path, split="train", mode="planning", check_image_hashes=True)
                report["final_study_full_corpus_validation_wall_seconds"] = time.perf_counter() - started
                return len(loaded.examples) == SAMPLES and loaded.validation_report["dataset_fingerprint_sha256"] == base.CORPUS_SHA
            checks = {"source": lambda: source_identity(plan, base, repo) == before,
                "plan": lambda: base.digest(args.plan) == plan_sha and base.digest(output / "plan.json") == plan_sha,
                "dataset_manifest": lambda: base.digest(dataset_path / "dataset.json") == base.MANIFEST_SHA,
                "source_archives": lambda: all(base.digest(output / "source" / name) == value for name, value in before.items()),
                "final_full_corpus": final_corpus}
            with base.finalization_signals(): finalize(output, report, checks, base, plan)
    return 0 if report["status"] == "ABBA_COMPLETE_EXACT_PARITY_NOT_PROMOTED" else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--fit-arm", choices=[arm for arm, _ in ARMS])
    parser.add_argument("--lease-fd", type=int)
    args = parser.parse_args(argv)
    return run(args, load_helper())


if __name__ == "__main__":
    raise SystemExit(main())
