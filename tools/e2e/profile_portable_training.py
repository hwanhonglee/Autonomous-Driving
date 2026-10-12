#!/usr/bin/env python3
"""HH_260906 - Profile two fresh real sixteen-step fits without changing the trainer or admitting data."""

from __future__ import annotations

import argparse
from contextlib import contextmanager, nullcontext
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import time


SCHEMA = "portable_e2e.training_profiler_diagnostic.v1"
PLAN_SCHEMA = "portable_e2e.training_profiler_plan.v1"
SOURCE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
WORKSPACE = Path.home() / "personal/hwanhong/portable_e2e"
DATASET = "datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3"
MANIFEST_SHA = "18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c"
CORPUS_SHA = "56d9ff663612a090cd61698b53cf0cf39b0a7ae0de6b42d096957a94b6c92047"
TRAIN_SHA = "d957e72c1eea755fed5b4ac682e775983861c16104fe083b7cfb6cbb4fed1928"
MODEL_CONFIG = "portable_e2e/config/perspective_trajectory_physical_v1.model.json"
MODEL_SHA = "e96e31c96cafa41b57b67b9531ae9fff6bf21fd7e418ea78f9062fcb1dcfe74d"
SOURCE_PATHS = ("portable_e2e/__init__.py", "portable_e2e/train.py", "portable_e2e/model.py",
    "portable_e2e/losses.py", "portable_e2e/contract.py", "portable_e2e/dataset.py",
    "portable_e2e/torch_dataset.py", "portable_e2e/stop_primitive_research.py",
    "portable_e2e/runtime_contract.py", "portable_e2e/config/common_10hz_v1.contract.json", MODEL_CONFIG)
TRAIN_CONFIG = dict(seed=20260903, batch_size=4, learning_rate=1e-4, weight_decay=1e-4,
    max_steps=16, checkpoint_interval=16, num_workers=0, maximum_gradient_norm=5.0,
    verify_image_sha256=True, sampling_policy="uniform_without_replacement", domain_ratios=[])
LOSS_CONFIG = dict(xy_weight=1., speed_weight=.2, yaw_weight=.1, kinematic_speed_weight=.05,
    final_displacement_weight=.5, candidate_score_weight=.1)
ARMS = ("A_unprofiled", "B_cpu_cuda_profiled")
PROFILE_OPTIONS = dict(record_shapes=False, with_stack=False, profile_memory=False,
    with_flops=False, with_modules=False)
DENIALS = dict(vehicle_control_approved=False, data_admission=False, model_promotion=False,
    test_neural_network_use=False, production_source_changes=False)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def utc():
    return datetime.now(timezone.utc).isoformat()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)),
            "input must be a regular file without symlink components")
    return path


def digest(path):
    with regular(path).open("rb") as stream:
        h = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def json_read(path):
    def pairs(items):
        value = {}
        for key, child in items:
            require(key not in value, "duplicate JSON key")
            value[key] = child
        return value
    return json.loads(regular(path).read_bytes(), object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON")))


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def timestamp(value):
    require(isinstance(value, str), "timestamp must be an explicit UTC string")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    require(parsed.tzinfo is not None and parsed.utcoffset().total_seconds() == 0, "UTC timestamp required")
    return parsed


def validate_plan(plan):
    # HH_260906 - The diagnostic has one immutable small schedule, not a general training launcher.
    expected = dict(schema=PLAN_SCHEMA, diagnostic_id="hh260909-training-profiler-16step-pair-v1",
        source_commit=SOURCE_COMMIT, gpu_uuid=GPU_UUID, dataset=DATASET,
        dataset_manifest_sha256=MANIFEST_SHA, corpus_fingerprint_sha256=CORPUS_SHA,
        train_fingerprint_sha256=TRAIN_SHA, model_config=MODEL_CONFIG, model_config_sha256=MODEL_SHA,
        expected_train_samples=1147, expected_train_episodes=3, train_config=TRAIN_CONFIG,
        loss_config=LOSS_CONFIG, arms=list(ARMS), profiler_options=PROFILE_OPTIONS,
        checkpoint_ignored_fields=["created_at_utc"], finish_before_utc="2026-09-09T01:00:00Z",
        external_wall_timeout_seconds=360, internal_wall_timeout_seconds=330,
        safety_reserve_seconds=60, approval=DENIALS)
    require(isinstance(plan, dict), "plan must be an object")
    for key, value in expected.items():
        require(json.dumps(plan.get(key), sort_keys=True) == json.dumps(value, sort_keys=True),
                "unreviewed plan field: " + key)
    require(set(plan.get("source_sha256", {})) == set(SOURCE_PATHS), "source inventory differs")
    for value in [*plan["source_sha256"].values(), plan.get("profiler_source_sha256")]:
        require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value), "source SHA required")
    require(timestamp(plan["declared_at_utc"]) < timestamp(plan["finish_before_utc"]), "invalid declaration time")
    return plan


def verify_budget(plan, observed=None):
    observed = observed or datetime.now(timezone.utc)
    require(timestamp(plan["declared_at_utc"]) <= observed, "plan declaration is in the future")
    require((timestamp(plan["finish_before_utc"]) - observed).total_seconds() >= 420,
            "reserve the entire external 360 seconds plus 60 seconds before the user deadline")


def source_identity(plan, repo, script):
    # HH_260906 - New profiler bytes are pinned separately; unchanged trainer bytes must also match reviewed offline Git.
    result = {}
    for name in SOURCE_PATHS:
        payload = regular(repo / name).read_bytes()
        # HH_260906 - Historical verification is offline-only, including partial-clone repositories with missing objects.
        git_env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
        historical = subprocess.check_output(["git", "-c", "protocol.allow=never", "show", SOURCE_COMMIT + ":" + name],
            cwd=repo, env=git_env, timeout=10)
        require(payload == historical and sha(payload) == plan["source_sha256"][name], "training source mismatch: " + name)
        result[name] = sha(payload)
    require(result[MODEL_CONFIG] == MODEL_SHA, "physical model config differs")
    result["independent_profiler.py"] = digest(script)
    require(result["independent_profiler.py"] == plan["profiler_source_sha256"], "profiler source mismatch")
    return result


def checked_output(output, dataset, workspace):
    output = Path(output).absolute()
    require(not output.exists() and not output.is_symlink(), "output already exists")
    require(not any(p.is_symlink() for p in output.parents), "output parents must not be symlinks")
    require(output.is_relative_to(workspace / "runs/diagnostics") and output != workspace / "runs/diagnostics",
            "fresh output must be inside personal runs/diagnostics")
    require(not output.resolve().is_relative_to(dataset.resolve()), "output must not be inside the dataset")
    return output


def check_environment(workspace, repo):
    require(Path(sys.prefix).absolute() == workspace / "venvs/py312", "personal py312 venv required")
    require(Path.cwd() == repo and repo.resolve() == repo, "run from the real personal repository")
    expected = dict(CUDA_VISIBLE_DEVICES=GPU_UUID, CUBLAS_WORKSPACE_CONFIG=":4096:8",
        PYTHONNOUSERSITE="1", OMP_NUM_THREADS="16", MKL_NUM_THREADS="16")
    require(all(os.environ.get(k) == v for k, v in expected.items()), "explicit GPU0/deterministic/thread environment required")
    require(not any(os.environ.get(k) for k in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE")),
            "inherited Python search overrides are prohibited")
    require("torch" not in sys.modules, "GPU lease and idle check must precede importing torch")
    require(not any(name == "portable_e2e" or name.startswith("portable_e2e.") for name in sys.modules),
            "verified training imports must not reuse preloaded project modules")


@contextmanager
def gpu_lease(workspace):
    path = workspace / "runs/campaigns/.gpu0_training.lock"
    require(path.parent.is_dir() and path.parent.resolve() == path.parent and not path.is_symlink(), "unsafe lease path")
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        os.close(fd)


def gpu_idle(repo):
    def query(fields, kind):
        return subprocess.check_output(["nvidia-smi", "-i", "0", "--query-" + kind + "=" + fields,
            "--format=csv,noheader"], cwd=repo, text=True, timeout=10).strip()
    identity = query("index,uuid", "gpu")
    require(identity == "0, " + GPU_UUID, "physical GPU0 identity changed")
    processes = query("gpu_uuid,pid", "compute-apps")
    require(not processes, "GPU0 is occupied; no foreign process may be stopped")
    return dict(physical_index=0, uuid=GPU_UUID, pre_cuda_compute_processes=0)


def verify_torch_uuid(properties):
    # HH_260906 - The recorded PyTorch ABI uses the exact canonical UUID without NVIDIA-SMI's GPU- prefix.
    observed = str(getattr(properties, "uuid", None))
    require(observed == GPU_UUID.removeprefix("GPU-"), "torch GPU UUID differs")
    return observed


@contextmanager
def bounded_signals(seconds):
    # HH_260906 - External timeout is still mandatory for native calls that cannot process a Python signal promptly.
    def stop(signum, _frame):
        raise TimeoutError("bounded profiler interrupted by signal " + str(signum))
    old = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM)}
    try:
        for sig in old:
            signal.signal(sig, stop)
        signal.alarm(seconds)
        yield
    finally:
        signal.alarm(0)
        for sig, handler in old.items():
            signal.signal(sig, handler)


@contextmanager
def finalization_signals():
    # HH_260906 - Preserve partial evidence against repeated cooperative signals; an external SIGKILL cannot be made recoverable.
    signals = (signal.SIGTERM, signal.SIGINT, signal.SIGALRM)
    old = {sig: signal.getsignal(sig) for sig in signals}
    signal.alarm(0)
    try:
        for sig in signals:
            signal.signal(sig, signal.SIG_IGN)
        yield
    finally:
        for sig, handler in old.items():
            signal.signal(sig, handler)


def exact_difference(a, b, torch, path="$", *, ignore_created=False):
    # HH_260906 - Compare structured checkpoint contents bit-for-bit; never silently ignore optimizer/RNG/device metadata.
    if isinstance(a, torch.Tensor) or isinstance(b, torch.Tensor):
        if not isinstance(a, torch.Tensor) or not isinstance(b, torch.Tensor):
            return path + ": tensor type mismatch"
        if a.dtype != b.dtype or a.shape != b.shape or a.layout != b.layout or a.requires_grad != b.requires_grad:
            return path + ": tensor ABI mismatch"
        if a.layout != torch.strided:
            return path + ": unsupported tensor layout"
        if a.stride() != b.stride():
            return path + ": tensor strides differ"
        if (a.is_floating_point() or a.is_complex()) and not bool(torch.isfinite(a).all() & torch.isfinite(b).all()):
            return path + ": nonfinite tensor"
        # HH_260906 - The unchanged float32 trainer and integer RNG/step tensors support NumPy byte views even on old CPU test Torch.
        av = a.detach().cpu().contiguous().numpy().tobytes()
        bv = b.detach().cpu().contiguous().numpy().tobytes()
        return None if av == bv else path + ": tensor bytes differ"
    if type(a) is not type(b):
        return path + ": scalar/container type mismatch"
    if isinstance(a, dict):
        ignored = {"created_at_utc"} if ignore_created and path == "$" else set()
        if ignore_created and path == "$" and ("created_at_utc" not in a or "created_at_utc" not in b):
            return path + ": missing sole ignored creation timestamp"
        if set(a) - ignored != set(b) - ignored:
            return path + ": mapping keys differ"
        for key in a:
            if key not in ignored:
                issue = exact_difference(a[key], b[key], torch, path + "/" + str(key))
                if issue:
                    return issue
        return None
    if isinstance(a, (list, tuple)):
        if len(a) != len(b):
            return path + ": sequence length differs"
        for index, (av, bv) in enumerate(zip(a, b)):
            issue = exact_difference(av, bv, torch, path + "/" + str(index))
            if issue:
                return issue
        return None
    if isinstance(a, float):
        return None if math.isfinite(a) and math.isfinite(b) and struct.pack("!d", a) == struct.pack("!d", b) else path + ": float differs/nonfinite"
    require(a is None or isinstance(a, (str, bool, int)), "unsupported checkpoint scalar type")
    return None if a == b else path + ": scalar differs"


def read_metrics(path):
    raw = regular(path).read_bytes()
    require(raw.endswith(b"\n"), "metrics must retain complete newline-terminated records")
    rows = [json.loads(line, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("nonfinite metrics")))
            for line in raw.splitlines()]
    require(len(rows) == 16, "every arm must retain exactly sixteen real metric rows")
    for index, row in enumerate(rows, 1):
        require(type(row.get("global_step")) is int and row["global_step"] == index and row.get("samples_seen") == index * 4,
                "metrics step/exposure sequence differs")
        for key in ("loss", "gradient_norm", "regression_loss", "candidate_score_loss"):
            require(type(row.get(key)) in (int, float) and math.isfinite(row[key]), "nonfinite/missing metric: " + key)
    return rows


def profile_rows(profiler):
    rows = []
    for event in profiler.key_averages():
        row = dict(key=str(event.key), count=int(event.count), device_type=str(event.device_type))
        for key in ("cpu_time_total", "self_cpu_time_total", "device_time_total", "self_device_time_total"):
            value = float(getattr(event, key))
            require(math.isfinite(value) and value >= 0, "invalid profiler aggregate: " + key)
            row[key + "_us"] = value
        require(row["count"] > 0, "empty profiler aggregate")
        rows.append(row)
    require(rows, "profiler produced no aggregate records")
    return rows


def train_pair(*, output, dataset, model_config, torch, trainer, loss_config, train_config, report):
    common = dict(dataset=dataset, dataset_fingerprint_sha256=TRAIN_SHA, corpus_fingerprint_sha256=CORPUS_SHA,
        model_config=model_config, train_config=train_config, loss_config=loss_config,
        device_name="cuda:0", training_split="train", resume=False)
    activities = [torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA]
    require(set(activities).issubset(torch.profiler.supported_activities()), "CPU/CUDA profiler activities unavailable")
    for arm in ARMS:
        item = dict(arm=arm, status="STARTED", started_at_utc=utc(), profiling=arm == ARMS[1])
        report["arms"].append(item)
        torch.cuda.synchronize(0)
        start = time.perf_counter()
        context = torch.profiler.profile(activities=activities, **PROFILE_OPTIONS) if item["profiling"] else nullcontext()
        with context as profiler:
            call_start = time.perf_counter()
            result = trainer.train_model(run_dir=output / arm, **common)
            torch.cuda.synchronize(0)
            item["train_call_wall_seconds_including_final_sync"] = time.perf_counter() - call_start
        item["context_wall_seconds_including_profiler_start_stop"] = time.perf_counter() - start
        require(result.get("status") == "TRAINING_TARGET_REACHED" and
            result.get("state", {}).get("global_step") == 16 and result["state"].get("samples_seen") == 64,
            "trainer did not complete the exact small diagnostic fit")
        item.update(status="FIT_COMPLETE", completed_at_utc=utc(), state=result["state"])
        if item["profiling"]:
            write_json(output / "key_averages.json", dict(schema=SCHEMA, units="microseconds", rows=profile_rows(profiler),
                notice="Inclusive times overlap; self times and CUDA kernels are not whole-wall utilization percentages."))
            trace_path = output / "cpu_cuda_trace.json"
            require(not trace_path.exists(), "trace output must be fresh")
            profiler.export_chrome_trace(str(trace_path))
            trace = json_read(trace_path)
            events = trace.get("traceEvents", [])
            require(isinstance(events, list), "invalid profiler trace")
            kernels = sum(event.get("cat") == "kernel" for event in events if isinstance(event, dict))
            item["trace_event_count"] = len(events)
            item["cuda_kernel_event_count"] = kernels
            require(kernels > 0, "requested CUDA profiling did not retain CUDA kernel events")
    rows = [read_metrics(output / arm / "metrics.jsonl") for arm in ARMS]
    issue = exact_difference(rows[0], rows[1], torch)
    report["metrics_parity"] = dict(rows_per_arm=16, total_retained_rows=32, exact=issue is None, first_difference=issue)
    checkpoints = [trainer._read_checkpoint_file(output / arm / "checkpoints/latest.pt", torch.device("cpu")) for arm in ARMS]
    for index, (payload, checkpoint_sha) in enumerate(checkpoints):
        report["arms"][index]["checkpoint_sha256"] = checkpoint_sha
        require(payload.get("state", {}).get("global_step") == 16 and payload["state"].get("samples_seen") == 64,
                "checkpoint did not retain all sixteen updates")
    checkpoint_issue = exact_difference(checkpoints[0][0], checkpoints[1][0], torch, ignore_created=True)
    report["checkpoint_parity"] = dict(exact=checkpoint_issue is None, first_difference=checkpoint_issue,
        ignored_top_level_fields=["created_at_utc"], weights_only=True)
    require(issue is None and checkpoint_issue is None, "profiler changed metrics or structured final checkpoint; no equivalence claim")


def finalize_output(output, report, checks, deadline):
    # HH_260906 - Failed, missing or modified inputs must not prevent persisting the usable partial diagnostic directory.
    report["postcheck_errors"] = []
    for name, check in checks.items():
        try:
            require(check(), name + " changed")
        except BaseException as caught:
            report["postcheck_errors"].append(dict(check=name, error_type=type(caught).__name__, error=str(caught)))
    report["files"] = {}
    for path in sorted(output.rglob("*")):
        if path.is_symlink() or path.is_file():
            try:
                report["files"][str(path.relative_to(output))] = dict(sha256=digest(path), bytes=path.stat().st_size)
            except BaseException as caught:
                report["postcheck_errors"].append(dict(check="output_inventory", path=str(path.relative_to(output)),
                    error_type=type(caught).__name__, error=str(caught)))
    report["completed_at_utc"] = utc()
    report["deadline_met"] = datetime.now(timezone.utc) < timestamp(deadline)
    report["unperformed_arms"] = [arm for arm in ARMS if arm not in {item["arm"] for item in report["arms"]}]
    if report["postcheck_errors"] or not report["deadline_met"]:
        report["status"] = "FAILED_DIAGNOSTIC_NOT_PROMOTED"
    write_json(output / "report.json", report)
    # HH_260906 - Only successfully inventoried regular files enter the checksum manifest; unsafe files remain disclosed failures.
    files = {name: record["sha256"] for name, record in report["files"].items()}
    files["report.json"] = digest(output / "report.json")
    with (output / "SHA256SUMS").open("x") as stream:
        for name, value in sorted(files.items()):
            stream.write(value + "  " + name + "\n")


def run(plan_path, output_dir):
    plan_path = regular(plan_path)
    plan = validate_plan(json_read(plan_path))
    plan_sha = digest(plan_path)
    repo = WORKSPACE / "autoware_e2e"
    check_environment(WORKSPACE, repo)
    verify_budget(plan)
    dataset_path = (WORKSPACE / DATASET).resolve(strict=True)
    require(dataset_path.is_relative_to((WORKSPACE.parent / "dataset").resolve()), "dataset escapes personal data root")
    output = checked_output(output_dir, dataset_path, WORKSPACE)
    script = regular(__file__)
    sources = source_identity(plan, repo, script)
    require(digest(dataset_path / "dataset.json") == MANIFEST_SHA, "dataset manifest mismatch")
    report = dict(schema=SCHEMA, status="STARTED", started_at_utc=utc(), plan_sha256=plan_sha,
        source_commit=SOURCE_COMMIT, source_sha256=sources, arms=[], approval=DENIALS,
        notice="Two fresh whole-model diagnostic fits, not full campaigns or fair throughput A/B. A precedes B; initialization, caches and profiler overhead differ. Safety checks and original num_workers=0 remain unchanged. Full-corpus integrity scans may open test JSON/JPEG, but no test sample enters the neural network. Traces are private pending path review.")
    error = None
    with gpu_lease(WORKSPACE), bounded_signals(330):
        report["gpu_preflight"] = gpu_idle(repo)
        verify_budget(plan)
        output.mkdir(parents=True, exist_ok=False)
        try:
            (output / "plan.json").write_bytes(plan_path.read_bytes())
            for name, value in sources.items():
                source = script if name == "independent_profiler.py" else repo / name
                destination = output / "source" / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                with destination.open("xb") as stream:
                    stream.write(source.read_bytes())
                require(digest(destination) == value, "source archive differs")
            # HH_260906 - No torch/model import or CUDA context exists until the personal cooperative lease and idle proof succeed.
            import torch
            from portable_e2e import train as trainer
            from portable_e2e.dataset import load_training_examples
            from portable_e2e.model import ModelConfig
            from portable_e2e.losses import TrajectoryLossConfig
            from portable_e2e.torch_dataset import Common10TorchDataset
            require(Path(trainer.__file__).resolve() == repo / "portable_e2e/train.py", "unexpected imported trainer path")
            require(torch.cuda.device_count() == 1, "only assigned physical GPU0 may be visible")
            report["torch_device_uuid"] = verify_torch_uuid(torch.cuda.get_device_properties(0))
            require(torch.get_num_threads() == 16, "actual torch CPU thread count differs")
            config = ModelConfig.from_mapping(json_read(repo / MODEL_CONFIG))
            started = time.perf_counter()
            loaded = load_training_examples(dataset_path, split="train", mode="planning", check_image_hashes=True)
            dataset = Common10TorchDataset(loaded.examples, config, verify_image_sha256=True, split="train")
            report["initial_full_corpus_validation_wall_seconds"] = time.perf_counter() - started
            require(len(dataset) == 1147 and len({e.episode_id for e in dataset.examples}) == 3,
                    "complete legacy development train split required")
            require(dataset.fingerprint_sha256 == TRAIN_SHA and loaded.validation_report["dataset_fingerprint_sha256"] == CORPUS_SHA,
                    "selected train or full corpus fingerprint mismatch")
            cfg = {**TRAIN_CONFIG, "domain_ratios": ()}
            train_pair(output=output, dataset=dataset, model_config=config, torch=torch, trainer=trainer,
                loss_config=TrajectoryLossConfig(**LOSS_CONFIG), train_config=trainer.TrainConfig(**cfg), report=report)
            started = time.perf_counter()
            after = load_training_examples(dataset_path, split="train", mode="planning", check_image_hashes=True)
            require(after.fingerprint_sha256 == loaded.fingerprint_sha256 and
                after.validation_report["dataset_fingerprint_sha256"] == CORPUS_SHA, "dataset changed during diagnostic")
            report["final_full_corpus_validation_wall_seconds"] = time.perf_counter() - started
            report["status"] = "PROFILE_COMPLETE_EXACT_PARITY_NOT_PROMOTED"
        except BaseException as caught:
            error = caught
            report.update(status="FAILED_DIAGNOSTIC_NOT_PROMOTED", error_type=type(caught).__name__, error=str(caught))
        finally:
            checks = {
                "source": lambda: source_identity(plan, repo, script) == sources,
                "plan": lambda: digest(plan_path) == plan_sha and digest(output / "plan.json") == plan_sha,
                "dataset_manifest": lambda: digest(dataset_path / "dataset.json") == MANIFEST_SHA,
                "source_archives": lambda: all(digest(output / "source" / name) == value for name, value in sources.items()),
            }
            with finalization_signals():
                finalize_output(output, report, checks, plan["finish_before_utc"])
    if error is not None:
        raise error
    require(report["status"] == "PROFILE_COMPLETE_EXACT_PARITY_NOT_PROMOTED", "diagnostic postchecks failed")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    report = run(args.plan, args.output_dir)
    print(json.dumps({k: report[k] for k in ("status", "plan_sha256", "metrics_parity", "checkpoint_parity")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
