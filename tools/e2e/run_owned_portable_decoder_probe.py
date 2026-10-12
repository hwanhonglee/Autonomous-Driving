#!/usr/bin/env python3
"""HH_260906 - Bound one numerical decoder diagnostic to the existing personal venv and owned GPU0 lease."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from scripts.e2e import run_portable_training_campaign as shared

WORKSPACE = shared.WORKSPACE
REPO = WORKSPACE / "autoware_e2e"
INPUT = "datasets/diagnostics/carla-v4-decoder-inputs-20260908-v1"
OUTPUT = "runs/diagnostics/hh260908-decoder-representability-v1"
SCRIPT = "scripts/e2e/probe_portable_decoder_representability.py"
DEADLINE = "2026-09-08T01:00:00Z"


def require(value, message):
    if not value:
        raise RuntimeError(message)


def budget(deadline, *, now=None):
    # HH_260906 - Reserve cooperative work, independent wall cap, owned escalation and final evidence before the user boundary.
    require(deadline == DEADLINE, "unreviewed user work boundary")
    remaining = (datetime.fromisoformat(deadline.replace("Z", "+00:00"))
        - (datetime.now(timezone.utc) if now is None else now)).total_seconds()
    require(remaining >= 4200, "insufficient time for 3900-second outer cap and cleanup reserve")
    return remaining


def validate_plan(plan):
    require(plan.get("schema") == "portable_e2e.owned_decoder_probe_plan.v1", "unreviewed decoder campaign")
    for field, value in (("gpu_uuid", shared.GPU_UUID), ("input_relative", INPUT), ("output_relative", OUTPUT),
                         ("finish_before_utc", DEADLINE), ("cooperative_wall_seconds", 3600),
                         ("outer_wall_seconds", 3900), ("causal_model_training", False),
                         ("training_data_approved", False), ("automatic_retry", False)):
        require(type(plan.get(field)) is type(value) and plan[field] == value, f"unreviewed {field}")
    require(re.fullmatch(r"[0-9a-f]{40}", plan.get("source_commit", "")), "source commit must be exact")
    for field in ("probe_sha256", "input_manifest_sha256", "runner_sha256", "shared_runner_sha256"):
        require(re.fullmatch(r"[0-9a-f]{64}", plan.get(field, "")), f"missing exact {field}")
    return plan


def verify(plan):
    # HH_260906 - Do not mutate, reset, build or install into the remote repository.
    require(shared.run_inventory(["git", "rev-parse", "HEAD"], REPO) == plan["source_commit"], "remote HEAD differs")
    require(not shared.run_inventory(["git", "status", "--porcelain", "--untracked-files=all"], REPO), "remote worktree is not clean")
    for name, key in ((SCRIPT, "probe_sha256"), ("scripts/e2e/run_owned_portable_decoder_probe.py", "runner_sha256"),
                      ("scripts/e2e/run_portable_training_campaign.py", "shared_runner_sha256")):
        require(shared.digest(REPO / name) == plan[key], "reviewed execution source changed")
    require(shared.digest(WORKSPACE / INPUT / "manifest.json") == plan["input_manifest_sha256"], "diagnostic input manifest changed")


def environment():
    env = shared.stage_environment()
    env.update(OMP_NUM_THREADS="4", MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4", NUMEXPR_NUM_THREADS="4")
    return env


def write_new(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def terminate_owned(child):
    # HH_260906 - Only this freshly created child session can receive termination; no GPU PID lookup is used for killing.
    if child is None or child.poll() is not None:
        return
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        child.wait(timeout=90)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=5)


@contextmanager
def cleanup_signal_guard():
    # HH_260906 - A repeated termination request must not abandon the owned child or release its GPU lease early.
    previous = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        for number in previous:
            signal.signal(number, signal.SIG_IGN)
        yield
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)


def run(plan_path):
    plan_path = Path(plan_path)
    plan_bytes = plan_path.read_bytes()
    plan = validate_plan(json.loads(plan_bytes))
    require(Path(sys.prefix) == WORKSPACE / "venvs/py312", "use only the existing personal py312 venv")
    require(Path(__file__).resolve().parent.parent.parent == REPO, "runner is outside approved training repository")
    require((WORKSPACE / INPUT).resolve().is_relative_to(WORKSPACE.parent / "dataset"), "input is outside personal dataset directory")
    output = WORKSPACE / OUTPUT
    require(not output.exists() and not output.is_symlink(), "one create-only execution; no retry or overwrite")
    require(output.parent.resolve().is_relative_to(WORKSPACE), "output parent escapes personal workspace")
    budget(plan["finish_before_utc"])
    verify(plan)
    lease_path = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
    require(not lease_path.is_symlink(), "cooperative GPU lease must not be a symlink")
    with shared.termination_guard(), lease_path.open("a+") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        shared.assert_gpu_idle(REPO)
        budget(plan["finish_before_utc"])
        verify(plan)
        output.mkdir(parents=True, exist_ok=False)
        command = [str(WORKSPACE / "venvs/py312/bin/python"), "-m", "scripts.e2e.probe_portable_decoder_representability",
            "probe", "--input-root", str(WORKSPACE / INPUT), "--output-dir", str(output / "probe"),
            "--expected-manifest-sha256", plan["input_manifest_sha256"], "--expected-script-sha256", plan["probe_sha256"],
            "--device", "cuda:0", "--max-wall-seconds", "3600"]
        start, child, failure, returncode = time.monotonic(), None, None, None
        write_new(output / "owner_started.json", {"schema": "portable_e2e.owned_decoder_probe.v1", "status": "RUNNING",
            "started_at_utc": shared.now(), "owner_pid": os.getpid(), "plan": plan,
            "plan_file_sha256": hashlib.sha256(plan_bytes).hexdigest(), "gpu0_idle_checked": True,
            "causal_model_training": False, "vehicle_control_approved": False})
        try:
            with (output / "probe.log").open("x") as log:
                child = subprocess.Popen(command, cwd=REPO, env=environment(), stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                write_new(output / "owner_child.json", {"pid": child.pid, "pgid": child.pid,
                    "started_at_utc": shared.now(), "command": command})
                returncode = child.wait(timeout=3900)
                if returncode != 0:
                    failure = "probe_nonzero_exit"
        except (Exception, KeyboardInterrupt) as error:
            failure = f"{type(error).__name__}: {error}"
        finally:
            with cleanup_signal_guard():
                try:
                    terminate_owned(child)
                except Exception as error:
                    failure = f"owned_cleanup_failed: {error}"
                if child is not None:
                    returncode = child.poll()
                try:
                    verify(plan)
                    require(plan_path.read_bytes() == plan_bytes, "execution plan changed")
                    postcheck = True
                except Exception as error:
                    postcheck, failure = False, f"postcheck_failed: {error}"
                summary_path = output / "probe/summary.json"
                summary, summary_sha = {}, None
                try:
                    if summary_path.is_file():
                        summary = json.loads(summary_path.read_text())
                        require(isinstance(summary, dict), "malformed probe summary")
                        summary_sha = shared.digest(summary_path)
                except Exception as error:
                    failure = f"probe_summary_unreadable: {error}"
                complete = failure is None and returncode == 0 and postcheck and summary.get("status") == "COMPLETE_NOT_ADMITTED"
                write_new(output / "owner_result.json", {"status": "COMPLETE_NOT_ADMITTED" if complete else "FAILED_OR_PARTIAL_NOT_ADMITTED",
                    "completed_at_utc": shared.now(), "elapsed_wall_seconds": time.monotonic() - start,
                    "returncode": returncode, "child_exited": child is None or child.poll() is not None,
                    "failure": failure, "source_and_plan_postcheck_pass": postcheck,
                    "probe_summary_sha256": summary_sha,
                    "training_data_approved": False, "causal_model_training": False, "vehicle_control_approved": False})
        return 0 if complete else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", type=Path, required=True)
    return run(parser.parse_args(argv).plan)


if __name__ == "__main__":
    raise SystemExit(main())
