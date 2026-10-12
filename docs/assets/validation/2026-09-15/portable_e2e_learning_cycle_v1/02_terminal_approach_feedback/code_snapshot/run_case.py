#!/usr/bin/env python3
"""HH_260906 - Execute one frozen v9 case with durable readiness and pre-wrapper failure records; never retry."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

BASE = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
CASES = ("01_coast_A", "02_feedback_B")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe_path(path):
    # HH_260906 - A matching file digest cannot authorize symlink-based output or source redirection.
    path = path.absolute()
    if ".." in path.parts or any(item.is_symlink() for item in (path, *path.parents)):
        raise ValueError("unsafe or symlink path: " + str(path))
    return path


def verify_pins(plan, expected_plan_sha):
    if sha(safe_path(BASE / "plan.json")) != expected_plan_sha:
        raise ValueError("prospective plan changed")
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256", "analysis_sha256"):
        for name, pin in plan[category].items():
            path = Path(name)
            if path.is_absolute() or ".." in path.parts or sha(safe_path(ROOT / path)) != pin:
                raise ValueError("prospective file identity differs: " + name)


def now():
    return datetime.now(timezone.utc).isoformat()


def write_new(path, value):
    # HH_260906 - Preserve each stage as a separate create-only receipt instead of overwriting an earlier outcome.
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--plan-sha256", required=True)
    args = parser.parse_args()
    plan_path = safe_path(BASE / "plan.json")
    if sha(plan_path) != args.plan_sha256:
        raise ValueError("prospective plan SHA mismatch")
    plan = json.loads(plan_path.read_text())
    if tuple(case["id"] for case in plan["cases"]) != CASES or plan["automatic_retry"] is not False:
        raise ValueError("prospective case order or no-retry contract differs")
    verify_pins(plan, args.plan_sha256)
    index = CASES.index(args.case)
    case = plan["cases"][index]
    if any(safe_path(BASE / name).exists() for name in CASES[index:]):
        raise ValueError("case already exists or a later case ran out of order")
    continuation = []
    if index:
        from scripts.e2e.summarize_carla_terminal_approach import summarize_one
        for prior in plan["cases"][:index]:
            report, _, _, _ = summarize_one(BASE / prior["id"], prior, plan)
            if report.get("execution_and_scalar_pixel_protocol_clear") is not True:
                raise ValueError("prior case failed the independent continuation gate: " + prior["id"])
            continuation.append({"case_id": prior["id"], "status": report["status"],
                                 "execution_and_scalar_pixel_protocol_clear": True,
                                 "all_original_future_metrics_clear": report["all_original_future_metrics_clear"]})
    execution = safe_path(BASE / "execution" / args.case)
    execution.parent.mkdir(exist_ok=True)
    execution.mkdir()
    write_new(execution / "declaration.json", {"schema": "carla.v9_case_execution_declaration.v1",
        "declared_at_utc": now(), "case_id": args.case, "arm": case["arm"], "plan_sha256": args.plan_sha256,
        "command": case["command"], "maximum_wrapper_attempts": 1, "automatic_retry": False,
        "prior_case_continuation": continuation, "training_data_approved": False})
    readiness = execution / "readiness.json"
    print("Checking fixed bounded port readiness for " + args.case, flush=True)
    try:
        checked = subprocess.run([sys.executable, "-m", "scripts.e2e.probe_carla_port_readiness",
                                  "--port", "2100", "--output", str(readiness)], cwd=ROOT)
    except (OSError, KeyboardInterrupt) as error:
        write_new(execution / "result.json", {"schema": "carla.v9_case_execution_result.v1", "case_id": args.case,
            "completed_at_utc": now(), "status": "READINESS_INVOCATION_FAILED_NOT_ATTEMPTED",
            "wrapper_attempt_count": 0, "error": type(error).__name__ + ": " + str(error),
            "automatic_retry": False, "training_data_approved": False})
        raise
    if checked.returncode != 0:
        write_new(execution / "result.json", {"schema": "carla.v9_case_execution_result.v1", "case_id": args.case,
            "completed_at_utc": now(), "status": "READINESS_BLOCKED_NOT_ATTEMPTED", "wrapper_attempt_count": 0,
            "exit_code": checked.returncode, "readiness_sha256": sha(readiness) if readiness.exists() else None,
            "automatic_retry": False, "training_data_approved": False})
        return checked.returncode
    # HH_260906 - Readiness is not a reservation; the existing owned wrapper still performs its own final admission.
    try:
        verify_pins(plan, args.plan_sha256)
    except (OSError, ValueError) as error:
        write_new(execution / "result.json", {"schema": "carla.v9_case_execution_result.v1", "case_id": args.case,
            "completed_at_utc": now(), "status": "PIN_MISMATCH_NOT_ATTEMPTED", "wrapper_attempt_count": 0,
            "error": str(error), "automatic_retry": False, "training_data_approved": False})
        raise
    command = case["command"]
    write_new(execution / "wrapper_started.json", {"schema": "carla.v9_wrapper_invocation.v1",
        "started_at_utc": now(), "case_id": args.case, "command": command,
        "readiness_sha256": sha(readiness), "wrapper_attempt_count": 1, "automatic_retry": False})
    print("Executing one declared wrapper attempt: " + args.case, flush=True)
    try:
        with (execution / "wrapper.log").open("xb") as log:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    except (OSError, KeyboardInterrupt) as error:
        # HH_260906 - Preserve uncertainty after interruption; never claim that a possibly running child was cleaned up.
        write_new(execution / "result.json", {"schema": "carla.v9_case_execution_result.v1", "case_id": args.case,
            "completed_at_utc": now(), "status": "WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED",
            "wrapper_attempt_count": 1, "error": type(error).__name__ + ": " + str(error),
            "automatic_retry": False, "training_data_approved": False})
        raise
    owner = BASE / args.case / "owner_result.json"
    write_new(execution / "result.json", {"schema": "carla.v9_case_execution_result.v1",
        "case_id": args.case, "arm": case["arm"], "completed_at_utc": now(), "wrapper_attempt_count": 1,
        "exit_code": result.returncode, "status": "WRAPPER_COMPLETED" if result.returncode == 0 else
            "WRAPPER_FAILED_WITH_OUTPUT" if (BASE / args.case).exists() else "PRELAUNCH_BLOCKED_NOT_CAPTURED",
        "readiness_sha256": sha(readiness), "wrapper_log_sha256": sha(execution / "wrapper.log"),
        "owner_result_sha256": sha(owner) if owner.exists() else None,
        "automatic_retry": False, "training_data_approved": False})
    print(json.dumps({"case": args.case, "wrapper_exit_code": result.returncode,
                      "execution_record": str(execution / "result.json")}), flush=True)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
