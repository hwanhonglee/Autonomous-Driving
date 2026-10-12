"""HH_261004 - Dispatch one fixed private GPU0 branch-only TRAIN experiment; never install or deploy."""
from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PERSONAL = "/home/pro6000/personal/hwanhong"
WORKSPACE = PERSONAL + "/portable_e2e"
STUDY = "temporal_route_branch_geometry_pilot_20261004_v1"
SOURCE = WORKSPACE + "/source_worktrees/" + STUDY
AUTHORITY = WORKSPACE + "/plans/training/" + STUDY + "/ROOT_TRAIN_only_authorization.json"
OUTPUT = WORKSPACE + "/runs/training/" + STUDY
PLAN = ROOT / "storage/experiments/plans/2026-10-04/temporal_route_branch_geometry_pilot_v1"
GRANT = PLAN / "ROOT_TRAIN_only_authorization.json"
TEST = "learning/tests/test_temporal_route_branch_geometry_pilot_v1_20261004.py"
PYTHON = WORKSPACE + "/venvs/py312/bin/python"
UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "pro6000-training"]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    require(path.resolve(strict=True) == path and path.is_file(), "canonical nonsymlink source required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path, document):
    # HH_261004 - Every attempted operation has a new receipt; never replace a failed attempt.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")


def frozen_inputs():
    grant = json.loads(GRANT.read_text())
    require(grant["authorized"] is True and type(grant["physical_GPU_index"]) is int
            and grant["physical_GPU_index"] == 0 and grant["model_promoted"] is False
            and grant["vehicle_control_approved"] is False, "TRAIN-only private GPU0 grant required")
    contract = grant["contract"]
    require(len(contract["sources"]) == 36 and contract["optimizer_updates"] == 1000
            and contract["exposures"] == 5000 and contract["trainable_tensor_count"] == 4
            and contract["frozen_tensor_count"] == 97, "exact fixed branch-only intervention required")
    records = []
    for relative, expected in dict(contract["sources"], **{TEST: grant["CPU_test_sha256"]}).items():
        path = ROOT / relative
        require(digest(path) == expected, "prospectively sealed source changed: " + relative)
        records.append(dict(relative=relative, sha256=expected,
                            data=base64.b64encode(path.read_bytes()).decode("ascii")))
    records.append(dict(relative=None, sha256=digest(GRANT),
                        data=base64.b64encode(GRANT.read_bytes()).decode("ascii")))
    require(len(records) == 38, "exact36 sources, CPU test and fresh grant required")
    return grant, records


REMOTE_STAGE = r'''
# HH_261004 - Stage authenticated bytes only into an exclusive fresh personal TRAIN source tree.
import base64, hashlib, json, os
from pathlib import Path
data = json.load(__import__('sys').stdin)
w = Path('/home/pro6000/personal/hwanhong/portable_e2e')
s = w / 'source_worktrees/temporal_route_branch_geometry_pilot_20261004_v1'
a = w / 'plans/training/temporal_route_branch_geometry_pilot_20261004_v1/ROOT_TRAIN_only_authorization.json'
def safe(path):
    assert path.is_relative_to(w) and path.resolve() == path
    assert not any(p.is_symlink() for p in (path, *path.parents))
safe(w); assert w.is_dir() and not s.exists() and not a.parent.exists()
assert len(data['records']) == 38 and data['workspace'] == str(w)
validated = []
for row in data['records']:
    target = a if row['relative'] is None else s / row['relative']
    safe(target)
    raw = base64.b64decode(row['data'], validate=True)
    assert len(raw) <= 2097152 and hashlib.sha256(raw).hexdigest() == row['sha256']
    assert target == a or target.is_relative_to(s)
    validated.append((target, raw, row['sha256']))
assert len(set(p for p, _, _ in validated)) == 38
s.mkdir(parents=True, exist_ok=False); a.parent.mkdir(parents=True, exist_ok=False)
for target, raw, expected in validated:
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    assert hashlib.sha256(target.read_bytes()).hexdigest() == expected
print(json.dumps(dict(status='STAGED38_CREATE_ONLY_PRIVATE_FILES', source_root=str(s),
    authorization=str(a), sha256={str(p): h for p, _, h in validated})))
'''


def remote(argv, phase, payload=None):
    command = SSH + [shlex.join(argv)]
    print(json.dumps(dict(status="RUNNING_REMOTE_" + phase, environment="PRO6000 private venv",
                          physical_GPU_index=0, maximum_optimizer_updates=1000)), flush=True)
    result = subprocess.run(command, input=payload, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    require(len(result.stdout) <= 32 * 1024**2 and len(result.stderr) <= 8 * 1024**2,
            "bounded actual remote result required")
    receipt = dict(phase=phase, UTC=datetime.now(timezone.utc).isoformat(), argv=command,
        returncode=result.returncode, stdout=result.stdout.decode(), stderr=result.stderr.decode(),
        dispatcher_sha256=digest(Path(__file__)), authority_sha256=digest(GRANT))
    write_new(PLAN / ("ROOT_remote_" + phase + ".json"), receipt)
    require(result.returncode == 0, "remote " + phase + " failed; complete failure receipt retained")
    return receipt


def command(mode, grant_sha):
    # HH_261004 - Preserve exact qualified original asset paths, schedule and strict numerical environment.
    argv = ["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", "-u", "PYTHONUSERBASE", "-u", "PYTHONSTARTUP",
        "PYTHONNOUSERSITE=1", "OMP_NUM_THREADS=4", "CUBLAS_WORKSPACE_CONFIG=:4096:8",
        "CUDA_VISIBLE_DEVICES=" + (UUID if mode == "fit" else ""), PYTHON, "-B",
        SOURCE + "/tools/workspace_python.py", "-m",
        "scripts.e2e.train_temporal_route_branch_geometry_pilot_v1_20261004", "--", "--mode", mode]
    prior = ROOT / "storage/experiments/plans/2026-10-04/temporal_retention_pair_pilot_v2/ROOT_TRAIN_only_authorization.json"
    assets = json.loads(prior.read_text())["asset_arguments"]
    require(set(assets) == {"parent-run", "warm-parent-weights", "warm-parent-metadata", "backbone-parent-weights",
        "backbone-parent-metadata", "recovery-train", "recovery-conversion", "left-train", "right-train"},
        "original authenticated9 asset arguments required")
    for name, path in assets.items():
        require(path.startswith(PERSONAL + "/"), "asset outside personal folder")
        argv += ["--" + name, path]
    argv += ["--parent-history-sha256", "3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3",
        "--left-conversion-sha256", "f70350f3aeab5b88069759a259bf4b3059c775429eec6fabda38a70b966f2664",
        "--right-conversion-sha256", "2e6ea6df02f55b1ebf7a9d0463214882161a2e2942e5570a7c702d6159e6b8d1",
        "--pilot-authority", WORKSPACE + "/plans/training/temporal_retention_pair_pilot_20261004_v2/ROOT_TRAIN_only_authorization.json",
        "--pilot-authority-sha256", "9a54f2beb5ac3b02c1526d13fb13e2325cb4e3c5b87751e8eb94ff9fb73d4b0c",
        "--completed-pilot-declaration", WORKSPACE + "/runs/training/temporal_retention_pair_pilot_20261004_v2/declaration.json",
        "--gradient-declaration", WORKSPACE + "/runs/diagnostics/temporal_route_joint_train_gradient_audit_20261004_v1/declaration.json",
        "--gradient-summary", WORKSPACE + "/runs/diagnostics/temporal_route_joint_train_gradient_audit_20261004_v1/summary.json",
        "--authorization", AUTHORITY, "--authorization-sha256", grant_sha, "--output", OUTPUT]
    return argv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("inspect", "prepare", "fit"), required=True)
    args = parser.parse_args()
    grant, records = frozen_inputs()
    grant_sha = digest(GRANT)
    if args.mode == "inspect":
        print(json.dumps(dict(status="LOCAL_DISPATCH_INSPECTION_NO_SSH", file_count=38,
            authorization_sha256=grant_sha, preflight_argv=command("validate-only", grant_sha),
            fit_argv=command("fit", grant_sha)), indent=2))
    elif args.mode == "prepare":
        require(not (PLAN / "ROOT_remote_stage.json").exists(), "staging already attempted; fresh revision required after failure")
        remote([PYTHON, "-B", "-c", REMOTE_STAGE], "stage",
            json.dumps(dict(workspace=WORKSPACE, records=records)).encode())
        checked = remote(command("validate-only", grant_sha), "CPU_metadata")
        data = json.loads(checked["stdout"])
        require(data["status"] == "PREPARED_BRANCH4_TRAIN_CPU_ONLY"
            and data["declaration"]["source_sha256"] == grant["contract"]["sources"]
            and len(data["declaration"]["immutable_input_sha256"]) == 35,
            "exact private36-source/35-input CPU validation required")
        write_new(PLAN / "ROOT_prepared_for_GPU0.json", dict(status="PREPARED_NOT_RUNNING",
            dispatcher_sha256=digest(Path(__file__)), authorization_sha256=grant_sha,
            CPU_receipt_sha256=digest(PLAN / "ROOT_remote_CPU_metadata.json"),
            immediate_GPU0_availability_and_existing_lease_required=True, model_promoted=False))
    else:
        checked = json.loads((PLAN / "ROOT_prepared_for_GPU0.json").read_text())
        require(checked["dispatcher_sha256"] == digest(Path(__file__)) and checked["authorization_sha256"] == grant_sha
            and checked["CPU_receipt_sha256"] == digest(PLAN / "ROOT_remote_CPU_metadata.json"),
            "actual byte-exact private CPU preflight required before GPU job")
        write_new(PLAN / "ROOT_execution_attempt.json", dict(UTC=datetime.now(timezone.utc).isoformat(),
            dispatcher_sha256=digest(Path(__file__)), authority_sha256=grant_sha,
            operator_has_serialized_local_CARLA=True, automatic_retry=False,
            physical_GPU_index=0, optimizer_updates_budget=1000))
        remote(command("fit", grant_sha), "GPU0_fixed1000_fit")
        print(json.dumps(dict(status="FIXED_TRAIN_JOB_ENDED_ROOT_FINAL_REVIEW_PENDING", model_promoted=False)))


if __name__ == "__main__":
    main()
