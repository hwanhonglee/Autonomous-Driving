"""HH_261004 - Stage and dispatch one private TRAIN-only gradient diagnostic without installations."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
PERSONAL = "/home/pro6000/personal/hwanhong"
W = PERSONAL + "/portable_e2e"
STUDY = "temporal_route_joint_train_gradient_audit_20261004_v1"
S = W + "/source_worktrees/" + STUDY
A = W + "/plans/diagnostics/" + STUDY + "/ROOT_TRAIN_gradient_authorization.json"
OUT = W + "/runs/diagnostics/" + STUDY
PLAN = ROOT / "storage/experiments/plans/2026-10-04/temporal_route_joint_train_gradient_audit_v1"
GRANT = PLAN / "ROOT_TRAIN_gradient_authorization.json"
TEST = "learning/tests/test_temporal_route_joint_train_gradients_v1_20261004.py"
PYTHON = W + "/venvs/py312/bin/python"
UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "pro6000-training"]


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path, value):
    # HH_261004 - Normal diagnostic outputs are exclusive; failed attempts cannot be silently replaced.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def frozen_local_inputs():
    require(GRANT.is_file() and not GRANT.is_symlink(), "fresh prospective authorization required")
    grant = json.loads(GRANT.read_text())
    require(grant["authorized"] is True and grant["physical_GPU_index"] == 0
            and grant["model_promoted"] is False and grant["vehicle_control_approved"] is False,
            "exact TRAIN-only authority required")
    sources = grant["contract"]["sources"]
    require(len(sources) == 35 and grant["contract"]["operation_budget"]["optimizer_updates"] == 0,
            "exact35-source nonmutating diagnostic required")
    records = []
    for relative, expected in dict(sources, **{TEST: grant["audit_test_sha256"]}).items():
        path = ROOT / relative
        require(path.resolve() == path and path.is_file() and sha(path) == expected,
                "frozen local source changed: " + relative)
        records.append(dict(relative=relative, sha256=expected,
                            data=base64.b64encode(path.read_bytes()).decode("ascii")))
    records.append(dict(relative=None, sha256=sha(GRANT),
                        data=base64.b64encode(GRANT.read_bytes()).decode("ascii")))
    return grant, records


REMOTE_STAGE = r'''
# HH_261004 - Receive only authenticated bytes into a brand-new personal source tree and grant.
import base64, hashlib, json, os
from pathlib import Path
data = json.load(__import__('sys').stdin)
w = Path('/home/pro6000/personal/hwanhong/portable_e2e')
s = w / 'source_worktrees/temporal_route_joint_train_gradient_audit_20261004_v1'
a = w / 'plans/diagnostics/temporal_route_joint_train_gradient_audit_20261004_v1/ROOT_TRAIN_gradient_authorization.json'
def safe(path):
    assert path.is_relative_to(w) and not any(p.is_symlink() for p in (path, *path.parents))
    assert path.resolve() == path
safe(w); assert w.is_dir() and not s.exists() and not a.parent.exists()
assert len(data['records']) == 37 and data['workspace'] == str(w)
validated = []
for row in data['records']:
    target = a if row['relative'] is None else s / row['relative']
    safe(target)
    raw = base64.b64decode(row['data'], validate=True)
    assert len(raw) <= 2097152 and hashlib.sha256(raw).hexdigest() == row['sha256']
    assert target == a or target.is_relative_to(s)
    validated.append((target, raw, row['sha256']))
assert len(set(p for p, _, _ in validated)) == 37
s.mkdir(parents=True, exist_ok=False); a.parent.mkdir(parents=True, exist_ok=False)
for target, raw, expected in validated:
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('xb') as f: f.write(raw); f.flush(); os.fsync(f.fileno())
    assert hashlib.sha256(target.read_bytes()).hexdigest() == expected
print(json.dumps({'status':'STAGED37_CREATE_ONLY_PRIVATE_FILES','source_root':str(s),
                  'authorization':str(a),'sha256':{str(p):h for p,_,h in validated}}))
'''


def remote(args, phase, input_bytes=None):
    command = SSH + [shlex.join(args)]
    print(json.dumps(dict(status="RUNNING_REMOTE_" + phase, environment="PRO6000 private venv",
                          physical_GPU_index=0, optimizer_updates=0)), flush=True)
    result = subprocess.run(command, input=input_bytes, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    require(len(result.stdout) <= 32 * 1024**2 and len(result.stderr) <= 8 * 1024**2,
            "bounded actual remote diagnostic output required")
    receipt = dict(phase=phase, UTC=datetime.now(timezone.utc).isoformat(), argv=command,
                   returncode=result.returncode, stdout=result.stdout.decode(), stderr=result.stderr.decode(),
                   dispatcher_sha256=sha(Path(__file__)), authority_sha256=sha(GRANT))
    write_new(PLAN / ("ROOT_remote_" + phase + ".json"), receipt)
    require(result.returncode == 0, "remote " + phase + " failed; complete receipt retained")
    return receipt


def command(mode, grant_sha):
    # HH_261004 - These arguments are reconstructed from the sealed completed pilot, not a claimed historical argv.
    args = ["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", "-u", "PYTHONUSERBASE", "-u", "PYTHONSTARTUP",
            "PYTHONNOUSERSITE=1", "OMP_NUM_THREADS=4", "CUBLAS_WORKSPACE_CONFIG=:4096:8",
            "CUDA_VISIBLE_DEVICES=" + (UUID if mode == "run" else ""), PYTHON, "-B",
            S + "/tools/workspace_python.py", "-m",
            "scripts.e2e.audit_temporal_route_joint_train_gradients_v1_20261004", "--", "--mode", mode]
    inputs = json.loads((ROOT / "storage/experiments/plans/2026-10-04/temporal_retention_pair_pilot_v2/ROOT_TRAIN_only_authorization.json").read_text())["asset_arguments"]
    require(set(inputs) == {"parent-run", "warm-parent-weights", "warm-parent-metadata", "backbone-parent-weights",
                           "backbone-parent-metadata", "recovery-train", "recovery-conversion", "left-train", "right-train"},
            "original authenticated9 asset arguments required")
    for key, value in inputs.items():
        require(value.startswith(PERSONAL + "/"), "input outside private personal root")
        args += ["--" + key, value]
    args += ["--parent-history-sha256", "3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3",
             "--left-conversion-sha256", "f70350f3aeab5b88069759a259bf4b3059c775429eec6fabda38a70b966f2664",
             "--right-conversion-sha256", "2e6ea6df02f55b1ebf7a9d0463214882161a2e2942e5570a7c702d6159e6b8d1",
             "--pilot-authority", W + "/plans/training/temporal_retention_pair_pilot_20261004_v2/ROOT_TRAIN_only_authorization.json",
             "--pilot-authority-sha256", "9a54f2beb5ac3b02c1526d13fb13e2325cb4e3c5b87751e8eb94ff9fb73d4b0c",
             "--completed-pilot-declaration", W + "/runs/training/temporal_retention_pair_pilot_20261004_v2/declaration.json",
             "--authorization", A, "--authorization-sha256", grant_sha, "--output", OUT]
    return args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("inspect", "prepare", "run"), required=True)
    args = parser.parse_args()
    grant, records = frozen_local_inputs()
    grant_sha = sha(GRANT)
    if args.mode == "inspect":
        print(json.dumps(dict(status="LOCAL_NO_SSH_DISPATCH_INSPECTION", file_count=len(records),
                              authorization_sha256=grant_sha, preflight_argv=command("validate-only", grant_sha),
                              run_argv=command("run", grant_sha)), indent=2)); return
    if args.mode == "prepare":
        require(not (PLAN / "ROOT_remote_stage.json").exists(), "stage already attempted; no overwrite/retry")
        remote([PYTHON, "-B", "-c", REMOTE_STAGE], "stage",
               json.dumps(dict(workspace=W, records=records)).encode())
        backend = ("import runpy; w=runpy.run_path(" + repr(S + "/tools/workspace_python.py") + ");w['configure']();"
                   "from scripts.e2e import audit_temporal_route_joint_train_gradients_v1_20261004 as a;"
                   "b=a.attention_backend();v=b.get_fastpath_enabled();"
                   "scope=a.matched_attention_math();p=scope.__enter__();"
                   "assert p==v and b.get_fastpath_enabled() is False;scope.__exit__(None,None,None);"
                   "assert b.get_fastpath_enabled()==v and not a.torch.cuda.is_initialized();"
                   "print('ACTUAL_PRIVATE_MHA_DISABLE_AND_RESTORE_PASS_NO_CUDA')")
        remote(["env", "-u", "PYTHONPATH", "-u", "PYTHONHOME", "-u", "PYTHONUSERBASE", "-u", "PYTHONSTARTUP",
                "PYTHONNOUSERSITE=1", "CUDA_VISIBLE_DEVICES=", PYTHON, "-B", "-c", backend], "actual_MHA_CPU")
        result = remote(command("validate-only", grant_sha), "CPU_metadata")
        data = json.loads(result["stdout"])
        require(data["status"] == "PREPARED_TRAIN_GRADIENT_AUDIT_CPU_METADATA_ONLY"
                and data["declaration"]["sources"] == grant["contract"]["sources"]
                and len(data["declaration"]["immutable_input_sha256"]) == 32,
                "actual private full32-pin preflight differs")
        write_new(PLAN / "ROOT_prepared_for_GPU0.json", dict(status="PREPARED_NOT_RUNNING_NOT_TRAINING",
                  dispatcher_sha256=sha(Path(__file__)), authorization_sha256=grant_sha,
                  CPU_receipt_sha256=sha(PLAN / "ROOT_remote_CPU_metadata.json"), physical_GPU_index=0,
                  optimizer_updates=0, no_simulator_job_running_assertion_required_from_operator=True))
    else:
        receipt = json.loads((PLAN / "ROOT_prepared_for_GPU0.json").read_text())
        require(receipt["dispatcher_sha256"] == sha(Path(__file__)) and receipt["authorization_sha256"] == grant_sha
                and receipt["CPU_receipt_sha256"] == sha(PLAN / "ROOT_remote_CPU_metadata.json"),
                "byte-exact actual private preflight required")
        write_new(PLAN / "ROOT_execution_attempt.json", dict(UTC=datetime.now(timezone.utc).isoformat(),
                  dispatcher_sha256=sha(Path(__file__)), authority_sha256=grant_sha,
                  operator_has_serialized_local_CARLA=True, no_automatic_retry=True))
        remote(command("run", grant_sha), "GPU0_gradient_measurement")
        print(json.dumps(dict(status="REMOTE_GRADIENT_JOB_ENDED_ROOT_POSTCHECK_PENDING", optimizer_updates=0)), flush=True)


if __name__ == "__main__":
    main()
