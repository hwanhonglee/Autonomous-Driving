"""HH_261002 - Read-only completed-pair mirror evidence; root must separately seal any research authority."""
from __future__ import annotations
import argparse
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import stat
import subprocess

ROOT = Path(__file__).resolve().parents[2]
REMOTE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
RUN = REMOTE/"runs/training/temporal_intrinsic_curve_pair_20261002_v2"
SOURCE = REMOTE/"source_worktrees/temporal_intrinsic_curve_pair_20261002_v2"
PYTHON = REMOTE/"venvs/py312/bin/python"
LOCAL = ROOT/"storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_v2"
RECEIPT = ROOT/"storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_mirror_v2"
SCHEMA = "portable_e2e.temporal_intrinsic_pair_completed_mirror_evidence.20261002.v1"
TRAINING = "portable_e2e.temporal_intrinsic_curve_paired_continuation.20261002.v1"
SSH = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15", "pro6000-training"]


def require(value, message):
    if not value: raise ValueError(message)


def canonical(path):
    path = Path(path).absolute()
    require(".." not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink path required")
    return path


def digest(path):
    path = canonical(path); require(stat.S_ISREG(path.stat().st_mode), "regular artifact required")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024**2), b""): value.update(chunk)
    return value.hexdigest()


def read(path, expected=None):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key"); result[key] = value
        return result
    require(canonical(path).stat().st_size <= 16*1024**2, "bounded JSON artifact required")
    payload = Path(path).read_bytes()
    require(expected is None or hashlib.sha256(payload).hexdigest()==expected, "JSON changed between inventory and parsing")
    return json.loads(payload, object_pairs_hook=unique,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON: "+value)))


def inventory(root):
    root = canonical(root); require(root.is_dir(), "existing regular run directory required")
    result = {}
    for folder, dirs, files in os.walk(root, followlinks=False):
        require(all(not (Path(folder)/name).is_symlink() for name in dirs), "directory symlink rejected")
        for name in sorted(files):
            path = canonical(Path(folder)/name); relative = str(path.relative_to(root))
            require(not any(ord(c)<32 for c in relative), "unsafe artifact name")
            result[relative] = dict(size=path.stat().st_size, sha256=digest(path))
    return dict(sorted(result.items()))


def inspect_pair(run, source, summary_sha, declaration_sha, authorization_sha):
    # HH_261002 - Identical stdlib function runs remotely and in synthetic tests; never deserialize PT/NPZ or touch CUDA.
    run, source = canonical(run), canonical(source)
    files, source_files = inventory(run), inventory(source)
    def document(path):
        folder, census = (run, files) if path.is_relative_to(run) else (source, source_files)
        return read(path, census[str(path.relative_to(folder))]["sha256"])
    require(digest(run/"summary.json")==summary_sha and digest(run/"declaration.json")==declaration_sha, "externally pinned completion differs")
    declaration, summary = document(run/"declaration.json"), document(run/"summary.json")
    require(declaration["schema"]==summary["schema"]==TRAINING and summary["status"]=="BOTH_FIXED6000_COMPLETE_NOT_PROMOTED"
        and summary["actual_batch_bytes_equal"] is True and summary["all71_retained_per_arm"] is True
        and summary["model_promoted"] is False and summary["vehicle_control_approved"] is False, "unfinished/denied pair cannot be mirrored for authority review")
    sources = declaration["source_sha256"]
    require(len(sources)==29 and declaration["protocol"]["sources"]==sources
        and declaration["authorization_sha256"]==authorization_sha and digest(source/"authorization.json")==authorization_sha, "exact29 source/root authorization required")
    authorization, transfer = document(source/"authorization.json"), document(source/"transfer_receipt.json")
    require(authorization["contract"]==declaration["protocol"] and transfer["file_sha256"]==dict(sources, **{"authorization.json":authorization_sha})
        and transfer["source_file_count"]==29 and transfer["authorization_file_count"]==1, "prepared source/authorization provenance differs")
    for name, pin in sources.items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts and str(relative)==name, "canonical source key required")
        require(digest(source/relative)==pin, "live private source changed")
    endpoint = {}; journals = []
    for arm in ("control", "intrinsic_curve"):
        folder = run/arm; result = document(folder/"fit_result.json"); child = document(folder/"declaration.json")
        require(child==dict(declaration, arm=arm) and result==summary["arms"][arm] and result["optimizer_updates"]==6000
            and result["examples_seen"]==24000 and result["completed_fixed_endpoint"] is True
            and [r["optimizer_updates"] for r in result["checkpoints"]]==list(range(500,6001,500)), "all12 fixed endpoints/update counts required")
        history = folder/"history.jsonl"; require(history.stat().st_size<=64*1024**2, "bounded full history required")
        payload = history.read_bytes(); require(hashlib.sha256(payload).hexdigest()==files[str(history.relative_to(run))]["sha256"], "history changed before parsing")
        rows = [json.loads(line) for line in payload.splitlines()]
        require(len(rows)==6000, "all6000 actual history rows required")
        for i, row in enumerate(rows, 1):
            require(type(row["optimizer_updates"]) is int and row["optimizer_updates"]==i and row["examples_seen"]==4*i and row["sample_ids"]==declaration["fixed_sample_ids"][i-1]
                and row["legacy_curved"] is (i%2==1) and row["intrinsic_coefficient"]==(.05 if arm=="intrinsic_curve" and i%2 else 0.)
                and isinstance(row["indices"],list) and len(row["indices"])==4 and all(type(v) is int for v in row["indices"])
                and isinstance(row["batch_sha256"],str) and len(row["batch_sha256"])==64 and math.isfinite(row["loss"]), "actual fixed schedule/objective history differs")
        schedule = json.dumps([r["indices"] for r in rows],sort_keys=True,separators=(",",":"),allow_nan=False).encode()
        require(hashlib.sha256(schedule).hexdigest()==declaration["fixed_schedule_sha256"], "actual full6000 sampler indices differ from declared fixed schedule")
        journals.append([(r["indices"], r["sample_ids"], r["batch_sha256"]) for r in rows])
        for receipt in result["checkpoints"]:
            step = receipt["optimizer_updates"]; export = folder/("exports/step_%06d"%step); d = document(export/"metadata.json")
            require(digest(folder/("checkpoints/step_%06d.pt"%step))==receipt["checkpoint_sha256"]
                and digest(export/"model_state.npz")==receipt["npz_sha256"] and digest(export/"metadata.json")==receipt["metadata_sha256"]
                and d["optimizer_updates"]==step and d["arm"]==arm and d["source_sha256"]==sources
                and d["declaration_sha256"]==digest(folder/"declaration.json"), "checkpoint/export/metadata identities differ")
        for name, pin in sources.items(): require(digest(folder/"source"/name)==pin, "archived source differs")
        screens = summary["endpoint_diagnostic_screens"][arm]
        require(screens["diagnostic_screen_count"]==len(screens["checks"])==len({r["name"] for r in screens["checks"]})==71
            and all(type(r["passed"]) is bool for r in screens["checks"]), "all71 outcomes including failures required")
        last = result["checkpoints"][-1]
        endpoint[arm] = dict(weights_sha256=last["npz_sha256"], metadata_sha256=last["metadata_sha256"],
            checkpoint_sha256=last["checkpoint_sha256"], declaration_sha256=digest(folder/"declaration.json"))
    require(journals[0]==journals[1], "paired actual indices/tokens/batch bytes differ")
    opened = document(run/"evaluation_opened_after_both_final.json")
    require(opened["arms"]==summary["arms"] and opened["checkpoint_selection"] is False, "evaluation must open after BOTH fixed endpoints")
    for arm in ("parent", "control", "intrinsic_curve"):
        for cohort in ("recovery_train", "recovery_holdout", "v4", "legacy", "known599", "intrinsic_legacy_TRAIN_only"):
            require(isinstance(document(run/(arm+"_"+cohort+".json")), dict), "complete raw evaluation metrics required")
    require(inventory(run)==files and inventory(source)==source_files, "source/run changed during read-only inspection")
    return dict(run=str(run), source=str(source), files=files, source_files=source_files, sources=sources,
        endpoints=endpoint, protocol=declaration["protocol"], authorization_sha256=authorization_sha,
        summary_sha256=summary_sha, declaration_sha256=declaration_sha)


def remote_command(args):
    names = (require, canonical, digest, read, inventory, inspect_pair)
    program = "import hashlib,json,math,os,stat\nfrom pathlib import Path\nTRAINING="+repr(TRAINING)+"\n"
    program += "\n".join(inspect.getsource(fn) for fn in names)
    program += "\nprint(json.dumps(inspect_pair(*json.loads("+repr(json.dumps([str(RUN),str(SOURCE),args.expected_summary_sha256,
        args.expected_declaration_sha256,args.authorization_sha256]))+")),sort_keys=True,allow_nan=False))"
    return SSH+[shlex.join(["env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP",
        "PYTHONNOUSERSITE=1","CUDA_VISIBLE_DEVICES=",str(PYTHON),"-I","-B","-c",program])]


def write_new(path, value):
    with Path(path).open("x") as stream: json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("expected-summary-sha256","expected-declaration-sha256","authorization-sha256","expected-tool-sha256"):
        parser.add_argument("--"+name,required=True)
    args = parser.parse_args(argv)
    require(all(isinstance(v,str) and len(v)==64 and set(v)<=set("0123456789abcdef") for v in vars(args).values()), "external SHA256 pins required")
    require(digest(__file__)==args.expected_tool_sha256 and all(not canonical(p).exists() for p in (LOCAL,RECEIPT)), "reviewed source/fresh destinations required")
    RECEIPT.mkdir(parents=True); write_new(RECEIPT/"started.json",dict(schema=SCHEMA,remote_run=str(RUN),remote_mutated=False,optimizer_started=False))
    with (RECEIPT/Path(__file__).name).open("xb") as stream: stream.write(Path(__file__).read_bytes())
    def invoke(command, label, timeout):
        result = subprocess.run(command,capture_output=True,timeout=timeout)
        for suffix,payload in (("stdout",result.stdout),("stderr",result.stderr)):
            with (RECEIPT/(label+"_"+suffix+".txt")).open("xb") as stream: stream.write(payload)
        require(result.returncode==0, "read-only mirror stage failed: "+label)
        return result.stdout
    try:
        before = json.loads(invoke(remote_command(args),"remote_before",1800))
        required = 5*1024**3+128*1024**2+sum(v["size"] for v in before["files"].values())
        require(shutil.disk_usage(LOCAL.parent).free>=required, "5GiB floor/full mirror/write reserve required; no deletion")
        for name,pin in before["sources"].items(): require(digest(ROOT/name)==pin, "local frozen29 source differs")
        LOCAL.mkdir()
        invoke(["rsync","--recursive","--perms","--protect-args","--no-links","--ignore-existing","-e","ssh -o BatchMode=yes -o ConnectTimeout=15",
            "--","pro6000-training:"+str(RUN)+"/",str(LOCAL)+"/"],"rsync",1800)
        after = json.loads(invoke(remote_command(args),"remote_after",1800))
        require(before==after and inventory(LOCAL)==before["files"], "remote/local full artifact census changed during mirror")
        require(shutil.disk_usage(LOCAL.parent).free>=5*1024**3+128*1024**2, "free floor/write reserve changed; keep partial evidence")
        for name,pin in before["sources"].items(): require(digest(ROOT/name)==pin, "local frozen29 source changed")
        require(digest(__file__)==args.expected_tool_sha256, "mirror source changed")
        require(digest(RECEIPT/Path(__file__).name)==args.expected_tool_sha256, "archived mirror source differs")
        write_new(RECEIPT/"completed_pair_for_root_review.json",dict(before,schema=SCHEMA,pair_run=str(LOCAL),
            status="MIRROR_COMPLETE_ROOT_SEAL_REQUIRED",numeric_state_validated=False,simulator_control_authorized=False,
            model_promoted=False,vehicle_control_approved=False,source_tool_sha256=args.expected_tool_sha256))
    except Exception as error:
        write_new(RECEIPT/"failure_receipt.json",dict(schema=SCHEMA,error=type(error).__name__+": "+str(error),
            partial_mirror_retained=True,remote_mutated=False,model_promoted=False)); raise
    print(json.dumps(dict(status="MIRROR_COMPLETE_ROOT_SEAL_REQUIRED",pair_run=str(LOCAL),receipt=str(RECEIPT))))
    return 0


if __name__=="__main__": raise SystemExit(main())
