"""HH_261005 - Copy sealed private sources and dispatch exactly one CPU-only TRAIN admission.

No installation, GPU authorization, fitting, evaluation-role access or retry is
implemented. All grants, sources, archives and full bounded raw process output
are create-only; a failed admission stays failed and is returned to ROOT.
"""
from __future__ import annotations

import base64
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2"
OUT = PLAN / "SOURCE_transfer_and_CPU_admission"
AUTH_SHA = "4b437183cb0bd70b3589d3e03797b0a2ab8f977dd841ea6f5cef69debb9d4a29"
REVIEW_SHA = "6376d69df20a5e0dc246d5a48c63664c847889539927b5782d8ee4fca6b007fb"
CONTRACT_SHA = "cdde23e8f6265f49a4009e07be0e2e0cddc8b881ed4dca33307501e36ffd65c1"
OLD_PROVENANCE = {
    "tools/e2e/train_temporal_route_joint_three_seed_train_only_v1_20261005.py": "903ed22c1492f2924f73015627bef16687fd92f3ff6768e35aadda7cc72da93d",
    "learning/tests/test_temporal_route_joint_three_seed_train_only_v1_20261005.py": "8b50d39fba60f964934a4705262eb51c00300ba0bb915307c5793bccb161cbd6",
}


def require(value, message):
    if not value: raise RuntimeError(message)


def sha(value):
    return hashlib.sha256(value).hexdigest()


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def canonical(path):
    path = Path(path).absolute()
    require(path == path.resolve(strict=True) and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink path required")
    return path


def write_new(path, value):
    with path.open("xb") as stream: stream.write(encode(value) + b"\n")


def identity(pid):
    text = (Path("/proc") / str(pid) / "stat").read_text(); fields = text[text.rfind(")") + 2:].split()
    return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]), start_ticks=int(fields[19]))


# HH_261005 - The transported program is stdlib only; its actual child argv uses the existing private venv.
REMOTE = r'''
import base64, hashlib, io, json, os, pathlib, selectors, shutil, signal, subprocess, sys, tarfile, time
from datetime import datetime, timezone
def require(v,m):
    if not v: raise RuntimeError(m)
def sha(v): return hashlib.sha256(v).hexdigest()
def encode(v): return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def new(p,v):
    with p.open('xb') as f:f.write(encode(v)+b'\n')
def ident(pid):
    t=(pathlib.Path('/proc')/str(pid)/'stat').read_text();f=t[t.rfind(')')+2:].split()
    return dict(pid=pid,ppid=int(f[1]),pgid=int(f[2]),session=int(f[3]),start_ticks=int(f[19]))
def canonical(p):
    require(p==p.resolve(strict=True) and not any(x.is_symlink() for x in (p,*p.parents)),'canonical private input required')
    return p
package=json.loads(sys.stdin.buffer.read()); workspace=pathlib.Path('/home/pro6000/personal/hwanhong/portable_e2e')
study='temporal_route_joint_three_seed_train_only_20261005_v2'
source=workspace/'source_worktrees'/study; campaign=workspace/'runs/training'/study; plans=workspace/'plans/training'/study
created=False; created_paths=[]; child=None; child_id=None; status='FAILED_NO_RETRY'; result={}; files={}; os.umask(0o077)
def owned_stop():
    if child is None or child.poll() is not None:return
    now=ident(child.pid)
    require(child_id is not None and all(now[k]==child_id[k] for k in ('pid','ppid','pgid','session','start_ticks'))
        and now['ppid']==os.getpid() and now['pgid']==now['session']==now['pid'],'owned CPU generation required before signal')
    os.kill(child.pid,signal.SIGTERM)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        require(ident(child.pid)['start_ticks']==child_id['start_ticks'],'CPU generation changed before final owned signal')
        os.kill(child.pid,signal.SIGKILL);child.wait(timeout=5)
try:
    require(sha(base64.b64decode(package['authority']))==package['authority_sha256'],'CPU authority transfer changed')
    authority=json.loads(base64.b64decode(package['authority']))
    require(authority['authorized_modes']==['validate-only'] and authority['physical_GPU_index']==0
        and authority['schema']=='portable_e2e.temporal_route_joint_three_seed_train_only.20261005.v2.authorization'
        and sha(encode(authority['contract']))==package['contract_sha256'],'exact CPU-only grant required')
    archive=base64.b64decode(package['archive']);require(sha(archive)==package['archive_sha256'],'source archive transfer changed')
    entries=package['entries'];require(len(entries)==33 and len({r['path'] for r in entries})==33,'exact31+2 regular sources required')
    canonical(workspace)
    free=shutil.disk_usage(workspace).free
    require(free>=5*1024**3+512*1024**2,'existing5GiB floor+512MiB reserve required before private copy; no cleanup')
    result['free_before_transfer_bytes']=free
    for p in (source,campaign,plans):
        canonical(p.parent);require(not p.exists() and not p.is_symlink(),'fresh exact directory required: '+str(p))
    for p in (plans,source,campaign):
        p.mkdir();created_paths.append(str(p))
        if p==plans:created=True
    new(plans/'caller_started.json',dict(schema='portable_e2e.three_seed_CPU_admission_transfer.20261005.v1',
        UTC=datetime.now(timezone.utc).isoformat(),caller=ident(os.getpid()),CUDA_VISIBLE_DEVICES='',
        process_count=1,GPU_fit_authorized=False,automatic_retry=False,package_manifest=package['manifest']))
    with (plans/'sealed_sources.tar').open('xb') as f:f.write(archive)
    expected={r['path']:r for r in entries}
    with tarfile.open(fileobj=io.BytesIO(archive),mode='r:') as tar:
        members=tar.getmembers();require(len(members)==33 and {m.name for m in members}==set(expected),'complete exact archive members required')
        for member in members:
            require(member.isfile() and member.name in expected and not member.name.startswith('/')
                and '..' not in pathlib.PurePosixPath(member.name).parts,'only exact regular relative archive entries')
            raw=tar.extractfile(member).read();row=expected[member.name]
            require(len(raw)==member.size==row['bytes'] and sha(raw)==row['sha256'],'source member changed')
            destination=source/member.name;destination.parent.mkdir(parents=True,exist_ok=True)
            canonical(destination.parent)
            with destination.open('xb') as f:f.write(raw)
    auth_path=plans/'ROOT_CPU_TRAIN_only_authorization.json'
    with auth_path.open('xb') as f:f.write(base64.b64decode(package['authority']))
    review=base64.b64decode(package['review']);require(sha(review)==package['review_sha256'],'ROOT review transfer changed')
    with (plans/'ROOT_source_and_CPU_admission_review.json').open('xb') as f:f.write(review)
    def seals():
        for row in entries:
            p=canonical(source/row['path']);require(p.is_file() and p.stat().st_size==row['bytes'] and sha(p.read_bytes())==row['sha256'],'current source changed')
        require(sha((plans/'sealed_sources.tar').read_bytes())==package['archive_sha256']
            and sha(canonical(auth_path).read_bytes())==package['authority_sha256']
            and sha((plans/'ROOT_source_and_CPU_admission_review.json').read_bytes())==package['review_sha256'],'archive or ROOT grant changed')
    seals();new(plans/'transfer_receipt.json',dict(source_count=33,canonical_runtime_source_count=31,old_V1_provenance_count=2,
        entries=entries,archive_sha256=package['archive_sha256'],authority_sha256=package['authority_sha256'],
        ROOT_review_sha256=package['review_sha256'],current_and_archive_exact=True,no_installation=True))
    argv=[str(workspace/'venvs/py312/bin/python'),str(source/'tools/workspace_python.py'),'-m',
        'scripts.e2e.train_temporal_route_joint_three_seed_train_only_v2_20261005','--','--mode','validate-only',
        '--seed','2026100501','--output',str(campaign/'seed_2026100501'),'--authorization',str(auth_path),
        '--authorization-sha256',package['authority_sha256']]
    require(pathlib.Path(argv[0]).is_file(),'existing private venv Python missing; no install/retry')
    require(not (plans/'ROOT_GPU0_TRAIN_only_authorization.json').exists(),'GPU grant must remain absent during CPU admission')
    env=dict(os.environ,CUDA_VISIBLE_DEVICES='',PYTHONDONTWRITEBYTECODE='1')
    stdout_path=plans/'CPU_admission.stdout';stderr_path=plans/'CPU_admission.stderr'
    with stdout_path.open('xb') as out,stderr_path.open('xb') as err:
        child=subprocess.Popen(argv,cwd=str(source),env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
        for _ in range(100):
            try:child_id=ident(child.pid);break
            except FileNotFoundError:
                if child.poll() is not None:break
                time.sleep(.001)
        require(child_id is not None and child_id['ppid']==os.getpid()
            and child_id['pgid']==child_id['session']==child.pid,'actual CPU child generation/session required')
        new(plans/'CPU_child_started.json',dict(UTC=datetime.now(timezone.utc).isoformat(),caller=ident(os.getpid()),
            child=child_id,argv=argv,environment_override={'CUDA_VISIBLE_DEVICES':'','PYTHONDONTWRITEBYTECODE':'1'},
            stdout_limit_bytes=16*1024**2,stderr_limit_bytes=16*1024**2,wall_limit_seconds=600,
            GPU_fit_authorized=False,known_DEV_HOLDOUT_TEST_opened=False))
        sel=selectors.DefaultSelector();sel.register(child.stdout,selectors.EVENT_READ,out);sel.register(child.stderr,selectors.EVENT_READ,err)
        counts={out:0,err:0};begin=time.monotonic()
        while sel.get_map():
            require(time.monotonic()-begin<=600,'bounded CPU admission exceeded600s; no retry')
            for key,_ in sel.select(timeout=.1):
                raw=os.read(key.fileobj.fileno(),65536)
                if not raw:sel.unregister(key.fileobj);continue
                stream=key.data;remaining=16*1024**2-counts[stream]
                stream.write(raw[:remaining]);stream.flush();counts[stream]+=min(len(raw),remaining)
                require(len(raw)<=remaining,'bounded raw CPU output exceeded16MiB; preserve prefix/failure, no retry')
        exit_code=child.wait(timeout=5);seals()
    raw_stdout=stdout_path.read_bytes();raw_stderr=stderr_path.read_bytes()
    result=dict(exit_code=exit_code,child=child_id,argv=argv,stdout_bytes=len(raw_stdout),stderr_bytes=len(raw_stderr),
        stdout_sha256=sha(raw_stdout),stderr_sha256=sha(raw_stderr),current_archive_sources_and_grants_exact=True,
        campaign_seed_outputs_created=list(p.name for p in campaign.iterdir()),GPU_grant_exists=(plans/'ROOT_GPU0_TRAIN_only_authorization.json').exists())
    require(exit_code==0,'actual CPU admission failed; retain all raw output, no install/retry')
    admitted=json.loads(raw_stdout)
    require(admitted['status']=='PREPARED_TRAIN_ONLY_NO_CUDA_NO_OUTPUT' and not result['campaign_seed_outputs_created']
        and not result['GPU_grant_exists'],'CPU-only/no-output proof required')
    report=admitted['declaration']['all_seed_schedules'];require(set(report)=={'2026100501','2026100502','2026100503'}
        and len({r['schedule_sha256'] for r in report.values()})==3,'all3 actual distinct schedules required')
    result.update(actual_seed_schedule_sha256={k:v['schedule_sha256'] for k,v in report.items()},CPU_admission_status=admitted['status'])
    status='COMPLETE_CPU_ONLY_ADMISSION_NOT_TRAINING'
except BaseException as error:
    result['error']=repr(error)
    try:owned_stop()
    except BaseException as cleanup_error:result['owned_CPU_cleanup_error']=repr(cleanup_error)
    if child is not None:result['child_exit_code']=child.poll()
finally:
    if created:
        result.update(status=status,UTC=datetime.now(timezone.utc).isoformat(),GPU_fit_authorized=False,created_private_directories=created_paths,
            automatic_retry=False,failed_attempt_retained=status!='COMPLETE_CPU_ONLY_ADMISSION_NOT_TRAINING')
        new(plans/'CPU_admission_result.json',result)
        for p in sorted(plans.iterdir()):
            if p.is_file():files[p.name]=dict(bytes=p.stat().st_size,sha256=sha(p.read_bytes()),base64=base64.b64encode(p.read_bytes()).decode())
    print(json.dumps(dict(status=status,result=result,created_private_directories=created_paths,remote_plans_root=str(plans),files=files),allow_nan=False),flush=True)
'''


def main():
    canonical(PLAN); require(not OUT.exists() and not OUT.is_symlink(), "fresh local transfer output required; no retry")
    authority = canonical(PLAN / "ROOT_CPU_TRAIN_only_authorization.json").read_bytes()
    review = canonical(PLAN / "ROOT_source_and_CPU_admission_review.json").read_bytes()
    require(sha(authority) == AUTH_SHA and sha(review) == REVIEW_SHA, "ROOT grant/review changed")
    approved = json.loads(authority); require(sha(encode(approved["contract"])) == CONTRACT_SHA
        and approved["authorized_modes"] == ["validate-only"] and len(approved["contract"]["sources"]) == 31,
        "exact31-source CPU-only approval required")
    sources = dict(approved["contract"]["sources"], **OLD_PROVENANCE); entries = []; archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w:", format=tarfile.USTAR_FORMAT) as tar:
        for name, expected in sorted(sources.items()):
            require(not name.startswith("/") and ".." not in Path(name).parts, "relative exact source required")
            path = canonical(ROOT / name); require(path.is_file(), "regular source required")
            raw = path.read_bytes(); require(sha(raw) == expected, "source changed before sealed transfer: " + name)
            info = tarfile.TarInfo(name); info.size = len(raw); info.mode = 0o600; info.mtime = 0
            tar.addfile(info, io.BytesIO(raw)); entries.append(dict(path=name, bytes=len(raw), sha256=expected))
    payload = archive.getvalue(); OUT.mkdir()
    package = dict(archive=base64.b64encode(payload).decode(), archive_sha256=sha(payload), entries=entries,
        authority=base64.b64encode(authority).decode(), authority_sha256=AUTH_SHA,
        review=base64.b64encode(review).decode(), review_sha256=REVIEW_SHA, contract_sha256=CONTRACT_SHA)
    package["manifest"] = dict(schema="portable_e2e.three_seed_CPU_admission_transfer.20261005.v1",
        provenance="HH_261005 - Transfer exact private source bytes; one CPU-only admission, no fit or install.",
        source_count=33, canonical_runtime_source_count=31, old_V1_provenance_count=2, entries=entries,
        archive_sha256=sha(payload), archive_bytes=len(payload), ROOT_CPU_authority_sha256=AUTH_SHA,
        ROOT_review_sha256=REVIEW_SHA, contract_sha256=CONTRACT_SHA, helper_sha256=sha(Path(__file__).read_bytes()),
        transported_stdlib_program_sha256=sha(REMOTE.encode()), maximum_raw_output_bytes_per_stream=16*1024**2,
        CPU_wall_limit_seconds=600, free_floor_bytes=5*1024**3, write_reserve_bytes=512*1024**2,
        GPU_fit_authorized=False, automatic_retry=False)
    write_new(OUT / "transfer_manifest.json", package["manifest"])
    with (OUT / "sealed_sources.tar").open("xb") as stream: stream.write(payload)
    import shlex
    argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "pro6000-training", "python3 -c " + shlex.quote(REMOTE)]
    write_new(OUT / "caller_started.json", dict(UTC=datetime.now(timezone.utc).isoformat(), caller=identity(os.getpid()),
        SSH_alias="pro6000-training", SSH_BatchMode=True, helper_sha256=sha(Path(__file__).read_bytes()),
        archive_sha256=sha(payload), GPU_fit_authorized=False, automatic_retry=False))
    child = None; ssh_identity = None
    try:
        child = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        ssh_identity = identity(child.pid)
        require(ssh_identity["ppid"] == os.getpid() and ssh_identity["pgid"] == ssh_identity["session"] == child.pid,
            "exact newly owned SSH child session required")
        write_new(OUT / "SSH_child_started.json", dict(child=ssh_identity, caller=identity(os.getpid()),
            argv_prefix=argv[:7], remote_program_sha256=sha(REMOTE.encode())))
        stdout, stderr = child.communicate(encode(package), timeout=660)
        with (OUT / "SSH.stdout").open("xb") as stream: stream.write(stdout)
        with (OUT / "SSH.stderr").open("xb") as stream: stream.write(stderr)
        require(len(stdout) <= 64 * 1024**2 and len(stderr) <= 16 * 1024**2, "bounded transported raw output required")
        write_new(OUT / "SSH_exited.json", dict(exit_code=child.returncode, stdout_bytes=len(stdout), stderr_bytes=len(stderr),
            stdout_sha256=sha(stdout), stderr_sha256=sha(stderr)))
        require(child.returncode == 0, "sealed transfer/CPU caller failed; no retry")
        response = json.loads(stdout)
        require(response.get("remote_plans_root") == "/home/pro6000/personal/hwanhong/portable_e2e/plans/training/temporal_route_joint_three_seed_train_only_20261005_v2",
            "unexpected private result boundary")
        for name, row in response["files"].items():
            require(name == Path(name).name and name not in (".", ".."), "one exact remote result basename required")
            raw = base64.b64decode(row["base64"]); require(len(raw) == row["bytes"] and sha(raw) == row["sha256"], "remote result transport changed")
            with (OUT / ("remote_" + name)).open("xb") as stream: stream.write(raw)
        for row in entries:
            require(sha(canonical(ROOT / row["path"]).read_bytes()) == row["sha256"], "local source changed during transfer")
        require(sha(canonical(PLAN / "ROOT_CPU_TRAIN_only_authorization.json").read_bytes()) == AUTH_SHA
            and sha(canonical(PLAN / "ROOT_source_and_CPU_admission_review.json").read_bytes()) == REVIEW_SHA,
            "local ROOT grants changed during actual CPU admission")
        write_new(OUT / "result.json", dict(status=response["status"], result=response["result"],
            all33_local_sources_unchanged=True,
            remote_current_archive_source_postcheck_exact=response["result"].get("current_archive_sources_and_grants_exact"),
            created_private_directories=response.get("created_private_directories"), local_ROOT_grants_unchanged=True,
            remote_result_file_sha256={name: row["sha256"] for name, row in response["files"].items()},
            GPU_fit_authorized=False, automatic_retry=False))
        print(json.dumps(dict(status=response["status"], result=response["result"], local_output=str(OUT)), allow_nan=False))
        return 0 if response["status"] == "COMPLETE_CPU_ONLY_ADMISSION_NOT_TRAINING" else 2
    except BaseException as error:
        cleanup = dict(SSH_child_started=child is not None, SSH_child_exit_code=None, owned_generation_signalled=False)
        if isinstance(error, subprocess.TimeoutExpired):
            for name, raw in (("SSH_timeout.stdout", error.stdout), ("SSH_timeout.stderr", error.stderr)):
                if raw is not None:
                    with (OUT / name).open("xb") as stream: stream.write(raw)
        if child is not None:
            try:
                if child.poll() is None:
                    current = identity(child.pid)
                    require(ssh_identity is not None and current == ssh_identity and current["ppid"] == os.getpid()
                        and current["pgid"] == current["session"] == child.pid,
                        "known SSH child generation required before cleanup")
                    child.terminate(); cleanup["owned_generation_signalled"] = True
                    try: child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        require(identity(child.pid) == ssh_identity, "SSH generation changed before final owned signal")
                        child.kill(); child.wait(timeout=5)
                cleanup["SSH_child_exit_code"] = child.poll()
            except BaseException as cleanup_error: cleanup["error"] = repr(cleanup_error)
        write_new(OUT / "failure.json", dict(error=repr(error), failed_attempt_retained=True, automatic_retry=False,
            GPU_fit_authorized=False, local_output=str(OUT), SSH_cleanup=cleanup,
            remote_completion_if_transport_failed="UNKNOWN; remote bounded CPU caller retains its own result"))
        raise


if __name__ == "__main__": raise SystemExit(main())
