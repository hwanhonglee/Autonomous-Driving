"""HH_261005 - Prepare one sealed GPU0 seed3 process; never dispatch another seed or retry.

Full training/log/checkpoint evidence stays in the remote private workspace.
The local 64MiB floor plus1MiB reserve applies only to <=128KiB remote metadata
receipts and does not replace any CARLA, runtime, training or evaluation guard.
Execution requires the immutable GPU grant, an independent whole-seed2 postcheck,
and ROOT's exact new source/test-bound seed3 review plus its caller-supplied SHA.
"""
from __future__ import annotations

import base64
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2"
CPU = PLAN / "SOURCE_transfer_and_CPU_admission"
OUT = PLAN / "seed_2026100503_fit_supervisor_v1"
GPU_AUTH_SHA = "a2f25569e74ed99845757a4954846a9cab0ea500cde7adc3736b89f71c779ee0"
CPU_AUTH_SHA = "4b437183cb0bd70b3589d3e03797b0a2ab8f977dd841ea6f5cef69debb9d4a29"
CONTRACT_SHA = "cdde23e8f6265f49a4009e07be0e2e0cddc8b881ed4dca33307501e36ffd65c1"
ADMISSION_SHA = "3f5ae127e3e9b734586e30d34d5c5cb6eb63b00fad8a0ab10c409eef11303480"
TRANSFER_MANIFEST_SHA = "bf96e1eb6d03c2a74bc876b6319cbf0bf4eaa2fef544b4cfbd29bee30ad19b79"
SOURCE_ARCHIVE_SHA = "a16591999e09dad04d3aa367e15ba841ebb3c9d2bad83a9f982515a40a5996ec"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
LOCAL_LIMIT = 128 * 1024
TEST_SOURCE = ROOT / "learning/tests/test_supervise_three_seed_joint6000_seed3_v1_20261005.py"
SEED2_POST_SOURCE = ROOT / "tools/maintenance/postcheck_three_seed_joint6000_seed2_v1_20261005.py"
SEED2_POST = PLAN / "seed_2026100502_fit_supervisor_v1/ROOT_whole_seed2_training_postcheck.json"
EXECUTION_REVIEW = PLAN / "ROOT_seed3_GPU0_execution_review.json"
SEED3_SCHEDULE_SHA = "6b43bd0be8665824a69ac0c273da2478cec1f2b61d149487bf306131d3d5d81c"


def require(value, message):
    if not value: raise RuntimeError(message)


def sha(value): return hashlib.sha256(value).hexdigest()


def encode(value): return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def canonical(path):
    path = Path(path).absolute()
    require(path == path.resolve(strict=True) and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink input required")
    return path


def identity(pid):
    text = (Path("/proc") / str(pid) / "stat").read_text(); fields = text[text.rfind(")") + 2:].split()
    return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]), start_ticks=int(fields[19]))


def same_generation(a, b):
    return all(a.get(k) == b.get(k) for k in ("pid", "pgid", "session", "start_ticks"))


def admit_family(current, known, leader):
    # HH_261005 - Only observed ancestry admits a new child; orphan membership alone is not ownership.
    known = dict(known); changed = True
    while changed:
        changed = False
        for pid, row in current.items():
            if pid in known:
                require(same_generation(row, known[pid]), "owned PID generation changed")
                continue
            parent = current.get(row["ppid"])
            if parent is not None and parent["pid"] in known and same_generation(parent, known[parent["pid"]]):
                require(row["start_ticks"] >= leader["start_ticks"], "child predates this session")
                known[pid] = dict(row); changed = True
    return known, [row for pid, row in current.items() if pid not in known]


REMOTE_PREFIX = r'''
import base64, ctypes, hashlib, json, os, pathlib, selectors, shutil, signal, subprocess, sys, time
from datetime import datetime, timezone
def require(v,m):
    if not v:raise RuntimeError(m)
def sha(v):return hashlib.sha256(v).hexdigest()
def encode(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def new(p,v):
    with p.open('xb') as f:f.write(encode(v)+b'\n')
def canonical(p):
    require(p==p.resolve(strict=True) and not any(x.is_symlink() for x in (p,*p.parents)),'canonical private path required')
    return p
def identity(pid):
    t=(pathlib.Path('/proc')/str(pid)/'stat').read_text();f=t[t.rfind(')')+2:].split()
    return dict(pid=pid,ppid=int(f[1]),pgid=int(f[2]),session=int(f[3]),start_ticks=int(f[19]))
'''

# HH_261005 - The exact local pure-tested generation/ancestry functions also execute remotely.
REMOTE_BODY = r'''
package=json.loads(sys.stdin.buffer.read());workspace=pathlib.Path('/home/pro6000/personal/hwanhong/portable_e2e')
study='temporal_route_joint_three_seed_train_only_20261005_v2';plans=workspace/'plans/training'/study
source=workspace/'source_worktrees'/study;campaign=workspace/'runs/training'/study
output=campaign/'seed_2026100503';receipt=plans/'seed_2026100503_fit_supervisor_v1'
gpu_path=plans/'ROOT_GPU0_TRAIN_only_authorization.json';cpu_path=plans/'ROOT_CPU_TRAIN_only_authorization.json'
created=False;child=None;leader=None;known={};unknown=[];result={};status='FAILED_NO_RETRY';os.umask(0o077)
seed2_receipt=plans/'seed_2026100502_fit_supervisor_v1'
seed2_fit=campaign/'seed_2026100502/control/fit_result.json'
def supervisor_signal(signum,frame):
    result['supervisor_received_signal']=signum
    for value in (signal.SIGHUP,signal.SIGTERM,signal.SIGINT):signal.signal(value,signal.SIG_IGN)
    raise RuntimeError('owned supervisor interrupted by signal '+str(signum))
for value in (signal.SIGHUP,signal.SIGTERM,signal.SIGINT):signal.signal(value,supervisor_signal)
def parent_death_guard(expected_parent):
    require(ctypes.CDLL(None,use_errno=True).prctl(1,int(signal.SIGTERM),0,0,0)==0,'Linux owned-leader parent-death guard required')
    if os.getppid()!=expected_parent:os.kill(os.getpid(),signal.SIGTERM)
def group():
    rows={}
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdecimal():continue
        try:r=identity(int(p.name))
        except (FileNotFoundError,ProcessLookupError):continue
        if leader and r['pgid']==leader['pgid'] and r['session']==leader['session']:rows[r['pid']]=r
    return rows
def observe():
    global known,unknown
    current=group();known,unknown=admit_family(current,known,leader)
    return current
def cleanup():
    if leader is None:return dict(leader_started=False)
    current=observe();actions=[]
    for sig in (signal.SIGTERM,signal.SIGKILL):
        current=observe()
        for pid,row in current.items():
            if pid not in known:continue
            now=identity(pid);require(same_generation(now,known[pid]),'owned generation required before individual signal')
            try:os.kill(pid,sig);actions.append(dict(pid=pid,start_ticks=now['start_ticks'],signal=int(sig)))
            except ProcessLookupError:pass
        if child is not None and child.poll() is None:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:pass
        until=time.monotonic()+2
        while time.monotonic()<until and group():time.sleep(.05)
        if not group():break
    return dict(actions=actions,known_generations=list(known.values()),remaining_session=group(),
        unknown_session_members=unknown,foreign_signals_sent=False,child_exit_code=child.poll())
def verify_seed2():
    post=json.loads(base64.b64decode(package['seed2_postcheck']))
    require(sha(base64.b64decode(package['seed2_postcheck']))==package['seed2_postcheck_sha256']
        and sha(base64.b64decode(package['seed2_postcheck_source']))==post['source_sha256'],'ROOT independent seed2 postcheck source/receipt exact')
    for name,key in (('result.json','remote_result_sha256'),('fit.stdout','raw_stdout_sha256'),('fit.stderr','raw_stderr_sha256')):
        require(sha(canonical(seed2_receipt/name).read_bytes())==post[key],'closed seed2 full raw/result changed')
    require(sha(canonical(seed2_fit).read_bytes())==post['fit_result_sha256'],'closed seed2 fit result changed')
    # HH_261005 - Tighten this fresh last-seed revision before admission; do not rewrite the historical seed2 gate.
    seed2_output=campaign/'seed_2026100502'
    for name,key in (('summary.json','seed_summary_sha256'),('declaration.json','seed_declaration_sha256'),
        ('control/declaration.json','control_declaration_sha256'),('control/history.jsonl','history_sha256')):
        require(sha(canonical(seed2_output/name).read_bytes())==post[key],'closed seed2 summary/declaration/full history changed')
    first=json.loads(canonical(seed2_receipt/'result.json').read_bytes())
    require(first['status']=='COMPLETE_SEED2_FIXED6000_NOT_PROMOTED' and first['exit_code']==0
        and first['complete_raw_stdout_stderr_retained'] is True and not first['remaining_session']
        and not first['unknown_session_members'],'seed2 must have actually closed before seed3')
def seals(gpu_required=True):
    for row in package['entries']:
        p=canonical(source/row['path']);require(p.is_file() and p.stat().st_size==row['bytes']
            and sha(p.read_bytes())==row['sha256'],'a current source changed')
    require(sha(canonical(plans/'sealed_sources.tar').read_bytes())==package['archive_sha256']
        and sha(canonical(cpu_path).read_bytes())==package['CPU_authority_sha256']
        and sha(canonical(plans/'CPU_admission.stdout').read_bytes())==package['admission_sha256'],'sealed CPU evidence changed')
    require(sha(canonical(gpu_path).read_bytes())==package['GPU_authority_sha256'],'existing immutable GPU authority changed')
    verify_seed2()
try:
    canonical(workspace);canonical(source);canonical(plans);canonical(campaign)
    require(shutil.disk_usage(workspace).free>=5*1024**3+512*1024**2,'original remote5GiB+512MiB reserve required')
    require(not receipt.exists() and not receipt.is_symlink() and not output.exists() and not output.is_symlink()
        ,'fresh seed3/receipts required; no resume/overwrite')
    seals(gpu_required=False)
    grant=base64.b64decode(package['GPU_authority']);require(sha(grant)==package['GPU_authority_sha256'],'exact GPU grant transfer required')
    authority=json.loads(grant);admission=json.loads(canonical(plans/'CPU_admission.stdout').read_bytes())
    require(authority['authorized_modes']==['fit-one'] and authority['physical_GPU_index']==0
        and sha(encode(authority['contract']))==package['contract_sha256']
        and authority['schedule_sha256']=={k:v['schedule_sha256'] for k,v in admission['declaration']['all_seed_schedules'].items()},
        'ROOT GPU-only grant must bind all3 actual CPU schedule seals')
    receipt.mkdir();created=True
    new(receipt/'caller_started.json',dict(UTC=datetime.now(timezone.utc).isoformat(),caller=identity(os.getpid()),
        protocol=package['protocol'],GPU_authority_sha256=package['GPU_authority_sha256'],automatic_retry=False,
        additional_seed_launched=False))
    review=base64.b64decode(package['execution_review'])
    require(sha(review)==package['execution_review_sha256'],'ROOT exact new seed3 review required')
    review_doc=json.loads(review)
    require(review_doc['helper_sha256']==package['protocol']['helper_sha256']
        and review_doc['test_sha256']==package['protocol']['test_sha256']
        and review_doc['seed2_whole_postcheck_sha256']==package['seed2_postcheck_sha256']
        and review_doc['seed']==2026100503,'ROOT seed3 review must bind exact source/tests/closed seed2')
    with (receipt/'ROOT_seed3_GPU0_execution_review.json').open('xb') as f:f.write(review)
    with (receipt/'ROOT_whole_seed2_training_postcheck.json').open('xb') as f:f.write(base64.b64decode(package['seed2_postcheck']))
    with (receipt/'ROOT_seed2_postcheck_source.py').open('xb') as f:f.write(base64.b64decode(package['seed2_postcheck_source']))
    seals();before_free=shutil.disk_usage(workspace).free
    argv=[str(workspace/'venvs/py312/bin/python'),str(source/'tools/workspace_python.py'),'-m',
        'scripts.e2e.train_temporal_route_joint_three_seed_train_only_v2_20261005','--','--mode','fit-one',
        '--seed','2026100503','--output',str(output),'--authorization',str(gpu_path),
        '--authorization-sha256',package['GPU_authority_sha256']]
    env=dict(os.environ,CUDA_VISIBLE_DEVICES='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',
        CUBLAS_WORKSPACE_CONFIG=':4096:8',PYTHONDONTWRITEBYTECODE='1',
        OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1')
    with (receipt/'fit.stdout').open('xb') as out,(receipt/'fit.stderr').open('xb') as err:
        parent_pid=os.getpid()
        child=subprocess.Popen(argv,cwd=str(source),env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True,
            preexec_fn=lambda:parent_death_guard(parent_pid))
        leader=identity(child.pid);require(leader['ppid']==os.getpid() and leader['pgid']==leader['session']==child.pid,'owned fit leader required')
        known[leader['pid']]=leader
        new(receipt/'child_started.json',dict(UTC=datetime.now(timezone.utc).isoformat(),caller=identity(os.getpid()),leader=leader,
            argv=argv,environment_override={'CUDA_VISIBLE_DEVICES':env['CUDA_VISIBLE_DEVICES'],
            'CUBLAS_WORKSPACE_CONFIG':':4096:8','PYTHONDONTWRITEBYTECODE':'1'},free_before_fit_bytes=before_free,
            environment_threads={k:env[k] for k in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','PYTHONUNBUFFERED')},
            leader_parent_death_signal='SIGTERM',unhandled_supervisor_loss='descendant post-cleanup UNKNOWN; ROOT must independently inspect generations',
            runner_original_GPU0_lease_and_fresh_preflight=True,fit_loop_limit_seconds=5400,caller_limit_seconds=7200))
        selector=selectors.DefaultSelector();selector.register(child.stdout,selectors.EVENT_READ,out);selector.register(child.stderr,selectors.EVENT_READ,err)
        begin=time.monotonic();last_census=0;interruption=None;cleanup_result=None;interrupted_at=None
        while selector.get_map():
            if time.monotonic()-last_census>=1:
                observe();last_census=time.monotonic()
                if unknown and interruption is None:interruption='unproven new session ancestry'
            if time.monotonic()-begin>7200 and interruption is None:interruption='prospective caller7200s deadline'
            if shutil.disk_usage(workspace).free<5*1024**3+512*1024**2 and interruption is None:interruption='remote reserve no longer available'
            if interruption and cleanup_result is None:interrupted_at=time.monotonic();cleanup_result=cleanup()
            for key,_ in selector.select(timeout=.25):
                raw=os.read(key.fileobj.fileno(),65536)
                if not raw:selector.unregister(key.fileobj);continue
                key.data.write(raw);key.data.flush()
                if key.data.tell()>64*1024**2 and interruption is None:interruption='full raw log64MiB bound exceeded; whole chunk retained'
            if interrupted_at is not None and time.monotonic()-interrupted_at>20:
                raise RuntimeError('owned cleanup could not close all raw pipes within20s; remaining tail UNKNOWN')
        exit_code=child.wait(timeout=5);observe()
        if group():
            natural_until=time.monotonic()+5
            while group() and time.monotonic()<natural_until:observe();time.sleep(.05)
        if group():cleanup_result=cleanup()
        result.update(exit_code=exit_code,leader=leader,known_generations=list(known.values()),unknown_session_members=unknown,
            remaining_session=group(),interruption=interruption,cleanup=cleanup_result,
            complete_raw_stdout_stderr_retained=True,free_after_fit_bytes=shutil.disk_usage(workspace).free)
        seals();result['current_archive_sources_CPU_GPU_grants_unchanged']=True
    require(not result['remaining_session'] and not result['unknown_session_members'],'fit session closure unproven; no foreign signal')
    require(exit_code==0 and interruption is None,'one seed3 fit failed/interrupted; no retry or automatic next seed')
    summary=json.loads(canonical(output/'summary.json').read_bytes())
    require(summary['status']=='FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED' and summary['seed']==2026100503
        and summary['fit']['optimizer_updates']==6000 and summary['fit']['examples_seen']==24000
        and summary['final_endpoint']['all101_numeric_finite'] is True and summary['final_endpoint']['frozen47_exact'] is True,
        'real fixed6000 endpoint required')
    status='COMPLETE_SEED3_FIXED6000_NOT_PROMOTED'
except BaseException as error:
    result['error']=repr(error)
    if 'complete_raw_stdout_stderr_retained' not in result:
        result['complete_raw_stdout_stderr_retained']=False
        result['unreceived_pipe_tail']='UNKNOWN; all received whole chunks retained without truncation'
    try:result['exception_cleanup']=cleanup()
    except BaseException as cleanup_error:result['owned_cleanup_error']=repr(cleanup_error)
finally:
    artifacts={}
    if created:
        try:
            seals();result['post_current_archive_CPU_GPU_seals_exact']=True
        except BaseException as seal_error:
            status='FAILED_NO_RETRY';result['post_source_seal_error']=repr(seal_error)
        try:
            for p in sorted(receipt.iterdir()):
                require(not p.is_symlink(),'unexpected raw receipt alias; never delete')
                if p.is_file():artifacts[p.name]=dict(bytes=p.stat().st_size,sha256=sha(p.read_bytes()))
            if output.exists():
                for p in sorted(output.rglob('*')):
                    require(not p.is_symlink(),'unexpected seed output alias; never delete')
                    if p.is_file():artifacts['seed/'+str(p.relative_to(output))]=dict(bytes=p.stat().st_size,sha256=sha(p.read_bytes()))
        except BaseException as audit_error:
            status='FAILED_NO_RETRY';result['partial_artifact_inventory_error']=repr(audit_error)
        result.update(status=status,UTC=datetime.now(timezone.utc).isoformat(),automatic_retry=False,additional_seed_launched=False,
            failed_attempt_retained=status!='COMPLETE_SEED3_FIXED6000_NOT_PROMOTED',artifacts=artifacts)
        new(receipt/'result.json',result)
    response=dict(status=status,result=result,remote_receipt=str(receipt))
    encoded=encode(response)
    if len(encoded)>96*1024:
        failure=dict(error='metadata response exceeds96KiB; full result retained unchanged',
            full_result_path=str(receipt/'result.json'),full_result_sha256=sha((receipt/'result.json').read_bytes()),
            complete_response_bytes=len(encoded),additional_seed_launched=False,automatic_retry=False)
        new(receipt/'metadata_bound_failure.json',failure)
        encoded=encode(dict(status='FAILED_METADATA_BOUND_FULL_REMOTE_RESULT_RETAINED',result=failure,remote_receipt=str(receipt)))
    print(encoded.decode(),flush=True)
'''


def remote_program():
    import ast
    tree = ast.parse(Path(__file__).read_bytes())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ("same_generation", "admit_family")]
    require(len(functions) == 2, "one exact local ancestry implementation required")
    return REMOTE_PREFIX + "\n" + "\n".join(ast.unparse(n) for n in functions) + "\n" + REMOTE_BODY


def main(argv=None):
    import argparse
    parser=argparse.ArgumentParser(description='HH_261005 - One ROOT-reviewed independent seed3; never auto-dispatch')
    parser.add_argument('--review-sha256',required=True)
    args=parser.parse_args(argv)
    require(len(args.review_sha256)==64 and all(c in '0123456789abcdef' for c in args.review_sha256),'exact ROOT seed3 review CLI SHA required')
    require(shutil.disk_usage(canonical(PLAN)).free >= 64*1024**2 + 1024**2,
        "local metadata-only64MiB floor+1MiB closing reserve required; no CARLA guard change")
    require(not OUT.exists() and not OUT.is_symlink(), "fresh local supervisor receipts required")
    authority = canonical(PLAN / "ROOT_GPU0_TRAIN_only_authorization.json").read_bytes()
    admission_raw = canonical(CPU / "remote_CPU_admission.stdout").read_bytes()
    manifest_raw = canonical(CPU / "transfer_manifest.json").read_bytes()
    require(sha(manifest_raw) == TRANSFER_MANIFEST_SHA, "exact actual CPU transfer manifest required before package/write")
    manifest = json.loads(manifest_raw)
    require(sha(authority) == GPU_AUTH_SHA and sha(admission_raw) == ADMISSION_SHA, "exact ROOT GPU grant/actual CPU admission required")
    grant = json.loads(authority); admission = json.loads(admission_raw)
    require(grant["authorized_modes"] == ["fit-one"] and sha(encode(grant["contract"])) == CONTRACT_SHA
        and grant["schedule_sha256"] == {k: v["schedule_sha256"] for k, v in admission["declaration"]["all_seed_schedules"].items()},
        "GPU-only authority binds all3 actual schedules")
    rows = manifest["entries"]
    require(len(rows) == 33 and len({row["path"] for row in rows}) == 33
        and all(type(row["path"]) is str and str(Path(row["path"])) == row["path"]
            and not Path(row["path"]).is_absolute() and ".." not in Path(row["path"]).parts for row in rows)
        and manifest["archive_sha256"] == SOURCE_ARCHIVE_SHA
        and len(grant["contract"]["sources"]) == 31
        and all(next((row["sha256"] for row in rows if row["path"] == name), None) == digest
            for name, digest in grant["contract"]["sources"].items()),
        "exact admitted33 relative sources/archive and signed31 runtime closure required")
    review_raw=canonical(EXECUTION_REVIEW).read_bytes()
    post_raw=canonical(SEED2_POST).read_bytes()
    post_source_raw=canonical(SEED2_POST_SOURCE).read_bytes()
    require(sha(review_raw)==args.review_sha256,"exact ROOT execution review required before package/write")
    review=json.loads(review_raw); post=json.loads(post_raw)
    require(review["status"]=="APPROVED_EXACT_SEED3_GPU0_EXECUTION" and review["seed"]==2026100503
        and review["helper_sha256"]==sha(Path(__file__).read_bytes())
        and review["test_sha256"]==sha(canonical(TEST_SOURCE).read_bytes())
        and review["GPU_authority_sha256"]==GPU_AUTH_SHA and review["CPU_admission_sha256"]==ADMISSION_SHA
        and review["transfer_manifest_sha256"]==TRANSFER_MANIFEST_SHA
        and review["seed2_whole_postcheck_sha256"]==sha(post_raw)
        and review["seed2_postcheck_source_sha256"]==sha(post_source_raw)
        and review["schedule_sha256"]==SEED3_SCHEDULE_SHA
        and review["automatic_retry"] is False and review["additional_seed_auto_launch"] is False,
        "ROOT review must bind closed independent seed2 source/receipt and exact seed3 inputs")
    require(post["status"]=="PASS_WHOLE_SEED2_FIXED6000_NOT_PROMOTED" and post["seed"]==2026100502
        and post["source_sha256"]==sha(post_source_raw) and post["checked_optimizer_updates"]==6000
        and post["checked_examples_seen"]==24000 and all(post["checks"].get(k) is True for k in (
            "all_6000_schedule_rows_exact","all_12_checkpoint_triplets_exact","all_101_final_and_frozen47_exact",
            "source_current_and_archive_grants_exact","complete_raw_streams_exact","owned_session_closed",
            "all_failed_tails_preserved")),"independent whole actual seed2 closure required; a passed summary alone is insufficient")
    program = remote_program(); compile(program, "sealed_seed3_remote_supervisor", "exec")
    protocol = dict(provenance="HH_261005 - One exact seed3; no retry/next seed; full remote raw logs and all endpoint tails.",
        helper_sha256=sha(Path(__file__).read_bytes()), remote_program_sha256=sha(program.encode()),
        GPU_authority_sha256=GPU_AUTH_SHA, CPU_authority_sha256=CPU_AUTH_SHA, full_CPU_admission_sha256=ADMISSION_SHA,
        test_sha256=sha(canonical(TEST_SOURCE).read_bytes()),execution_review_sha256=args.review_sha256,
        seed2_whole_postcheck_sha256=sha(post_raw),seed2_postcheck_source_sha256=sha(post_source_raw),existing_GPU_grant_reused=True,
        contract_sha256=CONTRACT_SHA, physical_GPU_UUID=GPU_UUID, original_GPU0_lease="runs/campaigns/.gpu0_training.lock",
        original_remote_floor_bytes=5*1024**3, original_remote_write_reserve_bytes=512*1024**2,
        local_metadata_only_floor_bytes=64*1024**2, local_metadata_closing_reserve_bytes=1024**2,
        local_metadata_limit_bytes=LOCAL_LIMIT, full_raw_logs_remote=True, remote_log_bound_per_stream_bytes=64*1024**2,
        truncate_raw_logs=False, fit_loop_limit_seconds=5400, caller_deadline_seconds=7200, SSH_deadline_seconds=7260,
        source_count=33, canonical_runtime_source_count=31, no_installation=True, no_automatic_retry=True,
        environment_threads={'OMP_NUM_THREADS':'4','MKL_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1','PYTHONUNBUFFERED':'1'},
        SIGHUP_SIGTERM_SIGINT_cleanup='individual known-generation owned signals only',leader_parent_death_signal='SIGTERM',
        unhandled_supervisor_loss='descendant closure UNKNOWN; no claim or automatic retry after SSH disconnection',
        no_automatic_additional_seed=True, model_promoted=False, formal_main_approved=0, formal_sub_approved=0)
    package = dict(GPU_authority=base64.b64encode(authority).decode(), GPU_authority_sha256=GPU_AUTH_SHA,
        CPU_authority_sha256=CPU_AUTH_SHA, admission_sha256=ADMISSION_SHA, contract_sha256=CONTRACT_SHA,
        archive_sha256=manifest["archive_sha256"], entries=manifest["entries"], protocol=protocol,
        execution_review=base64.b64encode(review_raw).decode(),execution_review_sha256=args.review_sha256,
        seed2_postcheck=base64.b64encode(post_raw).decode(),seed2_postcheck_sha256=sha(post_raw),
        seed2_postcheck_source=base64.b64encode(post_source_raw).decode())
    OUT.mkdir(); total = 0
    def write(name, raw, *, closing=False):
        nonlocal total
        require(total + len(raw) <= (LOCAL_LIMIT if closing else LOCAL_LIMIT-16*1024),
            "metadata128KiB includes16KiB failure closing; large raw evidence remains remote")
        with (OUT / name).open("xb") as stream: stream.write(raw)
        total += len(raw)
    write("protocol.json", encode(protocol) + b"\n")
    write("caller_started.json", encode(dict(UTC=datetime.now(timezone.utc).isoformat(), caller=identity(os.getpid()),
        SSH_alias="pro6000-training", helper_sha256=protocol["helper_sha256"], one_seed=2026100503)) + b"\n")
    child = None; owned = None
    try:
        argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "pro6000-training", "python3 -c " + shlex.quote(program)]
        child = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        owned = identity(child.pid); require(owned["ppid"] == os.getpid() and owned["pgid"] == owned["session"] == child.pid, "owned SSH session required")
        write("SSH_child_started.json", encode(dict(child=owned, caller=identity(os.getpid()), remote_program_sha256=protocol["remote_program_sha256"])) + b"\n")
        stdout, stderr = child.communicate(encode(package), timeout=7260)
        write("SSH.stdout", stdout); write("SSH.stderr", stderr)
        require(child.returncode == 0, "seed3 remote caller failed; no retry")
        response = json.loads(stdout)
        require(response["remote_receipt"] == "/home/pro6000/personal/hwanhong/portable_e2e/plans/training/temporal_route_joint_three_seed_train_only_20261005_v2/seed_2026100503_fit_supervisor_v1",
            "one exact private remote receipt boundary required")
        require(sha(canonical(PLAN / "ROOT_GPU0_TRAIN_only_authorization.json").read_bytes()) == GPU_AUTH_SHA
            and sha(canonical(PLAN / "ROOT_CPU_TRAIN_only_authorization.json").read_bytes()) == CPU_AUTH_SHA
            and sha(canonical(EXECUTION_REVIEW).read_bytes()) == args.review_sha256
            and sha(canonical(SEED2_POST).read_bytes()) == sha(post_raw)
            and sha(canonical(SEED2_POST_SOURCE).read_bytes()) == sha(post_source_raw),
            "local CPU/GPU grants changed during seed3")
        write("exited.json", encode(dict(exit_code=child.returncode, status=response["status"],
            full_remote_metadata_sha256=sha(stdout), stderr_sha256=sha(stderr), remote_receipt=response["remote_receipt"],
            all_raw_logs_and_checkpoints_stay_remote=True, additional_seed_launched=False, automatic_retry=False)) + b"\n")
        print(json.dumps(dict(status=response["status"], remote_receipt=response["remote_receipt"], local_output=str(OUT),
            metadata_sha256=sha(stdout),actual_seed=2026100503), allow_nan=False))
        return 0 if response["status"] == "COMPLETE_SEED3_FIXED6000_NOT_PROMOTED" else 2
    except BaseException as error:
        cleanup = dict(SSH_child_started=child is not None)
        if child is not None:
            try:
                if child.poll() is None:
                    require(owned is not None and identity(child.pid) == owned, "owned SSH generation required before cleanup")
                    child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        require(identity(child.pid) == owned, "SSH generation changed before final owned signal")
                        child.kill();child.wait(timeout=5)
                cleanup["exit_code"] = child.poll()
            except BaseException as cleanup_error:cleanup["error"] = repr(cleanup_error)
        if isinstance(error, subprocess.TimeoutExpired):
            if error.stdout is not None:write("SSH_timeout.stdout", error.stdout)
            if error.stderr is not None:write("SSH_timeout.stderr", error.stderr)
        write("failure.json", encode(dict(error=repr(error), SSH_cleanup=cleanup, failed_attempt_retained=True,
            automatic_retry=False, remote_outcome="UNKNOWN if transport failed; bounded remote supervisor retains raw evidence")) + b"\n", closing=True)
        raise


if __name__ == "__main__": raise SystemExit(main())
