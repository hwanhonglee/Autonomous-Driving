"""HH_261005 - Prepare the same whole TRAIN-only seed3 postcheck with fresh V1 result identities; keep seed2 V2 frozen.

The remote CPU checker replays the sealed TRAIN membership/schedule and hashes
all existing evidence. It does not call an evaluator, loss, forward, optimizer,
CUDA, or a simulator. Actual execution needs ROOT's new exact source/test review.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess

ROOT=Path(__file__).resolve().parents[2]
PLAN=ROOT/"storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2"
OUT=PLAN/"seed_2026100503_fit_supervisor_v2"
REVIEW=PLAN/"ROOT_seed3_whole_training_postcheck_authorization_v1.json"
TEST=ROOT/"learning/tests/test_postcheck_three_seed_joint6000_seed3_v1_20261005.py"
SUPERVISOR=ROOT/"tools/maintenance/supervise_three_seed_joint6000_seed3_v2_20261005.py"
SUPERVISOR_SHA="106a56e0761d82b7c41cc305fa74763150014974aee613e3218917a9bd8a30af"
CPU_AUTH_SHA="4b437183cb0bd70b3589d3e03797b0a2ab8f977dd841ea6f5cef69debb9d4a29"
GPU_AUTH_SHA="a2f25569e74ed99845757a4954846a9cab0ea500cde7adc3736b89f71c779ee0"
ADMISSION_SHA="3f5ae127e3e9b734586e30d34d5c5cb6eb63b00fad8a0ab10c409eef11303480"
ARCHIVE_SHA="a16591999e09dad04d3aa367e15ba841ebb3c9d2bad83a9f982515a40a5996ec"
CONTRACT_SHA="cdde23e8f6265f49a4009e07be0e2e0cddc8b881ed4dca33307501e36ffd65c1"
MANIFEST=PLAN/"SOURCE_transfer_and_CPU_admission/transfer_manifest.json"
MANIFEST_SHA="bf96e1eb6d03c2a74bc876b6319cbf0bf4eaa2fef544b4cfbd29bee30ad19b79"
SEED=2026100503


def require(value,message):
    if not value:raise RuntimeError(message)


def sha(raw):return hashlib.sha256(raw).hexdigest()


def encode(value):return json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()


def digest(path):
    value=hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""):value.update(block)
    return value.hexdigest()


def canonical(path):
    path=Path(path).absolute()
    require(path==path.resolve(strict=True) and not any(p.is_symlink() for p in (path,*path.parents)),"canonical no-alias evidence required")
    return path


def identity(pid):
    text=(Path("/proc")/str(pid)/"stat").read_text();fields=text[text.rfind(")")+2:].split()
    return dict(pid=pid,ppid=int(fields[1]),pgid=int(fields[2]),session=int(fields[3]),start_ticks=int(fields[19]))


def verify_history(rows,order,tokens,hashes,domains):
    # HH_261005 - Validate the entire fixed denominator, not progress-print rows or selected checkpoints.
    require(len(rows)==len(order)==len(tokens)==len(hashes)==6000,"whole6000 history/order/token/tensor-digest denominator required")
    require(len(domains)==4 and len(set(domains))==4,"exact original four domain names required")
    elapsed=-1.0;curved=0
    for step,(row,indices,ids,batch_sha) in enumerate(zip(rows,order,tokens,hashes),1):
        require(type(row.get("optimizer_updates")) is int and row["optimizer_updates"]==step
            and type(row.get("examples_seen")) is int and row["examples_seen"]==4*step
            and type(row.get("indices")) is list and len(row["indices"])==4 and all(type(v) is int for v in row["indices"])
            and type(row.get("sample_ids")) is list and len(row["sample_ids"])==4 and all(type(v) is str for v in row["sample_ids"])
            and row["indices"]==indices and row["sample_ids"]==ids and row.get("batch_sha256")==batch_sha,
            "one actual history update/tokens/indices/batch digest differs")
        require(type(batch_sha) is str and len(batch_sha)==64 and all(c in "0123456789abcdef" for c in batch_sha),"real batch SHA required")
        expected=step%2==1
        require(type(row.get("legacy_curved")) is bool and row["legacy_curved"] is expected,"curved/straight3000 alternation changed")
        curved+=int(expected)
        require(row.get("intrinsic_coefficient")==0 and row.get("intrinsic_XY")==0,"unchanged CONTROL objective required")
        require(set(row.get("per_domain_gradients",{}))==set(domains)
            and all(v.get("all_present_branch_gradients_finite") is True
                and type(v.get("gradient_l2_norm")) in (int,float) and math.isfinite(v["gradient_l2_norm"])
                and v["gradient_l2_norm"]>=0 and type(v.get("unused_branch_tensors")) is int
                and v["unused_branch_tensors"]>=0 for v in row["per_domain_gradients"].values()),
            "one actual domain gradient receipt was not finite")
        for field in ("loss","preclip_gradient_norm","intrinsic_XY","intrinsic_coefficient"):
            require(type(row.get(field)) in (int,float) and math.isfinite(row[field]),"one actual history scalar was not finite")
        require(row["preclip_gradient_norm"]>=0,"a norm cannot be negative")
        for field in ("recovery0","recovery1","v4","legacy","learning_rates"):
            values=row.get(field)
            require(type(values) is dict and values and all(type(v) in (int,float) and math.isfinite(v) for v in values.values()),
                "one actual loss/domain/learning-rate scalar was not finite")
        now=row.get("elapsed_fit_seconds")
        require(type(now) in (int,float) and math.isfinite(now) and now>=0 and now>=elapsed and now<5400,"original fit wall limit/chronology required")
        elapsed=float(now)
    return dict(optimizer_updates=6000,examples_seen=24000,domain_exposures=[6000]*4,
        legacy_curved=curved,legacy_straight=6000-curved,all_6000_schedule_rows_exact=True)


REMOTE=r'''
import argparse, hashlib, importlib.util, json, os, pathlib, sys, time
from datetime import datetime, timezone
package=json.loads(sys.stdin.buffer.read());workspace=pathlib.Path('/home/pro6000/personal/hwanhong/portable_e2e')
study='temporal_route_joint_three_seed_train_only_20261005_v2';source=workspace/'source_worktrees'/study
plans=workspace/'plans/training'/study;seed=workspace/'runs/training'/study/'seed_2026100503'
receipt=plans/'seed_2026100503_fit_supervisor_v2';control=seed/'control';checks={};report={}
def read(path,limit=32*1024**2):
    path=canonical(path);require(path.is_file() and path.stat().st_size<=limit,'bounded regular immutable JSON required')
    return json.loads(path.read_bytes())
def identity(pid):
    s=(pathlib.Path('/proc')/str(pid)/'stat').read_text();f=s[s.rfind(')')+2:].split()
    return dict(pid=pid,ppid=int(f[1]),pgid=int(f[2]),session=int(f[3]),start_ticks=int(f[19]))
def session_rows(leader):
    result=[]
    for path in pathlib.Path('/proc').iterdir():
        if not path.name.isdecimal():continue
        try:row=identity(int(path.name))
        except (FileNotFoundError,ProcessLookupError):continue
        if row['pgid']==leader['pgid'] and row['session']==leader['session']:result.append(row)
    return result
try:
    require(os.environ.get('CUDA_VISIBLE_DEVICES')=='' and pathlib.Path(sys.prefix).resolve()==workspace/'venvs/py312','read-only CPU existing privatevenv required')
    canonical(source);canonical(seed);canonical(receipt)
    for name,pin in (('result.json',package['expected_remote_result_sha256']),
        ('fit.stdout',package['expected_fit_stdout_sha256']),('fit.stderr',package['expected_fit_stderr_sha256'])):
        require(digest(canonical(receipt/name))==pin,'ROOT-admitted completed supervisor/full raw bytes changed')
    require(digest(canonical(control/'fit_result.json'))==package['expected_fit_result_sha256'],
        'ROOT-admitted completed fixedfit result changed')
    first=read(receipt/'result.json');started=read(receipt/'child_started.json')
    require(first['status']=='COMPLETE_SEED3_FIXED6000_NOT_PROMOTED' and first['exit_code']==0
        and first['leader']==started['leader'] and first['complete_raw_stdout_stderr_retained'] is True
        and not first['remaining_session'] and not first['unknown_session_members'],'authentic ended complete supervisor required')
    require(not session_rows(started['leader']),'owned seed3 session must actually be absent before CPU postcheck')
    checks['owned_session_closed']=True
    require(started['argv'][started['argv'].index('--seed')+1]=='2026100503'
        and started['environment_override']['CUDA_VISIBLE_DEVICES']=='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',
        'exact actual seed3/GPU0 process receipt required')
    for name,pin in (('ROOT_CPU_TRAIN_only_authorization.json',package['CPU_authority_sha256']),
        ('ROOT_GPU0_TRAIN_only_authorization.json',package['GPU_authority_sha256']),
        ('CPU_admission.stdout',package['CPU_admission_sha256']),('sealed_sources.tar',package['archive_sha256'])):
        require(digest(canonical(plans/name))==pin,'CPU/GPU/admission/archive input changed')
    grant=read(plans/'ROOT_GPU0_TRAIN_only_authorization.json');contract=grant['contract']
    require(sha(encode(contract))==package['contract_sha256'],'original prospective contract changed')
    for name,pin in contract['sources'].items():require(digest(canonical(source/name))==pin,'one canonical runtime source changed')
    require(len(contract['sources'])==31,'exact31 runtime closure required')
    require(len(package['entries'])==33,'all33 admitted current/provenance files required')
    for row in package['entries']:
        path=canonical(source/row['path'])
        require(path.is_file() and path.stat().st_size==row['bytes'] and digest(path)==row['sha256'],
            'one actual current33 source/provenance file changed')
    require(first['post_current_archive_CPU_GPU_seals_exact'] is True and first['current_archive_sources_CPU_GPU_grants_unchanged'] is True,
        'supervisor actual33 current/archive proof required')
    for name,row in first['artifacts'].items():
        path=seed/name[5:] if name.startswith('seed/') else receipt/name
        require(canonical(path).is_file() and path.stat().st_size==row['bytes'] and digest(path)==row['sha256'],
            'one retained full raw/source/checkpoint artifact changed')
    checks['complete_raw_streams_exact']=True;checks['all_failed_tails_preserved']=True
    summary=read(seed/'summary.json');declaration=read(seed/'declaration.json');armdecl=read(control/'declaration.json')
    result=read(control/'fit_result.json');fresh=read(control/'fresh_initialization.json')
    require(summary['status']=='FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED' and summary['seed']==2026100503
        and summary['fit']==result and result['arm']=='control' and result['completed_fixed_endpoint'] is True
        and result['optimizer_updates']==6000 and result['examples_seen']==24000,'actual fixed6000 endpoint required')
    require(armdecl==dict(declaration,arm='control') and declaration['protocol']==contract
        and declaration['authorization_sha256']==package['GPU_authority_sha256'],'seed/control declaration exact identity required')
    for field in ('evaluation_only_payloads_opened','model_promoted','vehicle_control_approved'):
        require(summary[field] is False and declaration[field] is False,'TRAIN-only and no deployment scope required')
    require(summary['formal_main_approved']==summary['formal_sub_approved']==0,'no capability approval from training')
    spec=importlib.util.spec_from_file_location('hh_post_bootstrap',source/'tools/workspace_python.py')
    bootstrap=importlib.util.module_from_spec(spec);spec.loader.exec_module(bootstrap);bootstrap.configure(source)
    from scripts.e2e import train_temporal_route_joint_three_seed_train_only_v2_20261005 as trainer
    require(not trainer.torch.cuda.is_initialized(),'CPU-only checker cannot initialize CUDA')
    trainer.torch.set_num_threads(4)
    require(trainer.contract()==contract,'authentic current source-derived TRAIN protocol differs')
    args=trainer.parser().parse_args(['--mode','fit-one','--seed','2026100503','--output',str(seed),
        '--authorization',str(plans/'ROOT_GPU0_TRAIN_only_authorization.json'),'--authorization-sha256',package['GPU_authority_sha256']])
    training,audits,low,legacy,strata,train_receipts=trainer.pair.load_train(args)
    require(all(d.split=='train' for d in (training,low,legacy)) and train_receipts==declaration['TRAIN_receipts']
        and trainer.json_sha(train_receipts)==declaration['training_dataset_fingerprint_sha256'],'actual TRAIN labels/membership/futures must be unchanged')
    groups=trainer.previous.recovery_groups(training,audits);order=trainer.fixed_schedule(groups,strata,2026100503)
    schedule=trainer.verify_schedule(order,groups,strata,2026100503)
    require(schedule==declaration['all_seed_schedules']['2026100503'] and schedule['schedule_sha256']==grant['schedule_sha256']['2026100503'],
        'actual declared schedule must equal authenticated original sampler')
    tokens=[trainer.pair.expected_tokens((training,low,legacy),row) for row in order]
    require(tokens==declaration['fixed_sample_ids'],'all6000 original TRAIN token rows required')
    history_path=canonical(control/'history.jsonl');require(history_path.stat().st_size<=64*1024**2,'full bounded6000 history required')
    rows=[json.loads(line) for line in history_path.read_text().splitlines()]
    report['history']=verify_history(rows,order,tokens,summary['actual_batch_sha256'],trainer.original.BATCH_DOMAINS)
    checks['all_6000_schedule_rows_exact']=True
    for name,pin in declaration['immutable_input_sha256'].items():require(digest(canonical(pathlib.Path(name)))==pin,'one common/TRAIN/GPU input changed')
    images=trainer.pair.verify_training_images((training,low,legacy))
    require(images==summary['images_before']==summary['images_after'] and summary['TRAIN_receipts_before_after_exact'] is True,
        'all authentic current TRAIN/history-source images must equal both retained before/after proofs')
    report['actual_TRAIN_images']=images
    parent=trainer.original.load_joint_parent(trainer.COMMON_RUN/'exports/step_003000/model_state.npz',
        trainer.COMMON_RUN/'exports/step_003000/metadata.json',backbone_parent_weights=trainer.BACKBONE_RUN/'exports/step_003000/model_state.npz',
        backbone_parent_metadata=trainer.BACKBONE_RUN/'exports/step_003000/metadata.json',source_root=source)
    trainer.original.freeze_inputs(parent)
    require(trainer.state_sha(parent)==declaration['common_initial101_sha256']
        and trainer.original.frozen_fingerprint(parent.state_dict())==declaration['frozen47_sha256']
        and fresh==dict(initial101_sha256=declaration['common_initial101_sha256'],common101_sha256=declaration['common_initial101_sha256'],
            fresh_optimizer_state_empty=True,optimizer_updates_before_fit=0,optimizer_resume=False),
        'same authenticated common101/new emptyAdamW receipt required')
    final=trainer.verify_final_endpoint(control,result,parent,declaration,contract['sources'])
    require(final==summary['final_endpoint'],'independent all12 checkpoint triplets/final101/frozen47 differs')
    checks['all_12_checkpoint_triplets_exact']=True;checks['all_101_final_and_frozen47_exact']=True
    # HH_261005 - Repeat immutable seals after the TRAIN/parent numerical census; never rely on a stale before-only assertion.
    for row in package['entries']:
        path=canonical(source/row['path']);require(path.stat().st_size==row['bytes'] and digest(path)==row['sha256'],'one post-current33 file changed')
    for name,pin in (('ROOT_CPU_TRAIN_only_authorization.json',package['CPU_authority_sha256']),
        ('ROOT_GPU0_TRAIN_only_authorization.json',package['GPU_authority_sha256']),
        ('CPU_admission.stdout',package['CPU_admission_sha256']),('sealed_sources.tar',package['archive_sha256'])):
        require(digest(canonical(plans/name))==pin,'one post-immutable CPU/GPU/archive seal changed')
    for name,pin in declaration['immutable_input_sha256'].items():require(digest(canonical(pathlib.Path(name)))==pin,'one post-common/TRAIN/GPU input changed')
    checks['source_current_and_archive_grants_exact']=True
    require(not trainer.torch.cuda.is_initialized() and not session_rows(started['leader']),'CPU-only/ended scope remains exact after full verification')
    report.update(status='PASS_WHOLE_SEED3_FIXED6000_NOT_PROMOTED',seed=2026100503,checked_optimizer_updates=6000,
        checked_examples_seen=24000,source_sha256=package['source_sha256'],test_sha256=package['test_sha256'],checks=checks,
        remote_result_sha256=digest(receipt/'result.json'),fit_result_sha256=digest(control/'fit_result.json'),
        raw_stdout_sha256=digest(receipt/'fit.stdout'),raw_stderr_sha256=digest(receipt/'fit.stderr'),
        seed_declaration_sha256=digest(seed/'declaration.json'),control_declaration_sha256=digest(control/'declaration.json'),
        seed_summary_sha256=digest(seed/'summary.json'),history_sha256=digest(history_path),final_endpoint=final,
        checkpoint_count=12,source_current_and_archive_count=33,canonical_runtime_source_count=31,
        CUDA_initialized=False,NN_forward_called=False,optimizer_called=False,evaluation_payloads_opened=False,
        remote_files_written=False,model_promoted=False,formal_main_approved=0,formal_sub_approved=0)
except BaseException as error:
    report.update(status='FAILED_WHOLE_SEED3_POSTCHECK_RETAIN_ORIGINAL',error=repr(error),seed=2026100503,
        checks=checks,source_sha256=package['source_sha256'],failed_attempt_retained=True,model_promoted=False)
print(json.dumps(report,sort_keys=True,allow_nan=False),flush=True)
'''


def remote_program():
    nodes=[n for n in ast.parse(Path(__file__).read_bytes()).body if isinstance(n,ast.FunctionDef)
        and n.name in ("require","sha","encode","digest","canonical","verify_history")]
    require(len(nodes)==6,"one exact pure history/hash implementation required")
    header="import hashlib,json,math\nfrom pathlib import Path\n"
    return header+"\n".join(ast.unparse(n) for n in nodes)+"\n"+REMOTE


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--review-sha256",required=True);args=parser.parse_args(argv)
    review_raw=canonical(REVIEW).read_bytes();review=json.loads(review_raw)
    require(sha(review_raw)==args.review_sha256 and review["status"]=="APPROVED_READ_ONLY_SEED3_WHOLE_POSTCHECK"
        and review["source_sha256"]==digest(Path(__file__)) and review["test_sha256"]==digest(canonical(TEST))
        and review["supervisor_source_sha256"]==SUPERVISOR_SHA and digest(canonical(SUPERVISOR))==SUPERVISOR_SHA
        and review["GPU_authority_sha256"]==GPU_AUTH_SHA and review["CPU_admission_sha256"]==ADMISSION_SHA
        and review["seed"]==SEED and review["remote_writes"] is False and review["model_promoted"] is False,
        "exact ROOT source/test-bound read-only postcheck review required")
    require(shutil.disk_usage(canonical(OUT)).free>=64*1024**2+1024**2,"metadata-only local64MiB+1MiB closing reserve required")
    output=OUT/"ROOT_whole_seed3_training_postcheck_v1.json";rawout=OUT/"whole_seed3_postcheck_v1.SSH.stdout";rawerr=OUT/"whole_seed3_postcheck_v1.SSH.stderr"
    require(all(not p.exists() and not p.is_symlink() for p in (output,rawout,rawerr)),"fresh postcheck outputs; no overwrite/retry")
    program=remote_program();compile(program,"sealed_whole_seed3_CPU_postcheck","exec")
    manifest_raw=canonical(MANIFEST).read_bytes();require(sha(manifest_raw)==MANIFEST_SHA,'exact actual33 transfer manifest required')
    manifest=json.loads(manifest_raw)
    require(manifest['archive_sha256']==ARCHIVE_SHA and len(manifest['entries'])==33,'exact admitted33 archive closure required')
    package=dict(source_sha256=digest(Path(__file__)),test_sha256=digest(TEST),CPU_authority_sha256=CPU_AUTH_SHA,
        GPU_authority_sha256=GPU_AUTH_SHA,CPU_admission_sha256=ADMISSION_SHA,archive_sha256=ARCHIVE_SHA,contract_sha256=CONTRACT_SHA,
        entries=manifest['entries'],**{name:review[name] for name in ('expected_remote_result_sha256',
            'expected_fit_result_sha256','expected_fit_stdout_sha256','expected_fit_stderr_sha256')})
    command="CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 "+\
        "/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -c "+shlex.quote(program)
    child=None;owned=None;result={};stdout=b'';stderr=b''
    started=OUT/'whole_seed3_postcheck_v1.caller_started.json';failure=OUT/'whole_seed3_postcheck_v1.failure.json'
    require(all(not p.exists() and not p.is_symlink() for p in (started,failure)),'fresh caller/failure metadata required')
    with started.open('xb') as stream:stream.write(encode(dict(UTC=datetime.now(timezone.utc).isoformat(),caller=identity(os.getpid()),
        ROOT_review_sha256=args.review_sha256,source_sha256=package['source_sha256'],remote_writes=False))+b'\n')
    try:
        child=subprocess.Popen(["ssh","-o","BatchMode=yes","-o","ConnectTimeout=10","pro6000-training",command],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
        owned=identity(child.pid);require(owned['ppid']==os.getpid() and owned['pgid']==owned['session']==child.pid,'owned read-only SSH child required')
        stdout,stderr=child.communicate(encode(package),timeout=600)
        require(len(stdout)+len(stderr)<=112*1024,"whole read-only response exceeds local128KiB includingclosing; do not truncate")
        for path,raw in ((rawout,stdout),(rawerr,stderr)):
            with path.open("xb") as stream:stream.write(raw)
        require(child.returncode==0,"read-only remote checker failed; keep complete raw output")
        result=json.loads(stdout)
        require(digest(Path(__file__))==package["source_sha256"] and digest(TEST)==package["test_sha256"]
            and digest(REVIEW)==args.review_sha256 and digest(MANIFEST)==MANIFEST_SHA,"source/test/ROOT postcheck review/manifest changed")
        result.update(ROOT_review_sha256=args.review_sha256,UTC=datetime.now(timezone.utc).isoformat(),
            remote_program_sha256=sha(program.encode()),SSH_stdout_sha256=sha(stdout),SSH_stderr_sha256=sha(stderr))
        with output.open("xb") as stream:stream.write(encode(result)+b"\n")
    except BaseException as error:
        cleanup={}
        if child is not None:
            try:
                if child.poll() is None:
                    require(owned is not None and identity(child.pid)==owned,'exact owned SSH generation required before individual cleanup')
                    child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        require(identity(child.pid)==owned,'owned SSH generation changed before final signal')
                        child.kill();child.wait(timeout=5)
                cleanup['SSH_exit_code']=child.poll()
            except BaseException as cleanup_error:cleanup['error']=repr(cleanup_error)
        if isinstance(error,subprocess.TimeoutExpired):stdout=error.stdout or b'';stderr=error.stderr or b''
        receipt=dict(status='FAILED_READ_ONLY_POSTCHECK_CALLER_RETAINED',error=repr(error),cleanup=cleanup,
            raw_stdout_bytes_received=len(stdout),raw_stderr_bytes_received=len(stderr),
            raw_stdout_sha256=sha(stdout),raw_stderr_sha256=sha(stderr),unreceived_tail='UNKNOWN if transport interrupted',
            remote_writes=False,model_promoted=False,automatic_retry=False)
        if len(stdout)+len(stderr)<=112*1024:
            for path,raw in ((rawout,stdout),(rawerr,stderr)):
                if not path.exists():
                    with path.open('xb') as stream:stream.write(raw)
        else:receipt['oversized_full_received_output_not_locally_written']='whole received bytes hashed, not truncated; original evidence remains remote'
        with failure.open('xb') as stream:stream.write(encode(receipt)+b'\n')
        raise
    print(json.dumps({k:result[k] for k in ("status","seed","source_sha256")},allow_nan=False))
    return 0 if result["status"]=="PASS_WHOLE_SEED3_FIXED6000_NOT_PROMOTED" else 2


if __name__=="__main__":raise SystemExit(main())

