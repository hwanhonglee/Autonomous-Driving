"""HH_261005 - Prepare a separately authorized readonly whole fixed-five evaluation postcheck.

Preparation is not execution. The fixed ROOT review must seal the ended genuine
supervisor, its full raw streams and current authority before any SSH or payload
access. Future CPU metadata loaders reconstruct original tokens/geometry without
getitem, model construction/forward, loss, optimizer, CUDA, TEST or promotion.
Recorded final101/frozen47 training proofs and all42 endpoint bytes are rechecked;
this checker does not independently reload101 values or recompute metric averages.
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
import subprocess
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
STUDY = "temporal_route_joint_three_seed_final_evaluation_20261005_v1"
PLAN = ROOT / "storage/experiments/plans/2026-10-05" / STUDY
OUT = PLAN / "supervisor_v2"
EVAL_AUTHORITY = PLAN / "ROOT_final_evaluation_authority_v2.json"
EVAL_REVIEW = PLAN / "ROOT_final_evaluation_execution_review_v2.json"
REVIEW = PLAN / "ROOT_whole_final_evaluation_postcheck_authorization_v1.json"
TEST = ROOT / "learning/tests/test_postcheck_three_seed_final_evaluation_v1_20261005.py"
SCHEMA = "portable_e2e.three_seed_final_evaluation_whole_postcheck.20261005.v1"
SUPERVISOR = "tools/maintenance/supervise_three_seed_final_evaluation_v2_20261005.py"
SUPERVISOR_SHA = "aaa753aa2d7efcd34f50d52f94290af883b30149082efba1ffd08783022152fb"
SUPERVISOR_TEST = "learning/tests/test_supervise_three_seed_final_evaluation_v2_20261005.py"
SUPERVISOR_TEST_SHA = "5a557369f9b272941b563b89745761e2c21e34d4dad272333bab8e1c4978c1da"
RECOVERY_NAME = "tools/e2e/train_temporal_route_recovery_v3_20261001.py"
JOINT_NAME = "tools/e2e/train_temporal_route_joint_20261002.py"
RECOVERY_SHA = "6e1909f5574cf1449f56c4e0379507369f9a0f8841d05160a8e2cb861985452b"
JOINT_SHA = "971eb207d79fe6ed7e015da50a5351e9e30e13ce9631f2cf39267214379ae673"
ENDPOINTS = ("common3000", "GOOD6000", "seed_2026100501", "seed_2026100502", "seed_2026100503")
COUNTS = dict(recovery_train=2718, recovery_holdout=604, v4=327, legacy=2487,
    known599=599, intrinsic_legacy_TRAIN_only=328)
LATE_EPISODE_COHORTS = frozenset(("recovery_train", "recovery_holdout", "v4"))
MAX_LOCAL_BYTES, POST_SECONDS = 128 * 1024, 600


def require(value, message):
    if not value: raise RuntimeError(message)


def sha(raw): return hashlib.sha256(raw).hexdigest()


def encode(value): return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def canonical(path):
    path = Path(path).absolute()
    require(path == path.resolve(strict=True) and not any(p.is_symlink() for p in (path, *path.parents)),
        "canonical no-alias evidence required")
    return path


def digest(path):
    path = canonical(path); require(path.is_file(), "regular immutable evidence required")
    before = path.stat(); value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""): value.update(block)
    after = path.stat()
    require(all(getattr(before, k) == getattr(after, k) for k in
        ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")), "file generation changed during digest")
    return value.hexdigest()


def unique(pairs):
    value = {}
    for key, item in pairs:
        require(key not in value, "duplicate JSON key forbidden"); value[key] = item
    return value


def decode(raw):
    return json.loads(raw, object_pairs_hook=unique,
        parse_constant=lambda value: require(False, "nonfinite JSON constant forbidden"))


def read(path, limit=32 * 1024**2, expected=None):
    path = canonical(path); require(path.is_file() and path.stat().st_size <= limit, "bounded regular JSON required")
    pin = digest(path); raw = path.read_bytes()
    require(sha(raw) == pin and (expected is None or pin == expected), "externally sealed raw JSON changed")
    value = decode(raw); require(type(value) is dict and finite_tree(value), "finite JSON object required")
    return value


def finite_tree(value):
    if type(value) is dict:
        return all(type(k) is str and finite_tree(v) for k, v in value.items())
    if type(value) is list: return all(finite_tree(v) for v in value)
    if type(value) is float: return math.isfinite(value)
    return value is None or type(value) in (str, int, bool)


def identity(pid):
    # HH_261005 - Signal only this caller's exact direct SSH generation; never a group or a remote process.
    text = (Path("/proc") / str(pid) / "stat").read_text(); fields = text[text.rfind(")") + 2:].split()
    return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]), start_ticks=int(fields[19]))


def session_members(owned):
    rows = []
    for path in Path("/proc").iterdir():
        if not path.name.isdecimal(): continue
        try: row = identity(int(path.name))
        except (FileNotFoundError, ProcessLookupError): continue
        if row["pgid"] == owned["pgid"] and row["session"] == owned["session"]: rows.append(row)
    return rows


def cleanup_ssh(child, owned):
    result = dict(SSH_child_started=child is not None, signalled_direct_child_only=True)
    if child is None: return result
    try:
        if child.poll() is None:
            require(owned is not None and identity(child.pid) == owned, "owned SSH generation required before cleanup")
            child.terminate()
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                require(identity(child.pid) == owned, "SSH generation changed before final owned signal")
                child.kill(); child.wait(timeout=5)
        result["exit_code"] = child.poll()
        result["remaining_session"] = None if owned is None else session_members(owned)
        require(result["remaining_session"] == [], "SSH descendant/session closure UNKNOWN; no foreign signal")
    except BaseException as error: result["error"] = repr(error)
    return result


def planned_ledger(summary=None):
    # HH_261005 - UNKNOWN is not zero completed; a failing batch may have run without a retained scalar row.
    return dict(full_denominator=sum(COUNTS.values()) * len(ENDPOINTS),
        independently_verified_completed=None, status="UNKNOWN_UNTIL_WHOLE_RECONCILIATION",
        all30_planned_metric_files={e: {c: dict(expected=n, verified_completed=None) for c, n in COUNTS.items()} for e in ENDPOINTS},
        retained_original_reported_denominator=None if summary is None else summary.get("complete_denominator"))


def reconcile_records(records, metrics, expected):
    require(set(metrics) == set(ENDPOINTS) and set(expected) == set(COUNTS), "all five/six authentic cohorts required")
    require(all(len(expected[c]) == n for c, n in COUNTS.items()), "original full cohort token denominators required")
    require(all(type(i) is int and type(t) is str and type(e) is str for rows in expected.values() for i, t, e in rows),
        "authentic metadata index/token/episode scalar types required")
    iterator = iter(records); completed = {}
    for endpoint in ENDPOINTS:
        require(set(metrics[endpoint]) == set(COUNTS), "all30 original metric files required")
        completed[endpoint] = {}
        for cohort, count in COUNTS.items():
            rows = metrics[endpoint][cohort]["rows"]
            require(type(rows) is list and len(rows) == count, "one finalized metric row denominator differs")
            for ordinal, (index, token, episode) in enumerate(expected[cohort]):
                item = next(iterator, None)
                require(type(item) is dict and set(item) == {"endpoint", "cohort", "ordinal", "record"}
                    and item["endpoint"] == endpoint and item["cohort"] == cohort
                    and type(item["ordinal"]) is int and item["ordinal"] == ordinal,
                    "complete global fixed5/six journal order required; no missing/duplicate/retry")
                before, after = item["record"], rows[ordinal]
                require(type(before) is dict and type(after) is dict and finite_tree(before) and finite_tree(after)
                    and type(before.get("index")) is int and before["index"] == index
                    and type(after.get("index")) is int and after["index"] == index
                    and before.get("token") == after.get("token") == token, "authentic ordered metric token/index required")
                if cohort in LATE_EPISODE_COHORTS:
                    require("episode_id" not in before and after.get("episode_id") == episode
                        and set(after) == set(before) | {"episode_id"}
                        and before == {k: v for k, v in after.items() if k != "episode_id"},
                        "ONLY original late episode_id mutation may reconcile recovery metric rows")
                else:
                    require(before == after, "all other original journal/final row fields must match exactly")
                    if cohort in ("legacy", "known599"):
                        require(after.get("episode_id") == episode, "authentic original legacy episode required")
            completed[endpoint][cohort] = count
    require(next(iterator, None) is None, "extra journal rows forbidden")
    return dict(full_denominator=sum(COUNTS.values()) * len(ENDPOINTS),
        independently_verified_completed=sum(sum(v.values()) for v in completed.values()),
        verified_counts=completed, original3_late_episode_id_mutations_explicit=True,
        journal_finalized_metric_row_byte_equality_claimed=False)


def verify_denominator(summary, ledger, journal_bytes):
    recorded = summary["complete_denominator"]
    require(recorded["full_denominator"] == ledger["full_denominator"] == 35315
        and recorded["all_completed"] is True and recorded["failed_partial_rows_retained"] is True
        and recorded["last_unretained_actual_candidate"] is None
        and recorded["full_completed_scalar_journal_bytes"] == journal_bytes,
        "original complete journal/failed-tail denominator differs")
    exact = {e: {c: dict(expected=n, completed=n, UNEXECUTED=0) for c, n in COUNTS.items()} for e in ENDPOINTS}
    require(recorded["endpoints"] == exact and ledger["verified_counts"] == {e: dict(COUNTS) for e in ENDPOINTS},
        "all30 original fixed expected/completed/UNEXECUTED denominators required")


def verify_owned_receipts(first, started, caller, current):
    leader = started["leader"]; known = first["known_generations"]
    keys = ("pid", "ppid", "pgid", "session", "start_ticks")
    require(first["leader"] == leader and started["caller"] == caller["caller"]
        and all(type(leader[k]) is int and leader[k] > 0 for k in keys)
        and leader["pgid"] == leader["session"] == leader["pid"] and leader["ppid"] == caller["caller"]["pid"],
        "original exact owner/leader PID generations and new session required")
    require(type(known) is list and leader in known and len({r["pid"] for r in known}) == len(known)
        and all(all(type(r[k]) is int and r[k] > 0 for k in keys)
            and r["pgid"] == leader["pgid"] and r["session"] == leader["session"]
            and r["start_ticks"] >= leader["start_ticks"] for r in known), "all historical owned generations required")
    admitted = {leader["pid"]}; pending = [r for r in known if r != leader]
    while pending:
        children = [r for r in pending if r["ppid"] in admitted]
        require(children, "historical orphan/foreign ancestry cannot prove ownership")
        admitted.update(r["pid"] for r in children); pending = [r for r in pending if r not in children]
    require(first["remaining_session"] == {} and first["unknown_session_members"] == [] and current == [],
        "owned session must actually be absent; never signal foreign PID")


def verify_endpoint_documents(authority, immutable, reader=read):
    # HH_261005 - Recheck the actual9-document joins; a different authentic42-file seed is not this seed's endpoint.
    require(set(authority["completed_seeds"]) == {"2026100501", "2026100502", "2026100503"}, "all three fixed seed proofs required")
    for seed, entry in authority["completed_seeds"].items():
        documents = {}
        for name in ("whole_postcheck", "binding", "review"):
            path, pin = entry[name + "_path"], entry[name + "_sha256"]
            require(immutable.get(path) == pin, "one original ROOT document not sealed")
            documents[name] = reader(Path(path), expected=pin)
        post, binding, review = (documents[name] for name in ("whole_postcheck", "binding", "review"))
        require(type(post["seed"]) is int and post["seed"] == int(seed)
            and post["status"] == "PASS_WHOLE_SEED%d_FIXED6000_NOT_PROMOTED" % (int(seed)-2026100500)
            and post["checked_optimizer_updates"] == 6000 and post["checked_examples_seen"] == 24000
            and all(post["checks"].get(k) is True for k in ("all_6000_schedule_rows_exact", "all_12_checkpoint_triplets_exact",
                "all_101_final_and_frozen47_exact", "source_current_and_archive_grants_exact", "complete_raw_streams_exact",
                "owned_session_closed", "all_failed_tails_preserved")), "original complete fixed6000 training proofs required")
        require(type(binding.get("seed")) is int and binding["seed"] == int(seed)
            and binding.get("endpoint_root") == entry["endpoint_root"] and binding.get("whole_training_postcheck_sha256") == entry["whole_postcheck_sha256"]
            and type(review.get("seed")) is int and review["seed"] == int(seed) and review.get("endpoint_root") == entry["endpoint_root"]
            and review.get("whole_training_postcheck_sha256") == entry["whole_postcheck_sha256"]
            and review.get("endpoint_binding_sha256") == entry["binding_sha256"], "all7 original per-seed document joins required")
        require(type(binding["files"]) is dict and len(binding["files"]) == 42
            and all(type(name) is str and not Path(name).is_absolute() and str(Path(name)) == name and ".." not in Path(name).parts
                and immutable.get(str(Path(entry["endpoint_root"])/name)) == pin for name, pin in binding["files"].items()),
            "all42 original endpoint payload seals required for each fixed seed")


def pure_screens(source):
    # HH_261005 - Compile only the two exact pinned scalar functions; no trainer top level or Torch globals execute.
    functions = []
    for name, pin in ((RECOVERY_NAME, RECOVERY_SHA), (JOINT_NAME, JOINT_SHA)):
        path = canonical(Path(source) / name); require(digest(path) == pin, "genuine frozen screen source changed")
        tree = ast.parse(path.read_bytes())
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "endpoint_screens"]
        require(len(nodes) == 1 and not nodes[0].decorator_list, "one original pure screen function required")
        private = dict(math=math, require=require)
        if functions: private["previous"] = SimpleNamespace(endpoint_screens=functions[0])
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), private)
        functions.append(private["endpoint_screens"])
    return functions[-1]


def verify_screens(metrics, summary, function):
    common = {e: function(metrics["common3000"], metrics[e]) for e in ENDPOINTS[1:]}
    good = {e: function(metrics["GOOD6000"], metrics[e]) for e in ENDPOINTS[2:]}
    require(all(len(r["checks"]) == len({x["name"] for x in r["checks"]}) == 71 for r in (*common.values(), *good.values())),
        "all7x71 original screens required; no selected failures")
    require(common == summary["common3000_screens"] and good == summary["GOOD6000_reference_screens"],
        "genuine frozen screen values/order/thresholds/status differ")
    return dict(comparisons=7, screens_per_comparison=71, screen_count=497,
        failed_screen_count=sum(not c["passed"] for r in (*common.values(), *good.values()) for c in r["checks"]),
        genuine_screen_arithmetic_recomputed=True, metric_aggregate_arithmetic_independently_recomputed=False)


REMOTE = r'''
import importlib.util, pathlib, signal, stat, sys, tarfile
package=decode(sys.stdin.buffer.read()); workspace=Path('/home/pro6000/personal/hwanhong/portable_e2e')
study='temporal_route_joint_three_seed_final_evaluation_20261005_v1'; source=workspace/'source_worktrees'/study
plans=workspace/'plans/evaluation'/study; output=workspace/'runs/evaluation'/study; receipt=plans/'supervisor_v2'
checks={}; report=dict(status='FAIL_OR_UNKNOWN_RETAIN_ORIGINAL',complete_denominator=planned_ledger())
def deadline(signum,frame):raise TimeoutError('declared readonly CPU postcheck600s deadline; retain UNKNOWN')
signal.signal(signal.SIGALRM,deadline);signal.setitimer(signal.ITIMER_REAL,600)
def session_rows(leader):
    values=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal():continue
        try:s=(p/'stat').read_text();f=s[s.rfind(')')+2:].split()
        except (FileNotFoundError,ProcessLookupError):continue
        row=dict(pid=int(p.name),ppid=int(f[1]),pgid=int(f[2]),session=int(f[3]),start_ticks=int(f[19]))
        if row['pgid']==leader['pgid'] and row['session']==leader['session']:values.append(row)
    return values
def inventory(root):
    values={};total=0
    for parent,dirs,files in os.walk(canonical(root),followlinks=False):
        for name in dirs+files:
            p=Path(parent)/name;mode=p.lstat().st_mode
            require(stat.S_ISDIR(mode) or stat.S_ISREG(mode),'special/symlink output object denied')
            if stat.S_ISREG(mode):values[str(p.relative_to(root))]=dict(bytes=p.stat().st_size,sha256=digest(p));total+=p.stat().st_size
    return values,total
def all_seals(authority,protocol):
    for name,pin in protocol['source_sha256'].items():
        require(digest(source/name)==pin==digest(output/'sources'/name),'all35 current/output source bytes required')
    require(len(protocol['source_sha256'])==35,'exact35 project sources required')
    for path,pin in package['immutable_input_sha256'].items():require(digest(Path(path))==pin,'one original endpoint/ROOT input changed')
    require(digest(plans/'sealed_sources.tar')==protocol['archive_sha256']==package['source_archive_sha256'],'original source archive changed')
    with tarfile.open(canonical(plans/'sealed_sources.tar'),mode='r:') as tar:
        members=tar.getmembers();require([m.name for m in members]==sorted(protocol['source_sha256']),'exact35 ordered source archive members required')
        for m in members:
            require(m.isfile() and m.size<=1024**2 and sha(tar.extractfile(m).read())==protocol['source_sha256'][m.name],'genuine regular source archive member changed')
    require(digest(receipt/'result.json')==package['expected_remote_result_sha256']
        and digest(receipt/'evaluation.stdout')==package['expected_evaluation_stdout_sha256']
        and digest(receipt/'evaluation.stderr')==package['expected_evaluation_stderr_sha256'],'full ended raw/result changed')
    require(digest(plans/'ROOT_final_evaluation_authority.json')==package['evaluation_authority_sha256']
        and digest(plans/'ROOT_final_evaluation_execution_review.json')==package['evaluation_execution_review_sha256'],'ROOT evaluation grants changed')
try:
    require(os.environ.get('CUDA_VISIBLE_DEVICES')=='' and Path(sys.prefix).resolve()==workspace/'venvs/py312','existing CPU-only privatevenv required')
    first=read(receipt/'result.json',expected=package['expected_remote_result_sha256'])
    started=read(receipt/'child_started.json'); caller=read(receipt/'caller_started.json')
    authority=read(plans/'ROOT_final_evaluation_authority.json',expected=package['evaluation_authority_sha256'])
    execution=read(plans/'ROOT_final_evaluation_execution_review.json',expected=package['evaluation_execution_review_sha256'])
    protocol=caller['protocol'];require(protocol==execution['protocol'] and execution['immutable_input_sha256']==package['immutable_input_sha256']
        and len(package['immutable_input_sha256'])==145,'authentic original protocol/all145 input admission required')
    require(protocol['helper_sha256']==package['supervisor_source_sha256'] and protocol['test_sha256']==package['supervisor_test_sha256']
        and authority['contract']['source_sha256']==protocol['source_sha256'],'original complete source binding required')
    all_seals(authority,protocol)
    verify_endpoint_documents(authority,package['immutable_input_sha256'])
    actual={};total=0
    for root,prefix in ((plans,'plans/'),(output,'evaluation/')):
        table,size=inventory(root);actual.update({prefix+k:v for k,v in table.items()});total+=size
    original=first['artifacts'];require(actual==dict(original,**{'plans/supervisor_v2/result.json':dict(bytes=(receipt/'result.json').stat().st_size,sha256=package['expected_remote_result_sha256'])}),'whole original artifact tree differs')
    summary=read(output/'summary.json');report['complete_denominator']=planned_ledger(summary)
    checks['whole_raw_and_complete_artifact_tree_exact']=True
    verify_owned_receipts(first,started,caller,session_rows(started['leader']))
    require(started['argv']==[str(workspace/'venvs/py312/bin/python'),str(source/'tools/workspace_python.py'),'-m',
        'scripts.e2e.evaluate_temporal_route_joint_three_seed_final_v1_20261005','--','--authority',str(plans/'ROOT_final_evaluation_authority.json'),
        '--authority-sha256',package['evaluation_authority_sha256']],'exact genuine evaluation CLI required')
    require(started['environment_override']==dict(CUDA_VISIBLE_DEVICES='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',CUBLAS_WORKSPACE_CONFIG=':4096:8',
        PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1'),'original actual GPU0 environment required')
    checks['owned_session_closed']=True
    require(first['status']=='COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED' and first['exit_code']==0 and first['interruption'] is None
        and first['complete_raw_stdout_stderr_retained'] is True and first['automatic_retry'] is False
        and first['current_archive_sources_ROOT_evaluation_inputs_unchanged'] is True and first['post_current_archive_ROOT_evaluation_seals_exact'] is True,'original complete supervisor guards required')
    require(summary['status']=='ALL_FIXED_FINAL_OFFLINE_EVALUATION_COMPLETE_NOT_PROMOTED'
        and summary['all_source_endpoint_data_pins_before_after_exact'] is True and summary['complete_denominator']['all_completed'] is True
        and summary['complete_denominator']['full_denominator']==35315 and summary['model_promoted'] is False
        and summary['formal_main_approved']==summary['formal_sub_approved']==0,'genuine full-five evaluation completion required')
    # HH_261005 - Original CPU metadata/geometry loaders run ONLY after complete actual evidence; no sample image decoding or NN.
    spec=importlib.util.spec_from_file_location('hh_post_eval_bootstrap',source/'tools/workspace_python.py')
    bootstrap=importlib.util.module_from_spec(spec);spec.loader.exec_module(bootstrap);bootstrap.configure(source)
    from scripts.e2e import train_temporal_intrinsic_curve_pair_20261002 as pair
    require(not pair.torch.cuda.is_initialized(),'CPU postcheck must not initialize CUDA')
    pair.torch.set_num_threads(4)
    data=SimpleNamespace(**{name:Path(path) for name,path in authority['dataset_paths'].items()})
    training,audits,low,legacy,strata,train_receipts=pair.load_train(data)
    holdout,ho_audits,ho_receipt,known,known_receipt=pair.load_evaluation_only(data,training,low,legacy)
    datasets=dict(recovery_train=training,recovery_holdout=holdout,v4=low,legacy=legacy,known599=known)
    indices={name:list(range(len(value))) for name,value in datasets.items()};indices['legacy']=strata['eligible']
    indices['known599']=pair.replay.legacy_strata(known)['eligible']
    expected={c:[(i,datasets[c].examples[i].token,datasets[c].examples[i].episode_id) for i in values] for c,values in indices.items()}
    expected['intrinsic_legacy_TRAIN_only']=[(i,legacy.examples[i].token,legacy.examples[i].episode_id) for i in pair.curved_indices(strata)]
    opened=read(output/'evaluation_opened_after_all_three_final.json')
    require(opened['TRAIN']==train_receipts and opened['HOLDOUT']==ho_receipt and opened['known_DEV']==known_receipt
        and opened['endpoint_order']==list(ENDPOINTS),'authentic metadata loaders and fixed endpoint order required')
    image_receipt=pair.verify_training_images(tuple(datasets.values()))
    require(image_receipt==opened['source_images_before']==summary['source_images_before_after'],'all source camera bytes and recorded before/after receipts differ')
    data_pins=pair.evaluation_artifact_pins(data,known_receipt,holdout,known)
    require(all(digest(Path(p))==pin for p,pin in data_pins.items()),'original evaluation metadata source changed')
    metrics={e:{c:read(output/(e+'_'+c+'.json')) for c in COUNTS} for e in ENDPOINTS}
    journal=canonical(output/'all_completed_scalar_records.jsonl');require(journal.stat().st_size<=256*1024**2,'original whole scalar journal bound required')
    with journal.open('rb') as stream:report['complete_denominator']=reconcile_records((decode(line) for line in stream),metrics,expected)
    verify_denominator(summary,report['complete_denominator'],journal.stat().st_size)
    report['screens']=verify_screens(metrics,summary,pure_screens(source))
    require(report['complete_denominator']['independently_verified_completed']==35315,'all35315 authentic ordered scalar records required')
    checks['all35315_tokens_indices_order_and30_metric_records_exact']=True
    checks['all7x71_genuine_screens_recomputed']=True
    out_table,out_bytes=inventory(output)
    require(out_bytes==summary['output_bytes_before_closing']+(output/'summary.json').stat().st_size<=512*1024**2
        and summary['output_bytes_before_closing']<=512*1024**2-16*1024**2,'original output512MiB/closing16MiB accounting differs')
    require(all((receipt/name).stat().st_size<=64*1024**2 for name in ('evaluation.stdout','evaluation.stderr')),'successful raw stream bounds exceeded')
    all_seals(authority,protocol)
    after={}
    for root,prefix in ((plans,'plans/'),(output,'evaluation/')):after.update({prefix+k:v for k,v in inventory(root)[0].items()})
    require(after==actual and all(digest(Path(p))==pin for p,pin in data_pins.items())
        and pair.verify_training_images(tuple(datasets.values()))==image_receipt
        and not pair.torch.cuda.is_initialized() and not session_rows(started['leader']),'whole before/after/source/data/session guards required')
    checks['all_sources_endpoint42_ROOT_documents_archives_data_and_tree_before_after_exact']=True
    report.update(status='PASS_WHOLE_FIXED_FIVE_EVALUATION_NOT_PROMOTED',checks=checks,whole_artifact_file_count=len(actual),
        evaluation_output_bytes=out_bytes,total_plans_and_evaluation_bytes=total,source_current_archive_count=35,
        actual_source_images=image_receipt,final101_and_frozen47_prior_whole_post_proofs_and_endpoint_bytes_verified=True,
        all101_numeric_values_independently_reloaded_by_this_postcheck=False,metric_aggregate_arithmetic_independently_recomputed=False,
        NN_forward_called=False,loss_called=False,optimizer_called=False,CUDA_initialized=False,TEST_accessed=False,
        postchecker_explicit_remote_files_written=False,global_filesystem_write_sandbox_claimed=False,
        model_promoted=False,formal_main_approved=0,formal_sub_approved=0)
except BaseException as error:
    report.update(status='FAIL_OR_UNKNOWN_RETAIN_ORIGINAL',error=repr(error),checks=checks,failed_attempt_retained=True,
        model_promoted=False,formal_main_approved=0,formal_sub_approved=0)
finally:signal.setitimer(signal.ITIMER_REAL,0)
print(json.dumps(report,sort_keys=True,allow_nan=False),flush=True)
'''


def remote_program():
    names = ("require", "sha", "encode", "canonical", "digest", "unique", "decode", "read", "finite_tree",
        "planned_ledger", "reconcile_records", "verify_denominator", "verify_owned_receipts", "verify_endpoint_documents", "pure_screens", "verify_screens")
    tree = ast.parse(Path(__file__).read_bytes())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    require(len(functions) == len(names), "one exact scalar/hash implementation required")
    values = dict(ENDPOINTS=ENDPOINTS, COUNTS=COUNTS, LATE_EPISODE_COHORTS=tuple(sorted(LATE_EPISODE_COHORTS)),
        RECOVERY_NAME=RECOVERY_NAME, RECOVERY_SHA=RECOVERY_SHA, JOINT_NAME=JOINT_NAME, JOINT_SHA=JOINT_SHA)
    return "import ast,hashlib,json,math,os\nfrom pathlib import Path\nfrom types import SimpleNamespace\n" + \
        "\n".join(name + "=" + repr(value) for name, value in values.items()) + "\n" + \
        "\n".join(ast.unparse(n) for n in functions) + "\n" + REMOTE


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--review-sha256", required=True); args = parser.parse_args(argv)
    review = read(REVIEW, 1024**2, args.review_sha256)
    require(review.get("schema") == SCHEMA + ".ROOT_execution_review" and review.get("status") == "APPROVED_READONLY_WHOLE_FINAL_EVALUATION_POSTCHECK"
        and review.get("source_sha256") == digest(Path(__file__)) and review.get("test_sha256") == digest(TEST)
        and review.get("read_only_CPU_authorized") is True and review.get("NN_forward_authorized") is False
        and review.get("optimizer_authorized") is False and review.get("CUDA_authorized") is False,
        "fresh external ROOT completed-evaluation CPU post-review required before any SSH")
    require(digest(ROOT / SUPERVISOR) == SUPERVISOR_SHA and digest(ROOT / SUPERVISOR_TEST) == SUPERVISOR_TEST_SHA,
        "reviewed genuine supervisor/test changed")
    local = read(OUT / "SSH.stdout", 128*1024, review["expected_local_supervisor_stdout_sha256"])
    require(local["status"] == "COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED"
        and local["remote_receipt"] == "/home/pro6000/personal/hwanhong/portable_e2e/plans/evaluation/" + STUDY + "/supervisor_v2",
        "actual ended complete evaluation/exact remote receipt prerequisite")
    authority = read(EVAL_AUTHORITY, expected=review["evaluation_authority_sha256"])
    execution = read(EVAL_REVIEW, expected=review["evaluation_execution_review_sha256"])
    require(len(authority["contract"]["source_sha256"]) == 35
        and execution["archive_sha256"] == review["source_archive_sha256"], "authentic original35-source/archive closure required")
    for name, pin in authority["contract"]["source_sha256"].items():
        require(not Path(name).is_absolute() and str(Path(name)) == name and ".." not in Path(name).parts
            and digest(ROOT / name) == pin, "one local original evaluation source changed")
    package = dict(review, immutable_input_sha256=execution["immutable_input_sha256"],
        supervisor_source_sha256=SUPERVISOR_SHA, supervisor_test_sha256=SUPERVISOR_TEST_SHA)
    program = remote_program(); require(review["remote_program_sha256"] == sha(program.encode()), "exact CPU post-program review required")
    require(shutil.disk_usage(canonical(PLAN)).free >= 64*1024**2+1024**2, "metadata-only64MiB floor+1MiB reserve required")
    result_path = OUT / "ROOT_whole_final_evaluation_postcheck_v1.json"
    names = ("whole_final_evaluation_postcheck_v1.SSH.stdout", "whole_final_evaluation_postcheck_v1.SSH.stderr",
        "whole_final_evaluation_postcheck_v1.caller_started.json", "whole_final_evaluation_postcheck_v1.failure.json",
        "whole_final_evaluation_postcheck_v1.SSH_child_started.json", "whole_final_evaluation_postcheck_v1.SSH_timeout.stdout",
        "whole_final_evaluation_postcheck_v1.SSH_timeout.stderr")
    require(not result_path.exists() and not result_path.is_symlink()
        and all(not (OUT/name).exists() and not (OUT/name).is_symlink() for name in names), "fresh postcheck receipts only; no retry/overwrite")
    used = 0
    def write(path, raw, *, closing=False):
        nonlocal used
        require(used+len(raw) <= MAX_LOCAL_BYTES-(0 if closing else 16*1024), "whole local metadata128KiB/failed-closing16KiB bound required")
        with path.open("xb") as stream: require(stream.write(raw) == len(raw), "complete create-only metadata write required")
        used += len(raw)
    write(OUT/names[2], encode(dict(UTC=datetime.now(timezone.utc).isoformat(),review_sha256=args.review_sha256,
        source_sha256=review["source_sha256"],caller=identity(os.getpid()),readonly_CPU_only=True))+b"\n")
    child = None; owned = None
    try:
        argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "pro6000-training",
            "env CUDA_VISIBLE_DEVICES= PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 " +
            "/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -B -c " + shlex.quote(program)]
        child = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        owned = identity(child.pid)
        require(owned["ppid"] == os.getpid() and owned["pgid"] == owned["session"] == child.pid, "owned direct SSH session required")
        write(OUT/names[4], encode(dict(child=owned,argv=argv,caller=identity(os.getpid())))+b"\n")
        stdout, stderr = child.communicate(encode(package), timeout=POST_SECONDS)
        write(OUT/names[0], stdout); write(OUT/names[1], stderr)
        require(child.returncode == 0, "CPU postcheck transport failed; preserve original")
        require(session_members(owned) == [], "SSH session closure UNKNOWN; never signal unowned survivors")
        report = decode(stdout); require(type(report) is dict, "one complete CPU metadata response required")
        require(digest(REVIEW) == args.review_sha256 and digest(Path(__file__)) == review["source_sha256"]
            and digest(TEST) == review["test_sha256"] and digest(EVAL_AUTHORITY) == review["evaluation_authority_sha256"]
            and digest(EVAL_REVIEW) == review["evaluation_execution_review_sha256"],
            "local exact post-review/source/original grants changed")
        report.update(ROOT_review_sha256=args.review_sha256, SSH_stdout_sha256=sha(stdout), SSH_stderr_sha256=sha(stderr),
            source_sha256=review["source_sha256"], test_sha256=review["test_sha256"],SSH_session_closed=True)
        write(result_path, encode(report)+b"\n", closing=True)
        print(json.dumps(dict(status=report["status"],output=str(result_path)),allow_nan=False))
        return 0 if report["status"] == "PASS_WHOLE_FIXED_FIVE_EVALUATION_NOT_PROMOTED" else 2
    except BaseException as error:
        cleanup = cleanup_ssh(child, owned)
        retained = {}
        if isinstance(error, subprocess.TimeoutExpired):
            for name, raw in ((names[5], error.stdout), (names[6], error.stderr)):
                if raw is None: continue
                try: write(OUT/name, raw); retained[name] = dict(bytes=len(raw),sha256=sha(raw))
                except BaseException as retention_error: retained[name] = dict(bytes_received=len(raw),error=repr(retention_error),complete_retention=False)
        write(OUT/names[3], encode(dict(error=repr(error),SSH_cleanup=cleanup,partial_timeout_evidence=retained,
            complete_denominator=planned_ledger(),failed_attempt_retained=True,automatic_retry=False,
            remote_outcome="UNKNOWN on transport loss; no remote signal or retry"))+b"\n", closing=True)
        raise


if __name__ == "__main__": raise SystemExit(main())
