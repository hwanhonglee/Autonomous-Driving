"""HH_261005 - Prepare fresh V2 grant identities without changing fixed-final evaluation.

This source-only protocol cannot authorize itself. It requires all three actual
whole-training postchecks, a new ROOT evaluation authority and a caller-sealed
source/test/input execution review. No optimizer, TEST, simulator or promotion.
The reviewed seed3 generation/ancestry and full-pipe lifecycle is reused, with
fresh source/archive/receipt paths and a 7260s external evaluation deadline.
"""
from __future__ import annotations

import argparse
import ast
import base64
from datetime import datetime, timezone
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[2]
STUDY = "temporal_route_joint_three_seed_final_evaluation_20261005_v1"
PLAN = ROOT / "storage/experiments/plans/2026-10-05" / STUDY
OUT = PLAN / "supervisor_v2"
EVALUATOR = "tools/e2e/evaluate_temporal_route_joint_three_seed_final_v1_20261005.py"
TEST = ROOT / "learning/tests/test_supervise_three_seed_final_evaluation_v2_20261005.py"
AUTHORITY = PLAN / "ROOT_final_evaluation_authority_v2.json"
REVIEW = PLAN / "ROOT_final_evaluation_execution_review_v2.json"
PREVIOUS = ROOT / "tools/maintenance/supervise_three_seed_joint6000_seed3_v2_20261005.py"
PREVIOUS_SHA = "106a56e0761d82b7c41cc305fa74763150014974aee613e3218917a9bd8a30af"
EVALUATOR_SHA = "ed9428271a40b9b5ace147325fc9338972bd56902110b9c197900047aa93aba5"
EVALUATOR_TEST_SHA = "92c31a66cfc04560bb44c7107184a388ca900764722f0478d5b8263fd66712f1"
PERSONAL = Path("/home/pro6000/personal/hwanhong")
WORKSPACE = PERSONAL / "portable_e2e"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
LOCAL_LIMIT = 128 * 1024
MAX_PACKAGE = 8 * 1024**2


def require(value, message):
    if not value: raise RuntimeError(message)


def sha(raw): return hashlib.sha256(raw).hexdigest()


def encode(value): return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def canonical(path):
    path = Path(path).absolute()
    require(path == path.resolve(strict=True) and not any(p.is_symlink() for p in (path, *path.parents)),
        "canonical non-symlink input required")
    return path


def sealed_document(path, expected):
    raw = canonical(path).read_bytes()
    require(len(raw) <= 1024**2 and sha(raw) == expected, "exact bounded externally sealed document required")
    def unique(pairs):
        value = {}
        for key, item in pairs:
            require(key not in value, "duplicate JSON key forbidden"); value[key] = item
        return value
    return raw, json.loads(raw, object_pairs_hook=unique,
        parse_constant=lambda value: require(False, "nonfinite JSON forbidden"))


def relative_name(name):
    require(type(name) is str and name and not Path(name).is_absolute()
        and str(Path(name)) == name and all(p not in ("", ".", "..") for p in name.split("/")),
        "canonical relative source name required")
    return name


def remote_input_path(value):
    require(type(value) is str and Path(value).is_absolute() and str(Path(value)) == value
        and ".." not in Path(value).parts and Path(value).is_relative_to(PERSONAL),
        "exact private remote input boundary required")
    return value


def reviewed_previous():
    require(sha(canonical(PREVIOUS).read_bytes()) == PREVIOUS_SHA, "frozen genuine ownership lifecycle changed")
    spec = importlib.util.spec_from_file_location("_hh_frozen_evaluation_lifecycle", PREVIOUS)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def lifecycle_parts(previous):
    # HH_261005 - Reuse genuine source-pinned lifecycle AST, not a second loosely equivalent process controller.
    tree = ast.parse(previous.REMOTE_BODY)
    names = ("supervisor_signal", "parent_death_guard", "group", "observe", "cleanup")
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    require([n.name for n in functions] == list(names), "five exact frozen remote lifecycle functions required")
    signal_loop = [n for n in tree.body if isinstance(n, ast.For) and isinstance(n.target, ast.Name) and n.target.id == "value"]
    require(len(signal_loop) == 1, "one exact frozen signal setup required")
    outer = next(n for n in tree.body if isinstance(n, ast.Try))
    # HH_261005 - Select the genuine Popen/raw-stream block, not the earlier ROOT-document write blocks.
    owners = [n for n in outer.body if isinstance(n, ast.With) and any(isinstance(row, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "child" for target in row.targets)
        and isinstance(row.value, ast.Call) and isinstance(row.value.func, ast.Attribute)
        and isinstance(row.value.func.value, ast.Name) and row.value.func.value.id == "subprocess"
        and row.value.func.attr == "Popen" for row in n.body)]
    require(len(owners) == 1, "one genuine subprocess/raw-stream block required")
    owned = owners[0]
    require(len(owned.items) == 2 and "fit.stdout" in ast.unparse(owned.items[0].context_expr)
        and "fit.stderr" in ast.unparse(owned.items[1].context_expr), "genuine two full raw streams required")
    begin = next(i for i, n in enumerate(owned.body) if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "selector" for t in n.targets))
    loop = ast.Module(body=owned.body[begin:], type_ignores=[])
    count = 0
    class Deadline(ast.NodeTransformer):
        def visit_Constant(self, node):
            nonlocal count
            if type(node.value) is int and node.value == 7200:
                count += 1; return ast.copy_location(ast.Constant(7260), node)
            if node.value == "prospective caller7200s deadline":
                return ast.copy_location(ast.Constant("prospective caller7260s evaluation deadline"), node)
            if node.value == "current_archive_sources_CPU_GPU_grants_unchanged":
                return ast.copy_location(ast.Constant("current_archive_sources_ROOT_evaluation_inputs_unchanged"), node)
            return node
    loop = Deadline().visit(loop); require(count == 1, "one prospectively declared external deadline seam required")
    local = ast.parse(canonical(PREVIOUS).read_bytes())
    pure = [n for n in local.body if isinstance(n, ast.FunctionDef) and n.name in ("same_generation", "admit_family")]
    require(len(pure) == 2, "same frozen generation/ancestry functions required")
    return (previous.REMOTE_PREFIX + "\n" + "\n".join(ast.unparse(n) for n in pure),
        "\n".join(ast.unparse(n) for n in functions + signal_loop), ast.unparse(loop))


def source_archive(pins):
    require(type(pins) is dict and len(pins) == 35, "exact evaluator35 source closure required")
    rows, output = [], io.BytesIO()
    with tarfile.open(fileobj=output, mode="w", format=tarfile.USTAR_FORMAT) as archive:
        for name, pin in sorted(pins.items()):
            relative_name(name); raw = canonical(ROOT / name).read_bytes()
            require(sha(raw) == pin, "one declared source changed before fresh transfer")
            row = dict(path=name, bytes=len(raw), sha256=pin); rows.append(row)
            info = tarfile.TarInfo(name); info.size = len(raw); info.mode = 0o600
            info.uid = info.gid = info.mtime = 0
            archive.addfile(info, io.BytesIO(raw))
    raw = output.getvalue(); require(len(raw) <= 4 * 1024**2, "bounded code-only archive required")
    return raw, rows


def input_membership(authority, documents):
    # HH_261005 - Endpoint payloads stay remote; only externally sealed scalar ROOT documents are transferred.
    mapping = {}
    def add(path, pin):
        remote_input_path(path)
        require(type(pin) is str and len(pin) == 64 and all(c in "0123456789abcdef" for c in pin), "SHA input seal required")
        require(path not in mapping or mapping[path] == pin, "conflicting immutable input seal")
        mapping[path] = pin
    for entry in authority["baseline_assets"].values():
        for path, pin in entry["immutable_input_sha256"].items(): add(path, pin)
    require(set(authority["completed_seeds"]) == {"2026100501", "2026100502", "2026100503"}, "all three completed seeds required")
    require(len(documents) == 9 and len({row["remote_path"] for row in documents}) == 9,
        "exact three postcheck/binding/review documents for each seed required")
    require(all(sha(base64.b64decode(row["raw_base64"])) == row["sha256"] for row in documents),
        "every actual transferred scalar document byte seal required")
    by_path = {row["remote_path"]: row for row in documents}
    for seed, entry in authority["completed_seeds"].items():
        for path_key, sha_key in (("whole_postcheck_path", "whole_postcheck_sha256"),
                ("binding_path", "binding_sha256"), ("review_path", "review_sha256")):
            path, pin = entry[path_key], entry[sha_key]; add(path, pin)
            require(path in by_path and by_path[path]["sha256"] == pin, "every ROOT endpoint document exact transfer required")
        binding = json.loads(base64.b64decode(by_path[entry["binding_path"]]["raw_base64"]))
        post = json.loads(base64.b64decode(by_path[entry["whole_postcheck_path"]]["raw_base64"]))
        review = json.loads(base64.b64decode(by_path[entry["review_path"]]["raw_base64"]))
        require(type(post["seed"]) is int and post["seed"] == int(seed) and post["status"] == "PASS_WHOLE_SEED%d_FIXED6000_NOT_PROMOTED" % (int(seed)-2026100500)
            and post["checked_optimizer_updates"] == 6000 and post["checked_examples_seen"] == 24000
            and all(post["checks"].get(key) is True for key in ("all_6000_schedule_rows_exact", "all_12_checkpoint_triplets_exact",
                "all_101_final_and_frozen47_exact", "source_current_and_archive_grants_exact", "complete_raw_streams_exact",
                "owned_session_closed", "all_failed_tails_preserved")), "authentic whole-training PASS prerequisite")
        # HH_261005 - Join each authentic seed/root/post and review to the exact binding, never just a42-file shape.
        require(type(binding.get("seed")) is int and binding["seed"] == int(seed)
            and binding.get("endpoint_root") == entry["endpoint_root"]
            and binding.get("whole_training_postcheck_sha256") == entry["whole_postcheck_sha256"]
            and type(review.get("seed")) is int and review["seed"] == int(seed)
            and review.get("endpoint_root") == entry["endpoint_root"]
            and review.get("whole_training_postcheck_sha256") == entry["whole_postcheck_sha256"]
            and review.get("endpoint_binding_sha256") == entry["binding_sha256"],
            "actual per-seed endpoint/binding/review/whole-post join required")
        require(type(binding.get("files")) is dict and len(binding["files"]) == 42, "complete reviewed numeric42-file endpoint closure required")
        for name, pin in binding["files"].items(): add(str(Path(entry["endpoint_root"]) / relative_name(name)), pin)
    require(set(by_path) == {entry[key] for entry in authority["completed_seeds"].values()
        for key in ("whole_postcheck_path", "binding_path", "review_path")}, "no unrelated ROOT document transfer")
    return mapping


REMOTE_BODY = r'''
package=json.loads(sys.stdin.buffer.read());workspace=pathlib.Path('/home/pro6000/personal/hwanhong/portable_e2e')
study='temporal_route_joint_three_seed_final_evaluation_20261005_v1'
source=workspace/'source_worktrees'/study;plans=workspace/'plans/evaluation'/study
output=workspace/'runs/evaluation'/study;receipt=plans/'supervisor_v2'
authority_path=plans/'ROOT_final_evaluation_authority.json';review_path=plans/'ROOT_final_evaluation_execution_review.json'
created=False;child=None;leader=None;known={};unknown=[];result={};status='FAILED_NO_RETRY';os.umask(0o077)
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024**2),b''):h.update(b)
    return h.hexdigest()
def verify_inputs():
    for path,pin in package['immutable_inputs'].items():
        p=canonical(pathlib.Path(path));require(p.is_file() and digest(p)==pin,'one ROOT-reviewed endpoint/input changed')
def seals():
    for row in package['entries']:
        p=canonical(source/row['path']);require(p.is_file() and p.stat().st_size==row['bytes'] and digest(p)==row['sha256'],'one current evaluation source changed')
    require(digest(canonical(plans/'sealed_sources.tar'))==package['archive_sha256']
        and digest(canonical(authority_path))==package['authority_sha256']
        and digest(canonical(review_path))==package['review_sha256'],'evaluation source/archive/ROOT grants changed')
    verify_inputs()
try:
    canonical(workspace);require(shutil.disk_usage(workspace).free>=5*1024**3+512*1024**2,'original remote5GiB+512MiB reserve required')
    for p in (source,plans,output):require(not p.exists() and not p.is_symlink(),'new evaluation paths only; no resume/overwrite')
    authority=base64.b64decode(package['authority']);review=base64.b64decode(package['review']);archive=base64.b64decode(package['archive'])
    require(sha(authority)==package['authority_sha256'] and sha(review)==package['review_sha256']
        and sha(archive)==package['archive_sha256'],'externally sealed exact transfer required')
    doc=json.loads(authority);r=json.loads(review)
    require(r['status']=='APPROVED_EXACT_FIXED_FINAL_GPU0_EVALUATION' and r['protocol']==package['protocol']
        and r['authority_sha256']==package['authority_sha256'] and r['archive_sha256']==package['archive_sha256']
        and r['immutable_input_sha256']==package['immutable_inputs'],'ROOT exact prospective protocol/input review required')
    require(doc['all_three_whole_training_postchecks_PASS'] is True and doc['contract']['source_sha256']=={row['path']:row['sha256'] for row in package['entries']}
        and doc['purpose']=='fixed_final_known_DEV_research_evaluation_only' and doc['authorized'] is True
        and doc['GPU_physical_index']==0 and doc['TEST_access_authorized'] is False,'evaluation-only grant; no optimizer/TEST')
    for parent in (source.parent,plans.parent,output.parent):
        require(parent.parent==canonical(parent.parent),'canonical parent required')
        parent.mkdir(exist_ok=True);canonical(parent)
    source.mkdir();plans.mkdir();receipt.mkdir();created=True
    new(receipt/'caller_started.json',dict(UTC=datetime.now(timezone.utc).isoformat(),caller=identity(os.getpid()),protocol=package['protocol']))
    with (plans/'sealed_sources.tar').open('xb') as f:f.write(archive)
    import io,tarfile
    with tarfile.open(fileobj=io.BytesIO(archive),mode='r:') as tar:
        members=tar.getmembers();require(len(members)==35 and [m.name for m in members]==[row['path'] for row in package['entries']], 'exact35 ordered regular source members required')
        for member,row in zip(members,package['entries']):
            require(member.isfile() and member.size==row['bytes'] and not pathlib.Path(member.name).is_absolute()
                and '..' not in pathlib.Path(member.name).parts,'regular canonical source only')
            raw=tar.extractfile(member).read();require(sha(raw)==row['sha256'],'archive member seal required')
            p=source/member.name;p.parent.mkdir(parents=True,exist_ok=True)
            with p.open('xb') as f:f.write(raw)
    for name,raw in (('ROOT_final_evaluation_authority.json',authority),('ROOT_final_evaluation_execution_review.json',review)):
        with (plans/name).open('xb') as f:f.write(raw)
    for row in package['documents']:
        p=pathlib.Path(row['remote_path']);require(p.is_relative_to(plans/'input_documents'),'new exact scalar-document boundary required')
        raw=base64.b64decode(row['raw_base64']);require(sha(raw)==row['sha256'] and len(raw)<=1024**2,'scalar document seal/quota required')
        p.parent.mkdir(parents=True,exist_ok=True)
        with p.open('xb') as f:f.write(raw)
    seals();before_free=shutil.disk_usage(workspace).free
    argv=[str(workspace/'venvs/py312/bin/python'),str(source/'tools/workspace_python.py'),'-m',
        'scripts.e2e.evaluate_temporal_route_joint_three_seed_final_v1_20261005','--','--authority',str(authority_path),
        '--authority-sha256',package['authority_sha256']]
    env=dict(os.environ,CUDA_VISIBLE_DEVICES='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5',CUBLAS_WORKSPACE_CONFIG=':4096:8',
        PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1',PYTHONUNBUFFERED='1')
    with (receipt/'evaluation.stdout').open('xb') as out,(receipt/'evaluation.stderr').open('xb') as err:
        parent_pid=os.getpid()
        child=subprocess.Popen(argv,cwd=str(source),env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True,
            preexec_fn=lambda:parent_death_guard(parent_pid))
        leader=identity(child.pid);require(leader['ppid']==os.getpid() and leader['pgid']==leader['session']==child.pid,'owned evaluation leader required')
        known[leader['pid']]=leader
        new(receipt/'child_started.json',dict(UTC=datetime.now(timezone.utc).isoformat(),caller=identity(os.getpid()),leader=leader,
            argv=argv,environment_override={k:env[k] for k in ('CUDA_VISIBLE_DEVICES','CUBLAS_WORKSPACE_CONFIG','PYTHONDONTWRITEBYTECODE',
                'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','PYTHONUNBUFFERED')},free_before_bytes=before_free,
            leader_parent_death_signal='SIGTERM',runner_original_GPU0_lease_and_fresh_preflight=True,caller_deadline_seconds=7260,
            unhandled_supervisor_loss='descendant closure UNKNOWN; ROOT must independently inspect generations'))
'''

REMOTE_TAIL = r'''
    require(not result['remaining_session'] and not result['unknown_session_members'],'evaluation session closure unproven; no foreign signal')
    require(exit_code==0 and interruption is None,'one fixed-final evaluation failed/interrupted; no retry')
    summary=json.loads(canonical(output/'summary.json').read_bytes())
    require(summary['status']=='ALL_FIXED_FINAL_OFFLINE_EVALUATION_COMPLETE_NOT_PROMOTED'
        and summary['complete_denominator']['full_denominator']==35315 and summary['complete_denominator']['all_completed'] is True
        and summary['all_source_endpoint_data_pins_before_after_exact'] is True and summary['model_promoted'] is False
        and summary['formal_main_approved']==0 and summary['formal_sub_approved']==0,'whole fixed-five evaluation completion required; no promotion')
    status='COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED'
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
        try:seals();result['post_current_archive_ROOT_evaluation_seals_exact']=True
        except BaseException as seal_error:status='FAILED_NO_RETRY';result['post_source_seal_error']=repr(seal_error)
        try:
            for parent,prefix in ((plans,'plans/'),(output,'evaluation/')):
                if not parent.exists():continue
                for p in sorted(parent.rglob('*')):
                    require(not p.is_symlink(),'unexpected output alias; never delete')
                    if p.is_file():artifacts[prefix+str(p.relative_to(parent))]=dict(bytes=p.stat().st_size,sha256=digest(p))
        except BaseException as audit_error:status='FAILED_NO_RETRY';result['partial_artifact_inventory_error']=repr(audit_error)
        result.update(status=status,UTC=datetime.now(timezone.utc).isoformat(),automatic_retry=False,automatic_additional_job=False,
            failed_attempt_retained=status!='COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED',artifacts=artifacts,
            formal_main_approved=0,formal_sub_approved=0,optimizer_authorized=False,TEST_access_authorized=False)
        new(receipt/'result.json',result)
    response=dict(status=status,result=result,remote_receipt=str(receipt));encoded=encode(response)
    if len(encoded)>96*1024:
        failure=dict(error='metadata response exceeds96KiB; full result retained unchanged',full_result_path=str(receipt/'result.json'),
            full_result_sha256=digest(receipt/'result.json'),complete_response_bytes=len(encoded),automatic_retry=False)
        new(receipt/'metadata_bound_failure.json',failure)
        encoded=encode(dict(status='FAILED_METADATA_BOUND_FULL_REMOTE_RESULT_RETAINED',result=failure,remote_receipt=str(receipt)))
    print(encoded.decode(),flush=True)
'''


def remote_program(previous=None):
    previous = previous or reviewed_previous()
    prefix, functions, loop = lifecycle_parts(previous)
    head, body = REMOTE_BODY.split("try:\n", 1)
    return prefix + "\n" + head + "\n" + functions + "\ntry:\n" + body + "\n" + \
        "\n".join("        " + line for line in loop.splitlines()) + "\n" + REMOTE_TAIL


def validate_review(review, authority, protocol, archive_sha, inputs):
    require(review.get("status") == "APPROVED_EXACT_FIXED_FINAL_GPU0_EVALUATION"
        and review.get("protocol") == protocol and review.get("archive_sha256") == archive_sha
        and review.get("immutable_input_sha256") == inputs and review.get("all_three_whole_training_postchecks_PASS") is True
        and review.get("automatic_retry") is False and review.get("automatic_additional_job") is False
        and authority["contract"]["complete_record_denominator"] == 35315,
        "exact externally sealed final-evaluation execution review required")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--review-sha256", required=True); args = parser.parse_args(argv)
    require(not OUT.exists() and not OUT.is_symlink(), "fresh local supervisor receipt path required")
    require(shutil.disk_usage(canonical(PLAN)).free >= 64*1024**2 + 1024**2,
        "metadata-only64MiB floor+1MiB closing reserve; no CARLA guard change")
    review_raw, review = sealed_document(REVIEW, args.review_sha256)
    authority_raw, authority = sealed_document(AUTHORITY, review["authority_sha256"])
    require(sha(canonical(ROOT/EVALUATOR).read_bytes()) == EVALUATOR_SHA
        and sha(canonical(ROOT/"learning/tests/test_evaluate_temporal_route_joint_three_seed_final_v1_20261005.py").read_bytes()) == EVALUATOR_TEST_SHA,
        "frozen evaluator and scalar regression changed")
    spec = importlib.util.spec_from_file_location("_hh_sealed_evaluator_contract", ROOT/EVALUATOR)
    evaluator = importlib.util.module_from_spec(spec); spec.loader.exec_module(evaluator)
    proposed = evaluator.fixed_contract(authority["training_contract"]["sources"]); evaluator.validate_authority(authority, proposed)
    archive, entries = source_archive(proposed["source_sha256"])
    documents = []
    for row in review["transfer_documents"]:
        raw, value = sealed_document(row["local_path"], row["sha256"])
        remote_input_path(row["remote_path"])
        require(Path(row["remote_path"]).is_relative_to(WORKSPACE/"plans/evaluation"/STUDY/"input_documents"),
            "fresh evaluation scalar-document transfer only")
        documents.append(dict(remote_path=row["remote_path"], sha256=row["sha256"], raw_base64=base64.b64encode(raw).decode()))
    inputs = input_membership(authority, documents); previous = reviewed_previous(); program = remote_program(previous)
    compile(program, "sealed_fixed_final_evaluation_supervisor", "exec")
    protocol = dict(provenance="HH_261005 - One fixed-final evaluation; preserve all rows and failures without optimization or promotion.",
        helper_sha256=sha(Path(__file__).read_bytes()), test_sha256=sha(canonical(TEST).read_bytes()),
        frozen_lifecycle_source_sha256=PREVIOUS_SHA, remote_program_sha256=sha(program.encode()),
        authority_sha256=sha(authority_raw), source_count=35, source_sha256=proposed["source_sha256"],
        archive_sha256=sha(archive), original_remote_floor_bytes=5*1024**3, original_remote_write_reserve_bytes=512*1024**2,
        local_metadata_only_floor_bytes=64*1024**2, local_metadata_closing_reserve_bytes=1024**2, local_metadata_limit_bytes=LOCAL_LIMIT,
        evaluation_alarm_seconds=7200, external_caller_deadline_seconds=7260, SSH_deadline_seconds=7320,
        full_raw_logs_remote=True, raw_log_bound_per_stream_bytes=64*1024**2, truncate_raw_logs=False,
        physical_GPU_UUID=GPU_UUID, original_GPU0_lease="runs/campaigns/.gpu0_training.lock", private_venv=str(WORKSPACE/"venvs/py312"),
        environment_threads=dict(OMP_NUM_THREADS="4",MKL_NUM_THREADS="4",OPENBLAS_NUM_THREADS="1",PYTHONUNBUFFERED="1"),
        maximum_code_archive_bytes=4*1024**2, maximum_scalar_document_bytes=1024**2, maximum_package_bytes=MAX_PACKAGE,
        owned_individual_generation_cleanup_only=True, transport_loss_descendant_closure="UNKNOWN pending independent ROOT audit",
        automatic_retry=False, automatic_additional_job=False, no_installation=True,
        optimization_authorized=False, TEST_access_authorized=False, simulator_control_authorized=False,
        model_promoted=False, formal_main_approved=0, formal_sub_approved=0)
    validate_review(review, authority, protocol, sha(archive), inputs)
    package = dict(protocol=protocol, authority=base64.b64encode(authority_raw).decode(), authority_sha256=sha(authority_raw),
        review=base64.b64encode(review_raw).decode(),review_sha256=args.review_sha256, archive=base64.b64encode(archive).decode(),
        archive_sha256=sha(archive),entries=entries,documents=documents,immutable_inputs=inputs)
    payload = encode(package); require(len(payload) <= MAX_PACKAGE, "bounded source/document-only transfer required")
    OUT.mkdir(); total = 0
    def write(name, raw, *, closing=False):
        nonlocal total
        require(total+len(raw) <= LOCAL_LIMIT-(0 if closing else 16*1024), "metadata128KiB includes failed closing reserve")
        with (OUT/name).open("xb") as stream: stream.write(raw)
        total += len(raw)
    write("protocol.json", encode(protocol)+b"\n")
    write("caller_started.json", encode(dict(UTC=datetime.now(timezone.utc).isoformat(),caller=previous.identity(os.getpid()),
        helper_sha256=protocol["helper_sha256"],no_automatic_retry=True))+b"\n")
    child = None; owned = None
    try:
        argv = ["ssh","-o","BatchMode=yes","-o","ConnectTimeout=10","pro6000-training","python3 -c "+shlex.quote(program)]
        child = subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
        owned = previous.identity(child.pid)
        require(owned["ppid"]==os.getpid() and owned["pgid"]==owned["session"]==child.pid,"owned SSH session required")
        write("SSH_child_started.json", encode(dict(child=owned,caller=previous.identity(os.getpid())))+b"\n")
        stdout,stderr = child.communicate(payload,timeout=7320)
        write("SSH.stdout",stdout);write("SSH.stderr",stderr)
        require(child.returncode==0,"remote evaluation supervisor failed; no retry")
        response=json.loads(stdout)
        require(response["remote_receipt"]==str(WORKSPACE/"plans/evaluation"/STUDY/"supervisor_v2"),"exact remote receipt boundary required")
        require(sha(canonical(AUTHORITY).read_bytes())==sha(authority_raw) and sha(canonical(REVIEW).read_bytes())==args.review_sha256
            and sha(canonical(PREVIOUS).read_bytes())==PREVIOUS_SHA,"local review/authority/lifecycle changed")
        write("exited.json",encode(dict(exit_code=child.returncode,status=response["status"],full_remote_metadata_sha256=sha(stdout),
            stderr_sha256=sha(stderr),remote_receipt=response["remote_receipt"],raw_output_stays_remote=True,automatic_retry=False))+b"\n")
        print(json.dumps(dict(status=response["status"],local_output=str(OUT),metadata_sha256=sha(stdout))))
        return 0 if response["status"]=="COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED" else 2
    except BaseException as error:
        cleanup=dict(SSH_child_started=child is not None)
        if child is not None:
            try:
                if child.poll() is None:
                    require(owned is not None and previous.identity(child.pid)==owned,"owned SSH generation required before cleanup")
                    child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        require(previous.identity(child.pid)==owned,"SSH generation changed before final owned signal")
                        child.kill();child.wait(timeout=5)
                cleanup["exit_code"]=child.poll()
            except BaseException as cleanup_error:cleanup["error"]=repr(cleanup_error)
        if isinstance(error,subprocess.TimeoutExpired):
            if error.stdout is not None:write("SSH_timeout.stdout",error.stdout)
            if error.stderr is not None:write("SSH_timeout.stderr",error.stderr)
        write("failure.json",encode(dict(error=repr(error),SSH_cleanup=cleanup,failed_attempt_retained=True,automatic_retry=False,
            remote_outcome="UNKNOWN on transport loss; full received remote evidence retained"))+b"\n",closing=True)
        raise


if __name__ == "__main__": raise SystemExit(main())
