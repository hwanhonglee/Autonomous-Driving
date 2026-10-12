"""HH_261005 - Test fresh V2 grant identities and unchanged source-only scalar supervision.

No remote command, process signal, dataset, endpoint numerical load, Torch,
GPU query, training or evaluation is executed by this synthetic suite.
"""
import ast
import base64
from copy import deepcopy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tarfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/maintenance/supervise_three_seed_final_evaluation_v2_20261005.py"
spec = importlib.util.spec_from_file_location("_hh_evaluation_supervisor_scalar", SOURCE)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


def literal_assignments(path):
    return {node.targets[0].id: ast.literal_eval(node.value) for node in ast.parse(path.read_bytes()).body
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in ("REMOTE_PREFIX", "REMOTE_BODY")}


def previous():
    return SimpleNamespace(**literal_assignments(module.PREVIOUS))


def normalized(value): return ast.dump(value, include_attributes=False)


def documents_fixture():
    baselines = {name: {"immutable_input_sha256": {str(module.PERSONAL/"fixtures"/name): "a"*64}}
        for name in ("common3000", "GOOD6000")}
    authority = dict(baseline_assets=baselines, completed_seeds={})
    documents = []
    for number, seed in enumerate((2026100501, 2026100502, 2026100503), 1):
        parent = module.WORKSPACE/"plans/evaluation"/module.STUDY/"input_documents"/str(seed)
        endpoint = module.WORKSPACE/"runs/training/fixtures"/str(seed)
        post = dict(seed=seed,status="PASS_WHOLE_SEED%d_FIXED6000_NOT_PROMOTED"%number,
            checked_optimizer_updates=6000,checked_examples_seen=24000,checks={key: True for key in (
                "all_6000_schedule_rows_exact","all_12_checkpoint_triplets_exact","all_101_final_and_frozen47_exact",
                "source_current_and_archive_grants_exact","complete_raw_streams_exact","owned_session_closed","all_failed_tails_preserved")})
        post_pin=module.sha(module.encode(post))
        binding = dict(seed=seed,endpoint_root=str(endpoint),whole_training_postcheck_sha256=post_pin,
            files={"payload_%02d.json"%i: module.sha(str(i).encode()) for i in range(42)})
        review=dict(seed=seed,endpoint_root=str(endpoint),whole_training_postcheck_sha256=post_pin,
            endpoint_binding_sha256=module.sha(module.encode(binding)),status="synthetic scalar ROOT document")
        values = {"post.json":post,"binding.json":binding,"review.json":review}
        entry = dict(endpoint_root=str(endpoint))
        for name,path_key,sha_key in (("post.json","whole_postcheck_path","whole_postcheck_sha256"),
                ("binding.json","binding_path","binding_sha256"),("review.json","review_path","review_sha256")):
            raw=module.encode(values[name]);pin=module.sha(raw);path=str(parent/name)
            entry[path_key]=path;entry[sha_key]=pin
            documents.append(dict(remote_path=path,sha256=pin,raw_base64=base64.b64encode(raw).decode()))
        authority["completed_seeds"][str(seed)]=entry
    return authority,documents


def replace_document(authority,documents,seed,key,value):
    entry=authority["completed_seeds"][str(seed)]
    path=entry[key]
    row=next(row for row in documents if row["remote_path"]==path)
    raw=module.encode(value);pin=module.sha(raw)
    row.update(sha256=pin,raw_base64=base64.b64encode(raw).decode())
    entry[{"whole_postcheck_path":"whole_postcheck_sha256","binding_path":"binding_sha256","review_path":"review_sha256"}[key]]=pin


def test_actual_previous_source_sha():
    assert module.sha(module.PREVIOUS.read_bytes())==module.PREVIOUS_SHA


def test_exact_five_original_lifecycle_and_signal_setup():
    p=previous();prefix,functions,loop=module.lifecycle_parts(p)
    old=ast.parse(p.REMOTE_BODY)
    expected=[node for node in old.body if isinstance(node,ast.FunctionDef) and node.name in
        ("supervisor_signal","parent_death_guard","group","observe","cleanup")]
    expected += [node for node in old.body if isinstance(node,ast.For) and isinstance(node.target,ast.Name) and node.target.id=="value"]
    assert normalized(ast.parse(functions))==normalized(ast.Module(body=expected,type_ignores=[]))
    parsed=ast.parse(prefix)
    assert {node.name for node in parsed.body if isinstance(node,ast.FunctionDef)} >= {"identity","same_generation","admit_family"}
    assert "killpg" not in functions


def test_selector_inverse_only_one_prospective_deadline_and_receipt_label():
    p=previous();_,_,loop=module.lifecycle_parts(p)
    old=ast.parse(p.REMOTE_BODY);outer=next(n for n in old.body if isinstance(n,ast.Try))
    owned=next(n for n in outer.body if isinstance(n,ast.With) and any(isinstance(row,ast.Assign)
        and isinstance(row.value,ast.Call) and isinstance(row.value.func,ast.Attribute)
        and isinstance(row.value.func.value,ast.Name) and row.value.func.value.id=="subprocess"
        and row.value.func.attr=="Popen" for row in n.body))
    begin=next(i for i,n in enumerate(owned.body) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=="selector" for t in n.targets))
    expected=ast.Module(body=owned.body[begin:],type_ignores=[])
    class Undo(ast.NodeTransformer):
        def visit_Constant(self,node):
            if type(node.value) is int and node.value==7260:node.value=7200
            if node.value=="prospective caller7260s evaluation deadline":node.value="prospective caller7200s deadline"
            if node.value=="current_archive_sources_ROOT_evaluation_inputs_unchanged":node.value="current_archive_sources_CPU_GPU_grants_unchanged"
            return node
    assert normalized(Undo().visit(ast.parse(loop)))==normalized(expected)
    assert "20" in loop and "64 * 1024 ** 2" in loop


def test_remote_full_program_compiles_without_execution_and_real_bindings():
    program=module.remote_program(previous());compile(program,"scalar_only_remote_program","exec")
    tree=ast.parse(program);names={n.name for n in tree.body if isinstance(n,ast.FunctionDef)}
    assert {"group","observe","cleanup","parent_death_guard","same_generation","admit_family"} <= names
    assert program.count("subprocess.Popen(")==1
    assert "scripts.e2e.evaluate_temporal_route_joint_three_seed_final_v1_20261005" in program
    assert "'--', '--authority'" in program or "'--','--authority'" in program
    assert "'COMPLETE_FIXED_FIVE_EVALUATION_NOT_PROMOTED'" in program
    assert "'complete_raw_stdout_stderr_retained'" in program and "'unreceived_pipe_tail'" in program
    assert "'remaining_session'" in program and "'unknown_session_members'" in program
    assert "'SIGTERM'" in program and "'fit-one'" not in program
    assert "gpu1" not in program.lower() and "killpg" not in program


@pytest.mark.parametrize("fault",["missing_cleanup","missing_signal","no_selector","extra_deadline","duplicate_owned","wrong_raw_stream"])
def test_modified_frozen_lifecycle_shape_rejected(fault):
    p=previous();tree=ast.parse(p.REMOTE_BODY)
    if fault=="missing_cleanup":tree.body=[n for n in tree.body if not(isinstance(n,ast.FunctionDef) and n.name=="cleanup")]
    elif fault=="missing_signal":tree.body=[n for n in tree.body if not(isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=="value")]
    else:
        outer=next(n for n in tree.body if isinstance(n,ast.Try))
        owned=next(n for n in outer.body if isinstance(n,ast.With) and any(isinstance(row,ast.Assign)
            and isinstance(row.value,ast.Call) and isinstance(row.value.func,ast.Attribute)
            and isinstance(row.value.func.value,ast.Name) and row.value.func.value.id=="subprocess"
            and row.value.func.attr=="Popen" for row in n.body))
        if fault=="no_selector":owned.body=[n for n in owned.body if not(isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="selector" for t in n.targets))]
        elif fault=="extra_deadline":owned.body.append(ast.Assign(targets=[ast.Name(id="unexpected",ctx=ast.Store())],value=ast.Constant(7200)))
        elif fault=="duplicate_owned":outer.body.append(deepcopy(owned))
        else:
            for node in ast.walk(owned.items[0].context_expr):
                if isinstance(node,ast.Constant) and node.value=="fit.stdout":node.value="not_actual_raw.stdout"
    p.REMOTE_BODY=ast.unparse(ast.fix_missing_locations(tree))
    with pytest.raises((RuntimeError,StopIteration)):module.lifecycle_parts(p)


def test_exact_input_membership_all_fixed_five_endpoints():
    authority,documents=documents_fixture();inputs=module.input_membership(authority,documents)
    assert len(inputs)==2+9+3*42
    assert all(Path(path).is_relative_to(module.PERSONAL) for path in inputs)


@pytest.mark.parametrize("fault",["binding_seed","binding_seed_bool","binding_root","binding_post_pin",
    "review_seed","review_seed_bool","review_root","review_post_pin","review_binding_pin","swapped_authentic_seed2_metadata"])
def test_actual_per_seed_metadata_joins_not_just42_files(fault):
    authority,documents=documents_fixture();entry=authority["completed_seeds"]["2026100501"]
    binding=json.loads(base64.b64decode(next(row for row in documents if row["remote_path"]==entry["binding_path"])["raw_base64"]))
    review=json.loads(base64.b64decode(next(row for row in documents if row["remote_path"]==entry["review_path"])["raw_base64"]))
    if fault=="swapped_authentic_seed2_metadata":
        # HH_261005 - Simulate valid seed2 metadata/payload seals mislabeled as seed1; the authentic post1 stays seed1.
        other=authority["completed_seeds"]["2026100502"]
        binding=json.loads(base64.b64decode(next(row for row in documents if row["remote_path"]==other["binding_path"])["raw_base64"]))
        review=json.loads(base64.b64decode(next(row for row in documents if row["remote_path"]==other["review_path"])["raw_base64"]))
        entry["endpoint_root"]=other["endpoint_root"]
    elif fault.startswith("binding_"):
        key={"binding_seed":"seed","binding_seed_bool":"seed","binding_root":"endpoint_root","binding_post_pin":"whole_training_postcheck_sha256"}[fault]
        binding[key]=True if fault=="binding_seed_bool" else (2026100502 if fault=="binding_seed" else (str(module.PERSONAL/"wrong_root") if fault=="binding_root" else "f"*64))
    else:
        key={"review_seed":"seed","review_seed_bool":"seed","review_root":"endpoint_root","review_post_pin":"whole_training_postcheck_sha256","review_binding_pin":"endpoint_binding_sha256"}[fault]
        review[key]=True if fault=="review_seed_bool" else (2026100502 if fault=="review_seed" else (str(module.PERSONAL/"wrong_root") if fault=="review_root" else "f"*64))
    replace_document(authority,documents,2026100501,"binding_path",binding)
    if not fault.startswith("review_") and fault!="swapped_authentic_seed2_metadata":review["endpoint_binding_sha256"]=entry["binding_sha256"]
    replace_document(authority,documents,2026100501,"review_path",review)
    with pytest.raises(RuntimeError,match="per-seed"):module.input_membership(authority,documents)


@pytest.mark.parametrize("fault",["missing_seed","missing_document","duplicate_document","wrong_document_sha","changed_document_bytes",
    "wrong_post_seed","bool_post_seed","wrong_post_status","incomplete_updates","incomplete_examples","missing_check","false_check",
    "incomplete_binding","escaped_endpoint","noncanonical_member","conflicting_baseline"])
def test_input_admission_rejects_each_fault(fault):
    authority,documents=documents_fixture();entry=authority["completed_seeds"]["2026100503"]
    post=json.loads(base64.b64decode(next(row for row in documents if row["remote_path"]==entry["whole_postcheck_path"])["raw_base64"]))
    if fault=="missing_seed":del authority["completed_seeds"]["2026100503"]
    elif fault=="missing_document":documents.pop()
    elif fault=="duplicate_document":documents[-1]=deepcopy(documents[0])
    elif fault=="wrong_document_sha":documents[0]["sha256"]="f"*64
    elif fault=="changed_document_bytes":documents[0]["raw_base64"]=base64.b64encode(b'{}').decode()
    elif fault in ("wrong_post_seed","bool_post_seed","wrong_post_status","incomplete_updates","incomplete_examples","missing_check","false_check"):
        if fault=="wrong_post_seed":post["seed"]=2026100502
        elif fault=="bool_post_seed":post["seed"]=True
        elif fault=="wrong_post_status":post["status"]="COMPLETE_UNCHECKED"
        elif fault=="incomplete_updates":post["checked_optimizer_updates"]=5999
        elif fault=="incomplete_examples":post["checked_examples_seen"]=23999
        elif fault=="missing_check":post["checks"].pop("complete_raw_streams_exact")
        else:post["checks"]["owned_session_closed"]=False
        replace_document(authority,documents,2026100503,"whole_postcheck_path",post)
    elif fault in ("incomplete_binding","noncanonical_member"):
        binding=json.loads(base64.b64decode(next(row for row in documents if row["remote_path"]==entry["binding_path"])["raw_base64"]))
        if fault=="incomplete_binding":binding["files"].pop("payload_41.json")
        else:binding["files"]["../escape"]=binding["files"].pop("payload_41.json")
        replace_document(authority,documents,2026100503,"binding_path",binding)
    elif fault=="escaped_endpoint":entry["endpoint_root"]="/tmp/foreign"
    elif fault=="conflicting_baseline":authority["baseline_assets"]["GOOD6000"]["immutable_input_sha256"]=dict(authority["baseline_assets"]["common3000"]["immutable_input_sha256"]);authority["baseline_assets"]["GOOD6000"]["immutable_input_sha256"][str(module.PERSONAL/"fixtures/common3000")]="b"*64
    with pytest.raises(RuntimeError):module.input_membership(authority,documents)


@pytest.mark.parametrize("value",["../escape","a/../b","/tmp/a","a//b","a/./b","","a/"])
def test_relative_source_paths_fail_closed(value):
    with pytest.raises(RuntimeError):module.relative_name(value)


def test_source_archive_exact_regular35_and_reproducible(tmp_path,monkeypatch):
    monkeypatch.setattr(module,"ROOT",tmp_path);pins={}
    for i in range(35):
        p=tmp_path/"fixture"/("file_%02d.py"%i);p.parent.mkdir(exist_ok=True);p.write_bytes(("# HH_261005 - Scalar fixture %d.\n"%i).encode());pins[str(p.relative_to(tmp_path))]=module.sha(p.read_bytes())
    raw,rows=module.source_archive(pins);raw2,rows2=module.source_archive(pins)
    assert raw==raw2 and rows==rows2
    with tarfile.open(fileobj=io.BytesIO(raw),mode="r:") as archive:
        assert len(archive.getmembers())==35
        assert all(member.isfile() and member.mode==0o600 and member.mtime==0 for member in archive.getmembers())
        assert [member.name for member in archive.getmembers()]==[row["path"] for row in rows]


@pytest.mark.parametrize("fault",["count","hash","symlink","escape"])
def test_source_archive_faults(tmp_path,monkeypatch,fault):
    monkeypatch.setattr(module,"ROOT",tmp_path);pins={}
    for i in range(35):
        p=tmp_path/("file_%02d.py"%i);p.write_bytes(str(i).encode());pins[p.name]=module.sha(p.read_bytes())
    if fault=="count":pins.pop("file_00.py")
    elif fault=="hash":pins["file_00.py"]="a"*64
    elif fault=="symlink":(tmp_path/"file_00.py").unlink();(tmp_path/"file_00.py").symlink_to(tmp_path/"file_01.py")
    else:pins["../escape"]=pins.pop("file_00.py")
    with pytest.raises(RuntimeError):module.source_archive(pins)


def test_sealed_document_raw_lf_hash_not_canonical_hash(tmp_path):
    path=tmp_path/"document.json";raw=b'{"value":1}\n';path.write_bytes(raw)
    assert module.sealed_document(path,module.sha(raw))==(raw,{"value":1})
    with pytest.raises(RuntimeError):module.sealed_document(path,module.sha(module.encode({"value":1})))


@pytest.mark.parametrize("raw",[b'{"a":1,"a":2}',b'{"a":NaN}'])
def test_sealed_json_duplicate_nonfinite_rejected(tmp_path,raw):
    path=tmp_path/"document.json";path.write_bytes(raw)
    with pytest.raises(RuntimeError):module.sealed_document(path,module.sha(raw))


def test_review_exact_protocol_inputs_and_denominator():
    authority=dict(contract=dict(complete_record_denominator=35315));protocol={"synthetic":True};inputs={"synthetic":"a"*64}
    review=dict(status="APPROVED_EXACT_FIXED_FINAL_GPU0_EVALUATION",protocol=protocol,archive_sha256="b"*64,
        immutable_input_sha256=inputs,all_three_whole_training_postchecks_PASS=True,automatic_retry=False,automatic_additional_job=False)
    module.validate_review(review,authority,protocol,"b"*64,inputs)
    for key in review:
        broken=deepcopy(review);broken[key]=None
        with pytest.raises(RuntimeError):module.validate_review(broken,authority,protocol,"b"*64,inputs)


def test_no_actual_entry_or_process_invocation_in_test_and_guard_order():
    tree=ast.parse(SOURCE.read_bytes());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="main")
    source=ast.unparse(main)
    assert source.index("validate_review(") < source.index("OUT.mkdir(") < source.index("subprocess.Popen(")
    assert "timeout=7320" in source and "killpg" not in source
    assert "ownership" not in source or "unhandled" in source
    assert "'--review-sha256'" in source
    assert "actual_evaluation" not in globals()


def test_frozen_v1_inverse_ast_only_declared_grant_and_receipt_identities():
    # HH_261005 - Preserve the invalid old ROOT grant and frozen helper; only new admission identity fields change.
    old_path=ROOT/"tools/maintenance/supervise_three_seed_final_evaluation_v1_20261005.py"
    assert module.sha(old_path.read_bytes())=="6d7643eb5a0453c3dd8b87b05e2c2aeaaed8168e74a1749ea9d604c7be73bd65"
    original=ast.parse(old_path.read_bytes());fresh=ast.parse(SOURCE.read_bytes())
    fresh.body[0]=deepcopy(original.body[0])
    class Undo(ast.NodeTransformer):
        def visit_Constant(self,node):
            if type(node.value) is str:
                node.value=node.value.replace("supervisor_v2","supervisor_v1")\
                    .replace("test_supervise_three_seed_final_evaluation_v2_20261005.py","test_supervise_three_seed_final_evaluation_v1_20261005.py")\
                    .replace("ROOT_final_evaluation_authority_v2.json","ROOT_final_evaluation_authority.json")\
                    .replace("ROOT_final_evaluation_execution_review_v2.json","ROOT_final_evaluation_execution_review.json")
            return node
    assert normalized(Undo().visit(fresh))==normalized(original)
