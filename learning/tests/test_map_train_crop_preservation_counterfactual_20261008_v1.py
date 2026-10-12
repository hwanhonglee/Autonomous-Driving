"""HH_261008 - Scalar/AST/fake-files only; no original targets, NumPy/Pillow, model or training.

The fixed policy has its separate52-pure witness. These fixtures check NEW
counter transactions, complete20k/320k failure representation, archival budgets
and genuine frozen-source seams, not actual array replay or environment parity.
"""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/"tools/e2e/audit_map_train_crop_preservation_counterfactual_20261008_v1.py"


@pytest.fixture
def tool():
    spec=importlib.util.spec_from_file_location("_hh_crop_counterfactual_pure",SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


@pytest.fixture
def original(tool):
    old,policy=tool.modules();return old,policy


def row(old,tool,full=0,source=0,after=0,protected=True,rescued=False):
    histogram=[tool.PIXELS-after,0,0,0,0,after,0]
    original=old.exposure_counts(full,source,histogram,False)
    choice=dict(protected=protected,full_GT_negative=full==0,rescued=rescued)
    return original+tool.branch_counts(choice,source)


def fake_plan(tool,old):
    return dict(source_sha256={"/fake/source.py":"a"*64},input_sha256={"/fake/input.json":"b"*64},
        old_source_archives={"/fake/old_archive.py":"a"*64},new_source_sha256={"/fake/fresh.py":"c"*64},
        evidence=dict(journal=dict(slot_exposures=[133]*412+[132]*797)),
        prepare_process_generation=dict(PID=1,PPID=2,process_group=3,session=4,start_ticks=5,
            boot_id="00000000-0000-0000-0000-000000000000",executable="/fake/python",cmdline_sha256="d"*64))


def test_only_frozen_source_modules_loaded_and_original_globals_not_rebound(tool,original):
    old,policy=original
    assert Path(old.__file__).resolve()==tool.OLD and Path(policy.__file__).resolve()==tool.POLICY
    for path,pin in {**tool.REFERENCE_PINS,**tool.POLICY_PINS}.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==pin
    assert old.OUTPUT==tool.PARENT_OUTPUT and old.PLAN_DIR==tool.PARENT_DIR and tool.OUTPUT!=old.OUTPUT
    assert tool.fixed(policy)["old_authority_used_as_new_permission"] is False
    assert not any(name in tool.sys.modules for name in ("numpy","PIL","torch","carla","rclpy"))


def test_actual_entry_denies_missing_external_hashes_and_lowercase_own_plan_binding(tool,original):
    old,policy=original
    for plan_sha,grant_sha in ((None,None),("a"*64,None),("x"*64,"b"*64)):
        with pytest.raises(ValueError):tool.admission(plan_sha,grant_sha)
    plan=fake_plan(tool,old);grant=tool.authority_value(plan,"f"*64)
    assert grant["plan_path"]==str(tool.PLAN) and grant["plan_sha256"]=="f"*64
    assert "ROOT_plan_path" not in grant and "ROOT_plan_sha256" not in grant
    assert grant["TRAIN_target_counterfactual_replay_authorized"] is True
    assert grant["training_authorized"] is grant["NN_authorized"] is False


@pytest.mark.parametrize("field",["TRAIN_target_counterfactual_replay_authorized","training_authorized","NN_authorized"])
def test_integer_boolean_lookalike_authority_does_not_pass_semantic_admission(tool,original,monkeypatch,field):
    old,policy=original;plan=fake_plan(tool,old);plan.update(schema=tool.SCHEMA,fixed=tool.fixed(policy))
    grant=tool.authority_value(plan,"a"*64);grant[field]=int(grant[field])
    monkeypatch.setattr(tool,"read_json",lambda path,pin=None:grant if path==tool.AUTHORITY else plan)
    with pytest.raises(ValueError,match="typed ROOT"):tool.admission("a"*64,"b"*64)


def test_negative_and_unprotected_box_unchanged_without_candidate_crop(tool,original):
    old,policy=original;box=[0,20,360,400]
    for ordinal,full,after in ((1,4,0),(0,0,0),(2,4,1)):
        choice=policy.select_box(ordinal,box,full,after)
        assert choice["selected_box"] is box and choice["rescued"] is False
    choice=policy.select_box(0,box,4,0)
    assert choice["selected_box"]==[0,0,360,640] and tool.branch_counts(choice,0)==[1,0,0,1,0,1,0]
    assert tool.branch_counts(choice,2)==[1,0,0,1,0,0,1]


def test_transactional_candidate_25fields_repeated_slot_and_atomic_rejection(tool,original):
    old,_=original;stats=tool.empty_candidate(old);before=tool.encoded(stats)
    values=[row(old,tool)]*8;result=tool.commit_candidate(old,stats,[0]*8,values)
    assert tool.encoded(stats)==before and result["overall"][0]==result["slot_statistics"][0][0]==8
    assert result["overall"][18:]==[8,0,8,0,8,0,0] and sum(result["overall"][11:18])==8*tool.PIXELS
    bad=deepcopy(values);bad[-1][0]=2
    with pytest.raises(ValueError):tool.commit_candidate(old,stats,[0]*8,bad)
    assert tool.encoded(stats)==before


@pytest.mark.parametrize("fault",["bool","negative","oversize_pixel","bad_count","bad_histogram","bad_partition","short","slot_type","slot_outside"])
def test_complete_single_exposure_capacity_contract_denials(tool,original,fault):
    old,_=original;stats=tool.empty_candidate(old);values=[row(old,tool) for _ in range(8)];slots=[0]*8
    if fault=="bool":values[0][0]=True
    elif fault=="negative":values[0][8]=-1
    elif fault=="oversize_pixel":values[0][8]=tool.PIXELS+1
    elif fault=="bad_count":values[0][0]=2
    elif fault=="bad_histogram":values[0][11]-=1
    elif fault=="bad_partition":values[0][18]=0
    elif fault=="short":values[0].pop()
    elif fault=="slot_type":slots[0]=False
    else:slots[0]=1209
    with pytest.raises((ValueError,IndexError)):tool.commit_candidate(old,stats,slots,values)


def test_candidate_rescue_exact_counts_against_unchanged_baseline_per_slot(tool,original):
    old,_=original;base=old.empty_statistics();candidate=tool.empty_candidate(old)
    baseline_hist=[tool.PIXELS,0,0,0,0,0,0]
    baseline_row=old.exposure_counts(4,0,baseline_hist,False)
    candidate_hist=[tool.PIXELS-4,0,0,0,0,4,0]
    rescued=old.exposure_counts(4,4,candidate_hist,True)+[1,0,0,1,0,1,0]
    original_rows=[baseline_row]*8;candidate_rows=[rescued]*8
    base=old.commit_counts(base,[0]*8,original_rows,[4]*1209)
    candidate=tool.commit_candidate(old,candidate,[0]*8,candidate_rows)
    tool.candidate_partition_checks(candidate,base)
    assert candidate["overall"][4]==0 and base["shared_once_overall"][4]==8
    assert candidate["overall"][21]==8 and candidate["overall"][10]==32


@pytest.mark.parametrize("fault",["protection","negative","rescue","raw_reason","whole_crop","zero_after","classes","GT_pixels","slots"])
def test_whole_candidate_partitions_and_seven_class_denominator_not_assumed(tool,original,fault):
    old,_=original;base=old.empty_statistics();candidate=tool.empty_candidate(old)
    if fault=="slots":candidate["slot_statistics"].pop()
    else:
        column={"protection":18,"negative":20,"rescue":21,"raw_reason":23,"whole_crop":1,"zero_after":4,"classes":11,"GT_pixels":8}[fault]
        candidate["overall"][column]=1
    with pytest.raises(ValueError):tool.candidate_partition_checks(candidate,base)


@pytest.mark.parametrize("passed,failed",[(0,0),(0,1),(1,1),(19999,1),(20000,0)])
def test_full_original20k_two_arm_failure_denominator_bitmap_and_newhash_labels(tool,original,passed,failed):
    old,_=original;plan=fake_plan(tool,old)
    result=tool.result_value(old,plan,"a"*64,"b"*64,passed,failed,{"stage":"SYNTHETIC"},tool.empty_candidate(old),None,
        "c"*64,"d"*64,"e"*64,"00"*2500,{},None,plan["prepare_process_generation"],0.,False)
    assert result["status"]=="FAILED_WITH_FULL_ORIGINAL_DENOMINATOR"
    assert result["original_total_exposures"]==320000
    for cell in result["by_arm"].values():assert cell["PASS_batches"]+cell["FAIL_batches"]+cell["UNEXECUTED_batches"]==20000
    assert result["baseline_B8_target_chain_sha256"]=="c"*64 and result["candidate_NEW_B8_target_chain_sha256"]=="d"*64
    assert result["baseline_statistics_reference"]["path"]==str(tool.PARENT_OUTPUT/"result.json")
    assert len(result["verified_batch_bitmap_hex"])==5000 and result["NN"]==result["training_steps"]==0


def test_populated_full1209_seal_errors_and_partial_B8_fit_existing512K_proof(tool,original):
    old,_=original;plan=fake_plan(tool,old)
    assert 0<tool.closing_proof(old,plan)<=tool.CLOSING==524288
    plan["evidence"]["journal"]["slot_exposures"][0]=134
    with pytest.raises(ValueError):tool.closing_proof(old,plan)


def test_arbitrary_large_full_seal_error_envelope_denies_before_pixels(tool,original):
    old,_=original;plan=fake_plan(tool,old)
    plan["input_sha256"]={"/fake/"+str(n)+"x"*512:"f"*64 for n in range(1000)}
    with pytest.raises(ValueError):tool.closing_proof(old,plan)


def storage_fixture(tool,old,tmp_path,monkeypatch):
    plan_root=tmp_path/"plan";plan_root.mkdir();out=tmp_path/"out"
    skeleton=plan_root/"PLAN_skeleton.json";authority=plan_root/"ROOT_authority.skeleton.json"
    skeleton.write_bytes(b"{}\n");authority.write_bytes(b"{}\n")
    monkeypatch.setattr(tool,"PLAN_DIR",plan_root);monkeypatch.setattr(tool,"OUTPUT",out)
    monkeypatch.setattr(tool,"SKELETON",skeleton);monkeypatch.setattr(tool,"AUTHORITY_SKELETON",authority)
    source=tmp_path/"source.py";source.write_bytes(b"# fake\n")
    plan=fake_plan(tool,old);plan["new_source_sha256"]={str(source):hashlib.sha256(source.read_bytes()).hexdigest()}
    return plan,plan_root,out,source


def test_forecast_counts_existing_two_skeletons_then_fresh_archive_and_all_RAW_copies(tool,original,tmp_path,monkeypatch):
    old,_=original;plan,root,out,source=storage_fixture(tool,old,tmp_path,monkeypatch)
    actual=tool.storage_admission(old,plan,True)
    expected=6+source.stat().st_size+3*len(tool.encoded(plan))+2*len(tool.encoded(tool.authority_value(plan,"f"*64)))+65536+524288
    assert actual==expected<=tool.LIMIT and not out.exists() and not (root/"sources_by_SHA256").exists()
    (root/"old_candidate.json").write_bytes(b"{}")
    with pytest.raises(ValueError):tool.storage_admission(old,plan,True)


def test_forecast_large_sources_denies_without_arrays_output_or_limit_raise(tool,original,tmp_path,monkeypatch):
    old,_=original;plan,root,out,source=storage_fixture(tool,old,tmp_path,monkeypatch)
    source.write_bytes(b"x"*tool.LIMIT)
    with pytest.raises(ValueError):tool.storage_admission(old,plan,True)
    assert not out.exists() and not (root/"sources_by_SHA256").exists() and tool.LIMIT==2097152


def test_own_floor_and_sampled_quota_gate_retained_without_original_global_mutation(tool,tmp_path,monkeypatch):
    calls=[];budget=SimpleNamespace(guard=lambda:calls.append("original sampled guard"));old=object()
    monkeypatch.setattr(tool,"used",lambda:tool.LIMIT-tool.CLOSING+1)
    monkeypatch.setattr(tool.os,"statvfs",lambda _:SimpleNamespace(f_bavail=2*1024**3+64*1024**2,f_frsize=1))
    with pytest.raises(ValueError):tool.own_guard(old,budget)
    tool.own_guard(old,budget,False)
    monkeypatch.setattr(tool.os,"statvfs",lambda _:SimpleNamespace(f_bavail=2*1024**3,f_frsize=1))
    with pytest.raises(ValueError):tool.own_guard(old,budget,False)
    assert len(calls)==3


def test_combined_plan_output_quota_fail_before_any_write_and_create_only(tool,tmp_path,monkeypatch):
    plan=tmp_path/"plan";output=tmp_path/"out";plan.mkdir();output.mkdir()
    monkeypatch.setattr(tool,"PLAN_DIR",plan);monkeypatch.setattr(tool,"OUTPUT",output)
    (plan/"existing").write_bytes(b"x"*(tool.LIMIT-tool.CLOSING))
    with pytest.raises(ValueError):tool.write_new(output/"denied",b"x")
    assert not (output/"denied").exists()
    raw=b'{ "ns":18446744073709551615 }\n';tool.write_new(output/"closing",raw,True)
    assert (output/"closing").read_bytes()==raw and tool.used()==tool.LIMIT-tool.CLOSING+len(raw)
    with pytest.raises(FileExistsError):tool.write_new(output/"closing",raw,True)


def test_original_label_loader_crop_and_source_helpers_are_not_reimplemented_or_activated(tool,original):
    old,_=original;original_tree=ast.parse(tool.OLD.read_bytes());new_tree=ast.parse(SOURCE.read_bytes())
    original_functions={n.name:n for n in original_tree.body if isinstance(n,ast.FunctionDef)}
    names={n.name for n in new_tree.body if isinstance(n,ast.FunctionDef)}
    for name in ("load_targets","fill_exact","crop_ast","compile_target_crop","native_support","validate_batch","evidence"):
        assert name in original_functions and name not in names
    calls={ast.unparse(n.func) for n in ast.walk(new_tree) if isinstance(n,ast.Call)}
    assert {"old.compile_target_crop","old.load_targets","old.native_support","old.validate_batch","old.evidence"}<=calls
    assert not calls&{"old.execute","old.prepare","old.authorization","np.load","train_package","load_model","providers"}
    numeric_imports=[n for n in ast.walk(new_tree) if isinstance(n,ast.Import) and any(a.name=="numpy" for a in n.names)]
    assert len(numeric_imports)==1
    execute=next(n for n in new_tree.body if isinstance(n,ast.FunctionDef) and n.name=="execute")
    body=ast.unparse(execute)
    assert body.index("admission(plan_sha, grant_sha)")<body.index("old.lease()")<body.index("storage_admission")<body.index("import numpy")
    assert body.index("verify(old, plan, budget)")<body.index("import numpy")
    assert "setattr(old" not in body and "old.__dict__" not in body
    assert tool.LIMIT==2097152 and tool.CLOSING==524288 and tool.RSS==536870912 and tool.DEADLINE==900


def test_candidate_hash_commits_follow_baseline_hash_admission_and_all_flags_keep_nontraining(tool,original):
    _,policy=original;tree=ast.parse(SOURCE.read_bytes());execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="execute")
    body=ast.unparse(execute)
    loop=next(n for n in ast.walk(execute) if isinstance(n,ast.For) and isinstance(n.target,ast.Tuple)
        and [part.id for part in n.target.elts if isinstance(part,ast.Name)]==["ordinal","row"])
    expected=ast.parse('require(base_hash.hexdigest()==row["target_sha256"],"historical baseline B8 mismatch; no favorable subset")').body[0]
    guard=next(i for i,n in enumerate(loop.body) if ast.dump(n,include_attributes=False)==ast.dump(expected,include_attributes=False))
    start=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id=="next_candidate")
    committed=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Tuple)
        and any(isinstance(part,ast.Name) and part.id=="passed" for part in n.targets[0].elts))
    assert guard<start<committed
    assert "base_stats == baseline['statistics']" in body and "candidate_partition_checks(stats, base_stats)" in body
    assert "policy.exposure_index(ordinal, position)" in body and "policy.select_box" in body
    value=tool.fixed(policy)
    assert value["baseline_all20000_B8_hashes_required"] and value["candidate_hashes_new_not_historical"]
    assert value["NN"]==value["training_steps"]==0 and value["RGB_TRUE_DEV_HOLDOUT_TEST_weights_predictions_deniedRAW_bodies_opened"] is False
