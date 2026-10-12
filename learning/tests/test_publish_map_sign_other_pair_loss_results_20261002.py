"""HH_261002 - Synthetic full53 receipt reader and fixed stored panels only; no actual evaluation, model, GPU, or publication."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
from types import FunctionType,ModuleType,SimpleNamespace

import pytest

from scripts.e2e import publish_map_sign_other_pair_loss_results_20261002 as new


def write(path,value):
    path.write_bytes(json.dumps(value,sort_keys=True,allow_nan=False).encode())
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def study(tmp_path,monkeypatch):
    # HH_261002 - Reuse the original complete receipt fixture with ONLY a new private module namespace and truthful pair metadata.
    identity=dict(zip(new.IDENTITY_KEYS,("a"*64,"b"*64,"c"*64)))
    provider=ModuleType("hh_pair_loss_synthetic_provider")
    new.reader_namespace(identity,vars(provider))
    path=Path(__file__).with_name("test_publish_map_sign_margin_loss_results_20261002.py")
    spec=importlib.util.spec_from_file_location("hh_frozen_map_gallery_fixture",path)
    fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)
    function=fixtures.archives.__wrapped__
    archives=FunctionType(function.__code__,dict(function.__globals__,p=provider),function.__name__)(tmp_path,monkeypatch)
    caller_path,caller_sha,caller,schedule=archives
    summary=json.loads((provider.RUN/"summary.json").read_bytes())
    summary.update(source_count=12,direct_auxiliary_logits_are_only_sign5_other0=True,
        shared_feature_indirect_lane_or_recall_regression_possible=True,new_head=False,
        inference_argmax_or_threshold_changed=False)
    loss=summary["loss_contract"]
    loss.update(margin="sign_logit-other_logit",eligible_negative_target_class=0,
        direct_auxiliary_logit_gradient_classes=[0,5],
        conditional_probability="exp(sign)/(exp(sign)+exp(other)); NOT seven-class sign probability")
    loss_sha=provider.document_sha(loss);provider.LOSS_CONTRACT_SHA=loss_sha
    summary["loss_contract_sha256"]=loss_sha
    receipt=json.loads((provider.RUN/"intervention_archive_receipt.json").read_bytes())
    receipt.update(loss_contract_file_sha256=write(provider.RUN/"loss_contract.json",loss),loss_contract_sha256=loss_sha)
    caller["run"]["files"]["loss_contract.json"]=receipt["loss_contract_file_sha256"]
    caller["run"]["files"]["intervention_archive_receipt.json"]=write(provider.RUN/"intervention_archive_receipt.json",receipt)
    auth=json.loads((provider.RUN/"authorization.json").read_bytes());auth["loss_contract_sha256"]=loss_sha
    auth["contract"].update({key:summary[key] for key in ("source_count","direct_auxiliary_logits_are_only_sign5_other0",
        "shared_feature_indirect_lane_or_recall_regression_possible","new_head","inference_argmax_or_threshold_changed")})
    provider.AUTH_SHA=write(provider.RUN/"authorization.json",auth)
    caller["run"]["files"]["authorization.json"]=provider.AUTH_SHA
    summary["authorization_sha256"]=provider.AUTH_SHA
    caller["run"]["files"]["summary.json"]=write(provider.RUN/"summary.json",summary)
    identity.update(dict(zip(new.IDENTITY_KEYS,(provider.AUTH_SHA,provider.EQUAL_PROOF_SHA,loss_sha))))
    caller["actual_experiment_identity"]=dict(identity)
    caller_sha=write(caller_path,caller)
    return SimpleNamespace(provider=provider,identity=identity,caller=caller,path=caller_path,sha=caller_sha,schedule=schedule)


def test_full53_same350_predicates_exposures_no_TRAIN_model(study):
    report,panels,pins=study.provider.audit(study.path,study.sha);pins.recheck()
    assert report["source_count"]==12 and len(report["mirrored_run_inventory"])==53
    assert len(report["exploratory_screen"]["checks"])==27 and not report["exploratory_screen"]["passed_exploratory_only"]
    assert len(panels)==4 and report["full_cohort_counts"]=={key:2 for key in study.provider.SPLITS}
    assert all(row["examples_seen"]==32000 and row["every_actual_exposure_verified"]
        for row in report["actual_optimizer_exposures"].values())
    assert report["actual_experiment_identity"]==study.identity
    assert report["direct_auxiliary_logit_gradient_classes"]==[0,5]
    assert report["candidate_pair_conditional_probability_not_seven_class_probability"]
    assert all(report[key] is False for key in ("Torch_imported","model_inference_executed","pickle_loaded","raw_TRAIN_reopened",
        "model_promoted","vehicle_control_approved"))


@pytest.mark.parametrize("fault",["old_loss","old_source_count","nonpair_direct","new_head","changed_threshold","missing_artifact","bad_preflight","FALSE_changed"])
def test_old_or_unproven_identity_never_accepted(study,fault):
    p=study.provider;summary=json.loads((p.RUN/"summary.json").read_bytes())
    if fault=="old_loss":summary["loss_contract"]["margin"]="old seven-class margin"
    if fault=="old_source_count":summary["source_count"]=10
    if fault=="nonpair_direct":summary["direct_auxiliary_logits_are_only_sign5_other0"]=False
    if fault=="new_head":summary["new_head"]=True
    if fault=="changed_threshold":summary["inference_argmax_or_threshold_changed"]=True
    if fault in ("old_loss","old_source_count","nonpair_direct","new_head","changed_threshold"):
        study.caller["run"]["files"]["summary.json"]=write(p.RUN/"summary.json",summary)
    if fault=="missing_artifact":(p.RUN/"source/map_sign_other_margin_loss_v1_20261002.py").unlink()
    if fault=="bad_preflight":
        name="candidate_strict_CUDA_preflight.json";row=json.loads((p.RUN/name).read_bytes());row["all_arm_optimizer_updates_so_far"]=4000
        study.caller["run"]["files"][name]=write(p.RUN/name,row)
    if fault=="FALSE_changed":
        (p.RUN/"candidate/exposures.jsonl").write_bytes(b'{"actual_paired_TRUE_bits":[1]}')
        study.caller["run"]["files"]["candidate/exposures.jsonl"]=p.frozen.contract.digest(p.RUN/"candidate/exposures.jsonl")
    with pytest.raises(Exception):p.audit(study.path,write(study.path,study.caller))


@pytest.mark.parametrize("name",("read_study","audit","canvas","markdown"))
def test_original_AST_only_count_and_identity_labels_change(name):
    original=next(n for n in ast.parse(Path(new.previous.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)
    revised=new.revised_AST(name)
    old_constants=[n.value for n in ast.walk(original) if isinstance(n,ast.Constant)]
    new_constants=[n.value for n in ast.walk(revised) if isinstance(n,ast.Constant)]
    expected={"read_study":1,"audit":1,"canvas":1,"markdown":2}[name]
    assert sum(a!=b for a,b in zip(old_constants,new_constants))==expected
    assert len(old_constants)==len(new_constants)
    for old,value in zip([n for n in ast.walk(original) if isinstance(n,ast.Constant)],
        [n for n in ast.walk(revised) if isinstance(n,ast.Constant)]):value.value=old.value
    assert ast.dump(original)==ast.dump(revised)


def test_validate_and_synthetic_publish_create_only_same_FULL_pixels(study,monkeypatch):
    p=study.provider;before={name:getattr(new.previous,name) for name in ("OUTPUT","RUN","SOURCE","audit","read_study","publish")}
    args=SimpleNamespace(caller_pins=str(study.path),caller_pins_sha256=study.sha,validate_only=True)
    result=p.publish(args);assert not result["published"] and not p.OUTPUT.exists()
    monkeypatch.setattr(p.shutil,"disk_usage",lambda *a:SimpleNamespace(free=10*1024**3))
    args.validate_only=False;result=p.publish(args)
    assert result["published"] and result["png_count"]==4
    report=json.loads((p.OUTPUT/"results.json").read_bytes())
    assert report["source_count"]==12 and len(report["exploratory_screen"]["checks"])==27
    text=(p.OUTPUT/"README.md").read_text()
    assert "FAIL / 미채택" in text and "표지↔other 조건부 pair" in text and "메인 0/9" in text
    assert "17" not in p.OUTPUT.name and new.OUTPUT.name=="19_map_sign_other_pair_loss"
    assert all(p.frozen.contract.digest(p.OUTPUT/name)==pin for name,pin in report["publication_source_archive"].items())
    with pytest.raises(Exception):p.publish(args)
    assert before=={name:getattr(new.previous,name) for name in before}


@pytest.mark.parametrize("fault",["missing","foreign_field","wrong_authority","wrong_pin","old_caller"])
def test_external_actual_identity_and_public_cli_failclosed_before_archive(tmp_path,monkeypatch,fault):
    identity=dict(zip(new.IDENTITY_KEYS,("a"*64,"b"*64,"c"*64)))
    caller=dict(schema=new.CALLER_SCHEMA,output=str(new.OUTPUT),fixed_indices=new.previous.INDICES,
        actual_experiment_identity=identity,run=dict(files={"authorization.json":"a"*64}))
    if fault=="missing":caller.pop("actual_experiment_identity")
    if fault=="foreign_field":identity["extra"]="d"*64
    if fault=="wrong_authority":caller["run"]["files"]["authorization.json"]="d"*64
    if fault=="old_caller":caller["schema"]=new.previous.CALLER_SCHEMA
    path=tmp_path/"ROOT_caller.json";pin=write(path,caller)
    if fault=="wrong_pin":pin="0"*64
    monkeypatch.setattr(new.previous.frozen.Pins,"npz",lambda *a:pytest.fail("unproven caller must not open arrays"))
    with pytest.raises(Exception):new.caller_namespace(path,pin)


def test_exact_global_fixed10_and_FULL_native_FOV_unchanged():
    assert new.previous.INDICES=={"dev_town01":[0,29,58,88,117],"holdout_town02_p02":[0,57,115,174,231]}
    assert new.previous.SHAPE==(360,640) and new.previous.SPLITS["dev_town01"][0]==118
    assert new.previous.SPLITS["holdout_town02_p02"][0]==232
    assert len(new.SOURCE)==12 and new.SOURCE.items()>=new.previous.SOURCE.items()
    assert len(new.reader_namespace(dict(zip(new.IDENTITY_KEYS,("a"*64,"b"*64,"c"*64))))["required_run_files"]())==53


def test_PUBLIC_CLI_dispatch_and_original_provider_restoration_on_error(tmp_path,monkeypatch,capsys):
    before={name:getattr(new.previous,name) for name in ("OUTPUT","RUN","SOURCE","audit","read_study","publish")}
    seen=[]
    monkeypatch.setattr(new,"publish",lambda args:seen.append(vars(args)) or dict(published=False))
    assert new.main(["--caller-pins",str(tmp_path/"caller.json"),"--caller-pins-sha256","a"*64,"--validate-only"])==0
    assert seen[0]["validate_only"] and json.loads(capsys.readouterr().out)=={"published":False}
    with pytest.raises(Exception):new.reader_namespace({})
    assert before=={name:getattr(new.previous,name) for name in before}
