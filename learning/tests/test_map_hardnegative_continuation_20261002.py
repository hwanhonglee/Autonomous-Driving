"""HH_261002 - Synthetic fixed continuation, lineage, sampler and no-evaluation-before-fit tests."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools/e2e"))
import train_map_hardnegative_continuation_20261002 as study


@pytest.fixture(scope="module")
def inventory():
    # HH_261002 - Synthetic full49 records preserve census and stratum cardinalities without opening actual data.
    remaining=np.asarray(study.mining.CLASS_PIXELS,dtype=np.int64).copy()
    sign=np.zeros(1209,dtype=np.int64);sign[:391]=32;sign[391:584]=385;sign[391:547]+=1
    assert sign.sum()==remaining[5]
    reserved=np.zeros((1209,7),dtype=np.int64);reserved[:,5]=sign;reserved[584:729,0]=7
    remaining-=reserved.sum(0);rows=[]
    for i in range(1209):
        counts=reserved[i].copy();left=study.mining.PIXELS-int(counts.sum())
        for k in (0,1,2,3,4,6):
            n=min(left,int(remaining[k]));counts[k]+=n;remaining[k]-=n;left-=n
        assert left==0
        matrix=np.diag(counts)
        if 584<=i<729:matrix[0,0]-=1;matrix[0,5]+=1
        actual,proposed,diagonal=matrix.sum(1),matrix.sum(0),np.diag(matrix)
        rows.append(dict(index=i,sample_id=study.mining.slot_id(i),source_group=study.mining.source_group(i),
            split="TRAIN",appearance="FALSE_OR_ORIGINAL",full_FOV=True,rgb_sha256="a"*64,target_sha256="b"*64,
            parent_prediction_sha256="c"*64,confusion=matrix.tolist(),target_class_pixels=actual.tolist(),
            predicted_class_pixels=proposed.tolist(),sign_target_pixels=int(actual[5]),sign_true_positive_pixels=int(diagonal[5]),
            sign_false_negative_pixels=int(actual[5]-diagonal[5]),sign_false_positive_pixels=int(proposed[5]-diagonal[5]),
            other_to_sign_false_positive_pixels=int(matrix[0,5])))
    assert remaining.sum()==0
    result=study.mining.inventory(rows)
    assert result["stratum_frame_counts"]==dict(small_sign=391,large_sign=193,hard_negative=145)
    return result


@pytest.fixture(scope="module")
def schedule(inventory):
    return study.mining.schedules(inventory)


def authorization(source):
    return dict(schema=study.AUTHORIZATION_SCHEMA,source_sha256=source,contract=study.prepared_contract(),
        reviewer="synthetic-source-test-only",research_fit_authorized=True,native_denials_preserved=True,
        DEV_or_HOLDOUT_used_for_fit=False,automatic_model_promotion=False,vehicle_control_approved=False)


def test_contract_current_parent_and_exact_sampler_only():
    value=study.prepared_contract()
    assert value["initial_model_weights_sha256"]=="2284511d2b28c273ab731fcbc7b1326c8e040f4c1d84d57faacf2df86afd8310"
    assert value["dataset_origin_parent_weights_sha256"]!=value["initial_model_weights_sha256"]
    assert value["additional_updates_per_arm"]==4000 and value["examples_per_arm"]==32000
    assert value["total_lineage_updates_per_arm"]==14000 and value["parameter_count"]==245831
    assert value["architecture_unchanged"] and value["semantic_loss_unchanged"] and not value["labels_changed"]
    assert value["exploratory_screen"]["comparators"]==["parent","control"]
    assert value["evaluation_opens_only_after_both_final4000"] and value["known_evaluation_not_independent"]
    assert not value["training_authorized"] and not value["model_promoted"] and not value["vehicle_control_approved"]
    assert value["main_capabilities_approved"]==value["subfunctions_approved"]==0


@pytest.mark.parametrize("key",["research_fit_authorized","native_denials_preserved","DEV_or_HOLDOUT_used_for_fit", "automatic_model_promotion","vehicle_control_approved", "schema","source_sha256","reviewer"])
def test_external_exact_fit_authorization_required(key):
    source={"synthetic":"a"*64};value=authorization(source)
    study.validate_authorization(value,source)
    value[key]=not value[key] if type(value[key]) is bool else None
    with pytest.raises(ValueError):study.validate_authorization(value,source)


@pytest.mark.parametrize("key",["initial_model_weights_sha256","dataset_origin_parent_weights_sha256", "additional_updates_per_arm","total_lineage_updates_per_arm","gradient_norm_cap","class_weights","shared_crop_law", "known_evaluation_not_independent", "mining_artifact_sha256","exploratory_screen"])
def test_contract_mismatch_not_silently_accepted(key):
    source={"synthetic":"b"*64};value=authorization(source);value["contract"][key]=None
    with pytest.raises(ValueError):study.validate_authorization(value,source)


def test_bool_numeric_contract_alias_is_rejected():
    source={"synthetic":"a"*64};value=authorization(source)
    value["contract"]["independent_new_dev_sites"]=False
    with pytest.raises(ValueError):study.validate_authorization(value,source)


def write_exposures(path,arm,schedule):
    with path.open("x") as stream:
        for step in range(4000):
            stream.write(json.dumps(dict(step=step+1,physical_slot_indices=schedule[arm+"_indices"][step].tolist(),
                shared_crop_boxes=schedule["shared_crops"][step].tolist(),actual_paired_TRUE_bits=schedule["shared_FALSE_view_bits"][step].tolist()))+"\n")


@pytest.mark.parametrize("arm",study.ARMS)
def test_actual_full4000_journal_matches_all_schedule_denominators(tmp_path,schedule,arm):
    path=tmp_path/"exposures.jsonl";write_exposures(path,arm,schedule)
    result=study.verify_exposures(path,arm,schedule)
    assert result["steps"]==4000 and result["examples_seen"]==32000
    for key,suffix in (("sample_sequence_sha256",arm+"_indices"),("crop_sequence_sha256","shared_crops"),("view_sequence_sha256","shared_FALSE_view_bits")):
        assert result[key]==study.mining.array_sha(schedule[suffix])
    assert sum(result["physical_slot_exposures"])==32000 and min(result["physical_slot_exposures"])>=13


@pytest.mark.parametrize("kind",["missing","repeat_step","bool_step","wrong_train_index","heldout_index","TRUE_view","crop_reroll"])
def test_exposure_mutations_rejected(tmp_path,schedule,kind):
    path=tmp_path/"exposures.jsonl";write_exposures(path,"candidate",schedule)
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    if kind=="missing":rows.pop()
    elif kind=="repeat_step":rows[-1]["step"]=3999
    elif kind=="bool_step":rows[0]["step"]=True
    elif kind=="wrong_train_index":rows[0]["physical_slot_indices"][0]=1208
    elif kind=="heldout_index":rows[0]["physical_slot_indices"][0]=1209
    elif kind=="TRUE_view":rows[0]["actual_paired_TRUE_bits"][0]=1
    else:rows[0]["shared_crop_boxes"][0][0]+=1
    path.write_text("\n".join(json.dumps(row) for row in rows)+"\n")
    with pytest.raises(ValueError):study.verify_exposures(path,"candidate",schedule)


def evaluation():
    base=dict(iou={label:.8 for label in study.mining.LABELS},precision={label:.5 for label in study.mining.LABELS},
        recall={label:.8 for label in study.mining.LABELS},other_to_sign_false_positive_pixels=100)
    result={split:{name:copy.deepcopy(base) for name in ("parent",*study.ARMS)} for split in study.previous.EVAL_FILES}
    result["dev_town01"]["candidate"]["precision"]["traffic_sign"]=.6
    result["dev_town01"]["candidate"]["other_to_sign_false_positive_pixels"]=90
    return result


def test_predeclared_screens_both_comparators_and_no_promotion():
    result=study.diagnostic_screen(evaluation())
    assert len(result["checks"])==20 and all(result["checks"].values())
    assert result["passed_exploratory_only"] and not result["model_promoted"] and not result["independent_validation"]


@pytest.mark.parametrize("comparator",["parent","control"])
@pytest.mark.parametrize("kind",["precision_tie","FP_tie","recall_drop","road_drop","lane_drop","light_drop","town02_sign_drop","absent_sign"])
def test_any_fixed_screen_failure_retained(comparator,kind):
    values=evaluation()
    if kind=="precision_tie":values["dev_town01"][comparator]["precision"]["traffic_sign"]=.6
    elif kind=="FP_tie":values["dev_town01"][comparator]["other_to_sign_false_positive_pixels"]=90
    elif kind=="recall_drop":values["dev_town01"]["candidate"]["recall"]["traffic_sign"]=.779
    elif kind=="town02_sign_drop":values["holdout_town02_p02"]["candidate"]["iou"]["traffic_sign"]=.789
    elif kind=="absent_sign":values["dev_town01"][comparator]["precision"]["traffic_sign"]=None
    else:values["dev_town01"]["candidate"]["iou"][{"road_drop":"road","lane_drop":"lane_marking","light_drop":"traffic_light_fixture"}[kind]]=.789
    result=study.diagnostic_screen(values)
    assert not result["passed_exploratory_only"] and not all(result["checks"].values())


def test_full49_null_absent_iou_and_false_positive_not_hidden():
    matrix=np.zeros((7,7),dtype=np.int64);matrix[0,0]=90;matrix[0,5]=10
    result=study.metrics(matrix)
    assert result["iou"]["traffic_sign"] is None and result["precision"]["traffic_sign"]==0.
    assert result["recall"]["traffic_sign"] is None and result["false_positive_pixels"]["traffic_sign"]==10
    assert result["other_to_sign_false_positive_pixels"]==10 and result["confusion"]==matrix.tolist()


def test_fit_locally_rejected_before_files_or_cuda(monkeypatch):
    monkeypatch.setattr(study.os,"uname",lambda:type("Host",(),{"nodename":"synthetic-local"})())
    with pytest.raises(ValueError,match="private sourceworktree"):
        study.run(type("Args",(),{"authorization":"/must/not/open","authorization_sha256":"0"*64})())


def test_evaluation_cannot_open_with_one_or_incomplete_endpoint(monkeypatch):
    # HH_261002 - Reject missing endpoint before importing any actual prediction or label payload.
    fake_torch=type("FakeTorch",(),{})();monkeypatch.setitem(sys.modules,"torch",fake_torch)
    monkeypatch.setattr(study.contract,"read_json",lambda *a,**k:pytest.fail("evaluation labels opened before both fixed exports"))
    with pytest.raises(ValueError,match="BOTH"):
        study.evaluate_endpoints({"control":dict(steps=4000)},None,None)


def test_all_eight_source_dependencies_and_frozen_hashes():
    assert len(study.SOURCE_NAMES)==8
    for name,sha in study.FROZEN_SOURCE_SHA.items():
        path=ROOT/("learning/portable_e2e" if name in study.contract.ARCHITECTURE_SHA else "tools/e2e")/name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==sha
    # HH_261002 - Lazy imports used by run are explicitly in the exact flat source closure; no mutable glob.
    parsed=ast.parse(Path(study.__file__).read_text())
    for node in ast.walk(parsed):
        if isinstance(node,ast.Import):
            for alias in node.names:
                if alias.name.startswith(("map_","train_map_")):assert alias.name+".py" in study.SOURCE_NAMES


def test_minimal_eight_file_clone_imports_and_hashes_without_torch(tmp_path):
    for name in study.SOURCE_NAMES:
        path=ROOT/("learning/portable_e2e" if name in study.contract.ARCHITECTURE_SHA else "tools/e2e")/name
        shutil.copyfile(path,tmp_path/name)
    code="import sys,pathlib;import train_map_hardnegative_continuation_20261002 as m;assert 'torch' not in sys.modules;assert len(m.source_snapshot(pathlib.Path.cwd()))==8;assert m.prepared_contract()['additional_updates_per_arm']==4000;print('PASS8_NO_TORCH')"
    result=subprocess.run([sys.executable,"-B","-c",code],cwd=tmp_path,env={"PATH":"/usr/bin:/bin","PYTHONDONTWRITEBYTECODE":"1"},capture_output=True,text=True,timeout=20)
    assert result.returncode==0,result.stderr
    assert result.stdout.strip()=="PASS8_NO_TORCH"


def test_default_contract_initializes_no_cuda_or_output(capsys):
    assert study.main(["--mode","contract"])==0
    value=json.loads(capsys.readouterr().out)
    assert value["training_authorized"] is False and value["initial_completed_updates"]==10000


def test_validate_only_run_actual_api_chain_without_torch(monkeypatch,capsys,inventory,schedule):
    source={"synthetic":"d"*64};auth=authorization(source)
    monkeypatch.setattr(study,"environment_contract",lambda:None)
    monkeypatch.setattr(study,"source_snapshot",lambda root:source)
    monkeypatch.setattr(study.contract,"payload",lambda *a: (json.dumps(auth).encode(),"e"*64))
    monkeypatch.setattr(study,"load_inventory",lambda:(inventory,schedule,{"synthetic":"proof"}))
    monkeypatch.setattr(study.mining,"initial_lineage",lambda:None)
    manifest=dict(common_class_pixels=study.mining.CLASS_PIXELS,common_class_weights=study.mining.CLASS_WEIGHTS)
    monkeypatch.setattr(study.contract,"load_package",lambda *a:(manifest,[],None,None,None))
    monkeypatch.setattr(study,"verify_train_identity",lambda *a:None)
    monkeypatch.setitem(sys.modules,"torch",None)
    study.run(type("Args",(),{"authorization":"/synthetic/auth","authorization_sha256":"e"*64,"validate_only":True})())
    value=json.loads(capsys.readouterr().out)
    assert value["status"]=="CPU_PREFLIGHT_PASS_NO_CUDA_OR_OUTPUT" and value["training_authorized"] is True
    assert value["model_promoted"] is False
