"""HH_261002 - Synthetic prospective sampler-only v2 tests. No actual inventory/model/GPU/fit or site promotion."""

import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tools/e2e"))
import train_map_hardnegative_continuation_v2_20261002 as study
from learning.tests import test_map_hardnegative_continuation_20261002 as frozen_tests


@pytest.fixture(scope="module")
def inventory():
    return frozen_tests.inventory.__wrapped__()


@pytest.fixture(scope="module")
def original(inventory):
    return study.mining.schedules(inventory)


@pytest.fixture(scope="module")
def schedule(inventory,original):
    return study.schedules(inventory,original)


@pytest.fixture(scope="module")
def proof(inventory,original,schedule):
    return study.schedule_proof(inventory,schedule,original)


def authorization(source,proof):
    return dict(schema=study.AUTHORIZATION_SCHEMA,source_sha256=source,contract=study.prepared_contract(),
        new_sampler_proof_sha256=study.sampler_proof_sha256(proof),reviewer="synthetic-independent-source-review",
        research_fit_authorized=True,native_denials_preserved=True,DEV_or_HOLDOUT_used_for_fit=False,
        automatic_model_promotion=False,vehicle_control_approved=False)


def test_contract_declares_new_sampler_not_new_loss_or_known_site_independence():
    value=study.prepared_contract();old=study.previous.prepared_contract()
    assert value["schema"]==study.SCHEMA and value["source"]!=old["source"] and value["output"]!=old["output"]
    assert value["candidate_slot_roles"]==["all_TRAIN"]*4+["small_sign"]*2+["large_sign","hard_negative"]
    assert value["candidate_role_exposures"]==dict(all_TRAIN=16000,small_sign=8000,large_sign=4000,hard_negative=4000)
    for key in ("initial_model_weights_sha256","class_pixels","class_weights","learning_rate","weight_decay",
        "parameter_count","shared_crop_law","examples_per_arm","seed","exploratory_screen"):
        assert value[key]==old[key]
    assert value["hypothesis_adapted_after_prior_known_site_diagnostics"] is True
    assert value["known_evaluation_not_independent"] and not value["measured_recall_loss_sampler_causality_proven"]
    assert not value["training_authorized"] and not value["model_promoted"] and not value["vehicle_control_approved"]
    assert value["original_schedule_sha256"]==study.previous.MINING_SHA["proposed_schedule.npz"]


def test_full1209_schedule_roles_cycles_tails_and_original_crops_remain_exact(inventory,original,schedule,proof):
    before={k:v.copy() for k,v in original.items()}
    for key in ("control_indices","shared_crops","shared_FALSE_view_bits"):
        assert np.array_equal(schedule[key],original[key]) and schedule[key] is not original[key]
    assert np.array_equal(schedule["candidate_indices"][:,:4],original["candidate_indices"][:,:4])
    assert np.array_equal(schedule["candidate_indices"][:,6],original["candidate_indices"][:,5])
    assert np.array_equal(schedule["candidate_indices"][:,7],original["candidate_indices"][:,6:8].ravel()[:4000])
    for role,column,count,seed in (("small_sign",4,2,study.SEED+10),("large_sign",6,1,study.SEED+11),("hard_negative",7,1,study.SEED+12)):
        expected=study.mining.full_cycle(np.asarray(inventory["strata"][role]),4000*count,seed)
        assert np.array_equal(schedule["candidate_indices"][:,column:column+count].ravel(),expected)
    assert proof["examples_per_arm"]==32000 and proof["physical_TRAIN_slots"]==1209
    assert proof["full_cycles_and_partial_tails"]["small_sign"]==dict(required=8000,frame_count=391,full_cycles=20,partial_tail=180)
    assert proof["full_cycles_and_partial_tails"]["hard_negative"]==dict(required=4000,frame_count=145,full_cycles=27,partial_tail=85)
    assert min(proof["exposures"]["candidate_indices"])>=13 and min(proof["exposures"]["control_indices"])>=26
    assert all(np.array_equal(original[k],before[k]) for k in original)
    assert proof["original_sampler_schema"]!=proof["schema"] and proof["schema"]==study.SAMPLER_SCHEMA
    assert proof["original_schedule_sha256"]==study.previous.MINING_SHA["proposed_schedule.npz"]
    small=proof["old_vs_new_candidate_actual_stratum_exposures"]["small_sign"]
    negative=proof["old_vs_new_candidate_actual_stratum_exposures"]["hard_negative"]
    assert small["new_candidate_total_exposures"]-small["old_candidate_total_exposures"]==4000
    assert negative["new_candidate_total_exposures"]-negative["old_candidate_total_exposures"]==-4000


def test_old_inventory_and_original_schedule_authenticated_before_new_derivation(monkeypatch,inventory,original):
    calls=[];oldproof=study.mining.schedule_proof(inventory,original)
    monkeypatch.setattr(study,"_old_load",lambda:(calls.append("frozen-original-load") or inventory,original,oldproof))
    value,new,newproof=study.load_inventory()
    assert calls==["frozen-original-load"] and value is inventory
    assert newproof["original_sampler_proof_sha256"]==study.sampler_proof_sha256(oldproof)
    assert not np.array_equal(new["candidate_indices"],original["candidate_indices"])


@pytest.mark.parametrize("kind",["old_candidate","bad_dtype","TRUE","crop_reroll","heldout_index","missing_key"])
def test_new_schedule_cannot_alias_oldproof_or_change_pixels_role_or_crop(inventory,original,schedule,kind):
    changed={k:v.copy() for k,v in schedule.items()}
    if kind=="old_candidate":changed["candidate_indices"]=original["candidate_indices"].copy()
    elif kind=="bad_dtype":changed["candidate_indices"]=changed["candidate_indices"].astype(np.int32)
    elif kind=="TRUE":changed["shared_FALSE_view_bits"][0,0]=1
    elif kind=="crop_reroll":changed["shared_crops"][0,0,0]+=1
    elif kind=="heldout_index":changed["candidate_indices"][0,0]=1209
    else:changed.pop("control_indices")
    with pytest.raises(ValueError):study.schedule_proof(inventory,changed,original)


def test_original_schedule_mutation_is_rejected_before_rederivation(inventory,original):
    changed={k:v.copy() for k,v in original.items()};changed["candidate_indices"][0,4]=1208
    with pytest.raises(ValueError):study.schedules(inventory,changed)


@pytest.mark.parametrize("value",[None,True,1,"A"*64,"a"*63,""])
def test_new_external_proof_commit_required_and_strict(proof,value):
    source={"synthetic":"a"*64};auth=authorization(source,proof);auth["new_sampler_proof_sha256"]=value
    with study.activated():
        with pytest.raises(ValueError):study.validate_authorization(auth,source)


@pytest.mark.parametrize("mutation",["old_schema","role_counts","known_independent","bool_count","fit_scope"])
def test_original_authority_and_scope_guards_not_weakened(proof,mutation):
    source={"synthetic":"b"*64};auth=authorization(source,proof)
    if mutation=="old_schema":auth["schema"]=study.previous.AUTHORIZATION_SCHEMA
    elif mutation=="role_counts":auth["contract"]["candidate_role_exposures"]["small_sign"]=4000
    elif mutation=="known_independent":auth["contract"]["known_evaluation_not_independent"]=False
    elif mutation=="bool_count":auth["contract"]["independent_new_dev_sites"]=False
    else:auth["DEV_or_HOLDOUT_used_for_fit"]=True
    with study.activated():
        with pytest.raises(ValueError):study.validate_authorization(auth,source)


@pytest.mark.parametrize("bad_commit",[False,True])
def test_real_revised_validate_only_chain_checks_commit_before_torch(monkeypatch,capsys,inventory,original,proof,bad_commit):
    source={"synthetic":"d"*64};auth=authorization(source,proof)
    if bad_commit:auth["new_sampler_proof_sha256"]="f"*64
    original_proof=study.mining.schedule_proof(inventory,original)
    monkeypatch.setattr(study,"_old_load",lambda:(inventory,original,original_proof))
    monkeypatch.setattr(study,"environment_contract",lambda:None)
    monkeypatch.setattr(study,"source_snapshot",lambda root:source)
    monkeypatch.setattr(study.contract,"payload",lambda *a:(json.dumps(auth).encode(),"e"*64))
    monkeypatch.setattr(study.mining,"initial_lineage",lambda:None)
    manifest=dict(common_class_pixels=study.mining.CLASS_PIXELS,common_class_weights=study.mining.CLASS_WEIGHTS)
    monkeypatch.setattr(study.contract,"load_package",lambda *a:(manifest,[],None,None,None))
    monkeypatch.setattr(study.previous,"verify_train_identity",lambda *a:None)
    monkeypatch.setitem(sys.modules,"torch",None)
    # HH_261002 - The original global binding must be known, even when synthetic data supplies the same authenticated ABI.
    monkeypatch.setattr(study.previous,"load_inventory",study._old_load)
    with study.activated():
        args=SimpleNamespace(authorization="/synthetic/auth",authorization_sha256="e"*64,validate_only=True)
        if bad_commit:
            with pytest.raises(ValueError,match="before CUDA"):study.revised_run()(args)
        else:study.revised_run()(args)
    if not bad_commit:
        result=json.loads(capsys.readouterr().out)
        assert result["status"]=="CPU_PREFLIGHT_PASS_NO_CUDA_OR_OUTPUT"
        assert result["sampler_proof"]["schema"]==study.SAMPLER_SCHEMA and result["validation_used_during_optimization"] is False


def json_write_new(path,value):
    with path.open("x") as stream:json.dump(value,stream,allow_nan=False)


def test_new_schedule_separate_archive_createonly_and_postcheck(monkeypatch,tmp_path,schedule,proof):
    monkeypatch.setattr(study,"OUTPUT",tmp_path);monkeypatch.setattr(study.mining,"reserve",lambda *a:None)
    study.archive_schedule(schedule,proof,SimpleNamespace(write_new=json_write_new))
    old=tmp_path/"mining";old.mkdir();(old/"proposed_schedule.npz").write_bytes(b"immutable-old-c019-reference")
    before=(old/"proposed_schedule.npz").read_bytes()
    with pytest.raises(FileExistsError):study.archive_schedule(schedule,proof,SimpleNamespace(write_new=json_write_new))
    auth=tmp_path/"authorization.json";json_write_new(auth,dict(new_sampler_proof_sha256=study.sampler_proof_sha256(proof)))
    monkeypatch.setattr(study,"_old_postcheck",lambda *a:None)
    study.postcheck({}, {},auth,study.contract.digest(auth))
    assert (old/"proposed_schedule.npz").read_bytes()==before
    altered=json.loads(auth.read_text());altered["new_sampler_proof_sha256"]="0"*64
    auth.write_text(json.dumps(altered))
    with pytest.raises(ValueError):study.postcheck({}, {},auth,study.contract.digest(auth))


@pytest.mark.parametrize("kind",["reshape","reinterpret","key","proof_ABI"])
def test_raw_byte_hash_without_exact_shape_dtype_cannot_fake_schedule_archive(schedule,proof,kind):
    arrays={k:v.copy() for k,v in schedule.items()};value=copy.deepcopy(proof)
    if kind=="reshape":arrays["control_indices"]=arrays["control_indices"].reshape(8000,4)
    elif kind=="reinterpret":arrays["control_indices"]=arrays["control_indices"].view(np.uint64)
    elif kind=="key":arrays["new"]=arrays.pop("control_indices")
    else:value["tensor_abi"]["control_indices"]["shape"]=[8000,4]
    with pytest.raises(ValueError):study.verify_schedule_tensor_abi(arrays,value)


def test_scoped_provider_restoration_exception_and_nesting():
    names=("NAME","SCHEMA","SOURCE","OUTPUT","SOURCE_NAMES","prepared_contract","source_snapshot","load_inventory",
        "validate_authorization","environment_contract","postcheck")
    saved={k:getattr(study.previous,k) for k in names}
    with pytest.raises(RuntimeError):
        with study.activated():
            assert study.previous.SOURCE==study.SOURCE and study.previous.load_inventory is study.load_inventory
            assert callable(study.revised_run())
            with pytest.raises(ValueError):
                with study.activated():pass
            raise RuntimeError("synthetic")
    assert all(getattr(study.previous,k) is v for k,v in saved.items()) and not study._active


def test_fresh_nine_file_package_import_and_contract_no_torch(tmp_path):
    assert len(study.SOURCE_NAMES)==9
    for name in study.SOURCE_NAMES:
        path=ROOT/("learning/portable_e2e" if name in study.contract.ARCHITECTURE_SHA else "tools/e2e")/name
        shutil.copyfile(path,tmp_path/name)
    code="import sys,pathlib;import train_map_hardnegative_continuation_v2_20261002 as m;assert 'torch' not in sys.modules;assert len(m.source_snapshot(pathlib.Path.cwd()))==9;assert m.prepared_contract()['candidate_role_exposures']['small_sign']==8000;print('PASS9_NO_TORCH')"
    result=subprocess.run([sys.executable,"-B","-c",code],cwd=tmp_path,env={"PATH":"/usr/bin:/bin","PYTHONDONTWRITEBYTECODE":"1"},capture_output=True,text=True,timeout=20)
    assert result.returncode==0,result.stderr
    assert result.stdout.strip()=="PASS9_NO_TORCH"


def test_default_contract_reads_no_actual_inventory_or_arrays(monkeypatch,capsys):
    monkeypatch.setattr(study,"load_inventory",lambda:pytest.fail("actual inventory opened by code contract"))
    assert study.main(["--mode","contract"])==0
    value=json.loads(capsys.readouterr().out)
    assert value["schema"]==study.SCHEMA and not value["training_authorized"]
