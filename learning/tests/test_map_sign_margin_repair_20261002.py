"""HH_261002 - Synthetic declaration, unchanged-control and guarded loss-only runner tests."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"learning"));sys.path.insert(0,str(ROOT/"tools/e2e"))
import train_map_sign_margin_repair_20261002 as study
from learning.tests import test_map_hardnegative_continuation_20261002 as original_tests


@pytest.fixture(scope="module")
def inventory():
    # HH_261002 - The fixture synthesizes all1209 full49 rows, never actual protected arrays.
    return original_tests.inventory.__wrapped__()


@pytest.fixture(scope="module")
def original(inventory):
    return study.mining.schedules(inventory)


@pytest.fixture(scope="module")
def schedule(inventory,original):
    return study.schedules(inventory,original)


def runtime_descriptor():
    names=("python_executable","numpy","numpy_native","PIL.Image","PIL._imaging","torch","torch._C","torch.lib/libtorch.so")
    return dict(python_version="synthetic",python_implementation="CPython",python_prefix="/synthetic/private/venv",
        numpy="synthetic",pillow="synthetic",torch="synthetic",torch_compiled_CUDA="synthetic",
        provider_files={name:dict(path="/synthetic/providers/"+str(i),bytes=1,sha256="a"*64) for i,name in enumerate(names)})


def authorization(source,proof_sha="b"*64):
    return dict(schema=study.AUTHORIZATION_SCHEMA,source_sha256=source,contract=study.prepared_contract(),
        reviewer="synthetic-reviewer-NOT-ACTUAL-AUTH",research_fit_authorized=True,native_denials_preserved=True,
        DEV_or_HOLDOUT_used_for_fit=False,automatic_model_promotion=False,vehicle_control_approved=False,
        equal_global_proof_sha256=proof_sha,loss_contract_sha256=study.document_sha(study.losses.prepared_contract()),
        runtime_identity=runtime_descriptor())


def test_contract_one_loss_factor_keeps_old_numerical_law_and_false_scopes():
    value=study.prepared_contract();parent=study._old_contract()
    for key in ("initial_model_weights_sha256","dataset_origin_parent_weights_sha256","class_pixels","class_weights",
        "seed","learning_rate","weight_decay","gradient_norm_cap","parameter_count","batch","additional_updates_per_arm",
        "examples_per_arm","total_lineage_updates_per_arm","shared_crop_law","RGB_views","known_evaluation_files","exploratory_screen"):
        assert value[key]==parent[key]
    assert value["initial_model_weights_sha256"].startswith("2284511d")
    assert value["original_weighted_CE_unchanged"] and value["sampler_unchanged"] and value["architecture_unchanged"]
    assert value["semantic_loss_unchanged"] is False and value["loss_contract"]["coefficient"]==.10
    assert value["both_arm_indices_crops_FALSE_bits_byte_identical"] and value["preflight_optimizer_updates"]==0
    assert value["both_arm_strict_CUDA_preflights_before_ANY_optimizer_update"]
    assert value["main_capabilities_approved"]==value["subfunctions_approved"]==0
    assert not value["training_authorized"] and not value["model_promoted"] and not value["vehicle_control_approved"]


def test_equal_global_control_and_candidate_match_original_c019_control_every_draw(inventory,original,schedule):
    proof=study.schedule_proof(inventory,schedule,original);study.verify_schedule(schedule,proof)
    for arm in study.ARMS:assert np.array_equal(schedule[arm+"_indices"],original["control_indices"])
    for key in ("shared_crops","shared_FALSE_view_bits"):assert np.array_equal(schedule[key],original[key])
    assert schedule["candidate_indices"] is not schedule["control_indices"]
    assert min(proof["exposures"]["control_indices"])>0 and sum(proof["exposures"]["control_indices"])==32000
    assert sum(proof["source_group_exposures_each_arm"].values())==32000
    assert proof["original_schedule_sha256"]==study.previous.MINING_SHA["proposed_schedule.npz"]
    assert proof["training_authorized"] is False


@pytest.mark.parametrize("kind",["old_mixture","index","TRUE","crop","dtype","shape","missing"])
def test_schedule_change_rejected_not_treated_as_loss_only(inventory,original,schedule,kind):
    altered={k:v.copy() for k,v in schedule.items()}
    if kind=="old_mixture":altered["candidate_indices"]=original["candidate_indices"].copy()
    elif kind=="index":altered["candidate_indices"][0,0]=1209
    elif kind=="TRUE":altered["shared_FALSE_view_bits"][0,0]=1
    elif kind=="crop":altered["shared_crops"][0,0,0]+=1
    elif kind=="dtype":altered["control_indices"]=altered["control_indices"].astype(np.float64)
    elif kind=="shape":altered["candidate_indices"]=altered["candidate_indices"].reshape(8000,4)
    else:altered.pop("candidate_indices")
    with pytest.raises(ValueError):study.schedule_proof(inventory,altered,original)


def test_full_original_authentication_precedes_new_derivation(monkeypatch,inventory,original):
    called=[]
    def authenticated():
        called.append("frozen-auth");return inventory,original,study.mining.schedule_proof(inventory,original)
    monkeypatch.setattr(study,"_old_load",authenticated)
    _,arrays,proof=study.load_inventory()
    assert called==["frozen-auth"] and proof["schema"]==study.SCHEDULE_SCHEMA
    assert np.array_equal(arrays["candidate_indices"],original["control_indices"])
    monkeypatch.setattr(study,"_old_load",lambda:(_ for _ in ()).throw(ValueError("denied original capture")))
    with pytest.raises(ValueError,match="denied"):study.load_inventory()


@pytest.mark.parametrize("key",["equal_global_proof_sha256","loss_contract_sha256","runtime_identity",
    "research_fit_authorized","native_denials_preserved","DEV_or_HOLDOUT_used_for_fit","automatic_model_promotion",
    "vehicle_control_approved","schema","source_sha256","reviewer"])
def test_exact_external_authorization_required(key):
    source={"test":"c"*64};value=authorization(source)
    with study.activated():
        study.validate_authorization(value,source)
        value[key]=not value[key] if type(value[key]) is bool else None
        with pytest.raises(ValueError):study.validate_authorization(value,source)


@pytest.mark.parametrize("key",["coefficient","positive_reduction","negative_reduction","source_reduction",
    "control","candidate","fractional_threshold_ties","source_groups"])
def test_declared_loss_law_cannot_change(key):
    value=authorization({"test":"c"*64});value["contract"]["loss_contract"][key]=None
    with study.activated(),pytest.raises(ValueError):study.validate_authorization(value,value["source_sha256"])


@pytest.mark.parametrize("kind",["missing_numpy_native","missing_executable","missing_torch_lib","bool_bytes",
    "escape_path","bad_sha","invented_provider","missing_version"])
def test_runtime_provider_contract_typed_and_complete(kind):
    value=runtime_descriptor()
    if kind=="missing_numpy_native":value["provider_files"].pop("numpy_native")
    elif kind=="missing_executable":value["provider_files"].pop("python_executable")
    elif kind=="missing_torch_lib":value["provider_files"].pop("torch.lib/libtorch.so")
    elif kind=="bool_bytes":value["provider_files"]["numpy"]["bytes"]=True
    elif kind=="escape_path":value["provider_files"]["numpy"]["path"]="/synthetic/../escape"
    elif kind=="bad_sha":value["provider_files"]["numpy"]["sha256"]="x"*64
    elif kind=="invented_provider":value["provider_files"]["unknown"]={"path":"/x","bytes":1,"sha256":"a"*64}
    else:value.pop("torch_compiled_CUDA")
    with pytest.raises(ValueError):study.validate_runtime_descriptor(value)


def test_all_ten_flat_sources_import_without_torch_or_actual_arrays(tmp_path):
    assert len(study.SOURCE_NAMES)==10
    for name in study.SOURCE_NAMES:
        directory="learning/portable_e2e" if name in study.contract.ARCHITECTURE_SHA or name==Path(study.losses.__file__).name else "tools/e2e"
        shutil.copyfile(ROOT/directory/name,tmp_path/name)
    code="import sys,pathlib;import train_map_sign_margin_repair_20261002 as m;assert 'torch' not in sys.modules;assert len(m.source_snapshot(pathlib.Path.cwd()))==10;assert m.prepared_contract()['sampler_unchanged'];print('PASS10_NO_TORCH')"
    result=subprocess.run([sys.executable,"-B","-c",code],cwd=tmp_path,env={"PATH":"/usr/bin:/bin","PYTHONDONTWRITEBYTECODE":"1"},capture_output=True,text=True,timeout=20)
    assert result.returncode==0,result.stderr
    assert result.stdout.strip()=="PASS10_NO_TORCH"
    with (tmp_path/Path(study.previous.__file__).name).open("a") as stream:stream.write("\n# changed\n")
    with pytest.raises(ValueError):study.source_snapshot(tmp_path)


def test_original_frozen_ast_inverse_limits_revision_and_preflight_precedes_arm_loop():
    original,revised=study.revised_run_ast()
    calls=[n.func.id for n in ast.walk(revised) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    assert calls.count("arm_loss")==calls.count("both_cuda_preflights")==calls.count("verify_runtime_authorization")==1
    assert "all_gradients_finite" in calls and "archive_declaration" in calls
    before=ast.dump(original,include_attributes=False)
    assert "AdamW" in before and "evaluate_endpoints" in before and "allow_tf32" in before
    gpu=next(n for n in ast.walk(revised) if isinstance(n,ast.With) and any(isinstance(i.context_expr,ast.Call)
        and isinstance(i.context_expr.func,ast.Attribute) and i.context_expr.func.attr=="gpu0_lease" for i in n.items))
    first=next(i for i,n in enumerate(gpu.body) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
        and isinstance(n.value.func,ast.Name) and n.value.func.id=="both_cuda_preflights")
    arm_loop=next(i for i,n in enumerate(gpu.body) if isinstance(n,ast.For)
        and isinstance(n.target,ast.Name) and n.target.id=="arm"
        and isinstance(n.iter,ast.Name) and n.iter.id=="ARMS")
    assert first<arm_loop


def test_scoped_activation_restores_even_error_and_rejects_nesting():
    before=dict(vars(study.previous))
    with pytest.raises(RuntimeError):
        with study.activated():
            assert study.previous.prepared_contract is study.prepared_contract
            assert study.previous.SCHEMA==study.SCHEMA
            with pytest.raises(ValueError):
                with study.activated():pass
            raise RuntimeError("synthetic failure")
    for key in before:assert getattr(study.previous,key) is before[key]
    assert study._active is False and study._last_loss_stats is None


def evaluation():
    value=original_tests.evaluation()
    value["dev_town01"]["candidate"]["recall"]["traffic_sign"]=.85
    value["dev_town01"]["candidate"]["iou"]["traffic_sign"]=.85
    return value


def test_all_original_twenty_plus_seven_strict_screens_conjunctive():
    value=evaluation();result=study.diagnostic_screen(value)
    assert len(result["checks"])==27 and result["original_twenty_checks"]==study._old_screen(value)["checks"]
    assert result["passed_exploratory_only"] and not result["model_promoted"] and not result["independent_validation"]


@pytest.mark.parametrize("kind",["recall_tie","iou_tie","precision_tie","fp_91pct","town02_retention","road_retention"])
def test_any_new_or_retained_failure_rejects_without_posthoc_selection(kind):
    value=evaluation();candidate=value["dev_town01"]["candidate"]
    if kind=="recall_tie":candidate["recall"]["traffic_sign"]=.8
    elif kind=="iou_tie":candidate["iou"]["traffic_sign"]=.8
    elif kind=="precision_tie":candidate["precision"]["traffic_sign"]=.5
    elif kind=="fp_91pct":candidate["other_to_sign_false_positive_pixels"]=91
    elif kind=="town02_retention":value["holdout_town02_p02"]["candidate"]["iou"]["traffic_sign"]=.789
    else:candidate["iou"]["road"]=.789
    assert not study.diagnostic_screen(value)["passed_exploratory_only"]


def test_runtime_mismatch_rejects_before_cuda_or_any_model(monkeypatch):
    monkeypatch.setattr(study,"runtime_identity",lambda:runtime_descriptor())
    value=authorization({"x":"a"*64});value["runtime_identity"]["numpy"]="not_the_actual_runtime"
    with pytest.raises(ValueError,match="providers differ"):study.verify_runtime_authorization(value,SimpleNamespace())


def test_real_revised_validate_only_api_checks_runtime_and_does_not_import_cuda(monkeypatch,capsys,inventory,original,schedule):
    proof=study.schedule_proof(inventory,schedule,original);source={"test":"d"*64}
    auth=authorization(source,study.document_sha(proof));order=[]
    monkeypatch.setattr(study,"environment_contract",lambda:order.append("environment"))
    monkeypatch.setattr(study,"source_snapshot",lambda root:source)
    monkeypatch.setattr(study.contract,"payload",lambda *a:(json.dumps(auth).encode(),"e"*64))
    monkeypatch.setattr(study,"load_inventory",lambda:(inventory,schedule,proof))
    monkeypatch.setattr(study.mining,"initial_lineage",lambda:None)
    manifest=dict(common_class_pixels=study.mining.CLASS_PIXELS,common_class_weights=study.mining.CLASS_WEIGHTS)
    monkeypatch.setattr(study.contract,"load_package",lambda *a:(manifest,[],None,None,None))
    monkeypatch.setattr(study.previous,"verify_train_identity",lambda *a:order.append("actual_TRAIN_identity"))
    monkeypatch.setattr(study,"verify_runtime_authorization",lambda *a:order.append("runtime"))
    monkeypatch.setitem(sys.modules,"torch",None)
    with study.activated():study.revised_run()(SimpleNamespace(authorization="/synthetic/auth",authorization_sha256="e"*64,validate_only=True))
    value=json.loads(capsys.readouterr().out)
    assert value["status"]=="CPU_PREFLIGHT_PASS_NO_CUDA_OR_OUTPUT"
    assert "runtime" in order and "actual_TRAIN_identity" in order
    assert value["sampler_proof"]["schema"]==study.SCHEDULE_SCHEMA and value["loss_contract"]["coefficient"]==.10


def test_direct_control_CE_returns_identical_scalar_and_does_not_compute_aux(monkeypatch):
    import torch
    calls=[];logits=torch.zeros(8,7,2,2,requires_grad=True);target=torch.zeros(8,2,2,dtype=torch.int64);weights=torch.ones(7)
    expected=logits.sum()*0+3.
    def ce(*args):
        assert args==(logits,target,weights);calls.append("original_CE");return expected
    monkeypatch.setattr(study.losses,"candidate_loss",lambda *a:pytest.fail("control computed candidate auxiliary"))
    assert study.arm_loss("control",logits,target,weights,np.arange(8,dtype=np.int64),SimpleNamespace(semantic_loss=ce)) is expected
    assert calls==["original_CE"]


def test_candidate_preserves_actual_FALSE_source_group_identities(monkeypatch):
    import torch
    ids=np.asarray([0,496,497,728,729,968,969,1208],dtype=np.int64)
    expected=[study.mining.source_group(int(i)) for i in ids];called=[]
    def loss(logits,target,weights,groups,ce):
        assert groups==expected;called.append(True);return torch.tensor(1.),dict(auxiliary=.5)
    monkeypatch.setattr(study.losses,"candidate_loss",loss)
    study.arm_loss("candidate",None,None,None,ids,SimpleNamespace(semantic_loss=lambda *a:None))
    assert called==[True] and study._last_loss_stats["arm"]=="candidate"


@pytest.mark.parametrize("kind",["unknown_arm","bool_ids","heldout","shape"])
def test_arm_dispatch_never_accepts_foreign_or_boolean_ids(kind):
    ids=np.arange(8,dtype=np.int64);arm="candidate"
    if kind=="unknown_arm":arm="best"
    elif kind=="bool_ids":ids=ids.astype(bool)
    elif kind=="heldout":ids[0]=1209
    else:ids=ids.reshape(2,4)
    with pytest.raises(ValueError):study.arm_loss(arm,None,None,None,ids,None)


def test_all_parameter_guard_rejects_none_and_nonfinite_even_not_clipped():
    import torch
    p=torch.nn.Parameter(torch.zeros(245831));model=SimpleNamespace(parameters=lambda:[p])
    assert not study.all_gradients_finite(model)
    p.grad=torch.zeros_like(p);assert study.all_gradients_finite(model)
    p.grad[-1]=float("nan");assert not study.all_gradients_finite(model)
    p.grad[-1]=float("inf");assert not study.all_gradients_finite(model)
    p.grad.zero_();p.requires_grad_(False);assert not study.all_gradients_finite(model)


def test_both_preflights_precede_any_training_and_abort_unsupported_candidate(monkeypatch):
    import torch
    calls=[]
    class Model:
        def parameters(self):return [SimpleNamespace(numel=lambda:245831)]
        def to(self,device):assert device=="cuda:0";return self
        def train(self):return self
    monkeypatch.setattr(study.previous.previous,"load_weights",lambda *a:calls.append("load228"))
    monkeypatch.setattr(torch.cuda,"empty_cache",lambda:calls.append("empty_cache"))
    def preflight(arm,*args):
        calls.append(arm)
        if arm=="candidate":raise RuntimeError("strict deterministic CUDA unsupported")
    monkeypatch.setattr(study,"strict_cuda_preflight",preflight)
    guard=SimpleNamespace(seed_everything=lambda value:calls.append(("seed",value)),state_digest=lambda model:study.previous.INITIAL_STATE_SHA)
    with pytest.raises(RuntimeError,match="unsupported"):
        study.both_cuda_preflights(None,None,None,SimpleNamespace(MapContextRecovery=Model),None,guard)
    assert calls.count("control")==calls.count("candidate")==1 and calls.count("empty_cache")==2
    assert calls.index("control")<calls.index("candidate") and "optimizer_update" not in calls


def test_default_contract_no_auth_grant_or_payload_access(monkeypatch,capsys):
    monkeypatch.setattr(study,"runtime_identity",lambda:pytest.fail("default accessed runtime providers"))
    monkeypatch.setattr(study,"load_inventory",lambda:pytest.fail("default opened actual TRAIN"))
    assert study.main(["--mode","contract"])==0
    value=json.loads(capsys.readouterr().out)
    assert value["training_authorized"] is False and value["loss_contract"]["training_authorized"] is False


@pytest.mark.parametrize("kind",["NaN","Inf","float64_logits","float_target","negative_label","foreign_label",
    "bad_target_shape","bad_channels","empty_batch","non_tensor","boolean_group"])
def test_auxiliary_full_input_guards_no_masked_nonfinite_exception(kind):
    import torch
    logits=torch.zeros(1,7,2,2);target=torch.zeros(1,2,2,dtype=torch.int64)
    groups=[study.losses.SOURCE_GROUPS[0]]
    if kind in ("NaN","Inf"):logits[0,6,1,1]=float("nan" if kind=="NaN" else "inf")
    elif kind=="float64_logits":logits=logits.to(torch.float64)
    elif kind=="float_target":target=target.to(torch.float32)
    elif kind=="negative_label":target[0,0,0]=-1
    elif kind=="foreign_label":target[0,0,0]=7
    elif kind=="bad_target_shape":target=target.reshape(1,1,4)
    elif kind=="bad_channels":logits=logits[:,:6]
    elif kind=="empty_batch":logits=logits[:0];target=target[:0];groups=[]
    elif kind=="non_tensor":logits=np.zeros((1,7,2,2),dtype=np.float32)
    else:groups=[True]
    with pytest.raises(ValueError):study.losses.sign_margin_auxiliary(logits,target,groups)


@pytest.mark.parametrize("kind",["NaN","zero","negative","float64","wrong_count","not_callable"])
def test_candidate_original_class_weight_ABI_must_be_finite_positive(kind):
    import torch
    logits=torch.zeros(1,7,2,2);target=torch.zeros(1,2,2,dtype=torch.int64);weights=torch.ones(7)
    ce=lambda *a:pytest.fail("bad weights reached original CE")
    if kind=="NaN":weights[0]=float("nan")
    elif kind=="zero":weights[0]=0
    elif kind=="negative":weights[0]=-1
    elif kind=="float64":weights=weights.to(torch.float64)
    elif kind=="wrong_count":weights=weights[:6]
    else:ce=None
    with pytest.raises(ValueError):study.losses.candidate_loss(logits,target,weights,[study.losses.SOURCE_GROUPS[0]],ce)


def test_no_label_or_logit_rewrite_and_finite_gradients():
    import torch
    from portable_e2e.map_perception import semantic_loss
    logits=torch.linspace(-2,2,112,dtype=torch.float32).reshape(2,7,2,4).requires_grad_()
    target=torch.tensor([[[5,0,1,2],[3,4,5,6]],[[0,0,1,2],[3,4,6,6]]],dtype=torch.int64)
    weights=torch.tensor(study.mining.CLASS_WEIGHTS);before_logits=logits.detach().clone();before_target=target.clone()
    combined,_=study.losses.candidate_loss(logits,target,weights,list(study.losses.SOURCE_GROUPS[:2]),semantic_loss)
    combined.backward()
    assert torch.isfinite(logits.grad).all()
    assert torch.equal(logits.detach(),before_logits) and torch.equal(target,before_target)


def test_strict_CUDA_guard_fails_without_relaxing_or_touching_inputs(monkeypatch):
    import torch
    monkeypatch.setattr(torch,"are_deterministic_algorithms_enabled",lambda:False)
    with pytest.raises(ValueError,match="strict deterministic CUDA"):
        study.strict_cuda_preflight("candidate",None,None,None,None,None,None)


def test_strict_CUDA_preflight_rejects_CPU_parameters_before_any_batch_read(monkeypatch):
    import torch
    monkeypatch.setattr(torch,"are_deterministic_algorithms_enabled",lambda:True)
    monkeypatch.setattr(torch,"is_deterministic_algorithms_warn_only_enabled",lambda:False,raising=False)
    monkeypatch.setattr(torch.backends.cuda.matmul,"allow_tf32",False)
    monkeypatch.setattr(torch.backends.cudnn,"allow_tf32",False)
    model=SimpleNamespace(parameters=lambda:[torch.nn.Parameter(torch.zeros(245831))])
    with pytest.raises(ValueError,match="logicalCUDA0"):
        study.strict_cuda_preflight("candidate",model,None,None,None,None,None)
