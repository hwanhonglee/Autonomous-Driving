#!/usr/bin/env python3
"""HH_261002 - Prepare a fixed all-global map A/B with one sign-margin loss change.

Preparation grants no GPU or fit permission. Both arms replay the original
all-global control schedule from the same current228 parent. The candidate only
adds the declared auxiliary; all old data denials, CE, crop, optimizer, fixed
endpoints and twenty exploratory checks remain. Previously exposed known-site
diagnostics are not independent validation or automatic model promotion.
"""
from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import hashlib
import importlib
import json
from pathlib import Path
import platform
import sys
from types import FunctionType

import numpy as np

try:
    from . import train_map_hardnegative_continuation_20261002 as previous
except ImportError:
    import train_map_hardnegative_continuation_20261002 as previous
try:
    from portable_e2e import map_sign_margin_loss_v1_20261002 as losses
except ImportError:
    import map_sign_margin_loss_v1_20261002 as losses

mining, contract, require = previous.mining, previous.contract, previous.require
NAME = "map_sign_margin_repair_20261002_v1"
SCHEMA = "portable_e2e.map_sign_margin_repair.20261002.v1"
SCHEDULE_SCHEMA, AUTHORIZATION_SCHEMA = SCHEMA+".equal_global_schedule", SCHEMA+".authorization"
WORKSPACE, DATA, MINING_OUTPUT = previous.WORKSPACE, previous.DATA, previous.MINING_OUTPUT
SOURCE, OUTPUT = WORKSPACE/"source_worktrees"/NAME, WORKSPACE/"runs/diagnostics"/NAME
STEPS, BATCH, SEED, ARMS = previous.STEPS, previous.BATCH, previous.SEED, previous.ARMS
FROZEN_RUNNER_SHA = "0b6261a3bcf474cf7b02f81b6856d6cfd7a0c64b948b3f9d8c48769a24ca5dd3"
FROZEN_SOURCE_SHA = dict(previous.FROZEN_SOURCE_SHA, **{Path(previous.__file__).name:FROZEN_RUNNER_SHA})
SOURCE_NAMES = set(FROZEN_SOURCE_SHA)|{Path(__file__).name, Path(losses.__file__).name}
_old_load, _old_postcheck = previous.load_inventory, previous.postcheck
_old_auth, _old_environment = previous.validate_authorization, previous.environment_contract
_old_contract, _old_screen = previous.prepared_contract, previous.diagnostic_screen
_old_source, _old_schema = previous.SOURCE, previous.SCHEMA
_active, _last_loss_stats = False, None


def document_sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def _sha(value):
    return type(value) is str and len(value)==64 and all(c in "0123456789abcdef" for c in value)


def prepared_contract():
    # HH_261002 - Only loss and experiment namespace change; the old current-parent training law is retained.
    namespace = dict(_old_contract.__globals__,NAME=NAME,SCHEMA=SCHEMA,SOURCE=SOURCE,OUTPUT=OUTPUT)
    result = FunctionType(_old_contract.__code__,namespace,_old_contract.__name__)()
    result.update(sole_intervention="original_CE versus original_CE+0.10_source_positive_image_sign_margin",
        semantic_loss_unchanged=False, original_weighted_CE_unchanged=True, sampler_unchanged=True,
        loss_contract=losses.prepared_contract(), loss_contract_sha256=document_sha(losses.prepared_contract()),
        both_arm_slot_roles=["all_TRAIN"]*BATCH, both_arm_all_global_exposures=STEPS*BATCH,
        both_arm_indices_crops_FALSE_bits_byte_identical=True,
        immutable_original_inventory_and_c019_schedule_authenticated_first=True,
        original_schedule_sha256=previous.MINING_SHA["proposed_schedule.npz"],
        frozen_original_runner_sha256=FROZEN_RUNNER_SHA,
        new_equal_global_proof_external_SHA_required_before_CUDA=True,
        current_runtime_binary_provider_pins_external_before_and_after=True,
        historical_runtime_binary_identity_claimed=False,
        strict_CUDA_forward_backward_preflight_before_first_update=True,
        all_245831_parameter_gradients_present_and_finite_each_update=True,
        preflight_exposures_per_arm=BATCH, preflight_optimizer_updates=0,
        both_arm_strict_CUDA_preflights_before_ANY_optimizer_update=True,
        preflight_extra_TRAIN_only_diagnostic_exposures_not_in_32000_optimizer_exposures=True,
        current_runtime_threads=dict(intraop=16,interop=1), strict_deterministic_algorithms=True,
        CUDA_TF32_matrix_and_cudnn=False, failed_deterministic_preflight_aborts_not_retried=True,
        hypothesis_adapted_after_prior_known_site_diagnostics=True,
        strict_added_acceptance=dict(Town01_sign_precision_gain_vs_both=True,
            Town01_sign_recall_gain_vs_both=True,Town01_sign_IoU_gain_vs_both=True,
            Town01_other_to_sign_FP_max_ratio_vs_parent=.90, all_original_twenty_checks_required=True),
        sampler_loss_causality_or_improvement_proven=False)
    return result


def source_snapshot(root):
    contract.canonical(root)
    source = {name:contract.digest(root/name) for name in sorted(SOURCE_NAMES)}
    require(all(source[name]==sha for name,sha in FROZEN_SOURCE_SHA.items()),
        "all eight original runner/miner/model/helper sources must remain exact")
    return source


def schedules(value, original):
    # HH_261002 - Authenticate c019 first; neither its old mixture nor any new sign sampler is reinterpreted.
    mining.schedule_proof(value,original)
    return dict(control_indices=original["control_indices"].copy(),
        candidate_indices=original["control_indices"].copy(),
        shared_crops=original["shared_crops"].copy(),shared_FALSE_view_bits=original["shared_FALSE_view_bits"].copy())


def schedule_proof(value, arrays, original):
    wanted = schedules(value,original)
    require(type(arrays) is dict and set(arrays)==set(wanted), "four exact equal-global arrays required")
    for name, expected in wanted.items():
        got=arrays[name]
        require(isinstance(got,np.ndarray) and got.shape==expected.shape and got.dtype==expected.dtype
            and np.array_equal(got,expected), "equal-global schedule differs: "+name)
    old = mining.schedule_proof(value,original)
    counts = {arm:np.bincount(arrays[arm+"_indices"].ravel(),minlength=mining.TOTAL) for arm in ARMS}
    require(np.array_equal(counts["control"],counts["candidate"]) and min(counts["control"])>0,
        "all1209 slots and both full32000 denominators required")
    groups = {group:int(sum(counts["control"][i] for i in range(mining.TOTAL)
        if mining.source_group(i)==group)) for group in losses.SOURCE_GROUPS}
    return dict(schema=SCHEDULE_SCHEMA, steps_per_arm=STEPS,batch=BATCH,examples_per_arm=STEPS*BATCH,
        physical_TRAIN_slots=1209, both_arm_slot_roles=["all_TRAIN"]*BATCH,
        original_inventory_sha256=previous.MINING_SHA["inventory.json"],
        original_schedule_sha256=previous.MINING_SHA["proposed_schedule.npz"],
        original_sampler_proof_sha256=document_sha(old), initial_model_weights_sha256=mining.INITIAL_MODEL_SHA,
        tensor_sha256={k:mining.array_sha(v) for k,v in arrays.items()},
        tensor_abi={k:dict(shape=list(v.shape),dtype=str(v.dtype)) for k,v in arrays.items()},
        exposures={arm+"_indices":v.tolist() for arm,v in counts.items()},source_group_exposures_each_arm=groups,
        shared_crops_and_FALSE_bits_identical_to_c019=True, no_sign_dependent_crop_reroll=True,
        all1209_TRUE_views_unused_for_fit=True, known_DEV_or_HOLDOUT_used_for_fit=False,
        repeats_are_exposures_not_new_data=True, training_authorized=False,model_promoted=False)


def load_inventory():
    value,original,old_proof = _old_load()
    require(old_proof==mining.schedule_proof(value,original),"original mining proof changed")
    arrays=schedules(value,original)
    return value,arrays,schedule_proof(value,arrays,original)


def verify_schedule(schedule, proof):
    abi=dict(control_indices=dict(shape=[4000,8],dtype="int64"),candidate_indices=dict(shape=[4000,8],dtype="int64"),
        shared_crops=dict(shape=[4000,8,4],dtype="int16"),shared_FALSE_view_bits=dict(shape=[4000,8],dtype="uint8"))
    require(proof.get("schema")==SCHEDULE_SCHEMA and proof.get("tensor_abi")==abi
        and set(schedule)==set(proof["tensor_sha256"])==set(abi),"equal-global schedule exact ABI required")
    require(all(isinstance(v,np.ndarray) and list(v.shape)==abi[k]["shape"] and str(v.dtype)==abi[k]["dtype"]
        and mining.array_sha(v)==proof["tensor_sha256"][k] for k,v in schedule.items()),"schedule bytes or ABI differ")
    require(np.array_equal(schedule["control_indices"],schedule["candidate_indices"])
        and not bool(schedule["shared_FALSE_view_bits"].any()),"both arms must be the identical original global/FALSE schedule")


def runtime_identity():
    # HH_261002 - Pin the current actual executable/native providers, not an unverifiable historical binary claim.
    import torch
    import PIL
    from PIL import Image, _imaging
    numpy_core = importlib.import_module("numpy._core._multiarray_umath" if int(np.__version__.split(".")[0])>=2
        else "numpy.core._multiarray_umath")
    files={"python_executable":Path(sys.executable).resolve(),"numpy":Path(np.__file__).resolve(),
        "numpy_native":Path(numpy_core.__file__).resolve(),"PIL.Image":Path(Image.__file__).resolve(),
        "PIL._imaging":Path(_imaging.__file__).resolve(),"torch":Path(torch.__file__).resolve(),
        "torch._C":Path(torch._C.__file__).resolve()}
    libraries=sorted((Path(torch.__file__).resolve().parent/"lib").glob("*.so"))
    require(0<len(libraries)<=32 and sum(p.stat().st_size for p in libraries)<=8*1024**3,
        "bounded actual Torch native math provider closure required")
    for path in libraries: files["torch.lib/"+path.name]=path.resolve()
    return dict(python_version=platform.python_version(),python_implementation=platform.python_implementation(),
        python_prefix=str(Path(sys.prefix)), numpy=np.__version__,pillow=PIL.__version__,torch=torch.__version__,
        torch_compiled_CUDA=torch.version.cuda,
        provider_files={key:dict(path=str(path),bytes=path.stat().st_size,sha256=contract.digest(path))
            for key,path in sorted(files.items())})


def validate_runtime_descriptor(value):
    required={"python_executable","numpy","numpy_native","PIL.Image","PIL._imaging","torch","torch._C"}
    require(type(value) is dict and set(value)=={"python_version","python_implementation","python_prefix",
        "numpy","pillow","torch","torch_compiled_CUDA","provider_files"},"full current-runtime identity required")
    require(all(type(value[k]) is str and value[k] for k in set(value)-{"provider_files"}),"explicit runtime versions/prefix required")
    files=value["provider_files"]
    require(type(files) is dict and required < set(files) and len(files)<=len(required)+32
        and all(key in required or (type(key) is str and key.startswith("torch.lib/")
            and Path(key).name.endswith(".so") and ".." not in Path(key).parts) for key in files),
        "exact executable/NumPy/Pillow/Torch native identities required")
    for row in files.values():
        require(type(row) is dict and set(row)=={"path","bytes","sha256"} and type(row["path"]) is str
            and type(row["bytes"]) is int and row["bytes"]>0 and _sha(row["sha256"]),"typed runtime provider pin required")
        contract.canonical(Path(row["path"]))


def validate_authorization(value, source):
    _old_auth(value,source)
    require(_sha(value.get("equal_global_proof_sha256"))
        and value.get("loss_contract_sha256")==document_sha(losses.prepared_contract()),
        "external equal-global proof and exact fixed loss commitment required")
    validate_runtime_descriptor(value.get("runtime_identity"))


def verify_runtime_authorization(value, torch=None):
    if torch is None:
        import torch
    identity=runtime_identity()
    require(identity==value["runtime_identity"],"current executable/crop/Torch/native providers differ from external declaration")
    for module in (previous,mining,contract,contract.frozen,previous.previous,losses):
        require(Path(module.__file__).resolve()==SOURCE/Path(module.__file__).name,"imported source escaped pinned flat package")
    require(torch.cuda.is_initialized() is False,"no CUDA initialization before authenticated exclusive lease")
    return identity


def environment_contract():
    _old_environment()
    require(Path(__file__).resolve().parent==SOURCE and Path(losses.__file__).resolve()==SOURCE/Path(losses.__file__).name,
        "new runner/loss escaped exact private source package")


def archive_declaration(schedule,proof,guard):
    verify_schedule(schedule,proof);mining.reserve(OUTPUT)
    with (OUTPUT/"equal_global_schedule.npz").open("xb") as stream:np.savez_compressed(stream,**schedule)
    guard.write_new(OUTPUT/"equal_global_schedule_proof.json",proof)
    guard.write_new(OUTPUT/"loss_contract.json",losses.prepared_contract())
    guard.write_new(OUTPUT/"intervention_archive_receipt.json",dict(schema=SCHEMA+".archive",
        schedule_npz_sha256=contract.digest(OUTPUT/"equal_global_schedule.npz"),
        schedule_proof_file_sha256=contract.digest(OUTPUT/"equal_global_schedule_proof.json"),
        equal_global_proof_sha256=document_sha(proof),loss_contract_sha256=document_sha(losses.prepared_contract()),
        loss_contract_file_sha256=contract.digest(OUTPUT/"loss_contract.json"),
        original_c019_unmodified=True, training_authorized_by_this_receipt=False))


def arm_loss(arm,logits,target,weights,ids,perception):
    global _last_loss_stats
    require(arm in ARMS and isinstance(ids,np.ndarray) and ids.dtype==np.int64
        and ids.shape==(BATCH,) and bool(((ids>=0)&(ids<mining.TOTAL)).all()),"exact arm/TRAIN slot IDs required")
    if arm=="control":
        loss=perception.semantic_loss(logits,target,weights)
        _last_loss_stats=dict(arm=arm,original_CE=float(loss.detach()),auxiliary=0.,coefficient=0.,total_loss=float(loss.detach()))
        return loss
    groups=[mining.source_group(int(i)) for i in ids]
    loss,stats=losses.candidate_loss(logits,target,weights,groups,perception.semantic_loss)
    _last_loss_stats=dict(arm=arm,**stats)
    return loss


def all_gradients_finite(model):
    import torch
    parameters=list(model.parameters())
    return sum(p.numel() for p in parameters)==245831 and all(p.requires_grad and p.grad is not None
        and bool(torch.isfinite(p.grad).all()) for p in parameters)


def strict_cuda_preflight(arm,model,arrays,schedule,weights,perception,guard):
    # HH_261002 - The extra first-batch diagnostic makes zero updates and restores all RNG/state before the identical fit.
    import torch
    require(torch.are_deterministic_algorithms_enabled() and not torch.is_deterministic_algorithms_warn_only_enabled()
        and torch.backends.cuda.matmul.allow_tf32 is False and torch.backends.cudnn.allow_tf32 is False,
        "strict deterministic CUDA with both TF32 paths disabled required")
    require(all(p.device.type=="cuda" and p.device.index==0 and p.dtype==torch.float32 for p in model.parameters()),
        "all original model parameters must use logicalCUDA0 float32")
    before=guard.state_digest(model);cpu_rng=torch.get_rng_state();cuda_rng=torch.cuda.get_rng_state(0)
    ids,boxes,bits=schedule[arm+"_indices"][0],schedule["shared_crops"][0],schedule["shared_FALSE_view_bits"][0]
    try:
        rgb,target,actual_bits=previous.previous.batch_arrays(arrays,ids,boxes,bits,"control")
        require(np.array_equal(actual_bits,bits),"preflight actual FALSE/crop inputs differ")
        x=torch.tensor(rgb.transpose(0,3,1,2).copy(),dtype=torch.float32,device="cuda:0")/255
        y=torch.tensor(target.astype(np.int64),device="cuda:0")
        model.zero_grad(set_to_none=True);logits=model(x);loss=arm_loss(arm,logits,y,weights,ids,perception)
        require(bool(torch.isfinite(logits).all()) and bool(torch.isfinite(loss)),"nonfinite preflight forward")
        loss.backward();torch.cuda.synchronize()
        require(all_gradients_finite(model),"all245831 preflight gradients must exist and be finite")
        require(guard.state_digest(model)==before==previous.INITIAL_STATE_SHA,"preflight changed original current228 state")
        receipt=dict(schema=SCHEMA+".strict_CUDA_preflight",arm=arm,status="STRICT_CUDA_FORWARD_BACKWARD_PASS_ZERO_UPDATES",
            parameter_count=245831,all_parameter_gradients_present_and_finite=True,loss_stats=_last_loss_stats,
            actual_TRAIN_slot_indices=ids.tolist(),shared_crop_boxes=boxes.tolist(),actual_paired_TRUE_bits=actual_bits.tolist(),
            rgb_array_sha256=mining.array_sha(rgb),target_array_sha256=mining.array_sha(target),
            initial_state_sha256=before,optimizer_updates=0,all_arm_optimizer_updates_so_far=0,
            completed_fit_arms=[],extra_TRAIN_diagnostic_exposures=BATCH,
            known_DEV_or_HOLDOUT_opened=False,strict_deterministic_algorithms=True,TF32=False)
        guard.write_new(OUTPUT/(arm+"_strict_CUDA_preflight.json"),receipt)
    except Exception as error:
        guard.write_new(OUTPUT/(arm+"_strict_CUDA_preflight_failure.json"),dict(schema=SCHEMA+".strict_CUDA_preflight_failure",
            arm=arm,exception_type=type(error).__name__,exception_message=str(error),optimizer_updates=0,
            all_arm_optimizer_updates_so_far=0,completed_fit_arms=[],
            deterministic_guard_not_relaxed=True,known_DEV_or_HOLDOUT_opened=False))
        raise
    finally:
        model.zero_grad(set_to_none=True);torch.set_rng_state(cpu_rng);torch.cuda.set_rng_state(cuda_rng,0)


def both_cuda_preflights(arrays,schedule,weights,architecture,perception,guard):
    # HH_261002 - Fail an unsupported candidate before spending even the unchanged control's first update.
    import torch
    for arm in ARMS:
        guard.seed_everything(SEED);model=architecture.MapContextRecovery()
        previous.previous.load_weights(model,OUTPUT/"initial_model_weights.npz",mining.INITIAL_MODEL_SHA)
        require(guard.state_digest(model)==previous.INITIAL_STATE_SHA
            and sum(p.numel() for p in model.parameters())==245831,"same current228 preflight state required")
        model.to("cuda:0").train()
        try:
            strict_cuda_preflight(arm,model,arrays,schedule,weights,perception,guard)
        finally:
            del model;torch.cuda.empty_cache()


def diagnostic_screen(evaluation):
    original=_old_screen(evaluation)
    require(len(original["checks"])==20,"all original twenty exploratory checks required")
    checks=dict(original["checks"]);town=evaluation["dev_town01"]
    candidate=town["candidate"]
    for comparator in ("parent","control"):
        for metric,key in (("precision","precision"),("recall","recall"),("IoU","iou")):
            a,b=town[comparator][key]["traffic_sign"],candidate[key]["traffic_sign"]
            checks["ADDED_Town01_sign_"+metric+"_strict_gain_vs_"+comparator]=a is not None and b is not None and b>a
    checks["ADDED_Town01_other_to_sign_FP_at_most_90pct_parent"]=candidate["other_to_sign_false_positive_pixels"]<=.9*town["parent"]["other_to_sign_false_positive_pixels"]
    return dict(checks=checks,original_twenty_checks=original["checks"],added_check_count=7,
        passed_exploratory_only=all(checks.values()),independent_validation=False,
        model_promoted=False,vehicle_control_approved=False)


def postcheck(source,manifest,authorization,authorization_sha):
    _old_postcheck(source,manifest,authorization,authorization_sha)
    auth=contract.read_json(authorization,authorization_sha)
    receipt=contract.read_json(OUTPUT/"intervention_archive_receipt.json",contract.digest(OUTPUT/"intervention_archive_receipt.json"))
    require(receipt["schema"]==SCHEMA+".archive" and receipt["original_c019_unmodified"] is True
        and receipt["training_authorized_by_this_receipt"] is False,"intervention archive provenance differs")
    proof=contract.read_json(OUTPUT/"equal_global_schedule_proof.json",receipt["schedule_proof_file_sha256"])
    schedule=contract.read_npz(OUTPUT/"equal_global_schedule.npz",receipt["schedule_npz_sha256"])
    verify_schedule(schedule,proof)
    require(document_sha(proof)==auth["equal_global_proof_sha256"]==receipt["equal_global_proof_sha256"],"equal-global commitment changed")
    loss_contract=contract.read_json(OUTPUT/"loss_contract.json",receipt["loss_contract_file_sha256"])
    require(loss_contract==losses.prepared_contract() and document_sha(loss_contract)==auth["loss_contract_sha256"]==receipt["loss_contract_sha256"],
        "fixed loss commitment changed")
    require(runtime_identity()==auth["runtime_identity"],"current binary/crop/math providers changed during fit/evaluation")
    for arm in ARMS:
        witness=contract.read_json(OUTPUT/(arm+"_strict_CUDA_preflight.json"),contract.digest(OUTPUT/(arm+"_strict_CUDA_preflight.json")))
        require(witness["status"]=="STRICT_CUDA_FORWARD_BACKWARD_PASS_ZERO_UPDATES" and witness["arm"]==arm
            and witness["all_parameter_gradients_present_and_finite"] is True and witness["optimizer_updates"]==0,
            "both strict CUDA zero-update preflight receipts required")


@contextmanager
def activated():
    global _active, _last_loss_stats
    require(not _active and previous.SOURCE is _old_source and previous.SCHEMA==_old_schema
        and previous.load_inventory is _old_load and previous.prepared_contract is _old_contract,
        "cannot nest or replace an unknown original fit provider")
    values=dict(NAME=NAME,SCHEMA=SCHEMA,AUTHORIZATION_SCHEMA=AUTHORIZATION_SCHEMA,SOURCE=SOURCE,OUTPUT=OUTPUT,
        SOURCE_NAMES=SOURCE_NAMES,prepared_contract=prepared_contract,source_snapshot=source_snapshot,
        load_inventory=load_inventory,validate_authorization=validate_authorization,environment_contract=environment_contract,
        diagnostic_screen=diagnostic_screen,postcheck=postcheck)
    saved={key:getattr(previous,key) for key in values};_active=True;_last_loss_stats=None
    try:
        for key,value in values.items():setattr(previous,key,value)
        yield
    finally:
        for key,value in saved.items():setattr(previous,key,value)
        _active=False;_last_loss_stats=None


def revised_run_ast():
    # HH_261002 - An inverse AST proof limits edits to loss dispatch and additive declaration/finite-gradient guards.
    require(contract.digest(Path(previous.__file__))==FROZEN_RUNNER_SHA,"frozen V1 runner changed")
    tree=ast.parse(Path(previous.__file__).read_bytes())
    original=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="run")
    node=copy.deepcopy(original);undo=[]
    def after_unique(predicate,text):
        matches=[(parent,i) for parent in ast.walk(node) for field in ast.iter_fields(parent)
            if type(field[1]) is list for i,child in enumerate(field[1]) if predicate(child)]
        require(len(matches)==1,"one frozen insertion target required")
        parent,index=matches[0]
        body=next(value for _,value in ast.iter_fields(parent) if type(value) is list and len(value)>index and predicate(value[index]))
        added=ast.parse(text).body;body[index+1:index+1]=added;undo.append((body,added))
    after_unique(lambda n:isinstance(n,ast.Assign) and isinstance(n.value,ast.Call)
        and isinstance(n.value.func,ast.Name) and n.value.func.id=="load_inventory",
        'require(json.loads(auth_raw)["equal_global_proof_sha256"]==document_sha(proof),"external equal-global proof mismatch before CUDA")\nverify_runtime_authorization(json.loads(auth_raw))')
    after_unique(lambda n:isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
        and isinstance(n.value.func,ast.Attribute) and n.value.func.attr=="write_new"
        and any(isinstance(a,ast.BinOp) and isinstance(a.right,ast.Constant) and a.right.value=="started.json" for a in n.value.args),
        'archive_declaration(schedule,proof,guard)')
    # HH_261002 - Both authentic first-batch backwards precede the original arm loop and every AdamW update.
    after_unique(lambda n:isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="weights" for t in n.targets),
        'both_cuda_preflights(arrays,schedule,weights,architecture,perception,guard)')
    replacements=[]; status_replacements=[]
    for parent in ast.walk(node):
        for field,value in ast.iter_fields(parent):
            if value=="FIXED_CURRENT_PARENT_SAMPLER_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED" and isinstance(parent,ast.Constant):
                status_replacements.append((parent,field,value))
                setattr(parent,field,"FIXED_CURRENT_PARENT_SIGN_MARGIN_LOSS_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED")
            if not isinstance(value,ast.Call):continue
            if isinstance(value.func,ast.Attribute) and isinstance(value.func.value,ast.Name) and value.func.value.id=="perception" and value.func.attr=="semantic_loss":
                replacement=ast.parse('arm_loss(arm,model(x),y,weights,ids,perception)',mode="eval").body
                replacements.append((parent,field,value));setattr(parent,field,replacement)
            elif isinstance(value.func,ast.Name) and value.func.id=="require" and any(isinstance(a,ast.Constant)
                    and a.value=="nonfinite parameter gradient" for a in value.args):
                replacement=ast.parse('require(all_gradients_finite(model),"all245831 parameter gradients must exist and be finite")',mode="eval").body
                replacements.append((parent,field,value));setattr(parent,field,replacement)
    require(len(replacements)==2 and len(status_replacements)==1,"one original CE, gradient guard and truthful status label required")
    revised=copy.deepcopy(node)
    for parent,field,value in replacements:setattr(parent,field,value)
    for parent,field,value in status_replacements:setattr(parent,field,value)
    for body,added in reversed(undo):
        for child in added:body.remove(child)
    require(ast.dump(node,include_attributes=False)==ast.dump(original,include_attributes=False),
        "an original crop/model/optimizer/update/checkpoint/evaluation/source/data statement changed")
    return original,revised


def revised_run():
    _,node=revised_run_ast()
    namespace=dict(vars(previous),document_sha=document_sha,verify_runtime_authorization=verify_runtime_authorization,
        archive_declaration=archive_declaration,strict_cuda_preflight=strict_cuda_preflight,
        both_cuda_preflights=both_cuda_preflights,arm_loss=arm_loss,all_gradients_finite=all_gradients_finite)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(__file__)+":loss_only_revision","exec"),namespace)
    return namespace["run"]


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","runtime-contract","schedule-contract","fit"),default="contract")
    parser.add_argument("--authorization");parser.add_argument("--authorization-sha256")
    parser.add_argument("--validate-only",action="store_true");args=parser.parse_args(argv)
    if args.mode=="contract":print(json.dumps(prepared_contract(),indent=2,allow_nan=False));return 0
    if args.mode=="runtime-contract":print(json.dumps(runtime_identity(),indent=2,allow_nan=False));return 0
    with activated():
        if args.mode=="schedule-contract":
            source=source_snapshot(SOURCE);_,_,proof=load_inventory()
            print(json.dumps(dict(schema=SCHEMA+".prospective_schedule",source_sha256=source,
                equal_global_proof_sha256=document_sha(proof),sampler_proof=proof,
                loss_contract_sha256=document_sha(losses.prepared_contract()),
                no_CUDA_or_optimizer_or_output=True,fit_authorized=False),indent=2,allow_nan=False));return 0
        require(args.authorization is not None and args.authorization_sha256 is not None,"fit requires separate exact external authorization")
        revised_run()(args);return 0


if __name__=="__main__":raise SystemExit(main())
