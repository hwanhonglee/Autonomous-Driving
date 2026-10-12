#!/usr/bin/env python3
"""HH_261002 - Prepare a separately declared 4global/2small/1large/1negative map continuation.

This hypothesis follows previously exposed known-site results and is exploratory,
not independent validation. Source preparation grants no fit or GPU permission.
The frozen original inventory and c019 schedule are authenticated before a new
schedule is derived, committed by separate authorization and archived distinctly.
"""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import hashlib
import json
from pathlib import Path
from types import FunctionType

import numpy as np

try:
    from . import train_map_hardnegative_continuation_20261002 as previous
except ImportError:
    import train_map_hardnegative_continuation_20261002 as previous

mining, contract, require = previous.mining, previous.contract, previous.require
NAME = "map_hardnegative_continuation_20261002_v2"
SCHEMA = "portable_e2e.map_hardnegative_continuation.20261002.v2"
SAMPLER_SCHEMA = "portable_e2e.map_train_hardnegative_sampler.20261002.v2"
AUTHORIZATION_SCHEMA = SCHEMA+".authorization"
WORKSPACE, DATA, MINING_OUTPUT = previous.WORKSPACE, previous.DATA, previous.MINING_OUTPUT
SOURCE, OUTPUT = WORKSPACE/"source_worktrees"/NAME, WORKSPACE/"runs/diagnostics"/NAME
STEPS, BATCH, SEED, ARMS = previous.STEPS, previous.BATCH, previous.SEED, previous.ARMS
FROZEN_RUNNER_SHA = "0b6261a3bcf474cf7b02f81b6856d6cfd7a0c64b948b3f9d8c48769a24ca5dd3"
FROZEN_SOURCE_SHA = dict(previous.FROZEN_SOURCE_SHA, **{Path(previous.__file__).name:FROZEN_RUNNER_SHA})
SOURCE_NAMES = set(FROZEN_SOURCE_SHA)|{Path(__file__).name}
_old_load, _old_postcheck = previous.load_inventory, previous.postcheck
_old_auth, _old_environment, _old_contract = previous.validate_authorization, previous.environment_contract, previous.prepared_contract
_old_source, _old_schema = previous.SOURCE, previous.SCHEMA
_active = False


def sampler_proof_sha256(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def prepared_contract():
    # HH_261002 - Reuse every old parent/data/optimizer/evaluation number; disclose only the sampler and namespace revision.
    namespace = dict(_old_contract.__globals__,NAME=NAME,SCHEMA=SCHEMA,SOURCE=SOURCE,OUTPUT=OUTPUT)
    result = FunctionType(_old_contract.__code__,namespace,_old_contract.__name__)()
    result.update(sole_intervention="8global versus 4global+2small_sign+1large_sign+1hard_negative full-cycle exposure",
        sampler_schema=SAMPLER_SCHEMA,candidate_slot_roles=["all_TRAIN"]*4+["small_sign"]*2+["large_sign","hard_negative"],
        candidate_role_exposures=dict(all_TRAIN=16000,small_sign=8000,large_sign=4000,hard_negative=4000),
        immutable_original_inventory_and_schedule_authenticated_first=True,
        original_schedule_sha256=previous.MINING_SHA["proposed_schedule.npz"],
        frozen_original_runner_sha256=FROZEN_RUNNER_SHA,
        externally_pinned_new_sampler_proof_sha256_required_before_CUDA=True,
        derived_schedule_archive_is_distinct_from_original_c019=True,
        hypothesis_adapted_after_prior_known_site_diagnostics=True,
        measured_recall_loss_sampler_causality_proven=False,
        additional_small_sign_exposure_does_not_guarantee_recall_gain=True)
    return result


def source_snapshot(root):
    contract.canonical(root)
    pins = {name:contract.digest(root/name) for name in sorted(SOURCE_NAMES)}
    require(all(pins[name]==expected for name,expected in FROZEN_SOURCE_SHA.items()),
        "all frozen V1 runner/miner/model/helper source bytes must remain exact")
    return pins


def schedules(value, original_arrays):
    # HH_261002 - The unchanged frozen proof authenticates the old stored schedule; it is never reinterpreted as v2.
    mining.schedule_proof(value,original_arrays)
    checked = mining.validate_inventory(value)
    require(checked["strata_available"] is True,"all actual TRAIN strata required; no DEV supplementation")
    require(checked["stratum_frame_counts"]==dict(small_sign=391,large_sign=193,hard_negative=145),
        "exact authenticated actual stratum cardinalities required; no alternate inventory")
    candidate = np.empty_like(original_arrays["candidate_indices"])
    candidate[:,:4] = original_arrays["candidate_indices"][:,:4]
    for name,column,count,seed in (("small_sign",4,2,SEED+10),("large_sign",6,1,SEED+11),("hard_negative",7,1,SEED+12)):
        candidate[:,column:column+count] = mining.full_cycle(np.asarray(checked["strata"][name],dtype=np.int64),
            STEPS*count,seed).reshape(STEPS,count)
    return dict(control_indices=original_arrays["control_indices"].copy(),candidate_indices=candidate,
        shared_crops=original_arrays["shared_crops"].copy(),shared_FALSE_view_bits=original_arrays["shared_FALSE_view_bits"].copy())


def schedule_proof(value, arrays, original_arrays):
    expected = schedules(value,original_arrays)
    require(type(arrays) is dict and set(arrays)==set(expected),"exact new four-array schedule ABI required")
    for name,wanted in expected.items():
        got = arrays[name]
        require(isinstance(got,np.ndarray) and got.shape==wanted.shape and got.dtype==wanted.dtype
            and np.array_equal(got,wanted),"new independently declared schedule differs: "+name)
    old = mining.schedule_proof(value,original_arrays)
    new_counts = {arm:np.bincount(arrays[arm+"_indices"].ravel(),minlength=mining.TOTAL) for arm in ARMS}
    census = {}
    for role,ids in value["strata"].items():
        selected = np.asarray(ids,dtype=np.int64)
        prior = np.asarray(old["exposures"]["candidate_indices"],dtype=np.int64)[selected]
        fresh = new_counts["candidate"][selected]
        census[role] = dict(physical_frames=len(ids),old_candidate_total_exposures=int(prior.sum()),
            new_candidate_total_exposures=int(fresh.sum()),old_candidate_mean_exposures=float(prior.mean()),
            new_candidate_mean_exposures=float(fresh.mean()),old_candidate_range=[int(prior.min()),int(prior.max())],
            new_candidate_range=[int(fresh.min()),int(fresh.max())])
    return dict(schema=SAMPLER_SCHEMA,steps_per_arm=STEPS,batch=BATCH,examples_per_arm=STEPS*BATCH,
        physical_TRAIN_slots=1209,candidate_slot_roles=["all_TRAIN"]*4+["small_sign"]*2+["large_sign","hard_negative"],
        candidate_global_exposures=16000,candidate_small_exposures=8000,candidate_large_exposures=4000,
        candidate_hard_negative_exposures=4000,
        full_cycles_and_partial_tails={role:dict(required=n,frame_count=k,full_cycles=n//k,partial_tail=n%k)
            for role,n,k in (("control_global",32000,1209),("candidate_global",16000,1209),
                ("small_sign",8000,391),("large_sign",4000,193),("hard_negative",4000,145))},
        original_inventory_sha256=previous.MINING_SHA["inventory.json"],
        original_schedule_sha256=previous.MINING_SHA["proposed_schedule.npz"],original_sampler_schema=old["schema"],
        original_sampler_proof_sha256=sampler_proof_sha256(old),
        tensor_sha256={k:mining.array_sha(v) for k,v in arrays.items()},
        tensor_abi={k:dict(shape=list(v.shape),dtype=str(v.dtype)) for k,v in arrays.items()},
        exposures={arm+"_indices":counts.tolist() for arm,counts in new_counts.items()},
        old_vs_new_candidate_actual_stratum_exposures=census,
        every_TRAIN_slot_replayed=True,all_input_slots_and_failed_class_counts_retained=True,
        complete_stratum_cycles_and_partial_tails_retained=True,repeats_are_exposures_not_new_data=True,
        control_global_and_both_shared_crops_FALSE_bits_identical_to_original=True,
        sign_dependent_crop_reroll=False,known_DEV_or_HOLDOUT_used_for_fit=False,
        architecture_loss_class_weights_optimizer_unchanged=True,
        initial_model_weights_sha256=mining.INITIAL_MODEL_SHA,
        previously_exposed_known_site_hypothesis_not_independent=True,
        sampler_causality_or_recall_improvement_proven=False,training_authorized=False,model_promoted=False)


def load_inventory():
    # HH_261002 - First run the entire frozen V1 authentication path, including all old source/lineage and c019 byte/proof pins.
    value,original_arrays,original_proof = _old_load()
    require(original_proof==mining.schedule_proof(value,original_arrays),"original completed mining proof changed")
    arrays = schedules(value,original_arrays)
    return value,arrays,schedule_proof(value,arrays,original_arrays)


def validate_authorization(value, source):
    _old_auth(value,source)
    sha = value.get("new_sampler_proof_sha256")
    require(type(sha) is str and len(sha)==64 and all(c in "0123456789abcdef" for c in sha),
        "separate prospective authorization must pin the exact NEW sampler proof")


def environment_contract():
    _old_environment()
    require(Path(__file__).resolve().parent==SOURCE,"new sampler runner escaped the exact private source package")


def verify_schedule_tensor_abi(schedule, proof):
    # HH_261002 - Raw-byte hashes alone cannot distinguish a reshaped or reinterpreted archive; require the declared exact ABI too.
    abi = dict(control_indices=dict(shape=[4000,8],dtype="int64"),candidate_indices=dict(shape=[4000,8],dtype="int64"),
        shared_crops=dict(shape=[4000,8,4],dtype="int16"),shared_FALSE_view_bits=dict(shape=[4000,8],dtype="uint8"))
    require(proof["schema"]==SAMPLER_SCHEMA and proof["tensor_abi"]==abi
        and set(schedule)==set(proof["tensor_sha256"])==set(abi),"new schedule/proof exact shape/dtype ABI required")
    require(all(isinstance(v,np.ndarray) and list(v.shape)==abi[k]["shape"] and str(v.dtype)==abi[k]["dtype"]
        and mining.array_sha(v)==proof["tensor_sha256"][k] for k,v in schedule.items()),"new schedule bytes or ABI differ")


def archive_schedule(schedule, proof, guard):
    # HH_261002 - The original copied mining/proposed_schedule.npz remains c019; the new declaration is a separate create-only artifact.
    verify_schedule_tensor_abi(schedule,proof)
    mining.reserve(OUTPUT)
    with (OUTPUT/"new_declared_schedule.npz").open("xb") as stream: np.savez_compressed(stream,**schedule)
    mining.reserve(OUTPUT); guard.write_new(OUTPUT/"new_declared_schedule_proof.json",proof)
    guard.write_new(OUTPUT/"new_schedule_archive_receipt.json",dict(schema=SCHEMA+".schedule_archive",
        original_schedule_sha256=previous.MINING_SHA["proposed_schedule.npz"],new_sampler_proof_sha256=sampler_proof_sha256(proof),
        new_schedule_npz_sha256=contract.digest(OUTPUT/"new_declared_schedule.npz"),
        new_schedule_proof_file_sha256=contract.digest(OUTPUT/"new_declared_schedule_proof.json"),
        original_mining_artifacts_unmodified=True,training_authorized_by_this_receipt=False))


def postcheck(source, manifest, authorization, authorization_sha):
    _old_postcheck(source,manifest,authorization,authorization_sha)
    receipt = contract.read_json(OUTPUT/"new_schedule_archive_receipt.json",contract.digest(OUTPUT/"new_schedule_archive_receipt.json"))
    require(receipt["schema"]==SCHEMA+".schedule_archive"
        and receipt["original_schedule_sha256"]==previous.MINING_SHA["proposed_schedule.npz"]
        and receipt["original_mining_artifacts_unmodified"] is True
        and receipt["training_authorized_by_this_receipt"] is False,"new schedule archive provenance differs")
    proof = contract.read_json(OUTPUT/"new_declared_schedule_proof.json",receipt["new_schedule_proof_file_sha256"])
    auth = contract.read_json(authorization,authorization_sha)
    require(proof["schema"]==SAMPLER_SCHEMA and sampler_proof_sha256(proof)==auth["new_sampler_proof_sha256"]==receipt["new_sampler_proof_sha256"],
        "external NEW schedule commitment changed during fit")
    arrays = contract.read_npz(OUTPUT/"new_declared_schedule.npz",receipt["new_schedule_npz_sha256"])
    verify_schedule_tensor_abi(arrays,proof)


@contextmanager
def activated():
    global _active
    require(not _active and previous.SOURCE is _old_source and previous.SCHEMA==_old_schema
        and previous.load_inventory is _old_load and previous.prepared_contract is _old_contract,
        "new sampler cannot nest or replace an unknown provider")
    values = dict(NAME=NAME,SCHEMA=SCHEMA,AUTHORIZATION_SCHEMA=AUTHORIZATION_SCHEMA,SOURCE=SOURCE,OUTPUT=OUTPUT,
        SOURCE_NAMES=SOURCE_NAMES,prepared_contract=prepared_contract,source_snapshot=source_snapshot,
        load_inventory=load_inventory,validate_authorization=validate_authorization,environment_contract=environment_contract,postcheck=postcheck)
    saved = {k:getattr(previous,k) for k in values}; _active = True
    try:
        for k,v in values.items(): setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items(): setattr(previous,k,v)
        _active = False


def revised_run():
    # HH_261002 - Preserve the whole frozen fit; add only pre-CUDA proof authorization and pre-fit separate schedule archival.
    require(contract.digest(Path(previous.__file__))==FROZEN_RUNNER_SHA,"frozen V1 runner changed")
    tree = ast.parse(Path(previous.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="run")
    node = copy.deepcopy(original)
    assignments = [n for n in node.body if isinstance(n,ast.Assign) and isinstance(n.value,ast.Call)
        and isinstance(n.value.func,ast.Name) and n.value.func.id=="load_inventory"]
    require(len(assignments)==1,"one frozen pre-CUDA inventory assignment required")
    added = ast.parse('require(json.loads(auth_raw).get("new_sampler_proof_sha256")==sampler_proof_sha256(proof),"exact prospective NEW sampler proof must be externally committed before CUDA")').body[0]
    node.body.insert(node.body.index(assignments[0])+1,added)
    scopes = [n for n in ast.walk(node) if isinstance(n,ast.With) and any(isinstance(v.context_expr,ast.Call)
        and isinstance(v.context_expr.func,ast.Attribute) and v.context_expr.func.attr=="gpu0_lease" for v in n.items)]
    require(len(scopes)==1,"one frozen exclusive GPU0 lease required")
    scope = scopes[0]
    starts = [n for n in scope.body if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
        and isinstance(n.value.func,ast.Attribute) and n.value.func.attr=="write_new"
        and any(isinstance(a,ast.BinOp) and isinstance(a.right,ast.Constant) and a.right.value=="started.json" for a in n.value.args)]
    require(len(starts)==1,"one frozen before-fit start receipt required")
    archive = ast.parse("archive_schedule(schedule,proof,guard)").body[0]
    scope.body.insert(scope.body.index(starts[0])+1,archive)
    revised = copy.deepcopy(node);node.body.remove(added);scope.body.remove(archive)
    require(ast.dump(node,include_attributes=False)==ast.dump(original,include_attributes=False),
        "any optimizer, loss, crop, update, checkpoint, evaluation or frozen source guard statement changed")
    namespace = dict(vars(previous),sampler_proof_sha256=sampler_proof_sha256,archive_schedule=archive_schedule)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised],type_ignores=[])),str(__file__)+":same_fit_new_sampler_commit","exec"),namespace)
    return namespace["run"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","schedule-contract","fit"),default="contract")
    parser.add_argument("--authorization");parser.add_argument("--authorization-sha256")
    parser.add_argument("--validate-only",action="store_true")
    args = parser.parse_args(argv)
    if args.mode=="contract": print(json.dumps(prepared_contract(),indent=2,allow_nan=False)); return 0
    with activated():
        if args.mode=="schedule-contract":
            source = source_snapshot(SOURCE);_,_,proof = load_inventory()
            print(json.dumps(dict(schema=SCHEMA+".prospective_schedule",source_sha256=source,
                new_sampler_proof_sha256=sampler_proof_sha256(proof),sampler_proof=proof,
                no_CUDA_or_optimizer_or_output=True,fit_authorized=False),indent=2,allow_nan=False)); return 0
        require(args.authorization is not None and args.authorization_sha256 is not None,"fit requires separate external V2 authorization")
        revised_run()(args); return 0


if __name__=="__main__": raise SystemExit(main())
