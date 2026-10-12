"""HH_261008 - Prospective TRAIN-label crop counterfactual, never training or model evaluation.

Reproduce ALL original20k/B8 baseline hashes using the frozen label-only crop
AST. Even GLOBAL exposure ordinals alone may rescue genuinely positive targets
whose original NEAREST crop lost every sign pixel. Original sample IDs, random
proposals, FALSE-only scope, labels and all unprotected/negative boxes stay exact.
Original V2 per-slot baseline statistics remain a sealed referenced artifact;
the replay independently computes and compares them but does not duplicate them
in the new result. Candidate hashes are NEW, not historical byte equivalence.
No RGB, TRUE, DEV, HOLDOUT, TEST, weights, denied RAW, NN or optimizer is opened.
All actual work requires separate RAW ROOT plan/grant and the original lease.
512MiB/900s guards are sampled, not OS containment or hard preemption.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import struct
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve()
NAME="map_train_crop_preservation_counterfactual_v1"
SCHEMA="portable_e2e.map_train_crop_preservation_counterfactual.20261008.v1"
PLAN_DIR=ROOT/"storage/experiments/plans/2026-10-08"/NAME
OUTPUT=ROOT/"storage/experiments/diagnostics/2026-10-08"/NAME
PLAN=PLAN_DIR/"ROOT_plan.json";AUTHORITY=PLAN_DIR/"ROOT_authority.json"
SKELETON=PLAN_DIR/"PLAN_skeleton.json";AUTHORITY_SKELETON=PLAN_DIR/"ROOT_authority.skeleton.json"
TEST=ROOT/"learning/tests/test_map_train_crop_preservation_counterfactual_20261008_v1.py"
OLD=ROOT/"tools/e2e/audit_map_native_fusion_crop_support_20261008_v2.py"
POLICY=ROOT/"tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py"
POLICY_TEST=ROOT/"learning/tests/test_map_train_sign_preserving_crop_policy_20261008_v1.py"
PARENT_DIR=ROOT/"storage/experiments/plans/2026-10-08/map_native_fusion_crop_support_v2"
PARENT_OUTPUT=ROOT/"storage/experiments/diagnostics/2026-10-08/map_native_fusion_crop_support_v2"
REFERENCE_PINS={
    str(OLD):"0f6cae13380221070dd485d095b54d84c7cb7740b810d6616cfcbf862558e0dd",
    str(ROOT/"learning/tests/test_map_native_fusion_crop_support_20261008_v2.py"):"d44bbc7d6310cd4b0d40bee129180e31b7a9a8e394714d286212af0a0d9469b4",
    str(PARENT_DIR/"ROOT_plan.json"):"424263297373dd2308faa0a43ad77e69e35a8db8126c91044b8a43645202d88e",
    str(PARENT_DIR/"ROOT_authority.json"):"b11d3b6b0441881aa80e931d7e0e94c233d93a7a666eaf4d205edfab63dc9ac2",
    str(PARENT_OUTPUT/"result.json"):"a2c5f75ba70f8f4efe49041390bcc0ac84a683e803afd331ef43113a09f4f596",
    str(ROOT/"storage/experiments/diagnostics/2026-10-08/map_native_fusion_crop_support_v2_independent_postcheck_v1.json"):"84a941ab80d680437936cbb71456323db67b7d99503960147e3fe22ffa4f4c69"}
POLICY_PINS={str(POLICY):"566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f",
    str(POLICY_TEST):"0202372ce00c486936bfe74bce38a9b9561f81be25aa0d6ef8e4d7d6b3b81f0a"}
STEPS,BATCH,TOTAL,PIXELS=20000,8,1209,360*640
LIMIT,CLOSING,RSS,DEADLINE=2*1024**2,512*1024,512*1024**2,900
ROOT_METADATA_RESERVE=64*1024
BRANCH_COLUMNS=("protected_exposures","unprotected_exposures","full_GT_negative_exposures",
    "rescued_exposures","unchanged_box_exposures","rescued_original_raw_crop_cut_all_sign",
    "rescued_original_NEAREST_zero_from_positive_crop")


def require(value,message):
    if not value:raise ValueError(message)


def canonical(path,exists=True):
    path=Path(path);require(path.is_absolute() and path==path.resolve(strict=exists)
        and not any(p.is_symlink() for p in (path,*path.parents)),"canonical nonsymlink required")
    if exists:require(path.is_file(),"regular file required")
    return path


def digest(path,checkpoint=lambda:None):
    path=canonical(path);a=path.stat();require(a.st_nlink==1,"single-link input required");state=hashlib.sha256()
    with path.open("rb") as stream:
        for raw in iter(lambda:stream.read(1024**2),b""):checkpoint();state.update(raw)
    b=path.stat();require(all(getattr(a,k)==getattr(b,k) for k in ("st_dev","st_ino","st_size","st_mtime_ns","st_ctime_ns")),"input changed during SHA")
    return state.hexdigest()


def encoded(value):return (json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n").encode()


def read_json(path,pin=None):
    path=canonical(path);require(path.stat().st_size<=CLOSING,"metadata cap512KiB")
    raw=path.read_bytes();require(pin is None or hashlib.sha256(raw).hexdigest()==pin,"RAW metadata SHA differs")
    def pairs(items):
        value={}
        for k,v in items:require(k not in value,"duplicate JSON key");value[k]=v
        return value
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=lambda _:require(False,"nonfinite JSON"))


def source_module(path,pin):
    require(sys.flags.dont_write_bytecode and sys.flags.optimize==0,"assertions and -B required before any original source import")
    require(digest(path)==pin,"literal scientific/policy source differs")
    spec=importlib.util.spec_from_file_location("_hh_train_crop_"+Path(path).stem,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    require(not any(n in sys.modules for n in ("numpy","PIL","torch","carla","rclpy")),"source bootstrap imported runtime")
    return module


def modules():
    return source_module(OLD,REFERENCE_PINS[str(OLD)]),source_module(POLICY,POLICY_PINS[str(POLICY)])


def fixed(policy):
    return dict(schema=SCHEMA,policy=policy.contract(),baseline_steps=STEPS,candidate_steps=STEPS,batch=BATCH,
        original_arm_total_exposures=320000,shared_native_exposures=160000,physical_TRAIN_slots=1209,
        baseline_all20000_B8_hashes_required=True,baseline_per_slot_statistics_recomputed_and_compared_to_frozen_V2=True,
        candidate_hashes_new_not_historical=True,baseline_statistics_output_reference_not_duplicate=True,
        candidate_counters="original18 plus fixed7 branch counts; per4groups/per1209slots/all7classes",
        original_crop_AST_unchanged=True,all_sample_indices_random_proposals_FALSE_bits_unchanged=True,
        maximum_combined_PLAN_OUTPUT_bytes=LIMIT,closing_bytes=CLOSING,CPU_affinity=[3],threads=1,
        sampled_VmHWM_bytes=RSS,deadline_seconds=DEADLINE,sampled_not_OS_containment=True,
        floor_bytes=2*1024**3,reserve_bytes=64*1024**2,old_authority_used_as_new_permission=False,
        RGB_TRUE_DEV_HOLDOUT_TEST_weights_predictions_deniedRAW_bodies_opened=False,NN=0,training_steps=0,
        model_or_data_registry_changed=False,model_capability_or_causality_proven=False,main_approved=0,sub_approved=0)


def tables(old):
    for path,pin in REFERENCE_PINS.items():require(digest(path)==pin,"frozen V2 scientific/actual evidence changed")
    parent=read_json(PARENT_DIR/"ROOT_plan.json",REFERENCE_PINS[str(PARENT_DIR/"ROOT_plan.json")])
    result=read_json(PARENT_OUTPUT/"result.json",REFERENCE_PINS[str(PARENT_OUTPUT/"result.json")])
    require(parent["source_sha256"]==old.sources() and parent["input_sha256"]==old.input_pins(),"whole original current source/input table differs")
    require(result["status"]=="COMPLETE_ACTUAL_TRAIN_LABEL_SUPPORT_ONLY" and result["label_byte_equivalence_proven"] is True
        and result["by_arm"]["control"]["PASS_batches"]==result["by_arm"]["candidate"]["PASS_batches"]==STEPS,
        "actual whole20k baseline required")
    old_archives=old.source_archives(parent)
    require({str(p) for p in (PARENT_DIR/"sources_by_SHA256").iterdir()}==set(old_archives),"exact old unique archive census differs")
    for path,pin in old_archives.items():require(digest(path)==pin,"old actual source archive changed")
    source=dict(parent["source_sha256"])
    for path in (HERE,TEST,POLICY,POLICY_TEST,SKELETON,AUTHORITY_SKELETON):source[str(path)]=digest(path)
    require(all(source[path]==pin for path,pin in POLICY_PINS.items()),"fixed parity protection source changed")
    fresh={path:pin for path,pin in source.items() if path not in parent["source_sha256"]}
    require(len(fresh)==6,"exact six fresh own source/test/skeletons required")
    return dict(source_sha256=source,input_sha256={**parent["input_sha256"],**REFERENCE_PINS},
        old_source_archives=old_archives,new_source_sha256=fresh),result


def own_archives(plan):return {str(PLAN_DIR/"sources_by_SHA256"/(pin+".py")):pin for pin in set(plan["new_source_sha256"].values())}


def manifest(plan,plan_sha,grant_sha):
    return {**plan["source_sha256"],**plan["input_sha256"],**plan["old_source_archives"],**own_archives(plan),
        str(PLAN):plan_sha,str(AUTHORITY):grant_sha,str(PLAN_DIR/"candidate_plan.json"):plan_sha,
        str(OUTPUT/PLAN.name):plan_sha,str(OUTPUT/AUTHORITY.name):grant_sha}


def used():
    total=0
    for root in (PLAN_DIR,OUTPUT):
        canonical(root,False)
        if not root.exists():continue
        for path in root.rglob("*"):
            require(path==path.resolve(strict=True) and not path.is_symlink() and (path.is_file() or path.is_dir()),"owned output alias/type denied")
            if path.is_file():require(path.stat().st_nlink==1,"owned hardlink denied");total+=path.stat().st_size
    return total


def write_new(path,raw,closing=False):
    require(type(raw) is bytes and len(raw)<=CLOSING and any(path.is_relative_to(root) for root in (PLAN_DIR,OUTPUT)),"bounded own bytes required")
    canonical(path,False);require(used()+len(raw)<=LIMIT-(0 if closing else CLOSING),"unchanged combined2MiB quota exceeded")
    with path.open("xb") as stream:require(stream.write(raw)==len(raw),"partial write")


def empty_candidate(old):
    columns=[*old.COUNTER_COLUMNS,*BRANCH_COLUMNS]
    return dict(columns=columns,overall=[0]*len(columns),source_groups={g:[0]*len(columns) for g in old.GROUPS},
        slot_statistics=[[0]*len(columns) for _ in range(TOTAL)],partial_statistics_cover_PASS_batches_only=True)


def branch_counts(choice,original_source_sign):
    rescued=choice["rescued"]
    return [int(choice["protected"]),int(not choice["protected"]),int(choice["full_GT_negative"]),int(rescued),
        int(not rescued),int(rescued and original_source_sign==0),int(rescued and original_source_sign>0)]


def commit_candidate(old,stats,slots,values):
    require(len(slots)==len(values)==BATCH,"whole B8 candidate transactional commit required")
    result=dict(stats,overall=list(stats["overall"]),source_groups={g:list(v) for g,v in stats["source_groups"].items()},slot_statistics=list(stats["slot_statistics"]))
    for slot in set(slots):result["slot_statistics"][slot]=list(stats["slot_statistics"][slot])
    for slot,row in zip(slots,values):
        require(type(slot) is int and 0<=slot<TOTAL and len(row)==len(stats["columns"]) and all(type(v) is int and v>=0 for v in row),"complete typed candidate counters")
        require(stats["columns"]==[*old.COUNTER_COLUMNS,*BRANCH_COLUMNS] and row[0]==1
            and all(v<=1 for v in row[:8]+row[18:]) and all(v<=PIXELS for v in row[8:18])
            and sum(row[11:18])==PIXELS and row[18]+row[19]==row[21]+row[22]==1
            and row[23]+row[24]==row[21],"exact normal single-exposure counter bounds required")
        for destination in (result["overall"],result["source_groups"][old.source_group(slot)],result["slot_statistics"][slot]):
            for i,value in enumerate(row):destination[i]+=value
    return result


def candidate_partition_checks(stats,baseline):
    pairs=[(stats["overall"],baseline["shared_once_overall"])]
    pairs.extend((stats["source_groups"][key],row) for key,row in baseline["source_groups"].items())
    require(len(stats["slot_statistics"])==len(baseline["slot_statistics"])==TOTAL,"full1209 candidate slot table required")
    pairs.extend(zip(stats["slot_statistics"],baseline["slot_statistics"]))
    for candidate,original in pairs:
        require(len(candidate)==25 and len(original)==18 and all(type(n) is int and n>=0 for n in candidate+original),"typed complete candidate/baseline counters required")
        protected,unprotected,negative,rescued,unchanged,raw_rescued,resize_rescued=candidate[18:]
        require(candidate[0]==original[0]==protected+unprotected==rescued+unchanged
            and negative==original[0]-original[3] and raw_rescued+resize_rescued==rescued<=protected,
            "full protected/unprotected/negative/rescue partition differs")
        require(candidate[1]==original[1]+rescued and candidate[2]==original[2]-rescued
            and candidate[3]==original[3] and candidate[4]==original[4]-rescued
            and candidate[5]==original[5]-raw_rescued and candidate[6]==original[6]-resize_rescued,
            "full-FOV/positive/zero/raw-cut/resize-loss rescue identities differ")
        require(candidate[8]==original[8] and candidate[9]>=original[9] and candidate[10]>=original[10]
            and sum(candidate[11:18])==sum(original[11:18])==candidate[0]*PIXELS,
            "same whole-frame source and seven-class exposure support denominator differs")


def own_guard(old,budget,quota=True):
    budget.guard()
    # HH_261008 - There are no per-exposure file writes: scan own quota per B8 and before every write, not redundant160k trees.
    if quota:require(used()<=LIMIT-CLOSING,"own combined2MiB working/closing reserve denied")
    fs=os.statvfs(OUTPUT.parent)
    require(fs.f_bavail*fs.f_frsize>=2*1024**3+64*1024**2,"own filesystem2GiB floor plus64MiB reserve denied")


def result_value(old,plan,plan_sha,grant_sha,passed,failed,failure,stats,native,base_chain,candidate_chain,box_chain,bitmap,seals,memory,generation,elapsed,baseline_equal):
    complete=failure is None and passed==STEPS and failed==0 and not seals and baseline_equal
    return dict(schema=SCHEMA+".result",status="COMPLETE_TRAIN_CROP_COUNTERFACTUAL_NOT_TRAINING_APPROVAL" if complete else "FAILED_WITH_FULL_ORIGINAL_DENOMINATOR",
        **old.denominator(passed,failed),candidate_statistics=stats,baseline_statistics_reference=dict(path=str(PARENT_OUTPUT/"result.json"),sha256=REFERENCE_PINS[str(PARENT_OUTPUT/"result.json")],field="statistics"),
        baseline_all20k_batch_hashes_and_every_original_slot_counter_equal=baseline_equal,
        native_target_support=native,baseline_B8_target_chain_sha256=base_chain,candidate_NEW_B8_target_chain_sha256=candidate_chain,
        candidate_NEW_box_chain_sha256=box_chain,verified_batch_bitmap_hex=bitmap,protection="global_exposure_ordinal % 2 == 0",
        failure_tail=failure,closing_seal_errors=seals,observed_closing_memory=memory,current_PID_generation=generation,
        ROOT_plan_sha256=plan_sha,ROOT_authority_sha256=grant_sha,elapsed_seconds_until_result_write=elapsed,
        exact_seal_maps_in_archived_ROOT_plan=str(OUTPUT/PLAN.name),source_count=len(plan["source_sha256"]),input_count=len(plan["input_sha256"]),
        old_archives_reused_read_only=True,whole_original_source_input_archive_unchanged=not seals,
        RGB_TRUE_DEV_HOLDOUT_TEST_weights_predictions_deniedRAW_bodies_opened=False,
        NN=0,training_steps=0,model_performance_improved=False,main_approved=0,sub_approved=0,
        final_result_write_after_last_resource_sample=True,OS_containment_or_hard_preemption_claimed=False)


def closing_proof(old,plan):
    exposures=plan["evidence"]["journal"]["slot_exposures"]
    require(len(exposures)==TOTAL and all(type(n) is int and 132<=n<=133 for n in exposures) and sum(exposures)==160000,"full original slot exposure bounds required")
    stats=empty_candidate(old);maximum=STEPS*BATCH*PIXELS
    # Exact normal counter bounds: first8 and new7 are exposure counts; original10 remaining fields count pixels.
    def row(n):return [n]*8+[n*PIXELS]*10+[n]*7
    stats["overall"]=row(160000);stats["source_groups"]={g:row(160000) for g in old.GROUPS};stats["slot_statistics"]=[row(133) for _ in range(TOTAL)]
    errors={path:dict(expected="f"*64,actual=None,error=dict(repr_SHA256="f"*64,bytes=2**64-1,prefix_hex="f"*64)) for path in manifest(plan,"f"*64,"f"*64)}
    native=dict(sign_pixels_by_slot=[PIXELS]*TOTAL,class_pixels=[maximum]*7,target_hash_chain_sha256="f"*64,completed_slots=TOTAL,
        failed_slot=TOTAL-1,UNEXECUTED_slots=0,status="x"*64,failed_row_witness=dict(observed_ordinal=TOTAL-1,raw_jsonl_row_sha256="f"*64,row_type="dict",declared_index="x"*old.LINE_LIMIT,declared_index_type="x"*64,actual_target_sha256="f"*64,expected_target_sha256="f"*64),
        extra_row_witness=dict(observed_ordinal=2**64-1,raw_jsonl_row_sha256="f"*64,row_type="dict",declared_index="x"*old.LINE_LIMIT,declared_index_type="x"*64),missing_tail_witness=dict(first_missing_ordinal=TOTAL,UNEXECUTED_slots=TOTAL,observed_end_of_jsonl=True))
    partial=dict(ordinal=STEPS-1,examined_exposures=BATCH,physical_slot_indices=[TOTAL-1]*BATCH,shared_crop_boxes=[[0,0,360,640]]*BATCH,
        selected_boxes=[[0,0,360,640]]*BATCH,baseline_actual_target_sha256="f"*64,baseline_expected_target_sha256="f"*64,candidate_actual_target_sha256="f"*64,
        baseline_support=[[maximum]*18]*BATCH,candidate_support=[[maximum]*25]*BATCH)
    memory_max=dict(PID=2**64-1,VmRSS_bytes=2**64-1,VmHWM_bytes=2**64-1,inherited_ru_maxrss_bytes_NOT_GATE=2**64-1)
    failure=dict(stage="x"*64,error=old.bounded_error(ValueError("x"*old.LINE_LIMIT)),failed_batch=partial,resources=memory_max,closing_resource_error=old.bounded_error(ValueError("x"*old.LINE_LIMIT)))
    generation_max=dict(plan["prepare_process_generation"])
    for key in ("PID","PPID","process_group","session","start_ticks"):generation_max[key]=2**64-1
    value=result_value(old,plan,"f"*64,"f"*64,STEPS-1,1,failure,stats,native,"f"*64,"f"*64,"f"*64,"f"*(STEPS//4),errors,
        dict(memory_max,maximum_observed_VmHWM_bytes=2**64-1),generation_max,-1.7976931348623157e308,False)
    size=len(encoded(value));require(size<=CLOSING,"full populated1209/320k failure envelope exceeds512KiB; deny before pixels")
    return size


def authority_value(plan,plan_sha):
    return dict(schema=SCHEMA+".ROOT_authority",plan_path=str(PLAN),plan_sha256=plan_sha,source_sha256=plan["source_sha256"],
        TRAIN_target_counterfactual_replay_authorized=True,training_authorized=False,NN_authorized=False,
        RGB_TRUE_DEV_HOLDOUT_TEST_denied_RAW_authorized=False,model_promoted=False)


def storage_admission(old,plan,prospective=False):
    closing_proof(old,plan);raw=encoded(plan);grant=encoded(authority_value(plan,"f"*64))
    if prospective:
        require(not OUTPUT.exists() and PLAN_DIR.is_dir() and not PLAN_DIR.is_symlink()
            and {str(path) for path in PLAN_DIR.iterdir()}=={str(SKELETON),str(AUTHORITY_SKELETON)},
            "only the two sealed source-only skeletons may precede fresh preparation; no retry")
        sizes={pin:canonical(path).stat().st_size for path,pin in plan["new_source_sha256"].items()}
        needed=used()+sum(sizes.values())+3*len(raw)+2*len(grant)+ROOT_METADATA_RESERVE+CLOSING
    else:needed=used()+canonical(PLAN).stat().st_size+canonical(AUTHORITY).stat().st_size+CLOSING
    require(needed<=LIMIT,"full prospective combined2MiB reserve does not fit; no quota expansion")
    return needed


def verify(old,plan,budget=None,unloaded=True):
    checkpoint=budget.guard if budget else lambda:None
    require(old.environment(unloaded=unloaded)==plan["environment"],"selected original provider/environment differs")
    for table in (plan["source_sha256"],plan["input_sha256"],plan["old_source_archives"],own_archives(plan)):
        for path,pin in table.items():require(digest(path,checkpoint)==pin,"complete source/input/current/archive seal differs")
    require({str(p) for p in (PLAN_DIR/"sources_by_SHA256").iterdir()}==set(own_archives(plan)),"exact fresh unique archive census differs")
    require(used()<=LIMIT-CLOSING,"combined evidence reserve exceeded")


def prepare():
    old,policy=modules()
    with old.lease() as lock_id:
        budget=old.Budget();own_guard(old,budget);env=old.environment();table,baseline=tables(old)
        for path,pin in table["input_sha256"].items():require(digest(path,budget.guard)==pin,"whole original input differs")
        plan=dict(schema=SCHEMA,fixed=fixed(policy),**table,evidence=old.evidence(budget.guard),environment=env,
            lock_identity=lock_id,prepare_process_generation=old.process_generation(),baseline_result_SHA=REFERENCE_PINS[str(PARENT_OUTPUT/"result.json")])
        storage_admission(old,plan,True);archive=PLAN_DIR/"sources_by_SHA256";archive.mkdir(exist_ok=False)
        for path,pin in plan["new_source_sha256"].items():
            raw=canonical(path).read_bytes();require(hashlib.sha256(raw).hexdigest()==pin,"fresh source changed before copy")
            destination=archive/(pin+".py")
            if destination.exists():require(digest(destination)==pin,"fresh duplicate digest differs")
            else:write_new(destination,raw)
        write_new(PLAN_DIR/"candidate_plan.json",encoded(plan))
        return dict(status="CANDIDATE_ONLY_NOT_ROOT_AUTHORITY",sha256=digest(PLAN_DIR/"candidate_plan.json"),pixels_decoded=0,NN=0,training_steps=0)


def admission(plan_sha,grant_sha):
    require(all(type(value) is str and re.fullmatch("[0-9a-f]{64}",value) for value in (plan_sha,grant_sha)),"external RAW ROOT hashes required")
    plan=read_json(PLAN,plan_sha);grant=read_json(AUTHORITY,grant_sha)
    require(encoded(grant)==encoded(authority_value(plan,plan_sha)) and plan["schema"]==SCHEMA,"fresh exact typed ROOT authority required")
    require(read_json(PLAN_DIR/"candidate_plan.json")==plan and digest(PLAN_DIR/"candidate_plan.json")==plan_sha,"RAW candidate bytes differ")
    old,policy=modules();require(encoded(plan["fixed"])==encoded(fixed(policy)),"fixed typed parity protection contract differs")
    table,baseline=tables(old);require(all(plan[k]==value for k,value in table.items()),"all original/new current seal tables differ")
    return old,policy,plan,baseline


def execute(plan_sha,grant_sha):
    old,policy,plan,baseline=admission(plan_sha,grant_sha)
    require(not OUTPUT.exists(),"fresh actual result only; no resume/retry")
    with old.lease() as lock_id:
        require(lock_id==plan["lock_identity"],"original runtime lease generation differs")
        budget=old.Budget();generation=old.process_generation();own_guard(old,budget);storage_admission(old,plan);OUTPUT.mkdir(parents=True,exist_ok=False)
        passed=failed=0;failure=None;current=None;stage="admission";targets=None;native=None;baseline_equal=False
        base_stats=old.empty_statistics();stats=empty_candidate(old);base_chain=hashlib.sha256();candidate_chain=hashlib.sha256();box_chain=hashlib.sha256();bitmap=bytearray((STEPS+7)//8)
        try:
            for path,pin in ((PLAN,plan_sha),(AUTHORITY,grant_sha)):
                raw=canonical(path).read_bytes();require(hashlib.sha256(raw).hexdigest()==pin,"RAW ROOT evidence changed");write_new(OUTPUT/path.name,raw)
            own_guard(old,budget);verify(old,plan,budget);require(old.evidence(budget.guard)==plan["evidence"],"original full chronology differs")
            os.sched_setaffinity(0,{3});require(os.sched_getaffinity(0)=={3},"single CPU3 not applied");budget.strict_affinity=True;budget.guard()
            # HH_261008 - No array/image import is reachable before fresh ROOT admission, whole seals, lease and full storage proof.
            import numpy as np
            from PIL import Image
            require(str(Path(np.__file__).resolve(strict=True))==plan["environment"]["origins"]["numpy"]
                and str(Path(sys.modules["PIL"].__file__).resolve(strict=True))==plan["environment"]["origins"]["PIL"],"actual provider origin differs")
            crop=old.compile_target_crop(np,Image);stage="original_TRAIN_target_members";targets=old.load_targets(np,plan["evidence"]["archive_metadata"],budget)
            stage="native1209";native=old.native_progress();signs,pixels,native_chain=old.native_support(np,targets,budget,native)
            require(pixels==plan["evidence"]["source_class_pixels"],"native class contribution changed")
            for ordinal,row in enumerate(old.json_lines(old.RUN/"control/exposures.jsonl")):
                old.validate_batch(row,ordinal);require(ordinal<STEPS,"extra batch denied");stage="original_batch";own_guard(old,budget)
                current=dict(ordinal=ordinal,physical_slot_indices=row["physical_slot_indices"],shared_crop_boxes=row["shared_crop_boxes"],selected_boxes=[],
                    baseline_expected_target_sha256=row["target_sha256"],baseline_actual_target_sha256=None,candidate_actual_target_sha256=None,
                    examined_exposures=0,baseline_support=[],candidate_support=[])
                base_hash=hashlib.sha256();cand_hash=hashlib.sha256();batch_boxes=hashlib.sha256()
                for position,(slot,box) in enumerate(zip(row["physical_slot_indices"],row["shared_crop_boxes"])):
                    stage="original_proposed_NEAREST";own_guard(old,budget,False);target=targets[0][slot] if slot<old.ORIGINAL else targets[1][slot-old.ORIGINAL]
                    resized=crop(target,box);require(resized.shape==(360,640) and resized.dtype==np.uint8 and int(resized.max())<7,"original target crop ABI")
                    base_hash.update(memoryview(resized));hist=np.bincount(resized.ravel(),minlength=7).tolist()
                    top,left,height,width=box;source_sign=int(np.count_nonzero(target[top:top+height,left:left+width]==5))
                    base_values=old.exposure_counts(signs[slot],source_sign,hist,box==[0,0,360,640])
                    stage="fixed_even_ordinal_box_choice";choice=policy.select_box(policy.exposure_index(ordinal,position),box,signs[slot],hist[5]);selected=choice["selected_box"]
                    if choice["rescued"]:
                        candidate=crop(target,selected);candidate_hist=np.bincount(candidate.ravel(),minlength=7).tolist();candidate_source=signs[slot]
                        require(candidate.shape==(360,640) and candidate.dtype==np.uint8 and candidate_hist[5]==signs[slot]>0,"genuine full-FOV rescue support differs")
                    else:
                        require(selected is box,"negative/unprotected original proposal changed");candidate=resized;candidate_hist=hist;candidate_source=source_sign
                    cand_hash.update(memoryview(candidate));batch_boxes.update(struct.pack("<Q4h",policy.exposure_index(ordinal,position),*selected))
                    candidate_values=old.exposure_counts(signs[slot],candidate_source,candidate_hist,selected==[0,0,360,640])+branch_counts(choice,source_sign)
                    current["selected_boxes"].append(selected);current["baseline_support"].append(base_values);current["candidate_support"].append(candidate_values)
                    current["examined_exposures"]+=1;del resized,candidate,hist,candidate_hist;own_guard(old,budget,False)
                stage="ALL_original_B8_hash_admission";current["baseline_actual_target_sha256"]=base_hash.hexdigest();current["candidate_actual_target_sha256"]=cand_hash.hexdigest()
                require(base_hash.hexdigest()==row["target_sha256"],"historical baseline B8 mismatch; no favorable subset")
                stage="post_batch";budget.guard();require(old.process_generation()==generation,"self generation changed");require(used()<=LIMIT-CLOSING,"combined cap")
                next_base=old.commit_counts(base_stats,row["physical_slot_indices"],current["baseline_support"],signs)
                next_stats=commit_candidate(old,stats,row["physical_slot_indices"],current["candidate_support"])
                next_chain=base_chain.copy();next_chain.update(struct.pack("<Q",ordinal+1)+base_hash.digest())
                next_candidate=candidate_chain.copy();next_candidate.update(struct.pack("<Q",ordinal+1)+cand_hash.digest())
                next_boxes=box_chain.copy();next_boxes.update(struct.pack("<Q",ordinal+1)+batch_boxes.digest())
                next_bitmap=bytearray(bitmap);next_bitmap[ordinal//8]|=1<<(ordinal%8)
                base_stats,stats,base_chain,candidate_chain,box_chain,bitmap,passed,current=next_base,next_stats,next_chain,next_candidate,next_boxes,next_bitmap,passed+1,None
            require(passed==STEPS and base_chain.hexdigest()==plan["evidence"]["journal"]["expected_target_chain_sha256"],"full baseline20k/tail hash chain differs")
            require(base_stats==baseline["statistics"],"all original baseline per-slot/group/class support counters differ")
            candidate_partition_checks(stats,base_stats)
            require(stats["overall"][0]==160000 and [row[0] for row in stats["slot_statistics"]]==plan["evidence"]["journal"]["slot_exposures"]
                and sum(stats["overall"][11:18])==160000*PIXELS and stats["overall"][18:20]==[80000,80000],"full candidate exposure/class/protection partition differs")
            baseline_equal=True;stage="closing";verify(old,plan,budget,False)
        except BaseException as error:
            failed=int(current is not None);failure=dict(stage=stage,error=old.bounded_error(error),failed_batch=current,resources=budget.last)
        finally:
            targets=None;seals={}
            for path,pin in manifest(plan,plan_sha,grant_sha).items():
                try:
                    actual=digest(path)
                    if actual!=pin:seals[path]=dict(expected=pin,actual=actual)
                except BaseException as error:seals[path]=dict(expected=pin,actual=None,error=old.bounded_error(error))
            if seals:
                if failure is None:failure=dict(stage="closing_seals",failed_batch=None)
                failure["closing_seal_error_count"]=len(seals)
            try:
                memory=old.process_memory(budget.parser);budget.maximum_HWM=max(budget.maximum_HWM,memory["VmHWM_bytes"]);memory["maximum_observed_VmHWM_bytes"]=budget.maximum_HWM
                require(budget.maximum_HWM<=RSS and time.monotonic()-budget.started<DEADLINE and old.process_generation()==generation,"closing sampled memory/deadline/self generation denied")
                require(old.environment(unloaded=False)==plan["environment"],"closing selected providers differ")
            except BaseException as error:
                memory=locals().get("memory",dict(last_observed=budget.last))
                if failure is None:failure=dict(stage="closing_resources",failed_batch=None)
                failure["closing_resource_error"]=old.bounded_error(error)
            result=result_value(old,plan,plan_sha,grant_sha,passed,failed,failure,stats,native,base_chain.hexdigest(),candidate_chain.hexdigest(),box_chain.hexdigest(),bitmap.hex(),seals,memory,generation,time.monotonic()-budget.started,baseline_equal)
            write_new(OUTPUT/"result.json",encoded(result),True)
        return dict(status=result["status"],PASS_batches_per_arm=passed,FAIL_batches_per_arm=failed,UNEXECUTED_batches_per_arm=STEPS-passed-failed,path=str(OUTPUT/"result.json"))


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument("--mode",choices=("prepare","execute"),required=True)
    parser.add_argument("--plan-sha256");parser.add_argument("--authority-sha256");args=parser.parse_args()
    value=prepare() if args.mode=="prepare" else execute(args.plan_sha256,args.authority_sha256)
    print(encoded(value).decode(),end="");return 0 if value["status"] in ("CANDIDATE_ONLY_NOT_ROOT_AUTHORITY","COMPLETE_TRAIN_CROP_COUNTERFACTUAL_NOT_TRAINING_APPROVAL") else 2


if __name__=="__main__":raise SystemExit(main())
