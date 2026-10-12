"""HH_261002 - NEW final6000 joint audit: unchanged physical gates, explicit54/47 lineage, all three arms and parent comparison."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import json
from pathlib import Path

from . import audit_temporal_signed_recovery45_batch_20261002 as previous

ROOT,CARLA_ROOT,frozen,PROFILES=previous.ROOT,previous.CARLA_ROOT,previous.frozen,previous.PROFILES
require,digest,read_json,rows,add_pin,source_closure=previous.require,previous.digest,previous.read_json,previous.rows,previous.add_pin,previous.source_closure
SCHEMA="portable_e2e.actual_route_joint_final6000_signed45_archive_audit.20261002.v1"
RECEIPT_SCHEMA=SCHEMA+".caller_pins"
OWNER_SCHEMA="portable_e2e.actual_route_joint_final6000_signed45_three_arms.20261002.v1"
COLLECTOR_SCHEMA="portable_e2e.actual_route_joint_final6000_signed45.20261002.v1"
EXPORT_SCHEMA="portable_e2e.temporal_route_joint_export_research.20261002.v1"
TRAINING_SCHEMA="portable_e2e.temporal_route_joint_training.20261002.v1"
ARCHITECTURE_SCHEMA="portable_e2e.temporal_route_recovery_model.20261001.v3"
MODEL_ID="portable_e2e.dense_nav.learned_route_recovery.v3.research"
KERNEL_ID="fixed33_to257_align_corners_dense_gemm.20261001.v1"
PARENT_CHECKPOINT_SHA="a17544d99fd87467e4afe1c051dd598f72ab9cd0d79794f34d10291e1edce52f"
PARENT_NPZ_SHA="0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9"
PARENT_METADATA_SHA="4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d"
PARENT_DECLARATION_SHA="34c8a55028ea7b4cb2678a73701b16425d811fb10dd77afb5554183dd79ef6c5"
BACKBONE_NPZ_SHA="4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735"
BACKBONE_METADATA_SHA="88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec"
TRAINABLE_PREFIXES=("fusion.","output_norm.","trunk.","curvature_head.","speed_head.","route_recovery_branch.")
BATCH_DOMAINS=["recovery_noncentered_joint","recovery_all_joint","v4_joint","legacy_raw_curvature_and_speed"]
PROTOCOL=ROOT/"learning/portable_e2e/temporal_route_joint_export_research_20261002.py"
RUN=ROOT/"storage/experiments/training/2026-10-02/temporal_route_joint_v1"
PARENT=ROOT/"storage/experiments/training/2026-10-01/temporal_route_recovery_v3_v1/exports/step_003000"
BACKBONE=ROOT/"storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_v1/exports/step_003000"
BASELINE_BATCH=ROOT/"storage/experiments/validation/2026-10-02/temporal_signed_recovery45_town07_v3"
NEW_FROZEN={
    "tools/e2e/audit_temporal_signed_recovery45_batch_20261002.py":"bf2abf12c6e17b53dd3405cc12b2d56eb3b14b38236ed140676ee16ac722cc8a",
    "tools/e2e/research_temporal_route_joint_signed45_town07_20261002.py":"6926855c4eb66e8235b343a2aa8e16f5d6affa86ad20c1342bb6927f2d97f190",
    "tools/e2e/collect_temporal_route_joint_signed45_20261002.py":"013f02277db87828d67800b88e0113c7552bf64a2f534b228b3d88657f0f98ab",
    "learning/portable_e2e/temporal_route_joint_export_research_20261002.py":"3245e5ee72b6a2bca030249b3075704eb579a0331d0b0c4d6292068e7c192bab",
}
FROZEN_HELPERS=dict(previous.FROZEN_HELPERS,**NEW_FROZEN)
_baseline_schema=previous.SCHEMA
_baseline_contract,_baseline_reader,_baseline_build=previous.verify_contract,previous.transformed_trial_reader,previous.build_report
_old_export=previous.previous.verify_export
_active=False
_endpoint=None


def verify_frozen():
    previous.verify_frozen()
    for name,sha in FROZEN_HELPERS.items():require(digest(ROOT/name)==sha,"unknown NEW joint/frozen dependency: "+name)


def endpoint_pins(value):
    require(isinstance(value,dict) and set(value)=={"weights_sha256","metadata_sha256","checkpoint_sha256","declaration_sha256"}
        and all(previous.previous.sha(v) for v in value.values()), "caller must independently pin all four final6000 identities")
    return dict(value)


def explicit_numeric_protocol():
    """HH_261002 - Execute only the frozen pure metadata predicate/LR AST; never import Torch or read pickle/NPZ tensors."""
    tree=ast.parse(PROTOCOL.read_bytes())
    names={"verify_scope","learning_rate_config"}
    functions=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in names]
    require({n.name for n in functions}==names,"complete frozen pure metadata protocol functions required")
    namespace=dict(globals(),is_sha=previous.previous.sha)
    module=ast.fix_missing_locations(ast.Module(body=functions,type_ignores=[]))
    exec(compile(module,str(Path(__file__))+":exact_pure_joint_metadata_protocol","exec"),namespace)
    return namespace["verify_scope"]


def joint_baseline_contract():
    """HH_261002 - Remove ONLY the two false inherited old-learning assertions; every original physical/recovery threshold statement stays exact."""
    tree=ast.parse(Path(previous.__file__).read_bytes())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="verify_contract")
    original=copy.deepcopy(function)
    fixed=next(n for n in function.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="fixed" for t in n.targets))
    names={"current_fixed_final3000_V3_weights_unchanged","no_new_training_or_checkpoint_selection"}
    removed=[(i,copy.deepcopy(k)) for i,k in enumerate(fixed.value.keywords) if k.arg in names]
    require({k.arg for _,k in removed}==names,"exact inherited old learning assertions required")
    fixed.value.keywords=[k for k in fixed.value.keywords if k.arg not in names]
    parity=copy.deepcopy(function)
    restored=next(n for n in parity.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="fixed" for t in n.targets))
    for index,node in removed:restored.value.keywords.insert(index,node)
    require(ast.dump(parity,include_attributes=False)==ast.dump(original,include_attributes=False),"all unchanged physical contract AST must remain identical")
    namespace=dict(vars(previous));namespace.update(OWNER_SCHEMA=OWNER_SCHEMA,COLLECTOR_SCHEMA=COLLECTOR_SCHEMA,NEW_FROZEN=dict(previous.NEW_FROZEN,**NEW_FROZEN))
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    exec(compile(module,str(Path(__file__))+":physical_contract_minus_false_old_learning_assertions","exec"),namespace)
    return namespace["verify_contract"]


def verify_contract(plan,trial):
    joint_baseline_contract()(plan,trial)
    fixed=dict(optimizer_updates=6000,known_total_optimizer_updates=12200,parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=6200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        current_fixed_final6000_joint_weights_unchanged=True,original_backbone_weights_byte_unchanged=False,
        visual_encoder_frozen=True,semantic_head_parameters_frozen=True,semantic_outputs_preserved=False,stop_intent_trained=False,
        no_training_during_physical_study=True,no_checkpoint_selection=True,joint6000_training_completed_before_physical_study=True,
        training_schema=TRAINING_SCHEMA,new_numeric_export_schema=EXPORT_SCHEMA,
        actual_same_profiles_and_corrected_manual_first_gear_teacher_as_V3_baseline=True,
        actual_same_2p40_PI_45seconds_and_all_original_safety_thresholds=True,no_output_path_speed_or_intent_replacement=True,
        endpoint_screen_result_is_offline_not_physical_approval=True)
    require(all(previous.strict_equal(plan.get(k),v) for k,v in fixed.items()),"truthful fixed6000/54learned47fixed physical scope required")
    require("current_fixed_final3000_V3_weights_unchanged" not in plan and "no_new_training_or_checkpoint_selection" not in plan,
        "old3000/frozen-backbone/never-trained assertion cannot masquerade as NEW6000")
    require(plan.get("parent_numeric_export_sha256")==dict(weights=PARENT_NPZ_SHA,metadata=PARENT_METADATA_SHA)
        and plan.get("backbone_parent_numeric_export_sha256")==dict(weights=BACKBONE_NPZ_SHA,metadata=BACKBONE_METADATA_SHA),"both exact warmstart and mixed-backbone parent pairs required")
    require(_endpoint is not None and plan["source_checkpoint_sha256"]==_endpoint["checkpoint_sha256"]
        and plan["new_fixed_endpoint_export_sha256"]==dict(weights=_endpoint["weights_sha256"],metadata=_endpoint["metadata_sha256"])
        and plan["actual_training_declaration_sha256"]==_endpoint["declaration_sha256"],"four externally pinned final6000 endpoint identities differ")
    argv=plan["worker_command"]
    for name,value in _endpoint.items():
        option="--expected-"+name.replace("_","-")
        require(argv.count(option)==1 and argv[argv.index(option)+1]==value,"private child must receive same exact final6000 caller identity")


def verify_export(metadata,plan,*,caller_pins=None):
    pins=endpoint_pins(caller_pins if caller_pins is not None else _endpoint)
    explicit_numeric_protocol()(metadata)
    require(metadata["checkpoint_sha256"]==pins["checkpoint_sha256"]==plan["source_checkpoint_sha256"]
        and metadata["npz_sha256"]==pins["weights_sha256"] and metadata["declaration_sha256"]==pins["declaration_sha256"]
        and plan["new_fixed_endpoint_export_sha256"]==dict(weights=pins["weights_sha256"],metadata=pins["metadata_sha256"]),"no intermediate or observed-best endpoint accepted")
    require(metadata.get("new_signed_recovery_cohort_moving_only") is True
        and metadata.get("fixed_recovery_native_frame_offsets")==[80,682]
        and metadata.get("fixed_recovery_driving_relative_window_seconds")==[4.,34.1]
        and metadata.get("startup_recovery_fitted") is False,"same rolling-only recovery learning scope required")
    shapes,dtypes=metadata.get("state_tensor_shapes"),metadata.get("state_tensor_dtypes")
    require(isinstance(shapes,dict) and isinstance(dtypes,dict) and len(shapes)==len(dtypes)==101 and set(shapes)==set(dtypes)
        and all(v=="float32" for v in dtypes.values())
        and all(isinstance(v,list) and all(type(d) is int and d>=0 for d in v) for v in shapes.values()),"complete finite numeric-only101 shape/dtype ABI required")
    names=sorted(k for k in shapes if k.startswith(TRAINABLE_PREFIXES));fixed=sorted(set(shapes)-set(names))
    require(len(names)==54 and len(fixed)==47 and metadata.get("trainable_state_names")==names
        and metadata.get("frozen_state_names")==fixed,"exact54 permitted learning/47 fixed state partition required")
    sources=metadata["source_sha256"]
    require(len(sources)==26 and all(plan["source_sha256"].get(name)==sha for name,sha in sources.items()),"all26 exact training sources must be in physical source/archive closure")
    parent_pairs={str(PARENT/"model_state.npz"):PARENT_NPZ_SHA,str(PARENT/"metadata.json"):PARENT_METADATA_SHA,
        str(BACKBONE/"model_state.npz"):BACKBONE_NPZ_SHA,str(BACKBONE/"metadata.json"):BACKBONE_METADATA_SHA}
    require(all(plan["asset_sha256"].get(path)==sha for path,sha in parent_pairs.items()),"live+archived both exact parent asset pairs required")


def transformed_trial_reader():
    """HH_261002 - Reuse the full independent900/450/904 authenticator; change only the new truthful metadata revision label."""
    tree=ast.parse(Path(previous.__file__).read_bytes())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="transformed_trial_reader")
    original=copy.deepcopy(function)
    anchors=[i for i,n in enumerate(function.body) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=="namespace" for t in n.targets)]
    require(len(anchors)==1,"frozen signed-reader compilation anchor differs")
    insert=ast.parse("""revision=next(k for k in returned.value.keywords if k.arg=='training_data_revision')
require(ast.dump(revision.value,include_attributes=False)==ast.dump(ast.parse(\"metadata['recovery_data_revision']\",mode='eval').body,include_attributes=False),'exact original metadata label anchor required')
revision.value=ast.parse(\"metadata['current_training_purpose']\",mode='eval').body
""").body
    function.body[anchors[0]:anchors[0]]=insert
    parity=copy.deepcopy(function);del parity.body[anchors[0]:anchors[0]+len(insert)]
    require(ast.dump(parity,include_attributes=False)==ast.dump(original,include_attributes=False),"all frozen signed-reader source statements must remain exact")
    namespace=dict(vars(previous));namespace.update(verify_contract=verify_contract)
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    exec(compile(module,str(Path(__file__))+":same_authenticator_new_metadata_label_only","exec"),namespace)
    return namespace["transformed_trial_reader"]()


@contextmanager
def activated(pins):
    global _active,_endpoint
    require(not _active and not previous._active,"joint audit cannot nest with baseline audit")
    verify_frozen();_endpoint=endpoint_pins(pins)
    names=dict(SCHEMA=SCHEMA,RECEIPT_SCHEMA=RECEIPT_SCHEMA,OWNER_SCHEMA=OWNER_SCHEMA,COLLECTOR_SCHEMA=COLLECTOR_SCHEMA,
        FROZEN_HELPERS=FROZEN_HELPERS,NEW_FROZEN=dict(previous.NEW_FROZEN,**NEW_FROZEN),verify_contract=verify_contract,
        transformed_trial_reader=transformed_trial_reader)
    saved={k:getattr(previous,k) for k in names};legacy=previous.previous
    require(legacy.verify_export is _old_export,"unknown original export validator binding")
    old_export=legacy.verify_export;_active=True
    try:
        for k,v in names.items():setattr(previous,k,v)
        legacy.verify_export=verify_export
        yield
    finally:
        legacy.verify_export=old_export
        for k,v in saved.items():setattr(previous,k,v)
        _active=False;_endpoint=None


def parent_comparison(report,caller,pins):
    identity=caller.get("baseline_report")
    require(isinstance(identity,dict) and set(identity)=={"path","sha256"},"externally pinned all-three actual baseline report required for comparison")
    path=Path(identity["path"]);require(path.absolute()==path.resolve() and path.is_relative_to(ROOT),"canonical actual baseline report path required")
    add_pin(pins,path,identity["sha256"]);baseline=read_json(path)
    require(baseline["schema"]==_baseline_schema and baseline["batch"]==str(BASELINE_BATCH)
        and baseline["prospective_trials"]==3 and baseline.get("all_failed_tail_retained") is True
        and [t["signed_recovery_profile"] for t in baseline["trials"]]==list(PROFILES)
        and all(t["source_checkpoint_sha256"]==PARENT_CHECKPOINT_SHA
            and t["fixed_export_sha256"]==dict(weights=PARENT_NPZ_SHA,metadata=PARENT_METADATA_SHA) for t in baseline["trials"]),
        "same predeclared actual V3 baseline/fixed parent, not selected2-of3 or offline loss, required")
    for name,sha in baseline["source_sha256"].items():add_pin(pins,name,sha)
    comparisons=[]
    for old,new in zip(baseline["trials"],report["trials"]):
        require(old["trial_index"]==new["trial_index"] and old["signed_recovery_profile"]==new["signed_recovery_profile"],"same seeds/profile per-arm physical comparison required")
        def metrics(row):
            recovery=row["independent_signed_recovery"]
            return dict(joint_all_physical_gates_passed=row["joint_motion_body_policy_wall100ms_passed"],
                signed_recovery_passed=recovery["recovery_screen_passed"],center_comparator_passed=recovery.get("centered_setup_screen_passed",False),
                progress_m=row["native_actor_progress_m"],base_native_speed_maximum_mps=row["post_handoff_base_norm_maximum_mps"],
                actor_CTE_p95_m=row["post_handoff_actor_CTE_p95_m"],actor_CTE_maximum_m=row["post_handoff_actor_CTE_maximum_m"],
                observed_delivery_to_ACK_ms=row["delivery_to_ACK_ms"],recovery_convergence=recovery.get("convergence"),
                latched_failure=row.get("first_latched_failure"),policy100ms_passed=row["fresh_policy100ms_passed"],body_passed=row["original_body_screen_passed"])
        comparisons.append(dict(trial_index=new["trial_index"],case_id=new["signed_recovery_profile"]["case_id"],baseline=metrics(old),joint_candidate=metrics(new)))
    return dict(baseline_report_path=str(path),baseline_report_sha256=identity["sha256"],baseline_checkpoint_sha256=PARENT_CHECKPOINT_SHA,
        baseline_joint_pass_count=baseline["joint_motion_body_policy_wall_trial_pass_count"],
        candidate_joint_pass_count=report["joint_motion_body_policy_wall_trial_pass_count"],same_actual_profiles_seeds_teacher_PI_and_thresholds=True,
        difference_is_joint_learned_weights_not_output_substitution=True,all_three_parent_and_candidate_attempts_retained=True,
        heldout_or_best_attempt_selection_used=False,main9_or_generalized_performance_equivalence_claimed=False,per_arm=comparisons)


def build_report(*args):
    caller_path,caller_sha=Path(args[2]),args[3];caller_pin={};add_pin(caller_pin,caller_path,caller_sha);caller=read_json(caller_path)
    endpoint=endpoint_pins(caller.get("final_endpoint"))
    with activated(endpoint):result=_baseline_build(*args)
    for path in source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"):add_pin(result["source_sha256"],path)
    add_pin(result["source_sha256"],RUN/"declaration.json",endpoint["declaration_sha256"])
    comparison=parent_comparison(result,caller,result["source_sha256"])
    result.pop("original_frozen97_check_enforced_by_pinned_numeric_loader",None)
    result.update(optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,final_endpoint_sha256=endpoint,parent_comparison=comparison,
        no_torch_pickle_loaded=True,numeric_NPZ_values_not_reloaded_by_this_offline_auditor=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=True)
    frozen.assert_pins(result["source_sha256"]);return result


def markdown(report):
    def n(v):return "미측정" if v is None else "%.3f"%v
    lines=["# Town07 실제 비교 — 기존 회복 모델 vs 새 공동 학습 모델","","<!-- HH_261002 - Compare all actual arms under the same unchanged gates; learning/loss is not physical completion. -->","",
        "기존 모델의 전체 기준 통과 %d/3 → 새 모델 %d/3. 새 모델의 좌·우 복귀 조건 통과 %d/2이며 중앙 비교는 별도입니다." % (
            report["parent_comparison"]["baseline_joint_pass_count"],report["joint_motion_body_policy_wall_trial_pass_count"],report["signed_recovery_pass_count"]),"",
        "| 조건 | 기존 → 새 전체 기준 | 기존 → 새 전진 | 기존 → 새 native base 속도 최대 | 기존 → 새 관측→ACK 최대 |",
        "|---|---|---:|---:|---:|"]
    for row in report["parent_comparison"]["per_arm"]:
        a,b=row["baseline"],row["joint_candidate"]
        lines.append("| %s | %s → %s | %s → %s m | %s → %s m/s | %s → %s ms |" % (row["case_id"],
            "PASS" if a["joint_all_physical_gates_passed"] else "FAIL","PASS" if b["joint_all_physical_gates_passed"] else "FAIL",
            n(a["progress_m"]),n(b["progress_m"]),n(a["base_native_speed_maximum_mps"]),n(b["base_native_speed_maximum_mps"]),
            n(a["observed_delivery_to_ACK_ms"].get("maximum")),n(b["observed_delivery_to_ACK_ms"].get("maximum"))))
    lines += ["","새 모델은 같은 101-tensor 구조에서 fusion·곡률·속도 등 **54개 tensor를 6,000 update 공동 학습**하고 47개 입력/시각/semantic-head tensor를 고정합니다. 기존 모델의 ‘97개 backbone 전체 고정’과 다릅니다. semantic-head 파라미터가 고정되어도 공유 특징이 바뀌어 semantic 출력은 보존됐다고 주장하지 않습니다. STOP은 학습하지 않았습니다.","",
        "모델 가중치만 바뀌었고, 실제 spawn ±0.20 m·±1.5°/중앙·seed·manual1 전문가 출발·PI 목표 2.40 m/s·원래 제어/안전 기준은 같습니다. 원본 상태를 만든 이전 모델 ACK부터 부호 CTE/yaw를 독립 계산합니다. 전문가가 출발 중 복귀시킨 것을 모델 복귀로 세지 않습니다.","",
        "복귀 조건: 첫 실제 모델 효과의 오차가 0.10 m·0.5° 이상이고 부호가 맞아야 합니다. 10초 안에 두 오차 밴드로 들어가 연속 2초 유지합니다(확인은 최대 12초). 중앙 조건은 복귀 2조건에 더하지 않습니다. 복귀만 통과했어도 속도/주행/차체/지연 중 하나라도 미달이면 전체 기준은 FAIL입니다.","",
        "모든 세 실행 전체 45초·2,700 native 상태·1,350 실제 6카메라 묶음·8,100 JPEG·2,712 ACK를 남깁니다. 각 실행 전진 ≥90 m, native base 속도 ≤2.5 m/s, world 속도 ≤2.7 m/s, 기존 CTE·무충돌·차체900개/sweep899개 및 10 cm 여유, 정책100 ms와 관측 bundle→ACK100 ms 기준을 완화하지 않습니다. 실패 꼬리와 미측정 값도 그대로 보존합니다.","",
        "최종6000 NPZ/metadata/checkpoint/학습 선언의 네 SHA와 두 부모 가중치·metadata를 별도로 묶습니다. 이 감사는 pickle/.pt나 Torch를 불러 추론하지 않으며, 47개 고정값 동등성은 선언된 numeric loader가 강제합니다. 감사가 직접 tensor 전체를 다시 로딩했다고 주장하지 않습니다.","",
        "센서 노출/callback 진입부터의 완전한 센서→ACK 10 Hz는 아직 미측정입니다. 고정 TRAIN 회랑의 저속 예비 시험이며 다른 맵·완전한 목적지 정지·회전·실차/30–60 kph의 승인이 아닙니다. **정식 MAIN9 승인 0/9·하위 기능 0/30이며 개발은 계속됩니다.**", "",
        "[전체 비교·독립 확인 JSON](batch_outcome.json) · [원자료·소스 SHA256](source_evidence.json)",""]
    return "\n".join(lines)


def publish(output,*args):
    output=Path(output);require(not output.exists() and not output.is_symlink(),"fresh create-only joint audit output required")
    report=build_report(*args);frozen.assert_pins(report["source_sha256"]);output.mkdir(parents=True,exist_ok=False)
    for name,value in (("batch_outcome.json",report),("source_evidence.json",report["source_sha256"])):
        with (output/name).open("x") as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream:stream.write(markdown(report))
    frozen.assert_pins(report["source_sha256"])
    return dict(output=str(output),result_sha256=digest(output/"batch_outcome.json"),
        all_trials_joint_passed=report["all_trials_motion_body_policy_AND_wall100ms_passed"],simulator_or_GPU_used=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("batch","caller-receipts","output"):parser.add_argument("--"+name,type=Path,required=True)
    for name in ("batch-sha256","caller-receipts-sha256"):parser.add_argument("--"+name,required=True)
    parser.add_argument("--validate-only",action="store_true");args=parser.parse_args(argv)
    inputs=(args.batch,args.batch_sha256,args.caller_receipts,args.caller_receipts_sha256)
    result=build_report(*inputs) if args.validate_only else publish(args.output,*inputs)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
