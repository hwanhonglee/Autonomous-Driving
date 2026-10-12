"""HH_261002 - Audit every preview0.5 arm and retain the complete0.3/0.8 bracket without changing physical gates."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import json
from pathlib import Path

from . import audit_temporal_route_joint_preview08_signed45_batch_20261002 as previous

ROOT, CARLA_ROOT, frozen, PROFILES = previous.ROOT, previous.CARLA_ROOT, previous.frozen, previous.PROFILES
thread, joint, signed = previous.thread, previous.joint, previous.signed
require, digest, read_json, rows, add_pin, source_closure = previous.require, previous.digest, previous.read_json, previous.rows, previous.add_pin, previous.source_closure
strict_equal, endpoint_pins, measured_rear_errors = previous.strict_equal, previous.endpoint_pins, previous.measured_rear_errors
equivalent, number, acceleration_census = previous.equivalent, previous.number, previous.acceleration_census
tracker_math, pi_math = previous.tracker_math, previous.pi_math
BASELINE, BASELINE_SHA = previous.BASELINE, previous.BASELINE_SHA
SCHEMA = "portable_e2e.actual_route_joint_preview05_signed45_archive_audit.20261002.v1"
RECEIPT_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.actual_route_joint_thread4_preview05_signed45_three_arms.20261002.v1"
COLLECTOR_SCHEMA = "portable_e2e.actual_route_joint_thread4_preview05_signed45.20261002.v1"
PREVIEW_SCHEMA = "portable_e2e.causal_selected_speed_preview05_calibration.20261002.v1"
PREVIEW08_REPORT = ROOT/"storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_signed45_batch_audit_v1/batch_outcome.json"
PREVIEW08_REPORT_SHA = "4f55490483dfebca07c2c27a86ff77cfaf5a9da23d8337ae7819d361e4085f27"
PREVIEW08_BATCH = ROOT/"storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_compat_v2_signed45_town07_v1"
PREVIEW08_PLAN = PREVIEW08_BATCH.with_name(PREVIEW08_BATCH.name+"_plan.json")
PREVIEW08_PLAN_SHA = "a7387543f35afb5eb04d1931b9a46d16a583ef02ea69b984e9ea236f620a867b"
OWNER_SOURCE = ROOT/"tools/e2e/research_temporal_route_joint_preview05_signed45_town07_20261002.py"
FROZEN_AUDITOR_SHA = "988f50cc68993d2f0044e93558f6c63e820290551e69212c33322b56173b32dd"
NEW_FROZEN = dict(previous.NEW_FROZEN, **{
    "tools/e2e/audit_temporal_route_joint_preview08_signed45_batch_20261002.py": FROZEN_AUDITOR_SHA,
    "tools/e2e/research_temporal_route_joint_preview05_signed45_town07_20261002.py": "509ea11a411619fc4c3202d98705463d014522ea2b30c1f19c3737a7b327a114",
    "tools/e2e/collect_temporal_route_joint_preview05_signed45_20261002.py": "e0b3f5aeec01ce035545ccc92dd26cba274463024486928a3ef72de099892085",
})
FROZEN_HELPERS = dict(previous.FROZEN_HELPERS, **NEW_FROZEN)
_verify_frozen, _build, _publish = previous.verify_frozen, previous.build_report, previous.publish
_calibration = previous.calibration_contract
_active, _prior = False, None
METRICS = ("joint_motion_body_policy_wall100ms_passed", "native_actor_progress_m", "post_handoff_base_norm_maximum_mps",
    "post_handoff_actor_CTE_p95_m", "post_handoff_actor_CTE_maximum_m", "delivery_to_ACK_ms", "fresh_policy100ms_passed", "original_body_screen_passed")


def verify_frozen():
    require(digest(Path(previous.__file__)) == FROZEN_AUDITOR_SHA, "frozen0.8 independent auditor changed")
    _verify_frozen()
    for name, sha in FROZEN_HELPERS.items(): require(digest(ROOT/name) == sha, "unknown preview0.5 dependency: "+name)


def calibration_contract():
    result = _calibration()
    result.update(schema=PREVIEW_SCHEMA,configured_speed_preview_s=.5,
        single_factor_bracket_preview_seconds=[.3,.5,.8],no_prior_outcome_selected_or_relabelled=True)
    return result


def transformed_definition(name):
    """HH_261002 - Inverse-AST prove only the new speed scalar or explicit compatibility assertion differs."""
    tree = ast.parse(Path(previous.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == name)
    changed = copy.deepcopy(original); substitutions = []
    expected = {"preview_targets":3,"preview_census":1,"extra_trial_evidence":1,"verify_contract":1}
    for node in ast.walk(changed):
        if name == "preview_targets" and isinstance(node,ast.Constant) and type(node.value) is float and node.value == .8:
            substitutions.append((node,"value",.8)); node.value = .5
        elif name in ("preview_census","extra_trial_evidence") and isinstance(node,ast.keyword) and node.arg == ("speed_preview_s" if name == "preview_census" else "configured_speed_preview_s"):
            require(isinstance(node.value,ast.Constant) and type(node.value.value) is float and node.value.value == .8,"unexpected preview keyword")
            substitutions.append((node.value,"value",.8)); node.value.value = .5
        elif name == "verify_contract" and isinstance(node,ast.Compare) and ast.dump(node.left,include_attributes=False) == ast.dump(ast.parse("plan.get('collector_and_physical_preview_calibration_unchanged')",mode="eval").body,include_attributes=False):
            require(len(node.ops) == len(node.comparators) == 1 and isinstance(node.ops[0],ast.Is) and isinstance(node.comparators[0],ast.Constant) and node.comparators[0].value is True,"unexpected old compatibility predicate")
            substitutions.append((node.comparators[0],"value",True)); node.comparators[0].value = False
    require(len(substitutions) == expected[name], "exact preview-only AST substitution count differs")
    altered = copy.deepcopy(changed)
    for node, field, value in substitutions: setattr(node,field,value)
    require(ast.dump(changed,include_attributes=False) == ast.dump(original,include_attributes=False),"non-preview audit statement changed")
    namespace = dict(vars(previous),PREVIEW_SCHEMA=PREVIEW_SCHEMA,preview_targets=preview_targets,preview_census=preview_census,
        extra_trial_evidence=extra_trial_evidence,calibration_contract=calibration_contract,NEW_FROZEN=NEW_FROZEN)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[altered],type_ignores=[])),str(Path(__file__))+":exact0p5_"+name,"exec"),namespace)
    return namespace[name]


def preview_targets(selected,config): return transformed_definition("preview_targets")(selected,config)
def preview_census(*args): return transformed_definition("preview_census")(*args)
def extra_trial_evidence(*args): return transformed_definition("extra_trial_evidence")(*args)


def prior_context(endpoint,pins):
    # HH_261002 - Execute only the pinned pure-stdlib receipt reader, not the physical owner or its Torch imports.
    add_pin(pins,OWNER_SOURCE,NEW_FROZEN[str(OWNER_SOURCE.relative_to(ROOT))])
    tree = ast.parse(OWNER_SOURCE.read_bytes())
    function = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "prior_preview08_context")
    namespace = dict(Path=Path,require=require,digest=digest,read_json=read_json,ROOT=ROOT,
        PREVIEW08_BATCH=PREVIEW08_BATCH,PREVIEW08_PLAN=PREVIEW08_PLAN,PREVIEW08_PLAN_SHA=PREVIEW08_PLAN_SHA)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),str(Path(__file__))+":pinned0p8_receipts","exec"),namespace)
    result = namespace["prior_preview08_context"](endpoint)
    require(len(result["receipt_source_sha256"]) == 13,"all three original0.8 receipts/declarations required")
    for path,sha in result["receipt_source_sha256"].items(): add_pin(pins,path,sha)
    return result


def prior_proof(caller,endpoint,pins):
    require(caller.get("prior_preview08_report") == dict(path=str(PREVIEW08_REPORT),sha256=PREVIEW08_REPORT_SHA),"exact independent allthree0.8 report required")
    add_pin(pins,PREVIEW08_REPORT,PREVIEW08_REPORT_SHA)
    require(PREVIEW08_REPORT.stat().st_size <= 32*1024**2,"only exact bounded0.8 report may exceed generic4MiB")
    old = json.loads(PREVIEW08_REPORT.read_text())
    require(old.get("schema") == previous.SCHEMA and old.get("batch") == str(PREVIEW08_BATCH) and old.get("final_endpoint_sha256") == endpoint
        and old.get("prospective_trials") == 3 and old.get("all_failed_tail_retained") is True
        and old.get("all_actual_native_states") == 2700 and old.get("all_actual_camera_bundles") == 1350
        and old.get("all_actual_ACKs") == 2712 and old.get("actual_JPEG_assets") == 8100
        and [r["signed_recovery_profile"] for r in old["trials"]] == list(PROFILES)
        and strict_equal(old.get("cpu_thread_numerical_preset"),thread.pure_preset()[0])
        and old.get("causal_selected_speed_preview_calibration",{}).get("configured_speed_preview_s") == .8,
        "same final6000/CPU4/PI/three full0.8 conditions required")
    for name,sha in old["source_sha256"].items(): add_pin(pins,name,sha)
    context = prior_context(endpoint,pins)
    require(all(c["original_basic_screen_passed"] is r["joint_motion_body_policy_wall100ms_passed"] for c,r in zip(context["trials"],old["trials"])),"old diagnostic and independent allarm outcomes differ")
    return old,context


def verify_contract(plan,trial):
    transformed_definition("verify_contract")(plan,trial)
    require(_prior is not None and strict_equal(plan.get("prior_complete_preview08_context"),_prior[1]),"complete unmodified0.8 receipt context differs")
    require(plan.get("exact_baseline_reader_compatibility_unchanged") is True and plan.get("zero_new_training_and_no_checkpoint_selection") is True
        and plan.get("original0p3_and0p8_source_outputs_unchanged") is True,"no source/model/checkpoint intervention allowed")
    size = sum(Path(path).stat().st_size for path in _prior[1]["receipt_source_sha256"])
    require(plan.get("prior_preview08_receipt_archive_bytes_per_trial") == size
        and plan.get("additional_three_trial_preview08_receipt_archive_reserve_bytes") == 3*size
        and all(plan["asset_sha256"].get(path) == sha for path,sha in _prior[1]["receipt_source_sha256"].items()),"all thirteen prior receipts/three copies/reserve must be declared")


@contextmanager
def activated(prior):
    global _active,_prior
    require(not _active and not previous._active and previous.build_report is _build and previous.publish is _publish,"new0.5 auditor cannot nest or accept unknown delegate")
    verify_frozen()
    values = dict(SCHEMA=SCHEMA,RECEIPT_SCHEMA=RECEIPT_SCHEMA,OWNER_SCHEMA=OWNER_SCHEMA,COLLECTOR_SCHEMA=COLLECTOR_SCHEMA,
        PREVIEW_SCHEMA=PREVIEW_SCHEMA,NEW_FROZEN=NEW_FROZEN,FROZEN_HELPERS=FROZEN_HELPERS,verify_frozen=verify_frozen,
        calibration_contract=calibration_contract,preview_targets=preview_targets,preview_census=preview_census,
        extra_trial_evidence=extra_trial_evidence,verify_contract=verify_contract)
    saved = {k:getattr(previous,k) for k in values}; _active,_prior = True,prior
    try:
        for k,v in values.items(): setattr(previous,k,v)
        yield
    finally:
        for k,v in reversed(tuple(saved.items())): setattr(previous,k,v)
        _active,_prior = False,None


def bracket_comparison(report,prior):
    parent = report["parent_comparison"]
    require(len(parent["per_arm"]) == len(prior["trials"]) == 3,"all three preview conditions required")
    per_arm = []
    for i,(row,old) in enumerate(zip(parent["per_arm"],prior["trials"])):
        require(row["trial_index"] == i and row["case_id"] == PROFILES[i]["case_id"],"fixed profile ordering differs")
        per_arm.append(dict(trial_index=i,case_id=row["case_id"],preview03=row["baseline"],
            preview05=row["preview08"],preview08={k:old[k] for k in METRICS},
            preview03_additive_native_rate=row["baseline_additive_native_rate"],preview05_additive_native_rate=row["candidate_additive_native_rate"],
            preview08_additive_native_rate=old["independent_additive_native_acceleration"]))
        row["preview05"] = row.pop("preview08")
    return dict(configured_preview_seconds=[.3,.5,.8],same_fixed_final6000_weights_CPU4_PI2p4_profiles_seeds_and_old_thresholds=True,
        original0p3_report=dict(path=str(BASELINE),sha256=BASELINE_SHA),original0p8_report=dict(path=str(PREVIEW08_REPORT),sha256=PREVIEW08_REPORT_SHA),
        all_nine_actual_attempts_retained=True,no_best_trial_or_checkpoint_selection=True,old_outcomes_and_source_files_rewritten=False,
        pass_counts={"preview03":parent["baseline_original_joint_pass_count"],"preview05":report["joint_motion_body_policy_wall_trial_pass_count"],"preview08":prior["joint_motion_body_policy_wall_trial_pass_count"]},per_arm=per_arm)


def build_report(batch,batch_sha,caller_path,caller_sha):
    pins = {}; add_pin(pins,caller_path,caller_sha); caller = read_json(caller_path)
    require(caller.get("schema") == RECEIPT_SCHEMA,"new0.5 caller schema required")
    endpoint = endpoint_pins(caller.get("final_endpoint")); prior = prior_proof(caller,endpoint,pins)
    with activated(prior): report = _build(batch,batch_sha,caller_path,caller_sha)
    report["source_sha256"].update(pins)
    for path in source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"): add_pin(report["source_sha256"],path)
    report["all_preview_bracket_comparison"] = bracket_comparison(report,prior[0])
    report["prior_complete_preview08_context"] = prior[1]
    report["original0p3_and0p8_reports_and_all_failed_source_outputs_preserved"] = True
    frozen.assert_pins(report["source_sha256"]); return report


def markdown(report):
    def n(v): return "미측정" if v is None else "%.3f"%v
    bracket = report["all_preview_bracket_comparison"]
    lines = ["# 같은 주행 모델: 속도 추종 시점 0.3·0.5·0.8초 전체 비교","",
        "<!-- HH_261002 - Retain all nine actual attempts and every fixed45s failure while comparing one controller scalar. -->","",
        "전체 기준: 0.3초 %d/3 · 0.5초 %d/3 · 0.8초 %d/3. 정식 메인 0/9 · 하위 0/30입니다."%tuple(bracket["pass_counts"][k] for k in ("preview03","preview05","preview08")),"",
        "| 출발 조건 | 추종 시점 | 전체 기준 | 전진 거리 | 실제 속도 최대 | 수신→ACK 최대 |","|---|---:|---|---:|---:|---:|"]
    for arm in bracket["per_arm"]:
        for key,label in (("preview03","0.3초"),("preview05","0.5초"),("preview08","0.8초")):
            row = arm[key]
            lines.append("| %s | %s | %s | %s m | %s m/s | %s ms |"%(arm["case_id"],label,"PASS" if row["joint_motion_body_policy_wall100ms_passed"] else "FAIL",n(row["native_actor_progress_m"]),n(row["post_handoff_base_norm_maximum_mps"]),n(row["delivery_to_ACK_ms"].get("maximum"))))
    lines += ["","가중치·NN/디코더·실제 v0·CPU4/1·PI2.4·좌우/중앙 출발·seed·45초·90m·속도2.5·차체10cm·100ms는 같습니다. 바뀐 것은 예측 속도에서 제어기가 따라갈 시점뿐입니다. 어느 비교 시점이든 0.05m/s 이하 또는 명시적 non-GO이면 원래0.3초 tracker를 통째로 사용합니다.","",
        "모든 fresh 선택64점·원 tracker·PI 적분·ACK 5개 필드를 독립 재계산합니다. held 명령은 PI를 다시 적분하지 않습니다. 추가20Hz actor world 속도 변화율 +3/−6 검사는 첫 driving 직전 warmup 경계도 포함합니다. 벡터 가속도와 abs2.9 관측은 별도이며 학습 승인으로 바꾸지 않습니다.","",
        "새0.5초의 3조건·2,700상태·1,350 실제6카메라 묶음·8,100JPEG·2,712ACK·실패 꼬리와 기존0.3/0.8의 모든 결과를 보존합니다. 좋은 실행 한 번을 골라 전체 성공으로 세지 않습니다. 제한된 Town07 저속 직진 비교이며 회전·종점 정지·30km/h·다른 맵·실차 승인, 센서 노출부터 전체10Hz의 증거가 아닙니다.","",
        "[전체 결과 JSON](batch_outcome.json) · [원자료/소스SHA](source_evidence.json)",""]
    return "\n".join(lines)


def publish(output,*args):
    output = Path(output)
    require(not output.exists() and not output.is_symlink() and output.absolute() == output.resolve()
        and not any(p.is_symlink() for p in output.parents),"fresh canonical create-only0.5 audit output required")
    require(not output.is_relative_to(Path(args[0]).resolve()) and not output.is_relative_to(PREVIEW08_BATCH)
        and not output.is_relative_to(PREVIEW08_REPORT.parent) and not output.is_relative_to(BASELINE.parent),"publication cannot modify immutable studies")
    report = build_report(*args); frozen.assert_pins(report["source_sha256"]); output.mkdir(parents=True,exist_ok=False)
    for name,value in (("batch_outcome.json",report),("source_evidence.json",report["source_sha256"])):
        with (output/name).open("x") as stream: json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream: stream.write(markdown(report))
    frozen.assert_pins(report["source_sha256"])
    return dict(output=str(output),result_sha256=digest(output/"batch_outcome.json"),all_trials_joint_passed=report["all_trials_motion_body_policy_AND_wall100ms_passed"],simulator_or_GPU_used=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("batch","caller-receipts","output"): parser.add_argument("--"+name,type=Path,required=True)
    for name in ("batch-sha256","caller-receipts-sha256"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--validate-only",action="store_true"); args = parser.parse_args(argv)
    inputs = (args.batch,args.batch_sha256,args.caller_receipts,args.caller_receipts_sha256)
    result = build_report(*inputs) if args.validate_only else publish(args.output,*inputs)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
