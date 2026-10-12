"""HH_261002 - Authenticate all thread4 signed45 attempts under original physical gates and disclosed floating differences."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import json
from pathlib import Path
from types import SimpleNamespace

from . import audit_temporal_route_joint_signed45_batch_20261002 as previous

ROOT, CARLA_ROOT, frozen, PROFILES = previous.ROOT, previous.CARLA_ROOT, previous.frozen, previous.PROFILES
require, digest, read_json, rows, add_pin, source_closure = previous.require, previous.digest, previous.read_json, previous.rows, previous.add_pin, previous.source_closure
strict_equal = previous.previous.strict_equal
endpoint_pins = previous.endpoint_pins
measured_rear_errors = previous.previous.measured_rear_errors
SCHEMA = "portable_e2e.actual_route_joint_thread4_signed45_archive_audit.20261002.v1"
RECEIPT_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.actual_route_joint_thread4_signed45_three_arms.20261002.v1"
COLLECTOR_SCHEMA = "portable_e2e.actual_route_joint_thread4_signed45.20261002.v1"
THREAD_SCHEMA = "portable_e2e.route_joint_thread4_numerical_preset.20261002.v1"
OLD_AUDIT_SCHEMA = previous.SCHEMA
BASELINE_SHA = "80c903d757a48f406421176771dfbba31d58ad5752251089154f9e8b7b38f172"
BASELINE_BATCH = ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_signed45_town07_v1"
BENCHMARK = ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_cpu_threads_v1"
OWNER_SOURCE = ROOT / "tools/e2e/research_temporal_route_joint_thread4_signed45_town07_20261002.py"
NEW_FROZEN = dict(previous.NEW_FROZEN, **{
    "tools/e2e/audit_temporal_route_joint_signed45_batch_20261002.py": "844f1586eaf331b2ff867e265c5955322980f5d172b7a7c1375ce9de4a2621f3",
    "tools/e2e/research_temporal_route_joint_thread4_signed45_town07_20261002.py": "8130063784148eda4a3ec985e42918ca54aba8cec948761844e87bcb82915ce4",
    "tools/e2e/collect_temporal_route_joint_thread4_signed45_20261002.py": "c4620641216713be49213edcefbf6a2cb5c57978fbcbd4b5f4e529f6d8500e15",
})
FROZEN_HELPERS = dict(previous.FROZEN_HELPERS, **NEW_FROZEN)
_verify, _extra, _build = previous.verify_contract, previous.previous.extra_trial_evidence, previous._baseline_build
_active = False


def verify_frozen():
    previous.verify_frozen()
    for name, sha in FROZEN_HELPERS.items(): require(digest(ROOT/name) == sha, "unknown thread4 audit dependency: " + name)


def pure_preset():
    """HH_261002 - Read only the pinned owner's literal numerical declaration; never import Torch or a CARLA owner."""
    tree = ast.parse(OWNER_SOURCE.read_bytes())
    assignments = [node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "BENCHMARK_PINS" for target in node.targets)]
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "numerical_preset"]
    require(len(assignments) == len(functions) == 1, "exact fixed numerical preset declaration required")
    pins = ast.literal_eval(assignments[0].value)
    namespace = dict(live=SimpleNamespace(THREAD_SCHEMA=THREAD_SCHEMA), BENCHMARK_PINS=pins)
    exec(compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
        str(Path(__file__)) + ":literal_thread4_preset_only", "exec"), namespace)
    return namespace["numerical_preset"](), pins


def benchmark_proof(plan):
    preset, pins = pure_preset()
    require(strict_equal(plan.get("cpu_thread_numerical_preset"), preset)
        and plan.get("numerical_preset_is_not_exact_equivalence_or_model_approval") is True
        and plan.get("CPU_thread_environment") == dict(OMP_NUM_THREADS="4", MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="4"),
        "declared floating thread preset must not masquerade as strict equivalence")
    assets = {str(BENCHMARK/name): sha for name, sha in pins.items()}
    require(all(digest(name) == sha for name, sha in assets.items()), "original offline receipts changed")
    declaration = read_json(BENCHMARK/"declaration.json"); summary = read_json(BENCHMARK/"summary.json")
    require(declaration["input_spec"]["final_endpoint"] == previous._endpoint
        and declaration["thread_arms"] == [1,2,4] and declaration["interop_threads"] == 1
        and declaration["fixed_camera_ordinals"] == list(range(55,183)), "same final6000 complete offline window required")
    require(summary["all_raw_decoded_selected_proposed_control_equivalent"] is False
        and summary["all_exact_normalized_inputs_equal"] is True and summary["all_failures_retained"] is True
        and summary["actual_physical_latency_or_driving_pass"] is False, "retain original strict-parity FALSE/offline-only outcome")
    require([row["threads"] for row in summary["attempts"]] == [1,2,4]
        and all(row["result_sha256"] == pins["threads_%d.json" % row["threads"]] for row in summary["attempts"]), "all three benchmark arms required")
    for section in ("source_and_asset_sha256", "archived_source_and_asset_sha256"):
        require(isinstance(declaration[section], dict) and declaration[section], "complete original live/archive benchmark proof required")
        for name, sha in declaration[section].items():
            path = Path(name)
            require(path.is_absolute() and path.resolve() == path and path.is_relative_to(ROOT) and not path.is_symlink(), "benchmark proof escaped canonical root")
            require(name not in assets or assets[name] == sha, "benchmark pin conflict")
            assets[name] = sha
    require(plan.get("benchmark_assets_sha256") == assets
        and all(plan["asset_sha256"].get(name) == sha for name, sha in assets.items()), "all original benchmark proofs must be declared/archived assets")
    expected_bytes = sum(Path(name).stat().st_size for name in assets)
    require(plan.get("sealed_benchmark_asset_archive_bytes_per_trial") == expected_bytes
        and plan.get("additional_three_trial_benchmark_archive_reserve_bytes") == 3*expected_bytes, "complete benchmark copy reserve differs")
    return preset


def verify_contract(plan, trial):
    _verify(plan, trial)
    benchmark_proof(plan)


def independent_thread_readback(episode, manifest, diagnostic, pins):
    path = episode/"cpu_thread_readback.jsonl"; add_pin(pins, path)
    settings, timing = rows(path), rows(episode/"wall_timing.jsonl")
    expected = [(row["command_frame"], row["command_reason"]) for row in timing
        if row.get("command_reason") in ("initial_drive_control", "next_driving_control") and row.get("command_frame") is not None]
    actual = [(row.get("command_frame"), row.get("command_reason")) for row in settings]
    passed = bool(expected) and actual == expected and len(actual) == len(set(actual)) and all(
        row.get("schema") == THREAD_SCHEMA and type(row.get("intra_op_threads")) is int and row["intra_op_threads"] == 4
        and type(row.get("inter_op_threads")) is int and row["inter_op_threads"] == 1
        and row.get("before_original_propose") is True and type(row.get("observed_monotonic_ns")) is int
        and row["observed_monotonic_ns"] > 0 for row in settings)
    # HH_261002 - Independently bracket each readback within the original measured proposal/ACK wall-clock interval.
    normal = [row for row in timing if row.get("command_reason") in ("initial_drive_control", "next_driving_control") and row.get("command_frame") is not None]
    passed = passed and all(row["proposal_started_monotonic_ns"] <= setting["observed_monotonic_ns"]
        <= row["proposal_finished_monotonic_ns"] <= row["ACK_started_monotonic_ns"] <= row["ACK_finished_monotonic_ns"]
        for setting, row in zip(settings, normal))
    expected_manifest = dict(schema=THREAD_SCHEMA,intra_op_threads=4,inter_op_threads=1,
        strict_original_offline_parity_passed=False,exact_bitwise_or_strict_decoded_equivalence_claimed=False,
        readback_journal="cpu_thread_readback.jsonl",every_normal_proposal_checked_before_handoff=True,
        hard_policy_deadline_seconds=.1,observed_bundle_to_ACK_deadline_seconds=.1,
        no_safety_gate_or_prediction_replacement=True,model_promoted=False,vehicle_control_approved=False)
    require(strict_equal(manifest["capture_contract"].get("cpu_thread_numerical_preset"),expected_manifest), "thread4 actual manifest differs")
    reported = diagnostic.get("actual_thread_settings", {})
    require(reported.get("passed") is passed and reported.get("normal_proposal_denominator") == len(expected)
        and reported.get("readback_count") == len(actual), "independent complete readback census differs from owner")
    return dict(passed=passed,normal_proposal_denominator=len(expected),readback_count=len(actual),
        before_prefills_handoff_fresh_and_held=True,within_original_proposal_to_ACK_wall_clock=True,
        interop_scope="isolated process1, not reversible global scope",source_sha256={str(path):pins[str(path)]})


def extra_trial_evidence(manifest, states, commands, receipts, route, plan, diagnostic, pins, episode):
    result = _extra(manifest, states, commands, receipts, route, plan, diagnostic, pins, episode)
    result["independent_cpu_thread_preset"] = independent_thread_readback(episode,manifest,diagnostic,pins)
    return result


def transformed_trial_reader():
    """HH_261002 - Preserve the frozen900/450/904 reader verbatim except new metadata label/evidence/conjunctive thread guard."""
    legacy = previous.previous.previous
    tree = ast.parse(Path(legacy.__file__).read_bytes())
    function = next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name == "load_trial")
    original = copy.deepcopy(function)
    anchors = [i for i,node in enumerate(function.body) if isinstance(node,ast.Expr) and isinstance(node.value,ast.Call)
        and any(isinstance(arg,ast.Constant) and arg.value == "independent driving/wall result differs from caller-pinned report" for arg in node.value.args)]
    require(len(anchors) == 1, "frozen raw-derived driving comparison anchor changed")
    injected = ast.parse("""extra = extra_trial_evidence(manifest,states,commands,receipts,read_json(episode/'route.json'),plan,diagnostic,pins,episode)
basic_driving_before_new_recovery = driving
driving = driving and extra['independent_actual_spawn']['actual_signed_spawn_identity_passed'] and extra['independent_manual1_initialization']['manual_first_gear_initialization_passed'] and extra['independent_signed_recovery']['case_screen_passed'] and extra['independent_cpu_thread_preset']['passed']
""").body
    function.body[anchors[0]:anchors[0]] = injected
    returned = function.body[-1]; require(isinstance(returned,ast.Return) and isinstance(returned.value,ast.Call), "frozen trial return differs")
    revision = next(item for item in returned.value.keywords if item.arg == "training_data_revision")
    original_revision = copy.deepcopy(revision.value)
    require(ast.dump(original_revision,include_attributes=False) == ast.dump(ast.parse("metadata['recovery_data_revision']",mode="eval").body,include_attributes=False), "original revision label anchor differs")
    revision.value = ast.parse("metadata['current_training_purpose']",mode="eval").body
    added = [ast.keyword(arg="signed_recovery_profile",value=ast.parse("plan['signed_recovery_profile']",mode="eval").body),
        ast.keyword(arg="basic_driving_before_new_recovery_passed",value=ast.Name(id="basic_driving_before_new_recovery",ctx=ast.Load())),
        ast.keyword(arg=None,value=ast.Name(id="extra",ctx=ast.Load()))]
    returned.value.keywords.extend(added)
    parity = copy.deepcopy(function); del parity.body[anchors[0]:anchors[0]+len(injected)]; del parity.body[-1].value.keywords[-len(added):]
    next(item for item in parity.body[-1].value.keywords if item.arg == "training_data_revision").value = original_revision
    require(ast.dump(parity,include_attributes=False) == ast.dump(original,include_attributes=False), "all original raw/archive/physical gate statements must remain identical")
    namespace = dict(vars(legacy)); namespace.update(verify_contract=verify_contract,extra_trial_evidence=extra_trial_evidence,
        audit_post_ACK_recording=previous.previous.writer.audit_post_ACK_recording)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),str(Path(__file__))+":exact_original_reader_plus_thread_witness","exec"),namespace)
    return namespace["load_trial"]


@contextmanager
def activated(endpoint):
    global _active
    require(not _active and not previous._active, "thread4 audit scope cannot nest")
    verify_frozen()
    values = dict(SCHEMA=SCHEMA,RECEIPT_SCHEMA=RECEIPT_SCHEMA,OWNER_SCHEMA=OWNER_SCHEMA,COLLECTOR_SCHEMA=COLLECTOR_SCHEMA,
        FROZEN_HELPERS=FROZEN_HELPERS,NEW_FROZEN=NEW_FROZEN,verify_contract=verify_contract,transformed_trial_reader=transformed_trial_reader)
    saved = {name:getattr(previous,name) for name in values}; _active=True
    try:
        for name,value in values.items():setattr(previous,name,value)
        with previous.activated(endpoint):yield
    finally:
        for name,value in saved.items():setattr(previous,name,value)
        _active=False


def parent_comparison(report,caller,pins):
    identity=caller.get("baseline_report")
    require(isinstance(identity,dict) and set(identity)=={"path","sha256"} and identity["sha256"]==BASELINE_SHA,
        "externally pinned complete joint1-thread baseline report required")
    path=Path(identity["path"]);require(path.is_absolute() and path.resolve()==path and path.is_relative_to(ROOT),"canonical baseline audit path required")
    add_pin(pins,path,BASELINE_SHA); old=read_json(path)
    require(old["schema"]==OLD_AUDIT_SCHEMA and old["batch"]==str(BASELINE_BATCH)
        and old["prospective_trials"]==3 and old["all_failed_tail_retained"] is True
        and old["final_endpoint_sha256"]==report["final_endpoint_sha256"]
        and [row["trial_index"] for row in old["trials"]]==[0,1,2]
        and [row["signed_recovery_profile"] for row in old["trials"]]==list(PROFILES)
        and all(row["source_checkpoint_sha256"]==report["final_endpoint_sha256"]["checkpoint_sha256"]
            and row["fixed_export_sha256"]==dict(weights=report["final_endpoint_sha256"]["weights_sha256"],metadata=report["final_endpoint_sha256"]["metadata_sha256"])
            and row["actual_native_state_count"]==900 and row["actual_camera_bundle_count"]==450 and row["actual_ACK_count"]==904
            for row in old["trials"]),"same weights/allthree joint1-thread baseline required")
    for name,sha in old["source_sha256"].items():add_pin(pins,name,sha)
    metrics=("joint_motion_body_policy_wall100ms_passed","native_actor_progress_m","post_handoff_base_norm_maximum_mps",
        "post_handoff_actor_CTE_p95_m","post_handoff_actor_CTE_maximum_m","delivery_to_ACK_ms","fresh_policy100ms_passed","original_body_screen_passed")
    comparisons=[dict(trial_index=b["trial_index"],case_id=b["signed_recovery_profile"]["case_id"],
        baseline={key:a[key] for key in metrics},thread4={key:b[key] for key in metrics}) for a,b in zip(old["trials"],report["trials"])]
    return dict(baseline_report_path=str(path),baseline_report_sha256=BASELINE_SHA,
        baseline_joint_pass_count=old["joint_motion_body_policy_wall_trial_pass_count"],thread4_joint_pass_count=report["joint_motion_body_policy_wall_trial_pass_count"],
        same_fixed_final6000_weights_profiles_seeds_teacher_PI_and_thresholds=True,
        difference_is_CPU_thread_numerical_preset_not_new_weights=True,strict_offline_parity_passed=False,
        all_three_parent_and_candidate_attempts_retained=True,no_best_attempt_selection=True,per_arm=comparisons)


def build_report(*args):
    caller=read_json(args[2]); endpoint=endpoint_pins(caller.get("final_endpoint"))
    with activated(endpoint): result=_build(*args)
    for path in source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"):add_pin(result["source_sha256"],path)
    add_pin(result["source_sha256"],previous.RUN/"declaration.json",endpoint["declaration_sha256"])
    result.pop("original_frozen97_check_enforced_by_pinned_numeric_loader",None)
    result.update(optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,final_endpoint_sha256=endpoint,
        no_torch_pickle_loaded=True,numeric_NPZ_values_not_reloaded_by_this_offline_auditor=True,
        model_weights_and_teacher_controller_unchanged=True,CPU_thread_numerical_preset_changed=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False,
        cpu_thread_numerical_preset=pure_preset()[0],all_three_complete_thread_readbacks_passed=all(row["independent_cpu_thread_preset"]["passed"] for row in result["trials"]))
    result["parent_comparison"]=parent_comparison(result,caller,result["source_sha256"])
    frozen.assert_pins(result["source_sha256"]);return result


def markdown(report):
    def n(value):return "미측정" if value is None else "%.3f"%value
    lines=["# Town07: 같은 모델의 CPU1 → CPU4 실제 주행 비교","","<!-- HH_261002 - Retain strict numerical parity FALSE and every unchanged physical threshold/outcome. -->","",
        "전체 기준: CPU1 %d/3 → CPU4 %d/3. 좌·우 복귀 %d/2, 중앙 비교는 별도입니다." % (report["parent_comparison"]["baseline_joint_pass_count"],report["joint_motion_body_policy_wall_trial_pass_count"],report["signed_recovery_pass_count"]),"",
        "| 조건 | CPU1 → CPU4 전체 기준 | 전진 | CTE95% | 관측→ACK 최대 |","|---|---|---:|---:|---:|"]
    for row in report["parent_comparison"]["per_arm"]:
        a,b=row["baseline"],row["thread4"]
        lines.append("| %s | %s → %s | %s → %s m | %s → %s m | %s → %s ms |"%(row["case_id"],"PASS" if a["joint_motion_body_policy_wall100ms_passed"] else "FAIL","PASS" if b["joint_motion_body_policy_wall100ms_passed"] else "FAIL",n(a["native_actor_progress_m"]),n(b["native_actor_progress_m"]),n(a["post_handoff_actor_CTE_p95_m"]),n(b["post_handoff_actor_CTE_p95_m"]),n(a["delivery_to_ACK_ms"].get("maximum")),n(b["delivery_to_ACK_ms"].get("maximum"))))
    lines += ["","가중치·출발 위치/seed·전문가 출발·PI2.40·90m·native 속도2.5·차체900/sweep899 및10cm·정책/관측→ACK100ms는 그대로입니다. 바뀐 것은 CPU intra-op4/interop1 preset이며 실제 모든 normal 제안(초기화/인계/새 추론/held)의 설정과 원래 wall-clock 구간을 독립 확인합니다.","",
        "이 변경은 엄격한 수치 동등성 통과가 아닙니다. 기존 고정128개 JPEG 복원 입력 benchmark의 동일 tensor SHA/raw1e-6는 통과했지만 decoded/selected1e-6는 실패했습니다. 관측된 차이는 XY최대3.46µm·속도1.94e-5m/s입니다. 원래 FALSE를 보존하고 예측/안전 기준을 교체하거나 허용오차를 완화하지 않습니다. Offline warm-window 결과는 실제 센서/ACK 성능의 승인이 아닙니다.","",
        "세 실행의 전체45초·2700상태·1350카메라묶음·8100JPEG·2712ACK와 모든 실패 꼬리를 남깁니다. 센서 노출/callback 진입부터의 완전한10Hz는 미측정입니다. 정식 MAIN9 승인0/9·하위0/30, 완전한 도착/정지·회전·다른 맵·실차 승인이 아니며 개발을 계속합니다.","","[독립 결과 JSON](batch_outcome.json) · [원자료/소스SHA](source_evidence.json)",""]
    return "\n".join(lines)


def publish(output,*args):
    output=Path(output);require(not output.exists() and not output.is_symlink(),"fresh create-only thread4 audit output required")
    report=build_report(*args);frozen.assert_pins(report["source_sha256"]);output.mkdir(parents=True,exist_ok=False)
    for name,value in (("batch_outcome.json",report),("source_evidence.json",report["source_sha256"])):
        with (output/name).open("x") as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream:stream.write(markdown(report))
    frozen.assert_pins(report["source_sha256"]);return dict(output=str(output),result_sha256=digest(output/"batch_outcome.json"),all_trials_joint_passed=report["all_trials_motion_body_policy_AND_wall100ms_passed"],simulator_or_GPU_used=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("batch","caller-receipts","output"):parser.add_argument("--"+name,type=Path,required=True)
    for name in ("batch-sha256","caller-receipts-sha256"):parser.add_argument("--"+name,required=True)
    parser.add_argument("--validate-only",action="store_true");args=parser.parse_args(argv)
    inputs=(args.batch,args.batch_sha256,args.caller_receipts,args.caller_receipts_sha256)
    result=build_report(*inputs) if args.validate_only else publish(args.output,*inputs)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
