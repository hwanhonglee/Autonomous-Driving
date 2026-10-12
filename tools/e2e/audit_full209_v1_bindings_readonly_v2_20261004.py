"""HH_261004 - Recompute authentic V1 metadata bindings in a fresh folder without regenerating or regrading the frozen drive."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from . import audit_town03_full209_diagnostic_v2_20261004 as fresh
from . import benchmark_full209_JPEG_normalization_v1_20261004 as sealed


ROOT = sealed.ROOT
PLAN = ROOT/"storage/experiments/plans/2026-10-04/full209_v1_bindings_readonly_v2/declaration.json"
OUTPUT = ROOT/"storage/experiments/diagnostics/2026-10-04/full209_v1_bindings_readonly_v2"
SCHEMA = "portable_e2e.full209_v1_bindings_readonly.20261004.v2"
ADAPTER_SHA256 = "57dfa5f1ee076e6c4f698cb7cb88b97cc24ca58a8b6fd343d5190eb681e6e11a"
ADAPTER_TEST_SHA256 = "7ebd96db3fae6fabae1c824a42c3440417d0ca2b53578fb58f0c431026874301"


def sources():
    value = fresh.verify_frozen()
    value.update(sealed.source_snapshot())
    for name,sha in {
        "tools/e2e/audit_town03_full209_diagnostic_v2_20261004.py": ADAPTER_SHA256,
        "learning/tests/test_audit_town03_full209_diagnostic_v2_20261004.py": ADAPTER_TEST_SHA256,
    }.items():
        sealed.require(sealed.digest(ROOT/name) == sha,"approved fresh diagnostic source changed")
        value[name] = sha
    value[str(Path(__file__).resolve().relative_to(ROOT))] = sealed.digest(Path(__file__))
    return value


def declaration():
    sealed.require(not PLAN.exists() and not OUTPUT.exists(),"fresh readonly plan/output required")
    document = dict(schema=SCHEMA+".prospective", provenance="HH_261004 - Seal whole actual data and new parser before independent read-only recomputation.",
        sources=sources(), inputs=sealed.input_snapshot(), environment=sealed.environment(),
        input_root=str(sealed.RUN), output=str(OUTPUT), actual_native_count=1836,
        actual_camera_count=918, actual_ACK_count=1840, actual_fresh_NN_count=883,
        no_full_body_regeneration_or_write_in_original_run=True,
        original_V1_FAIL_must_remain=True, original_thresholds_unchanged=True,
        learned_STOP_or_feature_approval=False, automatic_promotion=False)
    sealed.write_new_json(PLAN,document)
    return dict(status="DECLARED_NOT_EXECUTED", declaration_sha256=sealed.digest(PLAN))


def execute(expected):
    sealed.require(sealed.digest(PLAN) == expected,"exact prospective readonly declaration required")
    plan=sealed.read_json(PLAN)
    before=dict(sources=sources(),inputs=sealed.input_snapshot(),environment=sealed.environment())
    sealed.require(before == {name:plan[name] for name in before} and plan["schema"] == SCHEMA+".prospective"
        and plan["input_root"] == str(sealed.RUN) and plan["output"] == str(OUTPUT) and not OUTPUT.exists(),
        "prospective readonly closure differs")
    old,live=fresh._parents()
    episode=sealed.EPISODE
    states,cameras,commands,receipts,timings,guards=[old.right.jsonl(episode/name) for name in
        ("states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl","wall_timing.jsonl","native_guard.jsonl")]
    manifest=sealed.read_json(episode/"manifest.json")
    wrapper=sealed.read_json(episode/"catalog_goal_cleanup_receipt.json")
    original=sealed.read_json(sealed.RUN/"diagnostic_report.json")
    sealed.require(len(states)==1836 and len(cameras)==918 and len(receipts)==len(timings)==1840
        and original["status"]=="FAIL_KNOWN_DEV_FULL209_RESEARCH","whole immutable actual V1 failed census required")
    result=dict(schema=SCHEMA, provenance="HH_261004 - New metadata diagnosis only; retain original actual latency failures and formal0/9,0/30.",
        original_result_status=original["status"], declaration_sha256=expected,
        actual_native_count=len(states), actual_camera_count=len(cameras),actual_ACK_count=len(receipts),
        original_result_rewritten=False, formal_function_approved=False,
        official_completed_main_domains=0,official_completed_subfunctions=0,
        learned_STOP_claim=False, model_promoted=False, vehicle_control_approved=False,checks={},errors={})
    operations = {
        "authenticated_goal":lambda:fresh.audit_goal_receipt(states,commands,receipts,timings,guards,wrapper,manifest),
        "real_extended_ACK":lambda:fresh.analyze_transport(states,receipts,
            manifest["capture_contract"]["control_transport"]["bootstrap_observation"],
            wheelbase=manifest["coordinate_contract"]["wheelbase_m"],commands=commands,timings=timings,
            native_guards=guards,wrapper=wrapper,manifest=manifest),
        "exact_terminal_witness_and_decoder":lambda:fresh.audit_terminal_navigation(
            (sealed.RUN/"source"/old.ROUTE.relative_to(ROOT)).read_bytes(),states,commands,manifest=manifest,
            stream_source_sha256=sealed.read_json(sealed.RUN/"owner_result.json")["declaration_sha256"],
            episode_id="temporal_v2_"+sealed.RUN.name),
        "independent_interval_and_recording":lambda:fresh.independent_interval_and_recording(
            sealed.RUN,states=states,cameras=cameras,commands=commands,timings=timings,manifest=manifest),
    }
    for name,operation in operations.items():
        try:result["checks"][name]=operation()
        except Exception as error:result["errors"][name]=dict(type=type(error).__name__,message=str(error))
    try:
        callback=old.read_callback_receipts(episode/"callback_timing_receipts.json",sealed.digest(episode/"callback_timing_receipts.json"))
        census=sealed.read_json(episode/"learned_summary.json")["counts"]
        callback_report=live.audit_receipts(callback,cameras,commands,timings,census)
        result["actual_callback_timing"]=callback_report
        result["checks"]["native_stage_closure"]=live.audit_native_markers(callback,states,cameras,timings,callback_report,
            goal_complete=result["checks"].get("authenticated_goal",{}).get("passed") is True)
        result["checks"]["all_quota_walks"]=live.audit_quota_walks(
            sealed.read_json(episode/"quota_walk_timing_receipts.json"),cameras,timings)
    except Exception as error:result["errors"]["callback_native_quota"]=dict(type=type(error).__name__,message=str(error))
    after=dict(sources=sources(),inputs=sealed.input_snapshot(),environment=sealed.environment())
    result.update(sources_before=before["sources"],sources_after=after["sources"],inputs_before=before["inputs"],inputs_after=after["inputs"],
        whole_input_and_source_environment_unchanged=before==after)
    binding=result["checks"]
    passed=not result["errors"] and before==after \
        and binding.get("authenticated_goal",{}).get("passed") is True \
        and binding.get("real_extended_ACK",{}).get("status")=="PASS" \
        and binding.get("exact_terminal_witness_and_decoder",{}).get("passed") is True \
        and binding.get("independent_interval_and_recording",{}).get("bounded_capture_complete") is True \
        and binding.get("independent_interval_and_recording",{}).get("recording",{}).get("passed") is True \
        and binding.get("native_stage_closure",{}).get("native_pre_delivery_stage_closure_passed") is True \
        and binding.get("all_quota_walks",{}).get("passed") is True
    result["bindings_recomputed_passed"]=passed
    result["status"]="PASS_BINDINGS_ORIGINAL_DRIVE_LATENCY_FAIL" if passed else "FAIL_READONLY_BINDINGS"
    result["original_callback_latency_PASS_claimed"]=False
    sealed.write_new_json(OUTPUT/"report.json",result)
    summary=dict(status=result["status"],bindings_recomputed_passed=passed,errors=result["errors"],
        original_result_status=original["status"],original_result_rewritten=False,
        report_sha256=sealed.digest(OUTPUT/"report.json"),declaration_sha256=expected)
    sealed.write_new_json(OUTPUT/"summary.json",summary)
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("declare","execute"),required=True)
    parser.add_argument("--expected-declaration-sha256")
    args=parser.parse_args(argv)
    sealed.require((args.mode=="execute")==bool(args.expected_declaration_sha256),"execute requires exact declaration SHA")
    sealed.environment(configure=True)
    result=declaration() if args.mode=="declare" else execute(args.expected_declaration_sha256)
    print(json.dumps(result,indent=2,sort_keys=True,allow_nan=False))
    return 0 if result["status"] in ("DECLARED_NOT_EXECUTED","PASS_BINDINGS_ORIGINAL_DRIVE_LATENCY_FAIL") else 1


if __name__=="__main__":raise SystemExit(main())
