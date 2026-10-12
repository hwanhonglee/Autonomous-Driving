"""HH_261002 - Publish all preview08 attempts with frozen full45s actual cameras and vehicle-centered prior-ACK geometry."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path

from PIL import ImageDraw

from . import audit_temporal_route_joint_preview08_signed45_batch_20261002 as audit
from . import publish_temporal_route_joint_thread4_signed45_20261002 as previous

SCHEMA = "portable_e2e.actual_route_joint_preview08_signed45_all_attempt_publication.20261002.v1"
FROZEN_PUBLISHER_SHA = "266d60fce839ffbaa4768654a7c67106bee53bef2dfcc31e68d507f08f256c94"
_verify, _render, _load, _publish = previous.verify_joint_report, previous.render, previous.load_archive, previous._publish
_active = False
indices, png_indices, displayed_receipt = previous.indices, previous.png_indices, previous.displayed_receipt


def verify_joint_report(report):
    # HH_261002 - Preserve exact101/final6000 lineage while denying the false claim that preview-only changed weights or CPU threads.
    previous.previous.previous.verify_signed_report(report)
    fixed = dict(optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,semantic_head_parameters_frozen=True,
        original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,all101_tensor_values_independently_reloaded_by_this_audit=False,no_torch_pickle_loaded=True,
        model_weights_unchanged=True,teacher_startup_and_PI2p4_parameters_unchanged=True,CPU_thread_numerical_preset_changed=False,
        learned_tracker_speed_preview_changed=True,model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False)
    audit.require(all(audit.strict_equal(report.get(k),v) for k,v in fixed.items())
        and "original_frozen97_check_enforced_by_pinned_numeric_loader" not in report,"honest same final6000/CPU4/54learned47fixed preview-only publication required")
    audit.endpoint_pins(report.get("final_endpoint_sha256"))
    audit.require(audit.strict_equal(report.get("cpu_thread_numerical_preset"),audit.thread.pure_preset()[0])
        and audit.strict_equal(report.get("causal_selected_speed_preview_calibration"),audit.calibration_contract()),"same disclosed floatingCPU4 and exact preview08 intervention required")
    comparison = report.get("parent_comparison",{})
    audit.require(comparison.get("baseline_report_path") == str(audit.BASELINE) and comparison.get("baseline_report_sha256") == audit.BASELINE_SHA
        and comparison.get("same_fixed_final6000_weights_profiles_seeds_teacher_PI_threads_and_old_thresholds") is True
        and comparison.get("difference_is_causal_speed_preview_not_new_weights_or_changed_threads") is True
        and comparison.get("additional_rate_guard_applied_to_both_all_three_arms") is True
        and comparison.get("original_baseline_results_rewritten") is False
        and comparison.get("all_three_parent_and_candidate_attempts_retained") is True and comparison.get("no_best_attempt_selection") is True
        and len(comparison.get("per_arm",[])) == 3,"all three sameCPU4 original0.3s baseline outcomes required")
    for field,key in (("all_three_complete_thread_readbacks_passed","independent_cpu_thread_preset"),
        ("all_three_actual_preview_readbacks_passed","independent_preview_configuration"),
        ("all_three_additive_native_acceleration_screens_passed","independent_additive_native_acceleration")):
        audit.require(type(report.get(field)) is bool and report[field] == all(r[key]["passed"] for r in report["trials"]),"complete independent runtime/speed-rate census differs")


def load_archive(*args,**kwargs):
    # HH_261002 - Use the frozen six-camera/whole-state reader; add only explicit new report/source identity checks.
    audit.require(_active,"explicit preview publication scope required")
    report,bundles,pins = previous._load(*args,**kwargs)
    verify_joint_report(report)
    for path in audit.source_closure(Path(__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py"):
        audit.add_pin(pins,path)
    audit.add_pin(pins,Path(previous.__file__),FROZEN_PUBLISHER_SHA)
    audit.add_pin(pins,audit.ROOT/"tools/e2e/publish_temporal_route_joint_signed45_v2_20261002.py",previous.ROOT_ALIAS_ADAPTER_SHA)
    audit.frozen.assert_pins(pins); return report,bundles,pins


def render(bundle,index):
    # HH_261002 - Change text only; all real camera pixels, centered geometry and latest actual ANY-source prior ACK stay unchanged.
    canvas = previous._render(bundle,index); draw = ImageDraw.Draw(canvas)
    visual = previous.previous.previous.previous.visual
    row = bundle["trial"]
    draw.rectangle((0,0,1699,53),fill=visual.BG)
    draw.text((24,18),"Actual Town07 JOINT6000 CPU4 PREVIEW0.8 %s | ALL GATES %s"%(
        row["signed_recovery_profile"]["case_id"],"PASS" if row["joint_motion_body_policy_wall100ms_passed"] else "FAIL"),fill=visual.WHITE,font=visual.font(23,True))
    draw.rectangle((0,801,1699,859),fill=visual.BG)
    draw.text((24,807),"Same weights / CPU4 / PI2.40 / full45s | Only speed preview0.3 to0.8, original STOP guard | MAIN9 0/9",fill=visual.WHITE,font=visual.font(16))
    draw.text((24,835),"Real six cameras + vehicle-centered diagnostic / prior ANY ACK / all failed tails | NOT Autoware/RViz",fill=visual.MUTED,font=visual.font(16))
    return canvas


@contextmanager
def activated():
    global _active
    audit.require(not _active and not previous._active,"preview publisher cannot nest")
    audit.verify_frozen()
    audit.require(audit.digest(Path(previous.__file__)) == FROZEN_PUBLISHER_SHA and previous.verify_joint_report is _verify
        and previous.render is _render and previous.load_archive is _load,"unknown frozen full-duration publisher binding")
    names = dict(audit=audit,SCHEMA=SCHEMA,verify_joint_report=verify_joint_report,render=render,load_archive=load_archive)
    saved = {k:getattr(previous,k) for k in names}; _active = True
    try:
        for k,v in names.items(): setattr(previous,k,v)
        with previous.activated(): yield
    finally:
        for k,v in saved.items(): setattr(previous,k,v)
        _active = False


def publish(output,report_path,report_sha):
    output = Path(output)
    audit.require(not output.exists() and not output.is_symlink() and output.absolute() == output.resolve()
        and not any(p.is_symlink() for p in output.parents),"fresh canonical create-only preview media output required")
    # HH_261002 - Verify the canonical batch before rendering; never publish inside or overwrite the immutable run.
    audit.require(Path(report_path).absolute() == Path(report_path).resolve(),"canonical independent report required")
    first_read_pin = {}
    audit.add_pin(first_read_pin,report_path,report_sha)
    input_report = audit.read_json(report_path)
    audit.frozen.assert_pins(first_read_pin)
    audit.require(not output.is_relative_to(Path(input_report["batch"]).resolve()) and not output.is_relative_to(Path(report_path).parent.resolve()),
        "media output cannot modify raw batch or independent report folder")
    with activated():
        audit.frozen.assert_pins(first_read_pin)
        result = _publish(output,report_path,report_sha)
    result.update(cpu_intra_op_threads=4,cpu_inter_op_threads=1,original_speed_preview_seconds=.3,configured_speed_preview_seconds=.8,
        same_fixed_final6000_weights_and_threads=True,same_controller_except_declared_preview=False,
        all_three_full45s_and_failures_published=True,actual_media_count=18,GIF_count=3,PNG_count=15,autoware_UI_captured=False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--report",type=Path,required=True); parser.add_argument("--report-sha256",required=True); parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--validate-only",action="store_true"); parser.add_argument("--render-all-media",action="store_true")
    args = parser.parse_args(argv); audit.require(args.validate_only != args.render_all_media,"choose validation or root-authorized quiet rendering")
    if args.validate_only:
        with activated(),previous.previous.activated(): report,bundles,_ = load_archive(args.report,args.report_sha256)
        result = dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False,output_created=False)
    else: result = publish(args.output,args.report,args.report_sha256)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
