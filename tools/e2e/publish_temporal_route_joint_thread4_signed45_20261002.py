"""HH_261002 - Publish all thread4 outcomes using frozen full45s six-camera/prior-ACK rendering."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path

from PIL import ImageDraw

from . import audit_temporal_route_joint_thread4_signed45_batch_20261002 as audit
from . import publish_temporal_route_joint_signed45_batch_20261002 as previous

SCHEMA="portable_e2e.actual_route_joint_thread4_signed45_all_attempt_publication.20261002.v1"
PUBLISHER_SHA="e69c24827331f535fb00d64e6362a88e027a867e7448e5dfdd9310fade403a5b"
ROOT_ALIAS_ADAPTER_SHA="9ba64c8b37c9d2b80c252565196111946fab1ea29b6245210c9546a278fb1115"
_verify,_render,_load,_publish=previous.verify_joint_report,previous.render,previous.load_archive,previous.publish
_active=False
indices,png_indices,displayed_receipt=previous.indices,previous.png_indices,previous.displayed_receipt


def verify_joint_report(report):
    # HH_261002 - Keep the original54/47 and signed/raw-denominator checks without the old claim that weights changed.
    previous.previous.verify_signed_report(report)
    fixed=dict(optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,no_torch_pickle_loaded=True,
        model_weights_and_teacher_controller_unchanged=True,CPU_thread_numerical_preset_changed=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False)
    audit.require(all(audit.strict_equal(report.get(key),value) for key,value in fixed.items())
        and "original_frozen97_check_enforced_by_pinned_numeric_loader" not in report,"exact final6000 unchanged weights/changed numerical preset required")
    audit.endpoint_pins(report.get("final_endpoint_sha256"))
    audit.require(audit.strict_equal(report.get("cpu_thread_numerical_preset"),audit.pure_preset()[0]),"explicit original strict-parity FALSE required")
    comparison=report.get("parent_comparison",{})
    audit.require(comparison.get("baseline_report_sha256")==audit.BASELINE_SHA
        and comparison.get("same_fixed_final6000_weights_profiles_seeds_teacher_PI_and_thresholds") is True
        and comparison.get("difference_is_CPU_thread_numerical_preset_not_new_weights") is True
        and comparison.get("strict_offline_parity_passed") is False
        and comparison.get("all_three_parent_and_candidate_attempts_retained") is True
        and len(comparison.get("per_arm",[]))==3,"same actual allthree CPU1 baseline required")
    audit.require(report.get("all_three_complete_thread_readbacks_passed") == all(
        row.get("independent_cpu_thread_preset",{}).get("passed",False) for row in report["trials"]),"complete runtime readback census differs")


def load_archive(*args,**kwargs):
    # HH_261002 - The new audit exports measured_rear_errors directly; no missing old namespace alias is invoked.
    report,bundles,pins=_load(*args,**kwargs)
    verify_joint_report(report)
    for path in audit.source_closure(Path(__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py"):
        audit.add_pin(pins,path)
    audit.add_pin(pins,audit.ROOT/"tools/e2e/publish_temporal_route_joint_signed45_v2_20261002.py",ROOT_ALIAS_ADAPTER_SHA)
    audit.frozen.assert_pins(pins);return report,bundles,pins


def render(bundle,index):
    canvas=_render(bundle,index);draw=ImageDraw.Draw(canvas);row=bundle["trial"]
    visual=previous.previous.previous.visual
    draw.rectangle((0,0,1699,53),fill=visual.BG)
    draw.text((24,18),"Actual Town07 JOINT6000 CPU4/interop1 %s | ALL PHYSICAL GATES %s"%(
        row["signed_recovery_profile"]["case_id"],"PASS" if row["joint_motion_body_policy_wall100ms_passed"] else "FAIL"),fill=visual.WHITE,font=visual.font(24,True))
    draw.rectangle((0,801,1699,859),fill=visual.BG)
    draw.text((24,807),"Same final6000 weights / PI2.40 / all45s | Changed floating thread preset, strict parity FALSE | MAIN9 0/9",fill=visual.WHITE,font=visual.font(16))
    draw.text((24,835),"Actual six cameras + ego-centered diagnostic / prior ANY ACK / all failure tails | NOT Autoware/RViz",fill=visual.MUTED,font=visual.font(16))
    return canvas


@contextmanager
def activated():
    global _active
    audit.require(not _active and not previous._active,"thread4 publisher cannot nest")
    audit.verify_frozen()
    audit.require(audit.digest(Path(previous.__file__))==PUBLISHER_SHA and previous.verify_joint_report is _verify
        and previous.render is _render and previous.load_archive is _load,"frozen joint publisher binding changed")
    values=dict(audit=audit,SCHEMA=SCHEMA,verify_joint_report=verify_joint_report,render=render,load_archive=load_archive)
    saved={name:getattr(previous,name) for name in values};_active=True
    try:
        for name,value in values.items():setattr(previous,name,value)
        yield
    finally:
        for name,value in saved.items():setattr(previous,name,value)
        _active=False


def publish(output,report_path,report_sha):
    with activated():result=_publish(output,report_path,report_sha)
    result.update(cpu_intra_op_threads=4,cpu_inter_op_threads=1,strict_offline_parity_passed=False,
        same_fixed_final6000_weights_and_controller=True,all_three_full45s_and_failures_published=True)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--report",type=Path,required=True);parser.add_argument("--report-sha256",required=True)
    parser.add_argument("--output",type=Path,required=True);parser.add_argument("--validate-only",action="store_true")
    parser.add_argument("--render-all-media",action="store_true")
    args=parser.parse_args(argv);audit.require(args.validate_only != args.render_all_media,"choose validation or root-authorized quiet rendering")
    if args.validate_only:
        with activated(),previous.activated():report,bundles,_=load_archive(args.report,args.report_sha256)
        result=dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False,output_created=False)
    else:result=publish(args.output,args.report,args.report_sha256)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
