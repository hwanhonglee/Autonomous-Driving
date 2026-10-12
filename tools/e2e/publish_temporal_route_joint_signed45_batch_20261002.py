"""HH_261002 - Publish every final6000 signed45 outcome, with genuine baseline comparison and unchanged full45s display semantics."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path

from PIL import ImageDraw

from . import audit_temporal_route_joint_signed45_batch_20261002 as audit
from . import publish_temporal_signed_recovery45_batch_20261002 as previous

SCHEMA="portable_e2e.actual_route_joint_final6000_signed45_all_attempt_publication.20261002.v1"
BASELINE_PUBLISHER_SHA="ce85be061d543d8f7ceb8acef603e8f44fbe99d0c83e332b422a6e2127bfd192"
_render,_load_archive=previous.render,previous.load_archive
_active=False
indices,png_indices,displayed_receipt=previous.indices,previous.png_indices,previous.displayed_receipt


def verify_joint_report(report):
    previous.verify_signed_report(report)
    fixed=dict(optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=True,no_torch_pickle_loaded=True)
    audit.require(all(audit.previous.strict_equal(report.get(k),v) for k,v in fixed.items())
        and "original_frozen97_check_enforced_by_pinned_numeric_loader" not in report,
        "explicit final6000/54learned47fixed publication, not false frozen97, required")
    audit.endpoint_pins(report.get("final_endpoint_sha256"))
    comparison=report.get("parent_comparison",{})
    audit.require(comparison.get("baseline_checkpoint_sha256")==audit.PARENT_CHECKPOINT_SHA
        and comparison.get("all_three_parent_and_candidate_attempts_retained") is True
        and comparison.get("same_actual_profiles_seeds_teacher_PI_and_thresholds") is True
        and len(comparison.get("per_arm",[]))==3,"genuine all-three same-setup parent comparison required")


def load_archive(report_path,report_sha,*,decode_and_pin_JPEG=False):
    audit.require(_active,"explicit joint publication scope required")
    report,bundles,pins=_load_archive(report_path,report_sha,decode_and_pin_JPEG=decode_and_pin_JPEG)
    verify_joint_report(report)
    for path in audit.source_closure(Path(__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py"):
        audit.add_pin(pins,path)
    audit.frozen.assert_pins(pins);return report,bundles,pins


def render(bundle,index):
    # HH_261002 - Reuse all six real cameras, measured ego-centered map, prior ANY-source ACK and complete failure tails without edits.
    canvas=_render(bundle,index);draw=ImageDraw.Draw(canvas);trial=bundle["trial"];spec=trial["signed_recovery_profile"]
    draw.rectangle((0,0,1699,53),fill=previous.previous.visual.BG)
    joint="PASS" if trial["joint_motion_body_policy_wall100ms_passed"] else "FAIL"
    label="CENTER COMPARATOR" if trial["trial_index"]==2 else "SIGNED RECOVERY"
    draw.text((24,18),f"Actual Town07 JOINT6000 {spec['case_id']} | {label} | ALL PHYSICAL GATES {joint}",
        fill=previous.previous.visual.WHITE,font=previous.previous.visual.font(24,True))
    draw.rectangle((0,801,1699,859),fill=previous.previous.visual.BG)
    draw.text((24,807),"Same actual setup / PI2.40 / 45s | NEW54 learned +47 fixed tensors | MAIN9 approval 0/9, work continues",
        fill=previous.previous.visual.WHITE,font=previous.previous.visual.font(17))
    draw.text((24,835),"Real six-camera + ego-centered diagnostic / prior ACK only / all failure tails | NOT Autoware/RViz UI",
        fill=previous.previous.visual.MUTED,font=previous.previous.visual.font(16))
    return canvas


@contextmanager
def activated():
    global _active
    audit.require(not _active and not previous._active,"joint publication cannot nest")
    audit.verify_frozen()
    audit.require(audit.digest(previous.__file__)==BASELINE_PUBLISHER_SHA and previous.render is _render
        and previous.load_archive is _load_archive,"frozen baseline publisher source/binding changed")
    names=dict(audit=audit,SCHEMA=SCHEMA,render=render,load_archive=load_archive)
    saved={k:getattr(previous,k) for k in names};_active=True
    try:
        for k,v in names.items():setattr(previous,k,v)
        with previous.activated():yield
    finally:
        for k,v in saved.items():setattr(previous,k,v)
        _active=False


def publish(output,report_path,report_sha):
    with activated():result=previous._publish(output,report_path,report_sha)
    result.update(signed_recovery_conditions=2,centered_setup_comparators=1,all_failed_outcomes_published=True,
        actual_full_route_or_MAIN9_completion_claimed=False,model_protocol="final6000/54learned47fixed")
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--report",type=Path,required=True);parser.add_argument("--report-sha256",required=True)
    parser.add_argument("--output",type=Path,required=True);parser.add_argument("--validate-only",action="store_true")
    parser.add_argument("--render-all-media",action="store_true",help="Root-authorized rendering after all physical runs finish")
    args=parser.parse_args(argv);audit.require(args.validate_only != args.render_all_media,"choose validation or explicit root quiet rendering")
    if args.validate_only:
        with activated():report,bundles,_=load_archive(args.report,args.report_sha256)
        result=dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False,output_created=False)
    else:result=publish(args.output,args.report,args.report_sha256)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
