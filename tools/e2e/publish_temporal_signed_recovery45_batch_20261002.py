"""HH_261002 - Publish all actual signed/center outcomes with fixed full45s six-camera and ego-centered map coverage."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path

from PIL import ImageDraw

from . import audit_temporal_signed_recovery45_batch_20261002 as audit
from . import publish_temporal_route_recovery_fast45_batch_20261001 as previous

SCHEMA="portable_e2e.actual_signed_recovery45_all_attempt_publication.20261002.v1"
FROZEN_PUBLISHER_SHA="458ba01eafc74c5382684fc0aff7d3971b1d0b80bbfc615b6c8ca79760f49ee1"
_load_archive,_render,_publish=previous.load_archive,previous.render,previous.publish
_active=False
indices,png_indices,displayed_receipt=previous.indices,previous.png_indices,previous.displayed_receipt


def verify_signed_report(report):
    audit.require(report.get("schema") == audit.SCHEMA and report.get("signed_recovery_case_count") == 2
        and report.get("centered_setup_comparator_count") == 1
        and len(report.get("trials",[])) == 3
        and [r.get("signed_recovery_profile") for r in report["trials"]] == list(audit.PROFILES)
        and report.get("all_three_are_not_same_seed_repetitions") is True
        and report.get("first_effect_is_latest_prior_actual_learned_ACK") is True
        and report.get("no_teacher_recovery_counted_as_learned") is True
        and report.get("full_MAIN9_completion_claimed") is False
        and report.get("full_route_goal_stop_approved") is False
        and report.get("all_failed_tail_retained") is True,
        "independent full three-arm signed report with all failures required")
    audit.require(report["signed_recovery_pass_count"] == sum(r["independent_signed_recovery"]["recovery_screen_passed"]
        for r in report["trials"][:2]) and report["trials"][2]["independent_signed_recovery"]["recovery_screen_passed"] is False,
        "center comparator cannot enter the two-arm recovery denominator")


def load_archive(report_path,report_sha,*,decode_and_pin_JPEG=False):
    audit.require(_active, "explicit restored signed publication scope required")
    report,bundles,pins=_load_archive(report_path,report_sha,decode_and_pin_JPEG=decode_and_pin_JPEG)
    verify_signed_report(report)
    for path in audit.source_closure(Path(__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py"):
        audit.add_pin(pins,path)
    for bundle in bundles:
        projected=audit.measured_rear_errors(bundle["states"],dict(route=bundle["route"]))
        bundle["independent_rear_errors_by_frame"]={r["frame"]:r for r in projected}
    audit.frozen.assert_pins(pins)
    return report,bundles,pins


def render(bundle,index):
    # HH_261002 - Only title/diagnostic text changes; the frozen six real cameras, centered map and latest ANY-source prior ACK stay unchanged.
    canvas=_render(bundle,index); draw=ImageDraw.Draw(canvas)
    camera=bundle["cameras"][index]; row=bundle["trial"]; spec=row["signed_recovery_profile"]
    error=bundle["independent_rear_errors_by_frame"][camera["frame"]]
    recovery=row["independent_signed_recovery"]
    draw.rectangle((0,0,1699,94),fill=previous.visual.BG)
    title="CENTER SETUP COMPARATOR (not recovery)" if row["trial_index"] == 2 else "SIGNED LEARNED RECOVERY"
    joint="PASS" if row["joint_motion_body_policy_wall100ms_passed"] else "FAIL"
    draw.text((24,18),f"Actual Town07 {spec['case_id']} | {title} | JOINT {joint}",fill=previous.visual.WHITE,font=previous.visual.font(25,True))
    draw.text((24,61),f"Spawn {spec['signed_lateral_offset_ros_m']:+.2f}m / {spec['signed_yaw_offset_ros_deg']:+.1f}deg | actual rear {error['signed_rear_CTE_m']:+.3f}m / {error['signed_rear_yaw_error_deg']:+.3f}deg | {recovery['status']}",
        fill=previous.visual.MUTED,font=previous.visual.font(17))
    # HH_261002 - MAIN9 remains incomplete even when bounded signed recovery and wall timing pass.
    draw.rectangle((0,801,1699,859),fill=previous.visual.BG)
    draw.text((24,807),"Full45s / all failure tails / prior ACK only | 2 signed arms + 1 center | MAIN9: 0/9 approved, work continues",fill=previous.visual.WHITE,font=previous.visual.font(17))
    draw.text((24,835),"Real CARLA six-camera + ego-centered diagnostic map | NOT Autoware/RViz UI | NO synthetic driving images",fill=previous.visual.MUTED,font=previous.visual.font(16))
    return canvas


@contextmanager
def activated():
    """HH_261002 - Reuse the frozen full-duration renderer under explicit restored NEW audit/title bindings only."""
    global _active
    audit.require(not _active, "signed publication cannot nest")
    audit.verify_frozen()
    audit.require(audit.digest(previous.__file__) == FROZEN_PUBLISHER_SHA
        and previous.load_archive is _load_archive and previous.render is _render, "unknown frozen publisher source/binding")
    names=dict(audit=audit,SCHEMA=SCHEMA,load_archive=load_archive,render=render)
    saved={name:getattr(previous,name) for name in names};_active=True
    try:
        for name,value in names.items():setattr(previous,name,value)
        yield
    finally:
        for name,value in saved.items():setattr(previous,name,value)
        _active=False


def publish(output,report_path,report_sha):
    with activated():result=_publish(output,report_path,report_sha)
    result.update(signed_recovery_conditions=2,centered_setup_comparators=1,all_failed_outcomes_published=True,
        actual_full_route_or_MAIN9_completion_claimed=False)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--report",type=Path,required=True);parser.add_argument("--report-sha256",required=True)
    parser.add_argument("--output",type=Path,required=True);parser.add_argument("--validate-only",action="store_true")
    parser.add_argument("--render-all-media",action="store_true",help="Root-authorized quiet-window rendering of all three outcomes")
    args=parser.parse_args(argv)
    audit.require(args.validate_only != args.render_all_media, "choose validation or explicit root quiet-window rendering")
    if args.validate_only:
        with activated():report,bundles,_=load_archive(args.report,args.report_sha256)
        result=dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False,output_created=False,
            signed_recovery_conditions=2,centered_setup_comparators=1)
    else:result=publish(args.output,args.report,args.report_sha256)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False));return 0


if __name__ == "__main__":raise SystemExit(main())
