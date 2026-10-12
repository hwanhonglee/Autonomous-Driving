"""HH_261002 - Publish full45s real preview0.5 cameras with all0.3/0.5/0.8 outcomes and no best-trial selection."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import json
from pathlib import Path

from PIL import ImageDraw

from . import audit_temporal_route_joint_preview05_signed45_batch_20261002 as audit
from . import publish_temporal_route_joint_preview08_signed45_20261002 as previous

SCHEMA = "portable_e2e.actual_route_joint_preview05_signed45_all_attempt_publication.20261002.v1"
FROZEN_PUBLISHER_SHA = "8013a9e83745bfb2d0ded8331b946feeba8d295db951896e097b043ff770cc84"
_verify,_render,_load,_publish = previous.verify_joint_report,previous.render,previous.load_archive,previous.publish
_active = False
indices,png_indices,displayed_receipt = previous.indices,previous.png_indices,previous.displayed_receipt


def verify_joint_report(report):
    # HH_261002 - Reuse exact54/47/final6000 physical publication checks; add the full prospective bracket identity.
    tree = ast.parse(Path(previous.__file__).read_bytes())
    function = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "verify_joint_report")
    namespace = dict(vars(previous),audit=audit)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),str(Path(__file__))+":unchanged54_47_publication_checks","exec"),namespace)
    namespace["verify_joint_report"](report)
    audit.require(report.get("schema") == audit.SCHEMA and report.get("original0p3_and0p8_reports_and_all_failed_source_outputs_preserved") is True,
        "new independent0.5 audit and both prior failed studies required")
    bracket = report.get("all_preview_bracket_comparison",{})
    fixed = dict(configured_preview_seconds=[.3,.5,.8],same_fixed_final6000_weights_CPU4_PI2p4_profiles_seeds_and_old_thresholds=True,
        original0p3_report=dict(path=str(audit.BASELINE),sha256=audit.BASELINE_SHA),
        original0p8_report=dict(path=str(audit.PREVIEW08_REPORT),sha256=audit.PREVIEW08_REPORT_SHA),
        all_nine_actual_attempts_retained=True,no_best_trial_or_checkpoint_selection=True,old_outcomes_and_source_files_rewritten=False)
    audit.require(all(audit.strict_equal(bracket.get(k),v) for k,v in fixed.items()) and len(bracket.get("per_arm",[])) == 3,"all nine actual attempts and unchanged threshold bracket required")
    for i,arm in enumerate(bracket["per_arm"]):
        audit.require(arm["trial_index"] == i and arm["case_id"] == audit.PROFILES[i]["case_id"]
            and set(arm) == {"trial_index","case_id","preview03","preview05","preview08","preview03_additive_native_rate","preview05_additive_native_rate","preview08_additive_native_rate"},"complete fixed signed/center comparison required")
        candidate = report["trials"][i]
        audit.require(audit.equivalent(arm["preview05"],{k:candidate[k] for k in audit.METRICS})
            and audit.equivalent(arm["preview05_additive_native_rate"],candidate["independent_additive_native_acceleration"]),"published0.5 summary cannot substitute a prior attempt")
    audit.require(all(type(bracket.get("pass_counts",{}).get(key)) is int and bracket["pass_counts"][key] == sum(r[key]["joint_motion_body_policy_wall100ms_passed"] for r in bracket["per_arm"])
        for key in ("preview03","preview05","preview08")),"all three per-setting pass denominators differ")


def load_archive(*args,**kwargs):
    audit.require(_active,"explicit0.5 publication scope required")
    report,bundles,pins = _load(*args,**kwargs)
    verify_joint_report(report)
    for path in audit.source_closure(Path(__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py"): audit.add_pin(pins,path)
    audit.add_pin(pins,Path(previous.__file__),FROZEN_PUBLISHER_SHA)
    audit.frozen.assert_pins(pins); return report,bundles,pins


def render(bundle,index):
    # HH_261002 - Alter text only; frozen full six-camera pixels/centered geometry/prior ACK and failed tails remain exact.
    canvas = _render(bundle,index); draw = ImageDraw.Draw(canvas)
    visual = previous.previous.previous.previous.previous.visual
    row = bundle["trial"]
    draw.rectangle((0,0,1699,53),fill=visual.BG)
    draw.text((24,18),"Actual Town07 JOINT6000 CPU4 PREVIEW0.5 %s | ALL GATES %s"%(
        row["signed_recovery_profile"]["case_id"],"PASS" if row["joint_motion_body_policy_wall100ms_passed"] else "FAIL"),fill=visual.WHITE,font=visual.font(23,True))
    draw.rectangle((0,801,1699,859),fill=visual.BG)
    draw.text((24,807),"Same weights / CPU4 / PI2.40 / full45s | Complete preview0.3 /0.5 /0.8 bracket | MAIN9 0/9",fill=visual.WHITE,font=visual.font(16))
    draw.text((24,835),"Real six cameras + vehicle-centered diagnostic / prior ANY ACK / all failed tails | NOT Autoware/RViz",fill=visual.MUTED,font=visual.font(16))
    return canvas


@contextmanager
def activated():
    global _active
    audit.require(not _active and not previous._active,"new0.5 publisher cannot nest")
    audit.verify_frozen()
    audit.require(audit.digest(Path(previous.__file__)) == FROZEN_PUBLISHER_SHA and previous.verify_joint_report is _verify
        and previous.render is _render and previous.load_archive is _load and previous.publish is _publish,"unknown frozen0.8 publisher binding")
    values = dict(audit=audit,SCHEMA=SCHEMA,verify_joint_report=verify_joint_report,render=render,load_archive=load_archive)
    saved = {k:getattr(previous,k) for k in values}; _active = True
    try:
        for k,v in values.items(): setattr(previous,k,v)
        yield
    finally:
        for k,v in reversed(tuple(saved.items())): setattr(previous,k,v)
        _active = False


def publish(output,report_path,report_sha):
    output = Path(output)
    audit.require(not output.exists() and not output.is_symlink() and output.absolute() == output.resolve()
        and not any(p.is_symlink() for p in output.parents),"fresh canonical create-only0.5 media output required")
    first = {}; audit.add_pin(first,report_path,report_sha); report = audit.read_json(report_path); audit.frozen.assert_pins(first)
    audit.require(not output.is_relative_to(Path(report["batch"]).resolve()) and not output.is_relative_to(Path(report_path).parent.resolve())
        and not output.is_relative_to(audit.PREVIEW08_BATCH) and not output.is_relative_to(audit.PREVIEW08_REPORT.parent),"media cannot alter immutable evidence")
    with activated():
        audit.frozen.assert_pins(first); result = _publish(output,report_path,report_sha)
    # HH_261002 - Create an explicit local evidence index; never leave a README source link pointing to a missing file.
    provenance = output/"publication_provenance.json"
    index = dict(schema=SCHEMA+".source_index",independent_report_path=str(report_path),independent_report_sha256=report_sha,
        publication_provenance_path=str(provenance),publication_provenance_sha256=audit.digest(provenance),
        original0p3_report=report["all_preview_bracket_comparison"]["original0p3_report"],
        original0p8_report=report["all_preview_bracket_comparison"]["original0p8_report"],
        all_nine_actual_attempts_retained=True,source_pin_count=len(report["source_sha256"]),autoware_UI_captured=False)
    audit.frozen.assert_pins(first)
    with (output/"source_evidence.json").open("x") as stream: json.dump(index,stream,ensure_ascii=False,indent=2,allow_nan=False)
    result.update(configured_speed_preview_seconds=.5,full_comparison_speed_preview_seconds=[.3,.5,.8],
        all_nine_actual_attempts_in_comparison=True,source_evidence_index_sha256=audit.digest(output/"source_evidence.json"))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--report",type=Path,required=True); parser.add_argument("--report-sha256",required=True); parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--validate-only",action="store_true"); parser.add_argument("--render-all-media",action="store_true")
    args = parser.parse_args(argv); audit.require(args.validate_only != args.render_all_media,"choose validation or root-authorized quiet rendering")
    if args.validate_only:
        with activated(),previous.activated(),previous.previous.previous.activated(): report,bundles,_ = load_archive(args.report,args.report_sha256)
        result = dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False,output_created=False)
    else: result = publish(args.output,args.report,args.report_sha256)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
