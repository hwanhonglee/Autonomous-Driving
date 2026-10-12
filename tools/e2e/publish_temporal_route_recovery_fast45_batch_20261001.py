"""HH_261001 - Publish all three actual V3 attempts only after explicit quiet-window authorization."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw

from . import audit_temporal_route_recovery_fast45_batch_20261001 as audit
from . import publish_temporal_v2_actual_failure_20261001 as prior
from . import publish_town07_w1_pi_gallery_20261001 as visual

SCHEMA = "portable_e2e.actual_route_recovery_fast45_all_attempt_publication.20261001.v1"
HELPER_SHA = {
    "tools/e2e/publish_temporal_v2_actual_failure_20261001.py": "667a646e7f2dc34e3b02b34ecda3993ac27409434b727ac7eacaa700bbb14abe",
    "tools/e2e/publish_town07_w1_pi_gallery_20261001.py": "ecf3b011e96e9942376b3a2e5502bff86e727a54e9c128726291a05b865fd78e",
}


def indices(count=450):
    # HH_261001 - Retain ninety fixed half-second bins through all45s, not only accepted learned moments.
    audit.require(type(count) is int and count==450, "complete450camera denominator required")
    return list(range(0,count,5))


def png_indices(bundle):
    cameras,trial=bundle["cameras"],bundle["trial"]
    audit.require(len(cameras)==450, "complete450camera PNG index domain required")
    failure=trial.get("first_latched_failure")
    if failure is None:
        # HH_261001 - Fixed first/quarter/half/three-quarter/last coverage also applies to non-latched failures.
        return {"01_start.png":0,"02_quarter.png":112,"03_half.png":224,"04_three_quarters.png":337,"05_last.png":449}
    rejection=next((i for i,c in enumerate(cameras) if c["frame"]>=failure["outgoing_command_frame"]),None)
    effect=failure.get("first_observed_emergency_frame")
    applied=next((i for i,c in enumerate(cameras) if effect is not None and c["frame"]>=effect),None)
    if rejection is None:
        # HH_261001 - The final odd native frame may have no saved camera at/after its rejection; show only the original last camera.
        return {"01_start.png":0,"02_last_camera_before_terminal_rejection.png":449,
            "03_no_camera_at_terminal_rejection.png":449,"04_terminal_ACK_no_later_camera.png":449,"05_last.png":449}
    result={"01_start.png":0,"02_last_before_rejection.png":max(0,rejection-1),"03_rejection_observation.png":rejection,
            "05_last.png":449}
    # HH_261001 - A terminal ACK without an observed later camera is not invented or labelled applied.
    result["04_observed_emergency.png" if applied is not None else "04_terminal_ACK_no_later_camera.png"] = applied if applied is not None else 449
    return result


def load_archive(report_path,report_sha,*,decode_and_pin_JPEG=False):
    pins={}; audit.add_pin(pins,report_path,report_sha)
    report=audit.read_json(report_path)
    audit.require(report.get("schema")==audit.SCHEMA and report.get("prospective_trials")==3
        and len(report.get("trials",[]))==3 and [t["trial_index"] for t in report["trials"]]==[0,1,2]
        and report.get("all_failed_tail_retained") is True and report.get("vehicle_control_approved") is False
        and report.get("model_promoted") is False and report.get("startup_recovery_fitted") is False,
        "externally pinned independent all3 V3 audit required")
    for name,sha in report["source_sha256"].items(): audit.add_pin(pins,name,sha)
    for name,sha in HELPER_SHA.items(): audit.add_pin(pins,audit.ROOT/name,sha)
    for path in audit.source_closure(Path(__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py"):
        audit.add_pin(pins,path)
    for name in ("DejaVuSans.ttf","DejaVuSans-Bold.ttf"): audit.add_pin(pins,Path("/usr/share/fonts/truetype/dejavu")/name)
    batch=Path(report["batch"])
    audit.require(batch.absolute()==batch.resolve() and batch.is_relative_to(audit.ROOT/"storage/experiments/validation"), "canonical actual batch required")
    bundles=[]
    for trial in report["trials"]:
        folder=batch/("trial_%02d"%trial["trial_index"])
        episode=folder/"episode.partial"
        if not episode.is_dir(): episode=folder/"episode"
        names=("states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl")
        audit.require(all(str(episode/name) in pins for name in names), "all original display inputs must be in pinned audit")
        states,cameras,commands,receipts=[audit.rows(episode/name) for name in names]
        geometry_path=folder/"derived_body/all900_native_boxes_and899_sweeps.jsonl.gz"
        audit.require(str(geometry_path) in pins, "display geometry lacks independently pinned whole-interval audit")
        geometry=audit.rows(geometry_path)
        audit.require(len(states)==len(geometry)==900 and len(cameras)==450 and len(receipts)==904, "all raw native/camera/ACK/body rows required")
        by_frame={s["frame"]:s for s in states}; pictures=[]
        for camera in cameras:
            audit.require(set(camera["images"])==set(visual.CAMERAS), "six real cameras required")
            for name in visual.CAMERAS:
                path=prior.safe_image(episode,camera["images"][name]); pictures.append(path)
                audit.require(str(path) in pins and audit.digest(path)==pins[str(path)], "actual JPEG not in independent all2700 byte pins")
                if decode_and_pin_JPEG:
                    with Image.open(path) as picture:
                        audit.require(picture.format=="JPEG" and picture.size==(640,360), "real JPEG ABI differs")
                        picture.load()
        audit.require(len(pictures)==len(set(pictures))==2700, "all2700 distinct real JPEGs required")
        for state in states:
            receipt,_,_,_=prior.observed_prior_ack(state,receipts,commands,by_frame)
            audit.require(receipt["sequence"]==state["control_transport"]["expected_receipt_sequence"], "display source differs from latest prior ACK")
        bundles.append(dict(episode=episode,states=states,cameras=cameras,commands=commands,receipts=receipts,by_frame=by_frame,
            geometry={g["frame"]:g for g in geometry},trial=trial,route=audit.read_json(episode/"route.json")["route"]))
    audit.frozen.assert_pins(pins)
    return report,bundles,pins


def displayed_receipt(bundle,index):
    camera=bundle["cameras"][index]; state=bundle["by_frame"][camera["frame"]]
    receipt,command,plan,label=prior.observed_prior_ack(state,bundle["receipts"],bundle["commands"],bundle["by_frame"])
    return dict(camera_index=index,observed_frame=state["frame"],simulation_timestamp_s=state["timestamp"],
        prior_ACK_sequence=receipt["sequence"],prior_ACK_before_frame=receipt["before_frame"],observed_source=label,
        prior_learned_plan_drawn=bool(plan),current_observed_control=state["current_control"])


def render(bundle,index):
    camera=bundle["cameras"][index]; state=bundle["by_frame"][camera["frame"]]
    receipt,command,plan,label=prior.observed_prior_ack(state,bundle["receipts"],bundle["commands"],bundle["by_frame"])
    trial=bundle["trial"]
    driving="PASS" if trial["original_driving_screen_passed"] else "FAIL"
    wall="PASS" if trial["fresh_observed_delivery_to_ACK100ms_passed"] else "FAIL"
    canvas=Image.new("RGB",(1700,860),visual.BG); draw=ImageDraw.Draw(canvas)
    draw.text((24,18),f"Actual Town07 V3 trial {trial['trial_index']+1}/3 | BASIC DRIVING {driving} | WALL100ms {wall}",fill=visual.WHITE,font=visual.font(26,True))
    draw.text((24,63),"Fixed final3000 learned recovery / frozen97+learned4 / PI2.40 / no route-output substitution",fill=visual.MUTED,font=visual.font(18))
    draw.text((24,101),f"Observed t={state['timestamp']-bundle['states'][0]['timestamp']:.2f}s | frame={state['frame']} | base v0={math.hypot(state['vx'],state['vy']):.5f}m/s | rear-base CTE={state['route_cte_m']:.3f}m",fill=visual.WHITE,font=visual.font(19))
    for camera_index,name in enumerate(visual.CAMERAS):
        x,y=24+camera_index%3*416,150+camera_index//3*277
        draw.text((x,y),name,fill=visual.WHITE,font=visual.font(18,True))
        with Image.open(prior.safe_image(bundle["episode"],camera["images"][name])) as source:
            resampling=getattr(Image,"Resampling",Image)
            canvas.paste(source.convert("RGB").resize((400,225),resampling.LANCZOS),(x,y+30))
    route=[(p["x"],p["y"]) for p in bundle["route"]]
    traveled=[(s["x"],s["y"]) for s in bundle["states"] if s["frame"]<=state["frame"]]
    panel=visual.map_panel(state,bundle["geometry"][state["frame"]],route,traveled,plan)
    pd=ImageDraw.Draw(panel);pd.rectangle((14,500,407,528),fill=visual.PANEL)
    pd.text((20,505),"Purple: prior applied learned plan" if plan else "Purple: NONE / latest prior ACK",fill=visual.PURPLE if plan else visual.MUTED,font=visual.font(15))
    canvas.paste(panel,(1270,145))
    draw.text((24,731),f"Latest PRIOR accepted ACK #{receipt['sequence']}: {label}",fill="#ff9b9b" if command and command["control_source"]=="emergency_brake" else visual.WHITE,font=visual.font(19,True))
    ctrl=state["current_control"]
    draw.text((24,770),f"Observed throttle={ctrl['throttle']:.3f} brake={ctrl['brake']:.3f} steer={ctrl['steer']:.3f} | outgoing same-frame proposal NOT yet observed",fill=visual.WHITE,font=visual.font(18))
    draw.text((24,809),"Full45s /90 fixed2Hz frames |900native /450six-camera /2700JPEG | ALL failure tails | NOT Autoware/RViz UI",fill=visual.MUTED,font=visual.font(17))
    return canvas


def publish(output,report_path,report_sha):
    output=Path(output); audit.require(not output.exists() and not output.is_symlink(), "fresh create-only publication output required")
    report,bundles,pins=load_archive(report_path,report_sha,decode_and_pin_JPEG=True)
    output.mkdir(parents=True,exist_ok=False); media=[]; displayed=[]
    for bundle in bundles:
        folder=output/("trial_%02d"%bundle["trial"]["trial_index"]);folder.mkdir(exist_ok=False)
        fixed=indices(); frames=[render(bundle,i) for i in fixed]
        path=folder/"full45s_actual_six_cameras_2hz.gif"
        with path.open("xb") as stream: frames[0].save(stream,format="GIF",save_all=True,append_images=frames[1:],duration=500,loop=0,optimize=False)
        with Image.open(path) as saved:
            audit.require(saved.n_frames==90, "all90 full-duration GIF frames required")
            durations=[]
            for i in range(saved.n_frames): saved.seek(i);durations.append(saved.info["duration"])
            audit.require(durations==[500]*90 and sum(durations)==45_000, "full45s playback cannot be shortened")
        media.append(path); pngs=png_indices(bundle)
        for name,index in pngs.items():
            path=folder/name
            with path.open("xb") as stream: render(bundle,index).save(stream,format="PNG")
            media.append(path)
        displayed.append(dict(trial_index=bundle["trial"]["trial_index"],GIF_frames=[displayed_receipt(bundle,i) for i in fixed],
            supplemental_PNG_camera_indices=pngs,supplemental_PNG_frames={name:displayed_receipt(bundle,i) for name,i in pngs.items()},
            rejection_native_frame=bundle["trial"]["first_latched_failure"]["outgoing_command_frame"] if bundle["trial"].get("first_latched_failure") else None,
            PNG_rejection_semantics="first original camera at/after rejection; if no such camera exists the last earlier original camera is labelled explicitly; never a synthesized rejection exposure"))
    audit.frozen.assert_pins(pins)
    text=audit.markdown(report)+"\n## 실제 전체 화면\n\n"
    for bundle in bundles:
        name="trial_%02d"%bundle["trial"]["trial_index"]
        text+=f"### {bundle['trial']['trial_index']+1}회차 — 전체 45초\n\n![실제 6카메라와 차량 중심 지도]({name}/full45s_actual_six_cameras_2hz.gif)\n\n[실제 마지막 카메라]({name}/05_last.png)\n\n"
    text+="모든 지도는 측정 차량 위치를 중심으로 고정합니다. 보라색은 가장 최근 이전 ACK가 실제 모델 제어일 때만 표시하며 emergency ACK 뒤에는 지웁니다. 실제 CARLA 원 카메라와 기록 기반 진단이며 Autoware/RViz 화면 촬영이 아닙니다. GIF는 고정 0.5초 간격 90개 구간을 대표하고 마지막 원 카메라 449번도 별도 PNG에 남깁니다.\n"
    provenance=dict(schema=SCHEMA,source_sha256=pins,externally_pinned_independent_report_sha256=report_sha,
        whole_duration_frames_per_trial=90,playback_hz=2,playback_seconds_per_trial=45,actual_JPEG_count=8100,
        displayed_frames=displayed,failed_tail_cropped=False,prior_ACK_any_source=True,emergency_clears_purple_plan=True,
        measured_vehicle_center_fixed_in_all_scenes=True,autoware_UI_captured=False,synthetic_driving_image_generated=False,
        actual_media_sha256={str(p.relative_to(output)):audit.digest(p) for p in media},vehicle_control_approved=False,
        model_promoted=False,training_data_approved=False,full_RTE01_approved=False,startup_recovery_fitted=False)
    for name,value in (("batch_outcome.json",report),("source_evidence.json",pins),("publication_provenance.json",provenance)):
        with (output/name).open("x") as stream: json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream: stream.write(text)
    audit.frozen.assert_pins(pins)
    return dict(output=str(output),trial_count=3,GIF_frames_per_trial=90,actual_all_six_camera_media=True,original_raw_rewritten=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--report",type=Path,required=True);parser.add_argument("--report-sha256",required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--validate-only",action="store_true")
    parser.add_argument("--render-all-media",action="store_true",help="Root-authorized CPU-quiet publication; no rendering implied by validation")
    args=parser.parse_args(argv)
    audit.require(args.validate_only != args.render_all_media, "choose validate-only or explicit quiet-window render-all-media")
    if args.validate_only:
        report,bundles,_=load_archive(args.report,args.report_sha256)
        result=dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False,output_created=False)
    else: result=publish(args.output,args.report,args.report_sha256)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__=="__main__": raise SystemExit(main())
