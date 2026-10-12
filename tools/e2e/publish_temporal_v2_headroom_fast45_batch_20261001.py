"""HH_261001 - Create whole-duration actual six-camera failure media only after explicit CPU-quiet publication."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw

from . import audit_temporal_v2_headroom_fast45_batch_20261001 as audit
from . import publish_temporal_v2_actual_failure_20261001 as prior
from . import publish_town07_w1_pi_gallery_20261001 as visual


ROOT=audit.ROOT
OUTPUT=ROOT/"docs/assets/validation/2026-10-01/temporal_v2_headroom_fast45_actual_results_v1"
REPORT=audit.OUTPUT/"batch_outcome.json"
REPORT_SHA="a4cb4d3044edf7fbfc42f4d13b5eee707e49b7f6c75d0ac36266408297d83f36"
HELPER_SHA={"tools/e2e/audit_temporal_v2_headroom_fast45_batch_20261001.py":"c5778336d2d3306c57254da123f07e2d6c34fd859affc122bf1429ab3719b0ff",
            "tools/e2e/publish_temporal_v2_actual_failure_20261001.py":"667a646e7f2dc34e3b02b34ecda3993ac27409434b727ac7eacaa700bbb14abe",
            "tools/e2e/publish_town07_w1_pi_gallery_20261001.py":"ecf3b011e96e9942376b3a2e5502bff86e727a54e9c128726291a05b865fd78e"}


def indices(count=450):
    # HH_261001 - Ninety fixed half-second bins represent all45s; do not crop the latched brake tail.
    audit.require(type(count) is int and count==450,"complete450camera denominator required")
    return list(range(0,count,5))


def load_archive(*,decode_and_pin_JPEG=False):
    audit.require(audit.digest(REPORT)==REPORT_SHA,"independently audited all-three outcome changed")
    report=audit.read_json(REPORT)
    pins=dict(report["source_sha256"])
    pins[str(REPORT)]=REPORT_SHA
    pins[str(Path(__file__))]=audit.digest(__file__)
    for name,sha in HELPER_SHA.items():pins[str(ROOT/name)]=sha
    for name in ("DejaVuSans.ttf","DejaVuSans-Bold.ttf"):
        path=Path("/usr/share/fonts/truetype/dejavu")/name;pins[str(path)]=audit.digest(path)
    audit.assert_pins(pins)
    bundles=[]
    for trial in report["trials"]:
        episode=audit.BATCH/("trial_%02d"%trial["trial_index"])/"episode.partial"
        states,cameras,commands,receipts=[audit.rows(episode/name) for name in
            ("states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl")]
        geometry=audit.rows(episode.parent/"derived_body/all900_native_boxes_and899_sweeps.jsonl.gz")
        audit.require(len(states)==len(geometry)==900 and len(cameras)==450 and len(receipts)==904,"full actual capture missing")
        by_frame={s["frame"]:s for s in states}
        pictures=[]
        for camera in cameras:
            for name in visual.CAMERAS:
                path=prior.safe_image(episode,camera["images"][name])
                pictures.append(path)
                if decode_and_pin_JPEG:
                    pins[str(path)]=audit.digest(path)
                    with Image.open(path) as image:
                        audit.require(image.size==(640,360) and image.format=="JPEG","actual camera ABI differs")
                        image.load()
        audit.require(len(pictures)==len(set(pictures))==2700,"all2700 real six-camera assets required")
        for state in states:
            receipt,_,_,_=prior.observed_prior_ack(state,receipts,commands,by_frame)
            audit.require(receipt["sequence"]==state["control_transport"]["expected_receipt_sequence"],"displayed prior ACK differs from observation")
        bundles.append(dict(episode=episode,states=states,cameras=cameras,commands=commands,receipts=receipts,
                            by_frame=by_frame,geometry={g["frame"]:g for g in geometry},trial=trial,
                            route=audit.read_json(episode/"route.json")["route"]))
    audit.assert_pins(pins)
    return report,bundles,pins


def png_indices(bundle):
    cameras=bundle["cameras"]
    failure=bundle["trial"]["first_latched_failure"]
    first=next(i for i,c in enumerate(cameras) if c["frame"]>=failure["outgoing_command_frame"])
    applied=next(i for i,c in enumerate(cameras) if c["frame"]>=failure["first_observed_emergency_frame"])
    return {"01_startup.png":0,"02_last_model.png":max(0,first-1),"03_rejection_observation.png":first,
            "04_observed_emergency.png":applied,"05_full_capture_last_camera.png":449}


def render(bundle,camera_index):
    camera=bundle["cameras"][camera_index];state=bundle["by_frame"][camera["frame"]]
    receipt,command,plan,label=prior.observed_prior_ack(state,bundle["receipts"],bundle["commands"],bundle["by_frame"])
    canvas=Image.new("RGB",(1700,860),visual.BG);draw=ImageDraw.Draw(canvas)
    draw.text((24,18),f"Actual Town07 trial {bundle['trial']['trial_index']+1}/3 — DRIVING FAIL / BODY FAIL",fill=visual.WHITE,font=visual.font(28,True))
    draw.text((24,63),"Original final3000 / PI2.40 cap / single path / no teacher fallback after handoff",fill=visual.MUTED,font=visual.font(19))
    draw.text((24,101),f"Observed t={state['timestamp']-bundle['states'][0]['timestamp']:.2f}s | frame={state['frame']} | measured v0={math.hypot(state['vx'],state['vy']):.5f}m/s | rear-base CTE={state['route_cte_m']:.3f}m",fill=visual.WHITE,font=visual.font(20))
    for index,name in enumerate(visual.CAMERAS):
        x,y=24+index%3*416,150+index//3*277
        draw.text((x,y),name,fill=visual.WHITE,font=visual.font(18,True))
        with Image.open(prior.safe_image(bundle["episode"],camera["images"][name])) as source:
            canvas.paste(source.convert("RGB").resize((400,225),Image.Resampling.LANCZOS),(x,y+30))
    route=[(p["x"],p["y"]) for p in bundle["route"]]
    traveled=[(s["x"],s["y"]) for s in bundle["states"] if s["frame"]<=state["frame"]]
    panel=visual.map_panel(state,bundle["geometry"][state["frame"]],route,traveled,plan)
    pd=ImageDraw.Draw(panel);pd.rectangle((14,500,407,528),fill=visual.PANEL)
    pd.text((20,505),"Purple: prior applied learned plan" if plan else "Purple: NONE / current prior ACK",fill=visual.PURPLE if plan else visual.MUTED,font=visual.font(15))
    canvas.paste(panel,(1270,145))
    draw.text((24,731),f"Latest PRIOR accepted ACK #{receipt['sequence']}: {label}",fill="#ff9b9b" if command and command["control_source"]=="emergency_brake" else visual.WHITE,font=visual.font(19,True))
    ctrl=state["current_control"]
    draw.text((24,770),f"Actual observed throttle={ctrl['throttle']:.3f} brake={ctrl['brake']:.3f} steer={ctrl['steer']:.3f} | outgoing same-frame proposal NOT yet observed",fill=visual.WHITE,font=visual.font(18))
    draw.text((24,809),"All45s /90 fixed2Hz frames |900native /450six-camera /2700JPEG | latched emergency tail retained | NOT Autoware/RViz recording",fill=visual.MUTED,font=visual.font(17))
    return canvas


def publish(output=OUTPUT):
    output=Path(output)
    audit.require(not output.exists() and not output.is_symlink(),"fresh create-only publication required")
    report,bundles,pins=load_archive(decode_and_pin_JPEG=True)
    output.mkdir(parents=True,exist_ok=False)
    media=[]
    for bundle in bundles:
        folder=output/("trial_%02d"%bundle["trial"]["trial_index"]);folder.mkdir(exist_ok=False)
        frames=[render(bundle,i) for i in indices()]
        path=folder/"full45s_actual_six_cameras_2hz.gif"
        frames[0].save(path,save_all=True,append_images=frames[1:],duration=500,loop=0,optimize=False)
        with Image.open(path) as saved:audit.require(saved.n_frames==90,"all90 whole-duration GIF frames required")
        media.append(path)
        for name,index in png_indices(bundle).items():
            path=folder/name;render(bundle,index).save(path);media.append(path)
    audit.assert_pins(pins)
    # HH_261001 - This page contains its own original report, so all local evidence links remain valid.
    text=audit.markdown(report)+"\n## 전체 실제 화면\n\n"
    for bundle in bundles:
        name="trial_%02d"%bundle["trial"]["trial_index"]
        text+=f"### {bundle['trial']['trial_index']+1}회차 — 실패 후 정지까지 전체45초\n\n![실제6카메라와 차량 중심 지도]({name}/full45s_actual_six_cameras_2hz.gif)\n\n[첫 비상제동 ACK 관측]({name}/04_observed_emergency.png) · [마지막 실제 카메라]({name}/05_full_capture_last_camera.png)\n\n"
    text+="모든 지도는 측정 차량 위치를 중심으로 고정합니다. 보라색은 가장 최근 **이전 ACK가 실제 모델 제어**일 때만 표시하며, emergency ACK 뒤에는 지웁니다. 실제 CARLA 원 카메라+원 기록 진단이며 Autoware/RViz 화면 촬영이 아닙니다. GIF는0.5초 간격의90개 고정구간을 대표하고, 마지막 원 카메라449번도 별도 PNG에 남겼습니다.\n"
    for name,value in (("batch_outcome.json",report),("source_evidence.json",pins),
                       ("publication_provenance.json",dict(source_sha256=pins,whole_duration_frames_per_trial=90,
                         actual_JPEG_count=8100,failed_tail_cropped=False,prior_ACK_any_source=True,
                         emergency_clears_purple_plan=True,actual_media_sha256={str(p.relative_to(output)):audit.digest(p) for p in media},
                         vehicle_control_approved=False,model_promoted=False,training_data_approved=False))):
        with (output/name).open("x") as stream:json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream:stream.write(text)
    audit.assert_pins(pins)
    return dict(output=str(output),actual_all_six_camera_media=True,trial_count=3,GIF_frames_per_trial=90,original_raw_rewritten=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=OUTPUT)
    parser.add_argument("--validate-only",action="store_true")
    parser.add_argument("--render-all-media",action="store_true",help="Explicit quiet-window CPU publication, never implied by validate")
    args=parser.parse_args()
    audit.require(args.validate_only != args.render_all_media,"choose validate-only or explicit render-all-media")
    if args.validate_only:
        report,bundles,_=load_archive()
        print(json.dumps(dict(validated=True,trial_count=len(bundles),JPEG_pixels_loaded=False,rendered=False)))
    else:print(json.dumps(publish(args.output)))


if __name__=="__main__":main()
