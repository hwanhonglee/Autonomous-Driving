"""HH_261004 - Publish only authentic full209 known-DEV records under pre-capture selection rules, without inference or promotion."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

from PIL import Image, ImageDraw

from . import publish_temporal_route_joint_town03_right_20261002 as previous


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT/"storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1"
OUTPUT = ROOT/"docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/11_town03_full_route209"
POLICY_PATH = ROOT/"storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v1/ROOT_prospective_publication_policy.json"
SCHEMA = "portable_e2e.actual_town03_known_dev_straight_full209_publication.20261004.v1"
CALLER_SCHEMA = SCHEMA+".caller_pins"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_full209.20261004.v1"
POLICY_SCHEMA = "portable_e2e.town03_known_dev_straight_full209_prospective_publication_policy.20261004.v1"
ROUTE_SHA = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
PREVIOUS_SHA = "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8"
MODEL_CAMERA_ORDER = previous.MODEL_CAMERA_ORDER
require, prior, visual = previous.require, previous.prior, previous.visual
PNG_NUMERATORS = {"01_start.png":0,"02_quarter.png":1,"03_half.png":2,"04_three_quarters.png":3,"05_last.png":4}


def publication_sources():
    """HH_261004 - Pin this post-capture renderer closure independently; only the selection policy had to precede capture."""
    require(prior.digest(Path(previous.__file__)) == PREVIOUS_SHA,"reviewed pure full-denominator renderer changed")
    return previous.core.audit.source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py")


def camera_indices(count):
    require(type(count) is int and 1 <= count <= 1100,"actual bounded full209 camera count required")
    return sorted(set(range(0,count,5))|{count-1})


def png_indices(count):
    require(type(count) is int and 1 <= count <= 1100,"actual bounded PNG camera count required")
    return {name:(count-1)*numerator//4 for name,numerator in PNG_NUMERATORS.items()}


def playback(bundle,indices):
    """HH_261004 - Display actual timestamp bins plus the predeclared nominal100ms final-camera hold; do not create an observation."""
    cameras = bundle["cameras"]
    require(indices == camera_indices(len(cameras)),"only prospectively fixed full-interval camera bins permitted")
    stamps = [row["timestamp"] for row in cameras]
    require(all(type(value) in (int,float) and math.isfinite(value) for value in stamps)
        and all(abs(b-a-.1) <= 1.e-6 for a,b in zip(stamps,stamps[1:])),"actual original10Hz camera timestamps required")
    selected = [stamps[index] for index in indices]+[stamps[-1]+.1]
    duration = [round((b-a)*100)*10 for a,b in zip(selected,selected[1:])]
    require(all(type(value) is int and 0 < value <= 500 for value in duration)
        and sum(duration) == round((stamps[-1]-stamps[0]+.1)*100)*10,
        "full timestamp interval and nominal final hold cannot be shortened")
    return duration


def verify_policy(caller,pins,plan):
    reference = caller.get("prospective_publication_policy",{})
    require(set(reference) == {"path","sha256"} and previous.canonical(reference["path"]) == POLICY_PATH,
        "exact pre-episode selection policy reference required")
    pins.file(POLICY_PATH,reference["sha256"])
    policy = pins.json(POLICY_PATH)
    require(policy.get("schema") == POLICY_SCHEMA and policy.get("created_before_actual_episode") is True
        and policy.get("actual_episode_observed") is False and policy.get("run_root") == str(RUN)
        and policy.get("output") == str(OUTPUT) and policy.get("attempts") == 1
        and policy.get("automatic_retry") is False and policy.get("media_selection") == plan.get("prospective_visual_publication"),
        "sealed prospective run/output/media selection differs")
    declaration = policy.get("run_declaration",{})
    plan_path = RUN.with_name(RUN.name+"_plan.json")
    require(previous.canonical(declaration.get("path","")) == plan_path
        and declaration.get("sha256") == caller["run_files_sha256"]["declaration.json"],
        "pre-capture owner declaration differs from executed archive")
    pins.file(plan_path,declaration["sha256"])
    require(pins.json(plan_path) == plan,"executed declaration and prospective original differ")
    selection = policy["media_selection"]
    require(selection.get("gallery") == str(OUTPUT.relative_to(ROOT))
        and selection.get("GIF_camera_indices") == "every5th actual camera bundle starting0 plus exact last; unique sorted"
        and selection.get("GIF_playback") == "all selected timestamp intervals plus original final0.1s camera cadence"
        and selection.get("PNG_camera_indices") == "floor((actual_camera_count-1)*num/4), num=0,1,2,3,4; unique sorted"
        and selection.get("complete_actual_interval_including_failures_required") is True
        and selection.get("outcome_or_failure_dependent_frame_selection") is False
        and selection.get("actual_Autoware_or_RViz_screenshot_claim") is False
        and selection.get("illustrative_or_generated_image_allowed") is False,
        "unaltered whole-interval media rules required")
    require(policy.get("known_DEV_site_reused") is True and policy.get("independent_validation") is False,
        "known-DEV policy disclosure required")
    for key in ("full_RTE01_approved","full_MAIN9_completion_claimed","training_data_approved","vehicle_control_approved","model_promoted"):
        require(policy.get(key) is False,"prospective approval denial changed: "+key)
    return policy


def load_archive(caller_path,caller_sha,*,decode=False):
    """HH_261004 - Verify whole raw/source archives before selecting any scene, including failures and actual final prior-NN effects."""
    require("torch" not in sys.modules and "carla" not in sys.modules,"pure recorded publication cannot perform inference or simulator calls")
    pins = previous.Pins()
    pins.file(caller_path,caller_sha)
    caller = pins.json(caller_path)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("attempts") == 1
        and previous.canonical(caller["run_root"]) == RUN and previous.canonical(caller["output"]) == OUTPUT,
        "only the fresh exact full209 run/output may be published")
    require(set(caller["run_files_sha256"]) == previous.inventory(RUN),"every original archive file must be externally pinned")
    for name,sha in caller["run_files_sha256"].items():
        path = Path(name)
        require(not path.is_absolute() and ".." not in path.parts and path.as_posix() == name,"safe relative original archive member required")
        pins.file(RUN/path,sha)
    sources = publication_sources()
    require(set(caller["publication_source_sha256"]) == {str(path.relative_to(ROOT)) for path in sources},
        "exact post-capture pure renderer source closure required")
    for path in sources:
        pins.file(path,caller["publication_source_sha256"][str(path.relative_to(ROOT))])
    for name in ("DejaVuSans.ttf","DejaVuSans-Bold.ttf"):
        path = Path("/usr/share/fonts/truetype/dejavu")/name
        pins.file(path,caller["font_sha256"][str(path)])
    plan,owner,diagnostic = [pins.json(RUN/name) for name in ("declaration.json","owner_result.json","diagnostic_report.json")]
    require(plan.get("schema") == OWNER_SCHEMA and plan.get("output") == str(RUN) and plan.get("run_id") == "v1"
        and plan.get("town") == "Town03" and plan.get("scenario") == "straight" and plan.get("dataset_split") == "val"
        and plan.get("known_DEV_site_reused") is True and plan.get("independent_validation") is False
        and plan.get("attempts") == 1 and plan.get("automatic_retry") is False and plan.get("total_capture_seconds") == 110
        and plan.get("maximum_native_state_count") == 2200 and plan.get("maximum_camera_bundle_count") == 1100
        and plan.get("original_navigation_sha256") == ROUTE_SHA and plan.get("original_catalog_goal_tolerance_m") == 2.5
        and plan.get("navigation_terminal_controller_intervention_declared") is True
        and plan.get("learned_STOP_claim") is False,"exact disclosed full209 known-DEV contract required")
    require(owner.get("declaration_sha256") == caller["run_files_sha256"]["declaration.json"]
        and diagnostic.get("schema") == OWNER_SCHEMA+".diagnostic_report"
        and diagnostic.get("original_owner") == owner,"actual owner/declaration/diagnostic identities differ")
    for key in ("full_RTE01_approved","full_MAIN9_completion_claimed","training_data_approved","vehicle_control_approved","model_promoted","learned_STOP_claim"):
        require(diagnostic.get(key) is False,"original research denial changed: "+key)
    policy = verify_policy(caller,pins,plan)
    for table,root in ((plan["source_sha256"],ROOT),(plan["external_source_sha256"],None),(plan["asset_sha256"],None)):
        for name,sha in table.items():
            source = previous.canonical(root/name if root is not None else name)
            origin = ROOT if source.is_relative_to(ROOT) else previous.CARLA_ROOT
            require(source.is_relative_to(origin),"declared source escaped the owned archive roots")
            archive = RUN/("source" if origin == ROOT else "source_external")/source.relative_to(origin)
            pins.file(source,sha); pins.file(archive,sha)
    episode = RUN/("episode" if (RUN/"episode").is_dir() else "episode.partial")
    raw = {str(episode/name) for name in previous.inventory(episode)}
    require(set(diagnostic["raw_source_sha256"]) == raw,"all actual raw files and failed tails must remain in diagnostic")
    require(all(pins.values.get(name) == sha for name,sha in diagnostic["raw_source_sha256"].items()),
        "original raw diagnostic hashes differ from externally pinned archive")
    manifest,route,summary = [pins.json(episode/name) for name in ("manifest.json","route.json","learned_summary.json")]
    require(pins.values[str(episode/"route.json")] == ROUTE_SHA and route.get("town") == "Town03"
        and route.get("scenario") == "straight" and len(route.get("route",[])) == 211
        and manifest.get("status") == diagnostic.get("original_capture_status"),"actual complete original211-point catalog required")
    states,cameras,receipts = [pins.rows(episode/name) for name in ("states.jsonl","camera_frames.jsonl","control_receipts.jsonl")]
    unlogged = not (episode/"learned_control.jsonl").exists()
    commands = [] if unlogged else pins.rows(episode/"learned_control.jsonl")
    if unlogged:
        require(summary.get("handoff_occurred") is False and all(summary["counts"].get(key) == 0 for key in
            ("inference_count","prefill_count","learned_commands_acknowledged","emergency_commands")),
            "missing command journal cannot conceal actual model or emergency control")
    body = pins.json(RUN/"derived_body/summary.json")
    geometry = pins.rows(RUN/"derived_body/all_native_boxes_and_sweeps.jsonl.gz")
    require(body == diagnostic.get("independent_body") and body.get("all_native_boxes") == len(states)
        and body.get("all_snapshot_linear_sweeps") == len(states)-1 and body.get("lane_inset_m") == .1,
        "every actual native body/sweep row and original10cm inset required")
    by_frame = previous.verify_records(states,cameras,commands,receipts,geometry,unlogged_preNN=unlogged)
    require(diagnostic.get("actual_native_state_count") == len(states)
        and diagnostic.get("actual_camera_bundle_count") == len(cameras)
        and diagnostic.get("actual_ACK_receipt_count") == len(receipts),"original full actual denominator differs")
    images = []
    for camera in cameras:
        for name in visual.CAMERAS:
            path = previous.canonical(prior.safe_image(episode,camera["images"][name]))
            require(str(path) in pins.values,"real recorded image requires original external SHA")
            if decode:
                with Image.open(path) as image:
                    require(image.format == "JPEG" and image.size == (640,360),"real original640x360 JPEG ABI required")
                    image.load()
            images.append(path)
    require(len(images) == len(set(images)) == 6*len(cameras),"complete distinct six-camera image denominator required")
    trial = dict(trial_index=0,original_driving_screen_passed=diagnostic["learned_driving_screen_passed"],
        fresh_observed_delivery_to_ACK100ms_passed=diagnostic.get("observed_bundle_to_ACK_wall100ms_passed",False))
    bundle = dict(episode=episode,states=states,cameras=cameras,commands=commands,receipts=receipts,by_frame=by_frame,
        geometry={row["frame"]:row for row in geometry},route=route["route"],trial=trial,diagnostic=diagnostic,
        output=OUTPUT,unlogged_preNN=unlogged,original_learned_summary=summary,prospective_policy=policy)
    bundle["control_census"] = previous.control_census(bundle)
    require(bundle["control_census"]["observed_prior_learned_native_effect_count"] > 0
        or diagnostic.get("learned_driving_screen_passed") is False,"zero model effect cannot be published as driving PASS")
    pins.recheck()
    require(previous.inventory(RUN) == set(caller["run_files_sha256"]),"archive membership changed during read")
    return caller,bundle,pins


def render(bundle,index):
    """HH_261004 - Reuse only real six-camera and vehicle-centered route/travel/prior-ACK pixels, replacing stale turn captions."""
    out = previous.renderer_call(previous.core.render,bundle,index)
    draw = ImageDraw.Draw(out)
    status = bundle["diagnostic"]["status"]
    draw.rectangle((0,0,1699,91),fill=visual.BG)
    draw.text((24,18),"Actual Town03 known-DEV full209 | "+status,fill=visual.WHITE,font=visual.font(23,True))
    draw.text((24,63),"Final6000 / navigation-terminal V2 intervention / research only / unchanged full original catalog",fill=visual.MUTED,font=visual.font(17))
    draw.rectangle((0,801,1699,859),fill=visual.BG)
    draw.text((24,807),f"ALL {len(bundle['states'])}native / {len(bundle['cameras'])}six-camera | MAIN0/9 SUB0/30 | arrival is NOT learned STOP",fill=visual.WHITE,font=visual.font(16))
    draw.text((24,835),"Recorded CARLA diagnostic, NOT Autoware/RViz UI / complete failed tails preserved",fill=visual.MUTED,font=visual.font(16))
    return out


def markdown(bundle,pngs):
    states,cameras,diag = bundle["states"],bundle["cameras"],bundle["diagnostic"]
    text = "# 실제 Town03 209m 전체 경로 시험\n\n<!-- HH_261004 - Preserve actual whole-interval evidence and distinguish catalog cleanup from learned stopping. -->\n\n"
    text += f"원본 판정: **{diag['status']}**. 알려진 DEV 경로의 연구 시험이며 정식 완료는 메인0/9·하위0/30입니다. 새 terminal decoder와 전체 폴더 용량 계산 개입이 포함된 비교로, 모델만의 성능 향상이라고 주장하지 않습니다.\n\n"
    text += f"실제 상태 **{len(states)}개**, 실제6카메라 묶음 **{len(cameras)}개**와 실패 구간을 모두 보존했습니다. 첫→마지막 실제 상태 간격은 **{states[-1]['timestamp']-states[0]['timestamp']:.2f}초**이며, 상태 개수×50ms 표시 구간은 {len(states)*.05:.2f}초입니다.\n\n"
    text += f"GIF는 실제 카메라 타임스탬프 간격과 마지막 카메라의 사전 선언100ms 표시 유지로 총 {sum(playback(bundle,camera_indices(len(cameras))))/1000:.2f}초입니다. 이 마지막 표시 유지는 새 카메라·차량 상태·제어 관측이 아닙니다. 마지막 실제 카메라는 프레임 {cameras[-1]['frame']}, 마지막 실제 차량 상태는 프레임 {states[-1]['frame']}이며 둘을 합성하지 않았습니다.\n\n"
    text += "GIF는 첫 카메라부터 매5번째와 정확한 마지막을 선택하며, PNG는 전체 실제 카메라 수의0/¼/½/¾/마지막입니다. 결과나 실패 위치에 따라 좋은 장면만 고르지 않았습니다. 원래2.5m 목표 도착 뒤 정리 제동 ACK는 학습한 정차·정차 유지·실차 안전 증거가 아닙니다.\n\n"
    errors = {key:value for key,value in diag.items() if key.endswith("error") and value}
    if errors:
        text += "보존한 원본 오류:\n\n```json\n"+json.dumps(errors,ensure_ascii=False,indent=2)+"\n```\n\n"
    text += "![전체 실제6카메라 기록](full_actual_six_cameras_2hz.gif)\n\n"
    for name in pngs:
        text += f"![{name}]({name})\n\n"
    text += "차량 중심 진단 지도에서 파랑은 원래 경로, 초록은 그 시점까지의 실제 이동, 보라는 현재 상태에 앞서 ACK된 학습 모델 계획입니다. 교사 출발·긴급제동을 모델 주행으로 세지 않으며 긴급제동 이후 모델 계획을 지웁니다. 실제 CARLA 입력을 이용한 기록 기반 진단으로, Autoware/RViz 화면 캡처가 아닙니다. 전체 원본 판정·분모·원본SHA·선택 규칙은 [publication_provenance.json](publication_provenance.json)에 있습니다.\n"
    return text


def publish(args):
    caller,bundle,pins = load_archive(previous.canonical(args.caller_pins),args.caller_pins_sha256,decode=not args.validate_only)
    require(not OUTPUT.exists(),"full209 gallery revision is create-only")
    indices = camera_indices(len(bundle["cameras"]))
    durations = playback(bundle,indices)
    pngs = png_indices(len(bundle["cameras"]))
    if args.validate_only:
        return dict(validated=True,native_count=len(bundle["states"]),camera_count=len(bundle["cameras"]),GIF_frames=len(indices),rendered=False,output_created=False)
    ancestor = next(path for path in OUTPUT.parents if path.exists())
    require(shutil.disk_usage(ancestor).free >= 5*1024**3+256*1024**2,"5GiB plus256MiB publication reserve required")
    pins.recheck(); OUTPUT.mkdir(parents=True,exist_ok=False)
    frames = [render(bundle,index) for index in indices]
    gif = OUTPUT/"full_actual_six_cameras_2hz.gif"
    with gif.open("xb") as stream:
        frames[0].save(stream,format="GIF",save_all=True,append_images=frames[1:],duration=durations,loop=0,optimize=False)
    with Image.open(gif) as saved:
        require(saved.n_frames == len(indices),"every prospectively selected actual frame required")
        actual = []
        for index in range(saved.n_frames):
            saved.seek(index); actual.append(saved.info["duration"])
        require(actual == durations,"whole original timestamp playback bins changed")
    media = [gif]
    for name,index in pngs.items():
        path = OUTPUT/name
        with path.open("xb") as stream:
            render(bundle,index).save(stream,format="PNG")
        media.append(path)
    states,cameras = bundle["states"],bundle["cameras"]
    provenance = dict(schema=SCHEMA,caller_sha256=args.caller_pins_sha256,source_and_input_sha256=dict(pins.values),
        original_diagnostic=bundle["diagnostic"],original_learned_summary=bundle["original_learned_summary"],
        prospective_policy=bundle["prospective_policy"],GIF_camera_indices=indices,GIF_duration_ms=durations,
        GIF_total_duration_ms=sum(durations),PNG_camera_indices=pngs,control_census=bundle["control_census"],
        native_snapshot_span_s=states[-1]["timestamp"]-states[0]["timestamp"],native_count_times_50ms_s=len(states)*.05,
        final_nominal_camera_display_hold_s=.1,final_display_hold_is_new_observation=False,
        last_native_frame=states[-1]["frame"],last_native_timestamp=states[-1]["timestamp"],
        last_camera_frame=cameras[-1]["frame"],last_camera_timestamp=cameras[-1]["timestamp"],
        native_terminal_frame_without_camera_not_fabricated=True,failed_tail_cropped=False,
        displayed_GIF_frames=[previous.renderer_call(previous.core.displayed_receipt,bundle,index) for index in indices],
        displayed_PNG_frames={name:previous.renderer_call(previous.core.displayed_receipt,bundle,index) for name,index in pngs.items()},
        media_selection_sealed_before_actual_episode=True,publisher_sources_sealed_before_capture_claimed=False,
        original_raw_rewritten=False,known_DEV_site_reused=True,independent_validation=False,
        official_completed_main_domains=0,official_completed_subfunctions=0,learned_STOP_claim=False,
        full_RTE01_approved=False,full_MAIN9_completion_claimed=False,training_data_approved=False,
        vehicle_control_approved=False,model_promoted=False,model_inference_executed=False,simulator_started=False,
        autoware_UI_captured=False,synthetic_driving_image_generated=False,
        media_sha256={path.name:prior.digest(path) for path in media})
    pins.recheck(); require(previous.inventory(RUN) == set(caller["run_files_sha256"]),"whole raw/source archive changed during publication")
    with (OUTPUT/"publication_provenance.json").open("x") as stream:
        json.dump(provenance,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (OUTPUT/"README.md").open("x") as stream:
        stream.write(markdown(bundle,pngs))
    pins.recheck()
    return dict(output=str(OUTPUT),GIF_count=1,PNG_count=5,GIF_frames=len(indices),outcome=bundle["diagnostic"]["status"],original_raw_rewritten=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--caller-pins",required=True); parser.add_argument("--caller-pins-sha256",required=True)
    parser.add_argument("--validate-only",action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)),ensure_ascii=False,allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
