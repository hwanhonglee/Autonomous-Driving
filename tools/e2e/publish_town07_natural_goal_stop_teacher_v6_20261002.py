"""HH_261002 - Publish only recorded V6 teacher pixels/state; never learned plans or an Autoware UI."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw

from . import publish_town07_w1_pi_gallery_20261001 as visual

ROOT = Path(__file__).resolve().parents[2]
BATCH = ROOT / "storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v6"
CASE = BATCH / "pilot_00"
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/08_actual_goal_stop_teacher_v6"
SCHEMA = "carla.actual_goal_stop_teacher_v6_publication.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
TASK_SCHEMA = "carla.actual_natural_goal_stop_teacher.20261002.v6"
FUTURE_QA_SCHEMA = "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v6"
HELPER_SHA = "ecf3b011e96e9942376b3a2e5502bff86e727a54e9c128726291a05b865fd78e"
FONT_ROOT = Path("/usr/share/fonts/truetype/dejavu")
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EPISODE_ROLES = {
    "manifest": "manifest.json", "states": "states.jsonl", "cameras": "camera_frames.jsonl",
    "receipts": "control_receipts.jsonl", "route": "route.json",
    "teacher": "actual_teacher_control.jsonl", "governor": "actual_goal_governor.jsonl",
    "assignments": "actual_goal_target_assignments.jsonl", "hazards": "actual_hazard_overrides.jsonl",
    "pixels": "actual_camera_pixels.jsonl",
}
OTHER_ROLES = {
    "batch_plan": BATCH.with_name(BATCH.name + "_plan.json"), "case_plan": BATCH / "pilot_00_plan.json",
    "owner": CASE / "owner_result.json", "diagnostic": CASE / "diagnostic_report.json",
    "body_summary": CASE / "derived_body/summary.json",
    "body": CASE / "derived_body/all_native_bodies_and_sweeps.jsonl.gz",
}
require = visual.require


def valid_sha(value):
    require(type(value) is str and len(value) == 64 and all(ch in "0123456789abcdef" for ch in value),
            "exact lowercase externally supplied SHA256 required")
    return value


def canonical(path):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and not any(p.is_symlink() for p in (path, *path.parents))
            and path.resolve() == path, "canonical unescaped nonsymlink path required")
    return path


def digest(path):
    path = canonical(path)
    require(path.is_file(), "regular original file required: " + str(path))
    state = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            state.update(block)
    return state.hexdigest()


def add_pin(pins, path, expected=None):
    path = canonical(path)
    actual = digest(path)
    require(expected is None or actual == expected, "externally pinned bytes differ: " + str(path))
    require(str(path) not in pins or pins[str(path)] == actual, "conflicting source pin")
    pins[str(path)] = actual
    return path


def recheck(pins):
    for name, sha in pins.items():
        require(digest(name) == sha, "original source changed during publication: " + name)


def document(path):
    return json.loads(Path(path).read_bytes())


def rows(path):
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def stamp(row):
    value = row["timestamp"]
    require(type(value) in (int, float) and math.isfinite(value), "finite actual simulation timestamp required")
    return round(value * 1e9)


def sampled_indices(count):
    # HH_261002 - Cover the whole saved camera interval, including warmup, HOLD and tail, not only selected TRAIN anchors.
    require(type(count) is int and count > 0, "nonempty actual camera denominator required")
    values = list(range(0, count, 5))
    if values[-1] != count - 1:
        values.append(count - 1)
    return values


def playback_durations(cameras, indices):
    require(indices == sampled_indices(len(cameras)), "fixed every-fifth plus exact-last visual membership required")
    gaps = [stamp(cameras[b]) - stamp(cameras[a]) for a, b in zip(indices, indices[1:])]
    require(all(gap > 0 for gap in gaps), "strictly advancing camera playback clocks required")
    # HH_261002 - GIF has 10ms units; disclose the final100ms hold rather than claiming a new observed camera.
    result = [round(gap / 10_000_000) * 10 for gap in gaps] + [100]
    require(all(value > 0 for value in result), "a real camera gap cannot disappear at GIF clock precision")
    return result


def observed_prior_ack(state, receipts):
    # HH_261002 - Show ACK identity only; pedals shown on screen come from state.current_control, never an outgoing proposal.
    eligible = [r for r in receipts if r.get("status") == "ACKNOWLEDGED" and r.get("server_accepted") is True
                and type(r.get("before_frame")) is int and r["before_frame"] < state["frame"]]
    require(bool(eligible), "observed state requires an earlier accepted ACK")
    chosen = max(eligible, key=lambda r: r["sequence"])
    expected = state["control_transport"]["expected_receipt_sequence"]
    require(type(expected) is int and expected > 0 and chosen["sequence"] == expected,
            "latest prior ACK identity differs from recorded observation")
    return chosen


def validate_records(states, cameras, receipts, geometry, counts):
    require(set(counts) == {"native", "camera", "ACK"} and all(type(v) is int and v > 0 for v in counts.values()),
            "externally pinned positive full denominators required")
    require((len(states), len(cameras), len(receipts)) == (counts["native"], counts["camera"], counts["ACK"]),
            "full actual native/camera/ACK denominator differs")
    by_frame = {r["frame"]: r for r in states}
    require(len(by_frame) == len(states) and all(type(r["frame"]) is int for r in states)
            and all(b["frame"] == a["frame"] + 1 and abs(stamp(b)-stamp(a)-50_000_000) <= 2
                    for a, b in zip(states, states[1:])), "complete20Hz native chain required")
    require(len(geometry) == len(states) + 1 and set(by_frame) < set(geometry), "all native body records plus real bootstrap required")
    require(set(geometry)-set(by_frame) == {states[0]["frame"]-1}, "only actual immediately preceding bootstrap body allowed")
    require([r["sequence"] for r in receipts] == list(range(1, len(receipts)+1)), "complete original ACK sequence required")
    require(all(type(r["sequence"]) is int for r in receipts), "Boolean is not an original ACK sequence")
    for state in states:
        observed_prior_ack(state, receipts)
        require(state["capture_phase"] in ("stationary_warmup", "driving", "stationary_tail"), "unknown native phase")
        require(all(type(state[k]) in (int,float) and math.isfinite(state[k]) for k in ("x", "y", "yaw", "vx", "vy")),
                "finite actual measured rear-base state required")
        control = state["current_control"]
        require(all(type(control[k]) in (int,float) and math.isfinite(control[k]) for k in ("throttle", "brake", "steer")),
                "finite actually observed native pedals required")
    for i, camera in enumerate(cameras):
        require(camera["frame"] in by_frame and stamp(camera) == stamp(by_frame[camera["frame"]])
                and camera["capture_phase"] == by_frame[camera["frame"]]["capture_phase"], "camera/native exactframe binding required")
        require(set(camera["images"]) == set(camera["source_timestamps"]) == set(visual.CAMERAS)
                and all(type(v) in (int,float) and math.isfinite(v) and round(v*1e9) == stamp(camera)
                        for v in camera["source_timestamps"].values()), "all six sameframe real cameras required")
        if i:
            require(camera["frame"] == cameras[i-1]["frame"] + 2 and abs(stamp(camera)-stamp(cameras[i-1])-100_000_000) <= 2,
                    "complete10Hz camera chain required")
    return by_frame


def validate_pixels(cameras, pixels):
    # HH_261002 - Bind every original-BGRA identity receipt to its saved JPEG, not merely a freely supplied image hash.
    keys=[(p["frame"],p["camera"]) for p in pixels]
    expected={(c["frame"],name) for c in cameras for name in visual.CAMERAS}
    require(len(keys)==len(set(keys))==len(expected) and set(keys)==expected, "all distinct original camera source receipts required")
    by_key=dict(zip(keys,pixels))
    for camera in cameras:
        require(set(camera["original_BGRA_source_sha256"])==set(camera["saved_JPEG_sha256"])==set(visual.CAMERAS), "all original/saved camera hash identities required")
        for name in visual.CAMERAS:
            row=by_key[(camera["frame"],name)];sha=row["original_BGRA_sha256"]
            require(type(sha) is str and len(sha)==64 and all(ch in "0123456789abcdef" for ch in sha)
                and sha==camera["original_BGRA_source_sha256"][name]
                and row["saved_JPEG_sha256"]==camera["saved_JPEG_sha256"][name]
                and row["saved_JPEG_relative_path"]==camera["images"][name]
                and row["timestamp"]==camera["source_timestamps"][name]
                and row["encoding"]=="original_carla_bgra8" and row["JPEG_encoder_unchanged"] is True
                and row["pose_or_image_post_transformed"] is False, "original source pixels/time/encoding identity differs")


def png_indices(bundle):
    cameras, by_frame = bundle["cameras"], bundle["by_frame"]
    result = {"01_first.png": 0, "05_last.png": len(cameras)-1}
    for name, phase in (("02_cruise.png", "GO"), ("03_coast.png", "BRAKE_TO_STOP"), ("04_HOLD.png", "HOLD")):
        matching = [i for i,c in enumerate(cameras) if by_frame[c["frame"]]["capture_phase"] == "driving"
                    and by_frame[c["frame"]].get("goal_stop", {}).get("terminal_mission_phase") == phase]
        # HH_261002 - Use the middle declared GO camera for cruise and the first recorded coast/HOLD camera; never select by success.
        selected = matching[len(matching)//2 if phase == "GO" else 0] if matching else None
        # HH_261002 - Failed/missing phases remain absent; use the last real frame with an explicit unavailable label, not a synthetic event.
        result[name] = selected if selected is not None else len(cameras)-1
    return result


def missing_png_phases(bundle):
    phases={bundle["by_frame"][c["frame"]].get("goal_stop",{}).get("terminal_mission_phase")
            for c in bundle["cameras"] if bundle["by_frame"][c["frame"]]["capture_phase"]=="driving"}
    return {name:phase for name,phase in (("02_cruise.png","GO"),("03_coast.png","BRAKE_TO_STOP"),("04_HOLD.png","HOLD")) if phase not in phases}


def validate_future_qa(future, case_sha, manifest_sha, pins):
    # HH_261002 - A previous version's QA must never become the current teacher's admission or outcome by a filename alone.
    require(future["schema"] == FUTURE_QA_SCHEMA and future["teacher_task_schema"] == TASK_SCHEMA
            and future["case_plan_sha256"] == case_sha and future["native_manifest_sha256"] == manifest_sha,
            "future QA must authenticate this exact V6 native declaration and manifest")
    require(type(future["selected_anchor_count"]) is int and type(future["failed_fixed_window_count"]) is int
            and 0 <= future["failed_fixed_window_count"] <= future["selected_anchor_count"]
            and future["selected_anchor_count"] > 0 and type(future["source_capture_failed"]) is bool,
            "retain truthful nonempty complete future-QA failure denominator")
    for name, sha in future["source_artifact_sha256"].items():
        add_pin(pins, name, valid_sha(sha))
    for key in ("training_data_approved", "model_promoted", "vehicle_control_approved"):
        if key in future:
            require(future[key] is False, "teacher publication must not grant future-QA approval")
    return future


def outcome_annotation(bundle):
    diagnostic, future = bundle["diagnostic"], bundle["future"]
    outcome = "Owner: " + diagnostic["status"]
    if future is not None:
        outcome += (" | future64: " + ("FAIL" if future["source_capture_failed"] or future["failed_fixed_window_count"] else "CHECKS_CLEAR_NOT_ADMITTED")
                    + " " + str(future["failed_fixed_window_count"]) + "/" + str(future["selected_anchor_count"]) + " windows")
    else:
        outcome += " | future64: NOT_PROVEN"
    return outcome


def load_archive(caller_path, caller_sha):
    pins = {}; add_pin(pins, caller_path, valid_sha(caller_sha)); caller = document(caller_path)
    require(set(caller) == {"schema", "episode", "artifact_pins", "counts"} and caller["schema"] == CALLER_SCHEMA,
            "exact reviewed V6 publication caller receipt required")
    episode = canonical(caller["episode"])
    require(episode in (CASE/"episode", CASE/"episode.partial"), "one actual V6 episode only")
    expected = dict(OTHER_ROLES, **{role:episode/name for role,name in EPISODE_ROLES.items()})
    refs = caller["artifact_pins"]
    qa_roles = {"future_qa_summary", "future_qa_windows"}
    require(type(refs) is dict and set(refs) in (set(expected), set(expected)|qa_roles), "all source artifact roles and paired QA evidence required")
    for role, path in expected.items():
        ref = refs[role]
        require(type(ref) is dict and set(ref) == {"path", "sha256"} and ref["path"] == str(path), "exact artifact role/path required")
        add_pin(pins, path, valid_sha(ref["sha256"]))
    future = None
    if "future_qa_summary" in refs:
        ref = refs["future_qa_summary"]
        require(type(ref) is dict and set(ref) == {"path", "sha256"}, "exact future QA caller pin required")
        path = canonical(ref["path"])
        require(path.is_relative_to(ROOT/"storage/experiments/validation/2026-10-02") and not path.is_relative_to(CASE)
                and path.name == "summary.json", "derived V6 QA summary outside original raw case only")
        add_pin(pins, path, valid_sha(ref["sha256"])); future = document(path)
        window_ref = refs["future_qa_windows"]
        require(type(window_ref) is dict and set(window_ref) == {"path", "sha256"}
                and window_ref["path"] == str(path.parent/"all_fixed_anchor_true_future64.jsonl"),
                "whole true64 QA journal must accompany the exact summary")
        window_path = add_pin(pins, window_ref["path"], valid_sha(window_ref["sha256"]))
        windows = rows(window_path)
        require(len(windows) == future["selected_anchor_count"]
                and sum(not w["necessary_defined_domain_checks_clear"] for w in windows) == future["failed_fixed_window_count"]
                and all(len(w["true_future_rows"]) == 64 and all(r["valid"] is True for r in w["true_future_rows"])
                        for w in windows)
                and future["true_future_point_count"] == future["required_true_future_point_count"] == 64*len(windows),
                "complete failed and clear all64 windows must match the original QA denominator")
    batch, plan, diagnostic, manifest = [document(expected[k]) for k in ("batch_plan", "case_plan", "diagnostic", "manifest")]
    require(plan["schema"] == TASK_SCHEMA and plan["run_id"] == "v6" and plan["case_id"] == "pilot_00"
            and manifest["capture_contract"]["natural_goal_stop_teacher"] == plan, "actual V6 declaration/native identity differs")
    require(len(batch["cases"]) == 1 and batch["cases"][0]["declaration_sha256"] == pins[str(expected["case_plan"])],
            "actual one-case batch declaration differs")
    require(document(expected["owner"])["declaration_sha256"] == pins[str(expected["case_plan"])],
            "original owner must authenticate the exact case declaration")
    require(diagnostic["learned_control"] is False and diagnostic["teacher_completion_is_not_learned_goal_stop"] is True,
            "teacher evidence must not be renamed learned success")
    for role in ("body_summary", "diagnostic"):
        for name, sha in document(expected[role]).get("input_sha256", {}).items(): add_pin(pins, name, valid_sha(sha))
    for name,sha in plan["source_sha256"].items():
        relative = Path(name); require(not relative.is_absolute() and ".." not in relative.parts, "declared source path escaped")
        add_pin(pins, ROOT/relative, valid_sha(sha)); add_pin(pins, CASE/"source"/relative, sha)
    for section in ("external_source_sha256","asset_sha256"):
        for name,sha in plan[section].items():
            path=canonical(name);require(path.is_relative_to(ROOT) or path.is_relative_to(CARLA_ROOT), "declared external source ownership root differs")
            archive=CASE/("source" if path.is_relative_to(ROOT) else "source_external")/path.relative_to(ROOT if path.is_relative_to(ROOT) else CARLA_ROOT)
            add_pin(pins,path,valid_sha(sha));add_pin(pins,archive,sha)
    if future is not None:
        validate_future_qa(future, pins[str(expected["case_plan"])], pins[str(expected["manifest"])], pins)
    add_pin(pins, Path(visual.__file__).resolve(), HELPER_SHA); add_pin(pins, Path(__file__).resolve())
    for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"): add_pin(pins, FONT_ROOT/name)
    states, cameras, receipts, bodies = [rows(expected[k]) for k in ("states", "cameras", "receipts", "body")]
    geometry = {r["frame"]:r for r in bodies}; require(len(geometry) == len(bodies), "duplicate body frame")
    by_frame = validate_records(states,cameras,receipts,geometry,caller["counts"])
    validate_pixels(cameras,rows(expected["pixels"]))
    paths = []
    for camera in cameras:
        for name in visual.CAMERAS:
            relative = Path(camera["images"][name]); require(not relative.is_absolute() and ".." not in relative.parts and relative.parts[:1] == ("images",), "safe saved camera path required")
            path = canonical(episode/relative); paths.append(path)
            add_pin(pins,path,valid_sha(camera["saved_JPEG_sha256"][name]))
            with Image.open(path) as picture:
                require(picture.format == "JPEG" and picture.size == (640,360), "original six-camera JPEG ABI differs")
                picture.load()
    require(len(paths) == len(set(paths)) == len(cameras)*6, "all distinct actual JPEGs required")
    recheck(pins)
    return dict(episode=episode,states=states,cameras=cameras,receipts=receipts,geometry=geometry,by_frame=by_frame,
                route=document(expected["route"])["route"],diagnostic=diagnostic,future=future,pins=pins)


def render(bundle, index):
    camera=bundle["cameras"][index]; state=bundle["by_frame"][camera["frame"]]
    receipt=observed_prior_ack(state,bundle["receipts"])
    canvas=Image.new("RGB",(1700,860),visual.BG); draw=ImageDraw.Draw(canvas)
    draw.text((24,18),"Town07 V6 ACTUAL TEACHER | " + outcome_annotation(bundle),fill=visual.AMBER,font=visual.font(22,True))
    draw.text((24,63),"BasicAgent + manual1/coast/service HOLD | NO learned-model driving | NOT Autoware/RViz UI",fill=visual.MUTED,font=visual.font(18))
    elapsed=(stamp(state)-stamp(bundle["states"][0]))/1e9; speed=math.hypot(state["vx"],state["vy"])
    intended=state.get("goal_stop",{}).get("terminal_mission_phase","N/A")
    draw.text((24,101),f"Simulation t={elapsed:.2f}s | frame={state['frame']} | actual rear-base speed={speed:.5f}m/s | recorded phase={state['capture_phase']}",fill=visual.WHITE,font=visual.font(19))
    for i,name in enumerate(visual.CAMERAS):
        x,y=24+i%3*416,150+i//3*277; draw.text((x,y),name,fill=visual.WHITE,font=visual.font(18,True))
        with Image.open(bundle["episode"]/camera["images"][name]) as picture:
            resampling=getattr(Image,"Resampling",Image)
            canvas.paste(picture.convert("RGB").resize((400,225),resampling.LANCZOS),(x,y+30))
    route=[(p["x"],p["y"]) for p in bundle["route"]]
    traveled=[(s["x"],s["y"]) for s in bundle["states"] if s["frame"]<=state["frame"]]
    panel=visual.map_panel(state,bundle["geometry"][state["frame"]],route,traveled,[])
    pen=ImageDraw.Draw(panel);pen.rectangle((14,500,407,528),fill=visual.PANEL)
    pen.text((20,505),"Teacher only: NO neural trajectory",fill=visual.MUTED,font=visual.font(15));canvas.paste(panel,(1270,145))
    ctrl=state["current_control"]
    draw.text((24,731),f"OBSERVED native pedals: throttle={ctrl['throttle']:.3f} brake={ctrl['brake']:.3f} steer={ctrl['steer']:.3f} | prior ACK #{receipt['sequence']}",fill=visual.WHITE,font=visual.font(18,True))
    draw.text((24,770),f"Same-frame intended teacher phase: {intended} (annotation, NOT proof of already applied effect)",fill=visual.AMBER,font=visual.font(18))
    draw.text((24,809),outcome_annotation(bundle).partition(" | ")[2]
              + " | whole warmup/HOLD/tail retained | NOT TRAIN or model approval",fill=visual.AMBER,font=visual.font(17))
    return canvas


def publish(output, caller_path, caller_sha):
    output=canonical(output); require(output == OUTPUT and not output.exists(), "fresh fixed V6 publication category required")
    bundle=load_archive(caller_path,caller_sha); fixed=sampled_indices(len(bundle["cameras"]));durations=playback_durations(bundle["cameras"],fixed)
    output.mkdir(parents=True,exist_ok=False);frames=[render(bundle,i).quantize(colors=192) for i in fixed]
    gif=output/"full_recorded_interval_six_camera_ego_centered_2hz.gif"
    with gif.open("xb") as stream:frames[0].save(stream,format="GIF",save_all=True,append_images=frames[1:],duration=durations,loop=0,disposal=2,optimize=False)
    # HH_261002 - Encoding may coalesce identical display pixels; verify whole elapsed playback rather than inventing camera frames.
    with Image.open(gif) as saved:
        encoded_count = saved.n_frames
        encoded_duration = 0
        for index in range(encoded_count):
            saved.seek(index); encoded_duration += saved.info["duration"]
        require(encoded_duration == sum(durations), "encoded GIF must preserve the whole actual playback duration")
    media=[gif]; pngs=png_indices(bundle);missing=missing_png_phases(bundle)
    for name,index in pngs.items():
        path=output/name
        picture=render(bundle,index)
        if name in missing:
            pen=ImageDraw.Draw(picture);pen.rectangle((14,15,1685,55),fill=visual.PANEL)
            pen.text((24,18),"Requested "+missing[name]+" snapshot UNAVAILABLE | last actual frame, NOT a fabricated event",fill=visual.AMBER,font=visual.font(23,True))
        with path.open("xb") as stream:picture.save(stream,format="PNG")
        media.append(path)
    recheck(bundle["pins"])
    provenance=dict(schema=SCHEMA,actual_teacher=True,learned_control=False,autoware_UI_captured=False,
        image_generation_used=False,training_data_approved=False,model_promoted=False,vehicle_control_approved=False,
        main_functions_approved=0,subfunctions_approved=0,source_sha256=bundle["pins"],
        actual_counts=dict(native=len(bundle["states"]),camera=len(bundle["cameras"]),ACK=len(bundle["receipts"]),JPEG=len(bundle["cameras"])*6),
        GIF_camera_indices=fixed,GIF_frame_duration_ms=durations,GIF_total_duration_ms=sum(durations),
        GIF_encoded_frame_count=encoded_count,GIF_encoded_total_duration_ms=encoded_duration,
        source_camera_span_ns=stamp(bundle["cameras"][-1])-stamp(bundle["cameras"][0]),final_added_display_hold_ms=100,
        PNG_camera_indices=pngs,PNG_missing_phase_last_frame_fallback=missing,original_outcome=bundle["diagnostic"],future_QA=bundle["future"],
        whole_recorded_interval_retained=True,terminal_ACK_later_effect_not_fabricated=True,
        sameframe_intended_phase_is_annotation_not_observed_actuation=True,
        original_BGRA_hash_receipts_verified_but_BGRA_bytes_not_archived=True,
        files={p.name:dict(sha256=digest(p),bytes=p.stat().st_size) for p in media})
    with (output/"publication_provenance.json").open("x") as stream:json.dump(provenance,stream,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream:
        stream.write("# 실제 목적지 정차 교사 주행 v6\n\n이 화면은 실제 기록된 6카메라·측정 차량 위치·경로를 보여줍니다. BasicAgent 교사 제어이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다.\n\n"
                     +"원래 결과: `"+outcome_annotation(bundle)+"`. 실패·대기 상태와 전체 출발·정지 유지·후속 기록을 보존하며 학습 승인으로 해석하지 않습니다.\n\n"
                     +"![실제 전체 기록]("+gif.name+")\n\n마지막 실제 카메라: [PNG](05_last.png). 화면의 페달은 현재 측정값이고, 같은 프레임의 교사 의도 표시는 이미 적용됐다는 증거가 아닙니다.\n")
    recheck(bundle["pins"])
    return dict(status="ACTUAL_TEACHER_MEDIA_CREATED_NO_APPROVAL",output=str(output),GIF=1,PNG=5,
                source_sha256=digest(output/"publication_provenance.json"))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--caller-pins",type=Path,required=True);parser.add_argument("--caller-pins-sha256",required=True)
    parser.add_argument("--output",type=Path,default=OUTPUT);args=parser.parse_args(argv)
    print(json.dumps(publish(args.output,args.caller_pins,args.caller_pins_sha256),allow_nan=False));return 0


if __name__ == "__main__":raise SystemExit(main())
