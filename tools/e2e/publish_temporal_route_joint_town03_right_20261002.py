"""HH_261002 - Publish one complete variable-length Town03 attempt, including pre-NN failure; never infer or approve."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys
from types import FunctionType, SimpleNamespace

from PIL import Image, ImageDraw

from . import publish_temporal_route_recovery_fast45_batch_20261001 as core

prior, visual, require = core.prior, core.visual, core.audit.require
ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/16_actual_town03_right"
RUN_OUTPUTS = {"temporal_route_joint_town03_known_dev_right_v1": "v1_pre_nn_launch_failure",
    "temporal_route_joint_town03_known_dev_right_v2": "v2_controlled_green_basic_turn"}
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SCHEMA = "portable_e2e.actual_joint_town03_known_dev_right_publication.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_right.20261002.v1"
ROUTE_SHA = "47ea507b9a4228fdce79ef58eff1e1d2cf149c02dd2df413ebadf53756f771ee"
HELPERS = {
    "tools/e2e/publish_temporal_route_recovery_fast45_batch_20261001.py": "458ba01eafc74c5382684fc0aff7d3971b1d0b80bbfc615b6c8ca79760f49ee1",
    "tools/e2e/publish_temporal_v2_actual_failure_20261001.py": "667a646e7f2dc34e3b02b34ecda3993ac27409434b727ac7eacaa700bbb14abe",
    "tools/e2e/publish_town07_w1_pi_gallery_20261001.py": "ecf3b011e96e9942376b3a2e5502bff86e727a54e9c128726291a05b865fd78e",
    "tools/e2e/collect_temporal_route_joint_town03_right_20261002.py": "f5eff0f8da70533b94748ef10718d55fed63af9974ec1e2a080708ec4a46bf97",
    "tools/e2e/research_temporal_route_joint_town03_right_20261002.py": "ebfde24f322192b816584b10a52e7bf15a53090157d1753a37d08ec239286340"}
MODEL_CAMERA_ORDER = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")


def canonical(path):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve() and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink path required")
    return path


def sha(value):
    require(type(value) is str and re.fullmatch("[0-9a-f]{64}", value), "exact external SHA256 required"); return value


def strict_json(payload):
    def pairs(values):
        result = {}
        for key, value in values: require(key not in result, "duplicate JSON key"); result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs, parse_constant=lambda _: require(False, "nonfinite JSON"))


class Pins:
    # HH_261002 - Bind the exact parsed bytes once and preserve their first SHA through rendering.
    def __init__(self): self.values = {}
    def file(self, path, expected):
        path = canonical(path); expected = sha(expected)
        require(prior.digest(path) == expected and self.values.get(str(path), expected) == expected, "external or first-read pin differs")
        self.values[str(path)] = expected
    def payload(self, path):
        path = canonical(path); require(str(path) in self.values and path.stat().st_size <= 512 * 1024**2, "bounded externally pinned input required")
        value = path.read_bytes(); require(hashlib.sha256(value).hexdigest() == self.values[str(path)], "parsed bytes differ from first pin"); return value
    def json(self, path): return strict_json(self.payload(path))
    def rows(self, path):
        value = self.payload(path)
        if str(path).endswith(".gz"): value = gzip.decompress(value); require(len(value) <= 512 * 1024**2, "bounded gzip expansion required")
        return [strict_json(line) for line in value.splitlines() if line.strip()]
    def recheck(self):
        for path, expected in self.values.items(): require(prior.digest(canonical(path)) == expected, "original changed after first read: " + path)


def inventory(root):
    root = canonical(root); require(root.is_dir(), "completed actual archive required")
    paths = list(root.rglob("*")); require(not any(p.is_symlink() for p in paths), "archive symlink forbidden")
    return {str(p.relative_to(root)) for p in paths if p.is_file()}


def publication_sources():
    return core.audit.source_closure(Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py")


def output_for(run):
    require(run.parent == ROOT / "storage/experiments/validation/2026-10-02" and run.name in RUN_OUTPUTS,
        "exact prospective v1/v2 run-to-output mapping required")
    return canonical(OUTPUT / RUN_OUTPUTS[run.name])


def observed_prior_ack(state, receipts, commands, by_frame, *, unlogged_preNN=False):
    if not unlogged_preNN: return prior.observed_prior_ack(state, receipts, commands, by_frame)
    # HH_261002 - Only authenticated zero-NN early failure can lack the entire proposal journal; do not invent teacher rows.
    require(commands == [], "unlogged pre-NN adapter cannot hide an existing proposal")
    choices = [r for r in receipts if r.get("status") == "ACKNOWLEDGED" and r.get("server_accepted") is True and r["before_frame"] < state["frame"]]
    require(choices, "native observation lacks prior accepted ACK")
    receipt = max(choices, key=lambda r: r["sequence"])
    require(receipt.get("reason") in {"pre_capture_bootstrap", "stationary_warmup_start", "next_stationary_warmup_control",
        "initial_drive_control", "next_driving_control"}, "unlogged pre-NN ACK source is not an original startup phase")
    return receipt, None, [], "PRE-NN original native ACK / " + receipt["reason"] + " / no logged model plan"


def renderer_call(function, bundle, index):
    # HH_261002 - A private function namespace injects the zero-NN selector without modifying frozen helper globals.
    proxy = SimpleNamespace(safe_image=prior.safe_image, observed_prior_ack=lambda *args:
        observed_prior_ack(*args, unlogged_preNN=bundle["unlogged_preNN"]))
    delegated = FunctionType(function.__code__, dict(function.__globals__, prior=proxy), function.__name__, function.__defaults__, function.__closure__)
    return delegated(bundle, index)


def camera_indices(count):
    # HH_261002 - Fixed half-second bins plus the exact final original camera; never discard a failed suffix.
    require(type(count) is int and 1 <= count <= 1100, "actual bounded camera denominator required")
    return sorted(set(range(0, count, 5)) | {count - 1})


def png_indices(count):
    require(type(count) is int and count > 0, "actual image count required")
    return {name: (count - 1) * numerator // 4 for name, numerator in
        (("01_start.png", 0), ("02_quarter.png", 1), ("03_half.png", 2), ("04_three_quarters.png", 3), ("05_last.png", 4))}


def playback(bundle, indices):
    states, cameras = bundle["states"], bundle["cameras"]
    end = states[-1]["timestamp"] + .05
    stamps = [cameras[i]["timestamp"] for i in indices] + [end]
    durations = [int(round((b - a) * 100)) * 10 for a, b in zip(stamps, stamps[1:])]
    require(all(0 < value <= 500 for value in durations), "genuine whole-interval GIF bin timing required")
    require(sum(durations) == int(round((end - states[0]["timestamp"]) * 100)) * 10, "full native interval cannot be shortened")
    return durations


def verify_records(states, cameras, commands, receipts, geometry, *, unlogged_preNN=False):
    require(1 < len(states) <= 2200 and len(cameras) <= 1100, "actual whole event or110s bound required")
    frames = [s["frame"] for s in states]
    require(all(type(f) is int for f in frames) and frames == list(range(frames[0], frames[0] + len(frames))), "whole consecutive native chain required")
    require(all(type(s["timestamp"]) in (int, float) and math.isfinite(s["timestamp"]) for s in states)
        and all(abs(b["timestamp"] - a["timestamp"] - .05) <= 1.e-6 for a, b in zip(states, states[1:])), "original native20Hz stamps required")
    require([c["frame"] for c in cameras] == frames[::2], "every original even native camera required")
    require(len(geometry) == len(states) and [g["frame"] for g in geometry] == frames, "full native measured bodies/sweeps required")
    require([r["sequence"] for r in receipts] == list(range(1, len(receipts) + 1))
        and all(r.get("status") == "ACKNOWLEDGED" and r.get("server_accepted") is True for r in receipts), "whole accepted ACK chain required")
    sequences = [r["ack_receipt_sequence"] for r in commands]
    require(len(set(sequences)) == len(commands) and set(sequences) <= {r["sequence"] for r in receipts}
        and all(r.get("application", {}).get("acknowledged") is True for r in commands), "unique original accepted proposal ACKs required")
    by_frame = {s["frame"]: s for s in states}
    for camera in cameras:
        require(tuple(camera["camera_order"]) == MODEL_CAMERA_ORDER and set(camera["images"]) == set(visual.CAMERAS)
            and set(camera["source_timestamps"]) == set(visual.CAMERAS) and camera["timestamp"] == by_frame[camera["frame"]]["timestamp"]
            and all(t == camera["timestamp"] for t in camera["source_timestamps"].values()), "all6 exact native camera stamps/order required")
    for state in states:
        receipt, _, _, _ = observed_prior_ack(state, receipts, commands, by_frame, unlogged_preNN=unlogged_preNN)
        require(receipt["sequence"] == state["control_transport"]["expected_receipt_sequence"]
            and receipt["before_frame"] == receipt["after_ack_frame"] == state["frame"] - 1
            and state["control_transport"]["status"] == "PASS", "latest prior ANY-source native ACK required")
        requested, observed = receipt["requested_control"], state["current_control"]
        require(all(abs(requested[k] - observed[k]) <= 1.e-6 for k in ("throttle", "brake", "steer"))
            and all(requested[k] == observed[k] for k in ("hand_brake", "reverse", "manual_gear_shift"))
            and (not requested["manual_gear_shift"] or requested["gear"] == observed["gear"]), "actual prior applied pedal binding differs")
    return by_frame


def control_census(bundle):
    # HH_261002 - Outgoing NN requests and genuine later native effects have different, non-vacuous denominators.
    observed = []
    for state in bundle["states"]:
        _, command, _, _ = observed_prior_ack(state, bundle["receipts"], bundle["commands"], bundle["by_frame"],
            unlogged_preNN=bundle.get("unlogged_preNN", False))
        if command and command["application"].get("learned_control_applied") is True: observed.append(state["frame"])
    return dict(observed_prior_learned_native_effect_count=len(observed),
        first_observed_learned_frame=observed[0] if observed else None,
        last_observed_learned_frame=observed[-1] if observed else None,
        acknowledged_learned_command_count=sum(r.get("application", {}).get("learned_control_applied") is True for r in bundle["commands"]),
        fresh_learned_proposal_count=sum(r.get("learned_control_proposed") is True and r.get("held_between_camera_frames") is False for r in bundle["commands"]),
        teacher_startup_command_count=sum(r.get("control_source") == "expert_startup" for r in bundle["commands"]),
        emergency_command_count=sum(r.get("control_source") == "emergency_brake" for r in bundle["commands"]),
        camera_delivery_wall_timing_not_inferred=True)


def outcome_text(bundle):
    census = bundle["control_census"]; diag = bundle["diagnostic"]
    if not census["observed_prior_learned_native_effect_count"]:
        return "NO observed NN effect / setup or startup failure", "NN 제어 효과 관측 0개 — 출발/수집 실패, NN 우회전 판정 불가"
    if diag["joint_turn_motion_body_policy_wall100ms_passed"]:
        return "PASS precursor / NOT RTE03 approval", "예비 기준 PASS(기능 승인 아님)"
    return "FAIL / all failed tail retained", "FAIL(실패 전체 보존)"


def markdown(bundle, pngs):
    # HH_261002 - An early hard abort is a full available archive, not a completed 110-second NN turn test.
    diag = bundle["diagnostic"]; census = bundle["control_census"]; _, verdict = outcome_text(bundle)
    text = "# 실제 Town03 우회전 시도: 실패를 포함한 전체 기록\n\n" + verdict + "입니다. 목적지 정차나 정식 RTE03 성공이 아닙니다. 이미 사용한 DEV 경로 한 조건으로 신규 독립 평가 0개·메인0/9·하위0/30입니다.\n\n"
    if not census["observed_prior_learned_native_effect_count"]:
        text += "이 기록은 실제 NN 제어 효과를 관측하기 전에 끝났습니다. **NN 우회전 성능의 실패나 성공으로 세지 않습니다.** 교사·위험 감지의 제동과 모델 명령을 구분하며, 수신→ACK 처리 시간을 없던 자료로 만들지 않습니다.\n\n"
    else:
        text += "교사 출발 뒤 실제 NN 제어 효과가 관측된 기록입니다. 모델 제어·긴급제동·교사 출발을 실제 이전 ACK로 구분합니다.\n\n"
    text += f"실제 {len(bundle['states'])}개 상태·{len(bundle['cameras'])}개 6카메라 묶음을 전량 확인했습니다. 상태 최초→마지막은 {bundle['states'][-1]['timestamp'] - bundle['states'][0]['timestamp']:.2f}초이며, GIF는 마지막 상태의 50ms 표시 구간까지 포함합니다. 사전 110초 상한을 채웠다고 주장하지 않습니다. 관측된 NN 효과 {census['observed_prior_learned_native_effect_count']}개·ACK된 NN 명령 {census['acknowledged_learned_command_count']}개·신규 NN 제안 {census['fresh_learned_proposal_count']}개입니다. 고정 0.5초 간격과 정확한 마지막 카메라를 포함하며 PNG는 최초/¼/½/¾/마지막입니다. 실패 꼬리나 좋은 회전 장면만 고르지 않습니다.\n\n"
    errors = {key: diag[key] for key in ("original_capture_error", "diagnostic_error") if diag.get(key)}
    if errors: text += "원본 오류(변경 없이 보존): `" + json.dumps(errors, ensure_ascii=False).replace("`", "'") + "`\n\n"
    text += "전체 정식 주행·제어 전송·가감속 판정을 새로 PASS로 바꾸지 않습니다. 관측 뒤 cleanup ACK가 있어도 없는 미래 차량 자세나 카메라를 만들지 않습니다. 별도 원본 감사가 있으면 그 FAIL도 provenance에 그대로 보존합니다.\n\n![실제 전체 기록](full_actual_six_cameras_2hz.gif)\n\n"
    for name in pngs: text += f"![{name}]({name})\n\n"
    text += "차량 위치 중심 지도에서 파랑은 원 경로, 초록은 현재까지 실제 궤적, 보라는 현재 상태에 앞서 실제 ACK가 확인된 모델 계획입니다. 긴급제동 이후 보라를 지우며 교사 출발은 모델 제어로 세지 않습니다. 실제 CARLA 카메라와 기록 기반 진단으로 Autoware/RViz 촬영이 아닙니다. 전체 owner 판정·실패·원본 SHA·현재 프레임과 과거ACK는 [publication_provenance.json](publication_provenance.json)에 남겼습니다.\n"
    return text


def load_archive(caller_path, caller_sha, *, decode=False):
    require("torch" not in sys.modules and "carla" not in sys.modules, "no inference/simulator process required")
    pins = Pins(); pins.file(caller_path, caller_sha); caller = pins.json(caller_path)
    require(caller.get("schema") == CALLER_SCHEMA
        and caller.get("attempts") == 1 and caller.get("PNG_index_policy") == "first_quarter_half_threequarters_exact_last",
        "prospective single-case complete publication caller required")
    run = canonical(caller["run_root"])
    output = output_for(run); require(caller.get("output") == str(output), "cross-run or unknown output is forbidden")
    owner_schema = OWNER_SCHEMA[:-1] + run.name[-1]
    require(set(caller["run_files_sha256"]) == inventory(run), "entire externally pinned original archive required")
    for name, value in caller["run_files_sha256"].items():
        require(not Path(name).is_absolute() and ".." not in Path(name).parts and Path(name).as_posix() == name, "safe relative whole-archive member required")
        pins.file(run / name, value)
    sources = publication_sources()
    require(set(caller["publication_source_sha256"]) == {str(p.relative_to(ROOT)) for p in sources}, "complete publication code closure required")
    for path in sources: pins.file(path, caller["publication_source_sha256"][str(path.relative_to(ROOT))])
    for name, expected in HELPERS.items(): pins.file(ROOT / name, expected)
    for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"):
        path = Path("/usr/share/fonts/truetype/dejavu") / name; pins.file(path, caller["font_sha256"][str(path)])
    supplemental = {}
    for name, expected in caller.get("auxiliary_root_audit_sha256", {}).items():
        path = canonical(name)
        require(path.is_relative_to(ROOT / "storage/experiments/diagnostics/2026-10-02") and path.suffix == ".json",
            "supplemental externally pinned root diagnostic JSON required")
        pins.file(path, expected); supplemental[name] = pins.json(path)
    plan, owner, diagnostic = [pins.json(run / name) for name in ("declaration.json", "owner_result.json", "diagnostic_report.json")]
    require(plan["schema"] == owner_schema and plan["output"] == str(run) and plan["town"] == "Town03" and plan["scenario"] == "right"
        and plan["dataset_split"] == "val" and plan["known_DEV_site_reused"] is True and plan["independent_validation"] is False
        and plan["attempts"] == 1 and plan["automatic_retry"] is False and plan["total_capture_seconds"] == 110
        and plan["optimizer_updates"] == 6000 and plan["speed_preview_s"] == .5 and plan["controller_target_cap_mps"] == 2.4
        and plan["training_data_approved"] is False and plan["full_RTE03_approved"] is False, "original fixed known-DEV NN event contract required")
    if run.name.endswith("_v2"):
        green = plan.get("owned_green_signal_fixture", {})
        expected = dict(original_own_world_traffic_actors="ALL", state="Green", frozen=True, ignore_traffic_lights=False,
            all_original_hazard_detectors_and_emergency_returns_unchanged=True, known_v1_hazard_subtype_proven=False,
            closed_course_basic_turn_only_not_traffic_rule_performance=True)
        require(plan.get("actual_manual_first_gear_bootstrap_and_engagement_required") is True
            and all(type(green.get(k)) is type(v) and green[k] == v for k, v in expected.items()),
            "v2 controlled-green/manual setup must remain disclosed without disabling original hazards")
    require(owner["declaration_sha256"] == caller["run_files_sha256"]["declaration.json"]
        and diagnostic["schema"] == owner_schema + ".diagnostic_report" and diagnostic["original_owner"] == owner,
        "actual owner/diagnostic/declaration identity differs")
    for key in ("full_MAIN9_completion_claimed", "full_RTE03_approved", "training_data_approved", "vehicle_control_approved", "model_promoted"):
        require(diagnostic[key] is False, "original denial cannot be promoted: " + key)
    for table, base in ((plan["source_sha256"], ROOT), (plan["external_source_sha256"], CARLA_ROOT), (plan["asset_sha256"], None)):
        for name, expected in table.items():
            source = canonical((base / name) if base == ROOT else name)
            origin = ROOT if source.is_relative_to(ROOT) else CARLA_ROOT
            require(source.is_relative_to(origin), "declared source outside owned roots")
            archived = run / ("source" if origin == ROOT else "source_external") / source.relative_to(origin)
            pins.file(source, expected); pins.file(archived, expected)
    episode = run / ("episode" if (run / "episode").is_dir() else "episode.partial")
    raw_names = {str(episode / name) for name in inventory(episode)}
    require(set(diagnostic["raw_source_sha256"]) == raw_names, "all failed-tail raw files retained in original diagnostic")
    for name, expected in diagnostic["raw_source_sha256"].items(): require(pins.values.get(name) == expected, "original raw snapshot differs from root caller")
    manifest, route = pins.json(episode / "manifest.json"), pins.json(episode / "route.json")
    require(pins.values[str(episode / "route.json")] == ROUTE_SHA and route["town"] == "Town03" and route["scenario"] == "right"
        and len(route["route"]) == 213 and manifest["status"] == diagnostic["original_capture_status"], "entire unchanged213-point mission and original outcome required")
    require(manifest.get("training_data_approved") is False and manifest.get("vehicle_control_approved") is False,
        "original native research denials must remain unchanged")
    states, cameras, receipts = [pins.rows(episode / name) for name in ("states.jsonl", "camera_frames.jsonl", "control_receipts.jsonl")]
    summary = pins.json(episode / "learned_summary.json")
    unlogged_preNN = not (episode / "learned_control.jsonl").exists()
    if unlogged_preNN:
        counts = summary["counts"]
        require(summary.get("handoff_occurred") is False
            and all(type(counts.get(k)) is int and counts[k] == 0 for k in
                ("inference_count", "prefill_count", "learned_commands_acknowledged", "emergency_commands"))
            and all(s.get("research_control_source") in (None, "expert_startup") and s.get("research_inference_frame") is None for s in states),
            "missing proposal journal contradicts authenticated zero-NN summary/native sources")
        commands = []
    else: commands = pins.rows(episode / "learned_control.jsonl")
    body = pins.json(run / "derived_body/summary.json")
    geometry = pins.rows(run / "derived_body/all_native_boxes_and_sweeps.jsonl.gz")
    require(body == diagnostic["independent_body"] and body["all_native_boxes"] == len(states)
        and body["all_snapshot_linear_sweeps"] == len(states) - 1 and body["lane_inset_m"] == .1,
        "all actual body and10cm inset results, including failures, required")
    by_frame = verify_records(states, cameras, commands, receipts, geometry, unlogged_preNN=unlogged_preNN)
    require(diagnostic["actual_native_state_count"] == len(states) and diagnostic["actual_camera_bundle_count"] == len(cameras)
        and diagnostic["actual_ACK_receipt_count"] == len(receipts), "whole actual denominators differ")
    images = []
    for camera in cameras:
        for name in visual.CAMERAS:
            path = canonical(prior.safe_image(episode, camera["images"][name])); require(str(path) in pins.values, "real image lacks external pin")
            if decode:
                with Image.open(path) as image: require(image.format == "JPEG" and image.size == (640, 360), "real JPEG ABI differs"); image.load()
            images.append(path)
    require(len(images) == len(set(images)) == 6 * len(cameras), "full distinct six-camera denominator required")
    trial = dict(trial_index=0, original_driving_screen_passed=diagnostic["learned_driving_screen_passed"],
        fresh_observed_delivery_to_ACK100ms_passed=diagnostic.get("observed_bundle_to_ACK_wall100ms_passed", False))
    bundle = dict(episode=episode, states=states, cameras=cameras, commands=commands, receipts=receipts, by_frame=by_frame,
        geometry={g["frame"]: g for g in geometry}, route=route["route"], trial=trial, diagnostic=diagnostic,
        auxiliary_root_audits=supplemental, output=output, unlogged_preNN=unlogged_preNN, original_learned_summary=summary)
    bundle["control_census"] = control_census(bundle)
    if run.name.endswith("_v1"):
        require(bundle["control_census"]["observed_prior_learned_native_effect_count"] == 0
            and bundle["control_census"]["acknowledged_learned_command_count"] == 0
            and summary.get("handoff_occurred") is False
            and all(type(summary["counts"].get(k)) is int and summary["counts"][k] == 0 for k in
                ("inference_count", "prefill_count", "learned_commands_acknowledged")), "v1 pre-NN output cannot contain actual NN control")
    require(bundle["control_census"]["observed_prior_learned_native_effect_count"] > 0
        or diagnostic["joint_turn_motion_body_policy_wall100ms_passed"] is False, "zero observed NN effect cannot be claimed a driving PASS")
    pins.recheck(); require(inventory(run) == set(caller["run_files_sha256"]), "original archive membership changed")
    return caller, bundle, pins


def render(bundle, index):
    # HH_261002 - Reuse original camera/map/prior-ACK pixels; only remove Town07/fixed45s/old-weight text.
    out = renderer_call(core.render, bundle, index); draw = ImageDraw.Draw(out); diag = bundle["diagnostic"]
    verdict, _ = outcome_text(bundle)
    draw.rectangle((0, 0, 1699, 91), fill=visual.BG)
    draw.text((24, 18), "Actual Town03 known-DEV attempt | " + verdict, fill=visual.WHITE, font=visual.font(24, True))
    draw.text((24, 63), "Final6000 / CPU4 / PI2.40 / preview0.5 / one attempt / unchanged complete navigation", fill=visual.MUTED, font=visual.font(18))
    draw.rectangle((0, 801, 1699, 859), fill=visual.BG)
    draw.text((24, 807), f"Whole recorded interval: {len(bundle['states'])}native / {len(bundle['cameras'])}six-camera / {bundle['control_census']['observed_prior_learned_native_effect_count']}observed NN effects | MAIN0/9 SUB0/30", fill=visual.WHITE, font=visual.font(16))
    draw.text((24, 835), "Known-site diagnostic, NOT independent validation / recorded CARLA, NOT Autoware or RViz UI", fill=visual.MUTED, font=visual.font(16))
    return out


def publish(args):
    caller, bundle, pins = load_archive(canonical(args.caller_pins), args.caller_pins_sha256, decode=not args.validate_only)
    output = bundle["output"]; require(not output.exists(), "new mapped revision create-only required")
    indices = camera_indices(len(bundle["cameras"])); durations = playback(bundle, indices); pngs = png_indices(len(bundle["cameras"]))
    if args.validate_only: return dict(validated=True, native_count=len(bundle["states"]), camera_count=len(bundle["cameras"]),
        control_census=bundle["control_census"], GIF_frames=len(indices), rendered=False, output_created=False)
    ancestor = next(p for p in output.parents if p.exists()); require(shutil.disk_usage(ancestor).free >= 5 * 1024**3 + 256 * 1024**2, "5GiB plus256MiB publication reserve required")
    pins.recheck(); output.mkdir(parents=True, exist_ok=False)
    frames = [render(bundle, i) for i in indices]; gif = output / "full_actual_six_cameras_2hz.gif"
    with gif.open("xb") as stream: frames[0].save(stream, format="GIF", save_all=True, append_images=frames[1:], duration=durations, loop=0, optimize=False)
    with Image.open(gif) as saved:
        require(saved.n_frames == len(indices), "whole GIF interval required")
        actual = []
        for index in range(saved.n_frames): saved.seek(index); actual.append(saved.info["duration"])
        require(actual == durations, "original whole capture playback bins changed")
    media = [gif]
    for name, index in pngs.items():
        path = output / name
        with path.open("xb") as stream: render(bundle, index).save(stream, format="PNG")
        media.append(path)
    provenance = dict(schema=SCHEMA, caller_sha256=args.caller_pins_sha256, source_and_input_sha256=dict(pins.values),
        original_diagnostic=bundle["diagnostic"], original_learned_summary=bundle["original_learned_summary"],
        actual_proposal_journal_absent= bundle["unlogged_preNN"], actual_output=str(output), GIF_camera_indices=indices, GIF_duration_ms=durations,
        GIF_total_duration_ms=sum(durations), PNG_camera_indices=pngs, control_census=bundle["control_census"],
        original_native_snapshot_span_s=bundle["states"][-1]["timestamp"] - bundle["states"][0]["timestamp"],
        final_native_display_bin_s=.05, original_declared_interval_complete=bundle["diagnostic"].get("bounded_capture_complete") is True,
        full_110s_capture_claimed=len(bundle["states"]) == 2200 and bundle["diagnostic"].get("bounded_capture_complete") is True,
        auxiliary_root_audits=bundle["auxiliary_root_audits"], original_transport_and_motion_verdicts_not_promoted=True,
        displayed_GIF_frames=[renderer_call(core.displayed_receipt, bundle, i) for i in indices],
        displayed_PNG_frames={name: renderer_call(core.displayed_receipt, bundle, i) for name, i in pngs.items()},
        exact_last_original_camera_included=True, last_native_frame=bundle["states"][-1]["frame"], last_camera_frame=bundle["cameras"][-1]["frame"],
        native_terminal_frame_without_camera_not_fabricated=True, failed_tail_cropped=False, teacher_startup_not_counted_as_model_control=True,
        prior_ACK_any_source=True, emergency_clears_purple_plan=True, actual_full_RTE03_or_MAIN9_completion_claimed=False,
        known_DEV_site_reused=True, independent_validation=False, model_inference_executed=False, simulator_started=False,
        autoware_UI_captured=False, synthetic_driving_image_generated=False, model_promoted=False, vehicle_control_approved=False,
        media_sha256={path.name: prior.digest(path) for path in media})
    pins.recheck(); require(inventory(Path(caller["run_root"])) == set(caller["run_files_sha256"]), "original membership changed during publication")
    with (output / "publication_provenance.json").open("x") as stream: json.dump(provenance, stream, ensure_ascii=False, indent=2, allow_nan=False)
    diag = bundle["diagnostic"]
    with (output / "README.md").open("x") as stream: stream.write(markdown(bundle, pngs))
    pins.recheck()
    return dict(output=str(output), GIF_count=1, PNG_count=5, GIF_frames=len(indices), original_raw_rewritten=False, outcome=diag["status"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--caller-pins", required=True); parser.add_argument("--caller-pins-sha256", required=True); parser.add_argument("--validate-only", action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)), ensure_ascii=False, allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
