"""HH_261001 - Publish the complete original TemporalV2 failure with actual cameras and prior-ACK semantics.

This create-only publisher reads one byte-pinned archive. It never runs a
simulator, a model, or an actuator, and never replaces failed-tail source data.
"""

from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw

from . import publish_town07_w1_pi_gallery_20261001 as visual


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-01/temporal_v2_owned_camera_town07_screen_v1"
BODY = RUN / "audit_v1/native_body_and_failure_v2"
OUTPUT = ROOT / "docs/assets/validation/2026-10-01/temporal_v2_actual_failure_results_v1"
SCHEMA = "portable_e2e.temporal_v2_actual_failure_publication.20261001.v1"
FIXED_SHA256 = {
    RUN / "declaration.json": "c6f25e089366ce5f71c24ecffe705026843e9014da11704ee80a41a7b3dd013a",
    RUN / "diagnostic_report.json": "bb29abcaf6d7a21090fae2236bcc485f621307a8d577731fe727be324edc9c73",
    RUN / "owner_result.json": "4b47b54ce8daef01e51aa7f4d6e9f3c0a1c7e81cd126c0ecf2da70a6ce222b52",
    BODY / "summary.json": "e761b365ab84d71e3cb217dd8006ee132df28794b0684f082a54cdda611c07b0",
    BODY / "all400_native_body_and399_sweeps.jsonl.gz": "3c01236f938090bf92666027e3f0d2ef22eb07edabf954c229bfead98b6eaac2",
    ROOT / "tools/e2e/publish_town07_w1_pi_gallery_20261001.py": "ecf3b011e96e9942376b3a2e5502bff86e727a54e9c128726291a05b865fd78e",
}
CHECKPOINT_SHA = "7902216449dd8e7a601d63128a38773a5f3bf59c1660a7684b7f2d79791659b6"
PNG_INDICES = {
    "01_stationary_startup.png": 0,
    "02_last_accepted_learned_camera.png": 40,
    "03_rejection_observation_prior_learned_ACK.png": 41,
    "04_first_saved_camera_after_emergency_ACK.png": 42,
    "05_failed_tail_last_2hz_frame.png": 195,
}
require = visual.require


def digest(path):
    # HH_261001 - Hash regular original files, never follow an input alias or rewrite raw evidence.
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular original file required: " + str(path))
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            value.update(block)
    return value.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_bytes())


def read_rows(path):
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def assert_pins(pins):
    for name, expected in pins.items():
        require(isinstance(expected, str) and len(expected) == 64 and digest(name) == expected,
                "source or original artifact changed: " + str(name))


def safe_image(episode, name):
    relative = Path(name)
    require(not relative.is_absolute() and ".." not in relative.parts and relative.parts[:1] == ("images",),
            "safe original JPEG relative path required")
    path = episode / relative
    require(path.resolve(strict=True).is_relative_to(episode.resolve(strict=True))
            and not path.is_symlink(), "camera path escaped the original episode")
    return path


def verify_outcomes(body, diagnostic, owner, manifest):
    # HH_261001 - A geometry pass must not erase the first rejection, full tail, failed owner or denied approvals.
    require(body.get("status") == "PASS_GEOMETRY_ONLY_WITH_DRIVING_FAILURE_RETAINED"
            and body.get("native_state_count") == 400 and body.get("camera_bundle_count") == 200
            and body.get("acknowledged_receipt_count") == 404
            and body.get("whole_proposed_command_count") == 377,
            "complete fixed failed capture and geometry-only scope required")
    require(body.get("original_manifest_status") == manifest.get("status") == "failed"
            and body.get("original_failed_tail_cropped") is False
            and body.get("original_manifest_and_goal_flags_rewritten") is False
            and body.get("threshold_relaxed") is False, "original failed tail or original flags changed")
    for value in (body, diagnostic, manifest):
        require(value.get("vehicle_control_approved") is False and value.get("training_data_approved") is False,
                "original denial must remain explicit")
    require(owner.get("status") == "CAPTURE_RETURNED_NONZERO_RETAINED"
            and owner.get("learned_driving_screen_passed") is False
            and owner.get("model_promoted") is False and owner.get("full_RTE01_approved") is False
            and owner.get("source_hashes_match") is True and owner.get("external_source_hashes_match") is True,
            "failed owner or immutable source closure changed")
    require(body.get("source_checkpoint_sha256") == diagnostic.get("source_checkpoint_sha256") == CHECKPOINT_SHA,
            "fixed final3000 checkpoint identity differs")
    failure = body["first_latched_failure"]
    require(failure.get("first_rejection_frame") == 1562 and failure.get("first_rejection_ACK_sequence") == 86
            and failure.get("original_rejection_was_policy_deadline") is False
            and failure.get("failed_tail_cropped") is False and failure.get("no_post_rejection_resumption") is True
            and failure.get("unchanged_base_speed_envelope_mps") == 2.5
            and failure.get("original_emergency_reason") == "actual current speed exceeds unchanged low-speed envelope; no clipping",
            "first latched speed-envelope failure identity differs")
    require(math.isclose(failure["actual_unclipped_base_planar_speed_mps"], 2.505561754250865, abs_tol=1e-12)
            and math.isclose(diagnostic["learned_effect_progress_m"], 4.681280715894664, abs_tol=1e-12),
            "original unclipped speed or native actor progress changed")
    require(body["actual_state_outgoing_source_annotation_counts"]["emergency_brake"] == 318
            and body["independent_transport"]["status"] == "PASS"
            and body["independent_camera_alignment"]["status"] == "PASS", "full emergency/ACK/camera evidence required")
    for name, count in (("sampled_body_raw_lane", 400), ("sampled_body_inset_10cm_lane", 400),
                        ("previous_sweep_raw_lane", 399), ("previous_sweep_inset_10cm_lane", 399)):
        metric = body["body_geometry"][name]
        require(metric["status"] == "PASS" and metric["assessed_count"] == count and metric["violation_count"] == 0,
                "all original body and snapshot sweeps must remain visible")
    require(diagnostic.get("learned_driving_screen_passed") is False
            and diagnostic.get("predeclared_observed_bundle_to_ACK_wall_latency_screen_passed") is False
            and diagnostic.get("ten_hz_end_to_end_proven") is False
            and diagnostic["wall_timing"]["exact_bundle_delivery_to_ACK_seconds"]["count"] == 24,
            "independent driving and observed wall-latency failure must remain visible")


def verify_records(states, cameras, commands, receipts, geometry):
    # HH_261001 - Every native state and camera is retained, including warmup, startup and the latched failure tail.
    require(len(states) == 400 and len(cameras) == 200 and len(commands) == 377 and len(receipts) == 404,
            "no cropped or selected successful subset is legal")
    frames = [state["frame"] for state in states]
    require(frames == list(range(1480, 1880)), "original complete 20 Hz native frame chain required")
    require([row["frame"] for row in cameras] == frames[::2], "all original 10 Hz saved camera bundles required")
    require(set(geometry) == set(frames) and len(geometry) == 400, "whole-body records must match all native frames")
    by_frame = {state["frame"]: state for state in states}
    for camera in cameras:
        require(set(camera["images"]) == set(visual.CAMERAS)
                and set(camera["source_timestamps"]) == set(visual.CAMERAS)
                and camera["timestamp"] == by_frame[camera["frame"]]["timestamp"]
                and all(stamp == camera["timestamp"] for stamp in camera["source_timestamps"].values()),
                "six actual cameras must bind to the exact observed native snapshot")
    sequences = [row["sequence"] for row in receipts]
    require(sequences == list(range(1, 405)) and all(row.get("status") == "ACKNOWLEDGED"
            and row.get("server_accepted") is True for row in receipts), "all native ACK receipts must remain present")
    command_sequences = [row["ack_receipt_sequence"] for row in commands]
    require(len(command_sequences) == len(set(command_sequences))
            and set(command_sequences) <= set(sequences), "proposals must bind to unique original ACKs")
    require(Counter(row["control_source"] for row in commands) == {
            "expert_startup": 11, "learned_trajectory_tracker": 48, "emergency_brake": 318},
            "all three original control-source phases and failed tail required")
    for state in states:
        receipt, command, plan, label = observed_prior_ack(state, receipts, commands, by_frame)
        require(receipt["sequence"] == state["control_transport"]["expected_receipt_sequence"],
                "displayed prior ACK differs from actual current native control")
        require(state["control_transport"]["status"] == "PASS", "observed control binding must have passed")
    return by_frame


def observed_prior_ack(state, receipts, commands, by_frame):
    """HH_261001 - Select the latest PRIOR accepted ACK of any source, not the latest learned ACK.

    Same-frame outgoing proposals cannot be presented as observed actuator
    effects. Emergency ACKs explicitly clear every older purple learned plan.
    """
    prior = [row for row in receipts if row.get("status") == "ACKNOWLEDGED"
             and row.get("server_accepted") is True and row["before_frame"] < state["frame"]]
    require(prior, "native observation lacks an earlier accepted ACK")
    receipt = max(prior, key=lambda row: row["sequence"])
    choices = [row for row in commands if row.get("ack_receipt_sequence") == receipt["sequence"]]
    require(len(choices) <= 1, "ambiguous command/ACK association")
    command = choices[0] if choices else None
    if command is None:
        require(receipt.get("reason") in {"pre_capture_bootstrap", "stationary_warmup_start", "next_stationary_warmup_control"},
                "unclassified prior ACK must not be relabelled expert startup")
        return receipt, None, [], "stationary warmup / prior native brake ACK"
    application = command.get("application", {})
    require(application.get("acknowledged") is True, "unacknowledged proposal cannot be displayed as applied")
    source = application.get("applied_source")
    if source == "emergency_brake":
        require(application.get("learned_control_applied") is False, "emergency must never display an old learned plan")
        return receipt, command, [], "EMERGENCY BRAKE / latched rejection / purple plan cleared"
    if application.get("learned_control_applied") is True:
        require(source == command.get("control_source") == "learned_trajectory_tracker"
                and command["command_frame"] == receipt["before_frame"]
                and command["inference_frame"] <= command["command_frame"]
                and command["inference_frame"] in by_frame, "original learned ACK and inference identity required")
        origin = by_frame[command["inference_frame"]]
        c, s = math.cos(origin["yaw"]), math.sin(origin["yaw"])
        points = command["selected"]["xy_base_m"]
        require(len(points) == 64 and all(len(point) == 2 and all(math.isfinite(v) for v in point) for point in points),
                "unaltered single selected model trajectory ABI required")
        plan = [(origin["x"]+c*x-s*y, origin["y"]+s*x+c*y) for x, y in points]
        return receipt, command, plan, "prior ACKed learned plan / " + ("held" if command.get("held_between_camera_frames") else "fresh")
    require(source == "expert" and command.get("control_source") == "expert_startup"
            and application.get("learned_control_applied") is False, "unknown source cannot be disguised as expert startup")
    return receipt, command, [], "expert startup only / not learned driving"


def displayed_camera_indices(count=200):
    # HH_261001 - Forty fixed half-second bins cover the complete twenty-second capture, including failed tail.
    require(type(count) is int and count == 200, "fixed original 200-camera denominator required")
    return list(range(0, count, 5))


def load_bundle():
    assert_pins(FIXED_SHA256)
    body = read_json(BODY / "summary.json")
    pins = {Path(name): value for name, value in body["source_sha256"].items()}
    for name, value in FIXED_SHA256.items():
        require(name not in pins or pins[name] == value, "conflicting original source/artifact pins")
        pins[name] = value
    pins[Path(__file__)] = digest(__file__)
    pins[ROOT / "learning/tests/test_publish_temporal_v2_actual_failure_20261001.py"] = digest(
        ROOT / "learning/tests/test_publish_temporal_v2_actual_failure_20261001.py")
    for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"):
        path = Path("/usr/share/fonts/truetype/dejavu") / name
        pins[path] = digest(path)
    assert_pins(pins)
    episode = RUN / "episode.partial"
    diagnostic, owner, manifest = [read_json(path) for path in
        (RUN / "diagnostic_report.json", RUN / "owner_result.json", episode / "manifest.json")]
    verify_outcomes(body, diagnostic, owner, manifest)
    states, cameras, commands, receipts = [read_rows(episode / name) for name in
        ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl")]
    geometry_rows = read_rows(BODY / "all400_native_body_and399_sweeps.jsonl.gz")
    require(len(geometry_rows) == 400, "all four hundred body rows required before dictionary indexing")
    geometry = {row["frame"]: row for row in geometry_rows}
    by_frame = verify_records(states, cameras, commands, receipts, geometry)
    image_paths = []
    for camera in cameras:
        for name in visual.CAMERAS:
            path = safe_image(episode, camera["images"][name])
            require(path in pins and digest(path) == pins[path], "actual JPEG not present in pinned whole-capture audit")
            with Image.open(path) as image:
                require(image.size == (640, 360) and image.format == "JPEG", "original six-camera image ABI differs")
                image.load()
            image_paths.append(path)
    require(len(image_paths) == len(set(image_paths)) == 1200, "all 1200 distinct original JPEGs required")
    return dict(episode=episode, states=states, cameras=cameras, commands=commands, receipts=receipts,
        by_frame=by_frame, geometry=geometry, route=read_json(episode / "route.json")["route"],
        body=body, diagnostic=diagnostic, owner=owner, input_sha256=pins)


def render(bundle, camera_index):
    # HH_261001 - Render only real saved pixels and measured geometry; no fake Autoware desktop or inferred scene.
    camera = bundle["cameras"][camera_index]
    state = bundle["by_frame"][camera["frame"]]
    receipt, command, plan, label = observed_prior_ack(state, bundle["receipts"], bundle["commands"], bundle["by_frame"])
    canvas = Image.new("RGB", (1700, 850), visual.BG)
    draw = ImageDraw.Draw(canvas)
    draw.text((24, 17), "TemporalV2 actual FAILURE | full original 20 seconds retained", fill=visual.WHITE, font=visual.font(28, True))
    draw.text((24, 59), "Six recorded CARLA cameras + native footprint | NOT Autoware UI | body PASS does not mean driving PASS",
              fill=visual.MUTED, font=visual.font(19))
    elapsed = state["timestamp"]-bundle["states"][0]["timestamp"]
    v0 = math.hypot(state["vx"], state["vy"])
    geom = bundle["geometry"][state["frame"]]
    # HH_261001 - Do not display a negative pre-handoff distance from a future learned-phase reference.
    actor_progress = (f"{geom['independent_actor_arc_m']-bundle['geometry'][1515]['independent_actor_arc_m']:.2f}m"
                      if state["frame"] >= 1515 else "not started")
    draw.text((24, 99), f"t={elapsed:.2f}s | frame={state['frame']} | measured base v0={v0:.5f}m/s"
              f" | actor progress since first learned effect={actor_progress}", fill=visual.WHITE, font=visual.font(20))
    for index, name in enumerate(visual.CAMERAS):
        x, y = 24 + index % 3 * 416, 150 + index // 3 * 277
        draw.text((x, y), name, fill=visual.WHITE, font=visual.font(18, True))
        with Image.open(safe_image(bundle["episode"], camera["images"][name])) as source:
            resampling = getattr(Image, "Resampling", Image)
            canvas.paste(source.convert("RGB").resize((400, 225), resampling.LANCZOS), (x, y+30))
    route = [(point["x"], point["y"]) for point in bundle["route"]]
    traveled = [(row["x"], row["y"]) for row in bundle["states"] if row["frame"] <= state["frame"]]
    panel = visual.map_panel(state, geom, route, traveled, plan)
    panel_draw = ImageDraw.Draw(panel)
    panel_draw.rectangle((14, 500, 407, 528), fill=visual.PANEL)
    panel_draw.text((20, 505), "Purple: prior applied learned plan" if plan else "Purple: NONE for current prior ACK",
                    fill=visual.PURPLE if plan else visual.MUTED, font=visual.font(15))
    canvas.paste(panel, (1270, 145))
    control = state["current_control"]
    color = "#ff9b9b" if command and command.get("control_source") == "emergency_brake" else visual.WHITE
    draw.text((24, 728), f"Observed prior ACK #{receipt['sequence']}: {label}", fill=color, font=visual.font(20, True))
    draw.text((24, 766), f"Actual current control: throttle={control['throttle']:.3f} / brake={control['brake']:.3f}"
              f" / steer={control['steer']:.3f} | same-frame next proposal is NOT observed actuator effect",
              fill=visual.WHITE, font=visual.font(18))
    draw.text((24, 807), "2Hz playback of all 20s | 400 native states / 200 camera bundles / 1200 original JPEGs | 318 emergency commands retained",
              fill=visual.MUTED, font=visual.font(17))
    return canvas


def korean_readme(bundle):
    # HH_261001 - Separate geometric containment, inference time, observed bundle-to-ACK latency and driving failure.
    failure, diagnostic = bundle["body"]["first_latched_failure"], bundle["diagnostic"]
    wall = diagnostic["wall_timing"]["exact_bundle_delivery_to_ACK_seconds"]
    return f"""# TemporalV2 실제 주행 실패 — 전체 20초 기록

<!-- HH_261001 - Retain the complete actual failed capture and distinguish geometry from driving success. -->

결론: **차선 안에 차량 몸체는 유지했지만, 주행 시험은 실패했습니다.** 모델 주행 후 속도가 안전 상한을 넘어서 비상제동이 걸렸고, 남은 정지 구간까지 모두 보존했습니다. 정식 기능 승인 수는 여전히 **0/30**입니다. 이 자료로 9개 주행 대분류가 완료되었다고 판단하지 않습니다.

## 먼저 볼 자료

![전체 20초: 실제 6개 카메라와 차량 중심 지도](full20s_actual_six_cameras_2hz.gif)

GIF는 처음부터 끝까지 20초를 2 Hz, 40장으로 보여줍니다. 원자료는 20 Hz 상태 400개, 저장 카메라 10 Hz 묶음 200개, 실제 JPEG 1,200장 전체입니다. 정지 준비·전문가 초기 가속·모델 주행·비상제동 이후 실패 구간을 삭제하거나 성공 구간만 선택하지 않았습니다. 녹화 종료 직전 이미지는 마지막 0.5초 구간의 시작을 표시합니다.

| 실제 장면 | 설명 |
|---|---|
| [01 초기 정지 준비](01_stationary_startup.png) | 전문가/모델 주행이 아닌 정지 준비 ACK입니다. |
| [02 마지막 정상 모델 카메라](02_last_accepted_learned_camera.png) | 실제 관측 직전 ACK로 적용된 모델 경로만 보라색으로 표시합니다. |
| [03 속도 초과를 관측한 순간](03_rejection_observation_prior_learned_ACK.png) | 이 순간의 관측은 이전 모델 ACK의 효과입니다. 같은 프레임에서 새로 제안한 비상제동을 이미 적용된 것처럼 표시하지 않습니다. |
| [04 비상제동 ACK 이후](04_first_saved_camera_after_emergency_ACK.png) | 이전 ACK가 비상제동으로 바뀌면 보라색 모델 경로를 지우고 emergency brake로 표시합니다. |
| [05 실패 후 정지 끝부분](05_failed_tail_last_2hz_frame.png) | 학습 주행이나 전문가 초기 주행으로 잘못 표시하지 않습니다. |

지도는 모든 장면에서 실제 rear-axle base_link 위치를 화면 중심에 고정합니다. 파란색은 저장된 내비게이션 경로, 초록색은 지금까지 실제 관측한 궤적, 황색은 측정 차량 몸체입니다. 보라색은 **가장 최근의 이전 ACK가 모델 제어를 실제 적용한 경우에만** 표시합니다. CARLA 실제 카메라+기록 기반 진단 화면이며, Autoware/RViz 실시간 화면을 촬영한 것이라고 주장하지 않습니다.

## 왜 멈췄나?

모델로 실제 제어를 시작한 뒤 약 {failure['elapsed_since_first_learned_command_s']:.2f}초, frame {failure['first_rejection_frame']}에서 실제 base_link 평면 속도 `hypot(vx,vy)`가 **{failure['actual_unclipped_base_planar_speed_mps']:.8f} m/s**가 되었습니다. 기존 안전 상한 2.5 m/s보다 {failure['base_speed_excess_mps']:.8f} m/s 높았습니다. 속도를 낮게 잘라 보고하지 않았고, 원래 안전 검사대로 거절·비상제동을 유지했습니다.

이 최초 거절의 정책 처리 시간은 {failure['original_policy_seconds']*1000:.2f} ms, NN 추론 시간은 {failure['original_raw_NN_seconds']*1000:.2f} ms였습니다. **최초 비상제동 원인은 시간 초과가 아니라 실제 속도 상한 초과입니다.** 이후 비상제동 명령 318개를 전체 원자료에 남겼습니다. 모델 주행 시작 이후 native actor 기준 경로 진행량은 **{diagnostic['learned_effect_progress_m']:.3f} m**였고, 가장 긴 정지 구간은 {diagnostic['learned_effect_maximum_stationary_dwell_s']:.2f}초였습니다. 20초·30 m 이상 계속 주행하는 예비 주행 시험에도 미달이며, 정식 RTE-01 승인이 아닙니다.

## 각각 무엇이 통과했고 실패했나?

| 검사 | 결과와 해석 |
|---|---|
| 실제 주행 | **FAIL** — {diagnostic['learned_effect_progress_m']:.3f} m 진행 뒤 비상제동/정지. 주행 성공으로 승격하지 않았습니다. |
| 몸체·차선 경계 | **PASS, 기하 검사만** — 400개 차량 몸체와 399개 인접 snapshot sweep이 원래 차선 및 10 cm 안쪽 차선을 모두 지켰습니다. 연속시간 물리 전체나 주행 기능 완료를 증명하는 것은 아닙니다. |
| 충돌·차선 침범 이벤트 | 기록된 관측 이벤트 0개. 몸체 기하 통과와 별도로 확인했습니다. |
| native ACK/상태·카메라 정합 | **PASS** — 명령 수신과 관측 연결을 확인했으며 물리 액추에이터 전달 시간까지 증명한 것은 아닙니다. |
| 관측된 bundle delivery→ACK 벽시계 지연 | **FAIL** — fresh 모델 ACK {wall['count']}개 전체 기준 p95 {wall['p95']*1000:.3f} ms / 최대 {wall['maximum']*1000:.3f} ms. 기존 100 ms 수신→ACK 기준을 넘었습니다. |
| 센서 노출→ACK 및 10 Hz 실시간 전체 성능 | **미증명** — 센서 callback 진입/노출 시각 계측이 없으므로 위 지연을 end-to-end FPS라고 부르지 않습니다. |
| 학습 데이터/실차 제어/모델 승격 | **미승인** — 원래 manifest, 실패 결과와 승인 거부 값을 변경하지 않았습니다. |

## 다음 개선은 무엇인가?

첫째, 원래 모델 출력·측정 속도·2.5 m/s 안전 상한은 유지하고, **별도 선언한 PI 제어 목표 2.40 m/s**로 물리적 가속 overshoot 여유를 시험합니다. 이는 모델 자체 성능 향상과 구분되는 제어 보정입니다. 둘째, 327개 저속 TRAIN anchor만 반복한 모델의 기존 회전/직진 표현 손실을 줄이기 위해 **raw200 부모에서 저속 2개+기존 회전 1개+직진 1개를 혼합하는 새 고정 3,000-update 연구 학습**을 별도로 검증합니다. 새 checkpoint는 동일하게 실패 포함 폐루프 시험을 통과하기 전에는 승격하지 않습니다. 추가 헤드/기능 승인은 기본 주행 성능 회복 이후입니다.

## 원자료와 재현

원 실행: `{RUN.relative_to(ROOT)}`. 전체 몸체 검사: `{BODY.relative_to(ROOT)}`. 별도 [publication_provenance.json](publication_provenance.json)에 원 실행·모든 원 JPEG·원 source/archive·검사 결과·새 publisher와 PNG/GIF의 SHA-256을 기록했습니다. 생성 전후 원본 해시를 모두 검사하며 원자료를 다시 쓰지 않습니다. 이 페이지의 그림은 생성형 이미지가 아닙니다.

```bash
cd /home/a/autoware_e2e
python3 -B tools/workspace_python.py -m scripts.e2e.publish_temporal_v2_actual_failure_20261001 --
```

출력은 새 폴더만 생성합니다. 이미 존재하면 덮어쓰지 않고 중단합니다. `--validate-only`는 입력 검증만 수행하고 자료 폴더를 만들지 않습니다.
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "create-only publication directory required")
    bundle = load_bundle()
    indices = displayed_camera_indices()
    if args.validate_only:
        assert_pins(bundle["input_sha256"])
        print(json.dumps(dict(status="VALIDATED_ORIGINAL_FAILED_CAPTURE_NO_PUBLICATION", native_states=400,
            camera_bundles=200, original_JPEGs=1200, planned_GIF_frames=40, output_created=False)))
        return 0
    OUTPUT.mkdir(parents=True, exist_ok=False)
    displayed = []
    frames = []
    for index in indices:
        camera = bundle["cameras"][index]
        state = bundle["by_frame"][camera["frame"]]
        receipt, command, plan, label = observed_prior_ack(state, bundle["receipts"], bundle["commands"], bundle["by_frame"])
        frames.append(render(bundle, index))
        displayed.append(dict(camera_index=index, observed_frame=state["frame"], timestamp_s=state["timestamp"],
            prior_ACK_sequence=receipt["sequence"], prior_ACK_frame=receipt["before_frame"], observed_source=label,
            learned_plan_drawn=bool(plan), current_observed_control=state["current_control"]))
    gif = OUTPUT / "full20s_actual_six_cameras_2hz.gif"
    with gif.open("xb") as stream:
        frames[0].save(stream, format="GIF", save_all=True, append_images=frames[1:], duration=500, loop=0, optimize=False)
    files = [gif]
    for name, index in PNG_INDICES.items():
        path = OUTPUT / name
        with path.open("xb") as stream:
            render(bundle, index).save(stream, format="PNG")
        files.append(path)
    readme = OUTPUT / "README.md"
    with readme.open("x") as stream:
        stream.write(korean_readme(bundle))
    files.append(readme)
    with Image.open(gif) as image:
        require(image.n_frames == 40 and image.size == (1700, 850), "all forty full-capture GIF frames required")
        durations = []
        for index in range(image.n_frames):
            image.seek(index); durations.append(image.info["duration"])
        require(durations == [500]*40 and sum(durations) == 20_000, "no shortened or accelerated failed-tail playback")
    assert_pins(bundle["input_sha256"])
    provenance = dict(schema=SCHEMA,
        comment="HH_261001 - Preserve real full-capture failure pixels and clear the old learned plan after emergency ACK.",
        original_run=str(RUN), original_body_audit=str(BODY), actual_native_states=400,
        original_camera_bundles=200, checked_original_JPEGs=1200, GIF_frame_count=40, playback_hz=2,
        playback_seconds=20, failed_tail_cropped=False, original_emergency_command_count=318,
        displayed_frames=displayed, supplemental_PNG_camera_indices=PNG_INDICES,
        source_and_artifact_sha256={str(path): value for path, value in sorted(bundle["input_sha256"].items())},
        original_hashes_match_after_publication=True,
        created_file_sha256={path.name: digest(path) for path in files},
        map_renderer_source_sha256=FIXED_SHA256[ROOT / "tools/e2e/publish_town07_w1_pi_gallery_20261001.py"],
        map_plan_semantics="most recent prior accepted ACK of any source; purple only for learned_control_applied=true; emergency clears purple",
        actual_ack_is_not_physical_actuator_timing_proof=True,
        measured_vehicle_center_fixed_in_all_scenes=True, autoware_UI_captured=False,
        synthetic_driving_image_generated=False, body_geometry_passed=True, learned_driving_passed=False,
        formal_approved_subfeatures=0, formal_subfeature_total=30, full_RTE01_approved=False,
        vehicle_control_approved=False, training_data_approved=False, model_promoted=False,
        observed_bundle_delivery_to_ACK_screen_passed=False, ten_hz_end_to_end_proven=False)
    with (OUTPUT / "publication_provenance.json").open("x") as stream:
        json.dump(provenance, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")
    print(json.dumps(dict(status="ACTUAL_FAILED_CAPTURE_FULL20S_PUBLISHED_NOT_PROMOTED", output=str(OUTPUT))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
