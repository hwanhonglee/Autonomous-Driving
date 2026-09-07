#!/usr/bin/env python3
"""HH_260906 - Verify two actual stationary camera runs and publish pixel-preserving Low/Epic comparisons."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_carla_goal_stop_trials as base
from scripts.e2e import summarize_carla_low_speed_response as pedal

require = base.require
CAMERAS = base.CAMERAS
TICKS = (100, 120, 140, 160)
ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
RIG_SHA = "ad7a7f727edb4ad0da1fcf6a2f9d1c9ed0f453e0adc7a03c5298be50f699d508"
MAPPING_SHA = "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136"
CALIBRATION_SHA = "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea"
WORKER = "scripts/e2e/probe_carla_stationary_camera_quality.py"
SOURCES = pedal.SOURCES[:-1] + (WORKER,) + base.BOUNDS_SOURCE_PATHS


def decode_jpeg(path):
    """HH_260906 - Decode the captured JPEG without cropping, color correction, denoising or enhancement."""
    from PIL import Image
    with Image.open(path) as image:
        require(image.format == "JPEG" and image.size == (640, 360), "captured image format/dimensions differ from frozen rig")
        image.load()
        return image.convert("RGB")


def pixel_sha(image):
    """HH_260906 - A pixel digest binds decoded RGB bytes; it does not recover pre-JPEG sensor pixels."""
    return hashlib.sha256(image.tobytes()).hexdigest()


def validate_states(states):
    """HH_260906 - Validate all160 requested full-brake states, including the70-tick setup period."""
    require(len(states) == 160 and [row["tick"] for row in states] == list(range(1, 161)), "stationary state count/tick schedule differs")
    for index, row in enumerate(states):
        base.integer(row["frame"])
        timestamp, speed = base.number(row["timestamp"]), base.number(row["planar_speed_mps"])
        require(speed >= 0 and (row["tick"] <= 70 or speed <= .1), "stationary vehicle moved after settling")
        for name in ("x", "y", "z", "roll", "pitch", "yaw"):
            base.number(row["actor_transform"][name])
        if index:
            require(row["frame"] - states[index - 1]["frame"] == 1
                    and abs(timestamp - states[index - 1]["timestamp"] - .05) <= 1e-4, "native frame/timestamp cadence mismatch")
        control = base.control(row["reported_control"])
        require(all(abs(control[name] - value) <= 1e-6 for name, value in (("throttle", 0.), ("brake", 1.), ("steer", 0.)))
                and all(row["reported_control"][name] is False for name in ("hand_brake", "reverse", "manual_gear_shift")), "API-reported control differs from fixed brake request")


def validate_frames(frames, states, root, ledger):
    """HH_260906 - Verify all24 camera images and bind every recorded anchor to its same-tick state."""
    require(len(frames) == 4 and [row["tick"] for row in frames] == list(TICKS), "stationary camera anchor schedule differs")
    records = []
    for entry in frames:
        state = states[entry["tick"] - 1]
        require(entry["complete"] is True and set(entry["images"]) == set(CAMERAS)
                and entry["frame"] == state["frame"] and abs(base.number(entry["timestamp"]) - state["timestamp"]) <= 1e-6, "camera anchor is incomplete or not bound to its state")
        for camera in CAMERAS:
            item = entry["images"][camera]
            expected = f"images/tick_{entry['tick']:03d}_{camera}.jpg"
            require(item["path"] == expected and abs(base.number(item["timestamp"]) - state["timestamp"]) <= 1e-6, "camera path/timestamp mismatch")
            raw = pedal.read_bytes(root, "camera_audit/" + expected, ledger)
            require(hashlib.sha256(raw).hexdigest() == item["sha256"], "captured JPEG SHA mismatch")
            image = decode_jpeg(root / "camera_audit" / expected)
            records.append({"tick": entry["tick"], "camera": camera, "frame": entry["frame"],
                            "relative_path": "camera_audit/" + expected, "jpeg_sha256": item["sha256"],
                            "decoded_rgb_sha256": pixel_sha(image), "width": image.width, "height": image.height})
    require({path.name for path in (root / "camera_audit/images").iterdir()} == {Path(row["relative_path"]).name for row in records}, "missing or extra camera image payload")
    return records


def verify_run(root, quality):
    """HH_260906 - Require finalized measurement and owned-server cleanup, not merely present screenshots."""
    root = Path(root).resolve()
    ledger = []
    read = lambda name, **kwargs: base.read_file(root, name, ledger, **kwargs)
    plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    require(quality in ("Low", "Epic") and plan["quality"] == quality, "quality arm identity mismatch")
    require(owner["exit_code"] == 0 and plan["capture_mode"] == owner["capture_mode"] == "stationary-camera"
            and plan["worker_path"] == WORKER and plan["learned_model_control"] is owner["learned_model_control"] is False
            and plan["vehicle_control_approved"] is owner["vehicle_control_approved"] is False, "stationary measurement owner did not finalize successfully")
    require(set(plan["source_sha256"]) == set(SOURCES) == set(owner["source_checks"])
            and plan["source_bytes_archived"] is owner["source_bytes_unchanged_and_archived"] is True
            and all(value is True for value in owner["source_checks"].values()), "all eleven source archives/postchecks are required")
    require(re.fullmatch(r"[0-9a-f]{40}", plan["source_head_commit"]) is not None, "invalid source commit")
    require({path.relative_to(root / "provenance").as_posix() for path in (root / "provenance").rglob("*") if path.is_file() or path.is_symlink()} == set(SOURCES), "source archive path set mismatch")
    for name in SOURCES:
        raw = pedal.read_bytes(root, "provenance/" + name, ledger)
        require(hashlib.sha256(raw).hexdigest() == plan["source_sha256"][name], "archived executed source SHA mismatch")
        committed = subprocess.check_output(["git", "show", f"{plan['source_head_commit']}:{name}"], cwd=ROOT)
        require(raw == committed, "executed source bytes differ from recorded commit")
    pid = base.integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and plan["host"] == "127.0.0.1" and plan["port"] == started["port"]
            and plan["map"] == started["map"] == "Town07", "owned simulator identity mismatch")
    log = pedal.read_bytes(root, "server.log", ledger)
    previous_time = None
    for stage in ("ready", "after_capture", "stopped"):
        proof = read(f"lifecycle/{stage}.json")
        require(proof["status"] == "PASS" and proof["stage"] == stage and proof["read_only"] is True and proof["error"] is None
                and proof["owner_pid"] == proof["owner_pgid"] == pid and proof["generation_id"] == f"expert_{pid}"
                and proof["host"] == plan["host"] and proof["port"] == plan["port"] and proof["expected_map"] == "Town07", "lifecycle ownership proof mismatch")
        info = proof["server_log"]
        require(0 < info["size_bytes"] <= len(log) and hashlib.sha256(log[:info["size_bytes"]]).hexdigest() == info["sha256"], "lifecycle log prefix mismatch")
        if stage == "stopped":
            require(proof["mode"] == "stopped" and proof["port_released"] is True and proof["owner_process_state"] is None, "server not fully stopped")
        else:
            require(proof["mode"] == "running" and proof["active_map_basename"] == "Town07", "active simulator map mismatch")
        now = datetime.fromisoformat(proof["checked_at"])
        require(previous_time is None or now >= previous_time, "unordered lifecycle timestamps")
        previous_time = now
    require(datetime.fromisoformat(owner["completed_at_utc"]) >= previous_time, "owner finalized before cleanup")
    manifest = read("camera_audit/manifest.json")
    require(manifest["schema"] == "carla.stationary_camera_quality.v1" and manifest["status"] == "COMPLETE"
            and manifest["learned_model_control"] is manifest["training_data_approved"] is False
            and manifest["cleanup_errors"] == [], "camera audit not complete or not measurement-only")
    require(manifest["physics_hz"] == 20. and manifest["camera_sensor_tick_seconds"] == 0.
            and manifest["camera_bundle_observed_every_native_tick"] is True and manifest["scheduled_ticks"] == manifest["state_count"] == 160
            and manifest["saved_camera_ticks"] == list(TICKS) and manifest["saved_camera_anchor_count"] == 4
            and manifest["partial_camera_anchor_count"] == 0, "camera/native schedule or completion mismatch")
    require(manifest["all_commands"] == {"throttle": 0., "brake": 1., "steer": 0.} and manifest["vehicle_type"] == "vehicle.toyota.prius"
            and manifest["weather"] == "ClearNoon", "stationary vehicle/scenario changed")
    require(len(manifest["owned_actor_ids"]) == len(set(manifest["owned_actor_ids"])) == 7
            and set(manifest["destroy_rpc_confirmed_actor_ids"]) == set(manifest["owned_actor_ids"]), "not all owned actor destroy RPCs confirmed")
    pins = manifest["source_sha256"]
    require(pins == {"worker": plan["source_sha256"][WORKER], "collector_helpers": plan["source_sha256"]["scripts/e2e/collect_carla_vad_expert.py"],
                     "mapping": MAPPING_SHA, "calibration": CALIBRATION_SHA}, "worker/helper/rig source pin mismatch")
    require(pedal.canonical_sha(manifest["camera_specs"]) == RIG_SHA, "full camera specification differs from fixed six-camera rig")
    settings = manifest["world_settings"]
    require(settings["synchronous_mode"] is True and settings["fixed_delta_seconds"] == .05 and settings["no_rendering_mode"] is False
            and settings["substepping"] is True and settings["max_substep_delta_time"] * settings["max_substeps"] >= .05, "actual rendered20Hz settings invalid")
    route = pedal.read_bytes(root, "camera_audit/route.json", ledger)
    require(hashlib.sha256(route).hexdigest() == manifest["route_sha256"] == plan["route_sha256"] == started["route_sha256"] == ROUTE_SHA, "frozen route hash mismatch")
    states = read("camera_audit/states.jsonl", jsonl=True)
    validate_states(states)
    frames = read("camera_audit/camera_frames.jsonl", jsonl=True)
    images = validate_frames(frames, states, root, ledger)
    for item in ledger:
        require(base.sha(root / item["path"]) == item["sha256"], "evidence changed while auditing")
    result = {"quality": quality, "verification": "PASS", "state_count": 160, "saved_anchor_count": 4,
        "image_count": 24, "saved_ticks": list(TICKS), "maximum_post_settle_speed_mps": max(row["planar_speed_mps"] for row in states[70:]),
        "first_to_last_state_seconds": states[-1]["timestamp"] - states[0]["timestamp"], "commanded_simulation_seconds": 8.,
        "source_manifest": ledger, "source_commit": plan["source_head_commit"], "archived_source_sha256": plan["source_sha256"],
        "owner_exit_code": 0, "lifecycle": {stage: "PASS" for stage in ("ready", "after_capture", "stopped")},
        "image_evidence": images, "camera_specs": manifest["camera_specs"], "world_settings": settings,
        "control_notice": manifest["control_timing_notice"], "destroy_rpc_count": 7}
    return result, states


def verify_pair(low_root, epic_root):
    """HH_260906 - Compare the same frozen rig/ticks/pose observations without claiming pixel differences prove driving quality."""
    low, low_states = verify_run(low_root, "Low")
    epic, epic_states = verify_run(epic_root, "Epic")
    require(low["archived_source_sha256"] == epic["archived_source_sha256"] and low["source_commit"] == epic["source_commit"], "Low/Epic did not execute identical source")
    require(low["camera_specs"] == epic["camera_specs"] and low["world_settings"] == epic["world_settings"], "Low/Epic rig or physics timing differs")
    pose_differences = []
    for tick in TICKS:
        first, second = low_states[tick - 1]["actor_transform"], epic_states[tick - 1]["actor_transform"]
        position = math.sqrt(sum((first[name] - second[name]) ** 2 for name in ("x", "y", "z")))
        rotation = max(abs((first[name] - second[name] + 180.) % 360. - 180.) for name in ("roll", "pitch", "yaw"))
        require(position <= 1e-4 and rotation <= 1e-4, "same-tick Low/Epic camera poses differ materially")
        pose_differences.append({"tick": tick, "position_difference_m": position, "maximum_rotation_difference_deg": rotation})
    return {"schema": "portable_e2e.stationary_camera_quality_comparison.v1", "evidence_verification": "PASS",
        "runs": [low, epic], "total_states": 320, "total_saved_anchors": 8, "total_jpeg_images": 48,
        "same_tick_pose_differences": pose_differences, "route_sha256": ROUTE_SHA,
        "scope": {"stationary_appearance_only": True, "learned_model_evaluation": False, "training_data": False,
            "automatic_quality_promotion": False, "cropping_or_beautification": False,
            "physical_control_application_time_proven": False, "camera_frame_rate_benchmark": False},
        "limitations": ["Only Town07, one fixed spawn, one vehicle/rig/weather and four saved anchors per quality were compared.",
            "The worker drains all160 native camera bundles but saves only four anchors; discarded bundles cannot be redecoded offline.",
            "PNG conversion preserves decoded JPEG pixels, not the original pre-JPEG sensor buffer.",
            "Low/Epic differences may include materials, lighting, shadows and postprocessing; this comparison does not isolate one rendering subsystem.",
            "Local CarlaSettingsDelegate.cpp has distinct LowRoadMaterials/EpicRoadMaterials branches, but packaged-binary/source identity is not established.",
            "No road-material fix, camera FPS improvement, learned driving capability or training-data acceptance follows automatically."]}


def pilot_plan_provenance(low_root, epic_root, comparison):
    """HH_260906 - Preserve the original declaration-time error instead of backdating or certifying preregistration."""
    low_root, epic_root = Path(low_root).resolve(), Path(epic_root).resolve()
    require(low_root.parent.parent == epic_root.parent.parent, "stationary pilot arms do not share one campaign")
    campaign = low_root.parent.parent
    ledger = []
    pilot = base.read_file(campaign, "pilot_plan.json", ledger)
    low_owner = base.read_file(low_root, "owner_plan.json", [])
    epic_owner = base.read_file(epic_root, "owner_plan.json", [])
    sources = comparison["runs"][0]["archived_source_sha256"]
    require(pilot["source_commit"] == comparison["runs"][0]["source_commit"]
            and pilot["worker_sha256"] == sources[WORKER]
            and pilot["collector_sha256"] == sources["scripts/e2e/collect_carla_vad_expert.py"]
            and pilot["route_sha256"] == ROUTE_SHA and pilot["mapping_sha256"] == MAPPING_SHA
            and pilot["calibration_sha256"] == CALIBRATION_SHA, "pilot source/rig plan does not match actual runs")
    require(pilot["outputs_in_order"] == [low_root.relative_to(campaign).as_posix(), epic_root.relative_to(campaign).as_posix()]
            and pilot["qualities_in_order"] == ["Low", "Epic"] and pilot["maximum_attempts_per_quality"] == 1
            and pilot["automatic_retry"] is False, "pilot arm order or attempt budget differs")
    require(pilot["map"] == "Town07" and pilot["weather"] == "ClearNoon" and pilot["vehicle"] == "vehicle.toyota.prius"
            and pilot["physics_hz"] == 20 and pilot["native_camera_sensor_tick_seconds"] == 0
            and pilot["total_ticks_per_arm"] == 160 and pilot["saved_ticks_per_arm"] == list(TICKS)
            and pilot["camera_count"] == 6 and pilot["command"] == {"throttle": 0., "brake": 1., "steer": 0.}, "pilot measurement parameters differ")
    require(all(pilot[name] is False for name in ("training_data_approved", "learned_model_control", "physical_actuation_timing_proven", "camera_fps_benchmark")), "pilot claims leave stationary appearance scope")
    declared = datetime.fromisoformat(pilot["declared_at_utc"].replace("Z", "+00:00"))
    low_planned = datetime.fromisoformat(low_owner["planned_at_utc"])
    epic_planned = datetime.fromisoformat(epic_owner["planned_at_utc"])
    require(epic_planned > low_planned, "actual arm order contradicts the pilot plan")
    birth = subprocess.check_output(["stat", "--format=%w", str(campaign / "pilot_plan.json")], text=True).strip()
    return {"raw_path": "pilot_plan.json", "raw_sha256": ledger[0]["sha256"], "raw_record": pilot,
        "low_owner_planned_at_utc": low_owner["planned_at_utc"], "epic_owner_planned_at_utc": epic_owner["planned_at_utc"],
        "declared_timestamp_minus_low_owner_plan_seconds": (declared - low_planned).total_seconds(),
        "declared_timestamp_after_first_owner_plan": declared > low_planned,
        "filesystem_birth_time_observed": birth, "filesystem_birth_time_is_immutable_proof": False,
        "strict_preregistration_verified": False,
        "notice": "The original declared_at timestamp was rounded ahead and is later than the first owner plan. Preserve it unchanged. Current filesystem birth metadata and operator tool order are not immutable independent preregistration proof; actual image/evidence checks remain separate."}


def native_front_png(source, destination):
    """HH_260906 - Losslessly encode the already-decoded JPEG RGB pixels without resizing or enhancement."""
    from PIL import Image
    require(not destination.exists() and not destination.is_symlink(), "front PNG already exists")
    original = decode_jpeg(source)
    original.save(destination, format="PNG")
    with Image.open(destination) as decoded:
        decoded.load()
        require(decoded.mode == "RGB" and decoded.size == original.size and decoded.tobytes() == original.tobytes(), "PNG conversion altered decoded camera pixels")
    return {"jpeg_sha256": base.sha(source), "png_sha256": base.sha(destination), "decoded_rgb_sha256": pixel_sha(original), "width": 640, "height": 360}


def contact_sheet(low_root, epic_root, tick, destination):
    """HH_260906 - Place native640x360 full-FOV pixels on a labelled1920-wide canvas without interpolation."""
    from PIL import Image, ImageDraw, ImageFont
    order = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT")
    width, height, header, label_height = 640, 360, 48, 28
    sheet = Image.new("RGB", (3 * width, header + 4 * (height + label_height)), "white")
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 20)
    except OSError:
        font = ImageFont.load_default()
    draw.text((12, 12), f"Actual stationary camera comparison | tick{tick} | native pixels, full FOV, no correction", fill="black", font=font)
    cells = []
    for group in range(2):
        for quality_index, (quality, root) in enumerate((("Low", Path(low_root)), ("Epic", Path(epic_root)))):
            row = group * 2 + quality_index
            for column in range(3):
                camera = order[group * 3 + column]
                source = root / f"camera_audit/images/tick_{tick:03d}_{camera}.jpg"
                image = decode_jpeg(source)
                x, y = column * width, header + row * (height + label_height)
                draw.text((x + 8, y + 3), f"{camera} | {quality} | tick{tick}", fill="black", font=font)
                sheet.paste(image, (x, y + label_height))
                cells.append({"camera": camera, "quality": quality, "tick": tick, "x": x, "y": y + label_height,
                              "width": width, "height": height, "jpeg_sha256": base.sha(source), "decoded_rgb_sha256": pixel_sha(image)})
    require(not destination.exists() and not destination.is_symlink(), "contact sheet already exists")
    sheet.save(destination, format="PNG")
    for cell in cells:
        pixels = sheet.crop((cell["x"], cell["y"], cell["x"] + width, cell["y"] + height))
        require(pixel_sha(pixels) == cell["decoded_rgb_sha256"], "contact sheet cell differs from decoded source pixels")
    return {"path": destination.name, "sha256": base.sha(destination), "width": sheet.width, "height": sheet.height, "cells": cells}


def publish(low_root, epic_root, output):
    """HH_260906 - Publish only after both actual runs pass all evidence checks; refuse every existing target."""
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), "comparison output already exists")
    result = verify_pair(low_root, epic_root)
    result["pilot_plan_provenance"] = pilot_plan_provenance(low_root, epic_root, result)
    output.mkdir(parents=True)
    fronts, sheets = [], []
    for tick in TICKS:
        for quality, root in (("Low", Path(low_root)), ("Epic", Path(epic_root))):
            name = f"front_tick_{tick:03d}_{quality.lower()}.png"
            proof = native_front_png(root / f"camera_audit/images/tick_{tick:03d}_CAM_FRONT.jpg", output / name)
            fronts.append({"quality": quality, "tick": tick, "path": name, **proof})
        sheets.append(contact_sheet(low_root, epic_root, tick, output / f"six_camera_comparison_tick_{tick:03d}.png"))
    result["publication_notice"] = "Read-only evidence audit plus lossless conversion of decoded JPEG pixels. This is not a training dataset or an automated material-quality verdict."
    with (output / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    provenance = {"schema": "portable_e2e.stationary_camera_quality_publication.v1",
        "raw_roots": {"Low": Path(low_root).resolve().relative_to(ROOT).as_posix(), "Epic": Path(epic_root).resolve().relative_to(ROOT).as_posix()},
        "front_conversions": fronts, "contact_sheets": sheets,
        "audit_source_sha256": {str(Path(name).resolve().relative_to(ROOT)): base.sha(Path(name)) for name in (__file__, base.__file__, pedal.__file__)},
        "public_payload_sha256": {path.name: base.sha(path) for path in sorted(output.iterdir())},
        "notice": "Raw source files remain private and unchanged; RGB pixel hashes bind every native contact-sheet cell and front PNG to a captured JPEG."}
    with (output / "provenance.json").open("x", encoding="utf-8") as stream:
        json.dump(provenance, stream, indent=2, allow_nan=False)
        stream.write("\n")
    readme = """<!-- HH_260906 - Actual stationary image evidence only; do not confuse source hypotheses with a proven material fix. -->
# 정지 카메라 품질: Low / Epic 실제 화면 비교

동일한 Town07 위치·차량·6개 카메라·ClearNoon·20Hz 물리 설정으로, 브레이크1 / 스로틀0 / 조향0인 정지 상태를 각각 한 번 촬영했습니다. 두 실행 모두160개 상태 기록,4개 저장 시점과24장 JPEG, 실행 소스11개 및 서버 정리를 검증했습니다. 자율주행·학습·카메라 FPS 성능 시험은 아닙니다.

아래 비교표는 각 카메라의640×360 원본 JPEG를 디코딩한 픽셀을 **크기 변경·잘라내기·색 보정 없이** 배치했습니다. 같은 카메라의 Low/Epic 화면이 위아래로 대응합니다. 전체 화각을 유지하며 체크무늬를 지우거나 수정하지 않았습니다.

![tick100: 여섯 카메라 비교](six_camera_comparison_tick_100.png)
![tick120: 여섯 카메라 비교](six_camera_comparison_tick_120.png)
![tick140: 여섯 카메라 비교](six_camera_comparison_tick_140.png)
![tick160: 여섯 카메라 비교](six_camera_comparison_tick_160.png)

전방 카메라의 네 시점 각각에 대해 `front_tick_*_low.png` / `front_tick_*_epic.png`도 저장했습니다. 이 PNG는 JPEG 디코딩 결과를 무손실로 옮긴 파일이며 JPEG 압축 이전 센서 원본을 복원한 것은 아닙니다. PNG와 비교표의 모든 영상 칸에 대해 원본 JPEG 디코딩 픽셀 해시가 일치합니다.

[검증 결과와 원본 SHA](summary.json), [변환·비교표 픽셀 출처](provenance.json), [공개 파일 SHA256](SHA256SUMS).

해석상 주의: 로컬 CARLA 소스의 `CarlaSettingsDelegate.cpp`에는 Low/Epic의 도로 재질을 다르게 적용하는 코드가 있습니다. 그러나 실행한 패키지 바이너리가 그 소스로 빌드되었다는 증거는 없고, Low/Epic은 조명·그림자·후처리 등도 함께 바꿉니다. 따라서 이미지 차이만으로 특정 재질 코드가 원인이라고 확정하거나 문제가 해결됐다고 자동 판정하지 않습니다. 한 위치에서의 외관 비교는 전체 Town·학습 데이터·주행 성능 검증을 대신하지 않습니다.

계획 시각 오류도 보존했습니다. 원본 `pilot_plan.json`의 선언 시각19:04:00Z는 Low 실행 계획 기록19:03:53.568809Z보다6.431191초 늦습니다. 실제 작성 순서와 현재 파일 생성 시각은 실행 전 작성을 뒷받침하지만, 수정 불가능한 독립 사전 등록 증거를 대신하지는 않습니다. 선언 시각을 고쳐 쓰지 않았으며 **엄격한 사전 등록 검증은 통과로 표시하지 않습니다**. 이 한계는 원본 영상·제어·실행 정리의 사후 검증 결과와 구분합니다.

프레임100/120/140/160을 저장했으며70틱까지는 정착 구간입니다. 실행 중에는 모든160틱의6개 카메라 묶음을 받았지만, 저장하지 않은 묶음을 사후에 다시 디코딩 검증할 수는 없습니다. 제어 값은 API가 보고한 상태이고 물리 서브스텝에 실제 적용된 시점까지 입증하지 않습니다. 원본 촬영 파일은 개인 실행 산출물 폴더에 보존했으며 데이터셋에 추가하지 않았습니다.
"""
    with (output / "README.md").open("x", encoding="utf-8") as stream:
        stream.write(readme)
    with (output / "SHA256SUMS").open("x", encoding="utf-8") as stream:
        stream.write("".join(f"{base.sha(path)}  {path.name}\n" for path in sorted(output.iterdir()) if path.is_file() and path.name != "SHA256SUMS"))
    return result


def main(argv=None):
    """HH_260906 - No CARLA client, renderer process, GPU, or model is started by this evidence-only tool."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--low", type=Path, required=True)
    parser.add_argument("--epic", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = publish(args.low, args.epic, args.output)
    print(json.dumps({key: result[key] for key in ("evidence_verification", "total_states", "total_saved_anchors", "total_jpeg_images")}, indent=2))


if __name__ == "__main__":
    main()
