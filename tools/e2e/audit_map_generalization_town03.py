"""HH_260906 - Read-only source audit of two exact Town03 mapping TRAIN candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
CAPTURE_ROOT = ROOT / "storage/experiments/training/2026-09-27"
CATALOG_ROOT = ROOT / ("storage/experiments/validation/2026-09-01/"
                       "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps")
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_generalization_town03"
ROUTES = {
    "straight": {
        "owner": CAPTURE_ROOT / "map_generalization_town03_straight_p03",
        "route": CATALOG_ROOT / "town03/catalog/straight/routes/town03/straight/town03_straight_s0000_p03.json",
        "sha256": "aee6b0fd6e5a6dfa882cd172b4cb10c2e0865d2729d3233f5f121bb4be0674d4",
    },
    "right": {
        "owner": CAPTURE_ROOT / "map_generalization_town03_right_p03",
        "route": CATALOG_ROOT / "town03/catalog/turn/routes/town03/right/town03_right_s0000_p03.json",
        "sha256": "8aa4040617f733e5df11910d3ac8113307e05a53334f29bc3387eb1370e2bf3c",
    },
}
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
TAG_TO_LABEL = {1: "road", 24: "lane_marking", 2: "sidewalk", 7: "traffic_light_fixture",
                8: "traffic_sign", 6: "pole"}
COLORS = ((30, 42, 57), (122, 129, 138), (159, 133, 173), (248, 211, 70),
          (237, 93, 79), (229, 65, 185), (81, 190, 205))


def digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def option_value(argv: list[str], name: str) -> str:
    """HH_260906 - Reject missing or repeated collector options in owner provenance."""
    require(argv.count(name) == 1, f"collector option differs: {name}")
    index = argv.index(name)
    require(index + 1 < len(argv), f"collector option lacks value: {name}")
    return argv[index + 1]


def audit_owner(name: str, spec: dict, *, expected_frames: int = 240) -> dict:
    """HH_260906 - Bind source archive, route bytes, CARLA world and physical owner lifecycle."""
    owner, route_path, expected_sha = spec["owner"], spec["route"], spec["sha256"]
    source = owner / "mapping"
    require(owner.is_dir() and not owner.is_symlink() and source.is_dir() and not source.is_symlink(),
            f"{name}: missing/unsafe owned mapping capture")
    require(route_path.is_file() and not route_path.is_symlink() and digest(route_path) == expected_sha,
            f"{name}: pinned route bytes differ")
    route = read_json(route_path)
    require(route.get("town") == "Town03" and route.get("coordinate_reference") == "base_link"
            and len(route.get("route", [])) >= 2, f"{name}: route schema or town differs")
    plan_path, started_path = owner / "owner_plan.json", owner / "owner_started.json"
    result_path, summary_path = owner / "owner_result.json", source / "summary.json"
    plan, started, result, summary = (read_json(path) for path in
                                      (plan_path, started_path, result_path, summary_path))
    require(Path(plan["route_path"]).resolve() == route_path.resolve()
            and plan["route_sha256"] == started["route_sha256"] == summary["route_sha256"] == expected_sha,
            f"{name}: owner route identity differs")
    require(plan["map"] == started["map"] == summary["town"] == "Town03"
            and plan["capture_mode"] == result["capture_mode"] == "mapping-probe"
            and plan["quality"] == started["quality"] == "Epic"
            and plan.get("source_bytes_archived") is True
            and result.get("source_bytes_unchanged_and_archived") is True
            and result["exit_code"] == 0
            and summary["status"] == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and summary.get("research_split_intent") == "town03_additive_train_candidate"
            and summary["frame_count"] == expected_frames
            and summary["physics_driving"] is True and summary["motion_mode"] == "basic-agent"
            and summary.get("learned_model_control") is False
            and plan.get("learned_model_control") is False
            and result.get("learned_model_control") is False
            and summary["collision_events"] == 0
            and summary["live_replay_exact_match"] is True
            and not summary["cleanup_errors"]
            and summary["ignores_traffic_lights"] is True
            and summary["oracle_pose"] is True and summary["oracle_semantics"] is True
            and summary.get("hd_map_approved") is False
            and {int(key): value for key, value in summary["label_table"].items()} == TAG_TO_LABEL,
            f"{name}: physical mapping owner proof failed")
    argv = plan["collector_argv"]
    require(isinstance(argv, list) and len(argv) >= 2
            and Path(argv[0]).resolve() == source.resolve()
            and Path(argv[1]).resolve() == route_path.resolve()
            and option_value(argv, "--motion-mode") == "basic-agent"
            and option_value(argv, "--frames") == str(expected_frames)
            and argv.count("--ignore-traffic-lights") == 1
            and argv.count("--town03-generalization-train") == 1
            and "--town07-generalization-test" not in argv
            and "--town06-sign-scout" not in argv,
            f"{name}: collector scope differs")
    require(plan["host"] == "127.0.0.1" and option_value(argv, "--host") == "127.0.0.1"
            and int(option_value(argv, "--port")) == plan["port"] == started["port"],
            f"{name}: owner host/port differs")
    lifecycle_sha = {}
    for stage in ("ready", "after_capture", "stopped"):
        path = owner / "lifecycle" / f"{stage}.json"
        event = read_json(path)
        require(event["status"] == "PASS" and event["stage"] == stage
                and event["expected_map"] == "Town03"
                and event["owner_pid"] == started["server_pid"]
                and (stage == "stopped" or event.get("active_map_basename") == "Town03")
                and (stage != "stopped" or event.get("port_released") is True),
                f"{name}: CARLA lifecycle failed at {stage}")
        lifecycle_sha[stage] = digest(path)
    source_hashes = plan["source_sha256"]
    source_checks = result["source_checks"]
    require(bool(source_hashes) and set(source_checks) == set(source_hashes)
            and all(source_checks.values())
            and all(result.get("external_source_checks", {}).values()),
            f"{name}: source archive check incomplete")
    for relative, expected in source_hashes.items():
        subpath = Path(relative)
        archive = owner / "provenance" / subpath
        require(not subpath.is_absolute() and ".." not in subpath.parts
                and archive.is_file() and not archive.is_symlink()
                and archive.resolve().is_relative_to((owner / "provenance").resolve())
                and digest(archive) == expected,
                f"{name}: archived source differs: {relative}")
    return {
        "owner_plan_sha256": digest(plan_path), "owner_started_sha256": digest(started_path),
        "owner_result_sha256": digest(result_path), "mapping_summary_sha256": digest(summary_path),
        "lifecycle_sha256": lifecycle_sha, "archived_source_count": len(source_hashes),
        "route_sha256": expected_sha, "town": "Town03", "capture_mode": "mapping-probe",
        "research_split_intent": summary["research_split_intent"],
        "raw_frames": summary["frame_count"], "travel_m": summary["travel_m"],
        "collision_events": 0, "live_replay_exact_match": True, "cleanup_errors": [],
        "port_released_after_stop": True,
    }


def semantic_class_pixels(path: Path, *, frame_wh: tuple[int, int] = (640, 360)) -> list[int]:
    """HH_260906 - CARLA semantic labels are oracle targets; inference receives only RGB."""
    width, height = frame_wh
    with np.load(path, allow_pickle=False) as sensor:
        semantic = sensor["semantic"]
        require(semantic.shape == (height, width, 4) and semantic.dtype == np.uint8,
                "semantic source is not native BGRA tag bytes")
        tags = semantic[..., 2]
        pixels = [int(tags.size - sum(np.count_nonzero(tags == tag) for tag in TAG_TO_LABEL))]
        pixels.extend(int(np.count_nonzero(tags == tag)) for tag in TAG_TO_LABEL)
    # HH_260906 - Compute in the declared LABELS order, not dictionary/tag insertion order.
    by_name = {"other": pixels[0], **dict(zip(TAG_TO_LABEL.values(), pixels[1:]))}
    result = [by_name[label] for label in LABELS]
    require(sum(result) == width * height, "semantic class pixels do not cover native frame")
    return result


def audit_frames(source: Path, *, expected_frames: int = 240,
                 frame_wh: tuple[int, int] = (640, 360)) -> list[dict]:
    """HH_260906 - Decode every RGB pair, verify semantic shape, and hash immutable input bytes."""
    rows = read_json(source / "records.json")
    require(len(rows) == expected_frames, "mapping record count differs")
    frames = []
    previous_frame = previous_time = None
    for index, row in enumerate(rows):
        stem = f"frames/{index:04d}"
        require(row["stem"] == stem and isinstance(row["frame"], int)
                and (previous_frame is None or row["frame"] > previous_frame)
                and math.isfinite(row["timestamp_s"])
                and (previous_time is None or row["timestamp_s"] > previous_time)
                and all(math.isfinite(row[key]) for key in ("ego_x", "ego_y", "ego_yaw_deg", "speed_mps"))
                and row["speed_mps"] >= 0, "frame record order or pose differs")
        previous_frame, previous_time = row["frame"], row["timestamp_s"]
        rgb_path, sensor_path = source / (stem + ".png"), source / (stem + ".npz")
        require(rgb_path.is_file() and sensor_path.is_file()
                and not rgb_path.is_symlink() and not sensor_path.is_symlink(),
                f"missing/unsafe native frame pair: {stem}")
        with Image.open(rgb_path) as rgb:
            require(rgb.mode == "RGB" and rgb.size == frame_wh, f"invalid native RGB: {stem}")
            rgb.verify()
        class_pixels = semantic_class_pixels(sensor_path, frame_wh=frame_wh)
        frames.append({"stem": stem, "frame": row["frame"], "ego_x": row["ego_x"],
                       "ego_y": row["ego_y"], "timestamp_s": row["timestamp_s"],
                       "class_pixels": class_pixels, "rgb_sha256": digest(rgb_path),
                       "semantic_npz_sha256": digest(sensor_path)})
    require(len({row["stem"] for row in frames}) == expected_frames, "duplicate frame stems")
    return frames


def retain_indices(rows: list[dict], prior_xy: list[tuple[float, float]] | None = None,
                   minimum_m: float = 0.25) -> tuple[list[int], list[tuple[float, float]]]:
    """HH_260906 - Deduplicate against every already kept Town03 position, not another Town's XY."""
    kept_xy = list(prior_xy or [])
    kept = []
    for index, row in enumerate(rows):
        xy = (float(row["ego_x"]), float(row["ego_y"]))
        require(all(math.isfinite(value) for value in xy), "nonfinite ego XY")
        if all(math.hypot(xy[0] - old[0], xy[1] - old[1]) >= minimum_m for old in kept_xy):
            kept.append(index)
            kept_xy.append(xy)
    return kept, kept_xy


def add_pixels(rows: list[dict]) -> list[int]:
    return [sum(row["class_pixels"][index] for row in rows) for index in range(len(LABELS))]


def build_summary(specs: dict = ROUTES, *, expected_frames: int = 240,
                  frame_wh: tuple[int, int] = (640, 360), minimum_retained: int = 120) -> dict:
    """HH_260906 - Preserve both route proofs and the route-ordered global Town03 split contract."""
    require(set(specs) == {"straight", "right"}, "exact Town03 route pair required")
    routes = {}
    global_xy: list[tuple[float, float]] = []
    combined = []
    for name in ("straight", "right"):
        spec = specs[name]
        proof = audit_owner(name, spec, expected_frames=expected_frames)
        frames = audit_frames(spec["owner"] / "mapping", expected_frames=expected_frames,
                              frame_wh=frame_wh)
        within, _ = retain_indices(frames)
        kept, global_xy = retain_indices(frames, global_xy)
        retained = [frames[index] for index in kept]
        route = {
            "owner": str(spec["owner"]), "provenance": proof,
            "raw_frames": len(frames), "within_route_retained_frames_0p25m": len(within),
            "retained_frames_0p25m_global": len(kept),
            "within_route_retained_stems": [frames[index]["stem"] for index in within],
            "retained_stems": [row["stem"] for row in retained],
            "raw_class_pixels": add_pixels(frames),
            "retained_class_pixels": add_pixels(retained),
            "frames": frames,
            "retained_gate_minimum": minimum_retained,
            "retained_gate_passed": len(kept) >= minimum_retained,
        }
        routes[name] = route
        combined.extend({"route_id": name, **row} for row in retained)
    require(len({(row["route_id"], row["stem"]) for row in combined}) == len(combined),
            "combined retained records duplicate")
    passed = all(routes[name]["retained_gate_passed"] for name in ("straight", "right"))
    return {
        "schema": "portable_e2e.map_generalization_town03_audit.v1",
        "status": "QUALIFIED_TRAIN_CANDIDATES_ONLY" if passed else "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS",
        "scope": "two pinned Town03 BasicAgent mapping captures; no learned driving or model training",
        "town": "Town03", "route_order": ["straight", "right"],
        "native_frame_wh": list(frame_wh), "labels": list(LABELS),
        "retention_rule": "route-order straight then right; keep frame iff >=0.25m from every previously kept Town03 XY",
        "minimum_retained_per_route": minimum_retained,
        "routes": routes, "combined_retained_frames": len(combined),
        "combined_retained_class_pixels": add_pixels(combined),
        "combined_retained_records": combined,
        "dataset_admitted": False, "model_trained": False, "hd_map_approved": False,
        "town06_small_sign_gate_unchanged": "Town06 p06 whole-frame 1-4px tag8: 0/232, failed; not admitted here",
    }


def font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def preview(summary: dict, specs: dict = ROUTES) -> Image.Image:
    """HH_260906 - Show deterministic full-FOV TRAIN previews, never sealed Town07 images."""
    width, height = summary["native_frame_wh"]
    require((width, height) == (640, 360), "full-FOV preview requires native camera size")
    rows_to_show = []
    for name in ("straight", "right"):
        retained = summary["routes"][name]["retained_stems"]
        require(bool(retained), f"{name}: no retained preview frames")
        rows_to_show.extend((name, retained[index]) for index in (0, len(retained) // 2, len(retained) - 1))
    output = Image.new("RGB", (1300, 55 + len(rows_to_show) * 390 + 50), (247, 249, 252))
    draw = ImageDraw.Draw(output)
    draw.text((15, 12), "Town03 TRAIN candidates | RGB input vs CARLA oracle semantic label",
              fill=(25, 37, 49), font=font(22))
    for index, (name, stem) in enumerate(rows_to_show):
        source = specs[name]["owner"] / "mapping"
        with Image.open(source / (stem + ".png")) as rgb_image:
            rgb = rgb_image.convert("RGB")
        with np.load(source / (stem + ".npz"), allow_pickle=False) as sensor:
            tags = sensor["semantic"][..., 2]
            canvas = np.empty((height, width, 3), dtype=np.uint8)
            canvas[:] = COLORS[0]
            for tag, label in TAG_TO_LABEL.items():
                canvas[tags == tag] = COLORS[LABELS.index(label)]
        y = 55 + index * 390
        draw.text((15, y), f"{name} | {stem} | original 640x360; no model prediction",
                  fill=(25, 37, 49), font=font(16))
        output.paste(rgb, (10, y + 26))
        output.paste(Image.fromarray(canvas), (650, y + 26))
    draw.text((15, output.height - 38),
              "Oracle labels are targets only. BasicAgent drove; no learned control or HD-map approval.",
              fill=(66, 78, 90), font=font(16))
    return output


def write_report(summary: dict, image: Image.Image, output: Path = OUTPUT) -> None:
    require(not output.exists() and not output.is_symlink(), "fresh audit output required")
    for name in ("straight", "right"):
        require(not output.resolve().is_relative_to(ROUTES[name]["owner"].resolve()),
                "audit output must not modify original capture")
    output.mkdir(parents=True, exist_ok=False)
    image.save(output / "town03_rgb_oracle_full_fov.png")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    a, b = (summary["routes"][name] for name in ("straight", "right"))
    text = f"""# Town03 지도 TRAIN 후보 원본 감사

<!-- HH_260906 - The owned CARLA semantic source is training evidence, not a learned driving or HD-map result. -->

**결론: {summary['status']}.** 이는 두 Town03 경로의 수집 품질·중복 위치·라벨 분포를 확인한 결과이며, 지도 모델 학습이나 성능 향상을 뜻하지 않습니다. 차량은 CARLA BasicAgent가 움직였고 모델이 조향하지 않았습니다.

| 원본 640×360 | 직진 p03 | 우회전 p03 |
|---|---:|---:|
| 저장된 RGB/시맨틱 쌍 | {a['raw_frames']} | {b['raw_frames']} |
| 각 경로 자체 0.25m 중복 제거 | {a['within_route_retained_frames_0p25m']} | {b['within_route_retained_frames_0p25m']} |
| **두 경로 전체에서 보존** | **{a['retained_frames_0p25m_global']}** | **{b['retained_frames_0p25m_global']}** |
| 최종 보존 최소 {summary['minimum_retained_per_route']}장 | {'통과' if a['retained_gate_passed'] else '실패'} | {'통과' if b['retained_gate_passed'] else '실패'} |

두 경로는 같은 Town에서 위치가 겹칠 수 있으므로 순서대로(`straight` 뒤 `right`) **이전 모든 보존 XY와 0.25m 이상** 떨어진 프레임만 계산했습니다. 전체 보존 수는 **{summary['combined_retained_frames']}장**입니다. 다른 Town과 XY 거리를 계산하거나, 이 프레임을 서로 독립 TEST 사례로 세지 않습니다.

[전방 전체 RGB·CARLA 의미 라벨 PNG](town03_rgb_oracle_full_fov.png) · [파일별 SHA256·프레임 목록·클래스 픽셀 감사 JSON](summary.json)

후속 [독립 재검산된 913장 TRAIN 포장 기록](TRAIN_PACKAGE.md)은 이 감사와 별개로 원본·보존 목록·SHA를 다시 검증한 결과입니다.

각 owner의 exit 0, 원본 소스 archive 해시, 고정 route SHA/월드, CARLA lifecycle ready/after/stopped, 240장 센서 쌍 디코드, 충돌 0, live/replay 일치, cleanup 0을 검증했습니다. `traffic_sign`을 포함한 7개 클래스 픽셀 수는 **CARLA oracle 정답**으로 계산했고, 그림에 모델 예측은 없습니다. 이 감사는 원본을 변경하지 않았으며 `dataset_admitted=false`로 남습니다. 추가 TRAIN 패키지는 별도 포장기가 같은 프레임·해시·중복 규칙을 독립 재검산해야 합니다.

Town06 p06의 사전 소형 표지판 조건은 전체 프레임 1–4 px **0/232로 실패**한 채 유지됩니다. 이 Town03 감사가 그 판정을 뒤집거나 작은 표지판 검출력·자동 정밀지도·실차 준비를 입증하지 않습니다. Town07 평가 원본은 여기서 열거나 사용하지 않았습니다.
"""
    (output / "README.md").write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    result = build_summary()
    image = preview(result)
    write_report(result, image, args.output)
    print(json.dumps({"status": result["status"],
                      "combined_retained_frames": result["combined_retained_frames"],
                      "output": str(args.output)}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
