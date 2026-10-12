#!/usr/bin/env python3
"""HH_260929 - Audit saved Town02 p00 road and lane labels without changing source data."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1"
SOURCE = OWNER / "mapping"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/map_alternative_train_sources/visual_qa"
AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town02_train_scout/summary.json"
FRAME_IDS = ("0000", "0120", "0239")
DIAGNOSTIC_FRAME_ID = "0039"
PINNED_SHA256 = {
    "learning/config/site_split_guard_20260929.json": "da0245f3a863b3b03dc0e364eb067354a27d36fc123d2bb860e1a4bff7a9690c",
    "docs/assets/validation/2026-09-27/map_town02_train_scout/summary.json": "8467dcf470e5e4cef2f9aee884736127989c13b19f03254d43a6448824f2ac47",
    "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1/owner_plan.json": "6cd2142f8720a54bd3e403335f2cca819771e1295231428abcfdc8a9be2f08f2",
    "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1/owner_result.json": "4d4976d466898e6853629d8065bf6b0690c2681df3555c7ff7178f1219a480fd",
    "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1/mapping/summary.json": "c90dc348da99643fcaf58ac7067437a8b14d530dac33572a36e2ec8ff2657cc0",
    "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1/mapping/records.json": "9c74b16aeafe4c4e98a7f3f95ef0d98d667e835eb8bdd28cee86d39a65be0985",
}
EXPECTED_ROUTE_SHA256 = "520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def font(size: int) -> ImageFont.ImageFont:
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def read_inputs() -> tuple[list[dict], set[str], dict[str, str]]:
    """HH_260929 - Reject changed capture evidence before reading sensor frames."""
    for rel, expected in PINNED_SHA256.items():
        path = ROOT / rel
        require(path.is_file() and not path.is_symlink() and sha256(path) == expected,
                f"pinned input differs: {rel}")
    plan = json.loads((OWNER / "owner_plan.json").read_text())
    result = json.loads((OWNER / "owner_result.json").read_text())
    summary = json.loads((SOURCE / "summary.json").read_text())
    audit = json.loads(AUDIT.read_text())
    route = Path(plan["route_path"])
    require(route.is_file() and not route.is_symlink() and route.resolve().is_relative_to(ROOT)
            and sha256(route) == EXPECTED_ROUTE_SHA256,
            "owned route bytes differ")
    require(plan.get("route_sha256") == EXPECTED_ROUTE_SHA256
            and summary.get("route_sha256") == EXPECTED_ROUTE_SHA256
            and plan.get("capture_mode") == "mapping-probe"
            and plan.get("quality") == "Epic"
            and result.get("exit_code") == 0
            and result.get("source_bytes_unchanged_and_archived") is True
            and summary.get("town") == "Town02_Opt"
            and summary.get("status") == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and summary.get("motion_mode") == "basic-agent"
            and summary.get("frame_count") == 240
            and summary.get("collision_events") == 0
            and summary.get("live_replay_exact_match") is True
            and summary.get("label_table", {}).get("1") == "road"
            and summary.get("label_table", {}).get("24") == "lane_marking",
            "owned capture provenance or label table differs")
    rows = json.loads((SOURCE / "records.json").read_text())
    retained = audit["retained_stems"]
    require(len(rows) == 240 and len(retained) == 232 and len(set(retained)) == 232,
            "raw/retained record count differs")
    require(all(row["stem"] == f"frames/{i:04d}" for i, row in enumerate(rows)),
            "raw frame ordering differs")
    require(set(retained) <= {row["stem"] for row in rows}
            and all(f"frames/{i}" in retained for i in (*FRAME_IDS, DIAGNOSTIC_FRAME_ID)),
            "fixed visual frames are not retained")
    return rows, set(retained), {"route": sha256(route), **{k: sha256(ROOT / k) for k in PINNED_SHA256}}


def yellow_paint(rgb: np.ndarray) -> np.ndarray:
    """HH_260929 - Use a deliberately coarse RGB cue as descriptive QA only."""
    r, g, b = (rgb[..., i].astype(np.int16) for i in range(3))
    mask = (r >= 150) & (g >= 80) & (b <= 30) & (r >= g + 20)
    mask[:90] = False
    return mask


def read_frame(stem: str) -> tuple[np.ndarray, np.ndarray, dict[str, str]]:
    rgb_path = SOURCE / f"{stem}.png"
    sensor_path = SOURCE / f"{stem}.npz"
    require(rgb_path.is_file() and sensor_path.is_file()
            and not rgb_path.is_symlink() and not sensor_path.is_symlink(),
            f"source pair missing or symlinked: {stem}")
    with Image.open(rgb_path) as image:
        rgb = np.asarray(image.convert("RGB")).copy()
        require(image.mode == "RGB" and image.size == (640, 360), f"RGB format differs: {stem}")
    with np.load(sensor_path, allow_pickle=False) as sensor:
        require(set(sensor.files) == {"depth", "semantic", "world_from_camera"},
                f"sensor keys differ: {stem}")
        depth, semantic, pose = (sensor[k] for k in ("depth", "semantic", "world_from_camera"))
        require(depth.shape == semantic.shape == (360, 640, 4)
                and depth.dtype == semantic.dtype == np.uint8
                and pose.shape == (4, 4) and np.isfinite(pose).all(),
                f"sensor/pose shape differs: {stem}")
        tags = semantic[..., 2].copy()
    return rgb, tags, {"rgb": sha256(rgb_path), "sensor": sha256(sensor_path)}


def render_frame(stem: str, rgb: np.ndarray, tags: np.ndarray, metrics: dict) -> Image.Image:
    """HH_260929 - Preserve every 640x360 input pixel in all three comparison panels."""
    road, lane = tags == 1, tags == 24
    oracle = np.full_like(rgb, (25, 30, 38))
    oracle[road] = (37, 125, 200)
    oracle[lane] = (255, 219, 50)
    overlay = rgb.astype(np.float32)
    overlay[road] = overlay[road] * 0.68 + np.array((37, 125, 200)) * 0.32
    overlay[lane] = overlay[lane] * 0.28 + np.array((255, 40, 160)) * 0.72
    board = Image.new("RGB", (1920, 432), (245, 247, 250))
    draw = ImageDraw.Draw(board)
    headings = ("Source RGB | full FOV", "CARLA oracle | road blue, lane yellow",
                "Source + oracle | lane magenta")
    for i, panel in enumerate((rgb, oracle, overlay.astype(np.uint8))):
        board.paste(Image.fromarray(panel), (i * 640, 42))
        draw.text((i * 640 + 12, 11), headings[i], fill=(18, 29, 45), font=font(20))
    note = (f"Town02_Opt p00 frame {stem[-4:]} | Road=1 {metrics['road_pixels']:,} px | "
            f"RoadLines=24 {metrics['lane_pixels']:,} px | ORACLE, not a model prediction")
    draw.text((12, 410), note, fill=(18, 29, 45), font=font(16))
    return board


def main() -> None:
    rows, retained, input_hashes = read_inputs()
    selected = {f"frames/{frame}" for frame in (*FRAME_IDS, DIAGNOSTIC_FRAME_ID)}
    totals = {"road_pixels": 0, "lane_pixels": 0, "lane_yellow_pixels": 0,
              "rgb_yellow_pixels": 0, "road_positive_frames": 0, "lane_positive_frames": 0}
    low_yellow_recall = []
    selected_frames = {}
    for row in rows:
        stem = row["stem"]
        rgb, tags, hashes = read_frame(stem)
        if stem not in retained:
            continue
        road, lane = tags == 1, tags == 24
        yellow = yellow_paint(rgb)
        road_count, lane_count = int(road.sum()), int(lane.sum())
        overlap, yellow_count = int((yellow & lane).sum()), int(yellow.sum())
        require(road_count > 0 and lane_count > 0, f"missing road or lane tag: {stem}")
        totals["road_pixels"] += road_count
        totals["lane_pixels"] += lane_count
        totals["lane_yellow_pixels"] += overlap
        totals["rgb_yellow_pixels"] += yellow_count
        totals["road_positive_frames"] += 1
        totals["lane_positive_frames"] += 1
        if overlap / lane_count < 0.9:
            low_yellow_recall.append(stem)
        if stem in selected:
            selected_frames[stem] = {"road_pixels": road_count, "lane_pixels": lane_count,
                                     "lane_yellow_pixels": overlap,
                                     "rgb_yellow_pixels": yellow_count,
                                     "source_sha256": hashes,
                                     "rgb": rgb, "tags": tags}
    require(len(selected_frames) == len(selected) and totals["road_positive_frames"] == 232,
            "selected frames or retained totals differ")
    require(totals["road_pixels"] == 21_432_799 and totals["lane_pixels"] == 145_835,
            "published-road/lane-source audit totals differ")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    visual_hashes = {}
    boards = []
    for frame in (*FRAME_IDS, DIAGNOSTIC_FRAME_ID):
        stem = f"frames/{frame}"
        payload = selected_frames[stem]
        board = render_frame(stem, payload["rgb"], payload["tags"], payload)
        filename = f"town02_p00_{frame}_full_fov.png"
        path = OUTPUT / filename
        board.save(path)
        visual_hashes[filename] = sha256(path)
        if frame in FRAME_IDS:
            boards.append(board)
        del payload["rgb"], payload["tags"]
    contact = Image.new("RGB", (1920, 432 * len(boards)), "white")
    for i, board in enumerate(boards):
        contact.paste(board, (0, i * 432))
    contact_path = OUTPUT / "town02_p00_start_middle_end_full_fov.png"
    contact.save(contact_path)
    visual_hashes[contact_path.name] = sha256(contact_path)
    report = {
        "schema": "portable_e2e.map_town02_p00_road_lane_visual_qa.v1",
        "status": "QA_CONDITIONAL_FIXED_VIEWS_ONLY",
        "dataset_admitted": False,
        "train_eligible": False,
        "independent_dev_approved": False,
        "fixed_frames": list(FRAME_IDS),
        "posthoc_dark_paint_diagnostic_frame": DIAGNOSTIC_FRAME_ID,
        "source_sha256": input_hashes,
        "script_sha256": sha256(Path(__file__)),
        "retained_frame_count": len(retained),
        "totals": totals,
        "yellow_rgb_heuristic": "in rows y>=90: R>=150, G>=80, B<=30, R>=G+20; descriptive only",
        "lane_tag_yellow_fraction": totals["lane_yellow_pixels"] / totals["lane_pixels"],
        "rgb_yellow_tag_fraction": totals["lane_yellow_pixels"] / totals["rgb_yellow_pixels"],
        "frames_with_lane_tag_yellow_fraction_below_0p9": low_yellow_recall,
        "selected_frame_metrics": selected_frames,
        "visual_sha256": visual_hashes,
    }
    path = OUTPUT / "result.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n")
    print(json.dumps({"status": report["status"], "retained_frame_count": len(retained),
                      "totals": totals, "low_yellow_recall_frames": len(low_yellow_recall),
                      "report_sha256": sha256(path), "visual_sha256": visual_hashes},
                     sort_keys=True))


if __name__ == "__main__":
    main()
