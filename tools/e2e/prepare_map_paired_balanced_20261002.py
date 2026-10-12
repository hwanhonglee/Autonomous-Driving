#!/usr/bin/env python3
"""HH_261002 - Review full-FOV paired targets, then register only a separately authorized fit."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw

from . import map_paired_balanced_contract_20261002 as contract
from . import audit_map_paired_town01_p02_p04_20261001 as qa
from . import evaluate_ctrack_fixed_maps_20261001 as frozen

ROOT = frozen.ROOT
CAMPAIGN = ROOT / "storage/experiments/validation/2026-10-01/map_paired_p02_p04_v1"
CAPTURES = {name: CAMPAIGN / name / "mapping" for name in contract.ROUTES}
QA_PATHS = {name: CAMPAIGN / "qa" / (name + ".json") for name in contract.ROUTES}
REVIEW_OUTPUT = ROOT / "docs/assets/validation/2026-10-02/map_paired_appearance_review_v1"
REVIEW_PATH = REVIEW_OUTPUT / "manual_review_and_fit_authorization.json"
PLAN = ROOT / "docs/assets/validation/2026-10-02/map_paired_balanced_v1/TRAIN_PLAN.md"
OUTPUT = ROOT / "storage/experiments/training/2026-10-02" / contract.NAME
RENDER_SCHEMA = "portable_e2e.map_paired_balanced_render_review.20261002.v1"
MIN_FREE_BYTES = 5 * 1024**3
WRITE_RESERVE_BYTES = 512 * 1024**2


def reserve(path: Path) -> None:
    contract.canonical(path)
    parent = next(item for item in (path, *path.parents) if item.exists())
    contract.require(shutil.disk_usage(parent).free >= MIN_FREE_BYTES + WRITE_RESERVE_BYTES,
                     "5 GiB free plus 512 MiB write reserve required")


def copy_new(source: Path, target: Path, expected_sha: str | None = None) -> None:
    contract.canonical(source); contract.canonical(target)
    sha = contract.digest(source)
    contract.require(expected_sha is None or sha == expected_sha, "archive source bytes changed")
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as incoming, target.open("xb") as outgoing:
        shutil.copyfileobj(incoming, outgoing, 1024 * 1024)
    contract.require(contract.digest(source) == contract.digest(target) == sha, "archival copy differs")


def capture_contract(fresh_audit: bool) -> tuple[dict, dict]:
    """HH_261002 - Native False remains False; all240 each are required before derived permission."""
    reports, pins = {}, {}
    for name, source in CAPTURES.items():
        saved = contract.read_json(QA_PATHS[name], contract.QA_SHA[name])
        contract.require(saved.get("schema") == "portable_e2e.map_paired_town01_p02_p04_qa.v1"
                         and saved.get("status") == "QA_CONDITIONAL" and saved.get("frame_count") == 240
                         and saved.get("route_sha256") == contract.ROUTES[name]
                         and saved.get("intended_role") == qa.capture.ROLE
                         and saved.get("source_scope") == name and saved.get("source") == str(source)
                         and saved.get("train_eligible") is False and saved.get("dataset_admitted") is False
                         and saved.get("independent_dev_approved") is False
                         and saved.get("all_four_sensor_sync_pose_hashes_verified") is True
                         and saved.get("source_fingerprints_unchanged") is True,
                         "full paired QA/pending source scope differs: " + name)
        if fresh_audit:
            contract.require(saved == qa.audit(source), "saved QA differs from independent full re-audit")
        for path in (QA_PATHS[name], source / "summary.json", source / "records.json",
                     source.parent / "owner_plan.json", source.parent / "owner_result.json"):
            pins[str(path)] = contract.digest(path)
        summary = contract.read_json(source / "summary.json")
        owner = contract.read_json(source.parent / "owner_result.json")
        contract.require(summary.get("dataset_admitted") is False and summary.get("train_eligible") is False
                         and owner.get("dataset_admitted") is False and owner.get("train_eligible") is False
                         and owner.get("source_bytes_unchanged_and_archived") is True
                         and owner.get("collector_cleanup_ok") is True and owner.get("server_cleanup_ok") is True
                         and owner.get("after_capture_ok") is True and owner.get("stopped_ok") is True
                         and owner.get("error") is None, "owner/raw denials or lifecycle proof differs")
        reports[name] = saved
    return reports, pins


def raw_pair(name: str, index: int, record: dict, reports: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    contract.require(record.get("stem") == f"frames/{index:04d}", "fixed physical frame membership differs")
    source = CAPTURES[name]
    path = source / (record["stem"] + ".npz")
    sha = record["artifact_sha256"][".npz"]
    contract.require(reports[name]["frame_artifact_sha256"][record["stem"] + ".npz"] == sha,
                     "raw frame not linked to exact QA")
    data = contract.read_npz(path, sha)
    for key in ("rgb_false", "rgb_true", "semantic", "depth"):
        contract.require(data[key].shape == (360, 640, 4) and data[key].dtype == np.uint8,
                         "native four-camera BGRA ABI differs")
    return data["rgb_false"][..., [2, 1, 0]].copy(), data["rgb_true"][..., [2, 1, 0]].copy(), frozen.targets(data["semantic"])


def review_canvas(name, index, false, true, target):
    """HH_261002 - Native full FOV, unchanged RGB panels, and all seven semantic colors."""
    contract.require(false.shape == true.shape == (360, 640, 3) and target.shape == (360, 640)
                     and false.dtype == true.dtype == target.dtype == np.uint8 and int(target.max()) < 7,
                     "review tensor ABI differs")
    colors = frozen.COLORS[target]
    overlays = [np.rint(.6 * rgb.astype(np.float64) + .4 * colors.astype(np.float64)).astype(np.uint8)
                for rgb in (false, true)]
    image = Image.new("RGB", (1920, 850), (18, 24, 35))
    draw = ImageDraw.Draw(image)
    draw.text((12, 8), f"{name} fixed frame {index:04d} | ACTUAL CARLA | manual review only, NOT admission", fill="white")
    for x, label in ((0, "FALSE / OFF original RGB"), (640, "TRUE / ON original RGB"),
                     (1280, "Shared seven-class semantic GT (not model input)")):
        draw.text((x + 12, 32), label, fill="white")
    for x, pixels in ((0, false), (640, true), (1280, colors)):
        image.paste(Image.fromarray(pixels), (x, 56))
    for x, label in ((0, "FALSE + shared target, alpha .4"), (640, "TRUE + same target, alpha .4"),
                     (1280, "No prediction, no resizing/cropping, no omitted classes")):
        draw.text((x + 12, 424), label, fill="white")
    for x, pixels in ((0, overlays[0]), (640, overlays[1]), (1280, colors)):
        image.paste(Image.fromarray(pixels), (x, 448))
    for i, (label, color) in enumerate(zip(contract.LABELS, frozen.COLORS)):
        x = i * 270 + 12
        draw.rectangle((x, 823, x + 10, 833), fill=tuple(int(v) for v in color))
        draw.text((x + 15, 821), label, fill="white")
    return image


def recheck(pins: dict) -> None:
    for filename, sha in pins.items():
        contract.require(contract.digest(Path(filename)) == sha, "first-read input changed: " + filename)


def render_review(output: Path = REVIEW_OUTPUT) -> dict:
    contract.canonical(output)
    contract.require(output == REVIEW_OUTPUT and not output.exists(), "fresh canonical manual review output required")
    reserve(output)
    reports, pins = capture_contract(False)
    renderer_sources = {str(path): contract.digest(path) for path in
                        (Path(__file__).resolve(), Path(contract.__file__).resolve(),
                         Path(frozen.__file__).resolve(), Path(qa.__file__).resolve())}
    panels, raw_pins = {}, {}
    for name in contract.ROUTES:
        rows = contract.read_json(CAPTURES[name] / "records.json", pins[str(CAPTURES[name] / "records.json")])
        contract.require(len(rows) == 240, "all240 physical records required")
        for index in (0, 119, 239):
            false, true, target = raw_pair(name, index, rows[index], reports)
            raw_pins[str(CAPTURES[name] / (rows[index]["stem"] + ".npz"))] = rows[index]["artifact_sha256"][".npz"]
            panels[f"{name}_{index:04d}.png"] = review_canvas(name, index, false, true, target)
    pins.update(raw_pins)
    recheck(pins)
    reserve(output)
    output.mkdir(parents=True)
    archived_sources = {}
    for filename, sha in renderer_sources.items():
        path = Path(filename)
        target = output / "source" / path.name
        copy_new(path, target, sha)
        archived_sources[str(target.relative_to(output))] = {"original_path": filename, "sha256": sha}
    files = {}
    for filename, panel in panels.items():
        with (output / filename).open("xb") as stream:
            panel.save(stream, format="PNG")
        files[filename] = contract.digest(output / filename)
    manifest = {"schema": RENDER_SCHEMA, "study_id": contract.NAME,
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "frame_indices_per_route": [0, 119, 239], "rgb_conditions": ["false", "true"],
                "all_240_per_route_retained": True, "overlay_alpha": .4,
                "semantic_palette_rgb": frozen.COLORS.tolist(), "labels": list(contract.LABELS),
                "source_route_sha256": contract.ROUTES, "source_qa_sha256": contract.QA_SHA,
                "source_pins": pins, "archived_renderer_sources": archived_sources, "files": files,
                "manual_review_completed": False, "research_fit_authorized": False,
                "dataset_admitted": False, "gpu_used": False, "model_predictions_made": False}
    recheck(pins)
    contract.write_new(output / "render_manifest.json", manifest)
    with (output / "README.md").open("x") as stream:
        stream.write("# p02·p04 원본/정답 검토\n\n각 경로의 고정 0·119·239 프레임입니다. 위쪽은 실제 OFF/ON RGB와 동일한 7개 분류 정답, 아래쪽은 정답을 40%로 겹친 화면입니다.\n\n")
        stream.write("학습·모델 예측·데이터 승인은 하지 않았습니다. 전체 240 프레임은 그대로 남아 있고, 이 화면은 수동 검토용입니다. 가드레일 등 7개 분류로 표현되지 않는 세부 유형은 `other` 등 기존 분류 한계를 유지합니다.\n\n")
        for filename in files:
            stream.write(f"![{filename}]({filename})\n\n")
    return {"status": "FIXED_NATIVE_REVIEW_RENDERED_NOT_APPROVED", "output": str(output),
            "render_manifest_sha256": contract.digest(output / "render_manifest.json"), "png_count": 6,
            "research_fit_authorized": False, "gpu_used": False}


def render_contract(expected_sha: str) -> dict:
    report = contract.read_json(REVIEW_OUTPUT / "render_manifest.json", expected_sha)
    contract.require(report.get("schema") == RENDER_SCHEMA and report.get("study_id") == contract.NAME
                     and report.get("frame_indices_per_route") == [0, 119, 239]
                     and report.get("rgb_conditions") == ["false", "true"]
                     and report.get("source_route_sha256") == contract.ROUTES
                     and report.get("source_qa_sha256") == contract.QA_SHA
                     and report.get("manual_review_completed") is False
                     and report.get("research_fit_authorized") is False
                     and report.get("model_predictions_made") is False, "unreviewed native renderer receipt differs")
    expected_files = {f"{name}_{index:04d}.png" for name in contract.ROUTES for index in (0, 119, 239)}
    contract.require(set(report.get("files", {})) == expected_files, "fixed render panel membership differs")
    recheck(report["source_pins"])
    for filename, sha in report["files"].items():
        contract.require(contract.digest(REVIEW_OUTPUT / filename) == sha, "review image bytes changed")
    for filename, ref in report["archived_renderer_sources"].items():
        contract.require(Path(filename).parts[0] == "source" and ".." not in Path(filename).parts
                         and contract.digest(REVIEW_OUTPUT / filename) == ref["sha256"], "renderer source archive differs")
    # HH_261002 - The rendered builder snapshot is immutable; later new build-branch code is not retroactive rendering.
    return report


def prepare(expected_review_sha: str, expected_plan_sha: str, execute: bool) -> dict:
    contract.canonical(OUTPUT)
    contract.require(not OUTPUT.exists(), "fresh package required; no repair or overwrite")
    reserve(OUTPUT)
    review = contract.read_json(REVIEW_PATH, expected_review_sha)
    contract.payload(PLAN, expected_plan_sha)
    contract.review_contract(review, review.get("render_manifest_sha256"), expected_plan_sha, CAPTURES)
    render = render_contract(review["render_manifest_sha256"])
    reports, pins = capture_contract(True)
    pins.update(frozen.model_source_contract())
    original, parent = frozen.DATA / "train_b.npz", frozen.RUN / "B/weights.npz"
    contract.require(contract.digest(original) == contract.ORIGINAL_SHA
                     and contract.digest(parent) == contract.PARENT_SHA
                     and contract.digest(frozen.RUN / "B/checkpoint.pt") == contract.PARENT_CHECKPOINT_SHA,
                     "exact original729/B parent lineage required")
    pins.update({str(original): contract.ORIGINAL_SHA, str(parent): contract.PARENT_SHA,
                 str(frozen.RUN / "B/checkpoint.pt"): contract.PARENT_CHECKPOINT_SHA,
                 str(REVIEW_PATH): expected_review_sha, str(PLAN): expected_plan_sha,
                 str(REVIEW_OUTPUT / "render_manifest.json"): review["render_manifest_sha256"]})
    source_files = {"train_map_paired_balanced_20261002.py": ROOT / "tools/e2e/train_map_paired_balanced_20261002.py",
                    "map_paired_balanced_contract_20261002.py": Path(contract.__file__).resolve(), "plan.md": PLAN}
    for name in contract.FROZEN_SOURCE_SHA:
        source_files[name] = (ROOT / "tools/e2e" / name if name in
                             ("map_appearance_matched_contract_20261001.py", "train_map_appearance_matched_20261001.py")
                             else frozen.RUN / "source" / name)
        contract.require(contract.digest(source_files[name]) == contract.FROZEN_SOURCE_SHA[name],
                         "frozen training dependency differs: " + name)
    contract.require(set(source_files) == contract.SOURCE_FILES, "training source closure differs")
    source_pins = {name: contract.digest(path) for name, path in source_files.items()}
    pins.update({str(path): source_pins[name] for name, path in source_files.items()})
    pins[str(Path(__file__).resolve())] = contract.digest(Path(__file__).resolve())
    if not execute:
        recheck(pins)
        return {"status": "QA_MANUAL_AND_FIT_PERMISSION_PREFLIGHT_PASS", "study_id": contract.NAME,
                "counts": contract.COUNTS, "registration_created": False, "gpu_used": False}
    recheck(pins)
    reserve(OUTPUT)
    # HH_261002 - No derived TRAIN registration or output exists before both QA and real reviewer permission.
    OUTPUT.mkdir(parents=True)
    copy_new(original, OUTPUT / "original_train729.npz", contract.ORIGINAL_SHA)
    copy_new(parent, OUTPUT / "parent_weights.npz", contract.PARENT_SHA)
    false = np.empty((480, 360, 640, 3), dtype=np.uint8)
    true = np.empty_like(false)
    targets = np.empty((480, 360, 640), dtype=np.uint8)
    rows, captures = [], {}
    for route_index, (name, source) in enumerate(CAPTURES.items()):
        records = contract.read_json(source / "records.json", pins[str(source / "records.json")])
        contract.require(len(records) == 240, "no per-case sample filtering allowed")
        for index, record in enumerate(records):
            position = route_index * 240 + index
            false[position], true[position], targets[position] = raw_pair(name, index, record, reports)
            raw_path = source / (record["stem"] + ".npz")
            rows.append({"sample_id": f"{name}:{record['stem']}", "source_scope": name, "capture_index": index,
                         "carla_frame": record["frame"], "timestamp_s": record["timestamp_s"],
                         "raw_path": str(raw_path), "raw_sha256": record["artifact_sha256"][".npz"],
                         "route_sha256": contract.ROUTES[name], "qa_sha256": contract.QA_SHA[name],
                         "rgb_false_sha256": hashlib.sha256(false[position].tobytes()).hexdigest(),
                         "rgb_true_sha256": hashlib.sha256(true[position].tobytes()).hexdigest(),
                         "shared_target_sha256": hashlib.sha256(targets[position].tobytes()).hexdigest()})
        plan = contract.read_json(source.parent / "owner_plan.json", pins[str(source.parent / "owner_plan.json")])
        for filename in ("summary.json", "records.json"):
            copy_new(source / filename, OUTPUT / "provenance" / name / filename, pins[str(source / filename)])
        for filename in ("owner_plan.json", "owner_result.json"):
            copy_new(source.parent / filename, OUTPUT / "provenance" / name / filename,
                     pins[str(source.parent / filename)])
        copy_new(QA_PATHS[name], OUTPUT / "provenance" / name / "raw_qa.json", contract.QA_SHA[name])
        route = plan["route_archive"]
        copy_new(source.parent / route["archived_path"], OUTPUT / "provenance" / name / "route.json", route["sha256"])
        for logical, sha in plan["source_sha256"].items():
            copy_new(source.parent / "provenance" / logical,
                     OUTPUT / "provenance" / name / "capture_source" / logical, sha)
        for logical, ref in plan["external_source_provenance"].items():
            copy_new(source.parent / ref["archived_path"], OUTPUT / "provenance" / name / "external_source" / logical, ref["sha256"])
        captures[name] = {"source_capture": str(source), "qa_sha256": contract.QA_SHA[name],
                          "route_sha256": contract.ROUTES[name], "owner_plan_sha256": pins[str(source.parent / "owner_plan.json")],
                          "owner_result_sha256": pins[str(source.parent / "owner_result.json")],
                          "raw_source_dataset_admitted": False, "raw_source_train_eligible": False,
                          "all_240_retained": True, "independent_new_site": False}
        for filename, sha in reports[name]["frame_artifact_sha256"].items():
            contract.require(contract.digest(source / filename) == sha, "captured bytes changed during packaging")
    reserve(OUTPUT)
    with (OUTPUT / "paired_train480.npz").open("xb") as stream:
        np.savez_compressed(stream, rgb_false=false, rgb_true=true, target=targets)
    data = contract.read_npz(original, contract.ORIGINAL_SHA)
    contract.require(set(data) == {"rgb", "target"} and data["rgb"].shape == (729, 360, 640, 3)
                     and data["target"].shape == (729, 360, 640)
                     and data["rgb"].dtype == data["target"].dtype == np.uint8 and int(data["target"].max()) < 7,
                     "original729 ABI differs")
    pixels = np.bincount(data["target"].ravel(), minlength=7) + np.bincount(targets.ravel(), minlength=7)
    del data, false, true, targets
    indices, crops, bits = contract.schedule()
    schedule = contract.schedule_proof(indices, crops, bits)
    reserve(OUTPUT)
    with (OUTPUT / "training_schedule.npz").open("xb") as stream:
        np.savez_compressed(stream, indices=indices, crops=crops, balanced_view_bits=bits)
    files = {name: {"sha256": contract.digest(OUTPUT / name), "bytes": (OUTPUT / name).stat().st_size}
             for name in ("original_train729.npz", "paired_train480.npz", "training_schedule.npz", "parent_weights.npz")}
    copy_new(REVIEW_PATH, OUTPUT / "provenance/manual_review_and_fit_authorization.json", expected_review_sha)
    copy_new(REVIEW_OUTPUT / "render_manifest.json", OUTPUT / "provenance/render_manifest.json", review["render_manifest_sha256"])
    for filename, sha in render["files"].items():
        copy_new(REVIEW_OUTPUT / filename, OUTPUT / "provenance/render" / filename, sha)
    for filename, ref in render["archived_renderer_sources"].items():
        copy_new(REVIEW_OUTPUT / filename, OUTPUT / "provenance/render" / filename, ref["sha256"])
    copy_new(frozen.DATA / "manifest.json", OUTPUT / "provenance/original_train_manifest.json", frozen.DATA_SHA)
    recheck(pins)
    registry = {"schema": contract.REGISTRY_SCHEMA, "study_id": contract.NAME,
                "registered_at_utc": datetime.now(timezone.utc).isoformat(),
                "research_fit_authorized": True, "authorization_source_sha256": expected_review_sha,
                "training_plan_sha256": expected_plan_sha, "render_manifest_sha256": review["render_manifest_sha256"],
                "counts": contract.COUNTS, "arms": contract.ARMS, "optimization": contract.optimization(),
                "common_schedule": schedule, "data_files": files, "route_sha256": contract.ROUTES,
                "parent_weights_sha256": contract.PARENT_SHA, "parent_checkpoint_sha256": contract.PARENT_CHECKPOINT_SHA,
                "native_diagnostic_denials_preserved": True, "broad_registry_modified": False,
                "ctrack_train_eligible": False, "dev_or_test_used_for_fit": False,
                "vehicle_control_approved": False, "independent_new_dev_sites": 0,
                "scope": "this fixed research fit only; not broad dataset admission or vehicle/map approval",
                "source_captures": captures, "first_read_input_pins": pins,
                "copied_provenance_files": {str(path.relative_to(OUTPUT)): contract.digest(path)
                                             for path in sorted((OUTPUT / "provenance").rglob("*")) if path.is_file()},
                "known_evaluation_only": "Town01 DEV118/Town02p02 holdout232 repeatedly exposed; not independent certification"}
    contract.write_new(OUTPUT / "study_train_registry.json", registry)
    contract.write_new(OUTPUT / "paired_samples.json", rows)
    manifest = {"schema": contract.DATA_SCHEMA, "study_id": contract.NAME, "labels": list(contract.LABELS),
                "counts": contract.COUNTS, "arms": contract.ARMS, "route_sha256": contract.ROUTES,
                "parent_weights_sha256": contract.PARENT_SHA, "files": files,
                "paired_sample_ids": [row["sample_id"] for row in rows],
                "paired_samples_sha256": contract.digest(OUTPUT / "paired_samples.json"),
                "common_class_pixels": pixels.tolist(), "common_class_weights": contract.common_weights(pixels).tolist(),
                "class_counting": "original729 + new480 physical targets counted once, not twice per view",
                "schedule_contract": schedule, "study_registry_sha256": contract.digest(OUTPUT / "study_train_registry.json"),
                "raw_model_inputs": ["RGB"], "depth_or_pose_model_input": False, "source_receipts_modified": False,
                "native_diagnostic_denials_preserved": True, "ctrack_train_eligible": False,
                "dev_or_test_used_for_fit": False, "vehicle_control_approved": False, "independent_new_dev_sites": 0,
                "builder_sha256": pins[str(Path(__file__).resolve())], "original_train_manifest_sha256": frozen.DATA_SHA}
    contract.validate_manifest(manifest, registry)
    contract.write_new(OUTPUT / "manifest.json", manifest)
    for name, path in source_files.items():
        copy_new(path, OUTPUT / "source" / name, source_pins[name])
    copy_new(Path(__file__).resolve(), OUTPUT / "provenance/builder_sources" / Path(__file__).name,
             pins[str(Path(__file__).resolve())])
    bundle = {"schema": contract.SOURCE_SCHEMA, "study_id": contract.NAME, "files": source_pins,
              "data_manifest_sha256": contract.digest(OUTPUT / "manifest.json"),
              "study_registry_sha256": manifest["study_registry_sha256"], "training_plan_sha256": expected_plan_sha}
    contract.write_new(OUTPUT / "source/bundle.json", bundle)
    recheck(pins)
    result = {"status": "MATCHED_RESEARCH_PACKAGE_READY_NOT_FITTED", "output": str(OUTPUT), "counts": contract.COUNTS,
              "manifest_sha256": bundle["data_manifest_sha256"], "registry_sha256": bundle["study_registry_sha256"],
              "source_bundle_sha256": contract.digest(OUTPUT / "source/bundle.json"),
              "source_diagnostic_receipts_preserved": True, "gpu_used": False, "independent_new_dev_sites": 0}
    contract.write_new(OUTPUT / "build_result.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("mode", choices=("render-review", "validate", "build"))
    parser.add_argument("--expected-review-sha256")
    parser.add_argument("--expected-plan-sha256")
    args = parser.parse_args(argv)
    if args.mode == "render-review":
        result = render_review()
    else:
        contract.require(args.expected_review_sha256 and args.expected_plan_sha256,
                         "externally pinned review and prospective plan required")
        result = prepare(args.expected_review_sha256, args.expected_plan_sha256, args.mode == "build")
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
