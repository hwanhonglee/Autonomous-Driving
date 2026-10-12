#!/usr/bin/env python3
"""HH_260929 - Freeze a post-hoc confidence audit of three repeated Town05 DEV frames."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from scripts.e2e import audit_learned_mapping_bridge as bridge


ROOT = Path(__file__).resolve().parents[2]
HARDCASE = ROOT / "docs/assets/validation/2026-09-29/map_town05_projection_hardcase/run_v1"
SOURCE = ROOT / "storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping/frames"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/map_town05_confident_errors/result.json"
HARDCASE_SUMMARY_SHA256 = "cb4e9df72409ba9c8a6d5109c2255c85a1aeefacb78680c71516fe9ecd66b59c"
SEMANTIC_MAPPER_SHA256 = "e900ac60fa5e66ec621cfd6a108d3960013d19e696ea7bc74e1e619669b8105d"
INDICES = (0, 115, 231)
SCORE_FILTER = 0.5
DESCRIPTIVE_HIGH_SCORE = 0.9


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def describe_scores(scores: np.ndarray, mask: np.ndarray) -> dict:
    """HH_260929 - Count argmax scores; depth, stride and spatial filters are excluded."""
    selected = scores[mask]
    return {
        "pixels": int(selected.size),
        "score_at_least_0_5": int(np.count_nonzero(selected >= SCORE_FILTER)),
        "score_at_least_0_9": int(np.count_nonzero(selected >= DESCRIPTIVE_HIGH_SCORE)),
        "median_score": float(np.quantile(selected, 0.5, method="linear")) if selected.size else None,
        "p90_score": float(np.quantile(selected, 0.9, method="linear")) if selected.size else None,
    }


def audit_frame(target: np.ndarray, predicted: np.ndarray, scores: np.ndarray) -> dict:
    """HH_260929 - Expose confidently wrong classes without fitting a new threshold."""
    require(target.ndim == predicted.ndim == scores.ndim == 2
            and target.shape == predicted.shape == scores.shape,
            "co-registered two-dimensional arrays required")
    require(target.dtype == predicted.dtype == np.uint8
            and np.issubdtype(scores.dtype, np.floating),
            "uint8 classes and floating confidence required")
    require(int(target.max()) < 7 and int(predicted.max()) < 7
            and np.isfinite(scores).all() and np.all((scores >= 0) & (scores <= 1)),
            "seven-class finite probability contract required")
    lane = target == 3
    return {
        "road_correct": describe_scores(scores, (target == 1) & (predicted == 1)),
        "road_as_sidewalk": describe_scores(scores, (target == 1) & (predicted == 2)),
        "road_as_pole": describe_scores(scores, (target == 1) & (predicted == 6)),
        "lane_correct": describe_scores(scores, lane & (predicted == 3)),
        "lane_missed": describe_scores(scores, lane & (predicted != 3)),
        "all_wrong": describe_scores(scores, target != predicted),
    }


def run() -> dict:
    """HH_260929 - Reuse frozen predictions only; never infer, train or rewrite evidence."""
    summary_path = HARDCASE / "summary.json"
    require(digest(summary_path) == HARDCASE_SUMMARY_SHA256, "hardcase summary changed")
    require(digest(ROOT / "tools/e2e/audit_learned_mapping_bridge.py")
            == SEMANTIC_MAPPER_SHA256, "semantic mapper changed")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    require(summary.get("indices") == list(INDICES)
            and summary.get("repeated_dev_only") is True
            and summary.get("independent_dev_count") == 0
            and summary.get("min_model_confidence") == SCORE_FILTER
            and summary.get("model_input") == "RGB only"
            and summary.get("gpu_used") is False,
            "frozen repeated-DEV contract changed")
    rows = []
    input_hashes = {}
    for index in INDICES:
        raw_name = f"frames/{index:04d}.npz"
        predicted_name = f"frame_{index:04d}_prediction.npz"
        raw_path = SOURCE / f"{index:04d}.npz"
        predicted_path = HARDCASE / predicted_name
        require(digest(raw_path) == summary["input_sha256"][raw_name],
                f"raw sensor bytes changed: {index}")
        require(digest(predicted_path) == summary["artifact_sha256"][predicted_name],
                f"prediction bytes changed: {index}")
        with np.load(raw_path, allow_pickle=False) as source:
            require(set(source.files) == {"depth", "semantic", "world_from_camera"},
                    "raw sensor keys changed")
            target = bridge.semantic_target(source["semantic"])
        with np.load(predicted_path, allow_pickle=False) as prediction:
            require(set(prediction.files) == {"predicted_class", "confidence"},
                    "saved prediction keys changed")
            predicted = prediction["predicted_class"]
            scores = prediction["confidence"]
        require(target.shape == predicted.shape == scores.shape == (360, 640),
                "saved frame shape changed")
        diagnostics = audit_frame(target, predicted, scores)
        # HH_260929 - Tie the decoded target to the already sealed full-frame confusion.
        confusion = bridge.pixel_metrics(target, predicted)["confusion"]
        require(confusion == summary["rows"][len(rows)]["pixel_metrics"]["confusion"],
                "hardcase confusion replay differs")
        rows.append({"index": index, "errors": diagnostics})
        input_hashes[raw_name] = summary["input_sha256"][raw_name]
        input_hashes[predicted_name] = summary["artifact_sha256"][predicted_name]
    result = {
        "schema": "portable_e2e.map_town05_confident_errors.v1",
        "status": "POST_HOC_REPEATED_DEV_DIAGNOSTIC_ONLY",
        "source_summary_sha256": HARDCASE_SUMMARY_SHA256,
        "semantic_mapper_sha256": SEMANTIC_MAPPER_SHA256,
        "audit_script_sha256": digest(Path(__file__)),
        "input_sha256": input_hashes,
        "definitions": {
            "score": "maximum softmax probability of the saved argmax class",
            "score_at_least_0_5": "pixel meets frozen map score filter only; depth/stride/spatial filters not applied",
            "score_at_least_0_9": "descriptive high-score count; not a selected decision threshold",
            "median_and_p90": "NumPy linear quantiles of all pixels in the named class mask",
            "lane_missed": "oracle lane_marking pixels predicted as any class other than lane_marking",
            "road_as_sidewalk": "oracle road pixels predicted as sidewalk",
        },
        "rows": rows,
        "independent_validation": False,
        "post_hoc_after_failure": True,
        "threshold_tuned": False,
        "model_trained": False,
        "model_promoted": False,
        "gpu_used": False,
    }
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "fresh output required")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": result["status"], "output": str(OUTPUT),
                      "frame_0115_lane_missed": rows[1]["errors"]["lane_missed"]}))
    return result


if __name__ == "__main__":
    run()
