#!/usr/bin/env python3
"""HH_260929 - Quantify read-only RGB appearance shift in frozen map labels."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
MANIFEST_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
GROUPS = (
    ("Town01_train", "train_d0", 0, 497),
    ("Town02_Opt_train", "train_d1", 497, 729),
    ("Town05_Opt_repeated_dev", "dev_town05", 0, 232),
)
LABELS = (("road", 1), ("lane_marking", 3))
EDGES = (0, 32, 64, 96, 128, 192, 256)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    """HH_260929 - Verify archived input bytes without following an alias."""
    require(path.is_file() and not path.is_symlink(), f"regular input required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def accumulate(rgb: np.ndarray, target: np.ndarray, label_index: int,
               start: int, stop: int) -> dict:
    """HH_260929 - Count exact class-conditioned colors in bounded frame chunks."""
    require(rgb.shape[0] == target.shape[0] and rgb.shape[1:] == (360, 640, 3)
            and target.shape[1:] == (360, 640) and rgb.dtype == target.dtype == np.uint8
            and 0 <= start < stop <= len(rgb), "RGB/label ABI or slice changed")
    count = 0
    color_sum = np.zeros(3, dtype=np.int64)
    bins = np.zeros(len(EDGES) - 1, dtype=np.int64)
    for begin in range(start, stop, 8):
        end = min(begin + 8, stop)
        selected = rgb[begin:end][target[begin:end] == label_index].astype(np.int32)
        count += len(selected)
        color_sum += selected.sum(axis=0, dtype=np.int64)
        luma = (77 * selected[:, 0] + 150 * selected[:, 1]
                + 29 * selected[:, 2]) // 256
        bins += np.histogram(luma, bins=EDGES)[0]
    require(count > 0 and bins.sum() == count, "class absent or histogram lost pixels")
    return {"pixels": int(count),
            "rgb_mean": [round(float(value / count), 3) for value in color_sum],
            "luma_bins_0_31_32_63_64_95_96_127_128_191_192_255": bins.tolist(),
            "luma_0_63_fraction": round(float((bins[0] + bins[1]) / count), 6),
            "luma_128_255_fraction": round(float((bins[4] + bins[5]) / count), 6)}


def audit() -> dict:
    """HH_260929 - Keep repeated DEV pixels diagnostic-only, never a training split."""
    require(digest(DATA / "manifest.json") == MANIFEST_SHA256,
            "frozen map package changed")
    manifest = json.loads((DATA / "manifest.json").read_text(encoding="utf-8"))
    require(manifest.get("schema") == "portable_e2e.map_road_lane_ab_data.v1"
            and manifest.get("town05_split") == "DEV_ONLY"
            and manifest.get("split_contract", {}).get("test_used") is False,
            "map split declaration changed")
    groups = {}
    for name, file_key, begin, end in GROUPS:
        file = manifest["files"][file_key]
        path = DATA / file["file"]
        require(file["sha256"] == digest(path) and end <= file["count"],
                f"frozen {file_key} array changed")
        with np.load(path, allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"}, "array keys changed")
            rgb, target = archive["rgb"], archive["target"]
        groups[name] = {"frames": end - begin, "source_file": file["file"],
                        "source_sha256": file["sha256"],
                        "classes": {label: accumulate(rgb, target, index, begin, end)
                                    for label, index in LABELS}}
    return {"schema": "portable_e2e.map_appearance_diagnostic.v1",
            "data_manifest_sha256": MANIFEST_SHA256,
            "luma_edges": list(EDGES), "groups": groups,
            "dev_reused_for_diagnosis_only": True,
            "dev_used_for_training": False, "test_used": False,
            "new_model_training": False, "causal_claim": False,
            "model_promotion": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink()
            and args.output.parent.is_dir(), "fresh output path required")
    report = audit()
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status": "DIAGNOSTIC_COMPLETE", "output": str(args.output),
                      "frames": {name: row["frames"]
                                 for name, row in report["groups"].items()}},
                     sort_keys=True))


if __name__ == "__main__":
    main()
