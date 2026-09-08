#!/usr/bin/env python3
"""HH_260906 - Publish only single-log nuPlan metadata and source evidence, never DB, image or calibration payloads."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
PRIVATE = ROOT / "artifacts/training/2026-09-09/nuplan_single_db_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/05_real_data_readiness"
REPORT_SHA = "0b9a2a3de73c5e96af869307873a235c8279b4eaeaf6f328950973c3c97ef83e"
PLAN_SHA = "cabf374a3eb5c73fe46a01cb306358e3f8ca7a16aec2dbc5bf2bb23090e0fd22"
SOURCE_SHA = "8285ac1b1bdf9f1f78ee01fbbbaa265a1c341d8662c7c605beeddbe886a72b50"


def sha(raw): return hashlib.sha256(raw).hexdigest()


def checked(path, pins, expected=None):
    if not path.is_file() or any(p.is_symlink() for p in (path, *path.parents)):
        raise ValueError("source must be a regular nonsymlink file")
    raw = path.read_bytes(); digest = sha(raw)
    if expected is not None and digest != expected: raise ValueError("pinned source mismatch")
    if path in pins and pins[path] != digest: raise ValueError("publication input changed")
    pins[path] = digest
    return raw


def write(path, raw):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream: stream.write(raw)


def main():
    if OUTPUT.exists(): raise ValueError("refuse existing public category")
    pins = {}; source = checked(Path(__file__).resolve(), pins)
    raw_report = checked(PRIVATE / "remote/report.json", pins, REPORT_SHA)
    report = json.loads(raw_report)
    checked(PRIVATE / "remote/selection_plan.json", pins, PLAN_SHA)
    checked(PRIVATE / "selection_plan.json", pins, PLAN_SHA)
    checked(PRIVATE / "source_executed/inspect_nuplan_single_db.py", pins, SOURCE_SHA)
    checksums = checked(PRIVATE / "remote/SHA256SUMS", pins)
    names = set()
    for line in checksums.decode().splitlines():
        digest, name = line.split("  ", 1)
        if name not in {"report.json", "selection_plan.json"} or name in names: raise ValueError("unexpected original checksum entry")
        names.add(name); checked(PRIVATE / "remote" / name, pins, digest)
    if names != {"report.json", "selection_plan.json"}: raise ValueError("incomplete original checksum manifest")
    if report["status"] != "METADATA_INSPECTED_NOT_READY" or report["source_sha256"] != SOURCE_SHA or report["selection_plan_sha256"] != PLAN_SHA:
        raise ValueError("actual single-DB source/plan/status mismatch")
    for key in ("training_data_approved", "terms_consent_provided", "dataset_converted", "trained_model", "dataset_exported", "extracted", "pickle_or_orm_imported"):
        if report[key] is not False: raise ValueError("metadata evidence overclaims scope")
    if report["database_payloads_opened"] != 1 or report["camera_payloads_read"] != 0 or report["map_payloads_read"] != 0 or report["archive_identity_unchanged"] is not True:
        raise ValueError("payload denominator/immutability changed")
    measurements = report["metadata"]
    if measurements["counts"] != {"camera": 8, "image": 30080, "ego_pose": 37800, "lidar_pc": 7520, "scene": 20, "log": 1}:
        raise ValueError("actual native denominator changed")
    if sum(item["goal_pose_join_available"] for item in measurements["scene_route_goal_metadata"]) != 8:
        raise ValueError("goal availability denominator changed")
    sources = [
        (PRIVATE / "README.public.md", "README.md"),
        (PRIVATE / "remote/report.json", "original_execution/report.json"),
        (PRIVATE / "remote/selection_plan.json", "original_execution/selection_plan.json"),
        (PRIVATE / "remote/SHA256SUMS", "original_execution/SHA256SUMS"),
        (PRIVATE / "source_executed/inspect_nuplan_single_db.py", "source_executed/inspect_nuplan_single_db.py"),
        (PRIVATE / "execution_and_hardening.json", "execution_and_hardening.json"),
    ]
    OUTPUT.mkdir(parents=True, exist_ok=False)
    published = []
    for original, name in sources:
        raw = checked(original, pins); write(OUTPUT / name, raw)
        published.append({"published_path": name, "source_path": original.relative_to(ROOT).as_posix(), "source_sha256": sha(raw), "size_bytes": len(raw), "representation": "original_bytes"})
    write(OUTPUT / "source/publish_nuplan_single_db_metadata.py", source)
    alignment = measurements["camera_alignment"]
    channels = sorted(alignment["per_channel_signed_offset_us"], key=lambda c: alignment["per_channel_signed_offset_us"][c]["offset"]["p50"])
    stats = [alignment["per_channel_signed_offset_us"][c]["offset"] for c in channels]
    median = [item["p50"] / 1000 for item in stats]
    low = [(item["p50"] - item["minimum"]) / 1000 for item in stats]
    high = [(item["maximum"] - item["p50"]) / 1000 for item in stats]
    figure, axis = plt.subplots(figsize=(12, 6.75))
    axis.errorbar(median, list(range(8)), xerr=[low, high], fmt="o", color="#245fa1", capsize=5, markersize=8)
    for index, value in enumerate(median): axis.text(value + .65, index, f"{value:+.3f} ms", va="center", fontsize=11)
    axis.set_yticks(range(8), channels); axis.invert_yaxis(); axis.axvline(0, color="#777", linewidth=1)
    axis.set_xlim(-12, 42); axis.set_xlabel("Native timestamp minus nearest matched CAM_F0 timestamp (ms)")
    axis.grid(alpha=.25)
    axis.set_title("One log: staggered native camera observations", fontsize=18)
    figure.text(.5, .075, "Points: median; bars: observed min/max over all 3,760 CAM_F0 anchors. Native timestamps are unchanged.", ha="center", fontsize=10)
    figure.text(.5, .035, "All-eight bundle span: 42.328–42.473 ms. Six-camera mapping/calibration and causal route availability remain UNVERIFIED.", ha="center", fontsize=10)
    figure.tight_layout(rect=(0, .12, 1, .98))
    with (OUTPUT / "01_native_camera_timestamp_offsets.png").open("xb") as stream: figure.savefig(stream, format="png", dpi=100)
    plt.close(figure)
    for path, digest in list(pins.items()): checked(path, pins, digest)
    for path in OUTPUT.rglob("*"):
        if path.is_file() and path.suffix in (".json", ".md"):
            text = path.read_text()
            # HH_260906 - Exclude dot-delimited nuPlan date/log identifiers from standalone IPv4 detection; account paths remain rejected.
            if re.search(r"/home/[^/]+|/root/|(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])", text): raise ValueError("private path/IP found")
    manifest = {"schema": "nuplan.single_db_metadata_publication.v1", "comment": "HH_260906 - Preserve exact original metadata/source bytes and distinguish later parser hardening from the single executed DB read.",
        "published_at_utc": datetime.now(timezone.utc).isoformat(), "report_sha256": REPORT_SHA, "selection_plan_sha256": PLAN_SHA,
        "executed_worker_sha256": SOURCE_SHA, "publication_script_sha256": sha(source), "matplotlib_version": matplotlib.__version__,
        "raw_blob_or_database_exported": False, "image_payload_exported": False, "native_timestamps_changed": False, "dataset_admission": False,
        "inputs_unchanged_before_after": True, "source_files": published,
        "files": [{"path": p.relative_to(OUTPUT).as_posix(), "sha256": sha(p.read_bytes()), "size_bytes": p.stat().st_size} for p in sorted(OUTPUT.rglob("*")) if p.is_file()]}
    write(OUTPUT / "publication_manifest.json", (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode())
    payloads = sorted(p for p in OUTPUT.rglob("*") if p.is_file())
    write(OUTPUT / "SHA256SUMS", "".join(sha(p.read_bytes()) + "  " + p.relative_to(OUTPUT).as_posix() + "\n" for p in payloads).encode())
    print(json.dumps({"status": "PUBLISHED_METADATA_NOT_READY", "files": len(payloads) + 1, "report_sha256": REPORT_SHA}))


if __name__ == "__main__": main()
