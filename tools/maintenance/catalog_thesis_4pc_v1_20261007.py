"""HH_261007 - Catalog thesis evidence without moving, replaying, hashing large bags or deleting originals."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/maintenance/2026-10-07/thesis_4pc_catalog_v1"
SOURCES = {
    "published_four_pc": ROOT / "docs/vils/result",
    "shared_history": ROOT / "docs/vils",
    "pc4_documents": Path("/home/a/carla-autoware-universe/autoware/docs/vils"),
    "pc4_original_runs": Path("/home/a/carla-autoware-universe/autoware/pc4_vils_runs"),
    "historical_2025": Path("/home/a/VILS_Test"),
    "old_document_clone": Path("/home/a/Autonomous-Driving-vils-docs/docs/vils"),
}
SIZE_TARGETS = {
    "portable_workspace": ROOT,
    "portable_experiments": ROOT / "storage/experiments",
    "autoware_logs": ROOT / "autonomy/log",
    "autoware_build": ROOT / "autonomy/build",
    "published_four_pc": ROOT / "docs/vils/result",
    "pc4_original_runs": SOURCES["pc4_original_runs"],
    "historical_2025": SOURCES["historical_2025"],
    "old_document_clone": Path("/home/a/Autonomous-Driving-vils-docs"),
    "unreal_engine": Path("/home/a/UnrealEngine_4.26"),
    "carla_source": Path("/home/a/carla"),
    "carla_autoware_workspace": Path("/home/a/carla-autoware-universe"),
    "ros_directory": Path("/home/a/.ros"),
    "ccache": Path("/var/tmp/ccache"),
}
EXCLUDED_DIRECTORIES = {".git", ".ssh", ".codex", "__pycache__"}
MAX_OUTPUT_BYTES = 8 * 1024**2


def generation(value):
    return [value.st_dev, value.st_ino, value.st_mode, value.st_size, value.st_mtime_ns, value.st_ctime_ns]


def inventory(folder):
    # HH_261007 - Report links and races without following them or silently treating unavailable evidence as empty.
    folder = Path(folder)
    result = dict(root=str(folder), exists=folder.exists(), files=[], symlinks=[], special=[], errors=[])
    if not folder.is_dir() or folder.is_symlink():
        result["errors"].append("missing, non-directory or symbolic root")
        return result
    def error(value):
        result["errors"].append(str(value))
    for parent, directories, files in os.walk(folder, followlinks=False, onerror=error):
        directories[:] = sorted(name for name in directories if name not in EXCLUDED_DIRECTORIES)
        for name in list(directories):
            path = Path(parent) / name
            if path.is_symlink():
                result["symlinks"].append(str(path)); directories.remove(name)
        for name in sorted(files):
            path = Path(parent) / name
            try:
                first = path.lstat()
                if stat.S_ISLNK(first.st_mode):
                    result["symlinks"].append(str(path)); continue
                if not stat.S_ISREG(first.st_mode):
                    result["special"].append(str(path)); continue
                row = dict(path=str(path), relative=str(path.relative_to(folder)), bytes=first.st_size,
                    allocated_bytes=first.st_blocks * 512, uid=first.st_uid, nlink=first.st_nlink,
                    generation=generation(first), generation_stable=generation(first) == generation(path.lstat()))
                result["files"].append(row)
            except OSError as caught:
                result["errors"].append(str(caught))
    result["file_count"] = len(result["files"])
    result["logical_bytes"] = sum(row["bytes"] for row in result["files"])
    result["allocated_bytes_including_each_path"] = sum(row["allocated_bytes"] for row in result["files"])
    return result


def size(path):
    # HH_261007 - Retain du's raw outcome; an unreadable tree is not an exact complete size.
    command = ["du", "-x", "-B1", "-s", "--", str(path)]
    try:
        value = subprocess.run(command, capture_output=True, text=True, timeout=120)
        return dict(path=str(path), argv=command, returncode=value.returncode,
            stdout=value.stdout, stderr=value.stderr,
            allocated_bytes=int(value.stdout.split()[0]) if value.returncode == 0 else None)
    except subprocess.TimeoutExpired:
        return dict(path=str(path), argv=command, error="size query timed out", allocated_bytes=None)


def collect():
    start = time.monotonic()
    before = Path(__file__).read_bytes()
    catalogs = {key: inventory(path) for key, path in SOURCES.items()}
    sizes = {key: size(path) for key, path in SIZE_TARGETS.items()}
    if Path(__file__).read_bytes() != before:
        raise RuntimeError("catalog source changed")
    return dict(schema="portable_e2e.thesis_four_pc_storage_catalog.20261007.v1",
        provenance="HH_261007 - Metadata-only thesis and disk inventory; no source movement, deletion, replay, installation, simulator or training.",
        observed_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        source_sha256=hashlib.sha256(before).hexdigest(), elapsed_seconds=time.monotonic()-start,
        catalogs=catalogs, directory_sizes=sizes, available_bytes=shutil.disk_usage(ROOT).free,
        overlapping_catalogs_must_not_be_added_together=True,
        metadata_snapshot_not_content_hash_or_inactive_usage_proof=True,
        originals_moved=0, originals_deleted=0, GPU_training_simulator_ROS_or_actuation_called=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--write-catalog", action="store_true")
    args = parser.parse_args(argv)
    if not args.write_catalog:
        parser.error("explicit --write-catalog required for new metadata output")
    if OUTPUT.exists() or OUTPUT.resolve() != OUTPUT:
        raise RuntimeError("fresh canonical catalog output required")
    if shutil.disk_usage(ROOT).free < MAX_OUTPUT_BYTES + 64 * 1024**2:
        raise RuntimeError("metadata space reserve unavailable")
    report = collect()
    raw = (json.dumps(report, sort_keys=True, indent=2, allow_nan=False)+"\n").encode()
    stream = io.StringIO(newline=""); writer = csv.writer(stream)
    writer.writerow(["category", "original_path", "logical_bytes", "allocated_bytes", "metadata_generation_stable"])
    for category, catalog in report["catalogs"].items():
        for row in catalog["files"]:
            writer.writerow([category, row["path"], row["bytes"], row["allocated_bytes"], row["generation_stable"]])
    flat = stream.getvalue().encode()
    if len(raw)+len(flat) > MAX_OUTPUT_BYTES:
        raise RuntimeError("bounded metadata output exceeded")
    OUTPUT.mkdir(parents=True, mode=0o700)
    for name, payload in (("catalog.json", raw), ("file_locations.csv", flat)):
        with (OUTPUT/name).open("xb") as target:
            target.write(payload)
    print(json.dumps(dict(output=str(OUTPUT), output_bytes=len(raw)+len(flat), source_sha256=report["source_sha256"],
        catalog_sha256=hashlib.sha256(raw).hexdigest(), originals_moved=0, originals_deleted=0)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
