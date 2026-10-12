#!/usr/bin/env python3
"""HH_261001 - Record late cleanup state and raw RGB quality without changing v1 FAIL."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import struct
import zlib

try:
    from . import run_owned_ctrack_isolated_probe_20261001 as owner
    from .run_owned_ctrack_isolated_probe_20261001_v2 import rgb_quality
except ImportError:
    import run_owned_ctrack_isolated_probe_20261001 as owner
    from run_owned_ctrack_isolated_probe_20261001_v2 import rgb_quality


def raw_png_rgb(path: Path) -> bytes:
    data, offset, compressed = path.read_bytes(), 8, b""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    while offset < len(data):
        size = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset + 4:offset + 8]
        if kind == b"IHDR" and data[offset + 8:offset + 21] != struct.pack(">IIBBBBB", 640, 360, 8, 2, 0, 0, 0):
            raise ValueError("unexpected PNG geometry or encoding")
        if kind == b"IDAT":
            compressed += data[offset + 8:offset + 8 + size]
        offset += size + 12
    decoded = zlib.decompress(compressed)
    stride = 640 * 3 + 1
    if len(decoded) != stride * 360 or any(decoded[y * stride] != 0 for y in range(360)):
        raise ValueError("unexpected PNG filter or row length")
    return b"".join(decoded[y * stride + 1:(y + 1) * stride] for y in range(360))


def audit() -> dict:
    run = owner.DEFAULT_OUTPUT
    files = {name: run / name for name in (
        "owner_plan.json", "owner_started.json", "owner_result.json", "ready.json", "capture_result.json")}
    hashes = {name: owner.stage.sha256(path) for name, path in files.items()}
    original = json.loads(files["owner_result.json"].read_text())
    started = json.loads(files["owner_started.json"].read_text())
    ready = json.loads(files["ready.json"].read_text())
    plan = json.loads(files["owner_plan.json"].read_text())
    capture = json.loads(files["capture_result.json"].read_text())
    if original["status"] != "FAIL":
        raise ValueError("expected the preserved v1 FAIL receipt")
    pgid = started["server_pgid"]
    generations = []
    for prior in (started["launcher"], ready["editor"]):
        current = owner.process_record(prior["pid"])
        present = current is not None and current["start_ticks"] == prior["start_ticks"]
        generations.append({"original_pid": prior["pid"], "original_start_ticks": prior["start_ticks"],
                            "same_generation_present": present, "current_record": current})
    group = [p for p in owner.all_processes() if p["pgid"] == pgid and p["state"] != "Z"]
    try:
        owner.ports_free(plan["port_triplet"][0])
        ports_released, port_error = True, None
    except OSError as exc:
        ports_released, port_error = False, str(exc)
    owner.stage.verify(Path(plan["stage_root"]), smoke=False)
    source_ok = all(owner.stage.sha256(Path(path)) == record["sha256"] and
                    owner.stage.sha256(Path(record["archived_path"])) == record["sha256"]
                    for path, record in plan["source_provenance"].items())
    rgb = run / "capture/rgb.png"
    if owner.stage.sha256(rgb) != capture["artifacts"]["rgb.png"]:
        raise ValueError("raw RGB bytes changed")
    result = {"schema": "portable_e2e.ctrack_late_cleanup_and_rgb_audit.v1",
              "observed_at_utc": datetime.now(timezone.utc).isoformat(),
              "original_owner_status": original["status"], "original_receipt_sha256": hashes,
              "original_cleanup_errors": original["cleanup_errors"],
              "original_process_generations": generations, "live_original_pgid_members": group,
              "ports_released_now": ports_released, "port_error": port_error,
              "late_cleanup_status": "PASS" if not any(g["same_generation_present"] for g in generations)
                                       and not group and ports_released else "FAIL",
              "source_fingerprints_unchanged": source_ok,
              "raw_rgb_sha256": capture["artifacts"]["rgb.png"],
              "rgb_quality": rgb_quality(raw_png_rgb(rgb)),
              "road_pixels": capture["road_pixels"], "lane_pixels": capture["lane_pixels"],
              "original_result_overwritten": False, "signals_sent": [], "simulator_launched": False,
              "dataset_admitted": False, "train_eligible": False}
    if {name: owner.stage.sha256(path) for name, path in files.items()} != hashes:
        raise ValueError("original receipts changed during audit")
    return result


if __name__ == "__main__":
    result = audit()
    destination = owner.ROOT / "docs/assets/validation/2026-10-01/ctrack_isolation/v1_postmortem.json"
    owner.stage.write_json_new(destination, result)
    print(json.dumps(result, indent=2, sort_keys=True))
