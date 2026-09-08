#!/usr/bin/env python3
"""HH_260906 - Reconstruct four frozen substep trials' wall timing without importing current model code."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import inspect
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
from scripts.e2e import audit_carla_wall_timing_quality as timing

OUTPUT = Path(__file__).resolve().parents[1]
CAMPAIGN = ROOT / "artifacts/training/2026-09-09/substep_ab_v1"
AUDIT_ROOT = ROOT / "artifacts/training/2026-09-09/substep_ab_audit_complete_v2"
AUDIT_SHA = "a6b2f7117dbddd33ee14aaef79add93658b611fffc008ce5e4570ce2d2cc289b"
MANIFEST_SHA = "b5f65de1b275b9f8697d1dd01b05b520243ab9603e210e863338fcc9cf096fb3"
CASE_IDS = ("A_reference_10ms/run_001", "B_fine_5ms/run_001", "A_reference_10ms/run_002", "B_fine_5ms/run_002")
EXPECTED_TIMING = {
    "schema": "carla.expert_wall_timing.v1", "enabled": True, "clock": "time.perf_counter_ns",
    "stages": list(timing.STAGES), "per_camera_stages": ["camera_queue_wait", "jpeg_encode_write"],
    "journal": "wall_timing.jsonl", "all_attempts_retained": True, "sensor_timestamps_changed": False,
    "gui_display_fps_measured": False, "learned_inference_measured": False,
    "bootstrap_setup_teardown_in_native_tick_totals": False, "journal_write_outside_own_tick_total": True,
    "journal_errors_can_approve_data": False,
}


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def checked(path, ledger, expected=None):
    """HH_260906 - Preserve exact pre/post identities of inputs, not a claim that current capture sources still match history."""
    path = Path(path)
    timing.require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(ROOT), "unsafe input path")
    raw = path.read_bytes()
    relative = path.relative_to(ROOT).as_posix()
    entry = {"sha256": digest(raw), "size_bytes": len(raw)}
    timing.require(expected is None or entry == {k: expected[k] for k in entry}, "input differs from frozen evidence: " + relative)
    timing.require(relative not in ledger or ledger[relative] == entry, "input changed during reconstruction: " + relative)
    ledger[relative] = entry
    return raw


def decode(raw, lines=False):
    def invalid(value):
        raise ValueError("non-finite JSON constant: " + value)
    return [json.loads(line, parse_constant=invalid) for line in raw.splitlines()] if lines else json.loads(raw, parse_constant=invalid)


def write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def main():
    started = datetime.now(timezone.utc).isoformat()
    timing.require(not (OUTPUT / "summary.json").exists() and not (OUTPUT / "SHA256SUMS").exists(), "refuse existing analysis outputs")
    source_ledger, inputs = {}, {}
    # HH_260906 - These pure modules parse journals; current model.py is neither imported nor checked.
    modules = (timing, timing.base, timing.v4, timing.pilot, timing.ack)
    source_paths = [Path(__file__).resolve(), *(Path(module.__file__).resolve() for module in modules)]
    source_bytes = {path: checked(path, source_ledger) for path in source_paths}
    functions = {function.__module__ + "." + function.__name__: digest(inspect.getsource(function).encode())
        for function in (timing.analyze_timing, timing.validate_recorded_summary, timing.distribution)}
    audit_raw = checked(AUDIT_ROOT / "audit.json", inputs)
    checksum_raw = checked(AUDIT_ROOT / "SHA256SUMS", inputs)
    timing.require(digest(audit_raw) == AUDIT_SHA and digest(checksum_raw) == MANIFEST_SHA, "frozen full audit identity mismatch")
    audit = decode(audit_raw)
    timing.require(audit["status"] == "AUDITED_NOT_ADMITTED" and audit["finalized_case_count"] == 4
        and tuple(case["id"] for case in audit["cases"]) == CASE_IDS and audit["dataset_admission"] is False,
        "full four-case denied audit required")
    for entry in audit["source_manifest"]:
        checked(CAMPAIGN / entry["path"], inputs, entry)
    review = decode(checked(CAMPAIGN / "reviews/first_pair.json", inputs))
    interim_path = ROOT / review["interim_audit"]["path"]
    timing.require(digest(checked(interim_path, inputs)) == review["interim_audit"]["sha256"], "reviewed interim audit changed")
    cases = []
    for case in audit["cases"]:
        previous = case["audit"]
        timing.require(case["status"] == "FINALIZED" and previous["all_twelve_archives_match_owner_and_reviewed_commits"] is True
            and previous["training_data_approved"] is False and previous["dataset_admission"] is False, "trial provenance/admission mismatch")
        root = CAMPAIGN / case["id"]
        for entry in previous["source_manifest"]:
            checked(root / entry["path"], inputs, entry)
        prefixes = [name for name in ("episode", "episode.partial") if (root / name).is_dir()]
        timing.require(len(prefixes) == 1, "exactly one retained episode is required")
        prefix = root / prefixes[0]
        read = lambda name, lines=False: decode(checked(prefix / name, inputs), lines)
        manifest, states, cameras = read("manifest.json"), read("states.jsonl", True), read("camera_frames.jsonl", True)
        timing.require(timing.v4.strict_equal(manifest["capture_contract"]["wall_timing"], EXPECTED_TIMING), "timing contract changed")
        timing.require(manifest["provenance"]["wall_timing_helper_sha256"] == previous["archived_source_sha256"]["scripts/e2e/carla_wall_timing.py"], "timing helper archive mismatch")
        summary = read("wall_timing_summary.json")
        original = checked(prefix / "wall_timing.jsonl", inputs)
        declared = manifest["result"]["wall_timing"]
        timing.require(declared["summary_sha256"] == inputs[(prefix / "wall_timing_summary.json").relative_to(ROOT).as_posix()]["sha256"], "timing summary digest mismatch")
        recovered = (prefix / "wall_timing_recovery.jsonl").exists()
        timing.require(manifest["files"].get("wall_timing") == "wall_timing.jsonl"
            and manifest["files"].get("wall_timing_summary") == "wall_timing_summary.json"
            and (manifest["files"].get("wall_timing_recovery") == "wall_timing_recovery.jsonl" if recovered else "wall_timing_recovery" not in manifest["files"]), "journal file metadata mismatch")
        journal = checked(prefix / ("wall_timing_recovery.jsonl" if recovered else "wall_timing.jsonl"), inputs)
        rows = decode(journal, True)
        canonical = "".join(json.dumps(row, sort_keys=True, allow_nan=False) + "\n" for row in rows).encode()
        timing.require(canonical == journal and digest(journal) == summary["memory_records_sha256"], "memory row identity mismatch")
        independent = timing.analyze_timing(rows, states, cameras)
        persisted = timing.validate_recorded_summary(summary, independent, declared, journal_sha=digest(original),
            recovery_sha=digest(journal) if recovered else None, capture_succeeded=manifest["status"] == "complete")
        after_capture = decode(checked(root / "lifecycle/after_capture.json", inputs))
        timing.require(timing.v4.utc(manifest["created_at"]) <= timing.v4.utc(rows[0]["start_utc"])
            <= timing.v4.utc(rows[-1]["end_utc"]) <= timing.v4.utc(after_capture["checked_at"]), "journal outside capture lifecycle")
        camera_rows = [row for row in rows if row["camera_recorded"]]
        gaps = [right["camera_recorded_wall_ns"] - left["camera_recorded_wall_ns"] for left, right in zip(camera_rows, camera_rows[1:])]
        totals = sum(row["total_ns"] for row in rows if row["total_ns"] is not None)
        stages = {name: sum(row["stages"][name]["duration_ns"] or 0 for row in rows) for name in timing.STAGES}
        cases.append({"case_id": case["id"], "substep_profile": case["substep_profile"],
            "owner_exit_code": previous["owner_exit_code"], "capture_status": manifest["status"],
            "native_state_count": len(states), "camera_anchor_count": len(cameras),
            "journal_sha256": digest(original), "recovery_sha256": digest(journal) if recovered else None,
            "recorded_timing_status": summary["status"], "independent_journal_analysis": independent,
            "recorded_summary_comparison": persisted, "camera_completion_gap": timing.distribution(gaps),
            "in_tick_total_ns": totals, "stage_total_ns": stages,
            "stage_fraction_of_in_tick_total": {name: value / totals if totals else None for name, value in stages.items()},
            "unattributed_fraction_of_in_tick_total": sum(row["unattributed_ns"] or 0 for row in rows) / totals if totals else None,
            "capture_failure_not_hidden": manifest["status"] != "complete", "dataset_admission": False})
    # HH_260906 - Snapshot and recheck only analysis dependencies; historical model bytes stay evidence in raw archive paths.
    snapshot_root = OUTPUT / "source" / "dependencies"
    for path, raw in source_bytes.items():
        if path == Path(__file__).resolve():
            continue
        target = snapshot_root / path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as stream:
            stream.write(raw)
    for ledger in (source_ledger, inputs):
        for name, entry in list(ledger.items()):
            checked(ROOT / name, ledger, entry)
    timing.require("portable_e2e.model" not in sys.modules and "torch" not in sys.modules, "unexpected model/runtime import")
    report = {"schema": "carla_expert.substep_wall_timing_reconstruction.v1", "status": "RECONSTRUCTED_NOT_ADMITTED",
        "started_at_utc": started, "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        "full_numerical_audit_sha256": AUDIT_SHA, "full_numerical_audit_checksum_manifest_sha256": MANIFEST_SHA,
        "case_count": len(cases), "native_state_count": sum(c["native_state_count"] for c in cases),
        "camera_anchor_count": sum(c["camera_anchor_count"] for c in cases), "cases": cases,
        "source_identity": {"files": source_ledger, "functions": functions}, "input_manifest": inputs,
        "inputs_and_analysis_sources_unchanged_before_after": True, "all_attempts_retained": True,
        "current_model_source_read_or_imported": False, "gui_display_fps_measured": False,
        "learned_inference_measured": False, "hardware_load_measured": False, "dataset_admission": False,
        "stage_fraction_notice": "Fractions divide journaled stage sums by summed in-tick totals, excluding inter-tick gaps, persistence, bootstrap and setup/teardown. world_tick_snapshot includes simulator tick/render/snapshot waiting, not isolated physics or hardware attribution.",
        "failure_notice": "A failed capture retains PARTIAL_OR_FAILED_DIAGNOSTIC even when its timing journal is internally complete. No failed capture is upgraded or admitted.",
        "simulation_vs_wall_notice": "Fixed 20 Hz native and 10 Hz camera simulation cadence differs from faster-than-real-time offline camera file completion; neither measures display FPS or learned inference."}
    write_json(OUTPUT / "summary.json", report)
    for ledger in (source_ledger, inputs):
        for name, entry in list(ledger.items()):
            checked(ROOT / name, ledger, entry)
    payloads = sorted(path for path in OUTPUT.rglob("*") if path.is_file() and "__pycache__" not in path.parts)
    with (OUTPUT / "SHA256SUMS").open("x", encoding="utf-8") as stream:
        for path in payloads:
            stream.write(digest(path.read_bytes()) + "  " + path.relative_to(OUTPUT).as_posix() + "\n")
    print(json.dumps({"status": report["status"], "cases": len(cases), "summary_sha256": digest((OUTPUT / "summary.json").read_bytes())}))


if __name__ == "__main__":
    main()
