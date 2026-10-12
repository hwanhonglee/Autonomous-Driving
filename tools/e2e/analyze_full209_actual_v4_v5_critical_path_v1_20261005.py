"""HH_261005 - Compare all frozen V4/V5 scalar critical paths without decoding images, forwarding a model or regrading FAIL.

Reproduce: CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m
scripts.e2e.analyze_full209_actual_v4_v5_critical_path_v1_20261005 -- --execute-readonly
"""

import argparse
import math
from pathlib import Path
import statistics

from . import analyze_full209_actual_callback_critical_path_v1_20261004 as original


ROOT = original.ROOT
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_actual_v4_v5_critical_path_v1_20261005.py"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-05/full209_actual_v4_v5_critical_path_readonly_v1"
RUNS = {
    "V4": ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v4",
    "V5": ROOT / "storage/experiments/validation/2026-10-05/temporal_route_joint_town03_known_dev_straight_full209_v5",
}
FIXED_SOURCES = {
    "tools/e2e/analyze_full209_actual_callback_critical_path_v1_20261004.py": "08fb1bde42d495d4a4cdc8270c72c23002a89bb1de5401234ac1bcdfb3d81124",
    "tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py": "3ed5b103fe30db4fac9ac13a0d4c6c5f7990ed17fc3b747ab5a06f883b81c73c",
    "tools/e2e/collect_carla_vad_expert.py": "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08",
}
INPUTS = ("diagnostic_report.json",) + tuple("episode/" + name for name in (
    "callback_timing_receipts.json", "quota_walk_timing_receipts.json", "states.jsonl", "camera_frames.jsonl",
    "wall_timing.jsonl", "learned_control.jsonl", "route.json", "recording_after_ack.jsonl"))
require, digest = original.require, original.digest


def stats(values):
    result = original.stats(values)
    result["mean"] = statistics.mean(values)
    return result


def metrics(rows):
    require(rows, "complete nonempty scalar timing rows required")
    names = {name for name in rows[0] if name.endswith("_ms")}
    require(all({name for name in row if name.endswith("_ms")} == names for row in rows),
        "timing field denominator may not drop intermediate or failed rows")
    return {name: stats([row[name] for row in rows]) for name in sorted(names)}


def cycles(rows, key):
    # HH_261005 - N complete endpoints yield exactly N-1 intervals; no cold/intermediate/tail outlier is discarded.
    require(len(rows) >= 2, "at least two genuine consecutive timing endpoints required")
    clocks = [row[key] for row in rows]
    require(all(type(value) is int for value in clocks) and all(a < b for a, b in zip(clocks, clocks[1:])),
        "strictly increasing integer monotonic timing endpoints required")
    intervals = [(b - a) / 1e6 for a, b in zip(clocks, clocks[1:])]
    return dict(endpoint_count=len(clocks), interval_count=len(intervals),
        first_monotonic_ns=clocks[0], last_monotonic_ns=clocks[-1],
        complete_first_to_last_seconds=(clocks[-1] - clocks[0]) / 1e9,
        rate_hz=len(intervals) * 1e9 / (clocks[-1] - clocks[0]), interval_ms=stats(intervals),
        no_interval_after_last_endpoint_invented=True, cold_or_failed_endpoint_removed=False)


def recording_join(recordings, cameras, walls, groups):
    require(len(recordings) == len(cameras), "all camera JPEG flush receipts required")
    wall_by_ack = {row["ACK_sequence"]: row for row in walls}
    require(len(wall_by_ack) == len(walls), "unique full wall ACK denominator required")
    native_frames = list(groups)
    next_tick = {frame: groups[next_frame]["world_tick_started"]["stage_finished_monotonic_ns"]
        for frame, next_frame in zip(native_frames, native_frames[1:])}
    rows = []
    for receipt, camera in zip(recordings, cameras):
        frame = receipt["frame"]
        ack = wall_by_ack[receipt["ACK_receipt_sequence"]]
        start, end = receipt["recording_started_monotonic_ns"], receipt["recording_finished_monotonic_ns"]
        parallel = receipt["parallel_encoding_research"]
        require(frame == camera["frame"] and ack["command_frame"] == frame and ack["ACK_server_accepted"] is True
            and receipt["failure"] is None and receipt["enqueued_camera_count"] == receipt["written_camera_count"] == 6
            and receipt["acknowledged_before_recording"] is True and receipt["all_queued_images_written"] is True
            and receipt["no_source_image_dropped"] is True and receipt["source_timestamp_changed"] is False
            and receipt["source_pixels_changed"] is False and receipt["synchronous_flush_before_next_tick"] is True
            and ack["ACK_finished_monotonic_ns"] <= start <= end
            and (frame not in next_tick or end <= next_tick[frame])
            and parallel["all_submitted_encodes_joined"] is True and parallel["submitted_encode_count"] == 6
            and parallel["original_role_ordered_writes"] is True and parallel["background_work_after_flush"] is False
            and start <= parallel["all_encodes_joined_monotonic_ns"] <= end,
            "all exact JPEG flush/ACK/next-native chronology must remain admitted")
        require(math.isclose(receipt["recording_seconds"], (end - start) / 1e9, rel_tol=0, abs_tol=1e-12),
            "recorded JPEG duration differs from original integer clocks")
        rows.append(dict(frame=frame, ACK_sequence=receipt["ACK_receipt_sequence"],
            recording_started_monotonic_ns=start, recording_finished_monotonic_ns=end,
            synchronous_after_ACK_JPEG_flush_ms=(end - start) / 1e6))
    return rows


def seal():
    sources = {name: digest(ROOT / name) for name in FIXED_SOURCES}
    require(sources == FIXED_SOURCES, "frozen original scalar reconstruction or measurement source changed")
    sources.update({str(path.relative_to(ROOT)): digest(path) for path in (HERE, TEST)})
    inputs = {}
    for version, run in RUNS.items():
        for name in INPUTS:
            path = original.safe_path(run / name)
            require(0 < path.stat().st_size <= 64 * 1024**2, "bounded scalar original input required")
            inputs[version + "/" + name] = digest(path)
    return dict(sources_sha256=sources, scalar_inputs_sha256=inputs,
        whole_camera_payload_archive_rehashed=False, original_image_pixels_opened=False)


def analyze_run(run):
    report = original.json_file(run / "diagnostic_report.json")
    require(report["status"] == "FAIL_KNOWN_DEV_FULL209_RESEARCH"
        and report["learned_driving_screen_passed"] is False, "retain genuine original full209 FAIL")
    ep = run / "episode"
    value, quota = [original.json_file(ep / name) for name in
        ("callback_timing_receipts.json", "quota_walk_timing_receipts.json")]
    states, cameras, walls, commands = [original.json_lines(ep / name) for name in
        ("states.jsonl", "camera_frames.jsonl", "wall_timing.jsonl", "learned_control.jsonl")]
    require((len(states), len(cameras), len(walls)) == (1836, 918, 1840), "complete actual original denominators required")
    joined, census = original.source_join(value, cameras, commands, walls)
    require(len(joined) == 883, "all883 genuine fresh NN rows including every cold/failure/tail required")
    native, groups = original.native_paths(value, states, cameras, original.json_file(ep / "route.json"))
    fresh, walks = original.critical_paths(joined, groups, quota, cameras, walls)
    recordings = recording_join(original.json_lines(ep / "recording_after_ack.jsonl"), cameras, walls, groups)
    branches = {branch: [row for row in fresh if row["branch"] == branch] for branch in
        ("ordinary_exact_delegate", "authenticated_navigation_terminal_V2")}
    require([len(rows) for rows in branches.values()] == [817, 66], "actual V4/V5 branches differ; do not transplant V2 offline counts")
    decoded = cycles(fresh, "decoded_plan_ready_ns")
    require(math.isclose(decoded["rate_hz"], report["actual_callback_timing"]["observed_decoded_plan_wall_rate_hz"],
        rel_tol=0, abs_tol=1e-12), "original decoded-plan first-to-last rate must reproduce exactly")
    return dict(original_status=report["status"], original_FAIL_not_regraded=True,
        all_fresh_metrics=metrics(fresh),
        all1836_native_metrics=metrics([{key: value for key, value in row.items()
            if key != "outer_exact_provider_ms"} for row in native]),
        all918_quota_metrics=metrics(walks), all918_JPEG_flush_metrics=metrics(recordings),
        all_camera_native_metrics=metrics([row for row in native if "outer_exact_provider_ms" in row]),
        branch_metrics={name: dict(count=len(rows), metrics=metrics(rows)) for name, rows in branches.items()},
        cycles=dict(first_selected_callback=cycles(fresh, "first_callback_ns"), original_decoded_plan=decoded),
        source_join_census=census, all883_fresh_rows=fresh, all1836_native_rows=native,
        all918_quota_rows=walks, all918_JPEG_flush_rows=recordings,
        plan100ms_failed_frames=[row["frame"] for row in fresh if row["plan100ms_failed"]],
        ACK100ms_failed_frames=[row["frame"] for row in fresh if row["ACK100ms_failed"]],
        all1840_ACK_wall_rows_accounted=True, native_mark_count=len(value["native_stage_marks"]))


def analyze():
    before = seal()
    runs = {name: analyze_run(run) for name, run in RUNS.items()}
    after = seal()
    require(before == after, "frozen scalar inputs or reader sources changed during diagnosis")
    sections = ("all_fresh_metrics", "all1836_native_metrics", "all918_quota_metrics", "all918_JPEG_flush_metrics")
    delta = {section: {metric: runs["V5"][section][metric]["mean"] - runs["V4"][section][metric]["mean"]
        for metric in runs["V4"][section]} for section in sections}
    return dict(schema="portable_e2e.actual_full209_V4_V5_scalar_critical_path.20261005.v1",
        provenance="HH_261005 - Reuse original complete source/native/quota joins; compare every recorded scalar without model/image execution or approval changes.",
        input_source_seal=before, all_scalar_inputs_sources_unchanged=True, runs=runs,
        V5_minus_V4_mean_ms=delta, independent_run_difference_not_controlled_ABBA_effect=True,
        native_snapshot_to_actor_state_source="collect_carla_vad_expert.py:901 measured_vehicle_fields sequential FL wheel, FR wheel and speed-limit calls inside the larger snapshot/state boundary; local CARLA source shows two wheel CallAndWait RPCs and a snapshot-cached speed limit, not three RPCs",
        local_CARLA_source_to_installed_binary_equivalence_verified=False,
        individual_RPC_duration_or_thread_safety_proven=False, pose_transform_only_cost_claimed=False,
        proposal_to_input_ready_source="unchanged owned causal camera/history preparation, normalization, validation and exact tensor SHA guards; not isolated normalization timing",
        full_forward_cached_NN_boundary_not_only_CNN=True, all5_heads_or_GUARD_work_skipped=False,
        JPEG_flush_is_synchronous_after_ACK_not_legacy_enqueue_timer=True,
        metrics_overlap_and_different_denominators_not_summed_to_claim_cycle_gain=True,
        scheduling_external_load_GC_or_individual_RPC_cause_proven=False,
        original_failed_timings_preserved=True, hardware_exposure_latency_proven=False,
        camera_decoding_NN_forward_CARLA_training_TEST_or_runtime_activation_performed=False,
        formal_feature_or_vehicle_deployment_approved=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute-readonly", action="store_true", required=True)
    parser.parse_args(argv)
    require(not OUTPUT.exists(), "fresh create-only scalar diagnosis output required")
    report = analyze()
    original.safe_path(OUTPUT, missing_tail=True)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    original.write_new_json(OUTPUT / "report.json", report)
    summary = {name: report[name] for name in ("schema", "all_scalar_inputs_sources_unchanged", "V5_minus_V4_mean_ms")}
    summary["runs"] = {version: {key: value for key, value in run.items() if not key.endswith("_rows")}
        for version, run in report["runs"].items()}
    summary.update(report_sha256=digest(OUTPUT / "report.json"), reproducer_sha256=digest(HERE))
    original.write_new_json(OUTPUT / "summary.json", summary)
    print("COMPLETED_READONLY_ALL883x2_ORIGINAL_FAIL_PRESERVED " + str(OUTPUT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
