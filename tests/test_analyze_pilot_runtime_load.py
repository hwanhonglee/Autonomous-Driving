from __future__ import annotations

import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

from zoneinfo import ZoneInfo
import pytest


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "e2e" / "analyze_pilot_runtime_load.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_pilot_runtime_load", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)


def _representative_stack() -> dict:
    source = [2.5 + index * 0.2 for index in range(12)]
    slow_period = 0.2 / 0.249
    recovered_period = 0.2 / 0.999
    wall = [1_788_326_900.0]
    for index in range(1, len(source)):
        if index <= 5:
            delta = slow_period
        elif index == 6:
            delta = 0.16
        else:
            delta = recovered_period
        wall.append(wall[-1] + delta)

    lines = [
        "[INFO] [autoware_carla_interface-1]: process started with pid [10]",
        "[INFO] [rviz2-2]: process started with pid [11]",
        "[INFO] [vad_node-3]: process started with pid [12]",
        "[INFO] [vad_route_manager.py-4]: process started with pid [13]",
        "[INFO] [component_container_mt-5]: process started with pid [14]",
    ]
    for index, (wall_sec, source_sec) in enumerate(zip(wall, source), start=1):
        inference_ms = 34.0 if index <= 6 else 31.7
        lines.append(
            "[vad_node-3] "
            f"[INFO {wall_sec:.9f}] [vad]: VAD inference complete: "
            f"source_stamp_ns={round(source_sec * 1e9)} inference_ms={inference_ms:.3f} "
            f"published=true published_count={index} mailbox_taken={index} "
            "coalesced_drops=0 pending_source_stamp_ns=-1"
        )
    lines.append(
        "[vad_node-3] [INFO 1788326901.0] [vad]: VAD frame queued: "
        "assembled=12 capacity_pruned=0 superseded=0 mailbox_submitted=12 "
        "coalesced_drops=0 received_images_min=12 received_images_max=12"
    )
    return runtime.parse_stack_log("\n".join(lines))


def _persistent_stack(sample_count: int = 60, rtf: float = 0.248) -> dict:
    source = [2.5 + index * 0.2 for index in range(sample_count)]
    wall = [1_788_326_900.0]
    for index in range(1, sample_count):
        jitter = 1.0 + 0.01 * ((index % 5) - 2) / 2.0
        wall.append(wall[-1] + (0.2 / rtf) * jitter)
    lines = [
        "[INFO] [autoware_carla_interface-1]: process started with pid [10]",
        "[INFO] [rviz2-2]: process started with pid [11]",
        "[INFO] [vad_node-3]: process started with pid [12]",
        "[INFO] [vad_route_manager.py-4]: process started with pid [13]",
        "[INFO] [component_container_mt-5]: process started with pid [14]",
    ]
    for index, (wall_sec, source_sec) in enumerate(zip(wall, source), start=1):
        lines.append(
            "[vad_node-3] "
            f"[INFO {wall_sec:.9f}] [vad]: VAD inference complete: "
            f"source_stamp_ns={round(source_sec * 1e9)} inference_ms=34.000 "
            f"published=true published_count={index} mailbox_taken={index} "
            "coalesced_drops=0 pending_source_stamp_ns=-1"
        )
    lines.append(
        f"[vad_node-3] [INFO {wall[-1]:.9f}] [vad]: VAD frame queued: "
        f"assembled={sample_count} capacity_pruned=0 superseded=0 "
        f"mailbox_submitted={sample_count} coalesced_drops=0 "
        f"received_images_min={sample_count} received_images_max={sample_count}"
    )
    return runtime.parse_stack_log("\n".join(lines))


def _camera_records(source_seconds: list[float], spans_ms: list[float]):
    records = {topic: [] for topic in runtime.CAMERA_INFO_TOPICS}
    for frame_index, (source_sec, span_ms) in enumerate(zip(source_seconds, spans_ms)):
        source_ns = round(source_sec * 1.0e9)
        base_ns = 1_788_326_900_000_000_000 + frame_index * 200_000_000
        for camera_index, topic in enumerate(runtime.CAMERA_INFO_TOPICS):
            offset_ns = round(span_ms * 1.0e6 * camera_index / 5.0)
            records[topic].append(
                {"bag_ns": base_ns + offset_ns, "stamp_ns": source_ns, "topic": topic}
            )
    return records


def _vm_row(epoch: float, busy: float) -> dict:
    return {
        "epoch_sec": epoch,
        "runnable": 2.0,
        "blocked": 0.0,
        "swap_used_kib": 0.0,
        "free_kib": 10_000_000.0,
        "buffer_kib": 2_000_000.0,
        "cache_kib": 36_000_000.0,
        "swap_in_kib_s": 0.0,
        "swap_out_kib_s": 0.0,
        "block_in_kib_s": 0.0,
        "block_out_kib_s": 400.0,
        "interrupts_s": 1000.0,
        "context_switches_s": 2000.0,
        "cpu_user_percent": busy - 2.0,
        "cpu_system_percent": 2.0,
        "cpu_idle_percent": 100.0 - busy,
        "cpu_iowait_percent": 0.0,
        "cpu_steal_percent": 0.0,
        "cpu_busy_percent": busy,
    }


def _gpu_row(epoch: float, sm: float) -> dict:
    return {
        "epoch_sec": epoch,
        "power_w": 130.0 if sm < 20.0 else 157.0,
        "gpu_temp_c": 57.0,
        "sm_percent": sm,
        "memory_percent": 3.0,
        "framebuffer_mib": 4580.0,
        "rx_pcie_mib_s": 900.0 if sm < 20.0 else 1600.0,
        "tx_pcie_mib_s": 300.0 if sm < 20.0 else 400.0,
        "power_violation_percent": 0.0,
        "thermal_violation": 0.0,
    }


def _pid_row(epoch: float, pid: int, command: str, cpu: float) -> dict:
    return {
        "epoch_sec": epoch,
        "pid": pid,
        "command": command,
        "cpu_percent": cpu,
        "wait_percent": 0.0,
        "rss_kib": 100_000.0,
        "read_kib_s": 0.0,
        "write_kib_s": 10.0,
    }


def _representative_report(
    stack_override: dict | None = None,
    source_override: list[float] | None = None,
    spans_override: list[float] | None = None,
    host_busy_override: tuple[float, float] = (10.0, 16.0),
    gpu_sm_override: tuple[float, float] = (14.0, 21.0),
    latency_matched_override: int | None = None,
) -> dict:
    stack = stack_override or _representative_stack()
    vad = runtime.build_vad_runtime(stack)
    transition = vad["analysis_window_boundary"]["host_boundary_epoch_sec"]
    trial_start = vad["phases"]["full_run"]["wall_start_epoch_sec"] - 1.0
    trial_end = vad["phases"]["full_run"]["wall_end_epoch_sec"] + 1.0
    initial_epochs = [trial_start + 0.5, transition - 1.0]
    recovered_epochs = [transition + 0.1, trial_end - 0.1]

    vmstat = [_vm_row(epoch, host_busy_override[0]) for epoch in initial_epochs] + [
        _vm_row(epoch, host_busy_override[1]) for epoch in recovered_epochs
    ]
    gpu = [_gpu_row(epoch, gpu_sm_override[0]) for epoch in initial_epochs] + [
        _gpu_row(epoch, gpu_sm_override[1]) for epoch in recovered_epochs
    ]
    processes = {
        10: ("python3", (3.0, 12.0)),
        11: ("rviz2", (9.0, 19.0)),
        12: ("vad_node", (2.5, 8.0)),
        13: ("python3", (9.0, 43.0)),
        14: ("component_conta", (8.0, 25.0)),
        20: ("CarlaUE4-Linux-", (110.0, 138.0)),
        30: ("ffmpeg", (3.6, 4.5)),
        40: ("ros2", (30.0, 35.0)),
    }
    pid_rows = []
    for phase_index, epochs in enumerate((initial_epochs, recovered_epochs)):
        for epoch in epochs:
            for pid, (command, cpu_by_phase) in processes.items():
                pid_rows.append(_pid_row(epoch, pid, command, cpu_by_phase[phase_index]))

    source = source_override or [
        2.7,
        2.9,
        3.1,
        3.3,
        3.5,
        3.7,
        3.9,
        4.1,
        4.3,
        4.5,
        4.7,
    ]
    spans = spans_override or [647.112] * 4 + [600.0] + [9.458] * 6
    camera_records = _camera_records(source, spans)
    start_iso = datetime.fromtimestamp(trial_start, timezone.utc).isoformat()
    end_iso = datetime.fromtimestamp(trial_end, timezone.utc).isoformat()
    result = {
        "started_at": start_iso,
        "finished_at": end_iso,
        "success": False,
        "reason": "speed exposure contract failed",
        "metrics": {
            "sim_elapsed_sec": 10.0,
            "wall_elapsed_sec": 20.0,
            "maximum_observed_speed_mps": 9.7,
            "traveled_distance_m": 100.0,
        },
        "actual_path": [
            {"sim_time_sec": 3.5, "x": 279.0, "y": 17.3, "speed_mps": 7.9},
            {"sim_time_sec": 3.7, "x": 278.2, "y": 17.3, "speed_mps": 8.0},
        ],
    }
    latency = {
        "camera_bundle": {
            "matched_bundle_count": (
                len(source)
                if latency_matched_override is None
                else latency_matched_override
            ),
            "bundle_coverage_percent": 100.0,
        },
        "clock_domains": {"cross_domain_subtraction_used": False},
        "candidate_front_acceptance": {"acceptance_percent": 100.0},
        "data_quality_warnings": [],
    }
    recorder = {
        "camera_info_topic_count": 6,
        "subscribed_topics": list(runtime.CAMERA_INFO_TOPICS),
    }
    return runtime.build_runtime_load_analysis(
        result=result,
        latency=latency,
        stack=stack,
        recorder=recorder,
        camera_records=camera_records,
        vmstat_rows=vmstat,
        gpu_rows=gpu,
        pidstat={"logical_cpu_count": 24, "samples": pid_rows},
    )


def test_stack_parser_and_change_point_recover_expected_rtf_phases() -> None:
    stack = _representative_stack()

    report = runtime.build_vad_runtime(stack)

    assert report["sample_count"] == 12
    assert report["phase_detection"]["boundary_index"] == 6
    assert report["transition"]["initial_last_source_sim_sec"] == pytest.approx(3.5)
    assert report["transition"]["recovered_first_source_sim_sec"] == pytest.approx(3.7)
    assert report["phases"]["initial_slow"]["aggregate_rtf"] == pytest.approx(0.249)
    assert report["phases"]["recovered"]["aggregate_rtf"] == pytest.approx(0.999)
    assert report["phases"]["initial_slow"]["inference_ms"]["mean"] == pytest.approx(34.0)
    assert report["phases"]["recovered"]["inference_ms"]["mean"] == pytest.approx(31.7)
    assert report["integrity"]["final_counters"]["coalesced_drops"] == 0
    assert report["integrity"]["queue_counter_maxima"]["capacity_pruned"] == 0


def test_persistent_low_rtf_is_classified_without_inventing_a_recovery() -> None:
    report = runtime.build_vad_runtime(_persistent_stack())

    assert report["runtime_pattern"] == "persistent_low_rtf_no_recovery"
    assert report["phase_detection"]["detected"] is False
    assert report["transition"] is None
    assert report["analysis_window_boundary"]["kind"] == (
        "descriptive_midpoint_not_a_recovery"
    )
    assert report["phases"]["full_run"]["aggregate_rtf"] == pytest.approx(
        0.248, rel=0.01
    )
    assert report["phases"]["early_window"]["aggregate_rtf"] == pytest.approx(
        report["phases"]["late_window"]["aggregate_rtf"], rel=0.02
    )
    assert report["phase_detection"]["interval_near_realtime_fraction"] == 0.0
    assert report["phase_detection"]["rolling_rtf"]["max"] < 0.8


def test_camera_bundles_require_exact_same_stamp_and_keep_transition_guard() -> None:
    stack = _representative_stack()
    vad = runtime.build_vad_runtime(stack)
    source = [2.7, 2.9, 3.1, 3.3, 3.5, 3.7, 3.9, 4.1]
    records = _camera_records(source, [647.112] * 4 + [600.0, 9.458, 9.458, 9.458])

    camera = runtime.build_camera_runtime(records, vad)

    assert camera["matched_bundle_count"] == 8
    assert camera["source_rate_hz_from_median_period"] == pytest.approx(5.0)
    assert camera["phases"]["initial_slow_clean"]["bundle_count"] == 4
    assert camera["phases"]["initial_slow_clean"]["receipt_span_ms"]["mean"] == pytest.approx(
        647.112
    )
    assert camera["transition_guard"]["bundle_count"] == 1
    assert camera["phases"]["recovered_clean"]["receipt_span_ms"]["mean"] == pytest.approx(
        9.458
    )
    assert camera["comparison"]["mean_receipt_span_reduction_factor"] == pytest.approx(
        647.112 / 9.458
    )


def test_persistent_camera_stall_uses_descriptive_windows_without_collapse_claim() -> None:
    stack = _persistent_stack()
    vad = runtime.build_vad_runtime(stack)
    source = [2.7 + index * 0.2 for index in range(58)]
    records = _camera_records(source, [648.0] * len(source))

    camera = runtime.build_camera_runtime(records, vad)

    assert camera["runtime_pattern"] == "persistent_low_rtf_no_recovery"
    assert camera["comparison_phase_names"] == ["early_window", "late_window"]
    assert camera["phases"]["full_run"]["receipt_span_ms"]["mean"] == pytest.approx(
        648.0
    )
    assert camera["comparison"]["mean_receipt_span_first_to_second_ratio"] == pytest.approx(
        1.0
    )
    assert "mean_receipt_span_reduction_factor" not in camera["comparison"]


def test_telemetry_parsers_preserve_units_and_local_timezone() -> None:
    vmstat_text = """
procs -----------memory---------- ---swap-- -----io---- -system-- ------cpu----- -----timestamp-----
 r  b   swpd   free   buff  cache   si   so    bi    bo   in   cs us sy id wa st                 KST
 2  0      0 9994240 2799000 37652000    0    0     0   400 1000 2000  8  2 90  0  0 2026-09-02 14:30:00
"""
    gpu_text = """
#Date        Time         gpu    pwr  gtemp  mtemp     sm    mem    enc    dec    jpg    ofa   mclk   pclk  pviol  tviol     fb   bar1   ccpm  sbecc  dbecc    pci  rxpci  txpci
 20260902    14:30:00       0    129     57      -     14      3      0      0      0      0  10501   2055      0      0   4580     77      0      -      -      0    900    300
"""
    pidstat_text = """Linux 6.8.0 (host) \t09/02/26 \t_x86_64_\t(24 CPU)
# Time UID PID %usr %system %guest %wait %CPU CPU minflt/s majflt/s VSZ RSS %MEM kB_rd/s kB_wr/s kB_ccwr/s iodelay Command
14:30:00 1000 20 108.00 2.00 0.00 0.00 110.00 8 2.00 0.00 7108840 2389596 3.64 0.00 0.00 0.00 0 CarlaUE4-Linux-
"""

    vmstat = runtime.parse_vmstat(vmstat_text)
    gpu = runtime.parse_nvidia_dmon(gpu_text)
    pidstat = runtime.parse_pidstat(pidstat_text)

    expected_epoch = datetime(
        2026, 9, 2, 14, 30, tzinfo=ZoneInfo("Asia/Seoul")
    ).timestamp()
    assert vmstat[0]["epoch_sec"] == expected_epoch
    assert vmstat[0]["cpu_busy_percent"] == pytest.approx(10.0)
    assert vmstat[0]["free_kib"] == pytest.approx(9_994_240.0)
    assert gpu[0]["memory_temp_c"] is None
    assert gpu[0]["sm_percent"] == pytest.approx(14.0)
    assert pidstat["logical_cpu_count"] == 24
    assert pidstat["samples"][0]["cpu_percent"] == pytest.approx(110.0)
    assert pidstat["samples"][0]["command"] == "CarlaUE4-Linux-"


def test_recorder_parser_proves_six_camera_subscriptions_and_window() -> None:
    topic_lines = "\n".join(
        f"[INFO 100.1] [rosbag2_recorder]: Subscribed to topic '{topic}'"
        for topic in runtime.CAMERA_INFO_TOPICS
    )
    text = (
        "[INFO 100.000] [rosbag2_recorder]: Recording...\n"
        + topic_lines
        + "\n[INFO 200.000] [rosbag2_recorder]: Recording stopped"
    )

    recorder = runtime.parse_recorder_log(text)

    assert recorder["recording_start_epoch_sec"] == pytest.approx(100.0)
    assert recorder["recording_stop_epoch_sec"] == pytest.approx(200.0)
    assert recorder["camera_info_topic_count"] == 6
    assert recorder["warning_or_error_lines"] == []


def test_input_resolution_prefers_attempt_scoped_telemetry(tmp_path: Path) -> None:
    trial = tmp_path / "artifact" / "trial" / "attempt_002"
    trial.mkdir(parents=True)
    (trial / "latency").mkdir()
    (trial / "bag").mkdir()
    for path in (
        trial / "result.json",
        trial / "latency" / "e2e_latency.json",
        trial / "stack.log",
        trial / "recorder.log",
    ):
        path.write_text("{}", encoding="utf-8")
    telemetry = tmp_path / "artifact" / "host_telemetry" / "attempt_002"
    telemetry.mkdir(parents=True)
    for filename in ("vmstat.log", "nvidia_smi_dmon.log", "pidstat.log"):
        (telemetry / filename).write_text("fixture", encoding="utf-8")
    args = SimpleNamespace(
        trial_dir=trial,
        host_telemetry_dir=None,
        result=None,
        latency_json=None,
        bag=None,
        stack_log=None,
        recorder_log=None,
        vmstat_log=None,
        nvidia_dmon_log=None,
        pidstat_log=None,
    )

    paths = runtime._resolve_inputs(args)

    assert paths["vmstat"].parent == telemetry.resolve()


def test_build_report_records_load_increase_and_causal_boundary() -> None:
    report = _representative_report()

    assert report["schema_version"] == 3
    assert report["status"] == "complete"
    assert report["problems"] == []
    assert report["support"] == {
        "classification": (
            "transient_camera_delivery_path_stall_not_host_wide_saturation"
        ),
        "classification_supported": True,
        "classification_claims": {
            "camera_delivery_pattern": True,
            "host_wide_saturation_excluded": True,
        },
        "camera_pattern_supported": True,
        "no_host_wide_saturation_supported": True,
    }
    assert report["findings"]["classification"] == (
        "transient_camera_delivery_path_stall_not_host_wide_saturation"
    )
    assert report["findings"]["camera_hz_is_not_the_rtf_cause"]["supported"] is True
    assert report["findings"]["vad_inference_is_not_the_bottleneck"]["supported"] is True
    saturation = report["findings"]["host_wide_saturation_is_not_supported"]
    assert saturation["supported"] is True
    assert saturation["initial_cpu_busy_percent"] == pytest.approx(10.0)
    assert saturation["recovered_cpu_busy_percent"] == pytest.approx(16.0)
    confounding = report["findings"]["single_root_cause_may_not_be_claimed"]
    assert confounding["value"] is True
    assert confounding["status"] == "warm_up_and_scene_boundary_are_confounded"
    carla = report["host_load"]["process_groups"]["carla_server"]
    rviz = report["host_load"]["process_groups"]["rviz"]
    assert carla["phases"]["initial_slow"]["cpu_percent_of_one_logical_core"][
        "mean"
    ] == pytest.approx(110.0)
    assert rviz["phases"]["recovered"]["cpu_percent_of_one_logical_core"][
        "mean"
    ] == pytest.approx(19.0)
    assert report["host_load"]["gpu_device_total"][
        "per_process_attribution_available"
    ] is False


def test_build_report_marks_persistent_stall_and_midpoint_as_noncausal() -> None:
    stack = _persistent_stack()
    source = [2.7 + index * 0.2 for index in range(58)]
    report = _representative_report(stack, source, [648.0] * len(source))

    assert report["schema_version"] == 3
    assert report["findings"]["runtime_pattern"] == (
        "persistent_low_rtf_no_recovery"
    )
    assert report["findings"]["classification"] == (
        "persistent_camera_delivery_path_stall_without_recovery_not_host_wide_saturation"
    )
    assert report["findings"]["camera_delivery_pattern"]["supported"] is True
    assert report["findings"]["camera_delivery_stall_collapses_at_recovery"][
        "supported"
    ] is False
    assert report["findings"]["host_wide_saturation_is_not_supported"][
        "supported"
    ] is True
    assert report["transition_location"] is None
    assert report["runtime_boundary_location"]["kind"] == (
        "descriptive_midpoint_not_a_recovery"
    )


def test_classification_does_not_claim_unsupported_camera_pattern() -> None:
    source = [2.7 + index * 0.2 for index in range(11)]
    report = _representative_report(
        source_override=source,
        spans_override=[20.0] * len(source),
    )

    classification = report["findings"]["classification"]
    assert classification == (
        "transient_rtf_recovery_without_supported_camera_delivery_pattern_"
        "not_host_wide_saturation"
    )
    assert "camera_delivery_path_stall" not in classification
    assert report["support"]["camera_pattern_supported"] is False
    assert report["support"]["no_host_wide_saturation_supported"] is True
    assert report["support"]["classification_supported"] is True
    assert report["findings"]["camera_delivery_pattern"]["classification"] == (
        "abrupt_rtf_recovery_without_supported_receipt_span_collapse"
    )
    plot_text = "\n".join(runtime._plot_conclusion_lines(report))
    assert "camera-delivery pattern is not supported" in plot_text
    assert "stall is localized" not in plot_text


def test_classification_does_not_exclude_unsupported_host_saturation() -> None:
    report = _representative_report(
        host_busy_override=(95.0, 96.0),
        gpu_sm_override=(96.0, 97.0),
    )

    classification = report["findings"]["classification"]
    assert classification == (
        "transient_camera_delivery_pattern_host_wide_saturation_not_excluded"
    )
    assert not classification.endswith("not_host_wide_saturation")
    assert report["support"]["camera_pattern_supported"] is True
    assert report["support"]["no_host_wide_saturation_supported"] is False
    assert report["support"]["classification_supported"] is True
    plot_text = "\n".join(runtime._plot_conclusion_lines(report))
    assert "Host-wide saturation cannot be excluded" in plot_text
    assert "Host-wide saturation is not supported" not in plot_text


def test_evidence_contract_is_incomplete_when_crosscheck_disagrees() -> None:
    report = _representative_report(latency_matched_override=999)

    assert report["status"] == "incomplete"
    assert report["support"]["classification_supported"] is False
    assert report["problems"] == [
        "latency report and bag reconstruction disagree on matched camera bundles"
    ]


def test_atomic_outputs_are_valid_and_leave_no_staging_files(tmp_path: Path) -> None:
    report = _representative_report()
    (tmp_path / "runtime_load_analysis.json").write_text("old", encoding="utf-8")
    (tmp_path / "runtime_load_analysis.png").write_bytes(b"old")

    json_path, png_path = runtime.write_outputs_atomically(tmp_path, report)

    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 3
    assert png_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert not list(tmp_path.glob(".*.tmp"))
    assert not [
        path
        for path in tmp_path.glob(".*.png")
        if path.name != "runtime_load_analysis.png"
    ]


def test_atomic_outputs_preserve_previous_pair_when_rendering_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    json_path = tmp_path / "runtime_load_analysis.json"
    png_path = tmp_path / "runtime_load_analysis.png"
    json_path.write_bytes(b"old-json")
    png_path.write_bytes(b"old-png")

    def fail_render(_report, _output):
        raise RuntimeError("render failed")

    monkeypatch.setattr(runtime, "render_runtime_load_png", fail_render)

    with pytest.raises(RuntimeError, match="render failed"):
        runtime.write_outputs_atomically(tmp_path, _representative_report())

    assert json_path.read_bytes() == b"old-json"
    assert png_path.read_bytes() == b"old-png"
    assert not list(tmp_path.glob(".*.tmp"))
    assert not list(tmp_path.glob(".*.png"))
