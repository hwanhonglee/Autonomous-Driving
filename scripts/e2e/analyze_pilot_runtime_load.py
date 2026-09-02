#!/usr/bin/env python3
"""Classify and reconstruct a CARLA/VAD pilot's runtime-load behavior.

The report deliberately keeps three clock domains separate:

* VAD log wall timestamps are Unix/system time.
* camera message stamps are CARLA simulation time.
* rosbag timestamps are recorder receipt/system time.

An abrupt real-time-factor (RTF) recovery is detected from consecutive VAD
completion records when present; otherwise the full-run and rolling RTF are
used to distinguish a persistent low-RTF run from steady near-real-time
behavior.  Six-camera delivery skew is measured only for bundles whose
``camera_info`` messages have exactly the same simulation stamp.  This
localizes a delay in the camera delivery path, but it does not prove whether
conversion, reliable DDS flow control, rendering, or asset streaming caused
the delay.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import statistics
import tempfile
from collections import defaultdict
from datetime import date, datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from zoneinfo import ZoneInfo


CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CAMERA_INFO_TOPICS = tuple(
    f"/sensing/camera/{camera}/camera_info" for camera in CAMERAS
)

VMSTAT_FIELDS = (
    "runnable",
    "blocked",
    "swap_used_kib",
    "free_kib",
    "buffer_kib",
    "cache_kib",
    "swap_in_kib_s",
    "swap_out_kib_s",
    "block_in_kib_s",
    "block_out_kib_s",
    "interrupts_s",
    "context_switches_s",
    "cpu_user_percent",
    "cpu_system_percent",
    "cpu_idle_percent",
    "cpu_iowait_percent",
    "cpu_steal_percent",
)

NVIDIA_DMON_FIELDS = (
    "gpu_index",
    "power_w",
    "gpu_temp_c",
    "memory_temp_c",
    "sm_percent",
    "memory_percent",
    "encoder_percent",
    "decoder_percent",
    "jpeg_percent",
    "ofa_percent",
    "memory_clock_mhz",
    "processor_clock_mhz",
    "power_violation_percent",
    "thermal_violation",
    "framebuffer_mib",
    "bar1_mib",
    "ccpm_mib",
    "single_bit_ecc_errors",
    "double_bit_ecc_errors",
    "pcie_errors",
    "rx_pcie_mib_s",
    "tx_pcie_mib_s",
)

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
VAD_INFERENCE_RE = re.compile(
    r"\[INFO\s+(?P<wall>[0-9]+\.[0-9]+)\].*?"
    r"VAD inference complete:\s+source_stamp_ns=(?P<stamp>[0-9]+)\s+"
    r"inference_ms=(?P<latency>[0-9.]+)\s+published=(?P<published>true|false)\s+"
    r"published_count=(?P<published_count>[0-9]+)\s+"
    r"mailbox_taken=(?P<mailbox_taken>[0-9]+)\s+"
    r"coalesced_drops=(?P<coalesced_drops>[0-9]+)"
)
PROCESS_START_RE = re.compile(
    r"\[INFO\]\s+\[(?P<name>[^]]+)\]: process started with pid \[(?P<pid>[0-9]+)\]"
)
RECORDER_TOPIC_RE = re.compile(r"Subscribed to topic '([^']+)'")


class RuntimeLoadError(RuntimeError):
    """Raised when the evidence cannot support a runtime phase comparison."""


def percentile(values: Sequence[float], quantile: float) -> float:
    """Return a NumPy-compatible linearly interpolated percentile."""
    if not values:
        raise ValueError("percentile requires at least one value")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * quantile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def numeric_summary(values: Iterable[float | int | None]) -> dict[str, Any]:
    finite = [
        float(value)
        for value in values
        if value is not None and math.isfinite(float(value))
    ]
    if not finite:
        return {"available": False, "count": 0}
    return {
        "available": True,
        "count": len(finite),
        "min": min(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "p95": percentile(finite, 95.0),
        "p99": percentile(finite, 99.0),
        "max": max(finite),
    }


def _strip_ansi(text: str) -> str:
    return ANSI_ESCAPE.sub("", text)


def _iso_from_epoch(epoch_sec: float, zone: timezone | ZoneInfo = timezone.utc) -> str:
    return datetime.fromtimestamp(epoch_sec, zone).isoformat()


def _parse_iso_timestamp(value: str) -> float:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise RuntimeLoadError(f"timestamp has no timezone: {value}")
    return parsed.timestamp()


def parse_stack_log(text: str) -> dict[str, Any]:
    """Parse VAD timing/counters and launch-owned process identities."""
    clean = _strip_ansi(text)
    inference_samples: list[dict[str, Any]] = []
    for match in VAD_INFERENCE_RE.finditer(clean):
        wall_epoch = float(match.group("wall"))
        source_stamp_ns = int(match.group("stamp"))
        inference_samples.append(
            {
                "wall_epoch_sec": wall_epoch,
                "wall_utc": _iso_from_epoch(wall_epoch),
                "source_stamp_ns": source_stamp_ns,
                "source_sim_sec": source_stamp_ns * 1.0e-9,
                "inference_ms": float(match.group("latency")),
                "published": match.group("published") == "true",
                "published_count": int(match.group("published_count")),
                "mailbox_taken": int(match.group("mailbox_taken")),
                "coalesced_drops": int(match.group("coalesced_drops")),
            }
        )
    inference_samples.sort(key=lambda sample: sample["wall_epoch_sec"])

    launch_processes = []
    for match in PROCESS_START_RE.finditer(clean):
        raw_name = match.group("name")
        launch_processes.append(
            {
                "launch_name": raw_name,
                "node_name": re.sub(r"-[0-9]+$", "", raw_name),
                "pid": int(match.group("pid")),
            }
        )

    counter_keys = (
        "assembled",
        "capacity_pruned",
        "superseded",
        "mailbox_submitted",
        "coalesced_drops",
        "received_images_min",
        "received_images_max",
    )
    queue_records = []
    for line in clean.splitlines():
        if "VAD frame queued:" not in line:
            continue
        record: dict[str, int] = {}
        for key in counter_keys:
            match = re.search(rf"\b{key}=(-?[0-9]+)", line)
            if match:
                record[key] = int(match.group(1))
        if record:
            queue_records.append(record)

    queue_counter_maxima = {
        key: max((record[key] for record in queue_records if key in record), default=None)
        for key in counter_keys
    }
    final_inference = inference_samples[-1] if inference_samples else None
    return {
        "inference_samples": inference_samples,
        "launch_processes": launch_processes,
        "queue_record_count": len(queue_records),
        "queue_counter_maxima": queue_counter_maxima,
        "final_counters": (
            {
                key: final_inference[key]
                for key in ("published_count", "mailbox_taken", "coalesced_drops")
            }
            if final_inference
            else None
        ),
    }


def detect_rtf_recovery(
    samples: Sequence[Mapping[str, Any]], minimum_side_count: int = 3
) -> dict[str, Any]:
    """Detect an abrupt wall-period collapse while source cadence stays fixed.

    A boundary is the first sample after the largest positive drop between two
    consecutive output periods.  It is accepted only when both sides contain
    enough samples and their median periods differ by at least 1.5x.
    """
    if len(samples) < minimum_side_count * 2:
        raise RuntimeLoadError("not enough VAD samples to detect two runtime phases")
    wall_periods = [
        float(samples[index + 1]["wall_epoch_sec"])
        - float(samples[index]["wall_epoch_sec"])
        for index in range(len(samples) - 1)
    ]
    source_periods = [
        float(samples[index + 1]["source_sim_sec"])
        - float(samples[index]["source_sim_sec"])
        for index in range(len(samples) - 1)
    ]
    candidates = []
    for period_index in range(1, len(wall_periods)):
        boundary_index = period_index + 1
        if boundary_index < minimum_side_count:
            continue
        if len(samples) - boundary_index < minimum_side_count:
            continue
        drop = wall_periods[period_index - 1] - wall_periods[period_index]
        candidates.append((drop, boundary_index, period_index))
    if not candidates:
        raise RuntimeLoadError("no eligible VAD phase boundary")
    drop, boundary_index, period_index = max(candidates, key=lambda item: item[0])

    initial_internal = wall_periods[: boundary_index - 1]
    recovered_internal = wall_periods[boundary_index:]
    initial_median = statistics.median(initial_internal)
    recovered_median = statistics.median(recovered_internal)
    ratio = initial_median / recovered_median if recovered_median > 0.0 else math.inf
    if drop <= 0.0 or ratio < 1.5:
        raise RuntimeLoadError(
            "VAD cadence has no distinct abrupt recovery (median wall-period ratio < 1.5)"
        )
    return {
        "boundary_index": boundary_index,
        "initial_last_index": boundary_index - 1,
        "recovered_first_index": boundary_index,
        "largest_period_drop_sec": drop,
        "cross_boundary_wall_period_sec": wall_periods[boundary_index - 1],
        "initial_median_wall_period_sec": initial_median,
        "recovered_median_wall_period_sec": recovered_median,
        "median_wall_period_ratio": ratio,
        "source_period_sec": numeric_summary(source_periods),
        "period_index_before_drop": period_index - 1,
        "period_index_after_drop": period_index,
    }


def _vad_phase_metrics(samples: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if len(samples) < 2:
        raise RuntimeLoadError("a VAD phase requires at least two samples")
    wall_periods = [
        float(samples[index + 1]["wall_epoch_sec"])
        - float(samples[index]["wall_epoch_sec"])
        for index in range(len(samples) - 1)
    ]
    source_periods = [
        float(samples[index + 1]["source_sim_sec"])
        - float(samples[index]["source_sim_sec"])
        for index in range(len(samples) - 1)
    ]
    interval_rtf = [
        source / wall
        for source, wall in zip(source_periods, wall_periods)
        if wall > 0.0
    ]
    wall_span = float(samples[-1]["wall_epoch_sec"]) - float(
        samples[0]["wall_epoch_sec"]
    )
    source_span = float(samples[-1]["source_sim_sec"]) - float(
        samples[0]["source_sim_sec"]
    )
    inference = [float(sample["inference_ms"]) for sample in samples]
    wall_rate = (len(samples) - 1) / wall_span if wall_span > 0.0 else None
    inference_duty = (
        statistics.fmean(inference) * wall_rate / 10.0 if wall_rate is not None else None
    )
    return {
        "sample_count": len(samples),
        "source_sim_start_sec": float(samples[0]["source_sim_sec"]),
        "source_sim_end_sec": float(samples[-1]["source_sim_sec"]),
        "source_sim_span_sec": source_span,
        "wall_start_epoch_sec": float(samples[0]["wall_epoch_sec"]),
        "wall_end_epoch_sec": float(samples[-1]["wall_epoch_sec"]),
        "wall_start_utc": _iso_from_epoch(float(samples[0]["wall_epoch_sec"])),
        "wall_end_utc": _iso_from_epoch(float(samples[-1]["wall_epoch_sec"])),
        "wall_span_sec": wall_span,
        "aggregate_rtf": source_span / wall_span if wall_span > 0.0 else None,
        "wall_output_rate_hz": wall_rate,
        "wall_output_period_sec": numeric_summary(wall_periods),
        "source_period_sec": numeric_summary(source_periods),
        "interval_rtf": numeric_summary(interval_rtf),
        "inference_ms": numeric_summary(inference),
        "approximate_inference_wall_duty_percent": inference_duty,
    }


def _rolling_rtf(
    samples: Sequence[Mapping[str, Any]], interval_count: int = 20
) -> list[float]:
    """Return aggregate RTF for overlapping, fixed-size interval windows."""
    if len(samples) < 2:
        return []
    width = min(interval_count, len(samples) - 1)
    values = []
    for start in range(0, len(samples) - width):
        end = start + width
        wall_span = float(samples[end]["wall_epoch_sec"]) - float(
            samples[start]["wall_epoch_sec"]
        )
        source_span = float(samples[end]["source_sim_sec"]) - float(
            samples[start]["source_sim_sec"]
        )
        if wall_span > 0.0:
            values.append(source_span / wall_span)
    return values


def build_vad_runtime(stack: Mapping[str, Any]) -> dict[str, Any]:
    samples = list(stack.get("inference_samples", []))
    if not samples:
        raise RuntimeLoadError("stack log contains no VAD inference completions")

    interval_rtf: list[float | None] = [None]
    for previous, current in zip(samples, samples[1:]):
        wall_delta = float(current["wall_epoch_sec"]) - float(previous["wall_epoch_sec"])
        source_delta = float(current["source_sim_sec"]) - float(previous["source_sim_sec"])
        interval_rtf.append(source_delta / wall_delta if wall_delta > 0.0 else None)

    full_run = _vad_phase_metrics(samples)
    finite_interval_rtf = [value for value in interval_rtf if value is not None]
    rolling_rtf = _rolling_rtf(samples)
    minimum_side_count = min(20, max(3, len(samples) // 3))
    try:
        recovery = detect_rtf_recovery(samples, minimum_side_count=minimum_side_count)
    except RuntimeLoadError as error:
        recovery = None
        no_recovery_reason = str(error)
    else:
        no_recovery_reason = None

    if recovery is not None:
        boundary_index = int(recovery["boundary_index"])
        initial_samples = samples[:boundary_index]
        recovered_samples = samples[boundary_index:]
        initial = _vad_phase_metrics(initial_samples)
        recovered = _vad_phase_metrics(recovered_samples)
        initial_last = initial_samples[-1]
        recovered_first = recovered_samples[0]
        phases = {
            "full_run": full_run,
            "initial_slow": initial,
            "recovered": recovered,
        }
        transition: dict[str, Any] | None = {
            "initial_last_source_sim_sec": float(initial_last["source_sim_sec"]),
            "recovered_first_source_sim_sec": float(recovered_first["source_sim_sec"]),
            "initial_last_wall_epoch_sec": float(initial_last["wall_epoch_sec"]),
            "recovered_first_wall_epoch_sec": float(recovered_first["wall_epoch_sec"]),
            "initial_last_wall_utc": _iso_from_epoch(float(initial_last["wall_epoch_sec"])),
            "recovered_first_wall_utc": _iso_from_epoch(
                float(recovered_first["wall_epoch_sec"])
            ),
            "host_phase_boundary_epoch_sec": float(recovered_first["wall_epoch_sec"]),
        }
        comparison = {
            "first_phase": "initial_slow",
            "second_phase": "recovered",
            "rtf_second_to_first_factor": recovered["aggregate_rtf"]
            / initial["aggregate_rtf"],
            "rtf_recovery_factor": recovered["aggregate_rtf"] / initial["aggregate_rtf"],
            "inference_mean_change_percent": 100.0
            * (
                recovered["inference_ms"]["mean"]
                / initial["inference_ms"]["mean"]
                - 1.0
            ),
            "wall_output_rate_factor": recovered["wall_output_rate_hz"]
            / initial["wall_output_rate_hz"],
        }
        runtime_pattern = "transient_recovery"
        phase_detection = {"detected": True, **recovery}
        boundary = {
            "kind": "observed_abrupt_recovery",
            "first_phase": "initial_slow",
            "second_phase": "recovered",
            "first_last_source_sim_sec": float(initial_last["source_sim_sec"]),
            "second_first_source_sim_sec": float(recovered_first["source_sim_sec"]),
            "host_boundary_epoch_sec": float(recovered_first["wall_epoch_sec"]),
        }
    else:
        midpoint = len(samples) // 2
        if midpoint < 2 or len(samples) - midpoint < 2:
            raise RuntimeLoadError("not enough VAD samples for persistent-run windows")
        early_samples = samples[:midpoint]
        late_samples = samples[midpoint:]
        early = _vad_phase_metrics(early_samples)
        late = _vad_phase_metrics(late_samples)
        early_last = early_samples[-1]
        late_first = late_samples[0]
        near_realtime_fraction = (
            sum(value >= 0.8 for value in finite_interval_rtf)
            / len(finite_interval_rtf)
            if finite_interval_rtf
            else None
        )
        rolling_p95 = numeric_summary(rolling_rtf).get("p95")
        persistently_low = bool(
            full_run["aggregate_rtf"] < 0.8
            and near_realtime_fraction is not None
            and near_realtime_fraction < 0.05
            and rolling_p95 is not None
            and rolling_p95 < 0.8
        )
        if persistently_low:
            runtime_pattern = "persistent_low_rtf_no_recovery"
        elif 0.8 <= full_run["aggregate_rtf"] <= 1.2:
            runtime_pattern = "steady_near_realtime_no_recovery"
        else:
            runtime_pattern = "no_distinct_recovery"
        phases = {
            "full_run": full_run,
            "early_window": early,
            "late_window": late,
        }
        transition = None
        comparison = {
            "first_phase": "early_window",
            "second_phase": "late_window",
            "rtf_second_to_first_factor": late["aggregate_rtf"]
            / early["aggregate_rtf"],
            "inference_mean_change_percent": 100.0
            * (late["inference_ms"]["mean"] / early["inference_ms"]["mean"] - 1.0),
            "wall_output_rate_factor": late["wall_output_rate_hz"]
            / early["wall_output_rate_hz"],
        }
        phase_detection = {
            "detected": False,
            "reason": no_recovery_reason,
            "classification": runtime_pattern,
            "persistent_low_threshold_rtf": 0.8,
            "interval_near_realtime_threshold_rtf": 0.8,
            "interval_near_realtime_fraction": near_realtime_fraction,
            "rolling_interval_count": min(20, len(samples) - 1),
            "rolling_rtf": numeric_summary(rolling_rtf),
        }
        boundary = {
            "kind": "descriptive_midpoint_not_a_recovery",
            "first_phase": "early_window",
            "second_phase": "late_window",
            "first_last_source_sim_sec": float(early_last["source_sim_sec"]),
            "second_first_source_sim_sec": float(late_first["source_sim_sec"]),
            "host_boundary_epoch_sec": float(late_first["wall_epoch_sec"]),
        }

    return {
        "sample_count": len(samples),
        "runtime_pattern": runtime_pattern,
        "phase_detection": phase_detection,
        "transition": transition,
        "analysis_window_boundary": boundary,
        "comparison_phase_names": [comparison["first_phase"], comparison["second_phase"]],
        "phases": phases,
        "comparison": comparison,
        "integrity": {
            "all_published": all(bool(sample["published"]) for sample in samples),
            "final_counters": stack.get("final_counters"),
            "queue_counter_maxima": stack.get("queue_counter_maxima"),
        },
        "series": {
            "source_sim_sec": [float(sample["source_sim_sec"]) for sample in samples],
            "wall_epoch_sec": [float(sample["wall_epoch_sec"]) for sample in samples],
            "interval_rtf": interval_rtf,
            "inference_ms": [float(sample["inference_ms"]) for sample in samples],
            "rolling_rtf": rolling_rtf,
        },
    }


def _local_epoch(date_text: str, time_text: str, zone: ZoneInfo) -> float:
    return datetime.fromisoformat(f"{date_text}T{time_text}").replace(tzinfo=zone).timestamp()


def parse_vmstat(text: str, timezone_name: str = "Asia/Seoul") -> list[dict[str, Any]]:
    zone = ZoneInfo(timezone_name)
    samples = []
    for line in text.splitlines():
        tokens = line.split()
        if len(tokens) != len(VMSTAT_FIELDS) + 2:
            continue
        if not re.fullmatch(r"[0-9]{4}-[0-9]{2}-[0-9]{2}", tokens[-2]):
            continue
        try:
            values = [float(token) for token in tokens[: len(VMSTAT_FIELDS)]]
        except ValueError:
            continue
        record = dict(zip(VMSTAT_FIELDS, values))
        record["epoch_sec"] = _local_epoch(tokens[-2], tokens[-1], zone)
        record["local_timestamp"] = f"{tokens[-2]}T{tokens[-1]}"
        record["cpu_busy_percent"] = (
            record["cpu_user_percent"] + record["cpu_system_percent"]
        )
        samples.append(record)
    return samples


def _dmon_number(value: str) -> float | None:
    if value == "-":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_nvidia_dmon(
    text: str, timezone_name: str = "Asia/Seoul"
) -> list[dict[str, Any]]:
    zone = ZoneInfo(timezone_name)
    samples = []
    for line in text.splitlines():
        tokens = line.split()
        if len(tokens) != len(NVIDIA_DMON_FIELDS) + 2:
            continue
        if not re.fullmatch(r"[0-9]{8}", tokens[0]):
            continue
        date_text = f"{tokens[0][0:4]}-{tokens[0][4:6]}-{tokens[0][6:8]}"
        values = [_dmon_number(value) for value in tokens[2:]]
        record = dict(zip(NVIDIA_DMON_FIELDS, values))
        record["epoch_sec"] = _local_epoch(date_text, tokens[1], zone)
        record["local_timestamp"] = f"{date_text}T{tokens[1]}"
        samples.append(record)
    return samples


def parse_pidstat(
    text: str, timezone_name: str = "Asia/Seoul", date_hint: date | None = None
) -> dict[str, Any]:
    zone = ZoneInfo(timezone_name)
    cpu_match = re.search(r"\(([0-9]+) CPU\)", text)
    logical_cpu_count = int(cpu_match.group(1)) if cpu_match else None
    date_match = re.search(r"\b([0-9]{2}/[0-9]{2}/[0-9]{2})\b", text)
    if date_match:
        sample_date = datetime.strptime(date_match.group(1), "%m/%d/%y").date()
    elif date_hint is not None:
        sample_date = date_hint
    else:
        raise RuntimeLoadError("pidstat date is unavailable")

    samples = []
    date_text = sample_date.isoformat()
    for line in text.splitlines():
        tokens = line.split()
        if len(tokens) < 19 or not re.fullmatch(r"[0-9]{2}:[0-9]{2}:[0-9]{2}", tokens[0]):
            continue
        try:
            record = {
                "epoch_sec": _local_epoch(date_text, tokens[0], zone),
                "local_timestamp": f"{date_text}T{tokens[0]}",
                "uid": int(tokens[1]),
                "pid": int(tokens[2]),
                "user_cpu_percent": float(tokens[3]),
                "system_cpu_percent": float(tokens[4]),
                "guest_cpu_percent": float(tokens[5]),
                "wait_percent": float(tokens[6]),
                "cpu_percent": float(tokens[7]),
                "cpu_index": int(tokens[8]),
                "minor_faults_s": float(tokens[9]),
                "major_faults_s": float(tokens[10]),
                "vsz_kib": float(tokens[11]),
                "rss_kib": float(tokens[12]),
                "memory_percent": float(tokens[13]),
                "read_kib_s": float(tokens[14]),
                "write_kib_s": float(tokens[15]),
                "cancelled_write_kib_s": float(tokens[16]),
                "io_delay": float(tokens[17]),
                "command": " ".join(tokens[18:]),
            }
        except ValueError:
            continue
        samples.append(record)
    return {
        "logical_cpu_count": logical_cpu_count,
        "sample_date": date_text,
        "samples": samples,
    }


def parse_recorder_log(text: str) -> dict[str, Any]:
    clean = _strip_ansi(text)
    topics = RECORDER_TOPIC_RE.findall(clean)
    recording_match = re.search(
        r"\[INFO\s+([0-9]+\.[0-9]+)\].*?Recording\.\.\.", clean
    )
    stopped_match = re.search(
        r"\[INFO\s+([0-9]+\.[0-9]+)\].*?Recording stopped", clean
    )
    warnings = [line for line in clean.splitlines() if "[WARN" in line or "[ERROR" in line]
    return {
        "recording_start_epoch_sec": (
            float(recording_match.group(1)) if recording_match else None
        ),
        "recording_stop_epoch_sec": float(stopped_match.group(1)) if stopped_match else None,
        "subscribed_topics": topics,
        "subscribed_topic_count": len(topics),
        "camera_info_topics_subscribed": [
            topic for topic in CAMERA_INFO_TOPICS if topic in topics
        ],
        "camera_info_topic_count": sum(topic in topics for topic in CAMERA_INFO_TOPICS),
        "warning_or_error_lines": warnings,
    }


def read_camera_info_bag(bag: Path) -> dict[str, list[dict[str, Any]]]:
    """Read only the six lightweight CameraInfo topics from a ROS 2 bag."""
    try:
        import rosbag2_py
        import yaml
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:  # pragma: no cover - requires sourced ROS
        raise RuntimeLoadError(
            "ROS 2 Python modules are unavailable; source scripts/e2e/env.sh"
        ) from error

    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeLoadError(f"rosbag metadata does not exist: {metadata_path}")
    with metadata_path.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    information = metadata.get("rosbag2_bagfile_information", {})
    storage_id = information.get("storage_identifier")
    if not storage_id:
        raise RuntimeLoadError(f"storage_identifier missing from {metadata_path}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=str(storage_id)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in CAMERA_INFO_TOPICS if topic not in topic_types]
    if missing:
        raise RuntimeLoadError(f"six-camera CameraInfo evidence is incomplete: {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(CAMERA_INFO_TOPICS)))
    message_classes = {
        topic: get_message(topic_types[topic]) for topic in CAMERA_INFO_TOPICS
    }
    records: dict[str, list[dict[str, Any]]] = {
        topic: [] for topic in CAMERA_INFO_TOPICS
    }
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if topic not in message_classes:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        records[topic].append(
            {"bag_ns": int(bag_ns), "stamp_ns": stamp_ns, "topic": topic}
        )
    return records


def build_same_stamp_camera_bundles(
    records: Mapping[str, Sequence[Mapping[str, Any]]]
) -> list[dict[str, Any]]:
    """Join six cameras by exact simulation stamp, never by wall time."""
    by_topic: dict[str, dict[int, Mapping[str, Any]]] = {}
    for topic in CAMERA_INFO_TOPICS:
        topic_records = records.get(topic, [])
        if not topic_records:
            raise RuntimeLoadError(f"camera topic has no records: {topic}")
        by_topic[topic] = {
            int(record["stamp_ns"]): record
            for record in topic_records
            if int(record["stamp_ns"]) > 0
        }
    common_stamps = set.intersection(*(set(values) for values in by_topic.values()))
    bundles = []
    for stamp_ns in sorted(common_stamps):
        receipts = {
            topic: int(by_topic[topic][stamp_ns]["bag_ns"])
            for topic in CAMERA_INFO_TOPICS
        }
        earliest = min(receipts.values())
        latest = max(receipts.values())
        bundles.append(
            {
                "source_stamp_ns": stamp_ns,
                "source_sim_sec": stamp_ns * 1.0e-9,
                "earliest_receipt_ns": earliest,
                "latest_receipt_ns": latest,
                "receipt_span_ms": (latest - earliest) * 1.0e-6,
                "earliest_camera": min(receipts, key=receipts.get),
                "receipt_offset_ms": {
                    topic.removeprefix("/sensing/camera/").removesuffix("/camera_info"): (
                        receipt - earliest
                    )
                    * 1.0e-6
                    for topic, receipt in receipts.items()
                },
            }
        )
    return bundles


def _camera_bundle_summary(bundles: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    offsets: dict[str, dict[str, Any]] = {}
    for camera in CAMERAS:
        offsets[camera] = numeric_summary(
            bundle["receipt_offset_ms"][camera] for bundle in bundles
        )
    return {
        "bundle_count": len(bundles),
        "source_sim_start_sec": float(bundles[0]["source_sim_sec"]) if bundles else None,
        "source_sim_end_sec": float(bundles[-1]["source_sim_sec"]) if bundles else None,
        "receipt_span_ms": numeric_summary(
            float(bundle["receipt_span_ms"]) for bundle in bundles
        ),
        "receipt_offset_ms_by_camera": offsets,
    }


def build_camera_runtime(
    records: Mapping[str, Sequence[Mapping[str, Any]]],
    vad_runtime: Mapping[str, Any],
) -> dict[str, Any]:
    bundles = build_same_stamp_camera_bundles(records)
    if not bundles:
        raise RuntimeLoadError("no exact same-stamp six-camera bundles")
    boundary = vad_runtime["analysis_window_boundary"]
    first_edge = float(boundary["first_last_source_sim_sec"])
    second_start = float(boundary["second_first_source_sim_sec"])
    tolerance = 1.0e-6
    transient = vad_runtime["runtime_pattern"] == "transient_recovery"
    if transient:
        # The last slow VAD stamp is kept as a one-frame transition guard for
        # the camera comparison. This preserves the original clean-window
        # 60-kph result (647.112 -> 9.458 ms) and makes the exclusion explicit.
        first_bundles = [
            bundle
            for bundle in bundles
            if float(bundle["source_sim_sec"]) < first_edge - tolerance
        ]
        first_including_edge = [
            bundle
            for bundle in bundles
            if float(bundle["source_sim_sec"]) <= first_edge + tolerance
        ]
        second_bundles = [
            bundle
            for bundle in bundles
            if float(bundle["source_sim_sec"]) >= second_start - tolerance
        ]
        first_camera_phase = "initial_slow_clean"
        second_camera_phase = "recovered_clean"
        clean_phase_policy = (
            "initial excludes the last slow VAD source stamp as a one-frame transition "
            "guard; recovered begins at the first recovered VAD source stamp"
        )
    else:
        first_bundles = [
            bundle
            for bundle in bundles
            if float(bundle["source_sim_sec"]) <= first_edge + tolerance
        ]
        first_including_edge = first_bundles
        second_bundles = [
            bundle
            for bundle in bundles
            if float(bundle["source_sim_sec"]) >= second_start - tolerance
        ]
        first_camera_phase = "early_window"
        second_camera_phase = "late_window"
        clean_phase_policy = (
            "no recovery was detected; early/late camera windows follow the descriptive "
            "VAD midpoint and must not be interpreted as causal phases"
        )
    boundary_guard = [
        bundle
        for bundle in bundles
        if bundle not in first_bundles and bundle not in second_bundles
    ]
    if not first_bundles or not second_bundles:
        raise RuntimeLoadError("camera bundles do not cover both detected VAD phases")

    source_periods = [
        float(current["source_sim_sec"]) - float(previous["source_sim_sec"])
        for previous, current in zip(bundles, bundles[1:])
    ]
    first_summary = _camera_bundle_summary(first_bundles)
    second_summary = _camera_bundle_summary(second_bundles)
    phases = {
        "full_run": _camera_bundle_summary(bundles),
        first_camera_phase: first_summary,
        second_camera_phase: second_summary,
    }
    if transient:
        phases["initial_slow_including_edge"] = _camera_bundle_summary(
            first_including_edge
        )
    comparison = {
        "first_phase": first_camera_phase,
        "second_phase": second_camera_phase,
        "mean_receipt_span_first_to_second_ratio": first_summary["receipt_span_ms"][
            "mean"
        ]
        / second_summary["receipt_span_ms"]["mean"],
        "mean_receipt_span_change_ms": second_summary["receipt_span_ms"]["mean"]
        - first_summary["receipt_span_ms"]["mean"],
    }
    if transient:
        comparison["mean_receipt_span_reduction_factor"] = comparison[
            "mean_receipt_span_first_to_second_ratio"
        ]
    return {
        "matching_policy": "exact_equal_source_stamp_ns_across_all_six_camera_info_topics",
        "clock_domains": {
            "bundle_key": "ROS/CARLA simulation stamp",
            "receipt_span": "rosbag recorder receipt/system time",
            "cross_domain_subtraction_used": False,
        },
        "camera_count": len(CAMERA_INFO_TOPICS),
        "front_record_count": len(records.get(CAMERA_INFO_TOPICS[0], [])),
        "matched_bundle_count": len(bundles),
        "bundle_coverage_percent": 100.0
        * len(bundles)
        / len(records.get(CAMERA_INFO_TOPICS[0], [])),
        "source_period_sec": numeric_summary(source_periods),
        "source_rate_hz_from_median_period": (
            1.0 / statistics.median(source_periods) if source_periods else None
        ),
        "runtime_pattern": vad_runtime["runtime_pattern"],
        "clean_phase_policy": clean_phase_policy,
        "comparison_phase_names": [first_camera_phase, second_camera_phase],
        "phases": phases,
        "transition_guard": {
            "bundle_count": len(boundary_guard),
            "bundles": boundary_guard,
            "guard_kind": (
                "observed_transition_guard"
                if transient
                else "descriptive_midpoint_gap"
            ),
        },
        "comparison": comparison,
        "series": {
            "source_sim_sec": [float(bundle["source_sim_sec"]) for bundle in bundles],
            "receipt_span_ms": [float(bundle["receipt_span_ms"]) for bundle in bundles],
        },
        "interpretation_boundary": (
            "camera_info is a lightweight timestamp proxy. The span localizes a serial "
            "six-camera delivery delay, but cannot by itself distinguish image conversion, "
            "reliable DDS backpressure, worker scheduling, CARLA rendering, or asset streaming."
        ),
    }


def _phase_rows(
    rows: Sequence[Mapping[str, Any]], start: float, boundary: float, end: float
) -> tuple[list[Mapping[str, Any]], list[Mapping[str, Any]]]:
    initial = [row for row in rows if start <= float(row["epoch_sec"]) < boundary]
    recovered = [row for row in rows if boundary <= float(row["epoch_sec"]) <= end]
    return initial, recovered


def _vmstat_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "sample_count": len(rows),
        "cpu": {
            "user_percent": numeric_summary(row["cpu_user_percent"] for row in rows),
            "system_percent": numeric_summary(row["cpu_system_percent"] for row in rows),
            "busy_user_plus_system_percent": numeric_summary(
                row["cpu_busy_percent"] for row in rows
            ),
            "idle_percent": numeric_summary(row["cpu_idle_percent"] for row in rows),
            "iowait_percent": numeric_summary(row["cpu_iowait_percent"] for row in rows),
            "steal_percent": numeric_summary(row["cpu_steal_percent"] for row in rows),
        },
        "scheduler": {
            "runnable": numeric_summary(row["runnable"] for row in rows),
            "blocked": numeric_summary(row["blocked"] for row in rows),
        },
        "io_kib_s": {
            "block_in": numeric_summary(row["block_in_kib_s"] for row in rows),
            "block_out": numeric_summary(row["block_out_kib_s"] for row in rows),
        },
        "memory_mib": {
            "free": numeric_summary(row["free_kib"] / 1024.0 for row in rows),
            "cache": numeric_summary(row["cache_kib"] / 1024.0 for row in rows),
            "swap_used": numeric_summary(row["swap_used_kib"] / 1024.0 for row in rows),
        },
        "swap_io_kib_s": {
            "in": numeric_summary(row["swap_in_kib_s"] for row in rows),
            "out": numeric_summary(row["swap_out_kib_s"] for row in rows),
        },
    }


def _gpu_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    keys = (
        "power_w",
        "gpu_temp_c",
        "sm_percent",
        "memory_percent",
        "framebuffer_mib",
        "rx_pcie_mib_s",
        "tx_pcie_mib_s",
        "power_violation_percent",
        "thermal_violation",
    )
    return {
        "sample_count": len(rows),
        **{key: numeric_summary(row.get(key) for row in rows) for key in keys},
    }


def _launch_pids(
    launch_processes: Sequence[Mapping[str, Any]], prefix: str
) -> set[int]:
    return {
        int(process["pid"])
        for process in launch_processes
        if str(process["node_name"]).startswith(prefix)
    }


def _choose_recorder_pid(
    rows: Sequence[Mapping[str, Any]], excluded_pids: set[int]
) -> set[int]:
    candidates: dict[int, list[float]] = defaultdict(list)
    for row in rows:
        command = str(row["command"])
        pid = int(row["pid"])
        if pid in excluded_pids:
            continue
        if command in {"ros2", "record", "rosbag2_recorder"}:
            candidates[pid].append(float(row["cpu_percent"]))
    if not candidates:
        return set()
    selected = max(
        candidates,
        key=lambda pid: (statistics.fmean(candidates[pid]), len(candidates[pid])),
    )
    return {selected}


def classify_process_groups(
    pidstat_rows: Sequence[Mapping[str, Any]],
    launch_processes: Sequence[Mapping[str, Any]],
) -> dict[str, set[int]]:
    launch_owned = {int(process["pid"]) for process in launch_processes}
    commands: dict[int, str] = {}
    for row in pidstat_rows:
        commands[int(row["pid"])] = str(row["command"])
    groups = {
        "carla_server": {
            pid for pid, command in commands.items() if command.startswith("CarlaUE4")
        },
        "carla_bridge": _launch_pids(launch_processes, "autoware_carla_interface"),
        "vad_inference": _launch_pids(launch_processes, "vad_node"),
        "rviz": _launch_pids(launch_processes, "rviz2"),
        "route_manager": _launch_pids(launch_processes, "vad_route_manager.py"),
        "autoware_containers": _launch_pids(launch_processes, "component_container"),
        "desktop_capture": {
            pid for pid, command in commands.items() if command == "ffmpeg"
        },
        "vscode": {
            pid
            for pid, command in commands.items()
            if command in {"code", "code-insiders"}
        },
        "desktop_shell": {
            pid
            for pid, command in commands.items()
            if command in {"Xorg", "Xwayland", "gnome-shell"}
        },
    }
    groups["rosbag_recorder"] = _choose_recorder_pid(pidstat_rows, launch_owned)
    groups["carla_total"] = groups["carla_server"] | groups["carla_bridge"]
    return groups


def _process_group_summary(
    rows: Sequence[Mapping[str, Any]], pids: set[int], host_core_percent: float | None
) -> dict[str, Any]:
    selected = [row for row in rows if int(row["pid"]) in pids]
    if not selected:
        return {"available": False, "sample_count": 0, "pids": sorted(pids)}
    per_second: dict[float, dict[str, float]] = defaultdict(
        lambda: {
            "cpu_percent": 0.0,
            "wait_percent": 0.0,
            "rss_kib": 0.0,
            "read_kib_s": 0.0,
            "write_kib_s": 0.0,
        }
    )
    for row in selected:
        bucket = per_second[float(row["epoch_sec"])]
        for key in bucket:
            bucket[key] += float(row[key])
    cpu = numeric_summary(bucket["cpu_percent"] for bucket in per_second.values())
    return {
        "available": True,
        "sample_count": len(per_second),
        "pids": sorted({int(row["pid"]) for row in selected}),
        "commands": sorted({str(row["command"]) for row in selected}),
        "cpu_percent_of_one_logical_core": cpu,
        "wait_percent": numeric_summary(
            bucket["wait_percent"] for bucket in per_second.values()
        ),
        "rss_mib": numeric_summary(
            bucket["rss_kib"] / 1024.0 for bucket in per_second.values()
        ),
        "read_kib_s": numeric_summary(
            bucket["read_kib_s"] for bucket in per_second.values()
        ),
        "write_kib_s": numeric_summary(
            bucket["write_kib_s"] for bucket in per_second.values()
        ),
        "host_busy_cpu_share_percent": (
            100.0 * cpu["mean"] / host_core_percent
            if host_core_percent is not None and host_core_percent > 0.0
            else None
        ),
    }


def build_host_runtime(
    vmstat_rows: Sequence[Mapping[str, Any]],
    gpu_rows: Sequence[Mapping[str, Any]],
    pidstat: Mapping[str, Any],
    launch_processes: Sequence[Mapping[str, Any]],
    trial_start_epoch: float,
    window_boundary_epoch: float,
    trial_end_epoch: float,
    first_phase_name: str = "initial_slow",
    second_phase_name: str = "recovered",
    boundary_kind: str = "observed_abrupt_recovery",
) -> dict[str, Any]:
    first_vm, second_vm = _phase_rows(
        vmstat_rows, trial_start_epoch, window_boundary_epoch, trial_end_epoch
    )
    first_gpu, second_gpu = _phase_rows(
        gpu_rows, trial_start_epoch, window_boundary_epoch, trial_end_epoch
    )
    pid_rows = list(pidstat.get("samples", []))
    first_pid, second_pid = _phase_rows(
        pid_rows, trial_start_epoch, window_boundary_epoch, trial_end_epoch
    )
    if not first_vm or not second_vm:
        raise RuntimeLoadError("vmstat does not cover both analysis windows")
    if not first_gpu or not second_gpu:
        raise RuntimeLoadError("NVIDIA dmon does not cover both analysis windows")

    vm_phases = {
        "full_trial": _vmstat_summary(first_vm + second_vm),
        first_phase_name: _vmstat_summary(first_vm),
        second_phase_name: _vmstat_summary(second_vm),
    }
    gpu_phases = {
        "full_trial": _gpu_summary(first_gpu + second_gpu),
        first_phase_name: _gpu_summary(first_gpu),
        second_phase_name: _gpu_summary(second_gpu),
    }
    logical_cpu_count = pidstat.get("logical_cpu_count")
    groups = classify_process_groups(first_pid + second_pid, launch_processes)
    process_groups: dict[str, Any] = {}
    for name, pids in groups.items():
        phase_payload = {}
        for phase_name, rows in (
            (first_phase_name, first_pid),
            (second_phase_name, second_pid),
        ):
            busy_mean = vm_phases[phase_name]["cpu"][
                "busy_user_plus_system_percent"
            ]["mean"]
            host_core_percent = (
                busy_mean * int(logical_cpu_count)
                if logical_cpu_count is not None
                else None
            )
            phase_payload[phase_name] = _process_group_summary(
                rows, pids, host_core_percent
            )
        present_in_both_windows = all(
            phase_payload[phase]["available"]
            for phase in (first_phase_name, second_phase_name)
        )
        process_groups[name] = {
            "pids": sorted(pids),
            "phases": phase_payload,
            "present_in_both_windows": present_in_both_windows,
            "present_on_both_sides": present_in_both_windows,
        }

    first_busy = vm_phases[first_phase_name]["cpu"][
        "busy_user_plus_system_percent"
    ]["mean"]
    second_busy = vm_phases[second_phase_name]["cpu"][
        "busy_user_plus_system_percent"
    ]["mean"]
    first_sm = gpu_phases[first_phase_name]["sm_percent"]["mean"]
    second_sm = gpu_phases[second_phase_name]["sm_percent"]["mean"]
    comparison = {
        "first_phase": first_phase_name,
        "second_phase": second_phase_name,
        "cpu_busy_mean_change_percentage_points": second_busy - first_busy,
        "gpu_sm_mean_change_percentage_points": second_sm - first_sm,
        "cpu_load_increased_in_second_window": second_busy > first_busy,
        "gpu_load_increased_in_second_window": second_sm > first_sm,
    }
    if boundary_kind == "observed_abrupt_recovery":
        comparison["cpu_load_increased_after_recovery"] = second_busy > first_busy
        comparison["gpu_load_increased_after_recovery"] = second_sm > first_sm
    return {
        "phase_window_policy": {
            "boundary_kind": boundary_kind,
            first_phase_name: (
                "result.started_at <= telemetry timestamp < analysis-window boundary"
            ),
            second_phase_name: (
                "analysis-window boundary <= telemetry timestamp <= result.finished_at"
            ),
        },
        "comparison_phase_names": [first_phase_name, second_phase_name],
        "logical_cpu_count": logical_cpu_count,
        "vmstat": {"phases": vm_phases},
        "gpu_device_total": {
            "phases": gpu_phases,
            "per_process_attribution_available": False,
            "boundary": (
                "nvidia-smi dmon records device totals only; CARLA, VAD, and RViz GPU "
                "shares cannot be separated from these inputs"
            ),
        },
        "process_groups": process_groups,
        "comparison": comparison,
    }


def _nearest_path_point(
    actual_path: Sequence[Mapping[str, Any]], source_sim_sec: float
) -> dict[str, Any] | None:
    if not actual_path:
        return None
    best_index = min(
        range(len(actual_path)),
        key=lambda index: abs(float(actual_path[index]["sim_time_sec"]) - source_sim_sec),
    )
    traveled = 0.0
    for previous, current in zip(actual_path[:best_index], actual_path[1 : best_index + 1]):
        traveled += math.hypot(
            float(current["x"]) - float(previous["x"]),
            float(current["y"]) - float(previous["y"]),
        )
    point = actual_path[best_index]
    return {
        "sample_index": best_index,
        "sim_time_sec": float(point["sim_time_sec"]),
        "x": float(point["x"]),
        "y": float(point["y"]),
        "speed_mps": float(point.get("speed_mps", 0.0)),
        "traveled_from_first_recorded_path_point_m": traveled,
        "absolute_sim_time_error_sec": abs(
            float(point["sim_time_sec"]) - source_sim_sec
        ),
    }


def _latency_crosscheck(
    latency: Mapping[str, Any], camera_runtime: Mapping[str, Any]
) -> dict[str, Any]:
    camera_bundle = latency.get("camera_bundle", {})
    matched = camera_bundle.get("matched_bundle_count")
    bag_matched = camera_runtime["matched_bundle_count"]
    return {
        "latency_camera_bundle": camera_bundle,
        "candidate_front_acceptance": latency.get("candidate_front_acceptance"),
        "clock_domains": latency.get("clock_domains"),
        "data_quality_warnings": latency.get("data_quality_warnings", []),
        "matched_bundle_count_agrees": matched == bag_matched,
        "bag_recomputed_matched_bundle_count": bag_matched,
    }


def _classify_runtime_finding(
    runtime_pattern: str,
    camera_pattern_supported: bool,
    no_host_wide_saturation: bool,
) -> tuple[str, dict[str, bool]]:
    """Return a classification whose positive claims are evidence-backed.

    The runtime pattern itself comes from the VAD clock reconstruction.  A
    camera-delivery claim and a host-wide-saturation exclusion are added only
    when their independent thresholds pass.  Keeping the claims explicit lets
    downstream gates reject stale reports instead of trusting prose alone.
    """
    claims = {
        "camera_delivery_pattern": camera_pattern_supported,
        "host_wide_saturation_excluded": no_host_wide_saturation,
    }
    if runtime_pattern == "transient_recovery":
        if camera_pattern_supported and no_host_wide_saturation:
            classification = (
                "transient_camera_delivery_path_stall_not_host_wide_saturation"
            )
        elif camera_pattern_supported:
            classification = (
                "transient_camera_delivery_pattern_"
                "host_wide_saturation_not_excluded"
            )
        elif no_host_wide_saturation:
            classification = (
                "transient_rtf_recovery_without_supported_camera_delivery_pattern_"
                "not_host_wide_saturation"
            )
        else:
            classification = (
                "transient_rtf_recovery_without_supported_cause_attribution"
            )
    elif runtime_pattern == "persistent_low_rtf_no_recovery":
        if camera_pattern_supported and no_host_wide_saturation:
            classification = (
                "persistent_camera_delivery_path_stall_without_recovery_"
                "not_host_wide_saturation"
            )
        elif camera_pattern_supported:
            classification = (
                "persistent_camera_delivery_pattern_without_recovery_"
                "host_wide_saturation_not_excluded"
            )
        elif no_host_wide_saturation:
            classification = (
                "persistent_low_rtf_without_supported_camera_delivery_pattern_"
                "not_host_wide_saturation"
            )
        else:
            classification = (
                "persistent_low_rtf_without_supported_cause_attribution"
            )
    elif no_host_wide_saturation:
        classification = f"{runtime_pattern}_not_host_wide_saturation"
        claims["camera_delivery_pattern"] = False
    else:
        classification = runtime_pattern
        claims = {
            "camera_delivery_pattern": False,
            "host_wide_saturation_excluded": False,
        }
    return classification, claims


def build_runtime_load_analysis(
    *,
    result: Mapping[str, Any],
    latency: Mapping[str, Any],
    stack: Mapping[str, Any],
    recorder: Mapping[str, Any],
    camera_records: Mapping[str, Sequence[Mapping[str, Any]]],
    vmstat_rows: Sequence[Mapping[str, Any]],
    gpu_rows: Sequence[Mapping[str, Any]],
    pidstat: Mapping[str, Any],
    input_manifest: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a report from already parsed fixture-friendly evidence."""
    vad_runtime = build_vad_runtime(stack)
    camera_runtime = build_camera_runtime(camera_records, vad_runtime)
    trial_start = _parse_iso_timestamp(str(result["started_at"]))
    trial_end = _parse_iso_timestamp(str(result["finished_at"]))
    boundary = vad_runtime["analysis_window_boundary"]
    first_phase, second_phase = vad_runtime["comparison_phase_names"]
    host_runtime = build_host_runtime(
        vmstat_rows,
        gpu_rows,
        pidstat,
        stack.get("launch_processes", []),
        trial_start,
        boundary["host_boundary_epoch_sec"],
        trial_end,
        first_phase,
        second_phase,
        boundary["kind"],
    )

    metrics = result.get("metrics", {})
    overall_rtf = None
    if float(metrics.get("wall_elapsed_sec", 0.0)) > 0.0:
        overall_rtf = float(metrics["sim_elapsed_sec"]) / float(metrics["wall_elapsed_sec"])
    first_location = _nearest_path_point(
        result.get("actual_path", []), boundary["first_last_source_sim_sec"]
    )
    second_location = _nearest_path_point(
        result.get("actual_path", []), boundary["second_first_source_sim_sec"]
    )

    first_vad = vad_runtime["phases"][first_phase]
    second_vad = vad_runtime["phases"][second_phase]
    full_vad = vad_runtime["phases"]["full_run"]
    first_camera_phase, second_camera_phase = camera_runtime[
        "comparison_phase_names"
    ]
    first_camera = camera_runtime["phases"][first_camera_phase]
    second_camera = camera_runtime["phases"][second_camera_phase]
    first_host_cpu = host_runtime["vmstat"]["phases"][first_phase]["cpu"]
    second_host_cpu = host_runtime["vmstat"]["phases"][second_phase]["cpu"]
    full_host = host_runtime["vmstat"]["phases"]["full_trial"]
    first_gpu = host_runtime["gpu_device_total"]["phases"][first_phase]
    second_gpu = host_runtime["gpu_device_total"]["phases"][second_phase]
    full_gpu = host_runtime["gpu_device_total"]["phases"]["full_trial"]
    drops = vad_runtime["integrity"]

    camera_source_stable = (
        camera_runtime["source_period_sec"]["max"] < 0.201
        and camera_runtime["source_period_sec"]["min"] > 0.199
    )
    no_vad_drops = bool(
        drops.get("final_counters")
        and drops["final_counters"].get("coalesced_drops") == 0
        and (drops.get("queue_counter_maxima", {}).get("capacity_pruned") in (0, None))
        and (drops.get("queue_counter_maxima", {}).get("superseded") in (0, None))
    )
    source_period_ms = 1000.0 * camera_runtime["source_period_sec"]["median"]
    inference_below_source_period = full_vad["inference_ms"]["p99"] < source_period_ms
    no_host_wide_saturation = bool(
        full_host["cpu"]["busy_user_plus_system_percent"]["p95"] < 90.0
        and full_host["cpu"]["iowait_percent"]["max"] < 5.0
        and full_host["memory_mib"]["swap_used"]["max"] == 0.0
        and full_host["swap_io_kib_s"]["in"]["max"] == 0.0
        and full_host["swap_io_kib_s"]["out"]["max"] == 0.0
        and full_gpu["sm_percent"]["p95"] < 95.0
        and full_gpu["thermal_violation"]["max"] == 0.0
    )

    transient = vad_runtime["runtime_pattern"] == "transient_recovery"
    persistent_low = (
        vad_runtime["runtime_pattern"] == "persistent_low_rtf_no_recovery"
    )
    if transient:
        camera_pattern_supported = (
            first_camera["receipt_span_ms"]["mean"]
            > 2.0 * second_camera["receipt_span_ms"]["mean"]
        )
        camera_pattern = (
            "high_receipt_span_collapses_at_abrupt_rtf_recovery"
            if camera_pattern_supported
            else "abrupt_rtf_recovery_without_supported_receipt_span_collapse"
        )
        causal_status = "warm_up_and_scene_boundary_are_confounded"
        causal_reason = (
            "In this single moving run, elapsed warm-up time and vehicle position change "
            "together at the same transition. The evidence does not experimentally "
            "separate those triggers."
        )
    elif persistent_low:
        camera_pattern_supported = bool(
            camera_runtime["phases"]["full_run"]["receipt_span_ms"]["median"]
            > source_period_ms
            and second_camera["receipt_span_ms"]["median"] > source_period_ms
        )
        camera_pattern = (
            "persistent_high_receipt_span_without_recovery"
            if camera_pattern_supported
            else "persistent_low_rtf_without_supported_high_receipt_span"
        )
        causal_status = "persistent_low_rtf_has_no_transition_to_localize"
        causal_reason = (
            "No recovery occurs in this run, so there is no causal transition at which to "
            "separate elapsed warm-up, route position, DDS state, rendering, or asset state. "
            "The early/late split is descriptive only."
        )
    else:
        camera_pattern = "no_abrupt_recovery_for_camera_comparison"
        camera_pattern_supported = False
        causal_status = "no_observed_recovery"
        causal_reason = (
            "The analyzer found no abrupt recovery; early/late windows are descriptive and "
            "cannot identify a trigger."
        )

    finding_classification, classification_claims = _classify_runtime_finding(
        vad_runtime["runtime_pattern"],
        camera_pattern_supported,
        no_host_wide_saturation,
    )

    if camera_source_stable and no_vad_drops:
        camera_hz_explanation = (
            "The source stays at five simulation-Hz without observed VAD drops. A five-fps "
            "display can look mildly stepped at RTF~=1; wall-visible cadence is source "
            "cadence multiplied by RTF."
        )
    else:
        camera_hz_explanation = (
            "The available cadence/drop evidence does not support excluding the camera "
            "source as an RTF contributor."
        )
    if persistent_low and camera_source_stable and no_vad_drops:
        camera_hz_explanation += (
            " Here the persistent RTF~=0.248 makes the five sim-Hz source appear at about "
            "1.24 wall-Hz, but source cadence itself remains complete and stable."
        )

    camera_pattern_finding = {
        "supported": camera_pattern_supported,
        "classification": camera_pattern,
        "first_phase": first_camera_phase,
        "second_phase": second_camera_phase,
        "first_mean_receipt_span_ms": first_camera["receipt_span_ms"]["mean"],
        "second_mean_receipt_span_ms": second_camera["receipt_span_ms"]["mean"],
        "full_run_mean_receipt_span_ms": camera_runtime["phases"]["full_run"]
        ["receipt_span_ms"]["mean"],
        "first_to_second_ratio": camera_runtime["comparison"][
            "mean_receipt_span_first_to_second_ratio"
        ],
    }
    host_explanation = (
        "CPU, GPU, iowait, swap, and thermal evidence do not show host-wide exhaustion. "
        "A localized CARLA game/render-thread, render-fence, conversion, or publish wait "
        "is still possible."
        if no_host_wide_saturation
        else (
            "At least one CPU, GPU, iowait, swap, or thermal threshold failed; host-wide "
            "saturation cannot be excluded by this evidence."
        )
    )
    host_saturation_finding = {
        "supported": no_host_wide_saturation,
        "first_phase": first_phase,
        "second_phase": second_phase,
        "first_cpu_busy_percent": first_host_cpu["busy_user_plus_system_percent"]["mean"],
        "second_cpu_busy_percent": second_host_cpu["busy_user_plus_system_percent"]["mean"],
        "first_gpu_sm_percent": first_gpu["sm_percent"]["mean"],
        "second_gpu_sm_percent": second_gpu["sm_percent"]["mean"],
        "full_cpu_busy_p95_percent": full_host["cpu"][
            "busy_user_plus_system_percent"
        ]["p95"],
        "full_gpu_sm_p95_percent": full_gpu["sm_percent"]["p95"],
        "full_iowait_max_percent": full_host["cpu"]["iowait_percent"]["max"],
        "full_swap_used_max_mib": full_host["memory_mib"]["swap_used"]["max"],
        "explanation": host_explanation,
    }
    if transient:
        host_saturation_finding.update(
            {
                "initial_cpu_busy_percent": first_host_cpu[
                    "busy_user_plus_system_percent"
                ]["mean"],
                "recovered_cpu_busy_percent": second_host_cpu[
                    "busy_user_plus_system_percent"
                ]["mean"],
                "initial_gpu_sm_percent": first_gpu["sm_percent"]["mean"],
                "recovered_gpu_sm_percent": second_gpu["sm_percent"]["mean"],
                "load_increased_after_recovery": (
                    second_host_cpu["busy_user_plus_system_percent"]["mean"]
                    > first_host_cpu["busy_user_plus_system_percent"]["mean"]
                    and second_gpu["sm_percent"]["mean"]
                    > first_gpu["sm_percent"]["mean"]
                ),
            }
        )

    findings = {
        "classification": finding_classification,
        "runtime_pattern": vad_runtime["runtime_pattern"],
        "camera_hz_is_not_the_rtf_cause": {
            "supported": camera_source_stable and no_vad_drops,
            "source_rate_hz": camera_runtime["source_rate_hz_from_median_period"],
            "first_window_effective_wall_rate_hz": first_vad["wall_output_rate_hz"],
            "second_window_effective_wall_rate_hz": second_vad["wall_output_rate_hz"],
            "explanation": camera_hz_explanation,
        },
        "vad_inference_is_not_the_bottleneck": {
            "supported": inference_below_source_period,
            "full_run_mean_ms": full_vad["inference_ms"]["mean"],
            "full_run_p99_ms": full_vad["inference_ms"]["p99"],
            "first_window_mean_ms": first_vad["inference_ms"]["mean"],
            "second_window_mean_ms": second_vad["inference_ms"]["mean"],
            "camera_source_period_ms": source_period_ms,
        },
        "camera_delivery_pattern": camera_pattern_finding,
        "host_wide_saturation_is_not_supported": host_saturation_finding,
        "single_root_cause_may_not_be_claimed": {
            "value": True,
            "status": causal_status,
            "candidate_triggers": [
                "cold CARLA render/asset or reliable DDS transport warm-up/backpressure",
                "route/scene-dependent rendering or asset state",
                "localized image conversion, worker scheduling, or reliable publish wait",
            ],
            "reason": causal_reason,
        },
    }
    if transient:
        findings["camera_hz_is_not_the_rtf_cause"].update(
            {
                "initial_effective_wall_rate_hz": first_vad["wall_output_rate_hz"],
                "recovered_effective_wall_rate_hz": second_vad["wall_output_rate_hz"],
            }
        )
        findings["vad_inference_is_not_the_bottleneck"].update(
            {
                "initial_mean_ms": first_vad["inference_ms"]["mean"],
                "recovered_mean_ms": second_vad["inference_ms"]["mean"],
            }
        )
        findings["camera_delivery_stall_collapses_at_recovery"] = {
            "supported": camera_pattern_supported,
            "initial_clean_mean_receipt_span_ms": first_camera["receipt_span_ms"]["mean"],
            "recovered_mean_receipt_span_ms": second_camera["receipt_span_ms"]["mean"],
            "reduction_factor": camera_runtime["comparison"][
                "mean_receipt_span_reduction_factor"
            ],
        }
    else:
        findings["camera_delivery_stall_collapses_at_recovery"] = {
            "supported": False,
            "reason": "no abrupt RTF recovery was detected",
        }

    latency_crosscheck = _latency_crosscheck(latency, camera_runtime)
    problems = []
    if latency_crosscheck["matched_bundle_count_agrees"] is not True:
        problems.append(
            "latency report and bag reconstruction disagree on matched camera bundles"
        )
    if recorder.get("camera_info_topic_count") != len(CAMERA_INFO_TOPICS):
        problems.append("recorder evidence does not contain all six camera-info topics")

    classification_supported = bool(finding_classification) and not problems
    if classification_claims["camera_delivery_pattern"] and not camera_pattern_supported:
        classification_supported = False
        problems.append("classification asserts an unsupported camera-delivery pattern")
    if (
        classification_claims["host_wide_saturation_excluded"]
        and not no_host_wide_saturation
    ):
        classification_supported = False
        problems.append("classification excludes host-wide saturation without support")

    support = {
        "classification": finding_classification,
        "classification_supported": classification_supported,
        "classification_claims": classification_claims,
        "camera_pattern_supported": camera_pattern_supported,
        "no_host_wide_saturation_supported": no_host_wide_saturation,
    }

    return {
        "schema_version": 3,
        "analysis": "CARLA/VAD pilot runtime-load phase reconstruction",
        "status": "complete" if not problems else "incomplete",
        "problems": problems,
        "support": support,
        "input_manifest": dict(input_manifest or {}),
        "clock_domains": {
            "vad_wall": "Unix/system timestamps embedded in stack.log",
            "camera_source": "ROS/CARLA simulation stamps",
            "bag_receipt": "rosbag recorder receipt/system timestamps",
            "host_telemetry": "timezone-aware conversion of logged local timestamps",
            "cross_domain_subtraction_used": False,
        },
        "trial": {
            "started_at": result["started_at"],
            "finished_at": result["finished_at"],
            "success": result.get("success"),
            "reason": result.get("reason"),
            "sim_elapsed_sec": metrics.get("sim_elapsed_sec"),
            "wall_elapsed_sec": metrics.get("wall_elapsed_sec"),
            "overall_rtf": overall_rtf,
            "maximum_observed_speed_mps": metrics.get("maximum_observed_speed_mps"),
            "traveled_distance_m": metrics.get("traveled_distance_m"),
        },
        "vad_runtime": vad_runtime,
        "camera_delivery": camera_runtime,
        "latency_report_crosscheck": latency_crosscheck,
        "recorder": recorder,
        "host_load": host_runtime,
        "runtime_boundary_location": {
            "kind": boundary["kind"],
            "first_last": first_location,
            "second_first": second_location,
        },
        "transition_location": (
            {"initial_last": first_location, "recovered_first": second_location}
            if transient
            else None
        ),
        "findings": findings,
        "interpretation_boundaries": [
            camera_runtime["interpretation_boundary"],
            host_runtime["gpu_device_total"]["boundary"],
            "pidstat %CPU is percent of one logical CPU; group values may exceed 100%.",
            "Aggregate CPU idle does not exclude a single CARLA game-thread or render-fence wait.",
            (
                "This is a one-run observational change point, not a randomized causal experiment."
                if transient
                else (
                    "This is a one-run observational comparison; the descriptive "
                    "midpoint is not a change point or a randomized causal experiment."
                )
            ),
        ],
    }


def _clean_json(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_input_manifest(paths: Mapping[str, Path]) -> dict[str, Any]:
    manifest: dict[str, Any] = {}
    for label, path in paths.items():
        resolved = path.expanduser().resolve()
        if resolved.is_dir():
            files = sorted(item for item in resolved.iterdir() if item.is_file())
            manifest[label] = {
                "path": str(resolved),
                "files": [
                    {
                        "name": item.name,
                        "size_bytes": item.stat().st_size,
                        "sha256": _sha256(item),
                    }
                    for item in files
                ],
            }
        else:
            manifest[label] = {
                "path": str(resolved),
                "size_bytes": resolved.stat().st_size,
                "sha256": _sha256(resolved),
            }
    return manifest


def _plot_conclusion_lines(report: Mapping[str, Any]) -> list[str]:
    """Build plot conclusions without upgrading unsupported observations."""
    findings = report.get("findings", {})
    support = report.get("support", {})
    runtime_pattern = findings.get("runtime_pattern")
    lines = []

    camera_hz = findings.get("camera_hz_is_not_the_rtf_cause", {})
    if camera_hz.get("supported") is True:
        lines.append("- 5 sim-Hz source is stable; no VAD drops.")
    else:
        lines.append("- Stable, drop-free 5 sim-Hz evidence is not established.")

    if support.get("camera_pattern_supported") is True:
        if runtime_pattern == "transient_recovery":
            lines.append("- Six-camera receipt-span collapse accompanies recovery.")
        elif runtime_pattern == "persistent_low_rtf_no_recovery":
            lines.append("- High six-camera receipt spans persist across both windows.")
        else:
            lines.append("- The reported six-camera delivery pattern is supported.")
    else:
        lines.append("- A camera-delivery pattern is not supported by these thresholds.")

    if support.get("no_host_wide_saturation_supported") is True:
        lines.append("- Host-wide saturation is not supported.")
    else:
        lines.append("- Host-wide saturation cannot be excluded.")
    return lines


def render_runtime_load_png(report: Mapping[str, Any], output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vad = report["vad_runtime"]
    camera = report["camera_delivery"]
    host = report["host_load"]
    boundary = vad["analysis_window_boundary"]
    boundary_sim = boundary["second_first_source_sim_sec"]
    transient = vad["runtime_pattern"] == "transient_recovery"
    first_phase, second_phase = vad["comparison_phase_names"]
    first_camera_phase, second_camera_phase = camera["comparison_phase_names"]
    boundary_color = "#d62728" if transient else "#7f7f7f"
    boundary_style = "--" if transient else ":"

    figure, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
    figure.suptitle("CARLA/VAD Pilot Runtime Load Reconstruction", fontsize=16)

    source = vad["series"]["source_sim_sec"]
    rtf = vad["series"]["interval_rtf"]
    axes[0, 0].plot(source[1:], rtf[1:], color="#1f77b4", linewidth=1.0)
    axes[0, 0].axvline(boundary_sim, color=boundary_color, linestyle=boundary_style)
    axes[0, 0].axhline(1.0, color="black", linewidth=0.8, alpha=0.5)
    axes[0, 0].set(title="Interval RTF", xlabel="simulation time [s]", ylabel="sim / wall")
    axes[0, 0].grid(alpha=0.25)

    axes[0, 1].plot(
        camera["series"]["source_sim_sec"],
        camera["series"]["receipt_span_ms"],
        color="#ff7f0e",
        linewidth=1.0,
    )
    axes[0, 1].axvline(boundary_sim, color=boundary_color, linestyle=boundary_style)
    axes[0, 1].set(
        title="Six-camera same-stamp receipt span",
        xlabel="simulation stamp [s]",
        ylabel="receipt span [ms]",
    )
    axes[0, 1].grid(alpha=0.25)

    axes[1, 0].plot(
        source,
        vad["series"]["inference_ms"],
        color="#2ca02c",
        linewidth=1.0,
    )
    axes[1, 0].axvline(boundary_sim, color=boundary_color, linestyle=boundary_style)
    axes[1, 0].axhline(200.0, color="black", linewidth=0.8, alpha=0.4, label="5-Hz period")
    axes[1, 0].set(
        title="VAD inference latency", xlabel="simulation time [s]", ylabel="latency [ms]"
    )
    axes[1, 0].legend(loc="upper right")
    axes[1, 0].grid(alpha=0.25)

    cpu = [
        host["vmstat"]["phases"][phase]["cpu"]["busy_user_plus_system_percent"]["mean"]
        for phase in (first_phase, second_phase)
    ]
    gpu = [
        host["gpu_device_total"]["phases"][phase]["sm_percent"]["mean"]
        for phase in (first_phase, second_phase)
    ]
    x_positions = [0.0, 1.0]
    axes[1, 1].bar([x - 0.18 for x in x_positions], cpu, 0.36, label="CPU busy")
    axes[1, 1].bar([x + 0.18 for x in x_positions], gpu, 0.36, label="GPU SM")
    axes[1, 1].set_xticks(
        x_positions,
        (first_phase.replace("_", " "), second_phase.replace("_", " ")),
    )
    axes[1, 1].set(
        title="Host utilization by runtime window",
        ylabel="mean utilization [%]",
    )
    axes[1, 1].legend()
    axes[1, 1].grid(axis="y", alpha=0.25)

    process_names = (
        "carla_server",
        "carla_bridge",
        "rviz",
        "vad_inference",
        "route_manager",
        "rosbag_recorder",
        "desktop_capture",
    )
    available_names = [
        name
        for name in process_names
        if host["process_groups"].get(name, {}).get("phases", {}).get(
            first_phase, {}
        ).get("available")
    ]
    indices = list(range(len(available_names)))
    first_cpu = [
        host["process_groups"][name]["phases"][first_phase]
        ["cpu_percent_of_one_logical_core"]["mean"]
        for name in available_names
    ]
    second_cpu = [
        host["process_groups"][name]["phases"][second_phase]
        ["cpu_percent_of_one_logical_core"]["mean"]
        for name in available_names
    ]
    axes[2, 0].barh(
        [index + 0.18 for index in indices],
        first_cpu,
        0.36,
        label=first_phase.replace("_", " "),
    )
    axes[2, 0].barh(
        [index - 0.18 for index in indices],
        second_cpu,
        0.36,
        label=second_phase.replace("_", " "),
    )
    axes[2, 0].set_yticks(indices, [name.replace("_", " ") for name in available_names])
    axes[2, 0].set(
        title="Process CPU (100% = one logical CPU)", xlabel="mean pidstat CPU [%]"
    )
    axes[2, 0].legend()
    axes[2, 0].grid(axis="x", alpha=0.25)

    first_rtf = vad["phases"][first_phase]["aggregate_rtf"]
    second_rtf = vad["phases"][second_phase]["aggregate_rtf"]
    first_span = camera["phases"][first_camera_phase]["receipt_span_ms"]["mean"]
    second_span = camera["phases"][second_camera_phase]["receipt_span_ms"]["mean"]
    first_inference = vad["phases"][first_phase]["inference_ms"]["mean"]
    second_inference = vad["phases"][second_phase]["inference_ms"]["mean"]
    conclusion_text = "\n".join(_plot_conclusion_lines(report))
    if transient:
        summary_text = (
            f"Abrupt recovery at sim {boundary_sim:.1f} s\n"
            f"RTF: {first_rtf:.3f} -> {second_rtf:.3f}\n"
            f"6-camera span: {first_span:.2f} -> {second_span:.2f} ms\n"
            f"VAD inference: {first_inference:.2f} -> {second_inference:.2f} ms\n\n"
            f"Conclusion\n{conclusion_text}\n\n"
            "Causal boundary\n"
            "Warm-up/DDS/render/asset effects and a spatial\n"
            "scene boundary are confounded in this one run."
        )
    else:
        full_rtf = vad["phases"]["full_run"]["aggregate_rtf"]
        full_span = camera["phases"]["full_run"]["receipt_span_ms"]["mean"]
        summary_text = (
            "Persistent low RTF; no recovery detected\n"
            f"Full RTF: {full_rtf:.3f}\n"
            f"Early -> late RTF: {first_rtf:.3f} -> {second_rtf:.3f}\n"
            f"Full 6-camera span: {full_span:.2f} ms\n"
            f"Early -> late span: {first_span:.2f} -> {second_span:.2f} ms\n"
            f"VAD inference: {first_inference:.2f} -> {second_inference:.2f} ms\n\n"
            f"Conclusion\n{conclusion_text}\n\n"
            "Causal boundary\n"
            "The dotted midpoint is descriptive, not a recovery.\n"
            "No single trigger can be assigned from this run."
        )
    axes[2, 1].axis("off")
    axes[2, 1].text(
        0.02,
        0.98,
        summary_text,
        transform=axes[2, 1].transAxes,
        va="top",
        ha="left",
        fontsize=11,
        family="monospace",
        bbox={"boxstyle": "round", "facecolor": "#f5f5f5", "edgecolor": "#999999"},
    )
    figure.savefig(output, format="png", dpi=150, facecolor="white")
    plt.close(figure)


def write_outputs_atomically(output_dir: Path, report: Mapping[str, Any]) -> tuple[Path, Path]:
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_output = output_dir / "runtime_load_analysis.json"
    png_output = output_dir / "runtime_load_analysis.png"
    staged_paths: list[Path] = []
    try:
        descriptor, json_staged_name = tempfile.mkstemp(
            prefix=f".{json_output.name}.", suffix=".tmp", dir=output_dir
        )
        json_staged = Path(json_staged_name)
        staged_paths.append(json_staged)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(_clean_json(report), stream, indent=2, sort_keys=True, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())

        descriptor, png_staged_name = tempfile.mkstemp(
            prefix=f".{png_output.stem}.", suffix=".png", dir=output_dir
        )
        os.close(descriptor)
        png_staged = Path(png_staged_name)
        staged_paths.append(png_staged)
        render_runtime_load_png(report, png_staged)
        with png_staged.open("rb") as stream:
            os.fsync(stream.fileno())

        os.replace(json_staged, json_output)
        staged_paths.remove(json_staged)
        os.replace(png_staged, png_output)
        staged_paths.remove(png_staged)
        return json_output, png_output
    finally:
        for staged in staged_paths:
            try:
                staged.unlink()
            except FileNotFoundError:
                pass


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _required_file(path: Path, label: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise RuntimeLoadError(f"{label} does not exist: {resolved}")
    return resolved


def _resolve_inputs(args: argparse.Namespace) -> dict[str, Path]:
    trial = args.trial_dir.expanduser().resolve()
    if not trial.is_dir():
        raise RuntimeLoadError(f"trial directory does not exist: {trial}")
    telemetry_root = trial.parents[1] / "host_telemetry"
    attempt_telemetry = telemetry_root / trial.name
    default_telemetry = (
        attempt_telemetry
        if all(
            (attempt_telemetry / filename).is_file()
            for filename in ("vmstat.log", "nvidia_smi_dmon.log", "pidstat.log")
        )
        else telemetry_root
    )
    telemetry = (
        args.host_telemetry_dir.expanduser().resolve()
        if args.host_telemetry_dir
        else default_telemetry
    )
    paths = {
        "result": args.result or trial / "result.json",
        "latency": args.latency_json or trial / "latency" / "e2e_latency.json",
        "bag": args.bag or trial / "bag",
        "stack": args.stack_log or trial / "stack.log",
        "recorder": args.recorder_log or trial / "recorder.log",
        "vmstat": args.vmstat_log or telemetry / "vmstat.log",
        "nvidia_dmon": args.nvidia_dmon_log or telemetry / "nvidia_smi_dmon.log",
        "pidstat": args.pidstat_log or telemetry / "pidstat.log",
    }
    for key in ("result", "latency", "stack", "recorder", "vmstat", "nvidia_dmon", "pidstat"):
        paths[key] = _required_file(Path(paths[key]), key)
    paths["bag"] = Path(paths["bag"]).expanduser().resolve()
    if not paths["bag"].is_dir():
        raise RuntimeLoadError(f"bag directory does not exist: {paths['bag']}")
    return paths


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial-dir", type=Path, required=True)
    parser.add_argument("--host-telemetry-dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--result", type=Path)
    parser.add_argument("--latency-json", type=Path)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--stack-log", type=Path)
    parser.add_argument("--recorder-log", type=Path)
    parser.add_argument("--vmstat-log", type=Path)
    parser.add_argument("--nvidia-dmon-log", type=Path)
    parser.add_argument("--pidstat-log", type=Path)
    parser.add_argument("--timezone", default="Asia/Seoul")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = _resolve_inputs(args)
    result = _load_json(paths["result"])
    latency = _load_json(paths["latency"])
    stack = parse_stack_log(paths["stack"].read_text(encoding="utf-8", errors="replace"))
    recorder = parse_recorder_log(
        paths["recorder"].read_text(encoding="utf-8", errors="replace")
    )
    vmstat_rows = parse_vmstat(
        paths["vmstat"].read_text(encoding="utf-8", errors="replace"), args.timezone
    )
    gpu_rows = parse_nvidia_dmon(
        paths["nvidia_dmon"].read_text(encoding="utf-8", errors="replace"),
        args.timezone,
    )
    pidstat = parse_pidstat(
        paths["pidstat"].read_text(encoding="utf-8", errors="replace"),
        args.timezone,
    )
    camera_records = read_camera_info_bag(paths["bag"])
    report = build_runtime_load_analysis(
        result=result,
        latency=latency,
        stack=stack,
        recorder=recorder,
        camera_records=camera_records,
        vmstat_rows=vmstat_rows,
        gpu_rows=gpu_rows,
        pidstat=pidstat,
        input_manifest=build_input_manifest(paths),
    )
    json_output, png_output = write_outputs_atomically(args.output_dir, report)
    print(json.dumps({"json": str(json_output), "png": str(png_output)}, indent=2))
    return 0 if report.get("status") == "complete" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeLoadError as error:
        raise SystemExit(f"runtime-load analysis failed: {error}") from error
