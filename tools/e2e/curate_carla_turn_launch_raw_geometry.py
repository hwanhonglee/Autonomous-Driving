#!/usr/bin/env python3
"""HH_260906 - Publish compact all-eight raw geometry evidence with real-data charts, never training approval."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import ipaddress
import importlib.metadata
import json
import math
from pathlib import Path
import re

from portable_e2e import contract
from scripts.e2e import audit_carla_turn_launch_raw_geometry as diagnostic

ROOT = Path(__file__).resolve().parents[2]
DIAGNOSTIC_SHA256 = "32f1cd04669afc665d28cdf7040ee1517505a6543c3a420806a1816db06fbe49"
INPUTS = ("summary.json", "jpeg_audit.jsonl", "future_geometry_audit.jsonl", "native_snapshot_audit.jsonl")
ARTIFACT_SHA256 = {
    "summary.json": "7f1ed0bd40c7b1ec531e3402f3b90ad21eccc4e45ac513fb6d69b5fd953bce33",
    "future_geometry_audit.jsonl": "bbf894187bd129146da4515e513341b97795ed470fccce49ab25fbe5819353f9",
    "jpeg_audit.jsonl": "417d7f4c205bf0de8d0870961d178ca86b2425d2ae52128c4d0626daf9d4ba18",
    "native_snapshot_audit.jsonl": "0f4ff705919ed25db8bc2bba615506a497ab05c66a77a3928307ddfe7112124e",
}
CHECKSUM_SHA256 = "1df3034160cd3a9d3b4d686cb57415aba130dbd37f9d63649d388e5168fcc738"
METRICS = ("speed_acceleration", "speed_deceleration", "xy_acceleration", "xy_deceleration",
           "xy_curvature", "xy_lateral_acceleration", "yaw_label_heading_envelope")
require = diagnostic.require


def render_provenance():
    """HH_260906 - Record existing plotting versions and local source bytes without installing or changing dependencies."""
    return {"versions": {name: importlib.metadata.version(name) for name in ("matplotlib", "numpy", "pillow")},
        "backend": "Agg", "dpi": 150,
        "local_source_sha256": {Path(module.__file__).resolve().relative_to(ROOT).as_posix():
            contract.sha256_file(Path(module.__file__)) for module in (contract, diagnostic, diagnostic.raw, diagnostic.matrix)}}


def source_inputs(root):
    """HH_260906 - Authenticate every original diagnostic byte stream before using the compact published summary."""
    root = Path(root).resolve()
    require({p.name for p in root.iterdir()} == set(INPUTS) | {"SHA256SUMS"}, "raw diagnostic output inventory differs")
    checksum = contract._safe_file(root, "SHA256SUMS", "diagnostic checksums").read_bytes()
    require(hashlib.sha256(checksum).hexdigest() == CHECKSUM_SHA256, "reviewed original checksum-manifest bytes changed")
    entries = [line.split("  ") for line in checksum.decode().splitlines()]
    require(len(entries) == len(INPUTS) and all(len(e) == 2 for e in entries)
            and {e[1] for e in entries} == set(INPUTS), "diagnostic checksum inventory differs")
    pins = {}
    for expected, name in entries:
        path = contract._safe_file(root, name, "diagnostic input")
        actual = contract.sha256_file(path)
        require(actual == expected == ARTIFACT_SHA256[name], "diagnostic checksum or reviewed original artifact pin mismatch")
        pins[name] = {"sha256": actual, "size_bytes": path.stat().st_size}
    pins["SHA256SUMS"] = {"sha256": hashlib.sha256(checksum).hexdigest(), "size_bytes": len(checksum)}
    summary, summary_sha = contract._read_json_and_sha256(root / "summary.json")
    require(summary_sha == pins["summary.json"]["sha256"], "summary changed while parsed")
    return summary, pins


def validate_summary(summary):
    """HH_260906 - Missing cases, new approval claims or unreviewed executed diagnostics cannot become a public success."""
    require(summary.get("schema") == diagnostic.SCHEMA and summary.get("status") == "DIAGNOSED_NOT_ADMITTED"
            and summary.get("planned_case_count") == summary.get("finalized_case_count") == 8
            and summary.get("all_planned_cases_retained") is True, "all eight finalized raw diagnoses required")
    require(summary["source_identity"]["files"].get("scripts/e2e/audit_carla_turn_launch_raw_geometry.py") == DIAGNOSTIC_SHA256
            and summary["source_identity"]["files"].get("scripts/e2e/audit_carla_turn_launch_matrix.py") == diagnostic.MATRIX_AUDITOR_SHA256,
            "executed diagnostic/matrix source differs from reviewed versions")
    scope = summary["scope"]
    for key in ("training_data_approved", "dataset_admission", "common10_dataset_written", "source_flags_modified",
                "labels_modified", "training", "model_loaded", "model_inference", "live_simulator_access", "test_payload_read",
                "automatic_winner_selection", "native20hz_quality_replaced_by_10hz"):
        require(scope.get(key) is False, "raw diagnostic scope or approval changed")
    cases = summary["cases"]
    repetitions = Counter()
    expected = []
    for index, profile in enumerate(diagnostic.ORDER, 1):
        repetitions[profile] += 1
        expected.append((f"{index:02d}_{profile}_r{repetitions[profile]}", profile, repetitions[profile]))
    require(len(cases) == 8 and len({c["case_id"] for c in cases}) == 8
            and [(c["case_id"], c["profile"], c["replicate"]) for c in cases] == expected, "case identities or frozen order differ")
    require([(c["case_id"], c["profile"], c["replicate"]) for c in summary["independent_matrix_audit"]["cases"]] == expected,
            "case identities differ from independent matrix audit")
    for case in cases:
        require(case["status"] == "DIAGNOSED_NOT_ADMITTED" and case["training_data_approved"] is False
                and case["original_training_data_approved"] is False and case["original_development_only"] is True,
                "case missing geometry or original denial")
        require(case["transport_protocol"]["status"] == case["initialization_protocol"]["status"] == "PASS",
                "published geometry requires intact recorded-control/initialization provenance")
    return cases


def validate_streams(root, summary, pins):
    """HH_260906 - Recount all images, native observations and future windows; plots use deterministic midpoint driving anchors."""
    cases = validate_summary(summary)
    by_id = {c["case_id"]: c for c in cases}
    images, native, phases = Counter(), Counter(), defaultdict(Counter)
    native_last, image_ids = {}, set()
    for _, row in contract._iter_jsonl(root / "jpeg_audit.jsonl", expected_sha256=pins["jpeg_audit.jsonl"]["sha256"]):
        key = row["case_id"]
        require(key in by_id and row["decoded"] is True and row["width"] == 640 and row["height"] == 360
                and row["mode"] == "RGB" and row["referenced"] is True, "pixel stream has missing/corrupt/unreferenced evidence")
        identity = (key, row["path"])
        require(identity not in image_ids, "duplicate JPEG diagnostic row")
        image_ids.add(identity); images[key] += 1
    for _, row in contract._iter_jsonl(root / "native_snapshot_audit.jsonl", expected_sha256=pins["native_snapshot_audit.jsonl"]["sha256"]):
        key = row["case_id"]
        require(key in by_id, "unknown native case")
        if key in native_last:
            frame, timestamp = native_last[key]
            require(row["frame"] == frame + 1 and abs(row["timestamp_ns"] - timestamp - 50_000_000) <= 500,
                    "native snapshot sequence/cadence differs")
        native_last[key] = (row["frame"], row["timestamp_ns"])
        native[key] += 1; phases[key][row["capture_phase"]] += 1
    dispositions, anchor_phases, full, unassessed = defaultdict(Counter), defaultdict(Counter), Counter(), Counter()
    eligible, failures = Counter(), Counter()
    curvature = {key: {"violating_point_windows": 0, "first_point": 0, "later_points": 0,
        "xy_interval_speed_buckets_mps": {"lt0p1": 0, "0p1_to_0p5": 0, "0p5_to_1": 0, "ge1": 0},
        "anchor_phases": {}, "maximum_curvature_witness": None} for key in by_id}
    curvature_ticks, curvature_later_anchors, curvature_first_only = defaultdict(set), Counter(), Counter()
    chosen, driving_index, previous_anchor = {}, Counter(), {}
    for _, row in contract._iter_jsonl(root / "future_geometry_audit.jsonl", expected_sha256=pins["future_geometry_audit.jsonl"]["sha256"]):
        key, phase = row["case_id"], row["capture_phase"]
        require(key in by_id and row["training_data_approved"] is False, "unknown or approved future diagnostic")
        require(key not in previous_anchor or row["frame"] == previous_anchor[key] + 2, "camera-anchor order differs")
        previous_anchor[key] = row["frame"]
        dispositions[key][row["disposition"]] += 1; anchor_phases[key][phase] += 1
        if row.get("valid_future_points") == 64: full[key] += 1
        if "bound_diagnostic_unavailable" in row: unassessed[key] += 1
        if phase == "driving":
            target = by_id[key]["measured_future"]["anchor_count_by_phase"]["driving"] // 2
            if driving_index[key] == target:
                chosen[key] = {name: row[name] for name in ("frame", "anchor_timestamp_ns", "valid_future_points",
                    "diagnostic_future_xy_m", "diagnostic_body_yaw_delta_rad", "diagnostic_target_timestamp_ns")}
            driving_index[key] += 1
        measured = row.get("discrete_bound_diagnostic")
        if measured:
            require(measured["valid_points"] == row["valid_future_points"] == len(measured["steps"]), "assessed prefix denominator differs")
            for horizon in (10, 30, 64):
                if measured["valid_points"] >= horizon:
                    eligible[key, horizon] += 1
                    for metric in METRICS:
                        if any(step["metrics"][metric][1] for step in measured["steps"][:horizon]):
                            failures[key, horizon, metric] += 1
            violated = [s for s in measured["steps"] if s["metrics"]["xy_curvature"][1]]
            if violated:
                curvature_later_anchors[key] += any(s["index"] > 0 for s in violated)
                curvature_first_only[key] += all(s["index"] == 0 for s in violated)
            for step in violated:
                index = step["index"]
                current = row["diagnostic_future_xy_m"][index]
                previous = [0., 0.] if index == 0 else row["diagnostic_future_xy_m"][index - 1]
                displacement = [a-b for a, b in zip(current, previous)]
                distance = math.hypot(*displacement)
                speed = step["metrics"]["xy_speed_limit"][0]
                require(math.isclose(distance / step["dt_s"], speed, abs_tol=1e-10, rel_tol=1e-9), "XY interval speed differs from actual displacement")
                item = curvature[key]
                item["violating_point_windows"] += 1
                item["first_point" if index == 0 else "later_points"] += 1
                bucket = "lt0p1" if speed < .1 else "0p1_to_0p5" if speed < .5 else "0p5_to_1" if speed < 1. else "ge1"
                item["xy_interval_speed_buckets_mps"][bucket] += 1
                item["anchor_phases"][phase] = item["anchor_phases"].get(phase, 0) + 1
                curvature_ticks[key].add(step["episode_relative_100ms_tick"])
                value = step["metrics"]["xy_curvature"][0]
                if item["maximum_curvature_witness"] is None or value > item["maximum_curvature_witness"]["curvature_rad_per_m"]:
                    item["maximum_curvature_witness"] = {"anchor_frame": row["frame"], "anchor_capture_phase": phase,
                        "future_index": index, "target_timestamp_ns": step["target_timestamp_ns"],
                        "curvature_rad_per_m": value, "xy_interval_displacement_m": displacement,
                        "xy_interval_distance_m": distance, "xy_interval_speed_mps": speed,
                        "endpoint_planar_speed_mps": row["diagnostic_endpoint_planar_speed_mps"][index],
                        "anchor_recorded_velocity_base_mps": row["current_recorded_velocity_base_mps"]}
    for key, case in by_id.items():
        future, pixels = case["measured_future"], case["camera_pixels"]
        require(images[key] == pixels["image_count"] == pixels["decoded_count"] == 6 * pixels["camera_anchor_count"], "image/camera denominator differs")
        require(native[key] == case["raw_native_state_count"] and dict(phases[key]) == case["raw_native_state_counts_by_phase"], "native denominator differs")
        require(dict(anchor_phases[key]) == future["anchor_count_by_phase"] and dict(dispositions[key]) == future["disposition_counts"], "future phase/disposition denominator differs")
        require(sum(anchor_phases[key].values()) == future["camera_anchor_count"] == pixels["camera_anchor_count"]
                and full[key] == future["available_64_point_anchor_count"]
                and unassessed[key] == future["bound_unassessed_anchor_count"], "available versus assessed future counts differ")
        require(eligible[key, 64] == future["bound_assessed_64_point_anchor_count"], "full bound-assessed count differs")
        require(key in chosen, "midpoint driving anchor unavailable")
        for horizon in (10, 30, 64):
            block = future["all_available_prefixes"][str(horizon)]
            require(block["horizon_eligible_anchor_count"] == eligible[key, horizon], "horizon denominator differs")
            for metric in METRICS:
                require(block["metrics"][metric]["horizon_eligible_violating_anchor_count"] == failures[key, horizon, metric],
                        "published horizon violation count differs from full per-anchor stream")
        metric = future["all_available_prefixes"]["64"]["metrics"]["xy_curvature"]
        require(curvature[key]["violating_point_windows"] == metric["violating_step_count"]
                and len(curvature_ticks[key]) == metric["unique_episode_relative_100ms_violation_tick_count"],
                "curvature repeated windows versus unique target ticks differ")
        curvature[key].update(unique_episode_relative_100ms_target_ticks=len(curvature_ticks[key]),
            first_point_only_violating_anchors=curvature_first_only[key],
            anchors_with_later_point_violation=curvature_later_anchors[key])
    require(sum(images.values()) == summary["total_jpeg_count"] == summary["total_decoded_jpeg_count"], "total pixel count differs")
    return {"image_count": sum(images.values()), "native_state_count": sum(native.values()),
        "camera_anchor_count": sum(sum(v.values()) for v in anchor_phases.values()),
        "available_64_point_anchor_count": sum(full.values()), "bound_assessed_64_point_anchor_count": sum(eligible[k, 64] for k in by_id),
        "bound_unassessed_anchor_count": sum(unassessed.values()), "selection": "Each case's zero-based floor(driving_camera_count/2) anchor; fixed before looking at geometry.",
        "midpoint_driving_anchors": chosen, "curvature_conditioning_by_case": curvature,
        "conditioning_notice": "Speed buckets are descriptive, not a new pass gate or a filter. Tiny displacement makes direction-based curvature ill-conditioned; no sensor-noise or physics cause is proven. All original flagged windows remain flagged.",
        "all_stream_denominators_recomputed": True, "training_data_approved": False}


def charts(summary, stream_report, output):
    """HH_260906 - Plot real audited scalar/XY counts and uniformly selected ego-centered measured futures, not model predictions or footage."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    cases = summary["cases"]
    labels = [f".{c['profile'].split('_')[2][1:]} r{c['replicate']}" for c in cases]
    x = np.arange(8)
    fig, axes = plt.subplots(2, 1, figsize=(13, 9), constrained_layout=True)
    bounds = [c["raw_scalar_quality"]["speed_rate_qa"]["native_20hz"]["by_phase"]["all"] for c in cases]
    axes[0].bar(x - .17, [r["maximum_mps2"] for r in bounds], .34, label="Maximum native 20 Hz dv/dt")
    axes[0].bar(x + .17, [r["minimum_mps2"] for r in bounds], .34, label="Minimum native 20 Hz dv/dt")
    for bound in (-2.9, 2.9): axes[0].axhline(bound, color="crimson", linestyle="--", linewidth=1)
    axes[0].set(ylabel="Measured planar speed rate (m/s²)", title="All native intervals retained; dashed decoder bounds ±2.9 m/s²", xticks=x, xticklabels=labels)
    axes[0].legend(loc="lower right", fontsize=9)
    for index, (metric, label) in enumerate((("xy_acceleration", "XY acceleration"), ("xy_deceleration", "XY deceleration"),
                                            ("xy_curvature", "XY curvature"), ("xy_lateral_acceleration", "XY lateral"))):
        values = [c["measured_future"]["all_available_prefixes"]["64"]["metrics"][metric]["horizon_eligible_violating_anchor_count"] for c in cases]
        axes[1].bar(x + (index - 1.5) * .18, values, .18, label=label)
    axes[1].set(ylabel="Overlapping 6.4 s anchors flagged", title="Necessary discrete XY bounds only; not physical impossibility or admission", xticks=x, xticklabels=labels)
    axes[1].legend(fontsize=9)
    fig.suptitle("Frozen C-track launch matrix: all eight trials, including scalar failures", fontsize=14)
    fig.savefig(output / "all_eight_scalar_and_xy_bounds.png", dpi=150); plt.close(fig)
    selected = stream_report["midpoint_driving_anchors"]
    radius = max((max(abs(v) for v in point) for row in selected.values() for point in row["diagnostic_future_xy_m"] if None not in point), default=1.)
    radius = max(1., radius * 1.15)
    fig, axes = plt.subplots(2, 4, figsize=(14, 7.5), constrained_layout=True)
    for axis, case, label in zip(axes.flat, cases, labels):
        row = selected[case["case_id"]]
        points = [[0., 0.]] + row["diagnostic_future_xy_m"][:row["valid_future_points"]]
        axis.plot([p[0] for p in points], [p[1] for p in points], color="#1976a3", linewidth=2)
        axis.scatter([0], [0], marker=">", color="black", s=40, label="Ego anchor")
        for index in (9, 29, 63):
            if index < row["valid_future_points"]:
                point = row["diagnostic_future_xy_m"][index]
                axis.scatter(*point, s=20); axis.annotate(f"{(index+1)/10:g}s", point, fontsize=7)
        axis.set(xlim=(-radius, radius), ylim=(-radius, radius), aspect="equal", title=f"{label} / frame {row['frame']}", xlabel="Forward X (m)", ylabel="Left Y (m)")
        axis.grid(alpha=.25)
    fig.suptitle("Measured 6.4 s futures at each driving midpoint — fixed ego-centered axes\nRaw expert observations; no learned trajectory, camera capture or GUI-FPS claim", fontsize=12)
    fig.savefig(output / "eight_ego_centered_measured_futures.png", dpi=150); plt.close(fig)
    details = stream_report["curvature_conditioning_by_case"]
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), constrained_layout=True)
    bottom = np.zeros(8)
    for bucket, label in (("lt0p1", "XY interval speed <0.1 m/s"), ("0p1_to_0p5", "0.1–0.5 m/s"),
                          ("0p5_to_1", "0.5–1 m/s"), ("ge1", "≥1 m/s")):
        values = [details[c["case_id"]]["xy_interval_speed_buckets_mps"][bucket] for c in cases]
        axes[0].bar(x, values, bottom=bottom, label=label); bottom += values
    axes[0].set(xticks=x, xticklabels=labels, ylabel="Flagged point/window observations", title="Curvature flags by measured XY interval speed — descriptive bins, no filter")
    axes[0].legend(fontsize=9)
    axes[1].bar(x - .18, [details[c["case_id"]]["first_point"] for c in cases], .36, label="First future point")
    axes[1].bar(x + .18, [details[c["case_id"]]["later_points"] for c in cases], .36, label="Later future points")
    axes[1].set(xticks=x, xticklabels=labels, ylabel="Flagged point/window observations", title="Overlapping windows remain counted; not independent physical turning events")
    axes[1].legend()
    fig.suptitle("Low-displacement curvature conditioning — no source labels or verdicts changed", fontsize=13)
    fig.savefig(output / "curvature_low_displacement_conditioning.png", dpi=150); plt.close(fig)


def readme(summary, counted):
    """HH_260906 - Explain distinct scalar, XY and body-yaw evidence without promoting a favourable launch pedal."""
    lines = ["# C-track 출발 페달 8회: 원본 미래 궤적 진단", "", "학습 모델 주행이나 학습 데이터 승인이 아닙니다. 모든 원본의 `training_data_approved:false`·`development_only:true`를 유지합니다.", "",
        "순서 `.15 → .13 → .14 → .12 → .12 → .14 → .13 → .15`의 8회 전체를 포함합니다. 전부 같은 C-track 좌회전·14.4 km/h 목표·Epic·ACK·초기화 후 BasicAgent 설정입니다. 실패 시도도 제외하지 않았습니다.", "",
        "| 시도 | 원본 속도 QA | native 상태 | 카메라 묶음 | 64점 시간/자료 확보 | bounds 평가 | 곡률 위반/평가 | 횡가속 위반 | 미평가 |", "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for c in summary["cases"]:
        f = c["measured_future"]
        block = f["all_available_prefixes"]["64"]
        curvature = block["metrics"]["xy_curvature"]["horizon_eligible_violating_anchor_count"]
        lateral = block["metrics"]["xy_lateral_acceleration"]["horizon_eligible_violating_anchor_count"]
        lines.append(f"| {c['case_id']} | {'PASS' if c['raw_scalar_quality']['raw_scalar_quality_clear'] else 'FAIL'} | {c['raw_native_state_count']} | {f['camera_anchor_count']} | {f['available_64_point_anchor_count']} | {f['bound_assessed_64_point_anchor_count']} | {curvature}/{block['horizon_eligible_anchor_count']} | {lateral} | {f['bound_unassessed_anchor_count']} |")
    lines += ["", "곡률·횡가속 표는 6.4초 미래 창 기준입니다. 아래는 동일 위반을 겹치는 점/창, 고유 target tick, 첫 점/이후 점으로 구분한 기록이며 서로 다른 사건 수를 의미하지 않습니다.", "",
        "| 시도 | 위반 점/창 | 고유 100ms target tick | 첫 미래점 | 이후 미래점 | XY 구간 평균속도 <0.1 m/s | ≥1 m/s |",
        "|---|---:|---:|---:|---:|---:|---:|"]
    for c in summary["cases"]:
        d = counted["curvature_conditioning_by_case"][c["case_id"]]
        lines.append(f"| {c['case_id']} | {d['violating_point_windows']} | {d['unique_episode_relative_100ms_target_ticks']} | {d['first_point']} | {d['later_points']} | {d['xy_interval_speed_buckets_mps']['lt0p1']} | {d['xy_interval_speed_buckets_mps']['ge1']} |")
    lines += ["", f"원본 native 상태 {counted['native_state_count']:,}개, 카메라 묶음 {counted['camera_anchor_count']:,}개, JPEG {counted['image_count']:,}개의 진단 기록을 전부 다시 집계했습니다.", "",
        "![8회 속도 변화와 XY 필요조건](all_eight_scalar_and_xy_bounds.png)", "", "![차량 중심의 실제 미래 위치](eight_ego_centered_measured_futures.png)", "",
        "![곡률 지표의 저변위 조건](curvature_low_displacement_conditioning.png)", "",
        "## 지표를 구분해서 보기", "",
        "- native 20 Hz 속도 변화: 모든 준비·주행·종료 구간과 경계를 포함하며 decoder ±2.9 m/s² 기준을 유지합니다. runtime의 +3/−6 m/s² 속도 변화 기준과는 별개입니다. 10 Hz 결과로 native 실패를 대체하지 않습니다.",
        "- 1·3·6.4초 XY: 원점에서 첫 점, 이후 점 간 거리·방향 변화로 계산하는 **특정 이산 decoder의 필요조건**입니다. 실제 차량의 모든 물리적 주행 가능성을 판정하는 충분조건이 아닙니다.",
        "- 기록된 속도는 순간 속도, XY 거리/시간은 구간 평균 속도입니다. 둘의 불일치를 센서·차량 고장으로 단정하지 않습니다.",
        "- 차량 yaw 변화는 경로 접선 heading의 대리지표이며 XY 곡률과 별개입니다. 정지 근처의 짧은 변위는 heading 미평가로 남기며 PASS로 세지 않습니다.",
        "- 곡률 위반을 낮은 변위·속도 구간별로 다시 집계했습니다. 작은 위치 변화에서는 변위 방향으로 계산한 곡률이 매우 커질 수 있지만, 이를 실제 차량이 해당 곡률로 회전했다거나 센서 노이즈가 원인이라고 단정하지 않습니다. 기존 0.0001 m heading 평가 기준·위반 개수·라벨을 바꾸지 않았고 준비 구간을 삭제하지 않았습니다.",
        "- 준비 구간의 카메라 앵커도 포함합니다. 종료 6.5초는 미래 위치를 확인하는 자료로만 쓰며 해당 구간의 앵커를 학습 샘플로 만들지 않습니다. 겹치는 미래 구간 수는 독립적인 급출발·회전 횟수가 아닙니다.",
        "- 64개 미래 위치가 있어도 기존 경로 투영 계산이 불가능하면 bounds 미평가로 남깁니다. 가짜 goal/route 값을 넣거나 해당 앵커를 숨기지 않습니다.",
        "- 전체 3D ActorSnapshot과 world 벡터를 기록 공식과 비교했지만 CARLA API 기준점이 물리적 무게중심·바퀴 중심이라는 사실까지 증명하지는 않습니다. 기존 ax/ay 정의와 라벨을 바꾸지 않았습니다.",
        "- 두 번째 그림은 시도별 주행 카메라 순서의 중앙 앵커를 고정 규칙으로 선택한 실제 측정 위치입니다. 차량 원점은 모든 패널 중앙이며 같은 축 범위를 사용합니다. 학습 예측·새 주행 촬영·GIF·Autoware 화면이 아닙니다.", "",
        "[전체 원본 진단 요약](summary.json) · [스트림 재집계·그림 앵커·최대 곡률 원본 변위 증거](stream_validation.json) · [출처 및 원본 파일 SHA](provenance.json) · [공개 파일 SHA](SHA256SUMS)", "",
        "큰 per-anchor·JPEG·3D JSONL은 원본 비공개 진단 폴더에 보존하고 이 폴더에는 파일별 SHA/크기를 기록합니다. 복사된 `summary.json`은 실행 결과와 동일한 바이트입니다. 학습 변환기·Common10 데이터셋·optimizer를 호출하지 않았으며, 자동 페달 선택·데이터 승인·모델 승격은 없습니다.", ""]
    return "\n".join(lines)


def publish(root, output):
    """HH_260906 - Create only a new public folder after full-stream consistency and privacy checks."""
    root, output = Path(root).resolve(), Path(output)
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(root)
            and not output.resolve().is_relative_to(ROOT / "datasets"), "fresh publication must stay outside originals")
    script_sha = contract.sha256_file(Path(__file__))
    plotting = render_provenance()
    summary, pins = source_inputs(root)
    counted = validate_streams(root, summary, pins)
    original = (root / "summary.json").read_bytes()
    require(hashlib.sha256(original).hexdigest() == pins["summary.json"]["sha256"], "summary changed before publication")
    require(b"/home/" not in original and re.search(rb"\b[\w.-]+@(?:[\w.-]+)", original) is None,
            "private account/path metadata requires explicit redaction review")
    for address in re.findall(rb"\b(?:[0-9]{1,3}\.){3}[0-9]{1,3}\b", original):
        try:
            require(ipaddress.ip_address(address.decode()).is_loopback, "non-loopback IP metadata requires redaction review")
        except ValueError as error:
            raise diagnostic.raw.scalar.EvidenceError("invalid address-shaped metadata requires redaction review") from error
    for name, pin in pins.items():
        require(contract.sha256_file(root / name) == pin["sha256"], "source changed before publication")
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_bytes(original)
    (output / "stream_validation.json").write_text(json.dumps(counted, indent=2, allow_nan=False) + "\n")
    charts(summary, counted, output)
    (output / "README.md").write_text(readme(summary, counted))
    # HH_260906 - Later rendering cannot hide a changed original or changed local plotting/helper dependency.
    for name, pin in pins.items():
        require(contract.sha256_file(root / name) == pin["sha256"], "source changed during rendering/publication")
    require(render_provenance() == plotting, "render environment or helper source changed during publication")
    provenance = {"schema": "carla_expert.turn_launch_geometry_publication.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_artifact_root": root.relative_to(ROOT).as_posix() if root.is_relative_to(ROOT) else "<EXTERNAL_DIAGNOSTIC_ROOT>", "source_artifact_pins": pins,
        "executed_diagnostic_sha256": DIAGNOSTIC_SHA256, "curator_source_sha256": script_sha,
        "render_environment": plotting,
        "published_summary_transformation": "NONE_EXACT_BYTES", "published_summary_sha256": pins["summary.json"]["sha256"],
        "charts": "Actual audited native scalar metrics and streamed midpoint measured futures; no generated footage.",
        "large_diagnostic_jsonl_copied": False, "original_inputs_modified": False, "training_data_approved": False}
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2, allow_nan=False) + "\n")
    require(contract.sha256_file(Path(__file__)) == script_sha, "curator source changed during publication")
    (output / "SHA256SUMS").write_text("".join(f"{contract.sha256_file(p)}  {p.name}\n" for p in sorted(output.iterdir())))
    return counted


def main(argv=None):
    """HH_260906 - Publish a verified completed diagnostic without source-pin or admission override flags."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("diagnostic_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    publish(args.diagnostic_root, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
