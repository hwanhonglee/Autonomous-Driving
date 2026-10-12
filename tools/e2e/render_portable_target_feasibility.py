#!/usr/bin/env python3
"""HH_260906 - Publish exact audit bytes and measured charts to a new evidence category."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.contract import ContractError, _read_json_and_sha256
from portable_e2e.runtime_contract import RuntimeGateConfig

HORIZONS = ("1.0s", "3.0s", "6.4s")
SPEED_METRIC = "speed_deceleration"
XY_METRIC = "xy_deceleration"


def _require(value, message):
    if not value:
        raise ContractError(message)


def _count(value):
    _require(isinstance(value, int) and not isinstance(value, bool) and value >= 0,
             "audit counts must be nonnegative integers")
    return value


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive_chart_data(report):
    """HH_260906 - Derive phase overlap from measured anchor unions, never add them as independent stops."""
    _require(report.get("schema") == "portable_e2e.target_feasibility_audit.v1"
             and report.get("status") == "TARGET_ENVELOPE_AUDIT_COMPLETE", "source audit is incomplete or unsupported")
    scope = report.get("scope", {})
    _require(scope.get("test_sample_data_opened") is False and scope.get("labels_modified") is False
             and scope.get("model_inference_run") is False, "source audit scope is inconsistent")
    _require(report.get("physical_decoder_limits", {}).get("acceleration_and_deceleration_mps2") == 2.9,
             "publication requires the unchanged 2.9 m/s² decoder envelope")
    for name in ("dataset_manifest_sha256", "contract_sha256", "audit_script_sha256", "model_source_sha256"):
        _require(isinstance(report.get(name), str) and re.fullmatch(r"[0-9a-f]{64}", report[name]),
                 "source audit requires exact SHA-256 provenance")
    for item in report.get("input_manifest", []):
        path = PurePosixPath(item["path"])
        _require(not path.is_absolute() and ".." not in path.parts, "input manifest must use relative paths")
    _require(set(report["splits"]) == {"train", "val"}, "only train and val aggregate statistics may be published")
    split_rows, phase_rows = [], []
    for split in ("train", "val"):
        group = report["splits"][split]
        count = _count(group["sample_count"])
        _require(count > 0, "split sample count must be positive")
        for horizon in HORIZONS:
            stats = group["horizons"][horizon]
            _require(_count(stats["sample_count"]) == count, "horizon denominator differs from split")
            row = {"split": split, "horizon": horizon, "sample_count": count}
            for metric in (SPEED_METRIC, XY_METRIC):
                value = _count(stats["metrics"][metric]["violating_anchor_count"])
                _require(value <= count, "violating anchors exceed denominator")
                row[metric] = value
            split_rows.append(row)
    for episode in report["episodes"]:
        _require(episode["split"] in ("train", "val"), "held-out episode statistics must not be published")
        count = _count(episode["overall"]["sample_count"])
        for metric in (SPEED_METRIC, XY_METRIC):
            measured = episode["overall"]["horizons"]["6.4s"]["metrics"][metric]
            total = _count(measured["violating_anchor_count"])
            phases = measured["violating_anchors_by_capture_phase"]
            _require(set(phases) <= {"pre_tail", "stationary_tail"}, "phase chart requires known capture boundaries")
            before, tail = _count(phases.get("pre_tail", 0)), _count(phases.get("stationary_tail", 0))
            overlap = before + tail - total
            _require(0 <= total <= count and 0 <= overlap <= min(before, tail), "phase anchor union is inconsistent")
            phase_rows.append({"episode_id": episode["episode_id"], "map_id": episode.get("map_id"),
                "split": episode["split"], "metric": metric, "sample_count": count,
                "before_tail_only": before - overlap, "both_phases": overlap, "tail_only": tail - overlap,
                "total_violating_anchors": total})
    for split in ("train", "val"):
        _require(sum(_count(episode["overall"]["sample_count"]) for episode in report["episodes"]
                     if episode["split"] == split) == report["splits"][split]["sample_count"],
                 "episode sample counts differ from split denominator")
    return {"horizons": split_rows, "capture_phases_6p4s": phase_rows}


def _render_charts(data, output):
    # HH_260906 - These are plots of measured JSON values, not screenshots or generated driving footage.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5), sharey=True)
    for axis, split in zip(axes, ("train", "val")):
        rows = [row for row in data["horizons"] if row["split"] == split]
        for metric, offset, color, label in ((SPEED_METRIC, -0.19, "#21618c", "Speed labels"),
                                             (XY_METRIC, 0.19, "#d17b25", "Independent XY")):
            values = [100 * row[metric] / row["sample_count"] for row in rows]
            bars = axis.bar([index + offset for index in range(3)], values, 0.36, color=color, label=label)
            axis.bar_label(bars, labels=[str(row[metric]) for row in rows], padding=3)
        axis.set_xticks(range(3), HORIZONS)
        axis.set_title(f"{split.upper()} | {rows[0]['sample_count']:,} anchor windows")
        axis.set_xlabel("Cumulative future horizon")
        maximum_percent = max(100 * row[metric] / row["sample_count"]
                              for row in rows for metric in (SPEED_METRIC, XY_METRIC))
        axis.set_ylim(0, max(40, maximum_percent + 8))
        axis.grid(axis="y", alpha=0.2)
        axis.set_axisbelow(True)
    axes[0].set_ylabel("Windows exceeding decoder deceleration bound (%)")
    axes[0].legend(frameon=False)
    figure.suptitle("Target deceleration conflicts with the unchanged 2.9 m/s² decoder bound", fontweight="bold")
    figure.text(0.5, 0.025, "Labels above bars = measured counts. Windows overlap; these are not independent stops. No test samples or model driving.",
                ha="center", fontsize=10)
    figure.tight_layout(rect=(0, 0.075, 1, 0.95))
    figure.savefig(output / "01_train_val_horizon_deceleration.png", dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(13, 6.5), sharey=True)
    for axis, metric, title in zip(axes, (SPEED_METRIC, XY_METRIC), ("Speed labels", "Independent XY distances")):
        rows = [row for row in data["capture_phases_6p4s"] if row["metric"] == metric]
        left = [0] * len(rows)
        for key, color, label in (("before_tail_only", "#21618c", "Before tail only"),
                                   ("both_phases", "#7b4f9d", "Both phases"),
                                   ("tail_only", "#d17b25", "Tail only")):
            values = [row[key] for row in rows]
            bars = axis.barh(range(len(rows)), values, left=left, color=color, label=label)
            axis.bar_label(bars, labels=[str(value) if value else "" for value in values], label_type="center", color="white")
            left = [start + value for start, value in zip(left, values)]
        for index, total in enumerate(left):
            axis.text(total + 3, index, str(total), va="center", fontsize=10)
        axis.set_yticks(range(len(rows)), [f"{row['map_id']} ({row['split']})\nn={row['sample_count']}" for row in rows])
        axis.set_xlim(0, max(1, max(left)) * 1.19)
        axis.set_title(title)
        axis.set_xlabel("Violating 6.4 s anchor windows (count)")
        axis.grid(axis="x", alpha=0.2)
        axis.set_axisbelow(True)
    axes[0].invert_yaxis()
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.052), ncol=3, frameon=False)
    figure.suptitle("Goal-tail braking is a contributor, not the only observed conflict", fontweight="bold")
    figure.text(0.5, 0.02, "Disjoint window groups from phase-set intersection; 'before tail' is not necessarily far from the goal. No event-count claim.",
                ha="center", fontsize=10)
    figure.tight_layout(rect=(0, 0.13, 1, 0.95))
    figure.savefig(output / "02_deceleration_capture_phase_overlap.png", dpi=160)
    plt.close(figure)
    return matplotlib.__version__


def _readme(report, data, source_sha):
    train_count = report["splits"]["train"]["sample_count"]
    val_count = report["splits"]["val"]["sample_count"]
    eligible = sum(report["splits"][split]["horizons"]["6.4s"]["horizon_eligible_anchor_count"]
                   for split in ("train", "val"))
    invalid = sum(report["splits"][split]["horizons"]["6.4s"]["invalid_step_count"] for split in ("train", "val"))
    tail_counts = ", ".join(f"{episode.get('map_id')} " + str(episode["overall"]["horizons"]["6.4s"]
        ["metrics"][SPEED_METRIC]["violating_anchors_by_capture_phase"].get("stationary_tail", 0)) + "개"
        for episode in report["episodes"])
    rows = ["| 구간 | 미래 범위 | 전체 앵커 | 속도 라벨 감속 초과 | 독립 XY 감속 초과 |",
            "|---|---:|---:|---:|---:|"]
    for row in data["horizons"]:
        rows.append(f"| {row['split']} | {row['horizon']} | {row['sample_count']:,} | {row[SPEED_METRIC]} | {row[XY_METRIC]} |")
    return "\n".join([
        "<!-- HH_260906 - Publish measured label diagnostics without implying learned driving or deployment readiness. -->",
        "# 01 · 학습·검증 정답 궤적의 감속 한계 점검", "",
        f"현재 physical-v1 모델의 제한을 그대로 두고, 학습 {train_count:,}개·검증 {val_count:,}개 앵커의 정답 데이터를 검사한 자료입니다. "
        "test는 분할을 확인하는 에피소드 메타데이터만 읽었으며, test 샘플·이미지·궤적 통계는 열거나 사용하지 않았습니다.", "",
        "![학습·검증 미래 범위별 감속 충돌](01_train_val_horizon_deceleration.png)", "", *rows, "",
        "각 값은 정답의 누적 미래 구간에 한 번 이상 감속 한계 위반이 있는 **앵커 창 수**입니다. "
        "인접 앵커의 미래 구간은 서로 겹치므로, 서로 다른 급정지 횟수로 해석하면 안 됩니다. "
        f"이 데이터의 {train_count + val_count:,}개 중 {eligible:,}개 앵커는 64개 미래 포인트가 모두 유효하며, "
        f"마스크상 유효하지 않은 미래 포인트는 {invalid:,}개입니다.", "",
        "## 무엇을 계산했나", "",
        "- 속도 라벨: 현재 longitudinal vx를 모델과 동일하게 [0, 8.3333] m/s로 제한한 시작 속도부터 첫 미래 속도까지, "
        "이후 연속한 미래 속도 차이를 실제 100 ms 간격으로 나눴습니다.",
        "- 독립 XY: 현재 원점→첫 미래 위치와 연속한 미래 위치 사이 거리로 속도를 구하고, 그 속도 변화에서 감속을 계산했습니다. "
        "속도 라벨만으로 XY가 불가능하다고 단정하지 않습니다.",
        "- 비교 한계: **모델 디코더 가속·감속 ±2.9 m/s²**입니다. 별개의 runtime speed-rate gate는 "
        "**가속 +3 / 감속 −6 m/s²**이며, 위 그래프는 이 runtime gate의 실패 개수가 아닙니다.",
        "- XY 곡률·횡가속도 및 차량 yaw 대조 진단은 원본 JSON에 포함돼 있습니다. 아주 작은 위치 흔들림·끝점 속도와 "
        "구간 평균 속도의 차이·방향 proxy 특성에 민감하므로, 모든 XY나 모든 주행이 불가능하다는 판정으로 확대하지 않습니다.", "",
        "## 최종 정지 꼬리 구간과 그 이전을 분리", "",
        "![정지 꼬리 구간과 이전 구간 중첩](02_deceleration_capture_phase_overlap.png)", "",
        "`stationary_tail`은 수집 단계 이름입니다. 실제로 그 구간의 시작부터 이미 정지해 있었다는 뜻은 아닙니다. "
        f"속도 라벨의 tail 감속 한계 초과를 포함하는 앵커 수는 {tail_counts}입니다. "
        "그래프는 tail 이전 충돌도 따로 보여주므로 최종 goal-stop 변경만으로 전체 문제가 해결됐다고 할 수 없습니다.", "",
        "그래프의 `Before tail only / Both phases / Tail only`는 앵커 집합의 교집합을 분리한 **중복 없는 창 분류**입니다. "
        "`Before tail`은 목표에서 멀리 떨어진 구간과 동의어가 아닙니다. 현재 앵커가 interior여도 그 미래 창이 goal tail까지 "
        "도달할 수 있습니다. 원본 JSON은 현재 앵커의 목표 근접도와 미래 포인트의 단계·근접도를 따로 보존합니다.", "",
        "## 자료와 재현", "",
        "- [원본 감사 JSON · 바이트 그대로 복사](target_feasibility.json)",
        "- [그래프 입력 수치](chart_data.json)",
        "- [원본·스크립트·모델·계약 해시 및 재현 정보](provenance.json)",
        "- [파일 SHA256SUMS](SHA256SUMS)", "",
        f"원본 보고서 SHA-256: `{source_sha}`", "",
        f"감사 실행 스크립트 SHA-256: `{report['audit_script_sha256']}`", "",
        "보고서 생성 당시 새 감사 스크립트는 작업 트리에 있었으므로 실행 커밋을 주장하지 않습니다. "
        "실행 당시 파일 해시를 보존했으며, 이후 저장소 커밋과는 구분합니다.", "",
        "이 PNG는 **실측 JSON을 그린 그래프**이며 CARLA/Autoware 화면 캡처·GIF·학습 모델의 주행 영상이 아닙니다. "
        "모델 추론·차량 제어·라벨 수정·샘플 필터링·승격은 하지 않았습니다. 원본 라벨을 유지한 상태에서 "
        "수집 제어 개선과 별도의 고정 생성기/선택기 실험을 판단할 근거로만 사용합니다.", ""])


def render_report(source, expected_sha, output):
    source, output = Path(source), Path(output)
    _require(not output.exists() and not output.is_symlink(), "output category already exists")
    report, source_sha = _read_json_and_sha256(source)
    _require(source_sha == expected_sha, "source report SHA-256 mismatch")
    data = derive_chart_data(report)
    _require("/home/" not in json.dumps(report), "source report contains a private absolute home path")
    _require(_sha(REPO / "scripts/e2e/audit_portable_target_feasibility.py") == report["audit_script_sha256"],
             "current audit source differs from the executed source bytes")
    source_bytes = source.read_bytes()
    _require(hashlib.sha256(source_bytes).hexdigest() == source_sha, "source report changed during publication")
    output.mkdir(parents=True, exist_ok=False)
    (output / "target_feasibility.json").write_bytes(source_bytes)
    matplotlib_version = _render_charts(data, output)
    gate = RuntimeGateConfig()
    provenance = {"schema": "portable_e2e.target_feasibility_publication.v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_report_sha256": source_sha, "source_report_copied_byte_identically": True,
        "audit_script_sha256": report["audit_script_sha256"], "model_source_sha256": report["model_source_sha256"],
        "dataset_manifest_sha256": report["dataset_manifest_sha256"], "contract_sha256": report["contract_sha256"],
        "renderer_source_sha256": _sha(Path(__file__)), "matplotlib_version": matplotlib_version,
        "audit_executed_source_commit": None, "audit_source_commit_claim": "Not asserted: audit ran from reviewed working-tree source bytes before commit.",
        "runtime_gate_reference_only": {"maximum_acceleration_mps2": gate.maximum_acceleration_mps2,
            "maximum_deceleration_mps2": gate.maximum_deceleration_mps2,
            "source_sha256": _sha(REPO / "portable_e2e/runtime_contract.py"),
            "used_for_chart_violation_counts": False},
        "artifacts_are": "Measured target-data charts, not screenshots or autonomous driving footage.",
        "reproduce_audit": "CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 scripts/e2e/audit_portable_target_feasibility.py <prepared-v3-root> --dataset-manifest-sha256 "
            + report["dataset_manifest_sha256"] + " --output-json <new-audit.json>",
        "reproduce_publication": "python3 scripts/e2e/render_portable_target_feasibility.py <original-audit.json> --source-sha256 "
            + source_sha + " --output-dir <new-category-directory>"}
    for name, value in (("chart_data.json", data), ("provenance.json", provenance)):
        (output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output / "README.md").write_text(_readme(report, data, source_sha), encoding="utf-8")
    (output / "SHA256SUMS").write_text("".join(f"{_sha(path)}  {path.name}\n" for path in sorted(output.iterdir())), encoding="utf-8")
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_json", type=Path)
    parser.add_argument("--source-sha256", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = render_report(args.source_json, args.source_sha256, args.output_dir)
    except (ContractError, OSError, ValueError, TypeError, KeyError, ImportError) as error:
        print(f"TARGET_FEASIBILITY_PUBLICATION_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": "PUBLISHED_MEASURED_TARGET_CHARTS", "source_report_sha256": result["source_report_sha256"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
