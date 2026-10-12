#!/usr/bin/env python3
"""HH_261002 - Compare all seven known pixel classes against the exact unchanged map parent, without inference."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PARENT_ROOT = ROOT / "storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1"
RUN_ROOT = ROOT / "storage/experiments/training/2026-10-02/map_paired_balanced_completed_run_v1"
PUBLICATION = ROOT / "docs/assets/validation/2026-10-02/map_paired_balanced_results_v1"
PARENT_METRICS_SHA = "0cc82311b89d8dc37697ff65da6e1d8e38f58f8cb225887af345cdfdc4511171"
PARENT_SUMMARY_SHA = "127c15564bd2180eee4061fd6993f52f00bac0dcbc65aecc6505b5dae425b606"
PARENT_WEIGHTS_SHA = "ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e"
PARENT_CHECKPOINT_SHA = "410358363d1c9cdc5551a9cdc3f37fa1bd336a1c3fd17ea64e9a71cd11d74abf"
PUBLISHER_SHA = "33c717ddf51c1dc6aa2e56c15c8e131d5866983c97fc19b0421a5c0bf9ba1808"
SCHEMA = "portable_e2e.map_parent_control_balanced_comparison.20261002.v1"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
SPLITS = {
    "dev_town01": ("town01", 118, "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"),
    "holdout_town02_p02": ("p02", 232, "858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb"),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def canonical(path):
    """HH_261002 - Refuse escaped paths and all symlink ancestors, including fresh output parents."""
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts, "absolute nonescaped path required")
    require(not any(item.is_symlink() for item in (path, *path.parents)), "symlink path forbidden")
    require(path.resolve() == path, "canonical path required")
    return path


def sha_string(value):
    require(type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value),
            "lowercase SHA256 required")
    return value


class Pins:
    """HH_261002 - Bind exactly the first parsed bytes; reread every input before any derived write."""

    def __init__(self):
        self.values = {}

    def payload(self, path, expected):
        path = canonical(path)
        require(path.is_file(), "regular input file required: " + str(path))
        value = path.read_bytes()
        sha = hashlib.sha256(value).hexdigest()
        require(sha == sha_string(expected), "caller/frozen input SHA differs: " + str(path))
        require(str(path) not in self.values or self.values[str(path)] == sha, "first-read input changed")
        self.values[str(path)] = sha
        return value

    def document(self, path, expected):
        value = json.loads(self.payload(path, expected))
        require(type(value) is dict, "JSON object required")
        return value

    def recheck(self):
        for name, sha in self.values.items():
            self.payload(Path(name), sha)


def metrics_from_confusion(matrix, *, expected_pixels=None):
    """HH_261002 - Rows are actual classes, columns predicted; preserve all49 cells and undefined ratios."""
    require(type(matrix) is list and len(matrix) == 7 and all(type(row) is list and len(row) == 7
            and all(type(v) is int and v >= 0 for v in row) for row in matrix),
            "all49 nonnegative integer confusion cells required")
    target = [sum(row) for row in matrix]
    predicted = [sum(row[i] for row in matrix) for i in range(7)]
    diagonal = [matrix[i][i] for i in range(7)]
    total = sum(target)
    require(total > 0 and (expected_pixels is None or total == expected_pixels), "full-FOV pixel denominator differs")
    result = {"confusion": [list(row) for row in matrix], "pixel_count": total,
              "pixel_accuracy": sum(diagonal)/total}
    for key, values in (("target_pixels", target), ("predicted_pixels", predicted),
                        ("true_positive_pixels", diagonal),
                        ("false_negative_pixels", [a-b for a, b in zip(target, diagonal)]),
                        ("false_positive_pixels", [a-b for a, b in zip(predicted, diagonal)])):
        result[key] = dict(zip(LABELS, values))
    result["iou"] = {label: diagonal[i]/(target[i]+predicted[i]-diagonal[i]) if target[i] else None
                     for i, label in enumerate(LABELS)}
    result["precision"] = {label: diagonal[i]/predicted[i] if predicted[i] else None for i, label in enumerate(LABELS)}
    result["recall"] = {label: diagonal[i]/target[i] if target[i] else None for i, label in enumerate(LABELS)}
    return result


def numeric_equal(actual, expected):
    if expected is None:
        return actual is None
    return type(actual) in (int, float) and math.isfinite(actual) and math.isclose(actual, expected, rel_tol=0., abs_tol=1e-12)


def compare_reported(row, calculated, *, parent):
    """HH_261002 - Validate saved metrics instead of trusting headline IoU or summary-only claims."""
    for field in (("iou", "precision") if parent else ("iou",)):
        require(type(row.get(field)) is dict and set(row[field]) == set(LABELS)
                and all(numeric_equal(row[field][label], calculated[field][label]) for label in LABELS),
                "saved seven-class ratio differs: " + field)
    require(numeric_equal(row.get("pixel_accuracy"), calculated["pixel_accuracy"]), "saved pixel accuracy differs")
    if parent:
        require(row.get("true_class_pixels") == list(calculated["target_pixels"].values()), "parent actual class supports differ")
    else:
        for field in ("target_pixels", "predicted_pixels", "false_negative_pixels", "false_positive_pixels", "pixel_count"):
            require(row.get(field) == calculated[field], "saved full49 accounting differs: " + field)


def comparison(parent, published):
    """HH_261002 - Show both continuation gain and appearance tradeoff on the same target pixels; add no gates."""
    output = {}
    require(set(published) == set(SPLITS), "exact previously exposed118/232 evaluation splits required")
    for split, (parent_name, count, sha) in SPLITS.items():
        original = parent[parent_name]
        pm = metrics_from_confusion(original.get("confusion"), expected_pixels=count*360*640)
        compare_reported(original, pm, parent=True)
        require(set(published[split]) == {"control", "balanced"}, "exact two published arms required")
        models = {"parent": pm}
        for arm in ("control", "balanced"):
            row = published[split][arm]
            require(type(row.get("count")) is int and row["count"] == count and row.get("source_sha256") == sha
                    and row.get("known_exposed_diagnostic_not_independent") is True, "known evaluation count/source scope differs")
            current = metrics_from_confusion(row.get("confusion"), expected_pixels=count*360*640)
            compare_reported(row, current, parent=False)
            require(current["target_pixels"] == pm["target_pixels"], "parent/new full target class supports differ")
            models[arm] = current
        deltas = {}
        for left, right in (("control", "parent"), ("balanced", "parent"), ("balanced", "control")):
            deltas[left + "_minus_" + right] = {
                label: None if models[left]["iou"][label] is None or models[right]["iou"][label] is None
                else 100*(models[left]["iou"][label]-models[right]["iou"][label]) for label in LABELS}
        output[split] = {"count": count, "source_sha256": sha, "models": models, "iou_delta_percentage_points": deltas}
    return output


def audit(args):
    """HH_261002 - Read only existing pixel publication and exact parent/endpoint bytes; never load NPZ or PT."""
    run = canonical(RUN_ROOT)
    publication = canonical(PUBLICATION)
    require(canonical(args.results_json) == publication / "results.json", "fixed completed publication required")
    pins = Pins()
    report = pins.document(args.results_json, args.results_sha256)
    require(report.get("schema") == "portable_e2e.map_paired_balanced_results.20261002.v1"
            and report.get("status") == "FULL_ARCHIVE_AND_PIXEL_EVALUATION_VERIFIED_NOT_MODEL_APPROVAL"
            and report.get("study_id") == "map_paired_balanced_20261002_v1"
            and report.get("parent_weights_sha256") == PARENT_WEIGHTS_SHA
            and report.get("labels") == list(LABELS), "published source/version/parent differs")
    for field, expected in (("independent_new_dev_sites", 0), ("formal_main_functions_approved", 0),
                            ("formal_sub_functions_approved", 0), ("native_diagnostic_denials_preserved", True),
                            ("RGB_only_seven_class_perception_not_HD_AMG", True),
                            ("known_evaluation_used_for_fit_or_checkpoint_selection", False),
                            ("model_promoted", False), ("vehicle_control_approved", False)):
        require(type(report.get(field)) is type(expected) and report[field] == expected, "research scope differs: " + field)
    publisher = ROOT / "tools/e2e/publish_map_paired_balanced_results_20261002.py"
    pins.payload(publisher, PUBLISHER_SHA)
    archive = report.get("archived_publication_sources", {}).get("source/" + publisher.name, {})
    require(archive == {"original_path": str(publisher), "sha256": PUBLISHER_SHA}, "published source archive differs")
    pins.payload(publication / "source" / publisher.name, PUBLISHER_SHA)
    summary = pins.document(run / "summary.json", args.caller_summary_sha256)
    require(report.get("caller_summary_sha256") == args.caller_summary_sha256
            and summary.get("parent_weights_sha256") == PARENT_WEIGHTS_SHA
            and summary.get("known_evaluation") is not None, "caller completed run/parent pointer differs")
    first = report.get("first_read_source_and_artifact_pins", {})
    require(first.get(str(run / "summary.json")) == args.caller_summary_sha256, "original publisher first-read summary pin differs")
    for arm in ("control", "balanced"):
        info = report.get("arms", {}).get(arm, {})
        require(summary.get("arms_result", {}).get(arm, {}).get("steps") == 10000
                and summary["arms_result"][arm].get("examples_seen") == 80000, "fixed final10000/80000 endpoint differs")
        for filename, key in (("weights.npz", "weights_sha256"), ("checkpoint.pt", "checkpoint_sha256")):
            sha = sha_string(info.get(key))
            require(summary["arms_result"][arm].get(key) == sha and first.get(str(run / arm / filename)) == sha,
                    "published endpoint hash pointer differs")
            pins.payload(run / arm / filename, sha)
    parent = pins.document(PARENT_ROOT / "B/metrics.json", PARENT_METRICS_SHA)
    parent_summary = pins.document(PARENT_ROOT / "summary.json", PARENT_SUMMARY_SHA)
    require(parent_summary.get("B") == parent and parent.get("parameter_count") == 245831
            and parent.get("weights_sha256") == PARENT_WEIGHTS_SHA
            and parent.get("checkpoint_sha256") == PARENT_CHECKPOINT_SHA, "exact245831 parent metrics/summary linkage differs")
    pins.payload(PARENT_ROOT / "B/weights.npz", PARENT_WEIGHTS_SHA)
    pins.payload(PARENT_ROOT / "B/checkpoint.pt", PARENT_CHECKPOINT_SHA)
    calculated = comparison(parent, report.get("evaluation", {}))
    require(type(summary["known_evaluation"]) is dict and set(summary["known_evaluation"]) == set(SPLITS),
            "caller full evaluation splits differ")
    for split, (_, count, sha) in SPLITS.items():
        require(set(summary["known_evaluation"][split]) == {"control", "balanced"}, "caller evaluation arms differ")
        for arm in ("control", "balanced"):
            row = summary["known_evaluation"][split][arm]
            require(row.get("confusion") == calculated[split]["models"][arm]["confusion"]
                    and type(row.get("count")) is int and row["count"] == count
                    and row.get("source_sha256") == sha
                    and row.get("known_exposed_diagnostic_not_independent") is True,
                    "caller/published confusion/count/source differs")
    pins.recheck()
    return {"schema": SCHEMA, "evaluation": calculated, "labels": list(LABELS),
            "scope": "KNOWN_EXPOSED_PIXEL_DIAGNOSTIC_NOT_HD_AMG_OR_DRIVING_APPROVAL",
            "main_functions_approved": 0, "sub_functions_approved": 0, "independent_new_dev_sites": 0,
            "first_read_pins": dict(pins.values), "model_inference_executed": False,
            "NPZ_or_PT_loaded": False, "gate_or_threshold_added": False}, pins


def percent(value):
    return "미측정" if value is None else f"{100*value:.2f}%"


def pp(value):
    return "미측정" if value is None else f"{value:+.3f} pp"


def markdown(report):
    """HH_261002 - Make every class, denominator and false-positive tradeoff readable; never select favorable rows."""
    lines = ["# 기존 모델과 새 지도 학습 결과 비교", "",
             "**기존 parent 대비 개선**과 **같은 추가 학습에서 OFF/ON 혼합이 준 영향**은 다른 비교입니다. 아래에서 둘을 따로 표시합니다.", "",
             "parent는 Oct1의 동일 245,831-parameter 모델이며, control과 balanced는 그 가중치에서 각각 고정 10,000회 학습한 결과입니다.", "",
             "Town01 DEV 118장과 Town02 p02 holdout 232장은 이미 확인한 자료입니다. 새 독립 평가 지역은 0개이며, 메인 0/9·서브 0/30 공식 승인은 유지됩니다.", "",
             "이 수치는 RGB 영상의 7개 분류 결과입니다. 신호 상태·횡단보도·정지선·차선 연결·정밀 좌표를 갖춘 자동 정밀지도(AMG)나 자율주행 성공을 뜻하지 않습니다.", ""]
    for split, comparison_row in report["evaluation"].items():
        models, deltas = comparison_row["models"], comparison_row["iou_delta_percentage_points"]
        lines += [f"## {split}: {comparison_row['count']}장 전체 화면", "",
                  "| 분류 | 기존 IoU | control IoU | balanced IoU | control − 기존 | balanced − 기존 | balanced − control |",
                  "|---|---:|---:|---:|---:|---:|---:|"]
        for label in LABELS:
            lines.append("| " + label + " | " + " | ".join([percent(models[arm]["iou"][label]) for arm in ("parent", "control", "balanced")]
                + [pp(deltas[key][label]) for key in ("control_minus_parent", "balanced_minus_parent", "balanced_minus_control")]) + " |")
        lines += ["", "### 무엇을 놓치고, 잘못 표시했는지", "",
                  "정밀도(P)는 표시한 것 중 맞은 비율, 재현율(R)은 실제 대상 중 찾아낸 비율입니다. FN은 누락, FP는 오탐 픽셀 수입니다. 작은 물체는 도로 점수와 별도로 보아야 합니다.", "",
                  "| 분류 | 모델 | P | R | 정답 픽셀 | FN | FP |", "|---|---|---:|---:|---:|---:|---:|"]
        for label in LABELS:
            for arm, values in models.items():
                lines.append(f"| {label} | {arm} | {percent(values['precision'][label])} | {percent(values['recall'][label])} | {values['target_pixels'][label]} | {values['false_negative_pixels'][label]} | {values['false_positive_pixels'][label]} |")
        lines += ["", "<details>", "<summary>모든 49칸 confusion 보기: 행=정답, 열=예측</summary>", ""]
        for arm, values in models.items():
            lines += [f"### {arm}", "", "| 정답 / 예측 | " + " | ".join(LABELS) + " |", "|---|" + "---:|"*7]
            for label, row in zip(LABELS, values["confusion"]): lines.append("| " + label + " | " + " | ".join(map(str, row)) + " |")
            lines += ["", "전체 픽셀 정확도: " + percent(values["pixel_accuracy"]) + ".", ""]
        lines += ["</details>", ""]
    lines += ["## 해석 범위와 다음 단계", "",
              "정답이 없는 분류의 IoU·재현율은 미측정이며 오탐은 남깁니다. 예측이 없는 분류의 정밀도도 미측정입니다. 미측정을 0점이나 성공으로 바꾸지 않았고, 새 합격 기준을 만들지 않았습니다.", "",
              "control − 기존은 같은 알려진 자료에서 추가 학습 후의 변화입니다. balanced − control은 장면·정답·학습 순서·crop이 같은 두 팔 사이에서 OFF/ON 혼합이 준 영향입니다. 양수를 일반화 성공으로, 일부 개선을 모든 분류 개선으로 확대하지 않습니다.", "",
              "다음 지도 작업은 작은 물체의 거리·크기별 누락/오탐과 모든 480개 paired 장면의 appearance 영향 진단, 새 지역·날씨·rig의 고정 DEV 확보가 우선입니다. HD AMG에는 실제 보정·위치·깊이/삼각측량, 차선 vector·연결 topology와 정적 landmark의 다중 프레임 검증이 별도로 필요합니다.", "",
              "새 RGB 960장은 같은 480개 물리 장면의 두 조건입니다. 독립 장소 960개가 아닙니다. [기존 결과](README.md)의 원자료 미승인·실패 기록·전체 결과를 변경하지 않았습니다.", "",
              "## 원본 추적", "", "모델 추론·Torch·pickle·NPZ/PT 역직렬화 없이 저장된 confusion과 원본 바이트 해시만 읽었습니다. 비교표는 고정 source/caller pins를 확인한 연구 진단입니다.", ""]
    for path, sha in report["first_read_pins"].items(): lines.append(f"- `{path}` — `{sha}`")
    return "\n".join(lines) + "\n"


def publish(args):
    output = canonical(args.output)
    require(output == PUBLICATION / "PARENT_COMPARISON.md" and not output.exists(), "new fixed comparison file required")
    require(output.parent.is_dir() and (output.parent / "README.md").is_file(), "original publication must finish first")
    report, pins = audit(args)
    body = markdown(report)
    pins.recheck()
    if args.validate_only:
        return {"schema": SCHEMA, "published": False, "verified_split_counts": {name: row[1] for name, row in SPLITS.items()}}
    # HH_261002 - Exclusive create is the only write; never rewrite the publisher README, inputs or old comparisons.
    with output.open("x", encoding="utf-8") as stream:
        stream.write(body)
    pins.recheck()
    return {"schema": SCHEMA, "published": True, "output": str(output),
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--results-json", type=Path, default=PUBLICATION / "results.json")
    parser.add_argument("--results-sha256", required=True)
    parser.add_argument("--caller-summary-sha256", required=True)
    parser.add_argument("--output", type=Path, default=PUBLICATION / "PARENT_COMPARISON.md")
    parser.add_argument("--validate-only", action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)), ensure_ascii=False, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
