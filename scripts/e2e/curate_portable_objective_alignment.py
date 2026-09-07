#!/usr/bin/env python3
"""HH_260906 - Publish verified aggregate-only train/val objective diagnostics without opening test data."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.contract import ContractError, _loads_json
from scripts.e2e import summarize_portable_selector_weight as campaign_summary
from scripts.e2e.curate_independent_common10_capture_20260907 import sanitize

SEEDS = (20260903, 20260904, 20260905)
ARMS = {"C_expanded_data": 0.1, "D_selector_weight": 0.5}
SCHEMA = "portable_e2e.objective_alignment_diagnostic.v1"
# HH_260906 - Use the exact reviewed manifest rather than a report-selected dataset identity.
MANIFEST_SHA256 = campaign_summary.expansion.MANIFEST_SHA256
CORPUS_SHA256 = campaign_summary.expansion.CORPUS_SHA256
SPLITS = {"train": (1147, "d957e72c1eea755fed5b4ac682e775983861c16104fe083b7cfb6cbb4fed1928"),
          "val": (337, "631be7323f502cafc0dd66766104203bd5e7fee7494436c479bff1e0dddc0285")}
PAIRS = (("selected", "composite_oracle"), ("selected", "ade_oracle"), ("composite_oracle", "ade_oracle"))
MEAN_FIELDS = ("selected_ade_m", "composite_oracle_ade_m", "ade_oracle_ade_m", "selected_minus_ade_oracle_m",
               "composite_oracle_minus_ade_oracle_m", "selected_minus_composite_oracle_ade_m")
SCRIPT = "scripts/e2e/diagnose_portable_objective_alignment.py"


def require(value, message):
    if not value:
        raise ContractError(message)


def sha(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), "evidence must be a regular non-symlink file")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict:
    sha(path)
    result = _loads_json(path.read_text(), str(path))
    require(isinstance(result, dict), "evidence must be a JSON object")
    return result


def redacted(value):
    if isinstance(value, str):
        return re.sub(r"/tmp(?=/|$|[\s\"'])", "${TMP_ROOT}", sanitize(value))
    if isinstance(value, list):
        return [redacted(item) for item in value]
    if isinstance(value, dict):
        return {redacted(key): redacted(item) for key, item in value.items()}
    return value


def source_label(path: Path) -> str:
    path = path.resolve()
    return path.relative_to(REPO).as_posix() if path.is_relative_to(REPO) else redacted(str(path))


def source_hashes(commit: str) -> dict[str, str]:
    # HH_260906 - Check the complete imported Python source map against immutable Git blobs, not current edits.
    require(isinstance(commit, str) and re.fullmatch("[0-9a-f]{40}", commit) is not None, "diagnostic source commit must be pinned")
    result = subprocess.run(["git", "ls-tree", "-r", "--name-only", commit, "portable_e2e",
                             "scripts/e2e/diagnose_portable_selector.py"], cwd=REPO, capture_output=True, text=True, check=True, timeout=15)
    paths = [path for path in result.stdout.splitlines() if path.endswith(".py")]
    require(paths and "scripts/e2e/diagnose_portable_selector.py" in paths, "pinned diagnostic source tree is incomplete")
    return {path: campaign_summary.base._sha(campaign_summary.base._git_bytes(commit, path)) for path in paths}


def finite(value, *, nonnegative=False) -> float:
    require(type(value) in (int, float) and math.isfinite(value) and (not nonnegative or value >= 0), "invalid finite metric")
    return float(value)


def close(actual, expected, context, *, tolerance=1e-6):
    require(math.isclose(finite(actual), expected, rel_tol=tolerance, abs_tol=tolerance), f"recomputed metric differs: {context}")


def validate_summary(summary: dict, count: int) -> tuple[dict, list[str]]:
    # HH_260906 - Recompute every aggregate from recorded original-mask samples before omitting those samples publicly.
    require(summary.get("sample_count") == count and summary.get("candidate_count") == 6 and
            summary.get("predicted_points") == 64, "summary denominator/candidate/horizon mismatch")
    samples = summary.get("per_sample")
    require(isinstance(samples, list) and len(samples) == count, "per-sample denominator mismatch")
    histograms = {name: [0] * 6 for name in ("selected", "composite_oracle", "ade_oracle")}
    matrices = {f"{left}_{right}": [[0] * 6 for _ in range(6)] for left, right in PAIRS}
    sums = {field: 0.0 for field in MEAN_FIELDS}
    ids = []
    for index, sample in enumerate(samples):
        require(isinstance(sample, dict) and type(sample.get("index")) is int and sample["index"] == index,
                "sample order/index mismatch")
        sample_id = sample.get("sample_id")
        require(isinstance(sample_id, str) and sample_id and ":" in sample_id, "sample ID must retain its episode identity")
        ids.append(sample_id)
        mask = sample.get("target_valid")
        require(isinstance(mask, list) and len(mask) == 64 and all(type(value) is bool for value in mask)
                and any(mask) and mask == sorted(mask, reverse=True), "invalid original 64-point prefix mask")
        require(type(sample.get("valid_point_count")) is int and sample["valid_point_count"] == sum(mask), "valid-point denominator mismatch")
        indices = {name: sample.get(f"{name}_index") for name in histograms}
        require(all(type(value) is int and 0 <= value < 6 for value in indices.values()), "candidate index out of range")
        ades = sample.get("candidate_ade_m")
        require(isinstance(ades, list) and len(ades) == 6, "candidate ADE vector mismatch")
        ades = [finite(value, nonnegative=True) for value in ades]
        require(indices["ade_oracle"] == min(range(6), key=lambda candidate: ades[candidate]), "ADE oracle or first-index tie policy mismatch")
        selected, composite, oracle = [ades[indices[name]] for name in ("selected", "composite_oracle", "ade_oracle")]
        expected = dict(zip(MEAN_FIELDS, (selected, composite, oracle, selected - oracle, composite - oracle, selected - composite)))
        for field, expected_value in expected.items():
            close(sample.get(field), expected_value, field)
            sums[field] += sample[field]
        for name, chosen in indices.items():
            histograms[name][chosen] += 1
        for left, right in PAIRS:
            matrices[f"{left}_{right}"][indices[left]][indices[right]] += 1
    require(len(ids) == len(set(ids)), "duplicate sample IDs")
    aggregate = {"sample_count": count, "candidate_count": 6, "predicted_points": 64}
    for name, histogram in histograms.items():
        observed = summary.get(f"{name}_histogram")
        require(isinstance(observed, list) and all(type(value) is int for value in observed) and observed == histogram,
                "recomputed histogram differs")
        aggregate[f"{name}_histogram"] = histogram
    for name, matrix in matrices.items():
        observed = summary.get(f"{name}_confusion_rows_left_columns_right")
        require(isinstance(observed, list) and all(isinstance(row, list) and all(type(value) is int for value in row) for row in observed)
                and observed == matrix, "recomputed confusion matrix differs")
        agreement = sum(matrix[index][index] for index in range(6))
        require(type(summary.get(f"{name}_agreement_count")) is int and summary[f"{name}_agreement_count"] == agreement,
                "recomputed agreement count differs")
        close(summary.get(f"{name}_agreement_rate"), agreement / count, name, tolerance=1e-12)
        aggregate.update({f"{name}_confusion_rows_left_columns_right": matrix,
                          f"{name}_agreement_count": agreement, f"{name}_agreement_rate": agreement / count})
    for field, total in sums.items():
        close(summary.get(f"mean_{field}"), total / count, field, tolerance=1e-12)
        aggregate[f"mean_{field}"] = total / count
    require(set(summary) == set(aggregate) | {"per_sample"}, "unexpected summary fields require review")
    return aggregate, ids


def validate_inputs(input_root: Path, baseline: Path, candidate: Path, commit: str, script_sha: str) -> dict:
    require(re.fullmatch("[0-9a-f]{64}", script_sha or "") is not None and sha(REPO / SCRIPT) == script_sha,
            "diagnostic script SHA-256 differs from reviewed bytes")
    source = source_hashes(commit)
    expected = [f"seed_{seed}/{arm}/objective_alignment.json" for seed in SEEDS for arm in ARMS]
    actual = sorted(path.relative_to(input_root).as_posix() for path in input_root.rglob("objective_alignment.json"))
    require(actual == sorted(expected), "exactly six completed C/D diagnostic files are required")
    campaign = campaign_summary.summarize_campaign(baseline, candidate)
    require(campaign["status"] == "COMPLETE_NOT_PROMOTED", "paired C/D campaign evidence is incomplete")
    records, canonical_ids, runtime = [], {}, None
    for seed in SEEDS:
        for arm, weight in ARMS.items():
            relative = f"seed_{seed}/{arm}"
            path = input_root / relative / "objective_alignment.json"
            report = read_json(path)
            run_root = (baseline if arm == "C_expanded_data" else candidate) / relative
            evaluation, training, audit = [read_json(run_root / name) for name in ("evaluation/metrics.json", "training/run.json", "gate_v8.json")]
            fixed = {"schema": SCHEMA, "status": "DIAGNOSIS_COMPLETE", "diagnostic_source_commit": commit,
                     "diagnostic_script_sha256": script_sha, "source_sha256": source, "manifest_sha256": MANIFEST_SHA256,
                     "corpus_fingerprint_sha256": CORPUS_SHA256, "device": "cpu", "torch_num_threads": 4,
                     "torch_num_interop_threads": 4, "vehicle_control_approved": False,
                     "training_episode_count": 3, "evaluation_episode_count": 1}
            require(all(report.get(key) == value for key, value in fixed.items()), "diagnostic scope/source/fingerprint mismatch")
            require(training.get("train_config", {}).get("seed") == seed, "training seed does not match diagnostic path")
            loss = {"xy_weight": 1.0, "speed_weight": 0.2, "yaw_weight": 0.1, "kinematic_speed_weight": 0.05,
                    "final_displacement_weight": 0.5, "candidate_score_weight": weight}
            require(report.get("loss_config") == training.get("loss_config") == loss, "original checkpoint loss config mismatch")
            for name in ("checkpoint_sha256", "model_config_sha256", "model_parameter_count", "training_dataset_fingerprint_sha256",
                         "training_sampling_plan_sha256", "training_sampling_policy", "training_domain_samples_seen"):
                require(report.get(name) == evaluation.get(name), f"diagnostic checkpoint/evaluation {name} mismatch")
            require(report.get("checkpoint_id") == "portable_e2e.pytorch_checkpoint.v1", "unexpected checkpoint ABI")
            finite(report.get("diagnostic_wall_seconds"), nonnegative=True)
            observed_runtime = report.get("torch_version")
            require(isinstance(observed_runtime, str) and observed_runtime, "diagnostic torch version missing")
            if runtime is None:
                runtime = observed_runtime
            require(observed_runtime == runtime, "diagnostic runtime changed across runs")
            require(set(report.get("splits", {})) == set(SPLITS), "train and val must be separate; test or partial splits forbidden")
            public = redacted(copy.deepcopy(report))
            split_ids = {}
            for split, (count, fingerprint) in SPLITS.items():
                data = report["splits"][split]
                require(data.get("split") == split and data.get("dataset_fingerprint_sha256") == fingerprint,
                        "train/val split label or fingerprint mismatch")
                aggregate, ids = validate_summary(data["summary"], count)
                split_ids[split] = ids
                if split not in canonical_ids:
                    canonical_ids[split] = ids
                require(ids == canonical_ids[split], "sample coverage/order changed across seeds or arms")
                episodes = {sample_id.rsplit(":", 1)[0] for sample_id in ids}
                if split == "train":
                    require(episodes == set(training["training_episode_ids"]), "train sample episode identities differ from checkpoint")
                else:
                    require(len(episodes) == 1 and not episodes.intersection(training["training_episode_ids"]), "validation episode leakage")
                public["splits"][split]["summary"] = aggregate
            require(not set(split_ids["train"]).intersection(split_ids["val"]), "train/val sample leakage")
            require("publication_notice" not in report and "raw_source_sha256" not in report, "reserved publication fields already exist")
            raw_sha = sha(path)
            public.update(publication_notice="Aggregate-only redacted metadata view. Per-sample records are omitted; the private original is retained with raw_source_sha256. Train and val are never pooled.",
                          raw_source_sha256=raw_sha, omitted_per_sample_counts={split: count for split, (count, _) in SPLITS.items()})
            cpu_summary = public["splits"]["val"]["summary"]
            gpu_histogram = [audit["geometry"]["selector"]["selection_counts"][str(index)] for index in range(6)]
            gpu_reference = {"seed": seed, "arm": arm, "split": "val", "sample_count": 337,
                "cpu_selected_ade_m": cpu_summary["mean_selected_ade_m"],
                "original_gpu_selected_ade_m": evaluation["metrics"]["selected_ade_m"],
                "cpu_minus_original_gpu_selected_ade_m": cpu_summary["mean_selected_ade_m"] - evaluation["metrics"]["selected_ade_m"],
                "cpu_selected_histogram": cpu_summary["selected_histogram"], "original_gpu_selected_histogram": gpu_histogram,
                "selected_histograms_match": cpu_summary["selected_histogram"] == gpu_histogram,
                "original_gpu_evaluation_source": source_label(run_root / "evaluation/metrics.json"),
                "original_gpu_evaluation_sha256": sha(run_root / "evaluation/metrics.json"),
                "original_gpu_audit_source": source_label(run_root / "gate_v8.json"),
                "original_gpu_audit_sha256": sha(run_root / "gate_v8.json")}
            records.append({"seed": seed, "arm": arm, "source": path, "raw_source_sha256": raw_sha,
                            "public": public, "checkpoint_sha256": report["checkpoint_sha256"], "gpu_val_reference": gpu_reference})
    return {"records": records, "diagnostic_source_commit": commit, "diagnostic_script_sha256": script_sha,
            "source_sha256": source, "torch_version": runtime, "campaign_summary_status": campaign["status"]}


def render_heatmap(records: list[dict], path: Path) -> None:
    # HH_260906 - Plot measured aggregate agreements only, with separate train and validation denominators.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(19.2, 10.8), dpi=100)
    labels = [f"{row['seed']} / {'C: 0.1' if row['arm'] == 'C_expanded_data' else 'D: 0.5'}" for row in records]
    for axis, (split, (count, _)) in zip(axes, SPLITS.items()):
        values = [[100 * row["public"]["splits"][split]["summary"][f"{left}_{right}_agreement_rate"]
                   for left, right in PAIRS] for row in records]
        axis.imshow(values, vmin=0, vmax=100, cmap="Blues", aspect="auto")
        axis.set_xticks(range(3), ["Selected / loss oracle", "Selected / ADE oracle", "Loss / ADE oracle"], rotation=20, ha="right")
        axis.set_yticks(range(6), labels)
        axis.set_title(f"{split.upper()} — {count} samples per model\n" + ("In-sample diagnosis; not generalization" if split == "train" else "Development validation; NOT held-out test"))
        for row, values_row in enumerate(values):
            for column, value in enumerate(values_row):
                axis.text(column, row, f"{value:.2f}%", ha="center", va="center", color="white" if value > 55 else "black")
    figure.suptitle("C/D CPU objective alignment — three paired seeds, separate train and val", fontsize=18)
    figure.tight_layout(rect=(0, 0.03, 1, 0.94))
    with path.open("xb") as stream:
        figure.savefig(stream, format="png", dpi=100)
    plt.close(figure)


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + "\n")


def publish(input_root: Path, baseline: Path, candidate: Path, output: Path, commit: str, script_sha: str) -> dict:
    require(not output.exists() and not output.is_symlink(), "publication directory must be new")
    verified = validate_inputs(input_root, baseline, candidate, commit, script_sha)
    output.mkdir(parents=True, exist_ok=False)
    records, manifest, rows = verified["records"], [], []
    for record in records:
        path = output / "runs" / f"seed_{record['seed']}" / record["arm"] / "objective_alignment_aggregate.json"
        require(sha(record["source"]) == record["raw_source_sha256"], "raw diagnosis changed before publication")
        write_json(path, record["public"])
        manifest.append({"published_path": path.relative_to(output).as_posix(), "sha256": sha(path), "size_bytes": path.stat().st_size,
                         "source_path": source_label(record["source"]), "raw_source_sha256": record["raw_source_sha256"],
                         "raw_source_size_bytes": record["source"].stat().st_size, "representation": "aggregate_only_redacted_metadata_view"})
        for split in SPLITS:
            rows.append({"seed": record["seed"], "arm": record["arm"], "split": split, "device": "cpu",
                         "checkpoint_sha256": record["checkpoint_sha256"], **record["public"]["splits"][split]["summary"]})
    summary = {"schema": "portable_e2e.objective_alignment_publication.v1", "status": "VERIFIED_AGGREGATES_PUBLISHED",
               "vehicle_control_approved": False, "automatic_promotion": False, "test_opened": False,
               "diagnostic_source_commit": commit, "diagnostic_script_sha256": script_sha,
               "manifest_sha256": MANIFEST_SHA256, "corpus_fingerprint_sha256": CORPUS_SHA256,
               "rows": rows, "raw_input_count": 6, "raw_per_sample_records_omitted": 6 * (1147 + 337),
               "cpu_gpu_val_checks": [record["gpu_val_reference"] for record in records],
               "limitations": ["All histograms, confusion matrices, agreement rates and means were recomputed from recorded private per-sample rows.",
                   "Composite oracle indices originate from the pinned diagnostic's unchanged training loss; this publisher does not rerun models or reconstruct full tensor losses.",
                   "Train is in-sample diagnosis, val is development validation; neither establishes independent test performance or safe driving.",
                   "CPU reanalysis is separate from original GPU screening; different histograms establish changed decisions, not their cause. Paired logits/margins were not recorded."]}
    write_json(output / "summary.json", summary)
    render_heatmap(records, output / "01_train_val_objective_agreement.png")
    lines = ["# C/D CPU 학습 목표·평가 목표 일치 진단", "", "<!-- HH_260906 - Publish verified aggregates without pooling train and validation or exposing per-sample records. -->", "",
             "C는 score weight 0.1, D는 0.5입니다. 각각 3개 seed, 동일 v3 데이터의 train 1,147개와 val 337개를 CPU에서 따로 분석했습니다. 아래 집계는 원래 GPU 평가를 대체하지 않습니다.",
             "Train은 학습에 사용한 자료의 진단이지 일반화 성능이 아닙니다. Val은 개발 검증이며 Town04 held-out test는 열지 않았습니다.", "",
             "![학습·검증 분리 일치율](01_train_val_objective_agreement.png)", "",
             "| Seed | Arm | Split | N | 선택↔loss oracle | 선택↔ADE oracle | loss↔ADE oracle | Selected ADE m | Composite ADE m | ADE oracle m |",
             "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for row in rows:
        agreements = [f"{100 * row[f'{left}_{right}_agreement_rate']:.2f}%" for left, right in PAIRS]
        means = [f"{row[f'mean_{name}']:.5f}" for name in MEAN_FIELDS[:3]]
        lines.append(f"| {row['seed']} | {row['arm']} | {row['split']} | {row['sample_count']} | " + " | ".join(agreements + means) + " |")
    lines.extend(["", "## CPU 재분석과 원래 GPU val 평가", "",
        "같은 checkpoint라도 아래 원본 GPU 평가 수치는 그대로 보존합니다. histogram 불일치는 선택 결과가 달라졌음을 보여주지만 near-tie나 backend가 원인인지는 저장된 paired logits·margin이 없어 확정하지 않습니다.", "",
        "| Seed | Arm | CPU val ADE m | 원래 GPU val ADE m | CPU−GPU m | 선택 histogram 동일 |",
        "| --- | --- | ---: | ---: | ---: | --- |"])
    for record in records:
        item = record["gpu_val_reference"]
        lines.append(f"| {item['seed']} | {item['arm']} | {item['cpu_selected_ade_m']:.8f} | {item['original_gpu_selected_ade_m']:.8f} | "
                     f"{item['cpu_minus_original_gpu_selected_ade_m']:+.8f} | {'YES' if item['selected_histograms_match'] else 'NO'} |")
    lines.extend(["", "원래 학습 loss의 최적 후보(composite oracle)와 XY ADE만 최소인 후보(ADE oracle)는 정의가 다릅니다.",
                  "공개 JSON은 aggregate-only 치환본이며 `per_sample` 8,904개를 생략했습니다. 원본은 삭제하지 않았고 각 공개 파일에 원본 SHA를 남겼습니다.",
                  "히스토그램·혼동행렬·일치율·평균을 원본 sample별 기록으로 재계산했지만 이 발행 단계에서 모델이나 loss 텐서를 다시 실행하지는 않았습니다.",
                  "평균이나 train/val을 합쳐 하나의 성능 점수로 만들지 않습니다. CPU 진단 시간은 10 Hz 추론 성능 또는 차량 안전의 증거가 아닙니다.",
                  "", "- [검증 집계](summary.json)", "- [원본·공개본 해시](publication_manifest.json)", "- [전체 발행 파일 체크섬](SHA256SUMS)", ""])
    with (output / "README.md").open("x", encoding="utf-8") as stream:
        stream.write("\n".join(lines))
    require(all(sha(record["source"]) == record["raw_source_sha256"] for record in records), "raw diagnosis changed during publication")
    write_json(output / "publication_manifest.json", {"files": manifest, "vehicle_control_approved": False,
        "comment": "HH_260906 - Preserve private originals and bind each aggregate-only derivative to exact raw and public byte hashes."})
    files = sorted(path for path in output.rglob("*") if path.is_file())
    with (output / "SHA256SUMS").open("x") as stream:
        stream.write("".join(f"{sha(path)}  {path.relative_to(output).as_posix()}\n" for path in files))
    return {"published_files": len(files) + 1, "source_reports": 6, "summary_rows": len(rows), "output": str(output)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_root", type=Path)
    parser.add_argument("--baseline-campaign", required=True, type=Path)
    parser.add_argument("--candidate-campaign", required=True, type=Path)
    parser.add_argument("--diagnostic-source-commit", required=True)
    parser.add_argument("--diagnostic-script-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        print(json.dumps(publish(args.input_root, args.baseline_campaign, args.candidate_campaign, args.output,
                                 args.diagnostic_source_commit, args.diagnostic_script_sha256)))
    except (ContractError, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print(f"OBJECTIVE_PUBLICATION_ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
