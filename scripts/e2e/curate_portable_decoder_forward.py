#!/usr/bin/env python3
"""HH_260906 - Publish exact CPU forward evidence and retain the separate initial-audit rendering failure without changing old reports."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

from scripts.e2e import audit_portable_decoder_probe as base

ROOT = Path(__file__).resolve().parents[2]
FORWARD_SHA = "5230b1004dbeed1eeafcee2c78517788c6ed8b0d8fcc7fa02e8f0de4721112d2"
REFERENCE_SHA = "b79384cd6fadd750047a3bdd84cb51dda9d6c2aab7208a1cd1b18a1b824a1c07"
require = base.require


def exact_files(root, names, *, checksum_manifest=False):
    values = {}
    for name in names:
        path = base.contract._safe_file(root, name, "CPU verification publication")
        values[name] = base.contract._read_regular_file_bounded(path, 256 * 1024 * 1024, "CPU verification publication")
        require(b"/home/" not in values[name], "private account path cannot be copied into public evidence")
    if checksum_manifest:
        entries = [line.split("  ") for line in values["SHA256SUMS"].decode().splitlines()]
        require(len(entries) == len(names) - 1 and {name for _, name in entries} == set(names) - {"SHA256SUMS"}, "forward output hash inventory differs")
        for expected, name in entries:
            require(hashlib.sha256(values[name]).hexdigest() == expected, "forward output hash mismatch")
    return values


def parse_rows(contents):
    return [base.evidence._loads(line) for line in contents.splitlines()]


def validate_initial_summary(summary, records):
    require(len(records) == summary["initial_forward_unverified_count"] and summary["initial_forward_unverified_candidates"] == records
        and summary["status"] == "INITIALIZATION_UNVERIFIED_NOT_ADMITTED", "initial partial mismatch aggregate differs")


def validate_rows(summary, rows, originals):
    # HH_260906 - Recount all candidate comparisons and preserved original failures, rather than trusting the summary label.
    require(len(rows) == len(originals), "forward anchor count differs")
    mismatches, candidates, maxima, original_gates = 0, 0, {"x_m": 0., "y_m": 0., "speed_mps": 0.}, Counter()
    for index, (row, original) in enumerate(zip(rows, originals)):
        require(row["anchor_index"] == index and row["sample_id"] == original["sample_id"] and row["trial_id"] == original["trial_id"]
            and row["original_raw_violating_step_counts"] == original["original_raw_violating_step_counts"]
            and len(row["candidates"]) == len(original["candidates"]) == 6, "forward row/source identity differs")
        for k, (candidate, reference) in enumerate(zip(row["candidates"], original["candidates"])):
            require(candidate["initialization_index"] == k and candidate["compared_scalar_count"] == 192
                and candidate["original_runtime_gate"] == {"status": reference["runtime_gate_status"], "reason": reference["runtime_gate_reason"]},
                "candidate denominator or original gate changed")
            records = candidate["mismatches"]
            require(len(records) == candidate["mismatch_scalar_count"] and len({(r["point_index"], r["component"]) for r in records}) == len(records),
                "mismatch ledger missing or duplicated")
            require(candidate["status"] == ("UNVERIFIED" if records else "MATCH_WITH_PREDECLARED_TOLERANCE"), "candidate status differs")
            for name, value in candidate["maximum_absolute_difference_by_component"].items():
                require(name in maxima and isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0,
                    "invalid forward maximum difference")
                maxima[name] = max(maxima[name], value)
            require(set(candidate["maximum_absolute_difference_by_component"]) == set(maxima), "component maximum omitted")
            for r in records:
                require(type(r["point_index"]) is int and 0 <= r["point_index"] < 64 and r["component"] in maxima, "invalid mismatch coordinate")
                a, b = r["recorded_gpu_value"], r["replayed_cpu_value"]
                require(all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (a, b))
                    and not math.isclose(a, b, abs_tol=1e-5, rel_tol=1e-5) and r["absolute_difference"] == abs(a - b)
                    and candidate["maximum_absolute_difference_by_component"][r["component"]] >= abs(a - b), "mismatch is inside tolerance or unbound to maximum")
            mismatches += len(records); candidates += bool(records)
            original_gates[candidate["original_runtime_gate"]["status"]] += 1
    require(summary["anchor_count"] == len(rows) and summary["candidate_count"] == len(rows) * 6
        and summary["point_count"] == len(rows) * 384 and summary["compared_scalar_count"] == len(rows) * 1152
        and summary["mismatch_scalar_count"] == mismatches and summary["mismatch_candidate_count"] == candidates
        and summary["maximum_absolute_difference_by_component"] == maxima and summary["original_runtime_gate_counts"] == dict(original_gates)
        and summary["status"] == ("UNVERIFIED_NOT_ADMITTED" if mismatches else "MATCHED_NOT_ADMITTED"), "forward aggregate differs from complete rows")


def validate_forward(values, reference, reference_rows):
    summary = base.evidence._loads(values["summary.json"])
    require(summary["schema"] == "portable_e2e.independent_decoder_final_forward.v1" and summary["anchor_count"] == 1337
        and summary["device"] == "cpu" and summary["cuda_visible_devices"] == "" and summary["torch_threads"] == 4
        and summary["comparison_tolerance"] == {"absolute": 1e-5, "relative": 1e-5} and summary["source_and_input_postcheck_pass"] is True,
        "forward scope, denominator or tolerance differs")
    require(summary["source_sha256"] == {**reference["auditor_source_sha256"], "scripts/e2e/audit_portable_decoder_forward.py": FORWARD_SHA}
        and summary["historical_execution_source_proof"] == reference["historical_execution_source_proof"]
        and summary["input_pins"] == reference["input_verification"]["pins"], "forward source/input evidence differs")
    expected_keys = {"owner_started.json", "owner_result.json", "probe/started.json", "probe/summary.json", "probe/per_anchor.jsonl",
        "probe/optimization_history.jsonl", "probe/SHA256SUMS"}
    require(set(summary["original_gpu_output_pins"]) == expected_keys and all(pin == reference["owned_artifact_pins"][path]
        for path, pin in summary["original_gpu_output_pins"].items()), "original GPU output binding differs")
    require(summary["scope"] == {"no_grad": True, "model_parameters_instantiated": False, "optimizer_instantiated": False,
        "optimization_replayed": False, "training_data_approved": False, "runtime_gate_changed": False,
        "original_gpu_outputs_replaced": False, "jpeg_or_raw_capture_payload_read": False}, "forward approval or optimization scope differs")
    batches = summary["batches"]
    require(len(batches) == 6 and all(b["batch_start_anchor_index"] == index * 256 and b["batch_size"] == min(256, 1337 - index * 256)
        and all(re.fullmatch(r"[a-f0-9]{64}", b[key]) for key in ("optimized_latent_float32_sha256", "cpu_xy_float32_sha256", "cpu_speed_float32_sha256"))
        for index, b in enumerate(batches)), "forward batch identity or tensor SHA missing")
    rows = parse_rows(values["per_anchor_forward.jsonl"])
    validate_rows(summary, rows, reference_rows)
    require(summary["original_raw_curvature_failed_anchor_count"] == sum(bool(r["original_raw_violating_step_counts"]["xy_curvature"]) for r in rows) == 299,
        "original curvature failures omitted")
    return summary


def compare_initial_partial(values, reference, reference_rows):
    summary = base.evidence._loads(values["summary.json"])
    verification = base.evidence._loads(values["input_verification.json"])
    rows = parse_rows(values["per_anchor_numeric.jsonl"])
    require(summary["schema"] == reference["schema"] and summary["auditor_source_sha256"] == reference["auditor_source_sha256"]
        and summary["input_verification"] == verification == reference["input_verification"]
        and summary["owned_artifact_pins"] == reference["owned_artifact_pins"] and summary["comparison_tolerance"] == reference["comparison_tolerance"],
        "remote initial partial source/raw/input binding differs")
    require(summary["final_recorded_prediction_errors_and_gates"] == "VERIFIED" and summary["scope"] == reference["scope"]
        and summary["runtime_gate_failures"] == reference["runtime_gate_failures"] and len(rows) == len(reference_rows) == 1337,
        "remote initial numeric scope/failures differ")
    require("CLI exited 1" in values["EXECUTION_STATUS.md"].decode() and "ModuleNotFoundError" in values["EXECUTION_STATUS.md"].decode(),
        "partial rendering failure record missing")
    require(len(summary["initial_forward_checks"]) == 6 and all(check["status"] == "INITIAL_LATENT_SHA_MATCH"
        and check["sha256"] == before["sha256"] and check["cpu_initial_forward_only"] is True
        for check, before in zip(summary["initial_forward_checks"], reference["initial_forward_checks"])), "initial latent byte binding differs")
    discrepancies, discrepancy_records, largest_cpu_delta = [], [], 0.
    for row, original in zip(rows, reference_rows):
        require(row["sample_id"] == original["sample_id"] and row["anchor_index"] == original["anchor_index"] and len(row["candidates"]) == 6, "initial partial row differs")
        for k, (candidate, before) in enumerate(zip(row["candidates"], original["candidates"])):
            require(all(candidate[key] == before[key] for key in ("initialization_index", "runtime_gate_status", "runtime_gate_reason", "convergence_warning")),
                "remote initial partial reclassified original candidate")
            for key in ("ade_m", "fde_m", "maximum_xy_error_m", "speed_rmse_mps", "maximum_speed_error_mps", "final_objective_m2"):
                base.close(candidate[key], before[key])
            check, previous = candidate["initial_objective_verification"], before["initial_objective_verification"]
            require(check["recorded_gpu_objective_m2"] == previous["recorded_gpu_objective_m2"] and check["comparison_tolerance"] == {"absolute": 1e-5, "relative": 1e-5}, "initial objective binding differs")
            expected = base.initial_objective_check(check["recorded_gpu_objective_m2"], check["reconstructed_cpu_objective_m2"])
            require(check == expected, "remote initial objective verdict differs")
            largest_cpu_delta = max(largest_cpu_delta, abs(check["reconstructed_cpu_objective_m2"] - previous["reconstructed_cpu_objective_m2"]))
            if check["status"] == "INITIAL_FORWARD_UNVERIFIED":
                discrepancies.append((row["anchor_index"], k))
                discrepancy_records.append({"anchor_index": row["anchor_index"], "initialization_index": k, **check})
    validate_initial_summary(summary, discrepancy_records)
    return {"remote_initial_cli_status": "FAILED_DURING_RENDER_PARTIAL_FILES_RETAINED", "created_at_utc": summary["created_at_utc"],
        "remote_torch_version": summary["initial_forward_checks"][0]["torch_version"], "unverified_count": len(discrepancies),
        "same_unverified_candidate_indices_as_local": discrepancies == [(r["anchor_index"], r["initialization_index"]) for r in reference["initial_forward_unverified_candidates"]],
        "maximum_local_vs_remote_cpu_initial_objective_difference_m2": largest_cpu_delta,
        "maximum_remote_cpu_vs_gpu_initial_objective_difference_m2": max(r["absolute_difference_m2"] for r in summary["initial_forward_unverified_candidates"]),
        "cause_determined": False}


def publish(local_root, remote_root, partial_root, reference_root, output):
    require(not output.exists() and not output.is_symlink() and all(not output.resolve().is_relative_to(root.resolve())
        for root in (local_root, remote_root, partial_root, reference_root)), "new output must remain outside source evidence")
    source = base.sha(Path(__file__))
    originals = exact_files(reference_root, ("summary.json", "per_anchor_numeric.jsonl"))
    require(hashlib.sha256(originals["summary.json"]).hexdigest() == REFERENCE_SHA, "released category17 reference changed")
    reference, reference_rows = base.evidence._loads(originals["summary.json"]), parse_rows(originals["per_anchor_numeric.jsonl"])
    names = ("summary.json", "per_anchor_forward.jsonl", "SHA256SUMS")
    local, remote = exact_files(local_root, names, checksum_manifest=True), exact_files(remote_root, names, checksum_manifest=True)
    partial = exact_files(partial_root, ("summary.json", "per_anchor_numeric.jsonl", "input_verification.json", "EXECUTION_STATUS.md"))
    a, b = validate_forward(local, reference, reference_rows), validate_forward(remote, reference, reference_rows)
    partial_result = compare_initial_partial(partial, reference, reference_rows)
    result = {"schema": "portable_e2e.decoder_forward_publication.v1", "status": "NUMERICAL_COMPARISON_NOT_ADMITTED",
        "reference_category17_summary_sha256": REFERENCE_SHA, "curator_sha256": source,
        "local_cpu": {k: a[k] for k in ("status", "created_at_utc", "torch_version", "compared_scalar_count", "mismatch_scalar_count", "maximum_absolute_difference_by_component")},
        "remote_cpu": {k: b[k] for k in ("status", "created_at_utc", "torch_version", "compared_scalar_count", "mismatch_scalar_count", "maximum_absolute_difference_by_component")},
        "same_optimized_latent_hashes": all(x["optimized_latent_float32_sha256"] == y["optimized_latent_float32_sha256"] for x, y in zip(a["batches"], b["batches"])),
        "identical_cpu_xy_batch_hash_count": sum(x["cpu_xy_float32_sha256"] == y["cpu_xy_float32_sha256"] for x, y in zip(a["batches"], b["batches"])),
        "identical_cpu_speed_batch_hash_count": sum(x["cpu_speed_float32_sha256"] == y["cpu_speed_float32_sha256"] for x, y in zip(a["batches"], b["batches"])),
        "initial_partial": partial_result, "training_data_approved": False, "old_reports_replaced": False, "optimization_replayed": False}
    require(result["same_optimized_latent_hashes"], "local/remote optimized latent inputs differ")
    groups = {"01_local_cpu_final_forward": (local_root, local), "02_remote_cpu_final_forward": (remote_root, remote),
        "03_remote_cpu_initial_partial": (partial_root, partial)}
    for _, (root, files) in groups.items():
        for name, raw in files.items(): require(base.sha(root / name) == hashlib.sha256(raw).hexdigest(), "publication input changed")
    require(base.sha(Path(__file__)) == source, "curation source changed")
    output.mkdir(parents=True, exist_ok=False)
    manifest = []
    for category, (_, files) in groups.items():
        destination = output / category; destination.mkdir()
        for name, raw in files.items():
            (destination / name).write_bytes(raw)
            manifest.append({"path": f"{category}/{name}", "source_sha256": hashlib.sha256(raw).hexdigest(), "published_sha256": hashlib.sha256(raw).hexdigest(), "transformation": "NONE_EXACT_BYTES"})
    (output / "comparison.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    (output / "publication_manifest.json").write_text(json.dumps({"files": manifest, "source_metadata_redacted": False,
        "partial_initial_files_are_not_a_completed_remote_publication": True}, indent=2) + "\n")
    (output / "README.md").write_text(readme(result))
    (output / "SHA256SUMS").write_text("".join(f"{base.sha(path)}  {path.relative_to(output)}\n" for path in sorted(output.rglob("*")) if path.is_file()))
    return result


def readme(result):
    table = ["| CPU 검사 | Torch | 비교 스칼라 수 | 허용량 밖 | 최대 X 차이 (m) | 최대 Y 차이 (m) | 최대 속도 차이 (m/s) |",
        "|---|---|---:|---:|---:|---:|---:|"]
    for name, key in (("로컬", "local_cpu"), ("Pro6000 개인 venv", "remote_cpu")):
        row = result[key]; d = row["maximum_absolute_difference_by_component"]
        table.append(f"| {name} | {row['torch_version']} | {row['compared_scalar_count']:,} | {row['mismatch_scalar_count']:,} | {d['x_m']:.9g} | {d['y_m']:.9g} | {d['speed_mps']:.9g} |")
    p = result["initial_partial"]
    return ("# CPU 최종 전방계산 검증과 초기값 검사 미완료 기록\n\n<!-- HH_260906 - Distinguish final arithmetic verification from unchanged initial disagreement, optimization and data admission. -->\n\n"
        "기존 GPU 수치 근사의 **저장된 최종 잠재값**을 두 CPU 환경에서 동일 물리 decoder에 한 번씩 넣었습니다. "
        "1,337개 시작점 × 6후보 × 64점의 X·Y·속도 1,540,224개 값을 각각 비교했습니다. "
        "최적화·학습·GPU 실행·게이트 변경 없이 `no_grad` 전방계산만 했습니다.\n\n" + "\n".join(table) + "\n\n"
        "허용량은 처음과 같은 절대·상대 `1e-5`입니다. 판정식은 `|CPU-GPU| <= max(1e-5, 1e-5*max(|CPU|,|GPU|))`입니다. "
        "따라서 최대 X 절대 차이가 1e-5 m보다 커도 상대 오차 조건으로 통과할 수 있으며, 모든 절대 차이가 1e-5 이하라는 뜻은 아닙니다. "
        "이는 저장된 수치 결과의 전방계산 일치이지 원본 정답 오차·자율주행 성능·데이터 승인 PASS가 아닙니다. "
        "원래 출력 게이트 실패 34개와 원본 곡률 실패 시작점 299개는 그대로 보존했습니다.\n\n"
        f"두 CPU 실행의 최적화 잠재값 해시는 6개 배치 모두 같습니다. CPU 출력 바이트까지 동일한 배치는 XY {result['identical_cpu_xy_batch_hash_count']}/6, "
        f"속도 {result['identical_cpu_speed_batch_hash_count']}/6이므로 허용량 내 일치와 bit-identical은 구분합니다.\n\n"
        "## 초기값 검사는 여전히 별개로 확인 미완료\n\n"
        f"같은 Torch 2.13.0+cu130의 원격 CPU 초기값 재감사에서도 {p['unverified_count']}개 초기 목적함수 차이가 남았습니다. "
        f"최대 CPU–GPU 차이는 {p['maximum_remote_cpu_vs_gpu_initial_objective_difference_m2']:.9g} m²이며 기존 로컬 검사와 동일한 후보 인덱스입니다. "
        f"전체 초기 목적함수의 두 CPU 환경 사이 최대 차이는 {p['maximum_local_vs_remote_cpu_initial_objective_difference_m2']:.9g} m²입니다. "
        "Torch 버전을 맞춰도 차이가 없어지지 않았으므로 버전 차이만을 원인이라고 할 수 없습니다. CPU/GPU 연산 차이의 구체적 원인은 아직 확정하지 않았습니다.\n\n"
        "이 원격 초기값 CLI는 수치 검사 JSON 3개를 쓴 뒤 PNG 생성에서 `ModuleNotFoundError: No module named 'matplotlib'`로 종료 코드 1을 반환했습니다. "
        "패키지를 설치하거나 재시도하지 않았고, 원격 README·PNG·정상 SHA256SUMS는 생성되지 않았습니다. "
        "[부분 실행 기록](03_remote_cpu_initial_partial/EXECUTION_STATUS.md)과 JSON 3개를 원본 바이트 그대로 보관했습니다. "
        "이 폴더의 새 공개 해시 목록은 전송된 부분 파일을 검증할 뿐, 과거 CLI를 완료로 바꾸지 않습니다.\n\n"
        "## 자료\n\n"
        "[로컬 최종 전방계산](01_local_cpu_final_forward/summary.json), [원격 최종 전방계산](02_remote_cpu_final_forward/summary.json), "
        "[환경 비교](comparison.json), [복사 원본 해시](publication_manifest.json), [전체 SHA256SUMS](SHA256SUMS). "
        "각 최종 검사 폴더의 JSONL에는 모든 후보 비교 수와 최대 차이·불일치 목록이 있습니다. "
        "초기 GPU 최적화와 기존 실패 설명은 [변경하지 않은 category17](../17_decoder_representability/README.md)을 참조합니다. "
        "새 최종 전방계산 코드 SHA와 실행 날짜는 각 JSON에 기록했습니다. 원본 학습 데이터나 모델 체크포인트를 만들거나 변경하지 않았습니다.\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("local-root", "remote-root", "partial-root", "reference-root", "output-dir"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args(argv)
    publish(args.local_root, args.remote_root, args.partial_root, args.reference_root, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
