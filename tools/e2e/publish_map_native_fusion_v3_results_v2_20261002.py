#!/usr/bin/env python3
"""HH_261002 - Publish sealed MAP V3 stored predictions with the actual numeric training-receipt ABI."""
from __future__ import annotations

import argparse
import ast
import copy
import json
from pathlib import Path
import sys
from types import FunctionType

import numpy as np

from . import publish_map_sign_margin_loss_results_20261002 as previous

ROOT, frozen, require = previous.ROOT, previous.frozen, previous.require
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
NAME = "map_native_fusion_pair_20261002_v3"
REMOTE_RUN = "/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/" + NAME
SCHEMA = "portable_e2e.map_native_fusion_v3_results.20261002.v2"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
FIT_SCHEMA = "portable_e2e.map_native_fusion_pair.20261002.v3"
COMPLETION = ROOT / "storage/experiments/plans/2026-10-02/temporal_left_right_joint_v2/ROOT_previous_MAPV3_completed.json"
COMPLETION_SHA = "84452c157c221964545a1a9753adf9ca8f74daf098351308661d4c9cd0b1ab17"
PROVIDER_SHA = "f9b4b9096c83593d6ab924d94fc858356730c5b32fa122b4b18aa8acbe7cad0e"
INDICES, EVAL, SPLITS, SHAPE = previous.INDICES, previous.EVAL, previous.SPLITS, previous.SHAPE
FONT, LABELS, COLORS = previous.FONT, previous.LABELS, previous.COLORS
SOURCE = {**previous.SOURCE,
    "map_native_fusion_contract_20261002.py": "acc0b5b821302187d7a35c5e9d4208db0a4c103f0a50a21afd85e4a1161b4dc5",
    "map_native_fusion_v1_20261002.py": "88d494daf864d3cb4d28bb1226c4d8e997257960a67f2138f91eac1b8325592f",
    "train_map_native_fusion_pair_20261002.py": "3918b5a529f08e17cfd294d24a21fbd545ee0adccc0e1ac3adaf79c60eb3f841",
    "train_map_native_fusion_pair_v2_20261002.py": "7f124ef3bc67104c483f62c16af1daad12f9db4b6829634827ca5ae59e530edd",
    "train_map_native_fusion_pair_v3_20261002.py": "84884dd304be4eb6c02b6fbe15678f7b6a7a0a2aa907ed265320fa0dc9ac4a13"}
FAILED = sorted(("ADDED_Town01_sign_recall_strict_gain_vs_control", "ADDED_Town01_sign_recall_strict_gain_vs_parent",
    "Town01_sign_recall_vs_control", "Town01_sign_recall_vs_parent"))


def publication_sources():
    return previous.publication_sources() | {str(Path(__file__).resolve().relative_to(ROOT)): Path(__file__).resolve()}


def required_run_files():
    # HH_261002 - The complete52 census is the old opaque inventory, source15, and V3 numeric-only endpoint files.
    old = previous.previous.required_run_files("v1")
    return (old - {"source/" + name for name in previous.previous.TRAIN_SOURCE}
        - {arm + "/checkpoint.pt" for arm in ("control", "candidate")}) | {"source/" + name for name in SOURCE} | {
        "equal_global_schedule.npz", "equal_global_schedule_proof.json", "strict_CUDA_preflight.json", "loader_parity.json"}


def completion_and_run(caller, pins):
    require(caller.get("MAP_completion") == dict(path=str(COMPLETION), sha256=COMPLETION_SHA), "exact ROOT MAP completion required")
    proof = pins.json(COMPLETION, COMPLETION_SHA)
    require(proof.get("schema") == "portable_e2e.temporal_left_right_detached_launch.20261002.v2.MAP_completion"
        and type(proof.get("PID")) is int and proof["PID"] == 3472102
        and type(proof.get("process_start_ticks")) is int and proof["process_start_ticks"] == 140240638,
        "owned completed MAP process identity differs")
    for key in ("source_asset_numeric_endpoints_and_full_journals_verified", "all350_prediction_confusions_and_metrics_rebuilt_on_CPU",
        "all27_original_screens_recomputed_including_failures", "source15_and_original_archived_assets_runtime_unchanged",
        "owned_predecessor_generation_finished"):
        require(proof.get(key) is True, "ROOT complete verification missing: " + key)
    for key in ("model_promoted", "vehicle_control_approved"): require(proof.get(key) is False, "approval must remain false")
    for key in ("official_main_features_approved", "official_subfeatures_approved"):
        require(type(proof.get(key)) is int and proof[key] == 0, "formal approval cannot be inferred")
    require(type(proof.get("passed_diagnostic_screens")) is int and proof["passed_diagnostic_screens"] == 23
        and type(proof.get("diagnostic_screen_denominator")) is int and proof["diagnostic_screen_denominator"] == 27
        and sorted(proof.get("failed_diagnostic_screens", [])) == FAILED, "actual23/27 and all four failures required")
    census = proof.get("complete_output_file_sha256")
    require(type(census) is dict and set(census) == required_run_files() and len(census) == 52, "complete ROOT52 inventory required")
    ref = caller["run"]
    require(ref.get("root") == str(RUN) and type(ref.get("files")) is dict
        and set(ref["files"]) == previous.previous.inventory(RUN) == set(census), "exact fresh canonical mirror52 required")
    for name, row in census.items():
        path = RUN / previous.previous.relative_name(name)
        require(type(row) is dict and set(row) == {"sha256", "bytes"} and type(row["bytes"]) is int and row["bytes"] >= 0
            and ref["files"][name] == row["sha256"] and path.stat().st_size == row["bytes"], "ROOT census size/SHA differs: " + name)
        pins.file(path, row["sha256"])
    for name, expected in SOURCE.items():
        require(ref["files"]["source/" + name] == expected, "scientific source15 differs")
        folder = "learning/portable_e2e" if name in ("map_context_recovery.py", "map_perception.py", "map_sign_margin_loss_v1_20261002.py", "map_native_fusion_v1_20261002.py") else "tools/e2e"
        pins.file(ROOT / folder / name, expected)
    summary = pins.json(RUN / "summary.json", proof["summary_sha256"])
    evaluation = pins.json(RUN / "known_evaluation.json", proof["known_evaluation_sha256"])
    fit = pins.json(RUN / "fit_complete.json", proof["fit_complete_sha256"])
    require(summary.get("schema") == FIT_SCHEMA and summary.get("study_id") == NAME and summary.get("source_sha256") == SOURCE
        and summary.get("source_count") == 15 and summary.get("status") == "FIXED20K_ARCHITECTURE_PAIR_DIAGNOSTIC_NOT_PROMOTED"
        and summary.get("source_and_inputs_unchanged") is True and summary.get("training_authorized") is True
        and summary.get("model_promoted") is False and summary.get("vehicle_control_approved") is False
        and summary.get("known_evaluation") == evaluation, "actual fixed20k MAP V3 summary required")
    require(fit.get("status") == "BOTH_FIXED_FINAL20000_NUMERIC_ENDPOINTS_VERIFIED_BEFORE_KNOWN_EVALUATION"
        and fit.get("known_payload_opened") is False and fit.get("arms") == summary.get("arms_result")
        and fit.get("actual_full_journal_evidence") == summary.get("actual_boundary") == proof.get("actual_both_final_journals"),
        "ROOT-verified both numeric endpoints and complete journals must precede evaluation")
    arms = summary["arms_result"]
    require(set(arms) == {"control", "candidate"}, "both final arms required")
    # HH_261002 - Frozen writer adds metadata_sha256 only to summary arms; bind its exact file then compare the stored info without that field.
    for arm, value in arms.items():
        require(type(value.get("steps")) is int and value["steps"] == 20000
            and type(value.get("examples_seen")) is int and value["examples_seen"] == 160000
            and value.get("weights_sha256") == ref["files"][arm + "/weights.npz"]
            and value.get("metadata_sha256") == ref["files"][arm + "/training.json"]
            and pins.json(RUN / arm / "training.json", value["metadata_sha256"]) == {k: v for k, v in value.items() if k != "metadata_sha256"},
            "both final20000 /160000 endpoint identity required")
    return dict(root=RUN, summary=summary), proof


def read_prediction(pins, study, ref, split, arm, target):
    name = ("parent_diagnostic" if arm == "parent" else arm) + "/" + split + "_predictions.npz"
    arrays = pins.npz(RUN / name, ref["files"][name])
    require(set(arrays) == {"prediction"}, "stored prediction keys differ")
    prediction = arrays["prediction"]
    require(prediction.shape == target.shape and prediction.dtype == np.uint8 and int(prediction.max()) < 7, "entire native prediction ABI differs")
    metrics = previous.previous.confusion_metrics(target, prediction)
    row = study["summary"]["known_evaluation"][split][arm]
    frozen.reported_metrics(row, metrics, SPLITS[split][0], SPLITS[split][1])
    for key in ("true_positive_pixels", "false_negative_pixels", "precision", "recall", "other_to_sign_false_positive_pixels"):
        require(row.get(key) == metrics[key], "full-cohort derived metric differs: " + key)
    weight = ref["files"]["initial_model_weights.npz" if arm == "parent" else arm + "/weights.npz"]
    require(row.get("weights_sha256") == weight and row.get("prediction_file_sha256") == ref["files"][name]
        and type(row.get("evaluated_frames")) is int and row["evaluated_frames"] == SPLITS[split][0]
        and row.get("full_FOV") is True, "stored prediction/full-FOV endpoint binding differs")
    return prediction, row


def audit(caller_path, caller_sha):
    # HH_261002 - ROOT's complete52 seal supplies journal/endpoint attestation; all350 saved predictions are independently recomputed here.
    require("torch" not in sys.modules and "carla" not in sys.modules
        and frozen.contract.digest(Path(previous.__file__)) == PROVIDER_SHA
        and frozen.contract.digest(Path(previous.previous.__file__)) == previous.FROZEN_GALLERY_SHA
        and frozen.contract.digest(Path(frozen.__file__)) == previous.previous.FROZEN_PUBLISHER_SHA,
        "frozen publication providers and no Torch/CARLA required")
    pins = frozen.Pins(); caller = pins.json(caller_path, caller_sha)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("output") == str(OUTPUT) and caller.get("fixed_indices") == INDICES
        and caller.get("remote_run_root") == REMOTE_RUN and caller.get("remote_originals_independently_hashed") is True,
        "prospective fixed10 /actual remote-original caller required")
    require(set(caller.get("publication_source_sha256", {})) == set(publication_sources()), "complete publication source closure required")
    for name, path in publication_sources().items(): pins.file(path, caller["publication_source_sha256"][name])
    require(caller.get("font", {}).get("path") == str(FONT), "fixed readable font required"); pins.file(FONT, caller["font"]["sha256"])
    study, completion = completion_and_run(caller, pins)
    require(caller.get("evaluation") == dict(root=str(EVAL), files={s + ".npz": sha for s, (_, sha) in SPLITS.items()}), "entire known350 source refs required")
    manifest = pins.json(EVAL / "manifest.json", frozen.EVAL_MANIFEST_SHA)
    metrics, panels = {}, []
    for split, (count, sha) in SPLITS.items():
        require(all(manifest["files"][split].get(k) == v for k, v in dict(file=split + ".npz", count=count, sha256=sha).items()), "full evaluation manifest differs")
        arrays = pins.npz(EVAL / (split + ".npz"), sha)
        require(set(arrays) == {"rgb", "target"}, "original RGB/GT keys required")
        rgb, target = arrays["rgb"], arrays["target"]
        require(rgb.shape == (count, *SHAPE, 3) and target.shape == (count, *SHAPE) and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < 7, "native full350 FOV/classes cannot be cropped")
        predictions, metrics[split] = {}, {}
        for arm in ("parent", "control", "candidate"):
            predictions[arm], metrics[split][arm] = read_prediction(pins, study, caller["run"], split, arm, target)
        for index in INDICES[split]: panels.append((split, index, rgb[index].copy(), target[index].copy(),
            *(predictions[arm][index].copy() for arm in ("parent", "control", "candidate"))))
    screen = previous.diagnostic_screen(metrics)
    require(screen == study["summary"]["exploratory_screen_result"] and sorted(k for k, v in screen["checks"].items() if not v) == FAILED
        and sum(screen["checks"].values()) == completion["passed_diagnostic_screens"], "all original27 /23PASS /fourFAIL must reproduce")
    pins.recheck(); require(previous.previous.inventory(RUN) == set(caller["run"]["files"]), "complete52 mirror changed")
    report = dict(schema=SCHEMA, status="FULL350_STORED_MAP_V3_VERIFIED_23OF27_FOURFAIL_NOT_APPROVED", evaluation=metrics,
        exploratory_screen=screen, fixed_indices=INDICES, native_FOV_hw=list(SHAPE), full_cohort_counts={s: n for s, (n, _) in SPLITS.items()},
        source_count=15, actual_optimizer_exposures=completion["actual_both_final_journals"],
        complete_output_file_sha256=completion["complete_output_file_sha256"], ROOT_MAP_completion_sha256=COMPLETION_SHA,
        model_weights_sha256=dict(parent=caller["run"]["files"]["initial_model_weights.npz"],
            **{a: study["summary"]["arms_result"][a]["weights_sha256"] for a in ("control", "candidate")}),
        first_read_source_and_artifact_pins=dict(pins.values), caller_sha256=caller_sha,
        mirrored_run_inventory=sorted(caller["run"]["files"]), remote_run_root=REMOTE_RUN,
        both_arm_indices_crops_FALSE_bits_byte_identical=True, original_CE_unchanged=True,
        architecture_intervention=True, known_exposed_diagnostic_not_independent=True, independent_new_dev_sites=0,
        failed_checks_retained=True, model_inference_executed=False, Torch_imported=False, pickle_loaded=False,
        raw_TRAIN_reopened=False, endpoint_numeric_weights_loaded=False, full_journals_independently_reparsed_here=False,
        endpoint_and_full_journal_verification_attested_by_fixed_ROOT_completion=True,
        RGB_only_seven_class_perception_not_HD_AMG=True, traffic_light_fixture_not_signal_state=True, GT_is_not_a_model_map=True,
        recorded_diagnostic_rendering_not_Autoware_or_RViz_screenshot=True, live_mapping_latency_measured=False,
        model_promoted=False, vehicle_control_approved=False, formal_main_functions_approved=0, formal_subfunctions_approved=0)
    return report, panels, pins


def canvas(panel, font, report):
    # HH_261002 - Preserve the frozen full-FOV layout; change ONLY truthful architecture/stage captions in a private namespace.
    labels = {"Control / original CE +4000": "Control / original CE +20000",
        "Candidate / CE +0.10 sign margin +4000": "Candidate / native fusion +20000",
        "Same source/crop/FALSE views; loss only": "Same data/crop/CE; architecture only"}
    node = next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body if isinstance(n, ast.FunctionDef) and n.name == "canvas")
    revised = copy.deepcopy(node)
    for old, new in labels.items():
        hits = [n for n in ast.walk(revised) if isinstance(n, ast.Constant) and n.value == old]
        require(len(hits) == 1, "one frozen caption required"); hits[0].value = new
    inverse = copy.deepcopy(revised)
    for n in ast.walk(inverse):
        if isinstance(n, ast.Constant) and isinstance(n.value, str): n.value = {v: k for k, v in labels.items()}.get(n.value, n.value)
    require(ast.dump(inverse) == ast.dump(node), "native gallery layout changed")
    namespace = dict(vars(previous), SHAPE=SHAPE)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["canvas"](panel, font, report)


def markdown(report, files):
    pct = lambda value: "미측정(정답 없음)" if value is None else f"{100 * value:.2f}%"
    delta = lambda a, b: "미측정" if a is None or b is None else f"{100 * (b - a):+.2f}%p"
    checks = report["exploratory_screen"]["checks"]; failed = [k for k, v in checks.items() if not v]
    lines = ["# 지도 인식 구조 변경: 좋아진 부분과 남은 표지 누락", "", f"**기존 진단 전체 {sum(checks.values())}/27 통과, {len(failed)}개 미달. 모델 자동 채택·기능 승인은 하지 않습니다.**", "",
        "같은 10,000회 부모에서 기존 구조와 세밀한 영상·주변 맥락을 결합한 후보를 각각 20,000회 추가 학습했습니다. 각 160,000회 입력 노출이며 자료·순서·crop·원래 CE 손실은 같습니다. 반복 노출은 새로운 장소의 증가가 아닙니다.", "",
        "후보가 표지 오탐을 줄였더라도 재현율(실제 표지를 찾아낸 비율)의 4개 기준은 미달입니다. 아래는 부모·대조군·후보의 전체 수치와 모든 실패입니다. 이미 본 Town01 118장·Town02 232장 전체 350장을 검산했으며 신규 독립 장소는 0개입니다.", "",
        "RGB 7종 인식 연구이지 신호 상태·좌표 누적·차선 연결을 갖춘 정밀지도 AMG 완성품이 아닙니다. 실제 Autoware/RViz 화면이 아닌 기록 기반 진단 그림입니다. 주행 메인 0/9·하위 0/30, 실시간 100ms 성능은 미측정입니다.", "",
        "## 기존 기준 27개를 빠짐없이 확인", "", "| 기준 | 결과 |", "|---|---|"]
    lines += [f"| {name} | {'통과' if value else '미달'} |" for name, value in checks.items()]
    for split, rows in report["evaluation"].items():
        lines += ["", f"## {split}: 전체 분류 겹침(IoU)", "", "| 분류 | 부모 | 대조군 | 후보 | 후보−부모 | 후보−대조군 |", "|---|---:|---:|---:|---:|---:|"]
        for label in LABELS:
            a, b, c = (rows[arm]["iou"][label] for arm in ("parent", "control", "candidate"))
            lines.append(f"| {label} | {pct(a)} | {pct(b)} | {pct(c)} | {delta(a, c)} | {delta(b, c)} |")
        lines += ["", "재현율은 놓친 대상을, FP는 잘못 표시한 픽셀을 보여 줍니다. 증가·감소가 섞이므로 IoU만 보고 개선으로 판단하지 않습니다.", "",
            "| 분류 | 부모 재현율 / FP | 대조군 재현율 / FP | 후보 재현율 / FP | 후보 재현율−대조군 | 후보 FP−대조군 |", "|---|---:|---:|---:|---:|---:|"]
        for label in LABELS:
            values = [f"{pct(rows[arm]['recall'][label])} / {rows[arm]['false_positive_pixels'][label]}" for arm in ("parent", "control", "candidate")]
            lines.append("| " + label + " | " + " | ".join(values) + f" | {delta(rows['control']['recall'][label], rows['candidate']['recall'][label])} | {rows['candidate']['false_positive_pixels'][label] - rows['control']['false_positive_pixels'][label]:+d} |")
        lines += ["", "| 표지 지표 | 부모 | 대조군 | 후보 |", "|---|---:|---:|---:|"]
        for title, key in (("정밀도(표시 중 진짜인 비율)", "precision"), ("재현율(진짜 중 찾은 비율)", "recall")):
            lines.append("| " + title + " | " + " | ".join(pct(rows[arm][key]["traffic_sign"]) for arm in ("parent", "control", "candidate")) + " |")
        lines += ["| other→표지 오탐 픽셀 | " + " | ".join(str(rows[arm]["other_to_sign_false_positive_pixels"]) for arm in ("parent", "control", "candidate")) + " |"]
    lines += ["", "## 이전부터 고정한 동일 10개 전체 화각 장면", "", "위: 실제 RGB·정답·부모. 아래: 기존 구조·새 구조. 정답은 모델 입력이나 생성 지도가 아니며 성공 장면을 다시 선택하지 않았습니다.", ""]
    for name in files: lines += [f"![{name}]({name})", ""]
    lines += ["[results.json](results.json)에 49칸 confusion·모든 FN/FP·27개 판정과 전체 가중치/52개 원본 SHA를 보존했습니다. 저장 예측만 읽고 새 추론·Torch·pickle·GPU는 실행하지 않았습니다. 최종 숫자 가중치와 전량 학습 기록 검증은 고정 ROOT 완료 영수증의 검증이며 이 publisher가 다시 학습 기록을 해석한 것은 아닙니다.", ""]
    return "\n".join(lines)


def publish(args):
    # HH_261002 - Reuse the unchanged create-only writer, storage floor, source archive, and before/after input rechecks.
    namespace = dict(vars(previous), OUTPUT=OUTPUT, RUN=RUN, SPLITS=SPLITS, FONT=FONT,
        audit=audit, canvas=canvas, markdown=markdown, publication_sources=publication_sources)
    return FunctionType(previous.publish.__code__, namespace, "publish")(args)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--caller-pins", required=True); parser.add_argument("--caller-pins-sha256", required=True)
    parser.add_argument("--validate-only", action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)), ensure_ascii=False, sort_keys=True), flush=True); return 0


if __name__ == "__main__": raise SystemExit(main())
