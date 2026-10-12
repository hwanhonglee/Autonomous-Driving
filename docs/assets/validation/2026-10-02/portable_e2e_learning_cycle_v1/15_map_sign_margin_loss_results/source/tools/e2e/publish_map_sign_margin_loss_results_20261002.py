#!/usr/bin/env python3
"""HH_261002 - Publish fixed-index saved sign-loss predictions without models, Torch, or selection."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import publish_map_sign_sampler_v2_comparison_20261002 as previous

frozen, require = previous.frozen, previous.require
ROOT = previous.ROOT
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/15_map_sign_margin_loss_results"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_sign_margin_repair_v1"
NAME = "map_sign_margin_repair_20261002_v1"
REMOTE_RUN = "/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/" + NAME
FIT_SCHEMA = "portable_e2e.map_sign_margin_repair.20261002.v1"
SCHEMA = "portable_e2e.map_sign_margin_loss_results.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
INDICES, EVAL, SPLITS, SHAPE = previous.INDICES, previous.EVAL, previous.SPLITS, previous.SHAPE
INITIAL_SHA, DATA_SHA, REGISTRY_SHA = previous.INITIAL_SHA, previous.DATA_SHA, previous.REGISTRY_SHA
FONT, LABELS, COLORS = previous.FONT, previous.LABELS, previous.COLORS
FROZEN_GALLERY_SHA = "fd939a5ad05949523677dab947274d83f6226e27711ad2ee80977b78cd40aa42"
AUTH_SHA = "59de309a1e67c4626cc9ffc2209784cd7193753c90ba0f208774eff1ccde8cfd"
EQUAL_PROOF_SHA = "bcad56c9497b6860eec4d422599bbce4aaada2170d27b84427d0ae6aa737b727"
LOSS_CONTRACT_SHA = "d674c9a88a136652980bdcad5989249c07fe83ceeaab045280f0ce89878d97c3"
INITIAL_STATE_SHA = "12d647a948008dd61410bf26d3e431118ca46b2d0c3ff65c9289f0865f40916f"
SOURCE = {**previous.TRAIN_SOURCE,
    "map_sign_margin_loss_v1_20261002.py": "50029d77d4d216833a04ea140757a4f7b5d6cb8b183c9685bedacb05411e4286",
    "train_map_sign_margin_repair_20261002.py": "3ede51218b97e4172a9685b349b727ef9bef0c1a6e2c555b44ca94cc15e4d43f"}
MINING_SHA = dict(zip(("inventory.json", "started.json", "authorization.json", "records.jsonl", "parent_TRAIN_predictions.npz", "proposed_schedule.npz"),
    ("c785ce2954c461ba6373d56123ab2908817a517a4fa121d77235ecdb0ba259dd", "2c03f3c4c447bfc37f0a6bcf6448404b153c2605076cc528455cd3224c9c0adc",
     "f85a0687750bcabd88760c42d12111a4558a78e1da3533d3577ac41d4e2f76e3", "7f6df41b42c057ec0d25b7cedf4aaa277391eefb07b3a7815eb2590868fd8fcc",
     "aa4c786f9bfa0cc063683b3098ecd6f5707028e07c9dba9b55f3349b8df7ea93", "c019fd402c9886c147d81052bd4448a336b9a8b45f9d4f3f076b00fd23eca93a")))
LINEAGE_SHA = dict(zip(("control/training.json", "fit_complete.json", "summary.json", "source/bundle.json"),
    ("161649bee1416e485bb1dd13f780291fa05dee7565c3c5d8af593083174a991a", "71e5cb0623c759bb3a17bdcbed8601521c976cbf4fdb8aa1a8d5c48ff143c83e",
     "fa9749d67b27eb8169473350289ba61530f324736c61bb3a269adc6b070027d0", "2f30504c9380008fe0d6f17893749dfd1f377b39fbb398b4aad926a8da5afcdd")))


def document_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def publication_sources():
    return previous.publication_sources() | {
        str(Path(previous.__file__).resolve().relative_to(ROOT)): Path(previous.__file__).resolve(),
        str(Path(__file__).resolve().relative_to(ROOT)): Path(__file__).resolve()}


def local_source(name):
    return ROOT / ("learning/portable_e2e" if name in ("map_context_recovery.py", "map_perception.py", "map_sign_margin_loss_v1_20261002.py") else "tools/e2e") / name


def required_run_files():
    # HH_261002 - All51 artifacts include opaque PT bytes, both preflights, and unchanged full mining/lineage.
    return previous.required_run_files("v1") | {"source/" + name for name in SOURCE} | {
        "equal_global_schedule.npz", "equal_global_schedule_proof.json", "loss_contract.json",
        "intervention_archive_receipt.json", "control_strict_CUDA_preflight.json", "candidate_strict_CUDA_preflight.json"}


def diagnostic_screen(evaluation):
    original = previous.diagnostic_screen(evaluation)
    checks = dict(original["checks"]); town = evaluation["dev_town01"]; candidate = town["candidate"]
    for comparator in ("parent", "control"):
        for metric, key in (("precision", "precision"), ("recall", "recall"), ("IoU", "iou")):
            a, b = town[comparator][key]["traffic_sign"], candidate[key]["traffic_sign"]
            checks["ADDED_Town01_sign_" + metric + "_strict_gain_vs_" + comparator] = a is not None and b is not None and b > a
    checks["ADDED_Town01_other_to_sign_FP_at_most_90pct_parent"] = candidate["other_to_sign_false_positive_pixels"] <= .9 * town["parent"]["other_to_sign_false_positive_pixels"]
    require(len(checks) == 27, "all original20 and strict additional7 checks required")
    return dict(checks=checks, original_twenty_checks=original["checks"], added_check_count=7,
        passed_exploratory_only=all(checks.values()), independent_validation=False, model_promoted=False, vehicle_control_approved=False)


def read_study(ref, pins):
    # HH_261002 - Read the completed experiment, never import its training modules or reopen TRAIN pixels.
    root = frozen.contract.canonical(Path(ref["root"]))
    require(root == RUN and set(ref["files"]) == previous.inventory(root) == required_run_files(), "complete exact51 mirror required")
    for name, sha in ref["files"].items(): pins.file(root / previous.relative_name(name), sha)
    summary = pins.json(root / "summary.json", ref["files"]["summary.json"])
    require(summary.get("schema") == FIT_SCHEMA and summary.get("study_id") == NAME and summary.get("source_sha256") == SOURCE
        and summary.get("status") == "FIXED_CURRENT_PARENT_SIGN_MARGIN_LOSS_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED", "fixed loss-only experiment required")
    for name, sha in SOURCE.items(): pins.file(root / "source" / name, sha); pins.file(local_source(name), sha)
    values = dict(initial_model_weights_sha256=INITIAL_SHA, data_manifest_sha256=DATA_SHA, data_registry_sha256=REGISTRY_SHA,
        additional_updates_per_arm=4000, examples_per_arm=32000, initial_completed_updates=10000, total_lineage_updates_per_arm=14000,
        physical_TRAIN_slots=1209, parameter_count=245831, batch=8, seed=2026100203, learning_rate=3e-5, weight_decay=1e-4,
        gradient_norm_cap=5., stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145),
        source_and_inputs_unchanged=True, model_promoted=False, vehicle_control_approved=False, known_evaluation_not_independent=True,
        evaluation_opens_only_after_both_final4000=True, validation_used_during_optimization=False, independent_new_dev_sites=0,
        labels_changed=False, architecture_unchanged=True, semantic_loss_unchanged=False, original_weighted_CE_unchanged=True,
        sampler_unchanged=True, native_denials_preserved=True, training_authorized=True, main_capabilities_approved=0, subfunctions_approved=0,
        both_arm_indices_crops_FALSE_bits_byte_identical=True, both_arm_strict_CUDA_preflights_before_ANY_optimizer_update=True,
        preflight_exposures_per_arm=8, preflight_optimizer_updates=0, preflight_extra_TRAIN_only_diagnostic_exposures_not_in_32000_optimizer_exposures=True,
        historical_runtime_binary_identity_claimed=False)
    for key, wanted in values.items(): require(type(summary.get(key)) is type(wanted) and summary[key] == wanted, "fixed loss contract differs: " + key)
    auth = pins.json(root / "authorization.json", AUTH_SHA)
    require(summary.get("authorization_sha256") == AUTH_SHA and auth.get("schema") == FIT_SCHEMA + ".authorization"
        and auth.get("source_sha256") == SOURCE, "external actual fit authorization identity differs")
    for key, wanted in dict(research_fit_authorized=True, native_denials_preserved=True, DEV_or_HOLDOUT_used_for_fit=False,
        automatic_model_promotion=False, vehicle_control_approved=False).items(): require(auth.get(key) is wanted, "fit authority scope differs: " + key)
    # HH_261002 - A prepared contract grants no fit; only the separately pinned authorization changes this one flag.
    require(auth["contract"].get("training_authorized") is False and summary["training_authorized"] is True,
        "prepared permission and actual external research permission must remain distinct")
    for key, value in auth["contract"].items():
        if key != "training_authorized": require(key in summary and summary[key] == value, "external prepared contract not retained: " + key)
    require(summary["mining_artifact_sha256"] == MINING_SHA and summary["initial_model_receipt_sha256"] == LINEAGE_SHA,
        "current228 lineage and old c019 mining pins differ")
    for prefix, rows in (("mining", MINING_SHA), ("initial_lineage", LINEAGE_SHA)):
        for name, sha in rows.items(): pins.file(root / prefix / previous.relative_name(name), sha)
    pins.file(root / "inputs/manifest.json", DATA_SHA); pins.file(root / "inputs/study_train_registry.json", REGISTRY_SHA)
    initial = pins.npz(root / "initial_model_weights.npz", INITIAL_SHA); frozen.numeric_weights(initial, initial)
    complete = pins.json(root / "fit_complete.json", ref["files"]["fit_complete.json"])
    info = summary["arms_result"]
    require(set(info) == {"control", "candidate"} and complete.get("status") == "BOTH_FIXED_FINAL4000_ENDPOINTS_SAVED_BEFORE_KNOWN_EVALUATION"
        and complete.get("arms") == info, "both fixed endpoints must precede known-site evaluation")
    receipt = pins.json(root / "intervention_archive_receipt.json", ref["files"]["intervention_archive_receipt.json"])
    proof = pins.json(root / "equal_global_schedule_proof.json", receipt["schedule_proof_file_sha256"])
    schedule = pins.npz(root / "equal_global_schedule.npz", receipt["schedule_npz_sha256"])
    previous.validate_schedule(schedule, proof)
    require(document_sha(proof) == EQUAL_PROOF_SHA == auth["equal_global_proof_sha256"] == receipt["equal_global_proof_sha256"]
        and proof == summary["sampler_proof"] and proof["schema"] == FIT_SCHEMA + ".equal_global_schedule"
        and proof["both_arm_slot_roles"] == ["all_TRAIN"] * 8, "externally committed equal-global proof differs")
    old = pins.npz(root / "mining/proposed_schedule.npz", MINING_SHA["proposed_schedule.npz"])
    require(np.array_equal(schedule["control_indices"], schedule["candidate_indices"])
        and np.array_equal(schedule["control_indices"], old["control_indices"])
        and all(np.array_equal(schedule[k], old[k]) for k in ("shared_crops", "shared_FALSE_view_bits")), "sole intervention is not loss")
    loss_contract = pins.json(root / "loss_contract.json", receipt["loss_contract_file_sha256"])
    require(document_sha(loss_contract) == LOSS_CONTRACT_SHA == auth["loss_contract_sha256"] == receipt["loss_contract_sha256"]
        and loss_contract == summary["loss_contract"] and summary["loss_contract_sha256"] == LOSS_CONTRACT_SHA
        and receipt["schema"] == FIT_SCHEMA + ".archive" and receipt["original_c019_unmodified"] is True
        and receipt["training_authorized_by_this_receipt"] is False, "fixed loss/denial archive differs")
    models, exposures = {}, {}
    for arm, row in info.items():
        require(row["steps"] == 4000 and row["examples_seen"] == 32000 and row["parameter_count"] == 245831
            and row["initial_model_weights_sha256"] == INITIAL_SHA and row["initial_state_sha256"] == INITIAL_STATE_SHA
            and row["optimizer_initial_state_entries"] == 0, "same initial state/fresh fixed4000 endpoint required")
        require(pins.json(root / arm / "training.json", ref["files"][arm + "/training.json"]) == row, "arm endpoint receipt differs")
        models[arm] = pins.npz(root / arm / "weights.npz", row["weights_sha256"]); frozen.numeric_weights(models[arm], initial)
        pins.file(root / arm / "checkpoint.pt", row["checkpoint_sha256"])
        exposures[arm] = previous.exposure_proof(pins.payload(root / arm / "exposures.jsonl", row["full_exposure_journal_sha256"]), schedule, arm, row)
        history = [frozen.strict_json(line) for line in pins.payload(root / arm / "history.jsonl", ref["files"][arm + "/history.jsonl"]).splitlines()]
        require(len(history) == 40 and all(r.get("arm") == arm and type(r.get("step")) is int and r["step"] == (i + 1) * 100
            and frozen.finite_number(r.get("mean_loss_100")) and frozen.finite_number(r.get("wall_seconds"))
            and (not i or r["wall_seconds"] >= history[i - 1]["wall_seconds"]) for i, r in enumerate(history)), "all40 loss rows required")
        witness = pins.json(root / (arm + "_strict_CUDA_preflight.json"), ref["files"][arm + "_strict_CUDA_preflight.json"])
        for key, value in dict(schema=FIT_SCHEMA + ".strict_CUDA_preflight", arm=arm, status="STRICT_CUDA_FORWARD_BACKWARD_PASS_ZERO_UPDATES",
            parameter_count=245831, all_parameter_gradients_present_and_finite=True, initial_state_sha256=INITIAL_STATE_SHA,
            optimizer_updates=0, all_arm_optimizer_updates_so_far=0, completed_fit_arms=[], extra_TRAIN_diagnostic_exposures=8,
            known_DEV_or_HOLDOUT_opened=False, strict_deterministic_algorithms=True, TF32=False).items():
            require(type(witness.get(key)) is type(value) and witness[key] == value, "strict zero-update preflight differs: " + key)
        require(witness["actual_TRAIN_slot_indices"] == schedule[arm + "_indices"][0].tolist()
            and witness["shared_crop_boxes"] == schedule["shared_crops"][0].tolist()
            and witness["actual_paired_TRUE_bits"] == [0] * 8, "actual preflight first batch differs")
        for key in ("rgb_array_sha256", "target_array_sha256"): frozen.sha_string(witness[key])
    require(pins.json(root / "known_evaluation.json", ref["files"]["known_evaluation.json"]) == summary["known_evaluation"], "known350 receipt differs")
    return dict(root=root, summary=summary, schedule=schedule, models=models, initial=initial, exposures=exposures)


def audit(caller_path, caller_sha):
    require("torch" not in sys.modules and frozen.contract.digest(Path(previous.__file__)) == FROZEN_GALLERY_SHA
        and frozen.contract.digest(Path(frozen.__file__)) == previous.FROZEN_PUBLISHER_SHA, "unchanged publication providers/no Torch required")
    pins = frozen.Pins(); caller = pins.json(caller_path, caller_sha)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("output") == str(OUTPUT) and caller.get("fixed_indices") == INDICES,
        "prospective fixed10 caller required")
    require(caller.get("remote_originals_independently_hashed") is True
        and caller.get("remote_run_root") == REMOTE_RUN, "externally sealed actual private remote-original pins required")
    require(set(caller.get("publication_source_sha256", {})) == set(publication_sources()), "full publication source closure required")
    for name, path in publication_sources().items(): pins.file(path, caller["publication_source_sha256"][name])
    require(caller["font"]["path"] == str(FONT), "fixed font identity required"); pins.file(FONT, caller["font"]["sha256"])
    study = read_study(caller["run"], pins)
    require(caller["evaluation"] == dict(root=str(EVAL), files={s + ".npz": sha for s, (_, sha) in SPLITS.items()}), "exact known350 refs required")
    manifest = pins.json(EVAL / "manifest.json", frozen.EVAL_MANIFEST_SHA)
    metrics, panels = {}, []
    for split, (count, sha) in SPLITS.items():
        ref = manifest["files"][split]
        require(all(ref.get(k) == v for k, v in dict(file=split + ".npz", count=count, sha256=sha).items()), "known evaluation manifest differs")
        data = pins.npz(EVAL / (split + ".npz"), sha)
        require(set(data) == {"rgb", "target"}, "full RGB/GT keys required"); rgb, target = data["rgb"], data["target"]
        require(rgb.shape == (count, *SHAPE, 3) and target.shape == (count, *SHAPE) and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < 7, "full native FOV evaluation ABI differs")
        predictions, metrics[split] = {}, {}
        for arm in ("parent", "control", "candidate"):
            pred, row = previous.read_prediction(pins, study, caller["run"], split, arm, target)
            predictions[arm] = pred; metrics[split][arm] = row
        for index in INDICES[split]: panels.append((split, index, rgb[index].copy(), target[index].copy(),
            *(predictions[a][index].copy() for a in ("parent", "control", "candidate"))))
    screen = diagnostic_screen(metrics)
    require(screen == study["summary"]["exploratory_screen_result"], "all27 saved predicates differ")
    pins.recheck(); require(previous.inventory(RUN) == set(caller["run"]["files"]), "mirror inventory changed")
    report = dict(schema=SCHEMA, status="FULL350_STORED_LOSS_AB_VERIFIED_EXPLORATORY_NOT_APPROVED", evaluation=metrics,
        exploratory_screen=screen, fixed_indices=INDICES, native_FOV_hw=list(SHAPE), full_cohort_counts={s: n for s, (n, _) in SPLITS.items()},
        source_count=10, actual_optimizer_exposures=study["exposures"], additional_CUDA_preflight_exposures_each_arm=8,
        both_arm_indices_crops_FALSE_bits_byte_identical=True, model_weights_sha256=dict(parent=INITIAL_SHA,
            **{a: study["summary"]["arms_result"][a]["weights_sha256"] for a in ("control", "candidate")}),
        first_read_source_and_artifact_pins=dict(pins.values), caller_sha256=caller_sha, mirrored_run_inventory=sorted(caller["run"]["files"]),
        remote_run_root=REMOTE_RUN, remote_originals_independently_hashed_by_caller=True,
        original_CE_unchanged=True, candidate_auxiliary_coefficient=.10, known_exposed_diagnostic_not_independent=True,
        hypothesis_adapted_after_prior_known_site_diagnostics=True, independent_new_dev_sites=0, failed_checks_retained=True,
        model_inference_executed=False, Torch_imported=False, pickle_loaded=False, raw_TRAIN_reopened=False,
        frozen_fit_postcheck_receipts_authenticated=True, current_runtime_record_not_historical_binary_attestation=True,
        RGB_only_seven_class_perception_not_HD_AMG=True, traffic_light_fixture_not_signal_state=True, GT_is_not_a_model_map=True,
        model_promoted=False, vehicle_control_approved=False, formal_main_functions_approved=0, formal_subfunctions_approved=0)
    return report, panels, pins


def canvas(panel, font, report):
    split, index, rgb, *maps = panel; height, width = SHAPE
    require(rgb.shape == (*SHAPE, 3) and rgb.dtype == np.uint8 and len(maps) == 4
        and all(v.shape == SHAPE and v.dtype == np.uint8 and int(v.max()) < 7 for v in maps), "full RGB/four semantic panels required")
    screen = report["exploratory_screen"]; passed = sum(screen["checks"].values())
    verdict = "PASS exploratory only / NOT approved" if screen["passed_exploratory_only"] else "FAIL / NOT adopted"
    out = Image.new("RGB", (3 * width, 2 * height + 180), (18, 24, 35)); draw = ImageDraw.Draw(out)
    draw.text((12, 8), f"{split} | fixed index {index} | full350: {passed}/27 {verdict}", font=font, fill="white")
    titles = ["Original RGB / full FOV", "Ground truth / not model input", "Parent228 / initial10000",
        "Control / original CE +4000", "Candidate / CE +0.10 sign margin +4000"]
    for i, (title, pixels) in enumerate(zip(titles, [rgb, *(COLORS[m] for m in maps)])):
        x, y = (i % 3) * width, 44 + (i // 3) * (height + 40)
        draw.text((x + 10, y), title, font=font, fill="white"); out.paste(Image.fromarray(pixels), (x, y + 30))
    x, y = 2 * width + 12, height + 94
    notes = ["All350 evaluation images retained", "Same source/crop/FALSE views; loss only", "Saved predictions; NO inference", "Known sites, NOT independent", "Weights SHA256 (full hashes in results):"]
    notes += [arm + ": " + sha[:24] for arm, sha in report["model_weights_sha256"].items()]
    for i, text in enumerate(notes): draw.text((x, y + 27 * i), text, font=font, fill="white")
    for i, (label, color) in enumerate(zip(LABELS, COLORS)):
        x, y = 12 + i * 272, 2 * height + 133
        draw.rectangle((x, y, x + 12, y + 12), fill=tuple(int(c) for c in color)); draw.text((x + 17, y - 4), label, font=font, fill="white")
    draw.text((12, 2 * height + 155), "RGB7 only; light fixture NOT signal state; GT/predictions NOT HD map; main0/9 sub0/30; no automatic promotion.", font=font, fill="white")
    return out


def markdown(report, files):
    screen = report["exploratory_screen"]; failed = [k for k, v in screen["checks"].items() if not v]
    pct = lambda value: "미측정(분모 없음)" if value is None else f"{100 * value:.2f}%"
    verdict = "탐색 기준 PASS(기능 승인 아님)" if not failed else "FAIL / 미채택"
    lines = ["# 표지 손실 교정: 실제 결과와 원본 화면", "", f"전체 기존 평가 350장·27개 판정: **{27 - len(failed)}/27, {verdict}**.", "",
        "같은 10,000회 시작 모델에서 대조군·후보군을 각각 4,000회 추가 학습했습니다. 양쪽 실제 optimizer 노출은 각각 32,000회이며, CUDA 사전검사 8회 노출씩은 별도입니다. 영상·정답·crop·자료 순서·FALSE 영상·원 CE는 같고 후보만 표지 margin 손실 0.10을 더했습니다.", "",
        "Town01 118장·Town02 232장 전체를 검산했습니다. 이미 사용한 장소에서 만든 적응 가설의 진단이며 신규 독립 장소는 0개입니다. 아래 10장은 결과 전에 고정한 인덱스입니다. 7분류 인식은 신호 상태·좌표 누적·차선 연결을 가진 정밀지도 AMG가 아닙니다. 메인 0/9·하위 0/30, 자동 채택 금지입니다.", "",
        "미달 판정: " + (", ".join(failed) if failed else "없음. 독립 검증·기능 승인은 여전히 별도입니다."), ""]
    for split, cohort in report["evaluation"].items():
        lines += [f"## {split}: 전체 분류 IoU", "", "| 분류 | 부모228 | 대조군 | 손실 교정 후보 |", "|---|---:|---:|---:|"]
        for label in LABELS: lines.append("| " + label + " | " + " | ".join(pct(cohort[a]["iou"][label]) for a in ("parent", "control", "candidate")) + " |")
        lines += ["", "| 표지 지표 | 부모228 | 대조군 | 후보 |", "|---|---:|---:|---:|"]
        for label, key in (("정밀도", "precision"), ("재현율", "recall")):
            lines.append("| " + label + " | " + " | ".join(pct(cohort[a][key]["traffic_sign"]) for a in ("parent", "control", "candidate")) + " |")
        lines.append("| other→표지 오탐 픽셀 | " + " | ".join(str(cohort[a]["other_to_sign_false_positive_pixels"]) for a in ("parent", "control", "candidate")) + " |")
        lines += [""]
    lines += ["## 평가 전에 고정한 전체 화각 화면", "", "위는 실제 RGB·정답·시작 모델, 아래는 대조군·손실 교정 후보입니다. 정답은 모델 입력이나 모델이 만든 지도가 아닙니다.", ""]
    for name in files: lines += [f"![{name}]({name})", ""]
    lines += ["[results.json](results.json)에 전체 49칸 confusion, 7분류 FN/FP·정밀도·재현율, 27개 판정과 전체 가중치 SHA를 남겼습니다. 저장 예측만 읽었으며 새 추론·Torch·pickle·GPU를 사용하지 않았습니다.", ""]
    return "\n".join(lines)


def publish(args):
    frozen.contract.canonical(OUTPUT); require(not OUTPUT.exists(), "fresh fixed publication only")
    report, panels, pins = audit(Path(args.caller_pins), args.caller_pins_sha256)
    if args.validate_only: return dict(status=report["status"], published=False, fixed_preview_count=len(panels), full_evaluation_count=sum(n for n, _ in SPLITS.values()))
    ancestor = next(p for p in OUTPUT.parents if p.exists())
    require(shutil.disk_usage(ancestor).free >= 5 * 1024**3 + 128 * 1024**2, "5GiB floor plus128MiB publication reserve required")
    sources = {name: pins.payload(path, pins.values[str(path)]) for name, path in publication_sources().items()}
    pins.recheck(); OUTPUT.mkdir(parents=True); font = ImageFont.truetype(str(FONT), 18); files = {}
    for panel in panels:
        name = f"{panel[0]}_{panel[1]:04d}.png"
        with (OUTPUT / name).open("xb") as stream: canvas(panel, font, report).save(stream, format="PNG")
        files[name] = frozen.contract.digest(OUTPUT / name)
    for name, value in sources.items():
        path = OUTPUT / "source" / name; path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream: stream.write(value)
        require(frozen.contract.digest(path) == pins.values[str(publication_sources()[name])], "publication source archive differs")
    pins.recheck(); require(previous.inventory(RUN) == set(report["mirrored_run_inventory"]), "mirror changed during publication")
    report.update(created_at_utc=datetime.now(timezone.utc).isoformat(), files=files, original_sources_and_inputs_unchanged=True,
        publication_source_archive={"source/" + k: pins.values[str(v)] for k, v in publication_sources().items()})
    frozen.contract.write_new(OUTPUT / "results.json", report)
    with (OUTPUT / "README.md").open("x") as stream: stream.write(markdown(report, files))
    pins.recheck()
    return dict(status=report["status"], published=True, output=str(OUTPUT), png_count=len(files), results_sha256=frozen.contract.digest(OUTPUT / "results.json"), README_sha256=frozen.contract.digest(OUTPUT / "README.md"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--caller-pins", required=True); parser.add_argument("--caller-pins-sha256", required=True); parser.add_argument("--validate-only", action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)), ensure_ascii=False, sort_keys=True), flush=True); return 0


if __name__ == "__main__": raise SystemExit(main())
