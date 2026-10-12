"""HH_261001 - Train only the NEW learned route branch from actual signed recovery plus protected old TRAIN.

Fixed final3000, no holdout fitting/early selection, no installs or automatic
physical use. All97 original tensors remain byte-identical to mixed3000. New
actual recovery qualification is mandatory before even CPU data preflight.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import random
import time

if __name__ == "__mp_main__":
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_recovery_worker_workspace", _entry)
    _module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader
from torch.utils.data.dataloader import default_collate

from portable_e2e.dataset import _safe_file
from portable_e2e.deterministic_linear_interpolation_v2 import KERNEL_ID, DeterministicPathTimeDecoderV2
from portable_e2e.temporal_balanced_export_loader_research_20261001 import OWNER_SCHEMA as PARENT_OWNER_SCHEMA, load_balanced_research_npz
from portable_e2e.temporal_losses_v2 import temporal_trajectory_loss_v2
from portable_e2e.temporal_route_recovery_v3_20261001 import BRANCH_PREFIX, MODEL_ID, SCHEMA as ARCHITECTURE_SCHEMA, EXPORT_SCHEMA, RouteRecoveryTemporalModelV3, causal_near_route_features
from portable_e2e.temporal_route_recovery_export_loader_research_20261001 import (
    BATCH_DOMAINS, BRANCH_KEYS, PARENT_CHECKPOINT_SHA, PARENT_NPZ_SHA, PARENT_METADATA_SHA,
    PARENT_DECLARATION_SHA, backbone_fingerprint,
)
from scripts.e2e import train_temporal_v2_balanced_replay_20261001 as replay
from scripts.e2e import train_temporal_v2_low_speed_joint_20261001 as joint
from scripts.e2e import train_temporal_v2_raw_bootstrap_20261001 as raw_parent

ROOT = Path(__file__).resolve().parents[2]
PERSONAL, WORKSPACE, GPU_UUID = raw_parent.PERSONAL, raw_parent.WORKSPACE, raw_parent.GPU_UUID
SCHEMA = "portable_e2e.temporal_route_recovery_branch_training.20261001.v1"
CONVERSION_SCHEMA = "carla.actual_signed_recovery_common10_conversion.20261001.v3"
QUALIFICATION_SCHEMA = "carla.actual_signed_recovery_common10_qualification.20261001.v3"
TRAIN_BASENAME = "town07-signed-recovery-train-20261001-v3"
HOLDOUT_BASENAME = "town07-signed-recovery-holdout-20261001-v3"
MAX_UPDATES, MAX_WALL_SECONDS, SEED = 3000, 2700, 2026100104
BRANCH_PEAK_LR, BRANCH_FINAL_LR, WARMUP_UPDATES = 3e-4, 3e-5, 100
TRAIN_PROFILES = frozenset((offset, yaw) for offset in (-.20, 0., .20) for yaw in (-1.5, 0., 1.5))
HOLDOUT_PROFILES = frozenset(((-.15, 1.), (.15, -1.)))
SIGNED_CTE_KEY = "actual_measured_rear_base_signed_CTE_m"
require, digest, write_new = raw_parent.require, raw_parent.digest, raw_parent.write_new


def read(path):
    return json.loads(Path(path).read_bytes())


def source_snapshot():
    seeds = [Path(__file__).resolve(), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]
    paths = set(replay.imported_source_closure(ROOT, seeds))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    sources = {str(path.relative_to(ROOT)): digest(path) for path in sorted(paths)}
    require(all(sources.get(k) == sha for k, sha in replay.source_snapshot().items()), "all17 inherited training sources frozen")
    return sources


def signed_bin(value, threshold):
    require(type(value) in (float, int) and math.isfinite(value), "actual signed state must be finite")
    return -1 if value <= -threshold else 1 if value >= threshold else 0


def recovery_groups(dataset, audits):
    """HH_261001 - Every anchor remains in full-cycle; extra exposure uses observed current state, never initial-case guesses."""
    require(set(audits) == {r.token for r in dataset.examples}, "exact every-anchor signed audit required")
    full, focus, episode_profiles = {}, {}, {}
    for index, row in enumerate(dataset.examples):
        audit = audits[row.token]
        require(audit["episode_id"] == row.episode_id and audit.get("token", audit.get("sample_id")) == row.token,
                "signed audit/native sample identity differs")
        profile = (audit["declared_signed_lateral_offset_ros_m"], audit["declared_signed_yaw_offset_ros_deg"])
        require(profile in TRAIN_PROFILES,
                "actual9 TRAIN profile declarations differ")
        require(row.episode_id not in episode_profiles or episode_profiles[row.episode_id] == profile,
                "profile changed within original episode")
        episode_profiles[row.episode_id] = profile; full.setdefault(profile, []).append(index)
        grid = (signed_bin(audit[SIGNED_CTE_KEY], .05),
                signed_bin(audit["actual_yaw_route_heading_error_rad"], math.radians(.5)))
        if grid != (0, 0): focus.setdefault(grid, []).append(index)
    require(len(full) == len(episode_profiles) == 9 and all(len(v) == 302 for v in full.values()),
            "all9 actual TRAIN profiles must retain full302 anchors")
    require(focus and len({i for v in focus.values() for i in v}) > 0, "no genuine noncentered current states; no fake or centered substitute")
    return dict(full=full, focus=focus, episode_profiles=episode_profiles)


def fixed_batches(groups, legacy_strata):
    """HH_261001 - 1 signed-current-state focus +1 all9/full302 +1 V4 +1 alternating old geometry, no drop-last."""
    focus_keys, full_keys = sorted(groups["focus"]), sorted(groups["full"])
    require(focus_keys and full_keys and all(groups["focus"].values()) and all(groups["full"].values()),
            "genuine nonempty signed focus and all-profile queues required")
    focus = {k: replay.CycleQueue(v, SEED + 100 + i) for i, (k, v) in enumerate(sorted(groups["focus"].items()))}
    full = {k: replay.CycleQueue(v, SEED + 200 + i) for i, (k, v) in enumerate(sorted(groups["full"].items()))}
    low = replay.CycleQueue(list(range(327)), SEED + 300)
    old = {kind: {k: replay.CycleQueue(v, SEED + 400 + 20 * p + i)
                    for i, (k, v) in enumerate(sorted(legacy_strata[kind].items()))}
           for p, kind in enumerate(("curved", "straight"))}
    counts = Counter(); order = []
    for step in range(MAX_UPDATES):
        a = focus[focus_keys[step % len(focus_keys)]].next()
        profile = full_keys[step % len(full_keys)]; b = full[profile].next()
        kind = "curved" if step % 2 == 0 else "straight"
        maps = sorted(old[kind]); selected_map = maps[counts[kind] % len(maps)]; counts[kind] += 1
        order.append([a, b, 2718 + low.next(), 2718 + 327 + old[kind][selected_map].next()])
    return order


def exact_objective(recovery0, recovery1, v4_joint, legacy_raw_curvature):
    # HH_261001 - Per-anchor masked losses are already normalized; the effective B4 weights are fixed before fitting.
    return .25 * recovery0 + .25 * recovery1 + .25 * v4_joint + .0625 * legacy_raw_curvature


def branch_learning_rate(update):
    # HH_261001 - A NEW zero-output route branch gets its own prospectively fixed rate, not a pretrained backbone's tiny rate.
    require(type(update) is int and 1 <= update <= MAX_UPDATES, "fixed new branch learning-rate horizon required")
    if update <= WARMUP_UPDATES: return BRANCH_PEAK_LR * update / WARMUP_UPDATES
    progress = (update - WARMUP_UPDATES) / (MAX_UPDATES - WARMUP_UPDATES)
    return BRANCH_FINAL_LR + .5 * (BRANCH_PEAK_LR - BRANCH_FINAL_LR) * (1 + math.cos(math.pi * progress))


def learning_rate_config():
    return dict(peak=BRANCH_PEAK_LR, final=BRANCH_FINAL_LR, warmup_updates=WARMUP_UPDATES,
        schedule="fixed3000_linear_warmup_cosine_branch_only", units="AdamW optimizer learning rate",
        gradient_clip_l2=5., weight_decay=1e-4, inherited_joint_schedule_reused=False)


def verify_full_cohort(document):
    # HH_261001 - Both group receipts must prove all11 exact prospective sources passed, not a successful group subset.
    require(document.get("all_eleven_fixed_cohort_cases_qualified") is True
            and type(document.get("full_cohort_case_count")) is int and document["full_cohort_case_count"] == 11
            and type(document.get("full_cohort_fixed_selected_anchors")) is int and document["full_cohort_fixed_selected_anchors"] == 3322,
            "all11 actual cases and all3322 fixed windows must qualify before any fit")
    rows = document.get("full_cohort_case_provenance")
    expected = {"train_%02d" % i: ("train", 2026100510+i, *profile) for i, profile in enumerate(sorted(TRAIN_PROFILES))}
    expected.update(holdout_00=("dev", 2026100610, -.15, 1.), holdout_01=("dev", 2026100611, .15, -1.))
    require(isinstance(rows, list) and len(rows) == 11 and {r["case_id"] for r in rows} == set(expected)
            and len({r.get("native_episode") for r in rows}) == 11
            and all(isinstance(r.get("native_episode"), str) and r["native_episode"] for r in rows),
            "all11 distinct prospective v3 case identities required")
    for row in rows:
        require((row.get("dataset_split_intent"), row.get("seed"), row.get("signed_lateral_offset_ros_m"), row.get("signed_yaw_offset_ros_deg"))
                    == expected[row["case_id"]]
                and type(row.get("seed")) is int and type(row.get("selected_camera_anchors")) is int and row["selected_camera_anchors"] == 302
                and type(row.get("selected_span_ns")) is int and row["selected_span_ns"] >= 30_000_000_000
                and row.get("all_fixed_anchor_necessary_checks_clear") is True,
                "actual v3 profile/seed/split/span/whole-window proof differs")
        require(all(isinstance(row.get(k), str) and len(row[k]) == 64 and all(c in "0123456789abcdef" for c in row[k]) for k in
                    ("case_plan_sha256", "native_manifest_sha256", "native_state_sha256", "native_camera_sha256", "source_route_sha256")),
                "all11 exact native provenance hash pins required")
    return {r["case_id"]: r for r in rows}


def verify_conversion(conversion, split):
    # HH_261001 - Only prospectively gear-preselected v3 whole-case qualification is admissible; all v1/v2 failures remain excluded.
    require(split in ("train", "val"), "TRAIN or reserved HOLDOUT only")
    count, episodes = (2718, 9) if split == "train" else (604, 2)
    integers = dict(episode_count=episodes, train_examples=count if split == "train" else 0,
                    val_examples=count if split == "val" else 0, test_examples=0, true_future_points=count * 64)
    flags = dict(raw_native_denials_overridden=False, vehicle_control_approved=False, model_promoted=False,
        model_or_vehicle_release_approved=False, exact_native_ego_navigation_and_target_reconstruction=True,
        research_training_authorized=split == "train", research_holdout_evaluation_authorized=split == "val",
        holdout_training_or_checkpoint_selection_allowed=False, original_pending_metadata_unchanged=True,
        synthetic_history_created=False, labels_clipped=False, labels_or_pixels_transformed=False,
        future_labels_used_as_model_input=False, actual_hypot_current_speed_no_clipping=True,
        causal_navigation_support_not_future_distance=True)
    require(conversion.get("schema") == CONVERSION_SCHEMA
            and conversion.get("status") == ("QUALIFIED_RESEARCH_TRAIN_COMMON10" if split == "train" else "QUALIFIED_HOLDOUT_COMMON10_EVALUATION_ONLY")
            and all(type(conversion.get(k)) is int and conversion[k] == v for k, v in integers.items())
            and all(conversion.get(k) is v for k, v in flags.items()),
            "separately qualified actual v3 whole TRAIN versus never-fit HOLDOUT conversion required")
    verify_full_cohort(conversion)
    records = conversion.get("per_episode_native_qualification")
    require(isinstance(records, list) and len(records) == episodes
            and len({r["episode_id"] for r in records}) == len({r["case_id"] for r in records}) == episodes
            and all(r["split"] == split and type(r["selected_examples"]) is int and r["selected_examples"] == 302 for r in records)
            and {(r["signed_lateral_offset_ros_m"], r["signed_yaw_offset_ros_deg"]) for r in records}
                == (TRAIN_PROFILES if split == "train" else HOLDOUT_PROFILES),
            "all actual v3 profiles and unchanged302 anchors per case required")
    return count, episodes, {r["episode_id"]: r for r in records}


def verify_signed_audits(rows, examples, cases):
    # HH_261001 - Signed current pose is authenticated for sampling only, never fed into the neural model.
    expected = {r.token: r for r in examples}; audits = {}
    for value in rows:
        token = value.get("token")
        require(token in expected and token not in audits and value.get("sample_id") == token,
                "duplicate, omitted or foreign actual signed audit anchor")
        example = expected[token]; case = cases[example.episode_id]
        cte, yaw = value.get(SIGNED_CTE_KEY), value.get("actual_yaw_route_heading_error_rad")
        signed_bin(cte, .05); signed_bin(yaw, math.radians(.5))
        require(value.get("case_id") == case["case_id"] and value.get("episode_id") == example.episode_id
                and value.get("timestamp_ns") == example.anchor_timestamp_ns
                and value.get("source_native_manifest_sha256") == case["native_manifest_sha256"]
                and value.get("diagnostic_only_not_model_input") is True
                and value.get("declared_signed_lateral_offset_ros_m") == case["signed_lateral_offset_ros_m"]
                and value.get("declared_signed_yaw_offset_ros_deg") == case["signed_yaw_offset_ros_deg"]
                and type(value.get("actual_measured_rear_base_CTE_m")) in (int, float)
                and math.isclose(value["actual_measured_rear_base_CTE_m"], abs(cte), rel_tol=0., abs_tol=1e-12),
                "signed audit must bind actual original sample, timestamp, native source and profile")
        audits[token] = value
    require(set(audits) == set(expected), "full fixed signed audit token denominator differs")
    return audits


def verify_qualification(qualification, split, count, episodes):
    flags = dict(research_training_authorized=split == "train", research_holdout_evaluation_authorized=split == "val",
        holdout_training_or_checkpoint_selection_allowed=False, raw_native_denials_overridden=False,
        original_pending_metadata_unchanged=True, window_filtering_performed=False, thresholds_relaxed=False,
        labels_or_pixels_transformed=False, synthetic_history_created=False, training_run_performed=False,
        vehicle_control_approved=False, model_promoted=False, model_or_vehicle_release_approved=False)
    require(qualification.get("schema") == QUALIFICATION_SCHEMA and qualification.get("run_id") == "v3"
            and qualification.get("group") == ("train" if split == "train" else "holdout")
            and qualification.get("status") == ("QUALIFIED_RESEARCH_TRAIN" if split == "train" else "QUALIFIED_RESERVED_HOLDOUT_EVALUATION_ONLY")
            and all(qualification.get(k) is v for k, v in flags.items())
            and type(qualification.get("episode_count")) is int and qualification["episode_count"] == episodes
            and type(qualification.get("selected_camera_anchors")) is int and qualification["selected_camera_anchors"] == count
            and len(qualification.get("cases", [])) == episodes
            and all(r.get("all_fixed_anchor_necessary_checks_clear") is True and r.get("selected_camera_anchors") == 302
                    for r in qualification["cases"]), "authenticated whole actual recovery qualification scope differs")
    verify_full_cohort(qualification)


def load_admitted(path, split, conversion_path, conversion_sha):
    root, conversion_path = path.resolve(strict=True), conversion_path.resolve(strict=True)
    require(root.is_relative_to((PERSONAL / "dataset/prepared").resolve())
            and root.name == (TRAIN_BASENAME if split == "train" else HOLDOUT_BASENAME)
            and conversion_path.is_relative_to(root) and digest(conversion_path) == conversion_sha,
            "exact create-only personal prepared recovery/conversion identity required")
    conversion = read(conversion_path)
    count, episodes, cases = verify_conversion(conversion, split)
    dataset, receipt = raw_parent.load_view(root, split, conversion["dataset_manifest_sha256"])
    require(len(dataset) == count and len({r.episode_id for r in dataset.examples}) == episodes
            and receipt["validated_fingerprint_sha256"] == conversion["train_loader_fingerprint_sha256" if split == "train" else "val_loader_fingerprint_sha256"]
            and receipt["temporal_fingerprint_sha256"] == conversion["temporal_dataset_fingerprint_sha256"],
            "actual full Common10 loader/temporal denominator differs")
    evidence = []
    manifest = read(root / "dataset.json")
    for reference in manifest["episodes"]:
        manifest_path = _safe_file(root, reference["manifest"], "recovery episode manifest")
        episode, episode_document = manifest_path.parent, read(manifest_path)
        case = cases[episode_document["episode_id"]]
        require(reference["manifest"] == case["manifest"] and episode_document["split"] == split,
                "conversion case and prepared manifest/split differ")
        copied_receipt = _safe_file(episode, "moving_qualification.json", "recovery original qualification")
        require(digest(copied_receipt) == conversion["qualification_receipt_sha256"], "copied original recovery qualification differs")
        qualification = read(copied_receipt)
        verify_qualification(qualification, split, count, episodes)
        require(qualification["full_cohort_case_provenance"] == conversion["full_cohort_case_provenance"],
                "qualification and conversion must bind identical all11 native sources")
        qualified_cases = [r for r in qualification["cases"] if r["case_id"] == case["case_id"]]
        require(len(qualified_cases) == 1, "copied original qualification lacks unique declared native case")
        qualified = qualified_cases[0]
        for key in ("native_manifest_sha256", "source_route_sha256", "native_episode", "case_plan_sha256"):
            require(qualified[key] == case[key], "qualification versus conversion native provenance differs: " + key)
        for name, expected in (("native_manifest.json", case["native_manifest_sha256"]),
                               ("native_route.json", case["source_route_sha256"])):
            require(digest(_safe_file(episode, name, "original native evidence")) == expected,
                    "copied native manifest/causal route differs from pinned qualification")
        evidence.append(dict(episode=str(episode), qualification_sha256=digest(copied_receipt),
                             native_manifest_sha256=case["native_manifest_sha256"], native_route_sha256=case["source_route_sha256"]))
    audit_path = _safe_file(root, "signed_current_state_audit.jsonl", "actual signed current state")
    require(digest(audit_path) == conversion["signed_current_state_audit_sha256"], "whole signed current state audit differs")
    audits = verify_signed_audits([json.loads(line) for line in audit_path.read_bytes().splitlines()], dataset.examples, cases)
    return dataset, audits, dict(receipt, conversion_sha256=conversion_sha,
        qualification_receipt_sha256=conversion["qualification_receipt_sha256"], evidence=evidence,
        signed_audit_sha256=digest(audit_path), statistics=joint.cohort_statistics(dataset))


def load_parent(weights, metadata):
    owner = dict(schema=PARENT_OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id="new-route-branch-training-parent-numeric-validation",
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False, vehicle_control_approved=False,
        model_promoted=False, weights_sha256=PARENT_NPZ_SHA, metadata_sha256=PARENT_METADATA_SHA,
        checkpoint_sha256=PARENT_CHECKPOINT_SHA, declaration_sha256=PARENT_DECLARATION_SHA, fixed_endpoint_updates=3000)
    return load_balanced_research_npz(weights, metadata, weights_sha256=PARENT_NPZ_SHA,
        metadata_sha256=PARENT_METADATA_SHA, research_owner_binding=owner, source_root=ROOT)


def freeze_backbone(model):
    for name, parameter in model.named_parameters(): parameter.requires_grad_(name.startswith(BRANCH_PREFIX))
    require({name for name, p in model.named_parameters() if p.requires_grad} == BRANCH_KEYS,
            "exact four NEW branch tensors only trainable")


def evaluate_joint(model, decoder, dataset, device, audits=None):
    model.eval(); records = []
    with torch.no_grad():
        for offset in range(0, len(dataset), 4):
            indices = list(range(offset, min(offset + 4, len(dataset))))
            batch = raw_parent.move(default_collate([dataset[i] for i in indices]), device)
            raw, decoded, terms = joint.joint_forward(model, decoder, batch)
            xy_error = torch.linalg.norm(decoded["xy"][:, 0] - batch["target_xy"], dim=-1)
            speed_error = (decoded["instantaneous_speed"][:, 1:] - batch["target_speed_mps"]).abs()
            valid_speed = batch["spatial_speed_valid"]
            raw_speed_mae = ((raw["preferred_speed_knots"] - batch["spatial_preferred_speed"]).abs() * valid_speed).sum(-1) / valid_speed.sum(-1).clamp_min(1)
            features = causal_near_route_features(batch["route_xy"], batch["route_mask"])
            delta = model.route_recovery_branch(features) * (features[:, 2::3] > .5).any(-1)[:, None]
            near_valid = batch["spatial_curvature_valid"][:, :3]
            target_near = (batch["spatial_curvature"][:, :3] * near_valid).sum(-1) / near_valid.sum(-1).clamp_min(1)
            predicted_near = (raw["curvature_knots"][:, :3] * near_valid).sum(-1) / near_valid.sum(-1).clamp_min(1)
            for position, index in enumerate(indices):
                records.append(dict(index=index, token=dataset.examples[index].token,
                    ade_6p4s_m=float(xy_error[position].mean()), fde_6p4s_m=float(xy_error[position, -1]),
                    instantaneous_speed_mae_mps=float(speed_error[position].mean()),
                    raw_spatial_preferred_speed_mae_mps=float(raw_speed_mae[position]),
                    true_near_curvature_inv_m=float(target_near[position]), predicted_near_curvature_inv_m=float(predicted_near[position]),
                    learned_near_branch_logit_mean=float(delta[position, :3].mean()),
                    loss_is_batch_aggregate=joint.loss_numbers(terms)))
    result = dict(split=dataset.split, independent_validation=False, full_fixed_denominator=len(records), rows=records,
        mean_ade_6p4s_m=sum(r["ade_6p4s_m"] for r in records)/len(records),
        maximum_ade_6p4s_m=max(r["ade_6p4s_m"] for r in records),
        mean_fde_6p4s_m=sum(r["fde_6p4s_m"] for r in records)/len(records),
        mean_instantaneous_speed_mae_mps=sum(r["instantaneous_speed_mae_mps"] for r in records)/len(records))
    result["role"] = "TRAIN_resubstitution" if dataset.split == "train" else "never_fit_actual_signed_recovery_HOLDOUT"
    episode_by_token = {r.token: r.episode_id for r in dataset.examples}
    for row in result["rows"]: row["episode_id"] = episode_by_token[row["token"]]
    result["per_episode"] = {}
    for episode in sorted(set(episode_by_token.values())):
        selected = [r for r in result["rows"] if r["episode_id"] == episode]
        result["per_episode"][episode] = dict(count=len(selected),
            **{k: sum(r[k] for r in selected) / len(selected) for k in
               ("ade_6p4s_m", "fde_6p4s_m", "instantaneous_speed_mae_mps", "raw_spatial_preferred_speed_mae_mps")})
    if audits is not None:
        result["actual_signed_current_state_counts"] = dict(Counter(str((signed_bin(a[SIGNED_CTE_KEY], .05),
            signed_bin(a["actual_yaw_route_heading_error_rad"], math.radians(.5)))) for a in audits.values()))
    return result


def evaluate_legacy_raw(model, dataset, strata, device):
    # HH_261001 - Preserve original raw-loss definitions and add separately named physical raw speed MAE, with no high-speed decoding.
    model.eval(); records = []
    index_kind = {i: kind for kind in ("curved", "straight") for values in strata[kind].values() for i in values}
    with torch.no_grad():
        for offset in range(0, len(strata["eligible"]), 4):
            chosen = strata["eligible"][offset:offset+4]
            batch = raw_parent.move(default_collate([dataset[i] for i in chosen]), device)
            raw = raw_parent.forward(model, batch)
            mask = batch["spatial_speed_valid"]
            physical_mae = ((raw["preferred_speed_knots"] - batch["spatial_preferred_speed"]).abs() * mask).sum(-1) / mask.sum(-1).clamp_min(1)
            for position, index in enumerate(chosen):
                terms = temporal_trajectory_loss_v2(replay.slice_batch(raw, position, position+1), replay.slice_batch(batch, position, position+1))
                records.append(dict(index=index, token=dataset.examples[index].token,
                    episode_id=dataset.examples[index].episode_id, map=dataset.examples[index].map_id,
                    stratum=index_kind[index], raw_spatial_preferred_speed_mae_mps=float(physical_mae[position]), **joint.loss_numbers(terms)))
    fields = ("total", "raw_curvature", "raw_preferred_speed", "raw_spatial_preferred_speed_mae_mps")
    def aggregate(rows):
        return dict(count=len(rows), **{key: sum(r[key] for r in rows)/len(rows) for key in fields})
    groups = {name: aggregate(records if name == "all_eligible" else [r for r in records if r["stratum"] == name])
              for name in ("all_eligible", "curved", "straight")}
    per_episode = {episode: aggregate([r for r in records if r["episode_id"] == episode])
                   for episode in sorted({r["episode_id"] for r in records})}
    return dict(split=dataset.split, independent_validation=False, legacy_decoded=False,
        full_fixed_eligible_denominator=len(records), groups=groups, per_episode=per_episode, rows=records,
        speed_metric_notice="raw_preferred_speed is original SmoothL1 loss, NOT m/s; raw_spatial_preferred_speed_mae_mps is physical MAE. No legacy decoded MAE exists.")


def finite_branch_gradient(loss, model):
    # HH_261001 - Each declared domain contributes finite gradients independently; frozen parameters never enter autograd.grad.
    parameters = tuple(p for p in model.parameters() if p.requires_grad)
    values = torch.autograd.grad(loss, parameters, retain_graph=True, allow_unused=True)
    present = [v for v in values if v is not None]
    require(present and all(bool(torch.isfinite(v).all()) for v in present), "nonfinite or missing branch domain gradient")
    norm = torch.stack([v.detach().square().sum() for v in present]).sum().sqrt()
    require(bool(torch.isfinite(norm)), "nonfinite branch domain gradient norm")
    return dict(all_present_branch_gradients_finite=True, gradient_l2_norm=float(norm), unused_branch_tensors=sum(v is None for v in values))


def branch_sign_response(before, after, audits):
    # HH_261001 - Desired correction uses recorded expert labels ONLY in this diagnostic, never as a model feature.
    old = {r["token"]: r for r in before["rows"]}; groups = {}
    require(set(old) == {r["token"] for r in after["rows"]} == set(audits), "response requires exact before/final actual tokens")
    for row in after["rows"]:
        audit, prior = audits[row["token"]], old[row["token"]]
        grid = str((signed_bin(audit[SIGNED_CTE_KEY], .05),
                    signed_bin(audit["actual_yaw_route_heading_error_rad"], math.radians(.5))))
        wanted = row["true_near_curvature_inv_m"] - prior["predicted_near_curvature_inv_m"]
        response = row["predicted_near_curvature_inv_m"] - prior["predicted_near_curvature_inv_m"]
        groups.setdefault(grid, []).append(dict(token=row["token"], expert_minus_parent_near_curvature=wanted,
            actual_learned_near_curvature_change=response, learned_branch_logit=row["learned_near_branch_logit_mean"],
            signed_response_eligible=abs(wanted) >= .001, signed_response_agrees=(wanted * response > 0)))
    return dict(research_diagnostic_only=True, no_inference_oracle=True, feature_approved=False,
        groups={key: dict(all_anchor_count=len(rows), signed_response_denominator=sum(r["signed_response_eligible"] for r in rows),
            signed_response_agree_count=sum(r["signed_response_eligible"] and r["signed_response_agrees"] for r in rows), rows=rows)
                for key, rows in groups.items()})


def endpoint_screens(before, after):
    """HH_261001 - Geometry screens remain absolute; frozen-speed effects are diagnosed relative to the actual parent."""
    checks = []
    for cohort, episodes, count in (("recovery_train", 9, 2718), ("recovery_holdout", 2, 604), ("known599", 2, 599), ("legacy", 8, 2487)):
        old, new = before[cohort], after[cohort]
        denominator = "full_fixed_denominator" if cohort.startswith("recovery_") else "full_fixed_eligible_denominator"
        require(old[denominator] == new[denominator] == count and len(old["per_episode"]) == len(new["per_episode"]) == episodes
                and set(old["per_episode"]) == set(new["per_episode"])
                and [(r["index"], r["token"]) for r in old["rows"]] == [(r["index"], r["token"]) for r in new["rows"]],
                "fixed before/final full cohort episode/anchor denominator differs: " + cohort)
    require(before["v4"]["full_fixed_denominator"] == after["v4"]["full_fixed_denominator"] == 327
            and [r["token"] for r in before["v4"]["rows"]] == [r["token"] for r in after["v4"]["rows"]], "old V4 fixed327 denominator differs")
    def add(name, value, threshold):
        checks.append(dict(name=name, actual=value, threshold=threshold, passed=math.isfinite(value) and value <= threshold))
    for cohort in ("recovery_train", "recovery_holdout"):
        for episode, row in after[cohort]["per_episode"].items():
            add(cohort + "/" + episode + "/ADE", row["ade_6p4s_m"], .1)
            add(cohort + "/" + episode + "/FDE", row["fde_6p4s_m"], .25)
            add(cohort + "/" + episode + "/speed_MAE_parent_regression", row["instantaneous_speed_mae_mps"],
                before[cohort]["per_episode"][episode]["instantaneous_speed_mae_mps"] + .01)
    for key in ("mean_ade_6p4s_m", "mean_fde_6p4s_m", "mean_instantaneous_speed_mae_mps"):
        add("oldV4_TRAIN/" + key, after["v4"][key], before["v4"][key] * 1.10 + 1e-4)
    for cohort in ("known599", "legacy"):
        for label in ("aggregate", *sorted(after[cohort]["per_episode"])):
            old = before[cohort]["groups"]["all_eligible"] if label == "aggregate" else before[cohort]["per_episode"][label]
            new = after[cohort]["groups"]["all_eligible"] if label == "aggregate" else after[cohort]["per_episode"][label]
            add(cohort + "/" + label + "/raw_curvature", new["raw_curvature"], old["raw_curvature"] * 1.10 + 1e-4)
    require(len(checks) == 48 and len({r["name"] for r in checks}) == 48, "all48 prospectively fixed diagnostic screens required")
    return dict(fixed_endpoint_updates=3000, diagnostic_screen_count=48, checks=checks, all_diagnostic_screens_passed=all(c["passed"] for c in checks),
        independent_validation=False, feature_approved=False, vehicle_control_approved=False, model_promoted=False,
        absolute_speed_tracking_claimed=False, frozen_parent_7of10_not_relabelled=True)


def scope_fields():
    return dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, stop_intent_trained=False,
        labels_clipped=False, labels_extrapolated=False, backbone_frozen=True, independent_validation=False,
        legacy_decoded=False, checkpoint_selection_by_holdout=False, parent_all_diagnostic_screens_passed=False,
        parent_diagnostic_pass_count=7, runtime_intent="explicit_cruise_GO_not_head", training_episode_count=18,
        recovery_training_episode_count=9, replay_training_episode_count=9, holdout_episode_count=2)


def save_checkpoint(output, model, optimizer, step, sources, identity, holdout_fp, declaration_sha, backbone_sha):
    replay.storage_reserve(output)
    require(backbone_fingerprint(model.state_dict()) == backbone_sha, "original97 tensors changed; do not export")
    require(all(value.dtype == torch.float32 and bool(torch.isfinite(value).all()) for value in model.state_dict().values()),
            "all101 original/learned tensors must be finite float32 before checkpoint/export")
    state = dict(schema=SCHEMA, architecture_schema=ARCHITECTURE_SCHEMA, model_id=MODEL_ID,
        model_config=model.config.to_dict(), recovery_config=model.recovery_config.to_dict(),
        model_state_dict=model.state_dict(), optimizer_state_dict=optimizer.state_dict(), optimizer_updates=step,
        learning_rate=learning_rate_config(),
        additional_optimizer_updates=step, effective_batch_size=4, examples_seen=4 * step,
        recovery_examples_seen=2 * step, v4_examples_seen=step, legacy_examples_seen=step,
        parent_optimizer_updates=3000, parent_known_total_optimizer_updates=3200, known_total_optimizer_updates=3200 + step,
        maximum_optimizer_updates=3000, parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA, parent_npz_sha256=PARENT_NPZ_SHA,
        parent_metadata_sha256=PARENT_METADATA_SHA, backbone_state_sha256=backbone_sha,
        trainable_state_names=sorted(BRANCH_KEYS), batch_domains=BATCH_DOMAINS, source_sha256=sources,
        training_dataset_fingerprint_sha256=identity, holdout_dataset_fingerprint_sha256=holdout_fp,
        declaration_sha256=declaration_sha, **scope_fields())
    folder = output / "checkpoints"; folder.mkdir(exist_ok=True)
    path = folder / ("step_%06d.pt" % step)
    with path.open("xb") as stream: torch.save(state, stream)
    arrays = {k: v.detach().cpu().numpy().copy() for k, v in model.state_dict().items()}
    export = output / "exports" / ("step_%06d" % step); export.mkdir(parents=True, exist_ok=False)
    npz = export / "model_state.npz"
    with npz.open("xb") as stream: np.savez(stream, **arrays)
    metadata = {k: v for k, v in state.items() if k not in ("model_state_dict", "optimizer_state_dict")}
    metadata.update(schema=EXPORT_SCHEMA, checkpoint_sha256=digest(path), npz_sha256=digest(npz),
        state_tensor_shapes={k: list(v.shape) for k, v in arrays.items()},
        state_tensor_dtypes={k: str(v.dtype) for k, v in arrays.items()},
        numerical_revision=dict(kernel_id=KERNEL_ID, strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False))
    write_new(export / "metadata.json", metadata)
    return dict(optimizer_updates=step, path=str(path), sha256=digest(path), optimizer_state_retained=True,
                export_npz_sha256=digest(npz), export_metadata_sha256=digest(export / "metadata.json"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("recovery-train", "recovery-holdout", "recovery-conversion", "holdout-conversion",
                 "parent-weights", "parent-metadata", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--recovery-conversion-sha256", required=True); parser.add_argument("--holdout-conversion-sha256", required=True)
    parser.add_argument("--loader-workers", type=int, choices=(0, 1, 2), default=2)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    require(not args.output.exists() and not args.output.is_symlink()
            and args.output.resolve().is_relative_to((WORKSPACE / "runs/training").resolve()), "fresh create-only personal training output required")
    initial_storage = replay.storage_reserve(args.output.parent)
    torch.set_num_threads(4)
    sources = source_snapshot()
    low, legacy, known, old_strata, old_receipts = replay.load_all(replay.LOW_PATH, replay.LEGACY_PATH, replay.DEV_PATH)
    training, train_audits, train_receipt = load_admitted(args.recovery_train, "train", args.recovery_conversion, args.recovery_conversion_sha256)
    holdout, holdout_audits, holdout_receipt = load_admitted(args.recovery_holdout, "val", args.holdout_conversion, args.holdout_conversion_sha256)
    sets = [{r.token for r in data.examples} for data in (training, holdout, low, legacy, known)]
    require(all(not a & b for i, a in enumerate(sets) for b in sets[i+1:]), "cross-domain TRAIN/VAL/holdout token overlap")
    require(not {r.episode_id for r in training.examples} & {r.episode_id for r in holdout.examples}, "holdout episode leaked into TRAIN")
    require(read(args.recovery_conversion)["full_cohort_case_provenance"] == read(args.holdout_conversion)["full_cohort_case_provenance"],
            "TRAIN/HOLDOUT must originate from one identical prospective qualified all11 cohort")
    groups = recovery_groups(training, train_audits); order = fixed_batches(groups, old_strata)
    require(len(known.examples) == 665 and len(replay.legacy_strata(known)["eligible"]) == 599,
            "previously exposed known DEV denominator remains665 available/599 moving")
    parent = load_parent(args.parent_weights, args.parent_metadata)
    with torch.random.fork_rng(devices=[]):
        # HH_261001 - Seed the NEW first branch layer before GPU ownership; CPU validation never touches CUDA RNG.
        torch.random.default_generator.manual_seed(SEED)
        model = RouteRecoveryTemporalModelV3(parent.config)
        model.load_backbone_state(parent.state_dict()); freeze_backbone(model)
    backbone_sha = backbone_fingerprint(parent.state_dict())
    identity_data = dict(recovery=train_receipt, old=old_receipts, seed=SEED, actual_batch_domains=BATCH_DOMAINS)
    identity = hashlib.sha256(json.dumps(identity_data, sort_keys=True, allow_nan=False).encode()).hexdigest()
    focus_counts = {str(k): len(v) for k, v in groups["focus"].items()}
    focus_members = {i for values in groups["focus"].values() for i in values}
    declaration = dict(schema=SCHEMA, architecture_schema=ARCHITECTURE_SCHEMA, model_id=MODEL_ID,
        model_config=model.config.to_dict(), recovery_config=model.recovery_config.to_dict(),
        cohorts=dict(recovery_train=train_receipt, recovery_holdout=holdout_receipt, replay=old_receipts),
        parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA, parent_npz_sha256=PARENT_NPZ_SHA,
        parent_metadata_sha256=PARENT_METADATA_SHA, backbone_state_sha256=backbone_sha,
        source_sha256=sources, training_dataset_fingerprint_sha256=identity,
        holdout_dataset_fingerprint_sha256=holdout.fingerprint_sha256, trainable_state_names=sorted(BRANCH_KEYS),
        actual_noncentered_grid_counts=focus_counts, actual_profile_full302_counts={str(k): len(v) for k, v in groups["full"].items()},
        actual_noncentered_availability_per_profile={str(profile): sum(i in focus_members for i in indices)
            for profile, indices in groups["full"].items()},
        declared_focus_full_same_anchor_batch_count=sum(row[0] == row[1] for row in order),
        actual_dataset_available_train_anchor_count=5832, actual_eligible_train_pool_anchor_count=5532,
        declared_unique_source_exposure_counts=dict(recovery=len({i for row in order for i in row[:2]}),
            v4=len({r[2] for r in order}), legacy=len({r[3] for r in order})),
        actual_noncentered_rule="abs(actual rear-base CTE)>=.05m OR abs(actual route yaw error)>=.5deg",
        requested_optimizer_updates=3000, maximum_optimizer_updates=3000, effective_batch_size=4, seed=SEED,
        learning_rate=learning_rate_config(),
        batch_domains=BATCH_DOMAINS, fixed_schedule_sha256=hashlib.sha256(json.dumps(order, separators=(",", ":")).encode()).hexdigest(),
        objective=".5 mean(two per-anchor recovery joint) +.25 V4 joint +.25 legacy curvature-only (.25 raw curvature)",
        loss_denominators="unchanged true raw valid knots and decoded temporal masks per anchor; recovery mean2; no label extension",
        endpoint_screens="each9TRAIN+2HOLDOUT ADE<=.10,FDE<=.25; per-episode decoded speed<=parent+.01; oldV4/oldrawcurvature<=parent*1.10+1e-4",
        parent_diagnostic_status="7of10 unpromoted, including Town03 straight raw speed failure", initial_storage=initial_storage,
        maximum_fit_including_baseline_seconds=2700, postfit_checks_separately_timed=True,
        holdout_fitted=False, holdout_selected_checkpoint=False, **scope_fields())
    if args.validate_only:
        print(json.dumps(dict(status="VALIDATED_CPU_NO_CUDA_OR_OUTPUT", **declaration), indent=2, allow_nan=False)); return 0
    with raw_parent.gpu0_lease():
        owned_started = time.monotonic(); replay.storage_reserve(args.output.parent)
        gpu = raw_parent.gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "strict deterministic cuBLAS environment required")
        random.seed(SEED); torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
        torch.use_deterministic_algorithms(True); torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
        args.output.mkdir(parents=True, exist_ok=False); write_new(args.output / "declaration.json", declaration)
        declaration_sha = digest(args.output / "declaration.json"); joint.archive_sources(args.output, sources)
        device = torch.device("cuda:0"); model.to(device)
        decoder = DeterministicPathTimeDecoderV2().to(device)
        optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=BRANCH_PEAK_LR, weight_decay=1e-4)
        write_new(args.output / "started.json", dict(gpu=gpu, pid=os.getpid(), started_utc=datetime.now(timezone.utc).isoformat(),
            trainable_branch_parameter_count=sum(p.numel() for p in model.parameters() if p.requires_grad), frozen_backbone_sha256=backbone_sha))
        def evaluate_all(stage):
            metrics = dict(recovery_train=evaluate_joint(model, decoder, training, device, train_audits),
                recovery_holdout=evaluate_joint(model, decoder, holdout, device, holdout_audits),
                v4=evaluate_joint(model, decoder, low, device),
                known599=dict(evaluate_legacy_raw(model, known, replay.legacy_strata(known), device),
                    role="previously_exposed_known_moving_DEV599_raw_diagnostic", available_known_val_count=665, excluded_count=66),
                legacy=evaluate_legacy_raw(model, legacy, old_strata, device))
            for key, value in metrics.items(): write_new(args.output / (key + "_" + stage + ".json"), value)
            return metrics
        before = evaluate_all("before")
        kwargs = dict(dataset=ConcatDataset([training, low, legacy]), batch_sampler=order,
                      num_workers=args.loader_workers, pin_memory=True)
        if args.loader_workers: kwargs.update(prefetch_factor=2, persistent_workers=True, multiprocessing_context="spawn")
        loader = DataLoader(**kwargs); completed, wall_stop, checkpoints = 0, False, []
        with (args.output / "history.jsonl").open("x") as journal:
            for step, batch in enumerate(loader, 1):
                if time.monotonic() - owned_started >= MAX_WALL_SECONDS: wall_stop = True; break
                batch = raw_parent.move(batch, device); model.train(); optimizer.zero_grad(set_to_none=True)
                combined = (training.examples, low.examples, legacy.examples)
                expected = [combined[0][i].token if i < 2718 else combined[1][i-2718].token if i < 3045
                            else combined[2][i-3045].token for i in order[step-1]]
                require(list(batch["sample_id"]) == expected, "actual source token order differs from fixed TRAIN-only schedule")
                recovery_terms = []; low_terms = None
                for position in range(3):
                    _, _, terms = joint.joint_forward(model, decoder, replay.slice_batch(batch, position, position+1))
                    if position < 2: recovery_terms.append(terms)
                    else: low_terms = terms
                raw = raw_parent.forward(model, replay.slice_batch(batch, 3, 4))
                old_terms = temporal_trajectory_loss_v2(raw, replay.slice_batch(batch, 3, 4))
                old_loss = .25 * old_terms["raw_curvature"]
                domain_gradients = {"recovery0": finite_branch_gradient(recovery_terms[0]["total"], model),
                    "recovery1": finite_branch_gradient(recovery_terms[1]["total"], model),
                    "v4": finite_branch_gradient(low_terms["total"], model), "legacy_raw_curvature": finite_branch_gradient(old_loss, model)}
                loss = exact_objective(recovery_terms[0]["total"], recovery_terms[1]["total"], low_terms["total"], old_terms["raw_curvature"])
                require(bool(torch.isfinite(loss)), "new branch objective nonfinite")
                loss.backward()
                require(all(p.grad is None or bool(torch.isfinite(p.grad).all()) for p in model.parameters()), "branch gradients nonfinite")
                require(all(p.grad is None for name, p in model.named_parameters() if not name.startswith(BRANCH_PREFIX)), "frozen backbone received gradients")
                norm = torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 5.)
                for group in optimizer.param_groups: group["lr"] = branch_learning_rate(step)
                optimizer.step(); completed = step
                row = dict(optimizer_updates=step, examples_seen=4*step, indices=order[step-1], sample_ids=list(batch["sample_id"]),
                    loss=float(loss.detach()), recovery0=joint.loss_numbers(recovery_terms[0]), recovery1=joint.loss_numbers(recovery_terms[1]),
                    v4=joint.loss_numbers(low_terms), legacy_raw_curvature=float(old_terms["raw_curvature"].detach()),
                    per_domain_branch_gradients=domain_gradients,
                    learning_rate=branch_learning_rate(step),
                    preclip_branch_gradient_norm=float(norm), elapsed_owned_seconds=time.monotonic()-owned_started)
                journal.write(json.dumps(row, allow_nan=False) + "\n"); journal.flush()
                if step % 100 == 0: print(json.dumps(dict(status="ACTUAL_BRANCH_TRAINING", **row)), flush=True)
                if step % 500 == 0: checkpoints.append(save_checkpoint(args.output, model, optimizer, step, sources, identity,
                    holdout.fingerprint_sha256, declaration_sha, backbone_sha))
        fit_seconds = time.monotonic() - owned_started
        after, endpoint, post_seconds = None, None, 0.
        if completed == MAX_UPDATES:
            post_started = time.monotonic(); after = evaluate_all("final"); endpoint = endpoint_screens(before, after)
            write_new(args.output / "endpoint_diagnostic_screens.json", endpoint); post_seconds = time.monotonic() - post_started
            write_new(args.output / "actual_signed_branch_response_train.json", branch_sign_response(before["recovery_train"], after["recovery_train"], train_audits))
            write_new(args.output / "actual_signed_branch_response_holdout.json", branch_sign_response(before["recovery_holdout"], after["recovery_holdout"], holdout_audits))
        require(backbone_fingerprint(model.state_dict()) == backbone_sha, "final original97 tensors changed")
        replay.verify_sources(args.output, sources)
        summary = dict(schema=SCHEMA, status="FIXED3000_BRANCH_COMPLETE_NOT_PROMOTED" if completed == 3000 else "FIT_WALL_BUDGET_STOP_NOT_PROMOTED",
            optimizer_updates=completed, examples_seen=4*completed, recovery_examples_seen=2*completed,
            v4_examples_seen=completed, legacy_examples_seen=completed, wall_budget_stop=wall_stop,
            backbone_97_tensors_byte_identical=True, source_hashes_match=True, fit_including_baseline_seconds=fit_seconds,
            postfit_endpoint_seconds=post_seconds, elapsed_owned_seconds=time.monotonic()-owned_started,
            checkpoints=checkpoints, endpoint_diagnostic_screens=endpoint, **scope_fields())
        write_new(args.output / "summary.json", summary); print(json.dumps(summary, allow_nan=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
