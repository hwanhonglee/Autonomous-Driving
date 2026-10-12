"""HH_261001 - Publish the fixed V4-data/V3-route-branch endpoint without fitting, promotion or checkpoint selection.

Importing this create-only publisher uses only the standard library. Full numeric
archive validation uses NumPy lazily; the separately requested reviewed loader
is CPU-only and never loads a Torch pickle. No simulator or optimizer is started.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path
import shutil
import zipfile

from scripts.e2e import publish_temporal_v2_balanced_replay_results_20261001 as prior

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_route_recovery_moving_v4_results.20261001.v1"
NUMERIC_SCHEMA = "portable_e2e.temporal_route_recovery_moving_v4_numeric_receipt.20261001.v1"
RUN_SCHEMA = "portable_e2e.temporal_route_recovery_moving_training.20261001.v2"
EXPORT_SCHEMA = "portable_e2e.temporal_route_recovery_export_research.20261001.v3"
ARCHITECTURE_SCHEMA = "portable_e2e.temporal_route_recovery_model.20261001.v3"
MODEL_ID = "portable_e2e.dense_nav.learned_route_recovery.v3.research"
PARENT_CHECKPOINT_SHA = "c87c582fc4823d8ef3a00faf103c7aaa18481f2e1bfb7d38fea2f4d7136a99e9"
PARENT_NPZ_SHA = "4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735"
PARENT_METADATA_SHA = "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec"
PARENT_DECLARATION_SHA = "b6165e816ca20ae5dd37446afbed7b0e674efd76a77d99eb0cb3114b99d02e93"
PRIOR_FILE = "tools/e2e/publish_temporal_v2_balanced_replay_results_20261001.py"
PRIOR_SHA = "1933050161dd3b85fca6404c461e190241d7c3c49dc63eec7a7e864f58d52f12"
LOADER_FILE = "learning/portable_e2e/temporal_route_recovery_export_loader_research_20261001.py"
LOADER_SHA = "d9f1cb77adec368a48145ed522eef8508a40906c1d327b68d9f8fe03443a7670"
FROZEN_SOURCES = {
    "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py": "68c0dd33f9a774452b5b1863973cac6faaf8f534d8ae5e0426c8e6afe382b3a9",
    "tools/e2e/train_temporal_route_recovery_v3_20261001.py": "6e1909f5574cf1449f56c4e0379507369f9a0f8841d05160a8e2cb861985452b",
    "learning/portable_e2e/temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
    LOADER_FILE: LOADER_SHA,
}
SOURCE_NAMES = prior.SOURCE_NAMES | frozenset((
    "learning/portable_e2e/temporal_balanced_export_loader_research_20261001.py",
    "learning/portable_e2e/temporal_cached_encoder_v2.py",
    "learning/portable_e2e/temporal_cpu_finite_validation_v2_20261001.py",
    *FROZEN_SOURCES,
))
BRANCH_SHAPES = {"route_recovery_branch.0.weight": [32, 9], "route_recovery_branch.0.bias": [32],
    "route_recovery_branch.2.weight": [33, 32], "route_recovery_branch.2.bias": [33]}
BATCH_DOMAINS = ["recovery_noncentered_joint", "recovery_all_joint", "v4_joint", "legacy_raw_curvature"]
RATE = dict(peak=3e-4, final=3e-5, warmup_updates=100,
    schedule="fixed3000_linear_warmup_cosine_branch_only", units="AdamW optimizer learning rate",
    gradient_clip_l2=5., weight_decay=1e-4, inherited_joint_schedule_reused=False)
RAW_COMPONENTS = (*prior.RAW_COMPONENTS, "raw_spatial_preferred_speed_mae_mps")
JOINT_COMPONENTS = ("ade_6p4s_m", "fde_6p4s_m", "instantaneous_speed_mae_mps", "raw_spatial_preferred_speed_mae_mps")
COHORTS = ("recovery_train", "recovery_holdout", "v4", "known599", "legacy")
DOCUMENTS = ("summary.json", "declaration.json", "started.json", "endpoint_diagnostic_screens.json",
    *(cohort + "_" + stage + ".json" for cohort in COHORTS for stage in ("before", "final")),
    "actual_signed_branch_response_train.json", "actual_signed_branch_response_holdout.json",
    "exports/step_003000/metadata.json")
require, is_sha, regular, digest = prior.require, prior.is_sha, prior.regular, prior.digest
read_json, number, close, mean, identities, write_new = prior.read_json, prior.number, prior.close, prior.mean, prior.identities, prior.write_new


def signed_number(value):
    require(type(value) in (float, int) and math.isfinite(value), "finite signed diagnostic number required")
    return value


def canonical_path(path, *, exists=False):
    # HH_261001 - Reject symlink ancestors and lexical escapes before reading inputs or creating a report.
    absolute = Path(path).absolute()
    require(absolute == absolute.resolve(strict=exists)
            and not any(part.is_symlink() for part in (absolute, *absolute.parents)),
            "canonical path without symlink ancestors required: "+str(path))
    return absolute


def pin_payload(path, pins, *, expected_sha=None, limit=64*1024**2):
    path = canonical_path(path, exists=True)
    payload = regular(path, limit); sha = hashlib.sha256(payload).hexdigest()
    require(expected_sha is None or sha == expected_sha, "pinned evidence bytes differ: "+str(path))
    require(str(path) not in pins or pins[str(path)] == sha, "evidence changed after its first read: "+str(path))
    pins[str(path)] = sha
    return payload


def pin_json(path, pins, *, expected_sha=None):
    return json.loads(pin_payload(path, pins, expected_sha=expected_sha, limit=32*1024**2),
        object_pairs_hook=prior.unique_object,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite evidence: "+value)))


def recheck_pins(pins):
    # HH_261001 - The manifest describes precisely the bytes used, never later replacement evidence.
    require(all(digest(canonical_path(path, exists=True)) == sha for path, sha in pins.items()),
            "pinned evidence/source/parent/export changed before first report write")


def learning_rate(update):
    require(type(update) is int and 1 <= update <= 3000, "fixed branch update required")
    if update <= 100: return 3e-4 * update / 100
    return 3e-5 + .5 * (3e-4 - 3e-5) * (1 + math.cos(math.pi * (update - 100) / 2900))


def verify_moving_scope(obj):
    # HH_261001 - Rolling-only applies to the NEW recovery cohort; older replay remains honestly unchanged.
    flags = dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, stop_intent_trained=False,
        labels_clipped=False, labels_extrapolated=False, backbone_frozen=True, independent_validation=False,
        legacy_decoded=False, checkpoint_selection_by_holdout=False, parent_all_diagnostic_screens_passed=False,
        new_signed_recovery_cohort_moving_only=True, startup_recovery_fitted=False,
        raw_startup_retained_outside_new_signed_recovery_training=True, startup_or_route_goal_capability_claimed=False)
    require(all(obj.get(k) is value for k, value in flags.items())
            and obj.get("runtime_intent") == "explicit_cruise_GO_not_head"
            and obj.get("recovery_data_revision") == "prospective_actual_signed_recovery_moving_v4"
            and obj.get("startup_recovery_scope") == "NEW signed-recovery cohort only; previous center/cruise and old TRAIN replay unchanged",
            "truthful V4 rolling recovery scope cannot be promoted or rewritten")
    integers = dict(training_episode_count=18, recovery_training_episode_count=9, replay_training_episode_count=9,
        holdout_episode_count=2, parent_diagnostic_pass_count=7, fixed_recovery_anchors_per_episode=302)
    require(all(type(obj.get(k)) is int and obj[k] == value for k, value in integers.items()), "actual18TRAIN/2HOLDOUT fixed scope differs")
    require(obj.get("fixed_recovery_driving_relative_window_seconds") == [4.0, 34.1]
            and all(type(v) is float for v in obj["fixed_recovery_driving_relative_window_seconds"])
            and obj.get("fixed_recovery_native_frame_offsets") == [80, 682]
            and all(type(v) is int for v in obj["fixed_recovery_native_frame_offsets"])
            and type(obj.get("earliest_full_raw_history_driving_relative_seconds")) is float
            and obj["earliest_full_raw_history_driving_relative_seconds"] == 3.1
            and type(obj.get("true_future_seconds")) is float and obj["true_future_seconds"] == 6.4,
            "prospective4.0..34.1s rolling membership/history/future differs")


def verify_rate(rate):
    require(isinstance(rate, dict) and set(rate) == set(RATE)
            and all(type(rate[key]) is type(value) and rate[key] == value for key, value in RATE.items()),
            "new branch learning-rate schedule differs")


def verify_joint_report(report, denominator, split, episodes, per_episode_count, role):
    require(report.get("split") == split and report.get("independent_validation") is False
            and report.get("full_fixed_denominator") == denominator and report.get("role") == role,
            "joint evaluation full membership/split/role differs")
    rows = report["rows"]; identities(rows)
    require(len(rows) == denominator and {r["index"] for r in rows} == set(range(denominator)), "all fixed joint anchors required")
    for component, aggregate in (("ade_6p4s_m", "mean_ade_6p4s_m"), ("fde_6p4s_m", "mean_fde_6p4s_m"),
            ("instantaneous_speed_mae_mps", "mean_instantaneous_speed_mae_mps")):
        close(report[aggregate], mean(rows, component))
    close(report["maximum_ade_6p4s_m"], max(number(r["ade_6p4s_m"]) for r in rows))
    require(len(report["per_episode"]) == episodes and set(report["per_episode"]) == {r["episode_id"] for r in rows},
            "full joint episode denominator differs")
    for episode, group in report["per_episode"].items():
        selected = [r for r in rows if r["episode_id"] == episode]
        require(type(group.get("count")) is int and group["count"] == len(selected) == per_episode_count,
                "whole fixed per-episode joint count differs")
        for component in JOINT_COMPONENTS: close(group[component], mean(selected, component))
    for row in rows:
        for component in JOINT_COMPONENTS: number(row[component])
        for component in ("true_near_curvature_inv_m", "predicted_near_curvature_inv_m", "learned_near_branch_logit_mean"):
            signed_number(row[component])
    if denominator != 327:
        counts = report.get("actual_signed_current_state_counts")
        require(isinstance(counts, dict) and counts and all(type(n) is int and n > 0 for n in counts.values())
                and sum(counts.values()) == denominator, "full actual signed-current-state denominator differs")


def verify_raw_report(report, denominator, split, episodes):
    prior.verify_raw_report(report, denominator, split, episodes)
    require(report.get("speed_metric_notice") == "raw_preferred_speed is original SmoothL1 loss, NOT m/s; raw_spatial_preferred_speed_mae_mps is physical MAE. No legacy decoded MAE exists.",
            "raw SmoothL1 and physical speed MAE definitions must remain distinct")
    rows = report["rows"]
    for row in rows: number(row["raw_spatial_preferred_speed_mae_mps"])
    for name, group in report["groups"].items():
        selected = rows if name == "all_eligible" else [r for r in rows if r["stratum"] == name]
        close(group["raw_spatial_preferred_speed_mae_mps"], mean(selected, "raw_spatial_preferred_speed_mae_mps"))
    for episode, group in report["per_episode"].items():
        close(group["raw_spatial_preferred_speed_mae_mps"], mean([r for r in rows if r["episode_id"] == episode], "raw_spatial_preferred_speed_mae_mps"))


def endpoint_checks(before, after):
    # HH_261001 - Recompute every original48 screen at fixed3000; no rounding-based acceptance or best-step selection.
    checks = []
    def add(name, actual, threshold):
        actual, threshold = number(actual), number(threshold)
        checks.append(dict(name=name, actual=actual, threshold=threshold, passed=actual <= threshold))
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
    require(len(checks) == len({c["name"] for c in checks}) == 48, "all48 predeclared screens required")
    return checks


def verify_lineage(declaration, summary, metadata, declaration_sha):
    require(declaration.get("schema") == summary.get("schema") == RUN_SCHEMA
            and metadata.get("schema") == EXPORT_SCHEMA and metadata.get("architecture_schema") == ARCHITECTURE_SCHEMA
            and metadata.get("model_id") == MODEL_ID, "explicit V4 admission and V3 neural architecture schemas required")
    for obj in (declaration, summary, metadata): verify_moving_scope(obj)
    for obj in (declaration, metadata):
        verify_rate(obj.get("learning_rate"))
        require(obj.get("batch_domains") == BATCH_DOMAINS
                and obj.get("parent_checkpoint_sha256") == PARENT_CHECKPOINT_SHA
                and obj.get("parent_npz_sha256") == PARENT_NPZ_SHA
                and obj.get("parent_metadata_sha256") == PARENT_METADATA_SHA,
                "exact unpromoted mixed3000 parent and fixed sampler required")
    counts = dict(optimizer_updates=3000, examples_seen=12000, recovery_examples_seen=6000, v4_examples_seen=3000, legacy_examples_seen=3000)
    require(all(type(obj.get(k)) is int and obj[k] == value for obj in (summary, metadata) for k, value in counts.items()),
            "actual3000 updates/12000 domain exposures required")
    require(summary.get("status") == "FIXED3000_BRANCH_COMPLETE_NOT_PROMOTED" and summary.get("wall_budget_stop") is False
            and summary.get("backbone_97_tensors_byte_identical") is True and summary.get("source_hashes_match") is True,
            "completed fixed endpoint with unchanged97 tensors required")
    integers = dict(additional_optimizer_updates=3000, effective_batch_size=4, parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=3200, known_total_optimizer_updates=6200, maximum_optimizer_updates=3000)
    require(all(type(metadata.get(k)) is int and metadata[k] == value for k, value in integers.items())
            and declaration.get("requested_optimizer_updates") == declaration.get("maximum_optimizer_updates") == 3000
            and type(declaration.get("effective_batch_size")) is int and declaration["effective_batch_size"] == 4
            and type(declaration.get("seed")) is int and declaration["seed"] == 2026100104
            and metadata.get("declaration_sha256") == declaration_sha and declaration.get("holdout_fitted") is False
            and declaration.get("holdout_selected_checkpoint") is False,
            "fresh branch-only fixed endpoint/lineage/never-fit holdout declaration differs")
    require(metadata.get("trainable_state_names") == declaration.get("trainable_state_names") == sorted(BRANCH_SHAPES)
            and declaration.get("model_config") == metadata.get("model_config")
            and declaration.get("recovery_config") == metadata.get("recovery_config"), "exact neural config/trainable4 ABI differs")
    sources = declaration.get("source_sha256")
    require(isinstance(sources, dict) and set(sources) == SOURCE_NAMES and len(sources) == 24
            and all(is_sha(value) for value in sources.values())
            and all(sources.get(name) == sha for name, sha in FROZEN_SOURCES.items())
            and metadata.get("source_sha256") == sources, "reviewed original23 plus NEW V4 wrapper24 source closure differs")
    for field in ("training_dataset_fingerprint_sha256", "holdout_dataset_fingerprint_sha256", "backbone_state_sha256"):
        require(is_sha(metadata.get(field)) and metadata[field] == declaration.get(field), "exact declared data/backbone fingerprint differs")
    cohorts = declaration.get("cohorts", {})
    require(set(cohorts) == {"recovery_train", "recovery_holdout", "replay"}
            and cohorts["recovery_train"]["split"] == "train" and cohorts["recovery_train"]["sample_count"] == 2718
            and cohorts["recovery_holdout"]["split"] == "val" and cohorts["recovery_holdout"]["sample_count"] == 604
            and cohorts["replay"]["v4"]["statistics"]["anchor_count"] == 327
            and cohorts["replay"]["legacy"]["sample_count"] == 2787
            and cohorts["replay"]["legacy"]["split"] == "train"
            and cohorts["replay"]["known_dev"]["sample_count"] == 665
            and cohorts["replay"]["known_dev"]["split"] == "val", "actual whole cohort/replay counts differ")
    require(declaration.get("actual_dataset_available_train_anchor_count") == 5832
            and declaration.get("actual_eligible_train_pool_anchor_count") == 5532
            and declaration.get("maximum_fit_including_baseline_seconds") == 2700
            and declaration.get("postfit_checks_separately_timed") is True, "declared membership/time bound differs")
    checkpoints = summary["checkpoints"]
    require([r["optimizer_updates"] for r in checkpoints] == list(range(500, 3001, 500))
            and all(r.get("optimizer_state_retained") is True and all(is_sha(r.get(k)) for k in
                ("sha256", "export_npz_sha256", "export_metadata_sha256")) for r in checkpoints),
            "all six fixed checkpoints and optimizer receipts must remain retained")
    final = checkpoints[-1]
    require(final["sha256"] == metadata.get("checkpoint_sha256") and final["export_npz_sha256"] == metadata.get("npz_sha256"),
            "final3000 checkpoint/NPZ metadata linkage differs")
    return final


def verify_history(history, declaration, before):
    require(len(history) == 3000, "all actual3000 optimizer journal rows required")
    recovery = {r["index"]: r for r in before["recovery_train"]["rows"]}
    low = {r["index"]: r for r in before["v4"]["rows"]}
    legacy = {r["index"]: r for r in before["legacy"]["rows"]}
    forbidden = {r["token"] for key in ("recovery_holdout", "known599") for r in before[key]["rows"]}
    schedule, domain_exposure = [], {k: Counter() for k in ("recovery", "v4", "legacy")}
    actual_full_cycle_episodes, actual_legacy_geometry, actual_legacy_maps = Counter(), Counter(), {k: Counter() for k in ("curved", "straight")}
    duplicate_recovery = 0
    for step, row in enumerate(history, 1):
        require(type(row.get("optimizer_updates")) is int and row["optimizer_updates"] == step
                and type(row.get("examples_seen")) is int and row["examples_seen"] == step * 4,
                "actual optimizer ordering/exposure count differs")
        indices, tokens = row["indices"], row["sample_ids"]
        require(len(indices) == len(tokens) == 4 and all(type(i) is int for i in indices), "actual effective B4 journal required")
        wanted = []
        for position in (0, 1):
            require(indices[position] in recovery, "recovery slot escaped all2718 TRAIN anchors")
            wanted.append(recovery[indices[position]]["token"]); domain_exposure["recovery"].update([wanted[-1]])
        require(indices[2] - 2718 in low and indices[3] - 3045 in legacy, "V4 or eligible legacy source index differs")
        wanted.extend((low[indices[2]-2718]["token"], legacy[indices[3]-3045]["token"]))
        require(tokens == wanted and not set(tokens) & forbidden, "actual fitted token is foreign/VAL/HOLDOUT/TEST")
        domain_exposure["v4"].update([tokens[2]]); domain_exposure["legacy"].update([tokens[3]])
        actual_full_cycle_episodes.update([recovery[indices[1]]["episode_id"]])
        kind = "curved" if step % 2 == 1 else "straight"
        old = legacy[indices[3]-3045]
        require(old["stratum"] == kind, "actual old replay must alternate true curve/straight")
        actual_legacy_geometry.update([kind]); actual_legacy_maps[kind].update([old["map"]])
        duplicate_recovery += indices[0] == indices[1]
        for domain in BATCH_DOMAINS[:2] + ["v4", "legacy_raw_curvature"]:
            key = {"recovery_noncentered_joint": "recovery0", "recovery_all_joint": "recovery1"}.get(domain, domain)
            gradient = row["per_domain_branch_gradients"][key]
            require(gradient.get("all_present_branch_gradients_finite") is True, "nonfinite declared branch-domain gradients")
            number(gradient["gradient_l2_norm"])
            require(type(gradient.get("unused_branch_tensors")) is int and 0 <= gradient["unused_branch_tensors"] < 4,
                    "missing entire branch-gradient contribution")
        expected_loss = .25 * number(row["recovery0"]["total"]) + .25 * number(row["recovery1"]["total"])
        expected_loss += .25 * number(row["v4"]["total"]) + .0625 * number(row["legacy_raw_curvature"])
        close(row["loss"], expected_loss)
        require(math.isclose(number(row["learning_rate"]), learning_rate(step), rel_tol=1e-12, abs_tol=1e-15),
                "actual fixed learning rate differs")
        number(row["preclip_branch_gradient_norm"]); number(row["elapsed_owned_seconds"])
        schedule.append(indices)
    schedule_sha = hashlib.sha256(json.dumps(schedule, separators=(",", ":")).encode()).hexdigest()
    require(schedule_sha == declaration["fixed_schedule_sha256"], "actual fixed sampler order differs")
    declared_unique = declaration["declared_unique_source_exposure_counts"]
    require(all(len(domain_exposure[k]) == declared_unique[k] for k in domain_exposure)
            and len(domain_exposure["recovery"]) == 2718 and len(domain_exposure["v4"]) == 327,
            "full9 recovery or full327 replay anchors omitted; declared legacy unique count differs")
    require(duplicate_recovery == declaration["declared_focus_full_same_anchor_batch_count"], "actual focus/full collision count differs")
    require(len(actual_full_cycle_episodes) == 9 and sorted(actual_full_cycle_episodes.values()) == [333]*6 + [334]*3,
            "all9 full-profile roundrobin exposure differs")
    require(actual_legacy_geometry == Counter(curved=1500, straight=1500)
            and sorted(actual_legacy_maps["curved"].values()) == [750, 750]
            and sorted(actual_legacy_maps["straight"].values()) == [500, 500, 500], "protected geometry/map-balanced replay differs")
    require(sum(declaration["actual_profile_full302_counts"].values()) == 2718
            and len(declaration["actual_profile_full302_counts"]) == 9
            and set(declaration["actual_profile_full302_counts"].values()) == {302}
            and declaration["actual_noncentered_grid_counts"]
            and all(type(count) is int and count > 0 for count in declaration["actual_noncentered_grid_counts"].values()),
            "prospective full profiles/genuine actual noncentered strata required")
    return dict(actual_update_rows=3000, actual_anchor_exposures=12000, recovery_exposures=6000, v4_exposures=3000,
        legacy_exposures=3000, legacy_curved_exposures=1500, legacy_straight_exposures=1500,
        actual_unique_source_anchors={k: len(v) for k, v in domain_exposure.items()},
        actual_full_cycle_per_episode=dict(actual_full_cycle_episodes), actual_legacy_map_exposures={k: dict(v) for k, v in actual_legacy_maps.items()},
        recorded_schedule_sha256=schedule_sha, focus_full_same_anchor_batches=duplicate_recovery,
        legacy_equivalent_uniform_epochs_claimed=False, HOLDOUT_known_VAL_or_TEST_fitted=False)


def verify_response(response, before, after):
    require(response.get("research_diagnostic_only") is True and response.get("no_inference_oracle") is True
            and response.get("feature_approved") is False, "signed branch response is not inference or functional approval")
    original, final = ({r["token"]: r for r in report["rows"]} for report in (before, after))
    seen = set()
    require(set(response["groups"]) == set(after["actual_signed_current_state_counts"]), "signed response strata differ")
    for key, group in response["groups"].items():
        rows = group["rows"]
        require(type(group.get("all_anchor_count")) is int and group["all_anchor_count"] == len(rows)
                == after["actual_signed_current_state_counts"][key], "signed response complete strata denominator differs")
        for row in rows:
            token = row["token"]
            require(token in original and token not in seen, "foreign/duplicate signed response token")
            seen.add(token)
            wanted = final[token]["true_near_curvature_inv_m"] - original[token]["predicted_near_curvature_inv_m"]
            change = final[token]["predicted_near_curvature_inv_m"] - original[token]["predicted_near_curvature_inv_m"]
            require(math.isclose(signed_number(row["expert_minus_parent_near_curvature"]), wanted, rel_tol=1e-6, abs_tol=1e-7)
                    and math.isclose(signed_number(row["actual_learned_near_curvature_change"]), change, rel_tol=1e-6, abs_tol=1e-7)
                    and row.get("signed_response_eligible") is (abs(wanted) >= .001)
                    and row.get("signed_response_agrees") is (wanted * change > 0)
                    and row["learned_branch_logit"] == final[token]["learned_near_branch_logit_mean"], "signed response changed expert/model diagnostics")
        require(group.get("signed_response_denominator") == sum(r["signed_response_eligible"] for r in rows)
                and group.get("signed_response_agree_count") == sum(r["signed_response_eligible"] and r["signed_response_agrees"] for r in rows),
                "signed response retained denominator/count differs")
    require(seen == set(original) == set(final), "signed response omitted fixed anchors")


def raw_comparisons(before, after, scope):
    result = []
    for label in ("aggregate", *sorted(before["per_episode"]), "curved", "straight"):
        old = before["groups"]["all_eligible"] if label == "aggregate" else before["groups"][label] if label in ("curved", "straight") else before["per_episode"][label]
        new = after["groups"]["all_eligible"] if label == "aggregate" else after["groups"][label] if label in ("curved", "straight") else after["per_episode"][label]
        for component in RAW_COMPONENTS:
            result.append(dict(scope=scope, group=label, component=component, count=old["count"], parent=old[component],
                final=new[component], delta=new[component]-old[component]))
    return result


def build_report(documents, history, declaration_sha):
    declaration, summary, metadata = (documents[k] for k in ("declaration.json", "summary.json", "exports/step_003000/metadata.json"))
    final = verify_lineage(declaration, summary, metadata, declaration_sha)
    started = documents["started.json"]; gpu = started["gpu"]
    require(type(gpu.get("physical_index")) is int and gpu["physical_index"] == 0
            and gpu.get("uuid") == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5" and gpu.get("idle_before_torch") is True
            and started.get("frozen_backbone_sha256") == metadata["backbone_state_sha256"]
            and type(started.get("trainable_branch_parameter_count")) is int and started["trainable_branch_parameter_count"] == 1409,
            "actual owned GPU0/frozen97/learned1409-parameter branch receipt differs")
    before, after = ({key: documents[key+"_"+stage+".json"] for key in COHORTS} for stage in ("before", "final"))
    for key, count, split, episodes, per_count, role in (
            ("recovery_train", 2718, "train", 9, 302, "TRAIN_resubstitution"),
            ("recovery_holdout", 604, "val", 2, 302, "never_fit_actual_signed_recovery_HOLDOUT"),
            ("v4", 327, "train", 1, 327, "TRAIN_resubstitution")):
        prior.verify_pair(before[key], after[key], count)
        for report in (before[key], after[key]): verify_joint_report(report, count, split, episodes, per_count, role)
        require(all(row["learned_near_branch_logit_mean"] == 0. for row in before[key]["rows"]),
                "baseline must preserve exact original zero-output branch")
    for key, count, split, episodes in (("known599", 599, "val", 2), ("legacy", 2487, "train", 8)):
        prior.verify_pair(before[key], after[key], count)
        for report in (before[key], after[key]): verify_raw_report(report, count, split, episodes)
    require(all(r.get("available_known_val_count") == 665 and r.get("excluded_count") == 66
                and r.get("role") == "previously_exposed_known_moving_DEV599_raw_diagnostic" for r in (before["known599"], after["known599"])),
            "known599 is previously exposed diagnostic, not all665 or independent DEV")
    token_sets = [{r["token"] for r in before[key]["rows"]} for key in COHORTS]
    require(all(not a & b for i, a in enumerate(token_sets) for b in token_sets[i+1:]), "cross-cohort source token leakage")
    require({r["map"] for r in before["legacy"]["rows"]} == {"C_track_1_0_7", "Town01", "Town07"}, "actual old TRAIN map coverage differs")
    checks = endpoint_checks(before, after); endpoint = documents["endpoint_diagnostic_screens.json"]
    require(endpoint.get("fixed_endpoint_updates") == 3000 and endpoint.get("diagnostic_screen_count") == 48
            and endpoint.get("checks") == checks and all(type(c.get("passed")) is bool for c in endpoint["checks"])
            and endpoint.get("all_diagnostic_screens_passed") is all(c["passed"] for c in checks)
            and all(endpoint.get(k) is False for k in ("independent_validation", "feature_approved", "vehicle_control_approved",
                "model_promoted", "absolute_speed_tracking_claimed")) and endpoint.get("frozen_parent_7of10_not_relabelled") is True
            and summary.get("endpoint_diagnostic_screens") == endpoint, "original48-gate endpoint/summary must remain unchanged")
    journal = verify_history(history, declaration, before)
    for key, suffix in (("recovery_train", "train"), ("recovery_holdout", "holdout")):
        verify_response(documents["actual_signed_branch_response_"+suffix+".json"], before[key], after[key])
    joint_rows = []
    for key in ("recovery_train", "recovery_holdout", "v4"):
        for episode, group in after[key]["per_episode"].items():
            joint_rows.append(dict(cohort=key, episode_id=episode, count=group["count"],
                metrics={component: dict(parent=before[key]["per_episode"][episode][component], final=group[component],
                    delta=group[component]-before[key]["per_episode"][episode][component]) for component in JOINT_COMPONENTS}))
    raw_rows = raw_comparisons(before["known599"], after["known599"], "previously_exposed_known_DEV599_raw_not_independent")
    raw_rows += raw_comparisons(before["legacy"], after["legacy"], "legacy_TRAIN2487_resubstitution_raw_only")
    return dict(schema=SCHEMA, fixed_endpoint_updates=3000, official_function_approvals=0, official_function_total=30,
        official_main_function_approvals=0, official_main_function_total=9,
        official_subfunction_approvals=0, official_subfunction_total=30,
        model_promoted=False, vehicle_control_approved=False, simulator_control_authorized=False, independent_validation=False,
        main_training=False, checkpoint_selection_by_holdout=False, startup_recovery_fitted=False,
        startup_or_route_goal_capability_claimed=False, end_to_end_10Hz_validated=False, absolute_speed_tracking_claimed=False,
        physical_3x45_trials_included=False, future_owned_physical_trials_are_separate=True,
        diagnostic_pass_count=sum(c["passed"] for c in checks), diagnostic_gate_count=48,
        all_diagnostic_screens_passed=all(c["passed"] for c in checks), checks=checks,
        lineage=dict(parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA, parent_npz_sha256=PARENT_NPZ_SHA,
            parent_metadata_sha256=PARENT_METADATA_SHA, parent_diagnostic_pass_count=7, parent_diagnostic_gate_count=10,
            parent_known_updates=3200, additional_branch_updates=3000, known_total_updates=6200,
            final_checkpoint_sha256=metadata["checkpoint_sha256"], final_npz_sha256=metadata["npz_sha256"],
            final_metadata_sha256=final["export_metadata_sha256"], declaration_sha256=declaration_sha,
            frozen_backbone_sha256=metadata["backbone_state_sha256"], frozen_backbone_tensor_count=97,
            learned_branch_tensor_count=4, final_tensor_count=101, frozen_training_sources=declaration["source_sha256"]),
        data=dict(TRAIN_episode_count=18, recovery_TRAIN_episode_count=9, recovery_TRAIN_anchors=2718,
            old_V4_TRAIN_anchors=327, eligible_legacy_TRAIN_anchors=2487, excluded_legacy_TRAIN_anchors=300,
            available_TRAIN_anchors=5832, eligible_TRAIN_pool_anchors=5532, HOLDOUT_episode_count=2, HOLDOUT_anchors=604,
            HOLDOUT_fitted=False, HOLDOUT_checkpoint_selected=False, known_DEV_available=665, known_DEV_evaluated=599,
            known_DEV_excluded=66, known_DEV_is_independent=False, actual_recovery_HOLDOUT_never_fitted=True,
            HOLDOUT_scope="same fixed Town07 route, distinct actual offset/yaw/seeds; not unseen-map generalization",
            new_recovery_driving_window_seconds=[4.0, 34.1], raw_history_begins_seconds=3.1, true_future_seconds=6.4,
            startup_in_new_recovery_TRAIN=False, raw_startup_retained_as_context_only=True),
        actual_journal=journal, joint_comparisons=joint_rows, raw_comparisons=raw_rows,
        raw_metric_definitions=dict(raw_curvature="original masked SmoothL1 loss, not inverse-meter MAE",
            raw_preferred_speed="original masked SmoothL1 loss, not m/s",
            raw_spatial_preferred_speed_mae_mps="physical masked raw-speed MAE in m/s; no legacy decoding",
            joint_ade="mean XY error of full64 actual future positions over6.4s in m",
            joint_fde="XY error at actual6.4s final position in m",
            joint_instantaneous_speed_mae_mps="instantaneous decoded speed vs actual future speed, not chord-average target"),
        learned_branch_signed_response={key: documents["actual_signed_branch_response_"+suffix+".json"]
            for key, suffix in (("TRAIN", "train"), ("HOLDOUT", "holdout"))},
        learning_rate=declaration["learning_rate"], actual_gpu_preflight=gpu,
        timing={key: number(summary[key]) for key in ("fit_including_baseline_seconds", "postfit_endpoint_seconds", "elapsed_owned_seconds")})


def bounded_npz(path, sha, shapes, dtypes, *, pins=None):
    # HH_261001 - Every tensor is checked with allow_pickle=False; PT checkpoints are hashed only, never deserialized.
    import numpy as np
    payload = pin_payload(path, {} if pins is None else pins, expected_sha=sha, limit=32*1024**2)
    require(isinstance(shapes, dict) and isinstance(dtypes, dict) and set(shapes) == set(dtypes), "full numeric shape/dtype keys required")
    with zipfile.ZipFile(BytesIO(payload)) as archive:
        entries = archive.infolist()
        require(len(entries) == len(shapes) and len({e.filename for e in entries}) == len(entries)
                and {e.filename for e in entries} == {key+".npy" for key in shapes}
                and all(not e.is_dir() and not(e.flag_bits & 1) for e in entries)
                and sum(e.file_size for e in entries) <= 32*1024**2, "bounded full unique numeric archive required")
    result = {}
    with np.load(BytesIO(payload), allow_pickle=False) as arrays:
        require(set(arrays.files) == set(shapes), "full numeric state keys differ")
        for name in shapes:
            array = arrays[name]
            require(array.dtype == np.float32 and np.isfinite(array).all() and isinstance(shapes[name], list)
                    and all(type(v) is int and v > 0 for v in shapes[name]) and list(array.shape) == shapes[name]
                    and dtypes[name] == "float32", "finite float32 full tensor ABI differs: "+name)
            result[name] = array.copy()
    return result


def backbone_fingerprint(arrays):
    result = hashlib.sha256()
    for name in sorted(arrays):
        value = arrays[name]
        result.update(name.encode()); result.update(str(tuple(value.shape)).encode()); result.update(value.tobytes(order="C"))
    return result.hexdigest()


def verify_evidence_files(run, root, documents, *, summary_sha256, declaration_sha256, checkpoint_sha256,
        weights_sha256, metadata_sha256, parent_weights, parent_metadata, evidence_pins=None):
    # HH_261001 - Caller pins are external to the run, rather than trusting only self-reported final hashes.
    expected = {"summary.json": summary_sha256, "declaration.json": declaration_sha256,
        "checkpoints/step_003000.pt": checkpoint_sha256, "exports/step_003000/model_state.npz": weights_sha256,
        "exports/step_003000/metadata.json": metadata_sha256}
    pins = {} if evidence_pins is None else evidence_pins
    require(all(is_sha(value) for value in expected.values()), "external SHA pins required")
    for name, sha in expected.items(): pin_payload(run/name, pins, expected_sha=sha)
    metadata, declaration, summary = (documents[key] for key in ("exports/step_003000/metadata.json", "declaration.json", "summary.json"))
    final = summary["checkpoints"][-1]
    require(final["sha256"] == checkpoint_sha256 and final["export_npz_sha256"] == weights_sha256
            and final["export_metadata_sha256"] == metadata_sha256, "fixed final3000 external pins differ from retained receipts")
    pin_payload(root/PRIOR_FILE, pins, expected_sha=PRIOR_SHA)
    for name, sha in declaration["source_sha256"].items():
        require(name in SOURCE_NAMES, "unexpected imported source")
        pin_payload(root/name, pins, expected_sha=sha); pin_payload(run/"source"/name, pins, expected_sha=sha)
    parent = pin_json(parent_metadata, pins, expected_sha=PARENT_METADATA_SHA)
    require(parent.get("schema") == prior.EXPORT_SCHEMA and parent.get("checkpoint_sha256") == PARENT_CHECKPOINT_SHA
            and parent.get("declaration_sha256") == PARENT_DECLARATION_SHA and parent.get("known_total_optimizer_updates") == 3200
            and parent.get("optimizer_updates") == 3000 and parent.get("model_promoted") is False
            and parent.get("vehicle_control_approved") is False and parent.get("model_config") == metadata["model_config"],
            "unaltered parent ABI/failed-screen research lineage required")
    parent_arrays = bounded_npz(parent_weights, PARENT_NPZ_SHA, parent["state_tensor_shapes"], parent["state_tensor_dtypes"], pins=pins)
    require(len(parent_arrays) == 97 and not set(parent_arrays) & set(BRANCH_SHAPES)
            and backbone_fingerprint(parent_arrays) == metadata["backbone_state_sha256"], "actual97 parent tensor fingerprint differs")
    checkpoint_evidence = []
    for receipt in summary["checkpoints"]:
        step = receipt["optimizer_updates"]; folder = run/"exports"/("step_%06d" % step)
        checkpoint = run/"checkpoints"/("step_%06d.pt" % step)
        pin_payload(checkpoint, pins, expected_sha=receipt["sha256"])
        middle = pin_json(folder/"metadata.json", pins, expected_sha=receipt["export_metadata_sha256"])
        verify_moving_scope(middle); verify_rate(middle.get("learning_rate"))
        require(middle.get("schema") == EXPORT_SCHEMA and middle.get("source_sha256") == declaration["source_sha256"]
                and middle.get("declaration_sha256") == declaration_sha256 and middle.get("checkpoint_sha256") == receipt["sha256"]
                and middle.get("npz_sha256") == receipt["export_npz_sha256"]
                and middle.get("parent_checkpoint_sha256") == PARENT_CHECKPOINT_SHA
                and middle.get("backbone_state_sha256") == metadata["backbone_state_sha256"]
                and all(type(middle.get(k)) is int and middle[k] == value for k, value in dict(optimizer_updates=step,
                    additional_optimizer_updates=step, examples_seen=4*step, recovery_examples_seen=2*step,
                    v4_examples_seen=step, legacy_examples_seen=step, known_total_optimizer_updates=3200+step).items()),
                "retained fixed intermediate export lineage differs")
        arrays = bounded_npz(folder/"model_state.npz", receipt["export_npz_sha256"], middle["state_tensor_shapes"], middle["state_tensor_dtypes"], pins=pins)
        require(set(arrays) == set(parent_arrays) | set(BRANCH_SHAPES) and len(arrays) == 101
                and all(list(arrays[key].shape) == shape for key, shape in BRANCH_SHAPES.items())
                and all(arrays[key].shape == value.shape and arrays[key].tobytes(order="C") == value.tobytes(order="C")
                    for key, value in parent_arrays.items()), "one of actual97 parent tensors changed or101 branch ABI differs")
        checkpoint_evidence.append(dict(step=step, checkpoint_sha256=receipt["sha256"], npz_sha256=receipt["export_npz_sha256"],
            metadata_sha256=receipt["export_metadata_sha256"], tensor_count=101, frozen97_byte_identical=True,
            all_float32_finite=True, Torch_pickle_loaded=False))
    return dict(parent_npz_sha256=PARENT_NPZ_SHA, parent_metadata_sha256=PARENT_METADATA_SHA,
        original_backbone_tensor_count=97, learned_branch_tensor_count=4, fixed_exports=checkpoint_evidence,
        final_tensor_count=101, all_six_checkpoints_and_optimizer_receipts_retained=True,
        all_six_exports_finite_and_parent97_byte_identical=True, no_Torch_pickle=True)


def load_numeric_model(run, metadata, metadata_sha, owner_id, root, parent_weights, parent_metadata):
    # HH_261001 - Reviewed model construction is optional and must wait for root's released local CPU window.
    import importlib.util
    require(digest(root/LOADER_FILE) == LOADER_SHA, "reviewed recovery numeric loader changed")
    spec = importlib.util.spec_from_file_location("hh_recovery_publish_workspace", root/"tools/workspace_python.py")
    workspace = importlib.util.module_from_spec(spec); spec.loader.exec_module(workspace); workspace.configure(root)
    from portable_e2e.temporal_route_recovery_export_loader_research_20261001 import OWNER_SCHEMA, load_route_recovery_research_npz
    binding = dict(schema=OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=owner_id,
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False, vehicle_control_approved=False,
        model_promoted=False, weights_sha256=metadata["npz_sha256"], metadata_sha256=metadata_sha,
        checkpoint_sha256=metadata["checkpoint_sha256"], declaration_sha256=metadata["declaration_sha256"], fixed_endpoint_updates=3000)
    return load_route_recovery_research_npz(run/"exports/step_003000/model_state.npz", run/"exports/step_003000/metadata.json",
        weights_sha256=metadata["npz_sha256"], metadata_sha256=metadata_sha, parent_weights=parent_weights,
        parent_metadata=parent_metadata, research_owner_binding=binding, source_root=root)


def numeric_receipt(run, metadata, metadata_sha, owner_id, root, parent_weights, parent_metadata, enabled, archive_receipt):
    base = dict(schema=NUMERIC_SCHEMA, fixed_endpoint_updates=3000, checkpoint_sha256=metadata["checkpoint_sha256"],
        weights_sha256=metadata["npz_sha256"], metadata_sha256=metadata_sha, declaration_sha256=metadata["declaration_sha256"],
        loader_source=LOADER_FILE, loader_source_sha256=LOADER_SHA, numeric_validation_only=True, no_Torch_pickle=True,
        simulator_control_authorized=False, vehicle_control_approved=False, model_promoted=False,
        inference_loader_authorized=False, end_to_end_10Hz_validated=False, independent_validation=False,
        numpy_archive_validation=archive_receipt)
    if not enabled: return dict(base, status="PASS_ARCHIVES_ONLY_REVIEWED_MODEL_LOADER_NOT_RUN", reviewed_model_loader_run=False)
    require(isinstance(owner_id, str) and owner_id.strip(), "explicit CPU-only numeric loader owner required")
    model = load_numeric_model(run, metadata, metadata_sha, owner_id, root, parent_weights, parent_metadata)
    require(model._research_temporal_export_metadata == metadata
            and model._research_recovery_simulator_control_authorized is False
            and model._research_temporal_checkpoint_sha256 == metadata["checkpoint_sha256"]
            and len(model.state_dict()) == 101 and set(model.state_dict()) == set(metadata["state_tensor_shapes"])
            and not model.training and all(not p.requires_grad for p in model.parameters()), "reviewed numeric model loader altered endpoint/state/control scope")
    return dict(base, status="PASS_FULL101_NUMERIC_CPU_ONLY_NOT_CONTROL", reviewed_model_loader_run=True,
        owner_id=owner_id, loader_schema=model._research_recovery_numeric_loader_schema,
        original_export_metadata_unmodified=True, tensor_count=101, frozen97_byte_identical=True)


def comparison_markdown(report):
    lines = ["# 전후 전체 비교: 같은 고정 자료에서 평가", "", "새 TRAIN은 재평가, 새 HOLDOUT은 미학습 같은 Town07 경로입니다. 기존 known DEV는 이미 사용한 진단 자료입니다.", "",
        "## 새 회복 9개 TRAIN / 2개 HOLDOUT / 기존 저속 327개", "",
        "| 자료 | 기록 | 분모 | ADE 부모→최종 (m) | FDE 부모→최종 (m) | 순간 속도 MAE 부모→최종 (m/s) | raw 속도 MAE 부모→최종 (m/s) |",
        "|---|---|---:|---:|---:|---:|---:|"]
    for row in report["joint_comparisons"]:
        values = ["%.6f → %.6f" % (row["metrics"][key]["parent"], row["metrics"][key]["final"]) for key in JOINT_COMPONENTS]
        lines.append("| %s | %s | %d | %s |" % (row["cohort"], row["episode_id"], row["count"], " | ".join(values)))
    lines += ["", "## 기존 TRAIN 2487개 / known DEV 599개 — 모든 기록과 직진·회전", "",
        "`raw_curvature`와 `raw_preferred_speed`는 원래 SmoothL1 손실입니다. m/s 단위인 값은 `raw_spatial_preferred_speed_mae_mps`뿐입니다. 기존 고속 자료는 decoder를 거치지 않았습니다.", "",
        "| 자료 | 그룹/기록 | 분모 | 성분 | 부모 | 최종 | 변화 (최종−부모) |", "|---|---|---:|---|---:|---:|---:|"]
    for row in report["raw_comparisons"]:
        lines.append("| %s | %s | %d | %s | %.9f | %.9f | %+.9f |" % (row["scope"], row["group"], row["count"],
            row["component"], row["parent"], row["final"], row["delta"]))
    return "\n".join(lines)+"\n"


def markdown(report, numeric, run):
    passed, total = report["diagnostic_pass_count"], report["diagnostic_gate_count"]
    lines = ["# 차선 복귀 학습 결과 — 신규 V4 자료 / V3 주행 모델", "",
        "사전에 정한 48개 진단 중 **%d개 통과 / %d개 미달**입니다. 미달 결과도 모두 아래에 남겼습니다. 최종 3000회 가중치만 비교했으며 중간 체크포인트를 골라내지 않았습니다." % (passed, total-passed), "",
        "정식 주기능 승인 **0/9**, 하위 기능 승인 **0/30**. 이 보고서는 모델 학습·숫자 검증 결과이며 자율주행 성공이나 실차 사용 승인이 아닙니다. 실제 3×45초 폐루프 주행은 별도 실행과 결과로 확인해야 합니다.", "",
        "## 이번에 바꾼 부분", "",
        "기존 주행 모델의 97개 tensor는 원본과 바이트까지 동일하게 동결했습니다. 카메라와 실제 현재 위치 기준의 경로를 사용하는 작은 차선 복귀 branch 4개 tensor만 학습했습니다. "
        "새 기능 전체를 완성한 것이 아니라, 경로에서 벗어났을 때 원래 경로 쪽으로 조향할 수 있도록 경로의 좌우 정보 전달을 보강한 단계입니다.", "",
        "- 부모는 mixed3000이며 사전 진단 7/10 통과 상태였습니다. 그 상태를 성공 모델로 바꾸어 기록하지 않았습니다.",
        "- 새 branch 3000회 추가 → 알려진 누적 업데이트 6200회. 본격 전체 기능 학습 완료가 아닙니다.",
        "- 18개 TRAIN 기록: 새 실제 회복 9개 + 기존 저속 1개 + 기존 직진·회전 8개. 새 HOLDOUT 2개는 학습·체크포인트 선택에 사용하지 않았습니다.",
        "- 실제 12000회 노출: 회복 6000, 기존 저속 3000, 기존 직진·회전 3000. 배치 4개는 실제 비중앙 상태 1 + 전체 회복 자료 1 + 기존 저속 1 + 기존 회전/직진 번갈아 1입니다.",
        "- 새 회복 자료는 주행 상대시간 **4.0~34.1초**의 302개×9 기록입니다. 초기 정지·출발 회복은 이번 새 자료의 학습 범위가 아닙니다. 원시 startup은 지우지 않고 보존했습니다.",
        "- 신규 HOLDOUT은 같은 Town07 경로의 다른 실제 위치·각도·seed입니다. 다른 모든 맵이나 상황에 대한 일반화 증거가 아닙니다.",
        "- 기존 known DEV 665개 중 고정된 이동 599개를 진단했습니다. 이전부터 사용한 자료이므로 독립 검증으로 부르지 않습니다.",
        "- 속도·STOP head와 원래 backbone은 동결되어 있습니다. 속도 지표도 그대로 보고하며 절대 속도 추종 성공을 주장하지 않습니다. 지도 모델은 이번 학습에 포함하지 않았습니다.", "",
        "## 사전에 고정한 48개 판정 — 실패 포함 전부", "",
        "| 기준 | 최종 실제 값 | 허용 상한 | 결과 |", "|---|---:|---:|---|"]
    for check in report["checks"]:
        lines.append("| %s | %.9f | %.9f | %s |" % (check["name"], check["actual"], check["threshold"], "PASS" if check["passed"] else "FAIL"))
    timing = report["timing"]
    lines += ["", "## 파일과 해석", "",
        "[전체 전후 비교](comparisons.md)에 새 TRAIN 9개·HOLDOUT 2개·기존 저속 327개와 기존 raw 평가 전체·각 기록·직진/회전을 공개했습니다.", "",
        "[결과·계보 JSON](result.json), [원자료 SHA](evidence_manifest.json), [숫자 전용 검증](numeric_loader_receipt.json).", "",
        "숫자 검사: `%s`. 모든 6개 export의 101개 tensor와 원본97 바이트를 검산했습니다. Torch `.pt`는 SHA만 확인하고 역직렬화하지 않았습니다. "
        "숫자 검사 PASS는 simulator/실차 제어 승인이나 전체 10Hz 성공을 뜻하지 않습니다." % numeric["status"], "",
        "실제 GPU 소유 %.1f초: baseline 포함 fit %.1f초, 최종 평가 %.1f초. fit 예산은 2700초이며 최종 평가 시간은 별도입니다."
        % (timing["elapsed_owned_seconds"], timing["fit_including_baseline_seconds"], timing["postfit_endpoint_seconds"]), "",
        "다음 확인은 최종 고정 가중치의 실제 폐루프 차선 복귀·90m/45초·반복 재현성입니다. 이 발행자는 추가 학습이나 물리 주행을 시작하지 않습니다.", "",
        "원자료: `%s`" % run, ""]
    return "\n".join(lines)


def publish(run, output, *, summary_sha256, declaration_sha256, checkpoint_sha256, weights_sha256, metadata_sha256,
        parent_weights, parent_metadata, numeric_validation=False, owner_id=None, root=ROOT):
    run, output, root = canonical_path(run, exists=True), canonical_path(output), canonical_path(root, exists=True)
    require(run.is_dir() and not run.is_symlink() and root.is_dir() and not root.is_symlink(), "regular run/source directories required")
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir() and not output.parent.is_symlink(),
            "fresh create-only report directory with existing regular parent required")
    parent_weights, parent_metadata = canonical_path(parent_weights, exists=True), canonical_path(parent_metadata, exists=True)
    require(not output.is_relative_to(run) and not any(output.is_relative_to(asset.parent) for asset in (parent_weights, parent_metadata)),
            "report must remain outside immutable run and exact parent export directories")
    require(shutil.disk_usage(output.parent).free >= 5*1024**3+512*1024**2, "report mount must retain5GiB+512MiB reserve; no deletion")
    pins = {}
    documents = {name: pin_json(run/name, pins) for name in DOCUMENTS}
    history = [json.loads(line, object_pairs_hook=prior.unique_object,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite journal: "+value)))
        for line in pin_payload(run/"history.jsonl", pins, limit=32*1024**2).splitlines()]
    report = build_report(documents, history, declaration_sha256)
    archives = verify_evidence_files(run, root, documents, summary_sha256=summary_sha256, declaration_sha256=declaration_sha256,
        checkpoint_sha256=checkpoint_sha256, weights_sha256=weights_sha256, metadata_sha256=metadata_sha256,
        parent_weights=parent_weights, parent_metadata=parent_metadata, evidence_pins=pins)
    numeric = numeric_receipt(run, documents["exports/step_003000/metadata.json"], metadata_sha256, owner_id, root,
        parent_weights, parent_metadata, numeric_validation, archives)
    evidence_names = [*DOCUMENTS, "history.jsonl"]
    for step in range(500, 3001, 500):
        evidence_names += ["checkpoints/step_%06d.pt" % step, "exports/step_%06d/model_state.npz" % step,
            "exports/step_%06d/metadata.json" % step]
    pin_payload(Path(__file__), pins)
    recheck_pins(pins)
    manifest = dict(schema=SCHEMA+".evidence", created_utc=datetime.now(timezone.utc).isoformat(),
        source_run_unmodified=True, fixed_endpoint_only=True, first_read_bytes_pinned_and_rechecked=True,
        publisher_source_sha256=pins[str(Path(__file__).absolute())],
        helper_source_sha256=PRIOR_SHA, numeric_loader_source_sha256=LOADER_SHA,
        source_files={name: dict(path=str(run/name), sha256=pins[str(run/name)]) for name in sorted(set(evidence_names))},
        parent_files=dict(weights=dict(path=str(parent_weights), sha256=PARENT_NPZ_SHA),
            metadata=dict(path=str(parent_metadata), sha256=PARENT_METADATA_SHA)))
    # HH_261001 - Finish all audits before the first write; failed data or an existing report never gets overwritten.
    output.mkdir()
    write_new(output/"result.json", report); write_new(output/"numeric_loader_receipt.json", numeric)
    write_new(output/"evidence_manifest.json", manifest); write_new(output/"comparisons.md", comparison_markdown(report))
    write_new(output/"README.md", markdown(report, numeric, run))
    return dict(output=str(output), passed=report["diagnostic_pass_count"], total=48,
        official_main_approved="0/9", official_approved="0/30",
        promoted=False, numeric_status=numeric["status"], startup_recovery_claimed=False, physical_trials_included=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("run-root", "output", "parent-weights", "parent-metadata"): parser.add_argument("--"+name, type=Path, required=True)
    for name in ("summary", "declaration", "checkpoint", "weights", "metadata"): parser.add_argument("--"+name+"-sha256", required=True)
    parser.add_argument("--numeric-validation", action="store_true"); parser.add_argument("--owner-id")
    args = parser.parse_args()
    print(json.dumps(publish(args.run_root, args.output, summary_sha256=args.summary_sha256, declaration_sha256=args.declaration_sha256,
        checkpoint_sha256=args.checkpoint_sha256, weights_sha256=args.weights_sha256, metadata_sha256=args.metadata_sha256,
        parent_weights=args.parent_weights, parent_metadata=args.parent_metadata, numeric_validation=args.numeric_validation,
        owner_id=args.owner_id), ensure_ascii=False))


if __name__ == "__main__": main()
