"""HH_261001 - Publish the fixed mixed replay endpoint without promotion or checkpoint selection.

The report is create-only. Numeric loading is an explicitly requested, CPU-only
diagnostic; importing this publisher never imports Torch or starts a simulator.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_v2_balanced_replay_results.20261001.v1"
NUMERIC_SCHEMA = "portable_e2e.temporal_v2_balanced_numeric_validation_receipt.20261001.v1"
RUN_SCHEMA = "portable_e2e.temporal_v2_balanced_replay_screening.20261001.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_v2_balanced_replay_export.20261001.v1"
ENDPOINT_SCHEMA = "portable_e2e.balanced_replay_endpoint_screen.20261001.v1"
PARENT_SHA = "6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece"
TRAINER_SHA = "a1e3ad4ed9302b02701c7e3aa56de88afacf6681b275c1bc6986f79ebfd79103"
LOADER_SHA = "6f438e5c97c7d241ff26f4ff0ae0953adb2c1804d70842ba0b75de7fa1a2877f"
LOADER_FILE = "learning/portable_e2e/temporal_balanced_export_loader_research_20261001.py"
PROBES = [266, 273, 279, 286, 38, 257, 472, 664]
BATCH_DOMAINS = ["v4_joint", "v4_joint", "legacy_curved_raw", "legacy_straight_raw"]
RAW_COMPONENTS = ("total", "raw_curvature", "raw_preferred_speed")
SOURCE_NAMES = frozenset((
    "learning/portable_e2e/__init__.py", "learning/portable_e2e/config/common_10hz_v1.contract.json",
    "learning/portable_e2e/contract.py", "learning/portable_e2e/dataset.py",
    "learning/portable_e2e/deterministic_linear_interpolation_v2.py", "learning/portable_e2e/model.py",
    "learning/portable_e2e/path_time_decoder_v2.py", "learning/portable_e2e/stop_primitive_research.py",
    "learning/portable_e2e/temporal_dataset_v2.py", "learning/portable_e2e/temporal_losses_v2.py",
    "learning/portable_e2e/temporal_model_v2.py", "learning/portable_e2e/torch_dataset.py",
    "tools/canonical_workspace.py", "tools/e2e/train_temporal_v2_balanced_replay_20261001.py",
    "tools/e2e/train_temporal_v2_low_speed_joint_20261001.py",
    "tools/e2e/train_temporal_v2_raw_bootstrap_20261001.py", "tools/workspace_python.py",
))
DOCUMENTS = ("summary.json", "declaration.json", "started.json", "endpoint_diagnostic_screens.json",
    "known_dev_raw_before.json", "known_dev_raw_final.json", "known_moving_dev599_raw_before.json",
    "known_moving_dev599_raw_final.json", "legacy_train_raw_before.json", "legacy_train_raw_final.json",
    "v4_train_joint_before.json", "v4_train_joint_final.json", "exports/step_003000/metadata.json")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def is_sha(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def regular(path, limit=32 * 1024**2):
    # HH_261001 - Do not follow symbolic evidence files or allow unbounded document/archive reads.
    path = Path(path)
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= limit,
            "bounded regular evidence file required: " + str(path))
    return path.read_bytes()


def digest(path, limit=64 * 1024**2):
    return hashlib.sha256(regular(path, limit)).hexdigest()


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate evidence JSON key")
        result[key] = value
    return result


def read_json(path):
    return json.loads(regular(path), object_pairs_hook=unique_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError("nonfinite JSON: " + value)))


def number(value):
    require(type(value) in (int, float) and math.isfinite(value) and value >= 0,
            "finite nonnegative diagnostic number required")
    return value


def close(actual, expected):
    require(math.isclose(number(actual), number(expected), rel_tol=1e-6, abs_tol=1e-7),
            "stored aggregate differs from full fixed rows")


def mean(rows, component):
    require(rows, "empty metric denominator")
    return math.fsum(number(r[component]) for r in rows) / len(rows)


def identities(rows):
    require(isinstance(rows, list) and rows, "full fixed row list required")
    keys = [(r["index"], r["token"]) for r in rows]
    require(all(type(i) is int and i >= 0 and isinstance(t, str) and t for i, t in keys)
            and len(set(keys)) == len(keys) and len({t for _, t in keys}) == len(keys),
            "duplicate/invalid evaluation anchor identity")
    return keys


def verify_pair(before, after, denominator):
    require(before.get("independent_validation") is False and after.get("independent_validation") is False,
            "known/TRAIN diagnostics cannot become independent validation")
    require(identities(before["rows"]) == identities(after["rows"])
            and len(before["rows"]) == denominator, "before/final full fixed membership differs")
    require(all(all(new.get(k) == old[k] for k in ("episode_id", "map", "stratum") if k in old)
                for old, new in zip(before["rows"], after["rows"])), "fixed source episode/map/geometry stratum changed")


def verify_raw_report(report, denominator, split, episode_count):
    require(report.get("split") == split and report.get("legacy_decoded") is False
            and report.get("independent_validation") is False
            and report.get("full_fixed_eligible_denominator") == denominator,
            "raw diagnostic denominator/split/decoder scope differs")
    rows = report["rows"]; identities(rows)
    require(len(rows) == denominator and all(r.get("stratum") in ("curved", "straight") for r in rows),
            "raw diagnostic full row count/stratum differs")
    for row in rows:
        close(row["total"], .25 * number(row["raw_curvature"]) + number(row["raw_preferred_speed"]))
        require(all(row.get(k) == 0 for k in ("intent", "stop_distance", "initial_acceleration",
                                             "temporal_xy", "temporal_speed")), "legacy raw-only objective changed")
    groups = report["groups"]
    require(set(groups) == {"all_eligible", "curved", "straight"}, "raw groups differ")
    for name, group in groups.items():
        selected = rows if name == "all_eligible" else [r for r in rows if r["stratum"] == name]
        require(type(group["count"]) is int and group["count"] == len(selected), "raw group count differs")
        for component in RAW_COMPONENTS: close(group[component], mean(selected, component))
    episodes = {r["episode_id"] for r in rows}
    require(len(episodes) == episode_count and set(report["per_episode"]) == episodes,
            "full raw episode denominator differs")
    for episode, group in report["per_episode"].items():
        selected = [r for r in rows if r["episode_id"] == episode]
        require(type(group["count"]) is int and group["count"] == len(selected), "episode count differs")
        for component in RAW_COMPONENTS: close(group[component], mean(selected, component))


def verify_v4_report(report):
    require(report.get("split") == "train" and report.get("independent_validation") is False
            and report.get("full_fixed_denominator") == 327, "V4 is full327 TRAIN, not DEV")
    rows = report["rows"]; identities(rows)
    require(len(rows) == 327 and {r["index"] for r in rows} == set(range(327)), "V4 full index denominator differs")
    for field, component in (("mean_ade_6p4s_m", "ade_6p4s_m"), ("mean_fde_6p4s_m", "fde_6p4s_m"),
            ("mean_instantaneous_speed_mae_mps", "instantaneous_speed_mae_mps")):
        close(report[field], mean(rows, component))
    close(report["maximum_ade_6p4s_m"], max(number(r["ade_6p4s_m"]) for r in rows))


def verify_known8(report):
    require(report.get("independent_validation") is False and report.get("temporal_trajectory_evaluated") is False,
            "known8 raw probes are not temporal/independent approval")
    rows = report["rows"]; identities(rows)
    require([r["index"] for r in rows] == PROBES, "fixed known8 probes differ")
    for row in rows: close(row["total"], .25 * row["raw_curvature"] + row["raw_preferred_speed"])
    close(report["mean_total"], mean(rows, "total"))


def endpoint_checks(before599, after599, after8, after_v4):
    # HH_261001 - Recompute all ten original gates without rounding, tolerance-based acceptance, or selection.
    rows = []
    def add(name, actual, threshold, scope):
        rows.append(dict(name=name, actual=number(actual), threshold=number(threshold),
                         passed=actual <= threshold, evidence_scope=scope))
    add("V4_TRAIN_ADE", after_v4["mean_ade_6p4s_m"], .1, "TRAIN_resubstitution327")
    add("V4_TRAIN_FDE", after_v4["mean_fde_6p4s_m"], .25, "TRAIN_resubstitution327")
    add("V4_TRAIN_instantaneous_speed_MAE", after_v4["mean_instantaneous_speed_mae_mps"], .03, "TRAIN_resubstitution327")
    add("known_DEV8_raw_total", after8["mean_total"], .303911, "previously_exposed8_probes")
    require(set(before599["per_episode"]) == set(after599["per_episode"]), "known episode identities differ")
    for label, before, after in [("aggregate", before599["groups"]["all_eligible"], after599["groups"]["all_eligible"]),
            *[(ep, before599["per_episode"][ep], after599["per_episode"][ep]) for ep in sorted(before599["per_episode"])]]:
        for component in ("raw_curvature", "raw_preferred_speed"):
            add("known_DEV599/" + label + "/" + component, after[component], before[component] * 1.10 + 1e-4,
                "previously_exposed599_raw_diagnostic_not_independent")
    require(len(rows) == 10, "ten prospective endpoint gates required")
    return rows


def verify_lineage(declaration, summary, metadata, declaration_sha):
    require(declaration.get("schema") == summary.get("schema") == RUN_SCHEMA
            and metadata.get("schema") == EXPORT_SCHEMA, "explicit mixed run/export schema required")
    fields = dict(training_only=True, simulation_only=True, one_episode_screening=False, research_screening=True,
        mixed_raw_joint_training=True, main_training=False, model_promoted=False, vehicle_control_approved=False,
        independent_validation=False, stop_intent_trained=False, labels_clipped=False, labels_extrapolated=False,
        legacy_decoded=False, decoder_integrated=True, raw_spatial_only=False)
    require(all(obj.get(k) is value for obj in (declaration, summary, metadata) for k, value in fields.items()),
            "mixed research scope cannot be promoted or relabelled")
    require(all(type(obj.get("training_episode_count")) is int and obj["training_episode_count"] == 9
                and obj.get("runtime_intent") == "explicit_cruise_GO_not_head"
                for obj in (declaration, summary, metadata)), "nine-episode explicit scenario GO required")
    require(metadata.get("inference_loader_authorized") is False
            and metadata.get("single_episode_export_compatibility_claimed") is False,
            "original mixed export inference authorization must remain false")
    require(declaration.get("parent_checkpoint_sha256") == metadata.get("parent_checkpoint_sha256") == PARENT_SHA
            and type(metadata.get("parent_source_optimizer_updates")) is int and metadata["parent_source_optimizer_updates"] == 200
            and type(metadata.get("known_total_optimizer_updates")) is int and metadata["known_total_optimizer_updates"] == 3200
            and metadata.get("declaration_sha256") == declaration_sha,
            "exact raw200 to fixed3000 endpoint lineage differs")
    require(all(type(obj.get(k)) is int and obj[k] == value for obj in (summary, metadata)
                for k, value in dict(optimizer_updates=3000, examples_seen=12000,
                    v4_examples_seen=6000, legacy_examples_seen=6000).items()), "actual fixed update/exposure counts differ")
    require(summary.get("status") == "BOUNDED_BALANCED_REPLAY_COMPLETE_NOT_PROMOTED"
            and summary.get("source_hashes_match") is True and summary.get("wall_budget_stop") is False
            and summary.get("optimizer_state_retained") is True, "completed unpromoted run receipt required")
    require(declaration.get("requested_optimizer_updates") == declaration.get("maximum_optimizer_updates")
            == metadata.get("maximum_optimizer_updates") == 3000
            and declaration.get("start_optimizer_updates") == 0
            and declaration.get("resume_checkpoint_sha256") is None
            and declaration.get("effective_batch_size") == 4 and declaration.get("seed") == 2026100103
            and declaration.get("batch_domains") == metadata.get("batch_domains") == BATCH_DOMAINS,
            "fixed fresh optimizer sampler declaration differs")
    sources = declaration.get("source_sha256")
    require(isinstance(sources, dict) and set(sources) == SOURCE_NAMES and all(is_sha(v) for v in sources.values())
            and sources["tools/e2e/train_temporal_v2_balanced_replay_20261001.py"] == TRAINER_SHA
            and metadata.get("source_sha256") == sources, "pinned imported17 source closure differs")
    require(declaration.get("training_dataset_fingerprint_sha256") == metadata.get("training_dataset_fingerprint_sha256")
            and is_sha(metadata.get("training_dataset_fingerprint_sha256")), "mixed dataset fingerprint differs")
    cohorts = declaration["cohorts"]
    require(cohorts["legacy"]["sample_count"] == 2787 and cohorts["legacy"]["episode_count"] == 8
            and cohorts["legacy"]["split"] == cohorts["v4"]["split"] == "train"
            and cohorts["v4"]["statistics"]["anchor_count"] == 327
            and cohorts["known_dev"]["sample_count"] == 665 and cohorts["known_dev"]["split"] == "val"
            and declaration.get("known_dev_is_independent") is False, "TRAIN/knownVAL cohort counts differ")
    require(summary["final_checkpoint_sha256"] == metadata["checkpoint_sha256"], "fixed endpoint checkpoint differs")
    checkpoints = summary["checkpoints"]
    require([c["optimizer_updates"] for c in checkpoints] == list(range(500, 3001, 500)),
            "all fixed intermediate checkpoint receipts retained")
    final = checkpoints[-1]
    require(final["sha256"] == metadata["checkpoint_sha256"] and final["export_npz_sha256"] == metadata["npz_sha256"]
            and is_sha(final["export_metadata_sha256"]), "final3000 numeric export linkage differs")
    return final


def verify_history(rows, declaration, v4, legacy):
    require(len(rows) == 3000, "all3000 actual optimizer update journals required")
    low = {r["index"]: r["token"] for r in v4["rows"]}
    old = {r["index"]: r for r in legacy["rows"]}
    declared = declaration["legacy_strata"]["indices"]
    by_kind = {kind: {i for values in declared[kind].values() for i in values} for kind in ("curved", "straight")}
    require(len(by_kind["curved"]) == 328 and len(by_kind["straight"]) == 2159
            and not (by_kind["curved"] & by_kind["straight"])
            and by_kind["curved"] | by_kind["straight"] == set(old), "legacy replay eligible strata differ")
    schedule, exposure = [], Counter()
    for step, row in enumerate(rows, 1):
        require(row.get("step") == row.get("optimizer_updates") == step
                and row.get("examples_seen") == step * 4
                and row.get("v4_examples_seen") == row.get("legacy_examples_seen") == step * 2,
                "actual update/exposure journal differs")
        indices, tokens = row["indices"], row["sample_ids"]
        require(len(indices) == len(tokens) == 4 and indices[0] != indices[1], "declared effective B4 differs")
        require(tokens[:2] == [low.get(indices[0]), low.get(indices[1])], "journal V4 identity differs")
        for position, kind in ((2, "curved"), (3, "straight")):
            index = indices[position] - 327
            require(index in by_kind[kind] and tokens[position] == old[index]["token"]
                    and old[index]["stratum"] == kind, "journal legacy true stratum/identity differs")
        for domain in ("v4", "legacy"):
            require(row["domain_parameter_gradients"][domain]["all_present_parameter_gradients_finite"] is True,
                    "nonfinite domain gradient receipt")
        close(row["total_loss"], .5 * row["v4_joint"]["total"] + .5 * row["legacy_raw"]["total"])
        schedule.append(indices); exposure.update(tokens)
    schedule_sha = hashlib.sha256(json.dumps(schedule, separators=(",", ":")).encode()).hexdigest()
    require(schedule_sha == declaration["sampler_remaining_schedule_sha256"], "actual recorded sampler order differs")
    require(len(exposure) == 327 + 2487, "actual replay omitted eligible anchors")
    return dict(actual_update_rows=3000, actual_anchor_exposures=sum(exposure.values()),
                distinct_eligible_anchors=len(exposure), recorded_schedule_sha256=schedule_sha,
                known_VAL_or_TEST_fitted=False)


def comparisons(before, after, scope):
    # HH_261001 - Relative reduction of a loss is not a percentage improvement in driving quality.
    result = []
    for label in ["aggregate", *sorted(before["per_episode"])]:
        old = before["groups"]["all_eligible"] if label == "aggregate" else before["per_episode"][label]
        new = after["groups"]["all_eligible"] if label == "aggregate" else after["per_episode"][label]
        for component in RAW_COMPONENTS:
            result.append(dict(scope=scope, group=label, component=component, count=old["count"],
                parent=old[component], final=new[component], delta=new[component] - old[component]))
    for label in ("curved", "straight"):
        old, new = before["groups"][label], after["groups"][label]
        for component in RAW_COMPONENTS:
            result.append(dict(scope=scope, group=label, component=component, count=old["count"],
                parent=old[component], final=new[component], delta=new[component] - old[component]))
    return result


def build_report(documents, history, declaration_sha):
    declaration, summary = documents["declaration.json"], documents["summary.json"]
    metadata = documents["exports/step_003000/metadata.json"]
    final = verify_lineage(declaration, summary, metadata, declaration_sha)
    gpu = documents["started.json"]["gpu"]
    require(type(gpu.get("physical_index")) is int and gpu["physical_index"] == 0
            and gpu.get("uuid") == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
            and gpu.get("idle_before_torch") is True, "actual owned GPU0 UUID preflight receipt differs")
    for prefix, denominator, split, episodes in (("legacy_train_raw", 2487, "train", 8),
            ("known_moving_dev599_raw", 599, "val", 2)):
        before, after = documents[prefix + "_before.json"], documents[prefix + "_final.json"]
        verify_pair(before, after, denominator)
        for value in (before, after): verify_raw_report(value, denominator, split, episodes)
    before599, after599 = documents["known_moving_dev599_raw_before.json"], documents["known_moving_dev599_raw_final.json"]
    require(all(x.get("available_known_val_count") == 665 and x.get("excluded_count") == 66
                for x in (before599, after599)), "known599 is not all665 knownVAL")
    before8, after8 = documents["known_dev_raw_before.json"], documents["known_dev_raw_final.json"]
    verify_pair(before8, after8, 8)
    for value in (before8, after8): verify_known8(value)
    arithmetic_differences = {}
    for stage, probe_report, full_report in (("before", before8, before599), ("final", after8, after599)):
        full_rows = {(r["index"], r["token"]): r for r in full_report["rows"]}
        differences = []
        for probe in probe_report["rows"]:
            require((probe["index"], probe["token"]) in full_rows, "known8 probe is not a member of full knownDEV599")
            for component in RAW_COMPONENTS:
                # HH_261001 - Preserve distinct-batch raw arithmetic, never replace either metric with the other.
                differences.append(abs(number(probe[component]) - number(full_rows[(probe["index"], probe["token"])][component])))
        arithmetic_differences[stage + "_maximum_absolute_raw_component_difference"] = max(differences)
    before_v4, after_v4 = documents["v4_train_joint_before.json"], documents["v4_train_joint_final.json"]
    verify_pair(before_v4, after_v4, 327)
    for value in (before_v4, after_v4): verify_v4_report(value)
    checks = endpoint_checks(before599, after599, after8, after_v4)
    endpoint = documents["endpoint_diagnostic_screens.json"]
    require(endpoint.get("schema") == ENDPOINT_SCHEMA and endpoint.get("fixed_endpoint_updates") == 3000
            and endpoint.get("checks") == checks
            and all(type(check.get("passed")) is bool for check in endpoint["checks"])
            and endpoint.get("all_diagnostic_screens_passed") is all(c["passed"] for c in checks)
            and all(endpoint.get(k) is False for k in ("feature_approved", "vehicle_control_approved",
                                                       "model_promoted", "independent_validation")),
            "original ten-gate endpoint receipt differs; never rewrite acceptance")
    journal = verify_history(history, declaration, before_v4, documents["legacy_train_raw_before.json"])
    legacy_rows = documents["legacy_train_raw_before.json"]["rows"]
    require({r["map"] for r in legacy_rows} == {"C_track_1_0_7", "Town01", "Town07"}, "actual TRAIN map coverage differs")
    require(not ({r["token"] for r in legacy_rows} | {r["token"] for r in before_v4["rows"]})
            & {r["token"] for r in before599["rows"]}, "knownDEV tokens overlap actual TRAIN")
    raw_comparisons = comparisons(before599, after599, "previously_exposed_known_DEV599_raw_not_independent")
    raw_comparisons += comparisons(documents["legacy_train_raw_before.json"], documents["legacy_train_raw_final.json"],
                                   "eligible_legacy_TRAIN2487_resubstitution_raw_only")
    return dict(schema=SCHEMA, fixed_endpoint_updates=3000, checkpoint_selection_by_dev=False,
        official_function_approvals=0, official_function_total=30, model_promoted=False,
        simulator_control_authorized=False, vehicle_control_approved=False, independent_validation=False,
        main_training=False, stop_intent_trained=False, runtime_intent="explicit_cruise_GO_not_head",
        all_diagnostic_screens_passed=all(c["passed"] for c in checks),
        diagnostic_pass_count=sum(c["passed"] for c in checks), diagnostic_gate_count=10, checks=checks,
        lineage=dict(parent_checkpoint_sha256=PARENT_SHA, parent_updates=200, fresh_additional_updates=3000,
            known_total_updates=3200, final_checkpoint_sha256=metadata["checkpoint_sha256"],
            final_npz_sha256=metadata["npz_sha256"], final_metadata_sha256=final["export_metadata_sha256"],
            declaration_sha256=declaration_sha, training_dataset_fingerprint_sha256=metadata["training_dataset_fingerprint_sha256"],
            frozen_training_sources=declaration["source_sha256"]),
        data=dict(available_TRAIN_anchors=3114, TRAIN_episodes=9, TRAIN_map_count=3,
            TRAIN_maps=["C_track_1_0_7", "Town01", "Town07"], V4_TRAIN_anchors=327,
            eligible_legacy_TRAIN_anchors=2487, excluded_legacy_TRAIN_anchors=300,
            eligible_legacy_by_map=dict(Counter(r["map"] for r in legacy_rows)),
            curved_legacy_anchors=328, straight_legacy_anchors=2159, known_VAL_available=665,
            known_DEV_moving_evaluated=599, known_DEV_excluded=66, genuinely_independent_DEV_available=False,
            actual_exposures=12000, V4_exposures=6000, legacy_curved_exposures=3000,
            legacy_straight_exposures=3000, legacy_equivalent_uniform_epochs_claimed=False),
        v4_comparison={name: dict(parent=before_v4[name], final=after_v4[name],
            delta=after_v4[name] - before_v4[name]) for name in
            ("mean_ade_6p4s_m", "mean_fde_6p4s_m", "mean_instantaneous_speed_mae_mps")},
        known8_comparison=dict(parent=before8["mean_total"], final=after8["mean_total"],
            delta=after8["mean_total"] - before8["mean_total"]), raw_comparisons=raw_comparisons,
        known8_full599_distinct_batch_arithmetic=dict(arithmetic_differences, original_metrics_preserved=True,
            same_index_token_membership_verified=True, quality_gate_tolerance_changed=False),
        actual_journal=journal, timing={k: summary[k] for k in ("elapsed_owned_gpu_phase_wall_seconds",
            "fit_phase_including_baseline_seconds", "optimizer_loop_wall_seconds", "postfit_endpoint_evaluation_seconds")},
        actual_gpu_preflight=gpu,
        unchanged_export_authorization=dict(inference_loader_authorized=False, one_episode_screening=False,
                                            single_episode_export_compatibility_claimed=False))


def verify_evidence_files(run, root, documents, summary_sha, declaration_sha):
    require(digest(run / "summary.json") == summary_sha and digest(run / "declaration.json") == declaration_sha,
            "caller-pinned summary/declaration differs")
    metadata = documents["exports/step_003000/metadata.json"]
    final = documents["summary.json"]["checkpoints"][-1]
    require(digest(run / "exports/step_003000/metadata.json") == final["export_metadata_sha256"]
            and digest(run / "exports/step_003000/model_state.npz") == metadata["npz_sha256"]
            and digest(run / "checkpoints/step_003000.pt") == metadata["checkpoint_sha256"],
            "original final3000 checkpoint/NPZ/metadata bytes differ")
    for name, sha in documents["declaration.json"]["source_sha256"].items():
        require(name in SOURCE_NAMES and digest(root / name) == digest(run / "source" / name) == sha,
                "live/archived frozen imported source differs")


def load_numeric_model(run, metadata, metadata_sha, owner_id, root):
    # HH_261001 - This lazy path is allowed only after local CPU reservation release, never at import/report build.
    require(digest(root / LOADER_FILE) == LOADER_SHA, "reviewed mixed numeric loader source differs")
    entry = root / "tools/workspace_python.py"
    spec = importlib.util.spec_from_file_location("hh_balanced_publish_workspace", entry)
    workspace = importlib.util.module_from_spec(spec); spec.loader.exec_module(workspace); workspace.configure(root)
    from portable_e2e.temporal_balanced_export_loader_research_20261001 import OWNER_SCHEMA, load_balanced_research_npz
    binding = dict(schema=OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=owner_id,
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, weights_sha256=metadata["npz_sha256"],
        metadata_sha256=metadata_sha, checkpoint_sha256=metadata["checkpoint_sha256"],
        declaration_sha256=metadata["declaration_sha256"], fixed_endpoint_updates=3000)
    return load_balanced_research_npz(run / "exports/step_003000/model_state.npz",
        run / "exports/step_003000/metadata.json", weights_sha256=metadata["npz_sha256"],
        metadata_sha256=metadata_sha, research_owner_binding=binding, source_root=root)


def numeric_receipt(run, metadata, metadata_sha, owner_id, root, enabled):
    base = dict(schema=NUMERIC_SCHEMA, fixed_endpoint_updates=3000,
        checkpoint_sha256=metadata["checkpoint_sha256"], weights_sha256=metadata["npz_sha256"],
        metadata_sha256=metadata_sha, declaration_sha256=metadata["declaration_sha256"],
        loader_source_sha256=LOADER_SHA, loader_source=LOADER_FILE,
        numeric_validation_only=True, no_Torch_pickle=True, simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, inference_loader_authorized=False,
        independent_validation=False, end_to_end_10Hz_validated=False)
    if not enabled: return dict(base, status="NOT_RUN", full_numeric_state_validated=False)
    require(isinstance(owner_id, str) and owner_id.strip(), "numeric validation explicit owner id required")
    model = load_numeric_model(run, metadata, metadata_sha, owner_id, root)
    require(model._research_temporal_export_metadata == metadata
            and model._research_balanced_simulator_control_authorized is False
            and model._research_temporal_checkpoint_sha256 == metadata["checkpoint_sha256"],
            "numeric loader altered metadata or control scope")
    state = model.state_dict()
    require(set(state) == set(metadata["state_tensor_shapes"])
            and not model.training and all(not p.requires_grad for p in model.parameters()),
            "full frozen eval-state numeric validation required")
    return dict(base, status="PASS_NUMERIC_CPU_ONLY_NOT_CONTROL", full_numeric_state_validated=True,
        owner_id=owner_id, tensor_count=len(state), loader_schema=model._research_balanced_numeric_loader_schema,
        original_export_metadata_unmodified=True, float32_finite_full_state_validated=True)


def markdown(report, numeric, run):
    checks, timing = report["checks"], report["timing"]
    lines = ["# 3000회 균형 재학습: 개선 확인, 승인 보류", "",
        "저속 직진 데이터와 기존 회전·직진 데이터를 함께 학습했습니다. 기존 자료의 raw 손실은 개선됐지만, "
        "사전 기준 10개 중 **%d개 통과 / %d개 미달**입니다. 최종 3000회 가중치만 비교했으며 좋은 중간 결과를 골라내지 않았습니다."
        % (report["diagnostic_pass_count"], 10 - report["diagnostic_pass_count"]), "",
        "정식 기능 승인 **0/30**. 이번 자료는 연구용 학습 진단이며, 모든 맵 주행·회전 성공·실차 사용을 입증하지 않습니다. "
        "지도 모델은 이번 주행 모델 학습에 포함되지 않았습니다.", "", "## 무엇이 달라졌나", "",
        "| 지표 | 부모 raw200 | 최종 mixed3000 | 읽는 방법 |", "|---|---:|---:|---|"]
    labels = {"mean_ade_6p4s_m": "저속 경로 평균 오차 ADE (m)", "mean_fde_6p4s_m": "저속 6.4초 끝점 오차 FDE (m)",
              "mean_instantaneous_speed_mae_mps": "저속 순간 속도 오차 MAE (m/s)"}
    for key, row in report["v4_comparison"].items():
        lines.append("| %s | %.6f | %.6f | 같은 327개 TRAIN 재평가, 작을수록 좋음 |" % (labels[key], row["parent"], row["final"]))
    row = report["known8_comparison"]
    lines.extend(["| 기존 known DEV 8개 raw 합성 손실 | %.6f | %.6f | 이전에 여러 번 본 진단 표본, 독립 검증 아님 |"
        % (row["parent"], row["final"]), "",
        "속도 오차는 부모보다 커졌지만 절대 기준은 통과했습니다. 손실 감소 비율을 실제 주행 성능 개선율로 해석하면 안 됩니다.",
        "", "## 사전에 고정한 기준 10개 — 모두 공개", "",
        "| 기준 | 실제 최종 값 | 허용 상한 | 결과 |", "|---|---:|---:|---|"])
    for check in checks:
        lines.append("| %s | %.9f | %.9f | %s |" % (check["name"], check["actual"], check["threshold"],
                                                                  "PASS" if check["passed"] else "FAIL"))
    lines.extend(["", "Town03 직진 raw-speed 손실은 악화되어 보존 기준에 미달했습니다. "
        "전체 평균 개선만으로 이 실패를 숨기거나 승인하지 않습니다.", "", "## 학습량·데이터·계보", "",
        "- 부모: raw200 → 새 AdamW로 3000회 추가. 알려진 누적 업데이트는 3200회입니다.",
        "- 9개 TRAIN 주행 기록 / 3개 맵: Town07, Town01, C_track. 가용 3114개 중 저속 327개와 적격 기존 2487개를 사용했습니다.",
        "- 배치 4개: 저속 joint 2 + 실제 곡률 회전 raw 1 + 직진 raw 1. 실제 12000회 노출: 저속 6000, 기존 회전 3000, 기존 직진 3000.",
        "- 저속만 실제 v0·실제 미래 궤적과 decoder를 함께 학습했습니다. 기존 데이터의 최대 약 8.425 m/s 속도는 잘라내지 않고 raw-only로 유지했습니다.",
        "- known DEV는 665개 중 고정된 이동 599개를 전후 평가했습니다. 이미 반복 사용한 자료이며 새 독립 DEV는 없습니다. TEST를 학습·선택에 사용하지 않았습니다.",
        "- STOP/기타 기능 head를 새로 학습하지 않았습니다. GO는 저속 cruise 시나리오의 명시적 조건입니다.", "",
        "실제 GPU 소유 구간 %.1f초 (%.2f분): baseline 포함 fit %.1f초, optimizer loop %.1f초, 최종 진단 %.1f초."
        % (timing["elapsed_owned_gpu_phase_wall_seconds"], timing["elapsed_owned_gpu_phase_wall_seconds"] / 60,
           timing["fit_phase_including_baseline_seconds"], timing["optimizer_loop_wall_seconds"],
           timing["postfit_endpoint_evaluation_seconds"]), "", "## 결과 파일", "",
        "- [전후 전체 비교](raw_comparisons.md): known DEV599 전체·각 주행 기록과 기존 TRAIN 회전/직진·각 기록의 raw 성분을 전부 공개합니다.",
        "- [결과·계보 JSON](result.json), [원자료 SHA 목록](evidence_manifest.json), [숫자 전용 loader 검사](numeric_loader_receipt.json).", "",
        "숫자 loader: `%s`. 이는 CPU float32 가중치 파일 검사만 의미합니다. simulator/실차 제어 승인, 주행 성공, 전체 10 Hz 증명이 아닙니다."
        % numeric["status"], "", "원자료: `%s`" % run, "",
        "다음은 미달한 Town03 직진 속도와 저속 ADE/FDE의 실제 출력을 분석해, 독립 자료·폐루프 기준을 사전에 고정한 개선을 검토하는 것입니다. "
        "이번 보고서는 추가 학습이나 물리 주행을 시작하지 않습니다.", ""])
    return "\n".join(lines)


def comparison_markdown(rows):
    lines = ["# 전체 raw 성분 전후 비교", "", "TRAIN 재평가와 이미 노출된 known DEV 진단입니다. 폐루프 성공·독립 검증이 아닙니다.", "",
        "| 자료 | 그룹/기록 | 분모 | 성분 | 부모 | 최종 | 변화 (최종−부모) |", "|---|---|---:|---|---:|---:|---:|"]
    for row in rows:
        lines.append("| %s | %s | %d | %s | %.9f | %.9f | %+.9f |" % (row["scope"], row["group"], row["count"],
            row["component"], row["parent"], row["final"], row["delta"]))
    return "\n".join(lines) + "\n"


def write_new(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n")


def publish(run, output, *, summary_sha256, declaration_sha256, numeric_validation=False, owner_id=None, root=ROOT):
    run, output, root = Path(run), Path(output), Path(root)
    require(run.is_dir() and not run.is_symlink() and root.is_dir() and not root.is_symlink(), "regular run/source directories required")
    require(not output.exists() and not output.is_symlink() and output.parent.is_dir() and not output.parent.is_symlink(),
            "fresh create-only report directory with existing regular parent required")
    require(is_sha(summary_sha256) and is_sha(declaration_sha256), "external exact run summary/declaration SHA required")
    require(shutil.disk_usage(output.parent).free >= 5 * 1024**3, "report mount must retain 5GiB floor; no deletion")
    documents = {name: read_json(run / name) for name in DOCUMENTS}
    history = [json.loads(line, object_pairs_hook=unique_object) for line in regular(run / "history.jsonl").splitlines()]
    report = build_report(documents, history, declaration_sha256)
    verify_evidence_files(run, root, documents, summary_sha256, declaration_sha256)
    metadata = documents["exports/step_003000/metadata.json"]
    numeric = numeric_receipt(run, metadata, report["lineage"]["final_metadata_sha256"], owner_id, root, numeric_validation)
    sources = {name: dict(path=str(run / name), sha256=digest(run / name)) for name in (*DOCUMENTS, "history.jsonl")}
    manifest = dict(schema=SCHEMA + ".evidence", created_utc=datetime.now(timezone.utc).isoformat(),
        source_files=sources, source_run_unmodified=True, fixed_endpoint_only=True,
        publisher_source_sha256=digest(Path(__file__)), numeric_loader_source_sha256=LOADER_SHA)
    # HH_261001 - All audits finish before any report write; any existing output is never reused or overwritten.
    output.mkdir()
    write_new(output / "result.json", report); write_new(output / "evidence_manifest.json", manifest)
    write_new(output / "numeric_loader_receipt.json", numeric)
    write_new(output / "raw_comparisons.md", comparison_markdown(report["raw_comparisons"]))
    write_new(output / "README.md", markdown(report, numeric, run))
    return dict(output=str(output), passed=report["diagnostic_pass_count"], total=10,
                official_approved="0/30", promoted=False, numeric_status=numeric["status"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary-sha256", required=True); parser.add_argument("--declaration-sha256", required=True)
    parser.add_argument("--numeric-validation", action="store_true")
    parser.add_argument("--owner-id")
    args = parser.parse_args()
    print(json.dumps(publish(args.run_root, args.output, summary_sha256=args.summary_sha256,
        declaration_sha256=args.declaration_sha256, numeric_validation=args.numeric_validation,
        owner_id=args.owner_id), ensure_ascii=False))


if __name__ == "__main__":
    main()
