"""HH_261002 - Numeric-only loading of the NEW fixed LEFT20k research endpoint.

The export's denied release flags stay immutable.  An external ROOT completion
authority authenticates the actual full journal and nineteen TRAIN sources;
neither a diagnostic PASS nor this loader grants vehicle functionality.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import torch

from . import temporal_intrinsic_curve_export_research_20261002 as parent_api

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_left_joint_numeric_loader_research.20261002.v1"
OWNER_SCHEMA = "portable_e2e.temporal_left_joint_owner_binding.20261002.v1"
AUTHORITY_SCHEMA = "portable_e2e.temporal_left_joint_completed_run_authority.20261002.v1"
TRAINING_SCHEMA = "portable_e2e.temporal_actual_left_joint_continuation.20261002.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_actual_left_joint_export_research.20261002.v1"
TRAINER_PATH = "tools/e2e/train_temporal_left_joint_20261002.py"
TRAINER_SHA = "dc8963d4bd4941911b600f1409827f0961977ae9af75c7c146789cdba020aa3a"
LEFT_CONVERSION_SHA = "f70350f3aeab5b88069759a259bf4b3059c775429eec6fabda38a70b966f2664"
HOLDOUT_CONVERSION_SHA = "d90c022464e716ad6b86eec06395df892349869b981b91b91eecd056ae926594"
LEFT_BASENAME = "town01-low-speed-left45-train-20261002-v3"
PARENT, PARENT_HISTORY_SHA = parent_api.PARENT, parent_api.PARENT_HISTORY_SHA
BATCH_DOMAINS = ["actual_recovery_joint", "old_low_joint", "legacy_raw_only", "actual_LEFT347_joint"]
EXPOSURES = dict(recovery=20000, recovery_focus=10000, recovery_full=10000, old_low=20000,
    legacy=20000, legacy_curved=10000, legacy_straight=10000, LEFT=20000)
require, is_sha, canonical, pinned, json_sha = (parent_api.require, parent_api.is_sha,
    parent_api.canonical, parent_api.pinned, parent_api.json_sha)


def finite_json_number(text):
    value = float(text)
    require(math.isfinite(value), "nonfinite JSON number")
    return value


def invalid_json_constant(text):
    raise ValueError("nonfinite JSON constant: " + text)


def read_json(path, sha, maximum=4 * 1024**2):
    result = json.loads(pinned(path, sha, maximum), object_pairs_hook=parent_api.unique_json_object,
        parse_float=finite_json_number, parse_constant=invalid_json_constant)
    require(type(result) is dict, "exact JSON object required")
    return result


def learning_rate_config():
    return dict(parent_api.old.learning_rate_config(), schedule="fixed20000_linear_warmup_cosine_two_groups")


def learning_rate(step, group):
    require(type(step) is int and 1 <= step <= 20000 and group in ("shared", "heads"), "fixed20k LR index required")
    cfg = learning_rate_config(); peak, final = cfg[group + "_peak"], cfg[group + "_final"]
    if step <= cfg["warmup_updates"]:
        return peak * (step / cfg["warmup_updates"])
    fraction = (step - cfg["warmup_updates"]) / (20000 - cfg["warmup_updates"])
    return final + .5 * (peak - final) * (1 + math.cos(math.pi * fraction))


def verify_scope(d):
    require(type(d) is dict and d.get("schema") == EXPORT_SCHEMA and d.get("training_schema") == TRAINING_SCHEMA
        and d.get("architecture_schema") == parent_api.old.ARCHITECTURE_SCHEMA
        and d.get("model_id") == parent_api.old.MODEL_ID, "explicit NEW LEFT20k export schema required")
    numbers = dict(optimizer_updates=20000, additional_optimizer_updates=20000, maximum_optimizer_updates=20000,
        effective_batch_size=4, examples_seen=80000, recovery_examples_seen=20000, v4_examples_seen=20000,
        legacy_examples_seen=20000, LEFT_examples_seen=20000, parent_optimizer_updates=6000,
        parent_known_total_optimizer_updates=12200, known_total_optimizer_updates=32200,
        training_episode_count=19, recovery_training_episode_count=9, replay_training_episode_count=10,
        LEFT_training_episode_count=1, holdout_episode_count=2, trainable_state_tensor_count=54,
        frozen_state_tensor_count=47, trainable_parameter_count=2277827, seed=2026100201)
    require(all(type(d.get(k)) is int and d[k] == v for k, v in numbers.items()), "fixed20k/80k/19TRAIN/101-47 lineage differs")
    flags = dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, runtime_loader_implemented=False,
        stop_intent_trained=False, backbone_frozen=False, visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True, semantic_outputs_preserved=False, legacy_decoded=False,
        independent_validation=False, checkpoint_selection_by_holdout=False, labels_clipped=False,
        labels_extrapolated=False, startup_recovery_fitted=False, holdout_payload_loaded=False)
    require(all(d.get(k) is v for k, v in flags.items()) and d.get("runtime_intent") == "explicit_cruise_GO_not_head"
        and d.get("current_training_purpose") == "actual_LEFT347_DATA_and_schedule_joint_continuation"
        and d.get("batch_domains") == BATCH_DOMAINS and d.get("learning_rate") == learning_rate_config(),
        "truthful GO-only/54 learned/47 fixed/denied release scope required")
    require(all(d.get("parent_" + key + "_sha256") == PARENT[role] for key, role in
        (("npz", "weights_sha256"), ("metadata", "metadata_sha256"), ("checkpoint", "checkpoint_sha256"), ("declaration", "declaration_sha256")))
        and d.get("parent_history_sha256") == PARENT_HISTORY_SHA, "original GOOD6000 parent required, not intrinsic candidate")
    require(all(is_sha(d.get(k)) for k in ("npz_sha256", "checkpoint_sha256", "declaration_sha256", "history_sha256",
        "fixed_sample_ids_sha256", "fixed_schedule_sha256", "frozen_state_sha256", "authorization_sha256", "training_dataset_fingerprint_sha256")),
        "all actual endpoint/journal/schedule/authority pins required")
    sources = d.get("source_sha256", {})
    require(type(sources) is dict and len(sources) == 30 and sources.get(TRAINER_PATH) == TRAINER_SHA
        and all(is_sha(v) for v in sources.values()), "exact NEW30 source closure required")
    protocol = d.get("protocol", {})
    require(type(protocol) is dict and protocol.get("schema") == TRAINING_SCHEMA and protocol.get("sources") == sources
        and protocol.get("actual_parent_final6000") == PARENT and protocol.get("actual_parent_history_sha256") == PARENT_HISTORY_SHA
        and protocol.get("new_LEFT_conversion_sha256") == LEFT_CONVERSION_SHA and protocol.get("new_LEFT_basename") == LEFT_BASENAME
        and protocol.get("gpu_preflight_revision") == parent_api.GPU_REVISION
        and protocol.get("exposures") == EXPOSURES and protocol.get("batch_domains") == BATCH_DOMAINS
        and protocol.get("learning_rate") == learning_rate_config() and protocol.get("objective")
        == ".25*(joint_recovery+joint_LEFT+joint_oldlow+.25*raw_legacy_curvature+raw_legacy_speed)", "fixed unchanged loss/new DATA+schedule protocol required")
    gpu = protocol["gpu_preflight_revision"]
    require(all(type(gpu[k]) is int for k in ("physical_index", "maximum_driver_residual_MiB",
        "required_utilization_percent", "required_GPU0_compute_process_count"))
        and all(type(gpu[k]) is bool for k in ("physical_queries_before_torch_initialization",
            "original_gpu0_lease_unchanged", "training_math_or_labels_changed")), "strict GPU revision integer/Boolean types required")
    pnumbers = dict(fixed_new_updates=20000, known_final_total_optimizer_updates=32200, effective_batch_size=4,
        examples_seen=80000, training_episode_count=19, eligible_TRAIN_anchor_count=5879, new_LEFT_count=347,
        new_LEFT_full_cycles=57, new_LEFT_cycle_remainder=221, checkpoint_interval=1000)
    require(all(type(protocol.get(k)) is int and protocol[k] == v for k, v in pnumbers.items())
        and all(protocol.get(k) is v for k, v in dict(original_loss_functions_unchanged=True, intrinsic_loss_added=False,
            legacy_decoded=False, no_DEV_or_HOLDOUT_payload_until_verified_final=True, fixed_final_only=True,
            no_checkpoint_selection=True, runtime_architecture_changed=False, runtime_caps_changed=False,
            labels_clipped=False, temporal_retiming=False, future_model_inputs=False, stop_intent_trained=False,
            model_promoted=False, vehicle_control_approved=False, inference_loader_authorized=False).items()), "prospective fixed membership/loss/runtime law differs")
    require(d.get("numerical_revision") == dict(kernel_id=parent_api.old.KERNEL_ID, strict_deterministic_algorithms=True,
        tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False)
        and d.get("deferred_holdout_conversion_sha256") == HOLDOUT_CONVERSION_SHA, "unchanged deterministic decoder/deferred holdout required")
    cohorts = d.get("cohorts", {})
    require(type(cohorts) is dict and set(cohorts) == {"recovery", "v4", "legacy", "LEFT"}
        and all(row.get("split") == "train" for row in cohorts.values()), "nineteen TRAIN sources only; no evaluation cohort fit")
    for name, count, episodes in (("recovery", 2718, 9), ("legacy", 2787, 8), ("LEFT", 347, 1)):
        row = cohorts[name]
        require(type(row.get("sample_count")) is int and row["sample_count"] == count
            and type(row.get("episode_count")) is int and row["episode_count"] == episodes, "actual TRAIN full denominator differs: " + name)
    require(cohorts["v4"].get("statistics", {}).get("anchor_count") == 327
        and cohorts["LEFT"].get("conversion_sha256") == LEFT_CONVERSION_SHA
        and d.get("training_dataset_fingerprint_sha256") == json_sha(cohorts), "actual347 admission/four cohort fingerprints differ")


def verify_owner(binding, d, weights_sha, metadata_sha, authority_sha):
    require(type(binding) is dict and binding.get("schema") == OWNER_SCHEMA and binding.get("mode") == "RESEARCH_ONLY"
        and type(binding.get("owner_id")) is str and binding["owner_id"].strip()
        and binding.get("completed_run_authority_sha256") == authority_sha
        and binding.get("model_promoted") is False and binding.get("vehicle_control_approved") is False,
        "separate externally sealed research owner required")
    if binding.get("purpose") == "offline_cpu_numeric_validation_only":
        require(binding.get("simulator_control_authorized") is False, "numeric validation cannot actuate")
    else:
        require(binding.get("purpose") == "owned_carla_research_only" and binding.get("simulator_control_authorized") is True
            and binding.get("simulation_only") is True and is_sha(binding.get("owner_plan_sha256")), "prospective owned simulator-only authority required")
    require(binding.get("weights_sha256") == weights_sha and binding.get("metadata_sha256") == metadata_sha
        and binding.get("checkpoint_sha256") == d["checkpoint_sha256"] and binding.get("declaration_sha256") == d["declaration_sha256"]
        and type(binding.get("fixed_endpoint_updates")) is int and binding["fixed_endpoint_updates"] == 20000,
        "owner binds exact final20k only; no observed-best selection")


def verify_history(payload, declaration, d):
    rows = [json.loads(line, object_pairs_hook=parent_api.unique_json_object,
        parse_float=finite_json_number, parse_constant=invalid_json_constant) for line in payload.splitlines()]
    samples = declaration.get("fixed_sample_ids", [])
    require(len(rows) == len(samples) == 20000 and json_sha(samples) == d["fixed_sample_ids_sha256"], "all20k/80k actual sample journal required")
    indices, left_counts = [], [0] * 347
    for i, row in enumerate(rows):
        step = i + 1; ids = row.get("indices", []); actual = row.get("sample_ids", [])
        require(type(row.get("optimizer_updates")) is int and row["optimizer_updates"] == step
            and type(row.get("examples_seen")) is int and row["examples_seen"] == 4 * step
            and type(ids) is list and len(ids) == 4 and all(type(v) is int for v in ids)
            and 0 <= ids[0] < 2718 and 2718 <= ids[1] < 3045 and 3045 <= ids[2] < 5832 and 5832 <= ids[3] < 6179
            and type(actual) is list and len(actual) == 4 and all(type(v) is str and v for v in actual) and actual == samples[i]
            and is_sha(row.get("batch_sha256")) and row.get("learning_rates") == {g: learning_rate(step, g) for g in ("shared", "heads")}
            and row.get("legacy_curved") is (step % 2 == 1) and row.get("recovery_focus") is (step % 2 == 1), "actual fixed indices/samples/LRs differ")
        require(all(type(row.get(k)) in (int, float) and math.isfinite(row[k]) and row[k] >= 0
            for k in ("loss", "preclip_gradient_norm", "elapsed_fit_seconds")), "actual finite gradient/update receipt required")
        indices.append(ids); left_counts[ids[3] - 5832] += 1
    require(json_sha(indices) == declaration.get("fixed_schedule_sha256") == d["fixed_schedule_sha256"]
        and min(left_counts) == 57 and max(left_counts) == 58 and left_counts.count(58) == 221,
        "all347 fair full-cycle exposures and complete fixed schedule required")
    return dict(actual_updates=20000, actual_examples_seen=80000, all347_fair_exposure_counts=left_counts)


def completed_run(path, sha, d, root, parent):
    authority = read_json(path, sha)
    require(authority.get("schema") == AUTHORITY_SCHEMA and authority.get("mode") == "RESEARCH_ONLY"
        and authority.get("fixed_endpoint_completed") is True and authority.get("model_promoted") is False
        and authority.get("vehicle_control_approved") is False and authority.get("parent") == PARENT
        and authority.get("parent_history_sha256") == PARENT_HISTORY_SHA and authority.get("source_sha256") == d["source_sha256"]
        and authority.get("new_LEFT_conversion_sha256") == LEFT_CONVERSION_SHA, "ROOT-sealed completed20k run required")
    require(all(type(authority.get(k)) is str and Path(authority[k]).is_absolute() for k in ("run", "parent_run")),
        "external completed run and GOODparent paths must be absolute")
    run = canonical(authority["run"]); evidence = {str(path): (sha, 4 * 1024**2)}
    def document(relative, pin, maximum=4 * 1024**2):
        expected = authority[pin]; require(is_sha(expected), "external actual artifact SHA required")
        target = run / relative; evidence[str(target)] = (expected, maximum)
        return read_json(target, expected, maximum)
    summary = document("summary.json", "summary_sha256")
    fit = document("fit_result.json", "fit_result_sha256")
    declaration = document("declaration.json", "declaration_sha256", 16 * 1024**2)
    opened = document("evaluation_opened_after_final.json", "evaluation_boundary_sha256")
    require(summary.get("schema") == TRAINING_SCHEMA and summary.get("status") == "FIXED20000_ACTUAL_LEFT_JOINT_COMPLETE_NOT_PROMOTED"
        and summary.get("fit") == fit and summary.get("all71_original_screens_retained") is True and summary.get("full_LEFT347_retained") is True
        and summary.get("model_promoted") is False and summary.get("vehicle_control_approved") is False
        and summary.get("endpoint_diagnostic_screens", {}).get("diagnostic_screen_count") == 74
        and len(summary["endpoint_diagnostic_screens"].get("checks", [])) == 74, "full fixed postfit diagnostic completion required, not all-PASS promotion")
    require(type(fit.get("optimizer_updates")) is int and fit["optimizer_updates"] == 20000 and fit.get("examples_seen") == 80000
        and fit.get("completed_fixed_endpoint") is True and fit.get("DEV_or_HOLDOUT_read") is False
        and fit.get("model_promoted") is False and [r["optimizer_updates"] for r in fit.get("checkpoints", [])] == list(range(1000, 20001, 1000)),
        "all20 fixed checkpoint receipts/actual20k updates required")
    require(declaration.get("schema") == TRAINING_SCHEMA and declaration.get("protocol") == d["protocol"]
        and declaration.get("source_sha256") == d["source_sha256"] and declaration.get("cohorts") == d["cohorts"]
        and declaration.get("authorization_sha256") == d["authorization_sha256"] == authority.get("training_authorization_sha256")
        and json_sha(declaration["protocol"]) == authority.get("protocol_sha256") and d["declaration_sha256"] == authority["declaration_sha256"],
        "actual declaration/protocol/membership/source authority differs")
    sources = dict(parent._research_temporal_export_metadata["source_sha256"], **parent_api.ADDED_TRAINING_SOURCES, **{TRAINER_PATH: TRAINER_SHA})
    require(len(sources) == 30 and sources == d["source_sha256"], "all26 GOODparent plus3 frozen dependencies plus NEW trainer required")
    for name, expected in sources.items():
        relative = Path(name); require(not relative.is_absolute() and ".." not in relative.parts and str(relative) == name, "canonical source key required")
        for target in (root / relative, run / "source" / relative):
            evidence[str(target)] = (expected, 16 * 1024**2); pinned(target, expected, 16 * 1024**2)
    history_sha = authority.get("history_sha256"); require(history_sha == d["history_sha256"] and is_sha(history_sha), "externally pinned actual20k history required")
    history_path = run / "history.jsonl"; evidence[str(history_path)] = (history_sha, 128 * 1024**2)
    journal = verify_history(pinned(history_path, history_sha, 128 * 1024**2), declaration, d)
    pins = authority.get("endpoint", {})
    require(type(pins) is dict and set(pins) == set(PARENT) and all(is_sha(v) for v in pins.values())
        and pins["weights_sha256"] == d["npz_sha256"] and pins["checkpoint_sha256"] == d["checkpoint_sha256"]
        and pins["declaration_sha256"] == d["declaration_sha256"]
        and all(fit["checkpoints"][-1][k] == pins[v] for k, v in (("npz_sha256", "weights_sha256"), ("metadata_sha256", "metadata_sha256"), ("checkpoint_sha256", "checkpoint_sha256"))),
        "external four final20k pins must match actual checkpoint receipt")
    require(all(opened.get("endpoint", {}).get(k) is True for k in ("all101_numeric_finite", "frozen47_byte_identical", "all20000_actual_journal_rows_verified"))
        and all(opened["endpoint"].get(k) == pins[v] for k, v in (("npz_sha256", "weights_sha256"),
            ("metadata_sha256", "metadata_sha256"), ("checkpoint_sha256", "checkpoint_sha256")))
        and opened.get("checkpoint_selection") is False, "actual numeric/journal endpoint must precede evaluation opening")
    export = run / "exports/step_020000"
    for target, expected, maximum in ((export / "model_state.npz", pins["weights_sha256"], 32 * 1024**2),
        (export / "metadata.json", pins["metadata_sha256"], 4 * 1024**2), (run / "checkpoints/step_020000.pt", pins["checkpoint_sha256"], 128 * 1024**2)):
        evidence[str(target)] = (expected, maximum); pinned(target, expected, maximum)  # HH_261002 - PT is byte-hashed only, never deserialized.
    return authority, export, evidence, journal


def load_left_joint_research_npz(weights, metadata, *, weights_sha256, metadata_sha256, current_parent_weights,
        current_parent_metadata, warm_parent_weights, warm_parent_metadata, backbone_parent_weights, backbone_parent_metadata,
        completed_run_authority, completed_run_authority_sha256, research_owner_binding, source_root=ROOT):
    d = read_json(metadata, metadata_sha256); verify_scope(d)
    verify_owner(research_owner_binding, d, weights_sha256, metadata_sha256, completed_run_authority_sha256)
    root = canonical(source_root)
    parent = parent_api.load_current_parent(current_parent_weights, current_parent_metadata,
        warm_parent_weights=warm_parent_weights, warm_parent_metadata=warm_parent_metadata,
        backbone_parent_weights=backbone_parent_weights, backbone_parent_metadata=backbone_parent_metadata, source_root=root)
    pd = parent._research_temporal_export_metadata; warm = read_json(warm_parent_metadata, pd["parent_metadata_sha256"])
    ancestors = ((current_parent_weights, PARENT["weights_sha256"], 32 * 1024**2), (current_parent_metadata, PARENT["metadata_sha256"], 4 * 1024**2),
        (warm_parent_weights, pd["parent_npz_sha256"], 32 * 1024**2), (warm_parent_metadata, pd["parent_metadata_sha256"], 4 * 1024**2),
        (backbone_parent_weights, warm["parent_npz_sha256"], 32 * 1024**2), (backbone_parent_metadata, warm["parent_metadata_sha256"], 4 * 1024**2))
    authority, export, evidence, journal = completed_run(completed_run_authority, completed_run_authority_sha256, d, root, parent)
    parent_run = canonical(authority["parent_run"])
    require(canonical(current_parent_weights) == parent_run / "exports/step_006000/model_state.npz"
        and canonical(current_parent_metadata) == parent_run / "exports/step_006000/metadata.json", "actual original GOODparent run paths required")
    for target, sha, maximum in ((parent_run / "history.jsonl", PARENT_HISTORY_SHA, 128 * 1024**2),
        (parent_run / "declaration.json", PARENT["declaration_sha256"], 16 * 1024**2),
        (parent_run / "checkpoints/step_006000.pt", PARENT["checkpoint_sha256"], 128 * 1024**2)):
        evidence[str(target)] = (sha, maximum); pinned(target, sha, maximum)
    require(canonical(weights) == export / "model_state.npz" and canonical(metadata) == export / "metadata.json"
        and authority["endpoint"]["weights_sha256"] == weights_sha256 and authority["endpoint"]["metadata_sha256"] == metadata_sha256
        and d["model_config"] == parent.config.to_dict() and d["recovery_config"] == parent.recovery_config.to_dict(), "only externally bound fixed20k/config endpoint may load")
    state = parent_api.numeric_state(weights, d, parent.state_dict())
    model = parent_api.old.RouteRecoveryTemporalModelV3(parent.config, parent.recovery_config).eval()
    model.load_state_dict(state, strict=True); model.requires_grad_(False)
    model._research_temporal_checkpoint_sha256 = d["checkpoint_sha256"]
    model._research_temporal_export_npz_sha256 = weights_sha256; model._research_temporal_metadata_sha256 = metadata_sha256
    model._research_temporal_export_metadata = d; model._research_recovery_owner_binding = dict(research_owner_binding)
    model._research_recovery_numeric_loader_schema = SCHEMA
    model._research_recovery_simulator_control_authorized = research_owner_binding["simulator_control_authorized"]
    model._research_left_completed_run_authority_sha256 = completed_run_authority_sha256
    model._research_left_actual_journal_evidence = journal
    for path, (sha, maximum) in evidence.items(): pinned(path, sha, maximum)
    for path, sha, maximum in ancestors: pinned(path, sha, maximum)
    return model
