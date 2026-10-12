"""HH_261002 - Prospective architecture-only, matched original-CE final20k protocol.

Contract/runtime queries authorize no TRAIN payload reading or optimizer update.
All old sources and denied native captures remain untouched.  Known350 is an
already exposed diagnostic, not new validation, signal-state or HD-map approval.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
from types import FunctionType

import numpy as np

try:
    from . import train_map_hardnegative_continuation_20261002 as previous
    from . import train_map_sign_margin_repair_20261002 as screens
except ImportError:
    import train_map_hardnegative_continuation_20261002 as previous
    import train_map_sign_margin_repair_20261002 as screens

mining, data = previous.mining, previous.contract
require = data.require
NAME = "map_native_fusion_pair_20261002_v1"
SCHEMA = "portable_e2e.map_native_fusion_pair.20261002.v1"
EXPORT_SCHEMA = SCHEMA + ".numeric_map_endpoint"
AUTHORIZATION_SCHEMA, SCHEDULE_SCHEMA = SCHEMA + ".authorization", SCHEMA + ".equal_global_schedule"
WORKSPACE, DATA = mining.WORKSPACE, mining.DATA
SOURCE, OUTPUT = WORKSPACE / "source_worktrees" / NAME, WORKSPACE / "runs/diagnostics" / NAME
STEPS, BATCH, SEED, TOTAL = 20000, 8, mining.SEED, 1209
ARMS = ("control", "candidate")
PARAMETERS = {"parent": 245831, "control": 245831, "candidate": 397175}
PAIR_BUDGET_SECONDS, EVALUATION_BUDGET_SECONDS = 4 * 3600, 1800
FROZEN_SOURCE_SHA = {**screens.FROZEN_SOURCE_SHA,
    "train_map_sign_margin_repair_20261002.py": "3ede51218b97e4172a9685b349b727ef9bef0c1a6e2c555b44ca94cc15e4d43f",
    "map_sign_margin_loss_v1_20261002.py": "50029d77d4d216833a04ea140757a4f7b5d6cb8b183c9685bedacb05411e4286"}
NEW_SOURCES = {"map_native_fusion_v1_20261002.py", Path(__file__).name, "train_map_native_fusion_pair_20261002.py"}
SOURCE_NAMES = set(FROZEN_SOURCE_SHA) | NEW_SOURCES


def document_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def prepared_contract(loader_workers=0):
    require(type(loader_workers) is int and loader_workers in (0, 2), "exact loader0 or loader2 required")
    return dict(schema=SCHEMA, study_id=NAME, source=str(SOURCE), output=str(OUTPUT),
        source_count=13, arms=list(ARMS), arm_order=list(ARMS),
        initial_model_path=str(mining.INITIAL_MODEL_PATH), initial_model_weights_sha256=mining.INITIAL_MODEL_SHA,
        initial_model_receipt_sha256=dict(mining.INITIAL_RECEIPT_SHA), initial_model_state_sha256=previous.INITIAL_STATE_SHA,
        initial_completed_updates=10000, additional_updates_per_arm=STEPS, total_lineage_updates_per_arm=30000,
        shared_parent_completed_updates_after_fit=30000,
        candidate_new_branch_initial_completed_updates=0, candidate_new_branch_final_completed_updates=STEPS,
        parameter_count_by_arm=dict(PARAMETERS), candidate_new_parameters=151344,
        sole_intervention="zero_output_half_resolution_native48_residual64_dilation1_2_4_before_shared7class_classifier",
        input_rgb_shape=[360, 640, 3], labels=list(mining.LABELS), new_head=False, new_inputs=False,
        original_max_stride=4, new_residual_stride=2, upsampling="deterministic_ConvTranspose2d_then_original_style_crop",
        actual_first_TRAIN_initial_logits_bit_equal_required=True, shared_parent_state_bytes_equal_required=True,
        zero_initial_upstream_gradients_valid_but_all_trainable_gradients_present_finite_required=True,
        physical_TRAIN_slots=TOTAL, original_slots=729, FALSE_paired_slots=480,
        source_group_counts=dict(Town01_original=497, Town02_original=232, Town01_p02_FALSE=240, Town01_p04_FALSE=240),
        stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145),
        data_manifest_sha256=mining.DATA_MANIFEST_SHA, data_registry_sha256=mining.DATA_REGISTRY_SHA,
        original_inventory_sha256=dict(previous.MINING_SHA), class_pixels=list(mining.CLASS_PIXELS),
        class_weights=list(mining.CLASS_WEIGHTS), seed=SEED, batch=BATCH, examples_per_arm=STEPS * BATCH,
        full_cycles_per_arm=132, final_partial_cycle=412, sampler="identical_original_full_cycle_global_1209",
        crop="same_frozen_apply_crop_and_seed_law_both_arms; native360x640; no_sign_selection",
        FALSE_views_only=True, loader_workers=loader_workers, loader_start_method="spawn_if2; CPU_only",
        strict_loader0_loader2_first_two_TRAIN_batches_byte_parity_required=True,
        semantic_loss="unchanged_original_seven_class_weighted_pixel_CE_only_both_arms",
        auxiliary_loss=False, optimizer="fresh AdamW each arm", learning_rate=3e-5, weight_decay=1e-4,
        gradient_norm_cap=5., both_strict_CUDA_forward_backward_preflights_before_any_optimizer_update=True,
        physical_GPU_index=0, GPU_UUID=mining.GPU_UUID, exclusive_original_GPU0_lease=True,
        GPU_idle_precondition="exact_UUID_memory0MiB_util0_and_no_compute_process_before_CUDA",
        pair_fit_wall_seconds=PAIR_BUDGET_SECONDS, post_fit_evaluation_wall_seconds=EVALUATION_BUDGET_SECONDS,
        final_selection="both_fixed_final20000_only_no_early_best_resume_retry",
        BOTH_numeric_endpoints_and_all20000_matched_journals_verified_before_known_payload=True,
        numeric_export_schema=EXPORT_SCHEMA, checkpoint_pickle_loading=False, driving101_export=False,
        known_evaluation_files={k: dict(count=v[0], sha256=v[1]) for k, v in previous.previous.EVAL_FILES.items()},
        all_existing_27_screens_unchanged=True, known350_previously_exposed_not_independent=True,
        native_denials_preserved=True, DEV_or_HOLDOUT_used_for_fit=False,
        inference_latency_measured=False, hundred_ms_or_live_mapping_approved=False,
        theoretical_conv_MAC_native_RGB=dict(parent=6547046400, additional=8670412800, total=15217459200),
        training_authorized=False, model_promoted=False, vehicle_control_approved=False,
        main_capabilities_approved=0, subfunctions_approved=0)


def source_snapshot(root):
    data.canonical(root)
    result = {k: data.digest(root / k) for k in sorted(SOURCE_NAMES)}
    require(len(result) == 13 and all(result[k] == v for k, v in FROZEN_SOURCE_SHA.items()),
            "exact TEN frozen and THREE new source bytes required")
    return result


def schedule():
    # HH_261002 - Clone only the declared length; original schedule code, seed, crop law and module globals stay unchanged.
    old = data.frozen.schedule
    namespace = dict(old.__globals__, STEPS=STEPS, BATCH=BATCH, TOTAL_COUNT=TOTAL)
    indices, crops = FunctionType(old.__code__, namespace, old.__name__, old.__defaults__)(SEED)
    return dict(control_indices=indices, candidate_indices=indices.copy(), shared_crops=crops,
                shared_FALSE_view_bits=np.zeros((STEPS, BATCH), dtype=np.uint8))


def schedule_proof(values):
    expected = schedule()
    require(type(values) is dict and set(values) == set(expected), "exact matched schedule tensors required")
    for key, ref in expected.items():
        actual = values[key]
        require(isinstance(actual, np.ndarray) and actual.dtype == ref.dtype and actual.shape == ref.shape
                and np.array_equal(actual, ref), "full20k schedule ABI/seed/bytes differ: " + key)
    flat = values["control_indices"].ravel()
    return dict(schema=SCHEDULE_SCHEMA, seed=SEED, steps=STEPS, batch=BATCH,
        tensor_abi={k: dict(shape=list(v.shape), dtype=str(v.dtype)) for k, v in values.items()},
        tensor_sha256={k: mining.array_sha(v) for k, v in values.items()},
        sample_exposures=np.bincount(flat, minlength=TOTAL).tolist(), full_cycles=flat.size // TOTAL,
        last_partial_cycle=flat.size % TOTAL, total_exposures_per_arm=int(flat.size),
        native_denials_preserved=True, CTRL_and_candidate_exact_same_input_schedule=True)


def validate_authorization(value, source, proof, loader_workers):
    # HH_261002 - Compare Python JSON directly: float5.0 must not be silently reserialized as int5 by another runtime.
    require(type(source) is dict and set(source) == SOURCE_NAMES and all(sha(v) for v in source.values())
        and all(source[k] == v for k, v in FROZEN_SOURCE_SHA.items()), "exact thirteen-source identity required")
    require(type(value) is dict and value.get("schema") == AUTHORIZATION_SCHEMA
        and value.get("source_sha256") == source and len(source) == 13
        and json.dumps(value.get("contract"), sort_keys=True, allow_nan=False)
        == json.dumps(prepared_contract(loader_workers), sort_keys=True, allow_nan=False)
        and value.get("protocol_sha256") == document_sha(prepared_contract(loader_workers))
        and value.get("equal_global_schedule_sha256") == document_sha(proof), "exact prospective architecture/source/schedule authority required")
    for key, expected in (("research_fit_authorized", True), ("native_denials_preserved", True),
        ("DEV_or_HOLDOUT_used_for_fit", False), ("automatic_model_promotion", False), ("vehicle_control_approved", False)):
        require(value.get(key) is expected, "authorization scope differs: " + key)
    require(type(value.get("reviewer")) is str and len(value["reviewer"].strip()) >= 3, "actual external reviewer required")
    screens.validate_runtime_descriptor(value.get("runtime_identity"))


def environment_contract():
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == WORKSPACE / "venvs/py312"
        and Path.cwd() == SOURCE and Path(__file__).resolve().parent == SOURCE and sys.flags.optimize == 0,
        "canonical private flat source/existing personal py312/assertions required")
    expected = {"CUDA_VISIBLE_DEVICES": mining.GPU_UUID, "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
        "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "16", "MKL_NUM_THREADS": "16"}
    require(all(os.environ.get(k) == v for k, v in expected.items())
        and not any(os.environ.get(k) for k in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP")),
        "strict single physicalGPU0 deterministic environment required")
    for path in (SOURCE, DATA, previous.MINING_OUTPUT, mining.INITIAL_RUN, OUTPUT.parent):
        data.canonical(path); require(path.is_dir(), "canonical private directory required")
    data.canonical(OUTPUT); require(not OUTPUT.exists(), "fresh output only; no retry/resume/overwrite")
    mining.reserve(OUTPUT)


def verify_imports():
    for module in (previous, screens, mining, data, data.frozen, previous.previous, screens.losses):
        require(Path(module.__file__).resolve() == SOURCE / Path(module.__file__).name, "import escaped sealed flat source package")


def train_package():
    inventory, old_schedule, _ = previous.load_inventory()
    mining.initial_lineage()
    manifest, arrays, _, _, _ = data.load_package(DATA, mining.DATA_MANIFEST_SHA, mining.DATA_REGISTRY_SHA)
    require(manifest["common_class_pixels"] == mining.CLASS_PIXELS
        and manifest["common_class_weights"] == mining.CLASS_WEIGHTS, "original TRAIN CE weights differ")
    previous.verify_train_identity(inventory, arrays)
    values = schedule()
    require(np.array_equal(values["control_indices"][:4000], old_schedule["control_indices"])
        and np.array_equal(values["shared_crops"][:4000], old_schedule["shared_crops"]),
        "original global sampler/crop prefix differs")
    return manifest, arrays, values, schedule_proof(values)


def postcheck(source, manifest, authorization_path, authorization_sha, runtime):
    require(source_snapshot(SOURCE) == source, "source closure changed")
    data.payload(authorization_path, authorization_sha)
    data.payload(DATA / "manifest.json", mining.DATA_MANIFEST_SHA)
    data.payload(DATA / "study_train_registry.json", mining.DATA_REGISTRY_SHA)
    for name, row in manifest["files"].items(): data.payload(DATA / name, row["sha256"])
    for name, expected in previous.MINING_SHA.items(): data.payload(previous.MINING_OUTPUT / name, expected)
    mining.initial_lineage()
    require(screens.runtime_identity() == runtime, "current executable/crop/Torch providers changed")


def original_27_screen(evaluation):
    result = screens.diagnostic_screen(evaluation)
    require(len(result["checks"]) == 27, "original27 denominator differs")
    return result
