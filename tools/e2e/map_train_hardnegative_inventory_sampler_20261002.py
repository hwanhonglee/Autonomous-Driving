#!/usr/bin/env python3
"""HH_261002 - Prepare one TRAIN-only parent-error sampler intervention; no fit or site-role grant.

The default contract command reads no arrays and initializes no CUDA.  The mine
command is a separately declared GPU0 numeric forward, never an optimizer job.
Known DEV/HOLDOUT and protected map payloads are not opened.  Paired TRUE bytes
are verified by the frozen loader, but never passed to the mining network.
A successful inventory is evidence, not authorization for a later fit.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
try:
    from . import map_paired_balanced_contract_20261002 as data_contract
except ImportError:
    import map_paired_balanced_contract_20261002 as data_contract

require = data_contract.require
NAME = "map_train_hardnegative_20261002_v1"
INVENTORY_SCHEMA = "portable_e2e.map_train_parent_error_inventory.20261002.v1"
AUTHORIZATION_SCHEMA = "portable_e2e.map_train_parent_error_mining_authorization.20261002.v1"
SAMPLER_SCHEMA = "portable_e2e.map_train_hardnegative_sampler.20261002.v1"
DATA_MANIFEST_SHA = "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b"
DATA_REGISTRY_SHA = "ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d"
PARENT_SHA = data_contract.PARENT_SHA
INITIAL_MODEL_SHA = "2284511d2b28c273ab731fcbc7b1326c8e040f4c1d84d57faacf2df86afd8310"
LABELS = data_contract.LABELS
TOTAL, ORIGINAL, STEPS, BATCH, SEED = 1209, 729, 4000, 8, 2026100203
SMALL_MAX = 64
PIXELS = 360 * 640
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE = WORKSPACE / "source_worktrees" / NAME
DATA = WORKSPACE.parent / "dataset/map_paired_balanced_20261002_v1"
INITIAL_RUN = WORKSPACE / "runs/diagnostics/map_paired_balanced_20261002_v1"
INITIAL_MODEL_PATH = INITIAL_RUN / "control/weights.npz"
INITIAL_RECEIPT_SHA = {
    "control/training.json": "161649bee1416e485bb1dd13f780291fa05dee7565c3c5d8af593083174a991a",
    "fit_complete.json": "71e5cb0623c759bb3a17bdcbed8601521c976cbf4fdb8aa1a8d5c48ff143c83e",
    "summary.json": "fa9749d67b27eb8169473350289ba61530f324736c61bb3a269adc6b070027d0",
    "source/bundle.json": "2f30504c9380008fe0d6f17893749dfd1f377b39fbb398b4aad926a8da5afcdd",
}
OUTPUT = WORKSPACE / "runs/diagnostics" / NAME
MIN_FREE_BYTES, WRITE_RESERVE_BYTES = 5 * 1024**3, 512 * 1024**2
MAX_MINING_SECONDS = 900
FROZEN_SOURCE_SHA = {
    "map_paired_balanced_contract_20261002.py": "9ceab19d111c56bda27b53cdc4bb4e39b42380d0ade7e3eb2dc618d12a028d69",
    "map_appearance_matched_contract_20261001.py": "9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96",
    "train_map_paired_balanced_20261002.py": "3773493fd141afdb50dbf467e9d8622fd74a4a4474c83266fa99a1c72352e5be",
    "train_map_photometric_pilot.py": "4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484",
    **data_contract.ARCHITECTURE_SHA,
}
SOURCE_NAMES = set(FROZEN_SOURCE_SHA) | {Path(__file__).name}
CLASS_PIXELS = [114760064, 114816346, 46160650, 664319, 319234, 86973, 1746014]
CLASS_WEIGHTS = [0.13018396496772766, 0.1301521360874176, 0.20489764213562012,
                 1.4379799365997314, 1.80595064163208, 2.3095099925994873, 0.9813257455825806]


def slot_id(index):
    require(type(index) is int and 0 <= index < TOTAL, "exact TRAIN physical-slot integer required")
    if index < ORIGINAL:
        return f"original_train729:row/{index:04d}"
    paired = index - ORIGINAL
    return f"{('p02', 'p04')[paired // 240]}:frames/{paired % 240:04d}"


def source_group(index):
    # HH_261002 - Preserve source-array groups; a frame/group is not an independent physical sign object.
    require(type(index) is int and 0 <= index < TOTAL, "source slot out of bounds")
    # HH_261002 - Bind paired p02/p04 provenance to the actual Town01 TRAIN captures, not the Town02 original block.
    return "Town01_original497" if index < 497 else "Town02_original232" if index < 729 else "Town01_p02_FALSE240" if index < 969 else "Town01_p04_FALSE240"


def array_sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def frame_statistics(target, prediction):
    """HH_261002 - Count actual labels and parent full-FOV predictions without pseudo-label replacement."""
    for value in (target, prediction):
        require(isinstance(value, np.ndarray) and value.shape == (360, 640)
                and value.dtype == np.uint8 and int(value.max()) < 7, "exact seven-class uint8 full-FOV ABI required")
    matrix = np.bincount((target.astype(np.int64) * 7 + prediction).ravel(), minlength=49).reshape(7, 7)
    actual, proposed, diagonal = matrix.sum(1), matrix.sum(0), np.diag(matrix)
    return dict(confusion=matrix.tolist(), target_class_pixels=actual.tolist(),
        predicted_class_pixels=proposed.tolist(), sign_target_pixels=int(actual[5]),
        sign_true_positive_pixels=int(diagonal[5]), sign_false_negative_pixels=int(actual[5]-diagonal[5]),
        sign_false_positive_pixels=int(proposed[5]-diagonal[5]), other_to_sign_false_positive_pixels=int(matrix[0, 5]))


def validate_record(row, index):
    require(type(row) is dict and row.get("index") == index and type(row["index"]) is int
            and row.get("sample_id") == slot_id(index) and row.get("source_group") == source_group(index)
            and row.get("split") == "TRAIN" and row.get("appearance") == "FALSE_OR_ORIGINAL"
            and row.get("full_FOV") is True, "TRAIN identity/view/split differs")
    for name in ("rgb_sha256", "target_sha256", "parent_prediction_sha256"):
        value = row.get(name)
        require(type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value), "exact tensor SHA required")
    values = row.get("confusion")
    require(type(values) is list and len(values) == 7 and all(type(v) is list and len(v) == 7 for v in values)
            and all(type(v) is int and v >= 0 for line in values for v in line), "integer full49 confusion required")
    matrix = np.asarray(values, dtype=np.int64)
    require(int(matrix.sum()) == PIXELS, "full pixel denominator differs")
    actual, proposed, diagonal = matrix.sum(1), matrix.sum(0), np.diag(matrix)
    expected = dict(target_class_pixels=actual.tolist(), predicted_class_pixels=proposed.tolist(),
        sign_target_pixels=int(actual[5]), sign_true_positive_pixels=int(diagonal[5]),
        sign_false_negative_pixels=int(actual[5]-diagonal[5]), sign_false_positive_pixels=int(proposed[5]-diagonal[5]),
        other_to_sign_false_positive_pixels=int(matrix[0, 5]))
    require(all(type(row.get(k)) is type(v) and row[k] == v for k, v in expected.items()), "actual TP/FN/FP counts differ")


def inventory(records):
    require(type(records) is list and len(records) == TOTAL, "all1209 physical TRAIN slots required")
    for index, row in enumerate(records):
        validate_record(row, index)
    strata = dict(small_sign=[], large_sign=[], hard_negative=[])
    for row in records:
        count = row["sign_target_pixels"]
        if 0 < count <= SMALL_MAX: strata["small_sign"].append(row["index"])
        if count > SMALL_MAX: strata["large_sign"].append(row["index"])
        if count == 0 and row["other_to_sign_false_positive_pixels"] > 0:
            strata["hard_negative"].append(row["index"])
    matrix = np.asarray([r["confusion"] for r in records], dtype=np.int64).sum(0)
    require(matrix.sum(1).tolist() == CLASS_PIXELS, "exact fixed1209 TRAIN class-pixel census required")
    group_counts = {name: {group: sum(records[i]["source_group"] == group for i in ids)
        for group in sorted({r["source_group"] for r in records})} for name, ids in strata.items()}
    scope = dict(schema=INVENTORY_SCHEMA, study_id=NAME, parent_weights_sha256=INITIAL_MODEL_SHA,
        dataset_origin_parent_weights_sha256=PARENT_SHA, initial_model_weights_sha256=INITIAL_MODEL_SHA,
        initial_model_path=str(INITIAL_MODEL_PATH), initial_model_completed_updates=10000,
        initial_model_receipt_sha256=dict(INITIAL_RECEIPT_SHA),
        data_manifest_sha256=DATA_MANIFEST_SHA, data_registry_sha256=DATA_REGISTRY_SHA,
        labels=list(LABELS), physical_train_slots=TOTAL, original_slots=ORIGINAL, paired_FALSE_slots=480,
        records=records, confusion=matrix.tolist(), strata=strata,
        stratum_frame_counts={k: len(v) for k, v in strata.items()}, stratum_source_array_group_counts=group_counts,
        small_sign_full_frame_pixel_bounds=[1, SMALL_MAX], large_sign_minimum_full_frame_pixels=65,
        hard_negative_definition="GTsign=0 and actual pinned-parent predicted sign on truth-other pixels>0",
        parent_sign_TP_and_FN_all_recorded_not_positive_selection_filter=True,
        appearance_TRUE_not_mined=True, labels_changed=False, known_DEV_or_HOLDOUT_mined=False,
        source_denials_preserved=True, independent_objects_or_sites_inferred=False,
        training_authorized=False, model_promoted=False, vehicle_control_approved=False)
    scope["strata_available"] = all(bool(v) for v in strata.values())
    scope["status"] = "TRAIN_MINING_COMPLETE_NOT_FIT_AUTHORIZED" if scope["strata_available"] else "TRAIN_MINING_COMPLETE_STRATA_UNAVAILABLE_NO_SCHEDULE"
    return scope


def validate_inventory(value):
    require(type(value) is dict and "records" in value, "inventory document required")
    expected = inventory(value["records"])
    require(all(value.get(k) == v and type(value.get(k)) is type(v) for k, v in expected.items()), "mining inventory contract differs")
    return expected


def full_cycle(ids, required, seed):
    """HH_261002 - Preserve complete shuffled cycles and the last partial cycle; no error-ranked dropping."""
    require(type(required) is int and required > 0 and type(seed) is int and 0 <= seed <= 2**32-1, "bounded count/seed required")
    group = np.asarray(ids)
    require(group.ndim == 1 and group.size > 0 and np.issubdtype(group.dtype, np.integer)
            and group.dtype != np.bool_ and bool((group >= 0).all()) and len(np.unique(group)) == len(group), "nonempty unique integer stratum required")
    rng = np.random.default_rng(seed)
    return np.concatenate([rng.permutation(group).astype(np.int64) for _ in range((required+len(group)-1)//len(group))])[:required]


def schedules(value):
    """HH_261002 - Sole change is 4global/1small/1large/2hard-negative index exposure, not loss, pixels or crop law."""
    checked = validate_inventory(value)
    require(checked["strata_available"] is True, "all actual TRAIN strata required; no DEV supplementation or fake samples")
    control = full_cycle(np.arange(TOTAL, dtype=np.int64), STEPS*BATCH, SEED).reshape(STEPS, BATCH)
    candidate = np.empty_like(control)
    candidate[:, :4] = full_cycle(np.arange(TOTAL, dtype=np.int64), STEPS*4, SEED).reshape(STEPS, 4)
    for name, column, count, seed in (("small_sign", 4, 1, SEED+10), ("large_sign", 5, 1, SEED+11), ("hard_negative", 6, 2, SEED+12)):
        candidate[:, column:column+count] = full_cycle(np.asarray(checked["strata"][name], dtype=np.int64), STEPS*count, seed).reshape(STEPS, count)
    # HH_261002 - Reuse byte-identical first4000 frozen crop draws in BOTH arms; no crop reroll to retain a sign.
    _, frozen_crops = data_contract.frozen.schedule(SEED)
    return dict(control_indices=control, candidate_indices=candidate, shared_crops=frozen_crops[:STEPS].copy(),
        shared_FALSE_view_bits=np.zeros((STEPS, BATCH), dtype=np.uint8))


def schedule_proof(value, arrays):
    expected = schedules(value)
    require(type(arrays) is dict and set(arrays) == set(expected), "exact schedule arrays required")
    for name, wanted in expected.items():
        got = arrays[name]
        require(isinstance(got, np.ndarray) and got.dtype == wanted.dtype and got.shape == wanted.shape
                and np.array_equal(got, wanted), "declared schedule differs: "+name)
    return dict(schema=SAMPLER_SCHEMA, steps_per_arm=STEPS, batch=BATCH, examples_per_arm=STEPS*BATCH,
        candidate_slot_roles=["all_TRAIN", "all_TRAIN", "all_TRAIN", "all_TRAIN", "small_sign", "large_sign", "hard_negative", "hard_negative"],
        candidate_global_exposures=STEPS*4, candidate_small_exposures=STEPS, candidate_large_exposures=STEPS,
        candidate_hard_negative_exposures=STEPS*2, control_full_cycles=STEPS*BATCH//TOTAL,
        candidate_global_full_cycles=STEPS*4//TOTAL, every_TRAIN_slot_replayed=True,
        same_anchor_can_repeat_across_declared_slot_roles=True, repeats_are_exposures_not_new_data=True,
        complete_stratum_cycles_and_partial_tails_retained=True, post_crop_sign_presence_does_not_trigger_reroll=True,
        tensor_sha256={k: array_sha(v) for k, v in arrays.items()},
        exposures={k: np.bincount(arrays[k].ravel(), minlength=TOTAL).tolist() for k in ("control_indices", "candidate_indices")},
        optimization=dict(learning_rate=3e-5, weight_decay=1e-4, optimizer="fresh AdamW in each future arm",
            identical_initial_model_weights_sha256=INITIAL_MODEL_SHA,
            initial_model_completed_updates=10000, additional_updates=STEPS, total_lineage_updates_per_arm=14000,
            parameter_count=245831, gradient_norm_cap=5., checkpoint_selection="fixed_final4000_only"),
        common_class_pixels=list(CLASS_PIXELS), common_class_weights=list(CLASS_WEIGHTS),
        semantic_loss_unchanged=True, architecture_unchanged=True, crop_function_unchanged=True,
        source_denials_preserved=True, independent_new_dev_sites=0, future_fit_requires_separate_declaration=True,
        training_authorized=False, model_promoted=False, vehicle_control_approved=False)


def prepared_contract():
    return dict(schema=SAMPLER_SCHEMA, status="PREPARED_CODE_ONLY_NO_MINING_OR_FIT_EXECUTED", study_id=NAME,
        sole_intervention="TRAIN physical-slot exposure sampler only", parent_weights_sha256=INITIAL_MODEL_SHA,
        dataset_origin_parent_weights_sha256=PARENT_SHA, initial_model_weights_sha256=INITIAL_MODEL_SHA,
        initial_model_path=str(INITIAL_MODEL_PATH), initial_model_receipt_sha256=dict(INITIAL_RECEIPT_SHA),
        initial_model_completed_updates=10000, additional_updates_per_future_arm=STEPS, total_lineage_updates_per_future_arm=14000,
        input_data_manifest_sha256=DATA_MANIFEST_SHA, input_registry_sha256=DATA_REGISTRY_SHA,
        parent_architecture_parameter_count=245831, input_train_slots=TOTAL, mined_RGB_views="original729 + pairedFALSE480 ONLY",
        labels=list(LABELS), training_steps_per_arm=STEPS, batch=BATCH,
        candidate_quota="4 full-TRAIN cycle +1 small sign +1 large sign +2 parent FP-on-other sign-negative cycle",
        class_weights=list(CLASS_WEIGHTS), class_weight_causality_proven=False, loss_or_crop_changed=False,
        known_DEV118_or_HOLDOUT232_or_CTrack_payload_used=False, source_code_can_grant_new_site_roles=False,
        no_fit_entrypoint=True, mining_requires_external_exact_authorization=True,
        fresh_independent_site_count=0, training_authorized=False, model_promoted=False, vehicle_control_approved=False)


def reserve(path):
    data_contract.canonical(path)
    existing = next(p for p in (path, *path.parents) if p.exists())
    require(shutil.disk_usage(existing).free >= MIN_FREE_BYTES+WRITE_RESERVE_BYTES, "5GiB plus512MiB write reserve required")


def source_snapshot(root):
    data_contract.canonical(root)
    result = {name: data_contract.digest(root/name) for name in sorted(SOURCE_NAMES)}
    require(all(result[name] == value for name, value in FROZEN_SOURCE_SHA.items()), "frozen parent/helper source changed")
    return result


def validate_authorization(value, source):
    require(type(value) is dict and value.get("schema") == AUTHORIZATION_SCHEMA and value.get("study_id") == NAME
        and value.get("operation") == "TRAIN_ONLY_PARENT_NUMERIC_FORWARD_AND_INVENTORY_NO_FIT"
        and value.get("source_sha256") == source and value.get("data_manifest_sha256") == DATA_MANIFEST_SHA
        and value.get("data_registry_sha256") == DATA_REGISTRY_SHA and value.get("parent_weights_sha256") == INITIAL_MODEL_SHA
        and value.get("dataset_origin_parent_weights_sha256") == PARENT_SHA
        and value.get("initial_model_weights_sha256") == INITIAL_MODEL_SHA and value.get("initial_model_path") == str(INITIAL_MODEL_PATH)
        and value.get("initial_model_receipt_sha256") == INITIAL_RECEIPT_SHA
        and value.get("output") == str(OUTPUT), "exact prospective source/data/parent/output mining declaration required")
    for name, wanted in (("research_mining_authorized", True), ("fit_authorized", False),
                         ("known_DEV_or_HOLDOUT_access", False), ("native_denials_preserved", True),
                         ("vehicle_control_approved", False), ("automatic_model_promotion", False)):
        require(value.get(name) is wanted, "mining declaration scope differs: "+name)
    require(type(value.get("reviewer")) is str and len(value["reviewer"].strip()) >= 3, "actual reviewer required")


def validate_initial_lineage(training, fit, bundle):
    """HH_261002 - Mine/continue the actual completed10000 control model; never reset to the dataset's older origin model."""
    require(type(training) is dict and type(fit) is dict and type(bundle) is dict,
        "completed current-control receipts required")
    for key, expected in (("steps", 10000), ("examples_seen", 80000), ("parameter_count", 245831)):
        require(type(training.get(key)) is int and training[key] == expected, "completed current-control count differs: "+key)
    require(training.get("weights_sha256") == INITIAL_MODEL_SHA
        and training.get("checkpoint_sha256") == "da56f5113b60ec1188ab6a53b1e78f7bacdde70b140021bfd50f2507c873617f"
        and training.get("full_exposure_journal_sha256") == "c1f232cd863135bb1e44d76f88802f1e020f822fb2633d76f8d02fa329b5ca98"
        and fit.get("status") == "BOTH_FIXED_FINAL10000_ENDPOINTS_SAVED_BEFORE_DEV"
        and fit.get("arms", {}).get("control") == training, "fixed completed control endpoint lineage differs")
    require(bundle.get("schema") == "portable_e2e.map_paired_balanced_source.20261002.v1"
        and bundle.get("study_id") == "map_paired_balanced_20261002_v1"
        and bundle.get("data_manifest_sha256") == DATA_MANIFEST_SHA
        and bundle.get("study_registry_sha256") == DATA_REGISTRY_SHA
        and all(bundle.get("files", {}).get(name) == sha for name, sha in FROZEN_SOURCE_SHA.items()),
        "current-control exact data/source bundle differs")


def initial_lineage():
    # HH_261002 - Hash the summary receipt without opening known evaluation prediction/label payloads or using scores for mining.
    for relative, sha in INITIAL_RECEIPT_SHA.items(): data_contract.payload(INITIAL_RUN/relative, sha)
    data_contract.payload(INITIAL_MODEL_PATH, INITIAL_MODEL_SHA)
    training = data_contract.read_json(INITIAL_RUN/"control/training.json", INITIAL_RECEIPT_SHA["control/training.json"])
    fit = data_contract.read_json(INITIAL_RUN/"fit_complete.json", INITIAL_RECEIPT_SHA["fit_complete.json"])
    bundle = data_contract.read_json(INITIAL_RUN/"source/bundle.json", INITIAL_RECEIPT_SHA["source/bundle.json"])
    validate_initial_lineage(training, fit, bundle)


def mining_environment():
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == WORKSPACE/"venvs/py312"
        and Path.cwd() == SOURCE and Path(__file__).resolve().parent == SOURCE, "fresh personal sourceworktree and existing py312 required")
    required = {"CUDA_VISIBLE_DEVICES": GPU_UUID, "CUBLAS_WORKSPACE_CONFIG": ":4096:8", "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "16", "MKL_NUM_THREADS": "16"}
    require(all(os.environ.get(k) == v for k, v in required.items())
        and not any(os.environ.get(k) for k in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP")), "strict private physicalGPU0 environment required")
    for path in (SOURCE, DATA, OUTPUT.parent): data_contract.canonical(path)
    data_contract.canonical(OUTPUT)
    require(not OUTPUT.exists(), "fresh mining output only; no overwrite or retry")
    reserve(OUTPUT)


def mine(args):
    """HH_261002 - Optional ROOT-owned future GPU0 forward; no optimizer, DEV opening, approval or model promotion."""
    mining_environment()
    source = source_snapshot(SOURCE)
    declaration_bytes, declaration_sha = data_contract.payload(Path(args.authorization), args.authorization_sha256)
    authorization = json.loads(declaration_bytes)
    validate_authorization(authorization, source)
    initial_lineage()
    manifest, arrays, _, _, _ = data_contract.load_package(DATA, DATA_MANIFEST_SHA, DATA_REGISTRY_SHA)
    require(manifest["common_class_pixels"] == CLASS_PIXELS and manifest["common_class_weights"] == CLASS_WEIGHTS, "unchanged fixed TRAIN weights required")
    import torch
    import train_map_photometric_pilot as guard
    import train_map_paired_balanced_20261002 as previous_runner
    import map_context_recovery as model_module
    for module in (guard, previous_runner, model_module):
        require(Path(module.__file__).resolve().parent == SOURCE, "mining dependency escaped pinned sourceworktree")
    torch.set_num_threads(16); torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
    with guard.gpu0_lease():
        gpu = guard.preflight_gpu()
        model = model_module.MapContextRecovery()
        previous_runner.load_weights(model, INITIAL_MODEL_PATH, INITIAL_MODEL_SHA)
        require(sum(p.numel() for p in model.parameters()) == 245831, "exact original model architecture required")
        model.to("cuda:0").eval()
        initial_state_sha = guard.state_digest(model)
        reserve(OUTPUT); OUTPUT.mkdir()
        for name in source: guard.copy_new(SOURCE/name, OUTPUT/"source"/name)
        guard.copy_new(Path(args.authorization), OUTPUT/"authorization.json")
        guard.copy_new(INITIAL_MODEL_PATH, OUTPUT/"initial_model_weights.npz")
        for relative in INITIAL_RECEIPT_SHA: guard.copy_new(INITIAL_RUN/relative, OUTPUT/"initial_lineage"/relative)
        guard.write_new(OUTPUT/"started.json", dict(schema=INVENTORY_SCHEMA+".started", created_at_utc=datetime.now(timezone.utc).isoformat(),
            gpu=gpu, authorization_sha256=declaration_sha, source_sha256=source, initial_parent_state_sha256=initial_state_sha,
            initial_model_weights_sha256=INITIAL_MODEL_SHA, dataset_origin_parent_weights_sha256=PARENT_SHA,
            initial_model_completed_updates=10000, initial_model_receipt_sha256=dict(INITIAL_RECEIPT_SHA),
            physical_train_slots=TOTAL, no_optimizer=True, no_DEV_or_HOLDOUT_opened=True))
        predictions = np.empty((TOTAL, 360, 640), dtype=np.uint8)
        records, deadline = [], time.monotonic()+MAX_MINING_SECONDS
        original_rgb, original_target, pair_false, _, pair_target = arrays
        with (OUTPUT/"records.jsonl").open("x") as log, torch.no_grad():
            for begin in range(0, TOTAL, BATCH):
                require(time.monotonic() < deadline, "15minute bounded TRAIN mining forward exhausted")
                indices = list(range(begin, min(TOTAL, begin+BATCH)))
                images = [original_rgb[i] if i < ORIGINAL else pair_false[i-ORIGINAL] for i in indices]
                x = torch.tensor(np.stack(images).transpose(0, 3, 1, 2).copy(), dtype=torch.float32, device="cuda:0")/255
                logits = model(x)
                require(logits.shape == (len(indices), 7, 360, 640) and bool(torch.isfinite(logits).all()), "finite parent-logit fullFOV ABI differs")
                predicted = logits.argmax(1).cpu().numpy().astype(np.uint8)
                for j, i in enumerate(indices):
                    target = original_target[i] if i < ORIGINAL else pair_target[i-ORIGINAL]
                    predictions[i] = predicted[j]
                    row = dict(index=i, sample_id=slot_id(i), source_group=source_group(i), split="TRAIN", appearance="FALSE_OR_ORIGINAL", full_FOV=True,
                        rgb_sha256=array_sha(images[j]), target_sha256=array_sha(target), parent_prediction_sha256=array_sha(predicted[j]), **frame_statistics(target, predicted[j]))
                    records.append(row); log.write(json.dumps(row, allow_nan=False)+"\n")
                log.flush()
        require(guard.state_digest(model) == initial_state_sha, "parent weights changed during numeric mining")
        result = inventory(records)
        result.update(authorization_sha256=declaration_sha, source_sha256=source, no_optimizer=True,
            actual_parent_state_sha256=initial_state_sha, physical_gpu_index=0, gpu_uuid=GPU_UUID)
        reserve(OUTPUT)
        with (OUTPUT/"parent_TRAIN_predictions.npz").open("xb") as stream: np.savez_compressed(stream, prediction=predictions)
        if result["strata_available"]:
            planned = schedules(result)
            with (OUTPUT/"proposed_schedule.npz").open("xb") as stream: np.savez_compressed(stream, **planned)
            result["sampler_proof"] = schedule_proof(result, planned)
        require(source_snapshot(SOURCE) == source, "live source changed during mining")
        initial_lineage()
        require(data_contract.digest(OUTPUT/"initial_model_weights.npz") == INITIAL_MODEL_SHA, "archived initial numeric model changed")
        for relative, sha in INITIAL_RECEIPT_SHA.items(): require(data_contract.digest(OUTPUT/"initial_lineage"/relative) == sha, "archived current-model lineage changed")
        for name, sha in source.items(): require(data_contract.digest(OUTPUT/"source"/name) == sha, "archived mining source changed")
        for name, ref in manifest["files"].items(): require(data_contract.digest(DATA/name) == ref["sha256"], "TRAIN input changed during mining")
        registry = data_contract.read_json(DATA/"study_train_registry.json", DATA_REGISTRY_SHA)
        for name, sha in registry["copied_provenance_files"].items(): require(data_contract.digest(DATA/name) == sha, "source provenance changed during mining")
        require(data_contract.digest(DATA/"manifest.json") == DATA_MANIFEST_SHA and data_contract.digest(DATA/"paired_samples.json") == manifest["paired_samples_sha256"]
            and data_contract.digest(Path(args.authorization)) == declaration_sha and data_contract.digest(OUTPUT/"authorization.json") == declaration_sha,
            "metadata/declaration changed during mining")
        result.update(parent_TRAIN_prediction_file_sha256=data_contract.digest(OUTPUT/"parent_TRAIN_predictions.npz"),
            record_journal_sha256=data_contract.digest(OUTPUT/"records.jsonl"), source_and_inputs_unchanged=True)
        guard.write_new(OUTPUT/"inventory.json", result)
        print(json.dumps(dict(status=result["status"], output=str(OUTPUT), stratum_frame_counts=result["stratum_frame_counts"], fit_authorized=False)), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "mine"), default="contract")
    parser.add_argument("--authorization"); parser.add_argument("--authorization-sha256")
    args = parser.parse_args(argv)
    if args.mode == "contract": print(json.dumps(prepared_contract(), indent=2, allow_nan=False)); return 0
    require(args.authorization is not None and args.authorization_sha256 is not None, "mine requires externally pinned prospective authorization")
    mine(args); return 0


if __name__ == "__main__": raise SystemExit(main())
