"""HH_261002 - Synthetic TRAIN-only mining/sampler contracts; no actual arrays, inference, GPU or fit."""

import copy
import hashlib
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"tools/e2e"))
import map_train_hardnegative_inventory_sampler_20261002 as study


def row(index, matrix):
    actual, prediction, diagonal = matrix.sum(1), matrix.sum(0), np.diag(matrix)
    return dict(index=index, sample_id=study.slot_id(index), source_group=study.source_group(index),
        split="TRAIN", appearance="FALSE_OR_ORIGINAL", full_FOV=True,
        rgb_sha256="a"*64, target_sha256="b"*64, parent_prediction_sha256="c"*64,
        confusion=matrix.tolist(), target_class_pixels=actual.tolist(), predicted_class_pixels=prediction.tolist(),
        sign_target_pixels=int(actual[5]), sign_true_positive_pixels=int(diagonal[5]),
        sign_false_negative_pixels=int(actual[5]-diagonal[5]), sign_false_positive_pixels=int(prediction[5]-diagonal[5]),
        other_to_sign_false_positive_pixels=int(matrix[0, 5]))


@pytest.fixture(scope="module")
def records():
    # HH_261002 - Pure synthetic confusion rows preserve the real declared census without reading any label payload.
    remaining = np.asarray(study.CLASS_PIXELS, dtype=np.int64).copy()
    result = []
    for i in range(study.TOTAL):
        counts = np.zeros(7, dtype=np.int64)
        if i < 2:
            counts[5] = (31, 128)[i]
            counts[0] = study.PIXELS-counts[5]
        else:
            left = study.PIXELS
            for k in range(7):
                counts[k] = min(int(remaining[k]), left)
                left -= int(counts[k])
            assert left == 0
        remaining -= counts
        assert (remaining >= 0).all()
        matrix = np.diag(counts)
        if i == 2:
            matrix[0, 0] -= 7; matrix[0, 5] += 7
        result.append(row(i, matrix))
    assert remaining.sum() == 0
    return result


@pytest.fixture(scope="module")
def inventory(records):
    return study.inventory(records)


def quick_crops(seed):
    assert seed == study.SEED
    crops = np.tile(np.asarray([0, 0, 360, 640], dtype=np.int16), (10000, 8, 1))
    return np.zeros((10000, 8), dtype=np.int64), crops


def test_default_contract_no_fit_or_site_role_grant():
    value = study.prepared_contract()
    assert value["status"] == "PREPARED_CODE_ONLY_NO_MINING_OR_FIT_EXECUTED"
    assert value["no_fit_entrypoint"] and value["mining_requires_external_exact_authorization"]
    assert value["parent_architecture_parameter_count"] == 245831
    assert not value["source_code_can_grant_new_site_roles"] and not value["training_authorized"]
    assert not value["class_weight_causality_proven"] and not value["loss_or_crop_changed"]
    assert value["initial_model_weights_sha256"] == value["parent_weights_sha256"] == study.INITIAL_MODEL_SHA
    assert value["dataset_origin_parent_weights_sha256"] == study.PARENT_SHA != study.INITIAL_MODEL_SHA
    assert value["initial_model_completed_updates"] == 10000 and value["total_lineage_updates_per_future_arm"] == 14000


def test_full_frame_statistics_true_labels_and_parent_errors():
    target = np.zeros((360, 640), dtype=np.uint8)
    pred = target.copy()
    target.flat[:31] = 5; pred.flat[:23] = 5
    pred.flat[100:107] = 5
    value = study.frame_statistics(target, pred)
    assert value["sign_true_positive_pixels"] == 23
    assert value["sign_false_negative_pixels"] == 8
    assert value["sign_false_positive_pixels"] == value["other_to_sign_false_positive_pixels"] == 7
    assert sum(value["target_class_pixels"]) == study.PIXELS
    assert np.array_equal(target.flat[:31], np.full(31, 5, dtype=np.uint8))


@pytest.mark.parametrize("kind", ["dtype", "shape", "class", "float_nan"])
def test_numeric_frame_malformed_rejected(kind):
    target = np.zeros((360, 640), dtype=np.uint8)
    pred = target.copy()
    if kind == "dtype": pred = pred.astype(np.int64)
    if kind == "shape": pred = pred[:359]
    if kind == "class": pred[0, 0] = 7
    if kind == "float_nan": pred = np.full((360, 640), np.nan)
    with pytest.raises(ValueError): study.frame_statistics(target, pred)


def test_actual_strata_exact_count_and_no_fake_site_claim(inventory):
    assert study.validate_inventory(inventory) == inventory
    assert inventory["strata"]["small_sign"] == [0]
    assert 1 in inventory["strata"]["large_sign"]
    assert inventory["strata"]["hard_negative"] == [2]
    assert inventory["physical_train_slots"] == 1209
    assert not inventory["independent_objects_or_sites_inferred"]
    assert not inventory["training_authorized"] and not inventory["labels_changed"]
    assert np.asarray(inventory["confusion"]).sum(1).tolist() == study.CLASS_PIXELS


@pytest.mark.parametrize("index, expected", [(0, "Town01_original497"), (496, "Town01_original497"),
    (497, "Town02_original232"), (728, "Town02_original232"), (729, "Town01_p02_FALSE240"),
    (968, "Town01_p02_FALSE240"), (969, "Town01_p04_FALSE240"), (1208, "Town01_p04_FALSE240")])
def test_source_group_fixed_actual_capture_boundaries(index, expected):
    # HH_261002 - Assert independently named source blocks so a mistaken map label cannot self-validate.
    assert study.source_group(index) == expected


@pytest.mark.parametrize("index, wrong", [(729, "Town02_p02_FALSE240"), (969, "Town02_p04_FALSE240")])
def test_wrong_town02_paired_provenance_rejected(records, index, wrong):
    bad = copy.deepcopy(records); bad[index]["source_group"] = wrong
    with pytest.raises(ValueError): study.inventory(bad)


@pytest.mark.parametrize("kind", ["VAL", "TEST", "HOLDOUT", "TRUE", "reordered", "duplicate", "missing", "TP_fabricated", "bool_count", "changed_census"])
def test_no_split_view_membership_or_label_counter_leakage(records, kind):
    bad = copy.deepcopy(records)
    if kind in ("VAL", "TEST", "HOLDOUT"): bad[0]["split"] = kind
    if kind == "TRUE": bad[0]["appearance"] = "TRUE"
    if kind == "reordered": bad[0], bad[1] = bad[1], bad[0]
    if kind == "duplicate": bad[1] = copy.deepcopy(bad[0])
    if kind == "missing": bad.pop()
    if kind == "TP_fabricated": bad[0]["sign_true_positive_pixels"] += 1
    if kind == "bool_count": bad[0]["sign_true_positive_pixels"] = True
    if kind == "changed_census":
        matrix = np.asarray(bad[0]["confusion"]); matrix[0, 0] -= 1; matrix[1, 1] += 1
        bad[0] = row(0, matrix)
    with pytest.raises(ValueError): study.inventory(bad)


@pytest.mark.parametrize("field, value", [("training_authorized", True), ("known_DEV_or_HOLDOUT_mined", True),
                                         ("parent_weights_sha256", "d"*64), ("appearance_TRUE_not_mined", False)])
def test_inventory_cannot_grant_fit_or_alter_parent(inventory, field, value):
    bad = copy.deepcopy(inventory); bad[field] = value
    with pytest.raises(ValueError): study.validate_inventory(bad)


def test_empty_negative_stratum_retained_and_fail_closed(records):
    bad = copy.deepcopy(records)
    matrix = np.asarray(bad[2]["confusion"]); matrix[0, 0] += 7; matrix[0, 5] -= 7
    bad[2] = row(2, matrix)
    inv = study.inventory(bad)
    assert inv["status"] == "TRAIN_MINING_COMPLETE_STRATA_UNAVAILABLE_NO_SCHEDULE"
    assert len(inv["records"]) == 1209
    with pytest.raises(ValueError): study.schedules(inv)


def test_full_cycle_all_tails_no_replacement_within_cycle():
    ids = np.arange(7, dtype=np.int64)
    actual = study.full_cycle(ids, 103, 23)
    assert np.array_equal(actual, study.full_cycle(ids, 103, 23))
    for begin in range(0, 98, 7): assert np.array_equal(np.sort(actual[begin:begin+7]), ids)
    assert len(actual[98:]) == 5 and len(np.unique(actual[98:])) == 5
    assert not np.array_equal(actual, study.full_cycle(ids, 103, 24))


@pytest.mark.parametrize("ids,count,seed", [([], 8, 1), ([1, 1], 8, 1), ([-1, 2], 8, 1), ([True, False], 8, 1),
                                          ([1.0, 2.0], 8, 1), ([0, 1], True, 1), ([0, 1], 8, True), ([0, 1], 8, 2**32)])
def test_invalid_full_cycle_rejected(ids, count, seed):
    with pytest.raises(ValueError): study.full_cycle(ids, count, seed)


def test_exact_quota_all_train_replay_counts_and_finite_indices(inventory, monkeypatch):
    monkeypatch.setattr(study.data_contract.frozen, "schedule", quick_crops)
    arrays = study.schedules(inventory)
    proof = study.schedule_proof(inventory, arrays)
    control, candidate = arrays["control_indices"], arrays["candidate_indices"]
    assert control.shape == candidate.shape == (4000, 8)
    assert np.array_equal(candidate[:, :4].ravel(), control.ravel()[:16000])
    assert (candidate[:, 4] == 0).all() and (candidate[:, 6:] == 2).all()
    assert np.isin(candidate[:, 5], inventory["strata"]["large_sign"]).all()
    assert np.bincount(candidate[:, :4].ravel(), minlength=1209).min() >= 13
    assert np.bincount(control.ravel(), minlength=1209).min() >= 26
    assert proof["candidate_hard_negative_exposures"] == 8000
    assert proof["candidate_global_exposures"] == 16000 and proof["examples_per_arm"] == 32000
    assert proof["same_anchor_can_repeat_across_declared_slot_roles"]
    assert not arrays["shared_FALSE_view_bits"].any()
    assert not proof["training_authorized"] and not proof["model_promoted"]
    assert sum(proof["exposures"]["candidate_indices"]) == 32000


def test_actual_frozen_crop_prefix_unchanged(inventory):
    arrays = study.schedules(inventory)
    _, original = study.data_contract.frozen.schedule(study.SEED)
    assert np.array_equal(arrays["shared_crops"], original[:4000])
    assert arrays["shared_crops"].dtype == np.int16


@pytest.mark.parametrize("kind", ["index", "dtype", "crop", "view", "missing", "extra"])
def test_schedule_mutation_rejected(inventory, monkeypatch, kind):
    monkeypatch.setattr(study.data_contract.frozen, "schedule", quick_crops)
    arrays = study.schedules(inventory)
    if kind == "index": arrays["candidate_indices"][0, 6] = 3
    if kind == "dtype": arrays["control_indices"] = arrays["control_indices"].astype(np.int32)
    if kind == "crop": arrays["shared_crops"][0, 0, 0] = 1
    if kind == "view": arrays["shared_FALSE_view_bits"][0, 0] = 1
    if kind == "missing": arrays.pop("shared_crops")
    if kind == "extra": arrays["test_indices"] = arrays["control_indices"].copy()
    with pytest.raises(ValueError): study.schedule_proof(inventory, arrays)


def authorization(source):
    return dict(schema=study.AUTHORIZATION_SCHEMA, study_id=study.NAME,
        operation="TRAIN_ONLY_PARENT_NUMERIC_FORWARD_AND_INVENTORY_NO_FIT", source_sha256=source,
        data_manifest_sha256=study.DATA_MANIFEST_SHA, data_registry_sha256=study.DATA_REGISTRY_SHA,
        parent_weights_sha256=study.INITIAL_MODEL_SHA, dataset_origin_parent_weights_sha256=study.PARENT_SHA,
        initial_model_weights_sha256=study.INITIAL_MODEL_SHA, initial_model_path=str(study.INITIAL_MODEL_PATH),
        initial_model_receipt_sha256=dict(study.INITIAL_RECEIPT_SHA), output=str(study.OUTPUT), reviewer="synthetic-fixture-only",
        research_mining_authorized=True, fit_authorized=False, known_DEV_or_HOLDOUT_access=False,
        native_denials_preserved=True, vehicle_control_approved=False, automatic_model_promotion=False)


@pytest.mark.parametrize("field", ["source_sha256", "data_manifest_sha256", "parent_weights_sha256", "initial_model_weights_sha256",
                                 "dataset_origin_parent_weights_sha256", "initial_model_path", "initial_model_receipt_sha256", "fit_authorized",
                                 "known_DEV_or_HOLDOUT_access", "native_denials_preserved", "operation", "output"])
def test_declared_mining_no_scope_widening(field):
    source = {"synthetic": "a"*64}
    value = authorization(source)
    study.validate_authorization(value, source)
    value[field] = True if field in ("fit_authorized", "known_DEV_or_HOLDOUT_access") else False
    with pytest.raises(ValueError): study.validate_authorization(value, source)


def initial_lineage_fixture():
    training = dict(steps=10000, examples_seen=80000, parameter_count=245831, weights_sha256=study.INITIAL_MODEL_SHA,
        checkpoint_sha256="da56f5113b60ec1188ab6a53b1e78f7bacdde70b140021bfd50f2507c873617f",
        full_exposure_journal_sha256="c1f232cd863135bb1e44d76f88802f1e020f822fb2633d76f8d02fa329b5ca98")
    fit = dict(status="BOTH_FIXED_FINAL10000_ENDPOINTS_SAVED_BEFORE_DEV", arms=dict(control=copy.deepcopy(training)))
    bundle = dict(schema="portable_e2e.map_paired_balanced_source.20261002.v1", study_id="map_paired_balanced_20261002_v1",
        data_manifest_sha256=study.DATA_MANIFEST_SHA, study_registry_sha256=study.DATA_REGISTRY_SHA, files=dict(study.FROZEN_SOURCE_SHA))
    return training, fit, bundle


def test_current_control10000_lineage_separate_from_data_origin():
    values = initial_lineage_fixture(); study.validate_initial_lineage(*values)
    assert study.INITIAL_MODEL_PATH == study.INITIAL_RUN/"control/weights.npz"
    assert str(study.INITIAL_RUN).endswith("runs/diagnostics/map_paired_balanced_20261002_v1")
    assert set(study.INITIAL_RECEIPT_SHA) == {"control/training.json", "fit_complete.json", "summary.json", "source/bundle.json"}


@pytest.mark.parametrize("kind", ["old_origin_weights", "balanced_other_weights", "wrong_checkpoint", "bool_updates", "short_updates",
    "wrong_exposure", "wrong_fit_status", "mismatched_final_receipt", "wrong_source", "wrong_manifest", "wrong_registry"])
def test_current_control_parent_or_completed_receipt_tamper_rejected(kind):
    training, fit, bundle = initial_lineage_fixture()
    if kind == "old_origin_weights": training["weights_sha256"] = study.PARENT_SHA
    elif kind == "balanced_other_weights": training["weights_sha256"] = "a2eca2f5becc316f1b4ae34799d7def899e6d5426a2b33286357833c3703a57c"
    elif kind == "wrong_checkpoint": training["checkpoint_sha256"] = "a"*64
    elif kind == "bool_updates": training["steps"] = True
    elif kind == "short_updates": training["steps"] = 9999
    elif kind == "wrong_exposure": training["examples_seen"] -= 1
    elif kind == "wrong_fit_status": fit["status"] = "RUNNING"
    elif kind == "mismatched_final_receipt": fit["arms"]["control"]["weights_sha256"] = "a"*64
    elif kind == "wrong_source": bundle["files"][next(iter(bundle["files"]))] = "a"*64
    elif kind == "wrong_manifest": bundle["data_manifest_sha256"] = "a"*64
    else: bundle["study_registry_sha256"] = "a"*64
    with pytest.raises(ValueError): study.validate_initial_lineage(training, fit, bundle)


def test_actual_mining_rejected_locally_before_opening_arrays(monkeypatch):
    monkeypatch.setattr(study.os, "uname", lambda: type("Host", (), {"nodename": "synthetic-local"})())
    with pytest.raises(ValueError, match="personal sourceworktree"):
        study.mine(type("Args", (), {"authorization": "/must/not/open", "authorization_sha256": "a"*64})())


def test_frozen_numeric_sources_are_unmodified():
    paths = {name: ROOT/"tools/e2e"/name for name in study.FROZEN_SOURCE_SHA}
    for name in study.data_contract.ARCHITECTURE_SHA:
        paths[name] = ROOT/"learning/portable_e2e"/name
    for name, expected in study.FROZEN_SOURCE_SHA.items():
        assert hashlib.sha256(paths[name].read_bytes()).hexdigest() == expected
