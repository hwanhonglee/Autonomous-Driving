"""HH_261001 - Test synthetic numeric fixtures, not trained-model quality or control authorization."""
from io import BytesIO
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np
import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e import temporal_balanced_export_loader_research_20261001 as loader
from portable_e2e import temporal_simulation_research_v2_20261001 as frozen_live


@pytest.fixture(scope="module", autouse=True)
def one_thread():
    before = torch.get_num_threads(); torch.set_num_threads(1)
    yield
    torch.set_num_threads(before)


@pytest.fixture(scope="module")
def numeric_arrays():
    # HH_261001 - Random synthetic state shares the exact ABI but never masquerades as actual endpoint evidence.
    model = CachedTemporalModelV2(TemporalModelConfigV2()).eval()
    return {name: value.detach().numpy().copy() for name, value in model.state_dict().items()}


def document(arrays):
    return dict(schema=loader.EXPORT_SCHEMA, model_config=TemporalModelConfigV2().to_dict(),
        checkpoint_sha256="a"*64, declaration_sha256="b"*64, training_dataset_fingerprint_sha256="c"*64,
        parent_checkpoint_sha256=loader.PARENT_SHA256, parent_source_optimizer_updates=200,
        known_total_optimizer_updates=3200, optimizer_updates=3000, maximum_optimizer_updates=3000,
        examples_seen=12000, v4_examples_seen=6000, legacy_examples_seen=6000, training_episode_count=9,
        batch_domains=list(loader.BATCH_DOMAINS), training_only=True, simulation_only=True,
        one_episode_screening=False, research_screening=True, mixed_raw_joint_training=True,
        main_training=False, model_promoted=False, vehicle_control_approved=False, independent_validation=False,
        stop_intent_trained=False, raw_spatial_only=False, decoder_integrated=True, legacy_decoded=False,
        labels_clipped=False, labels_extrapolated=False, inference_loader_authorized=False,
        single_episode_export_compatibility_claimed=False, runtime_intent="explicit_cruise_GO_not_head",
        source_sha256=dict(loader.TRAINING_SOURCE_PINS),
        state_tensor_shapes={k: list(v.shape) for k, v in arrays.items()},
        state_tensor_dtypes={k: str(v.dtype) for k, v in arrays.items()},
        numerical_revision=dict(kernel_id="fixed33_to257_align_corners_dense_gemm.20261001.v1",
            strict_deterministic_algorithms=True, tf32_matrix_multiplication=False,
            finite_precision_bitwise_equivalence_claimed=False,
            frozen_decoder_sha256=loader.TRAINING_SOURCE_PINS["learning/portable_e2e/path_time_decoder_v2.py"],
            source_sha256={k: loader.TRAINING_SOURCE_PINS[k] for k in loader.NUMERICAL_SOURCE_KEYS}))


def owner(description, weights_sha, metadata_sha):
    return dict(schema=loader.OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id="synthetic_unit_test",
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, weights_sha256=weights_sha,
        metadata_sha256=metadata_sha, checkpoint_sha256=description["checkpoint_sha256"],
        declaration_sha256=description["declaration_sha256"], fixed_endpoint_updates=3000)


def export(tmp_path, arrays, description=None):
    npz, meta = tmp_path/"weights.npz", tmp_path/"metadata.json"
    np.savez(npz, **arrays)
    d = document(arrays) if description is None else description
    d["npz_sha256"] = hashlib.sha256(npz.read_bytes()).hexdigest()
    meta.write_text(json.dumps(d))
    return npz, meta, d


def load(npz, meta, description, binding=None):
    weights_sha, metadata_sha = hashlib.sha256(npz.read_bytes()).hexdigest(), hashlib.sha256(meta.read_bytes()).hexdigest()
    return loader.load_balanced_research_npz(npz, meta, weights_sha256=weights_sha,
        metadata_sha256=metadata_sha, research_owner_binding=owner(description, weights_sha, metadata_sha) if binding is None else binding)


def test_exact_mixed_numeric_cpu_load_preserves_false_authority_and_original_metadata(tmp_path, numeric_arrays, monkeypatch):
    npz, meta, d = export(tmp_path, numeric_arrays); before = meta.read_bytes()
    monkeypatch.setattr(torch, "load", lambda *a, **kw: (_ for _ in ()).throw(AssertionError("no Torch pickle permitted")))
    model = load(npz, meta, d)
    assert isinstance(model, CachedTemporalModelV2)
    assert model.training is False and all(not p.requires_grad for p in model.parameters())
    assert all(v.device.type == "cpu" and v.dtype == torch.float32 for v in model.state_dict().values())
    assert all(np.array_equal(model.state_dict()[k].numpy(), v) for k, v in numeric_arrays.items())
    assert model._research_temporal_export_metadata["one_episode_screening"] is False
    assert model._research_temporal_export_metadata["inference_loader_authorized"] is False
    assert model._research_balanced_simulator_control_authorized is False
    assert model._research_balanced_numeric_owner_binding["purpose"] == "offline_cpu_numeric_validation_only"
    assert meta.read_bytes() == before


def test_original_single_episode_loader_still_rejects_mixed_export(tmp_path, numeric_arrays):
    npz, meta, _ = export(tmp_path, numeric_arrays)
    with pytest.raises(ContractError, match="identity or research"):
        frozen_live.load_npz(npz, meta, weights_sha256=hashlib.sha256(npz.read_bytes()).hexdigest(),
            metadata_sha256=hashlib.sha256(meta.read_bytes()).hexdigest())


@pytest.mark.parametrize("key,value", [("optimizer_updates", 500), ("known_total_optimizer_updates", 3000),
    ("parent_source_optimizer_updates", 3000), ("training_episode_count", 1), ("training_episode_count", True),
    ("examples_seen", 6000), ("v4_examples_seen", 12000), ("legacy_examples_seen", 0),
    ("maximum_optimizer_updates", 4000), ("one_episode_screening", True),
    ("mixed_raw_joint_training", False), ("inference_loader_authorized", True),
    ("model_promoted", True), ("vehicle_control_approved", True), ("legacy_decoded", True),
    ("labels_clipped", True), ("schema", frozen_live.DETERMINISTIC_EXPORT_SCHEMA),
    ("batch_domains", ["v4_joint"]*4), ("parent_checkpoint_sha256", "0"*64)])
def test_scope_and_lineage_reject_early_checkpoint_or_falsified_flags(numeric_arrays, key, value):
    d = document(numeric_arrays); d["npz_sha256"] = "d"*64; d[key] = value
    with pytest.raises(ContractError):loader.verify_export_scope(d, "d"*64)


@pytest.mark.parametrize("mutation", ["missing", "extra", "wrong_trainer", "wrong_kernel", "tf32", "bitwise"])
def test_exact17_parent_and_numerical_source_pins_required(numeric_arrays, mutation):
    d = document(numeric_arrays); d["npz_sha256"] = "d"*64
    if mutation == "missing":d["source_sha256"].pop("tools/e2e/train_temporal_v2_raw_bootstrap_20261001.py")
    if mutation == "extra":d["source_sha256"]["unused.py"] = "0"*64
    if mutation == "wrong_trainer":d["source_sha256"]["tools/e2e/train_temporal_v2_balanced_replay_20261001.py"] = "0"*64
    if mutation == "wrong_kernel":d["numerical_revision"]["kernel_id"] = "other"
    if mutation == "tf32":d["numerical_revision"]["tf32_matrix_multiplication"] = True
    if mutation == "bitwise":d["numerical_revision"]["finite_precision_bitwise_equivalence_claimed"] = True
    with pytest.raises(ContractError):loader.verify_export_scope(d, "d"*64)


@pytest.mark.parametrize("key,value", [("mode", "APPROVED"), ("simulator_control_authorized", True),
    ("purpose", "drive_vehicle"), ("owner_id", ""), ("weights_sha256", "0"*64),
    ("metadata_sha256", "0"*64), ("checkpoint_sha256", "0"*64),
    ("declaration_sha256", "0"*64), ("fixed_endpoint_updates", 500)])
def test_external_owner_binding_is_exact_numeric_research_only(numeric_arrays, key, value):
    d = document(numeric_arrays); binding = owner(d, "d"*64, "e"*64); binding[key] = value
    with pytest.raises(ContractError):loader.verify_owner_binding(binding, d, "d"*64, "e"*64)


def test_duplicate_json_metadata_key_is_rejected():
    with pytest.raises(ContractError, match="duplicate"):
        json.loads('{"schema":"first","schema":"last"}', object_pairs_hook=loader.unique_json_object)


@pytest.mark.parametrize("mutation", ["missing_key", "extra_key", "wrong_shape", "float64", "nan", "shape_boolean"])
def test_full_numeric_abi_rejects_bad_npz_even_with_caller_hashes(tmp_path, numeric_arrays, mutation):
    arrays = dict(numeric_arrays); key = next(k for k, v in arrays.items() if v.ndim >= 2)
    original = document(arrays)
    if mutation == "missing_key":arrays.pop(key)
    if mutation == "extra_key":arrays["extra_tensor"] = np.zeros(1, dtype=np.float32)
    if mutation == "wrong_shape":arrays[key] = arrays[key].reshape(-1)
    if mutation == "float64":arrays[key] = arrays[key].astype(np.float64)
    if mutation == "nan":arrays[key] = arrays[key].copy(); arrays[key].flat[0] = np.nan
    if mutation == "shape_boolean":original["state_tensor_shapes"][key][0] = True
    npz, meta, d = export(tmp_path, arrays, original)
    with pytest.raises(ContractError):load(npz, meta, d)


def test_duplicate_npz_member_is_rejected(tmp_path, numeric_arrays):
    npz, meta, d = export(tmp_path, numeric_arrays)
    with zipfile.ZipFile(npz, "a") as archive:
        entry = archive.infolist()[0]; payload = archive.read(entry.filename)
        with pytest.warns(UserWarning):archive.writestr(entry.filename, payload)
    d["npz_sha256"] = hashlib.sha256(npz.read_bytes()).hexdigest(); meta.write_text(json.dumps(d))
    with pytest.raises(ContractError, match="duplicate"):
        load(npz, meta, d)


def test_missing_numeric_owner_binding_rejected_before_weights_read(tmp_path, numeric_arrays):
    npz, meta, d = export(tmp_path, numeric_arrays)
    with pytest.raises(ContractError, match="owner binding"):
        loader.load_balanced_research_npz(tmp_path/"missing.npz", meta, weights_sha256=d["npz_sha256"],
            metadata_sha256=hashlib.sha256(meta.read_bytes()).hexdigest(), research_owner_binding=None)


def test_metadata_wrong_external_sha_is_rejected_before_parse(tmp_path, numeric_arrays):
    npz, meta, d = export(tmp_path, numeric_arrays)
    with pytest.raises(ContractError, match="SHA differs"):
        loader.load_balanced_research_npz(npz, meta, weights_sha256=d["npz_sha256"],
            metadata_sha256="0"*64, research_owner_binding={})
