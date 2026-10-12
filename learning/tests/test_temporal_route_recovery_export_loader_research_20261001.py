"""HH_261001 - Test NEW export scope, immutable backbone and separate experimental ownership, not quality."""
import hashlib
from io import BytesIO
import json
import zipfile

import numpy as np
import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryConfigV3
from portable_e2e import temporal_route_recovery_export_loader_research_20261001 as loader


def description():
    sources = dict(loader.TRAINING_SOURCE_PINS)
    sources.update({k: "d" * 64 for k in loader.MINIMUM_SOURCE_KEYS - set(sources)})
    return dict(schema=loader.EXPORT_SCHEMA, architecture_schema=loader.ARCHITECTURE_SCHEMA, model_id=loader.MODEL_ID,
        optimizer_updates=3000, maximum_optimizer_updates=3000, additional_optimizer_updates=3000, effective_batch_size=4,
        examples_seen=12000, recovery_examples_seen=6000, v4_examples_seen=3000, legacy_examples_seen=3000,
        training_episode_count=18, recovery_training_episode_count=9, replay_training_episode_count=9, holdout_episode_count=2,
        parent_optimizer_updates=3000, parent_known_total_optimizer_updates=3200, known_total_optimizer_updates=6200,
        training_only=True, simulation_only=True, main_training=False, model_promoted=False, vehicle_control_approved=False,
        inference_loader_authorized=False, stop_intent_trained=False, labels_clipped=False, labels_extrapolated=False,
        backbone_frozen=True, independent_validation=False, legacy_decoded=False, checkpoint_selection_by_holdout=False,
        parent_all_diagnostic_screens_passed=False, parent_diagnostic_pass_count=7, runtime_intent="explicit_cruise_GO_not_head",
        trainable_state_names=sorted(loader.BRANCH_KEYS), batch_domains=loader.BATCH_DOMAINS,
        parent_checkpoint_sha256=loader.PARENT_CHECKPOINT_SHA, parent_npz_sha256=loader.PARENT_NPZ_SHA,
        parent_metadata_sha256=loader.PARENT_METADATA_SHA, npz_sha256="e" * 64, checkpoint_sha256="c" * 64,
        declaration_sha256="a" * 64, backbone_state_sha256="b" * 64, training_dataset_fingerprint_sha256="f" * 64,
        holdout_dataset_fingerprint_sha256="9" * 64, model_config=TemporalModelConfigV2().to_dict(),
        recovery_config=RouteRecoveryConfigV3().to_dict(), source_sha256=sources,
        numerical_revision=dict(kernel_id=loader.KERNEL_ID, strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False),
        learning_rate=dict(peak=3e-4, final=3e-5, warmup_updates=100,
            schedule="fixed3000_linear_warmup_cosine_branch_only", units="AdamW optimizer learning rate",
            gradient_clip_l2=5., weight_decay=1e-4, inherited_joint_schedule_reused=False))


def binding(purpose="offline_cpu_numeric_validation_only"):
    return dict(schema=loader.OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id="unit-test-no-control",
        vehicle_control_approved=False, model_promoted=False, purpose=purpose, simulator_control_authorized=False,
        weights_sha256="e" * 64, metadata_sha256="8" * 64, checkpoint_sha256="c" * 64,
        declaration_sha256="a" * 64, fixed_endpoint_updates=3000)


def test_new_architecture_and_actual_episode_exposure_contract():
    config, recovery = loader.verify_scope(description())
    assert config == TemporalModelConfigV2() and recovery == RouteRecoveryConfigV3()
    assert description()["training_episode_count"] == 18
    assert description()["parent_all_diagnostic_screens_passed"] is False


@pytest.mark.parametrize("key,value", [("optimizer_updates", 500), ("optimizer_updates", 3000.),
    ("training_episode_count", 9), ("recovery_training_episode_count", 18), ("holdout_episode_count", 0),
    ("recovery_examples_seen", 12000), ("backbone_frozen", False), ("model_promoted", True),
    ("inference_loader_authorized", True), ("parent_all_diagnostic_screens_passed", True),
    ("parent_checkpoint_sha256", "0" * 64), ("known_total_optimizer_updates", 3200),
    ("schema", "portable_e2e.temporal_v2_balanced_replay_export.20261001.v1"),
    ("trainable_state_names", ["speed_head.weight"]), ("batch_domains", ["recovery_joint"] * 4)])
def test_wrong_stage_or_falsified_scope_rejected(key, value):
    d = description(); d[key] = value
    with pytest.raises(ContractError): loader.verify_scope(d)


def test_recursive_closure_needs_old_frozen17_and_new_modules():
    d = description(); d["source_sha256"].pop(loader.MODEL_FILE)
    with pytest.raises(ContractError): loader.verify_scope(d)
    d = description(); d["source_sha256"]["tools/e2e/train_temporal_v2_balanced_replay_20261001.py"] = "0" * 64
    with pytest.raises(ContractError): loader.verify_scope(d)


def test_numeric_binding_remains_not_authorized_for_control():
    owner = binding(); loader.verify_owner(owner, description(), "e" * 64, "8" * 64)
    owner["simulator_control_authorized"] = True
    with pytest.raises(ContractError): loader.verify_owner(owner, description(), "e" * 64, "8" * 64)


def test_owned_carla_purpose_is_separate_root_plan_not_metadata_promotion():
    owner = binding("owned_carla_research_only")
    with pytest.raises(ContractError): loader.verify_owner(owner, description(), "e" * 64, "8" * 64)
    owner.update(simulator_control_authorized=True, simulation_only=True, owner_plan_sha256="7" * 64)
    d = description(); loader.verify_owner(owner, d, "e" * 64, "8" * 64)
    assert d["inference_loader_authorized"] is d["model_promoted"] is False
    owner["vehicle_control_approved"] = True
    with pytest.raises(ContractError): loader.verify_owner(owner, d, "e" * 64, "8" * 64)


@pytest.mark.parametrize("key,value", [("owner_id", ""), ("purpose", "real_vehicle"),
    ("fixed_endpoint_updates", 2500), ("weights_sha256", "0" * 64), ("metadata_sha256", "0" * 64),
    ("checkpoint_sha256", "0" * 64), ("declaration_sha256", "0" * 64)])
def test_external_owner_exact_binding_guards(key, value):
    owner = binding(); owner[key] = value
    with pytest.raises(ContractError): loader.verify_owner(owner, description(), "e" * 64, "8" * 64)


def test_backbone_fingerprint_includes_signed_zero_and_every_original_tensor_but_not_branch():
    state = {"curvature_head.bias": torch.tensor([0.]), "speed_head.bias": torch.tensor([2.4]),
             "route_recovery_branch.2.bias": torch.tensor([.1])}
    original = loader.backbone_fingerprint(state)
    state["route_recovery_branch.2.bias"][0] = 3.
    assert loader.backbone_fingerprint(state) == original
    state["curvature_head.bias"][0] = -0.
    assert loader.backbone_fingerprint(state) != original
    state["speed_head.bias"][0] = float("nan")
    with pytest.raises(ContractError): loader.backbone_fingerprint(state)


def test_metadata_does_not_use_bool_scale_equivalence():
    d = description(); d["recovery_config"]["lateral_scale_m"] = True
    with pytest.raises(ContractError): loader.verify_scope(d)


@pytest.mark.parametrize("key,value", [("kernel_id", "old_linear_atomic"), ("strict_deterministic_algorithms", False),
    ("tf32_matrix_multiplication", True), ("finite_precision_bitwise_equivalence_claimed", True)])
def test_numerical_revision_cannot_silently_change_decoder_math(key, value):
    d = description(); d["numerical_revision"][key] = value
    with pytest.raises(ContractError): loader.verify_scope(d)


def test_parent_passcount_is_not_float_or_boolean_equivalence():
    d = description(); d["parent_diagnostic_pass_count"] = 7.
    with pytest.raises(ContractError): loader.verify_scope(d)


@pytest.fixture
def numeric_fixture(tmp_path, monkeypatch):
    # HH_261001 - Small synthetic101-key archive exercises loading guards without GPU, CARLA or a CNN benchmark.
    state = {"backbone_%02d" % i: torch.tensor([float(i)]) for i in range(97)}
    state.update({k: torch.zeros(2, 2) if k.endswith("weight") else torch.zeros(2) for k in loader.BRANCH_KEYS})
    parent_state = {k: v.clone() for k, v in state.items() if k not in loader.BRANCH_KEYS}
    class SyntheticModel(torch.nn.Module):
        def __init__(self, *args):
            super().__init__(); self.values = {k: v.clone() for k, v in state.items()}
        def state_dict(self): return dict(self.values)
        def load_state_dict(self, values, strict=True):
            assert strict and set(values) == set(self.values); self.values = dict(values)
    monkeypatch.setattr(loader, "RouteRecoveryTemporalModelV3", SyntheticModel)
    parent = SyntheticModel(); parent.values = parent_state
    monkeypatch.setattr(loader, "load_balanced_research_npz", lambda *args, **kwargs: parent)
    original_pinned = loader.pinned
    def pinned(path, expected, bound):
        path = __import__("pathlib").Path(path)
        if path.is_relative_to(loader.ROOT): return b"synthetic-source-not-a-live-source-check"
        return original_pinned(path, expected, bound)
    monkeypatch.setattr(loader, "pinned", pinned)
    def call(change=None, duplicate=False):
        arrays = {k: v.numpy().copy() for k, v in state.items()}
        if change: change(arrays)
        weights, metadata = tmp_path / "model_state.npz", tmp_path / "metadata.json"
        with weights.open("wb") as stream: np.savez(stream, **arrays)
        if duplicate:
            with zipfile.ZipFile(weights, "a") as archive:
                payload = BytesIO(); np.save(payload, arrays[sorted(arrays)[0]], allow_pickle=False)
                with pytest.warns(UserWarning): archive.writestr(sorted(arrays)[0]+".npy", payload.getvalue())
        weight_sha = hashlib.sha256(weights.read_bytes()).hexdigest()
        d = description(); d.update(npz_sha256=weight_sha, backbone_state_sha256=loader.backbone_fingerprint(parent_state),
            state_tensor_shapes={k: list(v.shape) for k, v in state.items()}, state_tensor_dtypes={k: "float32" for k in state})
        metadata.write_text(json.dumps(d)); metadata_sha = hashlib.sha256(metadata.read_bytes()).hexdigest()
        owner = binding(); owner.update(weights_sha256=weight_sha, metadata_sha256=metadata_sha)
        return loader.load_route_recovery_research_npz(weights, metadata, weights_sha256=weight_sha, metadata_sha256=metadata_sha,
            parent_weights=tmp_path/"unused-parent.npz", parent_metadata=tmp_path/"unused-parent.json",
            research_owner_binding=owner)
    return call


def test_synthetic_full_numeric_abi_load_remains_research_only(numeric_fixture):
    model = numeric_fixture()
    assert len(model.state_dict()) == 101 and model._research_recovery_simulator_control_authorized is False
    assert model._research_temporal_export_metadata["inference_loader_authorized"] is False


@pytest.mark.parametrize("mode", ["float64", "nan", "object", "shape", "missing", "extra", "backbone_changed"])
def test_numeric_archive_rejects_dtype_nonfinite_pickle_shape_keys_or_backbone_mutation(numeric_fixture, mode):
    def change(arrays):
        key = sorted(loader.BRANCH_KEYS)[0]
        if mode == "float64": arrays[key] = arrays[key].astype(np.float64)
        if mode == "nan": arrays[key].flat[0] = float("nan")
        if mode == "object": arrays[key] = np.array([{"pickle": "forbidden"}], dtype=object)
        if mode == "shape": arrays[key] = np.zeros((99,), dtype=np.float32)
        if mode == "missing": del arrays[key]
        if mode == "extra": arrays["unlisted"] = np.zeros(1, dtype=np.float32)
        if mode == "backbone_changed": arrays["backbone_00"][0] = 123.
    with pytest.raises((ContractError, ValueError)): numeric_fixture(change)


def test_numeric_archive_rejects_duplicate_zip_members(numeric_fixture):
    with pytest.raises(ContractError, match="archive"): numeric_fixture(duplicate=True)


@pytest.mark.parametrize("key,value", [("peak", 3e-5), ("final", 3e-6), ("warmup_updates", 100.),
    ("inherited_joint_schedule_reused", True), ("gradient_clip_l2", 5), ("schedule", "select_best_holdout")])
def test_fresh_branch_lr_declared_in_export_cannot_reuse_old_backbone_schedule(key, value):
    d = description(); d["learning_rate"][key] = value
    with pytest.raises(ContractError): loader.verify_scope(d)
