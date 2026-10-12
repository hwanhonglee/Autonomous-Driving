# HH_260906 - Verify the versioned NumPy weight bundle and every fail-closed boundary.

from io import BytesIO
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np
import pytest
import torch

from portable_e2e import ContractError
from portable_e2e.evaluate import _checkpoint_sampling_provenance
from portable_e2e.losses import TrajectoryLossConfig
from portable_e2e.model import PHYSICAL_MODEL_ID
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel
import portable_e2e.runtime_weight_bundle as bundle_module
from portable_e2e.train import CHECKPOINT_ID
from portable_e2e.train import TrainConfig
from portable_e2e.train import _canonical_sha256
from portable_e2e.train import _device_abi
from portable_e2e.train import _runtime_abi
from portable_e2e.train import _sampling_plan_from_counts


def _small_model_config() -> ModelConfig:
    return ModelConfig(
        route_points=8,
        image_width=16,
        image_height=16,
        image_grid_width=1,
        image_grid_height=1,
        image_embedding=8,
        camera_fusion_width=8,
        ego_embedding=8,
        route_embedding=8,
        hidden_width=16,
        encoder_base_channels=4,
        ego_history_frames=3,
        maximum_step_m=0.1,
    )


def _checkpoint_payload(config=None) -> dict:
    config = _small_model_config() if config is None else config
    train_config = TrainConfig(
        batch_size=2,
        max_steps=1,
        checkpoint_interval=1,
    )
    sampling_plan = _sampling_plan_from_counts({"carla": 2}, train_config)
    state = {
        "epoch": 1,
        "next_batch_index": 0,
        "global_step": 1,
        "samples_seen": 2,
        "domain_samples_seen": {"carla": 2},
    }
    payload = {
        "checkpoint_id": CHECKPOINT_ID,
        "created_at_utc": "2026-09-06T00:00:00Z",
        "dataset_fingerprint_sha256": "d" * 64,
        "corpus_fingerprint_sha256": "c" * 64,
        "training_split": "train",
        "training_episode_ids": ["carla_fixture_episode"],
        "sampling_plan": sampling_plan,
        "sampling_plan_sha256": _canonical_sha256(sampling_plan),
        "sampling_metrics": {
            "samples_seen": 2,
            "domain_samples_seen": {"carla": 2},
        },
        "model_config": config.to_dict(),
        "model_config_sha256": _canonical_sha256(config.to_dict()),
        "train_config": train_config.to_dict(),
        "loss_config": TrajectoryLossConfig().to_dict(),
        "state": state,
        "model_state_dict": PerspectiveTrajectoryModel(config).state_dict(),
        "runtime_abi": _runtime_abi(),
        "device_abi": _device_abi(torch.device("cpu")),
    }
    _checkpoint_sampling_provenance(payload)
    return payload


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _export(monkeypatch, tmp_path, payload=None):
    checkpoint = tmp_path / "model.pt"
    checkpoint.write_bytes(b"secure-checkpoint-fixture")
    source_sha256 = _sha256(checkpoint)
    checkpoint_payload = _checkpoint_payload() if payload is None else payload
    calls = []

    def secure_decode(path):
        calls.append(path)
        return checkpoint_payload, source_sha256

    monkeypatch.setattr(bundle_module, "_secure_checkpoint_decode", secure_decode)
    output = tmp_path / "model.runtime.npz"
    exported = bundle_module.export_runtime_weight_bundle(
        checkpoint,
        output,
        expected_source_checkpoint_sha256=source_sha256,
    )
    assert calls == [checkpoint.absolute()]
    return checkpoint, output, exported, checkpoint_payload


def _load(output, exported):
    return bundle_module.load_runtime_weight_bundle(
        output,
        expected_bundle_sha256=exported.bundle_sha256,
        expected_source_checkpoint_sha256=exported.source_checkpoint_sha256,
        expected_model_config_sha256=exported.model_config_sha256,
        expected_corpus_fingerprint_sha256=exported.corpus_fingerprint_sha256,
    )


def _archive_members(path: Path) -> list[tuple[str, bytes]]:
    with zipfile.ZipFile(path, "r") as archive:
        return [(info.filename, archive.read(info)) for info in archive.infolist()]


def _write_archive(
    path: Path,
    members,
    *,
    compression=zipfile.ZIP_STORED,
):
    with zipfile.ZipFile(path, "w", compression=compression) as archive:
        for name, payload in members:
            archive.writestr(name, payload, compress_type=compression)


def _manifest_and_members(path: Path):
    members = _archive_members(path)
    manifest = json.loads(dict(members)[bundle_module.MANIFEST_MEMBER])
    return manifest, members


def _replace_manifest(members, manifest):
    payload = bundle_module._canonical_json_bytes(manifest) + b"\n"
    return [
        (name, payload if name == bundle_module.MANIFEST_MEMBER else value)
        for name, value in members
    ]


def _load_mutated(path, exported):
    return bundle_module.load_runtime_weight_bundle(
        path,
        expected_bundle_sha256=_sha256(path),
        expected_source_checkpoint_sha256=exported.source_checkpoint_sha256,
        expected_model_config_sha256=exported.model_config_sha256,
        expected_corpus_fingerprint_sha256=exported.corpus_fingerprint_sha256,
    )


def test_round_trip_preserves_provenance_and_strict_model_state(monkeypatch, tmp_path):
    _, output, exported, source_payload = _export(monkeypatch, tmp_path)

    loaded = _load(output, exported)
    reconstructed = loaded.to_torch_state_dict(torch)
    model = PerspectiveTrajectoryModel(_small_model_config())
    model.load_state_dict(reconstructed, strict=True)

    assert exported.bundle_sha256 == _sha256(output)
    assert loaded.source_checkpoint_sha256 == exported.source_checkpoint_sha256
    assert loaded.model_config_sha256 == source_payload["model_config_sha256"]
    assert loaded.corpus_fingerprint_sha256 == source_payload["corpus_fingerprint_sha256"]
    assert loaded.training_provenance["sampling_plan"] == source_payload["sampling_plan"]
    assert loaded.training_provenance["state"] == source_payload["state"]
    assert tuple(loaded.numpy_state_dict) == tuple(sorted(source_payload["model_state_dict"]))
    for key, expected in source_payload["model_state_dict"].items():
        assert torch.equal(model.state_dict()[key], expected)
        assert loaded.numpy_state_dict[key].flags.writeable is False


def test_round_trip_supports_the_distinct_physical_v1_model(monkeypatch, tmp_path):
    config = ModelConfig(
        **{
            **_small_model_config().to_dict(),
            "model_id": PHYSICAL_MODEL_ID,
            "maximum_step_m": 1.0,
        }
    )
    payload = _checkpoint_payload(config)
    _, output, exported, _ = _export(monkeypatch, tmp_path, payload=payload)

    loaded = _load(output, exported)
    model = PerspectiveTrajectoryModel(config)
    model.load_state_dict(loaded.to_torch_state_dict(torch), strict=True)

    assert loaded.model_config["model_id"] == PHYSICAL_MODEL_ID


@pytest.mark.parametrize("dtype", (torch.int64, torch.float64, torch.complex64))
def test_export_rejects_model_state_dtype_coercion(monkeypatch, tmp_path, dtype):
    payload = _checkpoint_payload()
    key = next(iter(payload["model_state_dict"]))
    payload["model_state_dict"][key] = payload["model_state_dict"][key].to(dtype)
    checkpoint = tmp_path / "model.pt"
    checkpoint.write_bytes(b"secure-checkpoint-fixture")
    source_sha256 = _sha256(checkpoint)
    monkeypatch.setattr(
        bundle_module,
        "_secure_checkpoint_decode",
        lambda path: (payload, source_sha256),
    )
    output = tmp_path / "model.runtime.npz"

    with pytest.raises(ContractError, match="dtype"):
        bundle_module.export_runtime_weight_bundle(
            checkpoint,
            output,
            expected_source_checkpoint_sha256=source_sha256,
        )

    assert not output.exists()


def test_export_rejects_model_state_shape_drift_before_strict_load(monkeypatch, tmp_path):
    payload = _checkpoint_payload()
    key = next(iter(payload["model_state_dict"]))
    payload["model_state_dict"][key] = payload["model_state_dict"][key].reshape(-1)
    checkpoint = tmp_path / "model.pt"
    checkpoint.write_bytes(b"secure-checkpoint-fixture")
    source_sha256 = _sha256(checkpoint)
    monkeypatch.setattr(
        bundle_module,
        "_secure_checkpoint_decode",
        lambda path: (payload, source_sha256),
    )
    output = tmp_path / "model.runtime.npz"

    with pytest.raises(ContractError, match="shape"):
        bundle_module.export_runtime_weight_bundle(
            checkpoint,
            output,
            expected_source_checkpoint_sha256=source_sha256,
        )

    assert not output.exists()


def test_manifest_pins_every_tensor_and_uses_only_stored_npy(monkeypatch, tmp_path):
    _, output, exported, source_payload = _export(monkeypatch, tmp_path)

    with zipfile.ZipFile(output, "r") as archive:
        manifest = json.loads(archive.read(bundle_module.MANIFEST_MEMBER))
        infos = archive.infolist()

    assert manifest["bundle_id"] == bundle_module.BUNDLE_ID
    assert manifest["schema_version"] == 1
    assert manifest["tensor_count"] == len(source_payload["model_state_dict"])
    assert len(manifest["tensors"]) == manifest["tensor_count"]
    assert all(info.compress_type == zipfile.ZIP_STORED for info in infos)
    assert all(record["tensor_sha256"] for record in manifest["tensors"])
    assert all(record["npy_sha256"] for record in manifest["tensors"])
    assert exported.total_tensor_bytes == manifest["total_tensor_bytes"]


def test_export_refuses_wrong_source_pin_without_output(monkeypatch, tmp_path):
    checkpoint = tmp_path / "model.pt"
    checkpoint.write_bytes(b"secure-checkpoint-fixture")
    payload = _checkpoint_payload()
    monkeypatch.setattr(
        bundle_module,
        "_secure_checkpoint_decode",
        lambda path: (payload, _sha256(checkpoint)),
    )
    output = tmp_path / "model.runtime.npz"

    with pytest.raises(ContractError, match="operator pin"):
        bundle_module.export_runtime_weight_bundle(
            checkpoint,
            output,
            expected_source_checkpoint_sha256="0" * 64,
        )

    assert not output.exists()


def test_export_is_atomic_and_never_overwrites(monkeypatch, tmp_path):
    checkpoint, output, exported, payload = _export(monkeypatch, tmp_path)
    original = output.read_bytes()
    monkeypatch.setattr(
        bundle_module,
        "_secure_checkpoint_decode",
        lambda path: (payload, exported.source_checkpoint_sha256),
    )

    with pytest.raises(ContractError, match="overwrite"):
        bundle_module.export_runtime_weight_bundle(
            checkpoint,
            output,
            expected_source_checkpoint_sha256=exported.source_checkpoint_sha256,
        )

    assert output.read_bytes() == original
    assert not list(tmp_path.glob(".*.tmp.*"))


def test_export_and_load_reject_symlinks(monkeypatch, tmp_path):
    checkpoint, output, exported, _ = _export(monkeypatch, tmp_path)
    source_link = tmp_path / "source-link.pt"
    source_link.symlink_to(checkpoint.name)
    output_link = tmp_path / "bundle-link.npz"
    output_link.symlink_to(output.name)

    with pytest.raises(ContractError, match="symlink"):
        bundle_module.export_runtime_weight_bundle(
            source_link,
            tmp_path / "other.npz",
            expected_source_checkpoint_sha256=exported.source_checkpoint_sha256,
        )
    with pytest.raises(ContractError, match="symlink"):
        _load(output_link, exported)


def test_loader_requires_exact_whole_file_and_provenance_pins(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)

    with pytest.raises(ContractError, match="bundle SHA-256"):
        bundle_module.load_runtime_weight_bundle(
            output,
            expected_bundle_sha256="0" * 64,
            expected_source_checkpoint_sha256=exported.source_checkpoint_sha256,
            expected_model_config_sha256=exported.model_config_sha256,
            expected_corpus_fingerprint_sha256=exported.corpus_fingerprint_sha256,
        )
    with pytest.raises(ContractError, match="source checkpoint pin"):
        bundle_module.load_runtime_weight_bundle(
            output,
            expected_bundle_sha256=exported.bundle_sha256,
            expected_source_checkpoint_sha256="0" * 64,
            expected_model_config_sha256=exported.model_config_sha256,
            expected_corpus_fingerprint_sha256=exported.corpus_fingerprint_sha256,
        )


def test_loader_rejects_schema_drift_with_a_new_valid_file_pin(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    manifest, members = _manifest_and_members(output)
    manifest["unexpected_field"] = "drift"
    mutated = tmp_path / "schema-drift.npz"
    _write_archive(mutated, _replace_manifest(members, manifest))

    with pytest.raises(ContractError, match="schema fields differ"):
        _load_mutated(mutated, exported)


def test_loader_rejects_duplicate_and_traversal_members(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    members = _archive_members(output)
    duplicate = tmp_path / "duplicate.npz"
    with pytest.warns(UserWarning, match="Duplicate name"):
        _write_archive(duplicate, [*members, members[0]])
    traversal = tmp_path / "traversal.npz"
    _write_archive(traversal, [*members, ("../escape.npy", b"not-an-array")])

    with pytest.raises(ContractError, match="duplicate member"):
        _load_mutated(duplicate, exported)
    with pytest.raises(ContractError, match="path traversal"):
        _load_mutated(traversal, exported)


def test_loader_rejects_duplicate_json_and_tensor_keys(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    manifest, members = _manifest_and_members(output)
    raw_manifest = dict(members)[bundle_module.MANIFEST_MEMBER]
    duplicate_json = tmp_path / "duplicate-json.npz"
    duplicate_payload = b'{"bundle_id":"duplicate",' + raw_manifest[1:]
    _write_archive(
        duplicate_json,
        [
            (
                name,
                duplicate_payload if name == bundle_module.MANIFEST_MEMBER else value,
            )
            for name, value in members
        ],
    )
    manifest["tensors"][1]["key"] = manifest["tensors"][0]["key"]
    manifest["model_state_sha256"] = bundle_module._state_index_sha256(manifest["tensors"])
    duplicate_tensor = tmp_path / "duplicate-tensor.npz"
    _write_archive(duplicate_tensor, _replace_manifest(members, manifest))

    with pytest.raises(ContractError, match="duplicate JSON key"):
        _load_mutated(duplicate_json, exported)
    with pytest.raises(ContractError, match="duplicate or not canonical"):
        _load_mutated(duplicate_tensor, exported)


def test_loader_rejects_all_compression_as_a_zip_bomb_boundary(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    compressed = tmp_path / "compressed.npz"
    _write_archive(
        compressed,
        _archive_members(output),
        compression=zipfile.ZIP_DEFLATED,
    )

    with pytest.raises(ContractError, match="zip-bomb gate"):
        _load_mutated(compressed, exported)


def test_loader_rejects_tensor_member_tampering_after_repinning_file(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    manifest, members = _manifest_and_members(output)
    target = manifest["tensors"][0]["member"]
    mutated_members = []
    for name, payload in members:
        if name == target:
            changed = bytearray(payload)
            changed[-1] ^= 1
            payload = bytes(changed)
        mutated_members.append((name, payload))
    mutated = tmp_path / "tensor-tamper.npz"
    _write_archive(mutated, mutated_members)

    with pytest.raises(ContractError, match="NPY SHA-256"):
        _load_mutated(mutated, exported)


def test_loader_rejects_object_dtype_before_numpy_decode(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    manifest, members = _manifest_and_members(output)
    record = manifest["tensors"][0]
    array = np.array(["unsafe"], dtype=object)
    stream = BytesIO()
    np.save(stream, array, allow_pickle=True)
    payload = stream.getvalue()
    record.update(
        {
            "dtype": array.dtype.str,
            "shape": list(array.shape),
            "nbytes": array.nbytes,
            "npy_nbytes": len(payload),
            "tensor_sha256": hashlib.sha256(array.tobytes()).hexdigest(),
            "npy_sha256": hashlib.sha256(payload).hexdigest(),
        }
    )
    manifest["total_tensor_bytes"] = sum(item["nbytes"] for item in manifest["tensors"])
    manifest["total_npy_bytes"] = sum(item["npy_nbytes"] for item in manifest["tensors"])
    manifest["model_state_sha256"] = bundle_module._state_index_sha256(manifest["tensors"])
    members = [
        (name, payload if name == record["member"] else value)
        for name, value in _replace_manifest(members, manifest)
    ]
    mutated = tmp_path / "object-dtype.npz"
    _write_archive(mutated, members)

    with pytest.raises(ContractError, match="dtype is not supported"):
        _load_mutated(mutated, exported)


def test_loader_rejects_nonfinite_tensor_even_when_all_hashes_are_rebuilt(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    manifest, members = _manifest_and_members(output)
    record = manifest["tensors"][0]
    original = dict(members)[record["member"]]
    array = np.load(BytesIO(original), allow_pickle=False)
    array.reshape(-1)[0] = np.nan
    stream = BytesIO()
    np.save(stream, array, allow_pickle=False)
    payload = stream.getvalue()
    record["npy_nbytes"] = len(payload)
    record["tensor_sha256"] = hashlib.sha256(array.tobytes(order="C")).hexdigest()
    record["npy_sha256"] = hashlib.sha256(payload).hexdigest()
    manifest["total_npy_bytes"] = sum(item["npy_nbytes"] for item in manifest["tensors"])
    manifest["model_state_sha256"] = bundle_module._state_index_sha256(manifest["tensors"])
    members = [
        (name, payload if name == record["member"] else value)
        for name, value in _replace_manifest(members, manifest)
    ]
    mutated = tmp_path / "nonfinite.npz"
    _write_archive(mutated, members)

    with pytest.raises(ContractError, match="NaN or Inf"):
        _load_mutated(mutated, exported)


def test_export_rejects_nonfinite_source_tensor(monkeypatch, tmp_path):
    payload = _checkpoint_payload()
    key = next(iter(payload["model_state_dict"]))
    payload["model_state_dict"][key].reshape(-1)[0] = float("nan")
    checkpoint = tmp_path / "model.pt"
    checkpoint.write_bytes(b"secure-checkpoint-fixture")
    source_sha256 = _sha256(checkpoint)
    monkeypatch.setattr(
        bundle_module,
        "_secure_checkpoint_decode",
        lambda path: (payload, source_sha256),
    )

    with pytest.raises(ContractError, match="NaN or Inf"):
        bundle_module.export_runtime_weight_bundle(
            checkpoint,
            tmp_path / "nonfinite-source.npz",
            expected_source_checkpoint_sha256=source_sha256,
        )


def test_loader_always_disables_numpy_pickle(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    original_load = bundle_module.np.load
    calls = []

    def checked_load(*args, **kwargs):
        calls.append(kwargs.get("allow_pickle"))
        return original_load(*args, **kwargs)

    monkeypatch.setattr(bundle_module.np, "load", checked_load)

    _load(output, exported)

    assert calls
    assert set(calls) == {False}


def test_torch_reconstruction_requires_only_from_numpy(monkeypatch, tmp_path):
    _, output, exported, _ = _export(monkeypatch, tmp_path)
    loaded = _load(output, exported)
    calls = []

    class LegacyTorchFacade:
        @staticmethod
        def from_numpy(value):
            calls.append(value)
            return (value.dtype.str, tuple(value.shape))

    state_dict = loaded.to_torch_state_dict(LegacyTorchFacade)

    assert tuple(state_dict) == tuple(loaded.numpy_state_dict)
    assert len(calls) == len(loaded.numpy_state_dict)
    assert all(value.flags.c_contiguous and value.flags.writeable for value in calls)
