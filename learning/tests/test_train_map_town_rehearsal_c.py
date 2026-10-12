"""HH_260906 - Test C's TRAIN-only sampler and frozen B contract without CUDA."""
import hashlib
import json

import numpy as np
import pytest

from scripts.e2e import train_map_town_rehearsal_c as candidate


def test_every_batch_has_exact_7_to_1_train_indices_and_is_deterministic():
    sampler = candidate.TownRehearsalSampler()
    first = np.random.default_rng(20260922)
    second = np.random.default_rng(20260922)
    outputs = []
    for _ in range(100):
        batch = sampler.sample_batch(first)
        assert batch.shape == (8,) and batch.dtype == np.int64
        assert np.count_nonzero(batch < 497) == 7
        assert np.count_nonzero((batch >= 497) & (batch < 913)) == 1
        outputs.append(batch)
    assert all(np.array_equal(row, sampler.sample_batch(second)) for row in outputs)
    assert len({int(row[-1]) for row in outputs}) > 1


@pytest.mark.parametrize("old,total", [(496, 913), (497, 912), (913, 913)])
def test_changed_train_partition_is_rejected(old, total):
    with pytest.raises(ValueError, match="frozen 497/913"):
        candidate.TownRehearsalSampler(old, total)


def test_sampling_contract_never_uses_dev_or_test():
    contract = candidate.TownRehearsalSampler().contract()
    assert contract["parent_range"] == [0, 497]
    assert contract["added_range"] == [497, 913]
    assert contract["parent_draws_per_batch"] == 7
    assert contract["added_draws_per_batch"] == 1
    assert contract["dev_or_test_used"] is False
    assert contract["replacement"] is True


def test_b_reference_requires_original_source_loss_and_scratch_contract(tmp_path, monkeypatch):
    root = tmp_path / "reference_b"
    (root / "source").mkdir(parents=True)
    blobs = {"source/train_map_native_ah.py": b"B trainer", "checkpoint.pt": b"B weights",
             "data_manifest.json": b"{}"}
    for name, blob in blobs.items():
        (root / name).write_bytes(blob)
    for name in candidate.B_SUPPORT_SHA256:
        (root / "source" / name).write_bytes(("support:" + name).encode())
    monkeypatch.setattr(candidate, "B_SUPPORT_SHA256", {
        name: hashlib.sha256(("support:" + name).encode()).hexdigest()
        for name in candidate.B_SUPPORT_SHA256})
    # HH_260906 - Substitute fixture support digests without reading CUDA or TEST frames.
    digests = {key: hashlib.sha256(value).hexdigest() for key, value in blobs.items()}
    package = json.loads(blobs["data_manifest.json"])
    (root / "started.json").write_text(json.dumps({
        "source_sha256": digests["source/train_map_native_ah.py"],
        "data_manifest_sha256": digests["data_manifest.json"],
        "parent_checkpoint_sha256": candidate.PARENT_CHECKPOINT_SHA256,
        "steps": 2000, "batch_size": 8, "seed": 20260922, "learning_rate": 3e-4,
        "initialization": "scratch", "train_count": 913, "dev_count": 118,
        "augmentation": "TRAIN only: probability 0.5 aligned crop, scale 0.65..1.0, resize to native; no flip"}))
    (root / "loss_contract.json").write_text(json.dumps({
        "dev_used": False, "train_pixels": candidate.B_TRAIN_PIXELS, "weights": candidate.B_WEIGHTS}))
    (root / "summary.json").write_text(json.dumps({"checkpoint_sha256": digests["checkpoint.pt"]}))
    parent = {"splits": {"train": {"count": 497}}}

    def digest(path):
        if path.parent == candidate.Path(__file__).resolve().parents[2] / "tools/e2e" and path.name in candidate.B_SUPPORT_SHA256:
            return candidate.B_SUPPORT_SHA256[path.name]
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def require(condition, message):
        if not condition:
            raise ValueError(message)

    old_source, old_checkpoint, old_package = (candidate.B_SOURCE_SHA256,
                                               candidate.B_CHECKPOINT_SHA256, candidate.PACKAGE_SHA256)
    try:
        candidate.B_SOURCE_SHA256 = digests["source/train_map_native_ah.py"]
        candidate.B_CHECKPOINT_SHA256 = digests["checkpoint.pt"]
        candidate.PACKAGE_SHA256 = digests["data_manifest.json"]
        started, loss = candidate.validate_b_reference(root, package, parent, digest, require)
        assert started["initialization"] == "scratch" and loss["dev_used"] is False
        (root / "started.json").write_text((root / "started.json").read_text().replace("\"scratch\"", "\"resume\""))
        with pytest.raises(ValueError, match="optimization contract"):
            candidate.validate_b_reference(root, package, parent, digest, require)
    finally:
        candidate.B_SOURCE_SHA256, candidate.B_CHECKPOINT_SHA256, candidate.PACKAGE_SHA256 = (
            old_source, old_checkpoint, old_package)
