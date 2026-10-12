"""HH_261001 - Keep the raw bootstrap bounded, personal, create-only and physical-GPU0-only."""

from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import train_temporal_v2_raw_bootstrap_20261001 as trainer


def test_bad_visible_gpu_is_rejected_before_cuda_or_nvidia_query(monkeypatch):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "1")
    monkeypatch.setattr(trainer.subprocess, "check_output", lambda *a, **k: pytest.fail("query must not run"))
    with pytest.raises(ValueError, match="physical GPU0"):
        trainer.gpu_preflight()


@pytest.mark.parametrize("row", [f"0, {trainer.GPU_UUID}, 10, 0", f"0, {trainer.GPU_UUID}, 0, 50",
                                "1, different-uuid, 0, 0"])
def test_busy_or_wrong_physical_gpu_is_rejected_before_cuda(monkeypatch, row):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", trainer.GPU_UUID)
    monkeypatch.setattr(trainer.subprocess, "check_output", lambda *a, **k: row)
    monkeypatch.setattr(torch.cuda, "is_available", lambda: pytest.fail("CUDA must not run"))
    with pytest.raises(ValueError, match="idle"):
        trainer.gpu_preflight()


def test_old_result_cannot_be_overwritten(tmp_path):
    path = tmp_path / "original.json"
    trainer.write_new(path, {"original": True})
    with pytest.raises(FileExistsError):
        trainer.write_new(path, {"original": False})
    assert trainer.json.loads(path.read_text()) == {"original": True}


@pytest.mark.parametrize("steps,batch", [(0, 2), (201, 2), (100, 0), (100, 5)])
def test_main_training_cannot_be_started_with_bootstrap_cli(tmp_path, steps, batch):
    with pytest.raises(ValueError, match="bounded"):
        trainer.main(["--train", str(tmp_path), "--known-dev", str(tmp_path), "--output", str(tmp_path/"new"),
                      "--steps", str(steps), "--batch-size", str(batch)])


def test_probe_is_declared_from_labels_not_neural_scores():
    rows = []
    for index in range(15):
        features = [0.] * 13
        features[7 if index < 5 else 8 if index < 10 else 9] = 1.
        rows.append(SimpleNamespace(features=features))
    dataset = SimpleNamespace(examples=rows)
    indices = trainer.fixed_probe(dataset, list(range(15)), maximum_per_direction=2)
    assert indices == [0, 4, 5, 9, 10, 14]


def test_raw_cohort_does_not_clip_future_speed_or_choose_by_output(monkeypatch):
    features = ([1., 0.], [1., .1], [1., 8.2])
    dataset = SimpleNamespace(examples=[SimpleNamespace(features=values) for values in features])
    monkeypatch.setattr(trainer, "spatial_expert_labels", lambda row: {"spatial_valid": torch.ones(33, dtype=torch.bool)})
    assert trainer.eligible_indices(dataset) == [1, 2]
