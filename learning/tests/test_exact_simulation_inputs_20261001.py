"""HH_261001 - Exact input archives are lossless, exclusive and bound to measured provenance."""

import json
from types import SimpleNamespace
import numpy as np
import pytest
import torch
from scripts.e2e import capture_exact_simulation_inputs_20261001 as archive


def test_snapshot_roundtrip_and_exclusive_create(tmp_path):
    tensors = dict(images=np.arange(24, dtype=np.float32).reshape(1, 2, 3, 4))
    timing = dict(now_ns=42)
    decision = dict(source_checkpoint_sha256='a' * 64, raw_output={'candidate_logits':[.1]}, total_seconds=.02)
    row = archive.save_input_snapshot(tmp_path, 0, tensors, timing, decision, .001)
    with np.load(tmp_path / row['tensor_file'], allow_pickle=False) as loaded:
        assert np.array_equal(loaded['images'], tensors['images'])
        assert archive.array_digest(loaded['images']) == row['tensor_sha256']['images']
    with pytest.raises(FileExistsError):
        archive.save_input_snapshot(tmp_path, 0, tensors, timing, decision, .001)


def test_hook_preserves_tensor_identity_and_restores(monkeypatch, tmp_path):
    tensors = {name:torch.ones(1) for name in archive.research.FIELDS}
    decision = dict(source_checkpoint_sha256='b' * 64, raw_output={'candidate_logits':[.2]}, total_seconds=.01)
    def build(*args, **kwargs):
        return tensors
    def step(self, inputs, **kwargs):
        assert archive.research.build_runtime_tensors(inputs) is tensors
        return decision
    monkeypatch.setattr(archive.research, 'build_runtime_tensors', build)
    monkeypatch.setattr(archive.research.SimulationResearchPolicy, 'step', step)
    policy = object.__new__(archive.research.SimulationResearchPolicy)
    inputs = SimpleNamespace(camera_frames=[SimpleNamespace(timestamp_ns=2)])
    with archive.exact_tensor_archive(tmp_path / 'fresh'):
        assert policy.step(inputs, now_ns=2, ego_stamp_ns=2, route_stamp_ns=2) is decision
    assert archive.research.build_runtime_tensors is build
    assert archive.research.SimulationResearchPolicy.step is step
    row = json.loads((tmp_path / 'fresh/index.jsonl').read_text())
    assert set(row['tensor_sha256']) == set(archive.research.FIELDS)
    assert row['inputs_or_outputs_modified'] is False


def test_exception_restores_hooks(monkeypatch, tmp_path):
    original = archive.research.SimulationResearchPolicy.step
    with pytest.raises(RuntimeError):
        with archive.exact_tensor_archive(tmp_path / 'fresh'):
            raise RuntimeError('test')
    assert archive.research.SimulationResearchPolicy.step is original
