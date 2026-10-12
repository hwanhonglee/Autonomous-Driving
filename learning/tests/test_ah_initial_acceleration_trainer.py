"""HH_261001 - Verify the fixed matched experiment and a real optimizer/checkpoint path on CPU."""

import json
from pathlib import Path
from types import SimpleNamespace

import torch

from portable_e2e.model import ModelConfig, PHYSICAL_STOPMIX_MODEL_ID, PerspectiveTrajectoryModel
from scripts.e2e import run_ah_initial_acceleration_ab_20261001 as experiment


def test_fixed_contract_and_frozen_dependency_pins():
    declared = experiment.contract()
    assert declared["optimizer_steps_per_arm"] == 3000
    assert declared["arms"] == {"A_ah_continuation": 0., "B_preclamp_acceleration": .05}
    assert declared["acceleration_points"] == 5
    assert not declared["decoder_changed"] and not declared["dev_during_training"]
    assert declared["final_dev_after_both_checkpoints"]
    root = Path(__file__).resolve().parents[2]
    plan = json.loads((root / "storage/experiments/training/2026-09-22/moving_recovery_k_v1/plan.json").read_text())
    assert experiment.verify_sources(root, plan)


def test_cpu_optimizer_and_final_checkpoint_smoke(tmp_path, monkeypatch):
    torch.set_num_threads(2)
    cfg = ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID, candidate_count=12, maximum_step_m=1.,
        image_width=80, image_height=48, image_embedding=8, camera_fusion_width=8,
        ego_embedding=8, route_embedding=8, hidden_width=16, encoder_base_channels=8)
    model = PerspectiveTrajectoryModel(cfg)
    initial = experiment.base._clone_state(model.state_dict())
    fingerprint = experiment.base.tensor_digest(initial)

    class Dataset:
        fingerprint_sha256 = "test-only-fingerprint"
        examples = [SimpleNamespace(token="train:" + str(i), sequence_index=i, episode_id="train") for i in range(4)]

        def __len__(self):
            return 4

        def __getitem__(self, index):
            ego = torch.zeros(10, 13)
            ego[:, 0] = 1.; ego[:, 1] = 1.; ego[:, 10] = 1.
            route = torch.stack((torch.arange(128), torch.zeros(128)), dim=1).float()
            target = torch.stack((torch.arange(1, 65) * .1, torch.zeros(64)), dim=1)
            return dict(sample_id=self.examples[index].token, episode_id="train",
                images=torch.zeros(6, 3, 48, 80), calibration=torch.zeros(6, 16),
                ego_history=ego, ego_history_mask=torch.ones(10, dtype=torch.bool),
                route_xy=route, route_mask=torch.ones(128, dtype=torch.bool),
                target_xy=target, target_speed_mps=torch.ones(64), target_yaw_rad=torch.zeros(64),
                target_valid=torch.ones(64, dtype=torch.bool))

    monkeypatch.setattr(experiment, "STEPS", 1)
    monkeypatch.setattr(experiment, "EPOCHS", 1)
    monkeypatch.setattr(experiment.base, "build_epoch_batches", lambda *args: ([(0, 1, 2, 3)], dict(test_only=True)))
    result = experiment.fit(experiment.ARMS[1], .05, initial, cfg, Dataset(), [0], tmp_path,
                            torch.device("cpu"), experiment.contract(), fingerprint)
    assert result["steps"] == 1
    assert result["final_model_state_sha256"] != fingerprint
    assert (tmp_path / result["checkpoint"]).is_file()
    assert (tmp_path / experiment.ARMS[1] / "train_probe_after.json").is_file()
    assert result["auxiliary_pair_totals"]["eligible_pairs"] > 0
