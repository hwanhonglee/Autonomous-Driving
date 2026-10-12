"""HH_261001 - CPU gradient diagnostic on actual TRAIN images; no optimization or DEV evaluation."""

import json
from pathlib import Path

import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.dataset import load_training_examples
from portable_e2e.initial_acceleration_research import capture_drive_latents, initial_acceleration_loss
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
from portable_e2e.simulation_research import FIELDS, load_ah_weights
from portable_e2e.torch_dataset import Common10TorchDataset
from scripts.e2e.dual_recovery_objectives import near_horizon_loss


def main():
    root = Path.cwd()
    weights = root / "storage/experiments/validation/2026-10-01/ah_simulation_weights/AH_simulation.npz"
    model = load_ah_weights(weights, weights.with_suffix(".json"),
        weights_sha256="5fd783def4d67aa8c2abe556187bbfaa06012b1cb2fb5bf04faad760771c9342",
        metadata_sha256="e1558f43b89fb2e967112692f0f348f8f6e457f4bed39b3d592ba02f3cf02de1")
    torch.set_num_threads(2)
    model.requires_grad_(True)
    loaded = load_training_examples(root / "storage/datasets/prepared/carla-common10-trainval-research-20260914-v1",
                                     split="train", check_image_hashes=False)
    dataset = Common10TorchDataset(loaded.examples, model.config, verify_image_sha256=True, split="train")
    results = []
    for index, example in enumerate(dataset.examples):
        if "town07" not in example.episode_id or example.sequence_index not in (0, 35, 37, 39, 40, 80, 300):
            continue
        batch = default_collate([dataset[index]])
        with capture_drive_latents(model) as captured:
            xy, speed, logits = model(*(batch[name] for name in FIELDS))
        base = trajectory_loss(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                               batch["target_valid"], TrajectoryLossConfig(candidate_regret_weight=.1),
                               target_yaw=batch["target_yaw_rad"])
        near = near_horizon_loss(xy, speed, batch["target_xy"], batch["target_speed_mps"],
                                 batch["target_valid"], base["oracle_candidate_index"])
        auxiliary, metrics = initial_acceleration_loss(captured[0], batch["ego_history"][:, -1, 1],
            batch["target_speed_mps"], batch["target_valid"], base["oracle_candidate_index"])
        grad_base = torch.autograd.grad(base["loss"] + .5 * near, model.trajectory_head.bias, retain_graph=True)[0]
        grad_aux = torch.autograd.grad(auxiliary, model.trajectory_head.bias)[0]
        oracle, selected = int(base["oracle_candidate_index"][0]), int(logits[0].argmax())
        candidate = min(oracle, 5)
        results.append(dict(token=example.token, current_speed_mps=example.features[1], oracle=oracle,
            selected=selected, first5_target_speed=batch["target_speed_mps"][0, :5].tolist(),
            first5_oracle_speed=speed[0, oracle, :5].tolist(),
            first5_drive_raw_acceleration=(2.9 * torch.tanh(captured[0][0, candidate, :5, 0])).tolist(),
            first5_base_plus_near_bias_gradient=grad_base.reshape(6, 64, 2)[candidate, :5, 0].tolist(),
            first5_auxiliary_bias_gradient=grad_aux.reshape(6, 64, 2)[candidate, :5, 0].tolist(),
            auxiliary_metrics={key: int(value) for key, value in metrics.items()}))
    print(json.dumps(dict(schema="portable_e2e.ah_train_raw_gradient_audit.v1", train_count=len(dataset),
        examples=results, optimizer_steps=0, dev_metrics_computed=False,
        dataset_validation_may_read_nontrain_contract_records=True), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
