"""HH_261001 - Read-only pinned TRAIN candidate/selector diagnosis, retaining all twelve plans and original targets."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.contract import ContractError
from portable_e2e.dataset import load_training_examples
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
from portable_e2e.simulation_research import FIELDS, load_ah_weights
from portable_e2e.torch_dataset import Common10TorchDataset
from scripts.e2e.research_initial_acceleration_candidate_20261001 import candidate_identity


ROOT = Path(__file__).resolve().parents[2]
PAIR = ROOT / "storage/experiments/training/2026-10-01/ah_initial_acceleration_ab_20261001_v1"
PINS = {
    "AH": (ROOT / "storage/experiments/validation/2026-10-01/ah_simulation_weights/AH_simulation.npz",
        "4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91",
        "5fd783def4d67aa8c2abe556187bbfaa06012b1cb2fb5bf04faad760771c9342",
        "e1558f43b89fb2e967112692f0f348f8f6e457f4bed39b3d592ba02f3cf02de1"),
    "A_ah_continuation": (PAIR / "exports/A_ah_continuation.npz",
        "2cb9f13c7a406e985a38cd7941a1bdba34fa7b4cbfce4280f8b3c4381229dc32",
        "43a8df8fb17d16c08976da94200a69066700c5e0f5326568369b5611305af2f8",
        "3dcc82e38ff908a258131a8f3cd22351392db6f115f2fc79fe0fcb02041880e4"),
    "B_preclamp_acceleration": (PAIR / "exports/B_preclamp_acceleration.npz",
        "daacd96567ef17f36f0fd7791fbad478a68a0341cf5833881a548d980fbe2cae",
        "5c5210e5ec8a860a3f879354f3a90f823aec17f78ea074f75f0ef487603429b2",
        "df4cfe263819c4326628c0913f99acb57d2ac9f71ca213f6449f89f3629b67fe"),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_model(name):
    weights, checkpoint, weight_sha, metadata_sha = PINS[name]
    def load():
        return load_ah_weights(weights, weights.with_suffix(".json"),
                              weights_sha256=weight_sha, metadata_sha256=metadata_sha)
    if name == "AH":
        return load()
    with candidate_identity(checkpoint):
        return load()


def probe_summary(records):
    def group(rows):
        return dict(count=len(rows), selected_equals_oracle=sum(r["selected"] == r["oracle"] for r in rows),
            selected_counts=dict(Counter(r["selected"] for r in rows)), oracle_counts=dict(Counter(r["oracle"] for r in rows)),
            mean_first5_selected_speed_mae=sum(r["near_speed_mae"] for r in rows) / len(rows) if rows else None)
    low = [row for row in records if row["current_speed"] <= .5]
    start = [row for row in low if row["first5_target_speed"][2] > row["current_speed"] + .1]
    return dict(all=group(records), low_speed=group(low), immediate_start=group(start))


def candidate_records(xy, speed, logits, batch, vx):
    costs, result = [], []
    for index in range(12):
        loss = trajectory_loss(xy[:, index:index+1], speed[:, index:index+1], logits[:, index:index+1],
            batch["target_xy"], batch["target_speed_mps"], batch["target_valid"],
            TrajectoryLossConfig(candidate_score_weight=0., candidate_regret_weight=0.),
            target_yaw=batch["target_yaw_rad"])
        cost = float(loss["regression_loss"])
        costs.append(cost)
        points, velocities, score = xy[0, index].tolist(), speed[0, index].tolist(), float(logits[0, index])
        error = None
        try:
            validate_and_select_trajectory([points], [velocities], [score],
                                          RuntimeGateConfig(candidate_count=1), current_speed_mps=vx)
        except ContractError as caught:
            error = str(caught)
        mask = batch["target_valid"][0, :5].to(speed.dtype)
        near_mae = float(((speed[0, index, :5] - batch["target_speed_mps"][0, :5]).abs() * mask).sum()
                         / mask.sum().clamp_min(1))
        result.append(dict(index=index, family="DRIVE" if index < 6 else "STOP", logit=score,
            composite_cost=cost, first5_speed_mae=near_mae, xy_base_m=points, speed_mps=velocities,
            original_runtime_gate_passed=error is None, original_runtime_gate_error=error))
    return torch.tensor(costs), result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("fresh TRAIN audit output required")
    torch.set_num_threads(2)
    loaded = load_training_examples(ROOT / "storage/datasets/prepared/carla-common10-trainval-research-20260914-v1",
                                    split="train", check_image_hashes=False)
    probe_files, probe_groups = {}, {}
    for name in PINS:
        if name == "AH":
            continue
        probe_groups[name] = {}
        for phase in ("before", "after"):
            path = PAIR / name / ("train_probe_" + phase + ".json")
            probe_files[str(path.relative_to(ROOT))] = sha(path)
            probe_groups[name][phase] = probe_summary(json.loads(path.read_text()))
    models = {}
    with torch.no_grad():
        for name in PINS:
            model = load_model(name).eval()
            dataset = Common10TorchDataset(loaded.examples, model.config, verify_image_sha256=True, split="train")
            last = {example.episode_id: example.sequence_index for example in dataset.examples}
            indices = [index for index, example in enumerate(dataset.examples)
                       if example.sequence_index in (0, 35, 37, 39, 40, 80, last[example.episode_id])]
            records, contexts = [], {}
            captured = []
            handle = model.candidate_head.register_forward_pre_hook(lambda module, inputs: captured.append(inputs[0].detach().clone()))
            try:
                for index in indices:
                    batch = default_collate([dataset[index]])
                    captured.clear()
                    xy, speed, logits = model(*(batch[field] for field in FIELDS))
                    vx = float(batch["ego_history"][0, -1, 1])
                    cost, candidates = candidate_records(xy, speed, logits, batch, vx)
                    oracle, selected = int(cost.argmin()), int(logits[0].argmax())
                    probability = logits[0].softmax(0)
                    onehot = torch.zeros_like(probability); onehot[oracle] = 1
                    logit_gradient = .1 * (probability - onehot) + .1 * probability * (cost - (probability * cost).sum())
                    example = dataset.examples[index]
                    records.append(dict(token=example.token, sequence_index=example.sequence_index,
                        current_speed_mps=vx, selected=selected, oracle=oracle,
                        selected_minus_oracle_logit=float(logits[0, selected] - logits[0, oracle]),
                        selected_minus_oracle_cost=float(cost[selected] - cost[oracle]),
                        exact_base_score_logit_gradient=logit_gradient.tolist(),
                        first5_target_speed=batch["target_speed_mps"][0, :5].tolist(),
                        target_xy=batch["target_xy"][0].tolist(), target_speed=batch["target_speed_mps"][0].tolist(),
                        target_valid=batch["target_valid"][0].tolist(), candidates=candidates))
                    if example.sequence_index in (0, 35, 37):
                        contexts[(example.episode_id, example.sequence_index)] = (captured[0], batch)
            finally:
                handle.remove()
            similarities = []
            for episode in last:
                for left, right in ((0, 35), (35, 37)):
                    if (episode, left) not in contexts or (episode, right) not in contexts:
                        continue
                    af, a = contexts[(episode, left)]; bf, b = contexts[(episode, right)]
                    differences = {field: float((a[field].to(torch.float32)-b[field].to(torch.float32)).abs().max()) for field in FIELDS}
                    similarities.append(dict(episode=episode, left_sequence=left, right_sequence=right,
                        fused_cosine_similarity=float(torch.nn.functional.cosine_similarity(af, bf)),
                        fused_max_absolute_difference=float((af-bf).abs().max()), input_max_absolute_differences=differences))
            models[name] = dict(checkpoint_sha256=PINS[name][1], weights_sha256=PINS[name][2],
                                metadata_sha256=PINS[name][3], records=records, stopped_context_similarity=similarities)
    report = dict(schema="portable_e2e.initial_acceleration_train_selector_audit.v1", optimizer_steps=0,
        dev_metrics_computed=False, test_data_opened=False, local_train_examples=len(loaded.examples),
        recomputation_scope="fixed probes from three original TRAIN episodes locally available; not all2787K1 anchors",
        dataset_contract_validation_may_read_nontrain_records=True, probe_input_sha256=probe_files,
        fixed_full_k1_probe_summaries=probe_groups, models=models, training_data_approved=False,
        moving_runtime_candidate_is_not_expert_truth_or_obstacle_safety_proof=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(output=str(args.output), models={name: len(item["records"]) for name, item in models.items()},
                         optimizer_steps=0, dev_metrics_computed=False)))


if __name__ == "__main__":
    main()
