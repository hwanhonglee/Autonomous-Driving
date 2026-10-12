"""HH_261001 - Fixed 3000-step AH continuation pair; one pre-clamp auxiliary is the sole intervention."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import torch
from torch.utils.data import DataLoader
from torch.utils.data.dataloader import default_collate

from portable_e2e import initial_acceleration_research as acceleration
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.runtime_contract import RuntimeGateConfig
from portable_e2e.simulation_research import AH_CHECKPOINT_SHA256, FIELDS
from portable_e2e.train import _read_checkpoint_file
from scripts.e2e import run_moving_multiroute_recovery_k as base
from scripts.e2e.dual_recovery_objectives import near_horizon_loss


SCHEMA = "portable_e2e.initial_acceleration_ab_20261001.v1"
ARMS = ("A_ah_continuation", "B_preclamp_acceleration")
STEPS = 3000
EPOCHS = 4
LEARNING_RATE = 3e-5
BASE_PLAN_SHA256 = "4c52532bedca9e146be0d06fb98f9e27b7263ce8cd13840426f81fa88dce4075"
NEAR_SOURCE_SHA256 = "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15"


def contract():
    """HH_261001 - Freeze settings in code; CLI offers no sweep, early stop or checkpoint-selection options."""
    return dict(schema=SCHEMA, parent_checkpoint_sha256=AH_CHECKPOINT_SHA256,
        arms={ARMS[0]: 0., ARMS[1]: acceleration.WEIGHT}, optimizer_steps_per_arm=STEPS,
        logical_epochs=EPOCHS, batch_size=4, exposures_per_arm=12000, seed=20260906,
        optimizer="AdamW", learning_rate=LEARNING_RATE, weight_decay=1e-4,
        maximum_gradient_norm=5., optimizer_reset_each_arm=True,
        base_loss=base.LOSS_CONFIG.to_dict(), near_horizon_weight=.5, near_horizon_points=20,
        acceleration_points=acceleration.POINTS, acceleration_target="adjacent recorded speed difference / 0.1",
        acceleration_mask="both labels valid; speeds in [0,30/3.6]; abs(acceleration)<=2.9; full-horizon oracle DRIVE only",
        impossible_targets_clipped=False, decoder_changed=False, new_head=False,
        full_model_trainable=True, training_data_and_original_masks_unchanged=True,
        sampling="exact first four K1 epoch schedules: two straight, one left, one right per batch",
        dev_during_training=False, final_dev_after_both_checkpoints=True,
        final_dev_passes_per_arm=1, test_evaluated=False, independent_validation=False,
        early_stopping=False, automatic_retry=False, hyperparameter_sweep=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
        limitation="Tests a pre-clamp gradient repair, not a remedy for scarce physically valid start trajectories.")


def verify_sources(root, plan):
    """HH_261001 - Require frozen K bytes under the canonical layout without rewriting the historical plan."""
    result = {}
    for logical, expected in plan["source_sha256"].items():
        relative = ("learning/" + logical if logical.startswith("portable_e2e/")
                    else "tools/" + logical.removeprefix("scripts/"))
        path = root / relative
        base.require(base.digest(path) == expected, "frozen dependency changed: " + relative)
        result[relative] = expected
    extras = [Path(__file__), Path(acceleration.__file__),
              root / "tools/e2e/dual_recovery_objectives.py", root / "tools/workspace_python.py",
              root / "tools/canonical_workspace.py", root / "learning/portable_e2e/simulation_research.py",
              root / "learning/portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"]
    base.require(base.digest(extras[2]) == NEAR_SOURCE_SHA256, "frozen AH near loss source differs")
    for path in extras:
        result[str(path.relative_to(root))] = base.digest(path)
    return result


def selected_train_indices(examples):
    """HH_261001 - Fixed annotation-index probes cover warmup/start/cruise/tail and every TRAIN episode."""
    last = {}
    for index, example in enumerate(examples):
        last[example.episode_id] = index
    return sorted(set(last.values()) | {index for index, example in enumerate(examples)
                    if example.sequence_index in (0, 35, 37, 39, 40, 80)})


def probe(model, dataset, indices, device):
    model.eval()
    records = []
    with torch.no_grad():
        for index in indices:
            batch = base._move_batch(default_collate([dataset[index]]), device)
            with acceleration.capture_drive_latents(model) as captured:
                xy, speed, logits = model(*(batch[name] for name in FIELDS))
            loss = base.trajectory_loss(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                batch["target_valid"], base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
            oracle, selected = int(loss["oracle_candidate_index"][0]), int(logits[0].argmax())
            auxiliary, metrics = acceleration.initial_acceleration_loss(captured[0], batch["ego_history"][:, -1, 1],
                batch["target_speed_mps"], batch["target_valid"], loss["oracle_candidate_index"])
            valid = batch["target_valid"][0, :5].to(speed.dtype)
            near_mae = ((speed[0, selected, :5] - batch["target_speed_mps"][0, :5]).abs() * valid).sum() / valid.sum().clamp_min(1)
            records.append(dict(token=dataset.examples[index].token, selected=selected, oracle=oracle,
                current_speed=float(batch["ego_history"][0, -1, 1]), first5_speed=speed[0, selected, :5].tolist(),
                first5_target_speed=batch["target_speed_mps"][0, :5].tolist(), near_speed_mae=float(near_mae),
                raw_acceleration_loss=float(auxiliary), metrics={key: int(value) for key, value in metrics.items()}))
    return records


def fit(arm, weight, initial, cfg, dataset, indices, output, device, declaration, initial_hash):
    base._seed_everything(base.SEED, device)
    model = base.PerspectiveTrajectoryModel(cfg).to(device)
    model.load_state_dict(initial, strict=True)
    base.require(base.tensor_digest(model.state_dict()) == initial_hash, "paired initialization differs")
    folder = output / arm / "training"
    folder.mkdir(parents=True)
    base.write_new(output / arm / "train_probe_before.json", probe(model, dataset, indices, device))
    optimizer = torch.optim.AdamW(model.parameters(), lr=LEARNING_RATE, weight_decay=base.WEIGHT_DECAY)
    model.train()
    step = 0
    started = time.monotonic()
    schedules = []
    tokens = hashlib.sha256()
    accumulated = Counter()
    with (folder / "history.jsonl").open("x") as log:
        for epoch in range(EPOCHS):
            batches, proof = base.build_epoch_batches(dataset.examples, base.ARMS[1], epoch)
            schedules.append(proof)
            for raw in DataLoader(dataset, batch_sampler=batches, num_workers=0):
                batch = base._move_batch(raw, device)
                optimizer.zero_grad(set_to_none=True)
                with acceleration.capture_drive_latents(model) as captured:
                    xy, speed, logits = model(*(batch[name] for name in FIELDS))
                base.require(len(captured) == 1, "one forward head observation required")
                loss = base.trajectory_loss(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                    batch["target_valid"], base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
                near = near_horizon_loss(xy, speed, batch["target_xy"], batch["target_speed_mps"],
                                         batch["target_valid"], loss["oracle_candidate_index"])
                auxiliary, metrics = acceleration.initial_acceleration_loss(captured[0], batch["ego_history"][:, -1, 1],
                    batch["target_speed_mps"], batch["target_valid"], loss["oracle_candidate_index"])
                total = loss["loss"] + .5 * near + weight * auxiliary
                base.require(bool(torch.isfinite(total)), "nonfinite paired loss")
                total.backward()
                norm = torch.nn.utils.clip_grad_norm_(model.parameters(), base.MAXIMUM_GRADIENT_NORM)
                base.require(bool(torch.isfinite(norm)), "nonfinite paired gradient")
                optimizer.step()
                base.require(base._nested_tensors_are_finite(model.state_dict()), "nonfinite paired model")
                base.require(base._nested_tensors_are_finite(optimizer.state_dict()), "nonfinite paired optimizer")
                step += 1
                for token in raw["sample_id"]:
                    tokens.update(token.encode()); tokens.update(b"\0")
                accumulated.update({key: int(value) for key, value in metrics.items()})
                report = dict(arm=arm, step=step, base_loss=float(loss["loss"].detach()),
                    near_loss=float(near.detach()), acceleration_loss=float(auxiliary.detach()),
                    total_loss=float(total.detach()), gradient_norm=float(norm),
                    elapsed_s=time.monotonic() - started)
                if step % 100 == 0:
                    log.write(json.dumps(report) + "\n"); log.flush()
                    print(json.dumps(report), flush=True)
    base.require(step == STEPS, "fixed training budget incomplete")
    payload = dict(checkpoint_schema=SCHEMA, arm=arm, model_config=cfg.to_dict(),
        model_state_dict=base._clone_state(model.state_dict()), final_model_state_sha256=base.tensor_digest(model.state_dict()),
        initial_model_state_sha256=initial_hash, parent_checkpoint_sha256=AH_CHECKPOINT_SHA256,
        fit=dict(global_step=STEPS, samples_seen=STEPS * 4, optimizer_state_included=False, optimizer_resumed=False),
        training_dataset_fingerprint_sha256=dataset.fingerprint_sha256, research_objective=declaration,
        acceleration_weight=weight, exposure_order_sha256=tokens.hexdigest(), training_data_approved=False,
        vehicle_control_approved=False, model_promoted=False)
    checkpoint = folder / "final_checkpoint.pt"
    base.save_new(checkpoint, payload)
    base.write_new(output / arm / "train_probe_after.json", probe(model, dataset, indices, device))
    result = dict(checkpoint=str(checkpoint.relative_to(output)), checkpoint_sha256=base.digest(checkpoint),
        steps=step, elapsed_training_s=time.monotonic() - started, exposure_order_sha256=tokens.hexdigest(),
        auxiliary_pair_totals=dict(accumulated), epoch_schedules=schedules,
        initial_model_state_sha256=initial_hash, final_model_state_sha256=payload["final_model_state_sha256"])
    base.write_new(output / arm / "training_receipt.json", result)
    del model, optimizer
    torch.cuda.empty_cache()
    return result


def evaluate_final(arm, receipt, loaded, cfg, output, device):
    """HH_261001 - Reuse frozen metric and gate definitions after both fixed finals exist; secure checkpoint load."""
    checkpoint = output / receipt["checkpoint"]
    payload, digest = _read_checkpoint_file(checkpoint, torch.device("cpu"))
    base.require(digest == receipt["checkpoint_sha256"] and payload["checkpoint_schema"] == SCHEMA
                 and payload["fit"]["global_step"] == STEPS and payload["arm"] == arm, "final identity differs")
    model = base.PerspectiveTrajectoryModel(cfg).to(device)
    model.load_state_dict(payload["model_state_dict"], strict=True)
    base.require(base.tensor_digest(model.state_dict()) == payload["final_model_state_sha256"], "final state differs")
    model.requires_grad_(False); model.eval()
    dataset = base.Common10TorchDataset(loaded.examples, cfg, verify_image_sha256=True, split="val")
    base.require(base.episode_inventory(dataset.examples) == base.DEV_EPISODE_COUNTS, "frozen DEV differs")
    sums = {name: {} for name in ("town03_right_turn_dev", "town03_straight_dev", "combined_val")}
    counts = {name: {} for name in sums}
    gate = RuntimeGateConfig(candidate_count=12, future_points=64)
    geometry = base._AuditAccumulator(gate)
    with torch.no_grad():
        for raw in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
            batch = base._move_batch(raw, device)
            xy, speed, logits = model(*(batch[name] for name in FIELDS))
            tensors = dict(candidate_xy=xy, candidate_speed=speed, candidate_logits=logits,
                target_xy=batch["target_xy"], target_speed=batch["target_speed_mps"],
                target_valid=batch["target_valid"], target_yaw=batch["target_yaw_rad"])
            base._accumulate_batch_metrics(sums["combined_val"], counts["combined_val"], loss_config=base.LOSS_CONFIG, **tensors)
            for name, episode in zip(("town03_right_turn_dev", "town03_straight_dev"), base.DEV_EPISODE_COUNTS):
                positions = [i for i, value in enumerate(raw["episode_id"]) if value == episode]
                if positions:
                    selection = torch.tensor(positions, dtype=torch.long, device=device)
                    base._accumulate_batch_metrics(sums[name], counts[name], loss_config=base.LOSS_CONFIG,
                                                   **{key: value.index_select(0, selection) for key, value in tensors.items()})
            for x, s, l, current in zip(xy.tolist(), speed.tolist(), logits.tolist(), batch["ego_history"][:, -1, 1].tolist()):
                geometry.add(base.audit_prediction(x, s, l, gate, current_speed_mps=current))
    reports = {name: base._metric_report(sums[name], counts[name]) for name in sums}
    for name, expected in (("town03_right_turn_dev", 337), ("town03_straight_dev", 328), ("combined_val", 665)):
        base.require(reports[name]["metric_counts"]["selected_ade_m"] == expected, "final DEV count differs")
    result = dict(schema=SCHEMA, arm=arm, checkpoint_sha256=digest, subgroups=reports, geometry=geometry.report(),
        evaluation_pass_index=1, sample_count=665, independent_validation=False, test_evaluated=False,
        model_promoted=False, vehicle_control_approved=False)
    base.write_new(output / arm / "final_dev_evaluation.json", result)
    del model
    torch.cuda.empty_cache()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-plan", type=Path, required=True)
    parser.add_argument("--parent-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    root = Path.cwd().resolve()
    workspace = base.check_run_environment(dict(repo_root=str(root), output_root=str(args.output)))
    base.require(not args.output.exists(), "fresh A/B output required")
    base.require(base.digest(args.baseline_plan) == BASE_PLAN_SHA256, "frozen K plan differs")
    plan = base.read_json(args.baseline_plan)
    base.validate_plan(plan)
    source_hashes = verify_sources(root, plan)
    base.require(base.digest(args.parent_checkpoint) == AH_CHECKPOINT_SHA256, "parent AH bytes differ")
    payload, digest = _read_checkpoint_file(args.parent_checkpoint, torch.device("cpu"))
    base.require(digest == AH_CHECKPOINT_SHA256 and payload["checkpoint_schema"] == "portable_e2e.near_horizon_ah.v1",
                 "secure parent AH identity differs")
    cfg = base.ModelConfig.from_mapping(payload["model_config"])
    initial = payload["model_state_dict"]
    initial_hash = base.tensor_digest(initial)
    base.require(initial_hash == payload["final_model_state_sha256"], "parent tensor digest differs")
    view = plan["views"][base.ARMS[1]]
    loaded, examples, scope = base._load_view(Path(view["path"]), "train", base.ARMS[1])
    base._verify_loaded_view(view, loaded, examples)
    base.require(scope == view["scope_precheck"], "TRAIN view scope differs")
    dataset = base.Common10TorchDataset(examples, cfg, verify_image_sha256=True, split="train")
    indices = selected_train_indices(examples)
    declaration = dict(contract(), source_sha256=source_hashes, baseline_plan_sha256=BASE_PLAN_SHA256,
        training_count=len(dataset), training_manifest_sha256=view["dataset_manifest_sha256"],
        training_dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        probe_tokens=[examples[index].token for index in indices],
        dataset_validation_may_read_nontrain_contract_records=True)
    if args.validate_only:
        print(json.dumps(dict(status="CPU_PREFLIGHT_PASS_NO_TRAINING", **declaration), indent=2))
        return
    with base.gpu0_lease(workspace):
        gpu = base.preflight_gpu()
        device = torch.device("cuda:0")
        args.output.mkdir(parents=True)
        base.write_new(args.output / "declaration.json", declaration)
        base.write_new(args.output / "started.json", dict(status="FIXED_PAIRED_TRAINING", gpu=gpu, started_at=base.utc_now()))
        for name, expected in source_hashes.items():
            base.copy_new(root / name, args.output / "source" / name)
        training = {arm: fit(arm, declaration["arms"][arm], initial, cfg, dataset, indices,
                             args.output, device, declaration, initial_hash) for arm in ARMS}
        base.require(training[ARMS[0]]["exposure_order_sha256"] == training[ARMS[1]]["exposure_order_sha256"],
                     "paired exposure order differs")
        base.write_new(args.output / "both_final_checkpoints_complete.json", training)
        # HH_261001 - No DEV metric computation occurs until both irreversible fixed-budget finals are saved.
        dev_view = plan["views"]["Town03_DEV"]
        dev, dev_examples, dev_scope = base._load_view(Path(dev_view["path"]), "val", "Town03_DEV")
        base._verify_loaded_view(dev_view, dev, dev_examples)
        base.require(dev_scope == dev_view["scope_precheck"], "DEV view scope differs")
        evaluations = {arm: evaluate_final(arm, training[arm], dev, cfg, args.output, device) for arm in ARMS}
        source_match = all(base.digest(root / name) == digest for name, digest in source_hashes.items())
        base.require(source_match, "source changed during paired training")
        summary = dict(schema=SCHEMA, status="COMPLETE_NOT_PROMOTED", training=training,
            evaluations=evaluations, B_vs_A=base.paired_nonregression(evaluations[ARMS[0]], evaluations[ARMS[1]]),
            strict_gates={arm: base.strict_gate(evaluations[arm]) for arm in ARMS},
            source_hashes_match=source_match, model_promoted=False, vehicle_control_approved=False,
            training_data_approved=False, completed_at=base.utc_now())
        base.write_new(args.output / "summary.json", summary)
        print(json.dumps(dict(status=summary["status"], training=training, strict_gates=summary["strict_gates"])), flush=True)


if __name__ == "__main__":
    main()
