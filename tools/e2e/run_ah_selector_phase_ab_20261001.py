"""HH_261001 - Fit only original AH selector heads in a fixed fully covering phase-weighting pair."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
from torch.utils.data import DataLoader

from portable_e2e import losses as loss_module
from portable_e2e.train import _read_checkpoint_file
from scripts.e2e import run_ah_initial_acceleration_ab_20261001 as previous
from scripts.e2e import selector_phase_research_20261001 as phase
from scripts.e2e import diagnose_stopmix_training_oracles as projection


base = previous.base
SCHEMA = "portable_e2e.selector_phase_ab_20261001.v1"
ARMS = ("S0_uniform_selector", "S1_phase_balanced_selector")
STEPS, POSITIONS, LEARNING_RATE = 3000, 12000, 1.e-4
PLAN_RELATIVE = "docs/assets/validation/2026-10-01/initial_acceleration_live_pair_v1/SELECTOR_PHASE_AB_PLAN.md"
PLAN_SHA256 = "bb719b049474c5a47816b6257d09c8b85e8bf61808fe58313cc2e9f9156188d4"


def contract():
    return dict(schema=SCHEMA, parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
        arms={ARMS[0]: "ordinary per-example mean", ARMS[1]: "0.5 ordinary mean +0.5 equal-phase mean"},
        steps_per_arm=STEPS, exposures_per_arm=POSITIONS, batch_size=4, seed=20260906,
        optimizer="AdamW", learning_rate=LEARNING_RATE, weight_decay=1.e-4, gradient_clip=5.,
        trainable_parameters=3084, trainable_names=list(phase.HEAD_NAMES), generator_and_normalization_frozen=True,
        architecture_changed=False, parent_optimizer_resumed=False, sampling_changed_vs_previous_full_model_pair=True,
        shared_sampling="seeded cyclic-uniform permutations, four complete2787 passes plus852; boundary duplicates moved to cycle end",
        phase_priority=list(phase.PHASES), stop_definition="frozen AH full-composite oracle>=6 OR all original valid target speeds<=0.1",
        immediate_start_definition="otherwise abs(current_vx)<=0.5, first3 target steps valid, target_v(.3)>max(0,current_vx)+0.1",
        low_other_definition="otherwise abs(current_vx)<=0.5", moving_definition="all remaining anchors",
        phase_weight="0.5+0.5*12000/(4*scheduled_phase_exposures), frozen once before either fit; >=0.5",
        loss="0.1 unchanged full-composite oracle CE +0.1 detached expected composite regret",
        no_new_oracle=True, positive_speed_candidate_used_as_oracle=False, dataset_rows_or_labels_modified=False,
        train_cache_only_before_both_finals=True, dev_during_training=False, dev_final_passes_per_arm=1,
        early_stopping=False, hyperparameter_sweep=False, test_evaluated=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
        limitation="Bounded AH phase-imbalance hypothesis; does not repair original target feasibility or prove a launch cure.")


def make_cache(model, dataset, device):
    model.requires_grad_(False); model.eval()
    projected, proof = projection.projected_costs(loss_module)
    arrays, contexts = {}, []
    handle = model.candidate_head.register_forward_pre_hook(lambda module, inputs: contexts.append(inputs[0].detach()))
    try:
        with torch.no_grad():
            for raw in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
                batch = base._move_batch(raw, device)
                contexts.clear()
                xy, speed, logits = model(*(batch[name] for name in previous.FIELDS))
                costs = projected(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                    batch["target_valid"], base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
                native = base.trajectory_loss(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                    batch["target_valid"], base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
                base.require(torch.equal(costs.argmin(1), native["oracle_candidate_index"]), "projected oracle differs")
                base.require(torch.equal(costs.min(1).values.mean(), native["regression_loss"]), "projected composite differs")
                values = dict(fused=contexts[0], xy=xy, speed=speed, logits=logits, costs=costs,
                    current=batch["ego_history"][:, -1, 1], target_xy=batch["target_xy"],
                    target_speed=batch["target_speed_mps"], target_yaw=batch["target_yaw_rad"], valid=batch["target_valid"])
                for name, tensor in values.items():
                    base.require(bool(torch.isfinite(tensor).all()), "nonfinite TRAIN cache")
                    arrays.setdefault(name, []).append(tensor.detach().clone())
    finally:
        handle.remove()
    result = {name: torch.cat(parts).contiguous() for name, parts in arrays.items()}
    base.require(len(result["fused"]) == len(dataset), "cache omitted TRAIN anchors")
    return result, proof


def train_report(model, cache, phases):
    with torch.no_grad():
        logits = torch.cat((model.candidate_head(cache["fused"]), model.stop_candidate_head(cache["fused"])), 1)
        selected, oracle = logits.argmax(1), cache["costs"].argmin(1)
        rows = torch.arange(len(selected), device=selected.device)
        first5 = cache["valid"][:, :5].to(torch.float32)
        mae = ((cache["speed"][rows, selected, :5] - cache["target_speed"][:, :5]).abs() * first5).sum(1) / first5.sum(1).clamp_min(1.)
        regret = cache["costs"][rows, selected] - cache["costs"].min(1).values
        groups = {}
        for identity, name in enumerate(phase.PHASES):
            mask = phases == identity
            groups[name] = dict(count=int(mask.sum()), oracle_agreement=int((selected[mask] == oracle[mask]).sum()),
                selected_counts=dict(Counter(selected[mask].tolist())), oracle_counts=dict(Counter(oracle[mask].tolist())),
                mean_selected_composite_regret=float(regret[mask].mean()), mean_first5_speed_mae=float(mae[mask].mean()))
        return dict(phase_groups=groups, sample_count=len(selected), selected_indices=selected.tolist(), oracle_indices=oracle.tolist(),
                    mean_composite_regret=float(regret.mean()), mean_first5_speed_mae=float(mae.mean()), dev_evaluated=False)


def fit(arm, initial, cfg, cache, phases, weights, batches, output, device, declaration, cache_hash):
    base._seed_everything(base.SEED, device)
    model = base.PerspectiveTrajectoryModel(cfg).to(device)
    model.load_state_dict(initial, strict=True)
    model.requires_grad_(False); model.eval()
    for name, parameter in model.named_parameters():
        parameter.requires_grad_(name in phase.HEAD_NAMES)
    trainable = [p for p in model.parameters() if p.requires_grad]
    base.require(sum(p.numel() for p in trainable) == 3084, "only original twin Linear heads may train")
    generator_hash = base.tensor_digest(phase.generator_state(model.state_dict()))
    base.write_new(output / arm / "train_before.json", train_report(model, cache, phases))
    folder = output / arm / "training"; folder.mkdir(parents=True)
    optimizer = torch.optim.AdamW(trainable, lr=LEARNING_RATE, weight_decay=1.e-4)
    started = time.monotonic()
    with (folder / "history.jsonl").open("x") as stream:
        for step, indices in enumerate(batches, 1):
            index = torch.tensor(indices, dtype=torch.long, device=device)
            optimizer.zero_grad(set_to_none=True)
            fused = cache["fused"][index].detach()
            logits = torch.cat((model.candidate_head(fused), model.stop_candidate_head(fused)), 1)
            weight = weights[phases[index]] if arm == ARMS[1] else torch.ones(len(index), device=device)
            loss = phase.selector_loss(logits, cache["costs"][index], weight)
            base.require(bool(torch.isfinite(loss)), "nonfinite selector loss")
            loss.backward()
            norm = torch.nn.utils.clip_grad_norm_(trainable, 5.)
            base.require(bool(torch.isfinite(norm)), "nonfinite selector gradient")
            optimizer.step()
            base.require(all(bool(torch.isfinite(p).all()) for p in trainable)
                         and base._nested_tensors_are_finite(optimizer.state_dict()), "nonfinite selector state")
            if step % 100 == 0:
                row = dict(arm=arm, step=step, loss=float(loss.detach()), gradient_norm=float(norm), elapsed_s=time.monotonic()-started)
                stream.write(json.dumps(row) + "\n"); stream.flush(); print(json.dumps(row), flush=True)
    base.require(step == STEPS, "selector fixed budget incomplete")
    base.require(generator_hash == base.tensor_digest(phase.generator_state(model.state_dict())), "frozen generator or buffer changed")
    base.require(cache_hash == base.tensor_digest(cache), "immutable TRAIN cache changed")
    base.write_new(output / arm / "train_after.json", train_report(model, cache, phases))
    payload = dict(checkpoint_schema=SCHEMA, arm=arm, parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
        model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
        final_model_state_sha256=base.tensor_digest(model.state_dict()), initial_model_state_sha256=base.tensor_digest(initial),
        fit=dict(global_step=STEPS, samples_seen=POSITIONS, optimizer_state_included=False, optimizer_resumed=False),
        training_dataset_fingerprint_sha256=declaration["training_dataset_fingerprint_sha256"],
        frozen_generator_state_sha256=generator_hash, frozen_train_candidate_cache_sha256=cache_hash,
        research_objective=declaration, exposure_order_sha256=declaration["exposure_order_sha256"],
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    path = folder / "final_checkpoint.pt"; base.save_new(path, payload)
    receipt = dict(checkpoint=str(path.relative_to(output)), checkpoint_sha256=base.digest(path), steps=STEPS,
        elapsed_training_s=time.monotonic()-started, generator_bit_identity=True, cache_bit_identity=True,
        frozen_generator_state_sha256=generator_hash, frozen_train_candidate_cache_sha256=cache_hash,
        exposure_order_sha256=declaration["exposure_order_sha256"])
    base.write_new(output / arm / "training_receipt.json", receipt)
    del model, optimizer
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-plan", type=Path, required=True)
    parser.add_argument("--parent-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    root = Path.cwd().resolve()
    workspace = base.check_run_environment(dict(repo_root=str(root), output_root=str(args.output)))
    base.require(not args.output.exists(), "fresh selector A/B output required")
    base.require(base.digest(args.baseline_plan) == previous.BASE_PLAN_SHA256, "frozen K plan differs")
    plan = base.read_json(args.baseline_plan); base.validate_plan(plan)
    sources = previous.verify_sources(root, plan)
    base.require(base.digest(root / PLAN_RELATIVE) == PLAN_SHA256, "predeclared selector plan differs")
    sources[PLAN_RELATIVE] = PLAN_SHA256
    for path in (Path(__file__), Path(phase.__file__), Path(projection.__file__)):
        sources[str(path.relative_to(root))] = base.digest(path)
    base.require(base.digest(args.parent_checkpoint) == previous.AH_CHECKPOINT_SHA256, "exact original AH parent required")
    payload, digest = _read_checkpoint_file(args.parent_checkpoint, torch.device("cpu"))
    base.require(digest == previous.AH_CHECKPOINT_SHA256 and payload["checkpoint_schema"] == "portable_e2e.near_horizon_ah.v1", "secure AH identity differs")
    cfg = base.ModelConfig.from_mapping(payload["model_config"]); initial = payload["model_state_dict"]
    base.require(base.tensor_digest(initial) == payload["final_model_state_sha256"], "AH tensor digest differs")
    view = plan["views"][base.ARMS[1]]
    loaded, examples, scope = base._load_view(Path(view["path"]), "train", base.ARMS[1])
    base._verify_loaded_view(view, loaded, examples)
    base.require(scope == view["scope_precheck"] and len(examples) == 2787, "complete original TRAIN scope differs")
    dataset = base.Common10TorchDataset(examples, cfg, verify_image_sha256=True, split="train")
    batches = phase.schedule(len(dataset))
    tokens = [examples[index].token for batch in batches for index in batch]
    token_hash = hashlib.sha256(b"".join(token.encode() + b"\0" for token in tokens)).hexdigest()
    declaration = dict(contract(), source_sha256=sources, baseline_plan_sha256=previous.BASE_PLAN_SHA256,
        training_count=len(dataset), unique_scheduled_train_count=len(set(tokens)), exposure_order_sha256=token_hash,
        training_manifest_sha256=view["dataset_manifest_sha256"], training_dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        dataset_validation_may_read_nontrain_contract_records=True)
    if args.validate_only:
        print(json.dumps(dict(status="CPU_PREFLIGHT_PASS_NO_TRAINING", **declaration), indent=2)); return
    with base.gpu0_lease(workspace):
        gpu = base.preflight_gpu(); device = torch.device("cuda:0")
        args.output.mkdir(parents=True)
        base.write_new(args.output / "declaration.json", declaration)
        base.write_new(args.output / "started.json", dict(status="FIXED_SELECTOR_PAIRED_TRAINING", gpu=gpu, started_at=base.utc_now()))
        for name in sources:
            base.copy_new(root / name, args.output / "source" / name)
        model = base.PerspectiveTrajectoryModel(cfg).to(device); model.load_state_dict(initial, strict=True)
        cache, projection_proof = make_cache(model, dataset, device)
        phases = phase.phase_ids(cache["current"], cache["target_speed"], cache["valid"], cache["costs"].argmin(1))
        weights, exposures = phase.phase_weights(phases, batches)
        cache_hash = base.tensor_digest(cache)
        with (args.output / "frozen_train_cache.npz").open("xb") as stream:
            np.savez_compressed(stream, **{name: value.cpu().numpy() for name, value in cache.items()}, phase_ids=phases.cpu().numpy())
        base.write_new(args.output / "cache_receipt.json", dict(cache_tensor_sha256=cache_hash,
            npz_sha256=base.digest(args.output / "frozen_train_cache.npz"), projection_proof=projection_proof,
            phase_counts=dict(zip(phase.PHASES, torch.bincount(phases, minlength=4).tolist())),
            scheduled_phase_exposures=dict(zip(phase.PHASES, exposures.tolist())),
            phase_weights=dict(zip(phase.PHASES, weights.tolist())), all_train_anchors_retained=True,
            source_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256))
        del model
        training = {arm: fit(arm, initial, cfg, cache, phases, weights, batches, args.output, device, declaration, cache_hash) for arm in ARMS}
        base.require(training[ARMS[0]]["frozen_generator_state_sha256"] == training[ARMS[1]]["frozen_generator_state_sha256"], "paired frozen generator differs")
        base.write_new(args.output / "both_final_checkpoints_complete.json", training)
        # HH_261001 - Only now open DEV evaluation inputs; reuse exact metric code via process-local schema binding.
        dev_view = plan["views"]["Town03_DEV"]
        dev, dev_examples, dev_scope = base._load_view(Path(dev_view["path"]), "val", "Town03_DEV")
        base._verify_loaded_view(dev_view, dev, dev_examples)
        base.require(dev_scope == dev_view["scope_precheck"], "DEV view differs")
        old_schema = previous.SCHEMA
        try:
            previous.SCHEMA = SCHEMA
            evaluations = {arm: previous.evaluate_final(arm, training[arm], dev, cfg, args.output, device) for arm in ARMS}
        finally:
            previous.SCHEMA = old_schema
        source_match = all(base.digest(root/name) == expected for name, expected in sources.items())
        base.require(source_match, "source changed during selector pair")
        summary = dict(schema=SCHEMA, status="COMPLETE_NOT_PROMOTED", training=training, evaluations=evaluations,
            S1_vs_S0=base.paired_nonregression(evaluations[ARMS[0]], evaluations[ARMS[1]]),
            strict_gates={arm: base.strict_gate(evaluations[arm]) for arm in ARMS}, source_hashes_match=source_match,
            model_promoted=False, training_data_approved=False, vehicle_control_approved=False, completed_at=base.utc_now())
        base.write_new(args.output / "summary.json", summary)
        print(json.dumps(dict(status=summary["status"], training=training, strict_gates=summary["strict_gates"])), flush=True)


if __name__ == "__main__":
    main()
