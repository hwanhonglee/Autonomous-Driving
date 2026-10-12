"""HH_261001 - Fixed full-model pair, removing only source-owned warmup anchor supervision."""

import argparse
import json
from pathlib import Path
import time

import torch
from torch.utils.data import DataLoader
from torch.utils.data.dataloader import default_collate

from portable_e2e.train import _read_checkpoint_file
from portable_e2e.runtime_contract import RuntimeGateConfig
from portable_e2e import losses as loss_module
from scripts.e2e import diagnose_stopmix_training_oracles as projection
from scripts.e2e import run_ah_initial_acceleration_ab_20261001 as previous
from scripts.e2e import warmup_supervision_research_20261001 as supervision


base = previous.base
SCHEMA = "portable_e2e.warmup_supervision_ab_20261001.v1"
ARMS = ("W0_full_supervision", "W1_warmup_anchor_off")
PLAN_RELATIVE = "docs/assets/validation/2026-10-01/selector_phase_ab/WARMUP_SUPERVISION_REPAIR_PROPOSAL.md"
PLAN_SHA256 = "fe80269a6b634073586f75c132159de5bad4b631a807f0d8368ce4f3bb98e764"
SCHEDULED_STEPS, UPDATES = 3000, 2999


def contract():
    return dict(schema=SCHEMA, parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
        arms={ARMS[0]: "all executed anchor weights one", ARMS[1]: "warmup zero; all other weights11996/11549"},
        full_model_trainable=True, architecture_changed=False, decoder_changed=False,
        optimizer="fresh AdamW per arm", learning_rate=3e-5, weight_decay=1e-4, gradient_clip=5.,
        seed=20260906, batch_size=4, scheduled_steps=SCHEDULED_STEPS, optimizer_updates=UPDATES,
        scheduled_samples=12000, forward_samples=11996, skipped_schedule_steps_one_based=[1693],
        masked_warmup_unique=105, active_unique=2682, active_normalization=supervision.ACTIVE_WEIGHT,
        original_stop_hold_count=200, all_stop_hold_supervision_preserved=True,
        base_loss=base.LOSS_CONFIG.to_dict(), near_horizon_weight=.5, near_horizon_points=20,
        oracle="unchanged dynamic full composite argmin for each row", initial_acceleration_auxiliary=False,
        sampler="same cyclic-uniform12000 order as selector pair; differs old balanced K1 full-model sampler",
        history_inputs_retained=True, labels_clipped=False, future_validity_masks_changed=False,
        driving_futures_crossing_tail_changed=False, capture_phase_added_as_input=False,
        dev_during_training=False, final_dev_after_both_checkpoints=True, final_dev_passes_per_arm=1,
        test_evaluated=False, independent_validation=False, early_stopping=False, sweep=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
        limitation="Warmup supervision hypothesis only; no claim of live stall cure or authentic traffic-stop labels.")


def probe_indices(examples, proof):
    fixed = set(previous.selected_train_indices(examples))
    fixed.update(i for i, row in enumerate(examples) if 1.5 <= row.features[1] <= 2.5)
    fixed.update(i for i, phase in enumerate(proof["phase_ids"]) if phase in (0, 1))
    return sorted(fixed)


def probe(model, dataset, indices, device):
    model.eval()
    reports = []
    gate = RuntimeGateConfig(candidate_count=12, future_points=64)
    projected, _ = projection.projected_costs(loss_module)
    with torch.no_grad():
        for index in indices:
            batch = base._move_batch(default_collate([dataset[index]]), device)
            xy, speed, logits = model(*(batch[name] for name in previous.FIELDS))
            native = base.trajectory_loss(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                batch["target_valid"], base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
            costs = projected(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
                batch["target_valid"], base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
            base.require(torch.equal(costs.argmin(1), native["oracle_candidate_index"]), "probe oracle projection differs")
            selected, oracle = int(logits[0].argmax()), int(native["oracle_candidate_index"][0])
            current = float(batch["ego_history"][0, -1, 1])
            x, s, l = xy[0].tolist(), speed[0].tolist(), logits[0].tolist()
            sorted_logits = sorted(l, reverse=True)
            reports.append(dict(index=index, token=dataset.examples[index].token, current_speed_mps=current,
                selected=selected, oracle=oracle, logit_margin=sorted_logits[0]-sorted_logits[1],
                selected_speed_minus_current={str(t): s[selected][step]-current for t, step in ((.3, 2), (.5, 4), (1., 9))},
                candidate_xy=x, candidate_speed=s, candidate_logits=l,
                candidate_composite_costs=costs[0].tolist(), selected_composite_regret=float(costs[0, selected]-costs[0, oracle]),
                target_speed=batch["target_speed_mps"][0].tolist(), target_xy=batch["target_xy"][0].tolist(),
                target_valid=batch["target_valid"][0].tolist(),
                candidate_gates=base.audit_prediction(x, s, l, gate, current_speed_mps=current),
                selected_speed_mae_mps=float(native["selected_speed_mae_mps"])))
    return reports


def fit(arm, initial, cfg, dataset, off, batches, proof, indices, output, device, declaration):
    base._seed_everything(base.SEED, device)
    model = base.PerspectiveTrajectoryModel(cfg).to(device)
    model.load_state_dict(initial, strict=True)
    initial_hash = base.tensor_digest(initial)
    base.require(initial_hash == base.tensor_digest(model.state_dict()), "paired initialization differs")
    model.requires_grad_(True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-5, weight_decay=1e-4)
    folder = output / arm / "training"; folder.mkdir(parents=True)
    model.train()
    updates, forwarded, active = 0, 0, 0
    started = time.monotonic()
    with (folder / "history.jsonl").open("x") as log:
        for step, raw in enumerate(DataLoader(dataset, batch_sampler=batches, num_workers=0), 1):
            ids = batches[step-1]
            if step in supervision.SKIP_ONE_BASED:
                base.require(all(bool(off[i]) for i in ids), "shared skipped batch is not all warmup")
                # HH_261001 - No forward or optimizer call: zero gradients would not disable AdamW/momentum.
                log.write(json.dumps(dict(scheduled_step=step, optimizer_updates=updates,
                    skipped=True, indices=list(ids), no_forward_or_optimizer_call=True)) + "\n")
                log.flush()
                continue
            batch = base._move_batch(raw, device)
            optimizer.zero_grad(set_to_none=True)
            xy, speed, logits = model(*(batch[name] for name in previous.FIELDS))
            values = [0. if bool(off[i]) else supervision.ACTIVE_WEIGHT for i in ids] if arm == ARMS[1] else [1.]*4
            weights = xy.new_tensor(values)
            total, original, near, _ = supervision.weighted_original_loss(xy, speed, logits, batch, weights)
            base.require(bool(torch.isfinite(total)), "nonfinite warmup-pair objective")
            total.backward()
            norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
            base.require(bool(torch.isfinite(norm)), "nonfinite warmup-pair gradient")
            optimizer.step()
            base.require(base._nested_tensors_are_finite(model.state_dict())
                         and base._nested_tensors_are_finite(optimizer.state_dict()), "nonfinite training state")
            updates += 1; forwarded += len(ids); active += sum(value > 0 for value in values)
            if step % 100 == 0:
                row = dict(arm=arm, scheduled_step=step, optimizer_updates=updates, total_loss=float(total.detach()),
                    base_loss=float(original.detach()), near_loss=float(near.detach()), gradient_norm=float(norm),
                    elapsed_s=time.monotonic()-started)
                log.write(json.dumps(row) + "\n"); log.flush(); print(json.dumps(row), flush=True)
    base.require(step == 3000 and updates == 2999 and forwarded == 11996
                 and active == (11549 if arm == ARMS[1] else 11996), "fixed fit accounting differs")
    payload = dict(checkpoint_schema=SCHEMA, arm=arm, parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
        model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
        final_model_state_sha256=base.tensor_digest(model.state_dict()), initial_model_state_sha256=initial_hash,
        fit=dict(global_step=updates, scheduled_steps=3000, samples_seen=forwarded, scheduled_samples=12000,
            active_supervision_exposures=active, skipped_schedule_steps_one_based=[1693],
            optimizer_state_included=False, optimizer_resumed=False), training_plan_sha256=PLAN_SHA256,
        training_dataset_fingerprint_sha256=dataset.fingerprint_sha256, research_objective=declaration,
        exposure_order_sha256=proof["exposure_order_sha256"], warmup_mask_sha256=proof["warmup_mask_sha256"],
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    checkpoint = folder / "final_checkpoint.pt"; base.save_new(checkpoint, payload)
    receipt = dict(checkpoint=str(checkpoint.relative_to(output)), checkpoint_sha256=base.digest(checkpoint),
        fit=payload["fit"], elapsed_training_s=time.monotonic()-started,
        exposure_order_sha256=proof["exposure_order_sha256"], warmup_mask_sha256=proof["warmup_mask_sha256"],
        final_model_state_sha256=payload["final_model_state_sha256"])
    base.write_new(output / arm / "training_receipt.json", receipt)
    del model, optimizer
    torch.cuda.empty_cache()
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-plan", type=Path, required=True)
    parser.add_argument("--parent-checkpoint", type=Path, required=True)
    parser.add_argument("--frozen-ah-cache", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    root = Path.cwd().resolve()
    workspace = base.check_run_environment(dict(repo_root=str(root), output_root=str(args.output)))
    base.require(not args.output.exists(), "fresh warmup A/B output required")
    plan = supervision.pinned(args.baseline_plan, previous.BASE_PLAN_SHA256)
    base.validate_plan(plan)
    sources = previous.verify_sources(root, plan)
    supervision.pinned(root / supervision.AUDIT_RELATIVE, supervision.AUDIT_SHA)
    base.require(base.digest(root / PLAN_RELATIVE) == PLAN_SHA256, "frozen warmup training protocol differs")
    for path in (Path(__file__), Path(supervision.__file__), Path(supervision.phase.__file__), Path(previous.__file__), Path(projection.__file__),
                 root / PLAN_RELATIVE, root / supervision.AUDIT_RELATIVE):
        sources[str(path.relative_to(root))] = base.digest(path)
    base.require(base.digest(args.parent_checkpoint) == previous.AH_CHECKPOINT_SHA256, "exact original AH parent required")
    payload, digest = _read_checkpoint_file(args.parent_checkpoint, torch.device("cpu"))
    base.require(digest == previous.AH_CHECKPOINT_SHA256 and payload["checkpoint_schema"] == "portable_e2e.near_horizon_ah.v1",
                 "secure parent identity differs")
    initial = payload["model_state_dict"]
    base.require(base.tensor_digest(initial) == payload["final_model_state_sha256"], "parent state differs")
    cfg = base.ModelConfig.from_mapping(payload["model_config"])
    view = plan["views"][base.ARMS[1]]
    loaded, examples, scope = base._load_view(Path(view["path"]), "train", base.ARMS[1])
    base._verify_loaded_view(view, loaded, examples)
    base.require(scope == view["scope_precheck"] and len(examples) == 2787, "TRAIN scope differs")
    dataset = base.Common10TorchDataset(examples, cfg, verify_image_sha256=True, split="train")
    off, batches, proof = supervision.make_provenance(Path(view["path"]), examples, root, args.frozen_ah_cache)
    indices = probe_indices(examples, proof)
    declaration = dict(contract(), source_sha256=sources, training_plan_sha256=PLAN_SHA256,
        training_manifest_sha256=supervision.MANIFEST_SHA, training_count=len(examples),
        training_dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        warmup_mask_sha256=proof["warmup_mask_sha256"], exposure_order_sha256=proof["exposure_order_sha256"],
        provenance_receipt_sha256=supervision.digest_json(proof), frozen_ah_cache_sha256=supervision.CACHE_SHA,
        fixed_probe_indices=indices, probe_tokens=[examples[i].token for i in indices],
        tail_anchor_count=proof["tail_anchor_count"], driving_future_crosses_tail_count=proof["driving_future_crosses_tail_count"],
        dataset_validation_may_read_nontrain_contract_records=True)
    if args.validate_only:
        print(json.dumps(dict(status="CPU_PREFLIGHT_PASS_NO_TRAINING", **declaration), indent=2)); return
    with base.gpu0_lease(workspace):
        gpu = base.preflight_gpu(); device = torch.device("cuda:0")
        args.output.mkdir(parents=True)
        base.write_new(args.output / "declaration.json", declaration)
        base.write_new(args.output / "provenance_receipt.json", proof)
        base.write_new(args.output / "started.json", dict(status="FIXED_WARMUP_FULL_MODEL_PAIR", gpu=gpu, started_at=base.utc_now()))
        for name in sources:
            base.copy_new(root/name, args.output/"source"/name)
        model = base.PerspectiveTrajectoryModel(cfg).to(device); model.load_state_dict(initial, strict=True)
        base.write_new(args.output/"parent_train_probe.json", probe(model, dataset, indices, device))
        del model
        training = {arm: fit(arm, initial, cfg, dataset, off, batches, proof, indices, args.output, device, declaration) for arm in ARMS}
        base.require(training[ARMS[0]]["exposure_order_sha256"] == training[ARMS[1]]["exposure_order_sha256"], "paired order differs")
        base.write_new(args.output/"both_final_checkpoints_complete.json", training)
        for arm in ARMS:
            final, digest = _read_checkpoint_file(args.output/training[arm]["checkpoint"], torch.device("cpu"))
            base.require(digest == training[arm]["checkpoint_sha256"], "final checkpoint differs before TRAIN probe")
            model = base.PerspectiveTrajectoryModel(cfg).to(device); model.load_state_dict(final["model_state_dict"], strict=True)
            base.write_new(args.output/arm/"train_probe_after.json", probe(model, dataset, indices, device))
            del model
        # HH_261001 - Only after both final checkpoints exist may known-DEV images/metrics be evaluated.
        dev_view = plan["views"]["Town03_DEV"]
        dev, dev_examples, dev_scope = base._load_view(Path(dev_view["path"]), "val", "Town03_DEV")
        base._verify_loaded_view(dev_view, dev, dev_examples)
        base.require(dev_scope == dev_view["scope_precheck"], "known DEV scope differs")
        old_schema, old_steps = previous.SCHEMA, previous.STEPS
        try:
            previous.SCHEMA, previous.STEPS = SCHEMA, UPDATES
            evaluations = {arm: previous.evaluate_final(arm, training[arm], dev, cfg, args.output, device) for arm in ARMS}
        finally:
            previous.SCHEMA, previous.STEPS = old_schema, old_steps
        source_match = all(base.digest(root/name) == expected for name, expected in sources.items())
        base.require(source_match, "source changed during warmup pair")
        result = dict(schema=SCHEMA, status="COMPLETE_NOT_PROMOTED", training=training, evaluations=evaluations,
            W1_vs_W0=base.paired_nonregression(evaluations[ARMS[0]], evaluations[ARMS[1]]),
            strict_gates={arm: base.strict_gate(evaluations[arm]) for arm in ARMS}, source_hashes_match=source_match,
            training_data_approved=False, model_promoted=False, vehicle_control_approved=False, completed_at=base.utc_now())
        base.write_new(args.output/"summary.json", result)
        print(json.dumps(dict(status=result["status"], training=training, strict_gates=result["strict_gates"])), flush=True)


if __name__ == "__main__":
    main()
