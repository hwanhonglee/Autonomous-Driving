"""HH_261001 - Four unchanged W1 schedules, one continuous optimizer, mandatory exact first-block identity."""

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import platform
import time

import torch
from torch.utils.data import DataLoader

from portable_e2e.train import _read_checkpoint_file
from scripts.e2e import run_ah_warmup_supervision_ab_20261001 as w1


base, previous, supervision = w1.base, w1.previous, w1.supervision
SCHEMA = "portable_e2e.warmup_duration_w2_20261001.v1"
ARM = "W2_warmup_anchor_off_4blocks"
BLOCKS, BLOCK_SCHEDULED, BLOCK_UPDATES = 4, 3000, 2999
FIT_BUDGET_SECONDS = 3600.
EXPECTED_W1_STATE = "ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f"
EXPECTED_W1_CHECKPOINT = "b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76"
W1_DECLARATION_SHA = "8de2bb58d6b7deadab5523c68fc12f45d854cf9a217c18e99606d9045d334f31"
W1_RECEIPT_SHA = "7115a4b49e6d9842c6a05d86b77286ec7e89c304222b548072f76733598e455a"
ORDER_SHA = "f0d3d522b3ec4787b84286f82e61017a6aee4619270a99451d3cea08e35110d9"
MASK_SHA = "2c46c730096452291e10eed040c228e31ca2dc6f2ef424a02820e8d04efc3b1e"
DATASET_FINGERPRINT = "4dc6f23b62cc3b65154fb3414b1b509f3ef4696f6aa8bda9f8ee4535143cceda"
PLAN_RELATIVE = "docs/assets/validation/2026-10-01/selector_phase_ab/W2_LONGER_TRAINING_PREREGISTRATION.md"
PLAN_SHA256 = "70229908028ac30cb9fd51a64e66d144485099280ebfd7128422e937dc5010b3"
NEW_SOURCES = ("tools/e2e/run_ah_warmup_duration_w2_20261001.py",
               "learning/tests/test_warmup_duration_w2_20261001.py", PLAN_RELATIVE)


def contract():
    return dict(schema=SCHEMA, arm=ARM, base_protocol=w1.contract(), duration_only_intervention=True,
        blocks=4, scheduled_batches_per_block=3000, optimizer_updates_per_block=2999,
        scheduled_batches=12000, optimizer_updates=11996, scheduled_samples=48000,
        forward_samples=47984, active_supervision_exposures=46196,
        skipped_schedule_steps_one_based=[1693, 4693, 7693, 10693],
        block_schedule_sha256=ORDER_SHA, warmup_mask_sha256=MASK_SHA,
        parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256, expected_first_block_state_sha256=EXPECTED_W1_STATE,
        optimizer_initializations=1, optimizer_resumed=False, optimizer_reset_between_blocks=False,
        rng_seed_calls_before_fit=1, rng_reset_between_blocks=False,
        first_block_checkpoint_purpose="identity only; mandatory exact equality, never model selection",
        fixed_fit_wall_budget_seconds=FIT_BUDGET_SECONDS,
        final_dev_passes=1, final_dev_only_after_fixed_end=True, dev_during_training=False,
        intermediate_train_probes=False, early_stopping=False, hyperparameter_sweep=False, automatic_retry=False,
        architecture_changed=False, decoder_changed=False, loss_changed=False, training_data_changed=False,
        labels_clipped=False, warmup_mask_changed=False, test_evaluated=False, independent_validation=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)


def validate_schedule(off, batches, proof):
    base.require(len(batches) == 3000 and supervision.digest_json(batches) == ORDER_SHA,
                 "exact W1 3000-batch schedule differs")
    base.require(supervision.digest_json(off.tolist()) == MASK_SHA
                 and len(off) == 2787 and int(off.sum()) == 105, "exact W1 warmup mask differs")
    base.require(proof["exposure_order_sha256"] == ORDER_SHA and proof["warmup_mask_sha256"] == MASK_SHA,
                 "provenance schedule/mask hash differs")
    skipped = [step for step, ids in enumerate(batches, 1) if all(bool(off[i]) for i in ids)]
    base.require(skipped == [1693] and tuple(batches[1692]) == (617, 619, 322, 317), "shared all-warmup skip differs")
    active = sum(not bool(off[i]) for step, ids in enumerate(batches, 1) if step != 1693 for i in ids)
    base.require(active == 11549 and supervision.ACTIVE_WEIGHT == 11996 / 11549, "active W1 weight/exposure differs")


def check_fit_budget(started):
    base.require(time.monotonic() - started <= FIT_BUDGET_SECONDS, "fixed 3600-second fit budget exhausted")


def cpu_tree(value):
    """HH_261001 - Preserve ordinary optimizer tensors for audit without modifying the live optimizer or resuming them."""
    if torch.is_tensor(value):
        return value.detach().cpu().clone()
    if isinstance(value, dict):
        return {key: cpu_tree(item) for key, item in value.items()}
    if isinstance(value, list):
        return [cpu_tree(item) for item in value]
    if isinstance(value, tuple):
        return tuple(cpu_tree(item) for item in value)
    return value


def optimize_batch(model, optimizer, raw, ids, off, device):
    """HH_261001 - Match the W1 update equations and call order exactly; only the outer duration changes."""
    batch = base._move_batch(raw, device)
    optimizer.zero_grad(set_to_none=True)
    xy, speed, logits = model(*(batch[name] for name in previous.FIELDS))
    values = [0. if bool(off[i]) else supervision.ACTIVE_WEIGHT for i in ids]
    weights = xy.new_tensor(values)
    total, original, near, _ = supervision.weighted_original_loss(xy, speed, logits, batch, weights)
    base.require(bool(torch.isfinite(total)), "nonfinite warmup-pair objective")
    total.backward()
    norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
    base.require(bool(torch.isfinite(norm)), "nonfinite warmup-pair gradient")
    optimizer.step()
    base.require(base._nested_tensors_are_finite(model.state_dict())
                 and base._nested_tensors_are_finite(optimizer.state_dict()), "nonfinite training state")
    return total, original, near, norm, sum(value > 0 for value in values)


def first_block_barrier(model, cfg, optimizer, optimizer_identity, counts, folder, initial_hash, declaration):
    """HH_261001 - Save a failed reproduction too, then refuse continuation; never load the old W1 checkpoint."""
    actual = base.tensor_digest(model.state_dict())
    checkpoint = folder / "first_block_identity_checkpoint.pt"
    base.save_new(checkpoint, dict(checkpoint_schema=SCHEMA, purpose="FIRST_BLOCK_IDENTITY_ONLY_NOT_SELECTION",
        arm=ARM, model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
        initial_model_state_sha256=initial_hash, final_model_state_sha256=actual,
        parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256, expected_W1_state_sha256=EXPECTED_W1_STATE,
        fit={**counts, "optimizer_state_included": False, "optimizer_resumed": False},
        training_plan_sha256=PLAN_SHA256, research_objective=declaration,
        training_data_approved=False, model_promoted=False, vehicle_control_approved=False))
    matched = actual == EXPECTED_W1_STATE
    receipt = dict(status="PASS_EXACT_W1_STATE" if matched else "FAIL_FIRST_BLOCK_IDENTITY",
        expected_model_state_sha256=EXPECTED_W1_STATE, actual_model_state_sha256=actual,
        checkpoint=str(checkpoint.name), checkpoint_sha256=base.digest(checkpoint),
        optimizer_same_object=id(optimizer) == optimizer_identity,
        optimizer_reloaded=False, checkpoint_used_for_model_selection=False, fit=counts)
    base.write_new(folder.parent / "first_block_identity.json", receipt)
    base.require(counts == dict(scheduled_steps=3000, global_step=2999, samples_seen=11996,
                              active_supervision_exposures=11549), "first-block accounting differs")
    base.require(receipt["optimizer_same_object"], "optimizer object changed before first-block barrier")
    base.require(matched, "first-block state differs from exact W1; preserved failure, no continuation or DEV")
    return receipt


def fit(initial, cfg, dataset, off, batches, proof, output, device, declaration):
    base._seed_everything(base.SEED, device)
    model = base.PerspectiveTrajectoryModel(cfg).to(device)
    model.load_state_dict(initial, strict=True)
    initial_hash = base.tensor_digest(initial)
    base.require(initial_hash == base.tensor_digest(model.state_dict()), "paired initialization differs")
    model.requires_grad_(True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-5, weight_decay=1e-4)
    optimizer_identity = id(optimizer)
    folder = output / ARM / "training"
    folder.mkdir(parents=True)
    model.train()
    updates = forwarded = active = scheduled = 0
    first_identity, blocks = None, []
    started = time.monotonic()
    try:
        with (folder / "history.jsonl").open("x") as log:
            for block in range(BLOCKS):
                block_start = time.monotonic()
                for step, raw in enumerate(DataLoader(dataset, batch_sampler=batches, num_workers=0), 1):
                    scheduled = block * BLOCK_SCHEDULED + step
                    check_fit_budget(started)
                    ids = batches[step - 1]
                    if step in supervision.SKIP_ONE_BASED:
                        base.require(all(bool(off[i]) for i in ids), "shared skipped batch is not all warmup")
                        log.write(json.dumps(dict(block=block+1, scheduled_step=scheduled, block_scheduled_step=step,
                            optimizer_updates=updates, skipped=True, indices=list(ids), no_forward_or_optimizer_call=True)) + "\n")
                        log.flush()
                        continue
                    total, original, near, norm, active_batch = optimize_batch(model, optimizer, raw, ids, off, device)
                    updates += 1
                    forwarded += len(ids)
                    active += active_batch
                    check_fit_budget(started)
                    if step % 100 == 0:
                        row = dict(arm=ARM, block=block+1, scheduled_step=scheduled, block_scheduled_step=step,
                            optimizer_updates=updates, total_loss=float(total.detach()), base_loss=float(original.detach()),
                            near_loss=float(near.detach()), gradient_norm=float(norm), elapsed_s=time.monotonic()-started)
                        log.write(json.dumps(row) + "\n")
                        log.flush()
                        print(json.dumps(row), flush=True)
                base.require(step == 3000 and updates == (block+1)*2999 and forwarded == (block+1)*11996
                             and active == (block+1)*11549, "fixed repeated W1 fit accounting differs")
                base.require(id(optimizer) == optimizer_identity, "optimizer object changed between blocks")
                counts = dict(scheduled_steps=scheduled, global_step=updates, samples_seen=forwarded,
                              active_supervision_exposures=active)
                if block == 0:
                    first_identity = first_block_barrier(model, cfg, optimizer, optimizer_identity,
                                                        counts, folder, initial_hash, declaration)
                blocks.append(dict(block=block+1, fit=counts, elapsed_block_s=time.monotonic()-block_start,
                    block_exposure_order_sha256=proof["exposure_order_sha256"], optimizer_same_object=True))
                base.write_new(folder / f"block_{block+1:02d}_accounting.json", blocks[-1])
                check_fit_budget(started)
        base.require(scheduled == 12000 and updates == 11996 and forwarded == 47984 and active == 46196,
                     "fixed four-block endpoint not reached")
        payload = dict(checkpoint_schema=SCHEMA, arm=ARM, parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
            model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
            optimizer_state_dict=cpu_tree(optimizer.state_dict()),
            final_model_state_sha256=base.tensor_digest(model.state_dict()), initial_model_state_sha256=initial_hash,
            fit=dict(global_step=updates, scheduled_steps=scheduled, samples_seen=forwarded, scheduled_samples=48000,
                active_supervision_exposures=active, skipped_schedule_steps_one_based=[1693, 4693, 7693, 10693],
                optimizer_state_included=True, optimizer_resumed=False, optimizer_reset_between_blocks=False),
            training_plan_sha256=PLAN_SHA256, training_dataset_fingerprint_sha256=dataset.fingerprint_sha256,
            research_objective=declaration, exposure_order_sha256=proof["exposure_order_sha256"],
            repeated_exposure_order_sha256=supervision.digest_json(batches * 4),
            warmup_mask_sha256=proof["warmup_mask_sha256"], first_block_identity=first_identity,
            training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
        checkpoint = folder / "final_checkpoint.pt"
        base.save_new(checkpoint, payload)
        receipt = dict(checkpoint=str(checkpoint.relative_to(output)), checkpoint_sha256=base.digest(checkpoint),
            fit=payload["fit"], elapsed_training_s=time.monotonic()-started, block_receipts=blocks,
            exposure_order_sha256=proof["exposure_order_sha256"], repeated_exposure_order_sha256=payload["repeated_exposure_order_sha256"],
            warmup_mask_sha256=proof["warmup_mask_sha256"], first_block_identity=first_identity,
            final_model_state_sha256=payload["final_model_state_sha256"])
        base.write_new(output / ARM / "training_receipt.json", receipt)
        return receipt
    except BaseException as error:
        failure = dict(status="FAIL_NO_CONTINUATION_OR_DEV", error=f"{type(error).__name__}: {error}",
            scheduled_step_observed=scheduled, optimizer_updates_completed=updates, forwarded_samples=forwarded,
            active_supervision_exposures=active, elapsed_training_s=time.monotonic()-started,
            first_block_identity=first_identity, model_promoted=False, training_data_approved=False)
        try:
            if first_identity is not None or not (folder / "first_block_identity_checkpoint.pt").exists():
                partial = folder / "failed_partial_checkpoint.pt"
                base.save_new(partial, dict(checkpoint_schema=SCHEMA, failure=failure,
                    model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
                    optimizer_state_dict=cpu_tree(optimizer.state_dict()), identity_only_not_candidate=True))
                failure["partial_checkpoint_sha256"] = base.digest(partial)
        except Exception as evidence_error:
            failure["partial_checkpoint_error"] = f"{type(evidence_error).__name__}: {evidence_error}"
        base.write_new(output / ARM / "failure.json", failure)
        raise
    finally:
        del model, optimizer
        if device.type == "cuda":
            torch.cuda.empty_cache()


@contextmanager
def final_evaluation_scope():
    old = previous.SCHEMA, previous.STEPS
    try:
        previous.SCHEMA, previous.STEPS = SCHEMA, 11996
        yield
    finally:
        previous.SCHEMA, previous.STEPS = old


def validate_reference(root, directory):
    declaration_path, receipt_path = directory / "declaration.json", directory / w1.ARMS[1] / "training_receipt.json"
    declaration = supervision.pinned(declaration_path, W1_DECLARATION_SHA)
    receipt = supervision.pinned(receipt_path, W1_RECEIPT_SHA)
    base.require(receipt["final_model_state_sha256"] == EXPECTED_W1_STATE
                 and receipt["checkpoint_sha256"] == EXPECTED_W1_CHECKPOINT,
                 "published W1 endpoint identity differs")
    base.require(receipt["fit"] == dict(active_supervision_exposures=11549, global_step=2999,
        optimizer_resumed=False, optimizer_state_included=False, samples_seen=11996,
        scheduled_samples=12000, scheduled_steps=3000, skipped_schedule_steps_one_based=[1693]),
        "published W1 fit accounting differs")
    for name, expected in declaration["source_sha256"].items():
        base.require(base.digest(root / name) == expected, "original W1 source changed: " + name)
    return declaration, receipt, {"W1_declaration.json": declaration_path, "W1_training_receipt.json": receipt_path}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("baseline-plan", "parent-checkpoint", "frozen-ah-cache", "w1-reference-output", "output"):
        parser.add_argument("--" + name, type=Path)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--contract-only", action="store_true")
    args = parser.parse_args(argv)
    if args.contract_only:
        print(json.dumps(dict(status="STATIC_CONTRACT_NO_DATA_OR_GPU", **contract()), sort_keys=True))
        return 0
    base.require(all(value is not None for value in (args.baseline_plan, args.parent_checkpoint,
        args.frozen_ah_cache, args.w1_reference_output, args.output)), "all five explicit input/output paths required")
    root = Path.cwd().resolve()
    workspace = base.check_run_environment(dict(repo_root=str(root), output_root=str(args.output)))
    base.require(not args.output.exists() and not args.output.is_symlink(), "fresh W2 output required")
    reference, _, reference_files = validate_reference(root, args.w1_reference_output)
    plan = supervision.pinned(args.baseline_plan, previous.BASE_PLAN_SHA256)
    base.validate_plan(plan)
    sources = previous.verify_sources(root, plan)
    sources.update(reference["source_sha256"])
    base.require(base.digest(root / PLAN_RELATIVE) == PLAN_SHA256, "frozen W2 training protocol differs")
    for name in NEW_SOURCES:
        sources[name] = base.digest(root / name)
    base.require(base.digest(args.parent_checkpoint) == previous.AH_CHECKPOINT_SHA256, "exact original AH parent required")
    payload, digest = _read_checkpoint_file(args.parent_checkpoint, torch.device("cpu"))
    base.require(digest == previous.AH_CHECKPOINT_SHA256 and payload["checkpoint_schema"] == "portable_e2e.near_horizon_ah.v1",
                 "secure parent identity differs")
    initial = payload["model_state_dict"]
    base.require(base.tensor_digest(initial) == payload["final_model_state_sha256"], "parent model state differs")
    cfg = base.ModelConfig.from_mapping(payload["model_config"])
    view = plan["views"][base.ARMS[1]]
    loaded, examples, scope = base._load_view(Path(view["path"]), "train", base.ARMS[1])
    base._verify_loaded_view(view, loaded, examples)
    base.require(scope == view["scope_precheck"] and len(examples) == 2787, "TRAIN scope differs")
    dataset = base.Common10TorchDataset(examples, cfg, verify_image_sha256=True, split="train")
    base.require(dataset.fingerprint_sha256 == DATASET_FINGERPRINT, "original W1 dataset fingerprint differs")
    off, batches, proof = supervision.make_provenance(Path(view["path"]), examples, root, args.frozen_ah_cache)
    validate_schedule(off, batches, proof)
    indices = w1.probe_indices(examples, proof)
    base.require(indices == reference["fixed_probe_indices"], "fixed W1 TRAIN probes differ")
    declaration = dict(contract(), source_sha256=sources, training_plan_sha256=PLAN_SHA256,
        baseline_plan_sha256=previous.BASE_PLAN_SHA256, reference_W1_declaration_sha256=W1_DECLARATION_SHA,
        reference_W1_receipt_sha256=W1_RECEIPT_SHA, training_count=2787,
        training_dataset_fingerprint_sha256=dataset.fingerprint_sha256, provenance_receipt_sha256=supervision.digest_json(proof),
        repeated_exposure_order_sha256=supervision.digest_json(batches * 4),
        fixed_probe_indices=indices, probe_tokens=[examples[i].token for i in indices],
        tail_anchor_count=proof["tail_anchor_count"], driving_future_crosses_tail_count=proof["driving_future_crosses_tail_count"],
        dataset_validation_may_read_nontrain_contract_records=True)
    if args.validate_only:
        print(json.dumps(dict(status="CPU_PREFLIGHT_PASS_NO_TRAINING", **declaration), sort_keys=True))
        return 0
    with base.gpu0_lease(workspace):
        gpu = base.preflight_gpu()
        device = torch.device("cuda:0")
        args.output.mkdir(parents=True)
        base.write_new(args.output / "declaration.json", declaration)
        base.write_new(args.output / "provenance_receipt.json", proof)
        base.write_new(args.output / "started.json", dict(status="FIXED_W2_FOUR_BLOCK_RESEARCH_FIT", gpu=gpu,
            started_at=base.utc_now(), python=platform.python_version(), torch=torch.__version__,
            cuda=torch.version.cuda, cudnn=torch.backends.cudnn.version()))
        for name in sources:
            base.copy_new(root / name, args.output / "source" / name)
        for name, path in reference_files.items():
            base.copy_new(path, args.output / "inputs" / name)
        base.copy_new(args.baseline_plan, args.output / "inputs/baseline_plan.json")
        receipt = fit(initial, cfg, dataset, off, batches, proof, args.output, device, declaration)
        base.write_new(args.output / "fixed_final_checkpoint_complete.json", receipt)
        final, checkpoint_sha = _read_checkpoint_file(args.output / receipt["checkpoint"], torch.device("cpu"))
        base.require(checkpoint_sha == receipt["checkpoint_sha256"], "final checkpoint differs before fixed TRAIN probe")
        model = base.PerspectiveTrajectoryModel(cfg).to(device)
        model.load_state_dict(final["model_state_dict"], strict=True)
        base.write_new(args.output / ARM / "train_probe_after.json", w1.probe(model, dataset, indices, device))
        del model
        torch.cuda.empty_cache()
        # HH_261001 - No DEV loading or evaluation until the fixed 11996-update endpoint is irreversibly saved.
        dev_view = plan["views"]["Town03_DEV"]
        dev, dev_examples, dev_scope = base._load_view(Path(dev_view["path"]), "val", "Town03_DEV")
        base._verify_loaded_view(dev_view, dev, dev_examples)
        base.require(dev_scope == dev_view["scope_precheck"], "known DEV scope differs")
        with final_evaluation_scope():
            evaluation = previous.evaluate_final(ARM, receipt, dev, cfg, args.output, device)
        matched = all(base.digest(root / name) == expected
                      and base.digest(args.output / "source" / name) == expected for name, expected in sources.items())
        base.require(matched, "source changed during W2 fit/evaluation")
        summary = dict(schema=SCHEMA, status="COMPLETE_NOT_PROMOTED", training=receipt, evaluation=evaluation,
            strict_gates=base.strict_gate(evaluation), source_hashes_match=matched,
            training_data_approved=False, model_promoted=False, vehicle_control_approved=False, completed_at=base.utc_now())
        base.write_new(args.output / "summary.json", summary)
        print(json.dumps(dict(status=summary["status"], training=receipt, strict_gates=summary["strict_gates"])), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
