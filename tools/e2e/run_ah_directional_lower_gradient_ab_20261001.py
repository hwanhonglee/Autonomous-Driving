"""HH_261001 - Matched fixed W1 reproduction versus an inward-only lower-speed backward surrogate."""

import argparse
from contextlib import contextmanager, nullcontext
import json
from pathlib import Path
import platform
import time

import torch
from torch.utils.data import DataLoader
from portable_e2e.train import _read_checkpoint_file
from portable_e2e import directional_speed_gradient_research as directional
from scripts.e2e import run_ah_warmup_duration_w2_20261001 as reference

base, w1, previous, supervision = reference.base, reference.w1, reference.previous, reference.supervision
optimize_batch = reference.optimize_batch
SCHEMA = 'portable_e2e.directional_lower_gradient_ab_20261001.v1'
ARMS = ('A_native_W1_reproduction', 'B_directional_lower_speed_gradient')
TRACE_STEPS = (1, 1000, 2000, 3000)
FIT_BUDGET_SECONDS = 1800.
PLAN_RELATIVE = 'docs/assets/validation/2026-10-01/selector_phase_ab/DIRECTIONAL_LOWER_GRADIENT_AB_PREREGISTRATION.md'
PLAN_SHA256 = '9fffd0ef2cef46a667b51affeb4e2f08eccaa3100e34fd19dd9ea7c4eff23908'
AUDIT_RELATIVE = 'docs/assets/validation/2026-10-01/warmup_supervision_ab/directional_gradient_forward_v1/result.json'
FIXED_SOURCES = {
    'learning/portable_e2e/directional_speed_gradient_research.py': '950989d6fae4d1895dd0ac1843fc18c0c5c507a71675b20d4e5cdd3ccfa10747',
    'learning/tests/test_directional_speed_gradient_research.py': '9f17fbfb740d8ff22b4d6a3a0568f55cc2d54b685f0c27f1171e18c6334398ea',
    'tools/e2e/audit_directional_speed_forward_20261001.py': '18f07abc5ed619042b94eed585b97cd56b7e629ba1bf2f37fca99eb4d65f7c96',
    AUDIT_RELATIVE: 'b8444dadd8c3797caab01f2dbfb1dc55be9b3654225993a060f39075d2bba646',
    PLAN_RELATIVE: PLAN_SHA256,
    'tools/e2e/run_ah_warmup_duration_w2_20261001.py': '8f79b6bd374afc8d2205e4c2b6cd16e20322f857fff26a0ccc916fd327ed29b7',
    'learning/tests/test_warmup_duration_w2_20261001.py': '4f2c6886d2992911625b04349dae66a5d2f36c06e276dca69cfcf8415f64b10f',
    reference.PLAN_RELATIVE: reference.PLAN_SHA256,
}
NEW_SOURCES = ('tools/e2e/run_ah_directional_lower_gradient_ab_20261001.py',
               'learning/tests/test_directional_lower_gradient_ab_20261001.py', PLAN_RELATIVE)
COUNTS = dict(global_step=2999, scheduled_steps=3000, samples_seen=11996,
    scheduled_samples=12000, active_supervision_exposures=11549, skipped_schedule_steps_one_based=[1693],
    optimizer_state_included=True, optimizer_resumed=False)


def contract():
    return dict(schema=SCHEMA, arms=list(ARMS), parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
        base_protocol=w1.contract(), fixed_fit=COUNTS, expected_A_model_state_sha256=reference.EXPECTED_W1_STATE,
        intervention='B recurrent DRIVE speed clamp backward passes only inward negative gradient below zero',
        forward_decoder_changed=False, runtime_decoder_changed=False, parameter_abi_changed=False,
        current_speed_clamp_changed=False, upper_clamp_gradient_changed=False, STOP_branch_changed=False,
        loss_changed=False, new_near_preview_loss=False, acceleration_auxiliary=False,
        labels_changed=False, training_mask_changed=False, original_stop_hold_supervision_preserved=True,
        native_control_first=True, preserve_failed_A_before_identity_check=True,
        fresh_optimizer_each_arm=True, same_initial_AH_each_arm=True, rng_seed_calls_per_arm=1,
        trace_scheduled_batches_B=list(TRACE_STEPS), trace_other_batches=False, trace_A=False,
        fixed_fit_wall_budget_seconds_per_arm=FIT_BUDGET_SECONDS,
        both_fixed_checkpoints_before_DEV=True, final_dev_passes_per_arm=1,
        early_stopping=False, automatic_retry=False, hyperparameter_sweep=False,
        test_evaluated=False, independent_validation=False, simulator_launched=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
        limitation='Nonlinear surrogate, not a derivative or guaranteed descent; shared/winner/data/control failures remain.')


def model_class(arm):
    base.require(arm in ARMS, 'undeclared directional-gradient arm')
    return base.PerspectiveTrajectoryModel if arm == ARMS[0] else directional.DirectionalLowerSpeedGradientModel


def trace_for(arm, step):
    base.require(arm in ARMS and 1 <= step <= 3000, 'invalid fixed trace coordinate')
    return directional.GradientTrace() if arm == ARMS[1] and step in TRACE_STEPS else None


def check_budget(started):
    base.require(time.monotonic()-started <= FIT_BUDGET_SECONDS, 'fixed 1800-second arm fit budget exhausted')


def native_identity_barrier(arm, receipt, output):
    if arm != ARMS[0]:
        return None
    actual = receipt['final_model_state_sha256']
    record = dict(status='PASS_EXACT_W1_STATE' if actual == reference.EXPECTED_W1_STATE else 'FAIL_NATIVE_W1_REPRODUCTION',
        actual_model_state_sha256=actual, expected_model_state_sha256=reference.EXPECTED_W1_STATE,
        checkpoint_sha256=receipt['checkpoint_sha256'], actual_fit=receipt['fit'],
        checkpoint_used_for_model_selection=False, failed_checkpoint_preserved=True)
    base.write_new(output / arm / 'native_identity.json', record)
    base.require(receipt['fit'] == COUNTS, 'native fixed accounting differs')
    base.require(actual == reference.EXPECTED_W1_STATE,
                 'native control differs from exact W1; actual final preserved, no B or DEV')
    return record


def fit(arm, initial, cfg, dataset, off, batches, proof, output, device, declaration):
    # HH_261001 - Preserve the exact W1 seeded construction/optimizer/update order; no extra probe after this seed.
    base._seed_everything(base.SEED, device)
    model = model_class(arm)(cfg).to(device)
    model.load_state_dict(initial, strict=True)
    initial_hash = base.tensor_digest(initial)
    base.require(initial_hash == base.tensor_digest(model.state_dict()), 'paired initialization differs')
    model.requires_grad_(True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-5, weight_decay=1e-4)
    folder = output / arm / 'training'
    folder.mkdir(parents=True)
    model.train()
    updates = forwarded = active = step = 0
    started = time.monotonic()
    traces = []
    try:
        with (folder / 'history.jsonl').open('x') as log:
            for step, raw in enumerate(DataLoader(dataset, batch_sampler=batches, num_workers=0), 1):
                check_budget(started)
                ids = batches[step-1]
                if step in supervision.SKIP_ONE_BASED:
                    base.require(all(bool(off[i]) for i in ids), 'shared skipped batch is not all warmup')
                    log.write(json.dumps(dict(scheduled_step=step, optimizer_updates=updates, skipped=True,
                        indices=list(ids), no_forward_or_optimizer_call=True))+'\n')
                    log.flush()
                    continue
                trace = trace_for(arm, step)
                with directional.trace_gradients(trace) if trace is not None else nullcontext():
                    total, original, near, norm, active_batch = optimize_batch(model, optimizer, raw, ids, off, device)
                updates += 1
                forwarded += len(ids)
                active += active_batch
                if trace is not None:
                    record = dict(scheduled_step=step, optimizer_updates=updates, indices=list(ids), **trace.receipt())
                    traces.append(record)
                    base.write_new(folder / f'gradient_trace_{step:04d}.json', record)
                check_budget(started)
                if step % 100 == 0:
                    row = dict(arm=arm, scheduled_step=step, optimizer_updates=updates, total_loss=float(total.detach()),
                        base_loss=float(original.detach()), near_loss=float(near.detach()), gradient_norm=float(norm),
                        elapsed_s=time.monotonic()-started)
                    log.write(json.dumps(row)+'\n')
                    log.flush()
                    print(json.dumps(row), flush=True)
        base.require(step == 3000 and updates == 2999 and forwarded == 11996 and active == 11549,
                     'fixed matched fit accounting differs')
        base.require([r['scheduled_step'] for r in traces] == (list(TRACE_STEPS) if arm == ARMS[1] else []),
                     'prospective detached gradient tracing schedule differs')
        payload = dict(checkpoint_schema=SCHEMA, arm=arm, parent_checkpoint_sha256=previous.AH_CHECKPOINT_SHA256,
            model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
            optimizer_state_dict=reference.cpu_tree(optimizer.state_dict()),
            final_model_state_sha256=base.tensor_digest(model.state_dict()), initial_model_state_sha256=initial_hash,
            fit=COUNTS, training_plan_sha256=PLAN_SHA256, training_dataset_fingerprint_sha256=dataset.fingerprint_sha256,
            research_objective=declaration, exposure_order_sha256=proof['exposure_order_sha256'],
            warmup_mask_sha256=proof['warmup_mask_sha256'], inference_uses_original_model=True,
            training_only_gradient_recipe='native' if arm == ARMS[0] else directional.RECIPE,
            training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
        checkpoint = folder / 'final_checkpoint.pt'
        base.save_new(checkpoint, payload)
        receipt = dict(checkpoint=str(checkpoint.relative_to(output)), checkpoint_sha256=base.digest(checkpoint),
            fit=payload['fit'], elapsed_training_s=time.monotonic()-started,
            exposure_order_sha256=proof['exposure_order_sha256'], warmup_mask_sha256=proof['warmup_mask_sha256'],
            final_model_state_sha256=payload['final_model_state_sha256'], gradient_trace=traces)
        base.write_new(output / arm / 'training_receipt.json', receipt)
        check_budget(started)
        native_identity_barrier(arm, receipt, output)
        return receipt
    except BaseException as error:
        failure = dict(status='FAIL_NO_CONTINUATION_OR_DEV', error=f'{type(error).__name__}: {error}',
            arm=arm, scheduled_step_observed=step, optimizer_updates_completed=updates,
            forwarded_samples=forwarded, active_supervision_exposures=active, elapsed_s=time.monotonic()-started,
            training_data_approved=False, model_promoted=False)
        try:
            if not (folder / 'final_checkpoint.pt').exists():
                partial = folder / 'failed_partial_checkpoint.pt'
                base.save_new(partial, dict(checkpoint_schema=SCHEMA, arm=arm, failure=failure,
                    model_config=cfg.to_dict(), model_state_dict=base._clone_state(model.state_dict()),
                    optimizer_state_dict=reference.cpu_tree(optimizer.state_dict()), identity_only_not_candidate=True))
                failure['partial_checkpoint_sha256'] = base.digest(partial)
        except Exception as evidence_error:
            failure['partial_checkpoint_error'] = f'{type(evidence_error).__name__}: {evidence_error}'
        base.write_new(output / arm / 'failure.json', failure)
        raise
    finally:
        del model, optimizer
        if device.type == 'cuda':
            torch.cuda.empty_cache()


@contextmanager
def evaluation_scope():
    saved = previous.SCHEMA, previous.STEPS
    try:
        previous.SCHEMA, previous.STEPS = SCHEMA, 2999
        yield
    finally:
        previous.SCHEMA, previous.STEPS = saved


def verify_new_sources(root):
    for name, expected in FIXED_SOURCES.items():
        base.require(base.digest(root/name) == expected, 'directional experiment source differs: '+name)
    audit = json.loads((root/AUDIT_RELATIVE).read_text())
    base.require(audit['status'] == 'PASS' and audit['frames'] == audit['exact_original_replays']
        == audit['exact_candidate_forward_replays'] == 149 and audit['parameters_updated'] == 0
        and audit['drive_xy_speed_logits_and_stop_unchanged'] is True, 'prior exact-forward prerequisite differs')
    result = dict(FIXED_SOURCES)
    for name in NEW_SOURCES:
        result[name] = base.digest(root/name)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('baseline-plan', 'parent-checkpoint', 'frozen-ah-cache', 'w1-reference-output', 'output'):
        parser.add_argument('--'+name, type=Path)
    parser.add_argument('--validate-only', action='store_true')
    parser.add_argument('--contract-only', action='store_true')
    args = parser.parse_args(argv)
    root = Path.cwd().resolve()
    new_sources = verify_new_sources(root)
    if args.contract_only:
        print(json.dumps(dict(status='STATIC_CONTRACT_NO_TRAINING', **contract()), sort_keys=True))
        return 0
    base.require(all(getattr(args, name) is not None for name in
        ('baseline_plan','parent_checkpoint','frozen_ah_cache','w1_reference_output','output')),
        'all five exact input/output paths required')
    workspace = base.check_run_environment(dict(repo_root=str(root), output_root=str(args.output)))
    base.require(not args.output.exists() and not args.output.is_symlink(), 'fresh matched pair output required')
    old, _, reference_files = reference.validate_reference(root, args.w1_reference_output)
    plan = supervision.pinned(args.baseline_plan, previous.BASE_PLAN_SHA256)
    base.validate_plan(plan)
    sources = previous.verify_sources(root, plan)
    sources.update(old['source_sha256'])
    sources.update(new_sources)
    base.require(base.digest(args.parent_checkpoint) == previous.AH_CHECKPOINT_SHA256, 'exact original AH parent required')
    payload, digest = _read_checkpoint_file(args.parent_checkpoint, torch.device('cpu'))
    base.require(digest == previous.AH_CHECKPOINT_SHA256 and payload['checkpoint_schema'] == 'portable_e2e.near_horizon_ah.v1',
                 'secure original AH parent identity differs')
    initial = payload['model_state_dict']
    base.require(base.tensor_digest(initial) == payload['final_model_state_sha256'], 'parent tensor identity differs')
    cfg = base.ModelConfig.from_mapping(payload['model_config'])
    view = plan['views'][base.ARMS[1]]
    loaded, examples, scope = base._load_view(Path(view['path']), 'train', base.ARMS[1])
    base._verify_loaded_view(view, loaded, examples)
    base.require(scope == view['scope_precheck'] and len(examples) == 2787, 'exact W1 TRAIN scope differs')
    dataset = base.Common10TorchDataset(examples, cfg, verify_image_sha256=True, split='train')
    base.require(dataset.fingerprint_sha256 == reference.DATASET_FINGERPRINT, 'exact W1 dataset differs')
    off, batches, proof = supervision.make_provenance(Path(view['path']), examples, root, args.frozen_ah_cache)
    reference.validate_schedule(off, batches, proof)
    indices = w1.probe_indices(examples, proof)
    base.require(indices == old['fixed_probe_indices'], 'fixed W1 TRAIN probe identities differ')
    declaration = dict(contract(), source_sha256=sources, training_plan_sha256=PLAN_SHA256,
        baseline_plan_sha256=previous.BASE_PLAN_SHA256, reference_W1_declaration_sha256=reference.W1_DECLARATION_SHA,
        reference_W1_receipt_sha256=reference.W1_RECEIPT_SHA, training_count=2787,
        training_dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        provenance_receipt_sha256=supervision.digest_json(proof), warmup_mask_sha256=proof['warmup_mask_sha256'],
        exposure_order_sha256=proof['exposure_order_sha256'], fixed_probe_indices=indices,
        probe_tokens=[examples[i].token for i in indices], tail_anchor_count=proof['tail_anchor_count'],
        driving_future_crosses_tail_count=proof['driving_future_crosses_tail_count'],
        dataset_validation_may_read_nontrain_contract_records=True)
    if args.validate_only:
        print(json.dumps(dict(status='CPU_PREFLIGHT_PASS_NO_TRAINING', **declaration), sort_keys=True))
        return 0
    with base.gpu0_lease(workspace):
        gpu = base.preflight_gpu()
        device = torch.device('cuda:0')
        args.output.mkdir(parents=True)
        base.write_new(args.output/'declaration.json', declaration)
        base.write_new(args.output/'provenance_receipt.json', proof)
        base.write_new(args.output/'started.json', dict(status='FIXED_DIRECTIONAL_GRADIENT_PAIR', gpu=gpu,
            started_at=base.utc_now(), python=platform.python_version(), torch=torch.__version__, cuda=torch.version.cuda))
        for name in sources:
            base.copy_new(root/name, args.output/'source'/name)
        for name, path in reference_files.items():
            base.copy_new(path, args.output/'inputs'/name)
        base.copy_new(args.baseline_plan, args.output/'inputs/baseline_plan.json')
        training = {}
        for arm in ARMS:
            training[arm] = fit(arm, initial, cfg, dataset, off, batches, proof, args.output, device, declaration)
        base.write_new(args.output/'both_final_checkpoints_complete.json', training)
        for arm in ARMS:
            final, checkpoint_sha = _read_checkpoint_file(args.output/training[arm]['checkpoint'], torch.device('cpu'))
            base.require(checkpoint_sha == training[arm]['checkpoint_sha256'], 'final checkpoint differs before fixed TRAIN probe')
            model = base.PerspectiveTrajectoryModel(cfg).to(device)
            model.load_state_dict(final['model_state_dict'], strict=True)
            base.write_new(args.output/arm/'train_probe_after.json', w1.probe(model, dataset, indices, device))
            del model
            torch.cuda.empty_cache()
        # HH_261001 - No DEV loading/evaluation before both final endpoints are saved; runtime evaluation uses native model only.
        dev_view = plan['views']['Town03_DEV']
        dev, dev_examples, dev_scope = base._load_view(Path(dev_view['path']), 'val', 'Town03_DEV')
        base._verify_loaded_view(dev_view, dev, dev_examples)
        base.require(dev_scope == dev_view['scope_precheck'], 'known DEV scope differs')
        with evaluation_scope():
            evaluations = {arm: previous.evaluate_final(arm, training[arm], dev, cfg, args.output, device) for arm in ARMS}
        matched = all(base.digest(root/name) == expected == base.digest(args.output/'source'/name)
                      for name, expected in sources.items())
        base.require(matched, 'source changed during matched fit/evaluation')
        result = dict(schema=SCHEMA, status='COMPLETE_NOT_PROMOTED', training=training, evaluations=evaluations,
            B_vs_A=base.paired_nonregression(evaluations[ARMS[0]], evaluations[ARMS[1]]),
            strict_gates={arm:base.strict_gate(evaluations[arm]) for arm in ARMS}, source_hashes_match=matched,
            training_data_approved=False, model_promoted=False, vehicle_control_approved=False, completed_at=base.utc_now())
        base.write_new(args.output/'summary.json', result)
        print(json.dumps(dict(status=result['status'], training=training, strict_gates=result['strict_gates'])), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
