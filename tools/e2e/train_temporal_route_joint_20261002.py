"""HH_261002 - Fit true curvature AND speed with protected replay and unchanged causal101-tensor V3 architecture.

Fixed final6000,18TRAIN episodes,2 never-fit HOLDOUT episodes. The previous
97-frozen branch fit could not learn its substantial raw speed error; this
explicit NEW protocol learns54 post-input tensors and freezes47 input/semantic
tensors. It does not train STOP, raise the2.5m/s decoder bound, or approve cars.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import random
import time

if __name__ == "__mp_main__":
    # HH_261002 - Spawn data workers configure namespace only; they never initialize CUDA.
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_joint_worker_workspace", _entry)
    _module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader

from portable_e2e.deterministic_linear_interpolation_v2 import KERNEL_ID, DeterministicPathTimeDecoderV2
from portable_e2e.temporal_losses_v2 import temporal_trajectory_loss_v2
from portable_e2e.temporal_route_joint_export_research_20261002 import (
    SCHEMA as LOADER_SCHEMA, TRAINING_SCHEMA as SCHEMA, EXPORT_SCHEMA, MAX_UPDATES,
    TRAINABLE_PREFIXES, FAST_PREFIXES, BATCH_DOMAINS, PARENT_CHECKPOINT_SHA, PARENT_NPZ_SHA,
    PARENT_METADATA_SHA, PARENT_DECLARATION_SHA, frozen_fingerprint, trainable_names,
    load_joint_parent, learning_rate_config,
)
from portable_e2e.temporal_route_recovery_v3_20261001 import MODEL_ID, SCHEMA as ARCHITECTURE_SCHEMA
from scripts.e2e import train_temporal_route_recovery_moving_v4_20261001 as moving

ROOT = Path(__file__).resolve().parents[2]
previous, replay, joint, raw_parent = moving.previous, moving.previous.replay, moving.previous.joint, moving.previous.raw_parent
WORKSPACE, PERSONAL, GPU_UUID = previous.WORKSPACE, previous.PERSONAL, previous.GPU_UUID
SEED, MAX_FIT_WALL_SECONDS = 2026100201, 90 * 60
TRAIN_RECEIPT_SHA = "1b76264229e4e56d0a4c2c11944d52123c46f3709db447b99fcc20b281cfb29a"
HOLDOUT_RECEIPT_SHA = "d90c022464e716ad6b86eec06395df892349869b981b91b91eecd056ae926594"
require, digest, write_new = raw_parent.require, raw_parent.digest, raw_parent.write_new


def source_snapshot():
    # HH_261002 - Add only this NEW imported trainer/loader; every original24 source pin remains unchanged.
    original = moving.source_snapshot()
    seeds = [Path(__file__).resolve(), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]
    paths = set(replay.imported_source_closure(ROOT, seeds))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(all(result.get(k) == sha for k, sha in original.items()), "all24 frozen causal training dependencies required")
    return result


def freeze_inputs(model):
    for name, parameter in model.named_parameters(): parameter.requires_grad_(name.startswith(TRAINABLE_PREFIXES))
    actual = sorted(name for name, p in model.named_parameters() if p.requires_grad)
    require(actual == trainable_names(model.state_dict()) and len(actual) == 54
        and sum(p.numel() for p in model.parameters() if p.requires_grad) == 2277827,
        "exact54 post-input trainable tensors/2277827 parameters required")
    return actual


def learning_rate(update, group):
    require(type(update) is int and 1 <= update <= MAX_UPDATES and group in ("shared", "heads"),
        "fixed6000 LR and declared group required")
    config = learning_rate_config(); peak, final = config[group + "_peak"], config[group + "_final"]
    if update <= config["warmup_updates"]: return peak * (update / config["warmup_updates"])
    fraction = (update - config["warmup_updates"]) / (MAX_UPDATES - config["warmup_updates"])
    return final + .5 * (peak - final) * (1 + math.cos(math.pi * fraction))


def optimizer_groups(model):
    return [dict(name=name, params=[p for k, p in model.named_parameters()
        if p.requires_grad and k.startswith(FAST_PREFIXES) == (name == "heads")], lr=learning_rate(1, name))
        for name in ("shared", "heads")]


def fixed_batches(groups, legacy_strata):
    # HH_261002 - Real observed-state focus +all9 full-cycle +old327 +alternating true legacy curve/straight.
    focus_keys, full_keys = sorted(groups["focus"]), sorted(groups["full"])
    require(focus_keys and len(full_keys) == 9 and all(groups["focus"].values()) and all(groups["full"].values()),
        "actual signed focus/full9 queues required")
    focus = {k: replay.CycleQueue(v, SEED+100+i) for i, (k, v) in enumerate(sorted(groups["focus"].items()))}
    full = {k: replay.CycleQueue(v, SEED+200+i) for i, (k, v) in enumerate(sorted(groups["full"].items()))}
    low = replay.CycleQueue(list(range(327)), SEED+300)
    old = {kind: {k: replay.CycleQueue(v, SEED+400+20*p+i) for i, (k, v) in enumerate(sorted(legacy_strata[kind].items()))}
        for p, kind in enumerate(("curved", "straight"))}
    require(all(old.values()) and all(v for maps in old.values() for v in maps.values()), "nonempty actual legacy TRAIN strata required")
    counts, order = Counter(), []
    for step in range(MAX_UPDATES):
        a = focus[focus_keys[step % len(focus_keys)]].next(); b = full[full_keys[step % 9]].next()
        kind = "curved" if step % 2 == 0 else "straight"
        maps = sorted(old[kind]); selected = maps[counts[kind] % len(maps)]; counts[kind] += 1
        order.append([a, b, 2718+low.next(), 3045+old[kind][selected].next()])
    return order


def exact_objective(recovery0, recovery1, low, legacy_curvature, legacy_speed):
    # HH_261002 - Per-anchor true masks are unchanged; speed replay is explicitly NEW, not hidden in a curvature-only declaration.
    return .25 * (recovery0 + recovery1 + low + .25 * legacy_curvature + legacy_speed)


def finite_clip_gradients(model):
    # HH_261002 - Finite individual gradients do not guarantee a finite aggregate norm; never step after norm overflow.
    norm = torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 5.)
    require(bool(torch.isfinite(norm)), "aggregate joint gradient norm nonfinite; no optimizer update permitted")
    return norm


def scope_fields():
    return dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, stop_intent_trained=False,
        backbone_frozen=False, visual_encoder_frozen=True, semantic_head_parameters_frozen=True,
        semantic_outputs_preserved=False, labels_clipped=False, labels_extrapolated=False,
        legacy_decoded=False, independent_validation=False, checkpoint_selection_by_holdout=False,
        startup_recovery_fitted=False, new_signed_recovery_cohort_moving_only=True,
        fixed_recovery_driving_relative_window_seconds=[4.0, 34.1], fixed_recovery_native_frame_offsets=[80, 682],
        runtime_intent="explicit_cruise_GO_not_head", training_episode_count=18, recovery_training_episode_count=9,
        replay_training_episode_count=9, holdout_episode_count=2,
        current_training_purpose="causal_basic_driving_joint_curvature_speed_continuation",
        speed_tracking_scope="offline2.5mps research only; not30kph SPD01", official_main_features_approved=0,
        official_subfeatures_approved=0, parent_diagnostic_pass_count=20, parent_diagnostic_screen_count=48,
        parent_all_diagnostic_screens_passed=False,
        architecture_training_interface_notice="same101 causal tensor architecture;54 weights learn,47 byte-fixed; STOP outputs not preserved or claimed")


def endpoint_screens(before, after):
    # HH_261002 - Retain all48 fixed absolute/preservation screens; add explicit true-speed improvement and replay guards.
    original = previous.endpoint_screens(before, after); checks = list(original["checks"])
    for cohort in ("recovery_train", "recovery_holdout"):
        for episode, row in after[cohort]["per_episode"].items():
            value = row["instantaneous_speed_mae_mps"]
            threshold = before[cohort]["per_episode"][episode]["instantaneous_speed_mae_mps"] * .8
            checks.append(dict(name=cohort+"/"+episode+"/speed_MAE20percent_improvement", actual=value,
                threshold=threshold, passed=math.isfinite(value) and value <= threshold))
    for cohort in ("legacy", "known599"):
        for label in ("aggregate", *sorted(after[cohort]["per_episode"])):
            old = before[cohort]["groups"]["all_eligible"] if label == "aggregate" else before[cohort]["per_episode"][label]
            new = after[cohort]["groups"]["all_eligible"] if label == "aggregate" else after[cohort]["per_episode"][label]
            value, threshold = new["raw_spatial_preferred_speed_mae_mps"], old["raw_spatial_preferred_speed_mae_mps"] * 1.05 + 1e-4
            checks.append(dict(name=cohort+"/"+label+"/raw_speed_MAE5percent_preservation", actual=value,
                threshold=threshold, passed=math.isfinite(value) and value <= threshold))
    require(len(checks) == len({r["name"] for r in checks}) == 71, "all48 original+11 speed gains+12 replay speed screens required")
    return dict(fixed_endpoint_updates=6000, diagnostic_screen_count=71, retained_original_screen_count=48, checks=checks,
        all_diagnostic_screens_passed=all(c["passed"] for c in checks), independent_validation=False,
        feature_approved=False, vehicle_control_approved=False, model_promoted=False,
        absolute_30kph_speed_tracking_claimed=False, fixed_final_only=True, holdout_selected_checkpoint=False)


def save_checkpoint(output, model, optimizer, step, sources, identity, holdout_fp, declaration_sha, frozen_sha):
    replay.storage_reserve(output)
    state = model.state_dict(); names = trainable_names(state)
    require(frozen_fingerprint(state) == frozen_sha and len(state) == 101,
        "all47 frozen tensors must remain exact before any export")
    require(all(v.dtype == torch.float32 and bool(torch.isfinite(v).all()) for v in state.values()), "finite101 float32 export required")
    document = dict(schema=SCHEMA, training_schema=SCHEMA, architecture_schema=ARCHITECTURE_SCHEMA, model_id=MODEL_ID,
        model_config=model.config.to_dict(), recovery_config=model.recovery_config.to_dict(), model_state_dict=state,
        optimizer_state_dict=optimizer.state_dict(), optimizer_updates=step, additional_optimizer_updates=step,
        maximum_optimizer_updates=6000, effective_batch_size=4, examples_seen=4*step,
        recovery_examples_seen=2*step, v4_examples_seen=step, legacy_examples_seen=step,
        parent_optimizer_updates=3000, parent_known_total_optimizer_updates=6200, known_total_optimizer_updates=6200+step,
        parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA, parent_npz_sha256=PARENT_NPZ_SHA,
        parent_metadata_sha256=PARENT_METADATA_SHA, parent_declaration_sha256=PARENT_DECLARATION_SHA,
        frozen_state_sha256=frozen_sha, frozen_state_names=sorted(set(state)-set(names)), trainable_state_names=names,
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, trainable_parameter_count=2277827,
        batch_domains=BATCH_DOMAINS, source_sha256=sources, training_dataset_fingerprint_sha256=identity,
        holdout_dataset_fingerprint_sha256=holdout_fp, declaration_sha256=declaration_sha,
        learning_rate=learning_rate_config(), **scope_fields())
    folder = output / "checkpoints"; folder.mkdir(exist_ok=True); checkpoint = folder/("step_%06d.pt" % step)
    with checkpoint.open("xb") as stream: torch.save(document, stream)
    export = output/"exports"/("step_%06d" % step); export.mkdir(parents=True, exist_ok=False)
    arrays = {k: v.detach().cpu().numpy().copy() for k, v in state.items()}; npz = export/"model_state.npz"
    with npz.open("xb") as stream: np.savez(stream, **arrays)
    metadata = {k: v for k, v in document.items() if k not in ("model_state_dict", "optimizer_state_dict")}
    metadata.update(schema=EXPORT_SCHEMA, checkpoint_sha256=digest(checkpoint), npz_sha256=digest(npz),
        state_tensor_shapes={k: list(v.shape) for k,v in arrays.items()}, state_tensor_dtypes={k: str(v.dtype) for k,v in arrays.items()},
        numerical_revision=dict(kernel_id=KERNEL_ID, strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False))
    write_new(export/"metadata.json", metadata)
    return dict(optimizer_updates=step, path=str(checkpoint), sha256=digest(checkpoint), optimizer_state_retained=True,
        export_npz_sha256=digest(npz), export_metadata_sha256=digest(export/"metadata.json"))


def load_cohorts(args):
    low, legacy, known, strata, old_receipts = replay.load_cohorts(replay.LOW_PATH, replay.LEGACY_PATH, replay.DEV_PATH)
    require(args.recovery_conversion_sha256 == TRAIN_RECEIPT_SHA and args.holdout_conversion_sha256 == HOLDOUT_RECEIPT_SHA,
        "exact already-qualified full11 V4 actual receipts required")
    with moving.activated():
        training, train_audits, train_receipt = previous.load_admitted(args.recovery_train, "train", args.recovery_conversion, TRAIN_RECEIPT_SHA)
        holdout, holdout_audits, holdout_receipt = previous.load_admitted(args.recovery_holdout, "val", args.holdout_conversion, HOLDOUT_RECEIPT_SHA)
    datasets = (training, holdout, low, legacy, known)
    tokens = [{r.token for r in d.examples} for d in datasets]
    episodes = [{r.episode_id for r in d.examples} for d in datasets]
    require(all(not a & b for i,a in enumerate(tokens) for b in tokens[i+1:])
        and all(not a & b for i,a in enumerate(episodes) for b in episodes[i+1:]), "cross-cohort repeated view/VAL/HOLDOUT leakage")
    require(previous.read(args.recovery_conversion)["full_cohort_case_provenance"] == previous.read(args.holdout_conversion)["full_cohort_case_provenance"],
        "one identical full11 actual qualification required")
    require(len(known.examples) == 665 and len(replay.legacy_strata(known)["eligible"]) == 599,
        "known previously-exposed665/599 diagnostic denominator unchanged")
    return training, train_audits, train_receipt, holdout, holdout_audits, holdout_receipt, low, legacy, known, strata, old_receipts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("recovery-train", "recovery-holdout", "recovery-conversion", "holdout-conversion", "parent-weights", "parent-metadata",
        "backbone-parent-weights", "backbone-parent-metadata", "output"): parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--recovery-conversion-sha256", required=True); parser.add_argument("--holdout-conversion-sha256", required=True)
    parser.add_argument("--loader-workers", type=int, choices=(0,1,2), default=2); parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    require(not args.output.exists() and not args.output.is_symlink()
        and args.output.resolve().is_relative_to((WORKSPACE/"runs/training").resolve()), "fresh create-only personal training run required")
    initial_storage = replay.storage_reserve(args.output.parent); torch.set_num_threads(4); sources = source_snapshot()
    (training, train_audits, train_receipt, holdout, holdout_audits, holdout_receipt,
        low, legacy, known, strata, old_receipts) = load_cohorts(args)
    model = load_joint_parent(args.parent_weights, args.parent_metadata, backbone_parent_weights=args.backbone_parent_weights,
        backbone_parent_metadata=args.backbone_parent_metadata, source_root=ROOT)
    # HH_261002 - No parameters are reset; all101 initialize exactly from the actual full numeric final3000 parent.
    train_names = freeze_inputs(model); frozen_sha = frozen_fingerprint(model.state_dict())
    groups = previous.recovery_groups(training, train_audits); order = fixed_batches(groups, strata)
    identity_data = dict(recovery=train_receipt, old=old_receipts, seed=SEED, batch_domains=BATCH_DOMAINS)
    identity = hashlib.sha256(json.dumps(identity_data, sort_keys=True, allow_nan=False).encode()).hexdigest()
    declaration = dict(schema=SCHEMA, architecture_schema=ARCHITECTURE_SCHEMA, model_id=MODEL_ID,
        model_config=model.config.to_dict(), recovery_config=model.recovery_config.to_dict(),
        parent_checkpoint_sha256=PARENT_CHECKPOINT_SHA, parent_npz_sha256=PARENT_NPZ_SHA,
        parent_metadata_sha256=PARENT_METADATA_SHA, parent_declaration_sha256=PARENT_DECLARATION_SHA,
        parent_optimizer_updates=3000, parent_known_total_optimizer_updates=6200,
        cohorts=dict(recovery_train=train_receipt, recovery_holdout=holdout_receipt, replay=old_receipts),
        source_sha256=sources, training_dataset_fingerprint_sha256=identity, holdout_dataset_fingerprint_sha256=holdout.fingerprint_sha256,
        trainable_state_names=train_names, frozen_state_names=sorted(set(model.state_dict())-set(train_names)),
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, trainable_parameter_count=2277827, frozen_state_sha256=frozen_sha,
        actual_noncentered_grid_counts={str(k): len(v) for k,v in groups["focus"].items()},
        actual_profile_full302_counts={str(k): len(v) for k,v in groups["full"].items()},
        requested_optimizer_updates=6000, maximum_optimizer_updates=6000, effective_batch_size=4,
        examples_planned=24000, recovery_exposures_planned=12000, v4_exposures_planned=6000, legacy_exposures_planned=6000,
        actual_dataset_available_train_anchor_count=5832, actual_eligible_train_pool_anchor_count=5532,
        fixed_schedule_sha256=hashlib.sha256(json.dumps(order, separators=(",", ":")).encode()).hexdigest(),
        declared_unique_source_exposure_counts=dict(recovery=len({i for r in order for i in r[:2]}),
            v4=len({r[2] for r in order}), legacy=len({r[3] for r in order})),
        objective=".25 recovery-focus joint +.25 recovery-full joint +.25 oldV4 joint +.25(.25 legacy raw curvature +legacy raw speed)",
        loss_denominators="per-anchor true valid spatial knots and three normalized true temporal horizons; unchanged masks/no extrapolation",
        selection_rule="fixed final6000 only; all71 screens retained; no best-checkpoint/DEV/holdout selection",
        seed=SEED, learning_rate=learning_rate_config(), batch_domains=BATCH_DOMAINS,
        maximum_fit_including_baseline_seconds=MAX_FIT_WALL_SECONDS, postfit_checks_separately_timed=True,
        initial_storage=initial_storage, holdout_fitted=False, holdout_selected_checkpoint=False,
        fair_three_seed_comparison_performed=False, **scope_fields())
    if args.validate_only:
        print(json.dumps(dict(status="VALIDATED_CPU_NO_CUDA_OR_OUTPUT", **declaration), indent=2, allow_nan=False)); return 0
    with raw_parent.gpu0_lease():
        owned_started = time.monotonic(); replay.storage_reserve(args.output.parent); gpu = raw_parent.gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "strict deterministic cuBLAS environment required")
        random.seed(SEED); torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
        torch.use_deterministic_algorithms(True); torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
        args.output.mkdir(parents=True, exist_ok=False); write_new(args.output/"declaration.json", declaration)
        declaration_sha = digest(args.output/"declaration.json"); joint.archive_sources(args.output, sources)
        device = torch.device("cuda:0"); model.to(device); decoder = DeterministicPathTimeDecoderV2().to(device)
        optimizer = torch.optim.AdamW(optimizer_groups(model), weight_decay=1e-4)
        write_new(args.output/"started.json", dict(gpu=gpu, pid=os.getpid(), started_utc=datetime.now(timezone.utc).isoformat(),
            actual_parent_all101_loaded=True, trainable_state_tensor_count=54, frozen_state_tensor_count=47,
            trainable_parameter_count=2277827, frozen_state_sha256=frozen_sha))
        def evaluate_all(stage):
            metrics = dict(recovery_train=previous.evaluate_joint(model, decoder, training, device, train_audits),
                recovery_holdout=previous.evaluate_joint(model, decoder, holdout, device, holdout_audits),
                v4=previous.evaluate_joint(model, decoder, low, device),
                known599=dict(previous.evaluate_legacy_raw(model, known, replay.legacy_strata(known), device),
                    role="previously_exposed_known_DEV599_raw_diagnostic", available_known_val_count=665, excluded_count=66),
                legacy=previous.evaluate_legacy_raw(model, legacy, strata, device))
            for key,value in metrics.items(): write_new(args.output/(key+"_"+stage+".json"), value)
            return metrics
        before = evaluate_all("before")
        options = dict(dataset=ConcatDataset([training, low, legacy]), batch_sampler=order,
            num_workers=args.loader_workers, pin_memory=True)
        if args.loader_workers: options.update(prefetch_factor=2, persistent_workers=True, multiprocessing_context="spawn")
        loader = DataLoader(**options); completed, wall_stop, checkpoints = 0, False, []
        with (args.output/"history.jsonl").open("x") as journal:
            for step,batch in enumerate(loader, 1):
                if time.monotonic()-owned_started >= MAX_FIT_WALL_SECONDS: wall_stop=True; break
                batch = raw_parent.move(batch, device); model.train(); optimizer.zero_grad(set_to_none=True)
                cohorts = (training.examples, low.examples, legacy.examples)
                expected = [cohorts[0][i].token if i<2718 else cohorts[1][i-2718].token if i<3045 else cohorts[2][i-3045].token for i in order[step-1]]
                require(list(batch["sample_id"]) == expected, "actual TRAIN tokens differ from fixed schedule")
                terms = []
                for position in range(3):
                    _,_,domain = joint.joint_forward(model, decoder, replay.slice_batch(batch, position, position+1)); terms.append(domain)
                old_batch = replay.slice_batch(batch, 3, 4); raw = raw_parent.forward(model, old_batch)
                old = temporal_trajectory_loss_v2(raw, old_batch)
                old_loss = .25*old["raw_curvature"]+old["raw_preferred_speed"]
                gradients = {name: previous.finite_branch_gradient(value, model) for name,value in zip(BATCH_DOMAINS,
                    (terms[0]["total"], terms[1]["total"], terms[2]["total"], old_loss))}
                loss = exact_objective(terms[0]["total"], terms[1]["total"], terms[2]["total"], old["raw_curvature"], old["raw_preferred_speed"])
                require(bool(torch.isfinite(loss)), "joint objective nonfinite"); loss.backward()
                require(all(p.grad is None or bool(torch.isfinite(p.grad).all()) for p in model.parameters()), "joint gradients nonfinite")
                require(all(p.grad is None for name,p in model.named_parameters() if name not in train_names), "frozen input/semantic tensor received gradient")
                norm = finite_clip_gradients(model)
                for group in optimizer.param_groups: group["lr"] = learning_rate(step, group["name"])
                optimizer.step(); completed=step
                row = dict(optimizer_updates=step, examples_seen=4*step, recovery_examples_seen=2*step,
                    v4_examples_seen=step, legacy_examples_seen=step, indices=order[step-1], sample_ids=list(batch["sample_id"]),
                    loss=float(loss.detach()), recovery0=joint.loss_numbers(terms[0]), recovery1=joint.loss_numbers(terms[1]),
                    v4=joint.loss_numbers(terms[2]), legacy=joint.loss_numbers(old), per_domain_trainable_gradients=gradients,
                    learning_rates={g["name"]:g["lr"] for g in optimizer.param_groups}, preclip_trainable_gradient_norm=float(norm),
                    elapsed_owned_seconds=time.monotonic()-owned_started)
                journal.write(json.dumps(row, allow_nan=False)+"\n"); journal.flush()
                if step%100 == 0: print(json.dumps(dict(status="ACTUAL_JOINT_PARTIAL_BACKBONE_TRAINING", **row)), flush=True)
                if step%500 == 0: checkpoints.append(save_checkpoint(args.output, model, optimizer, step, sources, identity,
                    holdout.fingerprint_sha256, declaration_sha, frozen_sha))
        fit_seconds = time.monotonic()-owned_started; endpoint, post_seconds = None, 0.
        if completed == MAX_UPDATES:
            post_started = time.monotonic(); after = evaluate_all("final"); endpoint = endpoint_screens(before, after)
            write_new(args.output/"endpoint_diagnostic_screens.json", endpoint)
            write_new(args.output/"actual_signed_branch_response_train.json", previous.branch_sign_response(before["recovery_train"], after["recovery_train"], train_audits))
            write_new(args.output/"actual_signed_branch_response_holdout.json", previous.branch_sign_response(before["recovery_holdout"], after["recovery_holdout"], holdout_audits))
            post_seconds = time.monotonic()-post_started
        require(frozen_fingerprint(model.state_dict()) == frozen_sha, "final47 frozen tensors changed")
        replay.verify_sources(args.output, sources)
        summary = dict(schema=SCHEMA, status="FIXED6000_JOINT_COMPLETE_NOT_PROMOTED" if completed==6000 else "FIT_WALL_BUDGET_STOP_NOT_PROMOTED",
            optimizer_updates=completed, examples_seen=4*completed, recovery_examples_seen=2*completed,
            v4_examples_seen=completed, legacy_examples_seen=completed, wall_budget_stop=wall_stop,
            frozen47_tensors_byte_identical=True, full97_frozen_claimed=False, source_hashes_match=True,
            fit_including_baseline_seconds=fit_seconds, postfit_endpoint_seconds=post_seconds,
            elapsed_owned_seconds=time.monotonic()-owned_started, checkpoints=checkpoints,
            endpoint_diagnostic_screens=endpoint, **scope_fields())
        write_new(args.output/"summary.json", summary); print(json.dumps(summary, allow_nan=False), flush=True)
    return 0


if __name__ == "__main__": raise SystemExit(main())
