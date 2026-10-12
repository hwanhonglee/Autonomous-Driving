"""HH_261004 - Compare replay retention on qualified TRAIN only before an expensive continuation.

This fixed 1000-update pair is an engineering pilot, NOT feature validation.
Both arms start at GOOD6000 and consume the same prefix of the original B5
schedule and learning rates. No DEV/HOLDOUT/TEST loader or simulator is used.
An endpoint is never selected or deployed automatically.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import sys
import time

from scripts.e2e import train_temporal_left_right_joint_v2_20261002 as frozen
from portable_e2e import trajectory_retention_loss_20261004 as retention

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_retention_train_pair_pilot.20261004.v1"
STUDY = "temporal_retention_pair_pilot_20261004_v1"
FROZEN_SHA = "cd991d5ba46b30eb23b5115a2dffb75130f2b182fb81701c13991977c8269a11"
UPDATES, COEFFICIENT, PANEL_SIZE = 1000, .25, 32
ARMS = ("control", "retention")
ROLES = ("OLD_RECOVERY_REPLAY", "OLD_LOW_SPEED_REPLAY", "OLD_LEGACY_REPLAY")
require, digest, write_new = frozen.require, frozen.digest, frozen.write_new
torch, np, pair, raw_parent = frozen.torch, frozen.np, frozen.pair, frozen.raw_parent
_created_output = None


def source_snapshot():
    require(digest(frozen.__file__) == FROZEN_SHA, "frozen B5 training source changed")
    old = frozen.source_snapshot()
    seeds = [Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]
    paths = set(frozen.replay.imported_source_closure(ROOT, seeds))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(all(result.get(k) == v for k, v in old.items()), "all original31 sources must remain exact")
    return result


def contract():
    # HH_261004 - Freeze the pilot budget and coefficient before seeing either endpoint.
    return dict(schema=SCHEMA, study_id=STUDY, sources=source_snapshot(),
        parent=frozen.pair.PARENT, parent_history_sha256=frozen.PARENT_HISTORY_SHA,
        arm_order=list(ARMS), updates_per_arm=UPDATES, exposures_per_arm=5 * UPDATES,
        effective_batch_size=5, seed=frozen.SEED, coefficient=COEFFICIENT,
        objective="original_B5_objective + (0 or .25) * mean_old_TRAIN_raw_retention",
        learning_rates="exact prefix of original fixed20000 schedule; no new cosine horizon",
        full_original_TRAIN_membership=6876, eligible_TRAIN_membership=6576,
        immutable_five_domain_schedule=True, teacher="frozen GOOD6000 neural predictions, not expert control",
        retention_roles=list(ROLES), qualification_loaders_unchanged=True,
        panel_anchors_per_domain=PANEL_SIZE, panel_selection="uniform index before optimizer, eligible legacy only",
        parent_and_both_arms_same_full_panel=True, panel_is_TRAIN_resubstitution_only=True,
        evaluation_only_payloads_opened=False, independent_generalization_claimed=False,
        source_image_SHA_pre_and_postfit_required=True, original47_frozen_tensors_exact=True,
        automatic_retry=False, automatic_longer_training=False, model_promoted=False,
        simulator_control_authorized=False, vehicle_control_approved=False,
        official_main_features_approved=0, official_subfeatures_approved=0)


def uniform_panel(count, eligible=None):
    require(type(count) is int and count >= PANEL_SIZE, "complete TRAIN cohort required")
    candidates = list(range(count)) if eligible is None else sorted(eligible)
    require(len(candidates) >= PANEL_SIZE and len(candidates) == len(set(candidates))
        and all(type(i) is int and 0 <= i < count for i in candidates), "honest eligible TRAIN panel required")
    return [candidates[round(i * (len(candidates) - 1) / (PANEL_SIZE - 1))] for i in range(PANEL_SIZE)]


def objective(base_loss, retained_loss, arm):
    require(arm in ARMS, "unknown paired intervention")
    # HH_261004 - CONTROL retains the exact original tensor and floating expression, not base + zero.
    return base_loss if arm == "control" else base_loss + COEFFICIENT * retained_loss


def read_authority(path, expected):
    value = frozen.pinned_read(path, expected)
    require(value.get("schema") == SCHEMA + ".authorization" and value.get("authorized") is True
        and value.get("contract") == contract() and type(value.get("physical_GPU_index")) is int
        and value["physical_GPU_index"] == 0
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "new exact source/parent/paired TRAIN-only authority required")
    return value


def freeze_teacher(parent, device):
    teacher = pair.fresh_model(parent, device).eval()
    for parameter in teacher.parameters():
        parameter.requires_grad_(False)
    return teacher


def reset_seed():
    torch.manual_seed(frozen.SEED); torch.cuda.manual_seed_all(frozen.SEED)
    random.seed(frozen.SEED); np.random.seed(frozen.SEED)


def teacher_raw(teacher, cpu_batch, device, cache):
    token = cpu_batch["sample_id"][0]
    byte_sha = pair.batch_digest(cpu_batch)
    if token not in cache:
        with torch.no_grad():
            prediction = raw_parent.forward(teacher, raw_parent.move(cpu_batch, device))
        cache[token] = (byte_sha, {k: prediction[k].detach().cpu().clone()
            for k in ("curvature_knots", "preferred_speed_knots")})
    require(cache[token][0] == byte_sha, "same TRAIN token was collated from different input bytes")
    return {k: v.to(device) for k, v in cache[token][1].items()}


def make_provenance(datasets, receipts, authority_sha, recovery_root, legacy_eligible):
    # HH_261004 - These are authenticated loader receipts, not a fabricated dataset admission.
    manifest_pins = ("5b6d12b00bb718e5871447ce79df05e5b65cd652fa3924085c11250a6ebf6449",
        frozen.replay.LOW_MANIFEST_SHA, raw_parent.TRAIN_MANIFEST_SHA)
    roots = (recovery_root, frozen.replay.LOW_PATH, frozen.replay.LEGACY_PATH)
    require(all(digest(path / "dataset.json") == sha for path, sha in zip(roots, manifest_pins)),
        "verified OLD TRAIN manifest changed before retention binding")
    result = []
    for pos, (dataset, role, key, manifest) in enumerate(zip(datasets, ROLES, ("recovery", "v4", "legacy"), manifest_pins)):
        loader_receipt_sha = frozen.json_sha(receipts[key])
        eligible = set(range(len(dataset))) if pos != 2 else set(legacy_eligible)
        result.append({r.token: retention.ReplayProvenance(r.token, "TRAIN", role, manifest,
            authority_sha, loader_receipt_sha, True, False) for i, r in enumerate(dataset.examples) if i in eligible})
        require(len(result[-1]) == len(eligible), "duplicate authenticated eligible TRAIN token")
    return result


def retention_term(raw, reference, batch, proof):
    return retention.trajectory_retention_loss(raw, reference,
        observed_spatial_valid=batch["spatial_valid"], curvature_valid=batch["spatial_curvature_valid"],
        preferred_speed_valid=batch["spatial_speed_valid"], provenance=[proof],
        sample_ids=batch["sample_id"], parent_checkpoint_sha256=pair.PARENT["checkpoint_sha256"])


def panel_measurements(parent, models, datasets, panels, provenance, device):
    decoder = frozen.original.DeterministicPathTimeDecoderV2().to(device)
    teacher = freeze_teacher(parent, device); cache = {}; result = {}
    with torch.no_grad():
        for name, model in (("parent", parent), *models.items()):
            model.to(device).eval(); domains = {}
            for p, (dataset, indices) in enumerate(zip(datasets, panels)):
                records = []
                for index in indices:
                    cpu = frozen.default_collate([dataset[index]]); batch = raw_parent.move(cpu, device)
                    if p == 2:
                        raw = raw_parent.forward(model, batch)
                        loss = frozen.original.temporal_trajectory_loss_v2(raw, batch); decoded = None
                    else:
                        raw, decoded, loss = frozen.joint.joint_forward(model, decoder, batch)
                    row = dict(index=index, sample_id=cpu["sample_id"][0],
                        true_raw_curvature_huber=float(loss["raw_curvature"]),
                        true_raw_speed_huber=float(loss["raw_preferred_speed"]),
                        observed_curvature_knots=int(batch["spatial_curvature_valid"].sum()),
                        observed_speed_knots=int(batch["spatial_speed_valid"].sum()))
                    if decoded is not None:
                        valid = batch["target_valid"][0]
                        errors = torch.linalg.vector_norm(decoded["xy"][0, 0] - batch["target_xy"][0], dim=-1)
                        row.update(true_ADE_m=float(errors[valid].mean()), true_FDE_m=float(errors[valid][-1]),
                            true_speed_MAE_mps=float((decoded["instantaneous_speed"][0, 1:] - batch["target_speed_mps"][0]).abs()[valid].mean()))
                    if p < 3:
                        reference = teacher_raw(teacher, cpu, device, cache)
                        row["parent_raw_retention"] = float(retention_term(raw, reference, batch, provenance[p][row["sample_id"]])["total"])
                    records.append(row)
                numeric = [k for k in records[0] if k.startswith("true_") or k == "parent_raw_retention"]
                domains[str(p)] = dict(rows=records, means={k: sum(r[k] for r in records) / len(records) for k in numeric},
                    fixed_denominator=len(records), TRAIN_resubstitution_only=True)
            result[name] = domains; model.cpu()
    return result


def fit_arm(output, arm, parent, dataset, order, teacher, provenance, device, source_pins):
    reset_seed(); model = pair.fresh_model(parent, device).train()
    decoder = frozen.original.DeterministicPathTimeDecoderV2().to(device)
    groups = frozen.original.optimizer_groups(model)
    optimizer = torch.optim.AdamW(groups, weight_decay=1e-4)
    cache = {}; frozen_sha = frozen.original.frozen_fingerprint(parent.state_dict())
    output.mkdir(exist_ok=False); started = time.monotonic()
    with (output / "history.jsonl").open("x") as journal:
        for step, indices in enumerate(order, 1):
            require(time.monotonic() - started <= 3600, "fixed one-hour per-arm infrastructure budget exceeded")
            cpu = frozen.default_collate([dataset[i] for i in indices]); byte_sha = pair.batch_digest(cpu)
            require(list(cpu["sample_id"]) == frozen.sample_ids(dataset.datasets, indices), "fixed B5 identities differ")
            batch = raw_parent.move(cpu, device); optimizer.zero_grad(set_to_none=True)
            base_loss, terms, legacy, _ = frozen.domains(model, decoder, batch)
            retained = []
            for p in range(3):
                old_cpu = frozen.replay.slice_batch(cpu, p, p + 1)
                old_batch = frozen.replay.slice_batch(batch, p, p + 1)
                raw = raw_parent.forward(model, old_batch)
                reference = teacher_raw(teacher, old_cpu, device, cache)
                retained.append(retention_term(raw, reference, old_batch,
                    provenance[p][old_cpu["sample_id"][0]])["total"])
            kept = torch.stack(retained).mean(); loss = objective(base_loss, kept, arm)
            require(bool(torch.isfinite(loss)), "nonfinite pilot objective")
            loss.backward(); frozen.original.finite_clip_gradients(model)
            require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in model.parameters() if p.requires_grad)
                and all(p.grad is None for p in model.parameters() if not p.requires_grad), "54 finite gradients/47 frozen required")
            for group in optimizer.param_groups:
                group["lr"] = frozen.learning_rate(step, group["name"])
            optimizer.step()
            row = dict(optimizer_updates=step, exposures=5 * step, indices=indices,
                sample_ids=list(cpu["sample_id"]), batch_sha256=byte_sha,
                original_B5_objective=float(base_loss.detach()), retention=float(kept.detach()),
                total=float(loss.detach()), elapsed_seconds=time.monotonic() - started)
            journal.write(json.dumps(row, allow_nan=False) + "\n"); journal.flush()
            if step % 100 == 0:
                print(json.dumps(dict(arm=arm, **row)), flush=True)
        require(step == UPDATES and len(model.state_dict()) == 101
            and frozen.original.frozen_fingerprint(model.state_dict()) == frozen_sha,
            "complete fixed1000 endpoint and original47 frozen weights required")
    require(all(torch.isfinite(v).all() for v in model.state_dict().values()), "nonfinite endpoint")
    npz = output / "model_state.npz"
    with npz.open("xb") as stream:
        np.savez(stream, **{k: v.detach().cpu().numpy() for k, v in model.state_dict().items()})
    result = dict(arm=arm, optimizer_updates=UPDATES, exposures=5 * UPDATES,
        fit_seconds=time.monotonic() - started, npz_sha256=digest(npz), history_sha256=digest(output / "history.jsonl"),
        frozen47_sha256=frozen_sha, cached_teacher_tokens=len(cache), inference_loader_authorized=False,
        model_promoted=False, vehicle_control_approved=False)
    write_new(output / "fit_result.json", result)
    write_new(output / "metadata.json", dict(result, schema=SCHEMA + ".numeric_research_export",
        source_sha256=source_pins, declaration_sha256=digest(output.parent / "declaration.json"),
        parent=frozen.pair.PARENT, model_id=model.model_id, model_config=model.config.to_dict(),
        recovery_config=model.recovery_config.to_dict(), resume_implemented=False,
        raw_npz_is_not_a_runtime_authorization=True))
    frozen.replay.verify_sources(output.parent, source_pins)
    return model.cpu(), result


def parser():
    value = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    value.add_argument("--mode", required=True, choices=("contract", "validate-only", "fit"))
    for name in ("parent-run", "warm-parent-weights", "warm-parent-metadata", "backbone-parent-weights", "backbone-parent-metadata",
        "recovery-train", "recovery-conversion", "left-train", "right-train", "output"):
        value.add_argument("--" + name, type=Path)
    value.add_argument("--parent-history-sha256", default=frozen.PARENT_HISTORY_SHA)
    value.add_argument("--left-conversion-sha256")
    value.add_argument("--right-conversion-sha256", default=frozen.RIGHT_CONVERSION_SHA)
    value.add_argument("--pilot-authority", type=Path)
    value.add_argument("--pilot-authority-sha256")
    return value


def run(args):
    global _created_output
    proposed = contract()
    if args.mode == "contract":
        print(json.dumps(proposed, indent=2)); return 0
    require(Path(sys.prefix).resolve() == frozen.original.WORKSPACE / "venvs/py312", "existing private venv only")
    require(all(getattr(args, key) is not None for key in ("parent_run", "warm_parent_weights", "warm_parent_metadata",
        "backbone_parent_weights", "backbone_parent_metadata", "recovery_train", "recovery_conversion", "left_train", "right_train",
        "output", "left_conversion_sha256", "pilot_authority", "pilot_authority_sha256")), "explicit authenticated TRAIN-only inputs required")
    require(args.parent_history_sha256 == frozen.PARENT_HISTORY_SHA, "only original GOOD6000 parent history")
    expected_output = frozen.original.WORKSPACE / "runs/training" / STUDY
    require(args.output == expected_output and not args.output.exists() and not any(p.is_symlink() for p in (args.output, *args.output.parents)),
        "fresh exact personal pilot output required")
    read_authority(args.pilot_authority, args.pilot_authority_sha256)
    torch.set_num_threads(4)
    training, audits, low, legacy, strata, receipts = pair.load_train(args)
    left, left_receipt, left_pins = frozen.load_LEFT(args); right, right_receipt, right_pins = frozen.load_RIGHT(args)
    datasets = (training, low, legacy, left, right); pair.disjoint(datasets)
    groups = frozen.previous.recovery_groups(training, audits)
    parent_order = frozen.original.fixed_batches(groups, strata)
    parent, pins, _ = pair.verify_parent(args, parent_order, (training, low, legacy))
    order = frozen.fixed_batches(groups, strata)[:UPDATES]; dataset = frozen.ConcatDataset(datasets)
    require(len(dataset) == 6876 and len(order) == UPDATES, "complete qualified cohorts and fixed1000 schedule required")
    pins.update(left_pins); pins.update(right_pins)
    receipts = dict(receipts, LEFT=left_receipt, RIGHT=right_receipt)
    panels = [uniform_panel(len(d), strata["eligible"] if i == 2 else None) for i, d in enumerate(datasets)]
    provenance = make_provenance(datasets[:3], receipts, args.pilot_authority_sha256, args.recovery_train, strata["eligible"])
    declaration = dict(proposed, source_sha256=proposed["sources"], dataset_receipts=receipts,
        fixed_schedule_sha256=frozen.json_sha(order), fixed_sample_ids=[frozen.sample_ids(datasets, r) for r in order],
        TRAIN_panels={str(i): [d.examples[j].token for j in panels[i]] for i, d in enumerate(datasets)},
        immutable_input_sha256=pins, authorization_sha256=args.pilot_authority_sha256)
    if args.mode == "validate-only":
        require(not torch.cuda.is_initialized(), "CPU preflight must not initialize CUDA")
        print(json.dumps(dict(status="PASS_QUALIFIED20_TRAIN_CPU_ONLY", declaration=declaration))); return 0
    require(args.mode == "fit", "pilot permits only contract/validate-only/fit")
    frozen.replay.storage_reserve(args.output.parent)
    with raw_parent.gpu0_lease():
        gpu = frozen.gpu_revision.gpu_preflight(); device = torch.device("cuda:0")
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "strict CuBLAS workspace required")
        torch.use_deterministic_algorithms(True); torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
        images = pair.verify_training_images(datasets); pair.verify_inputs(pins, proposed["sources"], [])
        args.output.mkdir(exist_ok=False); _created_output = args.output
        write_new(args.output / "declaration.json", declaration)
        frozen.joint.archive_sources(args.output, proposed["sources"])
        write_new(args.output / "started.json", dict(PID=os.getpid(), UTC=datetime.now(timezone.utc).isoformat(), GPU=gpu, images=images))
        teacher = freeze_teacher(parent, device); models = {}; fits = {}
        for arm in ARMS:
            models[arm], fits[arm] = fit_arm(args.output / arm, arm, parent, dataset, order, teacher, provenance, device, proposed["sources"])
            require(all(torch.equal(v.detach().cpu(), parent.state_dict()[k]) for k, v in teacher.state_dict().items())
                and all(p.grad is None and not p.requires_grad for p in teacher.parameters()),
                "all101 teacher states and zero teacher gradients must remain fixed")
        histories = [[json.loads(row) for row in (args.output / arm / "history.jsonl").read_text().splitlines()] for arm in ARMS]
        require(all(len(rows) == UPDATES and [r["optimizer_updates"] for r in rows] == list(range(1, UPDATES + 1)) for rows in histories)
            and all(a["indices"] == b["indices"] and a["sample_ids"] == b["sample_ids"] and a["batch_sha256"] == b["batch_sha256"]
                for a, b in zip(*histories)), "entire paired1000 input-byte and exposure journals must match")
        # HH_261004 - Only predeclared TRAIN resubstitution probes after BOTH endpoints, never feature approval.
        measurements = panel_measurements(parent, models, datasets, panels, provenance, device)
        pair.verify_inputs(pins, proposed["sources"], [args.output]); final_images = pair.verify_training_images(datasets)
        write_new(args.output / "summary.json", dict(schema=SCHEMA, status="COMPLETED_TRAIN_ONLY_PAIR_NOT_PROMOTED",
            fits=fits, measurements=measurements, final_images=final_images,
            original101_teacher_tensors_byte_fixed=True, teacher_parameter_gradients_absent=True,
            evaluation_only_payloads_opened=False, formal_main_approved=0, formal_sub_approved=0,
            model_promoted=False, simulator_control_authorized=False, vehicle_control_approved=False))
    return 0


def main(argv=None):
    global _created_output
    require(_created_output is None, "one non-reentrant pilot invocation required")
    args = parser().parse_args(argv)
    try:
        return run(args)
    except Exception as error:
        if _created_output is not None:
            write_new(_created_output / "failure_receipt.json", dict(schema=SCHEMA, exception_type=type(error).__name__,
                exception_message=str(error), all_partial_outputs_retained=True, model_promoted=False))
        raise
    finally:
        _created_output = None


if __name__ == "__main__":
    raise SystemExit(main())
