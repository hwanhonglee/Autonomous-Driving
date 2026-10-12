"""HH_261004 - Isolate geometry learning in the existing four-tensor route branch.

Fixed 1000 qualified TRAIN updates, never feature approval or deployment. The
other 97 state tensors remain GOOD6000 byte-exact at every update. This prevents
raw-speed/shared-feature drift but cannot repair the RIGHT raw-speed deficit.
Historical full 54 CONTROL is context, not a newly executed paired A/B.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

from scripts.e2e import audit_temporal_route_joint_train_gradients_v1_20261004 as audit

previous, frozen, pair, raw_parent = audit.previous, audit.frozen, audit.pair, audit.raw_parent
ROOT, require, digest, write_new, torch = audit.ROOT, audit.require, audit.digest, audit.write_new, audit.torch
np = frozen.np
SCHEMA = "portable_e2e.temporal_route_branch_geometry_pilot.20261004.v1"
STUDY = "temporal_route_branch_geometry_pilot_20261004_v1"
EXPORT_SCHEMA = SCHEMA + ".numeric_research_export"
GRADIENT_DECLARATION_SHA = "5b51918b3527440448b0b94311b2fd6f27956ac05b8fa009a86ac0c4837c2c32"
GRADIENT_SUMMARY_SHA = "870a0d3147609285c9e4eb98e9784a899e0cb0342f5790acafbc9b45bad081af"
BRANCH_SHAPES = {"route_recovery_branch.0.weight": [32, 9], "route_recovery_branch.0.bias": [32],
    "route_recovery_branch.2.weight": [33, 32], "route_recovery_branch.2.bias": [33]}
UPDATES, EXPOSURES, OLD_FACTOR, TURN_FACTOR = 1000, 5000, 1.05, .9
_created_output = None


def source_snapshot():
    old = audit.source_snapshot()
    paths = set(frozen.replay.imported_source_closure(ROOT,
        [Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(len(old) == 35 and len(result) == 36 and all(result.get(k) == v for k, v in old.items()),
        "all35 actual gradient audit dependencies plus one new trainer required")
    return result


def contract():
    # HH_261004 - These new engineering gates do not replace prospective closed-loop feature/safety criteria.
    return dict(schema=SCHEMA, study_id=STUDY, sources=source_snapshot(), parent=pair.PARENT,
        parent_history_sha256=frozen.PARENT_HISTORY_SHA, completed_pilot_declaration_sha256=audit.PILOT_DECLARATION_SHA,
        gradient_declaration_sha256=GRADIENT_DECLARATION_SHA, gradient_summary_sha256=GRADIENT_SUMMARY_SHA,
        optimizer_updates=UPDATES, exposures=EXPOSURES, effective_batch_size=5, seed=frozen.SEED,
        objective="exact original native B5 objective; no added retention coefficient",
        schedule="exact1000 prefix of original fixed20000 queues/LR; no new horizon",
        optimizer="new AdamW on four branch tensors only; original heads LR, weight_decay1e-4, clip_l2=5",
        trainable_shapes=BRANCH_SHAPES, trainable_tensor_count=4, trainable_parameter_count=1409,
        state_tensor_count=101, frozen_tensor_count=97, frozen97_bytes_every_update=True,
        training_mode="original train mode; eval+no_grad paired panel measurements",
        no_new_head=True, full_TRAIN_membership=6876, eligible_TRAIN_membership=6576,
        same_original30_input_pins=True, full_source_and_image_SHA_before_and_after=True,
        exact_old_panels=True, panel_count=160, anchors_per_domain=32, panel_is_TRAIN_resubstitution_only=True,
        raw_speed_byte_parity_count=160, decoded_instantaneous_speed_arc_parity_count=128,
        legacy_decoded_not_applicable_count=32, legacy_is_raw_only_no_extrapolation=True,
        chord_speed_may_change_with_geometry=True, actual_closed_loop_speed_not_guaranteed=True,
        old_gate=dict(domains=list(audit.DOMAINS[:3]), mean_and_peak_multiplier=OLD_FACTOR,
            metrics=["true_ADE_m", "true_FDE_m", "true_raw_curvature_huber"], unavailable_legacy_ADE_FDE="NOT_APPLICABLE",
            numerical_slack=0), turn_gate=dict(domains=list(audit.DOMAINS[3:]), mean_ADE_multiplier=TURN_FACTOR,
            mean_FDE_multiplier=1., numerical_slack=0),
        fixed_final_only=True, numeric_checkpoint_updates=[500, 1000], checkpoint_selection=False,
        historical_full54_control="context only, not new paired A/B", RIGHT_raw_speed_error_not_addressed=True,
        observation_based_time_context="previous full54 fixed1000 fit was11.06min; branch-only duration unmeasured",
        infrastructure_wall_limit_seconds=3600, automatic_retry=False, automatic_longer_training=False,
        physical_GPU_index=0, GPU_UUID=raw_parent.GPU_UUID, existing_private_GPU0_lease_required=True,
        CPU_actual_training_authorized=False, synthetic_CPU_unit_tests_allowed=True,
        evaluation_only_payloads_opened=False, independent_generalization_claimed=False,
        inference_loader_authorized=False, simulator_control_authorized=False, model_promoted=False,
        vehicle_control_approved=False, formal_main_approved=0, formal_sub_approved=0)


def freeze_branch(model):
    # HH_261004 - Start from the genuine current model; freeze all97 other states instead of constructing an auxiliary surrogate.
    state = model.state_dict()
    require(len(state) == 101 and {k: list(state[k].shape) for k in BRANCH_SHAPES} == BRANCH_SHAPES,
        "exact existing9-to32-to33 branch and101 state tensors required")
    for name, parameter in model.named_parameters(): parameter.requires_grad_(name in BRANCH_SHAPES)
    names = sorted(name for name, p in model.named_parameters() if p.requires_grad)
    require(names == sorted(BRANCH_SHAPES) and sum(p.numel() for p in model.parameters() if p.requires_grad) == 1409,
        "exact four branch tensors/1409 parameters required")
    return names


def frozen_digest(model):
    state = model.state_dict()
    require(len(state) == 101 and len(set(state) - set(BRANCH_SHAPES)) == 97, "complete97 frozen partition required")
    return audit.tensor_digest({k: v for k, v in state.items() if k not in BRANCH_SHAPES})


def check_partition(model, expected_frozen):
    require(sorted(k for k, p in model.named_parameters() if p.requires_grad) == sorted(BRANCH_SHAPES)
        and frozen_digest(model) == expected_frozen, "one of97 frozen states or four-tensor scope changed")
    require(all(p.grad is None for k, p in model.named_parameters() if k not in BRANCH_SHAPES), "frozen parameter gradient accumulated")


def read_authority(path, sha, proposed):
    value = frozen.pinned_read(path, sha)
    require(value.get("schema") == SCHEMA + ".authorization" and value.get("authorized") is True
        and value.get("contract") == proposed and type(value.get("physical_GPU_index")) is int
        and value["physical_GPU_index"] == 0 and value.get("model_promoted") is False
        and value.get("vehicle_control_approved") is False, "fresh exact branch-only TRAIN authority required")
    return value


def prepare(args, proposed):
    require(Path(sys.prefix).resolve() == frozen.original.WORKSPACE / "venvs/py312"
        and ROOT.resolve() == frozen.original.WORKSPACE / "source_worktrees" / STUDY,
        "exact fresh private source/existing personal venv required")
    required = ("parent_run", "warm_parent_weights", "warm_parent_metadata", "backbone_parent_weights", "backbone_parent_metadata",
        "recovery_train", "recovery_conversion", "left_train", "right_train", "left_conversion_sha256", "pilot_authority",
        "pilot_authority_sha256", "authorization", "authorization_sha256", "output")
    require(all(getattr(args, k) is not None for k in required), "explicit original TRAIN/model/new authority inputs required")
    expected = frozen.original.WORKSPACE / "runs/training" / STUDY
    require(args.output == expected and not args.output.exists()
        and not any(p.is_symlink() for p in (args.output, *args.output.parents)), "fresh exact private pilot output required")
    require(args.parent_history_sha256 == frozen.PARENT_HISTORY_SHA and args.pilot_authority_sha256 == audit.PILOT_AUTHORITY_SHA,
        "only authentic GOOD6000 and original TRAIN admission allowed")
    read_authority(args.authorization, args.authorization_sha256, proposed)
    pilot_path = frozen.original.WORKSPACE / "runs/training" / audit.pilot.STUDY / "declaration.json"
    gradient_path = frozen.original.WORKSPACE / "runs/diagnostics" / audit.STUDY
    require(args.completed_pilot_declaration == pilot_path and args.gradient_declaration == gradient_path / "declaration.json"
        and args.gradient_summary == gradient_path / "summary.json", "exact completed pilot and actual gradient evidence required")
    sealed = frozen.pinned_read(pilot_path, audit.PILOT_DECLARATION_SHA)
    gradient = frozen.pinned_read(args.gradient_declaration, GRADIENT_DECLARATION_SHA)
    measured = frozen.pinned_read(args.gradient_summary, GRADIENT_SUMMARY_SHA)
    require(sealed["parent"] == gradient["parent"] == pair.PARENT and sealed["source_sha256"] == audit.pilot.source_snapshot()
        and gradient["source_sha256"] == audit.source_snapshot() and len(sealed["immutable_input_sha256"]) == 30
        and len(gradient["immutable_input_sha256"]) == 32
        and all(gradient["immutable_input_sha256"].get(k) == v for k, v in sealed["immutable_input_sha256"].items()),
        "all original30 identities and35 unchanged gradient sources required")
    evidence = measured["measurements"]
    require(evidence.get("probe_count") == evidence.get("probe_denominator") == 160
        and evidence.get("total_autograd_grad_calls") == 1152 and evidence.get("optimizer_updates") == 0
        and evidence.get("original101_state_bytes_exact") is True and measured.get("MHA_fastpath_restored") is True
        and measured.get("model_promoted") is False and measured.get("evaluation_only_payloads_opened") is False,
        "completed unmodified actual160 TRAIN gradient audit required")
    original_pins = dict(sealed["immutable_input_sha256"])
    pins = dict(gradient["immutable_input_sha256"])
    for path, sha in pins.items():
        frozen.regular(path, frozen.original.PERSONAL); require(digest(path) == sha, "immutable lineage input changed: " + path)
    require(original_pins.get(str(args.pilot_authority)) == audit.PILOT_AUTHORITY_SHA, "original TRAIN authority path differs")
    torch.set_num_threads(4)
    training, audits, low, legacy, strata, receipts = pair.load_train(args)
    left, left_receipt, left_pins = frozen.load_LEFT(args); right, right_receipt, right_pins = frozen.load_RIGHT(args)
    datasets = (training, low, legacy, left, right); pair.disjoint(datasets)
    require(sum(map(len, datasets)) == 6876 and len(strata["eligible"]) == 2487, "complete qualified original TRAIN membership required")
    receipts = dict(receipts, LEFT=left_receipt, RIGHT=right_receipt)
    require(receipts == sealed["dataset_receipts"] and all(r.get("split") == "train" for r in receipts.values()), "TRAIN receipts changed")
    groups = frozen.previous.recovery_groups(training, audits)
    parent, checked, _ = audit.pilot.verify_parent(args, frozen.original.fixed_batches(groups, strata), datasets[:3])
    checked.update(left_pins); checked.update(right_pins)
    require(checked == original_pins, "all original30 reauthenticated model/admission inputs must match")
    order = frozen.fixed_batches(groups, strata)[:UPDATES]
    require(len(order) == UPDATES and frozen.json_sha(order) == sealed["fixed_schedule_sha256"]
        and [frozen.sample_ids(datasets, row) for row in order] == sealed["fixed_sample_ids"], "exact original1000 B5 prefix required")
    panels = [previous.uniform_panel(len(d), strata["eligible"] if i == 2 else None) for i, d in enumerate(datasets)]
    ids = audit.verify_panel_ids(datasets, panels, sealed["TRAIN_panels"])
    clone = pair.fresh_model(parent, torch.device("cpu")); names = freeze_branch(clone); frozen_sha = frozen_digest(clone)
    require(frozen_sha == frozen_digest(parent) and audit.state_digest(clone) == audit.state_digest(parent),
        "strict freeze must not alter any of the initial101 parent state bytes")
    del clone
    for path, sha in ((args.gradient_declaration, GRADIENT_DECLARATION_SHA), (args.gradient_summary, GRADIENT_SUMMARY_SHA),
            (args.authorization, args.authorization_sha256)):
        require(str(path) not in pins, "new evidence and authority inputs must be distinct"); pins[str(path)] = sha
    require(len(pins) == 35, "gradient32 plus declaration/summary/new branch authority required")
    declaration = dict(proposed, source_sha256=proposed["sources"], immutable_input_sha256=pins, dataset_receipts=receipts,
        TRAIN_panels=ids, trainable_state_names=names, frozen_state_names=sorted(set(parent.state_dict()) - set(names)),
        frozen97_sha256=frozen_sha, fixed_schedule_sha256=frozen.json_sha(order),
        fixed_sample_ids=sealed["fixed_sample_ids"], authorization_sha256=args.authorization_sha256)
    return parent, datasets, panels, order, pins, declaration


def panel_metrics(raw, decoded, batch):
    loss = frozen.original.temporal_trajectory_loss_v2(raw, batch)
    result = dict(true_raw_curvature_huber=float(loss["raw_curvature"]), true_raw_speed_huber=float(loss["raw_preferred_speed"]),
        observed_curvature_knots=int(batch["spatial_curvature_valid"].sum()), observed_speed_knots=int(batch["spatial_speed_valid"].sum()))
    if decoded is not None:
        mask = batch["target_valid"][0]; require(bool(mask.any()), "actual observed future required")
        errors = torch.linalg.vector_norm(decoded["xy"][0, 0] - batch["target_xy"][0], dim=-1)
        result.update(true_ADE_m=float(errors[mask].mean()), true_FDE_m=float(errors[mask][-1]),
            true_speed_MAE_mps=float((decoded["instantaneous_speed"][0, 1:] - batch["target_speed_mps"][0]).abs()[mask].mean()))
    return result


def parity(raw, reference, decoded, baseline):
    raw_equal = audit.tensor_digest({"preferred_speed_knots": raw["preferred_speed_knots"]}) == audit.tensor_digest(
        {"preferred_speed_knots": reference["preferred_speed_knots"]})
    require(raw_equal, "frozen97 raw preferred-speed bytes changed")
    if decoded is None:
        require(baseline is None, "legacy paired decoder availability differs")
        return dict(raw_speed_byte_equal=True, instantaneous_speed_byte_equal=None, arc_byte_equal=None,
            decoded_parity_reason="LEGACY_RAW_ONLY_NO_DECODER")
    require(baseline is not None, "paired genuine decoder required")
    same = {key: audit.tensor_digest({key: decoded[key]}) == audit.tensor_digest({key: baseline[key]})
        for key in ("instantaneous_speed", "arc_distance")}
    require(all(same.values()), "frozen97 GO instantaneous-speed/arc bytes changed")
    return dict(raw_speed_byte_equal=True, instantaneous_speed_byte_equal=same["instantaneous_speed"],
        arc_byte_equal=same["arc_distance"], decoded_parity_reason="PAIRED_GO_ACTUAL_INPUTS",
        chord_speed_not_an_invariance_claim=True)


def measure_panels(parent, model, datasets, panels, device):
    # HH_261004 - All32 anchors per domain, including first frames and failure-prone tails, are paired on exact actual bytes.
    teacher = previous.freeze_teacher(parent, device)
    model.to(device).eval()
    before = audit.state_digest(teacher)
    require(before == audit.state_digest(parent), "all101 reference states must be exact GOOD6000 before measurement")
    decoder = frozen.original.DeterministicPathTimeDecoderV2().to(device)
    output = {}
    with torch.no_grad():
        for domain, (dataset, indices) in enumerate(zip(datasets, panels)):
            rows = []
            for index in indices:
                cpu = frozen.default_collate([dataset[index]]); byte_sha = pair.batch_digest(cpu)
                require(cpu["sample_id"] == [dataset.examples[index].token], "original panel token differs")
                batch = raw_parent.move(cpu, device)
                if domain == 2:
                    actual = raw_parent.forward(model, batch); reference = raw_parent.forward(teacher, batch)
                    decoded = baseline = None
                else:
                    actual, decoded, _ = frozen.joint.joint_forward(model, decoder, batch)
                    reference, baseline, _ = frozen.joint.joint_forward(teacher, decoder, batch)
                row = dict(sample_id=cpu["sample_id"][0], index=index, split="TRAIN", batch_sha256=byte_sha,
                    parent=panel_metrics(reference, baseline, batch), candidate=panel_metrics(actual, decoded, batch),
                    parity=parity(actual, reference, decoded, baseline),
                    raw_output_sha256=audit.tensor_digest(actual), parent_output_sha256=audit.tensor_digest(reference))
                require(pair.batch_digest(cpu) == byte_sha, "collated panel bytes changed during paired measurement")
                rows.append(row)
            require(len(rows) == 32 and len({r["sample_id"] for r in rows}) == 32, "full fixed32 denominator required")
            output[str(domain)] = dict(domain=audit.DOMAINS[domain], denominator=32, rows=rows)
    require(audit.state_digest(teacher) == before and all(p.grad is None and not p.requires_grad for p in teacher.parameters()),
        "frozen GOOD6000 reference changed")
    return output


def gates(measurements):
    checks, raw_count, decoded_count, unavailable = [], 0, 0, 0
    for domain in range(5):
        data = measurements[str(domain)]; rows = data["rows"]
        require(data["denominator"] == len(rows) == 32 and len({r["sample_id"] for r in rows}) == 32, "complete fixed32 gate denominator required")
        for row in rows:
            require(row["split"] == "TRAIN" and row["parity"]["raw_speed_byte_equal"] is True, "actual full TRAIN/raw parity required")
            raw_count += 1
            if domain == 2:
                require(row["parity"]["instantaneous_speed_byte_equal"] is None and row["parity"]["arc_byte_equal"] is None
                    and row["parity"]["decoded_parity_reason"] == "LEGACY_RAW_ONLY_NO_DECODER", "legacy future cannot become invented parity")
                unavailable += 1
            else:
                require(row["parity"]["instantaneous_speed_byte_equal"] is True and row["parity"]["arc_byte_equal"] is True,
                    "genuine decoded speed/arc parity required"); decoded_count += 1
        metrics = ("true_raw_curvature_huber",) if domain == 2 else ("true_ADE_m", "true_FDE_m", "true_raw_curvature_huber")
        if domain < 3:
            for metric in metrics:
                for reduction in ("mean", "peak"):
                    values = [[float(row[arm][metric]) for row in rows] for arm in ("parent", "candidate")]
                    require(all(all(np.isfinite(v) and v >= 0 for v in series) for series in values), "finite nonnegative original metrics required")
                    prior, actual = [sum(v)/32 if reduction == "mean" else max(v) for v in values]
                    limit = prior * OLD_FACTOR
                    checks.append(dict(domain=domain, metric=metric, reduction=reduction, denominator=32,
                        parent=prior, candidate=actual, maximum=limit, passed=actual <= limit))
        else:
            for metric, factor in (("true_ADE_m", TURN_FACTOR), ("true_FDE_m", 1.)):
                values = [[float(row[arm][metric]) for row in rows] for arm in ("parent", "candidate")]
                require(all(all(np.isfinite(v) and v >= 0 for v in series) for series in values), "finite true turn metrics required")
                prior, actual = [sum(v)/32 for v in values]
                checks.append(dict(domain=domain, metric=metric, reduction="mean", denominator=32,
                    parent=prior, candidate=actual, maximum=prior * factor, passed=actual <= prior * factor))
    require((raw_count, decoded_count, unavailable) == (160, 128, 32) and len(checks) == 18, "all prospective gate checks required")
    return dict(status="TRAIN_ENGINEERING_GATES_MET_NOT_FEATURE_APPROVAL" if all(c["passed"] for c in checks)
        else "TRAIN_ENGINEERING_GATES_NOT_MET_NOT_PROMOTED", checks=checks, raw_speed_parity_count=raw_count,
        decoded_speed_arc_parity_count=decoded_count, legacy_decoded_unavailable_count=unavailable,
        numerical_slack=0, formal_main_approved=0, formal_sub_approved=0, model_promoted=False)


def export_numeric(output, model, update, declaration_sha, sources, frozen_sha):
    # HH_261004 - A full101 numeric checkpoint has a NEW4/97 identity; it is not authorized by the original54 loader.
    require(update in (500, 1000), "only declared numeric snapshots")
    check_partition(model, frozen_sha)
    state = model.state_dict(); require(all(v.dtype == torch.float32 and bool(torch.isfinite(v).all()) for v in state.values()), "finite float32 full101 export required")
    output.mkdir(parents=True, exist_ok=False)
    path = output / "model_state.npz"
    with path.open("xb") as stream: np.savez(stream, **{k: v.detach().cpu().numpy() for k, v in state.items()})
    metadata = dict(schema=EXPORT_SCHEMA, study_id=STUDY, architecture_schema=model.architecture_schema,
        model_id=model.model_id, model_config=model.config.to_dict(), recovery_config=model.recovery_config.to_dict(),
        optimizer_updates=update, exposures=5*update, fixed_final_update=UPDATES, parent=pair.PARENT,
        trainable_state_names=sorted(BRANCH_SHAPES), frozen_state_names=sorted(set(state) - set(BRANCH_SHAPES)),
        trainable_tensor_count=4, trainable_parameter_count=1409, frozen_tensor_count=97, frozen97_sha256=frozen_sha,
        state_tensor_shapes={k: list(v.shape) for k, v in state.items()}, state_tensor_dtypes={k: str(v.dtype).removeprefix("torch.") for k,v in state.items()},
        source_sha256=sources, declaration_sha256=declaration_sha, npz_sha256=digest(path),
        original54_loader_scope_compatible=False, inference_loader_authorized=False, simulator_control_authorized=False,
        model_promoted=False, vehicle_control_approved=False, resume_implemented=False)
    write_new(output / "metadata.json", metadata)
    with np.load(path, allow_pickle=False) as arrays:
        require(len(arrays.files) == len(set(arrays.files)) == 101 and set(arrays.files) == set(state), "numeric full101 archive differs")
        require(all(arrays[k].dtype == np.float32 and np.isfinite(arrays[k]).all()
            and arrays[k].tobytes() == v.detach().cpu().numpy().tobytes() for k,v in state.items()), "exported numeric bytes differ")
    return dict(update=update, model_state_sha256=metadata["npz_sha256"], metadata_sha256=digest(output / "metadata.json"), frozen97_sha256=frozen_sha)


def fit(output, parent, datasets, order, declaration, device):
    previous.reset_seed()
    parent_sha = audit.state_digest(parent)
    model = pair.fresh_model(parent, device).train(); freeze_branch(model)
    require(audit.state_digest(model) == parent_sha, "all101 fit-start state bytes must equal GOOD6000")
    frozen_sha = frozen_digest(parent); check_partition(model, frozen_sha)
    dataset = frozen.ConcatDataset(datasets)
    decoder = frozen.original.DeterministicPathTimeDecoderV2().to(device)
    optimizer = torch.optim.AdamW([dict(name="heads", params=[p for p in model.parameters() if p.requires_grad],
        lr=frozen.learning_rate(1, "heads"))], weight_decay=1e-4)
    started = time.monotonic(); snapshots = []
    with (output / "history.jsonl").open("x") as stream:
        for step, indices in enumerate(order, 1):
            require(time.monotonic() - started <= 3600, "fixed one-hour infrastructure budget exceeded")
            cpu = frozen.default_collate([dataset[i] for i in indices]); byte_sha = pair.batch_digest(cpu)
            require(cpu["sample_id"] == frozen.sample_ids(datasets, indices), "exact scheduled TRAIN identities required")
            batch = raw_parent.move(cpu, device); optimizer.zero_grad(set_to_none=True)
            loss, _, _, _ = frozen.domains(model, decoder, batch)
            require(bool(torch.isfinite(loss)), "finite unchanged native B5 objective required")
            loss.backward(); frozen.original.finite_clip_gradients(model)
            require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in model.parameters() if p.requires_grad), "four finite branch gradients required")
            check_partition(model, frozen_sha)
            optimizer.param_groups[0]["lr"] = frozen.learning_rate(step, "heads"); optimizer.step()
            check_partition(model, frozen_sha)
            require(pair.batch_digest(cpu) == byte_sha, "scheduled input bytes changed")
            row = dict(optimizer_updates=step, exposures=5*step, indices=indices, sample_ids=cpu["sample_id"], batch_sha256=byte_sha,
                original_B5_objective=float(loss.detach()), frozen97_sha256=frozen_sha, elapsed_seconds=time.monotonic() - started)
            audit.write_line(stream, row)
            if step % 100 == 0: print(json.dumps(dict(status="RUNNING_BRANCH_ONLY_TRAINING", **row)), flush=True)
            if step in (500, 1000):
                snapshots.append(export_numeric(output / "checkpoints" / f"step_{step:06d}", model, step,
                    digest(output / "declaration.json"), declaration["source_sha256"], frozen_sha))
    require(step == UPDATES and len(order) == UPDATES, "complete actual1000 update endpoint required")
    optimizer.zero_grad(set_to_none=True); check_partition(model, frozen_sha)
    require(audit.state_digest(parent) == parent_sha, "GOOD6000 CPU parent was modified during fit")
    return model, dict(optimizer_updates=UPDATES, exposures=EXPOSURES, elapsed_seconds=time.monotonic()-started,
        initial101_sha256=parent_sha, initial101_equal_to_GOOD6000=True, parent101_unmodified=True,
        frozen97_sha256=frozen_sha, checkpoints=snapshots, history_sha256=digest(output / "history.jsonl"))


def parser():
    value = previous.parser(); value.description = __doc__
    value.add_argument("--authorization", type=Path); value.add_argument("--authorization-sha256")
    value.add_argument("--completed-pilot-declaration", type=Path,
        default=frozen.original.WORKSPACE / "runs/training" / audit.pilot.STUDY / "declaration.json")
    base = frozen.original.WORKSPACE / "runs/diagnostics" / audit.STUDY
    value.add_argument("--gradient-declaration", type=Path, default=base / "declaration.json")
    value.add_argument("--gradient-summary", type=Path, default=base / "summary.json")
    return value


def run(args):
    global _created_output
    proposed = contract()
    if args.mode == "contract": print(json.dumps(proposed, indent=2)); return 0
    require(args.mode in ("validate-only", "fit") and not torch.cuda.is_initialized(), "private metadata before CUDA required")
    parent, datasets, panels, order, pins, declaration = prepare(args, proposed)
    if args.mode == "validate-only":
        require(not torch.cuda.is_initialized(), "CPU preflight cannot initialize CUDA")
        print(json.dumps(dict(status="PREPARED_BRANCH4_TRAIN_CPU_ONLY", declaration=declaration))); return 0
    frozen.replay.storage_reserve(args.output.parent)
    with raw_parent.gpu0_lease():
        gpu = frozen.gpu_revision.gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "unchanged strict deterministic CuBLAS required")
        torch.use_deterministic_algorithms(True); torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
        images = pair.verify_training_images(datasets); pair.verify_inputs(pins, proposed["sources"], [])
        args.output.mkdir(exist_ok=False); _created_output = args.output
        write_new(args.output / "declaration.json", declaration); frozen.joint.archive_sources(args.output, proposed["sources"])
        write_new(args.output / "started.json", dict(PID=os.getpid(), UTC=datetime.now(timezone.utc).isoformat(), GPU=gpu, images=images))
        device = torch.device("cuda:0")
        baseline = pair.fresh_model(parent, device).eval(); freeze_branch(baseline)
        before = measure_panels(parent, baseline, datasets, panels, device)
        write_new(args.output / "baseline_panels.json", before)
        check_partition(baseline, declaration["frozen97_sha256"]); del baseline
        model, fit_receipt = fit(args.output, parent, datasets, order, declaration, device)
        after = measure_panels(parent, model, datasets, panels, device)
        require(all(a["sample_id"] == b["sample_id"] and a["batch_sha256"] == b["batch_sha256"]
            and a["parent"] == b["parent"] for domain in before for a,b in zip(before[domain]["rows"], after[domain]["rows"])),
            "baseline/end exact same160 inputs and unchanged GOOD6000 metrics required")
        check_partition(model, declaration["frozen97_sha256"])
        write_new(args.output / "endpoint_panels.json", after)
        result = gates(after)
        history = [json.loads(row) for row in (args.output / "history.jsonl").read_text().splitlines()]
        require(len(history) == UPDATES and all(r["optimizer_updates"] == i+1 and r["exposures"] == 5*(i+1)
            and r["indices"] == order[i] and r["sample_ids"] == declaration["fixed_sample_ids"][i]
            and r["frozen97_sha256"] == declaration["frozen97_sha256"] for i,r in enumerate(history)), "all1000 immutable journals required")
        pair.verify_inputs(pins, proposed["sources"], [args.output]); final_images = pair.verify_training_images(datasets)
        write_new(args.output / "summary.json", dict(schema=SCHEMA, status="FIXED1000_BRANCH_GEOMETRY_COMPLETE_NOT_PROMOTED",
            fit=fit_receipt, TRAIN_engineering_gates=result, final_images=final_images, original97_byte_fixed=True,
            RIGHT_raw_speed_error_not_addressed=True, chord_speed_is_not_instantaneous_speed=True,
            inference_loader_authorized=False, simulator_control_authorized=False, model_promoted=False,
            vehicle_control_approved=False, evaluation_only_payloads_opened=False, formal_main_approved=0, formal_sub_approved=0))
    return 0


def main(argv=None):
    global _created_output
    require(_created_output is None, "one non-reentrant pilot invocation required")
    try: return run(parser().parse_args(argv))
    except BaseException as error:
        if _created_output is not None:
            write_new(_created_output / "failure_receipt.json", dict(schema=SCHEMA, exception_type=type(error).__name__,
                exception_message=str(error), all_partial_outputs_retained=True, model_promoted=False))
        raise
    finally: _created_output = None


if __name__ == "__main__": raise SystemExit(main())
