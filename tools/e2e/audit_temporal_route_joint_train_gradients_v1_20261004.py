"""HH_261004 - Diagnose GOOD6000 objective conflicts on sealed qualified TRAIN probes.

This is a gradient measurement, not training, a model export, feature approval,
or permission to open a new evaluation split. The student and neural retention
teacher both start at the identical GOOD6000 state. Retention is mathematically
zero at this point; any observed numerical residual is retained explicitly.
This audit cannot establish protection after an optimizer update. Synthetic
CPU unit tests exercise helpers only.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import sys
import time

from scripts.e2e import train_temporal_retention_pair_pilot_v2_20261004 as pilot

previous, frozen = pilot.previous, pilot.previous.frozen
pair, raw_parent, torch = previous.pair, previous.raw_parent, previous.torch
ROOT, require, digest, write_new = previous.ROOT, previous.require, previous.digest, previous.write_new
SCHEMA = "portable_e2e.temporal_route_joint_train_gradient_audit.20261004.v1"
STUDY = "temporal_route_joint_train_gradient_audit_20261004_v1"
PILOT_DECLARATION_SHA = "b838dbee70fe3c369107e5491f5a0cf66531bac71d0f50ebe992aacee5e83226"
PILOT_AUTHORITY_SHA = "9a54f2beb5ac3b02c1526d13fb13e2325cb4e3c5b87751e8eb94ff9fb73d4b0c"
DOMAINS = ("OLD_RECOVERY_REPLAY", "OLD_LOW_SPEED_REPLAY", "OLD_LEGACY_REPLAY", "LEFT_TRAIN", "RIGHT_TRAIN")
GROUP_PREFIXES = dict(fusion_norm=("fusion.", "output_norm."), trunk=("trunk.",),
    curvature_head=("curvature_head.",), speed_head=("speed_head.",), route_branch=("route_recovery_branch.",))
NATIVE = ("raw_curvature", "raw_preferred_speed", "temporal_xy", "temporal_speed", "preview_speed_regularizer", "domain_total")
PROTECTION = ("retention_curvature", "retention_preferred_speed", "retention_total")
TERMS = NATIVE + PROTECTION
_created_output = None


def source_snapshot():
    old = pilot.source_snapshot()
    paths = set(frozen.replay.imported_source_closure(ROOT,
        [Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(path.relative_to(ROOT)): digest(path) for path in sorted(paths)}
    require(len(old) == 34 and all(result.get(k) == v for k, v in old.items()),
        "all34 completed pilot sources must remain byte-exact")
    return result


def contract():
    # HH_261004 - Declare all 160 anchors and 1152 backward calls before inspecting gradients.
    return dict(schema=SCHEMA, study_id=STUDY, sources=source_snapshot(), parent=pair.PARENT,
        parent_history_sha256=frozen.PARENT_HISTORY_SHA,
        completed_pilot_declaration_sha256=PILOT_DECLARATION_SHA, original_immutable_input_count=30,
        panel_anchors_per_domain=32, total_probes=160, domains=list(DOMAINS),
        selection="exact sealed pilot uniform32 TRAIN panels; eligible legacy only",
        full_TRAIN_membership=6876, eligible_TRAIN_membership=6576,
        trainable_tensor_count=54, state_tensor_count=101, frozen_tensor_count=47,
        trainable_parameter_count=2277827, group_prefixes={k: list(v) for k, v in GROUP_PREFIXES.items()},
        measured_terms=list(TERMS), per_probe_named_tensor_and_group_norms_and_cosines=True,
        within_probe_all_applicable_term_pairs=True, cross_domain_same_panel_position_pairs=True,
        cross_domain_inputs_different_and_explicitly_identified=True, domain_mean_gradient_cosines=True,
        operation_budget=dict(student_forwards=161, frozen_teacher_forwards=161, decoder_forwards=128,
            gradient_calls_per_domain=[288, 288, 192, 192, 192], total_autograd_grad_calls=1152,
            optimizer_updates=0, runtime_estimate="measure actual progress; no unmeasured wall-time promise"),
        objective="unchanged original native raw/true-temporal/preview losses; legacy raw-only",
        domain_objective_coefficients=[.25, .25, .25, .125, .125],
        coefficients_scope="current B5 coefficients shown; uniform domain means are not its correlated fixed-batch aggregate",
        retention_teacher="identical frozen GOOD6000; mathematically zero initial residual, observed rounding retained",
        retention_limitation="zero initial gradient does not establish preservation after training",
        attention_backend="both audit forwards MHA fastpath disabled and restored; as-is first TRAIN residual recorded separately",
        matched_backend_diagnostic_not_original_training_runtime=True,
        masks_and_qualification_loaders_unchanged=True, inference_eval_mode_with_gradients=True,
        CPU_actual_audit_authorized=False, synthetic_CPU_unit_tests_allowed=True,
        physical_GPU_index=0, GPU_UUID=raw_parent.GPU_UUID, existing_private_GPU0_lease_required=True,
        source_and_all30_input_and_TRAIN_image_SHA_before_and_after=True,
        model_state_bytes_before_and_after=True, parameter_grad_accumulation=False,
        evaluation_only_payloads_opened=False, independent_generalization_claimed=False,
        model_promoted=False, simulator_control_authorized=False, vehicle_control_approved=False,
        formal_main_approved=0, formal_sub_approved=0)


def model_groups(model):
    # HH_261004 - Verify actual current ABI; never infer a trunk from an arbitrary name substring.
    state = model.state_dict()
    parameters = dict(model.named_parameters())
    names = tuple(sorted(name for name, value in parameters.items() if value.requires_grad))
    require(len(state) == 101 and set(state) - set(parameters) == {"image_grid_xy"} and len(names) == 54
        and list(names) == frozen.original.trainable_names(state)
        and sum(parameters[n].numel() for n in names) == 2277827, "exact current101/54 model ABI required")
    groups = {key: tuple(n for n in names if n.startswith(prefixes)) for key, prefixes in GROUP_PREFIXES.items()}
    flattened = [n for group in groups.values() for n in group]
    require(all(groups.values()) and len(flattened) == len(set(flattened)) == 54 and set(flattened) == set(names),
        "all54 names must belong to exactly one verified group")
    return names, dict(groups, all54=names)


def tensor_digest(values):
    result = hashlib.sha256()
    for name, tensor in sorted(values.items()):
        require(isinstance(tensor, torch.Tensor) and bool(torch.isfinite(tensor).all()), "finite tensor bytes required")
        value = tensor.detach().cpu().contiguous()
        result.update(json.dumps([name, str(value.dtype), list(value.shape)], separators=(",", ":")).encode())
        result.update(value.numpy().tobytes())
    return result.hexdigest()


def state_digest(model):
    return tensor_digest(model.state_dict())


def check_models(student, teacher, before):
    require(not student.training and not teacher.training, "both forwards must remain eval-mode")
    require(state_digest(student) == state_digest(teacher) == before, "all101 original model state bytes changed")
    require(all(p.grad is None for p in student.parameters())
        and all(p.grad is None and not p.requires_grad for p in teacher.parameters()),
        "autograd.grad must not accumulate parameter gradients or unfreeze teacher")
    model_groups(student)


def gradients(loss, names, parameters):
    require(isinstance(loss, torch.Tensor) and loss.numel() == 1 and bool(torch.isfinite(loss))
        and loss.requires_grad, "one finite graph-connected objective required")
    values = torch.autograd.grad(loss, parameters, retain_graph=True, allow_unused=True)
    result = {}
    for name, value in zip(names, values):
        require(value is None or bool(torch.isfinite(value).all()), "nonfinite gradient: " + name)
        # HH_261004 - CPU float64 statistics avoid finite-float32 aggregate norm overflow.
        result[name] = None if value is None else value.detach().to(device="cpu", dtype=torch.float64).contiguous()
    return result


def norm_record(values, names):
    present = [values[n] for n in names if values[n] is not None]
    nonzero = sum(bool(torch.count_nonzero(value)) for value in present)
    squared = sum(float(torch.sum(value * value)) for value in present)
    require(math.isfinite(squared) and squared >= 0, "finite float64 squared gradient norm required")
    return dict(status="ABSENT" if not present else ("EXACT_ZERO" if not nonzero else "FINITE_NONZERO"),
        norm=None if not present else math.sqrt(squared), tensor_count=len(names),
        present_tensor_count=len(present), absent_tensor_count=len(names) - len(present), nonzero_tensor_count=nonzero)


def norms(values, groups):
    return dict(named_tensors={n: norm_record(values, (n,)) for n in groups["all54"]},
        groups={key: norm_record(values, names) for key, names in groups.items()})


def cosine_record(left, right, names):
    a, b = norm_record(left, names), norm_record(right, names)
    reason = "DEFINED"
    if a["status"] == "ABSENT" or b["status"] == "ABSENT": reason = "ABSENT_GRADIENT"
    elif a["norm"] == 0 or b["norm"] == 0: reason = "ZERO_GRADIENT"
    if reason != "DEFINED": return dict(cosine=None, reason=reason, left=a, right=b)
    # HH_261004 - Missing components are mathematical zero in an otherwise supported group, but remain explicitly reported.
    dot = sum(float(torch.sum(left[n] * right[n])) for n in names if left[n] is not None and right[n] is not None)
    value = dot / (a["norm"] * b["norm"])
    require(math.isfinite(value) and -1.0000000001 <= value <= 1.0000000001, "finite bounded cosine required")
    return dict(cosine=max(-1., min(1., value)), reason=reason, left=a, right=b)


def cosines(left, right, groups):
    return dict(named_tensors={n: cosine_record(left, right, (n,)) for n in groups["all54"]},
        groups={key: cosine_record(left, right, names) for key, names in groups.items()})


def verify_panel_ids(datasets, panels, sealed):
    actual = {str(i): [d.examples[j].token for j in panels[i]] for i, d in enumerate(datasets)}
    require(len(actual) == 5 and all(len(v) == len(set(v)) == 32 for v in actual.values())
        and actual == sealed, "all160 exact original TRAIN panel IDs required")
    return actual


def read_authority(path, sha, proposed):
    authority = frozen.pinned_read(path, sha)
    require(authority.get("schema") == SCHEMA + ".authorization" and authority.get("authorized") is True
        and authority.get("contract") == proposed and type(authority.get("physical_GPU_index")) is int
        and authority["physical_GPU_index"] == 0 and authority.get("model_promoted") is False
        and authority.get("vehicle_control_approved") is False, "fresh exact TRAIN-gradient-only authority required")
    return authority


def prepare(args, proposed):
    require(Path(sys.prefix).resolve() == frozen.original.WORKSPACE / "venvs/py312"
        and ROOT.resolve().is_relative_to(frozen.original.WORKSPACE), "private source and existing personal venv only")
    required = ("parent_run", "warm_parent_weights", "warm_parent_metadata", "backbone_parent_weights",
        "backbone_parent_metadata", "recovery_train", "recovery_conversion", "left_train", "right_train",
        "left_conversion_sha256", "pilot_authority", "pilot_authority_sha256", "authorization", "authorization_sha256", "output")
    require(all(getattr(args, key) is not None for key in required), "explicit sealed TRAIN and fresh authority required")
    expected = frozen.original.WORKSPACE / "runs/diagnostics" / STUDY
    require(args.output == expected and not args.output.exists()
        and not any(p.is_symlink() for p in (args.output, *args.output.parents)), "fresh exact personal diagnostic output required")
    require(args.parent_history_sha256 == frozen.PARENT_HISTORY_SHA
        and args.pilot_authority_sha256 == PILOT_AUTHORITY_SHA, "fixed GOOD6000 and historical TRAIN binding required")
    old_path = frozen.original.WORKSPACE / "runs/training" / pilot.STUDY / "declaration.json"
    require(args.completed_pilot_declaration == old_path, "exact completed retention V2 declaration required")
    read_authority(args.authorization, args.authorization_sha256, proposed)
    sealed = frozen.pinned_read(old_path, PILOT_DECLARATION_SHA)
    require(sealed.get("schema") == pilot.SCHEMA and sealed.get("parent") == pair.PARENT
        and sealed.get("source_sha256") == pilot.source_snapshot()
        and sealed.get("authorization_sha256") == PILOT_AUTHORITY_SHA
        and len(sealed.get("immutable_input_sha256", {})) == 30, "complete original30 model/dataset/admission identities required")
    pins = dict(sealed["immutable_input_sha256"])
    for path, sha in pins.items():
        frozen.regular(path, frozen.original.PERSONAL)
        require(digest(path) == sha, "original pilot input changed: " + path)
    require(pins.get(str(args.pilot_authority)) == PILOT_AUTHORITY_SHA, "historical TRAIN authority path differs")
    torch.set_num_threads(4)
    training, audits, low, legacy, strata, receipts = pair.load_train(args)
    left, left_receipt, left_pins = frozen.load_LEFT(args)
    right, right_receipt, right_pins = frozen.load_RIGHT(args)
    datasets = (training, low, legacy, left, right)
    pair.disjoint(datasets)
    require(sum(map(len, datasets)) == 6876 and len(strata["eligible"]) == 2487, "full original qualified TRAIN membership required")
    receipts = dict(receipts, LEFT=left_receipt, RIGHT=right_receipt)
    require(receipts == sealed["dataset_receipts"]
        and all(receipt.get("split") == "train" for receipt in receipts.values()), "original TRAIN loader receipts differ")
    order = frozen.original.fixed_batches(frozen.previous.recovery_groups(training, audits), strata)
    parent, checked, _ = pilot.verify_parent(args, order, datasets[:3])
    checked.update(left_pins); checked.update(right_pins)
    require(checked == pins, "reauthenticated full30 inputs must exactly reproduce original completed pilot")
    panels = [previous.uniform_panel(len(d), strata["eligible"] if i == 2 else None) for i, d in enumerate(datasets)]
    panel_ids = verify_panel_ids(datasets, panels, sealed["TRAIN_panels"])
    provenance = previous.make_provenance(datasets[:3], receipts, PILOT_AUTHORITY_SHA, args.recovery_train, strata["eligible"])
    pins[str(old_path)] = PILOT_DECLARATION_SHA
    pins[str(args.authorization)] = args.authorization_sha256
    require(len(pins) == 32, "historical30 plus completed declaration and fresh authority must all be distinct")
    # HH_261004 - The authenticated numeric parent is frozen; only a disposable clone enables the original54 gradients.
    clone = pair.fresh_model(parent, torch.device("cpu")).eval()
    group_names = model_groups(clone)[1]
    del clone
    declaration = dict(proposed, source_sha256=proposed["sources"], immutable_input_sha256=pins,
        dataset_receipts=receipts, TRAIN_panels=panel_ids, authorization_sha256=args.authorization_sha256,
        group_names={key: list(value) for key, value in group_names.items()})
    return parent, datasets, panels, provenance, pins, declaration


def applicable_terms(model, teacher, decoder, cpu_batch, position, proof, device):
    # HH_261004 - Use identical actual collated inputs for every native/protection gradient in an anchor.
    batch = raw_parent.move(cpu_batch, device)
    for name in ("intent_valid", "stop_distance_valid", "initial_acceleration_valid"):
        require(name not in batch or not bool(batch[name].any()), "this fixed cruise audit must not silently add semantic training")
    if position == 2:
        raw = raw_parent.forward(model, batch)
        original = frozen.original.temporal_trajectory_loss_v2(raw, batch)
        total = .25 * original["raw_curvature"] + original["raw_preferred_speed"]
        terms = dict(raw_curvature=original["raw_curvature"], raw_preferred_speed=original["raw_preferred_speed"], domain_total=total)
        decoded = None
    else:
        raw, decoded, original = frozen.joint.joint_forward(model, decoder, batch)
        terms = {key: original["total" if key == "domain_total" else key] for key in NATIVE}
        reconstructed = (.25 * terms["raw_curvature"] + terms["raw_preferred_speed"] + terms["temporal_xy"]
            + terms["temporal_speed"] + .1 * terms["preview_speed_regularizer"])
        require(torch.allclose(reconstructed, terms["domain_total"], rtol=1e-6, atol=1e-7), "native loss decomposition differs")
    with torch.no_grad():
        reference = raw_parent.forward(teacher, batch)
    require(set(raw) == set(reference), "paired teacher/student output ABI differs")
    if position < 3:
        retained = previous.retention_term(raw, reference, batch, proof)
        terms.update(retention_curvature=retained["raw_curvature"], retention_preferred_speed=retained["raw_preferred_speed"],
            retention_total=retained["total"])
    not_applicable = {key: ("LEGACY_RAW_ONLY_NO_TEMPORAL_DECODING" if key in NATIVE else "NOT_AN_OLD_REPLAY_PROTECTION_ROLE")
        for key in TERMS if key not in terms}
    return batch, raw, reference, decoded, terms, not_applicable


@contextmanager
def matched_attention_math():
    # HH_261004 - Eval fastpath eligibility differs for trainable student and no-grad teacher; record and restore this diagnostic setting.
    backend = attention_backend()
    prior = backend.get_fastpath_enabled()
    backend.set_fastpath_enabled(False)
    try:
        yield prior
    finally:
        backend.set_fastpath_enabled(prior)


def attention_backend():
    backend = getattr(torch.backends, "mha", None)
    require(backend is not None and callable(getattr(backend, "get_fastpath_enabled", None))
        and callable(getattr(backend, "set_fastpath_enabled", None)), "explicit MHA fastpath control required for actual audit")
    return backend


def as_is_residual_observation(parent, dataset, index, proof, device):
    # HH_261004 - Preserve the native pre-audit discrepancy instead of silently relabeling matched-backend zero as the original residual.
    student = pair.fresh_model(parent, device).eval()
    teacher = previous.freeze_teacher(parent, device)
    before = state_digest(parent)
    cpu = frozen.default_collate([dataset[index]])
    batch = raw_parent.move(cpu, device)
    raw = raw_parent.forward(student, batch)
    with torch.no_grad(): reference = raw_parent.forward(teacher, batch)
    retained = previous.retention_term(raw, reference, batch, proof)
    record = dict(probe_id=cpu["sample_id"][0], split="TRAIN", original_index=index,
        batch_sha256=pair.batch_digest(cpu), student_eval_grad_enabled=True, teacher_eval_no_grad=True,
        MHA_fastpath_enabled=attention_backend().get_fastpath_enabled(), gradient_calls=0,
        raw_output_sha256=tensor_digest(raw), teacher_output_sha256=tensor_digest(reference),
        raw_outputs_bitwise_equal=all(torch.equal(raw[k].detach(), reference[k]) for k in raw),
        maximum_absolute_difference={key: float((raw[key].detach() - reference[key]).abs().max()) for key in raw},
        actual_as_is_retention_losses={key: float(retained[key].detach()) for key in ("raw_curvature", "raw_preferred_speed", "total")},
        distinction="as-is native pre-audit residual; matched-backend gradients below are a separate diagnostic")
    check_models(student, teacher, before)
    return record


def add_mean(store, domain, term, values):
    target = store.setdefault((domain, term), {})
    for name, value in values.items():
        if value is not None:
            if name not in target: target[name] = value.clone()
            else: target[name].add_(value)


def final_means(store, names):
    # HH_261004 - Denominator always includes the full32 declared probes, including zero/missing contributions.
    return {key: {name: None if name not in value else value[name] / 32 for name in names} for key, value in store.items()}


def cross_domain(rows, groups):
    records = []
    for (left_id, left), (right_id, right) in itertools.combinations(rows.items(), 2):
        for term in TERMS:
            if term not in left["gradients"] or term not in right["gradients"]:
                records.append(dict(left_domain=left_id, right_domain=right_id, term=term, cosine=None,
                    reason="TERM_NOT_APPLICABLE", left_probe_id=left["probe_id"], right_probe_id=right["probe_id"]))
            else:
                records.append(dict(left_domain=left_id, right_domain=right_id, term=term,
                    left_probe_id=left["probe_id"], right_probe_id=right["probe_id"],
                    left_batch_sha256=left["batch_sha256"], right_batch_sha256=right["batch_sha256"],
                    different_inputs=True, alignment=cosines(left["gradients"][term], right["gradients"][term], groups)))
    return records


def write_line(stream, value):
    stream.write(json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")) + "\n")
    stream.flush()


def audit(output, parent, datasets, panels, provenance, device):
    require(attention_backend().get_fastpath_enabled() is False, "both-forward matched attention diagnostic required")
    model = pair.fresh_model(parent, device).eval()
    teacher = previous.freeze_teacher(parent, device)
    before = state_digest(parent)
    names, groups = model_groups(model)
    parameters = tuple(dict(model.named_parameters())[name] for name in names)
    check_models(model, teacher, before)
    decoder = frozen.original.DeterministicPathTimeDecoderV2()
    means, seen, calls, started = {}, [], 0, time.monotonic()
    # HH_261004 - Exclusive append journals retain every declared probe and any incomplete failure tail.
    with (output / "probe_gradients.jsonl").open("x") as journal, (output / "paired_domain_gradients.jsonl").open("x") as paired:
        for panel_position in range(32):
            position_rows = {}
            for domain, dataset in enumerate(datasets):
                index = panels[domain][panel_position]
                cpu = frozen.default_collate([dataset[index]])
                token = dataset.examples[index].token
                require(cpu["sample_id"] == [token], "actual TRAIN sample ID differs from sealed membership")
                input_sha = pair.batch_digest(cpu)
                proof = None if domain >= 3 else provenance[domain][token]
                batch, raw, reference, decoded, terms, absent = applicable_terms(model, teacher, decoder, cpu, domain, proof, device)
                values = {key: gradients(loss, names, parameters) for key, loss in terms.items()}
                calls += len(values)
                for term, grads in values.items(): add_mean(means, domain, term, grads)
                row = dict(probe_id=token, domain=DOMAINS[domain], domain_index=domain, original_index=index,
                    panel_position=panel_position, split="TRAIN", batch_sha256=input_sha,
                    causal_input_sha256=pair.batch_digest({key: cpu[key] for key in raw_parent.FIELDS}),
                    original_mask_counts={key: int(batch[key].sum()) for key in
                        ("spatial_valid", "spatial_curvature_valid", "spatial_speed_valid", "target_valid")},
                    raw_output_sha256=tensor_digest(raw), teacher_raw_output_sha256=tensor_digest(reference),
                    paired_input_bytes_exact=True, identical_good6000_weights=True,
                    raw_outputs_bitwise_equal=all(torch.equal(raw[k].detach(), reference[k]) for k in raw),
                    maximum_absolute_teacher_difference={key: float((raw[key].detach() - reference[key]).abs().max()) for key in raw},
                    MHA_fastpath_enabled=False, matched_backend_diagnostic_not_original_training_runtime=True,
                    decoded_output_sha256=None if decoded is None else tensor_digest(decoded),
                    not_applicable_terms=absent,
                    terms={key: dict(loss=float(terms[key].detach()), gradient=norms(grads, groups)) for key, grads in values.items()},
                    within_probe_term_cosines={a + ":" + b: cosines(values[a], values[b], groups)
                        for a, b in itertools.combinations(values, 2)})
                write_line(journal, row)
                position_rows[domain] = dict(probe_id=token, batch_sha256=input_sha, gradients=values)
                seen.append(token)
                require(pair.batch_digest(cpu) == input_sha and all(p.grad is None for p in model.parameters()),
                    "original collated bytes and absent accumulated gradients must remain exact")
                del batch, raw, reference, decoded, terms, values, row
            write_line(paired, dict(panel_position=panel_position, pairs=cross_domain(position_rows, groups)))
            del position_rows
            print(json.dumps(dict(status="RUNNING_TRAIN_GRADIENT_DIAGNOSTIC_NOT_TRAINING", completed_probes=len(seen),
                total_probes=160, autograd_grad_calls=calls, total_calls=1152, elapsed_seconds=time.monotonic() - started)), flush=True)
    require(len(seen) == len(set(seen)) == 160 and calls == 1152, "all160 declared probes and1152 gradient calls required")
    averaged = final_means(means, names)
    mean_rows = {str(domain): {term: dict(probe_denominator=32, gradient=norms(value, groups))
        for (d, term), value in averaged.items() if d == domain} for domain in range(5)}
    mean_pairs = cross_domain({domain: dict(probe_id="MEAN_OF_ALL32_" + DOMAINS[domain], batch_sha256=None,
        gradients={term: value for (d, term), value in averaged.items() if d == domain}) for domain in range(5)}, groups)
    check_models(model, teacher, before)
    require(state_digest(parent) == before, "untouched CPU GOOD6000 parent bytes changed")
    return dict(status="COMPLETED_160_TRAIN_GRADIENT_PROBES_NOT_TRAINING_OR_APPROVAL", probe_ids=seen,
        probe_count=160, probe_denominator=160, total_autograd_grad_calls=calls, optimizer_updates=0,
        elapsed_seconds=time.monotonic() - started, model_state_sha256_before=before,
        model_state_sha256_after=state_digest(model), teacher_state_sha256_after=state_digest(teacher),
        original101_state_bytes_exact=True, teacher_frozen=True, accumulated_parameter_gradients_absent=True,
        group_names={key: list(value) for key, value in groups.items()}, domain_mean_gradients=mean_rows,
        domain_mean_pairs=mean_pairs, zero_retention_is_not_post_update_protection=True)


def parser():
    value = previous.parser()
    # HH_261004 - Reuse only argument spelling; the inherited fit path can never be dispatched.
    value.description = __doc__
    value._option_string_actions["--mode"].choices = ("contract", "validate-only", "run")
    value.add_argument("--completed-pilot-declaration", type=Path,
        default=frozen.original.WORKSPACE / "runs/training" / pilot.STUDY / "declaration.json")
    value.add_argument("--authorization", type=Path)
    value.add_argument("--authorization-sha256")
    return value


def run(args):
    global _created_output
    proposed = contract()
    if args.mode == "contract":
        print(json.dumps(proposed, indent=2)); return 0
    require(args.mode in ("validate-only", "run"), "no optimizer/training dispatch exists")
    require(not torch.cuda.is_initialized(), "metadata preflight must precede CUDA initialization")
    parent, datasets, panels, provenance, pins, declaration = prepare(args, proposed)
    if args.mode == "validate-only":
        require(not torch.cuda.is_initialized(), "CPU metadata preflight must not initialize CUDA")
        print(json.dumps(dict(status="PREPARED_TRAIN_GRADIENT_AUDIT_CPU_METADATA_ONLY", declaration=declaration))); return 0
    attention_backend()
    frozen.replay.storage_reserve(args.output.parent)
    with raw_parent.gpu0_lease():
        # HH_261004 - Reverify UUID, physical0 idleness and shared lease at the actual job boundary, never use GPU1.
        gpu = frozen.gpu_revision.gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "original strict deterministic CuBLAS required")
        torch.use_deterministic_algorithms(True)
        torch.backends.cudnn.benchmark = False; torch.backends.cudnn.deterministic = True
        torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
        images = pair.verify_training_images(datasets)
        pair.verify_inputs(pins, proposed["sources"], [])
        args.output.mkdir(exist_ok=False); _created_output = args.output
        write_new(args.output / "declaration.json", declaration)
        frozen.joint.archive_sources(args.output, proposed["sources"])
        write_new(args.output / "started.json", dict(PID=os.getpid(), UTC=datetime.now(timezone.utc).isoformat(), GPU=gpu, images=images))
        device = torch.device("cuda:0")
        first = datasets[0].examples[panels[0][0]].token
        native_residual = as_is_residual_observation(parent, datasets[0], panels[0][0], provenance[0][first], device)
        write_new(args.output / "native_as_is_residual.json", native_residual)
        with matched_attention_math() as prior_fastpath:
            measurements = audit(args.output, parent, datasets, panels, provenance, device)
        require(attention_backend().get_fastpath_enabled() == prior_fastpath, "prior attention fastpath flag must be restored")
        pair.verify_inputs(pins, proposed["sources"], [args.output])
        images_after = pair.verify_training_images(datasets)
        write_new(args.output / "summary.json", dict(schema=SCHEMA, measurements=measurements,
            as_is_native_residual=native_residual, MHA_prior_fastpath_enabled=prior_fastpath, MHA_fastpath_restored=True,
            matched_backend_diagnostic_not_original_training_runtime=True,
            images_before=images, images_after=images_after, immutable_input_count=len(pins),
            evaluation_only_payloads_opened=False, optimizer_updates=0, model_promoted=False,
            simulator_control_authorized=False, vehicle_control_approved=False, formal_main_approved=0, formal_sub_approved=0,
            journal_sha256={name: digest(args.output / name) for name in ("probe_gradients.jsonl", "paired_domain_gradients.jsonl")}))
    return 0


def main(argv=None):
    global _created_output
    require(_created_output is None, "one non-reentrant gradient audit required")
    try:
        return run(parser().parse_args(argv))
    except BaseException as error:
        if _created_output is not None:
            write_new(_created_output / "failure_receipt.json", dict(schema=SCHEMA, exception_type=type(error).__name__,
                exception_message=str(error), all_partial_outputs_retained=True, optimizer_updates=0, model_promoted=False))
        raise
    finally:
        _created_output = None


if __name__ == "__main__":
    raise SystemExit(main())
