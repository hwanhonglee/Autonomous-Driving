"""HH_261002 - Fresh GOOD6000, fixed20k B5 LEFT347/RIGHT697 research fit, never automatic execution.

Reuse the frozen losses, three protection queues, 101-tensor architecture and
54/47 trainable/frozen ABI. Keep all previous74 screens and their pinned numeric
thresholds; three RIGHT TRAIN resubstitution screens are not feature approval.
"""
from __future__ import annotations

import ast
import copy
from pathlib import Path
from types import FunctionType

if __name__ == "__mp_main__":
    # HH_261002 - Spawn readers configure only canonical imports, never CUDA or a training invocation.
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_left_right_joint_worker", _entry)
    _module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

from scripts.e2e import train_temporal_left_joint_20261002 as baseline

BASELINE_SHA = "dc8963d4bd4941911b600f1409827f0961977ae9af75c7c146789cdba020aa3a"
for _name in ("pair", "original", "replay", "previous", "joint", "raw_parent", "gpu_revision", "require", "digest",
    "write_new", "read", "regular", "json_sha", "torch", "np", "json", "math", "os", "sys", "time", "random",
    "argparse", "Counter", "datetime", "timezone", "ConcatDataset", "DataLoader", "default_collate", "MAX_UPDATES",
    "CHECKPOINT_INTERVAL", "MAX_FIT_SECONDS", "SEED", "PARENT_HISTORY_SHA", "LEFT_BASENAME", "LEFT_CONVERSION_SCHEMA",
    "LEFT_QUALIFICATION_SCHEMA", "LEFT_ADAPTER", "LEFT_SCOPE"):
    globals()[_name] = getattr(baseline, _name)
ROOT = Path(__file__).resolve().parents[2]
STUDY_ID = "temporal_left_right_joint_20261002_v1"
SCHEMA = "portable_e2e.temporal_actual_left_right_joint.20261002.v1"
EXPORT_SCHEMA = SCHEMA + ".numeric_export_research"
AUTH_SCHEMA = SCHEMA + ".prospective_authorization"
RIGHT_BASENAME = "town01-low-speed-right80-train-20261002-v1"
RIGHT_CONVERSION_SCHEMA = "carla.actual_town01_right80_common10_conversion.20261002.v1"
RIGHT_QUALIFICATION_SCHEMA = "carla.actual_town01_right80_common10_qualification.20261002.v1"
RIGHT_ADAPTER = "carla_actual_town01_right697_to_common10_20261002_v1"
RIGHT_CONVERSION_SHA = "2e6ea6df02f55b1ebf7a9d0463214882161a2e2942e5570a7c702d6159e6b8d1"
RIGHT_MANIFEST_SHA = "b9a0fa1cb2b0dd61265f0074e7461db5259a975a32fc9ac4322c50339ec6c259"
RIGHT_VALIDATION_SHA = "f2e31a082a74800464dc3f5392ce498af8130a081315f7c241da2b4f012da007"
RIGHT_QUALIFICATION_SHA = "5c625e4407c2d50e449bc97c20a6063506a2713dea8ebcfb08f770aa930d3c67"
RIGHT_LOADER_SHA = "a91d2424afaafb5645cc17d24f22beb6cbfd1073dc5e1ccf2a38a736a5a6831d"
RIGHT_TEMPORAL_SHA = "a50fe177c21a2b865c0dabed9fe2717c9fd7e26e827673a3b48c5e9365e3d67a"
ORIGINAL_SCREENS_SHA = "53e513743e4930dd38283f66ce4a476432e5beb57fdc87c5e88089ff07820328"
ORIGINAL_SCREENS_PATH = original.WORKSPACE / "runs/training/temporal_left_joint_20261002_v1/summary.json"
RIGHT_SCOPE = dict(rolling_RIGHT_only=True, startup_anchors_used_for_training=False,
    STOP_or_stationary_tail_supervision=False, raw_startup_and_tail_retained_as_context_only=True,
    fixed_driving_relative_anchor_offsets=[80, 1472], fixed_anchor_driving_seconds=[4., 73.6],
    new_independent_site_or_sealed_TEST_claimed=False, original_V1_failed_capture_not_qualified=True,
    full_generated_navigation_route_retained=True, capture_container_is_declared_not_measured_geometry=True,
    original32_and_failed_generated_p00_not_requalified=True, failed_RIGHT70_attempts_not_requalified=True,
    whole_nominal_clearance_is_not_actual_training_approval=True)
BATCH_DOMAINS = [*baseline.BATCH_DOMAINS, "actual_RIGHT697_joint"]
_created_output = None


def verify_frozen():
    require(digest(baseline.__file__) == BASELINE_SHA, "frozen LEFT20k trainer changed")


def clone(name):
    old = getattr(baseline, name)
    value = FunctionType(old.__code__, globals(), name, old.__defaults__, old.__closure__)
    value.__kwdefaults__ = old.__kwdefaults__; return value


for _name in ("is_sha", "left_scope_clear", "verify_LEFT_conversion", "load_LEFT", "learning_rate_config",
    "learning_rate", "sample_ids", "main"):
    globals()[_name] = clone(_name)


def revised_function_ast(name):
    # HH_261002 - Inverse AST proves only RIGHT identity/census or B5 accounting differs; frozen numerical guards stay intact.
    verify_frozen(); tree = ast.parse(Path(baseline.__file__).read_bytes())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    node = copy.deepcopy(old); changes = []
    def change(target, key, value):
        changes.append((target, key, getattr(target, key), value)); setattr(target, key, value)
    right = name in ("left_scope_clear", "verify_LEFT_conversion", "load_LEFT")
    names = {"left_scope_clear": "right_scope_clear", "verify_LEFT_conversion": "verify_RIGHT_conversion",
        "load_LEFT": "load_RIGHT", "LEFT_SCOPE": "RIGHT_SCOPE", "LEFT_BASENAME": "RIGHT_BASENAME",
        "LEFT_CONVERSION_SCHEMA": "RIGHT_CONVERSION_SCHEMA", "LEFT_QUALIFICATION_SCHEMA": "RIGHT_QUALIFICATION_SCHEMA",
        "LEFT_ADAPTER": "RIGHT_ADAPTER"}
    counts = {347: 697, 22208: 44608, 339: 689, 338: 688} if right else {4: 5, 80000: 100000}
    for part in ast.walk(node):
        if right and isinstance(part, ast.Name) and part.id in names: change(part, "id", names[part.id])
        elif right and isinstance(part, ast.FunctionDef) and part.name in names: change(part, "name", names[part.name])
        elif right and isinstance(part, ast.Attribute) and part.attr in ("left_train", "left_conversion_sha256"):
            change(part, "attr", part.attr.replace("left", "right"))
        elif isinstance(part, ast.Constant) and type(part.value) is int and part.value in counts:
            change(part, "value", counts[part.value])
        elif isinstance(part, ast.Constant) and type(part.value) is str:
            before = part.value
            after = before.replace("LEFT347", "RIGHT697").replace("LEFT", "RIGHT").replace("full347", "full697") if right else before
            if name == "verify_final" and before == "LEFT_examples_seen": after = before
            if name == "train" and before == "ACTUAL_LEFT_JOINT_CONTINUATION": after = "ACTUAL_LEFT_RIGHT_JOINT_CONTINUATION"
            if name in ("save_checkpoint", "verify_final"): after = after.replace("80k", "100k")
            if before != after: change(part, "value", after)
    if name in ("save_checkpoint", "train"):
        marker = "maximum_optimizer_updates" if name == "save_checkpoint" else "batch_sha256"
        targets = [n for n in ast.walk(node) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == "dict" and marker in {k.arg for k in n.keywords}]
        require(len(targets) == 1, "one exact checkpoint/update accounting dictionary required")
        extra = ast.keyword(arg="RIGHT_examples_seen", value=ast.Name(id="step", ctx=ast.Load())) if name == "save_checkpoint" \
            else ast.keyword(arg="RIGHT", value=ast.parse("joint.loss_numbers(terms[3])", mode="eval").body)
        change(targets[0], "keywords", [*targets[0].keywords, extra])
    if name == "verify_final":
        tuples = [n for n in ast.walk(node) if isinstance(n, ast.Tuple)
            and [v.value for v in n.elts if isinstance(v, ast.Constant)] ==
                ["recovery_examples_seen", "v4_examples_seen", "legacy_examples_seen", "LEFT_examples_seen"]]
        require(len(tuples) == 1, "one exact four-domain endpoint census required")
        change(tuples[0], "elts", [*tuples[0].elts, ast.Constant(value="RIGHT_examples_seen")])
    require(changes, "nonempty explicit domain/census revision required")
    for target, key, before, after in reversed(changes): setattr(target, key, before)
    require(ast.dump(node) == ast.dump(old), "another frozen numerical/source/denial statement changed")
    for target, key, before, after in changes: setattr(target, key, after)
    return node


for _name in ("left_scope_clear", "verify_LEFT_conversion", "load_LEFT", "save_checkpoint", "train", "verify_final"):
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised_function_ast(_name)], type_ignores=[])), __file__, "exec"), globals())
_right_verify = verify_RIGHT_conversion
_right_load = load_RIGHT


def verify_RIGHT_conversion(value):
    case = _right_verify(value)
    require(value["dataset_manifest_sha256"] == RIGHT_MANIFEST_SHA and value["validation_report_sha256"] == RIGHT_VALIDATION_SHA
        and value["qualification_receipt_sha256"] == RIGHT_QUALIFICATION_SHA
        and value["train_loader_fingerprint_sha256"] == RIGHT_LOADER_SHA and value["temporal_dataset_fingerprint_sha256"] == RIGHT_TEMPORAL_SHA,
        "only externally closed actual RIGHT80 receipts and complete loader fingerprints required")
    return case


def load_RIGHT(args):
    # HH_261002 - A direct loader call also requires the exact externally closed conversion, not an arbitrary valid-looking receipt.
    require(args.right_conversion_sha256 == RIGHT_CONVERSION_SHA, "only actual complete RIGHT80 conversion allowed")
    return _right_load(args)


def source_snapshot():
    verify_frozen(); old = baseline.source_snapshot()
    paths = set(replay.imported_source_closure(ROOT, [Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(len(old) == 30 and len(result) == 31 and all(result.get(k) == v for k, v in old.items()),
        "one fresh trainer plus all thirty frozen LEFT20k dependencies required")
    return result


def contract(left_conversion_sha256):
    old = clone("contract")(left_conversion_sha256)
    return dict(old, study_id=STUDY_ID, output=str(original.WORKSPACE / "runs/training" / STUDY_ID),
        effective_batch_size=5, examples_seen=100000, training_episode_count=20, original_TRAIN_anchor_count=6876,
        eligible_TRAIN_anchor_count=6576, exposures=dict(old["exposures"], RIGHT=20000),
        new_RIGHT_count=697, new_RIGHT_full_cycles=28, new_RIGHT_cycle_remainder=484,
        RIGHT_sampler="all697 full-cycle shuffled seed+900; no quality filtering or failed70 reuse",
        new_RIGHT_basename=RIGHT_BASENAME, new_RIGHT_conversion_sha256=RIGHT_CONVERSION_SHA,
        new_RIGHT_manifest_sha256=RIGHT_MANIFEST_SHA, new_RIGHT_validation_sha256=RIGHT_VALIDATION_SHA,
        new_RIGHT_qualification_sha256=RIGHT_QUALIFICATION_SHA, new_RIGHT_loader_sha256=RIGHT_LOADER_SHA,
        new_RIGHT_temporal_sha256=RIGHT_TEMPORAL_SHA,
        objective=".25*(joint_recovery+joint_oldlow+.25*raw_legacy_curvature+raw_legacy_speed+.5*joint_LEFT+.5*joint_RIGHT)",
        protected_domain_coefficients_unchanged=True, original_three_protection_queues_byte_identical=True,
        retained_original_diagnostic_screens=74, original_screen_summary_path=str(ORIGINAL_SCREENS_PATH),
        original_screen_summary_sha256=ORIGINAL_SCREENS_SHA, original_failed_screen_count_retained=14,
        original_GOOD6000_thresholds_must_not_be_rebased=True, total_diagnostic_screen_count=77,
        RIGHT_TRAIN_screens=dict(ADE_m=.1, FDE_m=.25, speed_MAE_mps=.05),
        new_RIGHT_independent_validation=False, same_101_tensor_architecture=True, new_head=False,
        intervention_scope="DATA+schedule+balanced turn objective coupled; NOT isolated DATA or loss causality",
        fixed_final_only=True, no_checkpoint_selection=True, detached_launcher_separately_required=True)


def pinned_read(path, expected):
    # HH_261002 - Authenticate and parse the same first buffer, including duplicate-key rejection.
    regular(path, original.PERSONAL); payload = Path(path).read_bytes()
    require(is_sha(expected) and pair.hashlib.sha256(payload).hexdigest() == expected, "external first-read SHA differs")
    return json.loads(payload, object_pairs_hook=pair.unique_json_object)


def validate_authorization(path, sha, proposed):
    value = pinned_read(path, sha)
    require(value.get("schema") == AUTH_SCHEMA and value.get("authorized") is True
        and json.dumps(value.get("contract"), sort_keys=True, allow_nan=False) == json.dumps(proposed, sort_keys=True, allow_nan=False)
        and value.get("contract_sha256") == json_sha(proposed) and value.get("actual_parent_history_sha256") == PARENT_HISTORY_SHA
        and type(value.get("GPU_physical_index")) is int and value["GPU_physical_index"] == 0
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "exact new source/data/B5/schedule/GOOD6000 GPU0 research authority required")
    return value


def fixed_batches(groups, strata):
    # HH_261002 - Keep every original four-column queue exactly; independently append the complete RIGHT697 queue.
    order = baseline.fixed_batches(groups, strata); right = replay.CycleQueue(range(697), SEED + 900)
    require(len(order) == MAX_UPDATES and all(len(row) == 4 for row in order), "unchanged full original four-domain schedule required")
    return [row + [6179 + right.next()] for row in order]


def domains(model, decoder, batch):
    terms = [joint.joint_forward(model, decoder, replay.slice_batch(batch, p, p + 1))[2] for p in (0, 3, 1, 4)]
    old = original.temporal_trajectory_loss_v2(raw_parent.forward(model, replay.slice_batch(batch, 2, 3)), replay.slice_batch(batch, 2, 3))
    loss = original.exact_objective(terms[0]["total"], .5 * (terms[1]["total"] + terms[3]["total"]),
        terms[2]["total"], old["raw_curvature"], old["raw_preferred_speed"])
    return loss, terms, old, (terms[0]["total"], terms[2]["total"], .25 * old["raw_curvature"] + old["raw_preferred_speed"],
        terms[1]["total"], terms[3]["total"])


def preflight_gradient(parent, dataset, order, device):
    value = clone("preflight_gradient")(parent, dataset, order, device)
    require(list(value["per_domain_gradients"]) == BATCH_DOMAINS, "all five actual TRAIN domains must have finite gradients")
    return dict(value, status="PASS_ALL5_ACTUAL_TRAIN_STRICT_BACKWARD_NO_UPDATE")


def evaluate(model, device, training, audits, low, legacy, strata, left, right, holdout, ho_audits, known):
    value = baseline.evaluate(model, device, training, audits, low, legacy, strata, left, holdout, ho_audits, known)
    decoder = original.DeterministicPathTimeDecoderV2().to(device); model.to(device)
    value["RIGHT"] = previous.evaluate_joint(model, decoder, right, device); model.cpu(); return value


def endpoint_screens(before, after, reference):
    # HH_261002 - Preserve the full LEFT denominator as well as all original thresholds before appending RIGHT diagnostics.
    require(len(before["LEFT"]["rows"]) == len(after["LEFT"]["rows"]) == 347
        and len({r["token"] for r in after["LEFT"]["rows"]}) == 347, "all347 unique LEFT anchors retained")
    old = baseline.endpoint_screens(before, after); recorded = reference["endpoint_diagnostic_screens"]["checks"]
    require(len(recorded) == len(old["checks"]) == 74 and len({r["name"] for r in recorded}) == 74
        and sum(not r["passed"] for r in recorded) == 14, "entire previous74 including fourteen failures must remain")
    require([(r["name"], r["threshold"]) for r in old["checks"]] == [(r["name"], r["threshold"]) for r in recorded],
        "original GOOD6000 numeric thresholds changed or were rebased")
    require(before["RIGHT"]["full_fixed_denominator"] == after["RIGHT"]["full_fixed_denominator"] == 697
        and len(before["RIGHT"]["rows"]) == len(after["RIGHT"]["rows"]) == 697
        and [r["token"] for r in before["RIGHT"]["rows"]] == [r["token"] for r in after["RIGHT"]["rows"]]
        and len({r["token"] for r in after["RIGHT"]["rows"]}) == 697, "all697 identical current/final RIGHT anchors required")
    checks = list(old["checks"])
    for name, limit in (("mean_ade_6p4s_m", .1), ("mean_fde_6p4s_m", .25), ("mean_instantaneous_speed_mae_mps", .05)):
        actual = after["RIGHT"][name]
        checks.append(dict(name="RIGHT_TRAIN_resubstitution/" + name, actual=actual, threshold=limit,
            passed=type(actual) in (float, int) and math.isfinite(actual) and actual <= limit))
    return dict(old, checks=checks, diagnostic_screen_count=77, retained_original_joint_screen_count=71,
        retained_original_LEFT_screen_count=3, retained_previous74_numeric_thresholds=True,
        retained_previous74_results=copy.deepcopy(recorded), original_failed14_results=[copy.deepcopy(r) for r in recorded if not r["passed"]],
        original_failed14_current_results=[copy.deepcopy(r) for r in checks if r["name"] in {v["name"] for v in recorded if not v["passed"]}],
        new_RIGHT_independent_validation=False, all_diagnostic_screens_passed=all(r["passed"] for r in checks))


def parser():
    value = baseline.parser(); value.description = __doc__
    value.add_argument("--right-train", type=Path)
    value.add_argument("--right-conversion-sha256", default=RIGHT_CONVERSION_SHA)
    return value


def run(argv=None):
    # HH_261002 - Explicit reviewed invocation only; final101 and100k journal verification still precede every DEV/HOLDOUT load.
    global _created_output
    args = parser().parse_args(argv); proposed = contract(args.left_conversion_sha256)
    require(args.right_conversion_sha256 == RIGHT_CONVERSION_SHA, "only full actual closed RIGHT80 conversion allowed")
    if args.mode == "contract": print(json.dumps(proposed, indent=2, allow_nan=False)); return 0
    require(Path(sys.prefix).resolve() == (original.WORKSPACE / "venvs/py312").resolve(), "existing private py312 only; no install")
    require(all(getattr(args, n) is not None for n in ("parent_run", "warm_parent_weights", "warm_parent_metadata", "backbone_parent_weights",
        "backbone_parent_metadata", "recovery_train", "recovery_conversion", "recovery_holdout", "holdout_conversion", "left_train", "right_train", "output", "authorization")),
        "explicit canonical assets and external authorization required")
    require(args.parent_history_sha256 == PARENT_HISTORY_SHA, "original GOOD6000 journal required; never LEFT20k or failed intrinsic parent")
    require(not args.output.exists() and ".." not in args.output.parts and not any(p.is_symlink() for p in (args.output, *args.output.parents))
        and args.output.resolve() == original.WORKSPACE / "runs/training" / STUDY_ID, "fresh exact new private output only")
    regular(args.authorization, original.PERSONAL); validate_authorization(args.authorization, args.authorization_sha256, proposed)
    replay.storage_reserve(args.output.parent); torch.set_num_threads(4)
    training, audits, low, legacy, strata, receipts = pair.load_train(args)
    left, left_receipt, left_pins = load_LEFT(args); right, right_receipt, right_pins = load_RIGHT(args)
    datasets = (training, low, legacy, left, right); pair.disjoint(datasets); groups = previous.recovery_groups(training, audits)
    parent_order = original.fixed_batches(groups, strata)
    parent, pins, parent_screens = pair.verify_parent(args, parent_order, (training, low, legacy))
    order = fixed_batches(groups, strata); receipts = dict(receipts, LEFT=left_receipt, RIGHT=right_receipt); dataset = ConcatDataset(datasets)
    require(len(dataset) == 6876 and len({r.episode_id for d in datasets for r in d.examples}) == 20, "all20 TRAIN records and6876 original anchors retained")
    pins.update(left_pins); pins.update(right_pins)
    for path in (args.authorization, args.recovery_conversion, args.recovery_train / "dataset.json", replay.LOW_PATH / "dataset.json",
        replay.LOW_PATH / "moving_conversion_qualification.json", replay.LEGACY_PATH / "dataset.json"):
        regular(path, original.PERSONAL); pins[str(path)] = digest(path)
    fields = dict(original.scope_fields(), training_episode_count=20, replay_training_episode_count=11,
        LEFT_training_episode_count=1, RIGHT_training_episode_count=1, current_training_purpose="actual_LEFT347_RIGHT697_balanced_joint_research",
        parent_diagnostic_pass_count=sum(c["passed"] for c in parent_screens["checks"]), parent_diagnostic_screen_count=71,
        parent_all_diagnostic_screens_passed=parent_screens["all_diagnostic_screens_passed"])
    declaration = dict(fields, schema=SCHEMA, protocol=proposed, parent_optimizer_updates=6000, parent_known_total_optimizer_updates=12200,
        parent_npz_sha256=pair.PARENT["weights_sha256"], parent_metadata_sha256=pair.PARENT["metadata_sha256"], parent_checkpoint_sha256=pair.PARENT["checkpoint_sha256"],
        parent_declaration_sha256=pair.PARENT["declaration_sha256"], parent_history_sha256=PARENT_HISTORY_SHA,
        model_id=original.MODEL_ID, architecture_schema=original.ARCHITECTURE_SCHEMA, model_config=parent.config.to_dict(), recovery_config=parent.recovery_config.to_dict(),
        source_sha256=proposed["sources"], cohorts=receipts, training_dataset_fingerprint_sha256=json_sha(receipts),
        deferred_holdout_conversion_sha256=original.HOLDOUT_RECEIPT_SHA, holdout_payload_loaded=False,
        trainable_state_names=original.freeze_inputs(parent), frozen_state_names=sorted(set(parent.state_dict()) - set(original.trainable_names(parent.state_dict()))),
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, trainable_parameter_count=2277827,
        effective_batch_size=5, seed=SEED, learning_rate=learning_rate_config(), batch_domains=BATCH_DOMAINS,
        fixed_schedule_sha256=json_sha(order), fixed_sample_ids=[sample_ids(datasets, row) for row in order],
        immutable_input_sha256=pins, authorization_sha256=args.authorization_sha256)
    declaration["fixed_sample_ids_sha256"] = json_sha(declaration["fixed_sample_ids"])
    if args.mode == "validate-only":
        require(not torch.cuda.is_initialized(), "CPU validation must not initialize CUDA")
        print(json.dumps(dict(status="VALIDATED_ALL20_TRAIN_CPU_NO_CUDA_NO_OUTPUT", **declaration), allow_nan=False)); return 0
    with raw_parent.gpu0_lease():
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "strict CuBLAS workspace required")
        gpu = gpu_revision.gpu_preflight(); device = torch.device("cuda:0")
        torch.use_deterministic_algorithms(True); torch.backends.cudnn.deterministic = True; torch.backends.cudnn.benchmark = False
        torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
        preflight = preflight_gradient(parent, dataset, order, device); pair.verify_inputs(pins, proposed["sources"], [])
        images = pair.verify_training_images(datasets); args.output.mkdir(); _created_output = args.output
        write_new(args.output / "declaration.json", declaration); joint.archive_sources(args.output, proposed["sources"])
        write_new(args.output / "started.json", dict(schema=SCHEMA, UTC=datetime.now(timezone.utc).isoformat(), PID=os.getpid(), GPU=gpu, preflight=preflight, TRAIN_images=images))
        final, result = train(args, parent, dataset, order, declaration, proposed["sources"], device)
        pair.verify_inputs(pins, proposed["sources"], [args.output])
        if not result["completed_fixed_endpoint"]:
            write_new(args.output / "summary.json", dict(schema=SCHEMA, status="INCOMPLETE_FIXED20K_NO_EVALUATION_NOT_PROMOTED", fit=result, model_promoted=False)); return 2
        endpoint = verify_final(args.output, result, parent, proposed["sources"])
        holdout, ho_audits, ho_receipt, known, known_receipt = pair.load_evaluation_only(args, training, low, legacy)
        for domain in (left, right):
            tokens, episodes = {r.token for r in domain.examples}, {r.episode_id for r in domain.examples}
            require(all(not tokens & {r.token for r in d.examples} and not episodes & {r.episode_id for r in d.examples} for d in (holdout, known)),
                "LEFT/RIGHT evaluation repeated-view leakage")
        pins.update(pair.evaluation_artifact_pins(args, known_receipt, holdout, known)); pair.verify_training_images((holdout, known))
        reference = pinned_read(ORIGINAL_SCREENS_PATH, ORIGINAL_SCREENS_SHA); pins[str(ORIGINAL_SCREENS_PATH)] = ORIGINAL_SCREENS_SHA
        write_new(args.output / "evaluation_opened_after_final.json", dict(endpoint=endpoint, HOLDOUT=ho_receipt, known_DEV=known_receipt,
            original74_summary_sha256=ORIGINAL_SCREENS_SHA, checkpoint_selection=False))
        post = time.monotonic(); metrics = {}
        for label, model in (("parent", parent), ("final", final)):
            metrics[label] = evaluate(model, device, training, audits, low, legacy, strata, left, right, holdout, ho_audits, known)
            for cohort, value in metrics[label].items(): write_new(args.output / (label + "_" + cohort + ".json"), value)
        screens = endpoint_screens(metrics["parent"], metrics["final"], reference)
        require(pair.load_train(args)[-1] == {k: v for k, v in receipts.items() if k not in ("LEFT", "RIGHT")}
            and load_LEFT(args)[1] == left_receipt and load_RIGHT(args)[1] == right_receipt, "all TRAIN loader/label/source fingerprints changed")
        post_ho, _, post_ho_receipt, post_known, post_known_receipt = pair.load_evaluation_only(args, training, low, legacy)
        require(post_ho_receipt == ho_receipt and post_known_receipt == known_receipt and post_ho.fingerprint_sha256 == holdout.fingerprint_sha256
            and post_known.fingerprint_sha256 == known.fingerprint_sha256, "full deferred evaluation images/labels changed")
        pair.verify_inputs(pins, proposed["sources"], [args.output]); final_images = pair.verify_training_images((*datasets, holdout, known))
        verify_final(args.output, result, parent, proposed["sources"])
        write_new(args.output / "summary.json", dict(schema=SCHEMA, status="FIXED20000_ACTUAL_LEFT_RIGHT_JOINT_COMPLETE_NOT_PROMOTED", fit=result,
            endpoint_diagnostic_screens=screens, all74_original_screens_retained=True, full_LEFT347_retained=True, full_RIGHT697_retained=True,
            postfit_evaluation_seconds=time.monotonic() - post, final_image_postcheck=final_images, model_promoted=False, vehicle_control_approved=False,
            official_main_features_approved=0, official_subfeatures_approved=0))
    return 0


verify_frozen()
if __name__ == "__main__": raise SystemExit(main())
