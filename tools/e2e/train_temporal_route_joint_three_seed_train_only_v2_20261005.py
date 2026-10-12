"""HH_261005 - Separate immutable CPU/GPU grants for three TRAIN-only warm-start trajectories.

This is three independent continuations of the SAME pretrained lineage, not
three fresh-random pipelines, a six-model A/B, feature approval, or a latency
repair. Only ROOT may subsequently authorize private CPU admission or GPU0
fitting. No DEV, HOLDOUT or TEST loader/evaluator is called here.
"""
from __future__ import annotations

import argparse
import ast
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
from types import FunctionType, SimpleNamespace

if __name__ == "__mp_main__":
    # HH_261005 - Spawn readers configure imports only, never CUDA or another fit.
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_three_seed_workspace", _entry)
    _module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

from scripts.e2e import train_temporal_intrinsic_curve_pair_v2_20261002 as gpu_revision

pair, original = gpu_revision.pair, gpu_revision.pair.original
raw_parent, replay, previous, joint = pair.raw_parent, pair.replay, pair.previous, pair.joint
torch, np = pair.torch, pair.np
require, digest, write_new, regular = pair.require, pair.digest, pair.write_new, pair.regular
ROOT = Path(__file__).resolve().parents[2]
WORKSPACE, PERSONAL = original.WORKSPACE, original.PERSONAL
SCHEMA = "portable_e2e.temporal_route_joint_three_seed_train_only.20261005.v2"
EXPORT_SCHEMA = SCHEMA + ".numeric_research_export"
STUDY = "temporal_route_joint_three_seed_train_only_20261005_v2"
SEEDS = (2026100501, 2026100502, 2026100503)
UPDATES, BATCH_SIZE = 6000, 4
CAMPAIGN_OUTPUT = WORKSPACE / "runs/training" / STUDY
AUTHORITY_DIRECTORY = WORKSPACE / "plans/training" / STUDY
COMMON_RUN = WORKSPACE / "runs/training/temporal_route_recovery_v3_20261001_v1"
BACKBONE_RUN = WORKSPACE / "runs/training/temporal_v2_balanced_replay_20261001_v1"
TRAIN_ROOT = PERSONAL / "dataset/prepared/town07-signed-recovery-train-20261001-v4"
COMMON_PINS = {
    str(COMMON_RUN / "exports/step_003000/model_state.npz"): original.PARENT_NPZ_SHA,
    str(COMMON_RUN / "exports/step_003000/metadata.json"): original.PARENT_METADATA_SHA,
    str(COMMON_RUN / "checkpoints/step_003000.pt"): original.PARENT_CHECKPOINT_SHA,
    str(COMMON_RUN / "declaration.json"): original.PARENT_DECLARATION_SHA,
    str(BACKBONE_RUN / "exports/step_003000/model_state.npz"): "4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735",
    str(BACKBONE_RUN / "exports/step_003000/metadata.json"): "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec",
}
TRAIN_PINS = {
    str(TRAIN_ROOT / "dataset.json"): "5b6d12b00bb718e5871447ce79df05e5b65cd652fa3924085c11250a6ebf6449",
    str(TRAIN_ROOT / "moving_conversion_qualification.json"): original.TRAIN_RECEIPT_SHA,
    str(replay.LOW_PATH / "dataset.json"): replay.LOW_MANIFEST_SHA,
    str(replay.LOW_PATH / "moving_conversion_qualification.json"): replay.LOW_CONVERSION_SHA,
    str(replay.LEGACY_PATH / "dataset.json"): raw_parent.TRAIN_MANIFEST_SHA,
}
BASE_PINS = {
    "tools/e2e/train_temporal_route_joint_20261002.py": "971eb207d79fe6ed7e015da50a5351e9e30e13ce9631f2cf39267214379ae673",
    "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py": "3da40c323c69b1d3e1a64b77f68955b4a8c169a895b5698db312f86d4b23c0c8",
    "tools/e2e/train_temporal_intrinsic_curve_pair_v2_20261002.py": "d2a585dcc0485f8fe5cde07a66e53ddec4ae5ea7f3a0613b8f32a5c2ce46940a",
    "learning/portable_e2e/temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
    "learning/portable_e2e/temporal_route_joint_export_research_20261002.py": "3245e5ee72b6a2bca030249b3075704eb579a0331d0b0c4d6292068e7c192bab",
}
# HH_261005 - This is a previously sealed role identifier only; its payload is never opened.
DEFERRED_HOLDOUT_FINGERPRINT = "d8c91b026f7cd8b1875891e611969cbc41c58eceb75233b488c6ab1059f639c1"


def json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def is_sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def validate_seed(value):
    require(type(value) is int and value in SEEDS, "one exact prospectively declared seed required")
    return value


def seed_output(seed):
    return CAMPAIGN_OUTPUT / ("seed_%d" % validate_seed(seed))


def authorization_path(mode):
    # HH_261005 - Never overwrite the CPU grant when ROOT later seals actual schedules for a GPU grant.
    require(mode in ("validate-only", "fit-one"), "one exact prospective authority phase required")
    names = {"validate-only": "ROOT_CPU_TRAIN_only_authorization.json",
        "fit-one": "ROOT_GPU0_TRAIN_only_authorization.json"}
    return AUTHORITY_DIRECTORY / names[mode]


def source_snapshot():
    for name, sha in BASE_PINS.items(): require(digest(ROOT / name) == sha, "frozen dependency changed: " + name)
    old = gpu_revision.source_snapshot()
    paths = set(replay.imported_source_closure(ROOT,
        [Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    paths.add(ROOT / "learning/tests/test_temporal_route_joint_three_seed_train_only_v2_20261005.py")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(all(result.get(k) == sha for k, sha in old.items()), "complete frozen29 source closure required")
    return result


def contract():
    return dict(schema=SCHEMA, sources=source_snapshot(), seeds=list(SEEDS),
        common_initialization_sha256=COMMON_PINS, TRAIN_admission_sha256=TRAIN_PINS,
        optimizer_updates_per_seed=UPDATES, effective_batch_size=BATCH_SIZE, exposures_per_seed=24000,
        domain_exposures=dict(recovery_focus=6000, recovery_full=6000, low=6000, legacy=6000,
            legacy_curved=3000, legacy_straight=3000),
        original_TRAIN_membership=5832, eligible_TRAIN_membership=5532, training_episode_count=18,
        training_model_state_count=101, trainable_state_count=54, frozen_state_count=47,
        trainable_parameter_count=2277827, objective="unchanged original joint B4 exact_objective",
        learning_rate=original.learning_rate_config(), batch_domains=original.BATCH_DOMAINS,
        initialization="same authenticated final3000; all101 states strict; no parameter reset",
        optimizer="fresh AdamW per independent process; empty state/step0; no resume",
        seed_scope="three optimizer trajectories conditional on ONE shared pretrained lineage",
        common_initialization_exposure="historical known DEV diagnostics; not a pristine or TEST-independent initializer",
        independently_random_pretraining_claimed=False, original_GOOD6000_counted_as_new_seed=False,
        training_model_A_B_performed=False, candidate_model_design_comparison_complete=False,
        future_runtime_pair="same endpoint weights on CPU/GPU, not six separately trained models",
        schedule="original full6000 B4, distinct per-seed queue shuffles; equal membership/domain counts",
        heldout_role_metadata_only=True, evaluation_only_payloads_opened=False,
        known_DEV_or_HOLDOUT_or_TEST_loss=False, learned_teacher_retention_loss=False,
        labels_clipped=False, labels_extrapolated=False, new_head=False, stop_intent_trained=False,
        latency_repair_claimed=False, fixed_final_only=True, checkpoint_updates=list(range(500, 6001, 500)),
        automatic_retry=False, automatic_longer_training=False, serial_resume=False,
        fit_wall_limit_seconds=pair.MAX_ARM_FIT_SECONDS, loader_workers=2,
        physical_GPU_index=0, physical_GPU_UUID=raw_parent.GPU_UUID,
        existing_GPU0_lease=str(WORKSPACE / "runs/campaigns/.gpu0_training.lock"),
        gpu_preflight_revision=dict(gpu_revision.GPU_REVISION), private_venv=str(WORKSPACE / "venvs/py312"),
        campaign_output=str(CAMPAIGN_OUTPUT), seed_outputs={str(s): str(seed_output(s)) for s in SEEDS},
        campaign_container="ROOT must create the declared parent before admission; this runner never creates an ancestor",
        early_failure_receipts="ROOT caller retains launched/failed/unexecuted seed attempts, including failures before output creation",
        queue_execution="ROOT launches each exact seed in a fresh process, serially, no automatic dispatch",
        authority_paths={mode: str(authorization_path(mode)) for mode in ("validate-only", "fit-one")},
        grant_transition="immutable CPU grant retained; separate fresh GPU0 grant only after three actual schedule SHAs",
        model_promoted=False, simulator_control_authorized=False, vehicle_control_approved=False,
        formal_main_approved=0, formal_sub_approved=0)


def validate_authority(value, proposed, mode):
    require(mode in ("validate-only", "fit-one") and isinstance(value, dict)
        and value.get("schema") == SCHEMA + ".authorization" and value.get("authorized") is True
        and value.get("contract") == proposed and value.get("contract_sha256") == json_sha(proposed)
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False
        and type(value.get("physical_GPU_index")) is int and value["physical_GPU_index"] == 0,
        "new exact ROOT source/input/protocol GPU0 authority required")
    modes = value.get("authorized_modes")
    require(isinstance(modes, list) and modes and len(modes) == len(set(modes))
        and all(x in ("validate-only", "fit-one") for x in modes) and mode in modes,
        "CPU admission cannot grant an unreviewed GPU fit")
    seals = value.get("schedule_sha256")
    if mode == "fit-one":
        require(isinstance(seals, dict) and set(seals) == {str(s) for s in SEEDS}
            and all(is_sha(h) for h in seals.values()) and len(set(seals.values())) == 3,
            "all three actual distinct schedules must be sealed before the first fit")
    return value


def fixed_schedule(groups, strata, seed):
    # HH_261005 - Private globals change ONLY queue RNG seed; no frozen module mutation.
    validate_seed(seed)
    function = original.fixed_batches
    namespace = dict(function.__globals__, SEED=seed)
    copied = FunctionType(function.__code__, namespace, function.__name__, function.__defaults__, function.__closure__)
    copied.__kwdefaults__ = function.__kwdefaults__
    require(copied.__code__ is function.__code__ and original.SEED == 2026100201,
        "exact original schedule code and untouched frozen global required")
    return copied(groups, strata)


def verify_schedule(order, groups, strata, seed):
    validate_seed(seed)
    require(isinstance(order, list) and len(order) == UPDATES and all(isinstance(r, list)
        and len(r) == 4 and all(type(i) is int for i in r) for r in order), "6000 real B4 index rows required")
    require(order == fixed_schedule(groups, strata, seed), "the actual order must equal the exact frozen sampler for this seed")
    focus, full = sorted(groups["focus"]), sorted(groups["full"])
    require(focus and len(full) == 9, "unchanged signed focus/full-nine cohorts required")
    counts = {"curved": 0, "straight": 0}
    for step, row in enumerate(order):
        kind = "curved" if step % 2 == 0 else "straight"
        maps = sorted(strata[kind]); selected = maps[counts[kind] % len(maps)]; counts[kind] += 1
        require(row[0] in groups["focus"][focus[step % len(focus)]]
            and row[1] in groups["full"][full[step % 9]] and 2718 <= row[2] < 3045
            and row[3] - 3045 in strata[kind][selected], "unchanged TRAIN strata/role/cycle membership required")
    return dict(seed=seed, schedule_sha256=json_sha(order), optimizer_updates=UPDATES,
        exposures=24000, domain_exposures=[6000, 6000, 6000, 6000],
        legacy_curved=counts["curved"], legacy_straight=counts["straight"],
        unique_sample_indices=[len({r[p] for r in order}) for p in range(4)])


def verify_schedules(orders, groups, strata):
    require(isinstance(orders, dict) and set(orders) == set(SEEDS), "all three prospective schedules required")
    reports = {str(s): verify_schedule(orders[s], groups, strata, s) for s in SEEDS}
    require(len({r["schedule_sha256"] for r in reports.values()}) == 3,
        "different seed numbers with the same actual order are not independent optimizer trajectories")
    return reports


def validate_fit_start(initial_sha, parent_sha, optimizer_state):
    require(is_sha(initial_sha) and initial_sha == parent_sha and isinstance(optimizer_state, dict)
        and optimizer_state.get("state") == {} and isinstance(optimizer_state.get("param_groups"), list)
        and len(optimizer_state["param_groups"]) == 2, "strict same101 initialization/new empty AdamW required")
    return dict(initial101_sha256=initial_sha, common101_sha256=parent_sha,
        fresh_optimizer_state_empty=True, optimizer_updates_before_fit=0, optimizer_resume=False)


def state_sha(model):
    value = hashlib.sha256(); state = model.state_dict()
    require(len(state) == 101, "complete common101 state required")
    for name, tensor in sorted(state.items()):
        require(tensor.dtype == torch.float32 and bool(torch.isfinite(tensor).all()), "finite float32 state required")
        value.update(name.encode()); value.update(str(tuple(tensor.shape)).encode())
        value.update(tensor.detach().contiguous().cpu().numpy().tobytes())
    return value.hexdigest()


def checkpoint_saver(seed):
    fields = original.scope_fields()
    fields.update(current_training_purpose="independent_seed_joint6000_TRAIN_only_reproduction",
        seed=seed, independently_random_pretraining_claimed=False, three_seed_model_A_B_performed=False,
        common_pretrained_lineage_shared=True, DEV_or_HOLDOUT_or_TEST_payload_loaded=False)
    function = original.save_checkpoint
    namespace = dict(function.__globals__, SCHEMA=SCHEMA, EXPORT_SCHEMA=EXPORT_SCHEMA,
        scope_fields=lambda: dict(fields))
    copied = FunctionType(function.__code__, namespace, function.__name__, function.__defaults__, function.__closure__)
    copied.__kwdefaults__ = function.__kwdefaults__
    def save(output, model, optimizer, step, declaration, declaration_sha, frozen_sha, arm):
        require(arm == "control" and declaration["seed"] == seed, "only unchanged B4 objective allowed")
        return copied(output, model, optimizer, step, declaration["source_sha256"],
            declaration["training_dataset_fingerprint_sha256"], DEFERRED_HOLDOUT_FINGERPRINT,
            declaration_sha, frozen_sha)
    return save


def validate_endpoint_metadata(description, declaration, checkpoint_sha, npz_sha, declaration_sha):
    # HH_261005 - Final warm3000->joint6000 lineage is 12200, not a serial GOOD6000 resume.
    expected = dict(schema=EXPORT_SCHEMA, training_schema=SCHEMA, seed=declaration["seed"],
        optimizer_updates=6000, additional_optimizer_updates=6000, maximum_optimizer_updates=6000,
        known_total_optimizer_updates=12200, parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=6200, examples_seen=24000, effective_batch_size=4,
        recovery_examples_seen=12000, v4_examples_seen=6000, legacy_examples_seen=6000,
        parent_checkpoint_sha256=original.PARENT_CHECKPOINT_SHA, parent_npz_sha256=original.PARENT_NPZ_SHA,
        parent_metadata_sha256=original.PARENT_METADATA_SHA, parent_declaration_sha256=original.PARENT_DECLARATION_SHA,
        checkpoint_sha256=checkpoint_sha, npz_sha256=npz_sha, declaration_sha256=declaration_sha,
        source_sha256=declaration["source_sha256"],
        training_dataset_fingerprint_sha256=declaration["training_dataset_fingerprint_sha256"],
        frozen_state_sha256=declaration["frozen47_sha256"], trainable_state_tensor_count=54,
        frozen_state_tensor_count=47, trainable_parameter_count=2277827, training_only=True,
        simulation_only=True, model_promoted=False, vehicle_control_approved=False,
        inference_loader_authorized=False, DEV_or_HOLDOUT_or_TEST_payload_loaded=False,
        common_pretrained_lineage_shared=True, independently_random_pretraining_claimed=False,
        three_seed_model_A_B_performed=False, stop_intent_trained=False)
    require(isinstance(description, dict) and all(type(description.get(k)) is type(v)
        and description[k] == v for k, v in expected.items()), "actual final6000 export lineage/scope differs")
    return dict(seed=declaration["seed"], fixed_endpoint_updates=6000,
        checkpoint_sha256=checkpoint_sha, npz_sha256=npz_sha, metadata_lineage_exact=True)


def verify_final_endpoint(output, result, parent, declaration, sources):
    require(result.get("optimizer_updates") == 6000 and result.get("examples_seen") == 24000
        and result.get("completed_fixed_endpoint") is True and result.get("arm") == "control"
        and [c["optimizer_updates"] for c in result.get("checkpoints", [])] == list(range(500, 6001, 500)),
        "all12 real fixed checkpoint receipts required")
    # HH_261005 - Authenticate every retained endpoint; do not select a favorable intermediate model.
    for row in result["checkpoints"]:
        step = row["optimizer_updates"]; folder = output / "exports" / ("step_%06d" % step)
        require(digest(output / "checkpoints" / ("step_%06d.pt" % step)) == row["sha256"]
            and digest(folder / "model_state.npz") == row["export_npz_sha256"]
            and digest(folder / "metadata.json") == row["export_metadata_sha256"], "one retained checkpoint/export changed")
    last = result["checkpoints"][-1]; folder = output / "exports/step_006000"
    description = pair.read(folder / "metadata.json")
    receipt = validate_endpoint_metadata(description, declaration, last["sha256"], last["export_npz_sha256"], digest(output / "declaration.json"))
    require((folder / "model_state.npz").stat().st_size <= 32 * 1024**2, "bounded complete101 numeric export required")
    expected = parent.state_dict()
    with np.load(folder / "model_state.npz", allow_pickle=False) as arrays:
        require(len(arrays.files) == len(set(arrays.files)) == len(expected) == 101
            and set(arrays.files) == set(expected), "exact101 endpoint names required")
        for name, tensor in expected.items():
            value = arrays[name]
            require(value.dtype == np.float32 and np.isfinite(value).all() and list(value.shape) == list(tensor.shape)
                == description["state_tensor_shapes"][name] and description["state_tensor_dtypes"][name] == "float32",
                "actual finite numeric shape/dtype differs: " + name)
            if not name.startswith(original.TRAINABLE_PREFIXES):
                require(np.array_equal(value, tensor.detach().cpu().numpy()), "one actual frozen47 export changed: " + name)
    replay.verify_sources(output, sources)
    return dict(receipt, metadata_sha256=last["export_metadata_sha256"], all101_numeric_finite=True, frozen47_exact=True)


def train_function(seed):
    # HH_261005 - Inverse AST proves numerical CONTROL/finite/source guards unchanged.
    tree = ast.parse(Path(pair.__file__).read_bytes())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "train_arm")
    node = copy.deepcopy(old)
    positions = [i for i, n in enumerate(node.body) if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "optimizer" for t in n.targets)]
    require(len(positions) == 1, "one frozen fresh AdamW construction required")
    index = positions[0] + 1
    addition = ast.parse("write_new(output/'fresh_initialization.json', validate_fit_start(state_sha(model), state_sha(parent), optimizer.state_dict()))").body[0]
    node.body.insert(index, addition)
    labels = [n for n in ast.walk(node) if isinstance(n, ast.Constant) and n.value == "ACTUAL_PAIRED_CONTINUATION"]
    require(len(labels) == 1, "one truthful progress label seam required")
    labels[0].value = "ACTUAL_INDEPENDENT_JOINT_TRAIN_SEED"
    node.body.pop(index); labels[0].value = "ACTUAL_PAIRED_CONTINUATION"
    require(ast.dump(node) == ast.dump(old), "another training numerical or safety statement changed")
    node.body.insert(index, addition); labels[0].value = "ACTUAL_INDEPENDENT_JOINT_TRAIN_SEED"
    namespace = dict(vars(pair), original=SimpleNamespace(**dict(vars(original), SEED=seed)),
        save_checkpoint=checkpoint_saver(seed), state_sha=state_sha, validate_fit_start=validate_fit_start)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["train_arm"]


def prepare(args, proposed, authority):
    require(Path(sys.prefix).resolve() == WORKSPACE / "venvs/py312"
        and ROOT.resolve() == WORKSPACE / "source_worktrees" / STUDY, "existing private venv/exact fresh source required")
    require(args.authorization == authorization_path(args.mode), "one exact prospective ROOT phase authority path required")
    require(args.output == seed_output(args.seed) and not args.output.exists()
        and not any(p.is_symlink() for p in (args.output, *args.output.parents)), "new exact seed output; no resume/symlink required")
    require(args.output.parent.is_dir() and not args.output.parent.is_symlink(), "ROOT-created campaign container must already exist")
    inputs = dict(COMMON_PINS, **TRAIN_PINS)
    inputs[str(authorization_path(args.mode))] = args.authorization_sha256
    for path, sha in inputs.items(): regular(path, PERSONAL); require(digest(path) == sha, "immutable common/TRAIN input changed: " + path)
    require(args.recovery_train == TRAIN_ROOT and args.recovery_conversion == TRAIN_ROOT / "moving_conversion_qualification.json",
        "only existing approved actual recovery TRAIN allowed")
    training, audits, low, legacy, strata, receipts = pair.load_train(args)
    require(all(d.split == "train" for d in (training, low, legacy)), "no evaluation role in optimizer dataset")
    groups = previous.recovery_groups(training, audits)
    orders = {s: fixed_schedule(groups, strata, s) for s in SEEDS}
    reports = verify_schedules(orders, groups, strata)
    if args.mode == "fit-one":
        require(authority["schedule_sha256"] == {s: r["schedule_sha256"] for s, r in reports.items()},
            "actual schedules differ from prospective ROOT seals")
    parent = original.load_joint_parent(COMMON_RUN / "exports/step_003000/model_state.npz",
        COMMON_RUN / "exports/step_003000/metadata.json",
        backbone_parent_weights=BACKBONE_RUN / "exports/step_003000/model_state.npz",
        backbone_parent_metadata=BACKBONE_RUN / "exports/step_003000/metadata.json", source_root=ROOT)
    names = original.freeze_inputs(parent)
    declaration = dict(schema=SCHEMA, protocol=proposed, seed=args.seed, source_sha256=proposed["sources"],
        all_seed_schedules=reports, fixed_schedule_sha256=reports[str(args.seed)]["schedule_sha256"],
        fixed_sample_ids=[pair.expected_tokens((training, low, legacy), r) for r in orders[args.seed]],
        training_dataset_fingerprint_sha256=json_sha(receipts), TRAIN_receipts=receipts,
        immutable_input_sha256=inputs, authorization_sha256=args.authorization_sha256,
        common_initial101_sha256=state_sha(parent), trainable_state_names=names,
        frozen47_sha256=original.frozen_fingerprint(parent.state_dict()),
        evaluation_only_payloads_opened=False, model_promoted=False, vehicle_control_approved=False)
    return parent, (training, low, legacy), strata, orders[args.seed], declaration


def parser():
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument("--mode", choices=("contract", "validate-only", "fit-one"), required=True)
    p.add_argument("--seed", type=int); p.add_argument("--authorization", type=Path)
    p.add_argument("--authorization-sha256"); p.add_argument("--output", type=Path)
    p.add_argument("--recovery-train", type=Path, default=TRAIN_ROOT)
    p.add_argument("--recovery-conversion", type=Path, default=TRAIN_ROOT / "moving_conversion_qualification.json")
    p.add_argument("--loader-workers", type=int, choices=(2,), default=2)
    return p


def main(argv=None):
    args = parser().parse_args(argv); proposed = contract(); created = False
    if args.mode == "contract": print(json.dumps(proposed, allow_nan=False)); return 0
    validate_seed(args.seed)
    require(args.authorization == authorization_path(args.mode) and is_sha(args.authorization_sha256), "exact ROOT phase authority path and external SHA required")
    regular(args.authorization, PERSONAL)
    payload = Path(args.authorization).read_bytes()
    require(hashlib.sha256(payload).hexdigest() == args.authorization_sha256, "ROOT authority changed before first read")
    authority = json.loads(payload, object_pairs_hook=pair.unique_json_object)
    validate_authority(authority, proposed, args.mode)
    try:
        require(not torch.cuda.is_initialized(), "CPU admission must precede CUDA")
        torch.set_num_threads(4); parent, datasets, strata, order, declaration = prepare(args, proposed, authority)
        replay.storage_reserve(args.output.parent)
        if args.mode == "validate-only":
            require(not torch.cuda.is_initialized(), "CPU admission cannot initialize CUDA")
            print(json.dumps(dict(status="PREPARED_TRAIN_ONLY_NO_CUDA_NO_OUTPUT", declaration=declaration), allow_nan=False)); return 0
        with raw_parent.gpu0_lease():
            gpu = gpu_revision.gpu_preflight(); device = torch.device("cuda:0")
            require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "strict deterministic cuBLAS workspace required")
            torch.use_deterministic_algorithms(True); torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True; torch.backends.cuda.matmul.allow_tf32 = False
            torch.backends.cudnn.allow_tf32 = False
            if hasattr(torch.backends.cuda, "enable_flash_sdp"):
                torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
            images_before = pair.verify_training_images(datasets)
            pair.verify_inputs(declaration["immutable_input_sha256"], proposed["sources"], [])
            replay.storage_reserve(args.output.parent); args.output.mkdir(exist_ok=False); created = True
            write_new(args.output / "declaration.json", declaration)
            write_new(args.output / "started.json", dict(PID=os.getpid(), UTC=datetime.now(timezone.utc).isoformat(),
                GPU=gpu, seed=args.seed, images=images_before, common_initial101_sha256=declaration["common_initial101_sha256"]))
            dataset = pair.ConcatDataset(datasets); curves = set(pair.curved_indices(strata))
            model, hashes, result = train_function(args.seed)(args, "control", parent, dataset, order, curves,
                declaration, proposed["sources"], device)
            original.freeze_inputs(model)
            require(original.frozen_fingerprint(model.state_dict()) == declaration["frozen47_sha256"]
                and state_sha(parent) == declaration["common_initial101_sha256"], "common101 or frozen47 changed")
            pair.verify_inputs(declaration["immutable_input_sha256"], proposed["sources"], [args.output / "control"])
            images_after = pair.verify_training_images(datasets)
            require(images_before == images_after, "full TRAIN image census changed during fit")
            post = pair.load_train(args)
            require(post[-1] == declaration["TRAIN_receipts"] and post[-2] == strata,
                "all original TRAIN labels/membership/strata fingerprints changed during fit")
            completed = result["completed_fixed_endpoint"]
            endpoint = verify_final_endpoint(args.output / "control", result, parent, declaration, proposed["sources"]) if completed else None
            write_new(args.output / "summary.json", dict(schema=SCHEMA,
                status="FIXED6000_SEED_TRAIN_COMPLETE_NOT_PROMOTED" if completed else "INCOMPLETE_SEED_ALL_TAILS_RETAINED",
                seed=args.seed, fit=result, actual_batch_sha256=hashes, images_before=images_before,
                images_after=images_after, final_endpoint=endpoint, TRAIN_receipts_before_after_exact=True,
                evaluation_only_payloads_opened=False, common_pretrained_lineage_shared=True,
                three_seed_A_B_performed=False, model_promoted=False, vehicle_control_approved=False,
                formal_main_approved=0, formal_sub_approved=0))
            return 0 if completed else 2
    except Exception as error:
        if created:
            write_new(args.output / "failure.json", dict(schema=SCHEMA, seed=args.seed, error=repr(error),
                failed_run_retained=True, automatic_retry=False, evaluation_only_payloads_opened=False,
                model_promoted=False, vehicle_control_approved=False))
        raise


if __name__ == "__main__": raise SystemExit(main())

