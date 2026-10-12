"""HH_261005 - Test prospective three-seed orchestration with scalar TRAIN fixtures only."""
import ast
import argparse
from collections import Counter
from contextlib import nullcontext
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import sys
from types import FunctionType, ModuleType, SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/train_temporal_route_joint_three_seed_train_only_v2_20261005.py"
SOURCE_V1 = ROOT / "tools/e2e/train_temporal_route_joint_three_seed_train_only_v1_20261005.py"
ORIGINAL = ROOT / "tools/e2e/train_temporal_route_joint_20261002.py"
REPLAY = ROOT / "tools/e2e/train_temporal_v2_balanced_replay_20261001.py"
PAIR = ROOT / "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"


def require(value, message):
    if not value:
        raise ValueError(message)


def function_node(path, name):
    tree = ast.parse(path.read_text())
    return next(node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name == name)


def genuine_sampler():
    # HH_261005 - Compile only the genuine scalar CycleQueue and frozen sampler, never import Torch or dataset loaders.
    scope = dict(require=require, random=random, Counter=Counter, SEED=2026100201, MAX_UPDATES=6000)
    queue = function_node(REPLAY, "CycleQueue")
    exec(compile(ast.Module(body=[queue], type_ignores=[]), str(REPLAY), "exec"), scope)
    scope["replay"] = SimpleNamespace(CycleQueue=scope["CycleQueue"])
    sampler = function_node(ORIGINAL, "fixed_batches")
    exec(compile(ast.Module(body=[sampler], type_ignores=[]), str(ORIGINAL), "exec"), scope)
    return scope["fixed_batches"]


def groups():
    profiles = [(cross, offset) for cross in (-.2, 0., .2) for offset in (-1.5, 0., 1.5)]
    full = {key: list(range(index * 302, (index + 1) * 302)) for index, key in enumerate(profiles)}
    focus = {(-1, -1): full[(-.2, -1.5)][:7], (-1, 0): full[(-.2, 0.)][:19],
        (-1, 1): full[(-.2, 1.5)][:115], (0, -1): full[(0., -1.5)][:104],
        (0, 1): full[(0., 1.5)][:63], (1, -1): full[(.2, -1.5)][:116],
        (1, 0): full[(.2, 0.)][:9], (1, 1): full[(.2, 1.5)][:7]}
    old = dict(curved={"C_track_1_0_7": list(range(260)), "Town01": list(range(260, 328))},
        straight={"C_track_1_0_7": list(range(328, 990)), "Town01": list(range(990, 1560)),
            "Town07": list(range(1560, 2487))})
    return dict(full=full, focus=focus), old


def digest_bytes(value):
    return hashlib.sha256(value).hexdigest()


def scalar_schedule_sha(rows):
    return digest_bytes(json.dumps(rows, separators=(",", ":"), allow_nan=False).encode())


def pure_module():
    # HH_261005 - Import no production module; real function ASTs execute against explicit stdlib/scalar providers.
    module = ModuleType("hh_three_seed_scalar_fixture")
    scope = vars(module)
    scope.update(__file__=str(SOURCE), ast=ast, argparse=argparse, copy=copy, datetime=datetime,
        timezone=timezone, hashlib=hashlib, json=json, os=os, Path=Path, sys=sys,
        FunctionType=FunctionType, SimpleNamespace=SimpleNamespace, require=require,
        ROOT=ROOT, WORKSPACE=Path("/home/pro6000/personal/hwanhong/portable_e2e"),
        PERSONAL=Path("/home/pro6000/personal/hwanhong"), BASE_PINS={}, COMMON_PINS={}, TRAIN_PINS={})
    tree = ast.parse(SOURCE.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign):
            try:
                value = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
            for target in node.targets:
                if isinstance(target, ast.Name):
                    scope[target.id] = value
                elif isinstance(target, ast.Tuple):
                    for item, scalar in zip(target.elts, value):
                        scope[item.id] = scalar
    scope["EXPORT_SCHEMA"] = scope["SCHEMA"] + ".numeric_research_export"
    scope["CAMPAIGN_OUTPUT"] = scope["WORKSPACE"] / "runs/training" / scope["STUDY"]
    scope["AUTHORITY_DIRECTORY"] = scope["WORKSPACE"] / "plans/training" / scope["STUDY"]
    scope["TRAIN_ROOT"] = scope["PERSONAL"] / "dataset/prepared/town07-signed-recovery-train-20261001-v4"
    scope["COMMON_RUN"] = scope["WORKSPACE"] / "runs/training/temporal_route_recovery_v3_20261001_v1"
    scope["BACKBONE_RUN"] = scope["WORKSPACE"] / "runs/training/temporal_v2_balanced_replay_20261001_v1"
    old_scope = {}
    originals = [function_node(ORIGINAL, name) for name in ("scope_fields", "save_checkpoint")]
    exec(compile(ast.Module(body=originals, type_ignores=[]), str(ORIGINAL), "exec"), old_scope)
    scope["original"] = SimpleNamespace(SEED=2026100201, fixed_batches=genuine_sampler(),
        scope_fields=old_scope["scope_fields"], save_checkpoint=old_scope["save_checkpoint"],
        learning_rate_config=lambda: {"shared_peak": 1e-5, "heads_peak": 3e-5},
        BATCH_DOMAINS=("focus", "full", "low", "legacy"))
    scope["pair"] = SimpleNamespace(__file__=str(PAIR), MAX_ARM_FIT_SECONDS=5400)
    scope["raw_parent"] = SimpleNamespace(GPU_UUID="GPU-synthetic-physical0")
    scope["gpu_revision"] = SimpleNamespace(GPU_REVISION={"physical_index": 0})
    selected = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future, *selected], type_ignores=[])), str(SOURCE), "exec"), scope)
    return module


@pytest.fixture
def m():
    return pure_module()


def authority(m, contract, modes=("validate-only", "fit-one")):
    return dict(schema=m.SCHEMA + ".authorization", authorized=True, contract=contract,
        contract_sha256=m.json_sha(contract), authorized_modes=list(modes),
        physical_GPU_index=0, model_promoted=False, vehicle_control_approved=False,
        schedule_sha256={str(seed): str(index + 1) * 64 for index, seed in enumerate(m.SEEDS)})


@pytest.mark.parametrize("seed", [2026100501, 2026100502, 2026100503])
def test_exact_seeds_and_unique_create_only_output_identity(m, seed):
    assert m.validate_seed(seed) == seed
    assert m.seed_output(seed) == m.CAMPAIGN_OUTPUT / ("seed_%d" % seed)
    assert len({m.seed_output(value) for value in m.SEEDS}) == 3


@pytest.mark.parametrize("seed", [True, False, 2026100501., "2026100501", 2026100201, 0, None])
def test_other_seed_type_or_undeclared_seed_refused(m, seed):
    with pytest.raises(ValueError, match="declared seed"):
        m.validate_seed(seed)


def test_all_three_genuine_seed_samplers_preserve_code_membership_and_equal_domains(m):
    g, old = groups()
    original_seed = m.original.fixed_batches.__globals__["SEED"]
    orders = {seed: m.fixed_schedule(g, old, seed) for seed in m.SEEDS}
    results = m.verify_schedules(orders, g, old)
    assert len({m.json_sha(value) for value in orders.values()}) == 3
    for seed, rows in orders.items():
        original = m.original.fixed_batches
        exact = FunctionType(original.__code__, dict(original.__globals__, SEED=seed))
        assert rows == exact(g, old) == m.fixed_schedule(g, old, seed)
        result = results[str(seed)]
        assert result["schedule_sha256"] == scalar_schedule_sha(rows)
        assert result["optimizer_updates"] == 6000 and result["exposures"] == 24000
        assert result["domain_exposures"] == [6000] * 4
        assert result["legacy_curved"] == result["legacy_straight"] == 3000
        assert result["unique_sample_indices"] == [sum(map(len, g["focus"].values())), 2718, 327, 2487]
        assert {row[1] for row in rows} == set(range(2718))
        assert {row[2] for row in rows} == set(range(2718, 3045))
        assert {row[3] for row in rows} == set(range(3045, 5532))
    assert original_seed == m.original.SEED == m.original.fixed_batches.__globals__["SEED"] == 2026100201
    assert not hasattr(m.original, "cuda")


@pytest.mark.parametrize("mutation", ["missing_tail", "bool_index", "foreign_role", "different_seed", "admissible_reorder"])
def test_schedule_cannot_swap_true_seed_order_or_drop_failure_denominator(m, mutation):
    g, old = groups(); seed = m.SEEDS[0]
    rows = m.fixed_schedule(g, old, seed)
    if mutation == "missing_tail": rows.pop()
    elif mutation == "bool_index": rows[0][0] = True
    elif mutation == "foreign_role": rows[0][3] = 5532
    elif mutation == "different_seed": rows = m.fixed_schedule(g, old, m.SEEDS[1])
    else:
        # HH_261005 - The low-domain membership remains admissible, but its genuine seeded ordering must not change.
        rows[0][2], rows[1][2] = rows[1][2], rows[0][2]
    with pytest.raises(ValueError): m.verify_schedule(rows, g, old, seed)


def test_missing_seed_and_reused_real_schedule_refused(m):
    g, old = groups(); rows = m.fixed_schedule(g, old, m.SEEDS[0])
    with pytest.raises(ValueError, match="all three"):
        m.verify_schedules({m.SEEDS[0]: rows}, g, old)
    with pytest.raises(ValueError): m.verify_schedules({seed: rows for seed in m.SEEDS}, g, old)


def test_cpu_authority_does_not_grant_fit_and_fit_needs_three_real_schedule_seals(m):
    proposed = {"sources": {"synthetic.py": "a" * 64}}
    cpu = authority(m, proposed, modes=("validate-only",))
    cpu.pop("schedule_sha256")
    assert m.validate_authority(cpu, proposed, "validate-only") is cpu
    with pytest.raises(ValueError, match="CPU admission"):
        m.validate_authority(cpu, proposed, "fit-one")
    fit = authority(m, proposed)
    assert m.validate_authority(fit, proposed, "fit-one") is fit


@pytest.mark.parametrize("fault", ["not_authorized", "bool_gpu", "gpu1", "source_change", "schema", "bad_contract_sha",
    "promotion", "vehicle", "duplicate_modes", "missing_seed_seal", "same_schedule_sha", "malformed_sha"])
def test_exact_root_authority_refuses_role_or_source_or_schedule_changes(m, fault):
    proposed = {"sources": {"synthetic.py": "a" * 64}}; value = authority(m, proposed)
    if fault == "not_authorized": value["authorized"] = False
    elif fault == "bool_gpu": value["physical_GPU_index"] = False
    elif fault == "gpu1": value["physical_GPU_index"] = 1
    elif fault == "source_change": value["contract"] = {"sources": {"synthetic.py": "b" * 64}}
    elif fault == "schema": value["schema"] = "old-authorization"
    elif fault == "bad_contract_sha": value["contract_sha256"] = "c" * 64
    elif fault == "promotion": value["model_promoted"] = True
    elif fault == "vehicle": value["vehicle_control_approved"] = True
    elif fault == "duplicate_modes": value["authorized_modes"].append("fit-one")
    elif fault == "missing_seed_seal": value["schedule_sha256"].pop(str(m.SEEDS[2]))
    elif fault == "same_schedule_sha": value["schedule_sha256"] = {str(seed): "d" * 64 for seed in m.SEEDS}
    else: value["schedule_sha256"][str(m.SEEDS[0])] = "D" * 64
    with pytest.raises(ValueError): m.validate_authority(value, proposed, "fit-one")


def test_fresh_same101_initialization_requires_empty_new_adamw_state(m):
    value = m.validate_fit_start("a" * 64, "a" * 64, {"state": {}, "param_groups": [{}, {}]})
    assert value["fresh_optimizer_state_empty"] is True and value["optimizer_updates_before_fit"] == 0
    assert value["optimizer_resume"] is False and value["initial101_sha256"] == value["common101_sha256"]


@pytest.mark.parametrize("fault", ["other_parent", "bad_sha", "resume_state", "missing_groups", "one_group"])
def test_wrong_parent_or_resumed_optimizer_refused(m, fault):
    initial, parent = "a" * 64, "a" * 64; optimizer = {"state": {}, "param_groups": [{}, {}]}
    if fault == "other_parent": parent = "b" * 64
    elif fault == "bad_sha": initial = "not-a-state-sha"
    elif fault == "resume_state": optimizer["state"] = {0: {"step": 1}}
    elif fault == "missing_groups": optimizer.pop("param_groups")
    else: optimizer["param_groups"].pop()
    with pytest.raises(ValueError, match="same101"): m.validate_fit_start(initial, parent, optimizer)


def test_train_function_inverse_preserves_every_control_math_and_guard_statement(m):
    captured = []
    def record_compile(tree, filename, mode):
        captured.append(copy.deepcopy(tree)); return compile(tree, filename, mode)
    m.compile = record_compile
    compiled = m.train_function(m.SEEDS[1])
    original = function_node(PAIR, "train_arm"); changed = captured[-1].body[0]
    additions = [index for index, node in enumerate(changed.body) if isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name)
        and node.value.func.id == "write_new" and "fresh_initialization.json" in ast.unparse(node)]
    assert len(additions) == 1
    index = additions[0]
    assert isinstance(changed.body[index - 1], ast.Assign)
    assert changed.body[index - 1].targets[0].id == "optimizer"
    labels = [node for node in ast.walk(changed) if isinstance(node, ast.Constant)
        and node.value == "ACTUAL_INDEPENDENT_JOINT_TRAIN_SEED"]
    assert len(labels) == 1
    changed.body.pop(index); labels[0].value = "ACTUAL_PAIRED_CONTINUATION"
    assert ast.dump(changed) == ast.dump(original)
    assert compiled.__globals__["original"].SEED == m.SEEDS[1]
    assert m.original.SEED == 2026100201
    assert "load_state_dict" not in ast.unparse(function_node(SOURCE, "train_function"))


def test_checkpoint_saver_delegates_exact_original_code_with_honest_seed_scope(m):
    seen = []
    def original_save(*values): seen.append((values, original_save.__globals__.copy())); return "saved"
    m.original.save_checkpoint = original_save
    save = m.checkpoint_saver(m.SEEDS[0])
    declaration = dict(seed=m.SEEDS[0], source_sha256={"fixture.py": "a" * 64}, training_dataset_fingerprint_sha256="b" * 64)
    assert save(Path("fixture"), None, None, 500, declaration, "c" * 64, "d" * 64, "control") == "saved"
    assert seen[0][0][6] == m.DEFERRED_HOLDOUT_FINGERPRINT
    copied = save.__closure__[0].cell_contents
    assert copied.__code__ is original_save.__code__
    assert copied.__globals__["SCHEMA"] == m.SCHEMA and copied.__globals__["EXPORT_SCHEMA"] == m.EXPORT_SCHEMA
    fields = copied.__globals__["scope_fields"]()
    assert fields["seed"] == m.SEEDS[0] and fields["common_pretrained_lineage_shared"] is True
    assert fields["DEV_or_HOLDOUT_or_TEST_payload_loaded"] is False
    assert fields["stop_intent_trained"] is False and fields["official_main_features_approved"] == 0
    with pytest.raises(ValueError): save(Path("fixture"), None, None, 500, declaration, "c" * 64, "d" * 64, "intrinsic_curve")


def test_train_only_source_calls_do_not_load_or_evaluate_other_roles():
    tree = ast.parse(SOURCE.read_text())
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    denied = {"load_cohorts", "load_evaluation_only", "evaluate_all", "evaluate", "endpoint_screens", "load_view"}
    assert not any(name.rsplit(".", 1)[-1] in denied for name in calls)
    assert "pair.load_train" in calls and "original.load_joint_parent" in calls
    assert "train_function(args.seed)" in calls and "gpu_revision.gpu_preflight" in calls
    assert "raw_parent.gpu0_lease" in calls
    main = ast.unparse(function_node(SOURCE, "main"))
    assert "'control'" in main and "for seed in" not in main
    assert "torch.cuda.is_initialized()" in main
    assert main.index("gpu_revision.gpu_preflight()") < main.index("torch.device('cuda:0')")
    assert "load_state_dict" not in main and "load_checkpoint" not in main


def test_contract_preserves_lineage_and_no_new_independent_or_functional_approval(m):
    m.source_snapshot = lambda: {"scalar-source.py": "a" * 64}
    value = m.contract()
    assert value["training_model_state_count"] == 101
    assert value["trainable_state_count"] == 54 and value["frozen_state_count"] == 47
    assert value["optimizer_updates_per_seed"] == 6000 and value["effective_batch_size"] == 4
    assert value["exposures_per_seed"] == 24000 and value["domain_exposures"]["legacy_curved"] == 3000
    assert value["physical_GPU_index"] == 0 and value["formal_main_approved"] == value["formal_sub_approved"] == 0
    assert "historical known DEV" in value["common_initialization_exposure"]
    for key in ("evaluation_only_payloads_opened", "known_DEV_or_HOLDOUT_or_TEST_loss", "serial_resume",
        "automatic_retry", "automatic_longer_training", "model_promoted", "stop_intent_trained",
        "candidate_model_design_comparison_complete", "original_GOOD6000_counted_as_new_seed"):
        assert value[key] is False
    assert value["checkpoint_updates"] == list(range(500, 6001, 500))
    assert "ROOT" in value["early_failure_receipts"]


def test_source_snapshot_keeps_old_closure_and_pins_new_source_and_test(m, tmp_path):
    m.ROOT = tmp_path; m.__file__ = str(tmp_path / "new.py")
    old = tmp_path / "old.py"; old.write_bytes(b"scalar frozen source")
    new = tmp_path / "new.py"; new.write_bytes(b"scalar prospective source")
    config = tmp_path / "learning/portable_e2e/config/common_10hz_v1.contract.json"
    test = tmp_path / "learning/tests/test_temporal_route_joint_three_seed_train_only_v2_20261005.py"
    for path in (config, test): path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b"scalar fixture")
    m.digest = lambda path: digest_bytes(Path(path).read_bytes())
    m.BASE_PINS = {"old.py": m.digest(old)}
    m.gpu_revision.source_snapshot = lambda: dict(m.BASE_PINS)
    m.replay = SimpleNamespace(imported_source_closure=lambda *_: [old, new])
    result = m.source_snapshot()
    assert result["old.py"] == m.BASE_PINS["old.py"]
    assert result[str(config.relative_to(tmp_path))] == m.digest(config)
    assert result[str(test.relative_to(tmp_path))] == m.digest(test)
    old.write_bytes(b"changed frozen source")
    with pytest.raises(ValueError, match="dependency changed"): m.source_snapshot()


def prepare_fixture(m, tmp_path):
    # HH_261005 - Scalar admissions exercise path/role orchestration, not real TRAIN labels, tensors or checkpoint loading.
    m.WORKSPACE = tmp_path / "workspace"; m.PERSONAL = tmp_path
    m.ROOT = m.WORKSPACE / "source_worktrees" / m.STUDY; m.ROOT.mkdir(parents=True)
    m.sys = SimpleNamespace(prefix=str(m.WORKSPACE / "venvs/py312"))
    m.CAMPAIGN_OUTPUT = m.WORKSPACE / "runs/training" / m.STUDY; m.CAMPAIGN_OUTPUT.mkdir(parents=True)
    m.AUTHORITY_DIRECTORY = m.WORKSPACE / "plans/training" / m.STUDY
    cpu_authority = m.authorization_path("validate-only")
    cpu_authority.parent.mkdir(parents=True)
    cpu_authority.write_bytes(b"sealed scalar ROOT CPU authorization")
    m.TRAIN_ROOT = tmp_path / "dataset/prepared/approved-TRAIN"
    m.COMMON_RUN = m.WORKSPACE / "runs/training/common-final3000"
    m.BACKBONE_RUN = m.WORKSPACE / "runs/training/backbone-final3000"
    fixture = tmp_path / "common-and-TRAIN-metadata.json"; fixture.write_bytes(b"sealed scalar only")
    m.digest = lambda path: digest_bytes(Path(path).read_bytes())
    m.COMMON_PINS = {str(fixture): m.digest(fixture)}; m.TRAIN_PINS = {}
    scope = dict(Path=Path, require=require)
    exec(compile(ast.Module(body=[function_node(PAIR, "regular")], type_ignores=[]), str(PAIR), "exec"), scope)
    m.regular = scope["regular"]
    g, strata = groups(); loaded = []
    datasets = [SimpleNamespace(split="train") for _ in range(3)]
    def load_train(args): loaded.append(args); return datasets[0], [], datasets[1], datasets[2], strata, {"role": "TRAIN"}
    m.pair.load_train = load_train
    m.pair.expected_tokens = lambda _, row: ["synthetic-TRAIN:%d" % index for index in row]
    m.previous = SimpleNamespace(recovery_groups=lambda *_: g)
    parent = SimpleNamespace(state_dict=lambda: {"state_%d" % index: None for index in range(101)})
    m.original.load_joint_parent = lambda *args, **kwargs: parent
    m.original.freeze_inputs = lambda _: ["state_%d" % index for index in range(54)]
    m.original.frozen_fingerprint = lambda _: "b" * 64
    m.state_sha = lambda _: "a" * 64
    args = SimpleNamespace(mode="validate-only", seed=m.SEEDS[0], output=m.seed_output(m.SEEDS[0]),
        authorization=cpu_authority, authorization_sha256=m.digest(cpu_authority), recovery_train=m.TRAIN_ROOT,
        recovery_conversion=m.TRAIN_ROOT / "moving_conversion_qualification.json", loader_workers=2)
    return args, datasets, loaded, fixture, g, strata


@pytest.mark.parametrize("fault", ["local_venv", "other_source", "other_authority", "other_output", "existing_output",
    "symlink_output", "missing_campaign", "changed_input", "VAL_path", "TEST_path", "role_loader"])
def test_prepare_refuses_wrong_runtime_role_parent_or_output_before_fit(m, tmp_path, fault):
    args, datasets, loaded, fixture, _, _ = prepare_fixture(m, tmp_path)
    if fault == "local_venv": m.sys.prefix = "/usr"
    elif fault == "other_source": m.ROOT = tmp_path
    elif fault == "other_authority": args.authorization = tmp_path / "self-issued.json"
    elif fault == "other_output": args.output = m.CAMPAIGN_OUTPUT / "resume-old"
    elif fault == "existing_output": args.output.mkdir()
    elif fault == "symlink_output": args.output.symlink_to(tmp_path, target_is_directory=True)
    elif fault == "missing_campaign": m.CAMPAIGN_OUTPUT.rmdir()
    elif fault == "changed_input": fixture.write_bytes(b"changed initialization or TRAIN admission")
    elif fault in ("VAL_path", "TEST_path"): args.recovery_train = tmp_path / fault
    else: datasets[1].split = "val"
    with pytest.raises((ValueError, FileNotFoundError)):
        m.prepare(args, {"sources": {}}, {})
    assert len(loaded) == (1 if fault == "role_loader" else 0)
    assert not (args.output / "declaration.json").exists()


def test_cpu_prepare_returns_only_three_train_datasets_and_all_seed_seals(m, tmp_path):
    args, datasets, loaded, _, _, _ = prepare_fixture(m, tmp_path)
    parent, actual, _, rows, declaration = m.prepare(args, {"sources": {"scalar.py": "d" * 64}}, {})
    assert actual == tuple(datasets) and len(loaded) == 1 and all(data.split == "train" for data in actual)
    assert len(parent.state_dict()) == 101 and len(rows) == 6000
    assert len(declaration["fixed_sample_ids"]) == 6000
    assert len(declaration["all_seed_schedules"]) == 3 and len(declaration["trainable_state_names"]) == 54
    assert declaration["evaluation_only_payloads_opened"] is False and not args.output.exists()


def test_fit_prepare_rejects_wrong_presealed_actual_seed_order(m, tmp_path):
    args, _, _, _, _, _ = prepare_fixture(m, tmp_path); args.mode = "fit-one"
    args.authorization = m.authorization_path(args.mode)
    args.authorization.write_bytes(b"sealed scalar ROOT GPU0 authorization")
    args.authorization_sha256 = m.digest(args.authorization)
    with pytest.raises(ValueError, match="prospective ROOT seals"):
        m.prepare(args, {"sources": {}}, {"schedule_sha256": {str(seed): "a" * 64 for seed in m.SEEDS}})


@pytest.mark.parametrize("optimizer_resume", [False, True])
def test_actual_copied_train_function_empty_loader_retains_incomplete_and_fresh_optimizer_guard(m, tmp_path, optimizer_resume):
    # HH_261005 - Execute the genuine copied orchestration around scalar stand-ins; no tensor/forward/gradient/CUDA runs occur.
    seed_calls, artifacts = [], {}
    parent = SimpleNamespace(actual_sha="a" * 64, state_dict=lambda: {})
    model = SimpleNamespace(actual_sha="a" * 64, train=lambda: None)
    model.cpu = lambda: model
    optimizer = SimpleNamespace(state_dict=lambda: {"state": {0: {"step": 1}} if optimizer_resume else {}, "param_groups": [{}, {}]})
    torch = SimpleNamespace(manual_seed=lambda value: seed_calls.append(("torch", value)),
        cuda=SimpleNamespace(manual_seed_all=lambda value: seed_calls.append(("cuda-stub", value))),
        optim=SimpleNamespace(AdamW=lambda *args, **kwargs: optimizer))
    m.pair.__dict__.update(torch=torch, np=SimpleNamespace(random=SimpleNamespace(seed=lambda value: seed_calls.append(("numpy-stub", value)))),
        random=SimpleNamespace(seed=lambda value: seed_calls.append(("python", value))),
        fresh_model=lambda *_: model, write_new=lambda path, value: artifacts.update({str(path): value}), digest=lambda _: "c" * 64,
        joint=SimpleNamespace(archive_sources=lambda *_: None), DataLoader=lambda **_: [],
        IntrinsicSpatialPathLossV1=lambda: SimpleNamespace(to=lambda _: None),
        time=__import__("time"), replay=SimpleNamespace(verify_sources=lambda *_: None))
    m.original.frozen_fingerprint = lambda _: "b" * 64
    m.original.DeterministicPathTimeDecoderV2 = lambda: SimpleNamespace(to=lambda _: None)
    m.original.optimizer_groups = lambda _: [{"name": "shared"}, {"name": "heads"}]
    m.state_sha = lambda value: value.actual_sha
    args = SimpleNamespace(output=tmp_path, loader_workers=2)
    compiled = m.train_function(m.SEEDS[2])
    if optimizer_resume:
        with pytest.raises(ValueError, match="empty AdamW"):
            compiled(args, "control", parent, [], [], set(), {"seed": m.SEEDS[2]}, {}, "scalar-device")
        assert not any(name.endswith("history.jsonl") for name in artifacts)
        assert not any(name.endswith("fresh_initialization.json") for name in artifacts)
    else:
        actual_model, hashes, result = compiled(args, "control", parent, [], [], set(), {"seed": m.SEEDS[2]}, {}, "scalar-device")
        assert actual_model is model and hashes == []
        assert result["optimizer_updates"] == result["examples_seen"] == 0 and result["completed_fixed_endpoint"] is False
        assert result["DEV_or_HOLDOUT_read"] is False and result["checkpoints"] == []
        start = next(value for name, value in artifacts.items() if name.endswith("fresh_initialization.json"))
        assert start["fresh_optimizer_state_empty"] is True and start["optimizer_updates_before_fit"] == 0
    assert seed_calls == [("torch", m.SEEDS[2]), ("cuda-stub", m.SEEDS[2]), ("numpy-stub", m.SEEDS[2]), ("python", m.SEEDS[2])]


def main_fixture(m, tmp_path, behavior):
    m.AUTHORITY_DIRECTORY = tmp_path / "phase-authorizations"
    m.AUTHORITY_DIRECTORY.mkdir()
    args = SimpleNamespace(mode="fit-one", seed=m.SEEDS[0], output=tmp_path / "seed", loader_workers=2,
        authorization=m.authorization_path("fit-one"), authorization_sha256=None)
    proposed = {"sources": {"scalar.py": "d" * 64}}
    value = authority(m, proposed); payload = json.dumps(value).encode(); args.authorization.write_bytes(payload)
    args.authorization_sha256 = digest_bytes(payload)
    m.parser = lambda: SimpleNamespace(parse_args=lambda _: args); m.contract = lambda: proposed
    m.regular = lambda path, _: path
    initialized = []
    m.torch = SimpleNamespace(cuda=SimpleNamespace(is_initialized=lambda: False), set_num_threads=lambda _: None,
        device=lambda value: value, use_deterministic_algorithms=lambda _: None,
        backends=SimpleNamespace(cudnn=SimpleNamespace(benchmark=None, deterministic=None, allow_tf32=None),
            cuda=SimpleNamespace(matmul=SimpleNamespace(allow_tf32=None))))
    parent = SimpleNamespace(state_dict=lambda: {})
    declaration = {"immutable_input_sha256": {}, "common_initial101_sha256": "a" * 64,
        "frozen47_sha256": "b" * 64, "TRAIN_receipts": {"role": "TRAIN"}}
    def prepare(*_):
        if behavior == "early_failure": raise ValueError("scalar admission failure")
        return parent, (None, None, None), {}, [], declaration
    m.prepare = prepare; m.replay = SimpleNamespace(storage_reserve=lambda _: None)
    m.raw_parent.gpu0_lease = lambda: nullcontext()
    m.gpu_revision.gpu_preflight = lambda: initialized.append("GPU0-preflight-scalar") or {"physical_index": 0}
    m.os = SimpleNamespace(environ={"CUBLAS_WORKSPACE_CONFIG": ":4096:8"}, getpid=lambda: 123)
    m.pair.unique_json_object = lambda pairs: dict(pairs)
    m.pair.verify_training_images = lambda _: {"source_count": 1}; m.pair.verify_inputs = lambda *_: None
    m.pair.load_train = lambda _: (None, None, None, None, {}, {"role": "TRAIN"})
    m.pair.ConcatDataset = lambda _: []; m.pair.curved_indices = lambda _: []
    def fit(*_):
        if behavior == "fit_failure": raise ValueError("retained scalar optimizer failure")
        return parent, ["c" * 64] * 123, {"completed_fixed_endpoint": False, "optimizer_updates": 123, "checkpoints": []}
    m.train_function = lambda _: fit
    m.original.freeze_inputs = lambda _: None; m.original.frozen_fingerprint = lambda _: "b" * 64
    m.state_sha = lambda _: "a" * 64
    artifacts = {}
    m.write_new = lambda path, value: artifacts.update({Path(path).name: copy.deepcopy(value)})
    return args, artifacts, initialized


@pytest.mark.parametrize("behavior", ["early_failure", "fit_failure", "partial"])
def test_public_main_preserves_partial_or_failure_without_retry_or_fake_seed_completion(m, tmp_path, behavior):
    args, artifacts, initialized = main_fixture(m, tmp_path, behavior)
    if behavior != "partial":
        with pytest.raises(ValueError): m.main([])
    else:
        assert m.main([]) == 2
        assert artifacts["summary.json"]["fit"]["optimizer_updates"] == 123
        assert len(artifacts["summary.json"]["actual_batch_sha256"]) == 123
        assert artifacts["summary.json"]["status"] == "INCOMPLETE_SEED_ALL_TAILS_RETAINED"
    if behavior == "early_failure":
        assert artifacts == {} and initialized == [] and not args.output.exists()
        # HH_261005 - The pre-output failure must remain with ROOT's caller, never be invented as a successful local attempt.
        assert "failure.json" not in artifacts
    elif behavior == "fit_failure":
        assert "declaration.json" in artifacts and "started.json" in artifacts and "failure.json" in artifacts
        assert artifacts["failure.json"]["failed_run_retained"] is True
        assert artifacts["failure.json"]["automatic_retry"] is False
    if initialized: assert initialized == ["GPU0-preflight-scalar"]
    assert all(value.get("evaluation_only_payloads_opened") is not True for value in artifacts.values())


def metadata_fixture(m):
    m.original.PARENT_CHECKPOINT_SHA = "1" * 64; m.original.PARENT_NPZ_SHA = "2" * 64
    m.original.PARENT_METADATA_SHA = "3" * 64; m.original.PARENT_DECLARATION_SHA = "4" * 64
    declaration = dict(seed=m.SEEDS[0], source_sha256={"scalar.py": "5" * 64},
        training_dataset_fingerprint_sha256="6" * 64, frozen47_sha256="7" * 64)
    description = dict(schema=m.EXPORT_SCHEMA, training_schema=m.SCHEMA, seed=m.SEEDS[0],
        optimizer_updates=6000, additional_optimizer_updates=6000, maximum_optimizer_updates=6000,
        known_total_optimizer_updates=12200, parent_optimizer_updates=3000, parent_known_total_optimizer_updates=6200,
        examples_seen=24000, effective_batch_size=4, recovery_examples_seen=12000, v4_examples_seen=6000, legacy_examples_seen=6000,
        parent_checkpoint_sha256="1" * 64, parent_npz_sha256="2" * 64, parent_metadata_sha256="3" * 64,
        parent_declaration_sha256="4" * 64, checkpoint_sha256="8" * 64, npz_sha256="9" * 64,
        declaration_sha256="a" * 64, source_sha256=declaration["source_sha256"],
        training_dataset_fingerprint_sha256="6" * 64, frozen_state_sha256="7" * 64,
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, trainable_parameter_count=2277827,
        training_only=True, simulation_only=True, model_promoted=False, vehicle_control_approved=False,
        inference_loader_authorized=False, DEV_or_HOLDOUT_or_TEST_payload_loaded=False,
        common_pretrained_lineage_shared=True, independently_random_pretraining_claimed=False,
        three_seed_model_A_B_performed=False, stop_intent_trained=False)
    return description, declaration


def test_endpoint_metadata_is_warm3000_plus6000_not_good6000_serial_resume(m):
    description, declaration = metadata_fixture(m)
    result = m.validate_endpoint_metadata(description, declaration, "8" * 64, "9" * 64, "a" * 64)
    assert result["metadata_lineage_exact"] is True and result["fixed_endpoint_updates"] == 6000
    assert description["known_total_optimizer_updates"] == 6200 + 6000 == 12200
    assert description["parent_optimizer_updates"] == 3000


@pytest.mark.parametrize("key", ["schema", "training_schema", "seed", "optimizer_updates", "additional_optimizer_updates",
    "maximum_optimizer_updates", "known_total_optimizer_updates", "parent_optimizer_updates", "parent_known_total_optimizer_updates",
    "examples_seen", "effective_batch_size", "recovery_examples_seen", "v4_examples_seen", "legacy_examples_seen",
    "parent_checkpoint_sha256", "parent_npz_sha256", "parent_metadata_sha256", "parent_declaration_sha256",
    "checkpoint_sha256", "npz_sha256", "declaration_sha256", "source_sha256", "training_dataset_fingerprint_sha256",
    "frozen_state_sha256", "trainable_state_tensor_count", "frozen_state_tensor_count", "trainable_parameter_count",
    "training_only", "simulation_only", "model_promoted", "vehicle_control_approved", "inference_loader_authorized",
    "DEV_or_HOLDOUT_or_TEST_payload_loaded", "common_pretrained_lineage_shared", "independently_random_pretraining_claimed",
    "three_seed_model_A_B_performed", "stop_intent_trained"])
def test_endpoint_metadata_rejects_each_lineage_scope_and_source_field_change(m, key):
    description, declaration = metadata_fixture(m)
    old = description[key]
    description[key] = not old if type(old) is bool else old + 1 if type(old) is int else {} if isinstance(old, dict) else "changed"
    with pytest.raises(ValueError, match="lineage/scope"):
        m.validate_endpoint_metadata(description, declaration, "8" * 64, "9" * 64, "a" * 64)


def test_endpoint_metadata_rejects_float_integer_or_bool_integer_substitution(m):
    description, declaration = metadata_fixture(m)
    description["optimizer_updates"] = 6000.
    with pytest.raises(ValueError): m.validate_endpoint_metadata(description, declaration, "8" * 64, "9" * 64, "a" * 64)


class ScalarArray:
    def __init__(self, values=(0.,), dtype="float32", finite=True):
        self.values, self.dtype, self.finite, self.shape = list(values), dtype, finite, (len(values),)


def endpoint_fixture(m, tmp_path):
    # HH_261005 - Stand-ins test archive and scalar ABI control flow, never actual NPZ/Torch weights or numerical equivalence.
    description, declaration = metadata_fixture(m)
    m.digest = lambda path: digest_bytes(Path(path).read_bytes())
    output = tmp_path / "control"; output.mkdir(); (output / "declaration.json").write_bytes(b"sealed scalar declaration")
    result = dict(optimizer_updates=6000, examples_seen=24000, completed_fixed_endpoint=True, arm="control", checkpoints=[])
    for step in range(500, 6001, 500):
        checkpoint = output / "checkpoints" / ("step_%06d.pt" % step); checkpoint.parent.mkdir(exist_ok=True)
        checkpoint.write_bytes(("scalar checkpoint %d" % step).encode())
        folder = output / "exports" / ("step_%06d" % step); folder.mkdir(parents=True)
        (folder / "model_state.npz").write_bytes(("scalar archive %d" % step).encode())
        (folder / "metadata.json").write_bytes(("scalar metadata %d" % step).encode())
        result["checkpoints"].append(dict(optimizer_updates=step, sha256=m.digest(checkpoint),
            export_npz_sha256=m.digest(folder / "model_state.npz"), export_metadata_sha256=m.digest(folder / "metadata.json")))
    last = result["checkpoints"][-1]
    description.update(checkpoint_sha256=last["sha256"], npz_sha256=last["export_npz_sha256"],
        declaration_sha256=m.digest(output / "declaration.json"))
    names = ["learn_%d" % index for index in range(54)] + ["fixed_%d" % index for index in range(47)]
    values = {name: ScalarArray() for name in names}
    description["state_tensor_shapes"] = {name: [1] for name in names}
    description["state_tensor_dtypes"] = {name: "float32" for name in names}
    class Archive:
        files = names
        def __enter__(self): return self
        def __exit__(self, *_): return False
        def __getitem__(self, name): return values[name]
    def tensor():
        value = SimpleNamespace(shape=(1,)); value.detach = lambda: value; value.cpu = lambda: value; value.numpy = lambda: [0.]
        return value
    parent = SimpleNamespace(state_dict=lambda: {name: tensor() for name in names})
    m.original.TRAINABLE_PREFIXES = ("learn_",)
    m.pair.read = lambda _: copy.deepcopy(description)
    m.np = SimpleNamespace(load=lambda *args, **kwargs: Archive(), float32="float32",
        isfinite=lambda value: SimpleNamespace(all=lambda: value.finite), array_equal=lambda value, expected: value.values == expected)
    audited = []
    m.replay = SimpleNamespace(verify_sources=lambda *args: audited.append(args))
    return output, result, parent, declaration, values, description, Archive, audited


def test_all12_actual_original_saver_keys_and_full101_frozen47_gates(m, tmp_path):
    output, result, parent, declaration, _, _, _, audited = endpoint_fixture(m, tmp_path)
    value = m.verify_final_endpoint(output, result, parent, declaration, {"scalar.py": "5" * 64})
    assert value["all101_numeric_finite"] is True and value["frozen47_exact"] is True
    assert len(result["checkpoints"]) == 12 and len(audited) == 1
    assert value["metadata_sha256"] == result["checkpoints"][-1]["export_metadata_sha256"]


@pytest.mark.parametrize("fault", ["missing_checkpoint", "old_pair_receipt_key", "checkpoint_bytes", "metadata_bytes",
    "duplicate_checkpoint", "missing_state", "dtype", "nonfinite", "shape", "frozen_value"])
def test_final_endpoint_refuses_missing_failed_or_altered_original_receipts_and_numeric_abi(m, tmp_path, fault):
    output, result, parent, declaration, values, description, archive, audited = endpoint_fixture(m, tmp_path)
    if fault == "missing_checkpoint": result["checkpoints"].pop()
    elif fault == "old_pair_receipt_key": result["checkpoints"][0]["checkpoint_sha256"] = result["checkpoints"][0].pop("sha256")
    elif fault == "checkpoint_bytes": (output / "checkpoints/step_000500.pt").write_bytes(b"changed retained prefix")
    elif fault == "metadata_bytes": (output / "exports/step_003000/metadata.json").write_bytes(b"changed non-final retained metadata")
    elif fault == "duplicate_checkpoint": result["checkpoints"][1] = result["checkpoints"][0]
    elif fault == "missing_state": archive.files = archive.files[:-1]
    elif fault == "dtype": values["fixed_0"].dtype = "float64"
    elif fault == "nonfinite": values["learn_0"].finite = False
    elif fault == "shape": description["state_tensor_shapes"]["fixed_0"] = [2]
    else: values["fixed_0"].values = [1.]
    with pytest.raises((ValueError, KeyError)):
        m.verify_final_endpoint(output, result, parent, declaration, {"scalar.py": "5" * 64})
    assert audited == []


@pytest.mark.parametrize("fault", ["TRAIN_receipts", "TRAIN_strata", "images", "source"])
def test_public_main_retains_failure_if_postfit_original_train_or_source_changes(m, tmp_path, fault):
    _, artifacts, _ = main_fixture(m, tmp_path, "partial")
    if fault == "TRAIN_receipts": m.pair.load_train = lambda _: (None, None, None, None, {}, {"changed": True})
    elif fault == "TRAIN_strata": m.pair.load_train = lambda _: (None, None, None, None, {"changed": []}, {"role": "TRAIN"})
    elif fault == "images":
        calls = []
        m.pair.verify_training_images = lambda _: calls.append(1) or {"source_count": len(calls)}
    else:
        calls = []
        def verify(*_):
            calls.append(1)
            if len(calls) == 2: raise ValueError("scalar source changed after fitting")
        m.pair.verify_inputs = verify
    with pytest.raises(ValueError): m.main([])
    assert artifacts["failure.json"]["failed_run_retained"] is True
    assert artifacts["failure.json"]["automatic_retry"] is False and "summary.json" not in artifacts


def test_v2_inverse_ast_changes_only_identity_and_phase_authority_routing():
    # HH_261005 - Restore only the reviewed authority split and identities; every numerical/training/cleanup AST remains V1.
    old = ast.parse(SOURCE_V1.read_text()); new = ast.parse(SOURCE.read_text())
    old_authority = next(node for node in old.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "AUTHORITY" for target in node.targets))
    new.body[0] = copy.deepcopy(old.body[0])
    new.body = [node for node in new.body if not isinstance(node, ast.FunctionDef) or node.name != "authorization_path"]
    assignments = 0
    for index, node in enumerate(new.body):
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "AUTHORITY_DIRECTORY" for target in node.targets):
            new.body[index] = copy.deepcopy(old_authority); assignments += 1
    assert assignments == 1
    identities = {
        "portable_e2e.temporal_route_joint_three_seed_train_only.20261005.v2": "portable_e2e.temporal_route_joint_three_seed_train_only.20261005.v1",
        "temporal_route_joint_three_seed_train_only_20261005_v2": "temporal_route_joint_three_seed_train_only_20261005_v1",
        "learning/tests/test_temporal_route_joint_three_seed_train_only_v2_20261005.py": "learning/tests/test_temporal_route_joint_three_seed_train_only_v1_20261005.py",
        "one exact prospective ROOT phase authority path required": "one exact prospective ROOT authority path required",
        "exact ROOT phase authority path and external SHA required": "exact ROOT authority path and external SHA required",
    }
    class Restore(ast.NodeTransformer):
        routes = 0
        removed_fields = 0
        def visit_Constant(self, node):
            if isinstance(node.value, str) and node.value in identities: node.value = identities[node.value]
            return node
        def visit_Call(self, node):
            if isinstance(node.func, ast.Name) and node.func.id == "dict":
                retained = [value for value in node.keywords if value.arg not in ("authority_paths", "grant_transition")]
                self.removed_fields += len(node.keywords) - len(retained); node.keywords = retained
            if isinstance(node.func, ast.Name) and node.func.id == "authorization_path":
                assert len(node.args) == 1 and ast.unparse(node.args[0]) == "args.mode"
                self.routes += 1
                return ast.copy_location(ast.Name(id="AUTHORITY", ctx=ast.Load()), node)
            return self.generic_visit(node)
    inverse = Restore(); restored = inverse.visit(new)
    assert inverse.routes == 3 and inverse.removed_fields == 2
    assert ast.dump(restored) == ast.dump(old)
    for name in ("validate_authority", "fixed_schedule", "verify_schedule", "verify_schedules", "validate_fit_start",
        "state_sha", "checkpoint_saver", "validate_endpoint_metadata", "verify_final_endpoint", "train_function"):
        assert ast.dump(function_node(SOURCE, name)) == ast.dump(function_node(SOURCE_V1, name))


def test_cpu_and_gpu_grants_have_distinct_fixed_paths_and_truthful_contract(m):
    cpu = m.authorization_path("validate-only"); gpu = m.authorization_path("fit-one")
    assert cpu.name == "ROOT_CPU_TRAIN_only_authorization.json"
    assert gpu.name == "ROOT_GPU0_TRAIN_only_authorization.json"
    assert cpu.parent == gpu.parent == m.AUTHORITY_DIRECTORY and cpu != gpu
    assert not hasattr(m, "AUTHORITY")
    m.source_snapshot = lambda: {"scalar-source.py": "a" * 64}
    value = m.contract()
    assert value["authority_paths"] == {"validate-only": str(cpu), "fit-one": str(gpu)}
    assert "immutable CPU grant retained" in value["grant_transition"]


@pytest.mark.parametrize("mode", ["contract", "fit", "cpu", None, True])
def test_unknown_phase_cannot_choose_an_authority_file(m, mode):
    with pytest.raises(ValueError, match="authority phase"):
        m.authorization_path(mode)


@pytest.mark.parametrize("mode,wrong_phase", [("fit-one", "validate-only"), ("validate-only", "fit-one")])
def test_wrong_phase_path_rejected_before_authority_read_or_loader(m, tmp_path, mode, wrong_phase):
    args, _, loaded, _, _, _ = prepare_fixture(m, tmp_path)
    args.mode = mode; args.authorization = m.authorization_path(wrong_phase)
    with pytest.raises(ValueError, match="phase authority path"):
        m.prepare(args, {"sources": {}}, {})
    assert loaded == []
    m.parser = lambda: SimpleNamespace(parse_args=lambda _: args)
    m.contract = lambda: {"scalar": True}
    m.regular = lambda *_: pytest.fail("wrong phase path must not read any authority or payload")
    with pytest.raises(ValueError, match="phase authority path"):
        m.main([])


def test_original_cpu_grant_bytes_remain_immutable_during_separate_gpu_grant_flow(m, tmp_path):
    # HH_261005 - Synthetic public orchestration exercises both grants without any actual model, CUDA or optimizer work.
    args, artifacts, initialized = main_fixture(m, tmp_path, "partial")
    gpu = m.authorization_path("fit-one"); gpu_before = gpu.read_bytes()
    proposed = m.contract(); cpu = m.authorization_path("validate-only")
    cpu_payload = json.dumps(authority(m, proposed, modes=("validate-only",))).encode()
    with cpu.open("xb") as stream: stream.write(cpu_payload)
    args.mode = "validate-only"; args.authorization = cpu; args.authorization_sha256 = digest_bytes(cpu_payload)
    assert m.main([]) == 0 and initialized == [] and artifacts == {}
    assert cpu.read_bytes() == cpu_payload and gpu.read_bytes() == gpu_before
    args.mode = "fit-one"; args.authorization = gpu; args.authorization_sha256 = digest_bytes(gpu_before)
    assert m.main([]) == 2 and initialized == ["GPU0-preflight-scalar"]
    assert cpu.read_bytes() == cpu_payload and gpu.read_bytes() == gpu_before
    assert artifacts["summary.json"]["status"] == "INCOMPLETE_SEED_ALL_TAILS_RETAINED"


def test_prepare_seals_only_the_current_phase_grant_as_an_immutable_input(m, tmp_path):
    args, _, _, _, _, _ = prepare_fixture(m, tmp_path)
    _, _, _, _, declaration = m.prepare(args, {"sources": {}}, {})
    pins = declaration["immutable_input_sha256"]
    assert pins[str(m.authorization_path("validate-only"))] == args.authorization_sha256
    assert str(m.authorization_path("fit-one")) not in pins
    args.mode = "fit-one"; args.authorization = m.authorization_path("fit-one")
    with args.authorization.open("xb") as stream: stream.write(b"scalar distinct GPU0 grant")
    args.authorization_sha256 = m.digest(args.authorization)
    g, strata = groups(); reports = m.verify_schedules({seed: m.fixed_schedule(g, strata, seed) for seed in m.SEEDS}, g, strata)
    seals = {seed: value["schedule_sha256"] for seed, value in reports.items()}
    _, _, _, _, declaration = m.prepare(args, {"sources": {}}, {"schedule_sha256": seals})
    pins = declaration["immutable_input_sha256"]
    assert pins[str(args.authorization)] == args.authorization_sha256
    assert str(m.authorization_path("validate-only")) not in pins
