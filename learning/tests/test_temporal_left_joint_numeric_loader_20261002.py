"""HH_261002 - Synthetic fixed20k journals/101 numeric states; no real weights/data/CUDA/control."""
import copy
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from portable_e2e import temporal_left_joint_numeric_loader_20261002 as loader
from scripts.e2e import temporal_left_joint_runtime_contract_20261002 as runtime
from scripts.e2e import train_temporal_left_joint_20261002 as trainer


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False))


def payload(rows):
    return b"\n".join(json.dumps(row, allow_nan=False).encode() for row in rows) + b"\n"


class FakeModel(torch.nn.Module):
    def __init__(self, config=None, recovery_config=None):
        super().__init__()
        self.config = config or SimpleNamespace(to_dict=lambda: {"synthetic101": True})
        self.recovery_config = recovery_config or SimpleNamespace(to_dict=lambda: {"syntheticRecovery": True})
        for prefix, count in (("speed_head", 54), ("intent_head", 47)):
            module = torch.nn.Module()
            for i in range(count):
                module.register_parameter("p%02d" % i, torch.nn.Parameter(torch.tensor([i / 100.], dtype=torch.float32)))
            self.add_module(prefix, module)


@pytest.fixture(scope="module")
def protocol():
    # HH_261002 - Contract is a source-only census, not a TRAIN payload read.
    return trainer.contract(loader.LEFT_CONVERSION_SHA)


@pytest.fixture(scope="module")
def history():
    rows = []
    for i in range(20000):
        step = i + 1
        ids = [i % 2718, 2718 + i % 327, 3045 + i % 2787, 5832 + i % 347]
        rows.append(dict(optimizer_updates=step, examples_seen=4 * step, indices=ids,
            sample_ids=["synthetic_TRAIN_%04d" % v for v in ids], batch_sha256="b" * 64,
            learning_rates={g: loader.learning_rate(step, g) for g in ("shared", "heads")},
            legacy_curved=bool(step % 2), recovery_focus=bool(step % 2),
            loss=.5, preclip_gradient_norm=.2, elapsed_fit_seconds=step / 10.))
    return rows


def description(proposed, parent):
    state = parent.state_dict()
    names = loader.parent_api.trainable_names(state)
    cohorts = dict(recovery=dict(split="train", sample_count=2718, episode_count=9),
        v4=dict(split="train", statistics=dict(anchor_count=327)),
        legacy=dict(split="train", sample_count=2787, episode_count=8),
        LEFT=dict(split="train", sample_count=347, episode_count=1, conversion_sha256=loader.LEFT_CONVERSION_SHA))
    return dict(trainer.original.scope_fields(), schema=loader.EXPORT_SCHEMA, training_schema=loader.TRAINING_SCHEMA,
        architecture_schema=loader.parent_api.old.ARCHITECTURE_SCHEMA, model_id=loader.parent_api.old.MODEL_ID,
        protocol=proposed, source_sha256=proposed["sources"], optimizer_updates=20000, additional_optimizer_updates=20000,
        maximum_optimizer_updates=20000, effective_batch_size=4, examples_seen=80000,
        recovery_examples_seen=20000, v4_examples_seen=20000, legacy_examples_seen=20000, LEFT_examples_seen=20000,
        parent_optimizer_updates=6000, parent_known_total_optimizer_updates=12200, known_total_optimizer_updates=32200,
        training_episode_count=19, recovery_training_episode_count=9, replay_training_episode_count=10,
        LEFT_training_episode_count=1, holdout_episode_count=2, seed=2026100201,
        parent_npz_sha256=loader.PARENT["weights_sha256"], parent_metadata_sha256=loader.PARENT["metadata_sha256"],
        parent_checkpoint_sha256=loader.PARENT["checkpoint_sha256"], parent_declaration_sha256=loader.PARENT["declaration_sha256"],
        parent_history_sha256=loader.PARENT_HISTORY_SHA, current_training_purpose="actual_LEFT347_DATA_and_schedule_joint_continuation",
        model_config=parent.config.to_dict(), recovery_config=parent.recovery_config.to_dict(),
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, trainable_parameter_count=2277827,
        trainable_state_names=names, frozen_state_names=sorted(set(state) - set(names)),
        state_tensor_shapes={k: list(v.shape) for k, v in state.items()}, state_tensor_dtypes={k: "float32" for k in state},
        learning_rate=loader.learning_rate_config(), batch_domains=loader.BATCH_DOMAINS, cohorts=cohorts,
        frozen_state_sha256=loader.parent_api.frozen_fingerprint(state), training_dataset_fingerprint_sha256=loader.json_sha(cohorts),
        deferred_holdout_conversion_sha256=loader.HOLDOUT_CONVERSION_SHA, holdout_payload_loaded=False,
        authorization_sha256="a" * 64, runtime_loader_implemented=False,
        numerical_revision=dict(kernel_id=loader.parent_api.old.KERNEL_ID, strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False))


@pytest.fixture
def full(tmp_path, monkeypatch, protocol, history):
    proposed = copy.deepcopy(protocol)
    parent = FakeModel()
    run, parent_run = tmp_path / "LEFT20k", tmp_path / "GOOD6000"
    current = parent_run / "exports/step_006000"
    current.mkdir(parents=True)
    assets = dict(current_parent_weights=current / "model_state.npz", current_parent_metadata=current / "metadata.json",
        warm_parent_weights=tmp_path / "warm.npz", warm_parent_metadata=tmp_path / "warm.json",
        backbone_parent_weights=tmp_path / "backbone.npz", backbone_parent_metadata=tmp_path / "backbone.json")
    for path in assets.values():
        path.write_bytes(b"synthetic ancestor pin, never deserialized")
    write(assets["warm_parent_metadata"], dict(parent_npz_sha256=digest(assets["backbone_parent_weights"]),
        parent_metadata_sha256=digest(assets["backbone_parent_metadata"])))
    write(parent_run / "declaration.json", {"synthetic_completed_parent": True})
    (parent_run / "history.jsonl").write_bytes(b"synthetic externally pinned GOOD6000 journal")
    (parent_run / "checkpoints").mkdir()
    (parent_run / "checkpoints/step_006000.pt").write_bytes(b"parent PT: hash only")
    parent_pins = dict(weights_sha256=digest(assets["current_parent_weights"]), metadata_sha256=digest(assets["current_parent_metadata"]),
        checkpoint_sha256=digest(parent_run / "checkpoints/step_006000.pt"), declaration_sha256=digest(parent_run / "declaration.json"))
    monkeypatch.setattr(loader, "PARENT", parent_pins)
    monkeypatch.setattr(loader, "PARENT_HISTORY_SHA", digest(parent_run / "history.jsonl"))
    proposed.update(actual_parent_final6000=parent_pins, actual_parent_history_sha256=loader.PARENT_HISTORY_SHA)
    parent._research_temporal_export_metadata = dict(source_sha256={k: v for k, v in proposed["sources"].items()
        if k not in loader.parent_api.ADDED_TRAINING_SOURCES and k != loader.TRAINER_PATH},
        parent_npz_sha256=digest(assets["warm_parent_weights"]), parent_metadata_sha256=digest(assets["warm_parent_metadata"]))
    monkeypatch.setattr(loader.parent_api, "load_current_parent", lambda *a, **k: parent)
    monkeypatch.setattr(loader.parent_api.old, "RouteRecoveryTemporalModelV3", FakeModel)
    monkeypatch.setattr(torch, "load", lambda *a, **k: (_ for _ in ()).throw(AssertionError("Torch pickle forbidden")))
    d = description(proposed, parent)
    export = run / "exports/step_020000"
    export.mkdir(parents=True)
    (run / "checkpoints").mkdir()
    checkpoint = run / "checkpoints/step_020000.pt"
    checkpoint.write_bytes(b"final PT: hash only; not a pickle")
    arrays = {k: v.detach().numpy().copy() for k, v in parent.state_dict().items()}
    declaration = dict(d, schema=loader.TRAINING_SCHEMA, fixed_sample_ids=[r["sample_ids"] for r in history],
        fixed_sample_ids_sha256=loader.json_sha([r["sample_ids"] for r in history]),
        fixed_schedule_sha256=loader.json_sha([r["indices"] for r in history]))
    write(run / "declaration.json", declaration)
    (run / "history.jsonl").write_bytes(payload(history))
    for name in proposed["sources"]:
        archived = run / "source" / name
        archived.parent.mkdir(parents=True, exist_ok=True)
        archived.write_bytes((trainer.ROOT / name).read_bytes())
    fit = dict(optimizer_updates=20000, examples_seen=80000, completed_fixed_endpoint=True,
        DEV_or_HOLDOUT_read=False, model_promoted=False, checkpoints=[dict(optimizer_updates=i,
            checkpoint_sha256="b" * 64, npz_sha256="c" * 64, metadata_sha256="d" * 64) for i in range(1000, 20001, 1000)])
    summary = dict(schema=loader.TRAINING_SCHEMA, status="FIXED20000_ACTUAL_LEFT_JOINT_COMPLETE_NOT_PROMOTED", fit=fit,
        all71_original_screens_retained=True, full_LEFT347_retained=True, model_promoted=False, vehicle_control_approved=False,
        endpoint_diagnostic_screens=dict(diagnostic_screen_count=74, checks=[dict(passed=False)] * 74))
    authority = {}

    def seal():
        with (export / "model_state.npz").open("wb") as stream:
            np.savez(stream, **arrays)
        d.update(npz_sha256=digest(export / "model_state.npz"), checkpoint_sha256=digest(checkpoint),
            declaration_sha256=digest(run / "declaration.json"), history_sha256=digest(run / "history.jsonl"),
            fixed_sample_ids_sha256=declaration["fixed_sample_ids_sha256"], fixed_schedule_sha256=declaration["fixed_schedule_sha256"])
        write(export / "metadata.json", d)
        pins = dict(weights_sha256=d["npz_sha256"], metadata_sha256=digest(export / "metadata.json"),
            checkpoint_sha256=d["checkpoint_sha256"], declaration_sha256=d["declaration_sha256"])
        fit["checkpoints"][-1].update(npz_sha256=pins["weights_sha256"], metadata_sha256=pins["metadata_sha256"],
            checkpoint_sha256=pins["checkpoint_sha256"])
        write(run / "fit_result.json", fit)
        write(run / "summary.json", summary)
        write(run / "evaluation_opened_after_final.json", dict(endpoint=dict(fit["checkpoints"][-1],
            all101_numeric_finite=True, frozen47_byte_identical=True, all20000_actual_journal_rows_verified=True), checkpoint_selection=False))
        authority.update(schema=loader.AUTHORITY_SCHEMA, mode="RESEARCH_ONLY", run=str(run), parent_run=str(parent_run),
            fixed_endpoint_completed=True, model_promoted=False, vehicle_control_approved=False,
            parent=parent_pins, parent_history_sha256=loader.PARENT_HISTORY_SHA, source_sha256=proposed["sources"],
            new_LEFT_conversion_sha256=loader.LEFT_CONVERSION_SHA, training_authorization_sha256=d["authorization_sha256"],
            protocol_sha256=loader.json_sha(proposed), endpoint=pins, declaration_sha256=d["declaration_sha256"],
            history_sha256=d["history_sha256"], summary_sha256=digest(run / "summary.json"), fit_result_sha256=digest(run / "fit_result.json"),
            evaluation_boundary_sha256=digest(run / "evaluation_opened_after_final.json"))
        write(tmp_path / "authority.json", authority)

    def invoke(changes=None):
        pins = authority["endpoint"]
        binding = runtime.owner_binding(pins, digest(tmp_path / "authority.json"), owner_id="synthetic-no-actuation")
        if changes:
            binding.update(changes)
        return loader.load_left_joint_research_npz(export / "model_state.npz", export / "metadata.json",
            weights_sha256=d["npz_sha256"], metadata_sha256=digest(export / "metadata.json"), **assets,
            completed_run_authority=tmp_path / "authority.json", completed_run_authority_sha256=digest(tmp_path / "authority.json"),
            research_owner_binding=binding, source_root=trainer.ROOT)

    seal()
    return SimpleNamespace(parent=parent, run=run, parent_run=parent_run, d=d, arrays=arrays, declaration=declaration,
        summary=summary, fit=fit, authority=authority, authority_path=tmp_path / "authority.json", assets=assets, seal=seal, invoke=invoke)


def test_complete_actual_synthetic20k_counts_and347_fair_cycles(history):
    declaration = dict(fixed_sample_ids=[r["sample_ids"] for r in history], fixed_schedule_sha256=loader.json_sha([r["indices"] for r in history]))
    d = dict(fixed_sample_ids_sha256=loader.json_sha(declaration["fixed_sample_ids"]), fixed_schedule_sha256=declaration["fixed_schedule_sha256"])
    result = loader.verify_history(payload(history), declaration, d)
    assert result["actual_updates"] == 20000 and result["actual_examples_seen"] == 80000
    assert sorted(result["all347_fair_exposure_counts"]) == [57] * 126 + [58] * 221


@pytest.mark.parametrize("key,value", [("optimizer_updates", True), ("optimizer_updates", 2), ("examples_seen", 8),
    ("sample_ids", ["foreign"] * 4), ("indices", [0, 2718, 3045, 6179]), ("indices", [0, 2718, 3044, 5832]),
    ("learning_rates", dict(shared=1e-5, heads=3e-5)), ("batch_sha256", "bad"), ("legacy_curved", False),
    ("recovery_focus", False), ("loss", -1), ("preclip_gradient_norm", True)])
def test_actual_history_cannot_be_replaced_by_completed_boolean(history, key, value):
    rows = list(history)
    rows[0] = dict(rows[0], **{key: value})
    declaration = dict(fixed_sample_ids=[r["sample_ids"] for r in history], fixed_schedule_sha256=loader.json_sha([r["indices"] for r in history]))
    d = dict(fixed_sample_ids_sha256=loader.json_sha(declaration["fixed_sample_ids"]), fixed_schedule_sha256=declaration["fixed_schedule_sha256"])
    with pytest.raises(ValueError):
        loader.verify_history(payload(rows), declaration, d)


@pytest.mark.parametrize("text", [b'NaN', b'Infinity', b'1e400'])
def test_unknown_journal_metric_nonfinite_still_rejects(history, text):
    raw = payload(history).replace(b'"loss": 0.5', b'"unclaimed_metric": ' + text + b', "loss": 0.5', 1)
    with pytest.raises(ValueError, match="nonfinite"):
        loader.verify_history(raw, {}, {})


def test_numeric101_load_preserves_failed_screens_and_immutable_export(full):
    before = {str(p): digest(p) for p in full.run.rglob("*") if p.is_file()}
    model = full.invoke()
    assert len(model.state_dict()) == 101 and not model.training and not any(p.requires_grad for p in model.parameters())
    assert all(torch.equal(model.state_dict()[k], v) for k, v in full.parent.state_dict().items())
    assert model._research_recovery_numeric_loader_schema == loader.SCHEMA
    assert model._research_recovery_simulator_control_authorized is False
    assert model._research_temporal_export_metadata["known_total_optimizer_updates"] == 32200
    assert model._research_temporal_export_metadata["inference_loader_authorized"] is False
    assert not any(r["passed"] for r in full.summary["endpoint_diagnostic_screens"]["checks"])
    assert before == {str(p): digest(p) for p in full.run.rglob("*") if p.is_file()}


def test_realistic_fixed80k_declaration_can_exceed_old4MiB_limit(full):
    # HH_261002 - Root measured the actual declaration at7.64MB; this is synthetic text, not genuine TRAIN tokens.
    full.declaration["synthetic_provenance_padding"] = "x" * (5 * 1024**2)
    write(full.run / "declaration.json", full.declaration)
    full.seal()
    assert 4 * 1024**2 < (full.run / "declaration.json").stat().st_size < 16 * 1024**2
    assert full.invoke()._research_left_actual_journal_evidence["actual_updates"] == 20000


def test_missing_journal_update_or_resealed_foreign_schedule_rejects(full, history):
    (full.run / "history.jsonl").write_bytes(payload(history[:-1]))
    full.seal()
    with pytest.raises(ValueError, match="all20k"):
        full.invoke()


def test_mutated_source_archive_detected_even_with_completed_summary(full):
    (full.run / "source" / loader.TRAINER_PATH).write_bytes(b"unreviewed source replacement")
    with pytest.raises(ValueError, match="SHA differs"):
        full.invoke()


def test_missing_final_numeric_endpoint_never_replaced_by_authority_boolean(full):
    (full.run / "exports/step_020000/model_state.npz").unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        full.invoke()


@pytest.mark.parametrize("change", ["frozen", "nan", "dtype", "shape", "missing"])
def test_full101_numeric_abi_and47_fixed_parent_bytes(full, change):
    if change == "frozen":
        full.arrays["intent_head.p00"] += .1
    elif change == "nan":
        full.arrays["speed_head.p00"][0] = np.nan
    elif change == "dtype":
        full.arrays["speed_head.p00"] = full.arrays["speed_head.p00"].astype(np.float64)
    elif change == "shape":
        full.arrays["speed_head.p00"] = np.zeros(2, dtype=np.float32)
    else:
        full.arrays.pop("speed_head.p00")
    full.seal()
    with pytest.raises(ValueError):
        full.invoke()


@pytest.mark.parametrize("key,value", [("schema", loader.parent_api.EXPORT_SCHEMA), ("optimizer_updates", 6000),
    ("optimizer_updates", 20000.), ("known_total_optimizer_updates", 18200), ("training_episode_count", 18),
    ("LEFT_examples_seen", 19999), ("frozen_state_tensor_count", 97), ("model_promoted", True),
    ("runtime_loader_implemented", True), ("inference_loader_authorized", True), ("stop_intent_trained", True),
    ("holdout_payload_loaded", True)])
def test_scope_rejects_old_or_falsified_release(full, key, value):
    full.d[key] = value
    with pytest.raises(ValueError):
        loader.verify_scope(full.d)


def test_exact30_source_and_actualLEFT347_only(full):
    d = copy.deepcopy(full.d)
    d["source_sha256"].pop(loader.TRAINER_PATH)
    with pytest.raises(ValueError):
        loader.verify_scope(d)
    d = copy.deepcopy(full.d)
    d["cohorts"]["LEFT"]["sample_count"] = 346
    with pytest.raises(ValueError):
        loader.verify_scope(d)
    d = copy.deepcopy(full.d)
    d["protocol"]["gpu_preflight_revision"]["physical_index"] = False
    with pytest.raises(ValueError):
        loader.verify_scope(d)
    d = copy.deepcopy(full.d)
    d["cohorts"]["LEFT"]["split"] = "val"
    with pytest.raises(ValueError):
        loader.verify_scope(d)


def test_completion_requires_actual_final20k_files_and_receipts(full):
    full.fit["optimizer_updates"] = 19999
    full.seal()
    with pytest.raises(ValueError, match="checkpoint receipts"):
        full.invoke()


@pytest.mark.parametrize("role", ["current_parent_weights", "current_parent_metadata", "warm_parent_weights", "warm_parent_metadata",
    "backbone_parent_weights", "backbone_parent_metadata", "parent_history"])
def test_ancestor_or_actual6000_history_mutation_midcall_rejects(full, monkeypatch, role):
    old = loader.parent_api.old.RouteRecoveryTemporalModelV3
    def constructor(*args, **kwargs):
        model = old(*args, **kwargs)
        path = full.parent_run / "history.jsonl" if role == "parent_history" else full.assets[role]
        path.write_bytes(b"mutated after initial ancestor read")
        return model
    monkeypatch.setattr(loader.parent_api.old, "RouteRecoveryTemporalModelV3", constructor)
    with pytest.raises(ValueError, match="SHA differs"):
        full.invoke()


def test_offline_never_actuates_and_physical_plan_is_separate(full):
    with pytest.raises(ValueError):
        full.invoke(dict(simulator_control_authorized=True))
    with pytest.raises(ValueError):
        full.invoke(dict(purpose="owned_carla_research_only", simulator_control_authorized=True, simulation_only=True))
    model = full.invoke(dict(purpose="owned_carla_research_only", simulator_control_authorized=True, simulation_only=True,
        owner_plan_sha256="e" * 64))
    assert model._research_recovery_simulator_control_authorized is True
    assert model._research_temporal_export_metadata["vehicle_control_approved"] is False


@pytest.mark.parametrize("field", ["run", "parent_run"])
def test_absolute_non_symlink_authority_paths_only(full, tmp_path, field):
    full.authority[field] = "relative-run"
    write(full.authority_path, full.authority)
    with pytest.raises(ValueError, match="absolute"):
        full.invoke()
    linked = tmp_path / "alias"
    linked.symlink_to(full.run, target_is_directory=True)
    full.authority[field] = str(linked)
    write(full.authority_path, full.authority)
    with pytest.raises(ValueError, match="non-symlink"):
        full.invoke()


def test_runtime_contract_and_binding_print_only_no_guessed_endpoint(capsys):
    assert runtime.main(["--mode", "contract"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["fixed_endpoint_updates"] == 20000 and result["known_total_updates"] == 32200
    assert result["optimization_or_inference_performed"] is False and result["source_count"] == 30
    with pytest.raises(ValueError):
        runtime.owner_binding(loader.PARENT, "a" * 64, owner_id="test", purpose="owned_carla_research_only")
    with pytest.raises(ValueError):
        runtime.main(["--mode", "contract", "--weights-sha256", "a" * 64])
