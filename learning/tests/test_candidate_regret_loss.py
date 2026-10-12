"""HH_260906 - Verify the optional detached cost-aware auxiliary without changing historical objectives or gates."""

from __future__ import annotations

import ast
import copy
import hashlib
import inspect
import json
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
import portable_e2e.losses as losses_module
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
import portable_e2e.train as training
import portable_e2e.evaluate as evaluation
import portable_e2e.audit_runtime as audit
import portable_e2e.runtime_weight_bundle as bundle
from portable_e2e.torch_dataset import Common10TorchDataset
from test_portable_e2e_model import _example, _small_model_config, _train_config
from test_portable_e2e_runtime_weight_bundle import _checkpoint_payload, _export


LEGACY_CONFIG = {
    "xy_weight": 1.0,
    "speed_weight": 0.2,
    "yaw_weight": 0.1,
    "kinematic_speed_weight": 0.05,
    "final_displacement_weight": 0.5,
    "candidate_score_weight": 0.1,
}


@pytest.fixture(autouse=True)
def single_thread_cpu():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def sample_inputs(dtype=torch.float64, *, masked=False, gradients=False):
    generator = torch.Generator().manual_seed(91)
    xy = torch.randn(2, 3, 64, 2, generator=generator, dtype=dtype).cumsum(2) * 0.1
    speed = torch.rand(2, 3, 64, generator=generator, dtype=dtype) * 3.0
    logits = torch.randn(2, 3, generator=generator, dtype=dtype)
    target = torch.randn(2, 64, 2, generator=generator, dtype=dtype).cumsum(1) * 0.1
    target_speed = torch.rand(2, 64, generator=generator, dtype=dtype) * 2.0
    valid = torch.ones(2, 64, dtype=torch.bool)
    if masked:
        valid[0, 17:] = False
        valid[1, 39:] = False
    for value in (xy, speed, logits):
        value.requires_grad_(gradients)
    yaw = torch.randn(2, 64, generator=generator, dtype=dtype) * 0.2
    return (xy, speed, logits, target, target_speed, valid), yaw


def legacy_loss_reference():
    # HH_260906 - Remove only the two explicit auxiliary branches and their local optional variable; pin every remaining historical AST node.
    node = ast.parse(inspect.getsource(trajectory_loss)).body[0]
    removed = []
    retained = []
    for statement in node.body:
        is_variable = isinstance(statement, ast.AnnAssign) and isinstance(statement.target, ast.Name) and statement.target.id == "candidate_regret_loss"
        is_guard = isinstance(statement, ast.If) and ast.dump(statement.test) in {
            ast.dump(ast.parse("cfg.candidate_regret_weight > 0.0", mode="eval").body),
            ast.dump(ast.parse("candidate_regret_loss is not None", mode="eval").body),
        }
        (removed if is_variable or is_guard else retained).append(statement)
    assert len(removed) == 3
    node.body = retained
    assert hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest() == "b7a82e2cfaa6c22f7f20310902cee635f801c0dee80613f8b1d76e569e58d85a"
    namespace = dict(losses_module.__dict__)
    exec(compile(ast.Module(body=[node], type_ignores=[]), "<frozen-legacy-loss>", "exec"), namespace)
    return namespace["trajectory_loss"]


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("masked", [False, True])
@pytest.mark.parametrize("with_yaw", [False, True])
def test_zero_weight_preserves_frozen_outputs_and_gradients_bitexact(dtype, masked, with_yaw):
    inputs, yaw = sample_inputs(dtype, masked=masked, gradients=True)
    kwargs = {"target_yaw": yaw} if with_yaw else {}
    expected = legacy_loss_reference()(*inputs, **kwargs)
    for config in (None, TrajectoryLossConfig(candidate_regret_weight=0.0)):
        actual = trajectory_loss(*inputs, config, **kwargs)
        assert actual.keys() == expected.keys()
        assert "candidate_regret_loss" not in actual
        assert all(torch.equal(actual[key], expected[key]) for key in actual)
        original_gradient = torch.autograd.grad(expected["loss"], inputs[:3], retain_graph=True)
        actual_gradient = torch.autograd.grad(actual["loss"], inputs[:3], retain_graph=True)
        assert all(torch.equal(a, b) for a, b in zip(original_gradient, actual_gradient))


@pytest.mark.parametrize("name,sha", [
    ("_validate_shapes", "e301b1921bf99d9c1cdbd8d8632d2fac1626edeb5b4409aff1eb7b156b47bfe2"),
    ("_last_valid_indices", "aab8f079bf99c3dbc8c7c97d4b56e480d57e4b685cad07f0582aa1f0eb1c4d44"),
])
def test_input_validation_and_mask_helpers_remain_source_identical(name, sha):
    assert hashlib.sha256(inspect.getsource(getattr(losses_module, name)).rstrip().encode()).hexdigest() == sha


def test_legacy_and_research_loss_contracts_are_canonical():
    assert TrajectoryLossConfig().to_dict() == LEGACY_CONFIG
    assert list(TrajectoryLossConfig().to_dict()) == list(LEGACY_CONFIG)
    assert TrajectoryLossConfig(candidate_regret_weight=0).to_dict() == LEGACY_CONFIG
    assert TrajectoryLossConfig.from_mapping(LEGACY_CONFIG).to_dict() == LEGACY_CONFIG
    research = {**LEGACY_CONFIG, "candidate_regret_weight": 0.1}
    assert TrajectoryLossConfig.from_mapping(research).to_dict() == research
    assert TrajectoryLossConfig().candidate_score_weight == 0.1


@pytest.mark.parametrize("value", [True, False, -0.1, float("nan"), float("inf"), -float("inf"), "0.1", None])
def test_invalid_coefficients_reject(value):
    with pytest.raises(ContractError, match="candidate_regret_weight"):
        TrajectoryLossConfig(candidate_regret_weight=value).validate()
    with pytest.raises(ContractError, match="candidate_regret_weight"):
        TrajectoryLossConfig.from_mapping({**LEGACY_CONFIG, "candidate_regret_weight": value})


@pytest.mark.parametrize("value", [
    None, [], {}, {**LEGACY_CONFIG, "unknown": 1},
    {key: value for key, value in LEGACY_CONFIG.items() if key != "yaw_weight"},
    {**LEGACY_CONFIG, "candidate_regret_weight": 0.0},
])
def test_loss_parser_rejects_unknown_missing_and_noncanonical_fields(value):
    with pytest.raises(ContractError, match="loss config"):
        TrajectoryLossConfig.from_mapping(value)


@pytest.mark.parametrize("masked", [False, True])
def test_regret_matches_original_composite_cost_not_an_alternative_ade_oracle(masked):
    inputs, yaw = sample_inputs(masked=masked)
    original = trajectory_loss(*inputs, target_yaw=yaw)
    enabled = trajectory_loss(*inputs, TrajectoryLossConfig(candidate_regret_weight=0.1), target_yaw=yaw)
    cost_rows = []
    for row in range(2):
        cost_rows.append(torch.stack([
            trajectory_loss(inputs[0][row:row+1, candidate:candidate+1], inputs[1][row:row+1, candidate:candidate+1],
                inputs[2][row:row+1, candidate:candidate+1], inputs[3][row:row+1], inputs[4][row:row+1],
                inputs[5][row:row+1], target_yaw=yaw[row:row+1])["regression_loss"]
            for candidate in range(3)
        ]))
    costs = torch.stack(cost_rows)
    regret = costs - costs.min(dim=1, keepdim=True).values
    expected = (inputs[2].softmax(1) * regret).sum(1).mean()
    assert torch.equal(enabled["candidate_regret_loss"], expected)
    assert torch.equal(enabled["loss"], original["loss"] + 0.1 * expected)
    for key in original:
        if key != "loss":
            assert torch.equal(enabled[key], original[key])


def test_auxiliary_has_only_direct_logit_gradients():
    inputs, yaw = sample_inputs(masked=True, gradients=True)
    original = trajectory_loss(*inputs, target_yaw=yaw)
    enabled = trajectory_loss(*inputs, TrajectoryLossConfig(candidate_regret_weight=0.1), target_yaw=yaw)
    old_gradients = torch.autograd.grad(original["loss"], inputs[:3], retain_graph=True)
    new_gradients = torch.autograd.grad(enabled["loss"], inputs[:3])
    assert torch.equal(old_gradients[0], new_gradients[0])
    assert torch.equal(old_gradients[1], new_gradients[1])
    assert not torch.equal(old_gradients[2], new_gradients[2])
    assert all(torch.isfinite(value).all() for value in new_gradients)
    assert not enabled["candidate_regret_loss"].requires_grad


def test_identical_candidate_ties_add_zero_risk_and_preserve_original_tie_break():
    inputs, yaw = sample_inputs(gradients=True)
    tied = (inputs[0][:, :1].expand(-1, 3, -1, -1), inputs[1][:, :1].expand(-1, 3, -1), *inputs[2:])
    original = trajectory_loss(*tied, target_yaw=yaw)
    enabled = trajectory_loss(*tied, TrajectoryLossConfig(candidate_regret_weight=0.1), target_yaw=yaw)
    assert enabled["candidate_regret_loss"].item() == 0.0
    assert torch.equal(original["loss"], enabled["loss"])
    assert torch.equal(enabled["oracle_candidate_index"], torch.zeros(2, dtype=torch.long))
    assert torch.equal(torch.autograd.grad(original["loss"], inputs[2], retain_graph=True)[0],
                       torch.autograd.grad(enabled["loss"], inputs[2])[0])


def test_auxiliary_permutation_equivariance_and_masked_suffix_invariance():
    inputs, yaw = sample_inputs(masked=True)
    cfg = TrajectoryLossConfig(candidate_regret_weight=0.1)
    baseline = trajectory_loss(*inputs, cfg, target_yaw=yaw)
    order = [2, 0, 1]
    permuted = trajectory_loss(inputs[0][:, order], inputs[1][:, order], inputs[2][:, order], *inputs[3:], cfg, target_yaw=yaw)
    assert torch.allclose(permuted["candidate_regret_loss"], baseline["candidate_regret_loss"], atol=1e-14, rtol=1e-14)
    changed = [value.clone() for value in inputs]
    changed_yaw = yaw.clone()
    for row, begin in ((0, 17), (1, 39)):
        changed[0][row, :, begin:] = 123.0
        changed[1][row, :, begin:] = 456.0
        changed[3][row, begin:] = -123.0
        changed[4][row, begin:] = 12.0
        changed_yaw[row, begin:] = 3.0
    altered = trajectory_loss(*changed, cfg, target_yaw=changed_yaw)
    assert torch.equal(baseline["candidate_regret_loss"], altered["candidate_regret_loss"])
    assert torch.equal(baseline["loss"], altered["loss"])


@pytest.mark.parametrize("index", [0, 1, 2, 3, 4])
@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_enabled_loss_retains_finite_checks_even_in_masked_suffix(index, value):
    inputs, yaw = sample_inputs(masked=True)
    inputs[index].reshape(-1)[-1] = value
    with pytest.raises(FloatingPointError):
        trajectory_loss(*inputs, TrajectoryLossConfig(candidate_regret_weight=0.1), target_yaw=yaw)


@pytest.mark.parametrize("mutation", ["logit_shape", "prefix", "empty"])
def test_enabled_loss_retains_shape_and_mask_checks(mutation):
    inputs, yaw = sample_inputs()
    inputs = list(inputs)
    if mutation == "logit_shape":
        inputs[2] = inputs[2][:, :2]
    elif mutation == "prefix":
        inputs[5][0, 1] = False
    else:
        inputs[5][0] = False
    with pytest.raises(ValueError):
        trajectory_loss(*inputs, TrajectoryLossConfig(candidate_regret_weight=0.1), target_yaw=yaw)


def test_single_candidate_has_zero_regret_without_removing_classification_or_regression():
    inputs, yaw = sample_inputs()
    single = (inputs[0][:, :1], inputs[1][:, :1], inputs[2][:, :1], *inputs[3:])
    original = trajectory_loss(*single, target_yaw=yaw)
    enabled = trajectory_loss(*single, TrajectoryLossConfig(candidate_regret_weight=0.1), target_yaw=yaw)
    assert enabled["candidate_regret_loss"].item() == 0.0
    assert torch.equal(original["loss"], enabled["loss"])


def test_finite_but_overflowing_weight_rejects_nonfinite_total_loss():
    inputs, yaw = sample_inputs(torch.float32)
    with pytest.raises(FloatingPointError, match="overflowed"):
        trajectory_loss(*inputs, TrajectoryLossConfig(candidate_regret_weight=1e308), target_yaw=yaw)


def test_auxiliary_evaluation_aggregation_is_sample_weighted_including_partial_batch():
    inputs, yaw = sample_inputs()
    cfg = TrajectoryLossConfig(candidate_regret_weight=0.1)
    sums, counts, auxiliary = {}, {}, {}
    expected_sum = 0.0
    for rows in (slice(None), slice(0, 1)):
        batch = tuple(value[rows] for value in inputs)
        batch_yaw = yaw[rows]
        losses = trajectory_loss(*batch, cfg, target_yaw=batch_yaw)
        expected_sum += float(losses["candidate_regret_loss"].item()) * len(batch[0])
        evaluation._accumulate_batch_metrics(sums, counts, candidate_xy=batch[0], candidate_speed=batch[1],
            candidate_logits=batch[2], target_xy=batch[3], target_speed=batch[4], target_valid=batch[5],
            target_yaw=batch_yaw, loss_config=cfg, auxiliary_loss_sums=auxiliary)
    assert auxiliary == {"candidate_regret_loss": expected_sum}
    assert all(count == 3 for count in counts.values())
    assert "candidate_regret_loss" not in sums


@pytest.mark.parametrize("option,weight", [(None, 0.0), ("0", 0.0), ("0.1", 0.1)])
def test_cli_passes_the_new_loss_contract_without_other_changes(monkeypatch, tmp_path, option, weight):
    captured = {}
    loaded = SimpleNamespace(examples=(), split="train", validation_report={"dataset_fingerprint_sha256": "a" * 64})
    monkeypatch.setattr(training, "_load_model_config", lambda _: _small_model_config())
    monkeypatch.setattr(training, "load_training_examples", lambda *args, **kwargs: loaded)
    monkeypatch.setattr(training, "Common10TorchDataset", lambda *args, **kwargs: SimpleNamespace(fingerprint_sha256="b" * 64))
    def record_train(dataset, **kwargs):
        captured.update(kwargs)
        return {"status": "TRAINING_TARGET_REACHED"}
    monkeypatch.setattr(training, "train_model", record_train)
    argv = ["dataset", "--run-dir", str(tmp_path / "run")]
    if option is not None:
        argv += ["--candidate-regret-weight", option]
    assert training.main(argv) == 0
    assert captured["loss_config"] == TrajectoryLossConfig(candidate_regret_weight=weight)
    assert not captured["resume"]
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize("value", ["-0.1", "nan", "inf", "-inf", "1e400"])
def test_invalid_cli_fails_before_model_dataset_or_output(monkeypatch, tmp_path, capsys, value):
    monkeypatch.setattr(training, "_load_model_config", lambda *args: pytest.fail("invalid coefficient reached model"))
    monkeypatch.setattr(training, "load_training_examples", lambda *args, **kwargs: pytest.fail("invalid coefficient reached data"))
    assert training.main(["dataset", "--run-dir", str(tmp_path / "run"), f"--candidate-regret-weight={value}"]) == 2
    assert "candidate_regret_weight" in capsys.readouterr().err
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize("weight", [0.0, 0.1])
def test_cpu_training_history_and_checkpoint_use_exact_loss_contract(monkeypatch, tmp_path, weight):
    config = _small_model_config()
    dataset = Common10TorchDataset((_example(tmp_path, 0),), config, split="train")
    captured = []
    monkeypatch.setattr(training, "_atomic_torch_save", lambda path, payload: captured.append(copy.deepcopy(payload)))
    cfg = TrajectoryLossConfig(candidate_regret_weight=weight)
    run_dir = tmp_path / "run"
    report = training.train_model(dataset, run_dir=run_dir, dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256="a" * 64, model_config=config, train_config=_train_config(1), loss_config=cfg)
    assert len(captured) == 1
    assert captured[0]["loss_config"] == cfg.to_dict()
    stored = json.loads((run_dir / "run.json").read_text())
    history = json.loads((run_dir / "metrics.jsonl").read_text())
    assert stored["loss_config"] == cfg.to_dict()
    assert history == report["last_metrics"] == stored["last_metrics"]
    assert ("candidate_regret_loss" in history) == (weight > 0.0)
    if weight:
        assert history["candidate_regret_loss"] >= 0.0
    assert all(torch.isfinite(value).all() for value in captured[0]["model_state_dict"].values())


def test_enabled_exact_resume_matches_fresh_training_bitexact(monkeypatch, tmp_path):
    # HH_260906 - Decode only our in-memory synthetic checkpoint so this test does not depend on unsafe legacy torch.load.
    config = _small_model_config()
    dataset = Common10TorchDataset((_example(tmp_path, 0),), config, split="train")
    captured = {}
    original_save = training._atomic_torch_save
    def save_and_capture(path, payload):
        original_save(path, payload)
        captured[str(path)] = copy.deepcopy(payload)
    monkeypatch.setattr(training, "_atomic_torch_save", save_and_capture)
    monkeypatch.setattr(training, "_read_checkpoint_file", lambda path, _: (copy.deepcopy(captured[str(path)]), "b" * 64))
    common = {"dataset_fingerprint_sha256": dataset.fingerprint_sha256, "corpus_fingerprint_sha256": "a" * 64,
              "model_config": config, "loss_config": TrajectoryLossConfig(candidate_regret_weight=0.1)}
    resumed_dir, fresh_dir = tmp_path / "resumed", tmp_path / "fresh"
    training.train_model(dataset, run_dir=resumed_dir, train_config=_train_config(1), **common)
    resumed = training.train_model(dataset, run_dir=resumed_dir, train_config=_train_config(2), resume=True, **common)
    fresh = training.train_model(dataset, run_dir=fresh_dir, train_config=_train_config(2), **common)
    assert resumed["last_metrics"] == fresh["last_metrics"]
    resumed_state = captured[str(resumed_dir / "checkpoints/latest.pt")]["model_state_dict"]
    fresh_state = captured[str(fresh_dir / "checkpoints/latest.pt")]["model_state_dict"]
    assert resumed_state.keys() == fresh_state.keys()
    assert all(torch.equal(resumed_state[key], fresh_state[key]) for key in resumed_state)
    assert (resumed_dir / "metrics.jsonl").read_bytes() == (fresh_dir / "metrics.jsonl").read_bytes()


@pytest.mark.parametrize("stored,requested", [(0.0, 0.1), (0.1, 0.0), (0.1, 0.2)])
def test_resume_cannot_change_or_enable_auxiliary(monkeypatch, tmp_path, stored, requested):
    model_config, train_config = _small_model_config(), _train_config(1)
    device = torch.device("cpu")
    sampling_plan = training._sampling_plan({"carla": (0,)}, train_config)
    payload = {
        "checkpoint_id": training.CHECKPOINT_ID,
        "dataset_fingerprint_sha256": "a" * 64, "corpus_fingerprint_sha256": "b" * 64,
        "model_config": model_config.to_dict(), "model_config_sha256": training._canonical_sha256(model_config.to_dict()),
        "training_split": "train", "training_episode_ids": ["training"],
        "sampling_plan": sampling_plan, "sampling_plan_sha256": training._canonical_sha256(sampling_plan),
        "runtime_abi": training._runtime_abi(), "device_abi": training._device_abi(device),
        "train_config": train_config.to_dict(), "loss_config": TrajectoryLossConfig(candidate_regret_weight=stored).to_dict(),
    }
    monkeypatch.setattr(training, "_read_checkpoint_file", lambda *_: (payload, "c" * 64))
    with pytest.raises(ContractError, match="checkpoint loss config does not match"):
        training._load_checkpoint(tmp_path / "unused.pt", model=None, optimizer=None,
            dataset_fingerprint_sha256="a" * 64, corpus_fingerprint_sha256="b" * 64,
            model_config=model_config, train_config=train_config,
            loss_config=TrajectoryLossConfig(candidate_regret_weight=requested), device=device,
            training_split="train", training_episode_ids=["training"], dataset_size=1,
            domain_indices={"carla": (0,)}, sampling_plan=sampling_plan)


@pytest.mark.parametrize("weight", [0.0, 0.1])
def test_evaluation_accepts_exact_contract_and_keeps_core_metrics_comparable(monkeypatch, tmp_path, weight):
    config = _small_model_config()
    payload = _checkpoint_payload(config)
    payload["loss_config"] = TrajectoryLossConfig(candidate_regret_weight=weight).to_dict()
    dataset = Common10TorchDataset(tuple(_example(tmp_path, index, episode_id="validation") for index in range(3)), config, split="val")
    monkeypatch.setattr(evaluation, "_read_checkpoint_file", lambda *_: (payload, "b" * 64))
    report = evaluation.evaluate_model(dataset, checkpoint_path=tmp_path / "fixture.pt", output_dir=tmp_path / "evaluation",
        dataset_fingerprint_sha256=dataset.fingerprint_sha256, corpus_fingerprint_sha256="c" * 64,
        evaluation_split="val", batch_size=2, render_count=0)
    assert report["sample_count"] == 3
    assert "candidate_regret_loss" not in report["metrics"]
    assert all("candidate_regret_loss" not in value["metrics"] for value in report["per_domain_metrics"].values())
    assert ("auxiliary_loss_metrics" in report) == bool(weight)
    assert ("loss_config" in report) == bool(weight)
    if weight:
        assert report["auxiliary_loss_metric_counts"] == {"candidate_regret_loss": 3}
        assert report["auxiliary_loss_metrics"]["candidate_regret_loss"] >= 0.0
        assert report["loss_config"] == payload["loss_config"]


@pytest.mark.parametrize("value", [LEGACY_CONFIG, {**LEGACY_CONFIG, "candidate_regret_weight": 0.1}])
def test_runtime_audit_accepts_both_exact_loss_contracts_without_gate_changes(monkeypatch, tmp_path, value):
    payload = _checkpoint_payload()
    payload["loss_config"] = dict(value)
    monkeypatch.setattr(audit, "_read_checkpoint_file", lambda *_: (payload, "b" * 64))
    actual = audit._read_checkpoint_for_audit(checkpoint_path=tmp_path / "fixture.pt",
        expected_checkpoint_sha256="b" * 64, corpus_fingerprint_sha256="c" * 64)
    assert actual[0] is payload
    assert actual[3]["checkpoint_sha256"] == "b" * 64


@pytest.mark.parametrize("value", [
    {**LEGACY_CONFIG, "candidate_regret_weight": 0.0},
    {**LEGACY_CONFIG, "candidate_regret_weight": True},
    {**LEGACY_CONFIG, "candidate_regret_weight": 0.1, "ignored": True},
])
def test_evaluator_and_runtime_audit_reject_invalid_extended_contracts(monkeypatch, tmp_path, value):
    config = _small_model_config()
    payload = _checkpoint_payload(config)
    payload["loss_config"] = value
    dataset = Common10TorchDataset((_example(tmp_path, 0, episode_id="validation"),), config, split="val")
    for module in (evaluation, audit):
        monkeypatch.setattr(module, "_read_checkpoint_file", lambda *_: (payload, "b" * 64))
    with pytest.raises(ContractError):
        audit._read_checkpoint_for_audit(checkpoint_path=tmp_path / "fixture.pt",
            expected_checkpoint_sha256="b" * 64, corpus_fingerprint_sha256="c" * 64)
    with pytest.raises(ContractError):
        evaluation.evaluate_model(dataset, checkpoint_path=tmp_path / "fixture.pt", output_dir=tmp_path / "evaluation",
            dataset_fingerprint_sha256=dataset.fingerprint_sha256, corpus_fingerprint_sha256="c" * 64,
            evaluation_split="val", render_count=0)
    assert not (tmp_path / "evaluation").exists()


def test_current_secure_runtime_bundle_still_rejects_research_loss_contract(monkeypatch, tmp_path):
    assert bundle._LOSS_CONFIG_FIELDS == frozenset(LEGACY_CONFIG)
    payload = _checkpoint_payload()
    payload["loss_config"] = TrajectoryLossConfig(candidate_regret_weight=0.1).to_dict()
    with pytest.raises(ContractError, match="training_provenance.loss_config"):
        _export(monkeypatch, tmp_path, payload)
    assert not (tmp_path / "model.runtime.npz").exists()
