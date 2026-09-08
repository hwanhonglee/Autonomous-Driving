"""HH_260906 - Test research DRIVE/STOP candidate geometry without training data admission or deployment."""

from __future__ import annotations

import ast
import copy
from dataclasses import replace
import hashlib
import inspect
import json
from pathlib import Path
import textwrap

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
from portable_e2e.dataset import FEATURE_NAMES
import portable_e2e.model as module
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel, PHYSICAL_STOPMIX_MODEL_ID, parameter_count
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
from portable_e2e import stop_primitive_research as primitive
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
import portable_e2e.runtime_weight_bundle as bundle
import portable_e2e.train as training
import portable_e2e.evaluate as evaluation
import portable_e2e.audit_runtime as audit
from portable_e2e.torch_dataset import Common10TorchDataset
from test_portable_e2e_model import _example, _small_model_config, _train_config
from test_portable_e2e_runtime_weight_bundle import _checkpoint_payload, _export


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"
PREVIOUS_SOURCE_SHA = {
    "__init__": "58143fed88dd18d21624537df09eb1bd9cea3602f3b2951b910a47eba4e4fd21",
    "forward": "fa8e253f07cb9f2c3fd366e55ef78950d0acd63a21ec6b60a7c1143dbb0cb45f",
    "_decode_physical_v1": "ffa4930c7624432a68df3ce9cf2c4585d4f98f00fc11ec6394a13fcfaceac420",
}


def pre_stopmix_source(name):
    """HH_260906 - Normalize only explicit new-ID branches and checked candidate-axis bindings to the byte-pinned prior methods."""
    source = inspect.getsource(getattr(PerspectiveTrajectoryModel, name))
    if name not in PREVIOUS_SOURCE_SHA:
        return source
    tree = ast.parse(textwrap.dedent(source)).body[0]
    removals = []
    new_id_guard = ast.dump(ast.parse("cfg.model_id == PHYSICAL_STOPMIX_MODEL_ID", mode="eval").body)
    for statement in tree.body:
        if isinstance(statement, ast.If) and ast.dump(statement.test) == new_id_guard:
            assert not statement.orelse
            removals.append(statement)
        elif name == "__init__" and isinstance(statement, ast.Assign) and ast.dump(statement) == ast.dump(ast.parse("drive_candidates = cfg.candidate_count").body[0]):
            removals.append(statement)
        elif name == "_decode_physical_v1":
            if isinstance(statement, ast.Assign) and len(statement.targets) == 1 and isinstance(statement.targets[0], ast.Name) and statement.targets[0].id in ("expected_candidates", "candidate_count"):
                removals.append(statement)
            elif isinstance(statement, ast.If) and ast.dump(statement.test) == ast.dump(ast.parse(
                "raw.ndim != 4 or tuple(raw.shape) != (ego_history.shape[0], expected_candidates, cfg.future_points, 2)", mode="eval").body):
                assert not statement.orelse
                removals.append(statement)
    assert len(removals) == {"__init__": 3, "forward": 1, "_decode_physical_v1": 3}[name]
    lines = source.splitlines(keepends=True)
    for statement in sorted(removals, key=lambda value: value.lineno, reverse=True):
        del lines[statement.lineno - 1:statement.end_lineno]
    source = "".join(lines)
    if name == "__init__":
        assert source.count("drive_candidates") == 2
        source = source.replace("drive_candidates", "cfg.candidate_count")
    elif name == "_decode_physical_v1":
        source = source.replace("        # HH_260906 - Bind the decoded DRIVE axis explicitly; stopmix owns six DRIVE latents, never twelve with discarded outputs.\n", "")
        assert source.count("candidate_count") == 3
        source = source.replace("candidate_count", "cfg.candidate_count")
    assert hashlib.sha256(source.rstrip().encode()).hexdigest() == PREVIOUS_SOURCE_SHA[name]
    return source


def config(model_id=PHYSICAL_STOPMIX_MODEL_ID, candidates=12):
    return replace(_small_model_config(), model_id=model_id, candidate_count=candidates, maximum_step_m=1.0)


def inputs(cfg, *, dtype=torch.float32, speed=2.0):
    generator = torch.Generator().manual_seed(781)
    ego = torch.randn(2, cfg.ego_history_frames, cfg.ego_features, generator=generator, dtype=dtype) * .1
    ego[:, :, FEATURE_NAMES.index("velocity_x_mps")] = speed
    route = torch.zeros(2, cfg.route_points, 2, dtype=dtype)
    route[:, :, 0] = torch.arange(cfg.route_points, dtype=dtype) * 10
    return {"images": torch.rand(2, 6, 3, cfg.image_height, cfg.image_width, generator=generator, dtype=dtype),
        "calibration": torch.rand(2, 6, cfg.calibration_features, generator=generator, dtype=dtype),
        "ego_history": ego, "ego_history_mask": torch.ones(2, cfg.ego_history_frames, dtype=torch.bool),
        "route_xy": route, "route_mask": torch.ones(2, cfg.route_points, dtype=torch.bool)}


@pytest.fixture(autouse=True)
def single_thread_cpu():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def test_explicit_config_changes_only_identity_and_candidate_axis():
    old = json.loads(CONFIG.with_name("perspective_trajectory_physical_v1.model.json").read_text())
    new = json.loads(CONFIG.read_text())
    assert new == {**old, "model_id": PHYSICAL_STOPMIX_MODEL_ID, "candidate_count": 12}
    assert ModelConfig.from_mapping(new).to_dict() == new
    assert ModelConfig().model_id == module.MODEL_ID
    torch.manual_seed(20260903)
    baseline = PerspectiveTrajectoryModel(ModelConfig.from_mapping(old))
    torch.manual_seed(20260903)
    mixed = PerspectiveTrajectoryModel(ModelConfig.from_mapping(new))
    assert parameter_count(baseline) == 954590
    assert parameter_count(mixed) == 1056362
    assert mixed.trajectory_head.out_features == 6 * 64 * 2
    assert mixed.candidate_head.out_features == mixed.stop_candidate_head.out_features == 6
    assert mixed.stop_head.out_features == 6 * 65
    assert set(mixed.state_dict()) - set(baseline.state_dict()) == {
        "stop_head.weight", "stop_head.bias", "stop_candidate_head.weight", "stop_candidate_head.bias"}
    assert all(torch.equal(value, mixed.state_dict()[name]) for name, value in baseline.state_dict().items())


@pytest.mark.parametrize("change", [{"candidate_count": 6}, {"candidate_count": 11}, {"candidate_count": 12.0},
    {"candidate_count": True}, {"future_points": 63}, {"maximum_step_m": 3.0}, {"ego_features": 14}, {"camera_count": 1}])
def test_research_config_rejects_other_shapes_and_physical_envelopes(change):
    with pytest.raises(ContractError):
        replace(config(), **change).validate()


@pytest.mark.parametrize("name", list(PREVIOUS_SOURCE_SHA))
def test_preexisting_methods_restore_only_explicit_new_logic_to_exact_prior_source(name):
    pre_stopmix_source(name)


@pytest.mark.parametrize("model_id", [module.MODEL_ID, module.PHYSICAL_MODEL_ID, module.CANDIDATE_RANK_MODEL_ID, module.PHYSICAL_NO_ACCEL_MODEL_ID])
@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_all_old_model_initialization_state_and_outputs_bitexact_to_prior_methods(model_id, dtype):
    # HH_260906 - Compile the byte-pinned old methods locally, without Git access, checkpoint loading or GPU initialization.
    namespace = dict(module.__dict__)
    source = "class LegacyPerspectiveTrajectoryModel(nn.Module):\n" + "\n".join(
        pre_stopmix_source(name) for name in PREVIOUS_SOURCE_SHA
    ) + (
        "\n    reset_parameters = PerspectiveTrajectoryModel.reset_parameters\n"
        "    _check_inputs = PerspectiveTrajectoryModel._check_inputs\n"
        "    _score_candidates = PerspectiveTrajectoryModel._score_candidates\n"
    )
    exec(compile(source, "<verified-pre-stopmix-model>", "exec"), namespace)
    legacy_class = namespace["LegacyPerspectiveTrajectoryModel"]
    cfg = config(model_id, candidates=3)
    torch.manual_seed(93)
    expected_model = legacy_class(cfg).to(dtype).eval()
    torch.manual_seed(93)
    actual_model = PerspectiveTrajectoryModel(cfg).to(dtype).eval()
    assert expected_model.state_dict().keys() == actual_model.state_dict().keys()
    assert all(torch.equal(value, actual_model.state_dict()[name]) for name, value in expected_model.state_dict().items())
    supplied = inputs(cfg, dtype=dtype)
    supplied["ego_history_mask"][0, 0] = False
    supplied["route_mask"][1, -2:] = False
    with torch.no_grad():
        expected, actual = expected_model(**supplied), actual_model(**supplied)
    assert all(torch.equal(first, second) for first, second in zip(actual, expected))


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("speed", [0.0, .01, .4, 2.0, 30.0 / 3.6])
def test_twelve_candidates_preserve_drive_outputs_and_exact_terminal_stop(dtype, speed):
    torch.manual_seed(17)
    baseline = PerspectiveTrajectoryModel(config(module.PHYSICAL_MODEL_ID, 6)).to(dtype).eval()
    torch.manual_seed(17)
    mixed = PerspectiveTrajectoryModel(config()).to(dtype).eval()
    supplied = inputs(mixed.config, dtype=dtype, speed=speed)
    before = {key: value.clone() for key, value in supplied.items()}
    with torch.no_grad():
        xy, velocity, logits = mixed(**supplied)
        drive = baseline(**supplied)
    assert xy.shape == (2, 12, 64, 2) and velocity.shape == (2, 12, 64) and logits.shape == (2, 12)
    assert all(torch.equal(value[:, :6], expected) for value, expected in zip((xy, velocity, logits), drive))
    assert all(torch.equal(supplied[key], before[key]) for key in supplied)
    assert all(torch.isfinite(value).all() for value in (xy, velocity, logits))
    stopped = velocity[:, 6:]
    assert torch.count_nonzero(stopped[:, :, -1]) == 0
    entry = torch.cat((torch.full((2, 6, 1), speed, dtype=dtype), stopped[:, :, :-1]), dim=2)
    assert torch.all(stopped <= entry)
    assert torch.all((entry - stopped) / .1 <= 2.9 + 256 * torch.finfo(dtype).eps)
    if speed == 0:
        assert torch.count_nonzero(xy[:, 6:]) == torch.count_nonzero(stopped) == 0
    for row in range(2):
        for candidate in range(12):
            forced = [-1.0] * 12
            forced[candidate] = 1.0
            selected = validate_and_select_trajectory(xy[row].tolist(), velocity[row].tolist(), forced,
                current_speed_mps=float(supplied["ego_history"][row, -1, FEATURE_NAMES.index("velocity_x_mps")]),
                config=RuntimeGateConfig(candidate_count=12))
            assert selected.candidate_index == candidate


def test_stop_primitive_source_and_all_scalar_bounds_are_unchanged():
    assert hashlib.sha256(inspect.getsource(primitive).encode()).hexdigest() == "4021eb2ba6d84260b6b658114fbe31a1e7ef826121e4d07690bfd4154919a92c"
    assert primitive.CANDIDATE_COUNT == module.STOPMIX_DRIVE_CANDIDATES == 6
    assert primitive.TIME_STEP_S == module.PHYSICAL_TIME_STEP_S == .1
    assert primitive.MAX_SPEED_MPS == module.PHYSICAL_MAXIMUM_SPEED_MPS == 30.0 / 3.6
    assert primitive.MAX_DECELERATION_MPS2 == module.PHYSICAL_MAXIMUM_ACCELERATION_MPS2 == 2.9
    assert primitive.MAX_CURVATURE_RAD_PER_M == module.PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M == .2
    assert primitive.MAX_LATERAL_ACCELERATION_MPS2 == module.PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2 == 2.8


@pytest.mark.parametrize("speed", [-.001, 30.0 / 3.6 + .1])
def test_new_stop_call_uses_same_explicit_vx_normalization_but_raw_runtime_gate_is_unchanged(speed):
    model = PerspectiveTrajectoryModel(config()).eval()
    supplied = inputs(model.config, speed=speed)
    normalized = copy.deepcopy(supplied)
    normalized["ego_history"][:, -1, FEATURE_NAMES.index("velocity_x_mps")] = min(max(speed, 0.0), 30.0 / 3.6)
    observed = []
    original = primitive.decode_stop_primitive
    def record(current, *args):
        observed.append(current.detach().clone())
        return original(current, *args)
    from unittest.mock import patch
    with patch.object(primitive, "decode_stop_primitive", record), torch.no_grad():
        xy, velocity, _ = model(**supplied)
    assert torch.equal(observed[0], normalized["ego_history"][:, -1, FEATURE_NAMES.index("velocity_x_mps")])
    assert torch.count_nonzero(velocity[:, 6:, -1]) == 0
    if speed > 30.0 / 3.6:
        with pytest.raises(ContractError):
            validate_and_select_trajectory(xy[0].tolist(), velocity[0].tolist(), [0.] * 12,
                current_speed_mps=speed, config=RuntimeGateConfig(candidate_count=12))


def test_new_heads_and_shared_context_receive_finite_gradients_for_moving_stop_candidates():
    model = PerspectiveTrajectoryModel(config())
    supplied = inputs(model.config)
    xy, velocity, logits = model(**supplied)
    loss = xy[:, 6:, :, 0].sum() + .3 * xy[:, 6:, :, 1].sum() + velocity[:, 6:].sum() + logits[:, 6:].sum()
    loss.backward()
    for name in ("stop_head.weight", "stop_head.bias", "stop_candidate_head.weight", "stop_candidate_head.bias", "fusion.0.weight", "image_encoder.features.0.weight"):
        gradient = dict(model.named_parameters())[name].grad
        assert gradient is not None and torch.isfinite(gradient).all() and torch.count_nonzero(gradient) > 0
    split = model.stop_head.weight.grad.reshape(6, 65, model.config.hidden_width)
    assert torch.count_nonzero(split[:, 0]) > 0 and torch.count_nonzero(split[:, 1:]) > 0


def test_already_stationary_stop_geometry_has_zero_latent_gradients_not_learned_braking_intent():
    model = PerspectiveTrajectoryModel(config())
    xy, speed, _ = model(**inputs(model.config, speed=0.0))
    (xy[:, 6:].sum() + speed[:, 6:].sum()).backward()
    assert model.stop_head.weight.grad is not None
    assert torch.count_nonzero(model.stop_head.weight.grad) == 0
    assert torch.count_nonzero(model.stop_head.bias.grad) == 0


@pytest.mark.parametrize("field", ["images", "calibration", "ego_history", "route_xy"])
@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_all_supplied_finite_checks_remain_before_stop_normalization(field, value):
    model = PerspectiveTrajectoryModel(config())
    supplied = inputs(model.config)
    supplied[field].reshape(-1)[0] = value
    with pytest.raises(FloatingPointError):
        model(**supplied)


@pytest.mark.parametrize("shape", [(2, 12, 64, 2), (2, 6, 63, 2), (2, 6, 64, 3), (2, 6, 64), (1, 6, 64, 2)])
def test_drive_decoder_rejects_discarded_or_malformed_candidate_axes(shape):
    model = PerspectiveTrajectoryModel(config())
    supplied = inputs(model.config)
    with pytest.raises(ValueError, match="DRIVE latents"):
        model._decode_physical_v1(torch.zeros(shape), supplied["ego_history"], supplied["route_xy"], supplied["route_mask"])


def test_generic_loss_can_score_all_twelve_candidates_without_future_model_inputs():
    model = PerspectiveTrajectoryModel(config())
    xy, speed, logits = model(**inputs(model.config))
    target_xy, target_speed = xy[:, 7].detach(), speed[:, 7].detach()
    valid = torch.ones(2, 64, dtype=torch.bool)
    loss = trajectory_loss(xy, speed, logits, target_xy, target_speed, valid, TrajectoryLossConfig())
    assert loss["oracle_candidate_index"].shape == (2,)
    # HH_260906 - A synthetic matched moving STOP target can supervise candidate preference, not a traffic-rule reason to stop.
    assert torch.equal(loss["oracle_candidate_index"], torch.full((2,), 7, dtype=torch.long))
    assert all(0 <= int(index) < 12 for index in loss["selected_candidate_index"])
    loss["loss"].backward()
    assert model.stop_candidate_head.bias.grad[1] < 0
    assert all(parameter.grad is None or torch.isfinite(parameter.grad).all() for parameter in model.parameters())
    assert set(inspect.signature(model.forward).parameters) == {
        "images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask"}


def test_generic_trainer_accepts_synthetic_twelve_candidate_unit_batch(monkeypatch, tmp_path):
    # HH_260906 - This one-step small CPU fixture verifies integration only; it is not a dataset-backed research fit.
    cfg = config()
    dataset = Common10TorchDataset((_example(tmp_path, 0),), cfg, split="train")
    captured = []
    monkeypatch.setattr(training, "_atomic_torch_save", lambda path, payload: captured.append(copy.deepcopy(payload)))
    report = training.train_model(dataset, run_dir=tmp_path / "synthetic-unit-run", model_config=cfg,
        dataset_fingerprint_sha256=dataset.fingerprint_sha256, corpus_fingerprint_sha256="c" * 64,
        train_config=_train_config(1), loss_config=TrajectoryLossConfig())
    assert report["state"]["global_step"] == 1 and report["vehicle_control_approved"] is False
    assert captured[0]["model_config"] == cfg.to_dict()
    assert captured[0]["loss_config"] == TrajectoryLossConfig().to_dict()
    assert all(torch.isfinite(value).all() for value in captured[0]["model_state_dict"].values())


def test_legacy_checkpoint_cannot_be_relabelled_as_stopmix_for_resume(monkeypatch, tmp_path):
    payload = _checkpoint_payload(config(module.PHYSICAL_MODEL_ID, 6))
    monkeypatch.setattr(training, "_read_checkpoint_file", lambda *_: (payload, "b" * 64))
    train_cfg = training.TrainConfig(**payload["train_config"])
    with pytest.raises(ContractError, match="model config does not match"):
        training._load_checkpoint(tmp_path / "never-loaded.pt", model=None, optimizer=None,
            dataset_fingerprint_sha256=payload["dataset_fingerprint_sha256"],
            corpus_fingerprint_sha256=payload["corpus_fingerprint_sha256"], model_config=config(),
            train_config=train_cfg, loss_config=TrajectoryLossConfig(), device=torch.device("cpu"),
            training_split="train", training_episode_ids=payload["training_episode_ids"], dataset_size=2,
            domain_indices={"carla": (0, 1)}, sampling_plan=payload["sampling_plan"])


def test_generic_evaluation_and_geometry_audit_accept_twelve_research_candidates(monkeypatch, tmp_path):
    cfg = config()
    dataset = Common10TorchDataset((_example(tmp_path, 0, episode_id="validation"),), cfg, split="val")
    payload = _checkpoint_payload(cfg)
    monkeypatch.setattr(evaluation, "_read_checkpoint_file", lambda *_: (payload, "b" * 64))
    report = evaluation.evaluate_model(dataset, checkpoint_path=tmp_path / "fixture.pt", output_dir=tmp_path / "evaluation",
        dataset_fingerprint_sha256=dataset.fingerprint_sha256, corpus_fingerprint_sha256="c" * 64,
        batch_size=1, render_count=0)
    assert report["sample_count"] == 1 and report["vehicle_control_approved"] is False
    monkeypatch.setattr(audit, "_read_checkpoint_file", lambda *_: (payload, "b" * 64))
    loaded, model_config, _, _ = audit._read_checkpoint_for_audit(checkpoint_path=tmp_path / "fixture.pt",
        expected_checkpoint_sha256="b" * 64, corpus_fingerprint_sha256="c" * 64)
    assert loaded is payload and model_config.candidate_count == 12


def test_current_secure_bundle_does_not_admit_new_research_model(monkeypatch, tmp_path):
    assert PHYSICAL_STOPMIX_MODEL_ID not in bundle.SUPPORTED_MODEL_IDS
    cfg = config().to_dict()
    with pytest.raises(ContractError, match="not supported by bundle"):
        bundle._validate_model_config(cfg, training._canonical_sha256(cfg))
    payload = _checkpoint_payload(config())
    with pytest.raises(ContractError, match="not supported by bundle"):
        _export(monkeypatch, tmp_path, payload)
    assert not (tmp_path / "model.runtime.npz").exists()
