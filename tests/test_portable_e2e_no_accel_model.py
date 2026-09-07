"""HH_260906 - Verify the research-only supplied-acceleration ablation on small synthetic CPU tensors."""

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
from portable_e2e.losses import TrajectoryLossConfig
import portable_e2e.model as model_module
from portable_e2e.model import (
    CANDIDATE_RANK_MODEL_ID, MODEL_ID, ModelConfig, PHYSICAL_MODEL_ID,
    PHYSICAL_NO_ACCEL_MODEL_ID, PerspectiveTrajectoryModel, parameter_count,
)
import portable_e2e.runtime_weight_bundle as bundle
import portable_e2e.train as training


ROOT = Path(__file__).resolve().parents[1]


def config(model_id=PHYSICAL_NO_ACCEL_MODEL_ID):
    return ModelConfig(model_id=model_id, route_points=8, candidate_count=3, image_width=32, image_height=16,
                       image_grid_width=2, image_grid_height=1, image_embedding=8, camera_fusion_width=16,
                       ego_embedding=8, route_embedding=8, hidden_width=16, encoder_base_channels=4,
                       ego_history_frames=3, maximum_step_m=1.0)


def inputs(cfg):
    torch.manual_seed(20260908)
    ego = torch.randn(2, cfg.ego_history_frames, cfg.ego_features) * 0.1
    ego[..., 0] = 1.0
    ego[..., 1] = 4.0
    route = torch.zeros(2, cfg.route_points, 2)
    route[..., 0] = torch.arange(cfg.route_points) * 10.0
    return {"images": torch.rand(2, 6, 3, cfg.image_height, cfg.image_width),
            "calibration": torch.rand(2, 6, cfg.calibration_features), "ego_history": ego,
            "ego_history_mask": torch.ones(2, cfg.ego_history_frames, dtype=torch.bool),
            "route_xy": route, "route_mask": torch.ones(2, cfg.route_points, dtype=torch.bool)}


def state_hash(model):
    digest = hashlib.sha256()
    for name, tensor in model.state_dict().items():
        digest.update(name.encode())
        digest.update(str(tuple(tensor.shape)).encode())
        digest.update(tensor.detach().cpu().numpy().tobytes())
    return digest.hexdigest()


def test_config_is_only_a_new_identity_and_parameter_state_is_unchanged():
    before = json.loads((ROOT / "portable_e2e/config/perspective_trajectory_physical_v1.model.json").read_text())
    after = json.loads((ROOT / "portable_e2e/config/perspective_trajectory_physical_no_accel_v1.model.json").read_text())
    assert after == {**before, "model_id": PHYSICAL_NO_ACCEL_MODEL_ID}
    assert ModelConfig.from_mapping(after).to_dict() == after
    assert len(FEATURE_NAMES) == 13
    assert FEATURE_NAMES[3:5] == ("acceleration_x_mps2", "acceleration_y_mps2")
    torch.manual_seed(20260903)
    physical = PerspectiveTrajectoryModel(ModelConfig.from_mapping(before))
    torch.manual_seed(20260903)
    research = PerspectiveTrajectoryModel(ModelConfig.from_mapping(after))
    assert parameter_count(physical) == parameter_count(research) == 954590
    assert state_hash(physical) == state_hash(research)
    assert tuple(physical.state_dict()) == tuple(research.state_dict())
    assert tuple(dict(physical.named_parameters())) == tuple(dict(research.named_parameters()))
    assert ModelConfig().model_id == MODEL_ID


@pytest.mark.parametrize("training_mode", [False, True])
@pytest.mark.parametrize("history_index", range(3))
def test_all_history_acceleration_values_are_ignored_without_mutating_inputs(training_mode, history_index):
    torch.manual_seed(20260903)
    model = PerspectiveTrajectoryModel(config()).train(training_mode)
    supplied = inputs(model.config)
    original = supplied["ego_history"].clone()
    with torch.no_grad():
        expected = model(**supplied)
        changed = {**supplied, "ego_history": original.clone()}
        changed["ego_history"][:, history_index, 3] = 123456.0
        changed["ego_history"][:, history_index, 4] = -654321.0
        actual = model(**changed)
    for old, new in zip(expected, actual):
        assert torch.equal(old, new)
    assert torch.equal(supplied["ego_history"], original)
    assert torch.all(changed["ego_history"][:, history_index, 3] == 123456.0)
    assert torch.all(changed["ego_history"][:, history_index, 4] == -654321.0)


def test_new_model_equals_same_weight_physical_model_with_explicitly_zeroed_history():
    torch.manual_seed(20260903)
    physical = PerspectiveTrajectoryModel(config(PHYSICAL_MODEL_ID)).eval()
    torch.manual_seed(20260903)
    research = PerspectiveTrajectoryModel(config()).eval()
    supplied = inputs(research.config)
    zeroed = {**supplied, "ego_history": supplied["ego_history"].clone()}
    zeroed["ego_history"][..., 3:5] = 0.0
    with torch.no_grad():
        actual = research(**supplied)
        expected = physical(**zeroed)
    assert [tuple(value.shape) for value in actual] == [(2, 3, 64, 2), (2, 3, 64), (2, 3)]
    assert all(torch.equal(first, second) for first, second in zip(actual, expected))
    assert all(torch.isfinite(value).all() for value in actual)


def test_masked_channels_have_zero_gradient_and_remaining_inputs_and_parameters_are_active():
    torch.manual_seed(20260903)
    model = PerspectiveTrajectoryModel(config()).eval()
    supplied = inputs(model.config)
    supplied["ego_history"].requires_grad_()
    original = supplied["ego_history"].detach().clone()
    xy, speed, logits = model(**supplied)
    objective = xy.square().mean() + speed.square().mean() + logits.square().mean()
    objective.backward()
    gradient = supplied["ego_history"].grad
    assert torch.isfinite(gradient).all()
    assert torch.count_nonzero(gradient[..., 3:5]) == 0
    assert torch.count_nonzero(gradient[..., :3]) > 0
    assert torch.count_nonzero(gradient[..., 5:]) > 0
    assert all(torch.count_nonzero(gradient[:, index, :]) > 0 for index in range(3))
    assert torch.count_nonzero(model.ego_input[0].weight.grad[:, 3:5]) == 0
    assert torch.count_nonzero(model.ego_input[0].weight.grad[:, :3]) > 0
    assert all(parameter.grad is None or torch.isfinite(parameter.grad).all() for parameter in model.parameters())
    assert torch.equal(supplied["ego_history"].detach(), original)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("history_index,channel", [(0, 3), (0, 4), (2, 3), (2, 4)])
def test_nonfinite_supplied_acceleration_is_rejected_even_if_its_history_is_masked(value, history_index, channel):
    model = PerspectiveTrajectoryModel(config()).eval()
    supplied = inputs(model.config)
    supplied["ego_history_mask"][:, 0] = False
    supplied["ego_history"][:, history_index, channel] = value
    with pytest.raises(FloatingPointError, match="ego_history contains NaN or Inf"):
        model(**supplied)


@pytest.mark.parametrize("changes", [{"maximum_step_m": 3.0}, {"ego_features": 11}, {"model_id": "physical_no_accel"}])
def test_research_id_does_not_relax_config_or_physical_envelope(changes):
    with pytest.raises(ContractError):
        replace(config(), **changes).validate()


def legacy_forward_source():
    # HH_260906 - Restore only the new gated block and dispatch spelling, then pin the entire historical method hash.
    source = inspect.getsource(PerspectiveTrajectoryModel.forward).replace(
        "\n        if cfg.model_id == PHYSICAL_NO_ACCEL_MODEL_ID:\n"
        "            # HH_260906 - Validate raw inputs first, then mask both acceleration channels at every history step without mutating the caller.\n"
        "            ego_history = ego_history.clone()\n"
        "            ego_history[..., 3:5] = 0.0\n", ""
    ).replace("if cfg.model_id in (PHYSICAL_MODEL_ID, CANDIDATE_RANK_MODEL_ID, PHYSICAL_NO_ACCEL_MODEL_ID):",
              "if cfg.model_id in (PHYSICAL_MODEL_ID, CANDIDATE_RANK_MODEL_ID):")
    assert hashlib.sha256(source.rstrip().encode()).hexdigest() == "ef20379b8988b9dce166a10091675346beed7de009996e16fd7dfcf3659d193f"
    return source


@pytest.mark.parametrize("model_id", [MODEL_ID, PHYSICAL_MODEL_ID, CANDIDATE_RANK_MODEL_ID])
def test_old_ids_are_bitexact_to_the_frozen_pre_ablation_forward(model_id):
    namespace = dict(model_module.__dict__)
    exec(compile(textwrap.dedent(legacy_forward_source()), "<verified-legacy-forward>", "exec"), namespace)
    torch.manual_seed(20260903)
    model = PerspectiveTrajectoryModel(config(model_id)).eval()
    supplied = inputs(model.config)
    original_hash = state_hash(model)
    with torch.no_grad():
        expected = namespace["forward"](model, **supplied)
        actual = model(**supplied)
    assert all(torch.equal(first, second) for first, second in zip(actual, expected))
    assert state_hash(model) == original_hash


@pytest.mark.parametrize("name,expected_sha", [
    ("__init__", "58143fed88dd18d21624537df09eb1bd9cea3602f3b2951b910a47eba4e4fd21"),
    ("_check_inputs", "2a87c2b47830d051d7771869a8f191b16a7fa5a69231da75f953e1fd3ad4088a"),
    ("_decode_physical_v1", "ffa4930c7624432a68df3ce9cf2c4585d4f98f00fc11ec6394a13fcfaceac420"),
])
def test_initializer_input_validation_and_physical_decoder_remain_source_identical(name, expected_sha):
    assert hashlib.sha256(inspect.getsource(getattr(PerspectiveTrajectoryModel, name)).rstrip().encode()).hexdigest() == expected_sha


def test_secure_bundle_keeps_research_id_unsupported_and_rejects_semantic_hash_mismatch():
    research = config().to_dict()
    physical = {**research, "model_id": PHYSICAL_MODEL_ID}
    research_sha = training._canonical_sha256(research)
    physical_sha = training._canonical_sha256(physical)
    assert research_sha != physical_sha
    assert PHYSICAL_NO_ACCEL_MODEL_ID not in bundle.SUPPORTED_MODEL_IDS
    with pytest.raises(ContractError, match="not supported by bundle v1"):
        bundle._validate_model_config(research, research_sha)
    with pytest.raises(ContractError, match="does not match model_config"):
        bundle._validate_model_config(physical, research_sha)
    assert bundle._validate_model_config(physical, physical_sha) == physical


@pytest.mark.parametrize("stored_id,requested_id", [(PHYSICAL_MODEL_ID, PHYSICAL_NO_ACCEL_MODEL_ID),
                                                    (PHYSICAL_NO_ACCEL_MODEL_ID, PHYSICAL_MODEL_ID)])
def test_resume_cannot_relabel_identically_shaped_parameters_across_model_ids(monkeypatch, tmp_path, stored_id, requested_id):
    stored = config(stored_id).to_dict()
    payload = {"checkpoint_id": training.CHECKPOINT_ID, "dataset_fingerprint_sha256": "a" * 64,
               "corpus_fingerprint_sha256": "b" * 64, "model_config": stored,
               "model_config_sha256": training._canonical_sha256(stored)}
    monkeypatch.setattr(training, "_read_checkpoint_file", lambda *args: (payload, "c" * 64))
    with pytest.raises(ContractError, match="checkpoint model config does not match"):
        training._load_checkpoint(tmp_path / "unused.pt", model=None, optimizer=None,
            dataset_fingerprint_sha256="a" * 64, corpus_fingerprint_sha256="b" * 64,
            model_config=config(requested_id), train_config=training.TrainConfig(), loss_config=TrajectoryLossConfig(),
            device=torch.device("cpu"), training_split="train", training_episode_ids=["synthetic"],
            dataset_size=1, domain_indices={"carla": [0]}, sampling_plan={})
