"""HH_261001 - Test true-label, causal-preview, deterministic-resume and bounded training contracts."""

import math

import pytest
import torch

from portable_e2e.contract import ContractError
from scripts.e2e import train_temporal_v2_low_speed_joint_20261001 as trainer


def test_causal_support_is_route_arc_not_expert_future_or_origin_distance():
    xy = torch.tensor([[[100., 0.], [103., 0.], [103., 4.], [999., 999.]]])
    valid = torch.tensor([[True, True, True, False]])
    assert trainer.causal_route_support(xy, valid).item() == 7.


def test_navigation_support_conservatively_caps_decoder_spatial_domain():
    xy = torch.tensor([[[0., 0.], [100., 0.]]])
    assert trainer.causal_route_support(xy, torch.ones(1, 2, dtype=torch.bool)).item() == 64.


@pytest.mark.parametrize("mask", [[True, False, True], [True, False, False], [False, False, False]])
def test_navigation_needs_contiguous_actual_points(mask):
    with pytest.raises(ContractError):
        trainer.causal_route_support(torch.zeros(1, 3, 2), torch.tensor([mask]))


def test_navigation_nonfinite_payload_is_rejected():
    xy = torch.zeros(1, 3, 2); xy[0, 2, 0] = float("nan")
    with pytest.raises(ContractError, match="finite"):
        trainer.causal_route_support(xy, torch.ones(1, 3, dtype=torch.bool))


def test_measured_speed_uses_observed_planar_reference():
    history = torch.zeros(1, 10, 13); history[0, -1, 1:3] = torch.tensor([1.2, .5])
    batch = dict(ego_history=history, ego_history_mask=torch.ones(1, 10, dtype=torch.bool))
    assert trainer.measured_current_speed(batch).item() == pytest.approx(1.3)


def test_measured_v0_is_not_clipped_into_decoder_envelope():
    history = torch.zeros(1, 10, 13); history[0, -1, 1] = 2.5001
    with pytest.raises(ContractError, match="no clipping"):
        trainer.measured_current_speed(dict(ego_history=history, ego_history_mask=torch.ones(1, 10, dtype=torch.bool)))


def test_preview_prior_changes_no_labels_or_raw_values_and_detaches_boundary():
    raw = torch.ones(1, 33, requires_grad=True)
    with torch.no_grad():
        raw[0, 8:10] = 3.
    valid = torch.arange(33)[None] <= 7
    original, original_mask = raw.detach().clone(), valid.clone()
    result = trainer.preview_speed_regularizer(raw, valid)
    result.backward()
    assert result.item() == pytest.approx(1.5)
    assert torch.equal(raw.detach(), original) and torch.equal(valid, original_mask)
    assert raw.grad[0, 7].item() == 0.
    assert raw.grad[0, 8].item() > 0. and raw.grad[0, 9].item() > 0.
    assert torch.equal(raw.grad[0, 10:], torch.zeros(23))


def test_no_unobserved_preview_prior_when_all_active_knots_have_true_labels():
    raw = torch.ones(2, 33, requires_grad=True)
    valid = torch.arange(33)[None].expand(2, 33) <= 9
    result = trainer.preview_speed_regularizer(raw, valid)
    result.backward()
    assert result.item() == 0. and torch.equal(raw.grad, torch.zeros_like(raw))


def test_preview_prior_requires_actual_nonempty_prefix():
    valid = torch.zeros(1, 33, dtype=torch.bool); valid[0, 5] = True
    with pytest.raises(ContractError, match="prefix"):
        trainer.preview_speed_regularizer(torch.ones(1, 33), valid)


def test_exact_batch_sampler_visits_all_327_anchors_without_dropping_epoch_tail():
    batches = list(trainer.deterministic_batches(327, 500))
    first_cycle = [i for b in batches for i in b][:327]
    assert len(batches) == 500 and all(len(b) == 4 for b in batches)
    assert sorted(first_cycle) == list(range(327))
    assert batches == list(trainer.deterministic_batches(327, 500))


def test_resume_sampling_is_exact_suffix_not_a_new_shuffle():
    full = list(trainer.deterministic_batches(327, 1500))
    assert list(trainer.deterministic_batches(327, 1500, start_update=500)) == full[500:]


def test_learning_rate_keeps_fixed_3000_update_horizon():
    assert trainer.learning_rate(1) == pytest.approx(3e-7)
    assert trainer.learning_rate(100) == pytest.approx(3e-5)
    assert trainer.learning_rate(3000) == pytest.approx(3e-6)
    assert trainer.learning_rate(500) > trainer.learning_rate(1500) > trainer.learning_rate(2500)


@pytest.mark.parametrize("updates", [0, 200, 499, 3001, 4000])
def test_unplanned_training_extent_is_refused_before_data_read(tmp_path, updates):
    with pytest.raises(ValueError, match="bounded"):
        trainer.main(["--train", str(tmp_path), "--train-manifest-sha256", "0"*64,
            "--conversion-result-sha256", "0"*64, "--known-dev", str(tmp_path),
            "--parent-checkpoint", str(tmp_path/"parent.pt"), "--output", str(tmp_path/"output"),
            "--updates", str(updates)])


def test_output_is_create_only(tmp_path):
    output = tmp_path/"result"; output.mkdir()
    with pytest.raises(ValueError, match="create-only"):
        trainer.main(["--train", str(tmp_path), "--train-manifest-sha256", "0"*64,
            "--conversion-result-sha256", "0"*64, "--known-dev", str(tmp_path),
            "--parent-checkpoint", str(tmp_path/"parent.pt"), "--output", str(output)])


def test_no_gpu_or_remote_job_is_started_by_import():
    assert trainer.GPU_UUID == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
    assert trainer.BATCH_SIZE == 4 and trainer.MAXIMUM_UPDATES == 3000


def test_numeric_export_roundtrip_is_pickle_free_and_legacy_compatible(tmp_path):
    from types import SimpleNamespace
    from portable_e2e.temporal_model_v2 import TemporalModelV2
    model = TemporalModelV2()
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-5)
    result = trainer.save_checkpoint(tmp_path, 500, model, optimizer,
        SimpleNamespace(fingerprint_sha256="f"*64), {}, "a"*64, 2000)
    with trainer.np.load(result["export_npz"], allow_pickle=False) as archive:
        assert set(archive.files) == set(model.state_dict())
        assert all(archive[name].dtype == trainer.np.float32 for name in archive.files)
    metadata = trainer.read(result["export_metadata"])
    assert metadata["model_config"] == model.config.to_dict()
    assert metadata["checkpoint_sha256"] == result["sha256"]
    assert metadata["npz_sha256"] == trainer.digest(result["export_npz"])
    assert metadata["stop_intent_trained"] is False
    assert metadata["simulation_only"] is True and metadata["model_promoted"] is False
    assert trainer.MAX_WALL_SECONDS == 2700


def test_joint_decoder_uses_unchanged_true_labels_and_has_finite_nonzero_gradients():
    from portable_e2e.path_time_decoder_v2 import PathTimeDecoderV2
    class RawModel(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.curvature = torch.nn.Parameter(torch.full((1, 33), .02))
            self.speed = torch.nn.Parameter(torch.full((1, 33), 1.2))
        def forward(self, **unused):
            return dict(curvature_knots=self.curvature, preferred_speed_knots=self.speed,
                        intent_logits=torch.zeros(1, 3), stop_distance_raw=torch.zeros(1, 1),
                        acceleration_prediction=torch.zeros(1, 1))
    batch = {name: torch.empty(0) for name in trainer.FIELDS}
    history = torch.zeros(1, 10, 13); history[:, -1, 1] = 1.
    positions = torch.zeros(1, 64, 2); positions[0, :, 0] = torch.arange(1, 65)*.1
    route = torch.zeros(1, 128, 2); route[0, :, 0] = torch.arange(128)*2.
    mask = torch.arange(33)[None] <= 3
    batch.update(ego_history=history, ego_history_mask=torch.ones(1, 10, dtype=torch.bool),
        route_xy=route, route_mask=torch.ones(1, 128, dtype=torch.bool), target_xy=positions,
        target_speed_mps=torch.ones(1, 64), target_valid=torch.ones(1, 64, dtype=torch.bool),
        spatial_curvature=torch.zeros(1, 33), spatial_preferred_speed=torch.ones(1, 33),
        spatial_valid=mask, spatial_curvature_valid=mask.clone(), spatial_speed_valid=mask.clone(),
        intent_target=torch.zeros(1, dtype=torch.long), intent_valid=torch.zeros(1, dtype=torch.bool),
        stop_distance_target=torch.zeros(1), stop_distance_valid=torch.zeros(1, dtype=torch.bool))
    saved = {name: value.clone() for name, value in batch.items()}
    model = RawModel()
    _, decoded, losses = trainer.joint_forward(model, PathTimeDecoderV2(), batch)
    losses["total"].backward()
    assert decoded["instantaneous_speed"][0, 0].item() == 1.
    assert losses["projected_time_supervision"] is True
    assert bool(torch.isfinite(model.curvature.grad).all()) and model.curvature.grad.abs().sum() > 0
    assert bool(torch.isfinite(model.speed.grad).all()) and model.speed.grad.abs().sum() > 0
    assert all(torch.equal(saved[name], value) for name, value in batch.items())


@pytest.mark.parametrize("field,value", [
    ("examples_seen", 1999), ("parent_checkpoint_sha256", "0"*64),
    ("stop_intent_trained", True), ("runtime_intent", "untrained_head"),
    ("training_only", False), ("one_episode_screening", False), ("main_training", True)])
def test_resume_rejects_wrong_lineage_or_exposure_accounting(tmp_path, field, value):
    document = dict(schema=trainer.SCHEMA, decoder_integrated=True, optimizer_updates=500,
        examples_seen=2000, parent_checkpoint_sha256=trainer.PARENT_SHA, stop_intent_trained=False,
        runtime_intent="explicit_cruise_GO_not_head", training_only=True, one_episode_screening=True,
        main_training=False, optimizer_state_dict={}, vehicle_control_approved=False, source_sha256={})
    document[field] = value
    checkpoint = tmp_path/"wrong.pt"; torch.save(document, checkpoint)
    with pytest.raises(ValueError, match="lineage"):
        trainer.verify_checkpoint(checkpoint, trainer.digest(checkpoint), resume=True)
