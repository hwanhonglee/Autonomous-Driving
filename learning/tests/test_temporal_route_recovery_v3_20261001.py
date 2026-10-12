"""HH_261001 - Test a NEW causal recovery architecture, never trained quality or control approval."""
from dataclasses import replace
import hashlib

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.temporal_cached_encoder_v2 import CachedTemporalModelV2, CameraFrameIdentityV2, TemporalCacheWindowV2
from portable_e2e.temporal_model_v2 import TemporalModelConfigV2, NOMINAL_TIME_OFFSETS_S
from portable_e2e.temporal_route_recovery_v3_20261001 import (
    BRANCH_PREFIX, MODEL_ID, SCHEMA, EXPORT_SCHEMA, RouteRecoveryConfigV3,
    RouteRecoveryTemporalModelV3, causal_near_route_features,
)


@pytest.fixture(scope="module", autouse=True)
def single_thread():
    previous = torch.get_num_threads(); torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def route(offset=0., tangent=0.):
    value = torch.zeros(1, 128, 2)
    value[..., 0] = torch.arange(128) * .5
    value[..., 1] = offset + tangent * value[..., 0]
    return value, torch.ones(1, 128, dtype=torch.bool)


def example(step=0, offset=.3, slope=.02):
    anchor = 2_000_000_000 + step * 100_000_000
    stamps = [anchor + offset for offset in (-800_000_000, -600_000_000, -400_000_000, -200_000_000, 0)]
    frames, images = [], []
    for stamp in stamps:
        generator = torch.Generator().manual_seed(stamp // 100_000_000 + 909)
        images.append(torch.rand(6, 3, 180, 320, generator=generator))
        frames.append(CameraFrameIdentityV2(stamp, tuple(hashlib.sha256((str(stamp) + ":" + str(camera)).encode()).hexdigest()
                                                          for camera in range(6))))
    xy, mask = route(offset, slope)
    inputs = dict(images=torch.stack(images)[None], calibration=torch.zeros(1, 6, 16),
        ego_history=torch.zeros(1, 10, 13), ego_history_mask=torch.ones(1, 10, dtype=torch.bool),
        route_xy=xy, route_mask=mask, time_valid=torch.ones(1, 5, dtype=torch.bool),
        relative_pose=torch.zeros(1, 5, 3), time_offsets_s=torch.tensor(NOMINAL_TIME_OFFSETS_S)[None])
    window = TemporalCacheWindowV2("recovery-synthetic", "train", "rig", "f" * 64, anchor, tuple(frames))
    return inputs, window


@pytest.fixture
def models():
    config = TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)
    torch.manual_seed(3101); parent = CachedTemporalModelV2(config).eval()
    torch.manual_seed(3101); new = RouteRecoveryTemporalModelV3(config).eval()
    return parent, new


def test_config_declares_new_model_and_exact_recovery_scales():
    config = RouteRecoveryConfigV3()
    assert config.stations_forward_x_m == (2., 4., 8.)
    assert config.lateral_scale_m == 1. and config.tangent_scale_rad == .1
    assert RouteRecoveryConfigV3.from_mapping(config.to_dict()) == config
    with pytest.raises(ContractError): replace(config, hidden_width=64).validate()
    with pytest.raises(ContractError): replace(config, lateral_scale_m=True).validate()
    with pytest.raises(ContractError): replace(config, stations_forward_x_m=(2, 4, 8)).validate()
    with pytest.raises(ContractError): RouteRecoveryConfigV3.from_mapping(dict(config.to_dict(), true_future="forbidden"))


def test_signed_lateral_and_tangent_are_not_recentred_or_diluted():
    xy, mask = route(offset=.4, tangent=-.05)
    features = causal_near_route_features(xy, mask)
    assert features.shape == (1, 9)
    assert torch.allclose(features[0, [0, 3, 6]], torch.tensor([.3, .2, 0.]), atol=1e-6)
    assert torch.allclose(features[0, [1, 4, 7]], torch.atan(torch.tensor(-.05)).expand(3) / .1, atol=1e-6)
    assert torch.equal(features[0, [2, 5, 8]], torch.ones(3))
    inverse, _ = route(offset=-.4, tangent=.05)
    opposite = causal_near_route_features(inverse, mask)
    assert torch.allclose(opposite[0, [0, 1, 3, 4, 6, 7]], -features[0, [0, 1, 3, 4, 6, 7]], atol=1e-6)


def test_missing_station_is_zero_masked_without_future_extrapolation():
    xy, mask = route(offset=1., tangent=.1)
    mask[:, 6:] = False
    features = causal_near_route_features(xy, mask)
    assert features[0, 2] == 1 and torch.equal(features[0, 3:], torch.zeros(6))
    changed = xy.clone(); changed[:, 6:, :] = 12345.
    assert torch.equal(causal_near_route_features(changed, mask), features)


def test_first_valid_forward_crossing_is_used_not_later_loop_or_backward_segment():
    xy, mask = route()
    xy[:, :4, :] = torch.tensor([[-1., 9.], [3., 1.], [0., -8.], [5., -3.]])
    features = causal_near_route_features(xy, mask)
    assert torch.allclose(features[0, 0], torch.tensor(3.))
    assert features[0, 2] == 1


@pytest.mark.parametrize("invalid", ["nonfinite_masked", "wrong_shape", "numeric_mask", "broken_prefix", "short_prefix", "float64"])
def test_branch_route_guards_preserve_invalid_input_fail_closed(invalid):
    xy, mask = route()
    if invalid == "nonfinite_masked": mask[:, 80:] = False; xy[:, 127, 1] = float("nan")
    if invalid == "wrong_shape": xy = xy[:, :-1]
    if invalid == "numeric_mask": mask = mask.float()
    if invalid == "broken_prefix": mask[:, 3] = False
    if invalid == "short_prefix": mask[:, 1:] = False
    if invalid == "float64": xy = xy.double()
    with pytest.raises(ContractError): causal_near_route_features(xy, mask)


def test_zero_init_is_bitwise_parent_and_keeps_all_original_state_names(models):
    parent, new = models; inputs, _ = example()
    old = parent.state_dict(); state = new.state_dict()
    assert set(state) - set(old) == {BRANCH_PREFIX + suffix for suffix in ("0.weight", "0.bias", "2.weight", "2.bias")}
    assert all(torch.equal(value, state[key]) for key, value in old.items())
    with torch.no_grad(): expected, actual = parent(**inputs), new(**inputs)
    assert set(expected) == set(actual)
    assert all(expected[key].numpy().tobytes() == actual[key].numpy().tobytes() for key in expected)
    assert new._recovery_features is None and new._recovery_active is False


def test_permanent_new_hook_adds_logit_before_original_tanh_and_only_changes_curvature(models):
    parent, new = models; inputs, _ = example()
    with torch.no_grad(): new.route_recovery_branch[-1].bias.fill_(.25)
    logits = []
    handle = parent.curvature_head.register_forward_hook(lambda module, args, value: logits.append(value.detach().clone()))
    try:
        with torch.no_grad(): old, actual = parent(**inputs), new(**inputs)
    finally: handle.remove()
    assert torch.allclose(actual["curvature_knots"], .2 * torch.tanh(logits[0] + .25), atol=0., rtol=0.)
    assert float(actual["curvature_knots"].abs().max()) <= .2
    assert all(torch.equal(old[key], actual[key]) for key in old if key != "curvature_knots")
    assert len(parent.curvature_head._forward_hooks) == 0 and len(new.curvature_head._forward_hooks) == 1


def test_branch_gradients_flow_and_joint_backbone_remains_trainable(models):
    _, new = models; inputs, _ = example()
    new.train()
    result = new(**inputs)
    (result["curvature_knots"].square().mean() + result["preferred_speed_knots"].mean()).backward()
    assert new.route_recovery_branch[-1].weight.grad is not None
    assert bool(torch.isfinite(new.route_recovery_branch[-1].weight.grad).all())
    assert float(new.route_recovery_branch[-1].weight.grad.abs().sum()) > 0
    assert new.curvature_head.weight.grad is not None and new.speed_head.weight.grad is not None
    with torch.no_grad(): new.route_recovery_branch[-1].weight.fill_(.001)
    new.zero_grad(); new(**inputs)["curvature_knots"].mean().backward()
    assert float(new.route_recovery_branch[0].weight.grad.abs().sum()) > 0


def test_unavailable_all_station_support_cannot_create_bias_correction(models):
    parent, new = models; inputs, _ = example()
    inputs["route_mask"][:, 2:] = False
    with torch.no_grad():
        new.route_recovery_branch[-1].bias.fill_(20.)
        old, actual = parent(**inputs), new(**inputs)
    assert torch.equal(old["curvature_knots"], actual["curvature_knots"])


def test_one_backbone_encoder_pass_and_no_third_full_input_validation(models):
    _, new = models; inputs, window = example()
    cnn, validations = [], []
    handle = new.image_encoder.features.register_forward_pre_hook(lambda module, args: cnn.append(args[0].shape[0]))
    original = new._validate_inputs
    def observe(*args): validations.append(1); return original(*args)
    new._validate_inputs = observe
    try:
        with torch.no_grad(): new.forward_cached(inputs, window=window)
    finally: handle.remove()
    assert cnn == [30] and len(validations) == 2


def test_cached_api_enters_branch_despite_parent_super_forward_and_reuses_causal_images(models):
    _, new = models
    with torch.no_grad(): new.route_recovery_branch[-1].bias.fill_(.12)
    for step in range(4):
        inputs, window = example(step)
        with torch.no_grad(): normal, cached = new(**inputs), new.forward_cached(inputs, window=window)
        assert all(torch.allclose(normal[k], cached[k], atol=1e-6, rtol=1e-6) for k in normal)
        assert new.cache_stats.cnn_images == (30 if step < 2 else 6)
        assert new._recovery_active is False and new._recovery_features is None


def test_cached_entire_new_branch_data_change_triggers_guarded_reset(models):
    _, new = models; inputs, window = example()
    with torch.no_grad(): new.forward_cached(inputs, window=window)
    new.route_recovery_branch[-1].bias.data.add_(.05)
    with torch.no_grad(): new.forward_cached(inputs, window=window)
    assert new.cache_stats.reset_reason == "model_state_changed" and new.cache_stats.cold_start is True


@pytest.mark.parametrize("api", ["normal", "cached"])
@pytest.mark.parametrize("mutation", ["removed", "replaced", "extra"])
def test_permanent_recovery_hook_cannot_be_silently_disabled(models, api, mutation):
    _, new = models; inputs, window = example()
    with torch.no_grad(): new.forward_cached(inputs, window=window)
    if mutation == "removed": new._recovery_hook_handle.remove()
    elif mutation == "replaced": new.curvature_head._forward_hooks[new._recovery_hook_handle.id] = lambda *args: args[-1]
    else: new.curvature_head.register_forward_hook(lambda *args: args[-1])
    with torch.no_grad(), pytest.raises(ContractError, match="permanent recovery"):
        new(**inputs) if api == "normal" else new.forward_cached(inputs, window=window)
    assert new._recovery_active is False and new._recovery_features is None


@pytest.mark.parametrize("api", ["normal", "cached"])
def test_context_clears_even_invalid_input_failure_and_reentry_is_forbidden(models, api):
    _, new = models; inputs, window = example()
    bad = dict(inputs); bad["images"] = inputs["images"].clone(); bad["images"][0, 0, 0, 0, 0, 0] = float("nan")
    with torch.no_grad(), pytest.raises(ContractError):
        new(**bad) if api == "normal" else new.forward_cached(bad, window=window)
    assert new._recovery_features is None and new._recovery_active is False
    new._begin_recovery_call()
    try:
        with torch.no_grad(), pytest.raises(ContractError, match="not reentrant"):
            new(**inputs) if api == "normal" else new.forward_cached(inputs, window=window)
        assert new._recovery_active is True
    finally: new._end_recovery_call()
    with torch.no_grad(): new(**inputs)


def test_hook_failure_clears_context_and_future_timestamp_is_still_rejected(models):
    _, new = models; inputs, window = example()
    with torch.no_grad(): new.route_recovery_branch[-1].bias.fill_(float("nan"))
    with torch.no_grad(), pytest.raises(ContractError): new.forward_cached(inputs, window=window)
    assert new._recovery_features is None and new._recovery_active is False
    with torch.no_grad(): new.route_recovery_branch[-1].bias.zero_()
    with torch.no_grad(), pytest.raises(ContractError):
        new.forward_cached(inputs, window=replace(window, anchor_timestamp_ns=window.anchor_timestamp_ns - 100_000_000))


def test_backbone_warm_start_is_explicit_complete_and_never_overwrites_branch(models):
    parent, new = models
    new.load_backbone_state(parent.state_dict())
    assert all(torch.equal(value, new.state_dict()[key]) for key, value in parent.state_dict().items())
    with pytest.raises(ContractError): new.load_backbone_state(new.state_dict())
    with torch.no_grad(): new.route_recovery_branch[-1].bias.fill_(.1)
    with pytest.raises(ContractError, match="zero-output"): new.load_backbone_state(parent.state_dict())


def test_training_interface_does_not_fabricate_cohort_loader_or_approval(models):
    _, new = models; interface = new.training_interface()
    assert interface["schema"] == SCHEMA and interface["model_id"] == MODEL_ID
    assert interface["export_schema"] == EXPORT_SCHEMA
    assert interface["required_recovery_training_data"] is True
    assert interface["true_future_labels_loss_only"] is True
    assert interface["V2_numeric_loader_compatible"] is False
    assert interface["model_promoted"] is interface["simulator_control_authorized"] is False
    assert interface["curvature_bound_inv_m"] == .2
