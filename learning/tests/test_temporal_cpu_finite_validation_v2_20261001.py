"""HH_261001 - Verify finite-predicate branch parity, every original guard, source mutation checks and final3000 raw equality."""

import ast
import copy
import hashlib
import inspect
from pathlib import Path
import textwrap

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.temporal_cached_encoder_v2 import (
    CachedTemporalModelV2, CameraFrameIdentityV2, TemporalCacheWindowV2)
from portable_e2e.temporal_model_v2 import TemporalModelV2, TemporalModelConfigV2
from portable_e2e.temporal_simulation_research_v2_20261001 import load_npz
from portable_e2e import temporal_cpu_finite_validation_v2_20261001 as proposed


@pytest.fixture(autouse=True)
def one_thread():
    previous = torch.get_num_threads(); torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def model():
    torch.manual_seed(7)
    result = CachedTemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128)).eval()
    result._research_temporal_checkpoint_sha256 = "1"*64
    result._research_temporal_export_npz_sha256 = "2"*64
    result._research_temporal_metadata_sha256 = "3"*64
    result._research_temporal_export_metadata = {"unit_fixture_not_source_approval": True}
    return result


@pytest.fixture
def inputs():
    result = dict(images=torch.zeros(1, 5, 6, 3, 180, 320), calibration=torch.zeros(1, 6, 16),
        ego_history=torch.zeros(1, 10, 13), ego_history_mask=torch.ones(1, 10, dtype=torch.bool),
        route_xy=torch.zeros(1, 128, 2), route_mask=torch.ones(1, 128, dtype=torch.bool),
        time_valid=torch.ones(1, 5, dtype=torch.bool), relative_pose=torch.zeros(1, 5, 3),
        time_offsets_s=torch.tensor([[-.8, -.6, -.4, -.2, 0.]]))
    result["route_xy"][0, :, 0] = torch.arange(128)
    return result


def window():
    stamps = (1_200_000_000, 1_400_000_000, 1_600_000_000, 1_800_000_000, 2_000_000_000)
    frames = tuple(CameraFrameIdentityV2(stamp,
        tuple(hashlib.sha256(f"fixture:{stamp}:{camera}".encode()).hexdigest() for camera in range(6))) for stamp in stamps)
    return TemporalCacheWindowV2("fixture", "train", "rig", "a"*64, stamps[-1], frames)


def test_ast_proves_every_original_validation_statement_except_finite_predicate_is_unchanged():
    old = ast.parse(textwrap.dedent(inspect.getsource(TemporalModelV2._validate_inputs)))
    new = ast.parse(textwrap.dedent(inspect.getsource(proposed.CPUFiniteCachedTemporalModelV2._validate_inputs)))
    original = ast.parse("if not bool(torch.isfinite(value).all().item()): pass").body[0].test
    replacement = ast.parse("if not cpu_float32_all_finite(value): pass").body[0].test
    class OnlyFinite(ast.NodeTransformer):
        count = 0
        def visit_If(self, node):
            if ast.dump(node.test) == ast.dump(original):
                node.test = copy.deepcopy(replacement); self.count += 1
            return self.generic_visit(node)
    patch = OnlyFinite(); old = patch.visit(old)
    assert patch.count == 1
    assert ast.dump(old, include_attributes=False) == ast.dump(new, include_attributes=False)


@pytest.mark.parametrize("layout", ["dense", "transpose", "stride", "offset", "readonly_numpy"])
@pytest.mark.parametrize("bad", [None, float("nan"), float("inf"), -float("inf")])
def test_predicate_equals_Torch_for_strides_offsets_and_nonfinite_values(layout, bad):
    value = torch.arange(100, dtype=torch.float32).reshape(10, 10)
    if bad is not None: value[5, 5] = bad
    if layout == "transpose": value = value.t()
    elif layout == "stride": value = value[:, ::2]
    elif layout == "offset": value = value[3:, 4:]
    elif layout == "readonly_numpy":
        view = value.numpy(); view.flags.writeable = False
    assert proposed.cpu_float32_all_finite(value) == bool(torch.isfinite(value).all().item())


@pytest.mark.parametrize("dtype", [torch.float64, torch.float16, torch.int64, torch.bool])
def test_unknown_dtype_is_fail_closed_not_silently_narrowed(dtype):
    with pytest.raises(ContractError, match="float32 CPU ABI"):
        proposed.cpu_float32_all_finite(torch.zeros(3, dtype=dtype))


@pytest.mark.parametrize("field", ["images", "calibration", "ego_history", "route_xy", "relative_pose", "time_offsets_s"])
def test_all_nonfinite_fields_are_rejected_even_in_masked_slots(inputs, field):
    old, new = model(), proposed.upgrade_cpu_finite_model(model())
    inputs[field].reshape(-1)[0] = float("nan")
    inputs["time_valid"][0, 0] = False
    inputs["ego_history_mask"][0, 0] = False
    for candidate in (old, new):
        with pytest.raises(ContractError, match="only finite values, including masked slots"):
            candidate._validate_inputs(inputs, "temporal")


@pytest.mark.parametrize("kind", ["images_shape", "numeric_mask", "wrong_dtype", "current_camera_missing",
    "current_ego_missing", "route_hole", "future_offset", "duplicate_offset", "nonpinned_offset",
    "current_pose", "yaw_radians", "history_mode"])
def test_remaining_original_branches_report_same_rejection(inputs, kind):
    history = "temporal"
    if kind == "images_shape": inputs["images"] = inputs["images"][:, :4]
    elif kind == "numeric_mask": inputs["time_valid"] = inputs["time_valid"].float()
    elif kind == "wrong_dtype": inputs["images"] = inputs["images"].double()
    elif kind == "current_camera_missing": inputs["time_valid"][0, -1] = False
    elif kind == "current_ego_missing": inputs["ego_history_mask"][0, -1] = False
    elif kind == "route_hole": inputs["route_mask"][0, 5] = False
    elif kind == "future_offset": inputs["time_offsets_s"][0, -1] = .01
    elif kind == "duplicate_offset": inputs["time_offsets_s"][0, 1] = -.8
    elif kind == "nonpinned_offset": inputs["time_offsets_s"][0, 0] = -.9
    elif kind == "current_pose": inputs["relative_pose"][0, -1, 0] = .01
    elif kind == "yaw_radians": inputs["relative_pose"][0, 0, 2] = 3.2
    elif kind == "history_mode": history = "future"
    messages = []
    for candidate in (model(), proposed.upgrade_cpu_finite_model(model())):
        with pytest.raises(ContractError) as error: candidate._validate_inputs(inputs, history)
        messages.append(str(error.value))
    assert messages[0] == messages[1]


def test_both_full_validations_are_retained_on_every_cached_call(inputs, monkeypatch):
    candidate = proposed.upgrade_cpu_finite_model(model())
    original = proposed.cpu_float32_all_finite
    calls = []
    def observed(tensor):
        calls.append(tuple(tensor.shape)); return original(tensor)
    monkeypatch.setattr(proposed, "cpu_float32_all_finite", observed)
    with torch.no_grad(): candidate.forward_cached(inputs, window=window())
    assert calls.count((1, 5, 6, 3, 180, 320)) == 2
    assert len(calls) == 12


def test_mutation_after_first_validation_is_seen_by_retained_second_scan(inputs, monkeypatch):
    candidate = proposed.upgrade_cpu_finite_model(model())
    original = proposed.cpu_float32_all_finite
    calls = []
    def mutate_between(tensor):
        result = original(tensor); calls.append(tuple(tensor.shape))
        if len(calls) == 6:
            inputs["images"][0, 0, 0, 0, 0, 0] = float("nan")
        return result
    monkeypatch.setattr(proposed, "cpu_float32_all_finite", mutate_between)
    with torch.no_grad(), pytest.raises(ContractError, match="only finite values"):
        candidate.forward_cached(inputs, window=window())


def test_full_weight_and_normalized_input_SHA_mutation_guards_remain(inputs):
    candidate = proposed.upgrade_cpu_finite_model(model())
    with torch.no_grad():
        candidate.forward_cached(inputs, window=window())
        candidate.forward_cached(inputs, window=window())
        assert candidate.cache_stats.cnn_images == 0
        inputs["images"][0, 0, 0, 0, 0, 0] = 1.
        candidate.forward_cached(inputs, window=window())
        assert candidate.cache_stats.cnn_images == 6 and candidate.cache_stats.reused_frame_groups == 4
        candidate.curvature_head.bias.data.add_(.001)
        candidate.forward_cached(inputs, window=window())
        assert candidate.cache_stats.reset_reason == "model_state_changed" and candidate.cache_stats.cold_start


def test_upgrade_preserves_exact_state_provenance_rng_and_original_cache_independence():
    original = model(); state = torch.random.get_rng_state().clone()
    candidate = proposed.upgrade_cpu_finite_model(original)
    assert torch.equal(torch.random.get_rng_state(), state)
    assert all(torch.equal(value, candidate.state_dict()[name]) for name, value in original.state_dict().items())
    assert all(value.data_ptr() != candidate.state_dict()[name].data_ptr() for name, value in original.state_dict().items())
    assert candidate._research_temporal_checkpoint_sha256 == original._research_temporal_checkpoint_sha256
    candidate._research_temporal_export_metadata["test"] = True
    assert "test" not in original._research_temporal_export_metadata


def test_actual_final3000_raw_outputs_equal_frozen_network_within_1e_minus6(inputs):
    root = Path(__file__).resolve().parents[2]
    export = root/"storage/experiments/training/2026-10-01/temporal_v2_low_speed_joint_v2/exports/step_003000"
    if not export.is_dir(): pytest.skip("final3000 export is unavailable on this independent checkout")
    original = load_npz(export/"model_state.npz", export/"metadata.json",
        weights_sha256="768c10aefef21b46587d34da79896dcf1d6acf0a7373eab27045d6c4e2460c90",
        metadata_sha256="eb74e50c5150b374aaa21bced41fd703507ed16c9fa863fd4a06e9a5896590b7")
    candidate = proposed.upgrade_cpu_finite_model(original)
    with torch.no_grad():
        for _ in range(2):
            first = original.forward_cached(inputs, window=window())
            second = candidate.forward_cached(inputs, window=window())
            for name in first:
                assert torch.allclose(first[name], second[name], atol=1.e-6, rtol=0.), name
                assert torch.equal(first[name], second[name]), name
