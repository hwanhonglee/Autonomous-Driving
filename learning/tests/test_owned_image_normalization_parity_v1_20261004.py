"""HH_261004 - Verify synthetic owned-normalization exact bytes and immutability without datasets, weights, GPU or live activation."""

import ast
import copy
import inspect
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from scripts.e2e import audit_owned_image_normalization_parity_v1_20261004 as proof


def test_extracted_statements_are_the_actual_frozen_original_pipeline():
    assert len(proof.original_image_statements()) == 8
    assert all(proof.digest(proof.ROOT / name) == sha for name, sha in proof.FROZEN_SOURCE_SHA.items())
    assert ast.unparse(proof.original_image_statements()[-1]) == proof.NORMALIZATION_ORIGINAL


@pytest.mark.parametrize("pattern", proof.PATTERNS)
@pytest.mark.parametrize("encoding", proof.ENCODINGS)
@pytest.mark.parametrize("readonly", [False, True])
def test_every_declared_synthetic_original_vs_candidate_normalized_byte_is_identical(pattern, encoding, readonly):
    result = proof.synthetic_case(pattern, encoding, readonly)
    assert result["exact_tensor_bytes_equal"] is True
    assert result["normalized_byte_count"] == 3 * 320 * 180 * 4
    assert result["source_and_resized_arrays_unchanged"] is True
    assert result["shared_mean_and_std_unchanged"] is True
    assert result["original_CHW_work_tensor_unchanged"] is True
    assert result["private_CHW_work_tensor_intentionally_mutated"] is True
    assert result["synthetic_input"] is True
    assert result["actual_recorded_JPEG_or_BGRA_parity_proven"] is False
    assert result["whole_model_or_closed_loop_parity_proven"] is False


@pytest.mark.parametrize("size", [(16, 16), (64, 32), (320, 180), (640, 360)])
def test_no_resize_or_channel_order_shortcut_is_needed_for_byte_parity(size):
    result = proof.synthetic_case("byte_ramp", "bgra8", True, size=size)
    assert result["normalized_shape"] == [3, size[1], size[0]]
    assert result["exact_tensor_bytes_equal"] is True


def test_deterministic_RGB_and_BGRA_sources_yield_the_same_normalized_RGB_bytes():
    a = proof.synthetic_case("seeded_noise", "rgb8", True)
    b = proof.synthetic_case("seeded_noise", "bgra8", True)
    assert a["normalized_tensor_SHA256"] == b["normalized_tensor_SHA256"]
    assert a["source_pixels_SHA256"] != b["source_pixels_SHA256"]


def test_immutable_source_is_backed_by_bytes_and_cannot_be_made_writable():
    frame = proof.synthetic_frame("checker", "bgra8", True)
    with pytest.raises(ValueError):
        frame.payload.flags.writeable = True
    assert frame.payload.flags.writeable is False


@pytest.mark.parametrize("fault", ["noncontiguous", "view", "numpy_alias", "statistics_alias", "zero_std", "wrong_dtype"])
def test_inplace_guard_denies_unknown_or_shared_work_storage_and_invalid_statistics(fault):
    frame = proof.synthetic_frame("byte_ramp", "rgb8", False)
    output, witness = proof.compile_image_harness(False)(frame, proof.ModelConfig(), "CAM_FRONT")
    tensor, mean, std = witness["work_tensor"], witness["mean"], witness["std"]
    if fault == "noncontiguous": tensor = tensor.transpose(1, 2)
    elif fault == "view": tensor = tensor.view_as(tensor)
    elif fault == "numpy_alias": tensor = torch.from_numpy(witness["resized_array"]).permute(2, 0, 1)
    elif fault == "statistics_alias": mean = tensor.reshape(-1)[:3].reshape(3, 1, 1)
    elif fault == "zero_std": std = torch.zeros_like(std)
    else: tensor = tensor.to(torch.float64)
    before = proof.array_digest(frame.payload)
    with pytest.raises(RuntimeError):
        proof.owned_prefix_witness(frame, witness["rgb_array"], witness["resized_array"], tensor, mean, std)
    assert proof.array_digest(frame.payload) == before


def test_shared_mean_and_std_remain_unchanged_across_independent_owned_image_operations():
    # HH_261004 - This scalar test does not introduce a live statistics cache; it proves PyTorch operands are not mutated.
    mean = torch.tensor(proof.runtime.RGB_MEAN, dtype=torch.float32).view(3, 1, 1)
    std = torch.tensor(proof.runtime.RGB_STD, dtype=torch.float32).view(3, 1, 1)
    original_mean, original_std = proof.tensor_bytes(mean), proof.tensor_bytes(std)
    for value in (0.0, 1 / 255, 127 / 255, 1.0):
        owned = torch.full((3, 16, 16), value, dtype=torch.float32)
        baseline = (owned - mean) / std
        owned.sub_(mean).div_(std)
        assert proof.tensor_bytes(baseline) == proof.tensor_bytes(owned)
        assert proof.tensor_bytes(mean) == original_mean and proof.tensor_bytes(std) == original_std


def test_harness_compilation_does_not_replace_or_activate_any_runtime_function():
    original = proof.runtime.build_runtime_tensors
    namespace_ids = {name: id(value) for name, value in original.__globals__.items()}
    source = proof.digest(Path(proof.runtime.__file__))
    proof.compile_image_harness(False)
    proof.compile_image_harness(True)
    assert proof.runtime.build_runtime_tensors is original
    assert {name: id(value) for name, value in original.__globals__.items()} == namespace_ids
    assert proof.digest(Path(proof.runtime.__file__)) == source


def test_prospective_JPEG_plan_is_not_live_BGRA_parity_or_runtime_adoption():
    contract = proof.prospective_contract()
    assert contract["prepared_synthetic_scope"]["expected_cases"] == 24
    later = contract["later_exact_JPEG_counterfactual"]
    assert later["all2700_JPEG_images_required"] is True
    assert later["source_SHA_pinned_before_payload_read"] is True
    assert later["JPEG_lossy_decode_not_identical_to_original_BGRA"] is True
    assert later["real_live_BGRA_source_parity_proven"] is False
    assert later["timing_gains_not_proven_yet"] is True
    assert contract["camera_SHA_or_tensor_SHA_omitted"] is False
    assert contract["actual_runtime_edited_or_activated"] is False
    assert contract["model_promoted"] is False and contract["formal_function_approval"] is False


def test_helper_has_no_dataset_payload_or_checkpoint_reader_and_no_CUDA_calls():
    tree = ast.parse(Path(proof.__file__).read_text())
    forbidden = ("load_runtime_weight_bundle", "torch.load", "load_contract", "load_rig_calibration",
        "_read_json_and_sha256", "configure_runtime_execution", "torch.cuda", "Image.open", "np.load")
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = ast.unparse(node.func)
            assert not any(name == item or name.startswith(item + ".") for item in forbidden)


def test_cli_prospective_only_executes_no_synthetic_math_or_outputs_files(monkeypatch, capsys):
    monkeypatch.setattr(proof, "synthetic_case", lambda *args, **kwargs: pytest.fail("unexpected proof execution"))
    proof.main(["--prospective-only"])
    result = json.loads(capsys.readouterr().out)
    assert result["synthetic_checks_executed"] is False and result["source_bytes_unchanged"] is True
    assert "synthetic_results" not in result


def test_cli_requires_explicit_CPU_disabled_execution_and_rejects_ambiguous_modes(monkeypatch):
    monkeypatch.delenv("CUDA_VISIBLE_DEVICES", raising=False)
    with pytest.raises(RuntimeError, match="GPU-disabled"):
        proof.main(["--execute-synthetic"])
    with pytest.raises(SystemExit):
        proof.main([])
    with pytest.raises(SystemExit):
        proof.main(["--execute-synthetic", "--prospective-only"])
