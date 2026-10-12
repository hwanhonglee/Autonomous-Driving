"""HH_261004 - Prepare a synthetic CPU byte-parity proof for owned image normalization, never alter a frozen live runtime."""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import inspect
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image
import torch

from portable_e2e import runtime
from portable_e2e.model import ModelConfig


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.owned_image_normalization_synthetic_parity.20261004.v1"
TEST_SOURCE = ROOT / "learning/tests/test_owned_image_normalization_parity_v1_20261004.py"
FROZEN_SOURCE_SHA = {
    "learning/portable_e2e/runtime.py": "bd1f672efe5cf571744bd4c5ef7655435dc67dddadc41c23fddddb82d721a57b",
    "learning/portable_e2e/torch_dataset.py": "a81d26b1c9aaf67db56cf993a41857133dccac78e42047754e106cff4c8987c9",
}
PATTERNS = ("black", "white", "primaries", "byte_ramp", "checker", "seeded_noise")
ENCODINGS = ("rgb8", "bgra8")
NORMALIZATION_ORIGINAL = "image_tensors.append((tensor - mean) / std)"
NORMALIZATION_CANDIDATE = "image_tensors.append(tensor.sub_(mean).div_(std))"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tensor_bytes(tensor):
    require(tensor.device.type == "cpu" and tensor.dtype == torch.float32, "CPU float32 byte witness required")
    return tensor.detach().contiguous().numpy().tobytes(order="C")


def array_digest(array):
    return hashlib.sha256(array.tobytes(order="C")).hexdigest()


def storage_info(tensor):
    """HH_261004 - Preserve the same complete allocation proof on this workspace's older typed-storage Torch API."""
    if hasattr(tensor, "untyped_storage"):
        storage = tensor.untyped_storage()
        byte_count = storage.nbytes()
    else:
        storage = tensor.storage()
        byte_count = storage.size() * tensor.element_size()
    return dict(pointer=storage.data_ptr(), byte_count=byte_count)


def original_image_statements():
    """HH_261004 - Extract the actual eight image statements instead of inventing a new PIL or channel-conversion pipeline."""
    require(all(digest(ROOT / name) == sha for name, sha in FROZEN_SOURCE_SHA.items()), "frozen runtime/statistics bytes changed")
    tree = ast.parse(inspect.getsource(runtime.build_runtime_tensors))
    loop = next(node for node in tree.body[0].body if isinstance(node, ast.For))
    expected = (
        "array = _rgb_array(frame, camera_name=camera_name)",
        "image = Image.fromarray(array, mode='RGB')",
        "resized = image.resize((config.image_width, config.image_height), resample=resampling)",
        "resized_array = np.asarray(resized, dtype=np.float32).copy()",
        "tensor = torch.from_numpy(resized_array).permute(2, 0, 1).contiguous().div_(255.0)",
        "mean = tensor.new_tensor(RGB_MEAN).view(3, 1, 1)",
        "std = tensor.new_tensor(RGB_STD).view(3, 1, 1)",
        NORMALIZATION_ORIGINAL,
    )
    require(tuple(ast.unparse(node) for node in loop.body) == expected, "actual original PIL/ownedCHW/div255/normalization AST differs")
    require(runtime.RGB_MEAN == (.485, .456, .406) and runtime.RGB_STD == (.229, .224, .225), "original RGB statistics differ")
    return copy.deepcopy(loop.body)


def owned_prefix_witness(frame, array, resized_array, tensor, mean, std):
    """HH_261004 - In-place arithmetic is admissible only on the fresh non-aliased CHW allocation, not source pixels or shared statistics."""
    require(tensor.device.type == "cpu" and tensor.dtype == torch.float32 and tensor.ndim == 3
        and tensor.shape[0] == 3 and tensor.is_contiguous() and tensor._base is None
        and tensor.storage_offset() == 0 and not tensor.requires_grad
        and storage_info(tensor)["byte_count"] == tensor.numel() * tensor.element_size(), "fresh complete owned CPU CHW storage required")
    require(not np.shares_memory(tensor.numpy(), resized_array)
        and not np.shares_memory(tensor.numpy(), array)
        and not np.shares_memory(tensor.numpy(), frame.payload), "normalization work tensor aliases a source/resize array")
    require(mean.device.type == std.device.type == "cpu" and mean.dtype == std.dtype == torch.float32
        and mean.shape == std.shape == (3, 1, 1) and bool(torch.isfinite(mean).all())
        and bool(torch.isfinite(std).all()) and bool((std > 0).all())
        and len({storage_info(value)["pointer"] for value in (tensor, mean, std)}) == 3,
        "independent original finite mean/std allocations required")
    return dict(source_payload=frame.payload, rgb_array=array, resized_array=resized_array,
        work_tensor=tensor, mean=mean, std=std, source_SHA=array_digest(frame.payload),
        rgb_SHA=array_digest(array), resized_SHA=array_digest(resized_array),
        mean_bytes=tensor_bytes(mean), std_bytes=tensor_bytes(std), original_work_bytes=tensor_bytes(tensor),
        owned_work_storage_pointer=storage_info(tensor)["pointer"])


def compile_image_harness(candidate=False):
    """HH_261004 - Compile a standalone CPU harness of the unchanged source; never assign or activate any runtime function."""
    statements = original_image_statements()
    original = copy.deepcopy(statements)
    if candidate:
        statements[-1] = ast.parse(NORMALIZATION_CANDIDATE).body[0]
        inverse = copy.deepcopy(statements)
        inverse[-1] = copy.deepcopy(original[-1])
        require(ast.dump(ast.Module(body=inverse, type_ignores=[])) == ast.dump(ast.Module(body=original, type_ignores=[])),
                "candidate inverse AST changed original resize/channel/allocation/math statements")
    # HH_261004 - Identical added witnesses surround only the copied normalization in this isolated proof harness.
    statements.insert(-1, ast.parse("witness = owned_prefix_witness(frame, array, resized_array, tensor, mean, std)").body[0])
    body = ast.parse("image_tensors = []\nresampling = getattr(Image, 'Resampling', Image).BILINEAR").body + statements
    body += ast.parse("return image_tensors[0], witness").body
    function = ast.FunctionDef(name="image_only_normalization_harness", args=ast.arguments(posonlyargs=[],
        args=[ast.arg(arg=name) for name in ("frame", "config", "camera_name")], kwonlyargs=[], kw_defaults=[], defaults=[]),
        body=body, decorator_list=[])
    namespace = dict(runtime.build_runtime_tensors.__globals__, owned_prefix_witness=owned_prefix_witness)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 "SYNTHETIC_CPU_normalization_harness_only", "exec"), namespace)
    return namespace[function.name]


def synthetic_frame(pattern, encoding, readonly):
    """HH_261004 - Produce deterministic synthetic pixels only; no capture, dataset payload or model weight is accessed."""
    require(pattern in PATTERNS and encoding in ENCODINGS and type(readonly) is bool, "declared synthetic fixture required")
    rows, cols = np.indices((360, 640), dtype=np.uint32)
    if pattern == "black": rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    elif pattern == "white": rgb = np.full((360, 640, 3), 255, dtype=np.uint8)
    elif pattern == "primaries": rgb = np.stack((np.where(cols % 3 == 0, 255, 0), np.where(cols % 3 == 1, 255, 0), np.where(cols % 3 == 2, 255, 0)), axis=-1).astype(np.uint8)
    elif pattern == "byte_ramp": rgb = np.stack(((cols + rows) % 256, (3 * cols + rows) % 256, (cols + 5 * rows) % 256), axis=-1).astype(np.uint8)
    elif pattern == "checker": rgb = np.repeat((((cols // 3 + rows // 5) % 2) * 255)[..., None], 3, axis=-1).astype(np.uint8)
    else: rgb = np.random.default_rng(20261004).integers(0, 256, size=(360, 640, 3), dtype=np.uint8)
    if encoding == "bgra8":
        payload = np.empty((360, 640, 4), dtype=np.uint8)
        payload[..., :3], payload[..., 3] = rgb[..., ::-1], ((cols + rows) % 256).astype(np.uint8)
    else:
        payload = rgb
    if readonly:
        # HH_261004 - Immutable byte backing is stronger than toggling a writable-array flag.
        payload = np.frombuffer(payload.tobytes(), dtype=np.uint8).reshape(payload.shape)
    return runtime.RuntimeCameraFrame(timestamp_ns=100_000_000, payload=payload, encoding=encoding)


def verify_witness_unchanged(witness, *, candidate):
    require(array_digest(witness["source_payload"]) == witness["source_SHA"]
        and array_digest(witness["rgb_array"]) == witness["rgb_SHA"]
        and array_digest(witness["resized_array"]) == witness["resized_SHA"]
        and tensor_bytes(witness["mean"]) == witness["mean_bytes"]
        and tensor_bytes(witness["std"]) == witness["std_bytes"], "original pixels/resize/shared statistics changed")
    if not candidate:
        require(tensor_bytes(witness["work_tensor"]) == witness["original_work_bytes"], "original out-of-place input work tensor changed")


def synthetic_case(pattern, encoding, readonly, *, size=(320, 180)):
    # HH_261004 - Extra shape-only synthetic cases use valid grid metadata; no model is instantiated and the deployed320x180 ABI is unchanged.
    config = ModelConfig(image_width=size[0], image_height=size[1],
                         image_grid_width=5 if size == (320, 180) else 1,
                         image_grid_height=3 if size == (320, 180) else 1)
    config.validate()
    frame = synthetic_frame(pattern, encoding, readonly)
    original, candidate = compile_image_harness(False), compile_image_harness(True)
    baseline, reference_witness = original(frame, config, "CAM_FRONT")
    proposed, candidate_witness = candidate(frame, config, "CAM_FRONT")
    verify_witness_unchanged(reference_witness, candidate=False)
    verify_witness_unchanged(candidate_witness, candidate=True)
    a, b = tensor_bytes(baseline), tensor_bytes(proposed)
    require(a == b, "synthetic original/candidate normalized tensor bytes differ")
    require(proposed is candidate_witness["work_tensor"]
        and storage_info(proposed)["pointer"] == candidate_witness["owned_work_storage_pointer"]
        and storage_info(baseline)["pointer"] != reference_witness["owned_work_storage_pointer"]
        and storage_info(proposed)["pointer"] != reference_witness["owned_work_storage_pointer"],
        "candidate must reuse only its own private CHW storage")
    return dict(pattern=pattern, encoding=encoding, immutable_source_bytes=readonly, image_size=list(size),
        source_pixels_SHA256=reference_witness["source_SHA"], normalized_shape=list(proposed.shape),
        normalized_byte_count=len(b), normalized_tensor_SHA256=hashlib.sha256(b).hexdigest(),
        exact_tensor_bytes_equal=True, source_and_resized_arrays_unchanged=True, shared_mean_and_std_unchanged=True,
        private_CHW_work_tensor_intentionally_mutated=True, original_CHW_work_tensor_unchanged=True,
        output_does_not_alias_original_pixels=True, synthetic_input=True,
        actual_recorded_JPEG_or_BGRA_parity_proven=False, whole_model_or_closed_loop_parity_proven=False)


def prospective_contract():
    """HH_261004 - Keep later exact-JPEG evaluation prospective and separate from both synthetic proof and live BGRA parity."""
    return dict(schema=SCHEMA + ".prospective_contract", source_sha256=dict(FROZEN_SOURCE_SHA),
        original_expression=NORMALIZATION_ORIGINAL, candidate_expression=NORMALIZATION_CANDIDATE,
        actual_runtime_edited_or_activated=False, normalization_result_cache_added=False,
        camera_SHA_or_tensor_SHA_omitted=False, GPU_or_model_inference_or_training=False,
        prepared_synthetic_scope=dict(patterns=list(PATTERNS), encodings=list(ENCODINGS),
            readonly_and_writable_sources=True, image_width=320, image_height=180, expected_cases=24,
            PIL_resize_CHW_div255_unchanged=True, exact_bytes_not_allclose=True,
            source_pixels_and_resize_allocation_and_shared_mean_std_preservation_required=True),
        later_exact_JPEG_counterfactual=dict(status="PREPARED_NOT_EXECUTED_NOT_AUTHORIZED_BY_THIS_HELPER",
            actual_archive_required="one separately declared immutable complete450x6 camera interval",
            all2700_JPEG_images_required=True, source_SHA_pinned_before_payload_read=True,
            malformed_or_failed_cases_retained_not_skipped=True,
            same_original_decode_resize_CHW_div255_pipeline_required=True,
            exact_every_normalized_tensor_bytes_and_SHA_required=True,
            privately_owned_CHW_allocation_proof_required=True, source_and_shared_statistic_immutability_required=True,
            paired_original_candidate_order="prospectively fixed AB/BA; no favorable subset selection",
            own_process_thread_readback_and_affinity_recorded=True,
            timing_gains_not_proven_yet=True, real_live_BGRA_source_parity_proven=False,
            JPEG_lossy_decode_not_identical_to_original_BGRA=True,
            no_runtime_replacement_or_model_adoption_from_JPEG_counterfactual=True),
        formal_function_approval=False, vehicle_control_approved=False, model_promoted=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--prospective-only", action="store_true")
    group.add_argument("--execute-synthetic", action="store_true")
    args = parser.parse_args(argv)
    before = {name: digest(ROOT / name) for name in FROZEN_SOURCE_SHA}
    require(before == FROZEN_SOURCE_SHA, "reviewed original runtime/statistics bytes differ")
    contract = prospective_contract()
    if args.execute_synthetic:
        require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "explicit GPU-disabled CPU proof required")
        contract["synthetic_results"] = [synthetic_case(pattern, encoding, readonly)
            for pattern in PATTERNS for encoding in ENCODINGS for readonly in (False, True)]
        contract["synthetic_case_count"] = len(contract["synthetic_results"])
        contract["all_synthetic_cases_exact_byte_parity_passed"] = True
        contract["CPU_thread_readback"] = dict(intra_op=torch.get_num_threads(), inter_op=torch.get_num_interop_threads())
    require(before == {name: digest(ROOT / name) for name in FROZEN_SOURCE_SHA}, "frozen source bytes changed during proof")
    contract["source_bytes_unchanged"] = True
    contract["synthetic_checks_executed"] = args.execute_synthetic
    print(json.dumps(contract, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
