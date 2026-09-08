"""HH_260906 - Independently quantify CPU selector permutation roundoff without changing source or gates."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

# HH_260906 - Match python -m pytest/interactive import precedence, not the installed deployment package.
sys.path.insert(0, str(Path.cwd()))

import torch


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compare(actual, expected):
    # HH_260906 - Signed integer bit distance is an ULP distance for these finite same-sign pairs.
    actual, expected = actual.detach(), expected.detach()
    assert torch.isfinite(actual).all() and torch.isfinite(expected).all()
    assert torch.equal(torch.signbit(actual), torch.signbit(expected))
    delta = (actual - expected).abs()
    integer = torch.int32 if actual.dtype == torch.float32 else torch.int64
    ulp = (actual.contiguous().view(integer).long() - expected.contiguous().view(integer).long()).abs()
    eps = torch.finfo(actual.dtype).eps
    return {"bitwise_equal": torch.equal(actual, expected), "unequal_count": int((actual != expected).sum()),
        "max_abs": float(delta.max()), "max_relative": float((delta / expected.abs().clamp_min(torch.finfo(actual.dtype).tiny)).max()),
        "max_ulp": int(ulp.max()), "dtype_epsilon": eps, "same_sign_all_pairs": True,
        "within_proposed_dtype_tolerance": torch.allclose(actual, expected, atol=eps, rtol=4 * eps)}


paths = ("portable_e2e/frozen_selector.py", "tests/test_portable_e2e_frozen_selector.py", "scripts/e2e/env.sh")
before = {p: digest(p) for p in paths}
spec = importlib.util.spec_from_file_location("independent_frozen_selector_fixture", paths[1])
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
torch.set_num_threads(1)
rows = []
for dtype in (torch.float32, torch.float64):
    for order in ([2, 5, 1, 3, 4, 0], [5, 4, 3, 2, 1, 0]):
        data = fixture.cache(count=2)
        head = fixture.module.make_head(fixture.original_head(), "candidate_reset", 123).to(dtype)
        context, xy, speed = (t.to(dtype) for t in (data.fused, data.candidate_xy, data.candidate_speed))
        geometry = torch.cat((xy.flatten(2) / 120., speed / fixture.module.PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)
        inputs = torch.cat((context[:, None].expand(-1, 6, -1), geometry), dim=2)
        inputs_permuted = torch.cat((context[:, None].expand(-1, 6, -1), torch.cat((
            xy[:, order].flatten(2) / 120., speed[:, order] / fixture.module.PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)), dim=2)
        actual = fixture.module.score_head(head, "candidate_reset", context, xy, speed)
        permuted = fixture.module.score_head(head, "candidate_reset", context, xy[:, order], speed[:, order])
        direct = head(inputs).squeeze(-1)
        one = torch.stack([torch.stack([head(candidate).squeeze(-1) for candidate in sample]) for sample in inputs])
        one_permuted = torch.stack([torch.stack([head(candidate).squeeze(-1) for candidate in sample]) for sample in inputs_permuted])
        first = head[0](inputs)
        first_permuted = head[0](inputs_permuted)
        corrupted = permuted.clone()
        corrupted[0, 0] += 1e-3
        rows.append({"dtype": str(dtype), "order": order,
            "normalized_inputs_exact_permutation": torch.equal(inputs_permuted, inputs[:, order]),
            "same_input_normalization": compare(actual, direct),
            "batched_permutation": compare(permuted, actual[:, order]),
            "first_linear_layer_permutation": compare(first_permuted, first[:, order]),
            "one_candidate_reference_permutation": compare(one_permuted, one[:, order]),
            "batched_versus_one_candidate_reference": compare(actual, one),
            "selected_candidate_original_index_unchanged": torch.equal(torch.tensor(order)[permuted.argmax(1)], actual.argmax(1)),
            "corrupted_logit_rejected": not torch.allclose(corrupted, actual[:, order], atol=torch.finfo(dtype).eps, rtol=4 * torch.finfo(dtype).eps)})
after = {p: digest(p) for p in paths}
assert before == after
print(json.dumps({"schema": "portable_e2e.selector_roundoff_independent_diagnostic.v1",
    "command": "source scripts/e2e/env.sh && PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 python3 artifacts/training/2026-09-08/final_code_validation_v1/selector_roundoff_diagnostic.py",
    "script_sha256": digest(__file__), "head_observed": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
    "source_sha256_before": before, "source_sha256_after": after, "source_unchanged_during_diagnostic": True,
    "torch_version": torch.__version__, "torch_num_threads": torch.get_num_threads(),
    "torch_num_interop_threads": torch.get_num_interop_threads(), "mkldnn_enabled": torch.backends.mkldnn.enabled,
    "records": rows,
    "conclusion": "Normalized inputs are exact permutations; batched shared Linear kernels can differ by floating-point roundoff, while evaluating each candidate separately is bitwise permutation-equivariant. No candidate-index input or cross-candidate computation exists in score_head. The precise BLAS dispatch or instruction cause was not established.",
    "scope": {"cpu_only": True, "model_production_source_modified": False, "test_source_modified_by_this_diagnostic": False,
        "training_run": False, "physical_or_runtime_gate_modified": False, "original_data_modified": False}}, indent=2, allow_nan=False))
