"""HH_261001 - Prove the single-copy builder preserves causal guards, exact values and cache isolation."""

import ast
import inspect
import textwrap

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.temporal_headroom_camera_research_v2_20261001 import HeadroomOwnedCameraTemporalSimulationResearchPolicyV2
from portable_e2e.temporal_owned_camera_history_v2_20261001 import own_observed_camera_bytes
from portable_e2e.temporal_simulation_research_v2_20261001 import TemporalSimulationResearchPolicyV2
from portable_e2e.temporal_single_copy_history_research_20261001 import _SingleCopyBuildMixin, SingleCopyOwnedHeadroomPolicyV2
from test_temporal_owned_camera_history_v2_20261001 import call, frame, policy, ready, one_thread


def test_builder_ast_diff_is_one_redundant_clone_only():
    before = ast.parse(textwrap.dedent(inspect.getsource(TemporalSimulationResearchPolicyV2.build_inputs)))
    after = ast.parse(textwrap.dedent(inspect.getsource(_SingleCopyBuildMixin.build_inputs)))
    before.body[0].body = [x for x in before.body[0].body if not isinstance(x, ast.Expr)]
    after.body[0].body = [x for x in after.body[0].body if not isinstance(x, ast.Expr)]
    class RemoveClone(ast.NodeTransformer):
        def visit_Assign(self, node):
            if isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Attribute) \
                    and node.value.func.attr == "clone" and isinstance(node.value.func.value, ast.Subscript):
                node.value = node.value.func.value
            return node
    before = RemoveClone().visit(before)
    assert ast.dump(before, include_attributes=False) == ast.dump(after, include_attributes=False)


def test_mro_preserves_owned_wrapper():
    mro = SingleCopyOwnedHeadroomPolicyV2.mro()
    assert mro.index(_SingleCopyBuildMixin) < mro.index(TemporalSimulationResearchPolicyV2)
    assert SingleCopyOwnedHeadroomPolicyV2.build_inputs.__module__.endswith("temporal_owned_camera_history_v2_20261001")


@pytest.mark.parametrize("owned", [False, True])
def test_all_inputs_window_hash_and_internal_cache_equal_original(owned):
    sources = [frame(i) for i in range(12)]
    if owned: sources = [own_observed_camera_bytes(f) for f in sources]
    before = policy(HeadroomOwnedCameraTemporalSimulationResearchPolicyV2)
    after = policy(SingleCopyOwnedHeadroomPolicyV2)
    for i in range(9, 12):
        first, fw = call(before, "build_inputs", sources[i], sources[i-9:i])
        second, sw = call(after, "build_inputs", sources[i], sources[i-9:i])
        assert fw == sw
        for name in first: assert torch.equal(first[name], second[name]), name
        assert before._last_owned_input_audit["normalized_input_tensor_sha256"] == after._last_owned_input_audit["normalized_input_tensor_sha256"]
        assert set(before._images) == set(after._images)
        for key in before._images: assert torch.equal(before._images[key], after._images[key])
        # HH_261001 - Returned tensor edits cannot mutate private normalized cache tensors.
        snapshot = {key: value.clone() for key, value in after._images.items()}
        second["images"].data.zero_()
        for key in snapshot: assert torch.equal(snapshot[key], after._images[key])


def test_actual_raw_decoder_selected_pi_controls_equal(monkeypatch):
    from portable_e2e import temporal_simulation_research_v2_20261001 as frozen
    monkeypatch.setattr(frozen.time, "perf_counter", lambda: 1.)
    sources = [own_observed_camera_bytes(frame(i)) for i in range(13)]
    before = policy(HeadroomOwnedCameraTemporalSimulationResearchPolicyV2)
    after = policy(SingleCopyOwnedHeadroomPolicyV2)
    ready(before, sources); ready(after, sources)
    for i in range(10, 13):
        first = call(before, "step", sources[i], sources[i-9:i])
        second = call(after, "step", sources[i], sources[i-9:i])
        assert first["learned_control_proposed"] and second["learned_control_proposed"]
        for name in ("raw_output", "decoded_output", "candidate_gates", "selected", "control", "longitudinal_pi_research"):
            assert first[name] == second[name], name


@pytest.mark.parametrize("change", ["future", "duplicate", "calibration", "source", "time"])
def test_original_guard_rejects_bad_history(change):
    from dataclasses import replace
    sources = [frame(i) for i in range(10)]
    if change == "future": sources[0] = frame(11)
    elif change == "duplicate": sources[1] = sources[0]
    elif change == "calibration": sources[0] = replace(sources[0], inputs=replace(sources[0].inputs, calibration=((0.,)*16,)*6))
    elif change == "source": sources[0] = replace(sources[0], source_sha256="b"*64)
    elif change == "time": sources[0] = replace(sources[0], pose_timestamp_ns=sources[0].timestamp_ns+1)
    p = policy(SingleCopyOwnedHeadroomPolicyV2)
    with pytest.raises(ContractError): call(p, "build_inputs", sources[-1], sources[:-1])
