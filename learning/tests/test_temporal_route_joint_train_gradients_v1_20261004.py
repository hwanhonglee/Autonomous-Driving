"""HH_261004 - Synthetic CPU checks for the frozen TRAIN gradient audit, not driving evidence."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from scripts.e2e import audit_temporal_route_joint_train_gradients_v1_20261004 as audit


def vectors(a, b):
    return {"a": None if a is None else torch.tensor(a, dtype=torch.float64),
            "b": None if b is None else torch.tensor(b, dtype=torch.float64)}


@pytest.mark.parametrize("left,right,expected", [
    (vectors([1, 2], None), vectors([1, 2], None), 1),
    (vectors([1, 2], None), vectors([-1, -2], None), -1),
    (vectors([1, 0], None), vectors([0, 1], None), 0),
    (vectors([1], None), vectors(None, [1]), 0)])
def test_real_cosines_including_orthogonal_missing_components(left, right, expected):
    result = audit.cosine_record(left, right, ("a", "b"))
    assert result["cosine"] == pytest.approx(expected)
    assert result["reason"] == "DEFINED"
    assert result["left"]["absent_tensor_count"] == 1


@pytest.mark.parametrize("left,right,reason", [
    (vectors(None, None), vectors([1], [2]), "ABSENT_GRADIENT"),
    (vectors([0], None), vectors([1], [2]), "ZERO_GRADIENT"),
    (vectors([0], [0]), vectors([0], [0]), "ZERO_GRADIENT")])
def test_absent_or_zero_cannot_be_fabricated_as_cosine_zero(left, right, reason):
    result = audit.cosine_record(left, right, ("a", "b"))
    assert result["cosine"] is None and result["reason"] == reason


def test_nonfinite_gradients_rejected_and_large_finite_norm_supported():
    with pytest.raises(ValueError, match="finite float64"):
        audit.norm_record(vectors([float("nan")], None), ("a", "b"))
    result = audit.norm_record(vectors([1e30, -1e30], None), ("a", "b"))
    assert result["norm"] == pytest.approx(2 ** .5 * 1e30)


def test_autograd_reports_unused_parameter_without_accumulation():
    active = torch.nn.Parameter(torch.tensor([2., -1.]))
    unused = torch.nn.Parameter(torch.tensor([3.]))
    loss = active.square().sum()
    values = audit.gradients(loss, ("a", "b"), (active, unused))
    assert torch.equal(values["a"], torch.tensor([4., -2.], dtype=torch.float64))
    assert values["b"] is None and active.grad is unused.grad is None
    assert audit.norm_record(values, ("b",))["status"] == "ABSENT"


def test_full_current101_model_group_partition_and_state_mutation_guard():
    # HH_261004 - Randomly initialized CPU architecture validates the ABI only, never a GOOD6000 performance claim.
    model = RouteRecoveryTemporalModelV3().eval()
    audit.frozen.original.freeze_inputs(model)
    names, groups = audit.model_groups(model)
    assert len(names) == 54 and len(model.state_dict()) == 101
    assert set(groups) == {*audit.GROUP_PREFIXES, "all54"}
    assert sum(len(value) for key, value in groups.items() if key != "all54") == 54
    teacher = audit.previous.freeze_teacher(model, torch.device("cpu"))
    before = audit.state_digest(model)
    audit.check_models(model, teacher, before)
    with torch.no_grad():
        dict(model.named_parameters())[names[0]].add_(1)
    with pytest.raises(ValueError, match="state bytes changed"):
        audit.check_models(model, teacher, before)


def test_trainable_group_mismatch_rejected():
    model = RouteRecoveryTemporalModelV3().eval()
    audit.frozen.original.freeze_inputs(model)
    model.speed_head.weight.requires_grad_(False)
    with pytest.raises(ValueError, match="model ABI"):
        audit.model_groups(model)


def test_protection_of_identical_parent_has_present_exact_zero_gradient():
    curvature = torch.nn.Parameter(torch.zeros(1, 33))
    speed = torch.nn.Parameter(torch.ones(1, 33))
    raw = dict(curvature_knots=curvature, preferred_speed_knots=speed)
    mask = torch.ones(1, 33, dtype=torch.bool)
    proof = audit.previous.retention.ReplayProvenance("synthetic", "TRAIN", "OLD_LOW_SPEED_REPLAY", "a"*64, "b"*64, "c"*64, True, False)
    result = audit.previous.retention.trajectory_retention_loss(raw, raw, observed_spatial_valid=mask,
        curvature_valid=mask, preferred_speed_valid=mask, provenance=(proof,), sample_ids=("synthetic",), parent_checkpoint_sha256="d"*64)
    for term in ("raw_curvature", "raw_preferred_speed", "total"):
        grads = audit.gradients(result[term], ("a", "b"), (curvature, speed))
        assert audit.norm_record(grads, ("a", "b"))["status"] == "EXACT_ZERO"
        assert audit.cosine_record(grads, grads, ("a", "b"))["cosine"] is None


def test_original_panel_membership_and_legacy_eligibility_cannot_change():
    datasets = [SimpleNamespace(examples=[SimpleNamespace(token=f"d{d}:{i}") for i in range(65)]) for d in range(5)]
    # HH_261004 - The real production selection uses the original eligible legacy pool, never model-based top samples.
    panels = [audit.previous.uniform_panel(65, range(33) if d == 2 else None) for d in range(5)]
    sealed = {str(d): [dataset.examples[j].token for j in panels[d]] for d, dataset in enumerate(datasets)}
    assert audit.verify_panel_ids(datasets, panels, sealed) == sealed
    changed = {key: list(value) for key, value in sealed.items()}; changed["2"][0] = "not-TRAIN"
    with pytest.raises(ValueError, match="panel IDs"):
        audit.verify_panel_ids(datasets, panels, changed)


def test_domain_mean_retains_full32_denominator_and_explicit_missing():
    store = {}
    audit.add_mean(store, 0, "curve", vectors([32], None))
    result = audit.final_means(store, ("a", "b"))[(0, "curve")]
    assert result["a"].item() == 1 and result["b"] is None


def test_source_contract_preserves34_and_has_fixed_real_operation_budget():
    value = audit.contract()
    assert len(value["sources"]) == 35
    assert all(value["sources"][key] == sha for key, sha in audit.pilot.source_snapshot().items())
    assert value["operation_budget"]["total_autograd_grad_calls"] == 1152
    assert sum(value["operation_budget"]["gradient_calls_per_domain"]) == 1152
    assert value["operation_budget"]["optimizer_updates"] == 0
    assert value["CPU_actual_audit_authorized"] is False
    assert value["evaluation_only_payloads_opened"] is False
    assert value["retention_limitation"] == "zero initial gradient does not establish preservation after training"


def test_parser_has_no_fit_dispatch():
    with pytest.raises(SystemExit): audit.parser().parse_args(["--mode", "fit"])
    assert audit.parser().parse_args(["--mode", "run"]).mode == "run"


def test_actual_audit_refuses_local_environment_before_loading(monkeypatch):
    monkeypatch.setattr(audit.pair, "load_train", lambda _: pytest.fail("real TRAIN loader must not run locally"))
    with pytest.raises(ValueError, match="personal venv"):
        audit.prepare(SimpleNamespace(), audit.contract())


def test_fresh_authority_contract_or_GPU_mismatch_rejected(monkeypatch):
    value = audit.contract()
    source = dict(schema=audit.SCHEMA + ".authorization", authorized=True, contract=value,
        physical_GPU_index=0, model_promoted=False, vehicle_control_approved=False)
    monkeypatch.setattr(audit.frozen, "pinned_read", lambda *_: source)
    assert audit.read_authority(Path("unused"), "a"*64, value) == source
    source["physical_GPU_index"] = 1
    with pytest.raises(ValueError, match="authority"):
        audit.read_authority(Path("unused"), "a"*64, value)


def test_cross_domain_missing_task_remains_not_applicable():
    groups = dict(all54=("a", "b"))
    rows = {0: dict(probe_id="train0", batch_sha256="a"*64, gradients={"raw_curvature": vectors([1], None)}),
        2: dict(probe_id="train2", batch_sha256="b"*64, gradients={"raw_curvature": vectors([-1], None)})}
    results = audit.cross_domain(rows, groups)
    defined = next(row for row in results if row["term"] == "raw_curvature")
    assert defined["alignment"]["groups"]["all54"]["cosine"] == -1
    missing = next(row for row in results if row["term"] == "temporal_xy")
    assert missing["cosine"] is None and missing["reason"] == "TERM_NOT_APPLICABLE"


@pytest.mark.parametrize("before", [False, True])
def test_attention_diagnostic_flag_is_restored_even_after_failure(monkeypatch, before):
    # HH_261004 - Older local Torch has no MHA flag API; mock only this backend switch, never install a new package.
    state = {"enabled": before}
    backend = SimpleNamespace(get_fastpath_enabled=lambda: state["enabled"],
        set_fastpath_enabled=lambda value: state.update(enabled=value))
    monkeypatch.setattr(torch.backends, "mha", backend, raising=False)
    with pytest.raises(RuntimeError, match="synthetic"):
        with audit.matched_attention_math() as prior:
            assert prior == before and torch.backends.mha.get_fastpath_enabled() is False
            raise RuntimeError("synthetic")
    assert torch.backends.mha.get_fastpath_enabled() == before


def test_unsupported_attention_backend_fails_closed(monkeypatch):
    monkeypatch.delattr(torch.backends, "mha", raising=False)
    with pytest.raises(ValueError, match="explicit MHA"):
        audit.attention_backend()


def test_existing_actual_attention_backend_toggle_and_restore():
    # HH_261004 - Local Torch1.8 lacks this API; only the real backend-specific check skips, and must run on private modern Torch before dispatch.
    if not hasattr(torch.backends, "mha"):
        pytest.skip("existing local Torch has no MHA fastpath API; synthetic interface tests are not private GPU evidence")
    before = torch.backends.mha.get_fastpath_enabled()
    with audit.matched_attention_math() as prior:
        assert prior == before and torch.backends.mha.get_fastpath_enabled() is False
    assert torch.backends.mha.get_fastpath_enabled() == before


def test_all32_model_dataset_authority_pins_rechecked_including_mutation(tmp_path):
    # HH_261004 - Exercise the real immutable verifier on synthetic files; no actual data or installed environment is changed.
    paths = []
    for index in range(32):
        path = tmp_path / f"synthetic-{index}.json"; path.write_text(str(index)); paths.append(path)
    pins = {str(path): audit.digest(path) for path in paths}
    audit.pair.verify_inputs(pins, {}, [])
    paths[29].write_text("changed original admission identity")
    with pytest.raises(ValueError, match="immutable input changed"):
        audit.pair.verify_inputs(pins, {}, [])


@pytest.mark.parametrize("domain,expected_count", [(0, 9), (1, 9), (2, 6), (3, 6), (4, 6)])
def test_exact_native_terms_and_protection_binding_without_acceleration_placeholder(monkeypatch, domain, expected_count):
    # HH_261004 - This tiny synthetic graph exercises orchestration, not captured data, real model weights or vehicle control.
    class Raw(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.curve = torch.nn.Parameter(torch.full((1, 33), .01))
            self.speed = torch.nn.Parameter(torch.ones(1, 33))

        def forward(self):
            return dict(curvature_knots=self.curve, preferred_speed_knots=self.speed,
                intent_logits=self.curve[:, :3], stop_distance_raw=self.speed[:, :1], acceleration_prediction=self.curve[:, :1])

    student = Raw().eval(); teacher = Raw().eval().requires_grad_(False)
    mask = torch.ones(1, 33, dtype=torch.bool)
    batch = dict(sample_id=["synthetic"], spatial_curvature=torch.zeros(1, 33), spatial_preferred_speed=torch.zeros(1, 33),
        spatial_valid=mask, spatial_curvature_valid=mask, spatial_speed_valid=mask, intent_valid=torch.tensor([False]),
        intent_target=torch.tensor([0]), stop_distance_valid=torch.tensor([False]), stop_distance_target=torch.tensor([0.]))
    assert "initial_acceleration_valid" not in batch
    monkeypatch.setattr(audit.raw_parent, "move", lambda data, _: data)
    monkeypatch.setattr(audit.raw_parent, "forward", lambda model, _: model())

    def synthetic_joint(model, _, data):
        raw = model()
        losses = audit.frozen.original.temporal_trajectory_loss_v2(raw, data)
        losses["preview_speed_regularizer"] = raw["preferred_speed_knots"].sum() * .01
        losses["total"] = losses["total"] + .1 * losses["preview_speed_regularizer"]
        return raw, {"synthetic_decoded": raw["curvature_knots"]}, losses

    monkeypatch.setattr(audit.frozen.joint, "joint_forward", synthetic_joint)
    proof = audit.previous.retention.ReplayProvenance("synthetic", "TRAIN", audit.DOMAINS[min(domain, 2)], "a"*64, "b"*64, "c"*64, True, False)
    _, _, _, decoded, terms, absent = audit.applicable_terms(student, teacher, None, batch, domain, proof, torch.device("cpu"))
    assert len(terms) == expected_count and set(terms) | set(absent) == set(audit.TERMS)
    assert (decoded is None) == (domain == 2)
    if domain < 3:
        assert all(float(terms[key].detach()) == 0 for key in audit.PROTECTION)
    else:
        assert absent["retention_total"] == "NOT_AN_OLD_REPLAY_PROTECTION_ROLE"
