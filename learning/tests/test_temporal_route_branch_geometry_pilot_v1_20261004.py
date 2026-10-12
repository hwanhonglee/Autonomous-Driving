"""HH_261004 - Synthetic CPU partition, parity and prospective-gate checks; not driving evidence."""
from types import SimpleNamespace

import pytest
import torch

from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from scripts.e2e import train_temporal_route_branch_geometry_pilot_v1_20261004 as pilot


def test_exact_existing_four_tensor_partition_and_synthetic_optimizer_update():
    model = RouteRecoveryTemporalModelV3().train()
    names = pilot.freeze_branch(model)
    assert names == sorted(pilot.BRANCH_SHAPES)
    assert sum(p.numel() for p in model.parameters() if p.requires_grad) == 1409
    before = pilot.frozen_digest(model)
    optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=1e-4)
    # HH_261004 - This single artificial scalar update touches random initial weights, never actual GOOD6000 or recorded TRAIN.
    loss = sum(p.square().sum() for p in model.parameters() if p.requires_grad)
    loss.backward(); optimizer.step()
    pilot.check_partition(model, before)
    assert all(p.grad is None for name,p in model.named_parameters() if name not in pilot.BRANCH_SHAPES)
    with torch.no_grad(): model.speed_head.bias.add_(1)
    with pytest.raises(ValueError, match="97 frozen"): pilot.check_partition(model, before)


def test_requires_grad_scope_change_rejected():
    model = RouteRecoveryTemporalModelV3(); pilot.freeze_branch(model); before = pilot.frozen_digest(model)
    model.speed_head.weight.requires_grad_(True)
    with pytest.raises(ValueError, match="scope changed"): pilot.check_partition(model, before)


def synthetic_panels():
    result = {}
    for domain in range(5):
        rows = []
        for index in range(32):
            metrics = dict(true_raw_curvature_huber=1.)
            if domain != 2: metrics.update(true_ADE_m=1., true_FDE_m=1.)
            candidate = dict(metrics)
            if domain >= 3: candidate["true_ADE_m"] = .8
            parity = dict(raw_speed_byte_equal=True, instantaneous_speed_byte_equal=None if domain == 2 else True,
                arc_byte_equal=None if domain == 2 else True, decoded_parity_reason="LEGACY_RAW_ONLY_NO_DECODER" if domain == 2 else "PAIRED_GO_ACTUAL_INPUTS")
            rows.append(dict(sample_id=f"synthetic:{domain}:{index}", split="TRAIN", parent=metrics, candidate=candidate, parity=parity))
        result[str(domain)] = dict(denominator=32, rows=rows)
    return result


def test_full160_engineering_gates_explicitly_not_feature_approval():
    result = pilot.gates(synthetic_panels())
    assert result["status"] == "TRAIN_ENGINEERING_GATES_MET_NOT_FEATURE_APPROVAL"
    assert len(result["checks"]) == 18 and all(row["denominator"] == 32 for row in result["checks"])
    assert (result["raw_speed_parity_count"], result["decoded_speed_arc_parity_count"], result["legacy_decoded_unavailable_count"]) == (160,128,32)
    assert result["model_promoted"] is False and result["formal_sub_approved"] == 0


def test_peak_failure_cannot_hide_in_good_mean():
    values = synthetic_panels(); values["1"]["rows"][0]["candidate"]["true_ADE_m"] = 1.1
    result = pilot.gates(values)
    mean = next(r for r in result["checks"] if r["domain"]==1 and r["metric"]=="true_ADE_m" and r["reduction"]=="mean")
    peak = next(r for r in result["checks"] if r["domain"]==1 and r["metric"]=="true_ADE_m" and r["reduction"]=="peak")
    assert mean["passed"] is True and peak["passed"] is False
    assert result["status"] == "TRAIN_ENGINEERING_GATES_NOT_MET_NOT_PROMOTED"


def test_zero_baseline_has_no_invented_slack():
    values = synthetic_panels()
    for row in values["2"]["rows"]: row["parent"]["true_raw_curvature_huber"] = 0.; row["candidate"]["true_raw_curvature_huber"] = 0.
    values["2"]["rows"][0]["candidate"]["true_raw_curvature_huber"] = 1e-15
    result = pilot.gates(values)
    assert any(r["domain"]==2 and r["maximum"]==0 and r["passed"] is False for r in result["checks"])


@pytest.mark.parametrize("change", ["short", "duplicate", "wrong_split", "fake_legacy", "failed_raw_parity"])
def test_full_declared_denominator_and_missing_decoding_are_required(change):
    values = synthetic_panels()
    if change == "short": values["1"]["rows"].pop()
    if change == "duplicate": values["1"]["rows"][1]["sample_id"] = values["1"]["rows"][0]["sample_id"]
    if change == "wrong_split": values["1"]["rows"][0]["split"] = "TEST"
    if change == "fake_legacy": values["2"]["rows"][0]["parity"]["instantaneous_speed_byte_equal"] = True
    if change == "failed_raw_parity": values["1"]["rows"][0]["parity"]["raw_speed_byte_equal"] = False
    with pytest.raises(ValueError): pilot.gates(values)


def test_byte_parity_and_legacy_unavailable_not_equal_to_success():
    raw = dict(preferred_speed_knots=torch.ones(1,33))
    decoded = dict(instantaneous_speed=torch.ones(1,65), arc_distance=torch.ones(1,65))
    assert pilot.parity(raw, raw, decoded, decoded)["instantaneous_speed_byte_equal"] is True
    assert pilot.parity(raw, raw, None, None)["instantaneous_speed_byte_equal"] is None
    changed = dict(preferred_speed_knots=raw["preferred_speed_knots"] + .01)
    with pytest.raises(ValueError, match="raw preferred-speed"): pilot.parity(changed, raw, decoded, decoded)
    changed_decoder = dict(decoded, arc_distance=decoded["arc_distance"] + .01)
    with pytest.raises(ValueError, match="instantaneous-speed/arc"): pilot.parity(raw, raw, changed_decoder, decoded)


def test_contract_preserves35_sources_and_prospective_fixed_budget():
    value = pilot.contract()
    assert len(value["sources"]) == 36
    assert all(value["sources"].get(k) == v for k,v in pilot.audit.source_snapshot().items())
    assert (value["optimizer_updates"],value["exposures"],value["trainable_tensor_count"],value["frozen_tensor_count"]) == (1000,5000,4,97)
    assert value["legacy_decoded_not_applicable_count"] == 32 and value["checkpoint_selection"] is False
    assert value["RIGHT_raw_speed_error_not_addressed"] is True


def test_private_actual_training_guard_rejects_synthetic_foreign_environment(monkeypatch):
    monkeypatch.setattr(pilot.sys,"prefix","/synthetic/nonprivate/venv")
    monkeypatch.setattr(pilot.pair,"load_train",lambda *_: pytest.fail("actual TRAIN must not open in synthetic test"))
    with pytest.raises(ValueError, match="private source"): pilot.prepare(SimpleNamespace(), pilot.contract())


def test_current_low_speed_decoder_curvature_cap_cannot_change_GO_timing():
    decoder = pilot.frozen.original.DeterministicPathTimeDecoderV2()
    preferred = torch.full((1,33),2.)
    args = (preferred,torch.tensor([2.]),torch.tensor([0]),torch.tensor([0.]),torch.tensor([64.]))
    straight = decoder(torch.zeros(1,33), *args)
    curve = decoder(torch.full((1,33),.199), *args)
    assert torch.equal(straight["instantaneous_speed"],curve["instantaneous_speed"])
    assert torch.equal(straight["arc_distance"],curve["arc_distance"])
    assert not torch.equal(straight["xy"],curve["xy"])


def test_fresh_authority_cannot_reuse_gradient_schema(monkeypatch):
    value = pilot.contract()
    source = dict(schema=pilot.audit.SCHEMA + ".authorization", authorized=True, contract=value,
        physical_GPU_index=0,model_promoted=False,vehicle_control_approved=False)
    monkeypatch.setattr(pilot.frozen,"pinned_read",lambda *_:source)
    with pytest.raises(ValueError,match="fresh exact"): pilot.read_authority("unused","a"*64,value)
