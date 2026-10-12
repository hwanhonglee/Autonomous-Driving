"""HH_261004 - CPU synthetic-only retention contracts; never read real datasets/checkpoints or invoke a simulator."""
from dataclasses import replace
import hashlib

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.trajectory_retention_loss_20261004 import (
    OLD_REPLAY_ROLES, ReplayProvenance, trajectory_retention_loss,
)

PIN = hashlib.sha256(b"synthetic provenance only").hexdigest()


def batch(size=2, dtype=torch.float64):
    student = dict(curvature_knots=torch.full((size, 33), .01, dtype=dtype, requires_grad=True),
                   preferred_speed_knots=torch.full((size, 33), 2., dtype=dtype, requires_grad=True))
    parent = dict(curvature_knots=torch.zeros((size, 33), dtype=dtype, requires_grad=True),
                  preferred_speed_knots=torch.ones((size, 33), dtype=dtype, requires_grad=True))
    ids = tuple("synthetic-%d" % i for i in range(size))
    roles = sorted(OLD_REPLAY_ROLES)
    provenance = tuple(ReplayProvenance(name, "TRAIN", roles[i % len(roles)], PIN, PIN, PIN, True, False)
                       for i, name in enumerate(ids))
    valid = torch.ones((size, 33), dtype=torch.bool)
    options = dict(observed_spatial_valid=valid.clone(), curvature_valid=valid.clone(),
                   preferred_speed_valid=valid.clone(), provenance=provenance, sample_ids=ids,
                   parent_checkpoint_sha256=PIN)
    return student, parent, options


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_same_raw_prediction_is_exact_zero_and_parent_gets_no_gradient(dtype):
    student, parent, options = batch(dtype=dtype)
    parent = {name: value.detach().clone().requires_grad_() for name, value in student.items()}
    result = trajectory_retention_loss(student, parent, **options)
    assert result["total"].item() == result["raw_curvature"].item() == result["raw_preferred_speed"].item() == 0
    result["total"].backward()
    assert all(torch.equal(value.grad, torch.zeros_like(value)) for value in student.values())
    assert all(value.grad is None for value in parent.values())


def test_raw_huber_scaling_and_student_only_gradient():
    student, parent, options = batch()
    result = trajectory_retention_loss(student, parent, **options)
    assert result["raw_curvature"].item() == pytest.approx(.005)
    assert result["raw_preferred_speed"].item() == pytest.approx(.5)
    assert result["total"].item() == pytest.approx(.50125)
    result["total"].backward()
    assert all(value.grad is not None and bool(torch.isfinite(value.grad).all()) and value.grad.abs().sum() > 0
               for value in student.values())
    assert all(value.grad is None for value in parent.values())


def test_masks_exclude_unobserved_knots_and_do_not_change_original_tensors():
    student, parent, options = batch()
    for name in ("observed_spatial_valid", "curvature_valid", "preferred_speed_valid"):
        options[name][:, 2:] = False
    options["curvature_valid"][:, 0] = False
    before = {name: value.clone() for name, value in options.items() if isinstance(value, torch.Tensor)}
    result = trajectory_retention_loss(student, parent, **options)
    result["total"].backward()
    assert student["curvature_knots"].grad[:, 0].abs().sum() == 0
    assert student["curvature_knots"].grad[:, 1].abs().sum() > 0
    assert all(value.grad[:, 2:].abs().sum() == 0 for value in student.values())
    assert result["curvature_valid_count"].tolist() == [1, 1]
    assert result["preferred_speed_valid_count"].tolist() == [2, 2]
    assert all(torch.equal(options[name], value) for name, value in before.items())


def test_each_anchor_is_weighted_equally_not_by_its_knot_count():
    student, parent, options = batch()
    options["curvature_valid"][:] = False
    options["preferred_speed_valid"][0, 1:] = False
    student["preferred_speed_knots"] = torch.cat((torch.full((1, 33), 2.), torch.full((1, 33), 3.))).double().requires_grad_()
    result = trajectory_retention_loss(student, parent, **options)
    assert result["preferred_speed_per_anchor"].tolist() == [.5, 1.5]
    assert result["total"].item() == 1.


def test_empty_masks_are_graph_connected_zero_and_not_approval():
    student, parent, options = batch()
    for name in ("observed_spatial_valid", "curvature_valid", "preferred_speed_valid"):
        options[name][:] = False
    result = trajectory_retention_loss(student, parent, **options)
    assert result["total"].item() == 0
    result["total"].backward()
    assert all(torch.equal(value.grad, torch.zeros_like(value)) for value in student.values())
    for name in ("dataset_admitted", "expert_labels_created", "runtime_changed", "model_promoted", "feature_approved", "decoded_path_retention"):
        assert result[name] is False


def test_stationary_curvature_stays_invalid_while_true_speed_zero_can_be_retained():
    student, parent, options = batch()
    options["observed_spatial_valid"][:, 1:] = False
    options["curvature_valid"][:] = False
    options["preferred_speed_valid"][:, 1:] = False
    parent["preferred_speed_knots"] = torch.zeros_like(parent["preferred_speed_knots"], requires_grad=True)
    result = trajectory_retention_loss(student, parent, **options)
    result["total"].backward()
    assert result["raw_curvature"].item() == 0 and result["preferred_speed_valid_count"].tolist() == [1, 1]
    assert student["curvature_knots"].grad.abs().sum() == 0
    assert student["preferred_speed_knots"].grad[:, 0].abs().sum() > 0
    assert student["preferred_speed_knots"].grad[:, 1:].abs().sum() == 0


@pytest.mark.parametrize("role", sorted(OLD_REPLAY_ROLES))
def test_all_three_old_roles_accept_high_speed_raw_without_retiming(role):
    student, parent, options = batch()
    options["provenance"] = tuple(replace(row, role=role) for row in options["provenance"])
    student["preferred_speed_knots"] = torch.full((2, 33), 15., dtype=torch.float64, requires_grad=True)
    assert trajectory_retention_loss(student, parent, **options)["total"].item() > 0


@pytest.mark.parametrize("split", ["DEV", "TEST", "HOLDOUT", "train", "TRAIN ", "", True, None])
def test_every_non_train_provenance_is_rejected(split):
    student, parent, options = batch()
    options["provenance"] = tuple(replace(row, split=split) for row in options["provenance"])
    with pytest.raises(ContractError, match="only TRAIN"):
        trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("role", ["LEFT", "RIGHT", "NEW_BEHAVIOR", "OLD", True, None])
def test_new_behavior_and_ambiguous_roles_cannot_enter_retention(role):
    student, parent, options = batch()
    options["provenance"] = tuple(replace(row, role=role) for row in options["provenance"])
    with pytest.raises(ContractError, match="OLD replay roles"):
        trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("field,value", [("qualified", False), ("qualified", 1), ("training_denied", True), ("training_denied", 0)])
def test_unqualified_denied_or_non_boolean_admission_flags_fail(field, value):
    student, parent, options = batch()
    options["provenance"] = tuple(replace(row, **{field: value}) for row in options["provenance"])
    with pytest.raises(ContractError, match="unqualified"):
        trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("field", ["dataset_manifest_sha256", "admission_authority_sha256", "qualification_sha256", "parent_checkpoint_sha256"])
@pytest.mark.parametrize("pin", ["", "x" * 64, "A" * 64, True])
def test_explicit_sha_bindings_required_even_when_masks_empty(field, pin):
    student, parent, options = batch()
    options["curvature_valid"][:] = options["preferred_speed_valid"][:] = False
    if field == "parent_checkpoint_sha256": options[field] = pin
    else: options["provenance"] = tuple(replace(row, **{field: pin}) for row in options["provenance"])
    with pytest.raises(ContractError, match="SHA256"):
        trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("fault", ["missing", "dict", "different_id", "blank_id", "string_rows", "string_ids"])
def test_per_anchor_provenance_and_actual_sample_alignment_are_strict(fault):
    student, parent, options = batch()
    if fault == "missing": options["provenance"] = options["provenance"][:1]
    elif fault == "dict": options["provenance"] = [vars(row) for row in options["provenance"]]
    elif fault == "different_id": options["sample_ids"] = tuple(reversed(options["sample_ids"]))
    elif fault == "blank_id": options["sample_ids"] = (" ", options["sample_ids"][1])
    elif fault == "string_rows": options["provenance"] = "not provenance"
    else: options["sample_ids"] = "not sample IDs"
    with pytest.raises(ContractError): trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("name", ["observed_spatial_valid", "curvature_valid", "preferred_speed_valid"])
@pytest.mark.parametrize("fault", ["numeric", "shape", "device", "not_tensor"])
def test_masks_are_exact_boolean_same_device_shapes(name, fault):
    student, parent, options = batch()
    if fault == "numeric": options[name] = options[name].float()
    elif fault == "shape": options[name] = options[name][:, :32]
    elif fault == "device":
        # HH_261004 - Older local Torch lacks meta tensors; override only the device label of a CPU-owned synthetic tensor.
        class WrongDeviceTensor(torch.Tensor):
            @property
            def device(self): return torch.device("cuda:0")
        options[name] = torch.Tensor._make_subclass(WrongDeviceTensor, options[name].clone(), False)
    else: options[name] = [[True] * 33] * 2
    with pytest.raises(ContractError, match="boolean"):
        trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("fault", ["hole", "curvature_outside", "speed_outside"])
def test_observed_support_is_never_extended_or_bridged(fault):
    student, parent, options = batch()
    options["observed_spatial_valid"][:, 2:] = False
    options["curvature_valid"][:, 2:] = options["preferred_speed_valid"][:, 2:] = False
    if fault == "hole": options["observed_spatial_valid"][:, 3] = True
    elif fault == "curvature_outside": options["curvature_valid"][:, 2] = True
    else: options["preferred_speed_valid"][:, 2] = True
    with pytest.raises(ContractError): trajectory_retention_loss(student, parent, **options)


@pytest.mark.parametrize("side", ["student", "parent"])
@pytest.mark.parametrize("fault", ["missing", "not_mapping", "shape", "empty", "integer", "half", "speed_dtype", "nan_masked", "inf", "curvature_bound", "negative_speed"])
def test_raw_abi_and_all_finite_values_are_strict(side, fault):
    student, parent, options = batch()
    value = student if side == "student" else parent
    if fault == "missing": value.pop("curvature_knots")
    elif fault == "not_mapping":
        if side == "student": student = []
        else: parent = []
    elif fault == "shape": value["curvature_knots"] = torch.zeros((2, 32), dtype=torch.float64)
    elif fault == "empty": value["curvature_knots"] = torch.zeros((0, 33), dtype=torch.float64)
    elif fault == "integer": value["curvature_knots"] = value["curvature_knots"].long()
    elif fault == "half": value["curvature_knots"] = value["curvature_knots"].half()
    elif fault == "speed_dtype": value["preferred_speed_knots"] = value["preferred_speed_knots"].float()
    else:
        key = "preferred_speed_knots" if fault == "negative_speed" else "curvature_knots"
        changed = value[key].detach().clone()
        changed[0, 32] = {"nan_masked": float("nan"), "inf": float("inf"), "curvature_bound": .201, "negative_speed": -.01}[fault]
        value[key] = changed
        options["curvature_valid"][:, 32] = options["preferred_speed_valid"][:, 32] = False
    with pytest.raises(ContractError): trajectory_retention_loss(student, parent, **options)


def test_parent_student_dtype_shape_mismatch_rejected():
    student, parent, options = batch()
    parent = {name: value.float() for name, value in parent.items()}
    with pytest.raises(ContractError, match="parent/student"):
        trajectory_retention_loss(student, parent, **options)


def test_provenance_ID_cannot_use_a_non_string_equality_override():
    # HH_261004 - Strict identity strings prevent arbitrary objects from spoofing an aligned sample ID.
    class EqualToAnything:
        def __eq__(self, other): return True
    student, parent, options = batch()
    options["provenance"] = tuple(replace(row, sample_id=EqualToAnything()) for row in options["provenance"])
    with pytest.raises(ContractError, match="aligned sample ID"):
        trajectory_retention_loss(student, parent, **options)


def test_shared_neural_parameters_receive_only_student_branch_gradient():
    # HH_261004 - Even accidentally graph-bearing parent outputs must not update the teacher or alter student masks.
    student_parameter = torch.nn.Parameter(torch.tensor(.2, dtype=torch.float64))
    parent_parameter = torch.nn.Parameter(torch.tensor(.1, dtype=torch.float64))
    student, parent, options = batch()
    def outputs(parameter):
        return dict(curvature_knots=(.2 * parameter.tanh()).expand(2, 33),
                    preferred_speed_knots=torch.nn.functional.softplus(parameter).expand(2, 33))
    result = trajectory_retention_loss(outputs(student_parameter), outputs(parent_parameter), **options)
    result["total"].backward()
    assert student_parameter.grad is not None and student_parameter.grad.abs() > 0
    assert parent_parameter.grad is None
