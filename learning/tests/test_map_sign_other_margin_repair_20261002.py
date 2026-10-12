"""HH_261002 - Synthetic conditional probability, exact zero nonpair gradients, and unchanged source/control laws only."""
import ast
import copy
from pathlib import Path
from unittest import mock

import pytest
import torch

from portable_e2e import map_sign_other_margin_loss_v1_20261002 as loss
from scripts.e2e import train_map_sign_other_margin_repair_20261002 as runner

GROUP = loss.SOURCE_GROUPS[0]


@pytest.mark.parametrize("label,expected", [(5,.5), (0,.5), (1,0.), (2,0.), (3,0.), (4,0.), (6,0.)])
def test_zero_logits_actual_pair_denominators(label, expected):
    x = torch.zeros(1,7,2,2,requires_grad=True); y = torch.full((1,2,2),label,dtype=torch.int64)
    result, stats = loss.sign_margin_auxiliary(x,y,[GROUP])
    assert result.item() == pytest.approx(expected*torch.log(torch.tensor(2.)).item(),abs=1.e-7)
    result.backward()
    assert x.grad is not None and torch.isfinite(x.grad).all()
    assert torch.equal(x.grad[:,[1,2,3,4,6]],torch.zeros_like(x.grad[:,[1,2,3,4,6]]))
    assert stats["images"][0]["valid_other_pixels"] == (4 if label==0 else 0)
    assert stats["groups"][GROUP]["negative_images"]==1


def test_conditional_pair_not_sevenclass_probability_and_nonpair_logit_invariance():
    x = torch.zeros(1,7,1,2,requires_grad=True); y = torch.tensor([[[5,0]]])
    first,_ = loss.sign_margin_auxiliary(x,y,[GROUP])
    changed=x.detach().clone(); changed[:,[1,2,3,4,6]]=100.; changed.requires_grad_(True)
    other,_ = loss.sign_margin_auxiliary(changed,y,[GROUP])
    assert torch.equal(first,other) and first.item()==pytest.approx(torch.log(torch.tensor(2.)).item())
    other.backward(); assert torch.equal(changed.grad[:,[1,2,3,4,6]],torch.zeros_like(changed.grad[:,[1,2,3,4,6]]))
    assert changed.grad[0,5,0,0]<0 and changed.grad[0,0,0,0]>0
    assert changed.grad[0,5,0,1]>0 and changed.grad[0,0,0,1]<0


def test_all_images_retained_for_negative_mean_and_sources_equal():
    x=torch.zeros(3,7,1,1,requires_grad=True);y=torch.tensor([[[5]],[[0]],[[3]]])
    a,stats=loss.sign_margin_auxiliary(x,y,[GROUP,GROUP,GROUP])
    ln2=torch.log(torch.tensor(2.)).item()
    assert a.item()==pytest.approx(.5*ln2+.5*ln2/3)
    assert stats["groups"][GROUP]["images"]==stats["groups"][GROUP]["negative_images"]==3
    b,_=loss.sign_margin_auxiliary(x,y,[GROUP,GROUP,loss.SOURCE_GROUPS[1]])
    assert b.item()==pytest.approx((.5*ln2+.25*ln2)/2)


def test_fractional_ties_exact64_mass_and_permutation_no_nonpair_gradient():
    x=torch.zeros(1,7,1,80,requires_grad=True);y=torch.zeros(1,1,80,dtype=torch.int64)
    value,stats=loss.sign_margin_auxiliary(x,y,[GROUP]);value.backward()
    row=stats["images"][0]
    assert row["effective_k"]==64 and row["threshold_tie_pixels"]==80 and row["fractional_tie_weight"]==.8
    assert x.grad[0,5].sum().item()==pytest.approx(.25) and x.grad[0,0].sum().item()==pytest.approx(-.25)
    assert torch.allclose(x.grad[0,5],torch.full_like(x.grad[0,5],.25/80))
    assert torch.equal(x.grad[:,[1,2,3,4,6]],torch.zeros_like(x.grad[:,[1,2,3,4,6]]))


def test_original_CE_called_once_same_logits_weights_and_cap_uncapped():
    x=torch.zeros(1,7,1,2,requires_grad=True);y=torch.tensor([[[5,0]]]);weights=torch.ones(7)
    original=torch.tensor(3.,requires_grad=True)
    provider=mock.Mock(return_value=original)
    value,stats=loss.candidate_loss(x,y,weights,[GROUP],provider)
    provider.assert_called_once_with(x,y,weights)
    assert value.item()==pytest.approx(3.+.10*torch.log(torch.tensor(2.)).item())
    assert stats["coefficient"]==.1 and stats["coefficient_is_cap"] is False


def test_exact_original_reduction_AST_except_two_math_changes():
    original=next(n for n in ast.parse(Path(loss.previous.__file__).read_bytes()).body
        if isinstance(n,ast.FunctionDef) and n.name=="sign_margin_auxiliary")
    changed=loss.revised_auxiliary_AST(); count=0
    for node in ast.walk(changed):
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="margin" for t in node.targets):
            node.value=ast.parse("logits[:, SIGN_CLASS]-torch.logsumexp(logits[:, nonsign],dim=1)",mode="eval").body;count+=1
        if isinstance(node,ast.Subscript) and ast.unparse(node)=="negative_nll[i][target[i] == OTHER_CLASS]":
            node.slice=ast.parse("~positive",mode="eval").body;count+=1
    assert count==2 and ast.dump(changed)==ast.dump(original)


def test_prepared_contract_all27_inputs_schedule_control_parameter_law_unchanged():
    old=runner.previous.prepared_contract();new=runner.prepared_contract()
    for key in ("initial_model_weights_sha256","physical_TRAIN_slots","stratum_frame_counts","additional_updates_per_arm",
        "batch","examples_per_arm","seed","parameter_count","optimizer","learning_rate","weight_decay",
        "gradient_norm_cap","shared_crop_law","class_pixels","class_weights","RGB_views","known_evaluation_files",
        "exploratory_screen","strict_added_acceptance","original_schedule_sha256"):
        assert new[key]==old[key]
    assert new["source_count"]==12 and new["all_existing_twenty_plus_seven_screens_unchanged"]
    assert new["loss_contract"]["eligible_negative_target_class"]==0
    assert new["training_authorized"] is False and new["new_head"] is False


@pytest.mark.parametrize("fail",[False,True])
def test_real_parent_scope_contract_restore_and_control_direct_CE(fail):
    before=runner.previous.SCHEMA,runner.previous.SOURCE,runner.previous.losses,runner.previous.previous.SOURCE
    with runner.activated():
        with runner.previous.activated():
            assert runner.previous.previous.prepared_contract()["schema"]==runner.SCHEMA
            assert runner.previous.previous.SOURCE==runner.SOURCE
            assert runner.previous.losses is loss
            if fail:
                try: raise RuntimeError("synthetic original failure")
                except RuntimeError: pass
        import numpy as np
        ids=np.zeros(8,dtype=np.int64);x=torch.zeros(8,7,1,1);y=torch.zeros(8,1,1,dtype=torch.int64);w=torch.ones(7)
        expected=torch.tensor(2.);provider=mock.Mock(return_value=expected)
        assert runner.previous.arm_loss("control",x,y,w,ids,type("P",(),{"semantic_loss":provider})) is expected
        provider.assert_called_once_with(x,y,w)
    assert before==(runner.previous.SCHEMA,runner.previous.SOURCE,runner.previous.losses,runner.previous.previous.SOURCE)
    assert not runner._active and not runner.previous._active


def test_sourcecount_all10_original_pins_retained_without_data_read():
    assert len(runner.FROZEN_SOURCE_SHA)==10 and len(runner.SOURCE_NAMES)==12
    assert runner.FROZEN_SOURCE_SHA[Path(runner.previous.__file__).name]==runner.PARENT_RUNNER_SHA
    assert runner.FROZEN_SOURCE_SHA[Path(loss.previous.__file__).name]==loss.PARENT_SHA
    runner.verify_frozen()
