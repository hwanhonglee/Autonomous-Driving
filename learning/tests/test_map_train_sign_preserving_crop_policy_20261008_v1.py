"""HH_261008 - Fixed scalar box policy fixtures only; no arrays, original payload or model runtime."""
import ast
import json
from pathlib import Path

import pytest
from scripts.e2e import map_train_sign_preserving_crop_policy_20261008_v1 as policy


@pytest.mark.parametrize("ordinal",[0,2,8,159998])
def test_only_even_positive_lost_support_receives_full_FOV(ordinal):
    proposal=[19,31,240,430];before=json.dumps(proposal)
    selected=policy.select_box(ordinal,proposal,11,0)
    assert selected["selected_box"]==list(policy.FULL_FOV) and selected["rescued"] is True
    assert selected["protected"] is True and selected["original_proposal_unchanged"] is False
    assert selected["same_selected_box_required_for_future_RGB_and_GT"] is True
    assert json.dumps(proposal)==before and selected["selected_box"] is not proposal


@pytest.mark.parametrize("ordinal,full,after",[(1,11,0),(3,1,0),(159999,230400,0),
    (0,0,0),(1,0,0),(0,11,1),(2,1,230400)])
def test_negative_unprotected_and_already_visible_proposals_byte_identical(ordinal,full,after):
    proposal=[0,10,360,500];raw=json.dumps(proposal).encode()
    selected=policy.select_box(ordinal,proposal,full,after)
    assert selected["selected_box"] is proposal and selected["rescued"] is False
    assert json.dumps(selected["selected_box"]).encode()==raw and json.dumps(proposal).encode()==raw


@pytest.mark.parametrize("count",[0,1,230400])
def test_genuine_full_FOV_copy_unchanged_at_zero_positive_and_maximum(count):
    proposal=[0,0,360,640]
    result=policy.select_box(0,proposal,count,count)
    assert result["selected_box"] is proposal and result["rescued"] is False


@pytest.mark.parametrize("ordinal,full,after",[(True,1,0),(1.0,1,0),(-1,1,0),(160000,1,0),
    (0,True,0),(0,1,False),(0,1.0,0),(0,1,0.0),(0,-1,0),(0,230401,0),
    (0,1,-1),(0,1,230401),(0,0,1)])
def test_support_and_chronology_types_bounds_and_negative_label_creation_deny(ordinal,full,after):
    with pytest.raises(ValueError):policy.select_box(ordinal,[0,0,240,430],full,after)


@pytest.mark.parametrize("box",[[0,0,360],[],[True,0,360,640],[0.,0,360,640],[-1,0,360,640],
    [0,-1,360,640],[0,0,0,640],[0,0,360,0],[1,0,360,640],[0,1,360,640],"0,0,360,640"])
def test_calibrated_crop_coordinates_not_coerced_or_clipped(box):
    with pytest.raises(ValueError):policy.select_box(0,box,1,0)


def test_full_FOV_must_match_exact_copy_count():
    with pytest.raises(ValueError):policy.select_box(0,list(policy.FULL_FOV),1,0)
    with pytest.raises(ValueError):policy.select_box(1,list(policy.FULL_FOV),7,6)


@pytest.mark.parametrize("batch,position,expected",[(0,0,0),(0,7,7),(1,0,8),(19999,7,159999)])
def test_global_chronology_uses_actual_batch_alignment(batch,position,expected):
    assert policy.exposure_index(batch,position)==expected


@pytest.mark.parametrize("batch,position",[(True,0),(0,False),(0.,0),(0,0.),(-1,0),(20000,0),(0,-1),(0,8)])
def test_wrong_batch_or_unaligned_position_fails_closed(batch,position):
    with pytest.raises(ValueError):policy.exposure_index(batch,position)


def test_fixed_even_ordinal_not_slot_or_performance_tuned_and_no_runtime_import():
    source=Path(policy.__file__).read_bytes();tree=ast.parse(source)
    assert not any(isinstance(part,(ast.Import,ast.ImportFrom)) for part in tree.body)
    function=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="select_box")
    assigned=next(part.value for part in function.body if isinstance(part,ast.Assign) and isinstance(part.targets[0],ast.Name) and part.targets[0].id=="protected")
    assert ast.dump(assigned,include_attributes=False)==ast.dump(ast.parse("global_exposure_ordinal % 2 == 0",mode="eval").body,include_attributes=False)
    value=policy.contract();assert value["exposures"]==160000 and value["sign_index"]==5
    assert value["training_steps"]==value["model_inferences"]==0 and value["actual_payload_read_authorized"] is False
    assert value["labels"]==["other","road","sidewalk","lane_marking","traffic_light_fixture","traffic_sign","pole"]
