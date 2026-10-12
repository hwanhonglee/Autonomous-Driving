"""HH_260906 - Check region-size accounting and non-causal selection decomposition."""
import numpy as np
import pytest
from scripts.e2e.analyze_dual_recovery_ah import region_stats,driving_breakdown


def test_region_counts_and_pixel_recall_not_instance_accuracy():
    target=np.zeros((2,12,12),dtype=np.uint8);pred=target.copy()
    target[0,0,0]=5;target[0,4:7,4:7]=5;target[1,2,2]=5
    pred[0,4,4]=5
    report=region_stats(target,pred,5)
    assert report['1-4']['regions']==2 and report['1-4']['zero_hit_regions']==2
    assert report['5-16']['regions']==1 and report['5-16']['pixel_recall']==pytest.approx(1/9)
    assert report['65-inf']['pixel_recall'] is None


def test_diagonals_are_one_eight_connected_region():
    target=np.eye(3,dtype=np.uint8)[None]
    assert region_stats(target,target,1)['1-4']['regions']==1


def test_empty_label_has_no_recall_claim():
    x=np.zeros((1,4,4),dtype=np.uint8)
    assert all(row['pixel_recall'] is None for row in region_stats(x,x,5).values())


def test_gap_preserves_candidate_error():
    m=dict(selected_ade_m=4.,oracle_ade_m=1.5,ade_1p0s_m=.1,ade_3p0s_m=.8,ade_6p4s_m=4.)
    g=driving_breakdown(dict(subgroups={'turn':dict(metrics=m)}))['turn']
    assert g['selection_gap_m']==2.5 and g['oracle_ade_m']==1.5
    m['selected_ade_m']=1.
    with pytest.raises(ValueError):driving_breakdown(dict(subgroups={'turn':dict(metrics=m)}))
