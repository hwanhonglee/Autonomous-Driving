"""HH_260906 - Verify TRAIN class balance and unchanged baseline formula without any GPU."""
import numpy as np
import pytest
from portable_e2e.map_learning_balance import class_weights,SignScaleStratifiedSampler


def test_baseline_formula_unchanged():
    counts=np.array([100,20,5,1])
    expected=1/np.sqrt(counts/counts.sum()+.001)
    assert np.allclose(class_weights(counts,'sqrt_frequency'),expected/expected.mean())


def test_equal_expected_class_contribution():
    counts=np.array([1000000,100,5,0])
    w=class_weights(counts,'class_balanced')
    assert np.allclose((counts*w)[:3],(counts*w)[0])
    assert w[-1]==0 and np.isfinite(w).all()


@pytest.mark.parametrize('counts',[[0,0],[-1,2],[np.nan,1]])
def test_invalid_histogram_refused(counts):
    with pytest.raises(ValueError):class_weights(counts,'class_balanced')


def test_sign_scale_sampler_has_fixed_train_only_quota_and_repeats():
    counts=np.array([0,0,1,4,5,64,65,200])
    sampler=SignScaleStratifiedSampler(counts)
    a=sampler.sample_batch(np.random.default_rng(19))
    b=sampler.sample_batch(np.random.default_rng(19))
    assert np.array_equal(a,b) and a.shape==(8,) and a.dtype==np.int64
    sampled=counts[a]
    assert np.count_nonzero((sampled>0)&(sampled<=64))==3
    assert np.count_nonzero(sampled>64)==1
    assert np.count_nonzero(sampled==0)==4
    assert sampler.contract()['stratum_frame_counts']==dict(small=4,larger=2,absent=2)


@pytest.mark.parametrize('counts',[[0,0,5],[0,65,80],[5,65,70],[],[-1,0,65],[0.1,5,65]])
def test_sign_scale_sampler_rejects_missing_or_invalid_train_strata(counts):
    with pytest.raises(ValueError):SignScaleStratifiedSampler(counts)


@pytest.mark.parametrize('limit',[0,-1,1.5,float('nan'),True])
def test_sign_scale_sampler_rejects_invalid_threshold(limit):
    with pytest.raises(ValueError):SignScaleStratifiedSampler([0,1,65],small_max_pixels=limit)
