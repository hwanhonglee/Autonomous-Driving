"""HH_260906 - TRAIN-only class weighting for a controlled small-object perception ablation."""
import numpy as np


def class_weights(pixel_counts, mode):
    counts=np.asarray(pixel_counts,dtype=np.float64)
    if counts.ndim!=1 or not np.isfinite(counts).all() or (counts<0).any() or counts.sum()<=0:
        raise ValueError('finite nonnegative class counts required')
    if mode=='sqrt_frequency':
        weights=1/np.sqrt(counts/counts.sum()+.001)
    elif mode=='class_balanced':
        # HH_260906 - Equalize expected TRAIN class contribution; unseen classes cannot be learned.
        weights=np.zeros_like(counts)
        present=counts>0
        weights[present]=1/counts[present]
    else:
        raise ValueError('unknown weighting mode')
    return weights/weights.mean()


class SignScaleStratifiedSampler:
    """HH_260906 - Expose small TRAIN signs without using DEV pixels or changing labels."""

    def __init__(self, sign_pixels_per_frame, *, small_max_pixels=64):
        counts=np.asarray(sign_pixels_per_frame)
        if (counts.ndim!=1 or counts.size==0 or not np.issubdtype(counts.dtype,np.integer)
                or (counts<0).any() or isinstance(small_max_pixels,bool)
                or not isinstance(small_max_pixels,(int,np.integer)) or small_max_pixels<1):
            raise ValueError('nonnegative integer TRAIN sign counts required')
        self.small=np.flatnonzero((counts>0)&(counts<=small_max_pixels))
        self.large=np.flatnonzero(counts>small_max_pixels)
        self.absent=np.flatnonzero(counts==0)
        if not all(len(group) for group in (self.small,self.large,self.absent)):
            raise ValueError('small, large and absent TRAIN sign strata required')
        self.small_max_pixels=small_max_pixels

    def sample_batch(self, rng):
        """HH_260906 - Fixed 3:1:4 quota retains sign-negative context against false positives."""
        ids=np.concatenate((rng.choice(self.small,3),rng.choice(self.large,1),
                            rng.choice(self.absent,4)))
        return rng.permutation(ids).astype(np.int64,copy=False)

    def contract(self):
        return dict(small_max_pixels=self.small_max_pixels,batch_size=8,
                    small_sign_frames_per_batch=3,larger_sign_frames_per_batch=1,
                    no_sign_frames_per_batch=4,stratum_frame_counts=dict(
                        small=int(len(self.small)),larger=int(len(self.large)),absent=int(len(self.absent))),
                    replacement_within_stratum=True,source_split='TRAIN only')
