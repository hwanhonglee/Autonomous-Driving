"""HH_261001 - Bound the raw-lane neighborhood accounting on synthetic masks."""

import numpy as np

from scripts.e2e.audit_map_train_lane_support_20261001 import lane_neighborhood_histogram


def test_single_pixel_has_one_neighbor():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    counts = lane_neighborhood_histogram(mask)
    assert sum(counts) == 1 and counts[1] == 1 and counts[9] == 0


def test_three_by_three_patch_has_one_complete_interior():
    mask = np.zeros((5, 5), dtype=bool)
    mask[1:4, 1:4] = True
    counts = lane_neighborhood_histogram(mask)
    assert sum(counts) == 9 and counts[9] == 1 and sum(counts[:9]) == 8


def test_wrong_dtype_is_rejected():
    try:
        lane_neighborhood_histogram(np.zeros((3, 3), dtype=np.uint8))
    except ValueError:
        return
    raise AssertionError("Non-boolean masks must be rejected")
