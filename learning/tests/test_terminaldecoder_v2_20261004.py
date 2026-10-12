"""HH_261004 - Verify a new semantic grid with unchanged inherited physics and no actuation."""

import math

import pytest
import torch

from portable_e2e.path_time_decoder_terminal_v1_20261002 import PathTimeDecoderTerminalV1, TerminalDecoderError
from portable_e2e.terminaldecoder_v2_20261004 import PathTimeDecoderTerminalV2, prepared_contract


@pytest.mark.parametrize("distance,boundary", [(17.9, .24), (9.999999999818003, .058948315382003784),
    (4.9999999999313065, .1028044056892395), (2.4999999999953406, .1028044056892395),
    (.001, .001), (.3, .0049), (64., .25)])
def test_semantic_anchors_are_exact_and_sampling_never_exceeds_old_maximum(distance, boundary):
    decoder = PathTimeDecoderTerminalV2()
    d, s = torch.tensor(distance, dtype=torch.float64), torch.tensor(boundary, dtype=torch.float64)
    grid = decoder._grid(d, s)
    assert grid[0].item() == 0. and grid[-1].item() == distance
    assert bool((grid == s).any())
    assert bool((grid[1:] > grid[:-1]).all())
    assert (grid[1:] - grid[:-1]).max().item() <= min(.25, distance / 256.) + 1.e-14
    for raw_knot in range(2, 65, 2):
        if raw_knot < distance:
            assert bool((grid == raw_knot).any())


@pytest.mark.parametrize("anchor", range(2, 65, 2))
@pytest.mark.parametrize("direction", [0., math.inf])
def test_nextafter_endpoint_cells_remain_positive_not_merged_or_relabelled(anchor, direction):
    distance = math.nextafter(float(anchor), direction)
    if distance > 64.:
        return
    grid = PathTimeDecoderTerminalV2()._grid(torch.tensor(distance, dtype=torch.float64), torch.tensor(.24, dtype=torch.float64))
    assert grid[-1].item() == distance
    assert bool((grid[1:] > grid[:-1]).all())
    if distance > anchor:
        assert grid[-2].item() == anchor
        assert grid[-1].item() - grid[-2].item() == distance - anchor


@pytest.mark.parametrize("distance,boundary", [(math.nan, .1), (math.inf, .1), (0., 0.), (-1., 0.), (1., -.1), (1., 1.1)])
def test_malformed_grid_values_reject(distance, boundary):
    with pytest.raises(TerminalDecoderError, match="actual D"):
        PathTimeDecoderTerminalV2()._grid(torch.tensor(distance, dtype=torch.float64), torch.tensor(boundary, dtype=torch.float64))


def test_genuinely_unrepresentable_subdivision_rejects_without_deleting_cells():
    tiny = math.nextafter(0., 1.)
    with pytest.raises(TerminalDecoderError, match="subdivided representably"):
        PathTimeDecoderTerminalV2()._grid(torch.tensor(tiny, dtype=torch.float64), torch.tensor(0., dtype=torch.float64))


def test_every_non_grid_frozen_terminal_method_is_exactly_inherited():
    assert set(PathTimeDecoderTerminalV2.__dict__) - {"__module__", "__doc__"} == {"_grid"}
    for method in ("forward", "_inputs", "_witness", "_raw_at", "_shape", "_caps", "_envelope", "_sample_shape", "_audit"):
        assert getattr(PathTimeDecoderTerminalV2, method) is getattr(PathTimeDecoderTerminalV1, method)
    contract = prepared_contract()
    assert contract["overridden_methods"] == ["_grid"]
    assert contract["near_cell_merge"] is False
    assert contract["authentic_distance_changed"] is False
    assert contract["vehicle_control_approved"] is False
