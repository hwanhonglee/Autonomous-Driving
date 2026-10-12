"""HH_260906 - Keep route proximity numerically correct and distinct from lane approval."""
import pytest
from scripts.e2e.audit_reference_corridor_w import distances_to_polyline


def test_projection_and_endpoint():
    assert distances_to_polyline([[5, 2], [12, 0]], [[0, 0], [10, 0]]).tolist() == pytest.approx([2, 2])


def test_duplicate_and_corner():
    assert distances_to_polyline([[9, 1]], [[0, 0], [0, 0], [10, 0], [10, 10]])[0] == pytest.approx(1)


def test_invalid_route():
    with pytest.raises(ValueError):
        distances_to_polyline([[0, 0]], [[float('nan'), 0], [1, 1]])
