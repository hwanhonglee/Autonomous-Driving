from __future__ import annotations

from pathlib import Path

from PIL import Image
import pytest

from portable_e2e.contract import ContractError
from portable_e2e.visualize import (
    _ego_centered_projection,
    _trajectory_pixel,
    render_trajectory_png,
)


def test_projection_centres_ego_and_keeps_all_quadrants_in_bounds() -> None:
    points = [(0.0, 0.0), (40.0, 18.0), (-30.0, -25.0)]
    origin_x, origin_y, scale = _ego_centered_projection(
        points,
        width=900,
        height=700,
        margin=60,
    )

    assert _trajectory_pixel(
        (0.0, 0.0),
        origin_x=origin_x,
        origin_y=origin_y,
        scale=scale,
    ) == (450, 350)
    for point in points:
        pixel_x, pixel_y = _trajectory_pixel(
            point,
            origin_x=origin_x,
            origin_y=origin_y,
            scale=scale,
        )
        assert 60 <= pixel_x <= 840
        assert 60 <= pixel_y <= 640


def test_render_centres_ego_and_preserves_forward_and_reverse_routes(
    tmp_path: Path,
) -> None:
    output = tmp_path / "centred.png"
    render_trajectory_png(
        output,
        route_xy=[(-25.0, -8.0), (0.0, 0.0), (35.0, 18.0)],
        target_xy=[(-20.0, -5.0), (0.0, 0.0), (30.0, 14.0)],
        target_valid=[True, True, True],
        candidate_xy=[
            [(-18.0, -6.0), (0.0, 0.0), (32.0, 16.0)],
            [(-15.0, 7.0), (0.0, 0.0), (28.0, -12.0)],
        ],
        candidate_logits=[2.0, 1.0],
        title="ego-centred turn",
    )

    with Image.open(output) as image:
        assert image.size == (900, 700)
        # The ego body covers the exact canvas centre.
        assert image.getpixel((450, 350)) != (248, 248, 248)


def test_projection_rejects_invalid_drawable_area() -> None:
    with pytest.raises(ContractError, match="margin leaves no drawable area"):
        _ego_centered_projection(
            [(0.0, 0.0)],
            width=320,
            height=240,
            margin=120,
        )
