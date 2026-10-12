"""Dependency-light trajectory evidence rendering with Pillow."""

from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile
from typing import Sequence

from PIL import Image, ImageDraw

from .contract import ContractError


_CANDIDATE_COLORS = (
    (220, 60, 60),
    (65, 105, 225),
    (255, 140, 0),
    (148, 0, 211),
    (0, 160, 160),
    (180, 120, 40),
    (120, 120, 120),
    (240, 105, 180),
)


def _finite_point(point: Sequence[float]) -> tuple[float, float]:
    if len(point) != 2:
        raise ContractError("trajectory visualization points must contain x and y")
    x, y = float(point[0]), float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ContractError("trajectory visualization received NaN or Inf")
    return x, y


def _ego_centered_projection(
    points: Sequence[tuple[float, float]],
    *,
    width: int,
    height: int,
    margin: int,
) -> tuple[float, float, float]:
    """Return an ego-centred projection that keeps every point in bounds."""
    if not points:
        raise ContractError("trajectory visualization projection needs points")
    if margin <= 0 or 2 * margin >= min(width, height):
        raise ContractError("trajectory visualization margin leaves no drawable area")
    forward_extent = max(10.0, max(abs(point[0]) for point in points))
    lateral_extent = max(5.0, max(abs(point[1]) for point in points))
    half_width = width / 2.0 - margin
    half_height = height / 2.0 - margin
    scale = min(
        half_width / (lateral_extent * 1.05),
        half_height / (forward_extent * 1.05),
    )
    return width / 2.0, height / 2.0, scale


def _trajectory_pixel(
    point: tuple[float, float],
    *,
    origin_x: float,
    origin_y: float,
    scale: float,
) -> tuple[int, int]:
    """Project ego x-forward/y-left coordinates to screen coordinates."""
    x, y = point
    return round(origin_x - y * scale), round(origin_y - x * scale)


def render_trajectory_png(
    output: Path | str,
    *,
    route_xy: Sequence[Sequence[float]],
    target_xy: Sequence[Sequence[float]],
    target_valid: Sequence[bool],
    candidate_xy: Sequence[Sequence[Sequence[float]]],
    candidate_logits: Sequence[float],
    title: str,
    width: int = 900,
    height: int = 700,
) -> None:
    """Render route, expert target, and every candidate in ego coordinates."""
    if len(candidate_xy) == 0 or len(candidate_xy) != len(candidate_logits):
        raise ContractError("candidate trajectories and logits must be nonempty and aligned")
    if len(target_xy) != len(target_valid):
        raise ContractError("target trajectory and valid mask must be aligned")
    if width < 320 or height < 240:
        raise ContractError("trajectory visualization canvas is too small")
    route = [_finite_point(point) for point in route_xy]
    target = [
        _finite_point(point) for point, valid in zip(target_xy, target_valid) if valid
    ]
    candidates = [
        [_finite_point(point) for point in candidate] for candidate in candidate_xy
    ]
    if not target:
        raise ContractError("trajectory visualization needs at least one valid target point")
    all_points = [(0.0, 0.0), *route, *target]
    for candidate in candidates:
        all_points.extend(candidate)
    margin = 60
    origin_x, origin_y, scale = _ego_centered_projection(
        all_points,
        width=width,
        height=height,
        margin=margin,
    )

    def pixel(point: tuple[float, float]) -> tuple[int, int]:
        return _trajectory_pixel(
            point,
            origin_x=origin_x,
            origin_y=origin_y,
            scale=scale,
        )

    image = Image.new("RGB", (width, height), (248, 248, 248))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, width - 1, height - 1), outline=(70, 70, 70), width=2)
    draw.text((20, 16), title, fill=(20, 20, 20))
    draw.text(
        (20, 38),
        "ego frame: x=forward (up), y=left (screen left)",
        fill=(80, 80, 80),
    )
    if len(route) >= 2:
        draw.line([pixel(point) for point in route], fill=(120, 120, 120), width=5)

    selected = max(range(len(candidate_logits)), key=lambda index: candidate_logits[index])
    for index, candidate in enumerate(candidates):
        if len(candidate) < 2:
            continue
        color = _CANDIDATE_COLORS[index % len(_CANDIDATE_COLORS)]
        line_width = 5 if index == selected else 2
        draw.line([pixel(point) for point in candidate], fill=color, width=line_width)
        endpoint = pixel(candidate[-1])
        draw.ellipse(
            (endpoint[0] - 4, endpoint[1] - 4, endpoint[0] + 4, endpoint[1] + 4),
            fill=color,
        )
    draw.line([pixel(point) for point in target], fill=(0, 150, 70), width=5)
    ego_x, ego_y = pixel((0.0, 0.0))
    draw.rectangle(
        (ego_x - 8, ego_y - 16, ego_x + 8, ego_y + 4),
        fill=(30, 30, 30),
        outline=(0, 120, 255),
        width=3,
    )
    draw.text((20, height - 42), "gray=route  green=expert  thick=color=selected", fill=(30, 30, 30))
    draw.text(
        (width - 260, height - 42),
        f"selected candidate: {selected}",
        fill=_CANDIDATE_COLORS[selected % len(_CANDIDATE_COLORS)],
    )

    output_path = Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.tmp.", dir=output_path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            image.save(stream, format="PNG")
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, output_path)
        except FileExistsError as error:
            raise ContractError(
                f"refusing to overwrite trajectory evidence: {output_path}"
            ) from error
    finally:
        if temporary.exists():
            temporary.unlink()
