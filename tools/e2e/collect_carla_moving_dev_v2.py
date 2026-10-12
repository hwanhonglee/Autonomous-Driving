#!/usr/bin/env python3
"""HH_260906 - Bind one declared DEV task to the unchanged expert collector."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path
import sys
from typing import Any, Iterator, Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.e2e import collect_carla_vad_expert as base


def moving_capture_module(args: argparse.Namespace) -> Any:
    # HH_260906 - This entrypoint never falls back to an undeclared ordinary capture.
    if (getattr(args, "moving_task_plan", None) is None
            or getattr(args, "moving_task_plan_sha256", None) is None):
        raise ValueError("DEV moving capture requires paired --moving-task-plan and --moving-task-plan-sha256")
    from scripts.e2e import carla_moving_capture_dev_v2 as moving

    moving.configuration_from_args(args)
    return moving


@contextmanager
def _bound_dev_capture() -> Iterator[None]:
    # HH_260906 - Only the owned process changes its hook; sealed collector bytes remain untouched.
    previous = base.moving_capture_module
    base.moving_capture_module = moving_capture_module
    try:
        yield
    finally:
        base.moving_capture_module = previous


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    # HH_260906 - The original strict parser calls the DEV contract before any output or CARLA access.
    with _bound_dev_capture():
        return base.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    # HH_260906 - Keep the binding through preflight, collection and final source/input rechecks.
    with _bound_dev_capture():
        return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
