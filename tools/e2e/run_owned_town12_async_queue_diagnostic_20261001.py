#!/usr/bin/env python3
"""HH_261001 - Own one Town12 startup with the distance-field build queue disabled."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

try:
    from . import run_owned_town12_distance_field_diagnostic_20261001 as diagnostic
except ImportError:
    from scripts.e2e import run_owned_town12_distance_field_diagnostic_20261001 as diagnostic


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_async_distance_field_queue_zero_v1"
CVAR = "r.AOAsyncBuildQueue"
OVERRIDE = f"-ini:Engine:[/Script/Engine.RendererSettings]:{CVAR}=0"
QUERY = f"-ExecCmds={CVAR}"
SOURCE_FILES = (*diagnostic.SOURCE_FILES,
                "tools/e2e/run_owned_town12_async_queue_diagnostic_20261001.py",
                "learning/tests/test_run_owned_town12_async_queue_diagnostic_20261001.py")


def diagnostic_contract() -> dict[str, Any]:
    return {
        "schema": "portable_e2e.town12_async_distance_field_startup_diagnostic.v1",
        "changed_renderer_cvar": CVAR,
        "expected_effective_integer": 0,
        "startup_override": OVERRIDE,
        "query_only_exec_command": QUERY,
        "shared_configuration_edit_requested": False,
        "hypothesis": "Disable the async distance-field build queue for game-thread requests.",
        "source_basis": {
            "file": "Engine/Source/Runtime/Engine/Private/DistanceFieldAtlas.cpp",
            "registration_line": 955,
            "queue_branch_line": 1119,
            "distance_field_replacement_line": 1266,
        },
        "limitation": (
            "Worker-thread requests still use the async queue. Cached distance fields "
            "and deferred render-state replacement remain enabled. Success after an "
            "earlier launch can reflect warmed DDC and does not prove causality. "
            "This is not a capture-quality validation."
        ),
        "capture_authorized": False,
        "training_data_admitted": False,
    }


@contextmanager
def configured_diagnostic() -> Iterator[None]:
    """HH_261001 - Select one changed CVar while retaining the original owner gates."""
    values = {
        "OUTPUT": OUTPUT, "CVAR": CVAR, "OVERRIDE": OVERRIDE, "QUERY": QUERY,
        "SOURCE_FILES": SOURCE_FILES, "diagnostic_contract": diagnostic_contract,
    }
    previous = {name: getattr(diagnostic, name) for name in values}
    try:
        for name, value in values.items():
            setattr(diagnostic, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(diagnostic, name, value)


def main(argv: list[str] | None = None) -> int:
    with configured_diagnostic():
        return diagnostic.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
