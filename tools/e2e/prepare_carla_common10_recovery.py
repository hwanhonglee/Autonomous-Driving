#!/usr/bin/env python3
"""HH_260906 - Bind sealed recovery qualification to unchanged Common10 conversion math."""
from __future__ import annotations

from contextlib import contextmanager
import importlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import e2e
from scripts.e2e import prepare_carla_common10_dataset as base

ADAPTER_ID = "carla_sealed_moving_recovery_to_common10_v1"


def __getattr__(name):
    """HH_260906 - Reuse the unchanged native reader, validator, and geometry helpers."""
    return getattr(base, name)


@contextmanager
def _bound_recovery_qualification():
    """HH_260906 - Scope the recovery authority to one owned conversion call and restore legacy dispatch."""
    previous = importlib.import_module("scripts.e2e.qualify_carla_moving_common10")
    from scripts.e2e import qualify_carla_moving_recovery_common10 as recovery
    old_attribute = getattr(e2e, "qualify_carla_moving_common10", previous)
    previous_writer = base._write_json

    def write_recovery_metadata(path, value):
        """HH_260906 - Mark only new adapter identity fields while retaining every native evidence byte."""
        if Path(path).name == "collection_config.json":
            if value.get("adapter_id") != "carla_fixed_moving_task_to_common10_v1":
                raise base.AdapterError("unexpected base moving adapter identity")
            value = dict(value, adapter_id=ADAPTER_ID)
        elif Path(path).name == "episode.json":
            provenance = value.get("source_provenance", {})
            if provenance.get("adapter_id") != "carla_fixed_moving_task_to_common10_v1":
                raise base.AdapterError("unexpected episode moving adapter identity")
            value = dict(value, source_provenance=dict(provenance, adapter_id=ADAPTER_ID))
        return previous_writer(path, value)

    # HH_260906 - Relative imports inside the base converter resolve this process-local sealed authority.
    e2e.qualify_carla_moving_common10 = recovery
    base._write_json = write_recovery_metadata
    try:
        yield
    finally:
        base._write_json = previous_writer
        e2e.qualify_carla_moving_common10 = old_attribute


def require_qualification(args):
    """HH_260906 - Refuse conversion unless both the receipt path and external digest are supplied."""
    if (getattr(args, "moving_qualification", None) is None
            or getattr(args, "moving_qualification_sha256", None) is None):
        raise base.AdapterError("recovery converter requires paired moving qualification and external SHA flags")


def prepare_dataset(args):
    """HH_260906 - Convert one qualified sealed recovery episode through unchanged Common10 logic."""
    require_qualification(args)
    with _bound_recovery_qualification():
        return base.prepare_dataset(args)


def main(argv=None):
    """HH_260906 - Parse with the base CLI but require recovery authority before any conversion work."""
    args = base._parse_args(argv)
    require_qualification(args)
    with _bound_recovery_qualification():
        return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
