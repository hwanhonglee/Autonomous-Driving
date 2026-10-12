#!/usr/bin/env python3
"""HH_260906 - Isolate DEV conversion authority while retaining unchanged Common10 conversion math."""
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

ADAPTER_ID = "carla_fixed_moving_dev_task_to_common10_v2"


def __getattr__(name):
    # HH_260906 - Reuse the original native reader, geometry conversion and immutable contract helpers.
    return getattr(base, name)


@contextmanager
def _bound_dev_qualification():
    previous = importlib.import_module("scripts.e2e.qualify_carla_moving_common10")
    from scripts.e2e import qualify_carla_moving_dev_common10_v2 as dev
    old_attribute = getattr(e2e, "qualify_carla_moving_common10", previous)
    previous_writer = base._write_json

    def write_dev_metadata(path, value):
        # HH_260906 - Change only new converter identity fields before their hashes are calculated; keep native bytes untouched.
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

    # HH_260906 - Relative imports inside base resolve this explicit owned-process dispatcher, not edited legacy code.
    e2e.qualify_carla_moving_common10 = dev
    base._write_json = write_dev_metadata
    try:
        yield
    finally:
        base._write_json = previous_writer
        e2e.qualify_carla_moving_common10 = old_attribute


def require_qualification(args):
    if (getattr(args, "moving_qualification", None) is None
            or getattr(args, "moving_qualification_sha256", None) is None):
        raise base.AdapterError("DEV converter requires paired moving qualification and external SHA flags")


def prepare_dataset(args):
    require_qualification(args)
    with _bound_dev_qualification():
        return base.prepare_dataset(args)


def main(argv=None):
    args = base._parse_args(argv)
    require_qualification(args)
    # HH_260906 - Main delegates printing/error handling too; all authorization remains the strict DEV qualifier.
    with _bound_dev_qualification():
        return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
