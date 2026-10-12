#!/usr/bin/env python3
"""HH_261001 - Validate original API before instrumentation, then retain the trusted binding."""

from __future__ import annotations

from contextlib import contextmanager
import sys

from scripts.e2e import capture_town12_j15824_spawn_lift_20261001 as inherited


@contextmanager
def checked_api_before_hooks():
    original = inherited.scout.source_api
    trusted = original()
    original_run = inherited.collector.run

    def run(args):
        # HH_261001 - This callback runs inside the deliberate CheckedPlanner instrumentation.
        # The exact original module/class pair was source-validated before installing those hooks.
        current = inherited.scout.source_api
        try:
            inherited.scout.source_api = lambda: trusted
            return original_run(args)
        finally:
            inherited.scout.source_api = current

    try:
        inherited.collector.run = run
        yield
    finally:
        inherited.collector.run = original_run


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--execute" not in arguments:
        return inherited.main(arguments)
    def path_option(name):
        from pathlib import Path
        return Path(arguments[arguments.index(name) + 1]) if name in arguments else None
    # HH_261001 - Place the trusted-binding adapter outside checked_run, not outside base.execute.
    with inherited.configured_worker(path_option("--output"), path_option("--scout")):
        with checked_api_before_hooks():
            return inherited.base.main(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
