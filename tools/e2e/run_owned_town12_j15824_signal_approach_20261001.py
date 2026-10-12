#!/usr/bin/env python3
"""HH_261001 - Own one explicit bound-signal approach policy trial after the retained Yellow-to-Red abort."""

from contextlib import contextmanager
import json

from scripts.e2e import run_owned_town12_j15824_expert9_v2stop_20261001 as inherited
from scripts.e2e import capture_town12_j15824_signal_approach_20261001 as worker


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_approach9_v1"
WORKER = ROOT / "tools/e2e/capture_town12_j15824_signal_approach_20261001.py"
FAILED_NINE = inherited.OUTPUT / "owner_result.json"
FAILED_NINE_SHA256 = "1d64de4ccbb4f464b1e45ead39ffe0ef298868a840be94994b5572176eaba8fa"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/run_owned_town12_j15824_signal_approach_20261001.py",
    "tools/e2e/capture_town12_j15824_signal_approach_20261001.py",
    "learning/tests/test_town12_j15824_signal_approach_20261001.py",
    str(FAILED_NINE.relative_to(ROOT)),
    str((FAILED_NINE.parent / "capture/signal_frames.jsonl").relative_to(ROOT)))


@contextmanager
def configured_owner():
    previous = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.protocol_record
    protocol = inherited.protocol_record
    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES = OUTPUT, WORKER, SOURCE_FILES
        inherited.protocol_record = lambda: {**protocol(), "additional_signal_policy_revision": worker.policy_record(),
            "signal_policy_source_trial": str(FAILED_NINE.parent), "prior_failed_raw_rewritten": False}
        yield
    finally:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.protocol_record = previous


def main(argv=None):
    worker.scout.require(worker.scout.digest(FAILED_NINE) == FAILED_NINE_SHA256,
                         "frozen prior Yellow-to-Red failure receipt changed")
    result = json.loads(FAILED_NINE.read_text())
    worker.scout.require(result.get("status") == "FAIL" and result.get("owned_processes_stopped_and_ports_released") is True
        and result.get("source_bytes_unchanged_and_archived") is True
        and result.get("capture", {}).get("error") == "CollectionError: moving front bumper reached the red stop boundary",
        "prior observed Yellow-to-Red approach failure/cleanup proof is absent")
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
