"""HH_261005 - Create the fresh diagnostics parent explicitly; preserve V1 pre-timing failure and all counting/timing math."""

from contextlib import contextmanager
from pathlib import Path

from . import benchmark_full209_native_quota_v1_20261005 as previous


V1_SHA = "ea1726ab0d041c00536c3a1e359e1f9a4fdb803e25c7ec6cdd0f83947c0da530"
V1_TEST_SHA = "d772a6000d5d793641c18a17e7279bf6bd1c8d5c959a0a2af734d9557cf2ec10"
HERE = Path(__file__).resolve()
TEST = previous.ROOT / "learning/tests/test_benchmark_full209_native_quota_v2_20261005.py"
PLAN_DIR = previous.ROOT / "storage/experiments/plans/2026-10-05/full209_native_quota_v2"
OUTPUT = previous.ROOT / "storage/experiments/diagnostics/2026-10-05/full209_native_quota_v2"
_sources, _execute = previous.sources, previous.execute
_active = False


def sources():
    # HH_261005 - Preserve both actual V1 source/test pins in the fresh closure, instead of hiding the failed precursor.
    path = Path(previous.__file__)
    old_test = previous.ROOT / "learning/tests/test_benchmark_full209_native_quota_v1_20261005.py"
    previous.io.require(previous.io.digest(path) == V1_SHA and previous.io.digest(old_test) == V1_TEST_SHA,
                        "frozen V1 native quota comparator/test changed")
    result = _sources()
    result.update({str(path): V1_SHA, str(old_test): V1_TEST_SHA})
    return result


def execute(plan):
    # HH_261005 - Only prepare the canonical new date parent; genuine V1 creates the exclusive leaf and journals every call/error.
    previous.io.safe_path(OUTPUT.parent, missing_tail=True)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    return _execute(plan)


@contextmanager
def activated():
    global _active
    previous.io.require(not _active, "native quota V2 cannot nest")
    changes = dict(SCHEMA="portable_e2e.V4_all_entries_native_quota_ABBA.20261005.v2",
                   PLAN_DIR=PLAN_DIR, PLAN=PLAN_DIR / "declaration.json", LIBRARY=PLAN_DIR / "lib_full_tree_quota_v2.so",
                   OUTPUT=OUTPUT, HERE=HERE, TEST=TEST, sources=sources, execute=execute)
    saved = {name: getattr(previous, name) for name in changes}
    _active = True
    try:
        for name, value in changes.items():
            setattr(previous, name, value)
        yield
    finally:
        for name, value in saved.items():
            setattr(previous, name, value)
        _active = False


def main(argv=None):
    with activated():
        return previous.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
