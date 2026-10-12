"""HH_261004 - Preserve frozen thread468 V1 execution and correct only JSON shape representation in a fresh revision.

V1 admission failed before any worker/NN because raw_heads tuple shapes become
JSON lists. V2 uses lists in that one contract field and new module/schema/paths;
all neural, tensor, decoder, timing and tolerance functions remain original V1.

After root review only, never as part of source preparation:
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.benchmark_full209_thread468_v2_20261004 -- --mode declare
CUDA_VISIBLE_DEVICES='' /usr/bin/python3 tools/workspace_python.py -m \
 scripts.e2e.benchmark_full209_thread468_v2_20261004 -- --mode execute \
 --expected-declaration-sha256 <printed prospective declaration SHA256>
"""

from contextlib import contextmanager
from pathlib import Path

from . import benchmark_full209_thread468_v1_20261004 as original


MODULE = "scripts.e2e.benchmark_full209_thread468_v2_20261004"
SCHEMA = "portable_e2e.full209_masked_copy_thread468_JPEG_counterfactual.20261004.v2"
HERE = Path(__file__).resolve()
TEST = original.ROOT / "learning/tests/test_full209_thread468_v2_20261004.py"
PLAN = original.ROOT / "storage/experiments/plans/2026-10-04/full209_thread468_JPEG_counterfactual_v2/declaration.json"
OUTPUT = original.ROOT / "storage/experiments/diagnostics/2026-10-04/full209_thread468_JPEG_counterfactual_v2"
ORIGINAL_SOURCE = Path(original.__file__).resolve()
ORIGINAL_TEST = original.TEST
ORIGINAL_SOURCE_SHA = "bf682420e3fe8ee637e008aff66fcb88d8690962bc9eb1ccb3b15cb3a3a9f447"
ORIGINAL_TEST_SHA = "6457c1f4d44991e149d8f811e86f96013315324ceeeadf3a1cfbca8655e14a93"
_ORIGINAL_CONTRACT = original.contract
_ORIGINAL_SOURCE_SNAPSHOT = original.source_snapshot


def contract():
    value = _ORIGINAL_CONTRACT()
    value["raw_heads"] = {name: list(shape) for name, shape in original.RAW_SHAPES.items()}
    return value


def source_snapshot():
    original.require(original.digest(ORIGINAL_SOURCE) == ORIGINAL_SOURCE_SHA
        and original.digest(ORIGINAL_TEST) == ORIGINAL_TEST_SHA, "frozen thread468 V1 source/test changed")
    value = _ORIGINAL_SOURCE_SNAPSHOT()
    value.update({str(path.relative_to(original.ROOT)): sha for path, sha in
        ((ORIGINAL_SOURCE, ORIGINAL_SOURCE_SHA), (ORIGINAL_TEST, ORIGINAL_TEST_SHA))})
    return value


@contextmanager
def activated_revision():
    # HH_261004 - Scope only revision identity, closure and JSON shape serialization; restore even when V1 retains a failure.
    fresh = dict(MODULE=MODULE, SCHEMA=SCHEMA, PLAN=PLAN, OUTPUT=OUTPUT, HERE=HERE, TEST=TEST,
        contract=contract, source_snapshot=source_snapshot)
    saved = {name: getattr(original, name) for name in fresh}
    try:
        for name, value in fresh.items():
            setattr(original, name, value)
        yield
    finally:
        for name, value in saved.items():
            setattr(original, name, value)


def main(argv=None):
    with activated_revision():
        return original.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
