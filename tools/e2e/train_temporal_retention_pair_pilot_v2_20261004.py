"""HH_261004 - Seal OLD TRAIN admission artifacts for the fresh undeployed pilot revision.

The V1 source/declaration remains intact; its source review found missing
postfit pins before any GPU work. V2 changes only input-identity protection,
study/schema identity and source closure, not either arm's training math.
"""
from contextlib import contextmanager
from pathlib import Path

from . import train_temporal_retention_pair_pilot_20261004 as previous

ROOT, require, digest = previous.ROOT, previous.require, previous.digest
SCHEMA = "portable_e2e.temporal_retention_train_pair_pilot.20261004.v2"
STUDY = "temporal_retention_pair_pilot_20261004_v2"
V1_SHA = "c10fdf5d184d09b05b22ea459c5081724ee71726f71ebdabeefc45ee9729bfb4"
_contract, _verify_parent = previous.contract, previous.pair.verify_parent
_active = False


def source_snapshot():
    require(digest(previous.__file__) == V1_SHA, "undeployed V1 must remain byte-exact")
    old = previous.frozen.source_snapshot()
    paths = set(previous.frozen.replay.imported_source_closure(ROOT,
        [Path(__file__), ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]))
    paths.add(ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json")
    result = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(all(result.get(k) == sha for k, sha in old.items()), "all original31 frozen sources required")
    return result


def contract():
    require(_active, "explicit revised pilot scope required")
    value = _contract()
    return dict(value, old_TRAIN_admission_artifacts_pinned_before_first_fit=True,
        pilot_authority_pre_and_postfit_pinned=True, undeployed_V1_no_GPU_work=True)


def verify_parent(args, order, datasets):
    parent, pins, screens = _verify_parent(args, order, datasets)
    extra = (args.pilot_authority, args.recovery_conversion, args.recovery_train / "dataset.json",
        previous.frozen.replay.LOW_PATH / "dataset.json",
        previous.frozen.replay.LOW_PATH / "moving_conversion_qualification.json",
        previous.frozen.replay.LEGACY_PATH / "dataset.json")
    for path in extra:
        previous.frozen.regular(path, previous.frozen.original.PERSONAL)
        sha = digest(path)
        require(str(path) not in pins or pins[str(path)] == sha, "conflicting OLD TRAIN admission pin")
        pins[str(path)] = sha
    require(pins[str(args.pilot_authority)] == args.pilot_authority_sha256,
        "pilot authority changed between authorized first read and input sealing")
    return parent, pins, screens


@contextmanager
def activated():
    global _active
    require(not _active and previous._created_output is None and previous.pair.verify_parent is _verify_parent,
        "exclusive revised pilot scope required")
    saved = {k: getattr(previous, k) for k in ("SCHEMA", "STUDY", "source_snapshot", "contract")}
    _active = True
    try:
        previous.SCHEMA, previous.STUDY = SCHEMA, STUDY
        previous.source_snapshot, previous.contract = source_snapshot, contract
        previous.pair.verify_parent = verify_parent
        yield
    finally:
        previous.pair.verify_parent = _verify_parent
        for key, value in saved.items(): setattr(previous, key, value)
        _active = False


def main(argv=None):
    with activated(): return previous.main(argv)


if __name__ == "__main__": raise SystemExit(main())
