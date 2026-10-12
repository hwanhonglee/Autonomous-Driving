"""HH_261002 - Explicit root dispatch of the separately authorized fresh29-source V2 pair; no install/promotion."""
from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PARENT_DISPATCH_SHA = "5dcfe7cd08a532c1b8534597eedec998d13a05708ff881aa8d2bc67960171273"
PARENT_FILE = ROOT / "tools/e2e/run_prepared_temporal_intrinsic_curve_pair_20261002.py"
PREPARE_FILE = ROOT / "tools/e2e/dispatch_temporal_intrinsic_curve_pair_v2_20261002.py"
REVISION_SOURCE = "tools/e2e/train_temporal_intrinsic_curve_pair_v2_20261002.py"
MODULE = "scripts.e2e.train_temporal_intrinsic_curve_pair_v2_20261002"


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), "canonical pinned regular file required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


require(digest(PARENT_FILE) == PARENT_DISPATCH_SHA, "exact frozen V1 actual dispatcher source required")
_spec = importlib.util.spec_from_file_location("hh_intrinsic_pair_frozen_actual_dispatch_v1", PARENT_FILE)
previous = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(previous)
PREPARED = ROOT / "storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_dispatch_v2"
SOURCE = previous.REMOTE / "source_worktrees/temporal_intrinsic_curve_pair_20261002_v2"
RUN = previous.REMOTE / "runs/training/temporal_intrinsic_curve_pair_20261002_v2"
SSH = previous.SSH
GPU = previous.GPU


def _clone(name, namespace, replacements, *, revision_guard=False):
    require(digest(PARENT_FILE) == PARENT_DISPATCH_SHA, "frozen V1 dispatcher changed")
    original = next(n for n in ast.parse(PARENT_FILE.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == name)
    changed = copy.deepcopy(original)
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and (type(node.value), node.value) in replacements:
            node.value = replacements[(type(node.value), node.value)]
    inverse = copy.deepcopy(changed)
    reverse = {(type(new), new): old for (kind, old), new in replacements.items()}
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and (type(node.value), node.value) in reverse:
            node.value = reverse[(type(node.value), node.value)]
    require(ast.dump(inverse, include_attributes=False) == ast.dump(original, include_attributes=False),
        "V2 actual dispatch preserves every original statement except exact identity/cardinality literals")
    if revision_guard:
        # HH_261002 - Add one externally pinned revision check before any receipt creation or SSH; retain all28 old pins.
        locations = [i for i, n in enumerate(changed.body) if isinstance(n, ast.Assign)
            and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) and n.targets[0].id == "sources"]
        require(len(locations) == 1, "unknown prepared-source assignment shape")
        guard = ast.parse("require(sources.get(REVISION_SOURCE) == NEW_REVISION_SHA, 'external V2 training revision SHA differs')").body[0]
        changed.body.insert(locations[0] + 1, guard)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace[name]


def _remote_preflight():
    syntax = ast.parse(previous.REMOTE_PREFLIGHT)
    replacements = {
        (str, "/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/temporal_intrinsic_curve_pair_20261002_v1"): str(SOURCE),
        (str, "/home/pro6000/personal/hwanhong/portable_e2e/runs/training/temporal_intrinsic_curve_pair_20261002_v1"): str(RUN),
        (int, 29): 30, (int, 28): 29,
    }
    counts = {key: 0 for key in replacements}
    for node in ast.walk(syntax):
        if isinstance(node, ast.Constant) and (type(node.value), node.value) in replacements:
            key = (type(node.value), node.value); node.value = replacements[key]; counts[key] += 1
    require(all(count == 1 for count in counts.values()), "unknown source/GPU preflight source shape")
    return ast.unparse(syntax)


def _namespace(prepare_sha, revision_sha):
    return dict(vars(previous), __file__=__file__, ROOT=ROOT, PREPARED=PREPARED, SOURCE=SOURCE, RUN=RUN,
        SSH=SSH, PREPARE_SHA=prepare_sha, REMOTE_PREFLIGHT=_remote_preflight(),
        REVISION_SOURCE=REVISION_SOURCE, NEW_REVISION_SHA=revision_sha)


def expected_command(authority):
    namespace = _namespace("0" * 64, "0" * 64)
    return _clone("expected_command", namespace,
        {(str, "scripts.e2e.train_temporal_intrinsic_curve_pair_20261002"): MODULE})(authority)


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--expected-prepare-source-sha256", required=True)
    parser.add_argument("--expected-revision-sha256", required=True)
    extra, remaining = parser.parse_known_args(argv)
    require(all(type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)
        for value in (extra.expected_prepare_source_sha256, extra.expected_revision_sha256)), "exact external V2 SHA256 required")
    require(digest(PREPARE_FILE) == extra.expected_prepare_source_sha256,
        "exact externally reviewed V2 preparation source required")
    namespace = _namespace(extra.expected_prepare_source_sha256, extra.expected_revision_sha256)
    namespace["expected_command"] = expected_command
    replacements = {(int, 28): 29,
        (str, "frozen28 local source bytes changed"): "frozen29 local source bytes changed"}
    return _clone("main", namespace, replacements, revision_guard=True)(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
