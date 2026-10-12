"""HH_261002 - Fresh V2 private preparation for the explicit driver-residual preflight revision; NO fit."""
from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
import hashlib
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import dispatch_temporal_intrinsic_curve_pair_20261002 as previous
from scripts.e2e import train_temporal_intrinsic_curve_pair_v2_20261002 as revision

ROOT = Path(__file__).resolve().parents[2]
PARENT_PREPARE_SHA = "d50f8ac2530503df4ab4a3647a84b1b2bdfbfe9bdb8904711a5efaf5b6210be5"
REVISION_SOURCE = "tools/e2e/train_temporal_intrinsic_curve_pair_v2_20261002.py"
MODULE = "scripts.e2e.train_temporal_intrinsic_curve_pair_v2_20261002"
SOURCE = previous.REMOTE / "source_worktrees/temporal_intrinsic_curve_pair_20261002_v2"
RUN = previous.REMOTE / "runs/training/temporal_intrinsic_curve_pair_20261002_v2"
_active = False


def require(value, message):
    if not value:
        raise ValueError(message)


def _parent_source():
    path = Path(previous.__file__)
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents))
        and hashlib.sha256(path.read_bytes()).hexdigest() == PARENT_PREPARE_SHA,
        "exact frozen V1 preparation source required")
    return path.read_text()


def _clone(name, namespace, replacements):
    original = next(n for n in ast.parse(_parent_source()).body if isinstance(n, ast.FunctionDef) and n.name == name)
    changed = copy.deepcopy(original); edits = []
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and (type(node.value), node.value) in replacements:
            old = node.value; node.value = replacements[(type(old), old)]; edits.append((node, old))
    inverse = copy.deepcopy(changed)
    reverse = {(type(new), new): old for (kind, old), new in replacements.items()}
    for node in ast.walk(inverse):
        if isinstance(node, ast.Constant) and (type(node.value), node.value) in reverse:
            node.value = reverse[(type(node.value), node.value)]
    require(ast.dump(inverse, include_attributes=False) == ast.dump(original, include_attributes=False),
        "V2 preparation must preserve every original statement except explicit identity/cardinality literals")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace[name]


def _remote_copy():
    syntax = ast.parse(previous.REMOTE_COPY)
    replacements = {
        (str, "source_worktrees/temporal_intrinsic_curve_pair_20261002_v1"):
            "source_worktrees/temporal_intrinsic_curve_pair_20261002_v2",
        (str, "runs/training/temporal_intrinsic_curve_pair_20261002_v1"):
            "runs/training/temporal_intrinsic_curve_pair_20261002_v2",
        (int, 29): 30, (int, 28): 29,
        (str, "EXACT28_SOURCE_PLUS_AUTH_CREATE_ONLY_COPIED"): "EXACT29_SOURCE_PLUS_AUTH_CREATE_ONLY_COPIED",
    }
    counts = {key: 0 for key in replacements}
    for node in ast.walk(syntax):
        if isinstance(node, ast.Constant) and (type(node.value), node.value) in replacements:
            key = (type(node.value), node.value); node.value = replacements[key]; counts[key] += 1
    require(counts == {(str, "source_worktrees/temporal_intrinsic_curve_pair_20261002_v1"): 1,
        (str, "runs/training/temporal_intrinsic_curve_pair_20261002_v1"): 1, (int, 29): 1, (int, 28): 1,
        (str, "EXACT28_SOURCE_PLUS_AUTH_CREATE_ONLY_COPIED"): 1}, "unknown remote copy source shape")
    return ast.unparse(syntax)


def _namespace():
    pair = revision.pair
    training = SimpleNamespace(original=pair.original, require=pair.require, digest=pair.digest, write_new=pair.write_new,
        PARENT=pair.PARENT, contract=revision.contract, source_snapshot=revision.source_snapshot,
        validate_authorization=pair.validate_authorization, authorize_parent_history=pair.authorize_parent_history)
    return dict(vars(previous), __file__=__file__, ROOT=ROOT, training=training, SOURCE=SOURCE, RUN=RUN,
        REMOTE_COPY=_remote_copy())


def command(auth_sha, *, validate):
    namespace = _namespace()
    fn = _clone("command", namespace, {(str, "scripts.e2e.train_temporal_intrinsic_curve_pair_20261002"): MODULE})
    return fn(auth_sha, validate=validate)


def package(sources, authorization, auth_sha):
    return _clone("package", _namespace(), {})(sources, authorization, auth_sha)


@contextmanager
def activated():
    global _active
    require(not _active, "V2 prepare cannot nest")
    _active = True
    try:
        # HH_261002 - The new authority keeps the frozen fit/math schema and explicitly appends its V2 GPU revision.
        with revision.activated():
            yield
    finally:
        _active = False


def main(argv=None):
    namespace = _namespace()
    namespace.update(command=command, package=package)
    replacements = {(int, 28): 29,
        (str, "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py"): REVISION_SOURCE,
        (str, "tools/e2e/dispatch_temporal_intrinsic_curve_pair_20261002.py"):
            "tools/e2e/dispatch_temporal_intrinsic_curve_pair_v2_20261002.py",
        (str, "exact externally reviewed runner/loss/all28 source pins required"):
            "exact externally reviewed V2 revision/loss/all29 source pins required"}
    with activated():
        return _clone("main", namespace, replacements)(argv)


if __name__ == "__main__":
    raise SystemExit(main())
