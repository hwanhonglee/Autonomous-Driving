"""HH_261002 - Preserve exact pre-rename JPEG literals; change no frozen physical, source, or publication verdict."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
from pathlib import Path

from . import publish_temporal_intrinsic_town03_right_20261002 as previous

ROOT = previous.ROOT
FROZEN_SHA = "875c835ac2a685b90b8b24533a03c329d465ccfc90e1d12f7be74e70de670acb"
BASE_SHA = "ab179034c9bd3e73f0776245695e0edcaacb9e316d2c580102786862a73bf7d8"
_verify, _sources = previous.verify_pair_and_recording, previous.publication_sources
_active = False


def verify_frozen():
    for name, expected in (("publish_temporal_intrinsic_town03_right_20261002.py", FROZEN_SHA),
        ("publish_temporal_route_joint_town03_right_20261002.py", BASE_SHA)):
        previous.previous.require(previous.previous.prior.digest(ROOT/"tools/e2e"/name) == expected,
            "both exact frozen complete-publication readers required")


def written_files_match(written, episode, images):
    # HH_261002 - Compare whole literal sets under ONE exact alias; no basename, mixed alias, source rewrite, or foreign run.
    audit = previous.previous
    episode = audit.canonical(episode)
    audit.require(episode.name in ("episode", "episode.partial") and isinstance(written, list)
        and len(written) == 6 and all(type(name) is str for name in written), "six exact original JPEG literals required")
    paths = [str(audit.canonical(name)) for name in written]
    audit.require(len(set(paths)) == 6, "six distinct original literal images required")
    relatives = [audit.prior.safe_image(episode, name).relative_to(episode) for name in images.values()]
    audit.require(len(relatives) == len(set(relatives)) == 6, "all six current pinned source paths required")
    return any(set(paths) == {str(episode.parent/alias/name) for name in relatives}
        for alias in ("episode", "episode.partial"))


def verification_function():
    verify_frozen()
    source = ROOT/"tools/e2e/publish_temporal_intrinsic_town03_right_20261002.py"
    original = next(node for node in ast.parse(source.read_bytes()).body
        if isinstance(node, ast.FunctionDef) and node.name == "verify_pair_and_recording")
    changed = copy.deepcopy(original); count = 0
    anchor = "set(row['written_files']) == expected_files"
    replacement = "written_files_match(row['written_files'], bundle['episode'], camera['images'])"
    for node in ast.walk(changed):
        if isinstance(node, ast.Compare) and ast.unparse(node) == anchor:
            node.__class__ = ast.Call
            node.__dict__.clear(); node.__dict__.update(ast.parse(replacement, mode="eval").body.__dict__)
            count += 1
    previous.previous.require(count == 1, "one exact literal-set comparison required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Call) and ast.unparse(node) == replacement:
            node.__class__ = ast.Compare
            node.__dict__.clear(); node.__dict__.update(ast.parse(anchor, mode="eval").body.__dict__)
    previous.previous.require(ast.dump(restored) == ast.dump(original), "another pair/ACK/source/denial guard changed")
    namespace = dict(vars(previous), written_files_match=written_files_match)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["verify_pair_and_recording"]


def publication_sources():
    return previous.previous.core.audit.source_closure(Path(__file__), ROOT/"tools/workspace_python.py", ROOT/"tools/canonical_workspace.py")


@contextmanager
def activated():
    global _active
    previous.previous.require(not _active and not previous._active and previous.verify_pair_and_recording is _verify
        and previous.publication_sources is _sources, "exclusive exact frozen publication bindings required")
    replacement = verification_function(); _active = True
    try:
        previous.verify_pair_and_recording, previous.publication_sources = replacement, publication_sources
        yield
    finally:
        previous.verify_pair_and_recording, previous.publication_sources = _verify, _sources
        _active = False


def main(argv=None):
    with activated(): return previous.main(argv)


if __name__ == "__main__": raise SystemExit(main())
