"""HH_261006 - Prepare recoverable cleanup of exact own inactive ccache M/R files; keep every stats/tag/directory."""
from __future__ import annotations

import ast
from contextlib import contextmanager
import fcntl
import importlib.util
import inspect
import os
from pathlib import Path
import stat
from types import FunctionType

ROOT = Path(__file__).resolve().parents[2]
ORIGINAL = ROOT / "tools/maintenance/recoverable_autonomy_build_objects_v2_20261005.py"
spec = importlib.util.spec_from_file_location("_frozen_object_cleanup_v2_for_cache", ORIGINAL)
known = importlib.util.module_from_spec(spec)
spec.loader.exec_module(known)
base = known.base
CACHE = Path("/var/tmp/ccache")
OUT = ROOT / "storage/experiments/maintenance/2026-10-06/ccache_mr_v1"
REMOTE = base.REMOTE_PERSONAL / "portable_e2e/archives/ccache_mr_20261006_v1"
TEST = ROOT / "learning/tests/test_recoverable_ccache_mr_v1_20261006.py"
SCHEMA = "portable_e2e.recoverable_exact_ccache_M_R.20261006.v1"
AUTH_SCHEMA = "portable_e2e.root_exact_ccache_M_R_authority.20261006.v1"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
FROZEN_PINS = dict(known.PINS)
FROZEN_PINS.update({ORIGINAL: "4e7933b02717eeb52a17ce8f68bee1b8738567d55fd65744766b5202d0ac46f3",
    known.TEST: "2dd2650e0d5235287dd29cd5699f288c47b8691e15e26ea2dc34adaa0cb6b4a8"})
ARCHIVE_SOURCES = (*FROZEN_PINS, Path(__file__).resolve(), TEST)
require, canonical, stamp, directory_identity = base.require, base.canonical, base.stamp, base.directory_identity


def scoped(relative):
    # HH_261006 - Exact current ccache layout only; suffix alone cannot authorize an arbitrary file or .o.
    path = Path(relative)
    parts = path.parts
    return (not path.is_absolute() and path.as_posix() == relative and len(parts) == 3
        and all(len(p) == 1 and p in "0123456789abcdef" for p in parts[:2])
        and len(parts[2]) == 32 and parts[2][-1] in "MR"
        and all(c in "0123456789abcdefghijklmnopqrstuvwxyz" for c in parts[2][:-1]))


def kept(relative):
    parts = Path(relative).parts
    return (Path(relative).as_posix() == relative and
        ((len(parts) == 2 and parts[0] in "0123456789abcdef" and len(parts[0]) == 1 and parts[1] == "CACHEDIR.TAG")
         or (len(parts) in (2, 3) and parts[-1] == "stats" and all(
             len(p) == 1 and p in "0123456789abcdef" for p in parts[:-1]))))


def sources():
    require(all(base.digest(path) == pin for path, pin in FROZEN_PINS.items()), "frozen .o source/test dependency changed")
    return {str(path.relative_to(ROOT)): base.digest(path) for path in ARCHIVE_SOURCES}


def private_scope():
    # HH_261006 - Rebind only private function globals; original modules, defaults and frozen artifacts are never mutated.
    namespace = dict(vars(base), BUILD=CACHE, OUT=OUT, REMOTE=REMOTE, SCHEMA=SCHEMA,
        AUTH_SCHEMA=AUTH_SCHEMA, TEST=TEST, __file__=str(Path(__file__).resolve()), scoped=scoped,
        ARCHIVE_SOURCES=ARCHIVE_SOURCES, sources=sources,
        exact_system_service=known.exact_system_service,
        BUILD_TOOLS=base.BUILD_TOOLS | {"ccache", "sccache", "distcc"})
    for name in ("opened_object", "verify_object_before_unlink", "process_audit", "remote_program"):
        original = getattr(base, name)
        namespace[name] = FunctionType(original.__code__, namespace, name,
            original.__defaults__, original.__closure__)
        namespace[name].__kwdefaults__ = original.__kwdefaults__
    return namespace


def census(root=CACHE, *, hashing=True):
    """HH_261006 - Visit all entries, hash every approved file, and separately pin every retained regular file."""
    root = canonical(root)
    require(root.is_dir() and root.stat().st_uid == base.UID, "exact owned cache directory required")
    directories, objects, retained = {}, {}, {}
    opened = private_scope()["opened_object"]
    def walk(folder):
        initial = stamp(folder.lstat())
        require(stat.S_ISDIR(initial["mode"]) and initial["uid"] == base.UID, "owned regular cache directory required")
        key = str(folder.relative_to(root))
        require(key == "." or len(Path(key).parts) <= 2 and all(
            len(p) == 1 and p in "0123456789abcdef" for p in Path(key).parts), "unlisted cache directory")
        directories[key] = directory_identity(folder.lstat())
        for entry in sorted(os.scandir(folder), key=lambda item: item.name):
            path = Path(entry.path); relative = str(path.relative_to(root)); st = entry.stat(follow_symlinks=False)
            require(not stat.S_ISLNK(st.st_mode), "cache symlink denied")
            if stat.S_ISDIR(st.st_mode):
                walk(path); continue
            require(stat.S_ISREG(st.st_mode) and st.st_uid == base.UID and st.st_nlink == 1,
                "owned single-link regular cache file required")
            require(scoped(relative) or kept(relative), "unknown cache file denied: " + relative)
            generation = stamp(st)
            if scoped(relative):
                parent, fd = opened(root, relative, directories)
                try:
                    require(stamp(os.fstat(fd)) == generation, "cache file changed before hashing")
                    value = base.fd_digest(fd) if hashing else None
                    require(stamp(os.fstat(fd)) == generation and stamp(path.lstat()) == generation,
                        "cache file changed during hashing")
                finally:
                    os.close(fd); os.close(parent)
                objects[relative] = dict(generation=generation, sha256=value)
            else:
                value = base.digest(path)
                require(stamp(path.lstat()) == generation, "retained stats/tag changed")
                retained[relative] = dict(generation=generation, sha256=value)
        require(stamp(folder.lstat()) == initial, "cache directory changed during census")
    walk(root)
    return dict(build_root=str(root), objects=objects, directories=directories, retained=retained,
        excluded_symlinks=[], object_count=len(objects),
        logical_bytes=sum(v["generation"]["size"] for v in objects.values()),
        allocated_bytes=sum(v["generation"]["blocks"] * 512 for v in objects.values()),
        objects_sha256=base.sha(base.encoded(objects)))


def preserved_audit(manifest):
    # HH_261006 - Owned unlink changes directory timestamps; keep directory identity and all stats/tag bytes exact.
    root = canonical(manifest["build_root"])
    current = census(root, hashing=False)
    require(current["directories"] == manifest["directories"] and current["retained"] == manifest["retained"],
        "retained cache directories/stats/tags changed")
    require(set(current["objects"]) <= set(manifest["objects"]) and all(
        row["generation"] == manifest["objects"][name]["generation"] for name, row in current["objects"].items()),
        "new or modified cache consumer output denied")
    rows = dict(directories=current["directories"], retained=current["retained"])
    return dict(rows_sha256=base.sha(base.encoded(rows)), no_unknown_or_object_reference=True,
        entry_count=len(current["objects"]) + len(current["retained"]) + len(current["directories"]) - 1,
        symlink_count=0, dangling_symlink_count=0, hits=[], unknown=[],
        preservation_scope="all cache directories and exact stats/CACHEDIR.TAG; installations/venvs/data/results untouched")


def checked_authority(args, inspection):
    require(canonical(args.authority) == OUT / ("ROOT_" + args.mode + "_authority.json"),
        "fixed fresh cache ROOT authority path required")
    value = base.read(args.authority, args.authority_sha256)
    require(value.get("schema") == AUTH_SCHEMA and value.get("mode") == args.mode
        and value.get("approved_cache_root") == str(CACHE) and value.get("remote_archive") == str(REMOTE)
        and value.get("inspection_sha256") == args.inspection_sha256
        and value.get("objects_sha256") == inspection["manifest"]["objects_sha256"]
        and value.get("source_sha256") == sources()
        and value.get("explicit_user_approved_exact_ccache_M_R_scope") is True
        and value.get("keep_all_stats_tags_directories_and_other_artifacts") is True
        and value.get("known_system_services_identified_but_privileged_FD_proof_not_claimed") is True,
        "cache-specific externally pinned ROOT authority required; old .o approval is not cache authority")
    if args.mode == "remove":
        require(value.get("backup_receipt_sha256") == args.backup_receipt_sha256
            and value.get("fail_closed_unknown_process_and_preservation_risk") is True,
            "backup-bound fresh cache removal authority required")
    return value


def prepared_main():
    namespace = private_scope()
    original_process = namespace["process_audit"]
    def process():
        return original_process(CACHE)
    namespace.update(inventory=census, install_audit=preserved_audit,
        checked_authority=checked_authority, process_audit=process)
    # HH_261006 - Preserve the genuine complete backup/remote verification/per-file intent/unlink/failure lifecycle.
    original = inspect.getsource(base.main)
    old = "for p in (Path(__file__).resolve(), TEST)"
    require(original.count(old) == 1, "original source archive seam changed")
    changed = original.replace(old, "for p in ARCHIVE_SOURCES")
    require(ast.dump(ast.parse(changed.replace("for p in ARCHIVE_SOURCES", old))) == ast.dump(ast.parse(original)),
        "another original lifecycle statement changed")
    exec(compile(changed, str(Path(__file__)) + ":private_original_main", "exec"), namespace)
    return namespace["main"]


@contextmanager
def serialized_runtime():
    # HH_261006 - Admission and every future real stage hold the existing local runtime lease without owner takeover.
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


def main(argv=None):
    sources()
    require(canonical(CACHE) == CACHE and os.getuid() == base.UID, "fixed cache root and UID required")
    with serialized_runtime():
        return prepared_main()(argv)


if __name__ == "__main__":
    raise SystemExit(main())
