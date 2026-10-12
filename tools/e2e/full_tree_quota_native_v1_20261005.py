"""HH_261005 - Prepare a source-sealed native full census, never activate it implicitly in a frozen capture."""

from __future__ import annotations

import ctypes
import hashlib
import os
from pathlib import Path
import stat
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path(__file__).with_suffix(".c")
SCHEMA = "portable_e2e.complete_native_fd_quota.20261005.v1"
# HH_261005 - Keep admitted dlopen descriptor names unique for process lifetime; libc must never reuse a cached /proc/fd name for another engine.
_ADMITTED_LIBRARY_DESCRIPTORS = []


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def compile_new(output, *, expected_source_sha256):
    """HH_261005 - Compile only the admitted C source into a fresh private artifact; no package or system installation."""
    output = Path(output)
    require(SOURCE.resolve() == SOURCE and not SOURCE.is_symlink() and digest(SOURCE) == expected_source_sha256,
            "native census C source seal differs")
    require(output.is_absolute() and output.resolve() == output and not output.exists() and not output.is_symlink(),
            "fresh real native quota output required")
    output.parent.mkdir(parents=True, exist_ok=True)
    # HH_261005 - gcc writes inside an atomically reserved mode0700 directory; hardlink publication cannot replace any target.
    build_directory = Path(tempfile.mkdtemp(prefix="HH_native_quota_build_", dir=output.parent))
    compiled = build_directory / "compiled.so"
    command = ["/usr/bin/gcc", "-std=c11", "-O2", "-fPIC", "-shared", "-Wall", "-Wextra", "-Werror",
               "-Wl,-z,relro,-z,now", str(SOURCE), "-o", str(compiled)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    require(digest(SOURCE) == expected_source_sha256 and compiled.is_file() and not compiled.is_symlink(),
            "source or native artifact changed during compilation")
    os.link(compiled, output, follow_symlinks=False)
    return dict(schema=SCHEMA + ".build", command=command, source_sha256=expected_source_sha256,
                library_sha256=digest(output), library=str(output), preserved_private_build=str(build_directory),
                atomic_no_replace_publication=True, install_performed=False)


class Counts(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint64) for name in ("bytes", "entries", "directories", "regular", "special")]


class NativeFullCensus:
    def __init__(self, library, *, expected_source_sha256, expected_library_sha256):
        """HH_261005 - Admit an immutable owned shared library before capture; no compilation or source reads inside the timer."""
        library = Path(library)
        require(SOURCE.resolve() == SOURCE and SOURCE.is_file() and not SOURCE.is_symlink(),
                "native quota C source must remain a canonical regular file")
        require(library.is_absolute() and library.resolve() == library and library.is_file() and not library.is_symlink(),
                "real native quota library required")
        require(digest(SOURCE) == expected_source_sha256 and digest(library) == expected_library_sha256,
                "native quota source/artifact seal differs")
        self.library, self.source_sha256, self.library_sha256 = library, expected_source_sha256, expected_library_sha256
        # HH_261005 - Load the already-hashed no-follow file descriptor, rejecting path/file generation changes around admission.
        descriptor = os.open(library, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        admitted = False
        try:
            before = os.fstat(descriptor)
            sealed = hashlib.sha256()
            for block in iter(lambda: os.read(descriptor, 1024 * 1024), b""):
                sealed.update(block)
            require(stat.S_ISREG(before.st_mode) and sealed.hexdigest() == expected_library_sha256,
                    "loaded native library descriptor seal differs")
            self._library = ctypes.CDLL("/proc/self/fd/" + str(descriptor))
            after, path_after = os.fstat(descriptor), library.lstat()
            signature = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)
            require(signature(before) == signature(after) == signature(path_after), "loaded native library generation changed")
            _ADMITTED_LIBRARY_DESCRIPTORS.append(descriptor)
            admitted = True
        finally:
            if not admitted:
                os.close(descriptor)
        self._call = self._library.hh_full_tree_quota_v1
        self._call.argtypes, self._call.restype = [ctypes.c_char_p, ctypes.POINTER(Counts)], ctypes.c_int

    def directory_bytes(self, directory, *, receipt=None):
        """HH_261005 - Inspect every directory entry on each call; preserve original logical-byte/special-node semantics."""
        root = Path(directory)
        require(root.is_absolute() and root.resolve() == root, "native quota root contains a symlink or relative path")
        try:
            metadata = root.lstat()
        except FileNotFoundError:
            return 0
        require(stat.S_ISDIR(metadata.st_mode) and not stat.S_ISLNK(metadata.st_mode), "real quota directory required")
        value = Counts()
        status = self._call(os.fsencode(root), ctypes.byref(value))
        if status:
            raise OSError(-status, "complete native quota traversal rejected", str(root))
        # HH_261005 - Also bind the path after the complete fd walk, rejecting a renamed or substituted root.
        after = root.lstat()
        require((after.st_dev, after.st_ino, after.st_mtime_ns, after.st_ctime_ns)
                == (metadata.st_dev, metadata.st_ino, metadata.st_mtime_ns, metadata.st_ctime_ns)
                and stat.S_ISDIR(after.st_mode) and not stat.S_ISLNK(after.st_mode),
                "quota root generation changed during complete traversal")
        if receipt is not None:
            receipt.update(complete_walk=True, entries=value.entries, directories_including_root=value.directories,
                           regular_files=value.regular, special_nodes_ignored_like_original=value.special,
                           regular_file_bytes=value.bytes, cached_or_incremental_byte_count_used=False,
                           source_payload_read=False, native_census_schema=SCHEMA,
                           native_census_source_sha256=self.source_sha256, native_census_library_sha256=self.library_sha256,
                           directory_generation_postcheck_required=True)
        return value.bytes
