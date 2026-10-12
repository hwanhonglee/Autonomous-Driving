"""HH_261008 - Preserve exact JSONL/print bytes with independently verified gzip rows.

This source-only codec grants no execution authority. Its caller must own the
runtime lease and provide the prospective whole-output quota guard. Compression,
indexing and flushing occur at the original append/print call, never in a hidden
background worker. A failed row remains pending; a complete prefix is not PASS.
"""
from __future__ import annotations

import builtins
import gzip
import hashlib
import json
import math
from pathlib import Path
import stat
import zlib

FORMAT = "HH_261008.lossless_JSONL_gzip_members.v1"
RAW_ROW_LIMIT = 512 * 1024
INDEX_ROW_LIMIT = 768
SHARD_LIMIT = 16 * 1024**2
FILE_LIMIT = 32 * 1024**2
# HH_261008 - Conservative diagnostic caps exceed observed V11 gzip maxima; no generic compression-ratio guarantee is asserted.
COMPONENT_LIMITS = {
    "direct_anchors": (12 * 1024, 918),
    "subscriber_anchors": (24 * 1024, 918),
    "publisher_anchors": (24 * 1024, 918),
    "subscriber_stdout": (12 * 1024, 919),
    "publisher_stdout": (16 * 1024, 919),
    "subscriber_output_calls": (512, 1836),
    "publisher_received_outputs": (1024, 1836),
    "comparison": (512, 918),
}


def prospective_capacity(*, metadata_limit=10 * 1024**2, stderr_limit=3 * 1024**2):
    """HH_261008 - A prospective allocation, not proof that every possible 512KiB row compresses enough."""
    positive(metadata_limit, "whole plan/source/other metadata reserve")
    positive(stderr_limit, "all uncompressed stderr reserve")
    components = {name: dict(compressed_row_limit=cap, max_rows=count,
        logical_total_limit=(cap + INDEX_ROW_LIMIT) * count)
        for name, (cap, count) in COMPONENT_LIMITS.items()}
    total = sum(value["logical_total_limit"] for value in components.values()) + metadata_limit + stderr_limit
    require(total <= 112 * 1024**2, "prospective whole-output working quota does not fit")
    return dict(format=FORMAT, components=components, metadata_limit=metadata_limit,
        stderr_limit=stderr_limit, total_reserved_bytes=total,
        working_limit_bytes=112 * 1024**2, closing_reserve_bytes=16 * 1024**2,
        remaining_working_bytes=112 * 1024**2-total,
        rowcap_overflow_retains_pending_raw_in_closing_reserve=True,
        guaranteed_generic_compression_ratio=False, runtime_or_feature_PASS=False)


def require(value, message):
    if not value:
        raise ValueError(message)


def positive(value, name):
    require(type(value) is int and value > 0, name + " must be a positive integer")
    return value


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def canonical(path, *, exists=True):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(strict=exists)
        and not any(item.is_symlink() for item in (path, *path.parents)),
        "canonical non-symlink codec path required")
    return path


def decode_row(raw):
    # HH_261008 - Validate without re-encoding: original spaces, escapes and float spelling survive.
    require(type(raw) is bytes and 0 < len(raw) <= RAW_ROW_LIMIT
        and raw.endswith(b"\n") and raw.count(b"\n") == 1, "one bounded complete JSONL byte row required")
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key denied")
            result[key] = value
        return result
    def invalid(value):
        raise ValueError("nonfinite JSON token denied: " + value)
    value = json.loads(raw.decode("utf-8"), object_pairs_hook=unique, parse_constant=invalid)
    require(type(value) is dict, "JSONL record must be an object")
    def finite(item):
        if type(item) is float:
            require(math.isfinite(item), "nonfinite/overflow JSON number denied")
        elif type(item) is dict:
            for child in item.values(): finite(child)
        elif type(item) is list:
            for child in item: finite(child)
    finite(value)
    return value


def identity(value):
    result = {}
    for name in ("ordinal", "native_frame", "timestamp_ns"):
        if name in value:
            require(type(value[name]) is int and value[name] >= 0, "actual record identity must be an integer")
            result[name] = value[name]
    return result


def index_bytes(value):
    raw = (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    require(len(raw) <= INDEX_ROW_LIMIT, "complete codec index row exceeds bound")
    return raw


def regular(path):
    path = canonical(path)
    info = path.lstat()
    require(stat.S_ISREG(info.st_mode) and info.st_nlink == 1, "single-link regular codec file required")
    return info


def decompress_member(member):
    require(type(member) is bytes and len(member) > 0, "actual gzip member bytes required")
    reader = zlib.decompressobj(16 + zlib.MAX_WBITS)
    raw = reader.decompress(member, RAW_ROW_LIMIT + 1)
    require(len(raw) <= RAW_ROW_LIMIT and reader.eof and not reader.unused_data
        and not reader.unconsumed_tail, "truncated, concatenated or oversized gzip member denied")
    decode_row(raw)
    return raw


class LosslessJSONLLedger:
    """HH_261008 - One owned logical ledger, with bounded shards and one index row per actual row."""
    def __init__(self, root, *, compressed_row_limit, logical_total_limit, max_rows,
                 guard, shard_limit=SHARD_LIMIT):
        self.root = canonical(root, exists=False)
        require(self.root.parent.is_dir() and not self.root.exists(), "fresh owned ledger directory required")
        self.row_limit = positive(compressed_row_limit, "compressed row limit")
        self.total_limit = positive(logical_total_limit, "logical ledger limit")
        self.max_rows = positive(max_rows, "row count")
        self.shard_limit = positive(shard_limit, "shard limit")
        require(self.row_limit <= self.shard_limit <= SHARD_LIMIT < FILE_LIMIT
            and self.max_rows * INDEX_ROW_LIMIT <= FILE_LIMIT and callable(guard),
            "bounded shards/index and external whole-output guard required")
        self.guard = guard
        self.count = self.total_bytes = self.raw_bytes = 0
        self.shard_number = -1
        self.shard_bytes = 0
        self._shard = None
        self._pending = None
        self._print_stream = None
        self.failure = None
        self.closed = False
        self._owned = {}
        self.guard(0)
        self.root.mkdir()
        info = self.root.lstat()
        self._root_identity = (info.st_dev, info.st_ino, info.st_uid)
        self._index = (self.root / "index.jsonl").open("xb")
        self._remember(self.root / "index.jsonl", 0)

    def _remember(self, path, size):
        info = regular(path)
        require(info.st_size == size, "owned codec file size changed")
        previous = self._owned.get(path.name)
        current = (info.st_dev, info.st_ino, info.st_uid, size, info.st_mtime_ns, info.st_ctime_ns)
        require(previous is None or previous[:3] == current[:3], "owned codec inode generation changed")
        self._owned[path.name] = current

    def _check_owned(self):
        require(canonical(self.root).is_dir(), "owned codec directory changed")
        info = self.root.lstat()
        require((info.st_dev, info.st_ino, info.st_uid) == self._root_identity, "owned ledger directory generation changed")
        require({path.name for path in self.root.iterdir()} == set(self._owned), "unexpected ledger file denied")
        for name, expected in self._owned.items():
            info = regular(self.root / name)
            require((info.st_dev, info.st_ino, info.st_uid, info.st_size, info.st_mtime_ns, info.st_ctime_ns) == expected,
                "owned codec file generation/size/mtime/ctime changed")

    @property
    def pending_raw(self):
        # HH_261008 - The caller may retain this exact failure row in its declared closing reserve.
        return self._pending if self._pending is not None else (
            self._print_stream.pending if self._print_stream is not None and self._print_stream.pending else None)

    def append(self, raw):
        require(not self.closed and self.failure is None and self._pending is None,
            "closed/failed codec cannot accept another row or erase its failure")
        require(self._print_stream is None or not self._print_stream.pending
            or raw is self._print_stream.pending, "prior print fragment must not cross into another anchor")
        self._pending = raw if type(raw) is bytes and len(raw) <= RAW_ROW_LIMIT else None
        stage = "validate_raw"
        try:
            value = decode_row(raw)
            require(self.count < self.max_rows, "declared logical row denominator exceeded")
            self._check_owned()
            stage = "compress"
            member = gzip.compress(raw, compresslevel=6, mtime=0)
            require(len(member) <= self.row_limit, "actual lossless compressed row exceeds prospective cap")
            rotate = self._shard is None or self.shard_bytes + len(member) > self.shard_limit
            number = self.shard_number + int(rotate)
            name = "rows_%06d.jsonl.gz" % number
            offset = 0 if rotate else self.shard_bytes
            entry = dict(format=FORMAT, row=self.count, shard=name, offset=offset,
                compressed_bytes=len(member), compressed_sha256=sha(member), raw_bytes=len(raw),
                raw_sha256=sha(raw), identity=identity(value))
            encoded = index_bytes(entry)
            added = len(member) + len(encoded)
            require(self.total_bytes + added <= self.total_limit, "logical compressed ledger quota exhausted")
            stage = "whole_output_guard"
            self.guard(added)
            if rotate:
                if self._shard is not None: self._shard.close()
                self._shard = (self.root / name).open("xb")
                self.shard_number = number
                self.shard_bytes = 0
                self._remember(self.root / name, 0)
            stage = "write_gzip_member"
            require(self._shard.write(member) == len(member), "partial gzip member write")
            self._shard.flush()
            stage = "write_index"
            require(self._index.write(encoded) == len(encoded), "partial index write")
            self._index.flush()
            self.shard_bytes += len(member)
            self.total_bytes += added
            self.raw_bytes += len(raw)
            self.count += 1
            self._remember(self.root / name, self.shard_bytes)
            self._remember(self.root / "index.jsonl", self._index.tell())
            self._pending = None
            return entry
        except BaseException as error:
            self.failure = dict(stage=stage, error=repr(error), accepted_rows=self.count,
                failed_raw_bytes=len(raw) if type(raw) is bytes else None,
                failed_raw_sha256=sha(raw) if self._pending is not None else None,
                pending_raw_retained=self._pending is not None,
                oversized_input_not_retained=self._pending is None,
                complete_prefix_is_PASS=False)
            raise

    def print(self, *values, sep=" ", end="\n", file=None, flush=False):
        # HH_261008 - A private original-print delegate, never a global stdout/builtins patch.
        require(file is None and type(flush) is bool, "owned original print destination/flush ABI required")
        if self._print_stream is None: self._print_stream = OriginalPrintStream(self)
        require(not self._print_stream.pending and self.failure is None and not self.closed,
            "prior original print fragment/failure must be retained before another print")
        try:
            return builtins.print(*values, sep=sep, end=end, file=self._print_stream, flush=flush)
        except BaseException as error:
            if self.failure is None:
                raw = self.pending_raw
                self.failure = dict(stage="original_print_fragment", error=repr(error), accepted_rows=self.count,
                    failed_raw_bytes=len(raw) if raw is not None else None,
                    failed_raw_sha256=sha(raw) if raw is not None else None,
                    pending_raw_retained=raw is not None, complete_prefix_is_PASS=False)
            raise

    def close(self):
        if self.closed: return
        prior_failure = self.failure is not None
        incomplete = self._print_stream is not None and bool(self._print_stream.pending)
        if incomplete and not prior_failure:
            raw = self.pending_raw
            self.failure = dict(stage="closing_incomplete_original_print",
                error="actual incomplete print fragment retained without fabricated newline", accepted_rows=self.count,
                failed_raw_bytes=len(raw), failed_raw_sha256=sha(raw), pending_raw_retained=True,
                complete_prefix_is_PASS=False)
        errors = []
        for stream in (self._shard, self._index):
            if stream is not None:
                try: stream.close()
                except BaseException as error: errors.append(repr(error))
        self.closed = True
        if errors:
            if self.failure is None:
                self.failure = dict(stage="closing_streams", error=repr(errors), accepted_rows=self.count,
                    pending_raw_retained=self.pending_raw is not None, complete_prefix_is_PASS=False)
            self.failure["closing_errors"] = errors
        # HH_261008 - Never replace an earlier genuine append/print error; new close faults still deny a successful close.
        if not prior_failure:
            require(not incomplete and not errors, "codec close failed: " + repr(self.failure))


class OriginalPrintStream:
    """HH_261008 - Preserve genuine builtin write fragments; never invent a final newline."""
    def __init__(self, ledger):
        self.ledger = ledger
        self.pending = b""

    def write(self, text):
        require(type(text) is str and self.ledger.failure is None and not self.ledger.closed,
            "live owned original text write required")
        raw = text.encode("utf-8")
        require(len(self.pending) + len(raw) <= RAW_ROW_LIMIT, "bounded original print fragment required")
        self.pending += raw
        if b"\n" in self.pending:
            # HH_261008 - Original one-row print calls are preserved; multiline/split-record attempts fail with their bytes retained.
            self.ledger.append(self.pending)
            self.pending = b""
        return len(text)

    def flush(self):
        require(not self.pending, "incomplete actual print fragment retained; no fabricated newline")
        for stream in (self.ledger._shard, self.ledger._index):
            if stream is not None: stream.flush()


def iter_rows(root, *, compressed_row_limit, logical_total_limit, max_rows):
    """HH_261008 - Consume the entire iterator to verify the complete index/shard census.

    An interrupted write is rejected, not repaired or re-labeled. The caller must
    preserve that raw/index/shard failure tail and distinguish it from a complete
    accepted prefix. No decoder invents a missing anchor.
    """
    root = canonical(root)
    root_info = root.lstat()
    require(stat.S_ISDIR(root_info.st_mode), "codec ledger directory required")
    positive(compressed_row_limit, "compressed row limit")
    positive(logical_total_limit, "logical ledger limit")
    positive(max_rows, "row count")
    paths = {path.name: path for path in root.iterdir()}
    require("index.jsonl" in paths and len(paths) <= max_rows + 1, "bounded ledger census required")
    before = {name: regular(path) for name, path in paths.items()}
    require(sum(info.st_size for info in before.values()) <= logical_total_limit
        and all(info.st_size <= FILE_LIMIT for info in before.values())
        and before["index.jsonl"].st_size <= max_rows * INDEX_ROW_LIMIT, "whole ledger size bound exceeded")
    expected_offsets = {}
    count = 0
    with paths["index.jsonl"].open("rb") as stream:
        while True:
            encoded = stream.readline(INDEX_ROW_LIMIT + 1)
            if not encoded: break
            require(len(encoded) <= INDEX_ROW_LIMIT and count < max_rows, "bounded complete index rows required")
            entry = decode_row(encoded)
            keys = {"format", "row", "shard", "offset", "compressed_bytes", "compressed_sha256",
                "raw_bytes", "raw_sha256", "identity"}
            require(set(entry) == keys and entry["format"] == FORMAT and type(entry["row"]) is int
                and entry["row"] == count, "exact ordered codec index required")
            name = entry["shard"]
            require(type(name) is str and name == "rows_%06d.jsonl.gz" % (len(expected_offsets)-1 if name in expected_offsets else len(expected_offsets))
                and name in paths, "sequential owned gzip shard required")
            offset = expected_offsets.get(name, 0)
            length = positive(entry["compressed_bytes"], "compressed member bytes")
            require(type(entry["offset"]) is int and entry["offset"] == offset
                and length <= compressed_row_limit and offset + length <= SHARD_LIMIT,
                "contiguous bounded gzip member required")
            with paths[name].open("rb") as member_stream:
                member_stream.seek(offset)
                member = member_stream.read(length)
            require(len(member) == length and sha(member) == entry["compressed_sha256"], "actual compressed member hash differs")
            raw = decompress_member(member)
            require(type(entry["identity"]) is dict and set(entry["identity"]) <= {"ordinal", "native_frame", "timestamp_ns"}
                and identity(entry["identity"]) == entry["identity"], "exact recorded index identity types required")
            require(type(entry["raw_bytes"]) is int and len(raw) == entry["raw_bytes"]
                and sha(raw) == entry["raw_sha256"] and identity(decode_row(raw)) == entry["identity"],
                "exact original byte hash/record identity differs")
            expected_offsets[name] = offset + length
            count += 1
            yield raw
    require(set(paths) == {"index.jsonl", *expected_offsets}, "unexpected/unindexed gzip tail retained, not accepted")
    actual_root = root.lstat()
    require({path.name for path in root.iterdir()} == set(paths)
        and (actual_root.st_dev, actual_root.st_ino, actual_root.st_uid)
        == (root_info.st_dev, root_info.st_ino, root_info.st_uid), "ledger directory/census changed during verification")
    for name, info in before.items():
        actual = regular(paths[name])
        require((actual.st_dev, actual.st_ino, actual.st_size, actual.st_mtime_ns, actual.st_ctime_ns)
            == (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns),
            "codec file changed during verification")
        if name != "index.jsonl": require(actual.st_size == expected_offsets[name], "unindexed gzip bytes retained, not accepted")


def verify_ledger(root, **limits):
    digest = hashlib.sha256()
    count = total = 0
    for raw in iter_rows(root, **limits):
        digest.update(raw)
        count += 1
        total += len(raw)
    return dict(format=FORMAT, rows=count, original_bytes=total, original_stream_sha256=digest.hexdigest(),
        exact_original_bytes_reconstructed=True, NN_inferences=0, training_steps=0,
        independent_runtime_performance_claimed=False)
