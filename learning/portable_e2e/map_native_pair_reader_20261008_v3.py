"""HH_261008 - V3 bounded, nontraining native TRAIN RGB/GT reads from sealed NPZs.

The outer owner MUST establish fresh data admission, ownership, deadlines and
source/provider seals before construction. Expected SHA strings authenticate
bytes, not authority, split provenance, consent, or permission to train. This
module has no CLI, trainer, registry writer, model import or authority grant.

Only original729 rgb/target and paired480 rgb_false/target are decoded. TRUE is
censused as ZIP metadata and covered by the compressed-file SHA, never inflated.
Returned arrays own exactly one native RGB/GT slice. Reader scratch is bounded
by 64KiB blocks and small headers; caller-retained arrays are outside that bound.
STORED seeks are direct. DEFLATED reads use lazy in-process state checkpoints;
bounded memory is NOT a throughput claim or a claim that only selected pixels
were decoded. Partial member reads do not verify full-member CRC/EOF/trailers.
Closing checks path/FD generations; the owner must separately perform any
required closing whole-file SHA. Generation checks are observations, not an OS
immutability guarantee against a privileged concurrent writer.

Construction hashes compressed bytes and checks bounded ZIP metadata only.
NPY header validation is deferred to the first query, before array allocation;
there is no constructor decode or warmup. Checkpoints hold no native row arrays
and are never serialized. Fixed 256KiB accounting per state reserves128KiB for
native decoder/metadata and two64KiB compressed-tail buffers, at most48 states
(12MiB accounted cache) plus the original1MiB working scratch. This reservation
is conservative accounting, NOT byte-exact native allocation measurement or OS
containment. The outer owner must enforce its whole-process memory/time guards.
The optional owner callback runs at16 generation-guard observations and at
mandatory constructor/read/close boundaries. Generation checks themselves are
never sampled away. Cache0 and cache12MiB use this same code and cadence; fewer
inflated blocks can reduce observations. Counters report attempted callbacks.
"""

import ast
from collections import OrderedDict
import hashlib
import os
from pathlib import Path
import stat
import struct
import threading
import zlib


ORIGINAL_COUNT, PAIRED_COUNT, TOTAL = 729, 480, 1209
HEIGHT, WIDTH, BLOCK = 360, 640, 64 * 1024
HEADER_LIMIT, CENTRAL_LIMIT, ARCHIVE_LIMIT = 256, 4096, 1024 ** 3
SCRATCH_LIMIT_BYTES = 1024 ** 2
NATIVE_RGB_BYTES, NATIVE_GT_BYTES = HEIGHT * WIDTH * 3, HEIGHT * WIDTH
# HH_261008 - Fixed decoder reservations plus two bounded compressed-tail copies; no native row cache.
CHECKPOINT_MAX_STATES = 48
CHECKPOINT_STATE_ACCOUNT_BYTES = 256 * 1024
CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES = 12 * 1024 ** 2
CHECKPOINT_STRIDE_BYTES = 32 * 1024 ** 2
OWNER_CHECKPOINT_GUARD_STRIDE = 16


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def _canonical(path):
    path = Path(path)
    _require(path.is_absolute() and ".." not in path.parts and path == path.resolve(strict=True)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical nonsymlink file required")
    result = path.lstat()
    _require(stat.S_ISREG(result.st_mode) and result.st_nlink == 1, "regular single-link file required")
    return path, result


def _generation(result):
    return (result.st_dev, result.st_ino, result.st_size, result.st_mtime_ns,
        result.st_ctime_ns, result.st_nlink, result.st_mode)


def _pread(fd, size, offset):
    _require(type(size) is int and 0 <= size <= BLOCK and type(offset) is int and offset >= 0,
        "bounded nonnegative positional read required")
    raw = os.pread(fd, size, offset)
    _require(len(raw) == size, "truncated bounded positional read")
    return raw


def _npy_header(stream, expected_shape):
    prefix = stream.read_exact(10)
    _require(prefix[:8] == b"\x93NUMPY\x01\x00", "exact NPYv1 required")
    size = int.from_bytes(prefix[8:], "little")
    _require(0 < size <= HEADER_LIMIT, "bounded NPY header required")
    raw = stream.read_exact(size)
    _require(raw.endswith(b"\n") and (10 + size) % 16 == 0, "NPY newline/alignment required")
    try:
        node = ast.parse(raw.decode("latin1"), mode="eval").body
        _require(isinstance(node, ast.Dict) and len(node.keys) == 3
            and all(isinstance(k, ast.Constant) and type(k.value) is str for k in node.keys)
            and len({k.value for k in node.keys}) == 3, "duplicate/malformed NPY keys denied")
        value = ast.literal_eval(node)
    except (SyntaxError, TypeError, UnicodeError, RecursionError) as error:
        raise ValueError("malformed bounded NPY header") from error
    _require(type(value) is dict and set(value) == {"descr", "fortran_order", "shape"}
        and value["descr"] == "|u1" and value["fortran_order"] is False
        and type(value["shape"]) is tuple and all(type(n) is int and n > 0 for n in value["shape"])
        and value["shape"] == expected_shape, "exact native C-order uint8 shape required")
    return prefix + raw


def _member_generation(owner, entry):
    return (str(owner.path), owner.expected_sha, owner.fingerprint, owner.pid,
        entry["name"], entry["size"], entry["compressed"], entry["CRC"],
        entry["compression"], entry["data_offset"], entry["flags"], entry["needed"])


def _closing_error(scope, error):
    # HH_261008 - Copy bounded native exception scalars without invoking foreign str/repr hooks.
    args = BaseException.args.__get__(error, type(error))
    message = args[0][:512] if args and type(args[0]) is str else "<non-string exception argument>"
    return dict(scope=scope, type=type(error).__name__[:64], message=message)


class _CheckpointStore:
    """HH_261008 - One bounded process-local cache shared by the four approved members."""

    def __init__(self, *, enabled=True):
        _require(type(enabled) is bool, "checkpoint cache enable requires exact bool")
        _require(CHECKPOINT_MAX_STATES * CHECKPOINT_STATE_ACCOUNT_BYTES
            <= CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES <= 16 * 1024 ** 2,
            "fixed conservative checkpoint budget required")
        self.enabled, self.states = enabled, OrderedDict()
        self.maximum_states = self.hits = self.misses = self.creations = self.evictions = 0
        self.total_prefix = self.total_selected = 0
        self.owner_checkpoint_calls = 0
        self.archive_guard_calls = 0

    def clear(self):
        self.states.clear()

    def discard_archive(self, owner):
        for key in tuple(self.states):
            if key[0][:4] == (str(owner.path), owner.expected_sha, owner.fingerprint, owner.pid):
                del self.states[key]

    def save(self, stream):
        if (not self.enabled or stream.decoder is None or not stream.cache_enabled
            or stream.position >= stream.entry["size"] or stream.decoder.eof):
            return
        stream.owner.guard()
        _require(stream.entry["name"] in (stream.owner.rgb_name, "target.npy")
            and type(stream.pending) is bytes and len(stream.pending) <= BLOCK
            and not stream.decoder.unused_data and type(stream.crc) is int,
            "only bounded approved member decoder state may be cached")
        generation = _member_generation(stream.owner, stream.entry)
        key = (generation, stream.position)
        if key in self.states:
            self.states.move_to_end(key)
            return
        # Evict before copy allocation so persistent states never exceed the fixed count.
        if len(self.states) == CHECKPOINT_MAX_STATES:
            self.states.popitem(last=False)
            self.evictions += 1
        stream.owner.guard()
        copied = stream.decoder.copy()
        _require(len(copied.unconsumed_tail) <= BLOCK and not copied.unused_data,
            "bounded copied compressed tail required")
        state = dict(member_generation=generation, position=stream.position,
            compressed_position=stream.compressed_position,
            compressed_FD_offset=stream.entry["data_offset"] + stream.compressed_position,
            pending=stream.pending, decoder=copied, crc=stream.crc)
        self.states[key] = state
        self.creations += 1
        self.maximum_states = max(self.maximum_states, len(self.states))
        _require(len(self.states) <= CHECKPOINT_MAX_STATES
            and len(self.states) * CHECKPOINT_STATE_ACCOUNT_BYTES <= CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES,
            "checkpoint count/accounting cap exceeded")

    def restore(self, stream, target):
        if not self.enabled:
            self.misses += 1
            return False
        generation = _member_generation(stream.owner, stream.entry)
        candidates = [key for key in self.states if key[0] == generation and key[1] <= target]
        if not candidates or stream.decoder is None:
            self.misses += 1
            return False
        key = max(candidates, key=lambda value: value[1])
        state = self.states[key]
        stream.owner.guard()
        _require(state["member_generation"] == generation and type(state["position"]) is int
            and stream.position <= state["position"] <= target < stream.entry["size"]
            and type(state["compressed_position"]) is int
            and 0 <= state["compressed_position"] <= stream.entry["compressed"]
            and state["compressed_FD_offset"] == stream.entry["data_offset"] + state["compressed_position"]
            and type(state["pending"]) is bytes and len(state["pending"]) <= BLOCK
            and type(state["crc"]) is int and 0 <= state["crc"] <= 0xffffffff
            and len(state["decoder"].unconsumed_tail) <= BLOCK
            and state["pending"] == state["decoder"].unconsumed_tail
            and not state["decoder"].eof and not state["decoder"].unused_data,
            "checkpoint position/tail/CRC/member generation differs")
        stream.decoder = state["decoder"].copy()
        stream.pending = state["pending"]
        stream.position, stream.compressed_position, stream.crc = (
            state["position"], state["compressed_position"], state["crc"])
        stream.owner.guard()
        self.states.move_to_end(key)
        self.hits += 1
        return True

    def snapshot(self):
        return dict(current_checkpoint_states=len(self.states),
            maximum_checkpoint_states_observed=self.maximum_states,
            max_checkpoint_states=CHECKPOINT_MAX_STATES if self.enabled else 0,
            checkpoint_state_account_bytes=CHECKPOINT_STATE_ACCOUNT_BYTES,
            cache_accounted_bytes=len(self.states) * CHECKPOINT_STATE_ACCOUNT_BYTES,
            maximum_cache_accounted_bytes=self.maximum_states * CHECKPOINT_STATE_ACCOUNT_BYTES,
            cache_account_limit_bytes=CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES if self.enabled else 0,
            checkpoint_cache_enabled=self.enabled,
            owner_checkpoint_calls=self.owner_checkpoint_calls,
            archive_generation_guard_calls=self.archive_guard_calls,
            owner_checkpoint_guard_stride=OWNER_CHECKPOINT_GUARD_STRIDE,
            checkpoint_hits=self.hits, checkpoint_misses=self.misses,
            checkpoint_creations=self.creations, checkpoint_evictions=self.evictions,
            total_prefix_body_bytes_inflated=self.total_prefix,
            total_selected_payload_bytes=self.total_selected,
            native_row_cache_payload_bytes=0,
            state_accounting_is_reservation_not_exact_native_allocation=True,
            checkpoint_stride_bytes=CHECKPOINT_STRIDE_BYTES)


class _MemberStream:
    """HH_261008 - Cap both compressed input and raw-DEFLATE output, including skipped prefixes."""

    def __init__(self, owner, entry):
        self.owner, self.entry = owner, entry
        self.position, self.compressed_position, self.crc = 0, 0, 0
        self.pending = b""
        self.decoder = zlib.decompressobj(-15) if entry["compression"] == 8 else None
        self.whole_member_crc_verified = False
        self.cache_enabled, self.checkpoint_hit = False, False
        self.restored_position, self.prefix_body_bytes_inflated = 0, 0
        self.next_checkpoint_position = CHECKPOINT_STRIDE_BYTES
        self.active_body_kind = None

    def begin_cached_query(self, target):
        _require(self.position <= target < self.entry["size"], "checkpoint target must precede selected native slice")
        self.cache_enabled = self.decoder is not None and self.owner.cache.enabled
        if self.decoder is not None:
            self.checkpoint_hit = self.owner.cache.restore(self, target)
            if self.checkpoint_hit:
                self.restored_position = self.position
            self.next_checkpoint_position = ((self.position // CHECKPOINT_STRIDE_BYTES) + 1) * CHECKPOINT_STRIDE_BYTES

    def _read(self, size):
        _require(0 < size <= BLOCK and self.position + size <= self.entry["size"],
            "member read outside declared payload")
        if self.decoder is None:
            raw = _pread(self.owner.fd, size, self.entry["data_offset"] + self.position)
        else:
            while True:
                if not self.pending and self.compressed_position < self.entry["compressed"]:
                    self.owner.guard()
                    amount = min(BLOCK, self.entry["compressed"] - self.compressed_position)
                    self.pending = _pread(self.owner.fd, amount,
                        self.entry["data_offset"] + self.compressed_position)
                    self.compressed_position += amount
                _require(self.pending or not self.decoder.eof, "truncated DEFLATE member")
                try:
                    raw = self.decoder.decompress(self.pending, size)
                except zlib.error as error:
                    raise ValueError("invalid bounded DEFLATE member") from error
                self.pending = self.decoder.unconsumed_tail
                if raw:
                    break
                _require(not self.decoder.eof and (self.pending
                    or self.compressed_position < self.entry["compressed"]), "nonprogress/truncated DEFLATE member")
        _require(0 < len(raw) <= size, "nonprogress/oversized decoded block")
        self.position += len(raw)
        if self.crc is not None:
            self.crc = zlib.crc32(raw, self.crc)
        if self.active_body_kind == "prefix":
            self.prefix_body_bytes_inflated += len(raw)
            self.owner.cache.total_prefix += len(raw)
        elif self.active_body_kind == "selected":
            self.owner.cache.total_selected += len(raw)
        if self.cache_enabled and self.position >= self.next_checkpoint_position:
            self.owner.cache.save(self)
            self.next_checkpoint_position = self.position + CHECKPOINT_STRIDE_BYTES
        return raw

    def read_exact(self, size):
        _require(type(size) is int and 0 <= size <= BLOCK, "bounded exact member read required")
        result = bytearray(size)
        offset = 0
        while offset < size:
            raw = self._read(size - offset)
            result[offset:offset + len(raw)] = raw
            offset += len(raw)
        return bytes(result)

    def skip(self, size):
        _require(type(size) is int and 0 <= size <= self.entry["size"] - self.position,
            "bounded declared prefix skip required")
        if self.decoder is None and size:
            # Direct STORED skipping does not observe prefix bytes and cannot certify their CRC.
            self.position += size
            self.crc = None
            return
        remaining = size
        self.active_body_kind = "prefix"
        try:
            while remaining:
                self.owner.guard()
                remaining -= len(self._read(min(BLOCK, remaining)))
        finally:
            self.active_body_kind = None

    def fill(self, destination):
        view = memoryview(destination).cast("B")
        offset = 0
        self.active_body_kind = "selected"
        try:
            while offset < len(view):
                self.owner.guard()
                raw = self._read(min(BLOCK, len(view) - offset))
                view[offset:offset + len(raw)] = raw
                offset += len(raw)
        finally:
            self.active_body_kind = None
        if self.position == self.entry["size"]:
            self._finish()

    def _finish(self):
        if self.decoder is not None:
            # A final zero-output drain checks terminator and surplus without unbounded flush().
            while not self.decoder.eof:
                self.owner.guard()
                if not self.pending and self.compressed_position < self.entry["compressed"]:
                    self.owner.guard()
                    amount = min(BLOCK, self.entry["compressed"] - self.compressed_position)
                    self.pending = _pread(self.owner.fd, amount,
                        self.entry["data_offset"] + self.compressed_position)
                    self.compressed_position += amount
                try:
                    raw = self.decoder.decompress(self.pending, 1)
                except zlib.error as error:
                    raise ValueError("invalid DEFLATE terminator") from error
                self.pending = self.decoder.unconsumed_tail
                _require(not raw, "decoded bytes beyond declared NPY payload")
                _require(self.decoder.eof or self.pending
                    or self.compressed_position < self.entry["compressed"], "missing DEFLATE terminator")
            _require(not self.pending and not self.decoder.unused_data
                and self.compressed_position == self.entry["compressed"], "extra compressed member/trailer bytes denied")
        if self.crc is not None:
            _require(self.crc == self.entry["CRC"], "whole-member ZIP CRC differs")
            self.whole_member_crc_verified = True


class _NativeArchive:
    """HH_261008 - Retain one sealed FD; reject oversized ZIP metadata before allocation/inflation."""

    def __init__(self, path, expected_sha, count, rgb_name, expected_names, *, checkpoint=None, cache=None):
        _require(_sha(expected_sha), "explicit lowercase expected whole-file SHA256 required")
        _require(type(count) is int and 0 < count <= ORIGINAL_COUNT, "bounded native sample count required")
        _require((rgb_name == "rgb.npy" and expected_names == ("rgb.npy", "target.npy"))
            or (rgb_name == "rgb_false.npy" and expected_names == ("rgb_false.npy", "rgb_true.npy", "target.npy")),
            "only original or paired FALSE member contracts permitted")
        _require(checkpoint is None or callable(checkpoint), "optional owner checkpoint must be callable")
        _require(cache is None or type(cache) is _CheckpointStore, "private fixed checkpoint store required")
        self.checkpoint = checkpoint
        self.cache = cache if cache is not None else _CheckpointStore()
        self.closing_errors = []
        self.path, before = _canonical(path)
        _require(22 <= before.st_size <= ARCHIVE_LIMIT, "archive size bound denied")
        self.expected_sha, self.fingerprint, self.pid = expected_sha, _generation(before), os.getpid()
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        self.count, self.rgb_name = count, rgb_name
        try:
            self.guard(owner_checkpoint=True)
            digest = hashlib.sha256()
            for offset in range(0, before.st_size, BLOCK):
                self.guard()
                digest.update(_pread(self.fd, min(BLOCK, before.st_size - offset), offset))
            self.guard(owner_checkpoint=True)
            _require(digest.hexdigest() == expected_sha, "whole-file SHA256 differs")
            self.entries = self._census(expected_names)
            self.headers = {}
            self.guard(owner_checkpoint=True)
        except BaseException:
            self.cache.discard_archive(self)
            os.close(self.fd)
            self.fd = None
            raise

    def _poll_owner(self, *, mandatory=False):
        if self.checkpoint is not None and (mandatory
            or self.cache.archive_guard_calls % OWNER_CHECKPOINT_GUARD_STRIDE == 0):
            self.cache.owner_checkpoint_calls += 1
            self.checkpoint()

    def guard(self, *, owner_checkpoint=False):
        try:
            _require(self.fd is not None and os.getpid() == self.pid, "closed/inherited reader denied")
            _, current = _canonical(self.path)
            _require(_generation(current) == self.fingerprint
                and _generation(os.fstat(self.fd)) == self.fingerprint, "path/FD generation changed")
            self.cache.archive_guard_calls += 1
            self._poll_owner(mandatory=owner_checkpoint)
        except BaseException:
            self.cache.clear()
            raise

    def _admit_headers(self):
        if self.headers:
            return False
        # HH_261008 - Authenticate both permitted native headers on first query before any array import/allocation.
        admitted = {}
        for name, shape in ((self.rgb_name, (self.count, HEIGHT, WIDTH, 3)),
                            ("target.npy", (self.count, HEIGHT, WIDTH))):
            self.guard()
            stream = _MemberStream(self, self.entries[name])
            header = _npy_header(stream, shape)
            _require(self.entries[name]["size"] == len(header)
                + self.count * (NATIVE_RGB_BYTES if name == self.rgb_name else NATIVE_GT_BYTES),
                "NPY member truncation/padding denied")
            admitted[name] = header
        self.guard()
        self.headers = admitted
        return True

    def _census(self, expected_names):
        footer = _pread(self.fd, 22, self.fingerprint[2] - 22)
        signature, disk, cd_disk, disk_count, count, size, offset, comment = struct.unpack("<4s4H2LH", footer)
        _require(signature == b"PK\x05\x06" and disk == cd_disk == comment == 0
            and count == disk_count == len(expected_names) and 0 < size <= CENTRAL_LIMIT
            and offset + size == self.fingerprint[2] - 22, "bounded exact ZIP footer required; ZIP64/footer extras denied")
        central = _pread(self.fd, size, offset)
        entries, cursor = {}, 0
        for _ in range(count):
            _require(cursor + 46 <= size, "truncated central ZIP header")
            fields = struct.unpack_from("<4s6H3L5H2L", central, cursor)
            sig, made, needed, flags, compression, _, _, crc, compressed, decoded, nlen, elen, clen, entry_disk, _, attrs, local = fields
            _require(sig == b"PK\x01\x02" and needed in (20, 45) and flags == 0
                and compression in (0, 8) and 0 < nlen <= 32 and elen == clen == entry_disk == 0
                and cursor + 46 + nlen <= size, "strict unencrypted regular ZIP central entry required")
            name = central[cursor + 46:cursor + 46 + nlen].decode("ascii", errors="strict")
            cursor += 46 + nlen
            mode = attrs >> 16 if made >> 8 == 3 else 0
            _require(name in expected_names and name not in entries and not attrs & 0x10
                and (mode & 0o170000 == 0 or stat.S_ISREG(mode)), "extra/duplicate/nonregular ZIP member denied")
            frame = NATIVE_GT_BYTES if name == "target.npy" else NATIVE_RGB_BYTES
            _require(self.count * frame + 10 <= decoded <= self.count * frame + 10 + HEADER_LIMIT
                and 0 < compressed <= decoded + decoded // 1000 + BLOCK
                and (compression != 0 or compressed == decoded), "declared ZIP payload size outside native bounds")
            entries[name] = dict(name=name, size=decoded, compressed=compressed, CRC=crc,
                compression=compression, flags=flags, needed=needed, local_offset=local)
        _require(cursor == size and set(entries) == set(expected_names), "exact ZIP census required")
        end = 0
        for entry in sorted(entries.values(), key=lambda value: value["local_offset"]):
            self.guard()
            local = entry["local_offset"]
            _require(local == end and local + 30 <= offset, "ZIP preamble/gap/overlap denied")
            raw = _pread(self.fd, 30, local)
            sig, needed, flags, compression, _, _, crc, compressed, decoded, nlen, elen = struct.unpack("<4s5H3L2H", raw)
            _require(sig == b"PK\x03\x04" and needed == entry["needed"] and flags == entry["flags"]
                and compression == entry["compression"] and crc == entry["CRC"]
                and nlen == len(entry["name"]) and elen in (0, 20)
                and local + 30 + nlen + elen <= offset, "bounded matching local ZIP header required")
            details = _pread(self.fd, nlen + elen, local + 30)
            _require(details[:nlen] == entry["name"].encode("ascii"), "local/central member name differs")
            if elen:
                # HH_261008 - Python3.10 force_zip64 stores matching inline sizes below its limit;
                # newer writers may use two sentinels. Both forms must agree with bounded central sizes.
                _require(details[nlen:nlen + 4] == b"\x01\x00\x10\x00",
                    "exact NumPy local ZIP64 tag/length required")
                extended_decoded, extended_compressed = struct.unpack("<QQ", details[nlen + 4:])
                inline = needed == 20 and (decoded, compressed) == (extended_decoded, extended_compressed)
                sentinels = needed == 45 and decoded == compressed == 0xffffffff
                _require(inline or sentinels, "exact inline or sentinel local ZIP64 size form required")
                decoded, compressed = extended_decoded, extended_compressed
            _require(decoded == entry["size"] and compressed == entry["compressed"], "local/central sizes differ")
            entry["data_offset"] = local + 30 + nlen + elen
            end = entry["data_offset"] + compressed
            _require(end <= offset, "ZIP body overlaps central directory")
        _require(end == offset, "extra archive body bytes denied")
        return entries

    def read(self, index):
        _require(type(index) is int and 0 <= index < self.count, "native integer sample index required")
        self.guard(owner_checkpoint=True)
        fresh_headers = self._admit_headers()
        # Importing the array provider is deferred until byte/header checks have completed.
        import numpy as np
        arrays, receipts = [], {}
        for name, shape in ((self.rgb_name, (HEIGHT, WIDTH, 3)), ("target.npy", (HEIGHT, WIDTH))):
            self.guard()
            stream = _MemberStream(self, self.entries[name])
            _require(stream.read_exact(len(self.headers[name])) == self.headers[name], "sealed NPY header changed")
            frame_bytes = NATIVE_RGB_BYTES if name == self.rgb_name else NATIVE_GT_BYTES
            target = len(self.headers[name]) + index * frame_bytes
            stream.begin_cached_query(target)
            stream.skip(target - stream.position)
            array = np.empty(shape, dtype=np.uint8, order="C")
            stream.fill(array)
            _require(array.flags.owndata and array.flags.c_contiguous and array.nbytes == frame_bytes,
                "exact owned native slice allocation required")
            if name == "target.npy":
                _require(int(array.max()) <= 6, "GT labels outside 0..6 denied")
            arrays.append(array)
            receipts[name] = dict(selected_payload_bytes=frame_bytes,
                uncompressed_position=stream.position,
                prefix_body_bytes_inflated=stream.prefix_body_bytes_inflated,
                logical_prefix_body_bytes=index * frame_bytes,
                prefix_body_bytes_reused=max(0, stream.restored_position - len(self.headers[name])),
                checkpoint_hit=stream.checkpoint_hit,
                checkpoint_restored_uncompressed_offset=stream.restored_position,
                header_bytes_decoded_before_native_slice=len(self.headers[name]) * (2 if fresh_headers else 1),
                whole_member_crc_verified=stream.whole_member_crc_verified,
                member_EOF_and_trailer_checked=stream.position == stream.entry["size"],
                compression=stream.entry["compression"])
            if name == "target.npy":
                receipts[name]["all_selected_GT_labels_checked_0_to_6"] = True
                receipts[name]["skipped_prefix_GT_labels_checked"] = False
        self.guard(owner_checkpoint=True)
        return arrays[0], arrays[1], receipts

    def close(self):
        if self.fd is None:
            return
        failure = None
        try:
            self.guard(owner_checkpoint=True)
        except BaseException as error:
            self.closing_errors.append(_closing_error("before_close_guard", error))
            failure = error
        try:
            os.close(self.fd)
        except BaseException as error:
            self.closing_errors.append(_closing_error("fd_close", error))
            if failure is None:
                failure = error
        finally:
            self.fd = None
            self.cache.discard_archive(self)
        try:
            self._poll_owner(mandatory=True)
        except BaseException as error:
            self.closing_errors.append(_closing_error("after_close_owner", error))
            if failure is None:
                failure = error
        _require(len(self.closing_errors) <= 3, "bounded per-archive closing ledger required")
        if failure is not None:
            raise failure


class BoundedNativePairReader:
    """HH_261008 - Fixed 1209 TRAIN slots; no TRUE selector or caller-controlled member names."""

    def __init__(self, original_path, original_sha, paired_path, paired_sha, *, checkpoint=None, cache_enabled=True):
        _require(_sha(original_sha) and _sha(paired_sha), "both explicit expected SHA256 pins required")
        _require(checkpoint is None or callable(checkpoint), "optional owner checkpoint must be callable")
        self._lock, self._archives, self._closed = threading.Lock(), [], False
        self._cache = _CheckpointStore(enabled=cache_enabled)
        self.last_read_receipt = None
        self.closing_errors = []
        try:
            self._archives.append(_NativeArchive(original_path, original_sha, ORIGINAL_COUNT,
                "rgb.npy", ("rgb.npy", "target.npy"), checkpoint=checkpoint, cache=self._cache))
            self._archives.append(_NativeArchive(paired_path, paired_sha, PAIRED_COUNT,
                "rgb_false.npy", ("rgb_false.npy", "rgb_true.npy", "target.npy"), checkpoint=checkpoint, cache=self._cache))
            _require(self._archives[0].fingerprint[:2] != self._archives[1].fingerprint[:2],
                "two distinct immutable native packages required")
        except BaseException:
            for archive in self._archives:
                try:
                    archive.close()
                except BaseException:
                    pass
            self._closed = True
            raise

    def read(self, slot):
        _require(type(slot) is int and 0 <= slot < TOTAL, "exact physical TRAIN integer slot 0..1208 required")
        _require(self._lock.acquire(blocking=False), "concurrent reader RPC denied")
        try:
            _require(not self._closed, "closed reader denied")
            self.last_read_receipt = None
            for archive in self._archives:
                archive.guard(owner_checkpoint=True)
            archive = self._archives[0] if slot < ORIGINAL_COUNT else self._archives[1]
            index = slot if slot < ORIGINAL_COUNT else slot - ORIGINAL_COUNT
            rgb, gt, members = archive.read(index)
            for item in self._archives:
                item.guard(owner_checkpoint=True)
            self.last_read_receipt = dict(slot=slot, package_path=str(archive.path),
                package_sha256=archive.expected_sha, members=members, rgb_true_decoded=False,
                cache_payload_bytes=0, scratch_limit_bytes=SCRATCH_LIMIT_BYTES,
                checkpoint_cache=self._cache.snapshot(),
                caller_admission_authenticated_by_reader=False, closing_whole_file_SHA_checked=False)
            return rgb, gt
        finally:
            self._lock.release()

    def stats_snapshot(self):
        """HH_261008 - Return bounded scalar counters, including after failure/close; not fresh input proof."""
        _require(self._lock.acquire(blocking=False), "concurrent stats/read RPC denied")
        try:
            return self._cache.snapshot()
        finally:
            self._lock.release()

    def close(self):
        _require(self._lock.acquire(blocking=False), "concurrent close/read RPC denied")
        try:
            if self._closed:
                return
            failure = None
            for ordinal, archive in enumerate(self._archives):
                try:
                    archive.close()
                except BaseException as error:
                    if failure is None:
                        failure = error
                finally:
                    prefix = "original:" if ordinal == 0 else "paired:"
                    self.closing_errors.extend(dict(scope=prefix + row["scope"],
                        type=row["type"], message=row["message"]) for row in archive.closing_errors)
            self._closed = True
            _require(len(self.closing_errors) <= 6, "bounded pair closing ledger required")
            if failure is not None:
                raise failure
        finally:
            self._lock.release()

    def __enter__(self):
        _require(not self._closed, "closed reader denied")
        return self

    def __exit__(self, *_):
        self.close()
