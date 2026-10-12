"""HH_261008 - V2 bounded, nontraining native TRAIN RGB/GT reads from sealed NPZs.

The outer owner MUST establish fresh data admission, ownership, deadlines and
source/provider seals before construction. Expected SHA strings authenticate
bytes, not authority, split provenance, consent, or permission to train. This
module has no CLI, trainer, registry writer, model import or authority grant.

Only original729 rgb/target and paired480 rgb_false/target are decoded. TRUE is
censused as ZIP metadata and covered by the compressed-file SHA, never inflated.
Returned arrays own exactly one native RGB/GT slice. Reader scratch is bounded
by 64KiB blocks and small headers; caller-retained arrays are outside that bound.
STORED seeks are direct. DEFLATED reads inflate the prefix on every new stream:
bounded memory is NOT a throughput claim or a claim that only selected pixels
were decoded. Partial member reads do not verify full-member CRC/EOF/trailers.
Closing checks path/FD generations; the owner must separately perform any
required closing whole-file SHA. Generation checks are observations, not an OS
immutability guarantee against a privileged concurrent writer.
"""

import ast
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


class _MemberStream:
    """HH_261008 - Cap both compressed input and raw-DEFLATE output, including skipped prefixes."""

    def __init__(self, owner, entry):
        self.owner, self.entry = owner, entry
        self.position, self.compressed_position, self.crc = 0, 0, 0
        self.pending = b""
        self.decoder = zlib.decompressobj(-15) if entry["compression"] == 8 else None
        self.whole_member_crc_verified = False

    def _read(self, size):
        _require(0 < size <= BLOCK and self.position + size <= self.entry["size"],
            "member read outside declared payload")
        if self.decoder is None:
            raw = _pread(self.owner.fd, size, self.entry["data_offset"] + self.position)
        else:
            while True:
                if not self.pending and self.compressed_position < self.entry["compressed"]:
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
        while remaining:
            self.owner.guard()
            remaining -= len(self._read(min(BLOCK, remaining)))

    def fill(self, destination):
        view = memoryview(destination).cast("B")
        offset = 0
        while offset < len(view):
            self.owner.guard()
            raw = self._read(min(BLOCK, len(view) - offset))
            view[offset:offset + len(raw)] = raw
            offset += len(raw)
        if self.position == self.entry["size"]:
            self._finish()

    def _finish(self):
        if self.decoder is not None:
            # A final zero-output drain checks terminator and surplus without unbounded flush().
            while not self.decoder.eof:
                if not self.pending and self.compressed_position < self.entry["compressed"]:
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

    def __init__(self, path, expected_sha, count, rgb_name, expected_names):
        _require(_sha(expected_sha), "explicit lowercase expected whole-file SHA256 required")
        _require(type(count) is int and 0 < count <= ORIGINAL_COUNT, "bounded native sample count required")
        _require((rgb_name == "rgb.npy" and expected_names == ("rgb.npy", "target.npy"))
            or (rgb_name == "rgb_false.npy" and expected_names == ("rgb_false.npy", "rgb_true.npy", "target.npy")),
            "only original or paired FALSE member contracts permitted")
        self.path, before = _canonical(path)
        _require(22 <= before.st_size <= ARCHIVE_LIMIT, "archive size bound denied")
        self.expected_sha, self.fingerprint, self.pid = expected_sha, _generation(before), os.getpid()
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        self.count, self.rgb_name = count, rgb_name
        try:
            self.guard()
            digest = hashlib.sha256()
            for offset in range(0, before.st_size, BLOCK):
                self.guard()
                digest.update(_pread(self.fd, min(BLOCK, before.st_size - offset), offset))
            self.guard()
            _require(digest.hexdigest() == expected_sha, "whole-file SHA256 differs")
            self.entries = self._census(expected_names)
            self.headers = {}
            for name, shape in ((rgb_name, (count, HEIGHT, WIDTH, 3)),
                                ("target.npy", (count, HEIGHT, WIDTH))):
                stream = _MemberStream(self, self.entries[name])
                header = _npy_header(stream, shape)
                _require(self.entries[name]["size"] == len(header)
                    + count * (NATIVE_RGB_BYTES if name == rgb_name else NATIVE_GT_BYTES),
                    "NPY member truncation/padding denied")
                self.headers[name] = header
            self.guard()
        except BaseException:
            os.close(self.fd)
            self.fd = None
            raise

    def guard(self):
        _require(self.fd is not None and os.getpid() == self.pid, "closed/inherited reader denied")
        _, current = _canonical(self.path)
        _require(_generation(current) == self.fingerprint
            and _generation(os.fstat(self.fd)) == self.fingerprint, "path/FD generation changed")

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
        self.guard()
        # Importing the array provider is deferred until byte/header checks have completed.
        import numpy as np
        arrays, receipts = [], {}
        for name, shape in ((self.rgb_name, (HEIGHT, WIDTH, 3)), ("target.npy", (HEIGHT, WIDTH))):
            self.guard()
            stream = _MemberStream(self, self.entries[name])
            _require(stream.read_exact(len(self.headers[name])) == self.headers[name], "sealed NPY header changed")
            frame_bytes = NATIVE_RGB_BYTES if name == self.rgb_name else NATIVE_GT_BYTES
            stream.skip(index * frame_bytes)
            array = np.empty(shape, dtype=np.uint8, order="C")
            stream.fill(array)
            _require(array.flags.owndata and array.flags.c_contiguous and array.nbytes == frame_bytes,
                "exact owned native slice allocation required")
            if name == "target.npy":
                _require(int(array.max()) <= 6, "GT labels outside 0..6 denied")
            arrays.append(array)
            receipts[name] = dict(selected_payload_bytes=frame_bytes,
                uncompressed_position=stream.position,
                prefix_body_bytes_inflated=index * frame_bytes if stream.decoder is not None else 0,
                whole_member_crc_verified=stream.whole_member_crc_verified,
                member_EOF_and_trailer_checked=stream.position == stream.entry["size"],
                compression=stream.entry["compression"])
            if name == "target.npy":
                receipts[name]["all_selected_GT_labels_checked_0_to_6"] = True
                receipts[name]["skipped_prefix_GT_labels_checked"] = False
        self.guard()
        return arrays[0], arrays[1], receipts

    def close(self):
        if self.fd is None:
            return
        try:
            self.guard()
        finally:
            os.close(self.fd)
            self.fd = None


class BoundedNativePairReader:
    """HH_261008 - Fixed 1209 TRAIN slots; no TRUE selector or caller-controlled member names."""

    def __init__(self, original_path, original_sha, paired_path, paired_sha):
        _require(_sha(original_sha) and _sha(paired_sha), "both explicit expected SHA256 pins required")
        self._lock, self._archives, self._closed = threading.Lock(), [], False
        self.last_read_receipt = None
        try:
            self._archives.append(_NativeArchive(original_path, original_sha, ORIGINAL_COUNT,
                "rgb.npy", ("rgb.npy", "target.npy")))
            self._archives.append(_NativeArchive(paired_path, paired_sha, PAIRED_COUNT,
                "rgb_false.npy", ("rgb_false.npy", "rgb_true.npy", "target.npy")))
            _require(self._archives[0].fingerprint[:2] != self._archives[1].fingerprint[:2],
                "two distinct immutable native packages required")
        except BaseException:
            for archive in self._archives:
                try:
                    archive.close()
                except Exception:
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
                archive.guard()
            archive = self._archives[0] if slot < ORIGINAL_COUNT else self._archives[1]
            index = slot if slot < ORIGINAL_COUNT else slot - ORIGINAL_COUNT
            rgb, gt, members = archive.read(index)
            for item in self._archives:
                item.guard()
            self.last_read_receipt = dict(slot=slot, package_path=str(archive.path),
                package_sha256=archive.expected_sha, members=members, rgb_true_decoded=False,
                cache_payload_bytes=0, scratch_limit_bytes=SCRATCH_LIMIT_BYTES,
                caller_admission_authenticated_by_reader=False, closing_whole_file_SHA_checked=False)
            return rgb, gt
        finally:
            self._lock.release()

    def close(self):
        _require(self._lock.acquire(blocking=False), "concurrent close/read RPC denied")
        try:
            if self._closed:
                return
            failure = None
            for archive in self._archives:
                try:
                    archive.close()
                except Exception as error:
                    if failure is None:
                        failure = error
            self._closed = True
            if failure is not None:
                raise failure
        finally:
            self._lock.release()

    def __enter__(self):
        _require(not self._closed, "closed reader denied")
        return self

    def __exit__(self, *_):
        self.close()
