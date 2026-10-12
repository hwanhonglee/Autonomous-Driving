"""HH_261008 - Independently check saved FIRST16 cache A/B bytes and metadata.

SOURCE_ONLY until ROOT reads this entire source and explicitly executes it with
external RAW plan/authority/source pins. Standard-library only: no producer,
NumPy, PIL, ZIP member, raster, model, training, simulator or remote import.
Compressed input hashes are whole-file seals, never member or pixel replay.
Pixel SHA/support/rescue measurements remain producer claims checked as stored
metadata. PNG inspection authenticates its container/IHDR/chunk CRC only; IDAT
is not inflated. Exact native decoder allocation and OS cold-start equivalence
are not claimed. Failures preserve original files and a fresh failed receipt.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import struct
import sys
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
NAME = "map_crop_rescue_reader_ab_v1"
PRODUCER_SCHEMA = "portable_e2e.map_crop_rescue_reader_ab.20261008.v1"
SCHEMA = "portable_e2e.map_crop_rescue_reader_ab_saved_check.20261008.v1"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08" / NAME
OUT = ROOT / "storage/experiments/diagnostics/2026-10-08" / NAME
RECEIPT = OUT.parent / (NAME + "_independent_v1.json")
TEST_SOURCE = ROOT / "learning/tests/test_check_map_crop_rescue_reader_ab_saved_20261008_v1.py"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
DATA = ROOT / "storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
STEPS, ARMS = list(range(1, 17)), ("CONTROL_V3_CACHE0", "CANDIDATE_V3_CACHE12M")
SELECTION = ROOT / "storage/experiments/plans/2026-10-08/map_crop_rescue_reader_ab_metadata_selection_v2.json"
SELECTION_PIN = "998fa695a1f2437d4f34d4bc1341c7c43256f038ef5cde072f4a7918f3b1f8e1"
BLOCK, MAX_METADATA, JOURNAL_LIMIT, PNG_LIMIT = 64*1024, 512*1024, 256*1024, 512*1024
FLOOR, RESERVE, RSS, DEADLINE = 2*1024**3, 64*1024**2, 512*1024**2, 900
CACHE_STATES, STATE_BYTES, CACHE_LIMIT, CACHE_STRIDE = 48, 256*1024, 12*1024**2, 32*1024**2
POLICY_PIN = "566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f"
SOURCE_NAMES = {
    "tools/e2e/verify_map_crop_batch_bytes_20261008_v3.py",
    "learning/tests/test_verify_map_crop_batch_bytes_20261008_v3.py",
    "learning/portable_e2e/map_crop_batch_worker_20261008_v3.py",
    "learning/tests/test_map_crop_batch_worker_20261008_v3.py",
    "learning/portable_e2e/map_native_pair_reader_20261008_v2.py",
    "learning/tests/test_map_native_pair_reader_20261008_v2.py",
    "learning/portable_e2e/map_crop_preservation_journal_20261008_v1.py",
    "learning/tests/test_map_crop_preservation_journal_20261008_v1.py",
    "tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py",
    "tools/e2e/map_appearance_matched_contract_20261001.py",
    "tools/workspace_python.py", "tools/canonical_workspace.py",
    "learning/portable_e2e/__init__.py", "learning/portable_e2e/contract.py",
    "learning/portable_e2e/map_native_pair_reader_20261008_v3.py",
    "learning/tests/test_map_native_pair_reader_20261008_v3.py",
    "tools/e2e/verify_map_crop_rescue_reader_ab_20261008_v1.py",
    "learning/tests/test_verify_map_crop_rescue_reader_ab_20261008_v1.py",
}
INPUTS = {
    str(DATA / "original_train729.npz"): "d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3",
    str(DATA / "paired_train480.npz"): "47c23a3f6fe67aa40ea7f5bb0c770fd6037463ed090f2b9390a89317c4dedee3",
    str(RUN / "control/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN / "candidate/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN / "inputs/manifest.json"): "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b",
    str(RUN / "inputs/study_train_registry.json"): "ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d",
    str(RUN / "inputs/paired_samples.json"): "899d82128ae9597beca6e22fbad4abaaff40c7e1fbc502418f946f71c72b9d6b",
    str(SELECTION): SELECTION_PIN,
}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def integer(value, minimum=0, maximum=2**63-1):
    return type(value) is int and minimum <= value <= maximum


def same(left, right):
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(same(left[k], right[k]) for k in left)
    if type(left) is list:
        return len(left) == len(right) and all(same(a, b) for a, b in zip(left, right))
    return left == right


def decode(raw):
    def pairs(rows):
        result = {}
        for key, value in rows:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def number(value):
        parsed = float(value)
        require(math.isfinite(parsed), "overflowing JSON float")
        return parsed
    return json.loads(raw, object_pairs_hook=pairs, parse_float=number,
        parse_constant=lambda _: require(False, "nonfinite JSON token"))


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n").encode()


def seconds(value):
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value < DEADLINE


def canonical(path, exists=True):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path == path.resolve(strict=exists)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical nonsymlink path required")
    return path


def generation(info):
    return [getattr(info, key) for key in ("st_mode", "st_uid", "st_gid", "st_nlink", "st_dev",
        "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")]


class Audit:
    """HH_261008 - Bound raw seals and retain before/open/close/after generations."""

    def __init__(self, lease):
        self.lease, self.started, self.files, self.maximum_HWM = lease, time.monotonic(), {}, 0
        self.pid = os.getpid()

    def guard(self):
        require(os.getpid() == self.pid and not self.lease.closed, "inherited or closed checker denied")
        path, fd = canonical(LOCK).stat(), os.fstat(self.lease.fileno())
        require(stat.S_ISREG(path.st_mode) and path.st_nlink == 1 and generation(path) == generation(fd),
            "held runtime lease FD/path differs")
        memory = {line.split(":")[0]: int(line.split()[1])*1024
            for line in Path("/proc/self/status").read_text().splitlines()
            if line.startswith(("VmRSS:", "VmHWM:"))}
        self.maximum_HWM = max(self.maximum_HWM, memory["VmHWM"])
        require(self.maximum_HWM <= RSS and time.monotonic()-self.started < DEADLINE,
            "sampled checker memory/deadline exceeded")
        fs = os.statvfs(ROOT)
        require(fs.f_bavail*fs.f_frsize >= FLOOR+RESERVE, "unchanged floor plus reserve unavailable")

    def read(self, path, expected=None, metadata=False, maximum=1024**3):
        path = canonical(path)
        before = path.stat()
        require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1, "regular single-link evidence required")
        require(before.st_size <= (min(MAX_METADATA, maximum) if metadata else maximum), "bounded evidence required")
        require(expected is None or sha(expected), "exact lowercase RAW SHA required")
        if str(path) in self.files:
            require(generation(before) == self.files[str(path)]["generation"],
                "previously admitted evidence generation changed before reread")
        digest, chunks = hashlib.sha256(), []
        with path.open("rb") as stream:
            opened = os.fstat(stream.fileno())
            for block in iter(lambda: stream.read(BLOCK), b""):
                self.guard()
                digest.update(block)
                if metadata:
                    chunks.append(block)
            closed = os.fstat(stream.fileno())
        observed = generation(before)
        require(observed == generation(opened) == generation(closed) == generation(path.stat()), "evidence generation changed")
        value = digest.hexdigest()
        require(expected is None or value == expected, "evidence RAW seal differs: "+str(path))
        witness = dict(sha256=value, bytes=before.st_size, generation=observed)
        if str(path) in self.files:
            require(same(witness, self.files[str(path)]), "evidence changed between independent reads")
        self.files[str(path)] = witness
        return b"".join(chunks) if metadata else value

    def close_seals(self):
        for path, witness in list(self.files.items()):
            self.read(path, witness["sha256"])
        self.guard()


def contract():
    return dict(declared_steps=STEPS, expected_batches_per_arm=16, expected_slot_exposures_per_arm=128,
        arms=list(ARMS), reader_source_same_in_both_arms=True,
        CONTROL_checkpoint_cache_enabled=False, CANDIDATE_checkpoint_cache_enabled=True,
        candidate_checkpoint_max_states=CACHE_STATES, checkpoint_state_account_bytes=STATE_BYTES,
        candidate_cache_account_limit_bytes=CACHE_LIMIT, checkpoint_stride_bytes=CACHE_STRIDE,
        owner_checkpoint_guard_stride=16, state_accounting_is_reservation_not_exact_native_allocation=True,
        original20k_batches_outside_declaration_per_arm=19984,
        selected_metadata_not_random_or_independent_TEST=True,
        metadata_positive_partial_crop_hints_do_not_prove_original_GT_zero=True,
        allowed_original_members=["rgb.npy", "target.npy"], allowed_paired_members=["rgb_false.npy", "target.npy"],
        paired_TRUE_member_decode_authorized=False, baseline_both_RGB_and_GT_SHA_required=True,
        same_worker_crop_policy_original_rows_FALSE_and_proposals=True, RNG_draws_added=0,
        CPU_affinity=[3], threads=1, sampled_self_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        floor_bytes=FLOOR, reserve_bytes=RESERVE, maximum_combined_PLAN_OUTPUT_bytes=2*1024**2,
        closing_bytes=512*1024, journal_maximum_bytes_per_arm=JOURNAL_LIMIT,
        optional_first_actual_rescue_PNG_maximum_bytes=PNG_LIMIT, PNG_measured_after_both_arms=True,
        PNG_semantic_GT_signmask_not_model_prediction=True, sampled_guards_not_OS_containment=True,
        old_target_only_authority_reused=False, architecture_changed=False,
        training_steps=0, NN=0, main_approved=0, sub_approved=0)


def authority(plan_pin):
    return dict(schema=PRODUCER_SCHEMA+".ROOT_authority", plan_path=str(PLAN_DIR/"ROOT_plan.json"), plan_sha256=plan_pin,
        limited_original_and_FALSE_RGB_GT_verification_authorized=True, old_target_only_authority_extended=False,
        same_source_reader_cache_AB_authorized=True, optional_actual_first_rescue_diagnostic_PNG_authorized=True,
        training_authorized=False, NN_authorized=False, DEV_HOLDOUT_TEST_weights_TRUE_deniedRAW_decode_authorized=False,
        output=str(OUT), CPU_affinity=[3], contract=contract())


BASELINE_FIELDS = {"step", "physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits", "RGB_sha256", "target_sha256"}


def box(value):
    require(type(value) is list and len(value) == 4 and all(integer(v, 0, 640) for v in value), "strict crop box required")
    top, left, height, width = value
    require(height > 0 and width > 0 and top+height <= 360 and left+width <= 640, "crop exceeds native bounds")


def baseline(row, step):
    require(type(row) is dict and set(row) == BASELINE_FIELDS and integer(row["step"], 1, 20000)
        and row["step"] == step, "exact original baseline step/fields required")
    require(type(row["physical_slot_indices"]) is list and len(row["physical_slot_indices"]) == 8
        and all(integer(v, 0, 1208) for v in row["physical_slot_indices"]), "eight exact native slots required")
    require(type(row["shared_crop_boxes"]) is list and len(row["shared_crop_boxes"]) == 8, "eight original boxes required")
    for value in row["shared_crop_boxes"]:
        box(value)
    require(same(row["actual_paired_TRUE_bits"], [0]*8) and sha(row["RGB_sha256"]) and sha(row["target_sha256"]),
        "FALSE-only bits and original RGB/GT SHA required")


def original_selected(audit):
    path = canonical(RUN/"control/exposures.jsonl")
    selected, count, first16 = [], 0, hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            raw = stream.readline(4097)
            if not raw:
                break
            audit.guard()
            require(len(raw) <= 4096 and raw.endswith(b"\n"), "bounded complete original metadata line required")
            count += 1
            require(count <= 20000, "extra original metadata row")
            row = decode(raw)
            baseline(row, count)
            if count <= 16:
                first16.update(raw)
                selected.append(dict(step=count, RAW_line_sha256=hashlib.sha256(raw).hexdigest(), baseline=row))
    require(count == 20000 and same([r["step"] for r in selected], STEPS), "full original20k metadata tail required")
    audit.read(path, INPUTS[str(path)])
    return selected, first16.hexdigest()


def check_batch(receipt, expected):
    fields = {"schema", "step", "journal", "native_slots", "actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT",
        "source_and_baseline_RAW_authenticated_by_worker", "training_steps", "NN"}
    require(type(receipt) is dict and set(receipt) == fields
        and receipt["schema"] == "portable_e2e.map_crop_batch_worker.20261008.v3.batch"
        and same(receipt["step"], expected["step"]), "exact worker V3 chronology required")
    row = receipt["journal"]
    jfields = {"schema", "policy_source_sha256", "baseline_exposure", "selected_crop_boxes", "full_GT_sign_pixels",
        "original_NEAREST_sign_pixels", "protected", "rescued", "candidate_RGB_sha256", "candidate_target_sha256"}
    require(type(row) is dict and set(row) == jfields
        and row["schema"] == "portable_e2e.map_crop_preservation_journal.20261008.v1"
        and row["policy_source_sha256"] == POLICY_PIN and same(row["baseline_exposure"], expected["baseline"]),
        "original baseline/policy join differs")
    baseline(row["baseline_exposure"], expected["step"])
    require(sha(row["candidate_RGB_sha256"]) and sha(row["candidate_target_sha256"]), "candidate byte SHA invalid")
    require(all(type(row[k]) is list and len(row[k]) == 8 for k in
        ("selected_crop_boxes", "full_GT_sign_pixels", "original_NEAREST_sign_pixels", "protected", "rescued")),
        "eight policy witnesses required")
    slots = receipt["native_slots"]
    require(type(slots) is list and len(slots) == 8, "eight actual native SHA receipts required")
    for position, native in enumerate(slots):
        require(type(native) is dict and set(native) == {"position", "slot", "native_RGB_sha256", "native_target_sha256",
            "reader_provenance_authenticated_by_worker"} and same(native["position"], position)
            and same(native["slot"], expected["baseline"]["physical_slot_indices"][position])
            and sha(native["native_RGB_sha256"]) and sha(native["native_target_sha256"])
            and native["reader_provenance_authenticated_by_worker"] is False, "native slot/SHA join differs")
        full, proposed = row["full_GT_sign_pixels"][position], row["original_NEAREST_sign_pixels"][position]
        require(integer(full, 0, 230400) and integer(proposed, 0, 230400), "strict GT support metadata required")
        protected = ((expected["step"]-1)*8+position) % 2 == 0
        rescued = protected and full > 0 and proposed == 0
        require(row["protected"][position] is protected and row["rescued"][position] is rescued,
            "protected/rescued policy metadata differs")
        chosen = [0, 0, 360, 640] if rescued else expected["baseline"]["shared_crop_boxes"][position]
        box(row["selected_crop_boxes"][position])
        require(same(row["selected_crop_boxes"][position], chosen), "same RGB/GT selected box differs")
    if not any(row["rescued"]):
        require(row["candidate_RGB_sha256"] == expected["baseline"]["RGB_sha256"]
            and row["candidate_target_sha256"] == expected["baseline"]["target_sha256"], "unchanged batch SHA differs")
    require(all(receipt[k] is True for k in ("actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT"))
        and receipt["source_and_baseline_RAW_authenticated_by_worker"] is False
        and same(receipt["NN"], 0) and same(receipt["training_steps"], 0), "worker scope claims differ")
    return row


def check_actual_batch(actual, row, step):
    require(type(actual) is dict and set(actual) == {"step", "original_RGB_GT_equal", "candidate_RGB_sha256",
        "candidate_target_sha256", "selected_crop_boxes", "selected_GT_sign_pixels", "rescued_slots", "batch_elapsed_seconds"}
        and same(actual["step"], step) and actual["original_RGB_GT_equal"] is True
        and actual["candidate_RGB_sha256"] == row["candidate_RGB_sha256"]
        and actual["candidate_target_sha256"] == row["candidate_target_sha256"]
        and same(actual["selected_crop_boxes"], row["selected_crop_boxes"])
        and same(actual["rescued_slots"], sum(row["rescued"])) and seconds(actual["batch_elapsed_seconds"]),
        "result batch/journal SHA/box/rescue join differs")
    counts = actual["selected_GT_sign_pixels"]
    require(type(counts) is list and len(counts) == 8 and all(integer(n, 0, 230400) for n in counts),
        "eight exact selected GT support counts required")
    rescues = []
    for position, rescued in enumerate(row["rescued"]):
        expected_count = row["full_GT_sign_pixels"][position] if rescued else row["original_NEAREST_sign_pixels"][position]
        require(counts[position] == expected_count, "selected actual GT support differs from its full/proposed box")
        if rescued:
            require(counts[position] > 0, "rescued full-FOV GT support must remain positive")
            rescues.append(dict(step=step, position=position, slot=row["baseline_exposure"]["physical_slot_indices"][position],
                full_GT_sign_pixels=row["full_GT_sign_pixels"][position], proposed_NEAREST_GT_sign_pixels=0,
                selected_NEAREST_GT_sign_pixels=counts[position], proposed_box=row["baseline_exposure"]["shared_crop_boxes"][position],
                selected_box=[0, 0, 360, 640], actual_same_box_for_RGB_GT=True))
    return rescues


def check_stats(value, enabled, after_close=False):
    ints = {"current_checkpoint_states", "maximum_checkpoint_states_observed", "max_checkpoint_states",
        "checkpoint_state_account_bytes", "cache_accounted_bytes", "maximum_cache_accounted_bytes",
        "cache_account_limit_bytes", "owner_checkpoint_calls", "archive_generation_guard_calls",
        "owner_checkpoint_guard_stride", "checkpoint_hits", "checkpoint_misses", "checkpoint_creations",
        "checkpoint_evictions", "total_prefix_body_bytes_inflated", "total_selected_payload_bytes",
        "native_row_cache_payload_bytes", "checkpoint_stride_bytes"}
    require(type(value) is dict and set(value) == ints | {"checkpoint_cache_enabled",
        "state_accounting_is_reservation_not_exact_native_allocation"} and all(integer(value[k]) for k in ints),
        "exact typed cache scalar fields required")
    require(value["checkpoint_cache_enabled"] is enabled
        and value["state_accounting_is_reservation_not_exact_native_allocation"] is True
        and value["checkpoint_state_account_bytes"] == STATE_BYTES and value["checkpoint_stride_bytes"] == CACHE_STRIDE
        and value["owner_checkpoint_guard_stride"] == 16 and value["native_row_cache_payload_bytes"] == 0,
        "same V3 accounting/guard policy differs")
    cap, states = (CACHE_LIMIT, CACHE_STATES) if enabled else (0, 0)
    require(value["cache_account_limit_bytes"] == cap and value["max_checkpoint_states"] == states
        and value["current_checkpoint_states"] <= value["maximum_checkpoint_states_observed"] <= states
        and value["cache_accounted_bytes"] == value["current_checkpoint_states"]*STATE_BYTES
        and value["maximum_cache_accounted_bytes"] == value["maximum_checkpoint_states_observed"]*STATE_BYTES <= cap,
        "cache current/peak48-state12MiB reservation bound differs")
    if not enabled:
        require(all(value[k] == 0 for k in ("checkpoint_hits", "checkpoint_creations", "checkpoint_evictions")),
            "cache0 cannot report restored/created/evicted states")
    if after_close:
        require(value["current_checkpoint_states"] == value["cache_accounted_bytes"] == 0, "closed cache retained states")


def check_last_slot(read, expected, enabled):
    # HH_261008 - Only the recorded final slot's stream history is joined; not all128 per-read CRC histories.
    require(type(read) is dict and same(read["slot"], expected["baseline"]["physical_slot_indices"][-1])
        and all(read[k] is False for k in ("rgb_true_decoded", "caller_admission_authenticated_by_reader", "closing_whole_file_SHA_checked"))
        and same(read["cache_payload_bytes"], 0) and same(read["scratch_limit_bytes"], 1024**2), "last slot scope differs")
    slot = read["slot"]
    index, count, rgb_name, package = ((slot, 729, "rgb.npy", DATA/"original_train729.npz") if slot < 729
        else (slot-729, 480, "rgb_false.npy", DATA/"paired_train480.npz"))
    require(read["package_path"] == str(package) and read["package_sha256"] == INPUTS[str(package)]
        and type(read["members"]) is dict and set(read["members"]) == {rgb_name, "target.npy"},
        "last slot original/FALSE package-member route differs")
    for name, member in read["members"].items():
        pixels = 360*640*(3 if name == rgb_name else 1)
        fields = ("selected_payload_bytes", "uncompressed_position", "prefix_body_bytes_inflated", "logical_prefix_body_bytes",
            "prefix_body_bytes_reused", "checkpoint_restored_uncompressed_offset", "header_bytes_decoded_before_native_slice", "compression")
        require(all(integer(member[k]) for k in fields) and member["selected_payload_bytes"] == pixels
            and member["compression"] in (0, 8) and member["logical_prefix_body_bytes"] == index*pixels
            and type(member["checkpoint_hit"]) is bool, "last slice strict prefix fields differ")
        header = member["uncompressed_position"]-(index+1)*pixels
        require(integer(header, 11, 266) and header % 16 == 0
            and member["header_bytes_decoded_before_native_slice"] in (header, 2*header),
            "last slice final position/header/native frame relation differs")
        if member["compression"] == 8:
            require(member["prefix_body_bytes_inflated"]+member["prefix_body_bytes_reused"] == index*pixels,
                "logical prefix is not newly inflated plus reused prefix")
        else:
            require(member["prefix_body_bytes_inflated"] == member["prefix_body_bytes_reused"] == 0
                and member["checkpoint_hit"] is False,
                "STORED direct seek cannot claim prefix inflation/reuse")
        if member["checkpoint_hit"]:
            require(member["checkpoint_restored_uncompressed_offset"]-header == member["prefix_body_bytes_reused"]
                and member["checkpoint_restored_uncompressed_offset"] <= header+index*pixels,
                "restored checkpoint offset/header/reused prefix relation differs")
        if not enabled or not member["checkpoint_hit"]:
            require(member["checkpoint_hit"] is False and member["prefix_body_bytes_reused"] == 0
                and member["checkpoint_restored_uncompressed_offset"] == 0, "non-restored slice reports checkpoint reuse")
        require(member["member_EOF_and_trailer_checked"] is (index == count-1)
            and member["whole_member_crc_verified"] is (index == count-1 and member["compression"] == 8),
            "partial CRC/EOF scope differs")
        if name == "target.npy":
            require(member["all_selected_GT_labels_checked_0_to_6"] is True and member["skipped_prefix_GT_labels_checked"] is False,
                "selected/prefix GT label coverage differs")
    check_stats(read["checkpoint_cache"], enabled)


def png_container(raw):
    """HH_261008 - Inspect bounded PNG container only; never inflate IDAT or decode pixels."""
    require(type(raw) is bytes and 8 <= len(raw) <= PNG_LIMIT and raw[:8] == b"\x89PNG\r\n\x1a\n", "bounded PNG signature required")
    cursor, chunks, header, idat, ended = 8, 0, None, 0, False
    while cursor < len(raw):
        require(not ended and cursor+12 <= len(raw), "PNG trailer/truncated chunk denied")
        size = struct.unpack_from(">I", raw, cursor)[0]
        kind = raw[cursor+4:cursor+8]
        require(size <= PNG_LIMIT and cursor+12+size <= len(raw) and all(65 <= c <= 90 or 97 <= c <= 122 for c in kind),
            "bounded valid PNG chunk required")
        data = raw[cursor+8:cursor+8+size]
        crc = struct.unpack_from(">I", raw, cursor+8+size)[0]
        require(zlib.crc32(data, zlib.crc32(kind)) & 0xffffffff == crc, "PNG chunk CRC differs")
        if chunks == 0:
            require(kind == b"IHDR" and size == 13, "first exact IHDR required")
            header = struct.unpack(">IIBBBBB", data)
            require(header == (1920, 768, 8, 2, 0, 0, 0), "actual diagnostic RGB1920x768 IHDR differs")
        elif kind == b"IHDR":
            require(False, "duplicate IHDR denied")
        if kind == b"IDAT":
            idat += size
        elif kind == b"IEND":
            require(size == 0, "empty IEND required")
            ended = True
        chunks += 1
        require(chunks <= 128, "bounded PNG chunk count required")
        cursor += 12+size
    require(ended and idat > 0 and cursor == len(raw), "complete PNG container/IDAT/IEND required")
    return dict(width=header[0], height=header[1], chunk_count=chunks, IDAT_compressed_bytes=idat,
        chunk_CRC_checked=True, IDAT_inflated=False, raster_pixels_decoded=False)


def merge_seals(*groups):
    result = {}
    for group in groups:
        require(type(group) is dict and all(type(path) is str and sha(pin) for path, pin in group.items()),
            "named lowercase seal mapping required")
        for path, pin in group.items():
            require(path not in result or result[path] == pin, "overlapping prospective seals disagree")
            result[path] = pin
    return result


def check_selection(value, selected, first16_sha):
    require(type(value) is dict and value["schema"] == "portable_e2e.map_crop_rescue_reader_ab_metadata_selection.20261008.v2"
        and value["state"] == "METADATA_ONLY_PREPARED_NOT_PAYLOAD_VERIFIED"
        and same(value["metadata_scan_steps"], STEPS) and same(value["metadata_exposures_examined"], 128)
        and value["first16_original_RAW_lines_sha256"] == first16_sha,
        "prospective FIRST16 original RAW metadata basis differs")
    require(all(value[k] is False for k in ("policy_changed", "proposals_rerolled",
        "successful_crop_or_rescue_cases_selected_after_payload_decode", "candidate_metadata_is_independent_TEST",
        "actual_RGB_rescue_effect_proven", "model_performance_gain_proven", "existing_thresholds_or_resource_caps_lowered"))
        and value["prospective_FIRST16_fixed_before_new_payload_decode"] is True
        and type(value["new_work"]) is dict and all(same(v, 0) for v in value["new_work"].values()),
        "prepared hints cannot assert new payload/rescue/model gain")
    prospective = {name: dict(reader_revision="V3", cache_enabled=index == 1, declared_batches=16,
        declared_slot_exposures=128, PASS_batches=0, FAIL_batches=0, UNEXECUTED_batches=16,
        original20000_batches_OUT_OF_SCOPE=19984) for index, name in enumerate(ARMS)}
    require(same(value["prospective_arms"], prospective)
        and same(value["resource_caps_preserved"], dict(sampled_VmHWM_bytes=RSS, deadline_seconds=DEADLINE,
            floor_bytes=FLOOR, reserve_bytes=RESERVE)), "same-code prospective arms or caps differ")
    candidates, seen = value["candidates"], set()
    require(type(candidates) is list and same(value["candidate_count"], 7) and len(candidates) == 7,
        "seven metadata hints required")
    for hint in candidates:
        require(type(hint) is dict and set(hint) == {"step", "position_zero_based", "global_ordinal_zero_based", "slot",
            "fullGT_sign_pixels", "proposed_box", "actual_crop_GT_zero_unknown"}, "exact hint metadata fields required")
        step, position = hint["step"], hint["position_zero_based"]
        require(integer(step, 1, 16) and integer(position, 0, 7) and (step, position) not in seen
            and hint["actual_crop_GT_zero_unknown"] is True, "hint chronology or cropped-GT unknown differs")
        seen.add((step, position))
        row = selected[step-1]["baseline"]
        ordinal = (step-1)*8+position
        require(same(hint["global_ordinal_zero_based"], ordinal) and ordinal % 2 == 0
            and same(hint["slot"], row["physical_slot_indices"][position])
            and same(hint["proposed_box"], row["shared_crop_boxes"][position])
            and hint["proposed_box"] != [0, 0, 360, 640] and integer(hint["fullGT_sign_pixels"], 1, 230400),
            "hint slot/proposal/positive support metadata join differs")
    return dict(schema=value["schema"], selection_RAW_sha256=SELECTION_PIN,
        first16_original_RAW_lines_sha256=first16_sha, candidates=candidates, cropped_GT_zero_verified=False,
        actual_RGB_rescue_effect_proven=False, candidate_metadata_is_independent_TEST=False,
        actual_new_payload_decodes_before_selection=0)


def inspect(audit, plan_pin, grant_pin, script_pin, test_pin):
    audit.read(Path(__file__).resolve(), script_pin)
    audit.read(TEST_SOURCE, test_pin)
    raw_plan = audit.read(PLAN_DIR/"ROOT_plan.json", plan_pin, True)
    raw_grant = audit.read(PLAN_DIR/"ROOT_authority.json", grant_pin, True)
    require(audit.read(PLAN_DIR/"candidate_plan.json", plan_pin, True) == raw_plan
        and audit.read(OUT/"ROOT_plan.json", plan_pin, True) == raw_plan
        and audit.read(OUT/"ROOT_authority.json", grant_pin, True) == raw_grant,
        "prepared and executed ROOT RAW copies differ")
    plan, grant = decode(raw_plan), decode(raw_grant)
    require(plan["schema"] == PRODUCER_SCHEMA and same(plan["contract"], contract())
        and same(plan["input_sha256"], INPUTS) and plan["output"] == str(OUT)
        and same(grant, authority(plan_pin)), "exact prospective bounded ROOT declaration/authority differs")
    sources = plan["source_sha256"]
    providers = plan["environment"]["selected_provider_sha256"]
    graphics = plan["diagnostic_graphics_environment"]["selected_graphics_source_sha256"]
    require(type(sources) is dict and set(sources) == {str(ROOT/name) for name in SOURCE_NAMES}
        and len(set(sources.values())) == 18 and same(plan["source_archive_count"], 18)
        and type(providers) is dict and len(providers) == 6
        and type(graphics) is dict and 2 <= len(graphics) <= 3, "exact18 sources/scientific6/optional graphics membership required")
    require(sources[str(ROOT/"learning/portable_e2e/map_crop_batch_worker_20261008_v3.py")]
        == "882f3a5fdd2eadfb1f48cabd8b87a3b3b9d9c03d0572dcd810ae326e152be456"
        and sources[str(ROOT/"tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py")] == POLICY_PIN
        and sources[str(ROOT/"tools/e2e/map_appearance_matched_contract_20261001.py")]
        == "9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96",
        "unchanged worker V3/original crop/policy pins differ")
    require(plan["environment"]["complete_transitive_provider_closure_claimed"] is False
        and plan["environment"]["historical_environment_equivalence_claimed"] is False
        and plan["diagnostic_graphics_environment"]["complete_transitive_graphics_provider_closure_claimed"] is False,
        "selected provider seals cannot assert transitive/historical equivalence")
    selected, first16_sha = original_selected(audit)
    require(same(selected, plan["selected_baseline_rows"]), "original20k metadata FIRST16 join differs")
    selection = decode(audit.read(SELECTION, SELECTION_PIN, True, 32*1024))
    scope = check_selection(selection, selected, first16_sha)
    auxiliary = merge_seals(selection["source_RAW_sha256"], selection["metadata_RAW_sha256"],
        {selection["old_v1_RAW_seal"]["path"]: selection["old_v1_RAW_seal"]["sha256"]})
    require(len(auxiliary) == 6 and all(Path(path).is_relative_to(ROOT) for path in auxiliary)
        and same(auxiliary, plan["auxiliary_metadata_sha256"]) and same(scope, plan["selection_scope"]),
        "selection source/result/legacy RAW provenance join differs")
    base_seals = merge_seals(sources, INPUTS, auxiliary, providers, graphics)
    archives = {str(PLAN_DIR/"sources"/(pin+".py")): pin for pin in sources.values()}
    all_producer_seals = merge_seals(base_seals, archives)
    for path, pin in all_producer_seals.items():
        audit.read(path, pin)
    result = decode(audit.read(OUT/"result.json", metadata=True))
    require(result["schema"] == PRODUCER_SCHEMA+".result" and result["ROOT_plan_sha256"] == plan_pin
        and result["ROOT_authority_sha256"] == grant_pin and same(result["declared_steps"], STEPS)
        and same(result["original20k_batches_outside_declaration_per_arm"], 19984)
        and same(result["fixed_sequential_arm_order"], list(ARMS)), "result ROOT/scope/order binding differs")
    require(result["status"] in ("COMPLETE_BYTES_ACTUAL_RESCUE_OBSERVED", "COMPLETE_BYTES_RESCUE_NOT_OBSERVED")
        and result["failure"] is None and same(result["cleanup_errors"], []) and result["full_failed_tail"] is None,
        "original incomplete/failed evidence retained; independent check cannot call it complete")
    require(all(same(result[k], 0) for k in ("training_steps", "NN", "formal_main_approved", "formal_sub_approved"))
        and result["actual_array_diagnostic_only"] is True and result["partial_prefix_decode_not_only_selected_rows"] is True
        and all(result[k] is False for k in ("architecture_changed", "model_performance_improved",
            "historical_provider_equivalence_claimed", "controlled_OS_cold_start_claimed",
            "old8_batch74second_run_used_as_comparable_timing_baseline", "original_journals_or_frozen_trainer_modified")),
        "actual result scope/model/time claims differ")
    require(all(same(result["comparison_"+k], value) for k, value in
        dict(PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0).items()), "full16 comparison denominator differs")
    memory = result["sampled_closing_memory"]
    require(type(memory) is dict and all(integer(memory[k], 1, RSS) for k in ("VmRSS", "VmHWM"))
        and integer(result["sampled_maximum_VmHWM_bytes"], memory["VmHWM"], RSS)
        and seconds(result["elapsed_seconds"]) and seconds(result["post_arm_comparison_seconds"])
        and integer(result["owner_budget_guard_calls_total"], 1),
        "producer sampled memory/deadline witness differs")
    require(same(result["upfront_seal_sha256"], base_seals)
        and same(result["upfront_source_archive_sha256"], archives)
        and same(result["closing_seal_sha256"], all_producer_seals)
        and same(result["closing_source_archive_sha256"], archives), "producer upfront/closing RAW seal witness differs")
    expected_observations = {path: dict(path=path, expected_sha256=pin, observed_sha256=pin, status="MATCHED")
        for path, pin in all_producer_seals.items()}
    observations = result["closing_seal_observations"]
    require(type(observations) is list and len(observations) == len(expected_observations)
        and all(type(row) is dict and row.get("path") in expected_observations for row in observations)
        and len({row["path"] for row in observations}) == len(observations)
        and all(same(row, expected_observations[row["path"]]) for row in observations), "closing MATCHED observation join differs")
    arms = result["arms"]
    require(type(arms) is dict and set(arms) == set(ARMS), "exact same-V3 two-arm results required")
    journals, rescued_by_arm, native_map = {}, {}, {}
    for index, name in enumerate(ARMS):
        enabled, arm = index == 1, arms[name]
        require(all(same(arm[k], value) for k, value in dict(PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0,
            expected_batches=16, expected_slot_exposures=128).items()), "per-arm16 full failed-attempt denominator differs")
        require(arm["reader_mode"] == ("same_V3_cache_enabled" if enabled else "same_V3_cache_disabled")
            and same(arm["reader_closing_observations"], [])
            and all(seconds(arm[k]) for k in ("startup_seconds", "close_seconds", "measured_arm_seconds"))
            and integer(arm["owner_budget_guard_calls_during_arm"], 1), "arm reader/runtime/cleanup witness differs")
        raw = audit.read(OUT/(name+"_RGB_GT_batches.jsonl"), metadata=True, maximum=JOURNAL_LIMIT)
        lines = raw.splitlines(keepends=True)
        require(len(lines) == 16 and all(line.endswith(b"\n") and len(line) <= 16*1024 for line in lines),
            "exact16 complete saved B8 records required")
        receipts = [decode(line) for line in lines]
        require(type(arm["batches"]) is list and len(arm["batches"]) == 16, "sixteen result batch measurements required")
        rescues = []
        for receipt, actual, expected in zip(receipts, arm["batches"], selected):
            row = check_batch(receipt, expected)
            rescues.extend(check_actual_batch(actual, row, expected["step"]))
            for native in receipt["native_slots"]:
                value = [native["native_RGB_sha256"], native["native_target_sha256"]]
                require(native["slot"] not in native_map or same(native_map[native["slot"]], value),
                    "same native slot reports different RGB/GT SHA")
                native_map[native["slot"]] = value
        require(same(arm["actual_rescue_witnesses"], rescues) and same(arm["rescued_slots"], len(rescues)),
            "all actual rescue support/box occurrence joins differ")
        for hint in selection["candidates"]:
            row = receipts[hint["step"]-1]["journal"]
            require(row["full_GT_sign_pixels"][hint["position_zero_based"]] == hint["fullGT_sign_pixels"],
                "metadata hint fullGT support differs from actual producer claim")
        check_stats(arm["cache_stats"], enabled)
        check_stats(arm["reader_cache_stats_after_close"], enabled, True)
        require(arm["cache_stats"]["total_selected_payload_bytes"] == 128*921600,
            "all128 native RGB/GT selected payload accounting differs")
        check_last_slot(arm["last_slot_read"], selected[-1], enabled)
        require(same(arm["last_slot_read"]["checkpoint_cache"], arm["cache_stats"]),
            "last actual slot and complete-arm cache snapshot differ")
        before, after = arm["cache_stats"], arm["reader_cache_stats_after_close"]
        changing = {"current_checkpoint_states", "cache_accounted_bytes", "owner_checkpoint_calls", "archive_generation_guard_calls"}
        require(all(same(before[k], after[k]) for k in before if k not in changing)
            and after["owner_checkpoint_calls"] == before["owner_checkpoint_calls"]+4
            and after["archive_generation_guard_calls"] == before["archive_generation_guard_calls"]+2,
            "close changed historical cache/inflation accounting or reversed guard counters")
        journals[name], rescued_by_arm[name] = receipts, rescues
    require(same(journals[ARMS[0]], journals[ARMS[1]]), "both arms complete128 native/byte/policy journal metadata differ")
    expected_comparisons = [dict(step=receipt["step"], candidate_RGB_equal=True, candidate_GT_equal=True,
        same_original_and_selected_boxes_and_counts=True, candidate_RGB_sha256=receipt["journal"]["candidate_RGB_sha256"],
        candidate_target_sha256=receipt["journal"]["candidate_target_sha256"]) for receipt in journals[ARMS[0]]]
    require(same(result["comparisons"], expected_comparisons), "full16 same candidate byte comparison join differs")
    observed = bool(rescued_by_arm[ARMS[0]])
    require(result["actual_rescue_observed"] is observed and result["rescue_policy_effect_on_actual_RGB_GT_observed"] is observed
        and result["status"] == ("COMPLETE_BYTES_ACTUAL_RESCUE_OBSERVED" if observed else "COMPLETE_BYTES_RESCUE_NOT_OBSERVED"),
        "actual positive rescue observed/not-observed status differs")
    png = result["diagnostic_PNG"]
    png_check = None
    if observed:
        first = rescued_by_arm[ARMS[0]][0]
        native = journals[ARMS[0]][first["step"]-1]["native_slots"][first["position"]]
        require(png["status"] == "COMPLETE_ACTUAL_RECORDED_INPUT_CROP_DIAGNOSTIC"
            and same(png["step"], first["step"]) and same(png["position"], first["position"])
            and png["path"] == "first_actual_rescue.png" and sha(png["encoded_sha256"])
            and integer(png["encoded_bytes"], 1, PNG_LIMIT) and same(png["maximum_bytes"], PNG_LIMIT)
            and png["semantic_GT_signmask_not_model_prediction"] is True
            and all(png[k] is False for k in ("actual_CARLA_or_RViz_screenshot", "ego_or_route_observed", "invented_ego_or_route_drawn")),
            "first actual rescue diagnostic PNG metadata differs")
        require(png["native_RGB_sha256"] == native["native_RGB_sha256"]
            and png["native_target_sha256"] == native["native_target_sha256"]
            and sha(png["proposed_RGB_sha256"]) and sha(png["proposed_target_sha256"])
            and same(png["native_GT_sign_pixels"], first["full_GT_sign_pixels"])
            and same(png["proposed_GT_sign_pixels"], 0)
            and same(png["rescued_GT_sign_pixels"], first["selected_NEAREST_GT_sign_pixels"])
            and png["actual_sample_bytes_and_counts_match_rescue_witness"] is True,
            "PNG sample native SHA and native/proposed/rescued GT support witness differ")
        png_raw = audit.read(OUT/"first_actual_rescue.png", png["encoded_sha256"], True, PNG_LIMIT)
        require(len(png_raw) == png["encoded_bytes"], "saved PNG encoded length differs")
        png_check = png_container(png_raw)
    else:
        require(same(png, dict(status="NOT_CREATED_RESCUE_NOT_OBSERVED", actual_rescue_observed=False))
            and not (OUT/"first_actual_rescue.png").exists(), "no-rescue result cannot claim diagnostic PNG")
    audit.close_seals()
    return dict(status="COMPLETE_INDEPENDENT_SAVED_METADATA_AND_BYTE_SEALS_ONLY", original_result_status=result["status"],
        declared_batches_per_arm=16, native_receipts_per_arm=128, native_receipts_both_arms=256,
        PASS_batches_per_arm=16, FAIL_batches_per_arm=0, UNEXECUTED_batches_per_arm=0,
        original20k_metadata_rows=20000, original20k_numeric_batches_independently_replayed=0,
        original20k_batches_outside_declaration_per_arm=19984, rescued_slots_per_arm=len(rescued_by_arm[ARMS[0]]),
        actual_rescue_claims_independently_joined_as_metadata=observed, candidate_pixel_hashes_independently_recomputed=False,
        producer_actual_pixel_claims_checked_as_metadata=True, last_slot_stream_histories_per_arm=1,
        all128_native_stream_CRC_histories_independently_checked=False, source_files=18, source_archives=18,
        compressed_and_metadata_inputs=8, selected_scientific_providers=6, selected_graphics_sources=len(graphics),
        PNG_container_check=png_check, native_decoder_allocation_measured_by_checker=False,
        model_performance_improved=False, original_result_failure=result["failure"], original_closing_errors=result["cleanup_errors"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    for name in ("plan-sha256", "authority-sha256", "script-sha256", "test-source-sha256"):
        parser.add_argument("--"+name, required=True)
    args = parser.parse_args()
    pins = [args.plan_sha256, args.authority_sha256, args.script_sha256, args.test_source_sha256]
    require(args.execute and all(sha(pin) for pin in pins), "explicit ROOT execution and external RAW pins required")
    require(sys.flags.dont_write_bytecode and sys.flags.optimize == 0, "clean non-bytecode checker required")
    canonical(RECEIPT, False)
    require(not RECEIPT.exists(), "fresh independent receipt required; no overwrite/retry")
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        audit = Audit(lease)
        try:
            try:
                audit.guard()
                outcome = inspect(audit, *pins)
            except Exception as error:
                outcome = dict(status="FAILED_INDEPENDENT_SAVED_CHECK_RETAINED", error_type=type(error).__name__,
                    error=str(error)[:4096], original_evidence_changed_or_deleted_by_checker=False)
            receipt = dict(schema=SCHEMA, provenance="HH_261008 - Independent stored metadata and RAW seals; no second pixel replay or model approval.",
                ROOT_plan_RAW_sha256=pins[0], ROOT_authority_RAW_sha256=pins[1], checker_source_sha256=pins[2],
                scalar_test_source_sha256=pins[3], result=outcome, independently_read_file_seals=audit.files,
                sampled_maximum_VmHWM_bytes=audit.maximum_HWM, elapsed_seconds=time.monotonic()-audit.started,
                sampled_guards_not_OS_containment=True, pixel_SHA_second_recomputations=0,
                array_decodes=0, PNG_IDAT_inflations=0, NN=0, training_steps=0, GPU=0, SSH=0,
                learned_control=False, actual_CARLA_capture=False, formal_main_approved=0, formal_sub_approved=0)
            raw = encoded(receipt)
            require(len(raw) <= MAX_METADATA, "bounded independent metadata receipt required")
            with RECEIPT.open("xb") as output:
                require(output.write(raw) == len(raw), "partial independent receipt write")
                output.flush()
            print(encoded(dict(status=outcome["status"], receipt=str(RECEIPT), bytes=len(raw))).decode(), end="")
            return 0 if outcome["status"].startswith("COMPLETE_") else 2
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
