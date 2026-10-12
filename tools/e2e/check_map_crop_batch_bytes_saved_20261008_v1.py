"""HH_261008 - Independently check saved limited RGB/GT evidence without array replay.

SOURCE_ONLY until ROOT separately reviews this entire file and authorizes its
explicit --execute command. This checker imports only the standard library. It
hashes compressed input files but never opens a ZIP member, imports a producer,
decodes an image/array, runs a model, trains, connects or changes old evidence.
The receipt verifies stored metadata, joins and byte seals; candidate pixel
hashes are producer evidence, not independently recomputed pixel hashes here.
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
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.map_crop_batch_bytes_saved_check.20261008.v1"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08/map_crop_batch_bytes_v3"
OUT = ROOT / "storage/experiments/diagnostics/2026-10-08/map_crop_batch_bytes_v3"
RECEIPT = OUT.parent / "map_crop_batch_bytes_v3_independent_v1.json"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
PLAN_SHA = "50a67978455e710ce451f3e0fff08f6bd4c213a780cdc66fbd66eba32b47c7ef"
AUTHORITY_SHA = "025d13950fa95b1dbc015425952450a0c28c75148cb618e5fe352bf2a2036b0e"
REVIEW_SHA = "bff62d0150eaf40383429ad5ec35f026cddb9ee5b1487d5ae3baec635bbf770d"
STEPS = [1, 2, 50, 100, 500, 1000, 10000, 20000]
MAX_METADATA, BLOCK = 512 * 1024, 64 * 1024
FLOOR, RESERVE, RSS, DEADLINE = 2 * 1024**3, 64 * 1024**2, 512 * 1024**2, 900
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
}
DATA = ROOT / "storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
INPUTS = {
    str(DATA / "original_train729.npz"): "d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3",
    str(DATA / "paired_train480.npz"): "47c23a3f6fe67aa40ea7f5bb0c770fd6037463ed090f2b9390a89317c4dedee3",
    str(RUN / "control/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN / "candidate/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN / "inputs/manifest.json"): "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b",
    str(RUN / "inputs/study_train_registry.json"): "ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d",
    str(RUN / "inputs/paired_samples.json"): "899d82128ae9597beca6e22fbad4abaaff40c7e1fbc502418f946f71c72b9d6b",
}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def integer(value, minimum=0, maximum=2**63 - 1):
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
        parse_constant=lambda _: require(False, "nonfinite JSON constant"))


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def canonical(path, exists=True):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path == path.resolve(strict=exists)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical nonsymlink path required")
    return path


def generation(info):
    return [getattr(info, field) for field in ("st_mode", "st_uid", "st_gid", "st_nlink",
        "st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")]


class Audit:
    def __init__(self, lease):
        self.lease, self.started = lease, time.monotonic()
        self.files, self.maximum_HWM = {}, 0

    def guard(self):
        path, fd = canonical(LOCK).stat(), os.fstat(self.lease.fileno())
        require(stat.S_ISREG(path.st_mode) and path.st_nlink == 1 and generation(path) == generation(fd),
            "held shared lease FD/path differs")
        memory = {line.split(":")[0]: int(line.split()[1]) * 1024
            for line in Path("/proc/self/status").read_text().splitlines()
            if line.startswith(("VmRSS:", "VmHWM:"))}
        self.maximum_HWM = max(self.maximum_HWM, memory["VmHWM"])
        require(self.maximum_HWM <= RSS and time.monotonic() - self.started < DEADLINE,
            "sampled checker memory/deadline exceeded")
        fs = os.statvfs(ROOT)
        require(fs.f_bavail * fs.f_frsize >= FLOOR + RESERVE, "unchanged floor plus reserve unavailable")

    def read(self, path, expected=None, metadata=False):
        path = canonical(path)
        before = path.stat()
        require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1, "regular single-link evidence required")
        require(before.st_size <= (MAX_METADATA if metadata else 1024**3), "bounded evidence file required")
        digest, chunks = hashlib.sha256(), []
        with path.open("rb") as stream:
            opened = os.fstat(stream.fileno())
            for block in iter(lambda: stream.read(BLOCK), b""):
                self.guard()
                digest.update(block)
                if metadata:
                    chunks.append(block)
            closed = os.fstat(stream.fileno())
        after = path.stat()
        observed = generation(before)
        require(observed == generation(opened) == generation(closed) == generation(after), "evidence generation changed")
        value = digest.hexdigest()
        require(expected is None or value == expected, "evidence RAW SHA differs: " + str(path))
        witness = dict(sha256=value, bytes=before.st_size, generation=observed)
        if str(path) in self.files:
            require(same(witness, self.files[str(path)]), "evidence changed between independent reads")
        self.files[str(path)] = witness
        return b"".join(chunks) if metadata else value

    def close_seals(self):
        # HH_261008 - Independently reread every admitted compressed/input/source/output byte seal.
        for path, witness in list(self.files.items()):
            self.read(path, witness["sha256"])


BASELINE_FIELDS = {"step", "physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits", "RGB_sha256", "target_sha256"}


def box(value):
    require(type(value) is list and len(value) == 4 and all(integer(v, 0, 640) for v in value), "strict crop box required")
    top, left, height, width = value
    require(height > 0 and width > 0 and top + height <= 360 and left + width <= 640, "crop exceeds native bounds")


def baseline(row, step):
    require(type(row) is dict and set(row) == BASELINE_FIELDS and integer(row["step"], 1, 20000)
        and row["step"] == step, "exact original baseline step/fields required")
    require(type(row["physical_slot_indices"]) is list and len(row["physical_slot_indices"]) == 8
        and all(integer(v, 0, 1208) for v in row["physical_slot_indices"]), "eight exact native slots required")
    require(type(row["shared_crop_boxes"]) is list and len(row["shared_crop_boxes"]) == 8, "eight original boxes required")
    for value in row["shared_crop_boxes"]:
        box(value)
    require(type(row["actual_paired_TRUE_bits"]) is list and len(row["actual_paired_TRUE_bits"]) == 8
        and all(type(v) is int and v == 0 for v in row["actual_paired_TRUE_bits"]), "FALSE-only bits required")
    require(sha(row["RGB_sha256"]) and sha(row["target_sha256"]), "original two byte hashes required")


def original_selected(audit):
    selected, count = [], 0
    path = canonical(RUN / "control/exposures.jsonl")
    with path.open("rb") as stream:
        while True:
            raw = stream.readline(4097)
            if not raw:
                break
            audit.guard()
            require(len(raw) <= 4096 and raw.endswith(b"\n"), "bounded original full journal line required")
            count += 1
            require(count <= 20000, "extra original metadata row")
            row = decode(raw)
            baseline(row, count)
            if count in STEPS:
                selected.append(dict(step=count, RAW_line_sha256=hashlib.sha256(raw).hexdigest(), baseline=row))
    require(count == 20000 and [row["step"] for row in selected] == STEPS, "full original20k metadata tail required")
    audit.read(path, INPUTS[str(path)])
    return selected


def check_batch(receipt, expected):
    receipt_fields = {"schema", "step", "journal", "native_slots", "actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT",
        "source_and_baseline_RAW_authenticated_by_worker", "training_steps", "NN"}
    require(type(receipt) is dict and set(receipt) == receipt_fields
        and receipt.get("schema") == "portable_e2e.map_crop_batch_worker.20261008.v3.batch"
        and type(receipt.get("step")) is int and receipt["step"] == expected["step"], "candidate outer chronology differs")
    row = receipt["journal"]
    journal_fields = {"schema", "policy_source_sha256", "baseline_exposure", "selected_crop_boxes",
        "full_GT_sign_pixels", "original_NEAREST_sign_pixels", "protected", "rescued",
        "candidate_RGB_sha256", "candidate_target_sha256"}
    require(type(row) is dict and set(row) == journal_fields
        and row["schema"] == "portable_e2e.map_crop_preservation_journal.20261008.v1"
        and row["policy_source_sha256"] == "566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f"
        and same(row["baseline_exposure"], expected["baseline"]), "candidate original baseline/policy join differs")
    baseline(row["baseline_exposure"], expected["step"])
    require(sha(row["candidate_RGB_sha256"]) and sha(row["candidate_target_sha256"]), "candidate byte hashes invalid")
    fields = ("selected_crop_boxes", "full_GT_sign_pixels", "original_NEAREST_sign_pixels", "protected", "rescued")
    require(all(type(row[k]) is list and len(row[k]) == 8 for k in fields), "eight candidate policy witnesses required")
    slots = receipt["native_slots"]
    require(type(slots) is list and len(slots) == 8, "eight actual native hash receipts required")
    for position, native in enumerate(slots):
        require(type(native) is dict and set(native) == {"position", "slot", "native_RGB_sha256",
            "native_target_sha256", "reader_provenance_authenticated_by_worker"}
            and type(native["position"]) is int and native["position"] == position
            and type(native["slot"]) is int and native["slot"] == expected["baseline"]["physical_slot_indices"][position]
            and sha(native["native_RGB_sha256"]) and sha(native["native_target_sha256"])
            and native["reader_provenance_authenticated_by_worker"] is False, "actual native slot/hash join differs")
        full, proposed = row["full_GT_sign_pixels"][position], row["original_NEAREST_sign_pixels"][position]
        require(integer(full, 0, 360*640) and integer(proposed, 0, 360*640), "strict GT support metadata required")
        protected = ((expected["step"] - 1) * 8 + position) % 2 == 0
        rescued = protected and full > 0 and proposed == 0
        require(row["protected"][position] is protected and row["rescued"][position] is rescued, "crop rescue metadata differs")
        chosen = [0, 0, 360, 640] if rescued else expected["baseline"]["shared_crop_boxes"][position]
        box(row["selected_crop_boxes"][position])
        require(same(row["selected_crop_boxes"][position], chosen), "RGB/GT shared selected box differs")
    if not any(row["rescued"]):
        require(row["candidate_RGB_sha256"] == expected["baseline"]["RGB_sha256"]
            and row["candidate_target_sha256"] == expected["baseline"]["target_sha256"], "unchanged batch hashes differ")
    require(all(receipt[k] is True for k in ("actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT"))
        and receipt["source_and_baseline_RAW_authenticated_by_worker"] is False
        and type(receipt["NN"]) is type(receipt["training_steps"]) is int
        and receipt["NN"] == receipt["training_steps"] == 0, "worker scope metadata differs")
    return dict(step=expected["step"], native_receipts=8, rescued_slots=sum(row["rescued"]),
        candidate_RGB_sha256=row["candidate_RGB_sha256"], candidate_target_sha256=row["candidate_target_sha256"])


def check_last_slot(expected, witness):
    """HH_261008 - Join only the recorded last slot, never all64 stream/CRC histories."""
    read = witness["last_slot_read"]
    require(same(witness["step"], expected["step"])
        and same(read["slot"], expected["baseline"]["physical_slot_indices"][-1])
        and read["rgb_true_decoded"] is False and read["caller_admission_authenticated_by_reader"] is False
        and read["closing_whole_file_SHA_checked"] is False
        and type(read["cache_payload_bytes"]) is int and read["cache_payload_bytes"] == 0
        and type(read["scratch_limit_bytes"]) is int and read["scratch_limit_bytes"] == 1024**2,
        "last-slot scope witness differs")
    slot = read["slot"]
    index, count, rgb_name, package = ((slot, 729, "rgb.npy", DATA / "original_train729.npz") if slot < 729
        else (slot - 729, 480, "rgb_false.npy", DATA / "paired_train480.npz"))
    require(read["package_path"] == str(package) and read["package_sha256"] == INPUTS[str(package)]
        and type(read["members"]) is dict and set(read["members"]) == {rgb_name, "target.npy"},
        "last native package/FALSE member routing differs")
    for name, member in read["members"].items():
        pixels = 360*640*(3 if name == rgb_name else 1)
        require(type(member["selected_payload_bytes"]) is int and member["selected_payload_bytes"] == pixels
            and type(member["compression"]) is int and member["compression"] in (0, 8)
            and type(member["prefix_body_bytes_inflated"]) is int
            and member["prefix_body_bytes_inflated"] == (index*pixels if member["compression"] == 8 else 0)
            and member["member_EOF_and_trailer_checked"] is (index == count - 1)
            and member["whole_member_crc_verified"] is (index == count - 1 and member["compression"] == 8),
            "partial native prefix/CRC/EOF witness differs")


def inspect(audit, script_pin):
    audit.read(Path(__file__).resolve(), script_pin)
    raw_plan = audit.read(PLAN_DIR / "ROOT_plan.json", PLAN_SHA, True)
    raw_authority = audit.read(PLAN_DIR / "ROOT_authority.json", AUTHORITY_SHA, True)
    audit.read(PLAN_DIR / "ROOT_execution_review_v1.json", REVIEW_SHA, True)
    require(audit.read(PLAN_DIR / "candidate_plan.json", PLAN_SHA, True) == raw_plan, "prepared RAW plan copy differs")
    require(audit.read(OUT / "ROOT_plan.json", PLAN_SHA, True) == raw_plan
        and audit.read(OUT / "ROOT_authority.json", AUTHORITY_SHA, True) == raw_authority, "RAW execution copies differ")
    plan, grant = decode(raw_plan), decode(raw_authority)
    require(plan["schema"] == "portable_e2e.map_crop_batch_bytes.20261008.v3"
        and same(plan["contract"]["declared_steps"], STEPS) and same(plan["input_sha256"], INPUTS)
        and plan["output"] == str(OUT), "exact limited ROOT plan scope differs")
    require(grant["schema"] == "portable_e2e.map_crop_batch_bytes.20261008.v3.ROOT_authority"
        and grant["plan_sha256"] == PLAN_SHA and grant["limited_original_and_FALSE_RGB_GT_verification_authorized"] is True
        and grant["old_target_only_authority_extended"] is False and grant["training_authorized"] is False
        and grant["NN_authorized"] is False, "ROOT authority scope differs")
    sources, providers = plan["source_sha256"], plan["environment"]["selected_provider_sha256"]
    require(type(sources) is dict and set(sources) == {str(ROOT / path) for path in SOURCE_NAMES}
        and type(providers) is dict and len(providers) == 6, "exact14 sources and6 selected providers required")
    for path, pin in sources.items():
        require(sha(pin), "source SHA metadata invalid")
        audit.read(path, pin)
        audit.read(PLAN_DIR / "sources" / (pin + ".py"), pin)
    for group in (INPUTS, providers):
        for path, pin in group.items():
            require(sha(pin), "input/provider SHA metadata invalid")
            audit.read(path, pin)
    selected = original_selected(audit)
    require(same(selected, plan["selected_baseline_rows"]), "original20k selected8 RAW metadata join differs")
    result = decode(audit.read(OUT / "result.json", metadata=True))
    require(result["schema"] == "portable_e2e.map_crop_batch_bytes.20261008.v3.result"
        and result["ROOT_plan_sha256"] == PLAN_SHA and result["ROOT_authority_sha256"] == AUTHORITY_SHA,
        "actual result ROOT binding differs")
    counts = {key: result[key] for key in ("PASS_batches", "FAIL_batches", "UNEXECUTED_batches")}
    require(all(integer(value, 0, 8) for value in counts.values()) and sum(counts.values()) == 8
        and same(result["declared_steps"], STEPS) and type(result["original20k_batches_outside_declaration"]) is int
        and result["original20k_batches_outside_declaration"] == 19992, "actual full limited denominator differs")
    # HH_261008 - A failed or incomplete original result remains evidence; never report it as completed8.
    require(result["status"] == "COMPLETE_LIMITED_RGB_GT_BYTES_NOT_TRAINING_OR_MODEL_GAIN"
        and same(counts, dict(PASS_batches=8, FAIL_batches=0, UNEXECUTED_batches=0))
        and result["failure"] is None and result["cleanup_errors"] == [], "original run not complete; failure/tail retained")
    require(all(type(result[k]) is int and result[k] == 0 for k in
        ("training_steps", "NN", "formal_main_approved", "formal_sub_approved"))
        and result["actual_array_diagnostic_only"] is True
        and all(result[k] is False for k in ("model_performance_improved", "historical_provider_equivalence_claimed",
            "original_journals_or_frozen_trainer_modified")), "actual result scope claims differ")
    memory = result["sampled_closing_memory"]
    require(type(memory) is dict and all(integer(memory[k], 1, RSS) for k in ("VmRSS", "VmHWM"))
        and integer(result["sampled_maximum_VmHWM_bytes"], memory["VmHWM"], RSS)
        and type(result["elapsed_seconds"]) in (int, float) and not isinstance(result["elapsed_seconds"], bool)
        and math.isfinite(result["elapsed_seconds"]) and 0 <= result["elapsed_seconds"] < DEADLINE,
        "result closing memory/deadline witness differs")
    require(same(result["declared_thread_environment"], {k: "1" for k in
        ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")})
        and result["partial_prefix_decode_not_only_selected_rows"] is True, "thread/prefix scope witness differs")
    raw_journal = audit.read(OUT / "candidate_RGB_GT_batches.jsonl", metadata=True)
    lines = raw_journal.splitlines(keepends=True)
    require(len(lines) == 8 and all(line.endswith(b"\n") and len(line) <= 16*1024 for line in lines), "exact8 complete candidate records required")
    checked = [check_batch(decode(line), expected) for line, expected in zip(lines, selected)]
    expected_comparisons = [dict(step=row["step"], original_RGB_GT_equal=True,
        rescued_slots=row["rescued_slots"], candidate_RGB_sha256=row["candidate_RGB_sha256"],
        candidate_target_sha256=row["candidate_target_sha256"]) for row in checked]
    require(same(result["selected_batch_comparisons"], expected_comparisons), "result/journal complete8 join differs")
    reads = result["last_slot_prefix_read_witnesses"]
    require(type(reads) is list and len(reads) == 8, "eight declared last-slot witnesses required")
    for expected, witness in zip(selected, reads):
        check_last_slot(expected, witness)
    audit.close_seals()
    return dict(status="COMPLETE_INDEPENDENT_SAVED_METADATA_AND_BYTE_SEALS_ONLY", declared_batches=8,
        PASS_batches=8, FAIL_batches=0, UNEXECUTED_batches=0, native_receipts=64,
        original20k_metadata_rows=20000, original20k_numeric_batches_independently_replayed=0,
        original20k_batches_outside_declaration=19992, checked_batches=checked,
        source_files=14, source_archives=14, compressed_inputs_and_metadata_inputs=7, selected_providers=6,
        candidate_pixel_hashes_independently_recomputed=False, producer_actual_pixel_claims_checked_as_metadata=True,
        original_result_status=result["status"], original_result_failure=result["failure"], original_closing_errors=result["cleanup_errors"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--script-sha256", required=True)
    args = parser.parse_args()
    require(args.execute and sha(args.script_sha256), "separate ROOT-authorized explicit execution required")
    require(sys.flags.dont_write_bytecode and sys.flags.optimize == 0, "clean non-bytecode checker required")
    canonical(RECEIPT, False)
    require(not RECEIPT.exists(), "fresh independent receipt required; no overwrite/retry")
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        audit = Audit(lease)
        try:
            audit.guard()
            try:
                outcome = inspect(audit, args.script_sha256)
            except Exception as failure:
                outcome = dict(status="FAILED_INDEPENDENT_SAVED_CHECK_RETAINED", error_type=type(failure).__name__,
                    error=str(failure)[:4096], original_evidence_changed_or_deleted_by_checker=False)
            receipt = dict(schema=SCHEMA, provenance="HH_261008 - Independent saved byte/metadata verification; no pixel replay or functionality approval.",
                ROOT_plan_RAW_sha256=PLAN_SHA, ROOT_authority_RAW_sha256=AUTHORITY_SHA,
                checker_source_sha256=args.script_sha256, result=outcome, independently_read_file_seals=audit.files,
                sampled_maximum_VmHWM_bytes=audit.maximum_HWM, elapsed_seconds=time.monotonic()-audit.started,
                sampled_guards_not_OS_containment=True, array_decodes=0, NN=0, training_steps=0,
                learned_control=False, actual_CARLA_capture=False, formal_main_approved=0, formal_sub_approved=0)
            raw = encoded(receipt)
            require(len(raw) <= MAX_METADATA, "bounded independent metadata receipt required")
            with RECEIPT.open("xb") as stream:
                require(stream.write(raw) == len(raw), "partial independent receipt write")
                stream.flush()
            print(encoded(dict(status=outcome["status"], receipt=str(RECEIPT), bytes=len(raw))).decode(), end="")
            return 0 if outcome["status"].startswith("COMPLETE_") else 2
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
