"""HH_261008 - Bind the genuine five-field native counts in fresh byte verification.

Prepare seals metadata and source only. Execute requires separate RAW ROOT plan
and authorization, an exclusive shared lease, CPU3, fixed budgets and providers.
Only approved original RGB/GT and paired FALSE RGB/GT members may be inflated.
The remaining19992 historical batches are OUT_OF_SCOPE, never claimed verified.
"""
from __future__ import annotations

import argparse
import ast
import fcntl
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
NAME = "map_crop_batch_bytes_v2"
SCHEMA = "portable_e2e.map_crop_batch_bytes.20261008.v2"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08" / NAME
OUT = ROOT / "storage/experiments/diagnostics/2026-10-08" / NAME
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
HELD_LEASE = None
DATA = ROOT / "storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
STEPS = [1, 2, 50, 100, 500, 1000, 10000, 20000]
NATIVE_COUNTS = dict(available_rgb_views=1689, logical_slots_per_arm=1209,
    new_rgb_views=960, original=729, physical_pairs=480)
FLOOR, RESERVE = 2 * 1024**3, 64 * 1024**2
LIMIT, CLOSING, RSS, DEADLINE = 2 * 1024**2, 512 * 1024, 512 * 1024**2, 900
INPUTS = {
    str(DATA / "original_train729.npz"): "d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3",
    str(DATA / "paired_train480.npz"): "47c23a3f6fe67aa40ea7f5bb0c770fd6037463ed090f2b9390a89317c4dedee3",
    str(RUN / "control/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN / "candidate/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN / "inputs/manifest.json"): "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b",
    str(RUN / "inputs/study_train_registry.json"): "ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d",
    str(RUN / "inputs/paired_samples.json"): "899d82128ae9597beca6e22fbad4abaaff40c7e1fbc502418f946f71c72b9d6b",
}
SOURCES = [
    "tools/e2e/verify_map_crop_batch_bytes_20261008_v2.py",
    "learning/tests/test_verify_map_crop_batch_bytes_20261008_v2.py",
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
]


def require(value, message):
    if not value:
        raise ValueError(message)


def canonical(path, exists=True):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(strict=exists)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical nonsymlink path required")
    return path


def generation(value):
    return [getattr(value, k) for k in ("st_mode", "st_uid", "st_gid", "st_nlink", "st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")]


def digest(path, checkpoint=lambda: None):
    path = canonical(path)
    before = path.stat()
    require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1, "single-link regular source required")
    h = hashlib.sha256()
    with path.open("rb") as stream:
        start = os.fstat(stream.fileno())
        for chunk in iter(lambda: stream.read(1024**2), b""):
            checkpoint()
            h.update(chunk)
        end = os.fstat(stream.fileno())
    require(generation(before) == generation(start) == generation(end) == generation(path.stat()), "file changed during hash")
    return h.hexdigest()


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def decode(raw):
    def pairs(rows):
        d = {}
        for key, value in rows:
            require(key not in d, "duplicate JSON key")
            d[key] = value
        return d
    def number(value):
        import math
        result = float(value)
        require(math.isfinite(result), "overflowing JSON float")
        return result
    return json.loads(raw, object_pairs_hook=pairs, parse_float=number,
        parse_constant=lambda _: require(False, "nonfinite JSON token"))


def typed_equal(left, right):
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(typed_equal(left[k], right[k]) for k in left)
    if type(left) is list:
        return len(left) == len(right) and all(typed_equal(a, b) for a, b in zip(left, right))
    return left == right


def lease_guard():
    require(HELD_LEASE is not None and not HELD_LEASE.closed, "actual held runtime lease required")
    path, fd = canonical(LOCK).stat(), os.fstat(HELD_LEASE.fileno())
    require(stat.S_ISREG(path.st_mode) and path.st_nlink == fd.st_nlink == 1
        and generation(path) == generation(fd), "held lease FD/path generation differs")
    return generation(fd)


def process_identity():
    fields = Path("/proc/self/stat").read_text().rsplit(")", 1)[1].split()
    return dict(PID=os.getpid(), PPID=os.getppid(), start_ticks=int(fields[19]),
        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(), uid=os.getuid(),
        process_group=os.getpgrp(), session=os.getsid(0),
        cmdline_sha256=hashlib.sha256(Path("/proc/self/cmdline").read_bytes()).hexdigest())


def owned_bytes():
    total = 0
    for folder in (PLAN_DIR, OUT):
        canonical(folder, False)
        if folder.exists():
            for path in folder.rglob("*"):
                canonical(path)
                if path.is_file():
                    require(path.stat().st_nlink == 1, "owned output hardlink denied")
                    total += path.stat().st_size
                else:
                    require(path.is_dir(), "owned special file denied")
    return total


def write_new(path, raw, closing=False):
    canonical(path, False)
    require(type(raw) is bytes and len(raw) <= CLOSING
        and (path.is_relative_to(PLAN_DIR) or path.is_relative_to(OUT)), "bounded own metadata required")
    require(owned_bytes() + len(raw) <= LIMIT - (0 if closing else CLOSING), "unchanged combined2MiB quota exceeded")
    with path.open("xb") as output:
        require(output.write(raw) == len(raw), "partial metadata write")


class Budget:
    def __init__(self):
        self.started = time.monotonic()
        self.maximum_HWM = 0
        self.last = None

    def guard(self):
        lease_guard()
        raw = Path("/proc/self/status").read_text()
        values = {line.split(":")[0]: int(line.split()[1]) * 1024 for line in raw.splitlines()
            if line.startswith(("VmRSS:", "VmHWM:"))}
        self.maximum_HWM = max(self.maximum_HWM, values["VmHWM"])
        self.last = values
        require(self.maximum_HWM <= RSS, "sampled512MiB self VmHWM exceeded")
        require(time.monotonic() - self.started < DEADLINE, "sampled900s deadline exceeded")
        v = os.statvfs(ROOT)
        require(v.f_bavail * v.f_frsize >= FLOOR + RESERVE, "2GiB floor plus64MiB reserve unavailable")
        require(owned_bytes() <= LIMIT - CLOSING, "512KiB closing space required")


def environment():
    require(sys.flags.dont_write_bytecode and sys.flags.optimize == 0 and os.sched_getaffinity(0) == {3}
        and os.environ.get("CUDA_VISIBLE_DEVICES") == "" and os.environ.get("PYTHONNOUSERSITE") == "1",
        "clean -B CPU3-only environment required")
    require(not any(os.environ.get(k) for k in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP")), "foreign Python paths denied")
    require(not any(k in sys.modules for k in ("torch", "carla", "rclpy", "tensorflow")), "model/simulator imports denied")
    require(all(os.environ.get(k) == "1" for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")),
        "declared thread1 provider environment required")
    paths = {}
    for name in ("numpy", "PIL"):
        spec = importlib.util.find_spec(name)
        require(spec and spec.origin, "existing numeric/image provider unavailable")
        paths[name] = str(canonical(spec.origin))
    ndir, pdir = Path(paths["numpy"]).parent, Path(paths["PIL"]).parent
    native = list((ndir / "core").glob("_multiarray_umath*.so")) or list((ndir / "_core").glob("_multiarray_umath*.so"))
    imaging = list(pdir.glob("_imaging*.so"))
    require(len(native) == len(imaging) == 1, "exact selected native providers required")
    selected = [*paths.values(), str(native[0]), str(imaging[0]), str(pdir / "Image.py"), str(Path(sys.executable).resolve())]
    return dict(origins=paths, versions={n: importlib.metadata.version(n) for n in ("numpy", "Pillow")},
        selected_provider_sha256={p: digest(p) for p in selected}, complete_transitive_provider_closure_claimed=False,
        historical_environment_equivalence_claimed=False)


def baseline_rows():
    from portable_e2e import map_crop_preservation_journal_20261008_v1 as journal
    selected = []
    with canonical(RUN / "control/exposures.jsonl").open("rb") as stream:
        count = 0
        while True:
            raw = stream.readline(4097)
            if not raw:
                break
            require(len(raw) <= 4096 and raw.endswith(b"\n"), "bounded complete original journal line required")
            count += 1
            row = decode(raw)
            journal.validate_baseline(row, count)
            if count in STEPS:
                selected.append(dict(step=count, RAW_line_sha256=hashlib.sha256(raw).hexdigest(), baseline=row))
    require(count == 20000 and [r["step"] for r in selected] == STEPS, "whole20k metadata and fixed eight rows required")
    return selected


def registration():
    manifest = decode(canonical(RUN / "inputs/manifest.json").read_bytes())
    registry = decode(canonical(RUN / "inputs/study_train_registry.json").read_bytes())
    require(manifest["schema"] == "portable_e2e.map_paired_balanced_data.20261002.v1"
        and registry["schema"] == "portable_e2e.map_paired_balanced_research_registration.20261002.v1"
        and manifest["study_id"] == registry["study_id"] == "map_paired_balanced_20261002_v1", "original package registration binding differs")
    require(all(doc[k] is False for doc in (manifest, registry) for k in
        ("ctrack_train_eligible", "dev_or_test_used_for_fit", "vehicle_control_approved"))
        and all(doc["native_diagnostic_denials_preserved"] is True for doc in (manifest, registry))
        and manifest["source_receipts_modified"] is False and registry["broad_registry_modified"] is False,
        "existing excluded-source and evaluation roles must remain unchanged")
    require(manifest["study_registry_sha256"] == INPUTS[str(RUN / "inputs/study_train_registry.json")]
        and manifest["paired_samples_sha256"] == INPUTS[str(RUN / "inputs/paired_samples.json")]
        and typed_equal(registry["data_files"], manifest["files"])
        and typed_equal(manifest["counts"], NATIVE_COUNTS), "original study metadata differs")
    captures = registry["source_captures"]
    for capture in captures.values() if type(captures) is dict else captures:
        require(capture["raw_source_dataset_admitted"] is False and capture["raw_source_train_eligible"] is False
            and capture["all_240_retained"] is True and capture["independent_new_site"] is False,
            "source captures remain denied RAW; package verification must not admit them")
    return dict(study_id=manifest["study_id"], original_research_fit_authorization_not_reused=True,
        raw_source_denials_preserved=True, existing_registration_files_modified=False)


def fixed():
    return dict(declared_steps=STEPS, expected_batches=8, expected_slot_exposures=64,
        original20k_batches_outside_declaration=19992, selected_metadata_not_random_or_independent_TEST=True,
        allowed_original_members=["rgb.npy", "target.npy"], allowed_paired_members=["rgb_false.npy", "target.npy"],
        paired_TRUE_member_decode_authorized=False, baseline_both_RGB_and_GT_SHA_required=True,
        CPU_affinity=[3], threads=1, sampled_self_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        floor_bytes=FLOOR, reserve_bytes=RESERVE, maximum_combined_PLAN_OUTPUT_bytes=LIMIT, closing_bytes=CLOSING,
        sampled_guards_not_OS_containment=True, old_target_only_authority_reused=False,
        training_steps=0, NN=0, main_approved=0, sub_approved=0)


def authority(plan, pin):
    return dict(schema=SCHEMA + ".ROOT_authority", plan_path=str(PLAN_DIR / "ROOT_plan.json"), plan_sha256=pin,
        limited_original_and_FALSE_RGB_GT_verification_authorized=True, old_target_only_authority_extended=False,
        training_authorized=False, NN_authorized=False, DEV_HOLDOUT_TEST_weights_TRUE_deniedRAW_decode_authorized=False,
        output=str(OUT), CPU_affinity=[3], contract=fixed())


def prepare():
    require(not PLAN_DIR.exists() and not OUT.exists(), "fresh plan/output required, no retry or resume")
    budget = Budget()
    budget.guard()
    env = environment()
    sources = {str(ROOT / path): digest(ROOT / path, budget.guard) for path in SOURCES}
    for path, pin in INPUTS.items():
        require(digest(path, budget.guard) == pin, "declared admitted TRAIN input changed")
    roles = registration()
    rows = baseline_rows()
    plan = dict(schema=SCHEMA, provenance="HH_261008 - Fresh limited byte-verification declaration, not training.",
        contract=fixed(), source_sha256=sources, input_sha256=INPUTS, environment=env,
        selected_baseline_rows=rows, registration_scope=roles, output=str(OUT), lock_generation=lease_guard(),
        prepare_process_generation=process_identity())
    raw = encoded(plan)
    copies = sum(Path(path).stat().st_size for path in sources)
    require(copies + len(raw) * 4 + 256 * 1024 + CLOSING + 64 * 1024 < LIMIT, "prospective source/journal/closing quota denied")
    PLAN_DIR.mkdir(parents=True, exist_ok=False)
    (PLAN_DIR / "sources").mkdir(exist_ok=False)
    for path, pin in sources.items():
        payload = Path(path).read_bytes()
        require(hashlib.sha256(payload).hexdigest() == pin, "fresh source copy changed")
        write_new(PLAN_DIR / "sources" / (pin + ".py"), payload)
    write_new(PLAN_DIR / "candidate_plan.json", raw)
    return dict(status="PREPARED_NOT_AUTHORIZED_OR_RUNNING", candidate_sha256=hashlib.sha256(raw).hexdigest(), RGB_or_GT_payloads_decoded=0)


def execute(plan_pin, grant_pin):
    plan_path, grant_path = PLAN_DIR / "ROOT_plan.json", PLAN_DIR / "ROOT_authority.json"
    require(digest(plan_path) == plan_pin and digest(grant_path) == grant_pin, "exact external RAW ROOT pins required")
    plan, grant = decode(plan_path.read_bytes()), decode(grant_path.read_bytes())
    require(plan["schema"] == SCHEMA and typed_equal(plan["contract"], fixed()) and typed_equal(grant, authority(plan, plan_pin))
        and plan["output"] == str(OUT) and not OUT.exists(), "new exact bounded authority/fresh output required")
    require(plan["input_sha256"] == INPUTS and set(plan["source_sha256"]) == {str(ROOT / p) for p in SOURCES}, "exact named seal membership required")
    budget = Budget()
    primary = None
    cleanup = []
    writer = reader = None
    passed, failed, current = 0, 0, None
    comparisons, reads = [], []
    OUT.mkdir(exist_ok=False)
    try:
        budget.guard()
        require(environment() == plan["environment"], "selected provider identity changed")
        require(lease_guard() == plan["lock_generation"], "held lease generation changed")
        seals = {**plan["source_sha256"], **INPUTS, **plan["environment"]["selected_provider_sha256"]}
        for path, pin in seals.items():
            require(digest(path, budget.guard) == pin, "input/provider/source admission changed")
        for path, pin in plan["source_sha256"].items():
            require(digest(PLAN_DIR / "sources" / (pin + ".py"), budget.guard) == pin, "source archive changed")
        require(typed_equal(registration(), plan["registration_scope"])
            and typed_equal(baseline_rows(), plan["selected_baseline_rows"]), "selected original journal rows or roles changed")
        write_new(OUT / "ROOT_plan.json", plan_path.read_bytes())
        write_new(OUT / "ROOT_authority.json", grant_path.read_bytes())
        from portable_e2e import map_native_pair_reader_20261008_v2 as native
        from portable_e2e import map_crop_batch_worker_20261008_v3 as worker
        crop = worker.compile_original_crop(ROOT / "tools/e2e/map_appearance_matched_contract_20261001.py")
        reader = native.BoundedNativePairReader(DATA / "original_train729.npz", INPUTS[str(DATA / "original_train729.npz")],
            DATA / "paired_train480.npz", INPUTS[str(DATA / "paired_train480.npz")])
        writer = worker.BatchJournalWriter(OUT / "candidate_RGB_GT_batches.jsonl", STEPS)
        for declared in plan["selected_baseline_rows"]:
            current = declared["step"]
            budget.guard()
            rgb, target, receipt = worker.build_batch(declared["baseline"], reader, crop, checkpoint=budget.guard)
            writer.append(rgb, target, receipt)
            passed += 1
            comparisons.append(dict(step=current, original_RGB_GT_equal=True,
                rescued_slots=sum(receipt["journal"]["rescued"]), candidate_RGB_sha256=worker.array_hash(rgb),
                candidate_target_sha256=worker.array_hash(target)))
            reads.append(dict(step=current, last_slot_read=reader.last_read_receipt))
            del rgb, target, receipt
            budget.guard()
    except Exception as error:
        primary = dict(type=type(error).__name__, message=str(error)[:1024], step=current)
        failed = int(current is not None and passed < len(STEPS))
    finally:
        for name, value in (("journal", writer), ("reader", reader)):
            if value is not None:
                try:
                    value.close()
                except Exception as error:
                    cleanup.append(dict(scope=name, type=type(error).__name__, message=str(error)[:1024]))
        for path, pin in {**plan["source_sha256"], **INPUTS, **plan["environment"]["selected_provider_sha256"]}.items():
            try:
                require(digest(path) == pin, "closing seal changed")
            except Exception as error:
                cleanup.append(dict(scope="closing_seal", path=path, type=type(error).__name__, message=str(error)[:1024]))
        try:
            budget.guard()
        except Exception as error:
            cleanup.append(dict(scope="closing_budget", type=type(error).__name__, message=str(error)[:1024]))
    result = dict(schema=SCHEMA + ".result", provenance="HH_261008 - Actual eight-batch RGB/GT verification, not training or feature approval.",
        status="COMPLETE_LIMITED_RGB_GT_BYTES_NOT_TRAINING_OR_MODEL_GAIN" if primary is None and not cleanup and passed == 8 else "FAILED_RETAINED",
        PASS_batches=passed, FAIL_batches=failed, UNEXECUTED_batches=8 - passed - failed,
        declared_steps=STEPS, original20k_batches_outside_declaration=19992,
        selected_batch_comparisons=comparisons, last_slot_prefix_read_witnesses=reads,
        failure=primary, cleanup_errors=cleanup, sampled_closing_memory=budget.last,
        sampled_maximum_VmHWM_bytes=budget.maximum_HWM, elapsed_seconds=time.monotonic() - budget.started,
        ROOT_plan_sha256=plan_pin, ROOT_authority_sha256=grant_pin,
        actual_array_diagnostic_only=True, training_steps=0, NN=0, model_performance_improved=False,
        formal_main_approved=0, formal_sub_approved=0, historical_provider_equivalence_claimed=False,
        current_process_generation=process_identity(), declared_thread_environment={k: os.environ[k]
            for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")},
        original_journals_or_frozen_trainer_modified=False, partial_prefix_decode_not_only_selected_rows=True)
    write_new(OUT / "result.json", encoded(result), closing=True)
    return result


def main():
    global HELD_LEASE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--plan-sha256")
    parser.add_argument("--authority-sha256")
    args = parser.parse_args()
    require(args.prepare != args.execute, "exactly one prepare or execute required")
    canonical(LOCK)
    with LOCK.open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        HELD_LEASE = lease
        try:
            value = prepare() if args.prepare else execute(args.plan_sha256, args.authority_sha256)
        finally:
            HELD_LEASE = None
            fcntl.flock(lease, fcntl.LOCK_UN)
    print(encoded(value).decode(), end="")
    return 0 if value["status"].startswith(("PREPARED_", "COMPLETE_")) else 2


if __name__ == "__main__":
    raise SystemExit(main())
