"""HH_261008 - Prepare a bounded FIRST16 reader-cache A/B without training or model claims.

Only separately RAW-pinned ROOT plan/authority may execute. Preparation seals
metadata/source and cannot decode RGB/GT payloads. Both arms use unchanged worker
V3, crop math and policy; historical original RGB and GT SHA must both reproduce.
Actual rescue0 remains rescue-not-observed. The remaining19984 batches are outside
this declaration. Fresh paths, ownership, budgets and complete failed tails persist.
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
NAME = "map_crop_rescue_reader_ab_v1"
SCHEMA = "portable_e2e.map_crop_rescue_reader_ab.20261008.v1"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08" / NAME
OUT = ROOT / "storage/experiments/diagnostics/2026-10-08" / NAME
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
HELD_LEASE = None
DATA = ROOT / "storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
STEPS = list(range(1, 17))
PROSPECTIVE_STEPS = (16, 32, 64)
ARMS = ("CONTROL_V3_CACHE0", "CANDIDATE_V3_CACHE12M")
SELECTION = ROOT / "storage/experiments/plans/2026-10-08/map_crop_rescue_reader_ab_metadata_selection_v2.json"
SELECTION_PIN = "998fa695a1f2437d4f34d4bc1341c7c43256f038ef5cde072f4a7918f3b1f8e1"
JOURNAL_LIMIT = 256 * 1024
PNG_LIMIT = 512 * 1024
CACHE_STATES, CACHE_STATE_BYTES = 48, 256 * 1024
CACHE_LIMIT, CACHE_STRIDE = 12 * 1024**2, 32 * 1024**2
OWNER_CHECKPOINT_GUARD_STRIDE = 16
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
    str(SELECTION): SELECTION_PIN,
}
OLD_SOURCES = [
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
]
SOURCES = OLD_SOURCES + [
    "learning/portable_e2e/map_native_pair_reader_20261008_v3.py",
    "learning/tests/test_map_native_pair_reader_20261008_v3.py",
    "tools/e2e/verify_map_crop_rescue_reader_ab_20261008_v1.py",
    "learning/tests/test_verify_map_crop_rescue_reader_ab_20261008_v1.py",
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
        self.process_generation = process_identity()
        self.guard_calls = 0

    def guard(self):
        self.guard_calls += 1
        lease_guard()
        require(process_identity() == self.process_generation, "current process generation changed")
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


def selected_imaging(directory):
    # HH_261008 - Optional _imagingtk/ft/cms/math/morph are not the RGB resize core.
    files = list(Path(directory).glob("_imaging.*.so"))
    require(len(files) == 1, "exact Pillow RGB resize core required")
    return files[0]


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
    imaging = selected_imaging(pdir)
    require(len(native) == 1, "exact selected native providers required")
    selected = [*paths.values(), str(native[0]), str(imaging), str(pdir / "Image.py"), str(Path(sys.executable).resolve())]
    return dict(origins=paths, versions={n: importlib.metadata.version(n) for n in ("numpy", "Pillow")},
        selected_provider_sha256={p: digest(p) for p in selected}, complete_transitive_provider_closure_claimed=False,
        historical_environment_equivalence_claimed=False)


def baseline_rows(limit=64):
    require(type(limit) is int and limit in PROSPECTIVE_STEPS, "fixed prospective metadata range required")
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
            if count <= limit:
                selected.append(dict(step=count, RAW_line_sha256=hashlib.sha256(raw).hexdigest(), baseline=row))
    require(count == 20000 and [r["step"] for r in selected] == list(range(1, limit+1)),
        "whole20k metadata and fixed initial rows required")
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
    return dict(declared_steps=STEPS, expected_batches_per_arm=16, expected_slot_exposures_per_arm=128,
        arms=list(ARMS), reader_source_same_in_both_arms=True,
        CONTROL_checkpoint_cache_enabled=False, CANDIDATE_checkpoint_cache_enabled=True,
        candidate_checkpoint_max_states=CACHE_STATES, checkpoint_state_account_bytes=CACHE_STATE_BYTES,
        candidate_cache_account_limit_bytes=CACHE_LIMIT, checkpoint_stride_bytes=CACHE_STRIDE,
        owner_checkpoint_guard_stride=OWNER_CHECKPOINT_GUARD_STRIDE,
        state_accounting_is_reservation_not_exact_native_allocation=True,
        original20k_batches_outside_declaration_per_arm=19984,
        selected_metadata_not_random_or_independent_TEST=True,
        metadata_positive_partial_crop_hints_do_not_prove_original_GT_zero=True,
        allowed_original_members=["rgb.npy", "target.npy"], allowed_paired_members=["rgb_false.npy", "target.npy"],
        paired_TRUE_member_decode_authorized=False, baseline_both_RGB_and_GT_SHA_required=True,
        same_worker_crop_policy_original_rows_FALSE_and_proposals=True, RNG_draws_added=0,
        CPU_affinity=[3], threads=1, sampled_self_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        floor_bytes=FLOOR, reserve_bytes=RESERVE, maximum_combined_PLAN_OUTPUT_bytes=LIMIT,
        closing_bytes=CLOSING, journal_maximum_bytes_per_arm=JOURNAL_LIMIT,
        optional_first_actual_rescue_PNG_maximum_bytes=PNG_LIMIT,
        PNG_measured_after_both_arms=True, PNG_semantic_GT_signmask_not_model_prediction=True,
        sampled_guards_not_OS_containment=True, old_target_only_authority_reused=False,
        architecture_changed=False, training_steps=0, NN=0, main_approved=0, sub_approved=0)


def authority(plan, pin):
    return dict(schema=SCHEMA + ".ROOT_authority", plan_path=str(PLAN_DIR / "ROOT_plan.json"), plan_sha256=pin,
        limited_original_and_FALSE_RGB_GT_verification_authorized=True, old_target_only_authority_extended=False,
        same_source_reader_cache_AB_authorized=True, optional_actual_first_rescue_diagnostic_PNG_authorized=True,
        training_authorized=False, NN_authorized=False, DEV_HOLDOUT_TEST_weights_TRUE_deniedRAW_decode_authorized=False,
        output=str(OUT), CPU_affinity=[3], contract=fixed())


def graphics_environment(env):
    # HH_261008 - Keep the six scientific providers distinct from optional diagnostic drawing sources.
    parent = Path(env["origins"]["PIL"]).parent
    selected = [parent / "ImageDraw.py", parent / "ImageFont.py"]
    fonts = list(parent.glob("_imagingft.*.so"))
    require(len(fonts) <= 1, "ambiguous optional native font provider")
    selected.extend(fonts)
    return dict(selected_graphics_source_sha256={str(path): digest(path) for path in selected},
        complete_transitive_graphics_provider_closure_claimed=False)


def selection_scope(selection, selected, first16_sha):
    """HH_261008 - Join seven upfront metadata hints without asserting an unobserved cropped-GT zero."""
    require(type(selection) is dict and selection["schema"] == "portable_e2e.map_crop_rescue_reader_ab_metadata_selection.20261008.v2"
        and selection["state"] == "METADATA_ONLY_PREPARED_NOT_PAYLOAD_VERIFIED"
        and type(selected) is list and len(selected) == 16
        and typed_equal(selection["metadata_scan_steps"], STEPS)
        and type(selection["metadata_exposures_examined"]) is int and selection["metadata_exposures_examined"] == 128
        and selection["first16_original_RAW_lines_sha256"] == first16_sha,
        "original upfront FIRST16 metadata basis differs")
    require(all(selection[key] is False for key in ("policy_changed", "proposals_rerolled",
        "successful_crop_or_rescue_cases_selected_after_payload_decode", "candidate_metadata_is_independent_TEST",
        "actual_RGB_rescue_effect_proven", "model_performance_gain_proven", "existing_thresholds_or_resource_caps_lowered"))
        and selection["prospective_FIRST16_fixed_before_new_payload_decode"] is True
        and all(type(value) is int and value == 0 for value in selection["new_work"].values()),
        "prepared metadata cannot claim a rescue, new payload execution or changed admission")
    require(typed_equal(selection["resource_caps_preserved"], dict(sampled_VmHWM_bytes=RSS,
        deadline_seconds=DEADLINE, floor_bytes=FLOOR, reserve_bytes=RESERVE)), "metadata resource limits differ")
    expected_arms = {name: dict(reader_revision="V3", cache_enabled=index == 1, declared_batches=16,
        declared_slot_exposures=128, PASS_batches=0, FAIL_batches=0, UNEXECUTED_batches=16,
        original20000_batches_OUT_OF_SCOPE=19984) for index, name in enumerate(ARMS)}
    require(typed_equal(selection["prospective_arms"], expected_arms), "final same-V3 arm names or zero execution counts differ")
    candidates = selection["candidates"]
    require(type(candidates) is list and type(selection["candidate_count"]) is int
        and selection["candidate_count"] == len(candidates) == 7, "exact seven metadata hints required")
    seen = set()
    for hint in candidates:
        require(type(hint) is dict and set(hint) == {"step", "position_zero_based", "global_ordinal_zero_based",
            "slot", "fullGT_sign_pixels", "proposed_box", "actual_crop_GT_zero_unknown"}, "exact candidate hint metadata required")
        step, position = hint["step"], hint["position_zero_based"]
        require(type(step) is type(position) is int and 1 <= step <= 16 and 0 <= position < 8
            and (step, position) not in seen and hint["actual_crop_GT_zero_unknown"] is True,
            "candidate chronology or unknown cropped-GT flag differs")
        seen.add((step, position))
        row = selected[step-1]["baseline"]
        ordinal = (step-1)*8+position
        require(type(hint["global_ordinal_zero_based"]) is int and hint["global_ordinal_zero_based"] == ordinal and ordinal % 2 == 0
            and type(hint["slot"]) is int and hint["slot"] == row["physical_slot_indices"][position]
            and typed_equal(hint["proposed_box"], row["shared_crop_boxes"][position])
            and hint["proposed_box"] != [0, 0, 360, 640]
            and type(hint["fullGT_sign_pixels"]) is int and 0 < hint["fullGT_sign_pixels"] <= 230400,
            "candidate hint does not join original position/slot/proposal/positive metadata support")
    return dict(schema=selection["schema"], selection_RAW_sha256=SELECTION_PIN,
        first16_original_RAW_lines_sha256=first16_sha, candidates=candidates,
        cropped_GT_zero_verified=False, actual_RGB_rescue_effect_proven=False,
        candidate_metadata_is_independent_TEST=False, actual_new_payload_decodes_before_selection=0)


def first16_metadata_sha(selected):
    digest16 = hashlib.sha256()
    with canonical(RUN / "control/exposures.jsonl").open("rb") as source:
        for row in selected:
            raw = source.readline(4097)
            require(0 < len(raw) <= 4096 and raw.endswith(b"\n")
                and hashlib.sha256(raw).hexdigest() == row["RAW_line_sha256"], "exact first16 original RAW metadata line changed")
            digest16.update(raw)
    return digest16.hexdigest()


def selection_metadata(selected):
    path = canonical(SELECTION)
    require(path.stat().st_size <= 32*1024, "bounded prepared selection metadata required")
    selection = decode(path.read_bytes())
    scope = selection_scope(selection, selected, first16_metadata_sha(selected))
    # HH_261008 - Auxiliary sources/results are read as sealed scalar provenance, never executed or inflated.
    auxiliary = {**selection["source_RAW_sha256"], **selection["metadata_RAW_sha256"],
        selection["old_v1_RAW_seal"]["path"]: selection["old_v1_RAW_seal"]["sha256"]}
    require(len(auxiliary) == 6 and all(Path(path).is_relative_to(ROOT) and type(pin) is str
        and len(pin) == 64 and all(c in "0123456789abcdef" for c in pin) for path, pin in auxiliary.items()),
        "exact bounded selection auxiliary RAW seals required")
    return scope, auxiliary


def maximum_receipt_bytes(baseline):
    """HH_261008 - Scalar envelope only; invalid preview counts/boxes are never written as real evidence."""
    from portable_e2e import map_crop_preservation_journal_20261008_v1 as journal
    journal.validate_baseline(baseline)
    # Four three-digit coordinates, six-digit pixel counts and False are maximum-width scalar bounds.
    row = dict(schema=journal.SCHEMA, policy_source_sha256=journal.POLICY_SOURCE_SHA256,
        baseline_exposure=baseline, selected_crop_boxes=[[100, 100, 100, 100] for _ in range(8)],
        full_GT_sign_pixels=[230400]*8, original_NEAREST_sign_pixels=[230400]*8,
        protected=[False]*8, rescued=[False]*8, candidate_RGB_sha256="f"*64, candidate_target_sha256="f"*64)
    receipt = dict(schema="portable_e2e.map_crop_batch_worker.20261008.v3.batch", step=baseline["step"], journal=row,
        native_slots=[dict(position=p, slot=slot, native_RGB_sha256="f"*64, native_target_sha256="f"*64,
            reader_provenance_authenticated_by_worker=False) for p, slot in enumerate(baseline["physical_slot_indices"])],
        actual_baseline_RGB_and_GT_hashes_reproduced=True, candidate_hashes_computed_from_actual_owned_arrays=True,
        same_box_for_actual_RGB_and_GT=True, source_and_baseline_RAW_authenticated_by_worker=False, training_steps=0, NN=0)
    return len(encoded(receipt))


def prospective_quota(rows, source_bytes, plan_bytes):
    require(type(rows) is list and len(rows) in PROSPECTIVE_STEPS
        and type(source_bytes) is int and source_bytes >= 0
        and type(plan_bytes) is int and plan_bytes > 0, "bounded exact prospective inputs required")
    journal_bytes = sum(maximum_receipt_bytes(row["baseline"]) for row in rows)
    # HH_261008 - Four plan-sized metadata copies, both full journals, PNG and unchanged closing reserve are prospective.
    total = source_bytes + 4*plan_bytes + 2*journal_bytes + PNG_LIMIT + CLOSING + 64*1024
    return dict(prospective_batches_per_arm=len(rows), source_archive_bytes=source_bytes,
        plan_copy_allowance_bytes=4*plan_bytes, estimated_maximum_journal_bytes_per_arm=journal_bytes,
        unchanged_journal_cap_bytes_per_arm=JOURNAL_LIMIT, optional_PNG_allowance_bytes=PNG_LIMIT,
        closing_bytes=CLOSING, extra_ROOT_metadata_allowance_bytes=64*1024,
        estimated_combined_PLAN_OUTPUT_bytes=total, maximum_combined_PLAN_OUTPUT_bytes=LIMIT,
        fits_unchanged_limits=journal_bytes <= JOURNAL_LIMIT and total < LIMIT,
        scalar_envelope_not_actual_RGB_GT_or_rescue_evidence=True,
        prospective_scope_change_authorized=False)


def prepare():
    require(not PLAN_DIR.exists() and not OUT.exists(), "fresh plan/output required, no retry or resume")
    budget = Budget()
    budget.guard()
    env = environment()
    graphics = graphics_environment(env)
    require(len(SOURCES) == len(set(SOURCES)) == 18, "exact original14 plus new reader/runner source-and-test seals required")
    sources = {str(ROOT / path): digest(ROOT / path, budget.guard) for path in SOURCES}
    require(len(set(sources.values())) == 18, "all18 source archive artifacts must be distinct")
    for path, pin in INPUTS.items():
        require(digest(path, budget.guard) == pin, "declared admitted TRAIN input changed")
    roles = registration()
    preview_rows = baseline_rows(64)
    selection, auxiliary = selection_metadata(preview_rows[:16])
    for path, pin in auxiliary.items():
        require(digest(path, budget.guard) == pin, "upfront selection source/result/legacy relation seal changed")
    plan = dict(schema=SCHEMA, provenance="HH_261008 - Fresh FIRST16 same-reader cache A/B, no training.",
        contract=fixed(), source_sha256=sources, input_sha256=INPUTS, environment=env,
        diagnostic_graphics_environment=graphics, selected_baseline_rows=preview_rows[:16],
        registration_scope=roles, output=str(OUT), lock_generation=lease_guard(),
        prepare_process_generation=process_identity(), source_archive_count=18,
        selection_scope=selection, auxiliary_metadata_sha256=auxiliary,
        selection_basis="Upfront FIRST16 metadata selection; original cropped GT zero remains unknown before payload execution")
    copies = sum(Path(path).stat().st_size for path in sources)
    projections = []
    for count in PROSPECTIVE_STEPS:
        preview = dict(plan, selected_baseline_rows=preview_rows[:count])
        projections.append(prospective_quota(preview_rows[:count], copies, len(encoded(preview))+16*1024))
    plan["prospective_16_32_64_quota"] = projections
    raw = encoded(plan)
    exact = prospective_quota(plan["selected_baseline_rows"], copies, len(raw))
    require(projections[0]["fits_unchanged_limits"] and exact["fits_unchanged_limits"],
        "prospective FIRST16 source/journals/PNG/closing quota denied; no threshold increase")
    PLAN_DIR.mkdir(parents=True, exist_ok=False)
    (PLAN_DIR / "sources").mkdir(exist_ok=False)
    for path, pin in sources.items():
        payload = Path(path).read_bytes()
        require(hashlib.sha256(payload).hexdigest() == pin, "fresh source copy changed")
        write_new(PLAN_DIR / "sources" / (pin + ".py"), payload)
    write_new(PLAN_DIR / "candidate_plan.json", raw)
    return dict(status="PREPARED_NOT_AUTHORIZED_OR_RUNNING", candidate_sha256=hashlib.sha256(raw).hexdigest(),
        source_archive_count=18, RGB_or_GT_payloads_decoded=0, actual_rescue_observed=False,
        prospective_16_32_64_quota=projections)


def bounded_reader_witness(receipt, *, require_read_scope=True):
    """HH_261008 - Retain the last actual read only; scalar metadata never replaces byte validation."""
    require(type(require_read_scope) is bool and type(receipt) is dict and len(receipt) <= 32,
        "bounded last-read scalar receipt required")
    def walk(value, depth=0, counter=None):
        counter = [0] if counter is None else counter
        counter[0] += 1
        require(depth <= 8 and counter[0] <= 256, "reader witness node bound exceeded")
        if type(value) in (int, bool) or value is None:
            require(type(value) is not int or 0 <= value < 2**63, "reader witness integer invalid")
            return value
        if type(value) is str:
            require(len(value) <= 4096 and "\0" not in value, "reader witness string invalid")
            return value
        require(type(value) is dict and len(value) <= 32
            and all(type(k) is str and len(k) <= 128 for k in value), "reader witness exact bounded dict required")
        return {key: walk(item, depth+1, counter) for key, item in value.items()}
    result = walk(receipt)
    if require_read_scope:
        require(result.get("rgb_true_decoded") is False and result.get("caller_admission_authenticated_by_reader") is False,
            "raw receipt cannot authenticate caller admission or permit TRUE")
    require(len(encoded(result)) <= 16*1024, "bounded last-read witness allowance exceeded")
    return result


def reader_stats(reader, enabled):
    stats = bounded_reader_witness(reader.stats_snapshot(), require_read_scope=False)
    require(stats["checkpoint_cache_enabled"] is enabled
        and stats["native_row_cache_payload_bytes"] == 0
        and stats["state_accounting_is_reservation_not_exact_native_allocation"] is True,
        "fixed cache mode/native-state reservation differs")
    require(stats["checkpoint_state_account_bytes"] == CACHE_STATE_BYTES
        and stats["checkpoint_stride_bytes"] == CACHE_STRIDE
        and stats["cache_account_limit_bytes"] == (CACHE_LIMIT if enabled else 0)
        and stats["max_checkpoint_states"] == (CACHE_STATES if enabled else 0)
        and stats["maximum_cache_accounted_bytes"] <= (CACHE_LIMIT if enabled else 0)
        and stats["maximum_checkpoint_states_observed"] <= (CACHE_STATES if enabled else 0)
        and stats["owner_checkpoint_guard_stride"] == OWNER_CHECKPOINT_GUARD_STRIDE,
        "unchanged fixed cache bounds exceeded")
    return stats


def closing_observations(reader):
    """HH_261008 - Keep secondary close evidence without recounting the same exception as a failed batch."""
    rows = reader.closing_errors
    require(type(rows) is list and len(rows) <= 6, "bounded reader close observations required")
    for row in rows:
        require(type(row) is dict and set(row) == {"scope", "type", "message"}
            and type(row["scope"]) is str and row["scope"] in {
                prefix+suffix for prefix in ("original:", "paired:")
                for suffix in ("before_close_guard", "fd_close", "after_close_owner")}
            and type(row["type"]) is str and 0 < len(row["type"]) <= 64
            and type(row["message"]) is str and len(row["message"]) <= 512,
            "exact safe closing scalar observations required")
    return [bounded_reader_witness(row, require_read_scope=False) for row in rows]


def denominator(passed, *, active_failure=False):
    """HH_261008 - A completed slot followed by a guard failure is never counted again as failed."""
    require(type(passed) is int and 0 <= passed <= 16 and type(active_failure) is bool
        and not (passed == 16 and active_failure), "strict declared16 active-attempt denominator required")
    failed = int(active_failure)
    return dict(PASS_batches=passed, FAIL_batches=failed, UNEXECUTED_batches=16-passed-failed)


def rescue_witnesses(receipt, selected_counts):
    require(type(selected_counts) is list and len(selected_counts) == 8
        and all(type(v) is int and 0 <= v <= 230400 for v in selected_counts), "actual selected GT count ABI differs")
    row = receipt["journal"]
    witnesses = []
    for position, rescued in enumerate(row["rescued"]):
        if rescued:
            require(row["full_GT_sign_pixels"][position] > 0 and row["original_NEAREST_sign_pixels"][position] == 0
                and selected_counts[position] > 0 and row["selected_crop_boxes"][position] == [0, 0, 360, 640]
                and receipt["same_box_for_actual_RGB_and_GT"] is True,
                "actual rescue must preserve positive GT support with the same full-FOV RGB/GT box")
            witnesses.append(dict(step=receipt["step"], position=position,
                slot=row["baseline_exposure"]["physical_slot_indices"][position],
                full_GT_sign_pixels=row["full_GT_sign_pixels"][position],
                proposed_NEAREST_GT_sign_pixels=row["original_NEAREST_sign_pixels"][position],
                selected_NEAREST_GT_sign_pixels=selected_counts[position],
                proposed_box=row["baseline_exposure"]["shared_crop_boxes"][position],
                selected_box=row["selected_crop_boxes"][position], actual_same_box_for_RGB_GT=True))
    return witnesses


def first_rescue_copy(rgb, target, receipt, rescues, worker):
    # HH_261008 - Both measured arms pay the same one-sample copy/hash cost only after both historical B8 hashes reproduce.
    if not rescues:
        return None
    witness = rescues[0]
    position = witness["position"]
    sample_rgb, sample_target = rgb[position].copy(), target[position].copy()
    native = receipt["native_slots"][position]
    require(worker.array_hash(sample_rgb) == native["native_RGB_sha256"]
        and worker.array_hash(sample_target) == native["native_target_sha256"],
        "FULL_FOV rescued actual bytes must equal that sample's native RGB and GT SHA")
    return dict(witness=witness, RGB=sample_rgb, target=sample_target,
        native_RGB_sha256=native["native_RGB_sha256"], native_target_sha256=native["native_target_sha256"])


def render_first_rescue(sample, original_crop, worker, budget):
    import io
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np
    started = time.monotonic()
    info = dict(status="STARTED", step=sample["witness"]["step"], position=sample["witness"]["position"],
        semantic_GT_signmask_not_model_prediction=True, actual_CARLA_or_RViz_screenshot=False,
        ego_or_route_observed=False, invented_ego_or_route_drawn=False)
    try:
        budget.guard()
        rgb, target = sample["RGB"], sample["target"]
        cropped_rgb, cropped_gt = original_crop(rgb, target, sample["witness"]["proposed_box"])
        worker.native_pair(cropped_rgb, cropped_gt)
        native_count = int(np.count_nonzero(target == 5))
        proposed_count = int(np.count_nonzero(cropped_gt == 5))
        require(worker.array_hash(rgb) == sample["native_RGB_sha256"]
            and worker.array_hash(target) == sample["native_target_sha256"]
            and native_count == sample["witness"]["full_GT_sign_pixels"]
            == sample["witness"]["selected_NEAREST_GT_sign_pixels"] and native_count > 0
            and proposed_count == sample["witness"]["proposed_NEAREST_GT_sign_pixels"] == 0,
            "actual PNG native/proposed/rescued bytes and sign counts must match the saved rescue witness")
        info.update(native_RGB_sha256=worker.array_hash(rgb), native_target_sha256=worker.array_hash(target),
            proposed_RGB_sha256=worker.array_hash(cropped_rgb), proposed_target_sha256=worker.array_hash(cropped_gt),
            native_GT_sign_pixels=native_count, proposed_GT_sign_pixels=proposed_count, rescued_GT_sign_pixels=native_count,
            actual_sample_bytes_and_counts_match_rescue_witness=True)
        canvas = Image.new("RGB", (1920, 768), (25, 25, 25))
        draw = ImageDraw.Draw(canvas)
        font = ImageFont.load_default()
        names = ("Native FULL", "Original proposed CROP", "Actual rescued FULL")
        for column, (panel_rgb, panel_gt) in enumerate(((rgb, target), (cropped_rgb, cropped_gt), (rgb, target))):
            budget.guard()
            x = column*640
            draw.text((x+8, 5), names[column], fill="white", font=font)
            canvas.paste(Image.fromarray(panel_rgb, "RGB"), (x, 24))
            count = proposed_count if column == 1 else native_count
            draw.text((x+8, 389), "GT sign mask label 5: "+str(count)+" pixels (no model)", fill="white", font=font)
            mask = Image.fromarray(panel_gt, "L").point(lambda value: 255 if value == 5 else 0).convert("RGB")
            canvas.paste(mask, (x, 408))
        info["render_seconds"] = time.monotonic()-started
        encoding = time.monotonic()
        with io.BytesIO() as buffer:
            canvas.save(buffer, format="PNG")
            raw = buffer.getvalue()
        info.update(encode_seconds=time.monotonic()-encoding, encoded_bytes=len(raw),
            encoded_sha256=hashlib.sha256(raw).hexdigest(), maximum_bytes=PNG_LIMIT)
        require(len(raw) <= PNG_LIMIT, "unchanged first-rescue PNG512KiB cap exceeded")
        budget.guard()
        writing = time.monotonic()
        write_new(OUT / "first_actual_rescue.png", raw)
        info.update(write_seconds=time.monotonic()-writing, path="first_actual_rescue.png", status="COMPLETE_ACTUAL_RECORDED_INPUT_CROP_DIAGNOSTIC")
        budget.guard()
    except Exception as error:
        info.update(status="FAILED_RETAINED", failure=dict(type=type(error).__name__, message=str(error)[:1024]))
    info["total_post_arm_seconds"] = time.monotonic()-started
    return info


def execute(plan_pin, grant_pin):
    plan_path, grant_path = PLAN_DIR / "ROOT_plan.json", PLAN_DIR / "ROOT_authority.json"
    require(digest(plan_path) == plan_pin and digest(grant_path) == grant_pin, "exact external RAW ROOT pins required")
    raw_plan, raw_grant = plan_path.read_bytes(), grant_path.read_bytes()
    plan, grant = decode(raw_plan), decode(raw_grant)
    require(plan["schema"] == SCHEMA and typed_equal(plan["contract"], fixed()) and typed_equal(grant, authority(plan, plan_pin))
        and plan["output"] == str(OUT) and not OUT.exists(), "new exact bounded authority/fresh output required")
    require(raw_plan == canonical(PLAN_DIR / "candidate_plan.json").read_bytes(), "ROOT plan must preserve original prepared RAW bytes")
    require(plan["input_sha256"] == INPUTS and set(plan["source_sha256"]) == {str(ROOT / p) for p in SOURCES}
        and plan["source_archive_count"] == 18, "exact named18 source seal membership required")
    budget = Budget()
    primary, cleanup, failure_tail = None, [], None
    upfront_seals, closing_seals, closing_seal_observations = {}, {}, []
    upfront_archives, closing_archives = {}, {}
    active_batch = comparison_active = arm_active = False
    writer = reader = None
    arms = {name: dict(PASS_batches=0, FAIL_batches=0, UNEXECUTED_batches=16, expected_batches=16,
        expected_slot_exposures=128, rescued_slots=0, actual_rescue_witnesses=[], batches=[],
        reader_mode="same_V3_cache_disabled" if index == 0 else "same_V3_cache_enabled",
        startup_seconds=None, measured_arm_seconds=None, last_slot_read=None, cache_stats=None)
        for index, name in enumerate(ARMS)}
    all_arm_rows, comparisons, samples = {name: {} for name in ARMS}, [], {}
    comparison_seconds = None
    current_arm, current_step = None, None
    phase = "admission"
    png = dict(status="NOT_CREATED_RESCUE_NOT_OBSERVED", actual_rescue_observed=False)
    OUT.mkdir(exist_ok=False)
    try:
        budget.guard()
        require(environment() == plan["environment"] and graphics_environment(plan["environment"]) == plan["diagnostic_graphics_environment"],
            "selected scientific or drawing provider identity changed")
        require(lease_guard() == plan["lock_generation"], "held lease generation changed")
        seals = {**plan["source_sha256"], **INPUTS, **plan["auxiliary_metadata_sha256"], **plan["environment"]["selected_provider_sha256"],
                 **plan["diagnostic_graphics_environment"]["selected_graphics_source_sha256"]}
        for path, pin in seals.items():
            upfront_seals[path] = digest(path, budget.guard)
            require(upfront_seals[path] == pin, "input/provider/source admission changed")
        for path, pin in plan["source_sha256"].items():
            archive = str(PLAN_DIR / "sources" / (pin + ".py"))
            upfront_archives[archive] = digest(archive, budget.guard)
            require(upfront_archives[archive] == pin, "source archive changed")
        require(typed_equal(registration(), plan["registration_scope"])
            and typed_equal(baseline_rows(16), plan["selected_baseline_rows"]), "selected original journal rows or roles changed")
        selection, auxiliary = selection_metadata(plan["selected_baseline_rows"])
        require(typed_equal(selection, plan["selection_scope"]) and typed_equal(auxiliary, plan["auxiliary_metadata_sha256"]),
            "upfront candidate hint/source/result/legacy metadata binding changed")
        source_bytes = sum(Path(path).stat().st_size for path in plan["source_sha256"])
        require(prospective_quota(plan["selected_baseline_rows"], source_bytes, len(raw_plan))["fits_unchanged_limits"],
            "fresh FIRST16 prospective quota denied")
        write_new(OUT / "ROOT_plan.json", raw_plan)
        write_new(OUT / "ROOT_authority.json", raw_grant)
        from portable_e2e import map_native_pair_reader_20261008_v3 as native
        from portable_e2e import map_crop_batch_worker_20261008_v3 as worker
        import numpy as np
        require(native.CHECKPOINT_MAX_STATES == CACHE_STATES
            and native.CHECKPOINT_STATE_ACCOUNT_BYTES == CACHE_STATE_BYTES
            and native.CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES == CACHE_LIMIT
            and native.CHECKPOINT_STRIDE_BYTES == CACHE_STRIDE
            and native.OWNER_CHECKPOINT_GUARD_STRIDE == OWNER_CHECKPOINT_GUARD_STRIDE,
            "fixed candidate cache ABI changed")
        crop = worker.compile_original_crop(ROOT / "tools/e2e/map_appearance_matched_contract_20261001.py")
        for index, current_arm in enumerate(ARMS):
            current_step = None
            enabled = index == 1
            arm = arms[current_arm]
            arm_active = True
            phase = "reader_startup"
            arm_started = time.monotonic()
            arm_guard_start = budget.guard_calls
            startup = time.monotonic()
            reader = native.BoundedNativePairReader(DATA / "original_train729.npz", INPUTS[str(DATA / "original_train729.npz")],
                DATA / "paired_train480.npz", INPUTS[str(DATA / "paired_train480.npz")],
                checkpoint=budget.guard, cache_enabled=enabled)
            arm["startup_seconds"] = time.monotonic()-startup
            arm["startup_scope"] = "fresh_reader_whole_SHA_and_ZIP_census; OS_page_cache_not_controlled"
            writer = worker.BatchJournalWriter(OUT / (current_arm + "_RGB_GT_batches.jsonl"), STEPS)
            require(writer.maximum_bytes == JOURNAL_LIMIT, "unchanged default256KiB writer required")
            for declared in plan["selected_baseline_rows"]:
                current_step = declared["step"]
                phase = "actual_batch"
                failure_tail = dict(arm=current_arm, step=current_step, original_baseline=declared, worker_receipt=None)
                active_batch = True
                budget.guard()
                batch_start = time.monotonic()
                rgb, target, receipt = worker.build_batch(declared["baseline"], reader, crop, checkpoint=budget.guard)
                failure_tail["worker_receipt"] = receipt
                selected_counts = [int(np.count_nonzero(target[position] == 5)) for position in range(8)]
                rescues = rescue_witnesses(receipt, selected_counts)
                if current_arm not in samples and rescues:
                    samples[current_arm] = first_rescue_copy(rgb, target, receipt, rescues, worker)
                writer.append(rgb, target, receipt)
                actual = dict(step=current_step, original_RGB_GT_equal=True,
                    candidate_RGB_sha256=worker.array_hash(rgb), candidate_target_sha256=worker.array_hash(target),
                    selected_crop_boxes=receipt["journal"]["selected_crop_boxes"],
                    selected_GT_sign_pixels=selected_counts, rescued_slots=len(rescues),
                    batch_elapsed_seconds=time.monotonic()-batch_start)
                # HH_261008 - Identical measured-arm bookkeeping; asymmetric cross-arm comparison happens afterward.
                all_arm_rows[current_arm][current_step] = dict(journal=receipt["journal"], candidate=actual, full_receipt=receipt)
                arm["batches"].append(actual)
                arm["actual_rescue_witnesses"].extend(rescues)
                arm["rescued_slots"] += len(rescues)
                arm["PASS_batches"] += 1
                arm["UNEXECUTED_batches"] -= 1
                active_batch = False
                arm["last_slot_read"] = bounded_reader_witness(reader.last_read_receipt)
                arm["cache_stats"] = reader_stats(reader, enabled)
                del rgb, target, receipt
                failure_tail = None
                budget.guard()
            arm["journal_summary"] = writer.summary()
            phase = "arm_close"
            close_started = time.monotonic()
            writer.close()
            writer = None
            reader.close()
            arm["reader_closing_observations"] = closing_observations(reader)
            arm["reader_cache_stats_after_close"] = reader_stats(reader, enabled)
            reader = None
            arm["close_seconds"] = time.monotonic()-close_started
            arm["measured_arm_seconds"] = time.monotonic()-arm_started
            arm["owner_budget_guard_calls_at_arm_end"] = budget.guard_calls
            arm["owner_budget_guard_calls_during_arm"] = budget.guard_calls-arm_guard_start
            arm_active = False
        comparison_started = time.monotonic()
        phase = "post_arm_comparison"
        for current_step in STEPS:
            control, candidate = (all_arm_rows[name][current_step] for name in ARMS)
            failure_tail = dict(scope="post_arm_comparison", step=current_step,
                CONTROL_worker_receipt=control["full_receipt"], CANDIDATE_worker_receipt=candidate["full_receipt"])
            comparison_active = True
            same = typed_equal(control["journal"], candidate["journal"])
            a, b = control["candidate"], candidate["candidate"]
            require(same and a["candidate_RGB_sha256"] == b["candidate_RGB_sha256"]
                and a["candidate_target_sha256"] == b["candidate_target_sha256"]
                and typed_equal(a["selected_GT_sign_pixels"], b["selected_GT_sign_pixels"]),
                "same original rows/crop policy must reproduce BOTH candidate arm RGB and GT SHA")
            comparisons.append(dict(step=current_step, candidate_RGB_equal=True, candidate_GT_equal=True,
                same_original_and_selected_boxes_and_counts=True,
                candidate_RGB_sha256=b["candidate_RGB_sha256"], candidate_target_sha256=b["candidate_target_sha256"]))
            comparison_active = False
            failure_tail = None
            budget.guard()
        comparison_seconds = time.monotonic()-comparison_started
        require(len(comparisons) == 16, "full declared16 arm byte comparisons required")
        if samples:
            phase = "post_arm_first_rescue_sample_parity"
            require(set(samples) == set(ARMS), "same actual first-rescue sample must exist in both arms")
            a, b = (samples[name] for name in ARMS)
            require(typed_equal(a["witness"], b["witness"]) and worker.array_hash(a["RGB"]) == worker.array_hash(b["RGB"])
                and worker.array_hash(a["target"]) == worker.array_hash(b["target"]), "actual first-rescue sample differs across arms")
            phase = "post_arm_PNG"
            png = render_first_rescue(b, crop, worker, budget)
            if png["status"] == "FAILED_RETAINED":
                raise ValueError("post-arm first-rescue PNG failed")
    except Exception as error:
        primary = dict(type=type(error).__name__, message=str(error)[:1024], phase=phase, arm=current_arm, step=current_step)
        if comparison_seconds is None and "comparison_started" in locals():
            comparison_seconds = time.monotonic()-comparison_started
        if current_arm is not None:
            arm = arms[current_arm]
            arm.update(denominator(arm["PASS_batches"], active_failure=active_batch and current_step is not None))
            if arm_active:
                arm["elapsed_until_failure_seconds"] = time.monotonic()-arm_started
                arm["owner_budget_guard_calls_during_arm"] = budget.guard_calls-arm_guard_start
            if reader is not None:
                try:
                    arm["cache_stats_at_failure"] = reader_stats(reader, current_arm == ARMS[1])
                    if reader.last_read_receipt is not None:
                        arm["last_slot_read_at_failure"] = bounded_reader_witness(reader.last_read_receipt)
                except Exception as secondary:
                    cleanup.append(dict(scope="failure_reader_witness", type=type(secondary).__name__, message=str(secondary)[:1024]))
    finally:
        for name, value in (("journal", writer), ("reader", reader)):
            if value is not None:
                try:
                    value.close()
                except Exception as error:
                    cleanup.append(dict(scope=name, type=type(error).__name__, message=str(error)[:1024]))
                finally:
                    if name == "reader" and current_arm is not None:
                        try:
                            arms[current_arm]["reader_closing_observations"] = closing_observations(value)
                        except Exception as error:
                            cleanup.append(dict(scope="reader_closing_observation_metadata", type=type(error).__name__, message=str(error)[:1024]))
        archive_pins = {str(PLAN_DIR / "sources" / (pin + ".py")): pin for pin in plan["source_sha256"].values()}
        for path, pin in {**plan["source_sha256"], **INPUTS, **plan["auxiliary_metadata_sha256"], **plan["environment"]["selected_provider_sha256"],
                **plan["diagnostic_graphics_environment"]["selected_graphics_source_sha256"], **archive_pins}.items():
            observation = dict(path=path, expected_sha256=pin, observed_sha256=None, status="NOT_VERIFIED")
            try:
                observation["status"] = "NOT_CHECKED_BEFORE_HASH"
                budget.guard()
                observation["status"] = "NOT_CHECKED_HASH_INCOMPLETE"
                closing_seals[path] = digest(path, budget.guard)
                observation["observed_sha256"] = closing_seals[path]
                if path in archive_pins:
                    closing_archives[path] = closing_seals[path]
                observation["status"] = "HASH_MISMATCH"
                require(closing_seals[path] == pin, "closing seal changed")
                observation["status"] = "MATCHED"
            except Exception as error:
                failure = dict(scope="closing_seal", path=path, type=type(error).__name__, message=str(error)[:1024])
                cleanup.append(failure)
                observation["failure"] = failure
            closing_seal_observations.append(observation)
        try:
            budget.guard()
        except Exception as error:
            cleanup.append(dict(scope="closing_budget", type=type(error).__name__, message=str(error)[:1024]))
    complete = primary is None and not cleanup and len(comparisons) == 16 and all(a["PASS_batches"] == 16 for a in arms.values())
    observed_rescue = any(a["rescued_slots"] > 0 for a in arms.values())
    complete_same_rescue = complete and all(a["rescued_slots"] > 0 for a in arms.values())
    if observed_rescue and png["status"] == "NOT_CREATED_RESCUE_NOT_OBSERVED":
        png = dict(status="NOT_CREATED_ARMS_INCOMPLETE", actual_rescue_observed=True)
    result = dict(schema=SCHEMA + ".result", provenance="HH_261008 - Actual FIRST16 same-code cache byte A/B; no learning.",
        status=("COMPLETE_BYTES_ACTUAL_RESCUE_OBSERVED" if complete_same_rescue else "COMPLETE_BYTES_RESCUE_NOT_OBSERVED") if complete else "FAILED_RETAINED",
        arms=arms, comparisons=comparisons, declared_steps=STEPS,
        original20k_batches_outside_declaration_per_arm=19984,
        actual_rescue_observed=observed_rescue, rescue_policy_effect_on_actual_RGB_GT_observed=complete_same_rescue,
        failure=primary, cleanup_errors=cleanup, full_failed_tail=failure_tail,
        diagnostic_PNG=png, sampled_closing_memory=budget.last, sampled_maximum_VmHWM_bytes=budget.maximum_HWM,
        elapsed_seconds=time.monotonic()-budget.started, owner_budget_guard_calls_total=budget.guard_calls,
        post_arm_comparison_seconds=comparison_seconds,
        ROOT_plan_sha256=plan_pin, ROOT_authority_sha256=grant_pin, actual_array_diagnostic_only=True,
        upfront_seal_sha256=upfront_seals, closing_seal_sha256=closing_seals,
        upfront_source_archive_sha256=upfront_archives, closing_source_archive_sha256=closing_archives,
        closing_seal_observations=closing_seal_observations,
        training_steps=0, NN=0, architecture_changed=False, model_performance_improved=False,
        formal_main_approved=0, formal_sub_approved=0, historical_provider_equivalence_claimed=False,
        controlled_OS_cold_start_claimed=False, fixed_sequential_arm_order=list(ARMS),
        timing_scope="same V3 code+guard cadence; cache bool differs; OS page-cache/order/native checkpoint accounting limits remain",
        old8_batch74second_run_used_as_comparable_timing_baseline=False, current_process_generation=process_identity(),
        original_journals_or_frozen_trainer_modified=False, partial_prefix_decode_not_only_selected_rows=True)
    result.update({"comparison_"+key: value for key, value in denominator(len(comparisons),
        active_failure=primary is not None and comparison_active).items()})
    require(all(arm["PASS_batches"]+arm["FAIL_batches"]+arm["UNEXECUTED_batches"] == 16 for arm in arms.values()),
        "declared per-arm16 failed-attempt denominator changed")
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
