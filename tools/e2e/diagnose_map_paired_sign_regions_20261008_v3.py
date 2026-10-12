#!/usr/bin/env python3
"""HH_261008 - Compare the same complete GT mask regions across three frozen arms.

HH_261008 - V3 preserves stopped V1 and V2 pure failures; normal-row allocation
is stricter and proven from the exact canonical producer shape, never truncated.
Paired changes are GT-region NET TP counts,
not newly correct/now-wrong pixel sets or persisted per-region locations.
SOURCE-READY only until a separately reviewed RAW ROOT authority admits a run.
An 8-connected label region is NOT an object instance, a road topology graph or
a confidence score. This diagnostic changes no masks, thresholds, weights or
original 27 exploratory predicates. Only selected target/prediction NPY members
are decoded; RGB, raw captures, TRAIN tensors, weights and models are forbidden.
"""
import argparse
import ast
from contextlib import ExitStack
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import re
import resource
import signal
import struct
import sys
import time
from types import SimpleNamespace
import zipfile

ROOT = Path(__file__).resolve().parents[2]
NAME = "map_paired_sign_regions_v3"
SCHEMA = "portable_e2e.map_paired_sign_regions.20261008.v3"
PLAN_ROOT = ROOT / "storage/experiments/plans/2026-10-08" / NAME
CANDIDATE = PLAN_ROOT / "prepared_candidate.json"
AUTHORITY = PLAN_ROOT / "ROOT_authority.json"
REVIEW = PLAN_ROOT / "ROOT_review.json"
SKELETON = PLAN_ROOT / "PLAN_skeleton.json"
AUTHORITY_SKELETON = PLAN_ROOT / "ROOT_authority.skeleton.json"
INVESTIGATION = ROOT / "storage/experiments/diagnostics/2026-10-08/mapping_nontraining_next_task_source_review_v1.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-08" / NAME
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
TEST = ROOT / "learning/tests/test_diagnose_map_paired_sign_regions_20261008_v3.py"
OLD = ROOT / "tools/e2e/diagnose_map_sign_components_20261007_v2.py"
OLD_RESULT = ROOT / "storage/experiments/diagnostics/2026-10-07/map_sign_components_v2/result.json"
RESULTS = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/results.json"
RUN = ROOT / "storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
DATA = ROOT / "storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1"
MANIFEST = RUN / "inputs/manifest.json"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
COHORTS = (("dev_town01", 118), ("holdout_town02_p02", 232))
ARMS = ("parent", "control", "candidate")
HW = (360, 640)
PIXELS = 360 * 640
LIMIT, CLOSING, RSS, DEADLINE = 2 * 1024**2, 512 * 1024, 512 * 1024**2, 900
WORKING = LIMIT - CLOSING
FLOOR, RESERVE, FRAME_RECORD_CAP = 2 * 1024**3, 64 * 1024**2, 2560
ROOT_METADATA_CAP = 32 * 1024
PACKAGES = {"numpy": "1.21.5", "scipy": "1.8.0"}
SCREEN_SOURCES = tuple(ROOT / "tools/e2e" / name for name in (
    "publish_map_sign_sampler_v2_comparison_20261002.py", "publish_map_sign_margin_loss_results_20261002.py"))
OLD_SOURCE_PINS = {
    str(OLD): "cb72db2bc043166259db66a3276d8e3edfa51ce390dc0b93428ac61316198ef8",
    str(ROOT / "learning/tests/test_diagnose_map_sign_components_20261007_v2.py"): "72f2cb8ad1c8d41768efdce4696168547757c02fb62b24063e75bf52810646b5",
    str(SCREEN_SOURCES[0]): "fd939a5ad05949523677dab947274d83f6226e27711ad2ee80977b78cd40aa42",
    str(SCREEN_SOURCES[1]): "f9b4b9096c83593d6ab924d94fc858356730c5b32fa122b4b18aa8acbe7cad0e",
    str(ROOT / "tools/e2e/publish_map_paired_balanced_results_20261002.py"): "33c717ddf51c1dc6aa2e56c15c8e131d5866983c97fc19b0421a5c0bf9ba1808",
}
META_PINS = {
    str(RESULTS): "a367c1f594cc5518ac6d6a0df8bff8aa51cd946395108967870fc948fe7fcd7b",
    str(MANIFEST): "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b",
    str(OLD_RESULT): "bdf4fa1583e73a74eaec8121895416e0f2539f4ed3e7f75cddd42c214d3009b6",
    str(ROOT / "storage/experiments/diagnostics/2026-10-07/map_sign_components_v2_independent_postcheck_v1.json"): "d998f84ecde70c1bfb3abc0656b8f18f8e4d963fb8bd7c5380bee5809675dbe5",
    str(ROOT / "storage/experiments/plans/2026-10-07/map_sign_components_v2/ROOT_authority.json"): "ebd2f51904914141ae0aaea95918b286c2e71ea7167a38df2a0f56e00b4636ae",
    str(ROOT / "storage/experiments/plans/2026-10-07/map_sign_components_v2/ROOT_review.json"): "67789d0f9a84e725a6f26bb2f0e9c48de958f62ba50bc77a08e67db25138cee9",
    # HH_261008 - Preserve the earlier before-array failure and all its 1050 unexecuted contexts.
    str(ROOT / "storage/experiments/diagnostics/2026-10-07/map_sign_components_v1/result.json"): "1ba8add2fae9a6389675b11221d8111bb3c0d1945cd14357afe633aa19cf6fb0",
}

# HH_261008 - Preserve the five stopped SOURCE-only V1 files and ROOT's finding; no prior actual run/test is invented.
V1_SOURCE_PINS = {
    str(ROOT / "tools/e2e/diagnose_map_paired_sign_regions_20261008_v1.py"): "ea1ff93a3270de80fa73599311efa8872092bd95440f10e53ae90c25f33e4bb2",
    str(ROOT / "learning/tests/test_diagnose_map_paired_sign_regions_20261008_v1.py"): "731bd012d4f241b838d1a84c30928a4ca0c71b7ff034349a5c402632c039cd06",
    str(ROOT / "storage/experiments/plans/2026-10-08/map_paired_sign_regions_v1/PLAN_skeleton.json"): "c7ea8283472196bbcd6693fcba30ffd34ade64a6ed9cf69ac12c689385bf8a5a",
    str(ROOT / "storage/experiments/plans/2026-10-08/map_paired_sign_regions_v1/ROOT_authority.skeleton.json"): "fba3a0f46d73de55cea261bba5639359f776e806ae98dccf64394372e844b28e",
    str(INVESTIGATION): "a7884502bc9d9a9fd34455e7b42eb6f2203a692a2a584bbf61e9c2894bc2a7c1",
}
V1_FINDING_PINS = {
    str(ROOT / "storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v1_source_finding_v1.json"): "f2463d216abdb0daf6fd13b1adcdc2220f76272f7064ea016ff55190396339b7",
}
META_PINS.update(V1_FINDING_PINS)


# HH_261008 - Keep the actual V2 scalar failures and its four immutable sources; preparation/arrays never followed that failure.
V2_SOURCE_PINS = {
    str(ROOT / "tools/e2e/diagnose_map_paired_sign_regions_20261008_v2.py"): "28ef01bbdbd24a34a4c67a2b227529debb1dd8c23fc7533ef48102f101a9889c",
    str(ROOT / "learning/tests/test_diagnose_map_paired_sign_regions_20261008_v2.py"): "9d1ddab41b6672f13ac3d94b33fedb3656bfa0eaaad3d3a0becc5402f886ca2f",
    str(ROOT / "storage/experiments/plans/2026-10-08/map_paired_sign_regions_v2/PLAN_skeleton.json"): "227395a9c965849eec719bb13cf4f26bf0e4af6081dc16b90b28213d860df7dc",
    str(ROOT / "storage/experiments/plans/2026-10-08/map_paired_sign_regions_v2/ROOT_authority.skeleton.json"): "02af5c7097c03cef6380bb6d2e1288ef3cad7b86146eb7bb1a47a2ad818e9a63",
}
V2_FAILURE_PINS = {
    str(ROOT / "storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v2_source_test_failure_v1.json"): "eadd369b11c7a6b4acdce5d9ae5d4fd13120a3bf2a25562afe85a012f81578ed",
}
META_PINS.update(V2_FAILURE_PINS)

ASSETS = {}
for cohort, target_sha, predictions in (
    ("dev_town01", "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3", (
        "c335375371ef30b07fb5042fbfbed902bb0758c0d15b307ccf683fdc3d79472a",
        "f05096b04655762543f82e5ed7390de917beaa88f4c100ba83bc39aed3c50988",
        "0f9f6a3f2db8aea63b5778e2c2e57444dd141c75149ebe5c626cd1310ebf94f1")),
    ("holdout_town02_p02", "858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb", (
        "8d8f47c4ec709acd19f0f49d0ecd214b068725125d34f5e8f8309597f1ad3b4b",
        "e31b9f4ec787870a855b6b2ca285bfbaa6c04ac431f0691331ab620bf17929fb",
        "ff55003d3f71aa8c83e62cfaff1673a028549b49170d12d9e68cd415f3973ee5"))):
    ASSETS[str(DATA / (cohort + ".npz"))] = target_sha
    for arm, sha in zip(ARMS, predictions):
        ASSETS[str(RUN / ("parent_diagnostic" if arm == "parent" else arm) / (cohort + "_predictions.npz"))] = sha
GRAPH_COLUMNS = (
    "GT_regions", "GT_pixels", "predicted_regions", "predicted_pixels", "TP_pixels", "FN_pixels", "FP_pixels",
    "overlap_edges", "missed_GT_regions", "one_overlap_GT_regions", "split_GT_regions",
    "isolated_predicted_regions", "one_GT_predicted_regions", "merged_predicted_regions",
    "isolated_FP_pixels", "overlapping_region_FP_pixels", "fully_covered_GT_regions", "partly_covered_GT_regions")
PAIR_COLUMNS = (
    "GT_regions", "GT_pixels", "reference_TP_pixels", "candidate_TP_pixels", "GT_region_net_TP_count_increase_pixels", "GT_region_net_TP_count_decrease_pixels",
    "equal_TP_regions", "increased_TP_regions", "decreased_TP_regions", "fully_lost_regions", "newly_hit_regions",
    "both_missed_regions", "both_hit_regions", "fully_lost_GT_pixels", "newly_hit_GT_pixels",
    *(f"reference_degree_{a}_candidate_degree_{b}" for a in ("zero", "one", "many") for b in ("zero", "one", "many")))


def require(value, message):
    if not value:
        raise ValueError(message)


def sha_text(value):
    return type(value) is str and re.fullmatch("[0-9a-f]{64}", value) is not None


def canonical(path):
    path = Path(path)
    require(path == path.resolve(strict=True) and path.is_file() and not path.is_symlink(), "canonical regular file required")
    return path


def fingerprint(path):
    st = canonical(path).stat()
    return tuple(getattr(st, name) for name in ("st_dev", "st_ino", "st_mode", "st_uid", "st_gid", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns"))


def digest(path, checkpoint=lambda: None):
    path = canonical(path); before = fingerprint(path); h = hashlib.sha256()
    with path.open("rb") as stream:
        for raw in iter(lambda: stream.read(1024**2), b""):
            checkpoint(); h.update(raw)
    require(fingerprint(path) == before, "file changed while hashing: " + str(path))
    return h.hexdigest()


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def read_json(path, cap=1024**2):
    require(canonical(path).stat().st_size <= cap, "bounded metadata required")
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key"); result[key] = value
        return result
    return json.loads(path.read_bytes(), object_pairs_hook=pairs,
                      parse_constant=lambda _: require(False, "nonfinite JSON denied"))


def verify_pins(pins, checkpoint=lambda: None):
    require(type(pins) is dict and pins, "nonempty exact pins required")
    for path, sha in pins.items():
        checkpoint()
        require(sha_text(sha) and digest(path, checkpoint) == sha, "source/input/provider seal changed: " + path)


def sources():
    paths = [Path(__file__).resolve(), TEST, SKELETON, AUTHORITY_SKELETON, INVESTIGATION, *map(Path, OLD_SOURCE_PINS),
             *map(Path, V1_SOURCE_PINS), *map(Path, V1_FINDING_PINS),
             *map(Path, V2_SOURCE_PINS), *map(Path, V2_FAILURE_PINS)]
    result = {str(path): digest(path) for path in paths}
    require(all(result[key] == value for key, value in OLD_SOURCE_PINS.items()), "frozen source changed")
    require(all(result[key] == value for key, value in V1_SOURCE_PINS.items()), "stopped V1 source/skeleton/investigation changed")
    require(all(result[key] == value for key, value in V1_FINDING_PINS.items()), "stopped V1 source finding changed")
    require(all(result[key] == value for key, value in V2_SOURCE_PINS.items()), "failed V2 source/skeleton changed")
    require(all(result[key] == value for key, value in V2_FAILURE_PINS.items()), "actual V2 pure failure changed")
    return result


def provider_pins():
    """HH_261008 - Pin selected existing provider files without importing numeric packages."""
    require(not {"numpy", "scipy", "torch"} & set(sys.modules), "numeric modules must not precede prepare/admission")
    fixed = {
        "numpy": ("__init__.py", "version.py", "core/__init__.py", "core/multiarray.py", "core/overrides.py",
                  "core/numeric.py", "core/fromnumeric.py", "lib/arraysetops.py", "lib/function_base.py"),
        "scipy": ("__init__.py", "version.py", "ndimage/__init__.py", "ndimage/_measurements.py", "ndimage/_ni_support.py"),
    }
    pins, origins = {}, {}
    for package, relatives in fixed.items():
        require(importlib.metadata.version(package) == PACKAGES[package], "installed provider version differs")
        spec = importlib.util.find_spec(package)
        require(spec is not None and type(spec.origin) is str, "existing top-level provider origin required")
        base = canonical(Path(spec.origin)).parent; origins[package] = str(base / "__init__.py")
        selected = [base / name for name in relatives]
        patterns = ("core/_multiarray_umath*.so",) if package == "numpy" else ("ndimage/_nd_image*.so", "ndimage/_ni_label*.so")
        binaries = []
        for pattern in patterns:
            matching = list(base.glob(pattern))
            require(len(matching) == 1, "one selected compiled provider required for "+pattern)
            binaries.extend(matching)
        for path in [*selected, *binaries]:
            pins[str(canonical(path))] = digest(path)
    require(not {"numpy", "scipy", "torch"} & set(sys.modules), "provider discovery imported numeric code")
    return {"selected_file_sha256": pins, "top_level_origins": origins,
            "complete_environment_or_binary_dependency_equivalence_claimed": False}


def declared_contract():
    return dict(schema=SCHEMA, labels=list(LABELS), sign_index=5, cohorts=dict(COHORTS), arms=list(ARMS),
        native_HW=list(HW), GT_frame_denominator=350, arm_frame_denominator=1050, connectivity=8,
        region_identity="fixed GT scan-order labels; an 8-connected mask region is not an object instance",
        paired_support_count_change_kind="sum_positive_negative_per_GT_region_TP_count_delta_not_pixel_set_change",
        actual_newly_correct_or_lost_pixel_sets_verified=False,region_locations_or_vectors_persisted=False,
        equal_TP_count_can_move_pixel_support=True,maximum_output_bytes_scope="OUTPUT_only_not_combined_PLAN_ROOT",
        order="cohort declaration, frame ordinal, parent/control/candidate", graph_columns=list(GRAPH_COLUMNS),
        paired_columns=list(PAIR_COLUMNS), comparison_references=["parent", "control"],
        only_decoded_members=["target.npy", "prediction.npy"], input_sha256={**ASSETS, **META_PINS},
        maximum_output_bytes=LIMIT, working_bytes=WORKING, closing_reserved_bytes=CLOSING,
        frame_record_cap_bytes=FRAME_RECORD_CAP, free_floor_bytes=FLOOR, extra_free_reserve_bytes=RESERVE,
        per_RAW_ROOT_authority_or_review_cap_bytes=ROOT_METADATA_CAP,
        current_process_VmRSS_and_VmHWM_limit_bytes=RSS, observed_guard_not_OS_sandbox=True, deadline_seconds=DEADLINE,
        one_CPU=True, runtime_packages=PACKAGES, runtime_lock=str(LOCK), RAW_ROOT_authority_required=True,
        no_self_grant=True, model_inferences=0, training_steps=0, GPU_used=False, RGB_decoded=False,
        raw_capture_or_TRAIN_or_weights_read=False, thresholds_or_masks_changed=False, original_27_criteria_unchanged=True,
        new_independent_DEV_sites=0, semantic_object_identity_verified=False, physical_HD_topology_verified=False,
        calibration_or_crop_causation_proved=False, model_performance_improved=False, formal_main_approved=0, formal_sub_approved=0)


def npy_header(stream, count):
    require(type(count) is int and count > 0, "typed NPY count required")
    require(stream.read(6) == b"\x93NUMPY", "NPY magic differs")
    version = stream.read(2); require(version in (b"\x01\x00", b"\x02\x00", b"\x03\x00"), "NPY version denied")
    raw = stream.read(2 if version == b"\x01\x00" else 4)
    require(len(raw) == (2 if version == b"\x01\x00" else 4), "truncated header length")
    length = int.from_bytes(raw, "little"); require(0 < length <= 16384, "bounded NPY header required")
    raw = stream.read(length); require(len(raw) == length, "truncated header")
    header = ast.literal_eval(raw.decode("utf-8" if version == b"\x03\x00" else "latin1"))
    require(type(header) is dict and set(header) == {"descr", "fortran_order", "shape"}
        and header["descr"] == "|u1" and header["fortran_order"] is False
        and type(header["shape"]) is tuple and all(type(v) is int for v in header["shape"])
        and header["shape"] == (count, *HW), "exact C-order uint8 native mask ABI required")
    return header


def checked_entry(archive, member, count):
    require(member in ("target.npy", "prediction.npy") and type(count) is int and count > 0, "selected mask-only ABI required")
    expected = ["rgb.npy", "target.npy"] if member == "target.npy" else ["prediction.npy"]
    require(sorted(archive.namelist()) == expected, "exact ZIP census; no paths/duplicates/foreign entries")
    info = archive.getinfo(member); total = count * PIXELS
    require(not info.flag_bits & 1 and info.compress_type in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)
        and total < info.file_size <= total + 16400 and info.compress_size <= total + 16400 + 256 * 1024,
        "bounded unencrypted selected member required")
    return info


class MaskStream:
    """HH_261008 - Reuse one uint8 byte buffer; selected-member CRC is required at EOF."""
    def __init__(self, path, member, count):
        self.path, self.member, self.count = canonical(path), member, count
        self.before = fingerprint(self.path); self.file = self.path.open("rb")
        self.archive = None; self.stream = None; self.buffer = bytearray(PIXELS)
        self.ordinal = 0; self.pixel_sha = hashlib.sha256(); self.complete = False
        try:
            self.archive = zipfile.ZipFile(self.file)
            info = checked_entry(self.archive, member, count)
            self.stream = self.archive.open(info)
            npy_header(self.stream, count)
        except BaseException:
            self.close(); raise

    def read_frame(self, np):
        require(type(self.ordinal) is int and self.ordinal < self.count, "extra frame read denied before progress mutation")
        offset = 0
        view = memoryview(self.buffer)
        try:
            while offset < PIXELS:
                part = view[offset:]
                try:
                    got = self.stream.readinto(part)
                finally:
                    part.release()
                require(type(got) is int and 0 < got <= PIXELS - offset, "incomplete/invalid frame bytes")
                offset += got
        finally:
            view.release()
        require(max(self.buffer) < 7, "foreign learned label denied")
        frame_sha = hashlib.sha256(self.buffer).hexdigest()
        value = np.frombuffer(self.buffer, dtype=np.uint8).reshape(HW)
        self.pixel_sha.update(self.buffer); self.ordinal += 1
        return value, frame_sha

    def finish(self):
        require(self.ordinal == self.count, "incomplete full selected-member denominator")
        require(self.stream.read(1) == b"", "trailing selected-member bytes")
        # ZipExtFile checks the selected member CRC on complete consumption.
        require(fingerprint(self.path) == self.before and
                tuple(getattr(os.fstat(self.file.fileno()), k) for k in
                ("st_dev", "st_ino", "st_mode", "st_uid", "st_gid", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns")) == self.before,
                "path/held-FD identity changed during selected-member replay")
        self.complete = True
        return self.pixel_sha.hexdigest()

    def close(self):
        if self.stream is not None: self.stream.close()
        if self.archive is not None: self.archive.close()
        if getattr(self, "file", None) is not None: self.file.close()

    def __enter__(self): return self
    def __exit__(self, *_): self.close()


def areas_valid(values):
    return (type(values) in (tuple, list) and len(values) <= PIXELS
            and all(type(v) is int and 1 <= v <= PIXELS for v in values) and sum(values) <= PIXELS)


def graph_summary(gt_areas, predicted_areas, edges):
    """HH_261008 - Keep the whole sparse overlap graph, not a fitted overlap threshold."""
    require(areas_valid(gt_areas) and areas_valid(predicted_areas), "bounded positive region areas required")
    require(type(edges) in (tuple, list) and len(edges) <= PIXELS, "bounded sparse edges required")
    ng, npred = len(gt_areas), len(predicted_areas)
    gt_tp, pred_tp, gd, pd = [0] * ng, [0] * npred, [0] * ng, [0] * npred
    previous = (0, 0); h = hashlib.sha256()
    for row in edges:
        require(type(row) in (tuple, list) and len(row) == 3 and all(type(v) is int for v in row), "typed sparse edge required")
        g, p, overlap = row
        require(1 <= g <= ng and 1 <= p <= npred and 0 < overlap <= PIXELS and (g, p) > previous,
                "ordered unique positive bounded overlap edge required")
        gt_tp[g-1] += overlap; pred_tp[p-1] += overlap; gd[g-1] += 1; pd[p-1] += 1
        previous = (g, p); h.update(struct.pack("<qqq", g, p, overlap))
    require(all(v <= a for v, a in zip(gt_tp, gt_areas)) and all(v <= a for v, a in zip(pred_tp, predicted_areas)),
            "overlap exceeds exact region support")
    tp = sum(gt_tp); require(tp == sum(pred_tp), "bipartite TP partition differs")
    counts = [ng, sum(gt_areas), npred, sum(predicted_areas), tp, sum(gt_areas)-tp, sum(predicted_areas)-tp,
        len(edges), sum(d == 0 for d in gd), sum(d == 1 for d in gd), sum(d > 1 for d in gd),
        sum(d == 0 for d in pd), sum(d == 1 for d in pd), sum(d > 1 for d in pd),
        sum(a for a, d in zip(predicted_areas, pd) if d == 0),
        sum(a-v for a, v, d in zip(predicted_areas, pred_tp, pd) if d > 0),
        sum(v == a for v, a in zip(gt_tp, gt_areas)), sum(0 < v < a for v, a in zip(gt_tp, gt_areas))]
    require(counts[8]+counts[9]+counts[10] == ng and counts[11]+counts[12]+counts[13] == npred
            and counts[14]+counts[15] == counts[6] and counts[8]+counts[16]+counts[17] == ng, "full region/error partition differs")
    return dict(counts=counts, GT_TP=tuple(gt_tp), GT_degree=tuple(gd), edge_sha256=h.hexdigest())


def paired_summary(gt_areas, reference, candidate):
    require(areas_valid(gt_areas), "exact fixed GT areas required")
    vectors = [o.get(k) for o in (reference, candidate) for k in ("GT_TP", "GT_degree")]
    require(all(type(v) is tuple and len(v) == len(gt_areas) and all(type(n) is int and 0 <= n <= PIXELS for n in v)
                for v in vectors), "same typed full GT-region vectors required")
    rtp, rd, ctp, cd = vectors
    require(all(t <= a for a, t in zip(gt_areas, rtp)) and all(t <= a for a, t in zip(gt_areas, ctp))
        and all((t == 0) == (d == 0) for t, d in zip(rtp, rd))
        and all((t == 0) == (d == 0) for t, d in zip(ctp, cd)), "GT degree/support contradiction")
    counts = [len(gt_areas), sum(gt_areas), sum(rtp), sum(ctp),
        sum(max(c-r, 0) for r, c in zip(rtp, ctp)), sum(max(r-c, 0) for r, c in zip(rtp, ctp)),
        sum(r == c for r, c in zip(rtp, ctp)), sum(c > r for r, c in zip(rtp, ctp)), sum(c < r for r, c in zip(rtp, ctp)),
        sum(r > 0 and c == 0 for r, c in zip(rtp, ctp)), sum(r == 0 and c > 0 for r, c in zip(rtp, ctp)),
        sum(r == c == 0 for r, c in zip(rtp, ctp)), sum(r > 0 and c > 0 for r, c in zip(rtp, ctp)),
        sum(a for a, r, c in zip(gt_areas, rtp, ctp) if r > 0 and c == 0),
        sum(a for a, r, c in zip(gt_areas, rtp, ctp) if r == 0 and c > 0)]
    transition = [0] * 9
    for a, b in zip(rd, cd): transition[min(a, 2)*3+min(b, 2)] += 1
    counts.extend(transition)
    require(sum(transition) == len(gt_areas) == sum(counts[6:9])
        and counts[3]-counts[2] == counts[4]-counts[5] and sum(counts[9:13]) == len(gt_areas), "whole paired region partition differs")
    return counts


def native_regions(mask, np, ndimage):
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=np.uint8))
    require(type(count) is int and 0 <= count <= PIXELS and labels.shape == HW and labels.dtype == np.int32,
            "exact native connected-label ABI required")
    areas = np.bincount(labels.ravel(), minlength=count+1)
    require(len(areas) == count+1 and sum(int(v) for v in areas) == PIXELS, "complete label census required")
    actual = tuple(int(v) for v in areas[1:])
    require(areas_valid(actual) and sum(actual) == int(mask.sum()), "all sign region support must be retained")
    values, first = np.unique(labels.ravel(), return_index=True)
    positive_first = [int(i) for v, i in zip(values, first) if int(v) != 0]
    require(positive_first == sorted(positive_first), "fixed raster-order region identity differs")
    return labels, actual


def native_graph(gt_labels, gt_areas, pred, np, ndimage):
    predicted, areas = native_regions(pred == 5, np, ndimage)
    common = (gt_labels > 0) & (predicted > 0)
    base = len(areas)+1
    packed = gt_labels[common].astype(np.int64)*base + predicted[common].astype(np.int64)
    keys, counts = np.unique(packed, return_counts=True)
    edges = [(int(k)//base, int(k)%base, int(n)) for k, n in zip(keys, counts)]
    result = graph_summary(gt_areas, areas, edges)
    require(result["counts"][4] == int(common.sum()), "whole native overlap TP differs")
    return result


def metrics(matrix):
    # HH_261008 - Keep the frozen Oct7 scalar metrics and null absent-class IoU exactly.
    require(len(matrix) == 7 and all(len(r) == 7 and all(type(v) is int and v >= 0 for v in r) for r in matrix), '49 confusion integers required')
    actual = [sum(r) for r in matrix]; predicted = [sum(r[i] for r in matrix) for i in range(7)]
    diagonal = [matrix[i][i] for i in range(7)]; union = [a + p - d for a, p, d in zip(actual, predicted, diagonal)]
    require(sum(actual) > 0, 'empty metric denominator')
    return dict(confusion=matrix, target_pixels=dict(zip(LABELS, actual)), predicted_pixels=dict(zip(LABELS, predicted)),
        true_positive_pixels=dict(zip(LABELS, diagonal)), false_negative_pixels=dict(zip(LABELS, [a-d for a,d in zip(actual,diagonal)])),
        false_positive_pixels=dict(zip(LABELS, [p-d for p,d in zip(predicted,diagonal)])),
        iou={k: diagonal[i]/union[i] if actual[i] else None for i,k in enumerate(LABELS)},
        precision={k: diagonal[i]/predicted[i] if predicted[i] else None for i,k in enumerate(LABELS)},
        recall={k: diagonal[i]/actual[i] if actual[i] else None for i,k in enumerate(LABELS)},
        pixel_accuracy=sum(diagonal)/sum(actual), absent_target_class_iou='null; false positives retained',
        other_to_sign_false_positive_pixels=matrix[0][5])


def original_screens(evaluation):
    previous = None
    for path in SCREEN_SOURCES:
        require(digest(path) == OLD_SOURCE_PINS[str(path)], "original predicate source changed")
        functions = [n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "diagnostic_screen"]
        require(len(functions) == 1 and not functions[0].decorator_list, "unique genuine predicate AST required")
        namespace = {"require": require, "previous": SimpleNamespace(diagnostic_screen=previous)}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
        previous = namespace["diagnostic_screen"]
    return previous(evaluation)


def ledger():
    return [[cohort, ordinal, arm, "UNEXECUTED"] for cohort, count in COHORTS for ordinal in range(count) for arm in ARMS]


def progress(rows, slot, status):
    require(type(slot) is int and 0 <= slot < len(rows) == 1050, "progress slot outside complete denominator")
    require(status in ("FRAME_CHECKED_PENDING_ARCHIVE_EOF", "PASS", "FAIL"), "unknown progress status")
    old = rows[slot][3]
    require(old == "UNEXECUTED" and status in ("FRAME_CHECKED_PENDING_ARCHIVE_EOF", "FAIL") or
            old == "FRAME_CHECKED_PENDING_ARCHIVE_EOF" and status in ("PASS", "FAIL"), "progress order/retry denied")
    rows[slot][3] = status


def parse_process_memory(text, pid):
    require(type(text) is str and 0 < len(text) <= 16384 and type(pid) is int and pid > 0, "bounded self status required")
    values = {}
    for line in text.splitlines():
        key, separator, raw = line.partition(":")
        if key not in ("Pid", "Tgid", "VmRSS", "VmHWM"): continue
        require(separator and key not in values, "duplicate/malformed self status field")
        parts = raw.split(); memory = key.startswith("Vm")
        require(len(parts) == (2 if memory else 1) and re.fullmatch("[0-9]+", parts[0])
                and (not memory or parts[1] == "kB"), "typed process status units required")
        value = int(parts[0]); require(value > 0, "zero process status denied")
        values[key] = value * 1024 if memory else value
    require(set(values) == {"Pid", "Tgid", "VmRSS", "VmHWM"} and values["Pid"] == values["Tgid"] == pid
            and values["VmHWM"] >= values["VmRSS"], "missing/foreign/impossible process memory")
    return dict(PID=pid, VmRSS_bytes=values["VmRSS"], VmHWM_bytes=values["VmHWM"])


def process_memory():
    with Path("/proc/self/status").open(encoding="ascii") as stream:
        witness = parse_process_memory(stream.read(16385), os.getpid())
    witness["inherited_ru_maxrss_bytes_NOT_GATE"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    return witness


def process_generation():
    raw = Path("/proc/self/stat").read_text()
    require(0 < len(raw) <= 16384 and raw.find("(") > 0 and raw.rfind(")") > raw.find("(")
            and raw[:raw.find("(")].strip() == str(os.getpid()), "bounded exact current PID stat required")
    suffix = raw[raw.rfind(")")+2:].split()
    require(len(suffix) >= 20 and suffix[19].isdigit(), "bounded process start ticks required")
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    require(re.fullmatch("[0-9a-f]{8}(-[0-9a-f]{4}){3}-[0-9a-f]{12}", boot), "kernel boot identity required")
    return dict(pid=os.getpid(), start_ticks=int(suffix[19]), boot_id=boot, uid=os.getuid(),
                ppid=os.getppid(), pgid=os.getpgrp(), session=os.getsid(0))


def guard(start, generation, lease=None):
    require(time.monotonic()-start < DEADLINE, "whole diagnostic deadline exceeded")
    require(process_generation() == generation, "current PID generation changed")
    if lease is not None:
        st = os.fstat(lease["fd"]); current = canonical(LOCK).stat()
        require((st.st_dev, st.st_ino) == (current.st_dev, current.st_ino) == (lease["dev"], lease["ino"]),
                "canonical held lease FD/path inode changed")
    memory = process_memory()
    require(max(memory["VmRSS_bytes"], memory["VmHWM_bytes"]) <= RSS, "sampled current-process memory cap exceeded")
    fs = os.statvfs(OUTPUT.parent)
    require(fs.f_bavail * fs.f_frsize >= FLOOR + RESERVE, "unchanged floor/reserve unavailable")
    return memory


def tree_bytes(path):
    total = 0
    for child in Path(path).rglob("*"):
        require(not child.is_symlink() and (child.is_file() or child.is_dir()), "unexpected evidence alias/type")
        if child.is_file(): total += child.stat().st_size
    return total


def write_raw(path, payload, cap=WORKING):
    require(type(payload) is bytes and path.parent == OUTPUT and OUTPUT == OUTPUT.resolve(strict=True), "fresh direct evidence file required")
    require(tree_bytes(OUTPUT)+len(payload) <= cap, "whole evidence budget exceeded before write")
    with path.open("xb") as stream:
        require(stream.write(payload) == len(payload), "partial evidence write")


def append_frame(stream, row):
    payload = encoded(row)
    require(len(payload) <= FRAME_RECORD_CAP and tree_bytes(OUTPUT)+len(payload) <= WORKING,
            "complete frame receipt exceeds declared working budget; never truncate")
    require(stream.write(payload) == len(payload), "partial frame receipt write")
    stream.flush()
    return payload



def normal_frame_row_capacity_proof():
    """HH_261008 - Bound only the exact normal producer shape; arbitrary custom rows are NOT guaranteed losslessly retained."""
    require(len(GRAPH_COLUMNS) == 18 and len(PAIR_COLUMNS) == 24 and PIXELS == 230400
        and COHORTS == (("dev_town01", 118), ("holdout_town02_p02", 232))
        and ARMS == ("parent", "control", "candidate"), "unchanged complete normal-row scope required")
    sha = "f" * 64
    maximum = dict(cohort=max((name for name, count in COHORTS), key=len), ordinal=max(count for name, count in COHORTS)-1,
        GT_pixel_sha256=sha, GT_scan_order_region_labels_i32_sha256=sha, GT_scan_order_area_i64_sha256=sha,
        arms=[dict(arm=arm, prediction_pixel_sha256=sha, confusion_sha256=sha,
                   sparse_overlap_edge_sha256=sha, counts=[PIXELS]*len(GRAPH_COLUMNS)) for arm in ARMS],
        paired=[dict(reference=reference, counts=[PIXELS]*len(PAIR_COLUMNS)) for reference in ("parent", "control")],
        status="FRAME_CHECKED_PENDING_ARCHIVE_EOF")
    size = len(encoded(maximum))
    require(size <= FRAME_RECORD_CAP == 2560, "full canonical normal-row structural bound exceeds stricter allocation")
    return dict(provenance="HH_261008 - Synthetic fieldwise maximum ASCII/integer row, not observed masks or numerical replay.",
        maximum_complete_normal_row_bytes=size, stricter_frame_record_allocation_bytes=FRAME_RECORD_CAP,
        graph_count_entries_per_arm=18, paired_count_entries_per_reference=24, arm_count=3, comparator_count=2,
        all_normal_count_fields_nonnegative_bounded_by_native_PIXELS=PIXELS, SHA256_text_fields=12, SHA256_text_characters=64,
        exact_cohorts=[name for name, count in COHORTS], maximum_ordinal=231,
        fixed_status="FRAME_CHECKED_PENDING_ARCHIVE_EOF", original_normal_producer_structure_required=True,
        arbitrary_row_size_guarantee=False, oversized_actual_row_truncated=False,
        total_OUTPUT_cap_bytes_unchanged=LIMIT, closing_bytes_unchanged=CLOSING)


def prepare():
    """Create only a prospective candidate; it is never an execution authority."""
    require(not CANDIDATE.exists() and not OUTPUT.exists(), "fresh candidate/output only")
    row_capacity = normal_frame_row_capacity_proof()
    with canonical(LOCK).open("rb") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        source = sources(); verify_pins({**ASSETS, **META_PINS})
        providers = provider_pins(); headers = {}
        for cohort, count in COHORTS:
            for path, member in [(DATA/(cohort+".npz"), "target.npy"), *(
                    (RUN/("parent_diagnostic" if arm == "parent" else arm)/(cohort+"_predictions.npz"), "prediction.npy") for arm in ARMS)]:
                with zipfile.ZipFile(canonical(path)) as archive:
                    entry = checked_entry(archive, member, count)
                    with archive.open(entry) as stream: header = npy_header(stream, count)
                headers[str(path)] = dict(member=member, shape=list(header["shape"]), descr=header["descr"],
                    central_directory_CRC_declared=entry.CRC, selected_CRC_recomputed=False)
        archive_bytes = sum(Path(path).stat().st_size for path in source)
        candidate = dict(schema=SCHEMA+".prepared_candidate", contract=declared_contract(), source_sha256=source,
            providers=providers, inspected_headers=headers, output=str(OUTPUT), source_archive_bytes=archive_bytes,
            actual_arrays_decoded=False, authority_created=False, normal_frame_row_capacity_proof=row_capacity)
        raw = encoded(candidate)
        # HH_261008 - Bound all350 prospective complete rows and metadata copies without assuming compression.
        forecast = archive_bytes + 350*FRAME_RECORD_CAP + 2*len(raw) + 128*1024
        require(forecast <= WORKING, "prospective source/row/RAW metadata working envelope cannot fit unchanged2MiB cap")
        with CANDIDATE.open("xb") as stream: require(stream.write(raw) == len(raw), "partial candidate write")
        return dict(status="PREPARED_NOT_AUTHORIZED", candidate_sha256=hashlib.sha256(raw).hexdigest(), working_upper_bound_bytes=forecast)


def admission(authority_sha):
    require(sha_text(authority_sha), "external RAW ROOT authority SHA required")
    require(digest(AUTHORITY) == authority_sha, "RAW ROOT authority bytes differ")
    authority = read_json(AUTHORITY, ROOT_METADATA_CAP)
    require(set(authority) == {"schema", "provenance", "candidate_sha256", "review_sha256", "output", "CPU_affinity",
            "stored_mask_region_diagnostic_authorized", "training_authorized", "model_inference_authorized"}
        and authority["schema"] == SCHEMA+".ROOT_authority" and authority["output"] == str(OUTPUT)
        and authority["stored_mask_region_diagnostic_authorized"] is True
        and authority["training_authorized"] is authority["model_inference_authorized"] is False,
        "exact external ROOT scope required; skeleton is never authority")
    require(sha_text(authority["candidate_sha256"]) and digest(CANDIDATE) == authority["candidate_sha256"]
        and sha_text(authority["review_sha256"]) and digest(REVIEW) == authority["review_sha256"], "RAW prospective candidate/review seals differ")
    require(canonical(REVIEW).stat().st_size <= ROOT_METADATA_CAP, "bounded RAW ROOT review required")
    candidate = read_json(CANDIDATE, 128*1024)
    require(candidate["schema"] == SCHEMA+".prepared_candidate" and candidate["contract"] == declared_contract()
        and candidate["source_sha256"] == sources() and candidate["providers"] == provider_pins()
        and candidate["output"] == str(OUTPUT) and candidate["actual_arrays_decoded"] is False
        and candidate["authority_created"] is False, "prepared source/provider/denominator contract changed")
    cpu = authority["CPU_affinity"]
    require(type(cpu) is list and len(cpu) == 1 and type(cpu[0]) is int and cpu[0] in os.sched_getaffinity(0), "one available declared CPU required")
    for key, expected in {"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
                          "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"}.items():
        require(os.environ.get(key) == expected, "clean single-CPU environment differs: "+key)
    require(sys.byteorder == "little", "explicit little-endian current process required")
    return authority, candidate, cpu


def original_reference():
    old, saved, manifest = read_json(OLD_RESULT), read_json(RESULTS), read_json(MANIFEST)
    previous_failure = read_json(ROOT / "storage/experiments/diagnostics/2026-10-07/map_sign_components_v1/result.json")
    require(previous_failure["status"] == "FAILED_WITH_FULL_DENOMINATOR"
        and len(previous_failure["frames"]) == 1050 and all(row["status"] == "UNEXECUTED" for row in previous_failure["frames"]),
        "original before-array failed attempt/all1050 unexecuted contexts must remain")
    require(old["status"] == "COMPLETE_DESCRIPTIVE_DIAGNOSTIC_NOT_MODEL_APPROVAL" and old["source_and_inputs_unchanged"] is True
        and old["original_criteria_verified"] is True and len(old["frames"]) == 1050
        and all(row["status"] == "PASS" for row in old["frames"]), "whole frozen V2 diagnostic required")
    lookup = {(row["cohort"], row["ordinal"], row["arm"]): row for row in old["frames"]}
    require(len(lookup) == 1050 and set(lookup) == {(r[0], r[1], r[2]) for r in ledger()}, "old whole1050 context identity differs")
    require(manifest["labels"] == list(LABELS) and manifest["raw_model_inputs"] == ["RGB"] and
        manifest["depth_or_pose_model_input"] is manifest["dev_or_test_used_for_fit"] is False
        and manifest["independent_new_dev_sites"] == 0 and saved["full_cohort_counts"] == dict(COHORTS), "unchanged known-exposed evaluation scope required")
    return old, saved, lookup


def execute(authority_sha):
    authority, candidate, cpu = admission(authority_sha)
    with canonical(LOCK).open("rb") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "fresh immutable output only")
        OUTPUT.mkdir(parents=True, exist_ok=False)
        start = time.monotonic(); generation = process_generation(); rows = ledger(); current = None
        lock_stat = os.fstat(lock.fileno())
        lease = dict(fd=lock.fileno(), dev=lock_stat.st_dev, ino=lock_stat.st_ino, path=str(LOCK))
        original_affinity = sorted(os.sched_getaffinity(0))
        error = None; partial = None; pins_ok = False; files_ok = {}; cells = {}; paired_cells = {}; evaluation = {}
        chain = hashlib.sha256(); slot = 0; processed_GT = 0
        result = dict(schema=SCHEMA, status="FAILED_WITH_COMPLETE_DENOMINATOR", contract=declared_contract(),
            authority_sha256=authority_sha, candidate_sha256=authority["candidate_sha256"], ledger_columns=["cohort", "ordinal", "arm", "status"],
            frames=rows, cells=cells, paired_cells=paired_cells, selected_member_EOF_CRC_and_pixel_SHA_verified=files_ok,
            model_inferences=0, training_steps=0, GPU_used=False, model_loaded=False, RGB_decoded=False,
            paired_support_count_change_kind="sum_positive_negative_per_GT_region_TP_count_delta_not_pixel_set_change",
            actual_newly_correct_or_lost_pixel_sets_verified=False,region_locations_or_vectors_persisted=False,
            equal_TP_count_can_move_pixel_support=True,maximum_output_bytes_scope="OUTPUT_only_not_combined_PLAN_ROOT",
            unselected_RGB_member_CRC_recomputed=False, semantic_object_identity_verified=False,
            physical_HD_topology_verified=False, calibration_or_crop_causation_proved=False,
            masks_or_thresholds_changed=False, model_performance_improved=False, independent_validation=False,
            formal_main_approved=0, formal_sub_approved=0, selected_provider_equivalence_not_full_environment=True,
            original_V1_failed_attempt_denominator_preserved=True)
        alarm_before = signal.signal(signal.SIGALRM, lambda *_: require(False, "whole diagnostic deadline exceeded"))
        signal.setitimer(signal.ITIMER_REAL, DEADLINE)
        try:
            check = lambda: guard(start, generation, lease)
            result["memory_at_start"] = check()
            os.sched_setaffinity(0, cpu); require(sorted(os.sched_getaffinity(0)) == cpu, "one CPU affinity not applied")
            verify_pins(candidate["source_sha256"], check); verify_pins({**ASSETS, **META_PINS}, check)
            verify_pins(candidate["providers"]["selected_file_sha256"], check)
            for number, (path, sha) in enumerate(candidate["source_sha256"].items()):
                payload = Path(path).read_bytes(); require(hashlib.sha256(payload).hexdigest() == sha, "source archive changed")
                write_raw(OUTPUT/f"source_{number:02d}_{Path(path).name}", payload)
            for path in (AUTHORITY, CANDIDATE, REVIEW): write_raw(OUTPUT/path.name, canonical(path).read_bytes())
            old, saved, lookup = original_reference()
            write_raw(OUTPUT/"started.json", encoded(dict(provenance="HH_261008 - Admit only the sealed stored-mask diagnostic.",
                at_UTC=datetime.now(timezone.utc).isoformat(), PID_generation=generation, held_lease=lease,
                declared_CPU=cpu, original_CPU_before_job=original_affinity)))
            # HH_261008 - No numeric import is reachable before RAW ROOT admission, seals, lease and resource checks.
            import numpy as np
            from scipy import ndimage
            require(np.__version__ == PACKAGES["numpy"] and importlib.metadata.version("scipy") == PACKAGES["scipy"]
                and str(canonical(Path(np.__file__))) == candidate["providers"]["top_level_origins"]["numpy"]
                and str(canonical(Path(sys.modules["scipy"].__file__))) == candidate["providers"]["top_level_origins"]["scipy"], "actual imported provider origin/version differs")
            result["runtime_versions"] = PACKAGES; check()
            matrix_all = {(cohort, arm): [[0]*7 for _ in range(7)] for cohort, _ in COHORTS for arm in ARMS}
            with (OUTPUT/"frame_regions.jsonl").open("xb") as records:
                for cohort, count in COHORTS:
                    first_slot = slot; graph_totals = {arm: [0]*len(GRAPH_COLUMNS) for arm in ARMS}
                    pair_totals = {arm: [0]*len(PAIR_COLUMNS) for arm in ("parent", "control")}
                    with ExitStack() as stack:
                        gt = stack.enter_context(MaskStream(DATA/(cohort+".npz"), "target.npy", count))
                        readers = {arm: stack.enter_context(MaskStream(RUN/("parent_diagnostic" if arm == "parent" else arm)/(cohort+"_predictions.npz"), "prediction.npy", count)) for arm in ARMS}
                        for ordinal in range(count):
                            current = slot; partial = dict(cohort=cohort, ordinal=ordinal, stage="shared_GT_frame")
                            check(); truth, truth_sha = gt.read_frame(np)
                            gt_labels, gt_areas = native_regions(truth == 5, np, ndimage)
                            observed = {}; arm_receipts = []
                            for arm in ARMS:
                                current = slot; partial["stage"] = arm+"_prediction_and_overlap"
                                check(); pred, pred_sha = readers[arm].read_frame(np)
                                matrix = np.bincount((truth*7+pred).ravel(), minlength=49).reshape(7, 7).tolist()
                                require(matrix == lookup[(cohort, ordinal, arm)]["confusion"] and sum(map(sum, matrix)) == PIXELS, "whole original49-cell frame confusion differs")
                                graph = native_graph(gt_labels, gt_areas, pred, np, ndimage)
                                require(graph["counts"][4:7] == [matrix[5][5], sum(matrix[5])-matrix[5][5], sum(r[5] for r in matrix)-matrix[5][5]], "graph/whole-pixel TP/FN/FP differs")
                                observed[arm] = graph
                                for i, n in enumerate(graph["counts"]): graph_totals[arm][i] += n
                                for i, row in enumerate(matrix):
                                    for j, n in enumerate(row): matrix_all[(cohort, arm)][i][j] += n
                                arm_receipts.append(dict(arm=arm, prediction_pixel_sha256=pred_sha, confusion_sha256=hashlib.sha256(encoded(matrix)).hexdigest(),
                                    sparse_overlap_edge_sha256=graph["edge_sha256"], counts=graph["counts"]))
                                progress(rows, slot, "FRAME_CHECKED_PENDING_ARCHIVE_EOF"); slot += 1
                            paired = []
                            for comparator in ("parent", "control"):
                                values = paired_summary(gt_areas, observed[comparator], observed["candidate"])
                                for i, n in enumerate(values): pair_totals[comparator][i] += n
                                paired.append(dict(reference=comparator, counts=values))
                            partial["stage"] = "full_GT_frame_receipt"
                            raw = append_frame(records, dict(cohort=cohort, ordinal=ordinal, GT_pixel_sha256=truth_sha,
                                GT_scan_order_region_labels_i32_sha256=hashlib.sha256(gt_labels.tobytes()).hexdigest(),
                                GT_scan_order_area_i64_sha256=hashlib.sha256(b"".join(struct.pack("<q", n) for n in gt_areas)).hexdigest(),
                                arms=arm_receipts, paired=paired, status="FRAME_CHECKED_PENDING_ARCHIVE_EOF"))
                            chain.update(raw); processed_GT += 1; check()
                        partial["stage"] = "selected_member_EOF_CRC_and_whole_pixel_SHA"
                        for key, reader in [("GT", gt), *readers.items()]:
                            pixel_sha = reader.finish()
                            reference = old["cells"][cohort+"/"+("parent" if key == "GT" else key)]
                            require(pixel_sha == reference["target_pixel_bytes_sha256" if key == "GT" else "prediction_pixel_bytes_sha256"], "whole selected-member pixel SHA differs")
                            files_ok[cohort+"/"+key] = dict(pixel_sha256=pixel_sha, selected_EOF_CRC_verified=True, path_FD_identity_unchanged=True)
                        evaluation[cohort] = {}
                        for arm in ARMS:
                            calculated = metrics(matrix_all[(cohort, arm)])
                            require(calculated == old["cells"][cohort+"/"+arm]["metrics"], "whole original frozen metrics differ")
                            for key, value in calculated.items(): require(saved["evaluation"][cohort][arm][key] == value, "original saved metric differs: "+key)
                            evaluation[cohort][arm] = calculated
                            cells[cohort+"/"+arm] = dict(graph_counts=graph_totals[arm], original_metrics=calculated)
                        paired_cells[cohort] = pair_totals
                        for index in range(first_slot, slot): progress(rows, index, "PASS")
                        current = None; partial = None
            screen = original_screens(evaluation)
            require(screen == old["original_exploratory_screen"] == saved["exploratory_screen"]
                and len(screen["checks"]) == 27 and sum(screen["checks"].values()) == 23, "unchanged23PASS4FAIL original27 predicates required")
            result.update(processed_GT_frames=processed_GT, full_frame_receipt_sha256=chain.hexdigest(),
                original_exploratory_screen=screen, original_27_criteria_verified=True)
        except BaseException as failure:
            error = type(failure).__name__+": "+str(failure)
            if current is not None and rows[current][3] in ("UNEXECUTED", "FRAME_CHECKED_PENDING_ARCHIVE_EOF"):
                progress(rows, current, "FAIL")
            result["failed_or_partial_context"] = partial
        finally:
            try:
                verify_pins(candidate["source_sha256"], lambda: guard(start, generation, lease))
                verify_pins({**ASSETS, **META_PINS}, lambda: guard(start, generation, lease))
                verify_pins(candidate["providers"]["selected_file_sha256"], lambda: guard(start, generation, lease))
                require(digest(AUTHORITY) == authority_sha and digest(CANDIDATE) == authority["candidate_sha256"]
                    and digest(REVIEW) == authority["review_sha256"], "RAW closing authority/plan/review changed")
                pins_ok = True
            except BaseException as failure:
                error = (error+"; " if error else "")+"closing seals: "+type(failure).__name__+": "+str(failure)
            signal.setitimer(signal.ITIMER_REAL, 0); signal.signal(signal.SIGALRM, alarm_before)
            try:
                result["memory_at_close"] = process_memory()
                require(result["memory_at_close"]["VmHWM_bytes"] <= RSS and process_generation() == generation, "closing memory/generation denied")
            except BaseException as failure:
                error = (error+"; " if error else "")+"closing process: "+type(failure).__name__+": "+str(failure)
            counts = {s: sum(r[3] == s for r in rows) for s in ("PASS", "FAIL", "FRAME_CHECKED_PENDING_ARCHIVE_EOF", "UNEXECUTED")}
            require(sum(counts.values()) == 1050, "complete declared denominator must survive failure")
            result.update(error=error, elapsed_seconds=time.monotonic()-start, status_counts=counts,
                processed_GT_frames=processed_GT, full_frame_receipt_sha256=chain.hexdigest(),
                source_inputs_selected_providers_unchanged=pins_ok,
                deadline_scope="processing and closing seals; bounded failure metadata can follow the deadline",
                resource_guard_scope="sampled current-process VmRSS/VmHWM, not an OS memory sandbox",
                frame_record_columns=dict(graph=list(GRAPH_COLUMNS), paired=list(PAIR_COLUMNS)))
            if error is None and pins_ok and counts["PASS"] == 1050 and len(files_ok) == 8:
                result["status"] = "COMPLETE_DESCRIPTIVE_PAIRED_MASK_REGIONS_NOT_MODEL_APPROVAL"
            closing = encoded(result)
            require(len(closing) <= CLOSING, "full1050 closing receipt exceeds prospective reserve; no truncation")
            write_raw(OUTPUT/"result.json", closing, LIMIT)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("source-contract", "prepare", "run"), default="source-contract")
    parser.add_argument("--authority-sha")
    args = parser.parse_args()
    if args.mode == "source-contract": value = declared_contract()
    elif args.mode == "prepare": value = prepare()
    else:
        value = execute(args.authority_sha)
        value = {key: value[key] for key in ("status", "status_counts", "elapsed_seconds", "error")}
    print(encoded(value).decode(), end="")
    return 2 if args.mode == "run" and not value["status"].startswith("COMPLETE_") else 0


if __name__ == "__main__":
    raise SystemExit(main())


