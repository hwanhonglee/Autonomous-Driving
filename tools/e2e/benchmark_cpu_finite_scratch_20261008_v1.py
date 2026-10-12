"""HH_261008 - Prepare a synthetic NumPy finite-scratch screen, not Torch/model/runtime equivalence.

An external RAW ROOT plan/authority is required before importing NumPy. Both
predicates scan the same float32 view; B caches only a private Boolean output,
never an input/result. First B allocation is INSIDE its timed call. Fixture
creation/SHA/guards are reported outside predicate timing and can warm caches;
there is no hidden predicate warmup, input copy or cache-flush claim. All32
alternating AB/BA pairs/case and post-timing overwrite checks are retained.
CPU3/one thread, VmHWM512MiB/time120s are sampled, NOT OS preemption. No Torch,
model, image, raw capture, ROS, GPU, training or package installation is used.
"""
from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import fcntl
import hashlib
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import stat
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_cpu_finite_scratch_benchmark_20261008_v1.py"
NAME = "cpu_finite_scratch_v1"
SCHEMA = "portable_e2e.synthetic_cpu_finite_scratch.20261008.v1"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08" / NAME
PLAN, AUTHORITY = PLAN_DIR / "ROOT_plan.json", PLAN_DIR / "ROOT_authority.json"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-08" / NAME
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
SHAPE = (1, 5, 6, 3, 180, 320)
ELEMENTS = math.prod(SHAPE)
MASKED_SLOT_END = (0, 0, 5, 2, 179, 319)
BITS = {"finite": 0x3E800000, "nan": 0x7FC00000, "positive_inf": 0x7F800000, "negative_inf": 0xFF800000}
LAYOUTS = ("dense", "strided_last_axis_step2", "offset17")
SOURCE_PINS = {
    "learning/portable_e2e/temporal_cpu_finite_validation_v2_20261001.py": "ecc6f7af5e8ee8dcbbc3a656c50e7de474f7a7bfa1e1eca630af40faa59e16c5",
    "learning/portable_e2e/temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
    "learning/portable_e2e/temporal_cached_encoder_v2.py": "55ef7371e0a1f4ec2b9cb854282e4d51b6c406b055ec5eceb8e286737f03adff",
}
PROVIDER_PINS = {
    "/usr/lib/python3/dist-packages/numpy/__init__.py": "02e54b39a5825a5eb95c74639b22593a8af816212f9bb1fdcb78f17112a5d27f",
    "/usr/lib/python3/dist-packages/numpy/core/_multiarray_umath.cpython-310-x86_64-linux-gnu.so": "1f8966952e5cff48e07f98505371b92d7c478ed01926f1a1d01940084feb5daf",
    "/usr/bin/python3.10": "a2f33a6e006989270f4340528eb61f8f97366e00a5d1b602ac8672ea44fc56ae",
}
ENV = {"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
       "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"}
LIMIT, CLOSING, ROW_LIMIT = 2 * 1024**2, 256 * 1024, 1024
FLOOR, RESERVE, HWM_LIMIT, DEADLINE = 2 * 1024**3, 64 * 1024**2, 512 * 1024**2, 120


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def canonical(path):
    path = Path(path)
    require(path.is_absolute() and path == path.resolve(strict=True) and stat.S_ISREG(path.lstat().st_mode), "canonical regular file required")
    return path


def fingerprint(path):
    s = canonical(path).stat()
    return [s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns]


def digest(path):
    path = canonical(path); before = fingerprint(path); result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            result.update(block)
    require(before == fingerprint(path), "file generation changed during SHA read")
    return result.hexdigest()


def read_json(path, pin=None):
    path = canonical(path); require(path.stat().st_size <= 128 * 1024, "bounded declaration required")
    raw = path.read_bytes(); require(pin is None or hashlib.sha256(raw).hexdigest() == pin, "RAW document SHA differs")
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON field"); result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=lambda value: require(False, "nonfinite JSON"))


def error_record(error):
    raw = repr(error).encode("utf8", errors="backslashreplace")
    return dict(type=type(error).__name__, raw_bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                prefix=raw[:128].decode("utf8", errors="replace"), full_error_bytes_retained=len(raw) <= 128)


def storage_used():
    total = 0
    for root in (PLAN_DIR, OUTPUT):
        require(root.is_absolute() and root == root.resolve(strict=False), "output/plan alias denied")
        if not root.exists():
            continue
        require(root.is_dir(), "owned storage root must be a directory")
        for parent, directories, names in os.walk(root, followlinks=False):
            for name in directories + names:
                p = Path(parent) / name; mode = p.lstat().st_mode
                require(p == p.resolve(strict=True) and (stat.S_ISDIR(mode) or stat.S_ISREG(mode)), "owned storage alias/special file denied")
                if stat.S_ISREG(mode):
                    total += p.stat().st_size
    return total


def write_new(path, value, *, closing=False):
    path = Path(path); raw = encoded(value)
    require(path.is_relative_to(PLAN_DIR) or path.is_relative_to(OUTPUT), "only declared owned storage writes")
    require(path.parent == path.parent.resolve(strict=True) and len(raw) <= CLOSING, "bounded canonical metadata write")
    require(storage_used() + len(raw) <= LIMIT - (0 if closing else CLOSING), "combined metadata quota exceeded")
    with path.open("xb") as stream:
        stream.write(raw)


def source_table():
    result = {}
    for relative, expected in SOURCE_PINS.items():
        actual = digest(ROOT / relative); require(actual == expected, "original finite/recovery/cache source changed")
        result[relative] = actual
    for path in (HERE, TEST):
        result[str(path.relative_to(ROOT))] = digest(path)
    # HH_261008 - Parse the genuine predicate, never import the Torch-dependent parent module.
    tree = ast.parse((ROOT / next(iter(SOURCE_PINS))).read_bytes())
    fn = next(part for part in tree.body if isinstance(part, ast.FunctionDef) and part.name == "cpu_float32_all_finite")
    expected = ast.parse("bool(np.isfinite(value.detach().numpy()).all())", mode="eval").body
    require(isinstance(fn.body[-1], ast.Return) and ast.dump(fn.body[-1].value, include_attributes=False) == ast.dump(expected, include_attributes=False), "exact original finite predicate AST required")
    return result


def strides(shape, itemsize):
    result = []
    for dimension in reversed(shape):
        result.append(itemsize); itemsize *= dimension
    return list(reversed(result))


def layout_contract(layout):
    require(layout in LAYOUTS, "fixed synthetic layout required")
    base_shape = list(SHAPE)
    if layout == "strided_last_axis_step2":
        base_shape[-1] *= 2
    if layout == "offset17":
        base_shape = [ELEMENTS + 17]
    offset = 17 if layout == "offset17" else 0
    actual_strides = strides(SHAPE, 4) if layout != "strided_last_axis_step2" else strides(base_shape, 4)
    if layout == "strided_last_axis_step2":
        actual_strides[-1] *= 2
    index = offset + sum(i * step // 4 for i, step in zip(MASKED_SLOT_END, actual_strides))
    return dict(layout=layout, view_shape=list(SHAPE), view_strides_bytes=actual_strides, data_offset_bytes=offset * 4,
                base_shape=base_shape, base_elements=math.prod(base_shape), invalid_base_index=index,
                dtype="float32", view_bytes=ELEMENTS * 4, base_bytes=math.prod(base_shape) * 4)


def fixture_sha(layout, kind):
    declared = layout_contract(layout); require(kind in BITS, "fixed synthetic value kind")
    h = hashlib.sha256(); ordinary = struct.pack("<I", BITS["finite"])
    def repeated(count):
        while count:
            length = min(count, 16384); h.update(ordinary * length); count -= length
    repeated(declared["invalid_base_index"]); h.update(struct.pack("<I", BITS[kind]))
    repeated(declared["base_elements"] - declared["invalid_base_index"] - 1)
    return h.hexdigest()


def cases():
    return [dict(case_id=layout + ":" + kind, **layout_contract(layout), value_kind=kind,
                 expected_finite=kind == "finite", expected_base_sha256=fixture_sha(layout, kind))
            for layout in LAYOUTS for kind in BITS]


def fixed_contract():
    return dict(shape=list(SHAPE), float32_view_bytes=20736000, layouts=list(LAYOUTS), values=list(BITS),
        invalid_index=list(MASKED_SLOT_END), hypothetical_masked_history_slot=0, masks_never_used_to_skip_values=True,
        pairs_per_case=32, order="AB_even_pair_BA_odd_pair", timed_calls=768, post_timing_overwrite_calls=48,
        total_predicate_calls=816, predicate_warmup_calls=0, first_B_allocation_inside_timer=True,
        private_scratch_per_case=True, input_view_copied_by_either_predicate=False,
        fixture_creation_hash_and_resource_guards_outside_predicate_timer=True,
        fixture_hash_may_warm_memory_cache=True, whole_cold_cache_claimed=False,
        CPU_affinity=[3], OMP=1, MKL=1, OPENBLAS=1, wall_seconds_sampled=120, VmHWM_bytes_sampled=HWM_LIMIT,
        time_or_memory_OS_preemption_claimed=False, canonical_shared_lease=str(LOCK), metadata_bytes=LIMIT,
        closing_reserve_bytes=CLOSING, prospective_raw_row_bytes=ROW_LIMIT, source_metadata_reserve=128*1024,
        prospective_raw_rows_bytes=816*ROW_LIMIT, result_reserve_bytes=128*1024,
        NumPy_version="1.21.5", Torch_interop_proven=False, original_model_two_validations_executed=False,
        actual_model_inputs_images_or_TRAIN_TEST_opened=False, NN_inferences=0, training_steps=0,
        ROS=False, CARLA=False, GPU=False, realtime_or_feature_or_adoption=False)


def schedule():
    rows = []
    for case in cases():
        for pair in range(32):
            for arm in (("A", "B") if pair % 2 == 0 else ("B", "A")):
                rows.append(dict(case_id=case["case_id"], phase="timed", pair=pair, arm=arm, expected=case["expected_finite"]))
        for step, expected in enumerate((not case["expected_finite"], case["expected_finite"])):
            for arm in ("A", "B"):
                rows.append(dict(case_id=case["case_id"], phase="overwrite_check", step=step, arm=arm, expected=expected))
    require(len(rows) == 816, "complete declared schedule")
    return rows


def prepare():
    require(not PLAN_DIR.exists() and not OUTPUT.exists(), "new revision only; no retry overwrite")
    require(not any(name == "numpy" or name.startswith(("numpy.", "torch", "rclpy", "carla")) for name in sys.modules), "source-only preparation must not import numeric/runtime packages")
    sources = source_table(); providers = {path: digest(path) for path in PROVIDER_PINS}
    require(providers == PROVIDER_PINS, "selected original NumPy/Python provider changed")
    roster = cases(); order = schedule()
    plan = dict(schema=SCHEMA, fixed=fixed_contract(), source_sha256=sources, provider_sha256=providers,
        cases=roster, schedule_sha256=hashlib.sha256(encoded(order)).hexdigest(), raw_original_sources_archived=True,
        selected_native_provider_not_copied=True, transitive_native_loader_closure_claimed=False)
    prospective = 816*ROW_LIMIT + sum((ROOT/name).stat().st_size for name in sources) + len(encoded(plan))*3 + 128*1024 + CLOSING
    require(prospective <= LIMIT, "prospective whole metadata capacity denied before preparation")
    PLAN_DIR.mkdir(parents=True); (PLAN_DIR/"sources").mkdir()
    for name, pin in sources.items():
        destination = PLAN_DIR/"sources"/name; destination.parent.mkdir(parents=True, exist_ok=True)
        raw = (ROOT/name).read_bytes(); require(hashlib.sha256(raw).hexdigest() == pin, "source copy drift")
        require(storage_used() + len(raw) <= LIMIT-CLOSING, "source copy quota")
        with destination.open("xb") as stream:
            stream.write(raw)
    write_new(PLAN_DIR/"candidate_plan.json", plan)
    return dict(status="PREPARED_NOT_AUTHORIZED", candidate_plan_sha256=digest(PLAN_DIR/"candidate_plan.json"),
                timed_calls=768, overwrite_calls=48, prospective_metadata_bytes=prospective)


def admit(plan_sha, authority_sha):
    require(all(type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)
                for value in (plan_sha, authority_sha)), "two external RAW SHA arguments required")
    plan = read_json(PLAN, plan_sha); grant = read_json(AUTHORITY, authority_sha)
    require(plan["schema"] == SCHEMA and plan["fixed"] == fixed_contract() and plan["cases"] == cases(), "exact prospective synthetic contract required")
    require(plan["source_sha256"] == source_table() and plan["provider_sha256"] == PROVIDER_PINS, "original/current source provider closure differs")
    require(plan["schedule_sha256"] == hashlib.sha256(encoded(schedule())).hexdigest(), "all fixed ABBA/overwrite slots required")
    require(grant == dict(schema=SCHEMA+".ROOT_authority", plan_path=str(PLAN), plan_sha256=plan_sha,
        source_sha256=plan["source_sha256"], synthetic_NumPy_predicate_only=True, Torch_authorized=False,
        NN_authorized=False, GPU_authorized=False, ROS_authorized=False, CARLA_authorized=False,
        TRAIN_TEST_authorized=False, training_authorized=False, model_promoted=False), "externally RAW-SHA-bound ROOT authority required")
    for name, pin in plan["source_sha256"].items():
        require(digest(PLAN_DIR/"sources"/name) == pin, "archived/current complete original source differs")
    for path, pin in plan["provider_sha256"].items():
        require(digest(path) == pin, "selected provider bytes changed")
    return plan


@contextmanager
def lease():
    with canonical(LOCK).open("rb") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield dict(fd=stream.fileno(), dev=os.fstat(stream.fileno()).st_dev, ino=os.fstat(stream.fileno()).st_ino)
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


class Budget:
    def __init__(self):
        self.started = time.monotonic(); self.max_HWM = 0; self.sampled_violation = False

    def guard(self, *, closing=False):
        fields = {}
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith(("VmRSS:", "VmHWM:")):
                key, value, unit = line.split(); require(unit == "kB", "Linux self-memory unit required")
                fields[key[:-1]] = int(value) * 1024
        require(set(fields) == {"VmRSS", "VmHWM"}, "actual own Linux memory fields required")
        self.max_HWM = max(self.max_HWM, fields["VmHWM"])
        self.sampled_violation = self.sampled_violation or self.max_HWM > HWM_LIMIT or time.monotonic()-self.started > DEADLINE
        require(closing or (self.max_HWM <= HWM_LIMIT and time.monotonic()-self.started <= DEADLINE), "sampled memory/time limit exceeded")
        require(shutil.disk_usage(ROOT).free >= FLOOR+RESERVE, "CPU2GiB floor plus64MiB shared reserve required")
        require(storage_used() <= LIMIT-(0 if closing else CLOSING), "combined metadata quota")


class Scratch:
    def __init__(self):
        self.buffer = None; self.allocations = 0; self.busy = False

    def predicate(self, np, view):
        require(not self.busy, "private scratch is serial, not reentrant")
        self.busy = True
        try:
            if self.buffer is None:
                self.buffer = np.empty(SHAPE, dtype=np.bool_, order="C"); self.allocations += 1
            require(type(self.buffer) is np.ndarray and tuple(self.buffer.shape) == SHAPE
                    and self.buffer.dtype == np.dtype("bool") and self.buffer.flags["C_CONTIGUOUS"]
                    and self.buffer.base is None, "private shape-exact owning Boolean scratch required")
            result = np.isfinite(view, out=self.buffer)
            require(result is self.buffer, "full overwritten original private output required")
            return bool(result.all())
        finally:
            self.busy = False


def measure(arm, np, view, scratch, clock=time.perf_counter_ns):
    row = dict(begun_ns=None, finished_ns=None, duration_ns=None, result=None, error=None,
               first_scratch_allocation=arm == "B" and scratch.buffer is None, allocations_before=scratch.allocations)
    try:
        begun = clock(); require(type(begun) is int and begun > 0, "actual monotonic begin: " + repr(begun))
        row["begun_ns"] = begun
        require(arm in ("A", "B"), "declared predicate arm")
        row["result"] = bool(np.isfinite(view).all()) if arm == "A" else scratch.predicate(np, view)
        finished = clock()
        require(type(finished) is int and finished >= row["begun_ns"], "actual monotonic end: " + repr(finished))
        row["finished_ns"] = finished
        row["duration_ns"] = row["finished_ns"]-row["begun_ns"]
    except BaseException as error:
        row["error"] = error_record(error)
    row["allocations_after"] = scratch.allocations
    return row


class Fixture:
    def __init__(self, np, declared):
        self.np, self.declared = np, declared
        self.base = np.full(declared["base_shape"], .25, dtype=np.float32, order="C")
        if declared["layout"] == "strided_last_axis_step2":
            self.view = self.base[..., ::2]
        elif declared["layout"] == "offset17":
            self.view = self.base[17:].reshape(SHAPE)
        else:
            self.view = self.base
        self.set_bits(BITS[declared["value_kind"]]); self.before_sha256 = self.verify()

    def set_bits(self, bits):
        self.base.view(self.np.uint32).reshape(-1)[self.declared["invalid_base_index"]] = self.np.uint32(bits)

    def verify(self):
        require(type(self.view) is self.np.ndarray and self.view.dtype == self.np.dtype("float32")
                and list(self.view.shape) == self.declared["view_shape"] and list(self.view.strides) == self.declared["view_strides_bytes"], "exact same declared float32 strided/offset view")
        offset = self.view.__array_interface__["data"][0]-self.base.__array_interface__["data"][0]
        require(offset == self.declared["data_offset_bytes"] and self.base.flags["C_CONTIGUOUS"], "original storage offset/base layout required")
        actual = hashlib.sha256(memoryview(self.base).cast("B")).hexdigest()
        require(actual == self.declared["expected_base_sha256"], "entire deterministic base fixture bytes differ")
        return actual


def summary(rows):
    result = {}
    for case in cases():
        values = {}
        for arm in ("A", "B"):
            selected = [r for r in rows if r["case_id"] == case["case_id"] and r["phase"] == "timed" and r["arm"] == arm]
            measured = [r["duration_ns"] for r in selected if type(r.get("duration_ns")) is int]
            if measured:
                ordered = sorted(measured)
                values[arm] = dict(n=len(measured), mean_ns=math.fsum(measured)/len(measured), median_ns=(ordered[(len(ordered)-1)//2]+ordered[len(ordered)//2])/2,
                    p95_nearest_rank_ns=ordered[math.ceil(.95*len(ordered))-1], min_ns=ordered[0], max_ns=ordered[-1],
                    failed_rows_included=sum(r.get("status")=="FAILED_RETAINED" for r in selected if type(r.get("duration_ns")) is int),
                    missing_duration_UNKNOWN=sum(type(r.get("duration_ns")) is not int for r in selected), first_allocation_not_removed=True)
        result[case["case_id"]] = values
    return result


def execute(plan_sha, authority_sha):
    require(sys.flags.optimize == 0 and sys.flags.dont_write_bytecode and sys.byteorder == "little", "unoptimized no-bytecode little-endian Python required")
    require(all(os.environ.get(k) == v for k,v in ENV.items()) and not any(os.environ.get(k) for k in ("LD_PRELOAD","LD_AUDIT","PYTHONHOME","PYTHONPATH")), "existing clean single-thread CPU environment required")
    require(not any(name == "numpy" or name.startswith(("numpy.","torch","rclpy","carla")) for name in sys.modules), "NumPy/runtime import before ROOT admission denied")
    require(str(Path(sys.executable).resolve()) == "/usr/bin/python3.10", "exact existing CPython3.10 provider required")
    plan = admit(plan_sha, authority_sha); require(not OUTPUT.exists(), "fresh actual output required")
    require(3 in os.sched_getaffinity(0), "prospective logical CPU3 unavailable")
    old_affinity = os.sched_getaffinity(0); rows = []; closing = []; error = None; fixtures = []; np = None
    planned = schedule()
    ledger_failure = None; pending_row = None; ledger_hash = hashlib.sha256(); ledger = None
    with lease() as lock:
        OUTPUT.mkdir(parents=True); budget = Budget()
        try:
            os.sched_setaffinity(0, {3})
            budget.guard(); write_new(OUTPUT/"started.json", dict(pid=os.getpid(), plan_sha256=plan_sha, authority_sha256=authority_sha,
                proc_stat=Path("/proc/self/stat").read_text().strip(), argv=sys.argv, affinity=sorted(os.sched_getaffinity(0)), lease=lock,
                NumPy_imported=False, NN_inferences=0, training_steps=0))
            ledger = (OUTPUT/"predicate_calls.jsonl").open("xb")
            specification = importlib.util.find_spec("numpy")
            require(specification is not None and str(Path(specification.origin).resolve()) == "/usr/lib/python3/dist-packages/numpy/__init__.py",
                    "foreign NumPy resolution denied BEFORE package import")
            np = importlib.import_module("numpy"); native = importlib.import_module("numpy.core._multiarray_umath")
            require(np.__version__ == "1.21.5" and str(Path(np.__file__).resolve()) in PROVIDER_PINS
                    and str(Path(native.__file__).resolve()) in PROVIDER_PINS, "actual pinned existing NumPy providers required")
            for declared in plan["cases"]:
                budget.guard(); fixture = Fixture(np, declared); scratch = Scratch(); active = [r for r in planned if r["case_id"] == declared["case_id"]]
                for item in active:
                    budget.guard(); require(os.sched_getaffinity(0) == {3}, "own CPU affinity drift")
                    if item["phase"] == "overwrite_check":
                        bits = BITS[declared["value_kind"]] if item["step"] else (BITS["nan"] if declared["expected_finite"] else BITS["finite"])
                        fixture.set_bits(bits)
                    observed = measure(item["arm"], np, fixture.view, scratch)
                    row = dict(ordinal=len(rows), **item, **observed)
                    row["status"] = "MATCH" if row["error"] is None and row["result"] is item["expected"] else "FAILED_RETAINED"
                    rows.append(row); pending_row = row
                    raw_row = encoded(row)
                    require(len(raw_row) <= ROW_LIMIT, "full raw timing row prospective cap")
                    require(storage_used()+len(raw_row) <= LIMIT-CLOSING, "full timing append quota")
                    require(ledger.write(raw_row) == len(raw_row), "partial raw timing write")
                    ledger.flush(); ledger_hash.update(raw_row); pending_row = None; budget.guard()
                    if row["status"] != "MATCH":
                        raise ValueError("predicate error/result mismatch; no retry or omitted tail")
                require(scratch.allocations == 1 and not scratch.busy, "exact one first timed B allocation/case")
                fixtures.append(dict(case_id=declared["case_id"], base_sha256_before=fixture.before_sha256, base_sha256_after=fixture.verify(), allocation_count=scratch.allocations))
                del fixture, scratch
        except BaseException as failure:
            error = error_record(failure)
            if pending_row is not None:
                ledger_failure = error_record(failure)
        finally:
            for label, action in (("source/current/archive/provider_postseals", lambda: admit(plan_sha, authority_sha)),
                                  ("sampled_closing_budget", lambda: budget.guard(closing=True)),
                                  ("restore_own_affinity", lambda: os.sched_setaffinity(0, old_affinity))):
                try:
                    action(); closing.append(dict(stage=label, passed=True, error=None))
                except BaseException as failure:
                    closing.append(dict(stage=label, passed=False, error=error_record(failure)))
            complete = len(rows)
            for item in planned[complete:]:
                rows.append(dict(ordinal=len(rows), **item, status="UNEXECUTED", begun_ns=None, finished_ns=None, duration_ns=None, result=None))
            require(len(rows) == 816, "all actual/failed/unexecuted fixed predicate slots retained")
            # HH_261008 - Completed calls are flushed at their original post-timer seam; fatal interruption can still leave an UNKNOWN in-flight call.
            try:
                if ledger is None:
                    ledger = (OUTPUT/"predicate_calls.jsonl").open("xb")
                if ledger_failure is None:
                    for row in rows[complete:]:
                        raw_row = encoded(row); require(len(raw_row)<=ROW_LIMIT and storage_used()+len(raw_row)<=LIMIT-CLOSING, "full unexecuted-tail append quota")
                        require(ledger.write(raw_row)==len(raw_row), "partial tail write"); ledger.flush(); ledger_hash.update(raw_row)
            except BaseException as failure:
                ledger_failure = error_record(failure)
            finally:
                if ledger is not None:
                    try:
                        ledger.close()
                    except BaseException as failure:
                        ledger_failure = error_record(failure)
            raw_ledger_sha256 = None
            try:
                raw_ledger_sha256 = digest(OUTPUT/"predicate_calls.jsonl")
                require(ledger_failure is not None or raw_ledger_sha256 == ledger_hash.hexdigest(), "full ledger differs from actual accepted byte chain")
            except BaseException as failure:
                closing.append(dict(stage="raw_ledger_final_SHA", passed=False, error=error_record(failure)))
                if ledger_failure is None:
                    ledger_failure = error_record(failure)
            timings = summary(rows)
            whole_seconds = time.monotonic()-budget.started
            budget.sampled_violation = budget.sampled_violation or whole_seconds > DEADLINE
            forbidden = [name for name in sys.modules if name.startswith(("torch","rclpy","carla"))]
            passed = error is None and ledger_failure is None and not budget.sampled_violation and all(r["status"] == "MATCH" for r in rows) and all(r["passed"] for r in closing) and not forbidden
            result = dict(schema=SCHEMA+".result", status="PASS_SYNTHETIC_NUMPY_PREDICATE_ONLY" if passed else "FAILED_RETAINED",
                error=error, expected_timed_calls=768, expected_overwrite_calls=48, actual_predicate_attempts=complete,
                MATCH=sum(r["status"]=="MATCH" for r in rows), FAILED=sum(r["status"]=="FAILED_RETAINED" for r in rows), UNEXECUTED=816-complete,
                raw_ledger_sha256=raw_ledger_sha256, accepted_flushed_prefix_sha256=ledger_hash.hexdigest(),
                raw_ledger_complete=ledger_failure is None, ledger_failure=ledger_failure, last_unaccepted_timing_row=pending_row,
                fatal_host_or_SIGKILL_inflight_observation_UNKNOWN=True, timings=timings, fixtures=fixtures, closing=closing,
                own_max_observed_VmHWM_bytes=budget.max_HWM, whole_seconds=whole_seconds,
                first_scratch_allocations_included=True, all_timing_rows_including_first_and_failures_retained=ledger_failure is None,
                NumPy_imported=np is not None, forbidden_imports=forbidden, transitive_native_loader_closure_claimed=False,
                sampled_not_OS_deadline_or_memory_containment=True, sampled_time_or_memory_violation=budget.sampled_violation,
                actual_Torch_view_interop_proven=False,
                original_model_two_validations_executed=False, NN_inferences=0, training_steps=0, ROS=False, CARLA=False,
                physical_GPU_used=False, realtime_or_feature_or_model_adoption=False)
            write_new(OUTPUT/"result.json", result, closing=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("prepare","execute"), required=True)
    parser.add_argument("--plan-sha256"); parser.add_argument("--authority-sha256")
    args = parser.parse_args()
    result = prepare() if args.mode == "prepare" else execute(args.plan_sha256, args.authority_sha256)
    print(encoded(result).decode().strip())
    return 0 if result["status"] in ("PREPARED_NOT_AUTHORIZED","PASS_SYNTHETIC_NUMPY_PREDICATE_ONLY") else 1


if __name__ == "__main__":
    raise SystemExit(main())
