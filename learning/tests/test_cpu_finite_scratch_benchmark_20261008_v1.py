"""HH_261008 - Scalar/AST/tiny-buffer fixtures; no NumPy, Torch, ROS or real benchmark runs.

FakeArray supplies an explicitly synthetic byte buffer, not NumPy/Torch ABI
evidence. Mocked execute fixtures test ledger/failure bookkeeping only; actual
providers, resource admission and timings need the separately authorized run.
"""
from contextlib import contextmanager
import ast
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/benchmark_cpu_finite_scratch_20261008_v1.py"
SPEC = importlib.util.spec_from_file_location("_hh_finite_scratch_scalar_fixture", SOURCE)
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


class FakeArray(bytearray):
    """Tiny Python buffer with fake shape metadata; never a native ndarray."""
    def __new__(cls, *args, **kwargs):
        return bytearray.__new__(cls)

    def __init__(self, shape, dtype, *, source=None, offset=0, strides=None, events=None):
        self.shape = tuple(shape); self.dtype = dtype; self.events = events
        self.strides = tuple(strides or M.strides(self.shape, 1 if dtype == "bool" else 4))
        self.flags = {"C_CONTIGUOUS": self.strides == tuple(M.strides(self.shape, 1 if dtype == "bool" else 4))}
        self.base = source
        if source is None:
            bytearray.__init__(self, b"\0" * math.prod(self.shape) if dtype == "bool" else struct.pack("<f", .25) * math.prod(self.shape))
            self.storage = self; self.offset = 0
        else:
            bytearray.__init__(self, b""); self.storage = source.storage; self.offset = source.offset + offset

    @property
    def __array_interface__(self):
        return {"data": (id(self.storage) + self.offset, False)}

    def __getitem__(self, key):
        if isinstance(key, tuple) and key[0] is Ellipsis:
            step = key[1].step
            shape = self.shape[:-1] + (self.shape[-1] // step,)
            strides = self.strides[:-1] + (self.strides[-1] * step,)
            return FakeArray(shape, self.dtype, source=self, strides=strides)
        if isinstance(key, slice) and key.start == 17:
            return FakeArray((math.prod(self.shape) - 17,), self.dtype, source=self, offset=17*4)
        return bytearray.__getitem__(self, key)

    def reshape(self, shape):
        if shape == -1:
            shape = (math.prod(self.shape),)
        shape = tuple(shape)
        assert math.prod(shape) == math.prod(self.shape)
        return FakeArray(shape, self.dtype, source=self)

    def view(self, dtype):
        return FakeUIntView(self)

    def bits(self):
        for indexes in itertools.product(*(range(n) for n in self.shape)):
            offset = self.offset + sum(i*s for i,s in zip(indexes, self.strides))
            yield struct.unpack_from("<I", self.storage, offset)[0]

    def all(self):
        assert self.dtype == "bool"
        if self.events is not None:
            self.events.append("all")
        return all(bytearray.__iter__(self))


class FakeUIntView:
    def __init__(self, array):
        self.array = array

    def reshape(self, shape):
        assert shape == -1
        return self

    def __setitem__(self, index, bits):
        struct.pack_into("<I", self.array.storage, self.array.offset + index*4, bits)


class FakeNumPy:
    ndarray = FakeArray
    bool_, float32 = "bool", "float32"
    uint32 = staticmethod(int)
    dtype = staticmethod(str)
    __version__ = "1.21.5"
    __file__ = "/usr/lib/python3/dist-packages/numpy/__init__.py"

    def __init__(self, events=None):
        self.events = events if events is not None else []; self.finite_calls = 0

    def empty(self, shape, dtype, order):
        assert order == "C"
        self.events.append("allocate")
        return FakeArray(shape, dtype, events=self.events)

    def full(self, shape, value, dtype, order):
        assert value == .25 and dtype == "float32" and order == "C"
        return FakeArray(shape, dtype)

    def isfinite(self, view, *, out=None):
        self.events.append("scan"); self.finite_calls += 1
        result = out if out is not None else FakeArray(view.shape, "bool", events=self.events)
        # HH_261008 - Deliberately overwrite every tiny synthetic Boolean cell.
        result[:] = bytes(int(bits & 0x7F800000 != 0x7F800000) for bits in view.bits())
        return result


@pytest.fixture
def tiny(monkeypatch):
    monkeypatch.setattr(M, "SHAPE", (1, 1, 2, 1, 2, 3))
    monkeypatch.setattr(M, "ELEMENTS", 12)
    monkeypatch.setattr(M, "MASKED_SLOT_END", (0, 0, 1, 0, 1, 2))
    return M.cases()


def declared_case(roster, layout, kind):
    return next(row for row in roster if row["case_id"] == layout + ":" + kind)


def test_contract_full_shape_and_no_realtime_or_model_claim():
    contract = M.fixed_contract()
    assert contract == json.loads(M.encoded(contract))
    assert contract["shape"] == [1, 5, 6, 3, 180, 320]
    assert contract["float32_view_bytes"] == 20736000
    assert contract["timed_calls"] == 768 and contract["post_timing_overwrite_calls"] == 48
    assert contract["CPU_affinity"] == [3] and contract["OMP"] == contract["MKL"] == contract["OPENBLAS"] == 1
    assert contract["first_B_allocation_inside_timer"] and contract["predicate_warmup_calls"] == 0
    for field in ("Torch_interop_proven", "original_model_two_validations_executed", "ROS", "CARLA", "GPU", "realtime_or_feature_or_adoption"):
        assert contract[field] is False


def test_all_pairs_ABBA_and_all_overwrite_slots(tiny):
    rows = M.schedule()
    assert len(rows) == 816
    assert sum(row["phase"] == "timed" for row in rows) == 768
    assert sum(row["phase"] == "overwrite_check" for row in rows) == 48
    for case in tiny:
        group = [row for row in rows if row["case_id"] == case["case_id"]]
        assert [row["arm"] for row in group[:8]] == list("ABBAABBA")
        assert [row["pair"] for row in group[:64]] == [pair for pair in range(32) for _ in range(2)]
        assert [row["expected"] for row in group[-4:]] == [not case["expected_finite"]]*2 + [case["expected_finite"]]*2


@pytest.mark.parametrize("layout", M.LAYOUTS)
@pytest.mark.parametrize("kind", M.BITS)
def test_whole_base_SHA_and_exact_view_false_true_overwrite(tiny, layout, kind):
    declared = declared_case(tiny, layout, kind)
    np = FakeNumPy(); fixture = M.Fixture(np, declared); scratch = M.Scratch()
    assert fixture.before_sha256 == declared["expected_base_sha256"]
    assert fixture.verify() == hashlib.sha256(fixture.base).hexdigest()
    assert bool(np.isfinite(fixture.view).all()) is declared["expected_finite"]
    assert scratch.predicate(np, fixture.view) is declared["expected_finite"]
    initial_buffer = scratch.buffer
    fixture.set_bits(M.BITS["nan"])
    assert scratch.predicate(np, fixture.view) is False
    fixture.set_bits(M.BITS["finite"])
    assert scratch.predicate(np, fixture.view) is True
    fixture.set_bits(M.BITS[kind])
    assert fixture.verify() == fixture.before_sha256
    assert scratch.buffer is initial_buffer and scratch.allocations == 1 and scratch.busy is False


def test_declared_full_view_stride_and_offset(tiny):
    dense = M.layout_contract("dense"); stride = M.layout_contract("strided_last_axis_step2"); offset = M.layout_contract("offset17")
    assert dense["base_bytes"] == 48 and dense["invalid_base_index"] == 11
    assert stride["base_bytes"] == 96 and stride["view_strides_bytes"][-1] == 8
    assert stride["invalid_base_index"] == 22 and stride["view_bytes"] == 48
    assert offset["data_offset_bytes"] == 68 and offset["invalid_base_index"] == 28
    assert M.fixture_sha("dense", "finite") == hashlib.sha256(struct.pack("<I", M.BITS["finite"])*12).hexdigest()


def test_stride_padding_is_not_a_value_in_the_declared_view(tiny):
    np = FakeNumPy(); fixture = M.Fixture(np, declared_case(tiny, "strided_last_axis_step2", "finite"))
    struct.pack_into("<I", fixture.base, 4, M.BITS["nan"])
    assert np.isfinite(fixture.view).all() is True and M.Scratch().predicate(np, fixture.view) is True
    with pytest.raises(ValueError, match="entire deterministic"):
        fixture.verify()


@pytest.mark.parametrize("fault", ["shape", "dtype", "view", "contiguous"])
def test_private_boolean_storage_fault_denied(tiny, fault):
    np = FakeNumPy(); fixture = M.Fixture(np, tiny[0]); scratch = M.Scratch()
    scratch.predicate(np, fixture.view)
    if fault == "shape": scratch.buffer.shape = (12,)
    elif fault == "dtype": scratch.buffer.dtype = "float32"
    elif fault == "view": scratch.buffer.base = fixture.base
    else: scratch.buffer.flags["C_CONTIGUOUS"] = False
    before = np.finite_calls
    with pytest.raises(ValueError, match="private shape-exact"):
        scratch.predicate(np, fixture.view)
    assert np.finite_calls == before and scratch.busy is False


def test_reentrant_or_replaced_output_has_no_fallback(tiny, monkeypatch):
    np = FakeNumPy(); fixture = M.Fixture(np, tiny[0]); scratch = M.Scratch(); scratch.busy = True
    with pytest.raises(ValueError, match="not reentrant"):
        scratch.predicate(np, fixture.view)
    scratch.busy = False
    original = np.isfinite
    monkeypatch.setattr(np, "isfinite", lambda view, out=None: original(view))
    with pytest.raises(ValueError, match="original private output"):
        scratch.predicate(np, fixture.view)
    assert scratch.busy is False and scratch.allocations == 1


def test_first_allocation_inside_the_first_timed_B_and_no_hidden_call(tiny):
    events = []; np = FakeNumPy(events); fixture = M.Fixture(np, tiny[0]); scratch = M.Scratch(); stamps = iter([10, 30, 40, 45])
    def clock():
        events.append("clock"); return next(stamps)
    first = M.measure("B", np, fixture.view, scratch, clock)
    assert events == ["clock", "allocate", "scan", "all", "clock"]
    assert first["duration_ns"] == 20 and first["first_scratch_allocation"] is True
    assert first["allocations_before"] == 0 and first["allocations_after"] == 1
    events.clear(); second = M.measure("B", np, fixture.view, scratch, clock)
    assert events == ["clock", "scan", "all", "clock"]
    assert second["duration_ns"] == 5 and second["first_scratch_allocation"] is False and np.finite_calls == 2


@pytest.mark.parametrize("invalid", [True, -1, 0, 1.5, float("nan"), None])
def test_bad_begin_clock_is_not_fabricated_as_native_ns(tiny, invalid):
    np = FakeNumPy(); fixture = M.Fixture(np, tiny[0])
    row = M.measure("A", np, fixture.view, M.Scratch(), lambda: invalid)
    assert row["error"] and row["begun_ns"] is row["finished_ns"] is row["duration_ns"] is None
    assert np.finite_calls == 0
    M.encoded(row)


@pytest.mark.parametrize("invalid", [True, 9, 1.5, float("inf"), None])
def test_bad_end_clock_preserves_actual_call_without_fake_end(tiny, invalid):
    np = FakeNumPy(); fixture = M.Fixture(np, tiny[0]); stamps = iter([10, invalid])
    row = M.measure("A", np, fixture.view, M.Scratch(), lambda: next(stamps))
    assert row["error"] and row["begun_ns"] == 10 and row["finished_ns"] is row["duration_ns"] is None
    assert row["result"] is True and np.finite_calls == 1
    M.encoded(row)


def test_predicate_error_preserved_no_synthetic_finished_clock(tiny, monkeypatch):
    np = FakeNumPy(); fixture = M.Fixture(np, tiny[0]); calls = []
    def fail(view): raise RuntimeError("original_scan_failure")
    monkeypatch.setattr(np, "isfinite", fail)
    row = M.measure("A", np, fixture.view, M.Scratch(), lambda: calls.append(1) or 10)
    assert calls == [1] and row["finished_ns"] is row["duration_ns"] is None
    assert "original_scan_failure" in row["error"]["prefix"]


def test_clock_exception_does_not_erase_begun_call(tiny):
    np = FakeNumPy(); fixture = M.Fixture(np, tiny[0]); calls = []
    def clock():
        calls.append(1)
        if len(calls) == 2: raise RuntimeError("clock_failed_after_scan")
        return 10
    row = M.measure("A", np, fixture.view, M.Scratch(), clock)
    assert row["begun_ns"] == 10 and row["result"] is True and row["finished_ns"] is None
    assert "clock_failed_after_scan" in row["error"]["prefix"]


def test_summary_keeps_first_and_failed_rows_and_unknowns(monkeypatch):
    monkeypatch.setattr(M, "cases", lambda: [{"case_id": "fixed"}])
    rows = [dict(case_id="fixed", phase="timed", arm="B", duration_ns=v, status=status)
            for v,status in [(100,"MATCH"), (1000,"FAILED_RETAINED"), (200,"MATCH"), (None,"UNEXECUTED"), (True,"FAILED_RETAINED")]]
    values = M.summary(rows)["fixed"]["B"]
    assert values["n"] == 3 and values["mean_ns"] == pytest.approx(1300/3)
    assert values["median_ns"] == 200 and values["p95_nearest_rank_ns"] == 1000
    assert values["failed_rows_included"] == 1 and values["missing_duration_UNKNOWN"] == 2
    assert values["first_allocation_not_removed"] is True


@pytest.fixture
def admission_tree(tmp_path, monkeypatch, tiny):
    plans = tmp_path/"plans"; plans.mkdir(); output = tmp_path/"output"
    source = tmp_path/"source.py"; source.write_bytes(b"# tiny declared source\n")
    provider = tmp_path/"provider.so"; provider.write_bytes(b"synthetic provider metadata only")
    sources = {"source.py": M.digest(source)}; providers = {str(provider): M.digest(provider)}
    monkeypatch.setattr(M, "ROOT", tmp_path); monkeypatch.setattr(M, "PLAN_DIR", plans)
    monkeypatch.setattr(M, "OUTPUT", output); monkeypatch.setattr(M, "PLAN", plans/"ROOT_plan.json")
    monkeypatch.setattr(M, "AUTHORITY", plans/"ROOT_authority.json"); monkeypatch.setattr(M, "PROVIDER_PINS", providers)
    monkeypatch.setattr(M, "source_table", lambda: sources.copy())
    (plans/"sources").mkdir(); (plans/"sources/source.py").write_bytes(source.read_bytes())
    plan = dict(schema=M.SCHEMA, fixed=M.fixed_contract(), cases=tiny, source_sha256=sources, provider_sha256=providers,
                schedule_sha256=hashlib.sha256(M.encoded(M.schedule())).hexdigest())
    M.PLAN.write_bytes(M.encoded(plan)); plan_sha = M.digest(M.PLAN)
    grant = dict(schema=M.SCHEMA+".ROOT_authority", plan_path=str(M.PLAN), plan_sha256=plan_sha,
                 source_sha256=sources, synthetic_NumPy_predicate_only=True, Torch_authorized=False,
                 NN_authorized=False, GPU_authorized=False, ROS_authorized=False, CARLA_authorized=False,
                 TRAIN_TEST_authorized=False, training_authorized=False, model_promoted=False)
    M.AUTHORITY.write_bytes(M.encoded(grant))
    return SimpleNamespace(plan=plan, grant=grant, plan_sha=plan_sha, authority_sha=M.digest(M.AUTHORITY), provider=provider)


def test_genuine_JSON_roundtrip_admission_without_numeric_import(admission_tree):
    assert M.admit(admission_tree.plan_sha, admission_tree.authority_sha) == admission_tree.plan
    assert not M.OUTPUT.exists()


@pytest.mark.parametrize("fault", ["source_archive", "provider", "extra_authorization", "schedule", "contract"])
def test_admission_source_provider_and_full_schedule_closed(admission_tree, fault):
    plan_sha, grant_sha = admission_tree.plan_sha, admission_tree.authority_sha
    if fault == "source_archive": (M.PLAN_DIR/"sources/source.py").write_bytes(b"changed")
    elif fault == "provider": admission_tree.provider.write_bytes(b"changed")
    elif fault == "extra_authorization":
        admission_tree.grant["NN_authorized"] = True
        M.AUTHORITY.write_bytes(M.encoded(admission_tree.grant)); grant_sha = M.digest(M.AUTHORITY)
    else:
        if fault == "schedule": admission_tree.plan["schedule_sha256"] = "0"*64
        else: admission_tree.plan["fixed"]["first_B_allocation_inside_timer"] = False
        M.PLAN.write_bytes(M.encoded(admission_tree.plan)); plan_sha = M.digest(M.PLAN)
    with pytest.raises(ValueError): M.admit(plan_sha, grant_sha)


@pytest.mark.parametrize("pin", [None, True, 5, "a"*63, "G"*64])
def test_external_RAW_authority_arguments_required_before_reads(pin, monkeypatch):
    monkeypatch.setattr(M, "read_json", lambda *args: pytest.fail("invalid external pin must deny before reads"))
    with pytest.raises(ValueError, match="two external RAW"): M.admit(pin, "a"*64)


def test_RAW_document_duplicate_and_nonfinite_denied(tmp_path):
    path = tmp_path/"bad.json"
    for raw in (b'{"a":1,"a":2}', b'{"a":NaN}'):
        path.write_bytes(raw)
        with pytest.raises(ValueError): M.read_json(path)
    path.write_bytes(b'{"value":5.0}\n')
    assert type(M.read_json(path)["value"]) is float
    with pytest.raises(ValueError, match="RAW document"): M.read_json(path, "a"*64)


def test_owned_alias_special_and_create_only_limits(tmp_path, monkeypatch):
    plans = tmp_path/"plans"; output = tmp_path/"output"; plans.mkdir(); output.mkdir()
    monkeypatch.setattr(M, "PLAN_DIR", plans); monkeypatch.setattr(M, "OUTPUT", output)
    actual = plans/"a.json"; M.write_new(actual, {"value": 1})
    with pytest.raises(FileExistsError): M.write_new(actual, {"value": 2})
    (output/"alias").symlink_to(actual)
    with pytest.raises(ValueError, match="alias"): M.storage_used()
    (output/"alias").unlink()
    with pytest.raises(ValueError, match="declared"): M.write_new(tmp_path/"outside.json", {})
    monkeypatch.setattr(M, "LIMIT", M.CLOSING+16)
    with pytest.raises(ValueError, match="quota"): M.write_new(output/"too_big.json", {"x":"A"*20})


def test_source_AST_retains_exact_A_and_full_B_without_where():
    tree = ast.parse(SOURCE.read_bytes())
    functions = {node.name: node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
    measure = functions["measure"]
    exact = ast.dump(ast.parse("bool(np.isfinite(view).all())", mode="eval").body, include_attributes=False)
    assert any(ast.dump(node, include_attributes=False) == exact for node in ast.walk(measure))
    calls = [node for node in ast.walk(functions["Scratch"]) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == "isfinite"]
    assert len(calls) == 1 and [kw.arg for kw in calls[0].keywords] == ["out"]
    imports = [node for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert not any((isinstance(node, ast.Import) and any(a.name.startswith(("numpy","torch","rclpy","carla")) for a in node.names))
                   or (isinstance(node, ast.ImportFrom) and (node.module or "").startswith(("numpy","torch","rclpy","carla"))) for node in imports)
    execute = functions["execute"]
    source = ast.get_source_segment(SOURCE.read_text(), execute)
    assert source.index("plan = admit(") < source.index('importlib.import_module("numpy")')
    assert source.index("with lease()") < source.index('importlib.import_module("numpy")')
    assert 'raw_ledger_sha256 == ledger_hash.hexdigest()' in source


@pytest.fixture
def mocked_execute(tmp_path, monkeypatch, tiny):
    """Lifecycle/bookkeeping simulation only; no genuine lease/provider/affinity admission."""
    output = tmp_path/"output"; plans = tmp_path/"plans"; plans.mkdir()
    monkeypatch.setattr(M, "ROOT", tmp_path); monkeypatch.setattr(M, "OUTPUT", output); monkeypatch.setattr(M, "PLAN_DIR", plans)
    monkeypatch.setattr(M, "sys", SimpleNamespace(flags=SimpleNamespace(optimize=0, dont_write_bytecode=True), byteorder="little",
                                               executable="/usr/bin/python3.10", modules={}, argv=["synthetic-bookkeeping-only"]))
    for key,value in M.ENV.items(): monkeypatch.setenv(key, value)
    for key in ("LD_PRELOAD","LD_AUDIT","PYTHONHOME","PYTHONPATH"): monkeypatch.delenv(key, raising=False)
    monkeypatch.setattr(M, "admit", lambda *args: {"cases": tiny})
    monkeypatch.setattr(M.os, "sched_getaffinity", lambda pid: {3})
    restored = []; monkeypatch.setattr(M.os, "sched_setaffinity", lambda pid,mask: restored.append(set(mask)))
    @contextmanager
    def fake_lease(): yield {"fd":37,"dev":1,"ino":1}
    monkeypatch.setattr(M, "lease", fake_lease)
    class FakeBudget:
        def __init__(self): self.started = M.time.monotonic(); self.max_HWM = 1; self.sampled_violation = False
        def guard(self, closing=False): pass
    monkeypatch.setattr(M, "Budget", FakeBudget)
    np = FakeNumPy(); native = SimpleNamespace(__file__="/usr/lib/python3/dist-packages/numpy/core/_multiarray_umath.cpython-310-x86_64-linux-gnu.so")
    monkeypatch.setattr(M.importlib.util, "find_spec", lambda name: SimpleNamespace(origin=np.__file__))
    monkeypatch.setattr(M.importlib, "import_module", lambda name: np if name == "numpy" else native)
    return SimpleNamespace(output=output, np=np, restored=restored)


def test_mocked_full_denominator_ledgers_all_first_calls_retained(mocked_execute):
    result = M.execute("a"*64, "b"*64)
    rows = [json.loads(raw) for raw in (mocked_execute.output/"predicate_calls.jsonl").read_bytes().splitlines()]
    assert result["status"] == "PASS_SYNTHETIC_NUMPY_PREDICATE_ONLY"
    assert len(rows) == result["actual_predicate_attempts"] == result["MATCH"] == 816
    assert result["FAILED"] == result["UNEXECUTED"] == 0 and mocked_execute.np.finite_calls == 816
    assert sum(row.get("first_scratch_allocation", False) for row in rows) == 12
    assert result["raw_ledger_sha256"] == result["accepted_flushed_prefix_sha256"]
    assert result["raw_ledger_complete"] and len(result["fixtures"]) == 12
    assert mocked_execute.restored == [{3}, {3}]


def test_mocked_scan_failure_keeps_original_one_and_815_unexecuted(mocked_execute, monkeypatch):
    monkeypatch.setattr(mocked_execute.np, "isfinite", lambda *args,**kwargs: (_ for _ in ()).throw(RuntimeError("original_fixture_failure")))
    result = M.execute("a"*64, "b"*64)
    rows = [json.loads(raw) for raw in (mocked_execute.output/"predicate_calls.jsonl").read_bytes().splitlines()]
    assert len(rows) == 816 and result["actual_predicate_attempts"] == result["FAILED"] == 1 and result["UNEXECUTED"] == 815
    assert "original_fixture_failure" in rows[0]["error"]["prefix"] and rows[0]["finished_ns"] is None
    assert all(row["status"] == "UNEXECUTED" for row in rows[1:])
    assert result["status"] == "FAILED_RETAINED" and result["raw_ledger_complete"]


def test_mocked_missing_raw_ledger_still_has_failed_closing_receipt(mocked_execute, monkeypatch):
    original = Path.open
    def denied(path, *args, **kwargs):
        if path.name == "predicate_calls.jsonl": raise PermissionError("raw_file_open_denied")
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, "open", denied)
    result = M.execute("a"*64, "b"*64)
    assert result["status"] == "FAILED_RETAINED" and result["actual_predicate_attempts"] == 0 and result["UNEXECUTED"] == 816
    assert result["raw_ledger_sha256"] is None and result["ledger_failure"]
    assert result["raw_ledger_complete"] is False and result["all_timing_rows_including_first_and_failures_retained"] is False
    assert (mocked_execute.output/"result.json").exists() and mocked_execute.restored[-1] == {3}


def test_mocked_partial_raw_write_keeps_exact_fragment_and_unaccepted_row(mocked_execute, monkeypatch):
    original = Path.open
    class PartialStream:
        def __init__(self, stream): self.stream = stream
        def write(self, raw): return self.stream.write(raw[:17])
        def flush(self): self.stream.flush()
        def close(self): self.stream.close()
    def partial(path, *args, **kwargs):
        stream = original(path, *args, **kwargs)
        return PartialStream(stream) if path.name == "predicate_calls.jsonl" and args == ("xb",) else stream
    monkeypatch.setattr(Path, "open", partial)
    result = M.execute("a"*64, "b"*64)
    raw = (mocked_execute.output/"predicate_calls.jsonl").read_bytes()
    assert result["status"] == "FAILED_RETAINED" and result["actual_predicate_attempts"] == 1 and result["UNEXECUTED"] == 815
    assert result["raw_ledger_complete"] is False and result["last_unaccepted_timing_row"]["ordinal"] == 0
    assert raw == M.encoded(result["last_unaccepted_timing_row"])[:17]
    assert result["raw_ledger_sha256"] == hashlib.sha256(raw).hexdigest()
    assert result["accepted_flushed_prefix_sha256"] == hashlib.sha256(b"").hexdigest()
    assert result["all_timing_rows_including_first_and_failures_retained"] is False


def test_mocked_postseal_error_retained_without_discarding_complete_rows(mocked_execute, monkeypatch):
    calls = []
    def admit(*args):
        calls.append(1)
        if len(calls) > 1: raise ValueError("actual_postsource_changed")
        return {"cases": M.cases()}
    monkeypatch.setattr(M, "admit", admit)
    result = M.execute("a"*64, "b"*64)
    assert result["status"] == "FAILED_RETAINED" and result["MATCH"] == 816 and result["UNEXECUTED"] == 0
    failed = [row for row in result["closing"] if not row["passed"]]
    assert len(failed) == 1 and "actual_postsource_changed" in failed[0]["error"]["prefix"]
    assert result["raw_ledger_complete"] and result["raw_ledger_sha256"] == result["accepted_flushed_prefix_sha256"]


def test_sampled_violation_cannot_become_PASS_during_closing(mocked_execute, monkeypatch):
    class ViolatingBudget:
        def __init__(self): self.started = M.time.monotonic(); self.max_HWM = 1; self.sampled_violation = False
        def guard(self, closing=False):
            if closing: self.sampled_violation = True
    monkeypatch.setattr(M, "Budget", ViolatingBudget)
    result = M.execute("a"*64, "b"*64)
    assert result["MATCH"] == 816 and result["status"] == "FAILED_RETAINED" and result["sampled_time_or_memory_violation"]
