"""HH_261008 - Pure scalar/byte fixtures, never real masks, packages or model inference.

The lifecycle fixtures inject explicit tiny fake provider modules and symbolic
pixel operations. They exercise retained progress and output bounds, not numeric
environment equivalence or a genuine NumPy/SciPy connected-component execution.
"""
import ast
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace
import zipfile

import pytest
from scripts.e2e import diagnose_map_paired_sign_regions_20261008_v1 as check


def header(count=1, dtype="|u1", fortran=False, version=b"\x01\x00", shape=None):
    raw = repr(dict(descr=dtype, fortran_order=fortran, shape=(count, *check.HW) if shape is None else shape)).encode()+b"\n"
    return b"\x93NUMPY"+version+len(raw).to_bytes(2 if version == b"\x01\x00" else 4, "little")+raw


def zipped(path, members, compression=zipfile.ZIP_DEFLATED):
    with zipfile.ZipFile(path, "w", compression=compression) as stream:
        for name, raw in members: stream.writestr(name, raw)


class FakeArray:
    def __init__(self, buffer): self.buffer = buffer
    def reshape(self, shape): self.shape = shape; return self


def fake_numpy_bytes():
    return SimpleNamespace(uint8="fixture-u1", frombuffer=lambda raw, dtype: FakeArray(raw))


@pytest.mark.parametrize("version", [b"\x01\x00", b"\x02\x00", b"\x03\x00"])
def test_exact_native_header_without_pixels(version):
    stream = io.BytesIO(header(version=version)+b"payload")
    result = check.npy_header(stream, 1)
    assert result == dict(descr="|u1", fortran_order=False, shape=(1,360,640))
    assert stream.read() == b"payload"


@pytest.mark.parametrize("payload", [b"", b"notnpy", b"\x93NUMPY\x04\x00", b"\x93NUMPY\x01\x00\x01",
    b"\x93NUMPY\x01\x00"+(16385).to_bytes(2,"little"), header(dtype="<f4"), header(fortran=True),
    header(shape=(True,360,640)), header(shape=(360,640)), b"\x93NUMPY\x01\x00\x05\x00{}"])
def test_header_abi_fail_closed(payload):
    with pytest.raises((ValueError, SyntaxError, UnicodeDecodeError)):
        check.npy_header(io.BytesIO(payload), 1)


def test_stream_reuses_one_buffer_target_only_and_requires_complete_EOF(tmp_path, monkeypatch):
    path = tmp_path/"fixture.npz"; first = bytes([5])*check.PIXELS; second = bytes([0])*check.PIXELS
    zipped(path, [("rgb.npy", b"malformed-and-never-open"), ("target.npy", header(count=2)+first+second)])
    opened = []; original = zipfile.ZipFile.open
    def tracked(self, member, *args, **kwargs):
        opened.append(member.filename if isinstance(member, zipfile.ZipInfo) else member)
        return original(self, member, *args, **kwargs)
    monkeypatch.setattr(zipfile.ZipFile, "open", tracked)
    with check.MaskStream(path, "target.npy", 2) as reader:
        a, sha_a = reader.read_frame(fake_numpy_bytes()); identity = id(reader.buffer)
        assert a.shape == check.HW and sha_a == hashlib.sha256(first).hexdigest()
        with pytest.raises(ValueError, match="incomplete full"): reader.finish()
        b, sha_b = reader.read_frame(fake_numpy_bytes())
        assert id(reader.buffer) == identity and a.buffer is b.buffer and sha_b == hashlib.sha256(second).hexdigest()
        assert reader.finish() == hashlib.sha256(first+second).hexdigest() and reader.complete is True
        with pytest.raises(ValueError, match="extra frame"): reader.read_frame(fake_numpy_bytes())
        assert reader.ordinal == 2
    assert opened == ["target.npy"]


@pytest.mark.parametrize("fault", ["foreign", "duplicate", "path", "dtype", "shape", "short", "tail", "unknownlabel"])
def test_selected_stream_denials_preserve_progress(tmp_path, fault):
    pixels = bytes([7 if fault == "unknownlabel" else 0])*check.PIXELS
    payload = header(dtype="<f4" if fault == "dtype" else "|u1",
                     shape=(2,360,640) if fault == "shape" else None)+pixels
    if fault == "short": payload = payload[:-1]
    if fault == "tail": payload += b"!"
    members = [("../prediction.npy" if fault == "path" else "prediction.npy", payload)]
    if fault == "foreign": members.append(("weights.npy", b"forbidden"))
    if fault == "duplicate": members.append(("prediction.npy", payload))
    path = tmp_path/"bad.npz"; zipped(path, members)
    with pytest.raises(ValueError):
        with check.MaskStream(path, "prediction.npy", 1) as reader:
            reader.read_frame(fake_numpy_bytes()); reader.finish()


def test_selected_member_bad_CRC_is_not_a_header_only_success(tmp_path):
    path = tmp_path/"crc.npz"; pixels = bytes([0])*check.PIXELS
    zipped(path, [("prediction.npy", header()+pixels)], zipfile.ZIP_STORED)
    raw = bytearray(path.read_bytes()); start = raw.index(b"\x93NUMPY")
    raw[start+len(header())+check.PIXELS-1] = 1
    path.write_bytes(raw)
    with pytest.raises(zipfile.BadZipFile):
        with check.MaskStream(path, "prediction.npy", 1) as reader:
            reader.read_frame(fake_numpy_bytes()); reader.finish()


@pytest.mark.parametrize("fault", ["zero", "negative", "bool", "oversized"])
def test_partial_readinto_progress_cannot_be_fabricated(tmp_path, fault):
    path = tmp_path/"reader.npz"; zipped(path, [("prediction.npy", header()+bytes(check.PIXELS))])
    with check.MaskStream(path, "prediction.npy", 1) as reader:
        actual = reader.stream
        class Broken:
            def readinto(self, view): return {"zero":0, "negative":-1, "bool":True, "oversized":len(view)+1}[fault]
            def close(self): actual.close()
        reader.stream = Broken()
        with pytest.raises(ValueError): reader.read_frame(fake_numpy_bytes())
        assert reader.ordinal == 0 and reader.pixel_sha.hexdigest() == hashlib.sha256(b"").hexdigest()


def test_sparse_split_merge_isolated_and_missed_are_mask_regions_not_objects():
    # GT1 splits into P1/P2; P2 connects GT1/GT2; GT3 is missed; P3 is isolated FP.
    value = check.graph_summary((4,3,2), (2,5,7), [(1,1,2), (1,2,1), (2,2,3)])
    assert value["GT_TP"] == (3,3,0) and value["GT_degree"] == (2,1,0)
    assert value["counts"] == [3,9,3,14,6,3,8,3,1,1,1,1,1,1,7,1,1,1]
    assert value["edge_sha256"] == hashlib.sha256(b"".join(struct.pack("<qqq", *r) for r in [(1,1,2),(1,2,1),(2,2,3)])).hexdigest()
    assert check.graph_summary((), (3,), [])["counts"] == [0,0,1,3,0,0,3,0,0,0,0,1,0,0,3,0,0,0]
    assert check.graph_summary((2,), (), [])["counts"] == [1,2,0,0,0,2,0,0,1,0,0,0,0,0,0,0,0,0]


@pytest.mark.parametrize("gt,pred,edges", [
    ((True,), (1,), []), ((0,), (), []), ((check.PIXELS+1,), (), []), ((check.PIXELS,1), (), []),
    ((1,), (1,), [(True,1,1)]), ((1,), (1,), [(1,1,0)]), ((1,), (1,), [(1,1,2)]),
    ((1,), (1,), [(2,1,1)]), ((1,), (1,), [(1,2,1)]), ((2,), (2,), [(1,1,1),(1,1,1)]),
    ((2,2), (2,2), [(2,1,1),(1,2,1)]), ((1,), (1,), [(1,1,1.0)]), ((1,), (1,), [(1,1)]),
])
def test_sparse_graph_corrupt_edges_or_areas_denied(gt, pred, edges):
    with pytest.raises(ValueError): check.graph_summary(gt, pred, edges)


def test_same_GT_region_loss_and_recovery_keep_every_region_and_pixel():
    areas = (4,3,2,5)
    reference = check.graph_summary(areas, (2,1,4), [(1,1,2),(2,2,1),(4,3,4)])
    candidate = check.graph_summary(areas, (3,2,4), [(2,1,3),(3,2,2),(4,3,4)])
    counts = check.paired_summary(areas, reference, candidate)
    assert counts[:15] == [4,14,7,9,4,2,1,2,1,1,1,0,2,4,2]
    assert sum(counts[15:]) == 4 and counts[15+1] == 1 and counts[15+3] == 1 and counts[15+4] == 2
    assert check.paired_summary((), check.graph_summary((),(),[]), check.graph_summary((),(),[])) == [0]*len(check.PAIR_COLUMNS)


@pytest.mark.parametrize("fault", ["length", "bool", "degree", "overlap", "vector_type"])
def test_paired_support_denials_are_atomic(fault):
    ref = check.graph_summary((2,), (1,), [(1,1,1)]); candidate = dict(ref)
    if fault == "length": candidate["GT_TP"] = ()
    elif fault == "bool": candidate["GT_TP"] = (True,)
    elif fault == "degree": candidate["GT_degree"] = (0,)
    elif fault == "overlap": candidate["GT_TP"] = (3,)
    elif fault == "vector_type": candidate["GT_TP"] = [1]
    before = json.dumps(ref, sort_keys=True)
    with pytest.raises(ValueError): check.paired_summary((2,), ref, candidate)
    assert json.dumps(ref, sort_keys=True) == before


def test_all1050_slots_AND350_fixed_GT_families_are_declared():
    rows = check.ledger()
    assert len(rows) == 1050 and len({(r[0], r[1]) for r in rows}) == 350
    assert rows[:3] == [["dev_town01",0,arm,"UNEXECUTED"] for arm in check.ARMS]
    assert rows[353] == ["dev_town01",117,"candidate","UNEXECUTED"]
    assert rows[354] == ["holdout_town02_p02",0,"parent","UNEXECUTED"]
    assert rows[-1] == ["holdout_town02_p02",231,"candidate","UNEXECUTED"]
    before = json.dumps(rows)
    for invalid in (-1,1050,True,1.0):
        with pytest.raises(ValueError): check.progress(rows, invalid, "FAIL")
        assert json.dumps(rows) == before
    check.progress(rows,0,"FRAME_CHECKED_PENDING_ARCHIVE_EOF")
    check.progress(rows,0,"FAIL")
    assert sum(r[3] == "FAIL" for r in rows) == 1 and sum(r[3] == "UNEXECUTED" for r in rows) == 1049
    with pytest.raises(ValueError): check.progress(rows,0,"PASS")


def test_no_PASS_before_selected_member_EOF_transition():
    rows = check.ledger()
    with pytest.raises(ValueError): check.progress(rows,0,"PASS")
    check.progress(rows,0,"FRAME_CHECKED_PENDING_ARCHIVE_EOF")
    check.progress(rows,0,"PASS")
    with pytest.raises(ValueError): check.progress(rows,0,"FAIL")


def test_original_scalar_metrics_have_identical_genuine_AST():
    source = check.OLD.read_bytes()
    assert hashlib.sha256(source).hexdigest() == check.OLD_SOURCE_PINS[str(check.OLD)]
    old = next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == "metrics")
    new = next(n for n in ast.parse(Path(check.__file__).read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "metrics")
    assert ast.dump(old, include_attributes=False) == ast.dump(new, include_attributes=False)


def test_genuine_original27_are_not_replaced_by_overlap_graph_thresholds():
    row = dict(precision={"traffic_sign":.5}, recall={"traffic_sign":.5}, iou={k:.5 for k in check.LABELS}, other_to_sign_false_positive_pixels=100)
    evaluation = {cohort:{arm:json.loads(json.dumps(row)) for arm in check.ARMS} for cohort,_ in check.COHORTS}
    evaluation["dev_town01"]["candidate"].update(precision={"traffic_sign":.6}, recall={"traffic_sign":.47}, other_to_sign_false_positive_pixels=80)
    evaluation["dev_town01"]["candidate"]["iou"]["traffic_sign"] = .6
    screen = check.original_screens(evaluation)
    assert len(screen["checks"]) == 27 and sum(screen["checks"].values()) == 23
    assert {k for k,v in screen["checks"].items() if not v} == {
        "Town01_sign_recall_vs_parent", "Town01_sign_recall_vs_control",
        "ADDED_Town01_sign_recall_strict_gain_vs_parent", "ADDED_Town01_sign_recall_strict_gain_vs_control"}
    assert screen["model_promoted"] is screen["vehicle_control_approved"] is False


def test_full_frame_and_closing_bounds_never_truncate(tmp_path, monkeypatch):
    monkeypatch.setattr(check,"OUTPUT",tmp_path)
    with (tmp_path/"frame_regions.jsonl").open("xb") as stream:
        with pytest.raises(ValueError): check.append_frame(stream, dict(oversize="x"*check.FRAME_RECORD_CAP))
        assert stream.tell() == 0
    check.write_raw(tmp_path/"working.bin", bytes(check.WORKING))
    with pytest.raises(ValueError): check.write_raw(tmp_path/"extra.bin",b"x")
    check.write_raw(tmp_path/"closing.bin",bytes(check.CLOSING),check.LIMIT)
    with pytest.raises(ValueError): check.write_raw(tmp_path/"over-total.bin",b"x",check.LIMIT)
    assert check.tree_bytes(tmp_path) == check.LIMIT and not (tmp_path/"extra.bin").exists()


def test_RAW_bytes_and_uint64_are_not_normalized(tmp_path, monkeypatch):
    monkeypatch.setattr(check,"OUTPUT",tmp_path)
    payload=b'{ "ns": 18446744073709551615, "float":5.0 }\n'
    check.write_raw(tmp_path/"raw.json",payload)
    assert (tmp_path/"raw.json").read_bytes() == payload
    parsed = check.read_json(tmp_path/"raw.json")
    assert type(parsed["ns"]) is int and parsed["ns"] == 2**64-1 and type(parsed["float"]) is float
    with pytest.raises(FileExistsError): check.write_raw(tmp_path/"raw.json",payload)
    with pytest.raises(ValueError): check.encoded(dict(nonfinite=float("nan")))


def test_no_authentication_from_skeleton_or_missing_external_SHA():
    with pytest.raises(ValueError, match="external RAW"): check.admission(None)
    skeleton=json.loads(check.AUTHORITY_SKELETON.read_text())
    assert skeleton["stored_mask_region_diagnostic_authorized"] is False and skeleton["candidate_sha256"] is None
    assert not check.declared_contract()["model_performance_improved"]
    assert len(check.ASSETS) == 8 and sum(dict(check.COHORTS).values())*len(check.ARMS) == 1050


@pytest.mark.parametrize("fault", ["missing", "units", "duplicate", "foreign", "reverse", "boolPID"])
def test_current_memory_parser_fail_closed(fault):
    text="Pid: 12\nTgid: 12\nVmRSS: 100 kB\nVmHWM: 200 kB\n"; pid=12
    if fault=="missing": text=text.replace("VmHWM: 200 kB\n","")
    elif fault=="units": text=text.replace(" kB"," MB")
    elif fault=="duplicate": text+="VmRSS: 100 kB\n"
    elif fault=="foreign": text=text.replace("Pid: 12","Pid: 13")
    elif fault=="reverse": text=text.replace("VmHWM: 200","VmHWM: 50")
    elif fault=="boolPID": pid=True
    with pytest.raises(ValueError): check.parse_process_memory(text,pid)


def test_memory_parser_and_inherited_rusage_are_separate(monkeypatch):
    text="Pid: 12\nTgid: 12\nVmRSS: 100 kB\nVmHWM: 200 kB\n"
    assert check.parse_process_memory(text,12) == dict(PID=12,VmRSS_bytes=102400,VmHWM_bytes=204800)
    monkeypatch.setattr(Path,"open",lambda *a,**k:io.StringIO(text))
    monkeypatch.setattr(check.os,"getpid",lambda:12)
    monkeypatch.setattr(check.resource,"getrusage",lambda _:SimpleNamespace(ru_maxrss=10**9))
    witness=check.process_memory()
    assert witness["inherited_ru_maxrss_bytes_NOT_GATE"] == 10**9*1024 and witness["VmHWM_bytes"] == 204800


class SymbolicPixels:
    """Explicit fake algebra, not a real tensor, ndarray or recorded image."""
    def __eq__(self, other): return self
    def __mul__(self, other): return self
    def __add__(self, other): return self
    def ravel(self): return self
    def tobytes(self): return b"SYNTHETIC_NO_MASK_BYTES"


class SymbolicConfusion:
    def reshape(self,*_): return self
    def tolist(self):
        matrix=[[0]*7 for _ in range(7)]; matrix[0][0]=check.PIXELS; return matrix


def lifecycle_fixture(tmp_path, monkeypatch, fault=None):
    """HH_261008 - Isolate fake IO/providers; no real assets or /proc probes are reached."""
    out=tmp_path/"result"; lock=tmp_path/"fake-lock"; lock.write_bytes(b"")
    dummy=tmp_path/"dummy.py"; dummy.write_text("# synthetic source only\n")
    sha=hashlib.sha256(dummy.read_bytes()).hexdigest()
    files=[]
    for attr in ("AUTHORITY","CANDIDATE","REVIEW"):
        path=tmp_path/(attr.lower()+".json"); path.write_bytes(b"{}"); monkeypatch.setattr(check,attr,path); files.append(path)
    monkeypatch.setattr(check,"OUTPUT",out); monkeypatch.setattr(check,"LOCK",lock)
    provider=tmp_path/"provider.py"; provider.write_text("# fake provider, never imported\n")
    candidate=dict(source_sha256={str(dummy):sha},providers=dict(selected_file_sha256={},
        top_level_origins=dict(numpy=str(provider),scipy=str(provider))))
    authority=dict(candidate_sha256="a"*64,review_sha256="a"*64)
    monkeypatch.setattr(check,"admission",lambda _: (authority,candidate,[3]))
    generation=dict(pid=12,start_ticks=1,boot_id="0"*8+"-"+"0"*4+"-"+"0"*4+"-"+"0"*4+"-"+"0"*12,uid=1,ppid=1,pgid=12,session=12)
    monkeypatch.setattr(check,"process_generation",lambda:generation)
    monkeypatch.setattr(check,"process_memory",lambda:dict(PID=12,VmRSS_bytes=100,VmHWM_bytes=200,inherited_ru_maxrss_bytes_NOT_GATE=10**12))
    monkeypatch.setattr(check,"guard",lambda *args,**kwargs:check.process_memory())
    monkeypatch.setattr(check.os,"sched_getaffinity",lambda _: {3,4})
    monkeypatch.setattr(check.os,"sched_setaffinity",lambda *args:None)
    calls=[]
    def pins(*_):
        calls.append(1)
        if fault=="before_import" and len(calls)==1: raise ValueError("SYNTHETIC_PRE_FIRST_FAILURE")
    monkeypatch.setattr(check,"verify_pins",pins); monkeypatch.setattr(check,"digest",lambda *args,**kwargs:"a"*64)
    np=ModuleType("numpy"); np.__version__=check.PACKAGES["numpy"]; np.__file__=str(provider)
    np.bincount=lambda *a,**k:SymbolicConfusion()
    scipy=ModuleType("scipy"); scipy.__file__=str(provider); scipy.ndimage=SimpleNamespace()
    monkeypatch.setitem(sys.modules,"numpy",np); monkeypatch.setitem(sys.modules,"scipy",scipy)
    monkeypatch.setattr(check.importlib.metadata,"version",lambda p:check.PACKAGES[p])
    monkeypatch.setattr(check,"native_regions",lambda *args:(SymbolicPixels(),()))
    monkeypatch.setattr(check,"native_graph",lambda *args:check.graph_summary((),(),[]))
    matrix=SymbolicConfusion().tolist()
    lookup={(r[0],r[1],r[2]):dict(confusion=matrix) for r in check.ledger()}
    cells={}
    for cohort,count in check.COHORTS:
        for arm in check.ARMS:
            total=[[n*count for n in row] for row in matrix]
            cells[cohort+"/"+arm]=dict(target_pixel_bytes_sha256="a"*64,prediction_pixel_bytes_sha256="a"*64,metrics=check.metrics(total))
    screen=dict(checks={str(i):i<23 for i in range(27)},passed_exploratory_only=False,
        model_promoted=False,vehicle_control_approved=False,independent_validation=False)
    old=dict(cells=cells,original_exploratory_screen=screen)
    saved=dict(evaluation={cohort:{arm:cells[cohort+"/"+arm]["metrics"] for arm in check.ARMS} for cohort,_ in check.COHORTS},exploratory_screen=screen)
    monkeypatch.setattr(check,"original_reference",lambda:(old,saved,lookup))
    monkeypatch.setattr(check,"original_screens",lambda _:screen)
    class FakeStream:
        def __init__(self,path,member,count): self.member=member; self.count=count; self.ordinal=0
        def __enter__(self): return self
        def __exit__(self,*_): return None
        def read_frame(self,*_):
            if self.member=="target.npy" and self.ordinal == (1 if fault=="mid" else 0) and fault in ("first","mid"):
                raise ValueError("SYNTHETIC_RETAINED_FRAME_FAILURE")
            self.ordinal+=1; return SymbolicPixels(),"a"*64
        def finish(self):
            if fault=="EOF": raise ValueError("SYNTHETIC_EOF_CRC_FAILURE")
            return "a"*64
    monkeypatch.setattr(check,"MaskStream",FakeStream)
    return out


@pytest.mark.parametrize("fault,passed,failed,pending,unexecuted,GT", [
    ("before_import",0,0,0,1050,0), ("first",0,1,0,1049,0), ("mid",0,1,3,1046,1),
    ("EOF",0,1,353,696,118),
])
def test_fake_lifecycle_preserves_first_mid_EOF_and_pre_first_denominators(tmp_path,monkeypatch,fault,passed,failed,pending,unexecuted,GT):
    out=lifecycle_fixture(tmp_path,monkeypatch,fault)
    result=check.execute("a"*64)
    assert result["status"] == "FAILED_WITH_COMPLETE_DENOMINATOR"
    assert result["status_counts"] == dict(PASS=passed,FAIL=failed,FRAME_CHECKED_PENDING_ARCHIVE_EOF=pending,UNEXECUTED=unexecuted)
    assert sum(result["status_counts"].values()) == len(result["frames"]) == 1050
    assert result["processed_GT_frames"] == GT and "SYNTHETIC_" in result["error"]
    assert result["formal_main_approved"] == result["formal_sub_approved"] == result["model_inferences"] == 0
    assert json.loads((out/"result.json").read_text())["frames"] == result["frames"]


def test_fake_full350_progression_is_NOT_actual_numeric_or_model_evidence(tmp_path,monkeypatch):
    out=lifecycle_fixture(tmp_path,monkeypatch)
    result=check.execute("a"*64)
    assert result["status"] == "COMPLETE_DESCRIPTIVE_PAIRED_MASK_REGIONS_NOT_MODEL_APPROVAL"
    assert result["status_counts"] == dict(PASS=1050,FAIL=0,FRAME_CHECKED_PENDING_ARCHIVE_EOF=0,UNEXECUTED=0)
    assert result["processed_GT_frames"] == 350 and len(result["selected_member_EOF_CRC_and_pixel_SHA_verified"]) == 8
    rows=(out/"frame_regions.jsonl").read_bytes().splitlines(keepends=True)
    assert len(rows)==350 and hashlib.sha256(b"".join(rows)).hexdigest()==result["full_frame_receipt_sha256"]
    assert all(json.loads(row)["status"]=="FRAME_CHECKED_PENDING_ARCHIVE_EOF" for row in rows)
    assert result["model_performance_improved"] is result["physical_HD_topology_verified"] is False
    assert check.tree_bytes(out) <= check.LIMIT and (out/"result.json").stat().st_size <= check.CLOSING
    assert json.loads((out/"started.json").read_text())["original_CPU_before_job"] == [3,4]


def test_no_numeric_imports_before_sealed_authority_and_no_dense_region_matrix():
    tree=ast.parse(Path(check.__file__).read_text())
    top=[n for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom))]
    names={alias.name.split(".")[0] for n in top for alias in n.names}
    assert not names & {"numpy","scipy","torch","carla","rclpy","PIL"}
    functions={n.name:n for n in tree.body if isinstance(n,ast.FunctionDef)}
    text=ast.unparse(functions["execute"])
    assert text.index("admission(authority_sha)") < text.index("flock") < text.index("import numpy")
    assert text.index("verify_pins(candidate['providers']") < text.index("import numpy")
    assert "read_frame" in text and "original_screens(evaluation)" in text and "SIGALRM" in text
    graph=ast.unparse(functions["native_graph"])
    assert "np.unique(packed, return_counts=True)" in graph and "np.zeros" not in graph
    assert "load_model" not in text and "np.load" not in text and "rgb.npy" not in text
    assert check.RSS==512*1024**2 and check.DEADLINE==900 and check.LIMIT==2*1024**2 and check.CLOSING==512*1024
