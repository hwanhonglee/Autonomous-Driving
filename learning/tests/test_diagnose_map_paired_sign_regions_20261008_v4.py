"""HH_261008 - Scalar/set/AST/fake-file witnesses only; no NumPy/SciPy or real mask replay.

V3 scientific functions and MaskStream remain literal AST-equivalent. The four
within-GT counts expose support turnover even when unchanged NET TP counts cancel.
Synthetic lifecycle fixtures are extracted exactly from the frozen V3 test,
then supplied explicit fake new seams; they are not evidence of numeric replay.
"""
import ast
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace
import zipfile

import pytest
from scripts.e2e import diagnose_map_paired_sign_regions_20261008_v4 as check


V3_SOURCE=check.ROOT/"tools/e2e/diagnose_map_paired_sign_regions_20261008_v3.py"
V3_TEST=check.ROOT/"learning/tests/test_diagnose_map_paired_sign_regions_20261008_v3.py"


class SetMask:
    """Tiny exact Boolean set algebra, not an actual ndarray or semantic image."""
    def __init__(self, values, universe=range(8)):
        self.values=frozenset(values);self.universe=frozenset(universe)
        self.shape=check.HW;self.dtype="EXPLICIT_FAKE_BOOL"
    def __and__(self,other):return SetMask(self.values&other.values,self.universe)
    def __or__(self,other):return SetMask(self.values|other.values,self.universe)
    def __invert__(self):return SetMask(self.universe-self.values,self.universe)


FAKE_NP=SimpleNamespace(ndarray=SetMask,bool_="EXPLICIT_FAKE_BOOL",
    any=lambda mask:bool(mask.values),count_nonzero=lambda mask:len(mask.values))


def test_equal_region_TP_NET_counts_can_hide_actual_support_turnover():
    truth=SetMask({0,1,2,3});reference=SetMask({0,1});candidate=SetMask({2,3})
    ref=check.graph_summary((4,),(2,),[(1,1,2)])
    cand=check.graph_summary((4,),(2,),[(1,1,2)])
    old=check.paired_summary((4,),ref,cand)
    assert old[2:6]==[2,2,0,0]
    counts=check.within_GT_pixel_partition(truth,reference,candidate,FAKE_NP)
    assert check.validate_pixel_partition(counts,old)==[0,2,2,0]
    assert counts[2]-old[4]==counts[1]-old[5]==2


@pytest.mark.parametrize("truth,reference,candidate,expected",[
    (set(),set(),set(),[0,0,0,0]),
    ({0,1,2,3},{0,1},{1,2},[1,1,1,1]),
    ({0,1,2},{0,1,2},{0,1,2},[3,0,0,0]),
    ({0,1,2},{0,1,2},set(),[0,3,0,0]),
    ({0,1,2},set(),{0,1,2},[0,0,3,0]),
    ({0,1,2},set(),set(),[0,0,0,3]),
])
def test_complete_exact_four_way_partition_and_NET_identities(truth,reference,candidate,expected):
    counts=check.within_GT_pixel_partition(SetMask(truth),SetMask(reference),SetMask(candidate),FAKE_NP)
    areas=(len(truth),) if truth else ()
    def graph(values):
        return check.graph_summary(areas,(len(values),) if values else (),[(1,1,len(values))] if values else [])
    pair=check.paired_summary(areas,graph(reference),graph(candidate))
    assert check.validate_pixel_partition(counts,pair)==expected
    assert sum(counts)==len(truth) and counts[0]+counts[1]==len(reference) and counts[0]+counts[2]==len(candidate)


@pytest.mark.parametrize("fault",["outsideGT","shape","dtype","foreignmask","negative","bool","short","TP","GT","NET","oversize"])
def test_partition_and_exact_original_NET_invariants_fail_closed(fault):
    truth=SetMask({0,1,2,3});reference=SetMask({0,1});candidate=SetMask({1,2})
    pair=check.paired_summary((4,),check.graph_summary((4,),(2,),[(1,1,2)]),check.graph_summary((4,),(2,),[(1,1,2)]))
    counts=[1,1,1,1]
    if fault=="outsideGT":reference=SetMask({0,4})
    elif fault=="shape":reference.shape=(1,1)
    elif fault=="dtype":reference.dtype="FOREIGN"
    elif fault=="foreignmask":reference=SimpleNamespace(shape=check.HW,dtype=FAKE_NP.bool_)
    elif fault=="negative":counts[0]=-1
    elif fault=="bool":counts[0]=True
    elif fault=="short":counts.pop()
    elif fault=="TP":pair[2]+=1
    elif fault=="GT":pair[1]+=1
    elif fault=="NET":pair[4]=2;pair[5]=2
    else:counts[0]=check.PIXELS+1
    with pytest.raises(ValueError):
        check.within_GT_pixel_partition(truth,reference,candidate,FAKE_NP)
        check.validate_pixel_partition(counts,pair)


def archive_fixture(tmp_path,monkeypatch):
    old=tmp_path/"old-output";old.mkdir();plan=tmp_path/"old-plan";plan.mkdir()
    sources=tmp_path/"sources";sources.mkdir()
    source={}
    for number in range(20):
        path=sources/f"original_{number:02d}.py";path.write_bytes(f"# frozen fake source {number}\n".encode())
        source[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    census={}
    for number,(path,pin) in enumerate(sorted(source.items())):
        target=old/f"source_{number:02d}_{Path(path).name}";target.write_bytes(Path(path).read_bytes())
        census[str(target)]=dict(sha256=pin,size=target.stat().st_size)
    for number in range(6):
        path=old/f"other_{number:02d}.json";path.write_bytes(b"{}\n")
        census[str(path)]=dict(sha256=hashlib.sha256(path.read_bytes()).hexdigest(),size=3)
    candidate=plan/"candidate.json";post=plan/"post.json"
    previous=dict(source_sha256=dict(sorted(source.items())),output=str(old))
    proof=dict(status="COMPLETE_METADATA_SCALAR_INTEGRITY_NOT_MODEL_APPROVAL",source_count_current=20,
        source_archive_count_output=20,complete_output_census=census)
    candidate.write_bytes(check.encoded(previous));post.write_bytes(check.encoded(proof))
    monkeypatch.setattr(check,"V3_OUTPUT",old);monkeypatch.setattr(check,"V3_CANDIDATE",candidate)
    monkeypatch.setattr(check,"V3_POSTCHECK",post);monkeypatch.setattr(check,"V3_SOURCE_PINS",dict(source))
    fresh={}
    for number in range(6):
        path=sources/f"fresh_{number:02d}.py";path.write_bytes(b"# fresh fake witness\n")
        fresh[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    full={**source,**fresh}
    def reseal():
        candidate.write_bytes(json.dumps(previous,separators=(",",":"),allow_nan=False).encode()+b"\n")
        post.write_bytes(check.encoded(proof))
        monkeypatch.setattr(check,"V3_EVIDENCE_PINS",{str(candidate):check.digest(candidate),str(post):check.digest(post)})
    reseal()
    return full,previous,proof,reseal


def test_all20_actual_archive_basenames_and_current_paths_tied_to_pins(tmp_path,monkeypatch):
    full,previous,proof,reseal=archive_fixture(tmp_path,monkeypatch)
    before={path:Path(path).read_bytes() for path in proof["complete_output_census"]}
    protocol=check.source_archive_protocol(full)
    assert len(protocol["reused_source_archive_sha256"])==20 and len(protocol["prior_output_sha256"])==26
    assert len(protocol["new_source_sha256"])==6 and protocol["historical_archives_read_only_no_copy_or_hardlink"] is True
    for number,(path,pin) in enumerate(sorted(previous["source_sha256"].items())):
        archive=str(check.V3_OUTPUT/f"source_{number:02d}_{Path(path).name}")
        assert protocol["reused_source_archive_sha256"][archive]==full[path]==pin
    assert {path:Path(path).read_bytes() for path in before}==before


@pytest.mark.parametrize("fault",["sourcepin","candidate_member","ordering","retarget","extraarchive","bytes","escape","symlink","hardlink","freshmissing"])
def test_archive_reuse_never_accepts_guessed_missing_extra_alias_or_mutated_history(tmp_path,monkeypatch,fault):
    full,previous,proof,reseal=archive_fixture(tmp_path,monkeypatch)
    first=next(iter(previous["source_sha256"]));target=next(path for path in proof["complete_output_census"] if Path(path).name.startswith("source_"))
    if fault=="sourcepin":full[first]="f"*64
    elif fault=="candidate_member":previous["source_sha256"].pop(first)
    elif fault=="ordering":previous["source_sha256"]=dict(reversed(list(previous["source_sha256"].items())))
    elif fault in ("retarget","extraarchive","escape"):
        old=Path(target if fault!="extraarchive" else next(path for path in proof["complete_output_census"] if "other_" in path))
        new=(old.parent/("source_extra.py" if fault=="extraarchive" else "source_00_WRONG.py")) if fault!="escape" else tmp_path/"escaped.py"
        old.rename(new);entry=proof["complete_output_census"].pop(str(old));proof["complete_output_census"][str(new)]=entry
    elif fault=="bytes":Path(target).write_bytes(b"# mutation\n")
    elif fault=="symlink":Path(target).unlink();Path(target).symlink_to(first)
    elif fault=="hardlink":Path(target).unlink();os.link(first,target)
    else:full.pop(next(path for path in full if path not in previous["source_sha256"]))
    reseal()
    with pytest.raises(ValueError):check.source_archive_protocol(full)


def maximum_row(cohort="holdout_town02_p02",ordinal=231,count=230400):
    sha="f"*64
    return dict(cohort=cohort,ordinal=ordinal,GT_pixel_sha256=sha,GT_scan_order_region_labels_i32_sha256=sha,
        GT_scan_order_area_i64_sha256=sha,
        arms=[dict(arm=arm,prediction_pixel_sha256=sha,confusion_sha256=sha,sparse_overlap_edge_sha256=sha,counts=[count]*18) for arm in check.ARMS],
        paired=[dict(reference=arm,counts=[count]*24,within_GT_correct_pixel_partition=[count]*4) for arm in ("parent","control")],
        status="FRAME_CHECKED_PENDING_ARCHIVE_EOF")


@pytest.mark.parametrize("cohort,ordinal",[("dev_town01",0),("dev_town01",117),("holdout_town02_p02",0),("holdout_town02_p02",231)])
@pytest.mark.parametrize("count",[0,1,9,10,99999,230400])
def test_full_row_fieldwise_bound_and_unchanged_2560_allocation(cohort,ordinal,count):
    proof=check.normal_frame_row_capacity_proof()
    assert len(check.encoded(maximum_row(cohort,ordinal,count)))<=proof["maximum_complete_normal_row_bytes"]==len(check.encoded(maximum_row()))
    assert 2099<proof["maximum_complete_normal_row_bytes"]<=check.FRAME_RECORD_CAP==2560
    assert proof["within_GT_correct_pixel_partition_entries_per_reference"]==4
    assert check.LIMIT==2097152 and check.WORKING==1572864 and check.CLOSING==524288


def test_capacity_shape_matches_exact_genuine_producer_not_a_minimal_UNEXECUTED_row():
    functions={part.name:part for part in ast.parse(Path(check.__file__).read_bytes()).body if isinstance(part,ast.FunctionDef)}
    actual=next(n.args[1] for n in ast.walk(functions["execute"]) if isinstance(n,ast.Call) and ast.unparse(n.func)=="append_frame")
    maximum=next(n.value for n in functions["normal_frame_row_capacity_proof"].body if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=="maximum")
    assert [k.arg for k in actual.keywords]==[k.arg for k in maximum.keywords]
    actual_pair=next(n.args[0] for n in ast.walk(functions["execute"]) if isinstance(n,ast.Call) and ast.unparse(n.func)=="paired.append")
    proof_pair=next(k.value.elt for k in maximum.keywords if k.arg=="paired")
    assert [k.arg for k in actual_pair.keywords]==[k.arg for k in proof_pair.keywords]
    assert ast.unparse(next(k.value for k in actual_pair.keywords if k.arg=="counts"))=="values"
    assert ast.unparse(next(k.value for k in actual_pair.keywords if k.arg=="within_GT_correct_pixel_partition"))=="partition"


def original_fake_lifecycle_namespace():
    require_names={"SymbolicPixels","SymbolicConfusion","lifecycle_fixture"}
    tree=ast.parse(V3_TEST.read_bytes())
    selected=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in require_names]
    assert {n.name for n in selected}==require_names
    namespace=dict(check=check,hashlib=hashlib,Path=Path,ModuleType=ModuleType,SimpleNamespace=SimpleNamespace,sys=sys)
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(V3_TEST)+":original_fake_only","exec"),namespace)
    return namespace


def lifecycle_fixture(tmp_path,monkeypatch,fault=None):
    old=original_fake_lifecycle_namespace();out=old["lifecycle_fixture"](tmp_path,monkeypatch,fault)
    monkeypatch.setattr(old["SymbolicPixels"],"__and__",lambda self,other:self,raising=False)
    previous_admission=check.admission
    def admitted(sha):
        authority,candidate,cpu=previous_admission(sha)
        candidate["source_archive_protocol"]=dict(new_source_sha256=candidate["source_sha256"],reused_source_archive_sha256={},prior_output_sha256={})
        return authority,candidate,cpu
    monkeypatch.setattr(check,"admission",admitted)
    monkeypatch.setattr(check,"source_archive_protocol",lambda source,*_:dict(new_source_sha256=source,reused_source_archive_sha256={},prior_output_sha256={}))
    def partition(*_):
        if fault=="partition":raise ValueError("SYNTHETIC_PARTITION_FAILURE")
        return [0,0,0,0]
    monkeypatch.setattr(check,"within_GT_pixel_partition",partition)
    return out


@pytest.mark.parametrize("fault,passed,failed,pending,unexecuted,GT",[
    ("before_import",0,0,0,1050,0),("first",0,1,0,1049,0),("mid",0,1,3,1046,1),
    ("EOF",0,1,353,696,118),("partition",0,1,2,1047,0)])
def test_new_partition_failures_and_original_full_denominator_lifecycle(tmp_path,monkeypatch,fault,passed,failed,pending,unexecuted,GT):
    out=lifecycle_fixture(tmp_path,monkeypatch,fault);result=check.execute("a"*64)
    assert result["status"]=="FAILED_WITH_COMPLETE_DENOMINATOR" and result["within_GT_correct_pixel_partition_counts_verified"] is False
    assert result["status_counts"]==dict(PASS=passed,FAIL=failed,FRAME_CHECKED_PENDING_ARCHIVE_EOF=pending,UNEXECUTED=unexecuted)
    assert len(result["frames"])==1050 and result["processed_GT_frames"]==GT
    assert json.loads((out/"result.json").read_bytes())["frames"]==result["frames"] and "SYNTHETIC_" in result["error"]


def test_fake_complete_not_actual_numeric_evidence_preserves_original27_and350_rows(tmp_path,monkeypatch):
    out=lifecycle_fixture(tmp_path,monkeypatch);result=check.execute("a"*64)
    assert result["status_counts"]==dict(PASS=1050,FAIL=0,FRAME_CHECKED_PENDING_ARCHIVE_EOF=0,UNEXECUTED=0)
    assert result["within_GT_correct_pixel_partition_counts_verified"] is True
    records=(out/"frame_regions.jsonl").read_bytes().splitlines(keepends=True)
    assert len(records)==350 and hashlib.sha256(b"".join(records)).hexdigest()==result["full_frame_receipt_sha256"]
    assert all(len(json.loads(raw)["paired"])==2 for raw in records)
    assert all(row==[0,0,0,0] for cell in result["within_GT_correct_pixel_partition_cells"].values() for row in cell.values())
    assert result["original_27_criteria_verified"] and sum(result["original_exploratory_screen"]["checks"].values())==23
    assert len(result["selected_member_EOF_CRC_and_pixel_SHA_verified"])==8
    assert result["model_performance_improved"] is False and check.tree_bytes(out)<=check.LIMIT


def test_every_unchanged_V3_function_and_MaskStream_are_literal_AST_identical():
    old={n.name:n for n in ast.parse(V3_SOURCE.read_bytes()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
    new={n.name:n for n in ast.parse(Path(check.__file__).read_bytes()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
    assert new.keys()==old.keys()|{"source_archive_protocol","validate_pixel_partition","within_GT_pixel_partition"}
    changed={"sources","declared_contract","normal_frame_row_capacity_proof","prepare","admission","execute"}
    for name in old.keys()-changed:assert ast.dump(new[name],include_attributes=False)==ast.dump(old[name],include_attributes=False),name
    for name in ("graph_summary","paired_summary","metrics","original_screens","MaskStream","native_regions","native_graph"):
        assert ast.dump(new[name],include_attributes=False)==ast.dump(old[name],include_attributes=False)
    columns=lambda tree,name:next(n for n in tree.body if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id==name)
    for name in ("GRAPH_COLUMNS","PAIR_COLUMNS"):
        assert ast.dump(columns(ast.parse(V3_SOURCE.read_bytes()),name),include_attributes=False)==ast.dump(columns(ast.parse(Path(check.__file__).read_bytes()),name),include_attributes=False)


def test_no_provider_import_before_actual_ROOT_lease_seals_and_historical_join():
    tree=ast.parse(Path(check.__file__).read_bytes());functions={n.name:n for n in tree.body if isinstance(n,ast.FunctionDef)}
    imports={a.name.split(".")[0] for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom)) for a in n.names}
    assert not imports&{"numpy","scipy","torch","PIL","rclpy","carla"}
    body=ast.unparse(functions["execute"])
    assert body.index("admission(authority_sha)")<body.index("flock")<body.index("source_archive_protocol")<body.index("import numpy")
    assert "np.load" not in body and "load_model" not in body and "rgb.npy" not in body
    assert check.RSS==536870912 and check.DEADLINE==900 and len(check.ASSETS)==8
    with pytest.raises(ValueError):check.admission(None)


def test_V3_twenty_sources_and_actual_failure_success_provenance_are_sealed():
    assert len(check.V3_SOURCE_PINS)==20 and len(check.V4_POSTCHECK_SOURCE_WITNESSES)==2
    for path,pin in {**check.V3_SOURCE_PINS,**check.V3_EVIDENCE_PINS}.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==pin
    current=check.sources();assert len(current)==26 and set(check.V3_SOURCE_PINS)<=set(current)
    assert len(set(current)-set(check.V3_SOURCE_PINS))==6
    assert set(check.V3_SOURCE_PINS)==set(json.loads(check.V3_CANDIDATE.read_bytes())["source_sha256"])
    assert check.declared_contract()["actual_newly_correct_or_lost_pixel_sets_verified"] is False
    assert check.declared_contract()["outside_GT_FP_turnover_measured"] is False
    skeleton=json.loads(check.SKELETON.read_bytes())
    assert skeleton["actual_array_decode_authorized"] is skeleton["self_execution_grant"] is False


def restore_only_v4_partition_and_archive_protocol(function):
    """HH_261008 - Remove exactly reviewed additions; unrelated statements/guards must still differ."""
    part=deepcopy(function)
    drops=[];kwargs={};replace_assign={};replace_for={}
    if part.name=="execute":
        drops=ast.parse('''partition_cells = {}
require(candidate["source_archive_protocol"] == source_archive_protocol(candidate["source_sha256"], check), "historical source/archive reuse changed before numeric imports")
partition_totals = {arm: [0]*len(PIXEL_PARTITION_COLUMNS) for arm in ("parent", "control")}
truth_sign = truth == 5
support_masks = {}
support_masks[arm] = (pred == 5) & truth_sign
partition = validate_pixel_partition(within_GT_pixel_partition(truth_sign, support_masks[comparator], support_masks["candidate"], np), values)
for i, n in enumerate(partition): partition_totals[comparator][i] += n
partition_cells[cohort] = partition_totals
require(candidate["source_archive_protocol"] == source_archive_protocol(candidate["source_sha256"], lambda: guard(start, generation, lease)), "closing historical source/archive reuse changed")
result["within_GT_correct_pixel_partition_counts_verified"] = True''').body
        kwargs={"within_GT_correct_pixel_partition_cells":"partition_cells", "within_GT_correct_pixel_partition_counts_verified":"False",
            "within_GT_correct_pixel_partition_columns":"list(PIXEL_PARTITION_COLUMNS)","outside_GT_FP_turnover_measured":"False",
            "within_GT_correct_pixel_partition":"partition"}
        # The only other use of this keyword is the separately named final column map.
        replace_for={"enumerate(candidate['source_archive_protocol']['new_source_sha256'].items())":"enumerate(candidate['source_sha256'].items())"}
    elif part.name=="declared_contract":
        kwargs={"within_GT_correct_pixel_partition_columns":"list(PIXEL_PARTITION_COLUMNS)",
            "within_GT_partition_scope":repr("same_GT_label5_pixels_only; actual counts, not persisted pixel locations"),
            "outside_GT_FP_turnover_measured":"False","historical_source_archives_reused_read_only":"20"}
    elif part.name=="normal_frame_row_capacity_proof":
        kwargs={"within_GT_correct_pixel_partition":"[PIXELS]*len(PIXEL_PARTITION_COLUMNS)",
            "within_GT_correct_pixel_partition_entries_per_reference":"4"}
    elif part.name=="prepare":
        drops=ast.parse("archive_protocol = source_archive_protocol(source)").body
        kwargs={"source_archive_protocol":"archive_protocol"}
        replace_assign={"archive_bytes = sum(Path(path).stat().st_size for path in archive_protocol['new_source_sha256'])":
            "archive_bytes = sum(Path(path).stat().st_size for path in source)"}
    counts={ast.dump(node,include_attributes=False):0 for node in drops};seen_kwargs={};seen_assign={};seen_for={};seen_admission=[]
    dump=lambda node:ast.dump(node,include_attributes=False)
    assignments={dump(ast.parse(before).body[0]):ast.parse(after).body[0] for before,after in replace_assign.items()}
    iterations={dump(ast.parse(before,mode="eval").body):ast.parse(after,mode="eval").body for before,after in replace_for.items()}
    class Restore(ast.NodeTransformer):
        def visit(self,node):
            signature=dump(node)
            if isinstance(node,ast.stmt) and signature in counts:counts[signature]+=1;return None
            if signature in assignments:seen_assign[signature]=seen_assign.get(signature,0)+1;return deepcopy(assignments[signature])
            return super().visit(node)
        def visit_For(self,node):
            signature=dump(node.iter)
            if signature in iterations:seen_for[signature]=seen_for.get(signature,0)+1;node.iter=deepcopy(iterations[signature])
            return self.generic_visit(node)
        def visit_Call(self,node):
            if isinstance(node.func,ast.Name) and node.func.id=="dict":
                retained=[]
                for keyword in node.keywords:
                    if keyword.arg not in kwargs:retained.append(keyword);continue
                    expected=kwargs[keyword.arg]
                    if part.name=="execute" and keyword.arg=="within_GT_correct_pixel_partition" and dump(keyword.value)==dump(ast.parse("list(PIXEL_PARTITION_COLUMNS)",mode="eval").body):
                        expected="list(PIXEL_PARTITION_COLUMNS)"
                    assert dump(keyword.value)==dump(ast.parse(expected,mode="eval").body)
                    key=(keyword.arg,expected);seen_kwargs[key]=seen_kwargs.get(key,0)+1
                node.keywords=retained
            return self.generic_visit(node)
        def visit_BoolOp(self,node):
            if part.name=="admission":
                expected=ast.parse('candidate["source_archive_protocol"] == source_archive_protocol(candidate["source_sha256"])',mode="eval").body
                matches=[i for i,value in enumerate(node.values) if dump(value)==dump(expected)]
                if matches:
                    assert matches==[len(node.values)-1];seen_admission.append(1);node.values.pop()
            return self.generic_visit(node)
    part=Restore().visit(part)
    assert all(number==1 for number in counts.values())
    assert all(number==1 for number in seen_kwargs.values())
    assert {key[0] for key in seen_kwargs}==set(kwargs)
    assert seen_assign=={key:1 for key in assignments} and seen_for=={key:1 for key in iterations}
    if part.name=="execute":assert seen_kwargs[("within_GT_correct_pixel_partition","partition")]==seen_kwargs[("within_GT_correct_pixel_partition","list(PIXEL_PARTITION_COLUMNS)")]==1
    assert seen_admission==([1] if part.name=="admission" else [])
    return part


def test_all_original_V3_processing_clocks_guards_and_formulas_restore_exactly():
    old={n.name:n for n in ast.parse(V3_SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef)}
    new={n.name:n for n in ast.parse(Path(check.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef)}
    for name in ("declared_contract","normal_frame_row_capacity_proof","prepare","admission","execute"):
        assert ast.dump(restore_only_v4_partition_and_archive_protocol(new[name]),include_attributes=False)==ast.dump(old[name],include_attributes=False),name


@pytest.mark.parametrize("fault",["wrong_formula","removed_guard","wrong_count_field","wrong_copy_scope"])
def test_narrow_inverse_cannot_hide_unapproved_original_math_or_admission_change(fault):
    functions={n.name:n for n in ast.parse(Path(check.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef)}
    part=deepcopy(functions["execute"])
    if fault=="wrong_formula":
        call=next(n for n in ast.walk(part) if isinstance(n,ast.Call) and ast.unparse(n.func)=="paired_summary")
        call.args[0]=ast.parse("()",mode="eval").body
    elif fault=="removed_guard":
        call=next(n for n in ast.walk(part) if isinstance(n,ast.Call) and ast.unparse(n.func)=="require" and isinstance(n.args[-1],ast.Constant) and n.args[-1].value=="whole original49-cell frame confusion differs")
        call.args[0]=ast.Constant(value=True)
    elif fault=="wrong_count_field":
        call=next(n for n in ast.walk(part) if isinstance(n,ast.Call) and ast.unparse(n.func)=="paired.append")
        next(k for k in call.args[0].keywords if k.arg=="within_GT_correct_pixel_partition").value=ast.parse("[]",mode="eval").body
        with pytest.raises(AssertionError):restore_only_v4_partition_and_archive_protocol(part)
        return
    else:
        loop=next(n for n in ast.walk(part) if isinstance(n,ast.For) and ast.unparse(n.iter)=="enumerate(candidate['source_archive_protocol']['new_source_sha256'].items())")
        loop.iter=ast.parse("enumerate(candidate['source_sha256'].items())",mode="eval").body
        with pytest.raises(AssertionError):restore_only_v4_partition_and_archive_protocol(part)
        return
    old=next(n for n in ast.parse(V3_SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="execute")
    assert ast.dump(restore_only_v4_partition_and_archive_protocol(part),include_attributes=False)!=ast.dump(old,include_attributes=False)
