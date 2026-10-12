"""HH_261005 - Verify private V5 whole-run sealer identities and external policy binding using synthetic files only."""

import ast
import copy
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
from types import SimpleNamespace

import pytest

from scripts.e2e import seal_town03_full209_actual_publication_v5_20261005 as m


_synthetic_declaration_sha=None


def invoke(mode,policy_sha):
    return m.main(["--expected-declaration-sha256",_synthetic_declaration_sha,
        "--expected-policy-sha256",policy_sha,"--mode",mode])


def test_frozen_source_chain_and_fresh_identity_do_not_read_episodes():
    m.verify_sources()
    assert m.p.RUN.name.endswith("full209_v5") and m.p.OUTPUT.name=="01_town03_full_route209_v5"
    assert m.p.CAPTURE_SCHEMA.endswith(".v1") and m.p.OWNER_SCHEMA.endswith(".v5")
    assert m.CALLER.parent==m.p.POLICY_PATH.parent and m.POSTCHECK_SCHEMA.endswith(".v5")
    assert not hasattr(m,"PLAN_SHA")


def test_exact_original_code_inverse_AST_and_all_frozen_globals_remain_unmodified():
    original=m.previous.previous.previous.previous
    before=dict(vars(original))
    save=m._delegate_save_new()
    assert save.__code__ is original.save_new.__code__ and save.__globals__["p"] is m.p
    tree=ast.parse(textwrap.dedent(inspect.getsource(original.main))).body[0]
    changed=copy.deepcopy(tree);m._identity_seams(changed)
    assert sum(isinstance(row,ast.Call) and ast.unparse(row.func)=="p.previous.previous.previous.previous.previous.inventory"
        for row in ast.walk(changed))==2
    m._identity_seams(changed,undo=True)
    assert ast.dump(changed)==ast.dump(tree)
    function=m._delegate_main("b"*64,"a"*64)
    assert function.__globals__["PLAN_SHA"]=="b"*64
    assert function.__globals__["POLICY_SHA"]=="a"*64
    assert function.__globals__["p"] is m.p and function.__globals__["CALLER"]==m.CALLER
    assert dict(vars(original))==before


@pytest.mark.parametrize("fault",["extra_inventory","missing_inventory","missing_schema"])
def test_wrong_original_AST_anchor_count_fails_closed(fault):
    tree=ast.parse(textwrap.dedent(inspect.getsource(m.previous.previous.previous.previous.main))).body[0]
    calls=[row for row in ast.walk(tree) if isinstance(row,ast.Call) and ast.unparse(row.func)=="p.previous.inventory"]
    if fault=="extra_inventory": tree.body.append(ast.Expr(value=copy.deepcopy(calls[0])))
    elif fault=="missing_inventory": calls[0].func=ast.parse("arbitrary.inventory",mode="eval").body
    else:
        for row in ast.walk(tree):
            if isinstance(row,ast.Constant) and row.value==m.ORIGINAL_POSTCHECK_SCHEMA: row.value="wrong"
    with pytest.raises(RuntimeError): m._identity_seams(tree)


@pytest.mark.parametrize("name",["PREVIOUS_SHA","PREVIOUS_TEST_SHA","PUBLISHER_SHA","TEST_SHA"])
def test_changed_source_pin_rejects_before_any_actual_policy_or_episode_read(monkeypatch,name):
    monkeypatch.setattr(m,name,"0"*64)
    with pytest.raises(RuntimeError): m.verify_sources()


def test_original_save_is_create_only_and_refuses_aliases(tmp_path):
    save=m._delegate_save_new()
    path=tmp_path/"private/receipt.json"
    sha=save(path,dict(synthetic_CPU_only=True,approved=False))
    before=path.read_bytes()
    assert hashlib.sha256(before).hexdigest()==sha
    with pytest.raises(RuntimeError): save(path,dict(approved=True))
    alias=tmp_path/"alias.json";alias.symlink_to(path)
    with pytest.raises(RuntimeError): save(alias,{})
    with pytest.raises(RuntimeError): save(Path("relative_receipt.json"),{})
    assert path.read_bytes()==before and alias.is_symlink()


@pytest.fixture
def archive(tmp_path,monkeypatch):
    # HH_261005 - Hash genuine temporary bytes; fake decoding is explicit control-flow coverage, never image or driving evidence.
    real=m.p
    run=tmp_path/"validation/full209_v5";run.mkdir(parents=True)
    output=tmp_path/"gallery/01_town03_full_route209_v5"
    policy=tmp_path/"plans/full209_v5/policy.json";policy.parent.mkdir(parents=True)
    policy.write_bytes(b'{"synthetic_CPU_only":true,"created_before_actual_episode":true}\n')
    plan=run.with_name(run.name+"_plan.json");payload=b'{"synthetic_CPU_only":true,"approved":false}\n'
    plan.write_bytes(payload);(run/"declaration.json").write_bytes(payload)
    for name in ("owner_result.json","diagnostic_report.json","failed_tail.bin"):
        (run/name).write_bytes(b"synthetic metadata or complete failure-tail bytes\n")
    publisher=tmp_path/"tools/fake_publisher.py";publisher.parent.mkdir();publisher.write_bytes(b"# synthetic CPU source\n")
    test=tmp_path/"tests/fake_test.py";test.parent.mkdir();test.write_bytes(b"# synthetic CPU test\n")
    digest=real.prior.digest
    calls=[]
    states=[dict(frame=100+index,timestamp=5+index*.05) for index in range(35)]
    bundle=dict(states=states,cameras=[dict(row) for row in states[::2]],diagnostic=dict(status="FAIL_SYNTHETIC_CPU_ONLY"))
    class Pins:
        values={"synthetic_external_pin":"0"*64}
        def recheck(self): calls.append("all_original_pins_rechecked")
    def load_archive(path,sha,*,decode=False):
        calls.append(dict(path=path,sha=sha,decode=decode))
        return json.loads(path.read_bytes()),bundle,Pins()
    inventory=real.previous.previous.previous.previous.previous.inventory
    fake=SimpleNamespace(ROOT=tmp_path,RUN=run,OUTPUT=output,POLICY_PATH=policy,__file__=str(publisher),
        _canonical=real._canonical,require=real.require,prior=real.prior,CALLER_SCHEMA=real.CALLER_SCHEMA,
        previous=SimpleNamespace(previous=SimpleNamespace(previous=SimpleNamespace(previous=SimpleNamespace(previous=SimpleNamespace(inventory=inventory))))),
        publication_sources=lambda:[publisher],load_archive=load_archive,camera_indices=real.camera_indices,
        png_indices=real.png_indices,playback=real.playback)
    monkeypatch.setattr(m,"p",fake);monkeypatch.setattr(m,"verify_sources",lambda:None)
    monkeypatch.setitem(globals(),"_synthetic_declaration_sha",digest(plan))
    for name,value in dict(PUBLISHER_SHA=digest(publisher),TEST=test,TEST_SHA=digest(test),
            CALLER=policy.parent/"caller.json",POSTCHECK=tmp_path/"postcheck/receipt.json").items():
        monkeypatch.setattr(m,name,value)
    return dict(run=run,output=output,policy=policy,plan=plan,calls=calls,bundle=bundle,digest=digest,policy_sha=digest(policy))


def test_caller_pins_complete_failed_tail_source_fonts_and_external_policy_create_only(archive,capsys):
    value=archive
    assert invoke("caller",value["policy_sha"]) is None
    caller=json.loads(m.CALLER.read_bytes());result=json.loads(capsys.readouterr().out)
    assert set(caller["run_files_sha256"])=={"declaration.json","owner_result.json","diagnostic_report.json","failed_tail.bin"}
    assert caller["schema"]==m.p.CALLER_SCHEMA and caller["prospective_publication_policy"]["sha256"]==value["policy_sha"]
    assert len(caller["font_sha256"])==2 and result["raw_file_count"]==4
    assert caller["formal_function_approval"] is caller["independent_validation"] is False
    assert not value["output"].exists() and value["calls"]==[]
    before=m.CALLER.read_bytes()
    with pytest.raises(RuntimeError): invoke("caller",value["policy_sha"])
    assert m.CALLER.read_bytes()==before


@pytest.mark.parametrize("fault",["missing_pin","malformed_pin","uppercase_pin","wrong_pin","changed_policy",
    "symlink_policy","unfinished_owner","unfinished_diagnostic","plan_change","gallery_exists"])
def test_wrong_or_unfinished_bindings_refuse_caller_without_archive_decode(archive,fault):
    value=archive;expected=value["policy_sha"]
    if fault=="missing_pin":
        with pytest.raises(SystemExit): m.main(["--mode","caller"])
        return
    if fault=="malformed_pin": expected="not-a-sha"
    elif fault=="uppercase_pin": expected=expected.upper()
    elif fault=="wrong_pin": expected="0"*64
    elif fault=="changed_policy": value["policy"].write_bytes(b"changed policy")
    elif fault=="symlink_policy":
        target=value["policy"].with_name("elsewhere.json");value["policy"].rename(target);value["policy"].symlink_to(target)
    elif fault=="unfinished_owner": (value["run"]/"owner_result.json").unlink()
    elif fault=="unfinished_diagnostic": (value["run"]/"diagnostic_report.json").unlink()
    elif fault=="plan_change": value["plan"].write_bytes(b"changed declaration")
    else: value["output"].mkdir(parents=True)
    with pytest.raises(RuntimeError): invoke("caller",expected)
    assert not m.CALLER.exists() and value["calls"]==[]


def synthetic_gallery(value,monkeypatch):
    invoke("caller",value["policy_sha"])
    output=value["output"];output.mkdir(parents=True)
    bundle=value["bundle"]
    indices=m.p.camera_indices(len(bundle["cameras"]));durations=m.p.playback(bundle,indices);pngs=m.p.png_indices(len(bundle["cameras"]))
    media={}
    for name in ["full_actual_six_cameras_2hz.gif",*pngs]:
        path=output/name;path.write_bytes(b"explicit synthetic media control-flow fixture, not a real image\n")
        media[name]=value["digest"](path)
    provenance=dict(caller_sha256=value["digest"](m.CALLER),failed_tail_cropped=False,GIF_camera_indices=indices,
        GIF_duration_ms=durations,PNG_camera_indices=pngs,GIF_total_duration_ms=sum(durations),media_sha256=media)
    (output/"publication_provenance.json").write_bytes(json.dumps(provenance).encode())
    class Saved:
        def __init__(self,path): self.path=path;self.n_frames=len(indices);self.info={"duration":durations[0]}
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def seek(self,index): self.info["duration"]=durations[index];value["calls"].append(("seek",index))
        def load(self): value["calls"].append(("load",self.path.name))
    old=m._namespace
    monkeypatch.setattr(m,"_namespace",lambda declaration_sha,policy_sha:{**old(declaration_sha,policy_sha),"Image":SimpleNamespace(open=Saved)})
    return provenance,indices,durations,pngs


def test_postcheck_keeps_full_native_camera_failed_tail_and_all_media_rechecks(archive,monkeypatch,capsys):
    value=archive;provenance,indices,durations,pngs=synthetic_gallery(value,monkeypatch)
    capsys.readouterr()
    assert invoke("postcheck",value["policy_sha"]) is None
    report=json.loads(m.POSTCHECK.read_bytes());result=json.loads(capsys.readouterr().out)
    assert report["schema"]==m.POSTCHECK_SCHEMA and report["original_status"]=="FAIL_SYNTHETIC_CPU_ONLY"
    assert report["native_state_count"]==35 and report["camera_bundle_count"]==18
    assert report["original_JPEG_count"]==108 and report["media_count"]==6
    assert report["GIF_frame_count"]==len(indices) and report["GIF_duration_ms"]==sum(durations)
    assert report["failed_tail_cropped"] is report["learned_STOP_claim"] is report["formal_function_approval"] is False
    assert report["prospective_policy_sha256"]==value["policy_sha"]
    assert result["postcheck_sha256"]==value["digest"](m.POSTCHECK)
    assert value["calls"][0]["decode"] is True
    assert value["calls"].count("all_original_pins_rechecked")==2
    assert sum(isinstance(row,tuple) and row[0]=="load" for row in value["calls"])==len(indices)+5


@pytest.mark.parametrize("fault",["caller","crop","GIF_indices","PNG_indices","duration","media_members","media_hash"])
def test_media_or_failure_tail_change_cannot_pass_postcheck(archive,monkeypatch,fault):
    value=archive;provenance,indices,durations,pngs=synthetic_gallery(value,monkeypatch)
    if fault=="caller": provenance["caller_sha256"]="0"*64
    elif fault=="crop": provenance["failed_tail_cropped"]=True
    elif fault=="GIF_indices": provenance["GIF_camera_indices"]=indices[:-1]
    elif fault=="PNG_indices": provenance["PNG_camera_indices"]["05_last.png"]=0
    elif fault=="duration": provenance["GIF_duration_ms"][0]+=100
    elif fault=="media_members": provenance["media_sha256"].pop("05_last.png")
    else: provenance["media_sha256"]["05_last.png"]="0"*64
    (value["output"]/"publication_provenance.json").write_bytes(json.dumps(provenance).encode())
    with pytest.raises(RuntimeError): invoke("postcheck",value["policy_sha"])
    assert not m.POSTCHECK.exists()


@pytest.mark.parametrize("declaration_pin", ["0"*64, "NOT_A_SHA", "B"*64])
def test_external_declaration_pin_is_independently_required(archive,declaration_pin):
    # HH_261005 - Wrong declaration pins cannot be replaced by a genuine policy or a completed synthetic archive.
    with pytest.raises(RuntimeError):
        m.main(["--expected-declaration-sha256",declaration_pin,
            "--expected-policy-sha256",archive["policy_sha"],"--mode","caller"])
    assert not m.CALLER.exists() and archive["calls"]==[]


def test_external_declaration_argument_cannot_be_omitted(archive):
    with pytest.raises(SystemExit):
        m.main(["--expected-policy-sha256",archive["policy_sha"],"--mode","caller"])
    assert not m.CALLER.exists() and archive["calls"]==[]


def test_declaration_alias_is_rejected_despite_matching_policy_and_file_bytes(archive):
    path=archive["plan"];other=path.with_name("foreign_declaration.json")
    path.rename(other);path.symlink_to(other)
    with pytest.raises(RuntimeError): invoke("caller",archive["policy_sha"])
    assert path.is_symlink() and not m.CALLER.exists() and archive["calls"]==[]


def test_whole_archive_real_inventory_provider_is_exact_frozen_original():
    # HH_261005 - The five-layer identity reaches the genuine original canonical membership walk, not a stand-in alias.
    original=m.previous.previous.previous.previous
    function=m._delegate_main("b"*64,"a"*64)
    assert function.__globals__["p"] is m.p
    assert m.p.previous.previous.previous.previous.previous.inventory is original.p.previous.inventory
    assert m._delegate_save_new().__code__ is original.save_new.__code__
