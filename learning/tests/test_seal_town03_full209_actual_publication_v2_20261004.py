"""HH_261004 - Verify the new pure V2 caller/postcheck identity and unchanged original guards using CPU-only fixtures.

All temporary archive and media bytes below are explicitly synthetic tests of
control flow, not driving or image evidence. No actual episode, camera payload,
simulator, model inference, training or remote environment is used.
"""

import ast
import copy
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
from types import SimpleNamespace

import pytest

from scripts.e2e import seal_town03_full209_actual_publication_v2_20261004 as m


def test_exact_frozen_sources_and_fresh_V2_identities():
    m.verify_sources()
    assert m.PUBLISHER_SHA=="354abb6ed79074af3de835ea4d0893d17755cf0a8010fd60917ef397de3e6669"
    assert m.p.RUN.name.endswith("full209_v2") and m.p.CAPTURE_SCHEMA.endswith(".v1")
    assert m.CALLER.parent==m.p.POLICY_PATH.parent
    assert m.CALLER.name=="ROOT_actual_publication_caller.json"
    assert m.POSTCHECK.parent.name=="town03_full209_v2_publication_postcheck"
    assert m.POSTCHECK_SCHEMA.endswith(".v2")
    assert m.PLAN_SHA=="797f217310db8f6a1533b063e6bfa07e975591873c8e36ed962e5a73d35e98b8"
    assert m.POLICY_SHA=="a386ae9955a6e6251095aa8da85165e83e8d8b899db1bc81bfc85470417c19a8"


def test_private_delegate_exact_save_code_and_inverse_proven_main_do_not_mutate_frozen_modules():
    original_p=m.previous.p
    original_main=m.previous.main
    original_save=m.previous.save_new
    before=dict(original_main.__globals__)
    save=m._delegate_save_new()
    assert save.__code__ is original_save.__code__ and save.__globals__["p"] is m.p
    assert save.__globals__ is not original_save.__globals__
    tree=ast.parse(textwrap.dedent(inspect.getsource(original_main))).body[0]
    changed=copy.deepcopy(tree)
    m._identity_seams(changed)
    assert sum(isinstance(row,ast.Call) and ast.unparse(row.func)=="p.previous.previous.inventory"
        for row in ast.walk(changed))==2
    assert sum(isinstance(row,ast.Constant) and row.value==m.POSTCHECK_SCHEMA for row in ast.walk(changed))==1
    m._identity_seams(changed,undo=True)
    assert ast.dump(changed)==ast.dump(tree)
    function=m._delegate_main()
    assert function.__globals__["p"] is m.p and function.__globals__["CALLER"]==m.CALLER
    assert function.__globals__["POSTCHECK"]==m.POSTCHECK and function.__globals__["__file__"]==m.__file__
    assert m.previous.main is original_main and m.previous.save_new is original_save and m.previous.p is original_p
    assert before==dict(original_main.__globals__)


@pytest.mark.parametrize("fault",["extra_inventory","missing_inventory","missing_schema"])
def test_changed_AST_seam_count_cannot_hide_guard_changes(fault):
    tree=ast.parse(textwrap.dedent(inspect.getsource(m.previous.main))).body[0]
    calls=[row for row in ast.walk(tree) if isinstance(row,ast.Call) and ast.unparse(row.func)=="p.previous.inventory"]
    if fault=="extra_inventory": tree.body.append(ast.Expr(value=copy.deepcopy(calls[0])))
    elif fault=="missing_inventory": calls[0].func=ast.parse("unrelated.inventory",mode="eval").body
    else:
        for row in ast.walk(tree):
            if isinstance(row,ast.Constant) and row.value==m.ORIGINAL_POSTCHECK_SCHEMA: row.value="wrong_schema"
    with pytest.raises(RuntimeError): m._identity_seams(tree)


@pytest.mark.parametrize("pin",["PREVIOUS_SHA","PUBLISHER_SHA","TEST_SHA"])
def test_source_change_is_denied_before_any_episode_read(monkeypatch,pin):
    monkeypatch.setattr(m,pin,"0"*64)
    with pytest.raises(RuntimeError): m.verify_sources()


def test_exact_original_create_only_save_refuses_overwrite_and_symlink(tmp_path):
    save=m._delegate_save_new()
    path=tmp_path/"new/private/receipt.json"
    sha=save(path,dict(provenance="HH_261004 - Synthetic CPU receipt only.",approved=False))
    before=path.read_bytes()
    assert hashlib.sha256(before).hexdigest()==sha and json.loads(before)["approved"] is False
    with pytest.raises(RuntimeError): save(path,dict(approved=True))
    assert path.read_bytes()==before
    alias=tmp_path/"alias.json"
    alias.symlink_to(path)
    with pytest.raises(RuntimeError): save(alias,{})
    assert alias.is_symlink() and path.read_bytes()==before
    with pytest.raises(RuntimeError): save(Path("relative_receipt.json"),{})


@pytest.fixture
def synthetic_completed_archive(tmp_path,monkeypatch):
    # HH_261004 - Real temporary bytes exercise original hashing; fake decode operations verify call order, never claim real media.
    real=m.p
    run=tmp_path/"validation/full209_v2";run.mkdir(parents=True)
    output=tmp_path/"gallery/12_town03_full_route209_v2"
    policy=tmp_path/"plans/full209_v2/ROOT_prospective_publication_policy.json";policy.parent.mkdir(parents=True)
    plan_path=run.with_name(run.name+"_plan.json")
    publisher=tmp_path/"tools/synthetic_publisher.py";publisher.parent.mkdir();publisher.write_bytes(b"# synthetic CPU publisher\n")
    test=tmp_path/"tests/synthetic_test.py";test.parent.mkdir();test.write_bytes(b"# synthetic CPU test\n")
    plan_bytes=b'{"synthetic_fixture_only":true,"approval":false}\n'
    plan_path.write_bytes(plan_bytes);(run/"declaration.json").write_bytes(plan_bytes)
    policy.write_bytes(b'{"synthetic_fixture_only":true,"created_before_actual_episode":true}\n')
    for name in ("owner_result.json","diagnostic_report.json","failed_tail.bin"):
        (run/name).write_bytes(b"synthetic CPU metadata or failed-tail bytes only\n")
    calls=[]
    states=[dict(frame=200+index,timestamp=10+index*.05) for index in range(35)]
    cameras=[dict(frame=row["frame"],timestamp=row["timestamp"]) for row in states[::2]]
    bundle=dict(states=states,cameras=cameras,diagnostic=dict(status="FAIL_SYNTHETIC_CPU_ONLY"))
    inventory=real.previous.previous.inventory
    digest=real.prior.digest
    class Pins:
        def recheck(self): calls.append("all_original_pins_rechecked")
        values={"synthetic_external_pin":"0"*64}
    def load_archive(path,sha,*,decode=False):
        calls.append(dict(path=path,sha=sha,decode=decode))
        caller=json.loads(path.read_bytes())
        return caller,bundle,Pins()
    fake=SimpleNamespace(ROOT=tmp_path,RUN=run,OUTPUT=output,POLICY_PATH=policy,__file__=str(publisher),
        require=real.require,prior=real.prior,CALLER_SCHEMA=real.CALLER_SCHEMA,
        previous=SimpleNamespace(previous=SimpleNamespace(inventory=inventory)),
        publication_sources=lambda:[publisher],load_archive=load_archive,
        camera_indices=real.camera_indices,png_indices=real.png_indices,playback=real.playback)
    monkeypatch.setattr(m,"p",fake);monkeypatch.setattr(m,"verify_sources",lambda:None)
    for name,value in dict(PLAN_SHA=digest(plan_path),POLICY_SHA=digest(policy),PUBLISHER_SHA=digest(publisher),
            TEST=test,TEST_SHA=digest(test),CALLER=policy.parent/"ROOT_actual_publication_caller.json",
            POSTCHECK=tmp_path/"diagnostics/v2_postcheck/ROOT_actual_whole_run_and_media.json").items():
        monkeypatch.setattr(m,name,value)
    return dict(real=real,fake=fake,run=run,output=output,calls=calls,bundle=bundle,digest=digest)


def test_original_caller_pins_every_failed_tail_source_and_font_create_only(synthetic_completed_archive,capsys):
    data=synthetic_completed_archive
    assert m.main(["--mode","caller"]) is None
    caller=json.loads(m.CALLER.read_bytes())
    result=json.loads(capsys.readouterr().out)
    assert caller["schema"]==m.p.CALLER_SCHEMA and caller["run_root"]==str(data["run"])
    assert set(caller["run_files_sha256"])=={"declaration.json","owner_result.json","diagnostic_report.json","failed_tail.bin"}
    assert caller["run_files_sha256"]["declaration.json"]==m.PLAN_SHA
    assert caller["publication_source_sha256"]=={"tools/synthetic_publisher.py":m.PUBLISHER_SHA}
    assert len(caller["font_sha256"])==2 and result["raw_file_count"]==4
    assert caller["prospective_publication_policy"]["sha256"]==m.POLICY_SHA
    assert caller["formal_function_approval"] is False and caller["independent_validation"] is False
    assert data["calls"]==[] and not data["output"].exists()
    before=m.CALLER.read_bytes()
    with pytest.raises(RuntimeError): m.main(["--mode","caller"])
    assert m.CALLER.read_bytes()==before


@pytest.mark.parametrize("fault",["owner_unfinished","diagnostic_unfinished","plan_tamper","policy_tamper","gallery_exists"])
def test_caller_refuses_unfinished_or_changed_actual_inputs(synthetic_completed_archive,fault):
    data=synthetic_completed_archive
    if fault=="owner_unfinished": (data["run"]/"owner_result.json").unlink()
    elif fault=="diagnostic_unfinished": (data["run"]/"diagnostic_report.json").unlink()
    elif fault=="plan_tamper": data["run"].with_name(data["run"].name+"_plan.json").write_bytes(b"changed")
    elif fault=="policy_tamper": m.p.POLICY_PATH.write_bytes(b"changed")
    else: data["output"].mkdir(parents=True)
    with pytest.raises(RuntimeError): m.main(["--mode","caller"])
    assert not m.CALLER.exists() and data["calls"]==[]


def _synthetic_gallery(data,monkeypatch):
    m.main(["--mode","caller"])
    output=data["output"];output.mkdir(parents=True)
    bundle=data["bundle"]
    indices=m.p.camera_indices(len(bundle["cameras"]))
    durations=m.p.playback(bundle,indices)
    pngs=m.p.png_indices(len(bundle["cameras"]))
    media={}
    for name in ["full_actual_six_cameras_2hz.gif",*pngs]:
        path=output/name;path.write_bytes(b"synthetic CPU fake media, not a real image\n")
        media[name]=data["digest"](path)
    value=dict(caller_sha256=data["digest"](m.CALLER),failed_tail_cropped=False,GIF_camera_indices=indices,
        GIF_duration_ms=durations,PNG_camera_indices=pngs,GIF_total_duration_ms=sum(durations),media_sha256=media)
    (output/"publication_provenance.json").write_bytes(json.dumps(value).encode())
    class Saved:
        def __init__(self,path):
            self.path=path;self.n_frames=len(indices);self.info={"duration":durations[0]}
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def seek(self,index): self.info["duration"]=durations[index];data["calls"].append(("seek",index))
        def load(self): data["calls"].append(("load",self.path.name))
    base=m._namespace
    monkeypatch.setattr(m,"_namespace",lambda:{**base(),"Image":SimpleNamespace(open=Saved)})
    return value,indices,durations,pngs


def test_postcheck_keeps_original_all_raw_JPEG_media_recheck_and_new_identity(synthetic_completed_archive,monkeypatch,capsys):
    data=synthetic_completed_archive
    value,indices,durations,pngs=_synthetic_gallery(data,monkeypatch)
    capsys.readouterr()
    assert m.main(["--mode","postcheck"]) is None
    result=json.loads(capsys.readouterr().out)
    receipt=json.loads(m.POSTCHECK.read_bytes())
    assert receipt["schema"]==m.POSTCHECK_SCHEMA and receipt["original_status"]=="FAIL_SYNTHETIC_CPU_ONLY"
    assert receipt["native_state_count"]==35 and receipt["camera_bundle_count"]==18
    assert receipt["original_JPEG_count"]==108 and receipt["media_count"]==6
    assert receipt["GIF_frame_count"]==len(indices) and receipt["GIF_duration_ms"]==sum(durations)
    assert receipt["failed_tail_cropped"] is False and receipt["full_RTE01_approved"] is False
    assert receipt["learned_STOP_claim"] is False and receipt["formal_function_approval"] is False
    assert result["postcheck_sha256"]==data["digest"](m.POSTCHECK)
    assert data["calls"][0]["decode"] is True
    assert sum(row=="all_original_pins_rechecked" for row in data["calls"])==2
    assert sum(isinstance(row,tuple) and row[0]=="seek" for row in data["calls"])==len(indices)
    assert sum(isinstance(row,tuple) and row[0]=="load" for row in data["calls"])==len(indices)+5


@pytest.mark.parametrize("fault",["caller","crop","GIF_indices","PNG_indices","duration","media_members","media_hash"])
def test_original_postcheck_guards_refuse_changed_media_or_failed_tail(synthetic_completed_archive,monkeypatch,fault):
    data=synthetic_completed_archive
    value,indices,durations,pngs=_synthetic_gallery(data,monkeypatch)
    if fault=="caller": value["caller_sha256"]="0"*64
    elif fault=="crop": value["failed_tail_cropped"]=True
    elif fault=="GIF_indices": value["GIF_camera_indices"]=indices[:-1]
    elif fault=="PNG_indices": value["PNG_camera_indices"]["05_last.png"]=0
    elif fault=="duration": value["GIF_duration_ms"][0]+=100
    elif fault=="media_members": value["media_sha256"].pop("05_last.png")
    else: value["media_sha256"]["05_last.png"]="0"*64
    (data["output"]/"publication_provenance.json").write_bytes(json.dumps(value).encode())
    with pytest.raises(RuntimeError): m.main(["--mode","postcheck"])
    assert not m.POSTCHECK.exists()
