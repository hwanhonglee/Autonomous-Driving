"""HH_261002 - Synthetic remove-only guard fixtures; never inspect/delete real /tmp, use SSH, or change frozen backup history."""
from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import remove_tmp_carla_contract_strict_nlink1_20261002 as tool


@pytest.fixture
def census(monkeypatch):
    target="/synthetic/one";key="one/file";identity=[1,2,33188,1000,1000,4,10,11,1]
    expected=dict(entries={key:dict(kind="file",sha256="a"*64,hardlink_first=key)},identities={key:identity})
    actual=copy.deepcopy(expected)
    monkeypatch.setattr(tool,"REGULAR_COUNT",1);monkeypatch.setattr(tool.parent,"TARGETS",(target,))
    monkeypatch.setattr(tool.parent,"census",lambda paths,local:actual)
    return SimpleNamespace(target=target,key=key,expected=expected,actual=actual)


def test_complete_stamp_passes_without_reinterpreting_history(census):
    assert tool.strict_verify_single_target(census.target,census.expected) is census.actual


@pytest.mark.parametrize("index",range(9))
def test_every_generation_field_strictly_retained(census,index):
    census.actual["identities"][census.key][index]+=1
    with pytest.raises(ValueError,match="STRICT full"):tool.strict_verify_single_target(census.target,census.expected)


def test_same_bytes_inode_mtime_but_ctime_changed_refused(census):
    census.actual["identities"][census.key][7]+=5
    with pytest.raises(ValueError,match="ctime/nlink"):tool.strict_verify_single_target(census.target,census.expected)


def test_external_inventory_hardlink_refused(census):
    census.expected["identities"][census.key][-1]=2
    with pytest.raises(ValueError,match="hardlinks are unsupported"):tool.strict_verify_single_target(census.target,census.expected)


def test_full_count_and_exact_target_required(census,monkeypatch):
    monkeypatch.setattr(tool,"REGULAR_COUNT",2)
    with pytest.raises(ValueError,match="regular files required"):tool.strict_verify_single_target(census.target,census.expected)
    with pytest.raises(ValueError,match="original exact"):tool.strict_verify_single_target("/foreign",census.expected)


@pytest.mark.parametrize("mutation",["sha","extra_file","missing_file"])
def test_complete_payload_no_subset(census,mutation):
    if mutation=="sha":census.actual["entries"][census.key]["sha256"]="b"*64
    elif mutation=="extra_file":census.actual["entries"]["one/new"]={}
    else:census.actual["entries"].pop(census.key)
    with pytest.raises(ValueError):tool.strict_verify_single_target(census.target,census.expected)


@pytest.fixture
def authorized(monkeypatch):
    pins={"new_wrapper.py":"c"*64,"new_test.py":"d"*64}
    authority=dict(schema=tool.parent.AUTH_SCHEMA,mode="remove",remove_revision_schema=tool.SCHEMA,
        remove_revision_authorized=True,remove_revision_source_sha256=pins,remove_revision_regular_file_count=25096,
        remove_revision_nlink_must_equal_one=True)
    auth=Path("/synthetic/authority.json");auth_sha="e"*64;own_sha="f"*64
    def digest(path):
        if str(path)==tool.parent.__file__:return tool.PARENT_SHA
        if Path(path)==tool.parent.TEST:return tool.PARENT_TEST_SHA
        return auth_sha if Path(path)==auth else own_sha
    monkeypatch.setattr(tool.parent,"digest",digest);monkeypatch.setattr(tool.parent,"read",lambda *args:copy.deepcopy(authority))
    monkeypatch.setattr(tool,"sources",lambda:copy.deepcopy(pins))
    return SimpleNamespace(authority=authority,pins=pins,args=["--expected-tool-sha256",own_sha,"--authorization",str(auth),"--authorization-sha256",auth_sha])


def test_original_main_mode_source_and_backup_history_preserved(authorized,monkeypatch):
    original=tool.parent.verify_single_target;calls=[]
    def main(args):
        calls.append(args);assert tool.parent.verify_single_target is not original;return 0
    monkeypatch.setattr(tool,"_main",main)
    assert tool.main(authorized.args)==0
    assert calls==[["--mode","remove","--expected-tool-sha256",tool.PARENT_SHA,"--authorization","/synthetic/authority.json","--authorization-sha256","e"*64]]
    assert tool.parent.verify_single_target is original and tool._active is False


@pytest.mark.parametrize("field,value",[("mode","backup"),("remove_revision_authorized",False),
    ("remove_revision_authorized",1),("remove_revision_schema","unknown"),("remove_revision_source_sha256",{}),
    ("remove_revision_regular_file_count",25096.),("remove_revision_nlink_must_equal_one",False)])
def test_separate_revision_authority_required_before_oldmain(authorized,monkeypatch,field,value):
    authorized.authority[field]=value
    monkeypatch.setattr(tool,"_main",lambda _:pytest.fail("unauthorized old main call"))
    with pytest.raises(ValueError):tool.main(authorized.args)


def test_scope_restores_even_partial_failure(authorized,monkeypatch):
    original=tool.parent.verify_single_target
    def fail(_):raise OSError("synthetic preserved failure")
    monkeypatch.setattr(tool,"_main",fail)
    with pytest.raises(OSError):tool.main(authorized.args)
    assert tool.parent.verify_single_target is original and tool._active is False


def test_unknown_binding_or_nesting_rejected(authorized,monkeypatch):
    monkeypatch.setattr(tool.parent,"verify_single_target",lambda *_:None)
    with pytest.raises(ValueError,match="non-nested"):tool.main(authorized.args)


def test_cli_only_remove_without_backup_or_inspect_mode():
    for mode in ("backup","inspect"):
        with pytest.raises(SystemExit):tool.main(["--mode",mode])
