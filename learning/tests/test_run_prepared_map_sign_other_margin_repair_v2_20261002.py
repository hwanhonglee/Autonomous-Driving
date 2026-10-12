"""HH_261002 - Reproduce actual flat12/nested validator authority semantics without TRAIN arrays, model, CUDA, or remote."""
import ast
import copy
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest
from scripts.e2e import run_prepared_map_sign_other_margin_repair_v2_20261002 as new


@pytest.mark.parametrize("bad",[False,True])
def test_REAL_flat12_original_nested_authority_validator(tmp_path,bad):
    # HH_261002 - The actual import style is flat, not the canonical workspace adapter that shallow mocks can conceal.
    from scripts.e2e import prepare_map_sign_other_margin_repair_20261002 as preparation
    sources=preparation.contract()["source_sha256"]
    for name,row in sources.items():(tmp_path/name).write_bytes(Path(row["local_path"]).read_bytes())
    program=r'''
import json,pathlib,sys
sys.path.insert(0,sys.argv[1])
import train_map_sign_other_margin_repair_20261002 as runner
pins={p.name:__import__("hashlib").sha256(p.read_bytes()).hexdigest() for p in pathlib.Path(sys.argv[1]).iterdir()}
value=runner.prepared_contract()
authority=dict(schema=runner.AUTHORIZATION_SCHEMA,source_sha256=pins,contract=value,
    research_fit_authorized=True,native_denials_preserved=True,DEV_or_HOLDOUT_used_for_fit=False,
    automatic_model_promotion=False,vehicle_control_approved=False,reviewer="synthetic ROOT",
    equal_global_proof_sha256="a"*64,loss_contract_sha256=runner.document_sha(value["loss_contract"]))
keys=("python_executable","numpy","numpy_native","PIL.Image","PIL._imaging","torch","torch._C","torch.lib/example.so")
authority["runtime_identity"]={k:"synthetic" for k in ("python_version","python_implementation","python_prefix","numpy","pillow","torch","torch_compiled_CUDA")}
authority["runtime_identity"]["provider_files"]={k:dict(path="/synthetic/"+k,bytes=1,sha256="b"*64) for k in keys}
before=(runner.previous.SCHEMA,runner.previous.previous.SCHEMA)
if sys.argv[2]=="bad":
    authority["contract"]=json.loads(json.dumps(value));authority["contract"]["gradient_norm_cap"]=5
else:
    assert type(authority["contract"]["gradient_norm_cap"]) is float
with runner.activated():
    with runner.previous.activated():
        assert runner.previous.previous.AUTHORIZATION_SCHEMA==runner.AUTHORIZATION_SCHEMA
        assert runner.previous.previous.prepared_contract()==authority["contract"]
        assert runner.previous._old_auth.__globals__ is vars(runner.previous.previous)
        try:
            runner.previous.validate_authorization(authority,pins)
            status="PASS"
        except ValueError as error:
            status="REJECTED";assert "exact prospective" in str(error)
assert before==(runner.previous.SCHEMA,runner.previous.previous.SCHEMA)
assert "torch" not in sys.modules
print(json.dumps(dict(status=status,source_count=len(pins),CUDA_or_model_imported=False)))
'''
    result=subprocess.run([sys.executable,"-B","-c",program,str(tmp_path),"bad" if bad else "good"],
        capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    value=json.loads(result.stdout)
    assert value["source_count"]==12 and value["status"]==("REJECTED" if bad else "PASS")
    assert not value["CUDA_or_model_imported"]


@pytest.mark.parametrize("bad",[False,True])
def test_v2_exact_JSON_guard_rejects_int5_even_when_python_dicts_equal(monkeypatch,bad):
    contract={"gradient_norm_cap":5.0};auth={"contract":copy.deepcopy(contract)}
    if bad:auth["contract"]["gradient_norm_cap"]=5
    assert auth["contract"]==contract
    monkeypatch.setattr(new,"_identity",lambda args:({"fit_contract":contract},auth,{}))
    if bad:
        with pytest.raises(ValueError,match="numeric representation"):new.identities(SimpleNamespace())
    else:assert new.identities(SimpleNamespace())[1] is auth


def test_exact_authority_path_only_remote_AST_and_all_scope_restore():
    parent=new.previous
    before=parent.OUTPUT,parent.REMOTE_AUTH,parent.SCHEMA,parent.REMOTE_CHECK,parent.identities,parent.write_new
    with new.activated():
        assert parent.OUTPUT==new.OUTPUT and parent.REMOTE_AUTH.name=="authorization_v2.json"
        revised=ast.parse(parent.REMOTE_CHECK);old=ast.parse(new._old_check)
        for node in ast.walk(revised):
            if isinstance(node,ast.Constant) and node.value==str(new.REMOTE_AUTH.relative_to(parent.WORKSPACE)):
                node.value=str(new._old_auth.relative_to(parent.WORKSPACE))
        assert ast.dump(revised,include_attributes=False)==ast.dump(old,include_attributes=False)
        with pytest.raises(ValueError,match="nested"):
            with new.activated():pass
    assert before==(parent.OUTPUT,parent.REMOTE_AUTH,parent.SCHEMA,parent.REMOTE_CHECK,parent.identities,parent.write_new)
    with pytest.raises(RuntimeError):
        with new.activated():raise RuntimeError("synthetic failure")
    assert before==(parent.OUTPUT,parent.REMOTE_AUTH,parent.SCHEMA,parent.REMOTE_CHECK,parent.identities,parent.write_new)
    assert not new._active


def test_unknown_parent_and_alias_fails_closed(monkeypatch):
    monkeypatch.setattr(new.previous,"identities",lambda args:None)
    with pytest.raises(ValueError,match="unknown"):
        with new.activated():pass


def test_REAL_parent_main_mock_transport_fresh_v2_authority_and_failure_v1_preserved(tmp_path,monkeypatch):
    from test_run_prepared_map_sign_other_margin_repair_20261002 import fixture,arguments,fake_processes
    args,value,auth,pins=fixture(tmp_path,monkeypatch)
    # HH_261002 - Keep the parent alias identities original; only the NEW authorized output is a synthetic test path.
    monkeypatch.setattr(new.previous,"OUTPUT",new._old_output)
    destination=tmp_path/"new_v2";monkeypatch.setattr(new,"OUTPUT",destination)
    failed=tmp_path/"retained_v1_failure.json";failed.write_text('{"status":"FAILED_RETAINED"}');before=failed.read_bytes()
    calls=fake_processes(monkeypatch,args,pins)
    assert new.main(arguments(args))==0
    assert str(new.REMOTE_AUTH) in calls[1][-1] and "authorization_v2.json" in calls[3][-1] and "authorization_v2.json" in calls[4][-1]
    receipt=json.loads((destination/"declaration.json").read_text())
    assert receipt["schema"]==new.SCHEMA and receipt["dispatch_revision_only"]
    assert receipt["frozen_parent_dispatcher_sha256"]==new.PARENT_SHA and receipt["TRAIN_source_count_unchanged"]==12
    assert (destination/"compatibility_source"/Path(new.__file__).name).is_file() and failed.read_bytes()==before
    assert new.previous.OUTPUT is new._old_output and new.previous.REMOTE_AUTH is new._old_auth
