"""HH_261002 - Seal only synthetic CPU command receipts; never remote execution, GPU, arrays, or optimization."""
import json
from pathlib import Path

import pytest
from scripts.e2e import prepare_map_sign_other_margin_repair_20261002 as prepare


def test_current_source_contract_TEN_frozen_plus_TWO_new_no_execution():
    value=prepare.contract()
    assert len(value["source_sha256"])==12 and not value["fit_authorized"] and not value["remote_command_executed"]
    assert value["CPU_prepare_only"] and value["only_physical_GPU0_allowed"] and value["all_commands_not_executed"]
    assert value["required_environment"]["CUDA_VISIBLE_DEVICES"].startswith("GPU-")
    assert value["required_environment"]["OMP_NUM_THREADS"]=="16"
    assert [c["stage"] for c in value["commands"]]==["runtime_identity","schedule_and_source_identity","TRAIN_only_CPU_preflight"]
    assert all(not c["optimizer_allowed"] and not c["CUDA_initialization_allowed"] for c in value["commands"])
    assert "--validate-only" in value["commands"][-1]["argv"]
    assert value["fit_contract"]["all_existing_twenty_plus_seven_screens_unchanged"]


def authority(tmp_path,monkeypatch,*,fault=None):
    monkeypatch.setattr(prepare,"PLAN",tmp_path)
    value=dict(schema=prepare.SCHEMA,source_sha256={"synthetic":"a"*64},fit_authorized=False)
    monkeypatch.setattr(prepare,"contract",lambda:value)
    auth=dict(schema=prepare.AUTH_SCHEMA,CPU_preparation_contract=value,CPU_preparation_authorized=True,
        fit_authorized=False,remote_execution_authorized=False)
    if fault=="fit":auth["fit_authorized"]=True
    if fault=="remote":auth["remote_execution_authorized"]=True
    if fault=="CPU":auth["CPU_preparation_authorized"]=False
    if fault=="contract":auth["CPU_preparation_contract"]={}
    path=tmp_path/"authority.json";path.write_text(json.dumps(auth))
    pin=prepare.digest(path)
    if fault=="SHA":pin="0"*64
    if fault=="duplicate":path.write_text('{"schema":"x","schema":"x"}');pin=prepare.digest(path)
    return path,pin


def test_source_seal_create_only_does_not_dispatch(tmp_path,monkeypatch,capsys):
    path,pin=authority(tmp_path,monkeypatch)
    assert prepare.main(["--mode","seal","--authority",str(path),"--authority-sha256",pin])==0
    receipt=json.loads((tmp_path/"prepared_CPU_command_contract.json").read_text())
    assert receipt["external_CPU_preparation_authority_sha256"]==pin and not receipt["fit_authorized"]
    result=json.loads(capsys.readouterr().out);assert not result["remote_executed"]
    original=(tmp_path/"prepared_CPU_command_contract.json").read_bytes()
    with pytest.raises(Exception,match="overwrite"):
        prepare.main(["--mode","seal","--authority",str(path),"--authority-sha256",pin])
    assert (tmp_path/"prepared_CPU_command_contract.json").read_bytes()==original


@pytest.mark.parametrize("fault",["fit","remote","CPU","contract","SHA","duplicate"])
def test_no_unknown_authority_or_expansive_permission(tmp_path,monkeypatch,fault):
    path,pin=authority(tmp_path,monkeypatch,fault=fault)
    with pytest.raises(Exception):prepare.main(["--mode","seal","--authority",str(path),"--authority-sha256",pin])
    assert not (tmp_path/"prepared_CPU_command_contract.json").exists()


def test_no_default_prepare_permission(tmp_path,monkeypatch):
    monkeypatch.setattr(prepare,"PLAN",tmp_path);monkeypatch.setattr(prepare,"contract",lambda:{})
    with pytest.raises(Exception,match="authority"):prepare.main(["--mode","seal"])
    assert list(tmp_path.iterdir())==[]


def test_actual_dispatcher_has_no_subprocess_network_or_Torch_execution():
    import ast
    tree=ast.parse(Path(prepare.__file__).read_bytes())
    imports=[n.module or "" for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    imports += [alias.name for n in ast.walk(tree) if isinstance(n,ast.Import) for alias in n.names]
    assert not set(imports)&{"subprocess","socket","torch","paramiko"}
