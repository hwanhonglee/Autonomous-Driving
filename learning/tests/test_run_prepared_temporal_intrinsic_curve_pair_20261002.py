"""HH_261002 - Synthetic stdlib dispatch receipts only; no SSH, CUDA, model, or real datasets."""
from __future__ import annotations
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import shlex
from types import SimpleNamespace
import pytest

SOURCE=Path(__file__).resolve().parents[2]/"tools/e2e/run_prepared_temporal_intrinsic_curve_pair_20261002.py"
spec=importlib.util.spec_from_file_location("prepared_intrinsic_actual_dispatch_test",SOURCE)
dispatch=importlib.util.module_from_spec(spec);spec.loader.exec_module(dispatch)


def sha(payload):return hashlib.sha256(payload).hexdigest()


@pytest.fixture
def prepared(tmp_path,monkeypatch):
    monkeypatch.setattr(dispatch,"ROOT",tmp_path)
    directory=tmp_path/"storage/experiments/training/2026-10-02/prepared"
    directory.mkdir(parents=True);monkeypatch.setattr(dispatch,"PREPARED",directory)
    source_names=["tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py",
        "learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py"]+[f"learning/fixture_{i}.py" for i in range(26)]
    pins={}
    for i,name in enumerate(source_names):
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);payload=f"# synthetic source {i}\n".encode()
        path.write_bytes(payload);pins[name]=sha(payload)
    monkeypatch.setattr(dispatch,"RUNNER_SHA",pins[source_names[0]]);monkeypatch.setattr(dispatch,"LOSS_SHA",pins[source_names[1]])
    authority="a"*64;command=shlex.join(dispatch.expected_command(authority))
    document={"source_sha256":pins,"authorization_sha256":authority,"dispatch_source_sha256":dispatch.PREPARE_SHA,
        "parent_actual6000_history_sha256":dispatch.HISTORY_SHA,"parent_four_actual_sha256":dispatch.PARENT_PINS,
        "physical_GPU_allowed":dispatch.GPU,"existing_private_venv":str(dispatch.REMOTE/"venvs/py312/bin/python"),
        "optimizer_started":False,"packages_installed":False,"remote_source":str(dispatch.SOURCE),"remote_run":str(dispatch.RUN),
        "proposed_fit_command_requires_separate_root_execution":command}
    raw=b'{"status":"VALIDATED_TRAIN_ONLY_CPU_NO_CUDA_NO_OUTPUT"}\n';(directory/"remote_validation_stdout.json").write_bytes(raw)
    summary={"status":"PRIVATE_REMOTE_TRAIN_ONLY_CPU_PREFLIGHT_PASS_NO_FIT","optimizer_started":False,
        "remote_validation_sha256":sha(raw),"proposed_fit_command_requires_separate_root_execution":command}
    calls=[]
    def run(argv,**kwargs):
        calls.append(("preflight",argv,kwargs));return SimpleNamespace(returncode=0,stdout=b'{"status":"PASS"}\n',stderr=b"")
    class Popen:
        def __init__(self,argv,stdout,stderr):
            calls.append(("fit",argv));self.pid=12345;stdout.write(b'{"started":true}\n');stderr.write(b"raw SSH diagnostic\n")
        def wait(self):return 0
    monkeypatch.setattr(dispatch.subprocess,"run",run);monkeypatch.setattr(dispatch.subprocess,"Popen",Popen)
    def seal():
        for name,value in (("declaration.json",document),("summary.json",summary)):
            (directory/name).write_text(json.dumps(value,sort_keys=True))
        return ["--mode","fit","--expected-prepared-declaration-sha256",sha((directory/"declaration.json").read_bytes()),
            "--expected-prepared-summary-sha256",sha((directory/"summary.json").read_bytes()),
            "--expected-authority-sha256",authority,"--output",str(tmp_path/"storage/experiments/training/2026-10-02/actual")]
    return SimpleNamespace(directory=directory,document=document,summary=summary,calls=calls,seal=seal,pins=pins)


def test_success_exact_command_and_exclusive_raw_receipts(prepared):
    argv=prepared.seal();assert dispatch.main(argv)==0
    assert [row[0] for row in prepared.calls]==["preflight","fit"]
    assert prepared.calls[1][1]==dispatch.SSH+[prepared.document["proposed_fit_command_requires_separate_root_execution"]]
    preflight=shlex.split(prepared.calls[0][1][-1]);assert preflight[:11]==[
        "env","-u","PYTHONPATH","-u","PYTHONHOME","-u","PYTHONUSERBASE","-u","PYTHONSTARTUP","PYTHONNOUSERSITE=1","python3"]
    output=Path(argv[-1]);assert (output/"actual_fit_stdout.jsonl").read_bytes()==b'{"started":true}\n'
    assert (output/"actual_fit_stderr.txt").read_bytes()==b"raw SSH diagnostic\n"
    assert json.loads((output/"started.json").read_bytes())["SSH_PID"]==12345
    assert json.loads((output/"completion.json").read_bytes())["exit_code"]==0
    assert json.loads((output/"source_postcheck.json").read_bytes())["model_promoted"] is False
    before={p.name:p.read_bytes() for p in output.iterdir() if p.is_file()}
    with pytest.raises(ValueError,match="fresh local"):dispatch.main(argv)
    assert before=={p.name:p.read_bytes() for p in output.iterdir() if p.is_file()}


@pytest.mark.parametrize("role,value",[("physical_GPU_allowed","GPU-other"),("existing_private_venv","/usr/bin/python3"),
    ("optimizer_started",True),("packages_installed",True),("parent_actual6000_history_sha256","b"*64),
    ("parent_four_actual_sha256",{}),("remote_run","/tmp/other")])
def test_wrong_preparation_rejected_without_launch(prepared,role,value):
    prepared.document[role]=value
    with pytest.raises(ValueError):dispatch.main(prepared.seal())
    assert prepared.calls==[]


@pytest.mark.parametrize("replacement",["CUDA_VISIBLE_DEVICES=1","CUDA_VISIBLE_DEVICES=GPU-other","CUDA_VISIBLE_DEVICES=0,1"])
def test_command_gpu_substitution_rejected(prepared,replacement):
    command=prepared.document["proposed_fit_command_requires_separate_root_execution"].replace("CUDA_VISIBLE_DEVICES="+dispatch.GPU,replacement)
    prepared.document["proposed_fit_command_requires_separate_root_execution"]=command
    prepared.summary["proposed_fit_command_requires_separate_root_execution"]=command
    with pytest.raises(ValueError,match="arbitrary command"):dispatch.main(prepared.seal())
    assert prepared.calls==[]


def test_preflight_fail_retains_raw_no_fit(prepared,monkeypatch):
    monkeypatch.setattr(dispatch.subprocess,"run",lambda *a,**k:SimpleNamespace(returncode=1,stdout=b"GPU not idle",stderr=b"original failure"))
    argv=prepared.seal()
    with pytest.raises(ValueError,match="no fit launched"):dispatch.main(argv)
    output=Path(argv[-1]);assert (output/"preflight_stdout.json").read_bytes()==b"GPU not idle"
    assert (output/"preflight_stderr.txt").read_bytes()==b"original failure"
    assert not (output/"actual_fit_stdout.jsonl").exists()
    assert json.loads((output/"failure_receipt.json").read_bytes())["foreign_processes_killed"] is False


def test_prelaunch_local_mutation_prevents_fit(prepared,monkeypatch):
    original=dispatch.subprocess.run
    def mutate(*args,**kwargs):
        result=original(*args,**kwargs);(dispatch.ROOT/next(iter(prepared.pins))).write_bytes(b"changed")
        return result
    monkeypatch.setattr(dispatch.subprocess,"run",mutate)
    with pytest.raises(ValueError,match="changed"):dispatch.main(prepared.seal())
    assert [row[0] for row in prepared.calls]==["preflight"]


def test_failed_ssh_keeps_exit_and_raw_without_success(prepared,monkeypatch):
    class Failed:
        def __init__(self,argv,stdout,stderr):self.pid=456;stdout.write(b"partial training");stderr.write(b"actual traceback")
        def wait(self):return 7
    monkeypatch.setattr(dispatch.subprocess,"Popen",Failed);argv=prepared.seal()
    with pytest.raises(ValueError,match="partial remote run retained"):dispatch.main(argv)
    output=Path(argv[-1]);assert json.loads((output/"completion.json").read_bytes())["exit_code"]==7
    assert (output/"actual_fit_stderr.txt").read_bytes()==b"actual traceback"
    assert not (output/"source_postcheck.json").exists()


def test_output_cannot_mutate_frozen_preparation(prepared):
    argv=prepared.seal();argv[-1]=str(prepared.directory/"new_child")
    with pytest.raises(ValueError,match="outside frozen preparation"):dispatch.main(argv)
    assert prepared.calls==[]


def test_ancestor_symlink_rejected(prepared,tmp_path):
    linked=tmp_path/"link";linked.symlink_to(prepared.directory,target_is_directory=True)
    with pytest.raises(ValueError,match="canonical"):dispatch.digest(linked/"remote_validation_stdout.json")


@pytest.mark.parametrize("payload",[b'{"duplicate":1,"duplicate":2}',b'{"bad":NaN}',b'{"bad":Infinity}'])
def test_malformed_json_fails_closed(tmp_path,payload):
    file=tmp_path/"input.json";file.write_bytes(payload)
    with pytest.raises(ValueError):dispatch.read_pinned(file,sha(payload))


def test_fixed_protocol_no_other_gpu_install_pickle_or_foreign_kill():
    command=dispatch.expected_command("a"*64)
    assert "CUDA_VISIBLE_DEVICES="+dispatch.GPU in command and command.count("fit")==1
    assert command[command.index("--parent-history-sha256")+1]==dispatch.HISTORY_SHA
    assert not any("pip" in item or "pickle" in item or item in ("kill","sudo") for item in command)
    assert '"nvidia-smi","-i","0"' in dispatch.REMOTE_PREFLIGHT
    assert not any(name in dispatch.REMOTE_PREFLIGHT for name in ("torch","terminate","kill("))


def idle_fixture(gpu_output,process_output):
    # HH_261002 - Compile ONLY the exact pure query predicate from the remote preflight; never execute its paths or SSH.
    syntax=ast.parse(dispatch.REMOTE_PREFLIGHT)
    function=next(node for node in syntax.body if isinstance(node,ast.FunctionDef) and node.name=="gpu0_idle_read_only")
    calls=[]
    def query(argv,**kwargs):
        calls.append(argv);assert argv[:3]==["nvidia-smi","-i","0"]
        return process_output if "--query-compute-apps=gpu_uuid,pid" in argv else gpu_output
    namespace={"subprocess":SimpleNamespace(check_output=query)}
    exec(compile(ast.Module(body=[function],type_ignores=[]),"<GPU0-query-only-fixture>","exec"),namespace)
    return namespace["gpu0_idle_read_only"],calls


@pytest.mark.parametrize("memory",[0,1,16])
def test_only_small_driver_residual_without_process_is_idle(memory):
    predicate,calls=idle_fixture(f"{dispatch.GPU}, {memory}, 0\n","")
    result=predicate();assert result["memory_used_MiB"]==memory and result["gpu0_compute_process_count"]==0
    assert len(calls)==2 and result["maximum_driver_residual_MiB"]==16


@pytest.mark.parametrize("gpu,processes",[
    ("GPU-other, 0, 0\n",""),(f"{dispatch.GPU}, 17, 0\n",""),(f"{dispatch.GPU}, -1, 0\n",""),
    (f"{dispatch.GPU}, 1, 1\n",""),(f"{dispatch.GPU}, 1, 0\n",f"{dispatch.GPU}, 333\n"),
    (f"{dispatch.GPU}, 0, 0\n",f"{dispatch.GPU}, 333\n"),(f"{dispatch.GPU}, 1, 0\n","N/A\n"),
    (f"{dispatch.GPU}, N/A, 0\n",""),(f"{dispatch.GPU}, 1, 0\nGPU-other, 0, 0\n","")])
def test_competing_process_unknown_or_active_gpu_is_never_ignored(gpu,processes):
    predicate,_=idle_fixture(gpu,processes)
    with pytest.raises((AssertionError,ValueError)):predicate()
