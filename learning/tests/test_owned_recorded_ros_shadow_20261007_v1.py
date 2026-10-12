"""HH_261007 - Pure synthetic protocol tests, NOT actual DDS/model/driving measurements."""
from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[2]


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/path);value=importlib.util.module_from_spec(spec)
    sys.modules[name]=value;spec.loader.exec_module(value);return value


@pytest.fixture
def worker():return module("tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v1.py","_hh_test_protocol_261007")


@pytest.fixture
def owner():return module("tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v1.py","_hh_test_owner_261007")


@pytest.fixture
def node():return module("autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v1.py","_hh_test_node_261007")


def test_sources_import_no_ros_torch_or_actuator(worker,owner,node):
    for name in ("rclpy","torch","carla"):
        assert name not in sys.modules
    assert worker.fixed_contract()["camera_anchors"]==918
    assert worker.fixed_contract()["CPU_threads"]==4
    assert worker.fixed_contract()["policy_deadline_seconds"]==.1
    assert worker.FLOOR==2*1024**3 and worker.QUOTA==128*1024**2
    assert "carla" not in " ".join(worker.worker_command("direct","a"*64,"b"*64)).lower()


def test_real_original_delegation_exact_ast(worker,node):
    ns=node.compile_original(worker)
    original={item.name:item for item in ast.parse(node.OLD.read_bytes()).body if isinstance(item,ast.FunctionDef)}
    assert set(ns)&set(original)=={"stamp_ns","stamp","dependencies","message_set","load_runtime","build_observed_frame","recorded_node","recorded_publisher"}
    for name in set(ns)&set(original):
        expected=compile(ast.Module(body=[original[name]],type_ignores=[]),str(node.OLD),"exec")
        expected_code=next(code for code in expected.co_consts if hasattr(code,"co_name") and code.co_name==name)
        assert ns[name].__code__.co_code==expected_code.co_code
    assert "json" in ns and "importlib" in ns
    assert worker.frozen_bundle().ACTUAL_ENTRY_REVIEWED is False


def test_worker_argv_full_bootstrap_separator(worker):
    argv=worker.worker_command("publisher","a"*64,"b"*64)
    assert argv[:3]==["/usr/bin/python3","-B",str(ROOT/"tools/workspace_python.py")]
    assert argv[5]=="--" and argv[6:8]==["--mode","publisher"]
    assert argv[-4:]==["--plan-sha256","a"*64,"--authority-sha256","b"*64]


def test_digest_atime_not_generation(worker,tmp_path):
    path=tmp_path/"scalar";path.write_bytes(b"abc")
    os.utime(path,ns=(1,2))
    assert worker.digest(path)==hashlib.sha256(b"abc").hexdigest()
    link=tmp_path/"link";link.symlink_to(path)
    with pytest.raises(ValueError):worker.digest(link)


@pytest.mark.parametrize("change",["starttime","uid","ppid","pgid","session","argv"])
def test_pid_generation_guard_all_fields(worker,monkeypatch,change):
    identity=dict(pid=23,starttime=900,uid=1,ppid=1,pgid=23,session=23,argv=["test"])
    actual=deepcopy(identity);actual[change]=["other"] if change=="argv" else identity[change]+1
    monkeypatch.setattr(worker,"process_identity",lambda pid:actual)
    with pytest.raises(ValueError):worker.same_generation(identity)


def test_raw_uint64_json_preserved(worker,tmp_path):
    value=dict(world=10640938066301165695,ns=1781234567123456789)
    path=tmp_path/"raw.json";worker.new_json(path,value)
    assert worker.bounded_json(path)==value
    assert path.read_bytes().endswith(b"\n")
    with pytest.raises(FileExistsError):worker.new_json(path,value)


@pytest.mark.parametrize("bad",[b'{"a":1,"a":2}',b'{"a":NaN}',b'{"a":Infinity}'])
def test_duplicate_or_nonfinite_json_denied(worker,tmp_path,bad):
    path=tmp_path/"bad.json";path.write_bytes(bad)
    with pytest.raises((ValueError,TypeError)):worker.bounded_json(path)


def test_budget_counts_plan_and_output(worker,tmp_path,monkeypatch):
    plan=tmp_path/"plan";output=tmp_path/"output";plan.mkdir();output.mkdir()
    (plan/"archive").write_bytes(b"123");(output/"logs").write_bytes(b"1234")
    monkeypatch.setattr(worker,"PLAN_DIR",plan);monkeypatch.setattr(worker,"OUTPUT",output)
    monkeypatch.setattr(worker.shutil,"disk_usage",lambda _:SimpleNamespace(free=worker.FLOOR+worker.SHARED+worker.QUOTA))
    assert worker.budget(startup=True)==7
    (output/"symlink").symlink_to(plan/"archive")
    with pytest.raises(ValueError):worker.budget()


def test_source_module_virtual_namespace_guard(worker,monkeypatch):
    view=dict(source_sha256={},generated_ROS_sources={})
    worker.verify_project_modules(view,[SimpleNamespace(__name__="scripts",__file__=None,__path__=[str(ROOT/"tools")])])
    for value in (SimpleNamespace(__name__="scripts",__file__=None,__path__=["/tmp/foreign"]),SimpleNamespace(__name__="portable_e2e",__file__=None)):
        with pytest.raises(ValueError):worker.verify_project_modules(view,[value])


def test_unsealed_autoware_overlay_denied(worker):
    with pytest.raises(ValueError):worker.verify_project_modules(dict(source_sha256={},generated_ROS_sources={}),
        [SimpleNamespace(__name__="autoware_planning_msgs",__file__="/home/a/carla-autoware-universe/autoware/install/foreign.py")])


@pytest.mark.parametrize("raw",[True,None,"0",float("nan"),float("inf"),[1.,float("nan")]])
def test_finite_heads_strict(node,raw):assert not node.finite_nested(raw)


class Array:
    dtype="float32"
    shape=(1,2)
    def tobytes(self):return b"actual synthetic finite fixture bytes"


class Tensor:
    def __init__(self,values):self.values=values
    def detach(self):return self
    def cpu(self):return self
    def contiguous(self):return self
    def numpy(self):return Array()
    def tolist(self):return self.values


class Model:
    def forward_cached(self,inputs,*,window):
        assert window=="real fixture window"
        return {name:Tensor([[1.,2.]]) for name in ("curvature_knots","preferred_speed_knots","candidate_logits","stop_intent_logits","acceleration_knots")}


class Policy:
    def __init__(self):self.model=Model()
    def build_inputs(self,frame):return {"images":Tensor([[1.,2.]])},"real fixture window"


class Adapter:
    def __init__(self):self._policy=Policy();self.calls=0
    def reset(self,*args,**kwargs):return "genuine reset"
    def accept(self,frame,**kwargs):
        tensors,window=self._policy.build_inputs(frame);self._policy.model.forward_cached(tensors,window=window)
        self.calls+=1;return SimpleNamespace(status="PREFILL",diagnostics={})


def test_actual_adapter_delegate_single_call_restore_and_prefill(node,worker):
    adapter=Adapter();audited=node.AuditedAdapter(adapter,worker)
    result=audited.accept("original frame")
    receipt=result.diagnostics["owned_recorded_numerical_event"]
    assert adapter.calls==1 and receipt["NN_executed"] and receipt["tensor_available"]
    assert len(receipt["raw_output"])==5 and receipt["frozen_policy_status"]=="PREFILL"
    assert "forward_cached" not in adapter._policy.model.__dict__
    assert "build_inputs" not in adapter._policy.__dict__
    assert audited.reset("failure")=="genuine reset"


def test_adapter_exception_methods_restored(node,worker):
    adapter=Adapter()
    def fault(*a,**kw):raise RuntimeError("original error")
    adapter._policy.model.forward_cached=fault
    with pytest.raises(RuntimeError):node.AuditedAdapter(adapter,worker).accept("same original frame")
    assert adapter._policy.model.forward_cached is fault
    assert "build_inputs" not in adapter._policy.__dict__


@pytest.mark.parametrize("nodes,topics",[
    ([("foreign","/")],[]), ([("owned","/"),("owned","/")],[]), ([("owned","/foreign")],[]),
    ([("owned","/")],["/control/command/actuation_cmd"]),
])
def test_closed_graph_denies_foreign_or_actuator(node,nodes,topics):
    graph=SimpleNamespace(get_node_names_and_namespaces=lambda:nodes,get_topic_names_and_types=lambda:[(topic,[]) for topic in topics])
    with pytest.raises(ValueError):node.graph_guard(graph,{"/allowed"},{"owned"})


def test_closed_graph_standard_parameter_only(node):
    graph=SimpleNamespace(get_node_names_and_namespaces=lambda:[("owned","/")],get_topic_names_and_types=lambda:[("/parameter_events",[])])
    assert node.graph_guard(graph,set(),{"owned"})["DDS_security_identity_claimed"] is False


@pytest.mark.parametrize("bad",["gid","name","namespace","duplicate","absent"])
def test_actual_gid_denies_unowned(node,bad):
    endpoint=SimpleNamespace(node_name="peer",node_namespace="/",endpoint_gid=b"abc")
    info=SimpleNamespace(publisher_gid=b"abc")
    if bad=="gid":info.publisher_gid=b"xyz"
    if bad=="name":endpoint.node_name="foreign"
    if bad=="namespace":endpoint.node_namespace="/foreign"
    endpoints=[] if bad=="absent" else [endpoint]* (2 if bad=="duplicate" else 1)
    with pytest.raises(ValueError):node.authenticated_gid(SimpleNamespace(get_publishers_info_by_topic=lambda topic:endpoints),"/allowed",info,"peer")


def test_actual_gid_same_endpoint(node):
    endpoint=SimpleNamespace(node_name="peer",node_namespace="/",endpoint_gid=b"abc")
    assert node.authenticated_gid(SimpleNamespace(get_publishers_info_by_topic=lambda topic:[endpoint]),"/allowed",SimpleNamespace(publisher_gid=b"abc"),"peer")=="616263"


def manifest(worker):
    roles=worker.frozen_bundle().ROLES
    yaws=(0,math.pi,math.pi/4,3*math.pi/4,-math.pi/4,-3*math.pi/4)
    return dict(runtime=dict(server_version="synthetic-not-actual"),cameras=[dict(name=role,width=640,height=360,enable_postprocess_effects=False,sensor_tick=0.,frame_id=role+"/camera_optical_link",
        intrinsic=dict(fx=320.,fy=320.,cx=320.,cy=180.),ros_extrinsic=dict(x=1.,y=0.,z=1.6,roll=0.,pitch=0.,yaw=yaw)) for role,yaw in zip(roles,yaws)])


def test_rig_uses_genuine_source_exporter(owner,worker):
    result=owner.genuine_rig(manifest(worker),worker)
    assert result["schema_id"]=="autoware-e2e.common10.rig.v1" and len(result["cameras"])==6
    for camera in result["cameras"]:
        q=owner.quaternion(camera["T_base_from_camera"])
        assert len(q)==4 and abs(sum(v*v for v in q)-1)<1e-12


@pytest.mark.parametrize("angle",[0.,.8,math.pi,-math.pi/2])
def test_quaternion_same_rotation(owner,angle):
    c,s=math.cos(angle),math.sin(angle);matrix=[c,-s,0.,0.,s,c,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.]
    q=owner.quaternion(matrix);assert abs(abs(q[2])-abs(math.sin(angle/2)))<1e-12


def synthetic_rows(count,trajectories=True):
    rows=dict(direct=[],subscriber=[],publisher=[])
    for ordinal in range(count):
        event=dict(tensor_sha256={"images":dict(dtype="float32",shape=[1,2],sha256="a"*64)},NN_executed=True,raw_output={"head":[[1.,2.]]})
        status="TRAJECTORY" if trajectories else "REJECTED"
        common=dict(ordinal=ordinal,native_frame=100+ordinal*2,timestamp_ns=1780000000000000000+ordinal*100000000,status=status)
        hashes={"/planning/portable_e2e/shadow_trajectory":"t"*64,"/planning/portable_e2e/shadow_path":"p"*64} if trajectories else {}
        rows["direct"].append(dict(common,policy_diagnostics=dict(owned_recorded_numerical_event=deepcopy(event)),input_CDR_sha256={"/image":"i"*64},converted_output_CDR_sha256=hashes))
        rows["subscriber"].append(dict(common,policy_diagnostics=dict(owned_recorded_numerical_event=deepcopy(event)),actual_received_input_CDR_sha256={"/image":"i"*64}))
        rows["publisher"].append(dict(common,published_input_CDR_sha256={"/image":"i"*64},actual_DDS_outputs={key:dict(sha256=value,received_ns=13) for key,value in hashes.items()}))
    return rows


def comparison(owner,worker,monkeypatch,tmp_path,rows):
    monkeypatch.setattr(worker,"OUTPUT",tmp_path)
    monkeypatch.setattr(worker,"frozen_bundle",lambda:SimpleNamespace(decode=lambda raw,limit:json.loads(raw)))
    observed=[];monkeypatch.setattr(worker,"append",lambda path,row:observed.append(row))
    for mode,values in rows.items():(tmp_path/(mode+"_anchors.jsonl")).write_bytes(b"".join(worker.encode(value) for value in values))
    return owner.compare(worker),observed


def test_full918_synthetic_comparison_nonvacuous(owner,worker,monkeypatch,tmp_path):
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,synthetic_rows(918))
    assert result["status"]=="PASS_RECORDED_DIRECT_DDS_PARITY_ONLY" and len(rows)==918
    assert result["P3_approved"] is False and result["formal_main_count"]==0


def test_full918_all_deadline_fail_no_positive_not_pass(owner,worker,monkeypatch,tmp_path):
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,synthetic_rows(918,False))
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED"
    assert result["counts"]["raw_equal"]==918 and result["counts"]["trajectory_comparable"]==0


def test_partial_failure_all_remaining918_retained(owner,worker,monkeypatch,tmp_path):
    values=synthetic_rows(2);values["subscriber"]=values["subscriber"][:1]
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,values)
    assert len(rows)==918 and result["counts"]["UNEXECUTED"]==917
    assert rows[-1]["status"]=="UNEXECUTED"


@pytest.mark.parametrize("fault",["raw","tensor","input","DDS","trajectory"])
def test_different_numerical_history_not_hidden(owner,worker,monkeypatch,tmp_path,fault):
    values=synthetic_rows(918)
    if fault=="raw":values["subscriber"][200]["policy_diagnostics"]["owned_recorded_numerical_event"]["raw_output"]["head"][0][0]+=1
    if fault=="tensor":values["subscriber"][200]["policy_diagnostics"]["owned_recorded_numerical_event"]["tensor_sha256"]["images"]["sha256"]="b"*64
    if fault=="input":values["direct"][200]["input_CDR_sha256"]["/image"]="x"*64
    if fault=="DDS":values["publisher"][200]["published_input_CDR_sha256"]["/image"]="x"*64
    if fault=="trajectory":values["publisher"][200]["actual_DDS_outputs"]["/planning/portable_e2e/shadow_path"]["sha256"]="x"*64
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,values)
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED" and rows[200]["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED"


def test_wrong_ordinal_or_native_frame_denied(owner,worker,monkeypatch,tmp_path):
    values=synthetic_rows(2);values["publisher"][1]["native_frame"]+=1
    with pytest.raises(ValueError):comparison(owner,worker,monkeypatch,tmp_path,values)


def test_owned_exec_admits_only_after_exact_argv(owner,worker,monkeypatch):
    identities=[dict(pid=23,ppid=os.getpid(),uid=os.getuid(),starttime=100,pgid=23,session=23,argv=[]),dict(pid=23,ppid=os.getpid(),uid=os.getuid(),starttime=100,pgid=23,session=23,argv=["declared"])]
    monkeypatch.setattr(worker,"process_identity",lambda pid:identities.pop(0));monkeypatch.setattr(owner.time,"sleep",lambda delay:None)
    assert owner.admit_process(SimpleNamespace(pid=23,poll=lambda:None),["declared"],worker)["argv"]==["declared"]


def test_owned_exec_generation_mismatch_denied(owner,worker,monkeypatch):
    identities=[dict(pid=23,ppid=os.getpid(),uid=os.getuid(),starttime=100,pgid=23,session=23,argv=[]),dict(pid=23,ppid=os.getpid(),uid=os.getuid(),starttime=101,pgid=23,session=23,argv=["declared"])]
    monkeypatch.setattr(worker,"process_identity",lambda pid:identities.pop(0));monkeypatch.setattr(owner.time,"sleep",lambda delay:None)
    with pytest.raises(ValueError):owner.admit_process(SimpleNamespace(pid=23,poll=lambda:None),["declared"],worker)


def test_cleanup_failed_exec_exact_child_only(owner,worker,monkeypatch):
    identity=dict(pid=23,ppid=os.getpid(),uid=os.getuid(),starttime=100,pgid=23,session=23,argv=[])
    calls=[];monkeypatch.setattr(worker,"process_identity",lambda pid:identity);monkeypatch.setattr(worker,"same_generation",lambda value:calls.append("generation"))
    process=SimpleNamespace(pid=23,poll=lambda:None,terminate=lambda:calls.append("terminate"),wait=lambda timeout:calls.append("wait"))
    owner.stop_owned(process,None,worker)
    assert calls==["generation","terminate","wait"]


def test_cleanup_foreign_child_never_signalled(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"process_identity",lambda pid:dict(pid=23,ppid=os.getpid()+1,uid=os.getuid(),argv=[]))
    process=SimpleNamespace(pid=23,poll=lambda:None,terminate=lambda:pytest.fail("foreign process signalled"))
    with pytest.raises(ValueError):owner.stop_owned(process,None,worker)


def test_model_only_good_final_and_no_learning(owner,worker):
    contract,pins=owner.model_contract(worker)
    assert contract["weights_sha256"].startswith("b24a6ddd") and len(pins)==6
    assert contract["research_owner_binding"]["fixed_endpoint_updates"]==6000
    assert contract["research_owner_binding"]["simulator_control_authorized"] is False


def test_original_deadline_not_removed_in_private_delegate(node,worker):
    ns=node.compile_original(worker);constants=[]
    def collect(code):
        constants.extend(code.co_consts)
        for value in code.co_consts:
            if hasattr(value,"co_consts"):collect(value)
    collect(ns["recorded_node"].__code__)
    assert 100_000_000 in constants
    assert "REJECTED_ROS_END_TO_END_DEADLINE" in constants


def test_publication_deadline_reject_raw_only_never_same_pass(owner,worker,monkeypatch,tmp_path):
    values=synthetic_rows(918);values["subscriber"][10]["status"]="REJECTED_ROS_END_TO_END_DEADLINE"
    values["publisher"][10]["status"]="REJECTED_ROS_END_TO_END_DEADLINE";values["publisher"][10]["actual_DDS_outputs"]={}
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,values)
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED" and rows[10]["status"]=="RAW_ONLY_PASS_OUTPUT_STATUS_DIFFERS"
    assert result["counts"]["raw_equal"]==918 and result["counts"]["status_equal"]==917


def test_source_enforces_log_rlimit_and_inherited_lock(worker,owner,node):
    tree=ast.parse(Path(owner.__file__).read_bytes());calls=[part for part in ast.walk(tree) if isinstance(part,ast.Call) and ast.unparse(part.func)=="subprocess.Popen"]
    assert len(calls)==1
    arguments={item.arg:ast.unparse(item.value) for item in calls[0].keywords}
    assert arguments["pass_fds"]=="(lock.fileno(),)" and arguments["preexec_fn"]=="death_guard"
    worker_tree=ast.parse(Path(worker.__file__).read_bytes())
    assert any(isinstance(part,ast.Call) and ast.unparse(part.func)=="resource.setrlimit" and "RLIMIT_FSIZE" in ast.unparse(part) for part in ast.walk(worker_tree))
    assert worker.fixed_contract()["ROS_LOG_DIR"]==str(worker.OUTPUT/"ros_logs")
    assert worker.fixed_contract()["Linux_parent_death_SIGTERM"] is True
    assert "graph_guard(node, set(), {'portable_temporal_recorded_direct'})" in ast.unparse(ast.parse(Path(node.__file__).read_bytes()))


def test_asymmetric_no_nn_never_hidden(owner,worker,monkeypatch,tmp_path):
    values=synthetic_rows(918)
    values["subscriber"][100]["policy_diagnostics"]["owned_recorded_numerical_event"]["NN_executed"]=False
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,values)
    assert result["counts"]["NN_presence_equal"]==917
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED"
