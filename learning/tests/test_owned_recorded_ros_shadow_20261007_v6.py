"""HH_261007 - Pure synthetic protocol tests, NOT actual DDS/model/driving measurements."""
from __future__ import annotations

import ast
import asyncio
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
def worker():return module("tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v6.py","_hh_test_protocol_261007")


@pytest.fixture
def owner():return module("tools/e2e/run_owned_temporal_recorded_ros_shadow_20261007_v6.py","_hh_test_owner_261007")


@pytest.fixture
def node():return module("autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v6.py","_hh_test_node_261007")


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
    worker.verify_project_modules(view,[SimpleNamespace(__name__="scripts.e2e",__file__=None,__path__=[str(ROOT/"tools/e2e")])])
    for value in (SimpleNamespace(__name__="scripts",__file__=None,__path__=["/tmp/foreign"]),SimpleNamespace(__name__="portable_e2e",__file__=None)):
        with pytest.raises(ValueError):worker.verify_project_modules(view,[value])


@pytest.mark.parametrize("name,path",[("scripts.e2e","/tmp/foreign"),("scripts.e2e.unknown",str(ROOT/"tools/e2e")),("scripts.analysis",str(ROOT/"tools/analysis"))])
def test_virtual_namespace_only_exact_two(worker,name,path):
    with pytest.raises(ValueError,match=name):worker.verify_project_modules(dict(source_sha256={},generated_ROS_sources={}),[SimpleNamespace(__name__=name,__file__=None,__path__=[path])])


def test_genuine_stdlib_bootstrap_namespace_no_ros_nn(worker,monkeypatch):
    source=ROOT/"tools/workspace_python.py";spec=importlib.util.spec_from_file_location("_hh_pure_bootstrap_261007",source)
    bootstrap=importlib.util.module_from_spec(spec);spec.loader.exec_module(bootstrap)
    before_modules=dict(sys.modules);before_path=list(sys.path);before_meta=list(sys.meta_path)
    try:
        bootstrap.configure(ROOT)
        package=__import__("importlib").import_module("scripts.e2e")
        assert package.__file__ is None and list(package.__path__)==[str(ROOT/"tools/e2e")]
        worker.verify_project_modules(dict(source_sha256={},generated_ROS_sources={}),[sys.modules["scripts"],package])
        assert "torch" not in sys.modules and "rclpy" not in sys.modules
    finally:
        sys.path[:]=before_path;sys.meta_path[:]=before_meta
        for name in set(sys.modules)-set(before_modules):sys.modules.pop(name)
        sys.modules.update(before_modules)


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


def test_actual_gid_missing_info_never_substituted_from_graph(node):
    endpoint=SimpleNamespace(node_name="peer",node_namespace="/",endpoint_gid=b"g"*24)
    graph=SimpleNamespace(get_publishers_info_by_topic=lambda topic:[endpoint])
    with pytest.raises(ValueError,match="GID is missing"):
        node.authenticated_gid(graph,"/allowed",SimpleNamespace(),"peer")


class FakeSingleThreadedExecutor:
    """HH_261007 - A pure fixture for executor delegation, not a DDS or native measurement."""
    def __init__(self,*args,**kwargs):self.calls=[]
    def _take_subscription(self,subscription):
        self.calls.append(("default_take",subscription));return "default message"
    async def _execute_subscription(self,subscription,message):
        self.calls.append(("default_execute",subscription,message));return subscription.callback(message)
    async def _execute_timer(self,*args):return "default timer"
    async def _execute_service(self,*args):return "default service"
    def add_node(self,node):self.calls.append(("add",node));return True
    def remove_node(self,node):self.calls.append(("remove",node))
    def spin_once(self,*,timeout_sec=None):self.calls.append(("spin",timeout_sec));return "private spin"
    def shutdown(self):self.calls.append(("shutdown",))


async def fake_await_or_execute(callback,*args):
    value=callback(*args)
    return await value if asyncio.iscoroutine(value) else value


def fake_typed_subscription(callback):
    # HH_261007 - Identity-only capsule fixtures; no generated/native function is executed in these tests.
    class MessageMetaclass(type):
        _TYPE_SUPPORT=object()
        _CREATE_ROS_MESSAGE=object()
        _DESTROY_ROS_MESSAGE=object()
        _CONVERT_TO_PY=object()
    class Message(metaclass=MessageMetaclass):pass
    return SimpleNamespace(handle=object(),msg_type=Message,raw=False,callback=callback)


def test_owned_metadata_executor_native_tuple_and_actual_gid(node):
    message=object();metadata=dict(publisher_gid=b"g"*24,source_timestamp=-1,received_timestamp=17,
        publication_sequence_number=18446744073709551615,reception_sequence_number=3,
        implementation_identifier="rmw_cyclonedds_cpp",from_intra_process=False)
    calls=[];received=[]
    subscription=fake_typed_subscription(lambda msg,info:received.append((msg,info)))
    bridge=SimpleNamespace(take_owned=lambda sub:(calls.append(sub) or (message,metadata)))
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,bridge,fake_await_or_execute)()
    executor.register_owned_subscription(subscription)
    actual=executor._take_subscription(subscription)
    asyncio.run(executor._execute_subscription(subscription,actual))
    assert calls==[subscription] and received[0][0] is message
    info=received[0][1]
    assert info.publisher_gid==metadata["publisher_gid"] and info.publication_sequence_number==18446744073709551615
    assert info.source_timestamp==-1 and info.received_timestamp==17
    endpoint=SimpleNamespace(node_name="peer",node_namespace="/",endpoint_gid=b"g"*24)
    assert node.authenticated_gid(SimpleNamespace(get_publishers_info_by_topic=lambda topic:[endpoint]),"/owned",info,"peer")==bytes(b"g"*24).hex()
    assert executor.calls==[]


def test_owned_metadata_executor_empty_take_no_callback(node):
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:None),fake_await_or_execute)()
    subscription=fake_typed_subscription(lambda *args:pytest.fail("empty native take invoked callback"))
    executor.register_owned_subscription(subscription)
    assert executor._take_subscription(subscription) is None
    assert asyncio.run(executor._execute_subscription(subscription,None)) is None


@pytest.mark.parametrize("value",[(object(),{}),(object(),dict(publisher_gid=b"short")),
    (object(),dict(publisher_gid=list(range(24)))),(None,dict(publisher_gid=b"g"*24)),
    [object(),dict(publisher_gid=b"g"*24)],(object(),SimpleNamespace(publisher_gid=b"g"*24))])
def test_owned_metadata_executor_missing_or_malformed_info_fails(node,value):
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:value),fake_await_or_execute)()
    subscription=fake_typed_subscription(lambda *args:pytest.fail("malformed metadata invoked callback"))
    executor.register_owned_subscription(subscription)
    with pytest.raises(ValueError):executor._take_subscription(subscription)
    with pytest.raises(ValueError):asyncio.run(executor._execute_subscription(subscription,value))


def test_metadata_executor_unowned_subscription_retains_default_behavior(node):
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:pytest.fail("unowned native take")),fake_await_or_execute)()
    received=[];subscription=SimpleNamespace(callback=lambda msg:received.append(msg))
    assert executor._take_subscription(subscription)=="default message"
    asyncio.run(executor._execute_subscription(subscription,"default message"))
    assert received==["default message"]
    assert [call[0] for call in executor.calls]==["default_take","default_execute"]
    assert type(executor)._execute_timer is FakeSingleThreadedExecutor._execute_timer
    assert type(executor)._execute_service is FakeSingleThreadedExecutor._execute_service


def test_metadata_executor_ownership_is_object_identity_not_equality(node):
    class Subscription:
        callback=staticmethod(lambda msg:None)
        def __eq__(self,other):return True
    owned=Subscription();foreign=Subscription()
    fields=fake_typed_subscription(lambda msg:None)
    for name in ("handle","msg_type","raw","callback"):setattr(owned,name,getattr(fields,name))
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:pytest.fail("foreign identity native take")),fake_await_or_execute)()
    executor.register_owned_subscription(owned)
    assert executor.owns_subscription(owned) and not executor.owns_subscription(foreign)
    assert executor._take_subscription(foreign)=="default message"


@pytest.mark.parametrize("field",["handle","msg_type","raw","callback"])
def test_owned_typed_subscription_mutation_denied_before_native_take(node,field):
    subscription=fake_typed_subscription(lambda *args:None)
    bridge=SimpleNamespace(take_owned=lambda sub:pytest.fail("unsafe changed typed subscription reached native take"))
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,bridge,fake_await_or_execute)()
    executor.register_owned_subscription(subscription)
    if field=="msg_type":subscription.msg_type=fake_typed_subscription(lambda *args:None).msg_type
    elif field=="raw":subscription.raw=True
    elif field=="callback":subscription.callback=lambda *args:None
    else:subscription.handle=object()
    with pytest.raises(ValueError):executor._take_subscription(subscription)
    with pytest.raises(ValueError):asyncio.run(executor._execute_subscription(subscription,None))


@pytest.mark.parametrize("name",["_TYPE_SUPPORT","_CREATE_ROS_MESSAGE","_DESTROY_ROS_MESSAGE","_CONVERT_TO_PY"])
def test_owned_generated_capsule_mutation_denied_before_native_take(node,name):
    subscription=fake_typed_subscription(lambda *args:None)
    bridge=SimpleNamespace(take_owned=lambda sub:pytest.fail("changed generated capsule reached unsafe native take"))
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,bridge,fake_await_or_execute)()
    executor.register_owned_subscription(subscription)
    setattr(type(subscription.msg_type),name,object())
    with pytest.raises(ValueError,match="capsule identity changed"):executor._take_subscription(subscription)


@pytest.mark.parametrize("invalid",["raw","message_type","capsule"])
def test_owned_registration_requires_initialized_typed_identity(node,invalid):
    subscription=fake_typed_subscription(lambda *args:None)
    if invalid=="raw":subscription.raw=True
    elif invalid=="message_type":subscription.msg_type=object()
    else:type(subscription.msg_type)._CONVERT_TO_PY=None
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:pytest.fail("invalid registration took message")),fake_await_or_execute)()
    with pytest.raises(ValueError):executor.register_owned_subscription(subscription)
    assert not executor.owns_subscription(subscription)


def test_owned_registration_cannot_rebind_mutated_identity(node):
    subscription=fake_typed_subscription(lambda *args:None)
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:pytest.fail("rebound identity took message")),fake_await_or_execute)()
    executor.register_owned_subscription(subscription)
    subscription.msg_type=fake_typed_subscription(lambda *args:None).msg_type
    with pytest.raises(ValueError,match="already registered"):executor.register_owned_subscription(subscription)
    with pytest.raises(ValueError):executor._take_subscription(subscription)


def test_private_rclpy_view_never_changes_original_module_or_global_executor(node):
    original_spin=lambda *args,**kwargs:pytest.fail("original global executor invoked")
    global_executor=object();original=SimpleNamespace(spin_once=original_spin,_global_executor=global_executor,shutdown=lambda:None)
    executor=FakeSingleThreadedExecutor();private=node.private_rclpy_view(original,executor);owned=object()
    assert private is not original and original.spin_once is original_spin and original._global_executor is global_executor
    assert private.spin_once(owned,timeout_sec=.02)=="private spin"
    assert executor.calls==[("add",owned),("spin",.02),("remove",owned)]
    assert original.spin_once is original_spin and original._global_executor is global_executor


def test_private_spin_removes_node_even_on_executor_error(node):
    class FailingExecutor(FakeSingleThreadedExecutor):
        def spin_once(self,**kwargs):raise RuntimeError("genuine spin failed")
    executor=FailingExecutor();owned=object();private=node.private_rclpy_view(SimpleNamespace(spin_once=lambda:None),executor)
    with pytest.raises(RuntimeError,match="genuine spin failed"):private.spin_once(owned,timeout_sec=.02)
    assert executor.calls==[("add",owned),("remove",owned)]


def test_native_bridge_load_requires_exact_fresh_sealed_module(node,worker,tmp_path,monkeypatch):
    # HH_261007 - Mock only module loading; this test does not import or execute a compiled native bridge.
    path=tmp_path/"synthetic_module.so";path.write_bytes(b"pure sealed fixture, not native code")
    bridge=SimpleNamespace(__file__=str(path),__name__="_hh_owned_dds_info_261007",take_owned=lambda sub:None)
    calls=[];monkeypatch.setattr(worker,"load_module",lambda actual,name:(calls.append((actual,name)) or bridge))
    view=dict(native_dds_bridge=dict(module_path=str(path),module_sha256=worker.digest(path)))
    assert node.load_native_bridge(view,worker) is bridge
    assert calls==[(path,"_hh_owned_dds_info_261007")]
    monkeypatch.setitem(sys.modules,"_hh_owned_dds_info_261007",bridge)
    with pytest.raises(ValueError,match="freshly loaded"):node.load_native_bridge(view,worker)


@pytest.mark.parametrize("fault",["sha","origin","api"])
def test_native_bridge_wrong_seal_origin_or_api_denied(node,worker,tmp_path,monkeypatch,fault):
    path=tmp_path/"synthetic_module.so";path.write_bytes(b"pure fixture")
    bridge=SimpleNamespace(__file__=str(path),__name__="_hh_owned_dds_info_261007",take_owned=lambda sub:None)
    view=dict(native_dds_bridge=dict(module_path=str(path),module_sha256=worker.digest(path)))
    if fault=="sha":view["native_dds_bridge"]["module_sha256"]="0"*64
    if fault=="origin":bridge.__file__=str(tmp_path/"missing-foreign-module.so")
    if fault=="api":bridge.take_owned=None
    monkeypatch.setattr(worker,"load_module",lambda actual,name:bridge)
    with pytest.raises((ValueError,FileNotFoundError)):node.load_native_bridge(view,worker)


def manifest(worker):
    roles=worker.frozen_bundle().ROLES
    yaws=(0,math.pi,math.pi/4,3*math.pi/4,-math.pi/4,-3*math.pi/4)
    return dict(runtime=dict(server_version="synthetic-not-actual"),cameras=[dict(name=role,model_index=index,width=640,height=360,enable_postprocess_effects=False,sensor_tick=0.,frame_id=role+"/camera_optical_link",
        intrinsic=dict(fx=320.,fy=320.,cx=320.,cy=180.),ros_extrinsic=dict(x=1.,y=0.,z=1.6,roll=0.,pitch=0.,yaw=yaw)) for index,(role,yaw) in enumerate(zip(roles,yaws))])


def test_rig_uses_genuine_source_exporter(owner,worker):
    result=owner.genuine_rig(manifest(worker),worker)
    assert result["schema_id"]=="autoware-e2e.common10.rig.v1" and len(result["cameras"])==6
    for camera in result["cameras"]:
        q=owner.quaternion(camera["T_base_from_camera"])
        assert len(q)==4 and abs(sum(v*v for v in q)-1)<1e-12


def test_rig_missing_model_index_denied(owner,worker):
    value=manifest(worker);del value["cameras"][0]["model_index"]
    with pytest.raises(ValueError):owner.genuine_rig(value,worker)


def test_archived_actual_calibration_metadata_only(owner,worker):
    value=json.loads((worker.EPISODE/"manifest.json").read_bytes())
    rig=owner.genuine_rig(value,worker)
    calibration=tuple(tuple((camera["K"][0]/640,camera["K"][4]/360,camera["K"][2]/640,camera["K"][5]/360,*camera["T_base_from_camera"][:12])) for camera in rig["cameras"])
    assert hashlib.sha256(json.dumps(calibration,allow_nan=False).encode()).hexdigest()==value["capture_contract"]["learned_research"]["stream_identity"]["rig_id"]
    assert len(rig["cameras"])==6


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
        calls=[dict(actually_published=True,entry_deadline_passed=True,completion_deadline_passed=True) for _ in range(2)] if trajectories else []
        rows["subscriber"].append(dict(common,policy_diagnostics=dict(owned_recorded_numerical_event=deepcopy(event)),actual_received_input_CDR_sha256={"/image":"i"*64},actual_output_publication_calls=calls,actual_full_publication_deadline_passed=True))
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


@pytest.mark.parametrize("begin,entered,finished,entry,completion",[(1,100000001,100000001,True,True),(1,100000002,100000003,False,False),(1,100000000,100000002,True,False)])
def test_actual_publication_timing_no_early_gate_substitution(node,begin,entered,finished,entry,completion):
    result=node.publication_timing(begin,entered,finished)
    assert result["entry_deadline_passed"] is entry and result["completion_deadline_passed"] is completion


@pytest.mark.parametrize("values",[(True,10,11),(1,0,1),(5,3,6),(1,3,2),(0,1,2)])
def test_actual_timing_invalid_chronology(node,values):
    with pytest.raises(ValueError):node.publication_timing(*values)


def test_late_true_publication_never_overall_pass(owner,worker,monkeypatch,tmp_path):
    values=synthetic_rows(918);values["subscriber"][80]["actual_full_publication_deadline_passed"]=False
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,values)
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED" and result["counts"]["trajectory_timely"]==917


def test_v1_failed_sources_immutable(worker):
    for path,pin in worker.V1_PINS.items():assert worker.digest(ROOT/path)==pin
    for path,pin in worker.V2_PINS.items():assert worker.digest(ROOT/path)==pin
    for path,pin in worker.V3_PINS.items():assert worker.digest(ROOT/path)==pin
