"""HH_261008 - Preserve scientific V11 regressions and test subscriber membership using public-API-shaped fakes; no actual DDS/model/driving measurement."""
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
def worker():return module("tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v12.py","_hh_test_protocol_261007")


@pytest.fixture
def owner():return module("tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v12.py","_hh_test_owner_261007")


@pytest.fixture
def node():return module("autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v12.py","_hh_test_node_261007")


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


def native_witness_fixture():
    # HH_261007 - Synthetic fixtures exercise true-handle validation without loading DDS or a message library.
    handle=2**63+17
    message_type=type("String",(),{"__module__":"std_msgs.msg._string"})
    info=SimpleNamespace(source_timestamp=2**61+3,received_timestamp=0,
        publication_sequence_number=2**64-1,reception_sequence_number=2**64-1,
        publisher_gid=bytes(range(1,17))+bytes(8),implementation_identifier="rmw_cyclonedds_cpp",from_intra_process=False,
        raw_rmw_publisher_gid=handle.to_bytes(8,sys.byteorder)+bytes(16),actual_publication_handle=handle,
        dds_reader=17,dds_participant=3,dds_reader_guid=bytes(range(16)),dds_topic_name="rt/owned",
        dds_type_name="std_msgs::msg::dds_::String_",dds_local_entity_count=512,
        publisher_gid_provenance="public_dds_matched_publication_data_for_actual_taken_handle")
    return info,message_type


def test_native_witness_preserves_raw_uint64_and_unsupported_fields(node):
    info,cls=native_witness_fixture(); witness=node.native_transport_witness(info,"/owned",cls)
    assert witness["actual_publication_handle"]==2**63+17
    assert witness["publication_sequence_number"]==2**64-1 and witness["received_timestamp"]==0
    assert witness["raw_rmw_publisher_gid"]==info.raw_rmw_publisher_gid.hex()
    assert json.loads(json.dumps(witness))==witness


@pytest.mark.parametrize("key,value",[("implementation_identifier","foreign"),("publisher_gid",bytes(23)),
    ("publisher_gid",bytes(16)+b"x"+bytes(7)),("raw_rmw_publisher_gid",bytes(24)),
    ("actual_publication_handle",True),("actual_publication_handle",2**64),
    ("publisher_gid_provenance","graph_fallback"),("dds_topic_name","rt/foreign"),
    ("dds_type_name","std_msgs::msg::dds_::Wrong_"),("dds_reader_guid",bytes(15)),
    ("dds_reader",True),("dds_participant",0),("dds_local_entity_count",513),
    ("from_intra_process",True),("source_timestamp",True),("received_timestamp",-1),
    ("publication_sequence_number",-1),("reception_sequence_number",2**64)])
def test_native_witness_unknown_or_mismatched_abi_denied(node,key,value):
    info,cls=native_witness_fixture();setattr(info,key,value)
    with pytest.raises(ValueError):node.native_transport_witness(info,"/owned",cls)


def test_native_witness_unexpected_metadata_denied(node):
    info,cls=native_witness_fixture(); info.graph_gid="not native"
    with pytest.raises(ValueError):node.native_transport_witness(info,"/owned",cls)


def test_latest_native_tails_and_fresh_plugin_probe_required_by_source(worker,owner,node):
    assert worker.BRIDGE_REPORT.parent.name=="owned_dds_info_bridge_v5"
    assert worker.fixed_contract()["installed_Cyclone_RMW_plugin_sealed"] is True
    assert "librmw_cyclonedds_cpp.so" in Path(owner.__file__).read_text()
    body=Path(node.__file__).read_text()
    assert 'actual_received_input_native_transport=dict(self.owned_input_transport)' in body
    assert '"_native_transport_tail.json"' in body and 'complete_transport_history_claimed=False' in body


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


class AuditArray:
    """HH_261007 - Scalar byte-work fixture only; never a genuine NumPy/CNN output."""
    dtype="float32"
    shape=(1,2)
    def __init__(self,tensor):self.tensor=tensor
    def tobytes(self):
        self.tensor.events.append("bytes:"+self.tensor.name)
        return json.dumps(self.tensor.values,allow_nan=False).encode()


class AuditTensor:
    """HH_261007 - Model the CPU tensor identity/version ABI without importing Torch."""
    dtype="float32"
    shape=(1,2)
    device=SimpleNamespace(type="cpu")
    def __init__(self,name,events,values=None):
        self.name=name;self.events=events;self.values=[[1.,2.]] if values is None else values
        self._version=0;self.hash_failure=False;self.after_numpy=None
    def detach(self):self.events.append("detach:"+self.name);return self
    def cpu(self):self.events.append("cpu:"+self.name);return self
    def contiguous(self):self.events.append("contiguous:"+self.name);return self
    def numpy(self):
        self.events.append("numpy:"+self.name)
        if self.hash_failure:raise RuntimeError("synthetic audit conversion failure")
        if self.after_numpy:self.after_numpy()
        return AuditArray(self)
    def tolist(self):self.events.append("tolist:"+self.name);return deepcopy(self.values)


class AuditFixtureModel:
    def __init__(self,events):
        self.events=events;self.forward_calls=0
        self.outputs={name:AuditTensor(name,events) for name in
            ("curvature_knots","preferred_speed_knots","candidate_logits","stop_intent_logits","acceleration_knots")}
    def forward_cached(self,inputs,*,window):
        assert window=="same admitted synthetic window"
        self.forward_calls+=1;self.events.append("forward")
        return self.outputs


class AuditFixturePolicy:
    def __init__(self,events):
        self.events=events;self.inputs={"images":AuditTensor("images",events)}
        self.model=AuditFixtureModel(events)
    def build_inputs(self,frame):
        self.events.append("build")
        return self.inputs,"same admitted synthetic window"


class AuditFixtureAdapter:
    def __init__(self,events,*,forward=True,status="TRAJECTORY",build=True):
        self.events=events;self._policy=AuditFixturePolicy(events);self.forward=forward;self.build=build
        self.status=status;self.calls=0;self.reset_calls=0;self.last_result=None
    def reset(self,*args,**kwargs):
        self.events.append("original_reset");self.reset_calls+=1
        return "original reset returned"
    def accept(self,frame,**kwargs):
        self.calls+=1
        if self.build:
            inputs,window=self._policy.build_inputs(frame)
            if self.forward:self._policy.model.forward_cached(inputs,window=window)
        self.last_result=SimpleNamespace(status=self.status,anchor_ns=123,diagnostics={"original_guard":True})
        self.events.append("original_accept_return")
        return self.last_result


def audit_byte_events(events):
    return [item for item in events if item.startswith(("numpy:","bytes:","tolist:"))]


def test_deferred_exact_original_event_only_after_flush_no_extra_forward(node,worker):
    immediate_events=[];direct_adapter=AuditFixtureAdapter(immediate_events)
    immediate=node.AuditedAdapter(direct_adapter,worker).accept("same frame")
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("same frame")
    assert result is adapter.last_result and audit_byte_events(events)==[]
    assert adapter.calls==adapter._policy.model.forward_calls==1 and deferred._pending is not None
    events.extend(("trajectory_publish","path_publish"))
    deferred.flush_pending(timestamp_ns=123,context="ACK")
    event=result.diagnostics["owned_recorded_numerical_event"]
    original=immediate.diagnostics["owned_recorded_numerical_event"]
    # HH_261007 - The unchanged base numerical receipt is compared, not an invented real-runtime parity claim.
    assert {key:event[key] for key in original}==original
    assert events.index("path_publish")<events.index("numpy:images")
    assert deferred._pending is None and adapter.calls==adapter._policy.model.forward_calls==1
    assert result.diagnostics["original_guard"] is True
    assert "build_inputs" not in adapter._policy.__dict__ and "forward_cached" not in adapter._policy.model.__dict__


def test_deferred_single_pending_refuses_second_accept_without_forward(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    deferred.accept("first")
    with pytest.raises(ValueError):deferred.accept("second")
    assert adapter.calls==adapter._policy.model.forward_calls==1 and audit_byte_events(events)==[]


@pytest.mark.parametrize("forward,build",[(False,True),(False,False)])
def test_deferred_prefill_and_original_no_tensor_branches_keep_NN_denominator(node,worker,forward,build):
    events=[];adapter=AuditFixtureAdapter(events,forward=forward,build=build,status="PREFILL")
    deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None);result=deferred.accept("original cold frame")
    assert audit_byte_events(events)==[]
    deferred.flush_pending(timestamp_ns=123,context="ACK")
    event=result.diagnostics["owned_recorded_numerical_event"]
    assert event["NN_executed"] is False and event["tensor_available"] is build
    assert event["raw_output"] is None and adapter._policy.model.forward_calls==0
    assert event["frozen_policy_status"]=="PREFILL" and deferred._pending is None


@pytest.mark.parametrize("fault",["input_version","output_version","input_identity","output_identity","input_keys","output_keys"])
def test_deferred_mutation_or_member_identity_change_denied_without_new_forward(node,worker,fault):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("original frame");inputs=adapter._policy.inputs;outputs=adapter._policy.model.outputs
    head="curvature_knots"
    if fault=="input_version":inputs["images"]._version+=1
    elif fault=="output_version":outputs[head]._version+=1
    elif fault=="input_identity":inputs["images"]=AuditTensor("images",events)
    elif fault=="output_identity":outputs[head]=AuditTensor(head,events)
    elif fault=="input_keys":inputs["unexpected"]=AuditTensor("unexpected",events)
    else:outputs.pop(head)
    with pytest.raises(ValueError):deferred.flush_pending(timestamp_ns=123,context="ACK")
    assert deferred._pending is None and deferred.failure_tail is not None
    assert result.diagnostics["original_guard"] is True and adapter._policy.model.forward_calls==1
    json.dumps(deferred.failure_tail,allow_nan=False)
    with pytest.raises(ValueError):deferred.accept("forbidden next")
    assert adapter.calls==1


def test_deferred_mutation_during_byte_work_denied_after_hash(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    deferred.accept("original frame")
    image=adapter._policy.inputs["images"]
    image.after_numpy=lambda:setattr(image,"_version",image._version+1)
    with pytest.raises(ValueError):deferred.flush_pending(timestamp_ns=123,context="ACK")
    assert "numpy:images" in events and deferred._pending is None and deferred.failure_tail is not None
    assert adapter._policy.model.forward_calls==1


def test_deferred_hash_failure_releases_refs_and_retains_failure_across_original_reset(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    deferred.accept("original frame");adapter._policy.inputs["images"].hash_failure=True
    with pytest.raises(RuntimeError,match="synthetic audit conversion failure"):
        deferred.flush_pending(timestamp_ns=123,context="ACK")
    assert deferred._pending is None and deferred.failure_tail is not None
    json.dumps(deferred.failure_tail,allow_nan=False)
    tail=deepcopy(deferred.failure_tail)
    with pytest.raises(ValueError):deferred.accept("next")
    assert deferred.reset("original failure reset")=="original reset returned"
    assert deferred.failure_tail==tail
    with pytest.raises(ValueError):deferred.accept("reset cannot bypass latched audit failure")
    assert adapter.calls==1 and adapter.reset_calls==1


def test_deferred_reset_flushes_before_original_reset_without_extra_forward(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("original frame")
    assert deferred.reset("original argument")=="original reset returned"
    assert events.index("numpy:images")<events.index("original_reset")
    assert result.diagnostics["owned_recorded_numerical_event"]["NN_executed"] is True
    assert adapter.calls==adapter._policy.model.forward_calls==adapter.reset_calls==1 and deferred._pending is None


@pytest.mark.parametrize("version",[True,False,-1,1.0,None,"0"])
def test_deferred_version_ABI_exact_nonnegative_integer_before_byte_work(node,version):
    events=[];tensor=AuditTensor("images",events);tensor._version=version
    with pytest.raises(ValueError):node.tensor_identity_snapshot({"images":tensor})
    assert audit_byte_events(events)==[]


def test_deferred_version_bool_equal_to_zero_denied_at_flush(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    deferred.accept("original frame");adapter._policy.inputs["images"]._version=False
    with pytest.raises(ValueError):deferred.flush_pending(timestamp_ns=123,context="ACK")
    assert deferred._pending is None and deferred.failure_tail is not None and audit_byte_events(events)==[]


@pytest.mark.parametrize("mapping",[[],None,{1:object()}])
def test_deferred_only_exact_named_tensor_dictionary(node,mapping):
    with pytest.raises(ValueError):node.tensor_identity_snapshot(mapping)


def test_deferred_wrong_ACK_anchor_retained_no_byte_work_or_extra_forward(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("original frame")
    with pytest.raises(ValueError):deferred.flush_pending(timestamp_ns=124,context="ACK")
    assert deferred._pending is None and deferred.failure_tail["anchor_ns"]==123
    assert audit_byte_events(events)==[] and adapter.calls==adapter._policy.model.forward_calls==1
    assert result.diagnostics["owned_recorded_numerical_event"]["tensor_sha256"] is None


def test_deferred_original_policy_exception_is_not_suppressed_and_providers_restored(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);original_error=RuntimeError("original policy failure")
    def fail(*args,**kwargs):raise original_error
    adapter.accept=fail;deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    with pytest.raises(RuntimeError) as error:deferred.accept("original frame")
    assert error.value is original_error and deferred._pending is None
    assert deferred.failure_tail["stage"]=="policy_or_pending_capture"
    assert "build_inputs" not in adapter._policy.__dict__ and "forward_cached" not in adapter._policy.model.__dict__
    assert adapter._policy.model.forward_calls==0 and audit_byte_events(events)==[]


def test_deferred_post_pending_diagnostics_attachment_failure_cannot_fabricate_empty_event(node,worker):
    # HH_261007 - Fail after genuine original inference and pending registration, not before the actual event exists.
    class DeniedAttachment(dict):
        def __setitem__(self,key,value):
            if key=="owned_recorded_numerical_event":raise RuntimeError("synthetic diagnostic attachment denied")
            return super().__setitem__(key,value)
    events=[];adapter=AuditFixtureAdapter(events);original_accept=adapter.accept
    def accept(*args,**kwargs):
        result=original_accept(*args,**kwargs);result.diagnostics=DeniedAttachment(result.diagnostics)
        return result
    adapter.accept=accept;deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    with pytest.raises(RuntimeError,match="synthetic diagnostic attachment denied"):
        deferred.accept("original frame")
    assert deferred._pending is None and deferred.failure_tail["NN_events"]==1
    assert deferred.failure_tail["tensor_events"]==1 and audit_byte_events(events)==[]
    assert adapter.calls==adapter._policy.model.forward_calls==1
    assert adapter.last_result.diagnostics=={"original_guard":True}
    with pytest.raises(ValueError):deferred.flush_pending(timestamp_ns=123,context="ACK")
    with pytest.raises(ValueError):node.materialize_owned_ACK(dict(timestamp_ns=123,status="FAILED_RETAINED"),deferred)
    with pytest.raises(ValueError):deferred.accept("forbidden next frame")
    assert deferred.reset("genuine reset still required")=="original reset returned"
    assert adapter.reset_calls==1 and deferred.close_audit()["pending_reference_count"]==0
    assert deferred.last_materialized_result is None and deferred.last_timing is None
    assert adapter.calls==adapter._policy.model.forward_calls==1 and audit_byte_events(events)==[]


def test_deferred_multiple_original_forward_events_fail_without_replay_or_hashes(node,worker):
    events=[];adapter=AuditFixtureAdapter(events)
    def twice(frame,**kwargs):
        inputs,window=adapter._policy.build_inputs(frame)
        adapter._policy.model.forward_cached(inputs,window=window)
        adapter._policy.model.forward_cached(inputs,window=window)
        return SimpleNamespace(status="TRAJECTORY",anchor_ns=123,diagnostics={})
    adapter.accept=twice;deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    with pytest.raises(ValueError):deferred.accept("original frame")
    assert adapter._policy.model.forward_calls==2 and deferred.failure_tail["NN_events"]==2
    assert deferred._pending is None and audit_byte_events(events)==[]
    with pytest.raises(ValueError):deferred.accept("no retry")
    assert adapter._policy.model.forward_calls==2


@pytest.mark.parametrize("status",["TRAJECTORY","PREFILL","REJECTED","FAILED_RETAINED"])
def test_owned_ACK_flush_only_diagnostic_fields_and_original_value_unchanged(node,worker,status):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("original frame")
    value=dict(timestamp_ns=123,status=status,ordinal=17,native_frame=42,
        selected=dict(xy=[[1.,2.]],speed=[2.5]),control=dict(brake=0.,steer=.1),
        policy_diagnostics=dict(original_guard=True,owned_recorded_numerical_event=deepcopy(result.diagnostics["owned_recorded_numerical_event"])))
    old=deepcopy(value);events.extend(("trajectory_publish","path_publish"))
    actual=node.materialize_owned_ACK(value,deferred);events.append("real_ACK_publish")
    assert value==old and actual is not value
    for key in ("timestamp_ns","status","ordinal","native_frame","selected","control"):
        assert actual[key]==old[key]
    audit=actual["policy_diagnostics"]["owned_recorded_numerical_event"]
    assert audit==result.diagnostics["owned_recorded_numerical_event"] and audit["NN_executed"] is True
    assert actual["policy_diagnostics"]["original_guard"] is True and "deferred_audit_pending" not in audit
    assert events.index("path_publish")<events.index("numpy:images")<events.index("real_ACK_publish")
    assert actual["owned_ACK_completion_includes_deferred_audit"] is True
    assert actual["policy_diagnostics"]["owned_recorded_deferred_audit_timing"]["ACK_completion_includes_audit_cost"] is True
    assert deferred._pending is None and adapter._policy.model.forward_calls==1


def test_owned_ACK_already_flushed_by_reset_reuses_actual_receipt_without_new_byte_work(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("original frame");deferred.reset("original node failure reset")
    before=list(events)
    value=dict(timestamp_ns=123,status="FAILED_RETAINED",policy_diagnostics={})
    actual=node.materialize_owned_ACK(value,deferred)
    assert events==before and actual["policy_diagnostics"]["owned_recorded_numerical_event"]==result.diagnostics["owned_recorded_numerical_event"]
    assert adapter._policy.model.forward_calls==1


def test_owned_ACK_before_any_policy_has_no_fabricated_tensor_or_NN_event(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    value=dict(timestamp_ns=123,status="FAILED_RETAINED",policy_diagnostics={})
    actual=node.materialize_owned_ACK(value,deferred)
    assert actual["policy_diagnostics"]=={} and value["policy_diagnostics"]=={}
    assert events==[] and adapter._policy.model.forward_calls==0


@pytest.mark.parametrize("hash_failure",[False,True])
def test_deferred_close_materializes_or_retains_failure_without_missing_prefix_claim(node,worker,hash_failure):
    events=[];adapter=AuditFixtureAdapter(events);deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:None)
    result=deferred.accept("last genuinely attempted frame");adapter._policy.inputs["images"].hash_failure=hash_failure
    if hash_failure:
        with pytest.raises(RuntimeError):deferred.close_audit()
        assert deferred.failure_tail is not None and result.diagnostics["owned_recorded_numerical_event"]["tensor_sha256"] is None
        tail=deferred.close_audit()
    else:
        tail=deferred.close_audit()
        assert result.diagnostics["owned_recorded_numerical_event"]["NN_executed"] is True
    assert tail["pending_reference_count"]==0 and tail["whole_interval_audit_claimed"] is False
    assert tail["another_forward_executed"] is False and tail["failure_tail"]==deferred.failure_tail
    assert deferred._pending is None and adapter._policy.model.forward_calls==1


def test_v9_source_seam_immediate_direct_deferred_subscriber_ACK_before_real_publish(node):
    tree=ast.parse(Path(node.__file__).read_bytes())
    run=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=="run")
    load=next(item for item in run.body if isinstance(item,ast.FunctionDef) and item.name=="audited_load")
    choice=next(part for part in ast.walk(load) if isinstance(part,ast.IfExp))
    assert ast.unparse(choice.test)=="mode == 'subscriber'"
    assert ast.unparse(choice.body.func)=="DeferredAuditedAdapter" and ast.unparse(choice.orelse.func)=="AuditedAdapter"
    owned=next(item for item in run.body if isinstance(item,ast.ClassDef) and item.name=="OwnedNode")
    factory=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_publisher")
    publish=next(item for item in factory.body if isinstance(item,ast.FunctionDef) and item.name=="publish")
    source=ast.unparse(publish)
    assert source.index("materialize_owned_ACK(")<source.index("serialize_message(msg)")<source.index("real_publish(msg)")
    assert "actual_full_publication_seconds=(finished - self.publisher_begin) / 1000000000.0" in source
    assert "actual_full_publication_deadline_passed=finished - self.publisher_begin <= 100000000" in source
    immediate=next(item for item in tree.body if isinstance(item,ast.ClassDef) and item.name=="AuditedAdapter")
    old=ast.parse((ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v8.py").read_bytes())
    original=next(item for item in old.body if isinstance(item,ast.ClassDef) and item.name=="AuditedAdapter")
    assert ast.dump(immediate,include_attributes=False)==ast.dump(original,include_attributes=False)


def test_failed_deferred_audit_tail_keeps_whole918_remaining_denominator(owner,worker,monkeypatch,tmp_path):
    values=synthetic_rows(2);values["subscriber"]=values["subscriber"][:1];values["publisher"]=[]
    result,rows=comparison(owner,worker,monkeypatch,tmp_path,values)
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED" and len(rows)==918
    assert rows[-1]["status"]=="UNEXECUTED" and result["P3_approved"] is False


class SyntheticActiveShadowNode:
    """HH_261007 - Registry and graph scalars only; never a real ROS/DDS node."""
    def __init__(self,token,events):
        self._hh_graph_active=True;self._hh_graph_owner_token=token
        self.owned_name="portable_temporal_recorded_shadow";self.events=events
        self.name=self.owned_name;self.namespace="/"
        self.names=[(self.name,"/"),("portable_temporal_recorded_publisher","/")]
        self.topics=[("/recorded/provenance",[])]
    def get_name(self):return self.name
    def get_namespace(self):return self.namespace
    def peer(self):self.events.append("actual_peer_generation_guard")
    def get_node_names_and_namespaces(self):self.events.append("full_node_census");return self.names
    def get_topic_names_and_types(self):self.events.append("full_topic_census");return self.topics


def active_shadow_fixture():
    token=object();events=[];actual=SyntheticActiveShadowNode(token,events)
    allowed={"/recorded/provenance"}
    expected={"portable_temporal_recorded_shadow","portable_temporal_recorded_publisher"}
    return actual,token,events,allowed,expected


def test_v10_before_accept_callback_mandatory_not_default_bypass(node,worker):
    adapter=AuditFixtureAdapter([])
    with pytest.raises(TypeError):node.DeferredAuditedAdapter(adapter,worker)
    assert adapter.calls==adapter._policy.model.forward_calls==0


@pytest.mark.parametrize("callback",[None,True,False,1,object()])
def test_v10_before_accept_noncallable_denied_without_original_work(node,worker,callback):
    events=[];adapter=AuditFixtureAdapter(events)
    with pytest.raises(ValueError):node.DeferredAuditedAdapter(adapter,worker,before_accept=callback)
    assert adapter.calls==adapter._policy.model.forward_calls==0 and events==[]


def test_v10_preaccept_selects_live_actual_instance_not_destroyed_probe(node,worker):
    actual,token,events,allowed,expected=active_shadow_fixture()
    probe=SyntheticActiveShadowNode(token,[]);probe._hh_graph_active=False
    probe.get_node_names_and_namespaces=lambda:pytest.fail("destroyed startup probe used for graph proof")
    adapter=AuditFixtureAdapter(events)
    def before():return node.active_shadow_graph_guard([probe,actual],allowed,expected,SyntheticActiveShadowNode,token)
    deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=before)
    result=deferred.accept("same admitted frame")
    assert result is adapter.last_result and deferred.before_accept is before
    assert events[:3]==["actual_peer_generation_guard","full_node_census","full_topic_census"]
    assert events.index("full_topic_census")<events.index("build")<events.index("forward")
    deferred.flush_pending(timestamp_ns=123)
    assert adapter.calls==adapter._policy.model.forward_calls==1


@pytest.mark.parametrize("fault",["unknown_node","unknown_topic","duplicate_node","foreign_namespace"])
def test_v10_current_graph_rejected_before_original_accept_build_or_forward(node,worker,fault):
    actual,token,events,allowed,expected=active_shadow_fixture()
    if fault=="unknown_node":actual.names.append(("foreign","/"))
    elif fault=="unknown_topic":actual.topics.append(("/control/command/actuation_cmd",[]))
    elif fault=="duplicate_node":actual.names.append(actual.names[0])
    else:actual.names[0]=(actual.name,"/foreign")
    adapter=AuditFixtureAdapter(events)
    deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=lambda:
        node.active_shadow_graph_guard([actual],allowed,expected,SyntheticActiveShadowNode,token))
    with pytest.raises(ValueError):deferred.accept("must not touch original providers")
    assert adapter.calls==adapter._policy.model.forward_calls==0 and "build" not in events
    assert deferred._pending is None and audit_byte_events(events)==[]
    assert "build_inputs" not in adapter._policy.__dict__ and "forward_cached" not in adapter._policy.model.__dict__


def test_v10_preaccept_original_exception_passthrough_no_hidden_inference(node,worker):
    events=[];adapter=AuditFixtureAdapter(events);original=RuntimeError("actual graph proof failed")
    def before():events.append("pre_accept");raise original
    deferred=node.DeferredAuditedAdapter(adapter,worker,before_accept=before)
    with pytest.raises(RuntimeError) as error:deferred.accept("same frame")
    assert error.value is original and events==["pre_accept"]
    assert adapter.calls==adapter._policy.model.forward_calls==0 and deferred._pending is None


@pytest.mark.parametrize("fault",["absent","destroyed","multiple","duplicate_registry","foreign_class","wrong_token","wrong_name","wrong_namespace","integer_active"])
def test_v10_active_registry_identity_not_destroyed_or_plausible_fake(node,fault):
    actual,token,events,allowed,expected=active_shadow_fixture();nodes=[actual]
    if fault=="absent":nodes=[]
    elif fault=="destroyed":actual._hh_graph_active=False
    elif fault=="multiple":nodes.append(SyntheticActiveShadowNode(token,[]))
    elif fault=="duplicate_registry":nodes.append(actual)
    elif fault=="foreign_class":nodes=[SimpleNamespace(**vars(actual),get_name=actual.get_name,get_namespace=actual.get_namespace,peer=actual.peer)]
    elif fault=="wrong_token":actual._hh_graph_owner_token=object()
    elif fault=="wrong_name":actual.name="foreign"
    elif fault=="wrong_namespace":actual.namespace="/foreign"
    else:actual._hh_graph_active=1
    with pytest.raises(ValueError):node.active_shadow_graph_guard(nodes,allowed,expected,SyntheticActiveShadowNode,token)
    assert events==[]


def test_v10_active_registry_peer_failure_precedes_graph_source(node):
    actual,token,events,allowed,expected=active_shadow_fixture();original=RuntimeError("actual peer generation changed")
    def peer():events.append("peer_failed");raise original
    actual.peer=peer
    with pytest.raises(RuntimeError) as error:node.active_shadow_graph_guard([actual],allowed,expected,SyntheticActiveShadowNode,token)
    assert error.value is original and events==["peer_failed"]


def test_v10_receive_graph_census_only_provenance_with_exact_instance(node):
    actual,token,events,allowed,expected=active_shadow_fixture()
    assert node.receive_graph_guard(actual,"/recorded/image","/recorded/provenance",allowed,expected) is None
    assert events==[]
    receipt=node.receive_graph_guard(actual,"/recorded/provenance","/recorded/provenance",allowed,expected)
    assert receipt["DDS_security_identity_claimed"] is False and events==["full_node_census","full_topic_census"]
    actual.names.append(("foreign","/"))
    with pytest.raises(ValueError):node.receive_graph_guard(actual,"/recorded/provenance","/recorded/provenance",allowed,expected)


def test_v11_source_receive_retains_per_message_owner_gid_native_CDR_and_callback(node):
    tree=ast.parse(Path(node.__file__).read_bytes())
    run=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=="run")
    owned=next(item for item in run.body if isinstance(item,ast.ClassDef) and item.name=="OwnedNode")
    factory=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_subscription")
    receive=next(item for item in factory.body if isinstance(item,ast.FunctionDef) and item.name=="receive")
    source=ast.unparse(receive)
    # HH_261007 - V11 measures the same true guards through an explicit local invoke delegate.
    invokes=sorted((part for part in ast.walk(receive) if isinstance(part,ast.Call)
        and isinstance(part.func,ast.Name) and part.func.id=="invoke"),key=lambda part:(part.lineno,part.col_offset))
    names=[part.args[0].value for part in invokes]
    assert names==list(node.RECEIVE_STAGE_NAMES)
    assert source.index("invoke('peer', self.peer)")<source.index("invoke('provenance_graph', receive_graph_guard")<source.index("invoke('publisher_gid', authenticated_gid")<source.index("invoke('native_witness', native_transport_witness")<source.index("invoke('CDR', serialize_message")<source.index("invoke('callback', callback, msg)")
    calls=[ast.unparse(part.func) for part in ast.walk(receive) if isinstance(part,ast.Call)]
    assert "graph_guard" not in calls and "hashlib.sha256" in calls
    admission=next(item for item in run.body if isinstance(item,ast.FunctionDef) and item.name=="before_subscriber_accept")
    assert ast.unparse(admission.body[0])=="worker.live_owner()"
    assert ast.unparse(admission.body[1])=="return active_shadow_graph_guard(owned_nodes, all_topics, expected, OwnedNode, owned_node_token)"
    destroy=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="destroy_node")
    destroy_try=next(item for item in destroy.body if isinstance(item,ast.Try))
    assert ast.unparse(destroy_try.finalbody[0])=="self._hh_graph_active = False"
    load=next(item for item in run.body if isinstance(item,ast.FunctionDef) and item.name=="audited_load")
    choice=next(part for part in ast.walk(load) if isinstance(part,ast.IfExp))
    assert [(item.arg,ast.unparse(item.value)) for item in choice.body.keywords]==[("before_accept","before_subscriber_accept")]


def test_v11_direct_loop_and_output_publish_guards_remain_frozen_V10_AST(node):
    current=ast.parse(Path(node.__file__).read_bytes())
    original=ast.parse((ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v10.py").read_bytes())
    def seams(tree):
        run=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=="run")
        branch=next(item for item in run.body if isinstance(item,ast.Try))
        direct=next(item for item in branch.body if isinstance(item,ast.If))
        owned=next(item for item in run.body if isinstance(item,ast.ClassDef) and item.name=="OwnedNode")
        publisher=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_publisher")
        publish=next(item for item in publisher.body if isinstance(item,ast.FunctionDef) and item.name=="publish")
        output_guards=[item for item in publish.body if isinstance(item,ast.If)
            and ast.unparse(item.test).startswith("topic in ('/planning/portable_e2e/shadow_trajectory'")]
        return direct.body,output_guards
    direct,publisher=seams(current);old_direct,old_publisher=seams(original)
    assert ast.dump(ast.Module(body=direct,type_ignores=[]),include_attributes=False)==ast.dump(ast.Module(body=old_direct,type_ignores=[]),include_attributes=False)
    assert len(publisher)==len(old_publisher)==2
    assert ast.dump(ast.Module(body=publisher,type_ignores=[]),include_attributes=False)==ast.dump(ast.Module(body=old_publisher,type_ignores=[]),include_attributes=False)


def test_v11_local_stage_exact_return_arguments_and_integer_chronology(node,monkeypatch):
    ticks=iter((100,135));monkeypatch.setattr(node.time,"perf_counter_ns",lambda:next(ticks))
    stages=[];result=object();argument=object();calls=[]
    def callback(value,*,named):calls.append((value,named));return result
    assert node.measured_local_stage(stages,"peer",callback,argument,named=argument) is result
    assert calls==[(argument,argument)]
    assert stages==[dict(stage="peer",begun_ns=100,finished_ns=135,duration_ns=35,
        failed_at_ns=None,failed_elapsed_ns=None,error=None)]
    assert node.compact_local_stages(stages,node.RECEIVE_STAGE_NAMES)==[[0,100,135,35]]


@pytest.mark.parametrize("exception",[RuntimeError("real callback failure"),SystemExit(2)])
def test_v11_local_stage_exception_retained_without_fabricated_finished(node,monkeypatch,exception):
    ticks=iter((100,145));monkeypatch.setattr(node.time,"perf_counter_ns",lambda:next(ticks));stages=[]
    def fail():raise exception
    with pytest.raises(type(exception)) as error:node.measured_local_stage(stages,"provenance_graph",fail)
    assert error.value is exception
    assert stages[0]["finished_ns"] is stages[0]["duration_ns"] is None
    assert stages[0]["failed_at_ns"]==145 and stages[0]["failed_elapsed_ns"]==45
    assert stages[0]["error"]==repr(exception)
    assert node.compact_local_stages(stages,node.RECEIVE_STAGE_NAMES)==[[1,100,None,None,145,45,repr(exception),None]]


@pytest.mark.parametrize("ticks",[(True,),(0,),(100,True),(100,99)])
def test_v11_local_stage_invalid_clock_never_invents_completion(node,monkeypatch,ticks):
    actual=iter(ticks);monkeypatch.setattr(node.time,"perf_counter_ns",lambda:next(actual));stages=[];calls=[]
    with pytest.raises(ValueError):node.measured_local_stage(stages,"CDR",lambda:calls.append("actual callback"))
    assert len(stages)==1 and stages[0]["finished_ns"] is None and "clock_error" in stages[0]
    if len(ticks)==1:assert calls==[] and stages[0]["begun_ns"] is None
    else:assert calls==["actual callback"] and stages[0]["begun_ns"]==100


def test_v11_local_stage_failed_clock_preserves_original_exception(node,monkeypatch):
    ticks=iter((100,99));monkeypatch.setattr(node.time,"perf_counter_ns",lambda:next(ticks));stages=[]
    original=RuntimeError("original failure")
    def fail():raise original
    with pytest.raises(RuntimeError) as error:node.measured_local_stage(stages,"callback",fail)
    assert error.value is original and stages[0]["failed_at_ns"] is None
    assert stages[0]["finished_ns"] is None and stages[0]["error"]==repr(original)


@pytest.mark.parametrize("stages,name",[((),"peer"),([{}]*8,"peer"),([],"invented_stage")])
def test_v11_local_stage_bound_and_name_before_callback(node,stages,name):
    with pytest.raises(ValueError):node.measured_local_stage(stages,name,lambda:pytest.fail("undeclared stage executed"))


def enabled_native_fixture(node,*,consume=True,callback_error=None):
    """HH_261007 - Actual fake take/execute path with identity scalars, never a native/DDS call."""
    message=object();metadata=dict(publisher_gid=b"g"*24,source_timestamp=2**61+5,
        received_timestamp=0,publication_sequence_number=2**64-1,reception_sequence_number=3)
    native_calls=[];received=[];consumed=[]
    def receive(msg,info):
        received.append((msg,info))
        if consume:consumed.append(executor.consume_native_timing(subscription,msg,info,receive))
        if callback_error:raise callback_error
        return "original callback return"
    subscription=fake_typed_subscription(receive);subscription.topic_name="/recorded/provenance"
    bridge=SimpleNamespace(take_owned=lambda sub:(native_calls.append(sub) or (message,metadata)))
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,bridge,fake_await_or_execute,native_timing_enabled=True)()
    executor.register_owned_subscription(subscription)
    return executor,subscription,message,metadata,native_calls,received,consumed


def test_v11_enabled_native_take_execute_consumes_exact_namespace_once(node):
    executor,subscription,message,metadata,calls,received,consumed=enabled_native_fixture(node)
    value=executor._take_subscription(subscription)
    assert value[0] is message and value[1] is metadata and calls==[subscription]
    assert asyncio.run(executor._execute_subscription(subscription,value))=="original callback return"
    assert received[0][0] is message and type(received[0][1]) is SimpleNamespace
    assert vars(received[0][1])==metadata and received[0][1].publisher_gid is metadata["publisher_gid"]
    assert len(consumed)==1 and [row["stage"] for row in consumed[0]]==list(node.NATIVE_STAGE_NAMES)
    assert all(type(row["begun_ns"]) is int and row["begun_ns"]>0 and row["finished_ns"]>=row["begun_ns"]
        and row["duration_ns"]==row["finished_ns"]-row["begun_ns"] for row in consumed[0])
    assert executor.native_timing_tail()["pending"]==[]
    with pytest.raises(ValueError):executor.consume_native_timing(subscription,message,received[0][1],subscription.callback)
    with pytest.raises(ValueError):asyncio.run(executor._execute_subscription(subscription,value))
    assert len(received)==len(calls)==1


@pytest.mark.parametrize("fault",["message","native_dict","unknown_take"])
def test_v11_native_execute_requires_actual_taken_message_and_original_metadata_identity(node,fault):
    executor,subscription,message,metadata,calls,received,consumed=enabled_native_fixture(node)
    value=(message,metadata) if fault=="unknown_take" else executor._take_subscription(subscription)
    if fault=="message":value=(object(),metadata)
    elif fault=="native_dict":value=(message,dict(metadata))
    with pytest.raises(ValueError):asyncio.run(executor._execute_subscription(subscription,value))
    assert received==consumed==[] and len(calls)==(0 if fault=="unknown_take" else 1)
    tail=executor.native_timing_tail()
    assert len(tail["pending"])==(0 if fault=="unknown_take" else 1)


@pytest.mark.parametrize("fault",["subscription","message","info","callback"])
def test_v11_native_consumption_rejects_equal_but_not_actual_objects(node,fault):
    executor,subscription,message,metadata,calls,received,consumed=enabled_native_fixture(node,consume=False)
    value=executor._take_subscription(subscription);asyncio.run(executor._execute_subscription(subscription,value))
    info=received[0][1];callback=subscription.callback;actual_subscription=subscription
    if fault=="subscription":actual_subscription=fake_typed_subscription(callback)
    elif fault=="message":message=object()
    elif fault=="info":info=SimpleNamespace(**vars(info))
    else:callback=lambda *args:None
    with pytest.raises(ValueError):executor.consume_native_timing(actual_subscription,message,info,callback)
    assert len(executor.native_timing_tail()["pending"])==1 and consumed==[]
    assert len(calls)==len(received)==1


def test_v11_pending_native_take_cannot_overwrite_or_exceed_32(node):
    count=[];bridge=SimpleNamespace(take_owned=lambda sub:(count.append(sub) or (object(),dict(publisher_gid=b"g"*24))))
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,bridge,fake_await_or_execute,native_timing_enabled=True)()
    subscriptions=[]
    for number in range(33):
        sub=fake_typed_subscription(lambda *args:None);sub.topic_name="/synthetic/"+str(number)
        executor.register_owned_subscription(sub);subscriptions.append(sub)
    for sub in subscriptions[:32]:executor._take_subscription(sub)
    with pytest.raises(ValueError):executor._take_subscription(subscriptions[0])
    with pytest.raises(ValueError):executor._take_subscription(subscriptions[32])
    assert len(count)==32 and len(executor.native_timing_tail()["pending"])==32


def test_v11_native_failure_full_scalar_tail_without_fake_success(node):
    original=RuntimeError("actual fake native bridge error");calls=[]
    def take(sub):calls.append(sub);raise original
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=take),fake_await_or_execute,native_timing_enabled=True)()
    sub=fake_typed_subscription(lambda *args:pytest.fail("failed native take delivered callback"));sub.topic_name="/recorded/provenance"
    executor.register_owned_subscription(sub)
    with pytest.raises(RuntimeError) as error:executor._take_subscription(sub)
    assert error.value is original and calls==[sub]
    tail=executor.native_timing_tail();assert tail["pending"]==[] and len(tail["failures"])==1
    stage=tail["failures"][0]["stages"][0]
    assert stage["error"]==repr(original) and stage["finished_ns"] is None
    json.dumps(tail,allow_nan=False)


def test_v11_native_callback_exception_preserves_original_and_consumed_stages(node):
    original=RuntimeError("original callback failure")
    executor,subscription,message,metadata,calls,received,consumed=enabled_native_fixture(node,callback_error=original)
    value=executor._take_subscription(subscription)
    with pytest.raises(RuntimeError) as error:asyncio.run(executor._execute_subscription(subscription,value))
    assert error.value is original and len(received)==len(consumed)==len(calls)==1
    assert len(consumed[0])==3 and executor.native_timing_tail()["pending"]==[]


@pytest.mark.parametrize("enabled",[None,1,"subscriber"])
def test_v11_native_timing_factory_exact_boolean_no_implicit_enable(node,enabled):
    with pytest.raises(ValueError):node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(),fake_await_or_execute,native_timing_enabled=enabled)


def test_v11_empty_native_take_and_unowned_callbacks_keep_genuine_behavior(node):
    native_calls=[]
    executor=node.metadata_executor_type(FakeSingleThreadedExecutor,SimpleNamespace(take_owned=lambda sub:(native_calls.append(sub) or None)),fake_await_or_execute,native_timing_enabled=True)()
    sub=fake_typed_subscription(lambda *args:pytest.fail("empty take produced callback"));sub.topic_name="/recorded/provenance"
    executor.register_owned_subscription(sub)
    assert executor._take_subscription(sub) is None
    assert asyncio.run(executor._execute_subscription(sub,None)) is None
    tail=executor.native_timing_tail()
    assert native_calls==[sub] and tail["pending"]==tail["failures"]==[]
    assert tail["empty_native_takes_profiled"] is False and tail["native_fields_modified"] is False
    received=[];foreign=SimpleNamespace(callback=lambda message:received.append(message))
    assert executor._take_subscription(foreign)=="default message"
    asyncio.run(executor._execute_subscription(foreign,"genuine default callback"))
    assert received==["genuine default callback"] and native_calls==[sub]
    assert type(executor)._execute_timer is FakeSingleThreadedExecutor._execute_timer
    assert type(executor)._execute_service is FakeSingleThreadedExecutor._execute_service


def local_receive_row_fixture(node,topic):
    """HH_261007 - Synthetic stage scalars, not measurements of DDS network or actual model latency."""
    native=[];receive=[]
    for name in node.NATIVE_STAGE_NAMES:node.measured_local_stage(native,name,lambda:None)
    for name in node.RECEIVE_STAGE_NAMES:node.measured_local_stage(receive,name,lambda:None)
    return dict(topic=topic,native_stages=native,stages=receive,CDR_sha256="a"*64,observed_stamps_ns=[123],
        observed_provenance=dict(ordinal=17,timestamp_ns=123),provenance_graph_full_census_performed=topic.endswith("/provenance"))


def test_v11_all19_actual_observed_topic_rows_and_partial_tail_not_padded(node):
    pure=SimpleNamespace(INPUT_PREFIX="/portable_e2e/recorded_replay",
        PROVENANCE_TOPIC="/portable_e2e/recorded_replay/provenance",ACK_TOPIC="/portable_e2e/recorded_replay/shadow_receipt",
        ROLES=("CAM_FRONT","CAM_BACK","CAM_FRONT_LEFT","CAM_BACK_LEFT","CAM_FRONT_RIGHT","CAM_BACK_RIGHT"))
    incoming,_=node.topic_sets(pure);assert len(incoming)==19
    rows=[local_receive_row_fixture(node,topic) for topic in sorted(incoming)]
    before=deepcopy(rows);receipt=dict(ordinal=17,timestamp_ns=123,
        source_selected_callback_provenance=dict(ordinal=17,timestamp_ns=123))
    hashes={row["topic"]:row["CDR_sha256"] for row in rows}
    compact=node.compact_receive_timing(rows,receipt,hashes)
    assert rows==before and compact["observed_count"]==len(compact["messages"])==19
    assert {row["topic"] for row in compact["messages"]}==incoming
    assert all(row["ordinal"]==17 and row["anchor_binding"]!="UNKNOWN" and len(row["receive_stages"])==7
        and len(row["native_stages"])==3 for row in compact["messages"])
    assert compact["padded_missing_messages"] is False and compact["local_clock_only"] is True
    assert compact["pure_DDS_network_delay_claimed"] is compact["stage_sum_is_wall_latency_claimed"] is False
    partial=node.compact_receive_timing(rows[:2],{},{} )
    assert partial["observed_count"]==len(partial["messages"])==2 and partial["padded_missing_messages"] is False
    assert all(row["ordinal"] is None and row["anchor_binding"]=="UNKNOWN" for row in partial["messages"])
    assert node.compact_receive_timing([],{},{} )["messages"]==[]


@pytest.mark.parametrize("fault",["provenance_ordinal","provenance_stamp","observed_stamp","no_admission","wrong_CDR","no_observed_provenance","other_observed_provenance"])
def test_v11_receive_timing_anchor_binding_only_original_admission_and_actual_CDR(node,fault):
    row=local_receive_row_fixture(node,"/recorded/provenance")
    receipt=dict(ordinal=17,timestamp_ns=123,source_selected_callback_provenance=dict(ordinal=17,timestamp_ns=123))
    hashes={row["topic"]:row["CDR_sha256"]}
    if fault=="provenance_ordinal":receipt["source_selected_callback_provenance"]["ordinal"]+=1
    elif fault=="provenance_stamp":receipt["source_selected_callback_provenance"]["timestamp_ns"]+=1
    elif fault=="observed_stamp":row["observed_stamps_ns"]=[124]
    elif fault=="no_admission":receipt.pop("source_selected_callback_provenance")
    elif fault=="no_observed_provenance":row["observed_provenance"]=None
    elif fault=="other_observed_provenance":row["observed_provenance"]=dict(ordinal=16,timestamp_ns=123)
    else:hashes[row["topic"]]="b"*64
    if fault in ("no_admission","wrong_CDR","no_observed_provenance","other_observed_provenance"):
        result=node.compact_receive_timing([row],receipt,hashes)
        assert result["messages"][0]["anchor_binding"]=="UNKNOWN" and result["messages"][0]["ordinal"] is None
    else:
        with pytest.raises(ValueError):node.compact_receive_timing([row],receipt,hashes)


@pytest.mark.parametrize("field,value",[("stamp",True),("stamp",1.0),("stamp",-1),("ordinal",True),("provenance_stamp",True)])
def test_v11_profile_stamp_and_original_provenance_exact_integer_ABI(node,field,value):
    row=local_receive_row_fixture(node,"/recorded/provenance")
    receipt=dict(ordinal=17,timestamp_ns=123,source_selected_callback_provenance=dict(ordinal=17,timestamp_ns=123))
    if field=="stamp":row["observed_stamps_ns"]=[value]
    elif field=="ordinal":receipt["source_selected_callback_provenance"]["ordinal"]=value
    else:receipt["source_selected_callback_provenance"]["timestamp_ns"]=value
    with pytest.raises(ValueError):node.compact_receive_timing([row],receipt,{row["topic"]:row["CDR_sha256"]})


def test_v11_native_pending_delivery_not_replayed_before_consumption(node):
    executor,subscription,message,metadata,calls,received,consumed=enabled_native_fixture(node,consume=False)
    value=executor._take_subscription(subscription);asyncio.run(executor._execute_subscription(subscription,value))
    with pytest.raises(ValueError):asyncio.run(executor._execute_subscription(subscription,value))
    assert len(calls)==len(received)==1 and consumed==[]
    assert executor.native_timing_tail()["pending"][0]["native_info_delivered"] is True
    executor.consume_native_timing(subscription,message,received[0][1],subscription.callback)
    assert executor.native_timing_tail()["pending"]==[]


def test_v11_source_local_stages_memory_only_ACK_wire_cost_and_failed_tail_inside_full_clock(node):
    tree=ast.parse(Path(node.__file__).read_bytes())
    run=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=="run")
    executor_calls=[part for part in ast.walk(run) if isinstance(part,ast.Call) and ast.unparse(part.func)=="metadata_executor_type"]
    assert len(executor_calls)==1
    assert [(key.arg,ast.unparse(key.value)) for key in executor_calls[0].keywords]==[("native_timing_enabled","mode == 'subscriber'")]
    owned=next(item for item in run.body if isinstance(item,ast.ClassDef) and item.name=="OwnedNode")
    subscriber=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_subscription")
    receive=next(item for item in subscriber.body if isinstance(item,ast.FunctionDef) and item.name=="receive")
    source=ast.unparse(receive)
    assert source.index("consume_native_timing(")<source.index("invoke('peer', self.peer)")<source.index("invoke('callback', callback, msg)")
    calls=[ast.unparse(part.func) for part in ast.walk(receive) if isinstance(part,ast.Call)]
    assert not any(name in calls for name in ("worker.append","worker.new_json","open","tensor_receipt","model.forward_cached"))
    publisher=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_publisher")
    publish=next(item for item in publisher.body if isinstance(item,ast.FunctionDef) and item.name=="publish")
    source=ast.unparse(publish)
    assert source.index("materialize_owned_ACK(")<source.index("compact_receive_timing(")<source.index("msg.data = pure.encode(value).decode()")<source.index("raw = serialize_message(msg)")<source.index("real_publish(msg)")<source.index("finished = time.perf_counter_ns()")
    assert "actual_full_publication_seconds=(finished - self.publisher_begin) / 1000000000.0" in source
    assert "actual_full_publication_deadline_passed=finished - self.publisher_begin <= 100000000" in source
    assert "owned_received_input_stage_timing" in source
    body=ast.unparse(run)
    assert "_receive_stage_tail.json" in body and "full_ACK_includes_instrumentation=True" in body
    assert "close_attempt(runtime_close_records, 'native_timing_release', executor.release_native_timing)" in body and "whole_interval_profile_claimed=False" in body


@pytest.mark.parametrize("field,value",[("begun_ns",True),("begun_ns",0),("finished_ns",True),
    ("finished_ns",99),("duration_ns",True),("duration_ns",None),("duration_ns",3)])
def test_v11_compact_success_clock_ABI_never_accepts_bool_or_invented_duration(node,field,value):
    stage=dict(stage="peer",begun_ns=100,finished_ns=110,duration_ns=10,failed_at_ns=None,failed_elapsed_ns=None,error=None)
    stage[field]=value
    with pytest.raises(ValueError):node.compact_local_stages([stage],node.RECEIVE_STAGE_NAMES)


@pytest.mark.parametrize("field,value",[("failed_at_ns",True),("failed_at_ns",99),("failed_elapsed_ns",True),
    ("failed_elapsed_ns",None),("failed_elapsed_ns",3)])
def test_v11_compact_failure_clock_ABI_preserves_unknown_not_fake_elapsed(node,field,value):
    stage=dict(stage="peer",begun_ns=100,finished_ns=None,duration_ns=None,failed_at_ns=110,failed_elapsed_ns=10,error="actual fixture failure")
    stage[field]=value
    with pytest.raises(ValueError):node.compact_local_stages([stage],node.RECEIVE_STAGE_NAMES)


def test_v11_provenance_census_flag_only_after_actual_helper_completion_source(node):
    tree=ast.parse(Path(node.__file__).read_bytes())
    run=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=="run")
    owned=next(item for item in run.body if isinstance(item,ast.ClassDef) and item.name=="OwnedNode")
    subscribe=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_subscription")
    receive=next(item for item in subscribe.body if isinstance(item,ast.FunctionDef) and item.name=="receive")
    source=ast.unparse(receive)
    assert "provenance_graph_full_census_performed=False" in source
    census=source.index("invoke('provenance_graph', receive_graph_guard")
    marked=source.index("row['provenance_graph_full_census_performed'] = topic == pure.PROVENANCE_TOPIC")
    assert census<marked<source.index("invoke('publisher_gid', authenticated_gid")


def test_v11_conservative_source_ABI_ACK_serialization_width_no_truncation(node,worker):
    # HH_261007 - Source-derived shapes with synthetic wide finite values; not actual NN, DDS or wire evidence.
    model_tree=ast.parse((ROOT/"learning/portable_e2e/temporal_model_v2.py").read_bytes())
    config=next(item for item in model_tree.body if isinstance(item,ast.ClassDef) and item.name=="TemporalModelConfigV2")
    knots=next(item.value.value for item in config.body if isinstance(item,ast.AnnAssign)
        and isinstance(item.target,ast.Name) and item.target.id=="spatial_knots")
    intents=next(len(item.value.elts) for item in model_tree.body if isinstance(item,ast.Assign)
        and any(isinstance(target,ast.Name) and target.id=="INTENT_NAMES_V2" for target in item.targets))
    assert knots==33 and intents==3
    shapes=dict(curvature_knots=[1,33],preferred_speed_knots=[1,33],intent_logits=[1,3],stop_distance_raw=[1,1],acceleration_prediction=[1,1])
    assert any(isinstance(part,ast.Dict) and [key.value for key in part.keys if isinstance(key,ast.Constant)]==list(shapes)
        for part in ast.walk(model_tree))
    wide=-1.1754943508222875e-38
    raw={name:[[wide]*shape[1]] for name,shape in shapes.items()}
    selected=dict(candidate_index=0,xy_base_m=[[wide,wide] for _ in range(64)],speed_mps=[wide]*64,
        heading_rad=[wide]*64,timestep_s=wide,logit_margin=None,planar_extent_m=wide)
    pure=SimpleNamespace(INPUT_PREFIX="/portable_e2e/recorded_replay",
        PROVENANCE_TOPIC="/portable_e2e/recorded_replay/provenance",ACK_TOPIC="/portable_e2e/recorded_replay/shadow_receipt",
        ROLES=("CAM_FRONT","CAM_BACK","CAM_FRONT_LEFT","CAM_BACK_LEFT","CAM_FRONT_RIGHT","CAM_BACK_RIGHT"))
    topics,_=node.topic_sets(pure)
    cameras=[dict(role=role,native_frame=2**63-1,receipt_sequence=2**63-1,
        original_process_image_object_id=2**63-1,actor_id=2**63-1,source_BGRA_encoding_prefixed_sha256="a"*64,
        jpeg_path="images/"+role+"/0000000917.jpg",jpeg_sha256="b"*64) for role in pure.ROLES]
    proof=dict(schema="portable_e2e.recorded_JPEG_ROS_shadow.20261005.v1.bundle",ordinal=917,native_frame=2**63-1,
        timestamp_ns=2**63-1,episode_id="temporal_v2_temporal_route_joint_town03_known_dev_straight_full209_v5",
        split="val",rig_id="c"*64,stream_source_sha256="d"*64,cameras=cameras,
        recorded_JPEG_only=True,original_BGRA_parity_claimed=False)
    rows=[local_receive_row_fixture(node,topic) for topic in sorted(topics)]
    for row in rows:
        row["observed_provenance"]=proof;row["observed_stamps_ns"]=[2**63-1]
        for stage in row["native_stages"]+row["stages"]:
            stage.update(begun_ns=2**62,finished_ns=2**63-1,duration_ns=2**62-1)
    info,message_type=native_witness_fixture();witness=node.native_transport_witness(info,"/owned",message_type)
    receipt=dict(schema=worker.SCHEMA+".shadow_receipt",ordinal=917,native_frame=2**63-1,timestamp_ns=2**63-1,
        status="TRAJECTORY",selected=selected,source_selected_callback_provenance=proof,
        policy_diagnostics=dict(raw_output=raw,decoded_output=dict(candidate_xy=[selected["xy_base_m"]],
            candidate_speed=[selected["speed_mps"]],candidate_logits=[wide],instantaneous_speed=[wide]*64,arc_distance=[wide]*64,intent=0),
            owned_recorded_numerical_event=dict(raw_output=raw,tensor_sha256={"tensor_"+str(i):dict(dtype="float32",shape=[1,5,6,3,180,320],sha256="e"*64) for i in range(12)},
                raw_tensor_sha256={name:dict(dtype="float32",shape=shape,sha256="f"*64) for name,shape in shapes.items()})),
        actual_received_input_CDR_sha256={topic:"a"*64 for topic in topics},
        actual_received_input_native_transport={topic:witness for topic in topics},actual_DDS_ACK_transport=witness,
        synthetic_serialization_bound=True,actual_wire_measurement=False,P3_approved=False)
    receipt["owned_received_input_stage_timing"]=node.compact_receive_timing(rows,receipt,receipt["actual_received_input_CDR_sha256"])
    encoded=worker.encode(receipt)
    assert len(encoded)<128*1024 and json.loads(encoded)==receipt
    assert receipt["owned_received_input_stage_timing"]["observed_count"]==19


@pytest.mark.parametrize("exception",[RuntimeError("original callback error"),SystemExit(7)])
def test_v11_callback_error_not_replaced_by_failed_failure_clock_read(node,monkeypatch,exception):
    clock_error=OSError("clock failed after original error");calls=[]
    def clock():
        calls.append("clock")
        if len(calls)==1:return 100
        raise clock_error
    monkeypatch.setattr(node.time,"perf_counter_ns",clock);stages=[]
    def fail():raise exception
    with pytest.raises(type(exception)) as error:node.measured_local_stage(stages,"callback",fail)
    assert error.value is exception and stages[0]["error"]==repr(exception)
    assert stages[0]["begun_ns"]==100 and stages[0]["failed_at_ns"] is stages[0]["finished_ns"] is None
    assert stages[0]["failed_elapsed_ns"] is stages[0]["duration_ns"] is None
    assert repr(clock_error) in stages[0]["clock_error"]


@pytest.mark.parametrize("phase",["start","completion"])
def test_v11_clock_read_exception_retains_unknown_end_and_exact_error(node,monkeypatch,phase):
    clock_error=OSError("actual clock unavailable");reads=[];callbacks=[]
    def clock():
        reads.append("read")
        if phase=="start" or len(reads)>1:raise clock_error
        return 100
    monkeypatch.setattr(node.time,"perf_counter_ns",clock);stages=[]
    with pytest.raises(OSError) as error:node.measured_local_stage(stages,"peer",lambda:callbacks.append("actual callback"))
    assert error.value is clock_error and len(stages)==1
    assert callbacks==([] if phase=="start" else ["actual callback"])
    assert stages[0]["begun_ns"]==(None if phase=="start" else 100)
    assert stages[0]["finished_ns"] is stages[0]["duration_ns"] is stages[0]["failed_at_ns"] is None
    assert repr(clock_error) in stages[0]["clock_error"]


@pytest.mark.parametrize("fault",["invalid_stamp","invalid_clock","nan_stamp","infinite_clock"])
def test_v11_closing_invalid_raw_timing_retained_but_never_accepted_as_measurement(node,fault):
    row=local_receive_row_fixture(node,"/recorded/provenance");rows=[row]
    if fault=="invalid_stamp":row["observed_stamps_ns"]=[True]
    elif fault=="invalid_clock":row["stages"][0]["begun_ns"]=None
    elif fault=="nan_stamp":row["observed_stamps_ns"]=[float("nan")]
    else:row["stages"][0]["finished_ns"]=float("inf")
    tail=node.receive_timing_closing_tail(rows)
    assert len(rows)==1 and rows[0] is row and tail["timing"] is None
    assert tail["compaction_error"] is not None and tail["raw_only_not_valid_timing"] is True
    assert tail["anchor_binding"]=="UNKNOWN" and tail["whole_interval_profile_claimed"] is False
    raw=tail["raw_observed_timing_rows"]
    assert len(raw)==1 and raw[0]["topic"]==row["topic"]
    if fault=="nan_stamp":
        assert raw[0]["observed_stamps_ns"][0]==dict(raw_type="float",raw_repr="nan",accepted_as_timing=False)
    elif fault=="infinite_clock":
        assert raw[0]["stages"][0]["finished_ns"]==dict(raw_type="float",raw_repr="inf",accepted_as_timing=False)
    json.dumps(tail,allow_nan=False)


def test_v11_valid_closing_tail_compacts_unknown_anchor_without_mutating_rows(node):
    row=local_receive_row_fixture(node,"/recorded/provenance");rows=[row];before=deepcopy(rows)
    tail=node.receive_timing_closing_tail(rows)
    assert rows==before and tail["compaction_error"] is None
    assert tail["timing"]["messages"][0]["anchor_binding"]=="UNKNOWN"
    assert tail["timing"]["observed_count"]==1


def test_v11_failure_ACK_inside_callback_retains_active_stage_until_actual_return(node):
    row=local_receive_row_fixture(node,"/recorded/provenance");row["stages"]=row["stages"][:6]
    finished_row=local_receive_row_fixture(node,"/recorded/previous");rows=[finished_row,row]
    receipt=dict(ordinal=17,timestamp_ns=123,status="FAILED_RETAINED")
    snapshots=[];journal=[]
    def callback():
        # HH_261007 - Simulate only ordering: the immutable ACK snapshot cannot invent this callback's future finish.
        snapshots.append(node.compact_receive_timing(rows,receipt,{}))
        journal.append("real ACK journal written")
        node.retain_unfinished_receive_rows(rows)
        assert len(rows)==1 and rows[0] is row
        assert row["stages"][-1]["finished_ns"] is row["stages"][-1]["failed_at_ns"] is None
        return "original callback result"
    assert node.measured_local_stage(row["stages"],"callback",callback)=="original callback result"
    assert journal==["real ACK journal written"] and len(rows)==1
    snapshot=snapshots[0]["messages"][-1]["receive_stages"][-1]
    assert snapshot[0]==6 and snapshot[2] is snapshot[3] is None
    assert row["stages"][-1]["finished_ns"] is not None and row["stages"][-1]["duration_ns"] is not None
    closing=node.receive_timing_closing_tail(rows)
    assert closing["compaction_error"] is None and closing["timing"]["messages"][0]["receive_stages"][-1][2] is not None
    node.retain_unfinished_receive_rows(rows)
    assert rows==[]


def test_v11_source_raw_closing_written_before_release_and_ACK_journal_before_active_retention(node):
    tree=ast.parse(Path(node.__file__).read_bytes())
    run=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=="run")
    main_try=next(item for item in run.body if isinstance(item,ast.Try))
    closing=main_try.finalbody[0];source=ast.unparse(closing)
    assert source.index("receive_timing_closing_tail(")<source.index("worker.new_json(")<source.index("executor.release_native_timing")<source.index("value._hh_receive_timing.clear")
    assert "tail['compaction_error'] is not None" in source and "timing tail validation:" in source
    owned=next(item for item in run.body if isinstance(item,ast.ClassDef) and item.name=="OwnedNode")
    factory=next(item for item in owned.body if isinstance(item,ast.FunctionDef) and item.name=="create_publisher")
    publish=next(item for item in factory.body if isinstance(item,ast.FunctionDef) and item.name=="publish")
    source=ast.unparse(publish)
    assert source.index("worker.append(records,")<source.index("self._hh_receive_journaled = True")<source.index("retain_unfinished_receive_rows(self._hh_receive_timing)")
    assert "self._hh_receive_timing.clear()" not in source


class PublicMembershipNode:
    """HH_261008 - Mirror installed public setter/remove behavior, including stale weak membership after remove_node."""
    def __init__(self,name="portable_temporal_recorded_shadow",**kwargs):
        self.owned_name=name;self._executor=None;self._hh_graph_active=True;self.destroy_calls=0;self.destroy_failure=None
    @property
    def executor(self):return self._executor
    @executor.setter
    def executor(self,value):
        current=self._executor
        if current is value:return
        if current is not None:current.remove_node(self)
        if value is not None:value.add_node(self)
        self._executor=value
    def destroy_node(self):
        self.destroy_calls+=1
        if self.destroy_failure is not None:raise self.destroy_failure
        self._hh_graph_active=False;return "genuine fake destroy result"


class PublicMembershipExecutor:
    def __init__(self):self.nodes=[];self.adds=0;self.spins=0;self.removes=0;self.shutdowns=0;self.failure=None;self.shutdown_result=True
    def get_nodes(self):return list(self.nodes)
    def add_node(self,node):
        if any(node is value for value in self.nodes):return False
        self.nodes.append(node);self.adds+=1;node.executor=self;return True
    def remove_node(self,node):
        self.nodes[:]=[value for value in self.nodes if value is not node];self.removes+=1
    def spin_once(self,*,timeout_sec=None):
        self.spins+=1
        if self.failure is not None:raise self.failure
        return timeout_sec
    def shutdown(self,*,timeout_sec=None):
        assert timeout_sec==0;self.shutdowns+=1
        if self.shutdown_result is not True:return self.shutdown_result
        self.nodes.clear();return True


def persistent_fixture(node,*,bind=True,subclass=True):
    class Owned(PublicMembershipNode):pass
    class GenuineShadow(Owned):pass
    token=object();actual=(GenuineShadow if subclass else Owned)();actual._hh_graph_owner_token=token
    owned=[actual];executor=PublicMembershipExecutor();live=[]
    membership=node.OwnedExecutorMembership(executor,Owned,token,owned,lambda:live.append("owner_generation_guard"))
    if bind:membership.bind_actual_node(actual)
    return actual,Owned,token,owned,executor,membership,live


def test_v12_genuine_subclass_exact_factory_object_three_spins_register_once(node):
    actual,Owned,token,owned,executor,membership,live=persistent_fixture(node)
    original=SimpleNamespace(spin_once=lambda *a,**k:pytest.fail("global executor called"),_global_executor=object())
    private=node.private_rclpy_view(original,executor,membership=membership)
    assert type(actual) is not Owned and isinstance(actual,Owned)
    for _ in range(3):assert private.spin_once(actual,timeout_sec=.02)==.02
    assert executor.adds==1 and executor.spins==3 and executor.removes==0 and len(live)==4
    assert membership.registered[id(actual)] is actual and membership.actual_type is type(actual)
    assert original._global_executor is not executor
    membership.detach(actual);membership.finish()
    assert actual.executor is None and executor.get_nodes()==[] and membership.registered=={}


@pytest.mark.parametrize("fault",["token","destroyed","foreign_executor","duplicate_membership","foreign_membership","unbound_probe","concrete_type","second_binding"])
def test_v12_wrong_object_token_lifetime_foreign_and_duplicate_fail_before_spin(node,fault):
    actual,Owned,token,owned,executor,membership,live=persistent_fixture(node,bind=fault!="unbound_probe")
    if fault=="token":actual._hh_graph_owner_token=object()
    elif fault=="destroyed":actual._hh_graph_active=False
    elif fault=="foreign_executor":actual.executor=PublicMembershipExecutor()
    elif fault=="duplicate_membership":executor.nodes[:]=[actual,actual]
    elif fault=="foreign_membership":executor.nodes[:]=[PublicMembershipNode()]
    elif fault=="concrete_type":
        class Other(Owned):pass
        actual.__class__=Other
    elif fault=="second_binding":
        with pytest.raises(ValueError):membership.bind_actual_node(actual)
        assert executor.spins==0;return
    with pytest.raises(ValueError):membership.spin_once(actual,timeout_sec=.02)
    assert executor.spins==executor.adds==0


def test_v12_foreign_object_not_source_registry_instance_and_callback_exception_retained(node):
    actual,Owned,token,owned,executor,membership,live=persistent_fixture(node)
    foreign=type(actual)();foreign._hh_graph_owner_token=token
    with pytest.raises(ValueError):membership.spin_once(foreign)
    original=RuntimeError("original callback failure");executor.failure=original
    with pytest.raises(RuntimeError) as raised:membership.spin_once(actual)
    assert raised.value is original and executor.adds==1 and executor.removes==0 and executor.nodes==[actual]
    actual._hh_graph_active=False;membership.detach(actual);membership.finish()
    assert actual.executor is None and not executor.nodes


def test_v12_cross_role_close_all_publisher_nodes_with_outer_None_and_stale_public_ref(node):
    class Owned(PublicMembershipNode):pass
    token=object();publisher=Owned("portable_temporal_recorded_publisher");publisher._hh_graph_owner_token=token
    executor=PublicMembershipExecutor();executor.add_node(publisher);executor.remove_node(publisher)
    assert publisher.executor is executor and executor.nodes==[]
    records=[];context=[];closed=[]
    node.close_owned_runtime([publisher],None,Owned,token,[executor],closed,None,lambda:context.append("shutdown"),records)
    assert publisher.executor is None and publisher.destroy_calls==1 and executor.shutdowns==1 and context==["shutdown"]
    assert all(value["success"] for value in records) and closed==[executor]


def test_v12_close_subscriber_probe_and_actual_and_direct_outer_without_global_membership(node):
    actual,Owned,token,owned,executor,membership,live=persistent_fixture(node)
    probe=Owned();probe._hh_graph_owner_token=token;probe._hh_graph_active=False;owned.insert(0,probe)
    membership.spin_once(actual)
    records=[];closed=[];node.close_owned_runtime(owned,actual,Owned,token,[executor],closed,membership,lambda:None,records)
    assert actual.executor is probe.executor is None and actual.destroy_calls==1 and probe.destroy_calls==0
    assert membership.closed and not membership.registered and executor.shutdowns==1
    direct=PublicMembershipNode("portable_temporal_recorded_direct");another=PublicMembershipExecutor();another.add_node(direct);another.remove_node(direct)
    node.close_owned_runtime([],direct,Owned,token,[another],[],None,lambda:None,records)
    assert direct.executor is None and direct.destroy_calls==1 and all(r["success"] for r in records)


@pytest.mark.parametrize("fault",["destroy","shutdown","context","foreign_token"])
def test_v12_close_records_failure_and_continues_all_owned_cleanup(node,fault):
    class Owned(PublicMembershipNode):pass
    token=object();actual=Owned();actual._hh_graph_owner_token=token;executor=PublicMembershipExecutor()
    if fault=="destroy":actual.destroy_failure=RuntimeError("destroy failure")
    elif fault=="shutdown":executor.shutdown_result=False
    elif fault=="foreign_token":actual._hh_graph_owner_token=object()
    def context():
        if fault=="context":raise RuntimeError("context close failure")
    records=[];node.close_owned_runtime([actual],None,Owned,token,[executor],[],None,context,records)
    assert any(row["success"] is False for row in records) and executor.shutdowns==1
    assert records[-1]["event"]=="rclpy_context_shutdown"
    assert actual.destroy_calls==(0 if fault=="foreign_token" else 1)


def v12_owned_class_fixture(node,worker):
    run=next(part for part in ast.parse(Path(node.__file__).read_bytes()).body if isinstance(part,ast.FunctionDef) and part.name=="run")
    cls=next(part for part in run.body if isinstance(part,ast.ClassDef) and part.name=="OwnedNode")
    records=[];owned=[];token=object();executor=PublicMembershipExecutor()
    namespace=dict(real_node=PublicMembershipNode,worker=worker,expected={"portable_temporal_recorded_shadow"},owned_node_token=token,
        owned_nodes=owned,mode="subscriber",all_topics=set(),graph_guard=lambda *a:None,membership=None,owned_executors=[executor],
        runtime_close_records=records,detach_private_node=node.detach_private_node,__name__="_hh_v12_owned_class_fake")
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(node.__file__),"exec"),namespace)
    return namespace["OwnedNode"],records,owned,token,executor,namespace


def test_v12_genuine_owned_destroy_public_detach_and_both_exceptions_recorded(node,worker):
    Owned,records,owned,token,executor,namespace=v12_owned_class_fixture(node,worker)
    actual=Owned("portable_temporal_recorded_shadow");executor.add_node(actual)
    assert actual.destroy_node()=="genuine fake destroy result" and actual.executor is None and not actual._hh_graph_active
    second=Owned("portable_temporal_recorded_shadow");foreign=PublicMembershipExecutor();second.executor=foreign
    second.destroy_failure=RuntimeError("actual destruction failure")
    with pytest.raises(ValueError,match="foreign executor"):second.destroy_node()
    assert not second._hh_graph_active and second.destroy_calls==1
    assert any("foreign executor" in str(row["error"]) for row in records) and any("actual destruction failure" in str(row["error"]) for row in records)
    assert foreign.shutdowns==0 and second.executor is foreign


@pytest.mark.parametrize("fault",[None,"probe_graph","probe_destroy","probe_shutdown"])
def test_v12_actual_probe_AST_public_close_then_fresh_executor_or_retained_exception_without_imports(node,worker,fault):
    # HH_261008 - Execute ONLY the genuine subscriber startup branch with fake providers; never its later Torch/model import.
    Owned,records,owned,token,executor,namespace=v12_owned_class_fixture(node,worker)
    run=next(part for part in ast.parse(Path(node.__file__).read_bytes()).body if isinstance(part,ast.FunctionDef) and part.name=="run")
    load=next(part for part in run.body if isinstance(part,ast.FunctionDef) and part.name=="audited_load")
    branch=next(part for part in load.body if isinstance(part,ast.If));ticks=iter((0.,.1,.2,1.));created=[];calls=[]
    original=RuntimeError("original startup probe failure")
    def graph(value,*args):
        calls.append(value)
        if len(calls)==1 and fault=="probe_destroy":value.destroy_failure=original
        if len(calls)>1 and fault=="probe_graph":raise original
    def factory(**kwargs):
        fresh=PublicMembershipExecutor();created.append(fresh);return fresh
    context=SimpleNamespace(get_default_context=lambda:object());ros=SimpleNamespace(Node=Owned,rclpy=node.private_rclpy_view(context,executor))
    namespace.update(executor=executor,membership=None,closed_executors=[],ros=ros,original_rclpy=context,
        MetadataExecutor=factory,OwnedExecutorMembership=node.OwnedExecutorMembership,private_rclpy_view=node.private_rclpy_view,
        close_attempt=node.close_attempt,shutdown_private=node.shutdown_private,graph_guard=graph,time=SimpleNamespace(monotonic=lambda:next(ticks)))
    if fault=="probe_shutdown":executor.shutdown_result=False
    worker_owner=worker.live_owner;worker.live_owner=lambda:None
    try:
        if fault is None:exec(compile(ast.Module(body=[branch],type_ignores=[]),str(node.__file__),"exec"),namespace)
        else:
            with pytest.raises(RuntimeError if fault=="probe_graph" else ValueError) as raised:
                exec(compile(ast.Module(body=[branch],type_ignores=[]),str(node.__file__),"exec"),namespace)
            if fault=="probe_graph":assert raised.value is original
    finally:worker.live_owner=worker_owner
    assert len(owned)==1 and owned[0].executor is None and owned[0]._hh_graph_active is False and executor.shutdowns==1
    if fault is None:
        assert len(created)==1 and namespace["executor"] is created[0] and namespace["membership"].executor is created[0]
        assert namespace["membership"].actual_node is None and namespace["closed_executors"]==[executor]
        assert all(row["success"] for row in records)
    else:
        assert created==[] and namespace["membership"] is None
        if fault!="probe_graph":assert any(row["success"] is False for row in records)
    assert "torch" not in sys.modules and "rclpy" not in sys.modules


def test_v12_source_probe_shutdown_fresh_executor_before_factory_bind_and_role_cleanup(node):
    run=next(part for part in ast.parse(Path(node.__file__).read_bytes()).body if isinstance(part,ast.FunctionDef) and part.name=="run")
    load=next(part for part in run.body if isinstance(part,ast.FunctionDef) and part.name=="audited_load")
    text=ast.unparse(load)
    assert text.index("shutdown_private(executor,")<text.index("executor = MetadataExecutor(")<text.index("membership = OwnedExecutorMembership(")<text.index("genuine_load(*a)")
    assert "probe_error = failure" in text and "probe_error is None" in text
    body=ast.unparse(run)
    assert body.index("node = ns['recorded_node']")<body.index("membership.bind_actual_node(node)")<body.index("subscriber_ready.json")
    assert "close_owned_runtime(owned_nodes, node, OwnedNode" in body and "runtime_close_records=runtime_close_records" in body
    assert "._cb_iter" not in body and "._nodes" not in body


def test_v12_all_original_scientific_functions_direct_and_publisher_AST_frozen_V11(node,owner,worker):
    path=ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261007_v11.py"
    old_tree=ast.parse(path.read_bytes());new_tree=ast.parse(Path(node.__file__).read_bytes())
    old={part.name:part for part in old_tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    new={part.name:part for part in new_tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    for name in old.keys()-{"run","private_rclpy_view","bindings"}:assert ast.dump(old[name],include_attributes=False)==ast.dump(new[name],include_attributes=False),name
    def branches(tree):
        run=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="run")
        guarded=next(part for part in run.body if isinstance(part,ast.Try));direct=guarded.body[0]
        owned=next(part for part in run.body if isinstance(part,ast.ClassDef) and part.name=="OwnedNode")
        return direct.body,direct.orelse[0].body,{part.name:part for part in owned.body if isinstance(part,ast.FunctionDef)}
    a,b,c=branches(old_tree);x,y,z=branches(new_tree)
    assert ast.dump(ast.Module(body=a+b,type_ignores=[]),include_attributes=False)==ast.dump(ast.Module(body=x+y,type_ignores=[]),include_attributes=False)
    for name in c.keys()-{"destroy_node"}:assert ast.dump(c[name],include_attributes=False)==ast.dump(z[name],include_attributes=False),name
    for name,pin in worker.V11_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V11_EVIDENCE)==3 and len(worker.EXECUTOR_API_PINS)==2
    assert worker.FILE_LIMIT==32*1024**2 and worker.QUOTA==128*1024**2 and worker.fixed_contract()["camera_anchors"]==918
    assert worker.fixed_contract()["policy_deadline_seconds"]==.1 and worker.fixed_contract()["executor_membership_latency_gain_measured"] is False
