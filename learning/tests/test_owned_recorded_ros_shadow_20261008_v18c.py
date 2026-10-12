"""HH_261008 - Preserve V18b's real pure failure; bind every genuine poll/fail external name and synthetic authority context.

Buffer fakes prove byte order/hash and call-chain preservation, not NumPy stride,
zero-dimensional or empty-shaped buffer ABI. Those actual NumPy cases require
a separately authorized bounded suite; generic all-shape equivalence is UNKNOWN.
"""
from __future__ import annotations

import ast
import builtins
from dataclasses import asdict
import symtable
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
V16_ONLY_FUNCTIONS={"past_hit_builder_ast","verify_past_hit_builder","install_past_hit_builder"}



def restore_only_post_spin_poll(part):
    # HH_261008 - Restore exactly the one subscriber scheduling call, never alter the original eight compiled functions.
    result=deepcopy(part)
    if not isinstance(result,ast.FunctionDef) or result.name!="run":return result
    class Restore(ast.NodeTransformer):
        def visit_Expr(self,item):
            if isinstance(item.value,ast.Call) and ast.unparse(item.value.func)=="node.poll" and not item.value.args and not item.value.keywords:return None
            return self.generic_visit(item)
    return Restore().visit(result)

def restore_only_past_hit_admission(part):
    # HH_261008 - Restore only new setup/live binding checks for inherited complete scientific/timer AST comparisons.
    class Restore(ast.NodeTransformer):
        def visit_Expr(self,item):
            if isinstance(item.value,ast.Call) and isinstance(item.value.func,ast.Name) and item.value.func.id in {"verify_past_hit_builder","install_past_hit_builder"}:return None
            return self.generic_visit(item)
        def visit_Assign(self,item):
            if len(item.targets)==1 and ast.unparse(item.targets[0])=="self.past_hit_binding":return None
            return self.generic_visit(item)
        def visit_Try(self,item):
            if len(item.body)==1 and isinstance(item.body[0],ast.Expr) and isinstance(item.body[0].value,ast.Call) and ast.unparse(item.body[0].value.func)=="verify_past_hit_builder":return None
            return self.generic_visit(item)
    return Restore().visit(restore_only_post_spin_poll(part))


def restore_only_receipt_hash_copy(part):
    # HH_261008 - Restore the single changed SHA input for inherited exact-scientific AST regression checks.
    result=restore_only_past_hit_admission(part)
    if isinstance(result,ast.FunctionDef) and result.name=="tensor_receipt":
        calls=[value for value in ast.walk(result) if isinstance(value,ast.Call) and ast.unparse(value.func)=="hashlib.sha256"]
        assert len(calls)==1 and len(calls[0].args)==1 and ast.unparse(calls[0].args[0])=="memoryview(array).cast('B')"
        calls[0].args[0]=ast.parse("array.tobytes()",mode="eval").body
    return result


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/path);value=importlib.util.module_from_spec(spec)
    sys.modules[name]=value;spec.loader.exec_module(value);return value


@pytest.fixture
def worker():return module("tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v18c.py","_hh_test_protocol_261007")


@pytest.fixture
def owner():return module("tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v18c.py","_hh_test_owner_261007")


@pytest.fixture
def node():return module("autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v18c.py","_hh_test_node_261007")


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


class Array(bytearray):
    dtype="float32"
    shape=(1,2)
    # HH_261008 - Supply an actual stdlib byte buffer; no NumPy object or fake buffer protocol is assumed.
    def __init__(self):super().__init__(b"actual synthetic finite fixture bytes")
    def tobytes(self):return bytes(self)


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
    # HH_261008 - Original comparison-math fixtures inject decoded rows; separate codec fixtures test actual synthetic disk bytes.
    monkeypatch.setattr(worker,"iter_records",lambda path:iter(rows[Path(path).name.removesuffix("_anchors.jsonl")]))
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


class AuditArray(bytearray):
    """HH_261007 - Scalar byte-work fixture only; never a genuine NumPy/CNN output."""
    dtype="float32"
    shape=(1,2)
    def __new__(cls,tensor):return super().__new__(cls)
    # HH_261008 - Keep genuine synthetic value bytes while making the old scalar fixture compatible with memoryview.
    def __init__(self,tensor):self.tensor=tensor;super().__init__(json.dumps(tensor.values,allow_nan=False).encode())
    def tobytes(self):
        self.tensor.events.append("bytes:"+self.tensor.name)
        return bytes(self)


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
    # HH_261008 - Normalize only the separately reviewed V16 builder admission checks, never the scientific audit chain.
    assert ast.dump(restore_only_past_hit_admission(immediate),include_attributes=False)==ast.dump(original,include_attributes=False)


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
    for name in old.keys()-{"run","private_rclpy_view","bindings"}:assert ast.dump(old[name],include_attributes=False)==ast.dump(restore_only_receipt_hash_copy(new[name]),include_attributes=False),name
    def branches(tree):
        run=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="run")
        guarded=next(part for part in run.body if isinstance(part,ast.Try));direct=guarded.body[0]
        owned=next(part for part in run.body if isinstance(part,ast.ClassDef) and part.name=="OwnedNode")
        return direct.body,direct.orelse[0].body,{part.name:part for part in owned.body if isinstance(part,ast.FunctionDef)}
    a,b,c=branches(old_tree);x,y,z=branches(new_tree)
    # HH_261008 - Only the journal storage/count seam changes in publisher; no original scientific/timestamp statement changes.
    x=deepcopy(x);y=deepcopy(y)
    class OriginalCount(ast.NodeTransformer):
        def visit_Assign(self,part):
            if len(part.targets)==1 and isinstance(part.targets[0],ast.Name) and part.targets[0].id=="completed" and ast.unparse(part.value)=="worker.count_records(records)":
                part.value=ast.parse('sum(1 for _ in records.open("rb")) if records.exists() else 0',mode="eval").body
            return self.generic_visit(part)
    x=[OriginalCount().visit(part) for part in x];y=[OriginalCount().visit(part) for part in y]
    assert ast.dump(ast.Module(body=a+b,type_ignores=[]),include_attributes=False)==ast.dump(ast.Module(body=x+y,type_ignores=[]),include_attributes=False)
    for name in c.keys()-{"destroy_node"}:assert ast.dump(c[name],include_attributes=False)==ast.dump(z[name],include_attributes=False),name
    for name,pin in worker.V11_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V11_EVIDENCE)==3 and len(worker.EXECUTOR_API_PINS)==2
    assert worker.FILE_LIMIT==32*1024**2 and worker.QUOTA==128*1024**2 and worker.fixed_contract()["camera_anchors"]==918
    assert worker.fixed_contract()["policy_deadline_seconds"]==.1 and worker.fixed_contract()["executor_membership_latency_gain_measured"] is False


def test_v14_baseline_transient_fixture_and_candidate_persistent_mode_share_cleanup(node,worker):
    assert worker.SUBSCRIBER_PERSISTENT_MEMBERSHIP is True
    fixed=worker.fixed_contract()
    assert fixed["subscriber_persistent_membership"] is True
    assert fixed["logging_protocol"]["total_reserved_bytes"]==106724864
    assert fixed["original_ACK_clock_includes_later_journal_or_console"] is False
    actual,Owned,token,owned,executor,membership,live=persistent_fixture(node)
    # HH_261008 - Reconstruct fixedFalse baseline only inside this scalar fixture; no runtime mode switch is exposed.
    private=node.private_rclpy_view(SimpleNamespace(),executor,membership=None)
    for _ in range(3):private.spin_once(actual,timeout_sec=.02)
    assert executor.adds==executor.removes==executor.spins==3
    assert membership.actual_node is actual and membership.registered=={} and executor.get_nodes()==[]
    assert actual.executor is executor
    records=[];node.close_owned_runtime(owned,actual,Owned,token,[executor],[],membership,lambda:None,records)
    assert actual.executor is None and membership.closed and all(row["success"] for row in records)
    actual,Owned,token,owned,executor,membership,live=persistent_fixture(node)
    private=node.private_rclpy_view(SimpleNamespace(),executor,membership=membership if worker.SUBSCRIBER_PERSISTENT_MEMBERSHIP else None)
    for _ in range(3):private.spin_once(actual,timeout_sec=.02)
    assert executor.adds==1 and executor.removes==0 and executor.spins==3
    assert membership.registered[id(actual)] is actual and executor.get_nodes()==[actual]
    records=[];node.close_owned_runtime(owned,actual,Owned,token,[executor],[],membership,lambda:None,records)
    assert actual.executor is None and membership.closed and all(row["success"] for row in records)


def test_v13_current_frozen_V12_and_codec_sources_sealed(worker):
    for name,pin in {**worker.V12_PINS,**worker.CODEC_PINS}.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V12_PINS)==len(worker.CODEC_PINS)==4
    for name,pin in worker.V13_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V13_PINS)==4


def test_v13_all_fixed_journals_exact_synthetic_bytes_and_exhausted_read(worker,monkeypatch,tmp_path):
    monkeypatch.setattr(worker,"OUTPUT",tmp_path);monkeypatch.setattr(worker,"budget",lambda **kw:0)
    monkeypatch.setattr(worker,"frozen_bundle",lambda:SimpleNamespace(decode=lambda raw,limit:json.loads(raw)))
    for name in ("direct_anchors","subscriber_anchors","publisher_anchors","subscriber_output_calls","publisher_received_outputs","comparison"):
        path=tmp_path/(name+".jsonl");values=[dict(ordinal=i,timestamp_ns=2**63+17+i,native_frame=10+i,value="same bytes") for i in range(2)]
        for value in values:worker.append(path,value)
        assert list(worker.iter_records(path))==values and worker.count_records(path)==2
        assert not path.exists()
        raw=list(worker.codec().iter_rows(*worker.ledger_binding(path)[:1],**worker.ledger_binding(path)[1]))
        assert raw==[worker.encode(value) for value in values]
    closing=worker.close_ledgers()
    assert not closing["codec_failure"] and len(closing["ledgers"])==6
    assert all(row["verification"]["exact_original_bytes_reconstructed"] for row in closing["ledgers"])
    with pytest.raises(ValueError):worker.append(tmp_path/"unknown.jsonl",{})


@pytest.mark.parametrize("fault",["guard","partial_index","oversized_print"])
def test_v13_codec_failure_retains_pending_or_descriptor_and_never_prefix_PASS(worker,monkeypatch,tmp_path,fault):
    monkeypatch.setattr(worker,"OUTPUT",tmp_path);monkeypatch.setattr(worker,"budget",lambda **kw:0)
    path=tmp_path/("subscriber.stdout" if fault=="oversized_print" else "subscriber_anchors.jsonl")
    value=worker.ledger(path)
    if fault=="guard":value.guard=lambda added:(_ for _ in ()).throw(ValueError("actual prospective quota denial"))
    if fault=="partial_index":
        original=value._index
        class Partial:
            def write(self,raw):original.write(raw[:12]);original.flush();raise OSError("actual partial synthetic index")
            def close(self):original.close()
        value._index=Partial()
    with pytest.raises((ValueError,OSError)):
        if fault=="oversized_print":value.print("A"*(512*1024+1),flush=True)
        else:worker.append(path,dict(ordinal=0,status="FAILED_RETAINED"))
    closing=worker.close_ledgers();row=closing["ledgers"][0]
    assert closing["codec_failure"] and row["complete_prefix_is_PASS"] is False
    if fault=="oversized_print":
        assert row["pending_raw_path"] is None and row["oversized_fragment_receipt"]["oversized_fragment_full_bytes_NOT_RETAINED"]
        assert row["oversized_fragment_receipt"]["lossless_reconstruction_of_failed_stream_claimed"] is False
    else:
        assert Path(row["pending_raw_path"]).read_bytes()==worker.encode(dict(ordinal=0,status="FAILED_RETAINED"))
        if fault=="partial_index":assert row["errors"] and row["verification"] is None


def test_v13_compare_bad_tail_keeps_full918_UNKNOWN_and_exhausts_all_other_roles(owner,worker,monkeypatch,tmp_path):
    monkeypatch.setattr(worker,"OUTPUT",tmp_path);finished=[];observed=[]
    def read(path):
        mode=path.name.removesuffix("_anchors.jsonl")
        if mode=="subscriber":raise ValueError("actual unindexed gzip tail retained")
        yield from synthetic_rows(2)[mode];finished.append(mode)
    monkeypatch.setattr(worker,"iter_records",read);monkeypatch.setattr(worker,"append",lambda path,row:observed.append(row))
    result=owner.compare(worker)
    assert set(finished)=={"direct","publisher"} and len(observed)==918
    assert result["counts"]["UNKNOWN"]==918 and result["counts"]["UNEXECUTED"]==0
    assert all(row["status"]=="UNKNOWN" and row["actually_executed_bundle_unknown"] for row in observed)
    assert result["compressed_journal_read_errors"]==dict(subscriber="ValueError('actual unindexed gzip tail retained')")
    assert result["status"]=="FAIL_OR_NONCOMPARABLE_RETAINED" and not result["unverifiable_tail_repaired"]


def console_fixture(node,worker,monkeypatch,tmp_path,mode="subscriber"):
    monkeypatch.setattr(worker,"OUTPUT",tmp_path);monkeypatch.setattr(worker,"budget",lambda **kw:0)
    pure=SimpleNamespace(SCHEMA="original.schema",decode=lambda raw,limit:json.loads(raw),encode=lambda value:json.dumps(value,sort_keys=True).encode())
    anchor=SimpleNamespace(ordinal=0,native_frame=2**62,timestamp_ns=2**63+1,provenance=dict(real_source_sha256="a"*64))
    index=SimpleNamespace(anchors=[anchor]);view=dict(replay_id="actual-run",admitted_authority_sha256="b"*64)
    value=dict(schema=pure.SCHEMA+".shadow_receipt",ordinal=0,native_frame=anchor.native_frame,timestamp_ns=anchor.timestamp_ns,
        replay_id=view["replay_id"],authority_sha256=view["admitted_authority_sha256"],status="TRAJECTORY",
        source_selected_callback_provenance=anchor.provenance,stages=dict(publisher_bundle_started_ns=100,receipt_publication_entered_ns=200,receipt_publication_finished_ns=300))
    return node.OriginalConsoleWitness(worker,pure,index,view,mode),value


def test_v13_console_return_clock_after_genuine_compress_flush_before_binding_no_ACK_change(node,worker,monkeypatch,tmp_path):
    witness,value=console_fixture(node,worker,monkeypatch,tmp_path);order=[];original=witness.ledger.print
    def actual(*args,**kw):order.append("actual codec print");return original(*args,**kw)
    witness.ledger.print=actual
    monkeypatch.setattr(node.time,"perf_counter_ns",lambda:(order.append("actual return clock") or 500))
    text=json.dumps(value);before=deepcopy(value);witness.print(text,flush=True)
    assert order==["actual codec print","actual return clock"] and value==before
    row=witness.rows[0]
    assert row["binding"]=="ACTUAL_RECEIPT_SOURCE_JOIN" and row["actual_until_original_console_log_complete_ms"]==.0004
    assert row["receipt_arg_byte_sha256"]==hashlib.sha256(text.encode()).hexdigest()
    closing=witness.closing();assert closing["UNEXECUTED"]==917 and not closing["original_ACK_clock_includes_later_journal_or_console"]
    assert closing["witness_binding_and_ROLE_CLOSING_serialization_excluded_from_per_anchor_clock"]
    assert list(worker.codec().iter_rows(witness.ledger.root,**worker.ledger_binding(tmp_path/"subscriber.stdout")[1]))==[(text+"\n").encode()]
    worker.close_ledgers()


@pytest.mark.parametrize("fault",["provenance","stage","chronology","authority","stamp","failure_receipt","summary"])
def test_v13_missing_or_wrong_console_binding_is_unknown_not_guessed(node,worker,monkeypatch,tmp_path,fault):
    witness,value=console_fixture(node,worker,monkeypatch,tmp_path,mode="publisher" if fault=="summary" else "subscriber")
    monkeypatch.setattr(node.time,"perf_counter_ns",lambda:500)
    if fault=="provenance":value["source_selected_callback_provenance"]={}
    elif fault=="stage":del value["stages"]["publisher_bundle_started_ns"]
    elif fault=="chronology":value["stages"]["receipt_publication_finished_ns"]=600
    elif fault=="authority":value["authority_sha256"]="c"*64
    elif fault=="stamp":value["timestamp_ns"]+=1
    elif fault=="failure_receipt":value=dict(value,status="FAILED_RETAINED");del value["source_selected_callback_provenance"]
    else:value=dict(expected=918,completed_receipts=0,UNEXECUTED=918)
    witness.print(json.dumps(value),flush=True);row=witness.rows[0]
    assert row["actual_until_original_console_log_complete_ms"] is None
    assert row["binding"]==("ORIGINAL_CLOSING_SUMMARY_NOT_ANCHOR" if fault=="summary" else "UNKNOWN")
    assert witness.closing()["bound_anchor_measurements"]==0
    worker.close_ledgers()


def test_v13_source_all_original_clock_publish_and_scientific_AST_unchanged(node,worker):
    old=ast.parse((ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v12.py").read_bytes())
    new=ast.parse(Path(node.__file__).read_bytes())
    def owned(tree):
        run=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="run")
        return next(part for part in run.body if isinstance(part,ast.ClassDef) and part.name=="OwnedNode")
    assert ast.dump(owned(old),include_attributes=False)==ast.dump(owned(new),include_attributes=False)
    ns=node.compile_original(worker);original=ast.parse(node.OLD.read_bytes())
    for name in ("recorded_node","recorded_publisher"):
        function=next(part for part in original.body if isinstance(part,ast.FunctionDef) and part.name==name)
        expected=compile(ast.Module(body=[function],type_ignores=[]),str(node.OLD),"exec")
        code=next(part for part in expected.co_consts if hasattr(part,"co_name") and part.co_name==name)
        assert ns[name].__code__.co_code==code.co_code
    run=next(part for part in new.body if isinstance(part,ast.FunctionDef) and part.name=="run");text=ast.unparse(run)
    assert "ns['print'] = console.print" in text and "sys.stdout" not in text
    assert "SUBSCRIBER_PERSISTENT_MEMBERSHIP else None" in text


def test_v13_console_clock_failure_retained_unknown_after_exact_original_bytes(node,worker,monkeypatch,tmp_path):
    witness,value=console_fixture(node,worker,monkeypatch,tmp_path)
    monkeypatch.setattr(node.time,"perf_counter_ns",lambda:(_ for _ in ()).throw(OSError("actual return clock unavailable")))
    witness.print(json.dumps(value),flush=True);row=witness.rows[0]
    assert row["original_console_returned_ns"] is None and row["binding"]=="UNKNOWN"
    assert row["actual_until_original_console_log_complete_ms"] is None and row["clock_error"]=="OSError('actual return clock unavailable')"
    worker.close_ledgers()


def test_v13_console_complete919_scalar_closing_bound_without_future_clock_or_callback_write(node,worker,monkeypatch,tmp_path):
    witness,value=console_fixture(node,worker,monkeypatch,tmp_path,mode="publisher")
    monkeypatch.setattr(node.time,"perf_counter_ns",lambda:2**63-1)
    witness.print(json.dumps(value),flush=True);sample=deepcopy(witness.rows[0]);witness.rows=[]
    for ordinal in range(918):
        row=deepcopy(sample);row.update(console_call=ordinal,ordinal=ordinal,native_frame=2**63-1,timestamp_ns=2**63-1)
        witness.rows.append(row)
    witness.rows.append(dict(console_call=918,original_console_returned_ns=2**63-1,binding="ORIGINAL_CLOSING_SUMMARY_NOT_ANCHOR",ordinal=None))
    closing=witness.closing()
    assert closing["UNEXECUTED"]==0 and closing["observed_original_console_calls"]==919
    assert len(worker.encode(closing))<=worker.JSON_LIMIT
    tree=ast.parse(Path(node.__file__).read_bytes());cls=next(part for part in tree.body if isinstance(part,ast.ClassDef) and part.name=="OriginalConsoleWitness")
    printed=next(part for part in cls.body if isinstance(part,ast.FunctionDef) and part.name=="print");calls={ast.unparse(part.func) for part in ast.walk(printed) if isinstance(part,ast.Call)}
    assert not (calls & {"self.worker.append","self.worker.new_json","time.sleep","node.publish","open"})
    worker.close_ledgers()


def test_v13_prospective_added_bytes_and_complete_stderr_metadata_bounds(worker,monkeypatch,tmp_path):
    plan=tmp_path/"plan";output=tmp_path/"output";plan.mkdir();output.mkdir()
    monkeypatch.setattr(worker,"PLAN_DIR",plan);monkeypatch.setattr(worker,"OUTPUT",output)
    monkeypatch.setattr(worker.shutil,"disk_usage",lambda _:SimpleNamespace(free=worker.FLOOR+worker.SHARED+worker.QUOTA))
    with pytest.raises(ValueError):worker.budget(additional=worker.QUOTA-worker.CLOSING+1)
    assert worker.budget(additional=worker.QUOTA,closing=True)==0
    with (output/"subscriber.stderr").open("wb") as stream:stream.truncate(1024**2+1)
    with pytest.raises(ValueError,match="stderr"):worker.budget()
    (output/"subscriber.stderr").unlink()
    with (plan/"metadata").open("wb") as stream:stream.truncate(10*1024**2+1)
    with pytest.raises(ValueError,match="metadata"):worker.budget()
    assert worker.budget(closing=True)==10*1024**2+1


def test_v14_all_node_science_console_logging_cleanup_AST_identical_to_frozen_V13(node,worker,owner):
    # HH_261008 - The actual node code (including every timer, transport, logger, cleanup and probe) is unchanged; only fixed source mode differs.
    old_node=ast.parse((ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v13.py").read_bytes())
    new_node=ast.parse(Path(node.__file__).read_bytes())
    def functions(tree):return {part.name:part for part in tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef)) and part.name not in V16_ONLY_FUNCTIONS}
    assert {name:ast.dump(part,include_attributes=False) for name,part in functions(old_node).items()}=={name:ast.dump(restore_only_receipt_hash_copy(part),include_attributes=False) for name,part in functions(new_node).items()}
    for actual,old_name,excluded in ((worker,"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v13.py",{"source_pins","fixed_contract","worker_command"}),
        (owner,"tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v13.py",{"prepare"})):
        previous=functions(ast.parse((ROOT/old_name).read_bytes()));current=functions(ast.parse(Path(actual.__file__).read_bytes()))
        assert previous.keys()==current.keys()
        for name in previous.keys()-excluded:assert ast.dump(previous[name],include_attributes=False)==ast.dump(restore_only_post_spin_poll(current[name]),include_attributes=False),name
    assert worker.fixed_contract()["subscriber_private_executor_membership"]=="once_per_exact_live_OwnedNode_then_public_executor_None_detach"
    assert worker.CODEC_PINS==module("tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v13.py","_hh_v13_fixed_comparison").CODEC_PINS


def test_v14_missing_actual_V13_provenance_denies_before_any_preparation_write(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V14_EVIDENCE",{"a":"a","b":"b","c":"c"})
    monkeypatch.setattr(worker,"V13_EVIDENCE",{})
    monkeypatch.setattr(worker,"budget",lambda **kw:pytest.fail("preparation progressed before completed V13 provenance"))
    with pytest.raises(ValueError,match="completed V13"):owner.prepare(worker)


def test_v14_exact_failed_V13_evidence_paths_and_closing_source_input_join(worker,owner):
    assert len(worker.V13_EVIDENCE)==3
    names=list(worker.V13_EVIDENCE)
    assert names==[str(ROOT/"storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v13/result.json"),
        str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_plan.json"),
        str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_execution_authority.json")]
    assert list(worker.V13_EVIDENCE.values())==["2b009f267a98ed089191e2fc3b735732768476d9e12482d64dd0e85a3cf273eb",
        "d2c2eb14d75b85cb7ec27a05ebde598c8598f85fb17de92e66a6481c99647174",
        "0d9f61da14aa1463895fea34f31d8470288c6f84d6e052a253ca3888b2e21293"]
    tree=ast.parse(Path(owner.__file__).read_bytes());prepare=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="prepare")
    source=ast.unparse(prepare)
    assert "v13_result['source_sha256_after'] == v13_plan['source_sha256']" in source
    assert "v13_result['input_sha256_after'] == v13_plan['input_sha256']" in source
    assert "input_pins.update(worker.V13_EVIDENCE)" in source
    assert "v13_result['status']" not in source


class ReceiptBuffer(bytearray):
    """HH_261008 - Genuine stdlib buffer with scalar ndarray-shaped metadata; not a NumPy compatibility proof."""
    def __new__(cls,raw,*,dtype="float32",shape=(1,2)):return super().__new__(cls)
    def __init__(self,raw,*,dtype="float32",shape=(1,2)):
        super().__init__(raw);self.dtype=dtype;self.shape=shape;self.tobytes_calls=0
    def tobytes(self):self.tobytes_calls+=1;return bytes(self)


class ReceiptTensor:
    def __init__(self,buffer,events):self.buffer=buffer;self.events=events;self.contiguous_done=False
    def detach(self):self.events.append("detach");return self
    def cpu(self):self.events.append("cpu");return self
    def contiguous(self):self.events.append("contiguous");self.contiguous_done=True;return self
    def numpy(self):
        self.events.append("numpy")
        assert self.contiguous_done,"the actual original contiguous step must precede buffer access"
        return self.buffer


def actual_frozen_V14_tensor_receipt(worker):
    path=ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v14.py"
    assert hashlib.sha256(path.read_bytes()).hexdigest()==worker.V14_PINS[str(path.relative_to(ROOT))]
    function=next(part for part in ast.parse(path.read_bytes()).body if isinstance(part,ast.FunctionDef) and part.name=="tensor_receipt")
    namespace=dict(hashlib=hashlib)
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(path),"exec"),namespace)
    return namespace["tensor_receipt"]


@pytest.mark.parametrize("byte_count,shape,dtype",[(20736000,(1,5,6,3,180,320),"float32"),(8,(1,2),"float32"),(5,(1,5),"bool"),(0,(0,),"uint8")])
def test_v15_exact_original_V14_hash_and_chain_without_tobytes_call(node,worker,byte_count,shape,dtype):
    # HH_261008 - The 20.7MiB storage fixture is stdlib-only; the empty byte buffer does not certify NumPy empty-shaped cast behavior.
    buffer=ReceiptBuffer(byte_count,shape=shape,dtype=dtype)
    if byte_count:buffer[0]=17;buffer[-1]=233
    original=actual_frozen_V14_tensor_receipt(worker);events=[]
    before=hashlib.sha256(buffer).hexdigest();expected=original({"images":ReceiptTensor(buffer,events)})
    assert events==["detach","cpu","contiguous","numpy"] and buffer.tobytes_calls==1
    events.clear();buffer.tobytes_calls=0
    result=node.tensor_receipt({"images":ReceiptTensor(buffer,events)})
    assert result==expected and result["images"]==dict(dtype=dtype,shape=list(shape),sha256=before)
    assert events==["detach","cpu","contiguous","numpy"] and buffer.tobytes_calls==0
    assert hashlib.sha256(buffer).hexdigest()==before


@pytest.mark.parametrize("changed",["shape","dtype","bytes"])
def test_v15_receipt_changes_track_the_same_original_metadata_and_actual_bytes(node,worker,changed):
    buffer=ReceiptBuffer(b"abcdefgh",shape=(1,2));original=actual_frozen_V14_tensor_receipt(worker)
    initial=node.tensor_receipt({"input":ReceiptTensor(buffer,[])})
    if changed=="shape":buffer.shape=(2,1)
    elif changed=="dtype":buffer.dtype="float64"
    else:buffer[3]^=1
    expected=original({"input":ReceiptTensor(buffer,[])});buffer.tobytes_calls=0
    result=node.tensor_receipt({"input":ReceiptTensor(buffer,[])})
    assert result==expected and result!=initial and buffer.tobytes_calls==0
    assert (result["input"]["sha256"]!=initial["input"]["sha256"])==(changed=="bytes")


def test_v15_all_five_raw_head_receipts_order_and_original_hashes_preserved(node,worker):
    names=("curvature_knots","preferred_speed_knots","intent_logits","stop_distance_raw","acceleration_prediction")
    values={name:ReceiptTensor(ReceiptBuffer((name+" actual raw synthetic bytes").encode()),[]) for name in reversed(names)}
    original=actual_frozen_V14_tensor_receipt(worker);expected=original(values)
    for value in values.values():value.buffer.tobytes_calls=0;value.events.clear()
    actual=node.tensor_receipt(values)
    assert actual==expected and list(actual)==sorted(names) and len(actual)==5
    assert all(value.buffer.tobytes_calls==0 and value.events==["detach","cpu","contiguous","numpy"] for value in values.values())


def test_v15_inverse_exact_single_hash_expression_and_all_other_node_functions_V14(node,worker):
    old=ast.parse((ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v14.py").read_bytes())
    new=ast.parse(Path(node.__file__).read_bytes())
    previous={part.name:part for part in old.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    current={part.name:part for part in new.body if isinstance(part,(ast.FunctionDef,ast.ClassDef)) and part.name not in V16_ONLY_FUNCTIONS}
    assert previous.keys()==current.keys()
    for name in previous:
        assert ast.dump(previous[name],include_attributes=False)==ast.dump(restore_only_receipt_hash_copy(current[name]),include_attributes=False),name
    actual=current["tensor_receipt"];chain=ast.unparse(actual)
    assert "value.detach().cpu().contiguous().numpy()" in chain and "array.tobytes()" not in chain
    assert "memoryview(array).cast('B')" in chain and "str(array.dtype)" in chain and "list(array.shape)" in chain
    assert worker.SUBSCRIBER_PERSISTENT_MEMBERSHIP is True


def test_v15_frozen_V14_and_actual_evidence_paths_admitted_without_success_claim(worker,owner):
    for name,pin in worker.V14_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V14_PINS)==4 and len(worker.V14_EVIDENCE)==3
    names=list(worker.V14_EVIDENCE)
    assert names==[str(ROOT/"storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14/result.json"),
        str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v14/ROOT_plan.json"),
        str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v14/ROOT_execution_authority.json")]
    assert list(worker.V14_EVIDENCE.values())==["acde13e028f079149d700d107c0eb392a371559ac14dd3886a18e7c8f7a3a29d",
        "849eca5259d031172107cd15d5adfc0749c9124d24fb134832115dfd6024b2d8",
        "84e80f54b4fd33e4eca009556f78bba5b57fe251a4d922fce32d4b568e1e3016"]
    tree=ast.parse(Path(owner.__file__).read_bytes());prepare=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="prepare")
    source=ast.unparse(prepare)
    assert "v14_result['source_sha256_after'] == v14_plan['source_sha256']" in source
    assert "v14_result['input_sha256_after'] == v14_plan['input_sha256']" in source
    assert "input_pins.update(worker.V14_EVIDENCE)" in source and "v14_result['status']" not in source


def test_v15_missing_actual_V14_provenance_denies_before_preparation_write(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V14_EVIDENCE",{})
    monkeypatch.setattr(worker,"budget",lambda **kw:pytest.fail("preparation progressed before complete V14 provenance"))
    with pytest.raises(ValueError,match="completed V14"):owner.prepare(worker)


class HistoryTensor:
    """HH_261008 - Scalar storage/alias fake; no Torch/NumPy shape, finite-kernel or timing equivalence claim."""
    def __init__(self,data,events,kind="image",copy=True):self.data=deepcopy(data) if copy else data;self.events=events;self.kind=kind;self._version=0
    def __getitem__(self,index):return HistoryTensor([self.data] if index is None else self.data[index],self.events,self.kind,copy=False)
    def detach(self):return self
    def clone(self):self.events.append(("clone",self.kind));return HistoryTensor(self.data,self.events,self.kind)


def history_fixture(node,worker,*,candidate,warm=True):
    # HH_261008 - Execute genuine frozen source AST with scalar providers, not its constructor/model/runtime imports.
    from collections import OrderedDict
    from types import ModuleType
    name="portable_e2e.temporal_simulation_research_v2_20261001"
    path=ROOT/next(iter(worker.BUILDER_PINS));tree=ast.parse(path.read_bytes())
    cls=next(part for part in tree.body if isinstance(part,ast.ClassDef) and part.name=="TemporalSimulationResearchPolicyV2")
    methods=[deepcopy(part) for part in cls.body if isinstance(part,ast.FunctionDef) and part.name in {"build_inputs","_validate_frame","_camera_digest","reset"}]
    events=[];calls=[];provider_results=[]
    class Inputs(SimpleNamespace):pass
    class Frame(SimpleNamespace):pass
    def runtime(inputs,config):
        calls.append(inputs.camera_frames[0].timestamp_ns)
        image=HistoryTensor([[[float(camera.payload[0]),float(i)] for i,camera in enumerate(inputs.camera_frames)]],events)
        value={"images":image,**{key:HistoryTensor([key,inputs.camera_frames[0].timestamp_ns],events,key) for key in
            ("ego_history","ego_history_mask","route_xy","route_mask","calibration")}}
        provider_results.append(value);return value
    def tensor(data,**kw):return HistoryTensor(data,events,"metadata")
    def stack(values):events.append(("stack",len(values)));return HistoryTensor([value.data for value in values],events)
    fake_torch=SimpleNamespace(zeros_like=lambda value:HistoryTensor([[0.,0.] for _ in value.data],events),stack=stack,tensor=tensor,bool="bool",float32="float32")
    module=ModuleType(name);module.__file__=str(path)
    module.__dict__.update(ObservedTemporalFrameV2=Frame,RuntimeInputs=Inputs,_require=worker.need,ContractError=ValueError,
        hashlib=hashlib,math=math,np=SimpleNamespace(),torch=fake_torch,ModelConfig=SimpleNamespace,build_runtime_tensors=runtime,
        OrderedDict=OrderedDict,TIME_OFFSETS_NS=(-800000000,-600000000,-400000000,-200000000,0),
        CameraFrameIdentityV2=lambda timestamp,hashes:(timestamp,hashes),TemporalCacheWindowV2=lambda *values:values,
        Sequence=list,TemporalSampleMetadata=Frame)
    pose_tree=ast.parse((ROOT/"learning/portable_e2e/temporal_dataset_v2.py").read_bytes())
    pose_methods=[part for part in pose_tree.body if isinstance(part,ast.FunctionDef) and part.name in {"_relative_pose","_yaw_from_quaternion"}]
    class_tree=ast.ClassDef(name="TemporalSimulationResearchPolicyV2",bases=[],keywords=[],body=methods,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=pose_methods+[class_tree],type_ignores=[])),str(path),"exec"),module.__dict__)
    policy=module.TemporalSimulationResearchPolicyV2();policy.identity=("episode","val","rig","a"*64);policy._images=OrderedDict()
    frames=[];calibration=tuple((float(i),) for i in range(6))
    for ordinal in range(10):
        stamp=ordinal*100000000
        cameras=tuple(SimpleNamespace(timestamp_ns=stamp,encoding="jpeg",payload=bytearray((ordinal+1,i+1))) for i in range(6))
        frames.append(Frame(timestamp_ns=stamp,pose_timestamp_ns=stamp,episode_id="episode",split="val",rig_id="rig",source_sha256="a"*64,
            position_map_m=(float(ordinal),float(ordinal)/2,0.),orientation_xyzw=(0.,0.,0.,1.),inputs=Inputs(camera_frames=cameras,calibration=calibration)))
    if warm:
        for frame in frames[:-1]:
            key=frame.timestamp_ns,tuple(policy._camera_digest(camera) for camera in frame.inputs.camera_frames)
            policy._images[key]=runtime(frame.inputs,SimpleNamespace())["images"][0].detach().clone()
    events.clear();calls.clear();provider_results.clear()
    old_module=sys.modules.get(name);sys.modules[name]=module
    try:
        binding=node.install_past_hit_builder(SimpleNamespace(_policy=policy),worker) if candidate else None
    finally:
        if old_module is None:sys.modules.pop(name,None)
        else:sys.modules[name]=old_module
    return SimpleNamespace(policy=policy,frames=frames,events=events,calls=calls,providers=provider_results,binding=binding,module=module)


def build_history(fixture,*,past=None,now=None):
    frame=fixture.frames[-1]
    return fixture.policy.build_inputs(frame,past_frames=fixture.frames[:-1] if past is None else past,
        now_ns=frame.timestamp_ns if now is None else now,ego_stamp_ns=frame.timestamp_ns,route_stamp_ns=frame.timestamp_ns)


def history_values(tensors):return {name:value.data for name,value in tensors.items()}


@pytest.mark.parametrize("warm",[False,True])
def test_v16_whole_nine_inputs_window_provenance_cache_and_current_clone_identical(node,worker,warm):
    baseline=history_fixture(node,worker,candidate=False,warm=warm);candidate=history_fixture(node,worker,candidate=True,warm=warm)
    a,wa=build_history(baseline);b,wb=build_history(candidate)
    assert history_values(a)==history_values(b) and wa==wb and len(a)==9
    assert list(baseline.policy._images)==list(candidate.policy._images)
    assert [value.data for value in baseline.policy._images.values()]==[value.data for value in candidate.policy._images.values()]
    assert baseline.calls==candidate.calls and candidate.calls==([900000000] if warm else [900000000,100000000,300000000,500000000,700000000])
    assert baseline.events.count(("clone","image"))==5 and candidate.events.count(("clone","image"))==(1 if warm else 5)
    assert baseline.events.count(("stack",5))==candidate.events.count(("stack",5))==1
    node.verify_past_hit_builder(candidate.policy,worker,candidate.binding)


def test_v16_final_stack_and_current_cache_preserve_bidirectional_storage_isolation(node,worker):
    fixture=history_fixture(node,worker,candidate=True);tensors,window=build_history(fixture)
    output=deepcopy(tensors["images"].data);cache=deepcopy([value.data for value in fixture.policy._images.values()])
    tensors["images"].data[0][0][0][0]=-999
    assert [value.data for value in fixture.policy._images.values()]==cache
    past_key=next(key for key in fixture.policy._images if key[0]==100000000)
    fixture.policy._images[past_key].data[0][0]=-777
    assert tensors["images"].data[0][0][0][0]==-999 and tensors["images"].data[0][1:]==output[0][1:]
    current_key=next(key for key in fixture.policy._images if key[0]==900000000)
    current_before=deepcopy(fixture.policy._images[current_key].data)
    fixture.providers[0]["images"].data[0][0][0]=-555
    assert fixture.policy._images[current_key].data==current_before and tensors["images"].data[0][-1]==output[0][-1]


def test_v16_fresh_source_digest_mutation_invalidates_past_hit_and_retains_missing_unmask(node,worker):
    baseline=history_fixture(node,worker,candidate=False);candidate=history_fixture(node,worker,candidate=True)
    for fixture in (baseline,candidate):fixture.frames[1].inputs.camera_frames[0].payload[0]=99
    a,wa=build_history(baseline);b,wb=build_history(candidate)
    assert history_values(a)==history_values(b) and wa==wb and candidate.calls==[900000000,100000000]
    assert b["images"].data[0][0][0][0]==99.
    x,wx=build_history(baseline,past=[]);y,wy=build_history(candidate,past=[])
    assert history_values(x)==history_values(y) and wx==wy
    assert y["time_valid"].data==[[False,False,False,False,True]] and wy[-1][:-1]==(None,None,None,None)


@pytest.mark.parametrize("fault",["future","duplicate","unsorted","rig","episode","pose","camera_stamp","skew","too_many"])
def test_v16_original_causal_identity_and_pose_denials_identical_without_mask_bypass(node,worker,fault):
    failures=[]
    for candidate in (False,True):
        fixture=history_fixture(node,worker,candidate=candidate);past=fixture.frames[:-1];now=None
        if fault=="future":past=[fixture.frames[-1]]
        elif fault=="duplicate":past=[past[0],past[0]]
        elif fault=="unsorted":past=list(reversed(past))
        elif fault=="rig":past[0].inputs.calibration=()
        elif fault=="episode":past[0].episode_id="foreign"
        elif fault=="pose":past[0].pose_timestamp_ns=-1
        elif fault=="camera_stamp":past[0].inputs.camera_frames[0].timestamp_ns=1
        elif fault=="skew":now=fixture.frames[-1].timestamp_ns+150000001
        elif fault=="too_many":past=past+[past[-1]]
        with pytest.raises(ValueError) as error:build_history(fixture,past=past,now=now)
        failures.append(str(error.value));assert fixture.calls==[]
    assert failures[0]==failures[1]


@pytest.mark.parametrize("fault",["binding","marker","code","defaults","kwdefaults","original_code","original_defaults","provider","class_method"])
def test_v16_mutated_private_or_original_builder_is_denied_before_and_after(node,worker,fault):
    fixture=history_fixture(node,worker,candidate=True);policy=fixture.policy;expected=fixture.binding
    if fault=="binding":policy.build_inputs=lambda *a,**kw:None
    elif fault=="marker":policy._hh_past_hit_binding=None
    elif fault=="code":expected["function"].__code__=(lambda *a,**kw:None).__code__
    elif fault=="defaults":expected["function"].__defaults__=("different",)
    elif fault=="kwdefaults":expected["function"].__kwdefaults__["past_frames"]=("different",)
    elif fault=="original_code":expected["genuine"].__code__=(lambda *a,**kw:None).__code__
    elif fault=="original_defaults":expected["genuine"].__defaults__=("different",)
    elif fault=="provider":expected["function"].__globals__["torch"]=object()
    elif fault=="class_method":type(policy).build_inputs=lambda *a,**kw:None
    with pytest.raises(ValueError):node.verify_past_hit_builder(policy,worker,expected)


def test_v16_live_audit_binding_checks_preserve_hook_restore_on_original_exception(node,worker):
    fixture=history_fixture(node,worker,candidate=True);policy=fixture.policy;model=SimpleNamespace(forward_cached=lambda *a,**kw:{})
    policy.model=model
    original=RuntimeError("genuine underlying failure")
    def fail(*a,**kw):build_history(fixture);raise original
    adapter=SimpleNamespace(_policy=policy,accept=fail)
    for cls in (node.AuditedAdapter,node.DeferredAuditedAdapter):
        audited=cls(adapter,worker,**({"before_accept":lambda:None} if cls is node.DeferredAuditedAdapter else {}))
        with pytest.raises(RuntimeError) as caught:audited.accept()
        assert caught.value is original
        node.verify_past_hit_builder(policy,worker,fixture.binding)
        assert "forward_cached" in model.__dict__ and policy.build_inputs is fixture.binding["bound"]
    policy._hh_past_hit_binding=None
    with pytest.raises(ValueError):audited.accept()


@pytest.mark.parametrize("fault",["subclass","already_bound","wrong_origin","already_installed","source_pin"])
def test_v16_setup_denies_foreign_or_broader_policy_and_duplicate_override(node,worker,monkeypatch,fault):
    fixture=history_fixture(node,worker,candidate=False);policy=fixture.policy
    monkeypatch.setitem(sys.modules,fixture.module.__name__,fixture.module)
    if fault=="subclass":policy.__class__=type("BroaderPolicy",(type(policy),),{})
    elif fault=="already_bound":policy.build_inputs=policy.build_inputs
    elif fault=="wrong_origin":fixture.module.__file__=str(ROOT/"AGENTS.md")
    elif fault=="already_installed":node.install_past_hit_builder(SimpleNamespace(_policy=policy),worker)
    elif fault=="source_pin":monkeypatch.setattr(worker,"digest",lambda path:"0"*64)
    with pytest.raises(ValueError):node.install_past_hit_builder(SimpleNamespace(_policy=policy),worker)


def test_v16_guard_detects_in_call_mutation_after_original_return_and_retains_deferred_failure(node,worker):
    fixture=history_fixture(node,worker,candidate=True);policy=fixture.policy;policy.model=SimpleNamespace(forward_cached=lambda *a,**kw:{})
    def underlying(*a,**kw):
        build_history(fixture)
        fixture.binding["function"].__kwdefaults__["past_frames"]=("mutation",)
        return SimpleNamespace(status="PREFILL",diagnostics={})
    audited=node.DeferredAuditedAdapter(SimpleNamespace(_policy=policy,accept=underlying),worker,before_accept=lambda:None)
    with pytest.raises(ValueError,match="builder code/defaults"):audited.accept()
    assert policy.build_inputs is fixture.binding["bound"] and "forward_cached" in policy.model.__dict__
    assert audited._pending is None and audited.failure_tail["stage"]=="private_builder_restore_guard"
    # HH_261008 - No pending row survives the guard exception; future accepts continue to deny, not fabricate a no-NN receipt.
    with pytest.raises(ValueError):audited.accept()


def test_v16_original_reset_and_missing_cache_clones_preserve_cold_window(node,worker):
    baseline=history_fixture(node,worker,candidate=False);candidate=history_fixture(node,worker,candidate=True)
    resets=[]
    for fixture in (baseline,candidate):
        fixture.policy.model=SimpleNamespace(reset_cache=lambda reason:resets.append(reason));fixture.policy.pi=None
        fixture.policy.reset("actual_original_reset")
        assert not fixture.policy._images and fixture.policy._prefill_count==0
    a,wa=build_history(baseline);b,wb=build_history(candidate)
    assert history_values(a)==history_values(b) and wa==wb and resets==["actual_original_reset"]*2
    assert candidate.events.count(("clone","image"))==5 and len(candidate.policy._images)<=9
    node.verify_past_hit_builder(candidate.policy,worker,candidate.binding)


def test_v16_exact_one_expression_inverse_AST_original_guards_and_no_other_predict_activation(node,worker,owner):
    candidate,path=node.past_hit_builder_ast(worker)
    text=ast.unparse(candidate)
    assert text.count("image.detach().clone()")==2 and "image = self._images[key].clone()" not in text
    assert "image = self._images[key]" in text and "torch.stack(groups)[None]" in text
    assert "self._camera_digest(camera)" in text and "self._validate_frame(frame)" in text
    assert "len(self._images) > 9" in text and "800003000" in text
    original_tree=ast.parse((ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v15.py").read_bytes())
    actual_tree=ast.parse(Path(node.__file__).read_bytes())
    old={part.name:part for part in original_tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    new={part.name:part for part in actual_tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    assert set(new)-set(old)==V16_ONLY_FUNCTIONS
    for name in old:assert ast.dump(old[name],include_attributes=False)==ast.dump(restore_only_past_hit_admission(new[name]),include_attributes=False),name
    run=ast.unparse(new["run"])
    assert run.index("actual frozen CPU4/inter1/no CUDA execution required")<run.index("install_past_hit_builder(adapter, worker)")<run.index("audited = DeferredAuditedAdapter")
    assert "SingleCopyOwnedHeadroomPolicyV2" not in run
    assert worker.SUBSCRIBER_PERSISTENT_MEMBERSHIP is True and worker.fixed_contract()["policy_deadline_seconds"]==.1
    for name,pin in worker.V15_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V15_PINS)==4 and len(worker.V15_EVIDENCE)==3
    for component in (worker,owner):
        source=Path(component.__file__).read_text()
        assert "V15_" in source
    source=Path(owner.__file__).read_text()
    assert "v15_result[\"source_sha256_after\"]==v15_plan[\"source_sha256\"]" in source
    assert "v15_result[\"input_sha256_after\"]==v15_plan[\"input_sha256\"]" in source and "input_pins.update(worker.V15_EVIDENCE)" in source


def test_v16_missing_actual_V15_provenance_denies_before_preparation_write(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V15_EVIDENCE",{})
    monkeypatch.setattr(worker,"budget",lambda **kw:pytest.fail("preparation progressed before complete V15 provenance"))
    with pytest.raises(ValueError,match="completed V15"):owner.prepare(worker)


def test_v18c_original_error_and_event_counts_survive_combined_default_tamper_restore_failure(node,worker):
    fixture=history_fixture(node,worker,candidate=True);policy=fixture.policy
    policy.model=SimpleNamespace(forward_cached=lambda *a,**kw:{})
    original=RuntimeError("original policy failure before restore guard")
    def underlying(*args,**kwargs):
        build_history(fixture)
        fixture.binding["function"].__kwdefaults__["past_frames"]=("mutation",)
        raise original
    audited=node.DeferredAuditedAdapter(SimpleNamespace(_policy=policy,accept=underlying),worker,before_accept=lambda:None)
    with pytest.raises(ValueError,match="builder code/defaults") as caught:audited.accept()
    tail=audited.failure_tail;prior=tail["prior_failure_tail"]
    assert caught.value.__context__ is original and tail["original_exception_context"]==repr(original)
    assert tail["error"]==tail["restore_guard_error"]==repr(caught.value)
    assert prior["error"]==repr(original) and prior["stage"]=="policy_or_pending_capture"
    assert tail["tensor_events"]==prior["tensor_events"]==1 and tail["NN_events"]==prior["NN_events"]==0
    assert tail["live_event_lists_already_cleared"] is True and audited._pending is None
    assert tail["another_forward_executed"] is False and tail["tensor_hashes_materialized"] is False
    assert policy.build_inputs is fixture.binding["bound"] and "forward_cached" in policy.model.__dict__
    with pytest.raises(ValueError):audited.flush_pending()


def test_v18c_production_inverse_exact_only_restore_guard_failure_record_changed(node,worker,owner):
    old_path=ROOT/"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v16.py"
    old_tree=ast.parse(old_path.read_bytes());new_tree=ast.parse(Path(node.__file__).read_bytes())
    old={part.name:part for part in old_tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    new={part.name:part for part in new_tree.body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    assert old.keys()==new.keys()
    def restore_guard_body(part):
        result=deepcopy(part)
        for current in ast.walk(result):
            if isinstance(current,ast.Try) and len(current.body)==1 and isinstance(current.body[0],ast.Expr) and ast.unparse(current.body[0].value).startswith("verify_past_hit_builder("):
                original=next(previous for previous in ast.walk(old["DeferredAuditedAdapter"]) if isinstance(previous,ast.Try) and len(previous.body)==1 and
                    isinstance(previous.body[0],ast.Expr) and ast.unparse(previous.body[0].value).startswith("verify_past_hit_builder("))
                current.handlers=deepcopy(original.handlers)
        return result
    for name in old:assert ast.dump(old[name],include_attributes=False)==ast.dump(restore_only_post_spin_poll(restore_guard_body(new[name])),include_attributes=False),name
    for name,pin in worker.V16_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V16_PINS)==4 and len(worker.V16_PREPARED_EVIDENCE)==1
    path=next(iter(worker.V16_PREPARED_EVIDENCE))
    assert path==str(ROOT/"storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v16/candidate_plan.json")
    assert list(worker.V16_PREPARED_EVIDENCE.values())==["5702e4a1c123f129b633907579dfc513861a8aa105daa703e4ff89b42338e4cd"]
    assert "input_pins.update(worker.V16_PREPARED_EVIDENCE)" in Path(owner.__file__).read_text()
    assert worker.fixed_contract()["raw_atol"]==worker.fixed_contract()["raw_rtol"]==1e-6


def test_v18c_missing_stopped_V16_candidate_denies_before_preparation_write(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V16_PREPARED_EVIDENCE",{})
    monkeypatch.setattr(worker,"budget",lambda **kw:pytest.fail("preparation progressed before stopped V16 provenance"))
    with pytest.raises(ValueError,match="unapproved V16"):owner.prepare(worker)


def v18_subscriber_loop_fixture(node, *, whole_loop=False):
    # HH_261008 - Compile ONLY the actual owned subscriber scheduling seam; no ROS/model implementation is imported.
    run=next(part for part in ast.parse(Path(node.__file__).read_bytes()).body if isinstance(part,ast.FunctionDef) and part.name=="run")
    loops=[part for part in ast.walk(run) if isinstance(part,ast.While) and ast.unparse(part.test)=="node.last + 1 < 918 and node.failed is None"]
    assert len(loops)==1
    tree=ast.parse("def iteration(node, worker, ros, graph_guard, all_topics, expected):\n    pass\n")
    tree.body[0].body=[deepcopy(loops[0])] if whole_loop else deepcopy(loops[0].body)
    namespace={};exec(compile(ast.fix_missing_locations(tree),"<actual V18 subscriber scheduling seam>","exec"),namespace)
    return namespace["iteration"]


def v18c_method_external_globals(methods):
    # HH_261008 - Compiler lexical scopes from genuine AST include globals used only inside generator/comprehension bodies.
    required=set()
    def inspect_scope(scope):
        required.update(symbol.get_name() for symbol in scope.get_symbols() if symbol.is_global() and symbol.is_referenced())
        for child in scope.get_children():inspect_scope(child)
    for method in methods:
        table=symtable.symtable(ast.unparse(method),"<genuine original method AST>","exec")
        inspect_scope(table.get_children()[0])
    return required


def v18c_verify_method_namespace(methods,namespace):
    required=v18c_method_external_globals(methods)
    missing=sorted(name for name in required if name not in namespace and not hasattr(builtins,name))
    if missing:raise ValueError("genuine original method external globals missing: "+repr(missing))
    return required


def v18_original_poll_fixture(node,worker,*,ordinal=0):
    # HH_261008 - Genuine frozen poll/fail AST, fake scalar ROS/admission/build/accept ONLY; no numerical ABI or live DDS proof.
    tree=ast.parse(node.OLD.read_bytes())
    recorded=next(part for part in tree.body if isinstance(part,ast.FunctionDef) and part.name=="recorded_node")
    original_class=next(part for part in recorded.body if isinstance(part,ast.ClassDef) and part.name=="ShadowNode")
    methods={part.name:part for part in original_class.body if isinstance(part,ast.FunctionDef)}
    selected=[deepcopy(methods[name]) for name in ("poll","fail")]
    events=[];counter=dict(fake_accepts=0,fake_frames=0,fake_resets=0);now=[1000]
    def clock():now[0]+=1000;return now[0]
    stamp=SimpleNamespace(sec=0,nanosec=200)
    header=lambda frame:SimpleNamespace(stamp=stamp,frame_id=frame)
    roles=("front","front_left","front_right","rear","rear_left","rear_right")
    optical={role:"optical/"+role for role in roles}
    anchor=SimpleNamespace(ordinal=ordinal,native_frame=100+ordinal,timestamp_ns=200,
        provenance={"fixture_only":True,"ordinal":ordinal},state=dict(x=0.,y=0.,z=0.,yaw=0.,command="LANEFOLLOW"))
    # HH_261008 - Next-index metadata is explicitly synthetic; it contains no recorded future, geometry, image or learning label.
    anchors=[SimpleNamespace(ordinal=i,native_frame=100+i,timestamp_ns=200+(i+1)*100,synthetic_metadata_only=True) for i in range(918)]
    anchors[ordinal]=anchor
    index=SimpleNamespace(anchors=anchors,context=dict(optical_frames=optical,rig_id="synthetic-fixture-not-a-real-rig"))
    def publish(topic):
        return SimpleNamespace(publish=lambda msg:events.append(("fake_publish",topic,msg.data)))
    def fake_accept(*args,**kwargs):
        counter["fake_accepts"]+=1;events.append("fake_original_accept")
        return SimpleNamespace(status="PREFILL",selected=None,diagnostics={"synthetic_no_NN":True})
    def reset(reason):counter["fake_resets"]+=1;events.append(("fake_reset",reason))
    def fake_build(*args):counter["fake_frames"]+=1;events.append("fake_original_build");return SimpleNamespace()
    adapter=SimpleNamespace(accept=fake_accept,reset=reset)
    pure=SimpleNamespace(ROLES=roles,EXPECTED_CAMERAS=918,SCHEMA="fake.fixture.not_runtime",
        require=worker.need,decode=lambda raw,limit:json.loads(raw),encode=worker.encode,
        verify_transport=lambda *args:1,verify_navigation=lambda *args:"LANEFOLLOW")
    def unsupported_native_semantic(*args,**kwargs):raise AssertionError("tiny scalar fixture has no native calibration/conversion implementation")
    ros=SimpleNamespace(String=lambda **kwargs:SimpleNamespace(**kwargs),Float32=lambda **kwargs:SimpleNamespace(**kwargs),Int8=lambda **kwargs:SimpleNamespace(**kwargs),
        Time=unsupported_native_semantic,ClockType=SimpleNamespace(ROS_TIME="synthetic-not-native-ROS-time"),
        old=SimpleNamespace(camera_info_message_to_live_calibration=unsupported_native_semantic,validate_live_extrinsic=unsupported_native_semantic,
            transform_message_to_matrix=unsupported_native_semantic,make_shadow_trajectory=unsupported_native_semantic,make_shadow_path=unsupported_native_semantic))
    authority=dict(replay_id="synthetic-fixture-only-not-ROOT-approval",admitted_authority_sha256="a"*64,TF_limits=dict(translation_m=.005,rotation_rad=.005))
    inputs={"provenance":SimpleNamespace(data='{"provenance":{"fixture_only":true}}'),
        "navigation":SimpleNamespace(data='{"fixture_only":true}'),
        "odometry":SimpleNamespace(header=header("map"),child_frame_id="base_link",
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.,z=0.),orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.),angular=SimpleNamespace(z=0.)))),
        "acceleration":SimpleNamespace(header=header("base_link"),accel=SimpleNamespace(accel=SimpleNamespace(linear=SimpleNamespace(x=0.,y=0.)))),
        "steering":SimpleNamespace(stamp=stamp,steering_tire_angle=0.),"clock":SimpleNamespace(clock=stamp)}
    for role in roles:
        inputs[role]=SimpleNamespace(header=header(optical[role]),format="jpeg",data=b"synthetic payload, not image evidence")
        inputs[role+"_info"]=SimpleNamespace(header=header(optical[role]))
    target=SimpleNamespace(last=ordinal-1,failed=None,inputs=inputs,first_camera_ns={200:1},tf_seen={name:200 for name in optical.values()},
        admission=SimpleNamespace(admit=lambda *args,**kwargs:(anchor,[],[]),denominator=lambda:{"fixture_only":True}),
        tf=SimpleNamespace(lookup_transform=unsupported_native_semantic),trajectory=publish("trajectory"),path=publish("path"),candidate=publish("candidate"),latency=publish("latency"),status=publish("status"),receipt=publish("ACK"))
    namespace=dict(pure=pure,index=index,rig=SimpleNamespace(cameras=[],features=[]),contract=SimpleNamespace(),execution={"fixture_only":True},
        ros=ros,adapter=adapter,authority=authority,asdict=asdict,validate_live_camera_calibration=unsupported_native_semantic,
        time=SimpleNamespace(perf_counter_ns=clock),deepcopy=deepcopy,math=math,
        stamp_ns=lambda value:value.sec*1000000000+value.nanosec,build_observed_frame=fake_build,print=lambda *args,**kwargs:events.append("fake_original_console"))
    v18c_verify_method_namespace(selected,namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),str(node.OLD),"exec"),namespace)
    from types import MethodType
    target.poll=MethodType(namespace["poll"],target);target.fail=MethodType(namespace["fail"],target)
    return SimpleNamespace(node=target,events=events,counter=counter,original_poll=target.poll)


@pytest.mark.parametrize("fault",["missing_navigation","missing_image","wrong_TF_stamp","stale_camera_stamp"])
def test_v18c_actual_original_poll_incomplete_inputs_never_build_or_accept(node,worker,fault):
    fixture=v18_original_poll_fixture(node,worker)
    if fault=="missing_navigation":fixture.node.inputs.pop("navigation")
    elif fault=="missing_image":fixture.node.inputs.pop("front")
    elif fault=="wrong_TF_stamp":fixture.node.tf_seen["optical/front"]=199
    else:fixture.node.inputs["front"].header.stamp=SimpleNamespace(sec=0,nanosec=199)
    events=fixture.events
    def spin(*args,**kwargs):events.extend(["native_receive_finish","owned_spin_return"])
    worker_fake=SimpleNamespace(live_owner=lambda:events.append("owned_peer"))
    ros_fake=SimpleNamespace(rclpy=SimpleNamespace(spin_once=spin))
    v18_subscriber_loop_fixture(node)(fixture.node,worker_fake,ros_fake,lambda *args:events.append("graph_guard"),set(),set())
    assert fixture.counter==dict(fake_accepts=0,fake_frames=0,fake_resets=0) and fixture.node.last==-1 and fixture.node.failed is None
    assert events==["owned_peer","graph_guard","native_receive_finish","owned_spin_return"]


def test_v18c_original_timer_already_processed_has_no_duplicate_accept_or_outputs(node,worker):
    fixture=v18_original_poll_fixture(node,worker);events=fixture.events
    def spin(*args,**kwargs):
        events.append("original_10ms_timer_handler");fixture.original_poll();events.append("owned_spin_return")
    iteration=v18_subscriber_loop_fixture(node)
    iteration(fixture.node,SimpleNamespace(live_owner=lambda:None),SimpleNamespace(rclpy=SimpleNamespace(spin_once=spin)),lambda *args:None,set(),set())
    assert fixture.counter["fake_accepts"]==fixture.counter["fake_frames"]==1 and fixture.node.last==0
    assert sum(isinstance(event,tuple) and event[:2]==("fake_publish","ACK") for event in events)==1
    assert not fixture.node.inputs and not fixture.node.first_camera_ns
    fixture.original_poll()
    assert fixture.counter["fake_accepts"]==1


def test_v18c_native_receive_finishes_and_spin_returns_before_original_poll(node,worker):
    fixture=v18_original_poll_fixture(node,worker);events=fixture.events
    def observed_poll():events.append("original_poll_enter");return fixture.original_poll()
    fixture.node.poll=observed_poll
    def spin(*args,**kwargs):events.extend(["native_take_complete","receive_callback_finished","native_execute_finished","owned_spin_return"])
    v18_subscriber_loop_fixture(node)(fixture.node,SimpleNamespace(live_owner=lambda:None),SimpleNamespace(rclpy=SimpleNamespace(spin_once=spin)),lambda *args:None,set(),set())
    assert events.index("receive_callback_finished")<events.index("native_execute_finished")<events.index("owned_spin_return")<events.index("original_poll_enter")<events.index("fake_original_build")
    assert fixture.counter["fake_accepts"]==1


@pytest.mark.parametrize("kind",["failed","last918"])
def test_v18c_whole_actual_while_guard_skips_failure_and_completed918(node,worker,kind):
    fixture=v18_original_poll_fixture(node,worker)
    if kind=="failed":fixture.node.failed="retained_original_failure"
    else:fixture.node.last=917
    def deny(*args,**kwargs):pytest.fail("completed/failed subscriber must not spin or start another poll")
    v18_subscriber_loop_fixture(node,whole_loop=True)(fixture.node,SimpleNamespace(live_owner=deny),SimpleNamespace(rclpy=SimpleNamespace(spin_once=deny)),deny,set(),set())
    fixture.original_poll()
    assert fixture.counter==dict(fake_accepts=0,fake_frames=0,fake_resets=0) and fixture.events==[]


def test_v18c_real_poll_at_last_declared_anchor_completes_once_then_loop_stops(node,worker):
    fixture=v18_original_poll_fixture(node,worker,ordinal=917);events=fixture.events
    def spin(*args,**kwargs):events.append("one_owned_spin_return")
    v18_subscriber_loop_fixture(node,whole_loop=True)(fixture.node,SimpleNamespace(live_owner=lambda:None),SimpleNamespace(rclpy=SimpleNamespace(spin_once=spin)),lambda *args:None,set(),set())
    assert fixture.node.last==917 and fixture.counter["fake_accepts"]==1 and events.count("one_owned_spin_return")==1
    fixture.original_poll()
    assert fixture.counter["fake_accepts"]==1


def test_v18c_spin_failure_preserves_original_exception_without_added_poll(node,worker):
    fixture=v18_original_poll_fixture(node,worker)
    def spin(*args,**kwargs):raise RuntimeError("original_owned_executor_failure")
    with pytest.raises(RuntimeError,match="original_owned_executor_failure"):
        v18_subscriber_loop_fixture(node)(fixture.node,SimpleNamespace(live_owner=lambda:None),SimpleNamespace(rclpy=SimpleNamespace(spin_once=spin)),lambda *args:None,set(),set())
    assert fixture.counter==dict(fake_accepts=0,fake_frames=0,fake_resets=0) and fixture.node.last==-1


def test_v18c_original_admission_failure_keeps_one_reset_ACK_and_never_accepts(node,worker):
    fixture=v18_original_poll_fixture(node,worker)
    fixture.node.inputs["clock"].clock=SimpleNamespace(sec=0,nanosec=199)
    iteration=v18_subscriber_loop_fixture(node)
    args=(fixture.node,SimpleNamespace(live_owner=lambda:None),SimpleNamespace(rclpy=SimpleNamespace(spin_once=lambda *args,**kwargs:None)),lambda *args:None,set(),set())
    iteration(*args);iteration(*args)
    assert fixture.node.failed is not None and fixture.counter==dict(fake_accepts=0,fake_frames=0,fake_resets=1)
    assert sum(isinstance(event,tuple) and event[:2]==("fake_publish","ACK") for event in fixture.events)==1


def test_v18c_exact_one_seam_inverse_AST_original_timer_and_full_provenance(worker,node,owner):
    def functions(path):return {part.name:part for part in ast.parse(Path(path).read_bytes()).body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    for current,oldpath,excluded in ((node,"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v16b.py",set()),
        (worker,"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v16b.py",{"source_pins","fixed_contract","worker_command"}),
        (owner,"tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v16b.py",{"prepare"})):
        previous=functions(ROOT/oldpath);actual=functions(current.__file__)
        assert previous.keys()==actual.keys()
        for name in previous.keys()-excluded:assert ast.dump(previous[name],include_attributes=False)==ast.dump(restore_only_post_spin_poll(actual[name]),include_attributes=False),name
    run=functions(node.__file__)["run"]
    poll_calls=[part for part in ast.walk(run) if isinstance(part,ast.Call) and ast.unparse(part.func)=="node.poll"]
    assert len(poll_calls)==1 and not poll_calls[0].args and not poll_calls[0].keywords
    loop=next(part for part in ast.walk(run) if isinstance(part,ast.While) and ast.unparse(part.test)=="node.last + 1 < 918 and node.failed is None")
    assert ast.unparse(loop.body[-1])=="node.poll()" and "spin_once" in ast.unparse(loop.body[-2])
    original=ast.parse(node.OLD.read_bytes())
    timers=[part for part in ast.walk(original) if isinstance(part,ast.Call) and isinstance(part.func,ast.Attribute) and part.func.attr=="create_timer"]
    assert len(timers)==1 and timers[0].args[0].value==.01 and ast.unparse(timers[0].args[1])=="self.poll"
    assert worker.fixed_contract()["policy_deadline_seconds"]==.1 and worker.fixed_contract()["camera_anchors"]==918
    assert worker.fixed_contract()["poll_runs_inside_receive_callback"] is False
    assert worker.fixed_contract()["additional_NN_warmup_calls"]==0 and "CPU_affinity" not in worker.fixed_contract()
    for name,pin in worker.V16B_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    assert len(worker.V16B_EVIDENCE)==3
    assert "input_pins.update(worker.V16B_EVIDENCE)" in Path(owner.__file__).read_text()


def test_v18c_missing_actual_V16b_provenance_denied_before_prepare(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V16B_EVIDENCE",{})
    monkeypatch.setattr(worker,"budget",lambda **kwargs:pytest.fail("source-only preparation passed missing actual V16b evidence"))
    with pytest.raises(ValueError,match="completed V16b"):owner.prepare(worker)


def test_v18c_correct_tiny_context_and_next_anchor_metadata_is_not_a_future_label(node,worker):
    fixture=v18_original_poll_fixture(node,worker)
    namespace=fixture.original_poll.__func__.__globals__
    index=namespace["index"]
    assert index.context["rig_id"]=="synthetic-fixture-not-a-real-rig"
    following=index.anchors[1]
    assert following.synthetic_metadata_only is True and following.timestamp_ns!=index.anchors[0].timestamp_ns
    assert not hasattr(following,"state") and not hasattr(following,"cameras") and not hasattr(following,"provenance")
    fixture.original_poll();fixture.original_poll()
    assert fixture.node.last==0 and fixture.node.failed is None
    assert fixture.counter==dict(fake_accepts=1,fake_frames=1,fake_resets=0)


def test_v18c_stopped_source_only_finding_and_original_four_bytes_preserved(worker):
    assert len(worker.V18_PINS)==4 and len(worker.V18_SOURCE_FINDING)==1
    for path,pin in worker.V18_PINS.items():assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==pin
    path,pin=next(iter(worker.V18_SOURCE_FINDING.items()))
    raw=Path(path).read_bytes();assert hashlib.sha256(raw).hexdigest()==pin
    finding=json.loads(raw)
    assert finding["source_sha256"]==worker.V18_PINS
    assert finding["status"]=="STOPPED_SOURCE_ONLY_NOT_A_TEST_OR_RUNTIME_FAILURE"
    assert finding["author_actual_tests"]==finding["author_actual_prepare_attempts"]==finding["author_NN_inferences"]==finding["author_ROS_runs"]==0


def test_v18c_all_node_and_scientific_AST_identical_stopped_V18(node,worker,owner):
    def functions(path):return {part.name:part for part in ast.parse(Path(path).read_bytes()).body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    for current,oldpath,excluded in ((node,"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v18.py",set()),
        (worker,"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v18.py",{"source_pins","worker_command"}),
        (owner,"tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v18.py",{"prepare"})):
        previous=functions(ROOT/oldpath);actual=functions(current.__file__)
        assert previous.keys()==actual.keys()
        for name in previous.keys()-excluded:assert ast.dump(previous[name],include_attributes=False)==ast.dump(actual[name],include_attributes=False),name
    assert "input_pins.update(worker.V18_SOURCE_FINDING)" in Path(owner.__file__).read_text()


def test_v18c_missing_original_source_finding_denies_before_budget(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V18_SOURCE_FINDING",{})
    monkeypatch.setattr(worker,"budget",lambda **kwargs:pytest.fail("missing stopped V18 finding passed admission"))
    with pytest.raises(ValueError,match="preserved V18 source-only"):owner.prepare(worker)


def v18c_genuine_poll_fail_AST(node):
    recorded=next(part for part in ast.parse(node.OLD.read_bytes()).body if isinstance(part,ast.FunctionDef) and part.name=="recorded_node")
    original_class=next(part for part in recorded.body if isinstance(part,ast.ClassDef) and part.name=="ShadowNode")
    return [part for part in original_class.body if isinstance(part,ast.FunctionDef) and part.name in ("poll","fail")]


def test_v18c_every_original_method_external_global_bound_before_invocation(node,worker):
    fixture=v18_original_poll_fixture(node,worker)
    namespace=fixture.original_poll.__func__.__globals__;methods=v18c_genuine_poll_fail_AST(node)
    required=v18c_verify_method_namespace(methods,namespace)
    expected={"pure","index","time","stamp_ns","authority","deepcopy","math","rig","validate_live_camera_calibration",
        "ros","build_observed_frame","contract","adapter","execution","asdict"}
    assert {name for name in required if not hasattr(builtins,name)}==expected
    assert namespace["asdict"] is asdict and callable(namespace["validate_live_camera_calibration"])
    for name in expected:
        incomplete=dict(namespace);incomplete.pop(name)
        with pytest.raises(ValueError,match="external globals missing"):v18c_verify_method_namespace(methods,incomplete)
    assert fixture.counter==dict(fake_accepts=0,fake_frames=0,fake_resets=0)


def test_v18c_every_literal_authority_and_index_context_key_exists_in_scalar_fixture(node,worker):
    fixture=v18_original_poll_fixture(node,worker);namespace=fixture.original_poll.__func__.__globals__
    def key_path(part):
        if isinstance(part,ast.Name):return [part.id]
        if isinstance(part,ast.Attribute):
            base=key_path(part.value)
            return None if base is None else base+[part.attr]
        if isinstance(part,ast.Subscript) and isinstance(part.slice,ast.Constant) and type(part.slice.value) is str:
            base=key_path(part.value)
            return None if base is None else base+[part.slice.value]
        return None
    paths=set()
    for method in v18c_genuine_poll_fail_AST(node):
        for part in ast.walk(method):
            path=key_path(part)
            if isinstance(part,ast.Subscript) and path and (path[0]=="authority" or path[:2]==["index","context"]):paths.add(tuple(path))
    assert paths=={("authority","replay_id"),("authority","admitted_authority_sha256"),("authority","TF_limits"),
        ("authority","TF_limits","translation_m"),("authority","TF_limits","rotation_rad"),("index","context","optical_frames"),("index","context","rig_id")}
    for path in paths:
        value=namespace[path[0]]
        for key in path[1:]:value=value[key] if isinstance(value,dict) else getattr(value,key)
        assert value is not None
    assert namespace["authority"]["replay_id"]=="synthetic-fixture-only-not-ROOT-approval"
    assert namespace["authority"]["TF_limits"]==dict(translation_m=.005,rotation_rad=.005)


def test_v18c_preserved_actual_V18b_five_pure_failures_without_ROS_or_NN(worker):
    assert len(worker.V18B_PINS)==4 and len(worker.V18B_TEST_FAILURE)==1
    for name,pin in worker.V18B_PINS.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==pin
    path,pin=next(iter(worker.V18B_TEST_FAILURE.items()));raw=Path(path).read_bytes()
    assert hashlib.sha256(raw).hexdigest()==pin
    failure=json.loads(raw)
    assert failure["source_sha256"]==worker.V18B_PINS
    assert failure["passed"]==356 and failure["failed"]==5 and failure["actual_exit_code"]==1
    assert failure["prepare_after_failed_suite"] is False and failure["ROOT_plan_or_authority_created"] is False
    assert failure["actual_NN_inferences"]==failure["actual_ROS_runs"]==failure["training_steps"]==0


def test_v18c_all_production_node_AST_identical_failed_V18b(node,worker,owner):
    def functions(path):return {part.name:part for part in ast.parse(Path(path).read_bytes()).body if isinstance(part,(ast.FunctionDef,ast.ClassDef))}
    for current,oldpath,excluded in ((node,"autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_20261008_v18b.py",set()),
        (worker,"tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v18b.py",{"source_pins","worker_command"}),
        (owner,"tools/e2e/run_owned_temporal_recorded_ros_shadow_20261008_v18b.py",{"prepare"})):
        previous=functions(ROOT/oldpath);actual=functions(current.__file__)
        assert previous.keys()==actual.keys()
        for name in previous.keys()-excluded:assert ast.dump(previous[name],include_attributes=False)==ast.dump(actual[name],include_attributes=False),name
    assert "input_pins.update(worker.V18B_TEST_FAILURE)" in Path(owner.__file__).read_text()


def test_v18c_missing_actual_pure_failure_denies_before_preparation(owner,worker,monkeypatch):
    monkeypatch.setattr(worker,"V18B_TEST_FAILURE",{})
    monkeypatch.setattr(worker,"budget",lambda **kwargs:pytest.fail("missing actual V18b pure-failure evidence passed preparation"))
    with pytest.raises(ValueError,match="preserved actual V18b"):owner.prepare(worker)
