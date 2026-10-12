"""HH_261008 - Preserve stopped V18; use the identical one post-spin original poll call and unchanged scientific/timing code.

The historical policy, tensor construction and 100 ms rejection remain unchanged.
Audit references are released after each anchor; no archived original-BGRA,
historical V5 terminal equivalence, actuation or feature approval is asserted.
"""
from __future__ import annotations

import ast
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from types import MappingProxyType, MethodType, SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
WORKER = ROOT / "tools/e2e/temporal_recorded_ros_shadow_worker_20261008_v18b.py"
OLD = ROOT / "autonomy/autoware_e2e_vad_launch/scripts/temporal_recorded_shadow_v1_20261005.py"


def bindings():
    import importlib.util
    spec = importlib.util.spec_from_file_location("_hh_recorded_worker_261007", WORKER)
    module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def compile_original(worker):
    # HH_261007 - Compile the genuine frozen functions in a PRIVATE namespace, not a flag-opened old main.
    worker.need(worker.digest(OLD) == worker.FROZEN[str(OLD.relative_to(ROOT))], "frozen node source drift")
    tree = ast.parse(OLD.read_bytes())
    names = {"stamp_ns", "stamp", "dependencies", "message_set", "load_runtime", "build_observed_frame", "recorded_node", "recorded_publisher"}
    namespace = dict(ROOT=ROOT, deepcopy=deepcopy, asdict=asdict, Path=Path, sys=sys, time=time, json=json,
        SimpleNamespace=SimpleNamespace, math=math, __name__="_hh_private_frozen_node", __file__=str(OLD))
    import importlib.util
    namespace["importlib"] = importlib
    functions = [item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name in names]
    worker.need({item.name for item in functions} == names, "exact frozen private delegation seams required")
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(OLD), "exec"), namespace)
    return namespace


def tensor_receipt(tensors):
    result = {}
    for name, value in sorted(tensors.items()):
        array = value.detach().cpu().contiguous().numpy()
        # HH_261008 - Hash the same contiguous byte span without allocating array.tobytes(); no concurrent-mutation/snapshot guarantee is added.
        result[name] = dict(dtype=str(array.dtype), shape=list(array.shape), sha256=hashlib.sha256(memoryview(array).cast("B")).hexdigest())
    return result


def past_hit_builder_ast(worker):
    # HH_261008 - Inverse-proof the exact original builder; no broader mixin/MRO/predict path is imported or activated.
    for name,pin in worker.BUILDER_PINS.items():worker.need(worker.digest(ROOT/name)==pin,"frozen normalized builder changed")
    original_path=ROOT/next(iter(worker.BUILDER_PINS))
    tree=ast.parse(original_path.read_bytes())
    original_class=next(part for part in tree.body if isinstance(part,ast.ClassDef) and part.name=="TemporalSimulationResearchPolicyV2")
    original=next(part for part in original_class.body if isinstance(part,ast.FunctionDef) and part.name=="build_inputs")
    candidate=deepcopy(original)
    hits=[part for part in ast.walk(candidate) if isinstance(part,ast.Assign) and ast.unparse(part)=="image = self._images[key].clone()"]
    worker.need(len(hits)==1,"exact single original past-cache-hit clone required")
    hits[0].value=ast.parse("self._images[key]",mode="eval").body
    inverse=deepcopy(candidate)
    plain=[part for part in ast.walk(inverse) if isinstance(part,ast.Assign) and ast.unparse(part)=="image = self._images[key]"]
    worker.need(len(plain)==1,"exact single inverse cache-hit assignment required")
    plain[0].value=deepcopy(next(part for part in ast.walk(original) if isinstance(part,ast.Assign) and ast.unparse(part)=="image = self._images[key].clone()").value)
    worker.need(ast.dump(inverse,include_attributes=False)==ast.dump(original,include_attributes=False),"original complete builder inverse AST differs")
    mixin_tree=ast.parse((ROOT/list(worker.BUILDER_PINS)[1]).read_bytes())
    mixin=next(part for part in mixin_tree.body if isinstance(part,ast.ClassDef) and part.name=="_SingleCopyBuildMixin")
    precedent=next(part for part in mixin.body if isinstance(part,ast.FunctionDef) and part.name=="build_inputs")
    worker.need(ast.dump(ast.Module(body=candidate.body[1:],type_ignores=[]),include_attributes=False)==
        ast.dump(ast.Module(body=precedent.body,type_ignores=[]),include_attributes=False),"candidate differs from the exact frozen single-copy builder body")
    return ast.fix_missing_locations(candidate),original_path


def verify_past_hit_builder(policy,worker,expected):
    # HH_261008 - Cheap live identity checks include audit-hook restoration; file hashing is admission-only, outside anchors.
    if expected is None:return
    worker.need(policy.__dict__.get("_hh_past_hit_binding") is expected and policy.build_inputs is expected["bound"],"admitted private builder binding changed")
    function=expected["function"];genuine=expected["genuine"]
    worker.need(type(policy) is expected["class"] and expected["class"].build_inputs is genuine and
        function.__code__ is expected["code"] and function.__defaults__ is expected["defaults"] and
        function.__kwdefaults__==expected["kwdefaults"] and genuine.__code__ is expected["genuine_code"] and
        genuine.__defaults__ is expected["genuine_defaults"] and genuine.__kwdefaults__==expected["genuine_kwdefaults"],
        "private builder code/defaults/original class changed")
    worker.need(all(function.__globals__.get(name) is value and genuine.__globals__.get(name) is value
        for name,value in expected["globals"].items()),"private/original builder providers changed")


def install_past_hit_builder(adapter,worker):
    candidate,path=past_hit_builder_ast(worker)
    policy=adapter._policy;module=sys.modules.get("portable_e2e.temporal_simulation_research_v2_20261001")
    worker.need(module is not None and Path(module.__file__).resolve(strict=True)==path,"actual original policy source identity differs")
    cls=module.TemporalSimulationResearchPolicyV2;genuine=cls.build_inputs
    worker.need(type(policy) is cls and "build_inputs" not in policy.__dict__ and "_hh_past_hit_binding" not in policy.__dict__ and
        policy.build_inputs.__func__ is genuine and genuine.__globals__ is vars(module) and genuine.__code__.co_filename==str(path),
        "exact unmodified original policy constructor/class/builder required")
    names={part.id for part in ast.walk(candidate) if isinstance(part,ast.Name) and isinstance(part.ctx,ast.Load)} & set(genuine.__globals__)
    providers={name:genuine.__globals__[name] for name in names}
    namespace=dict(genuine.__globals__)
    exec(compile(ast.Module(body=[candidate],type_ignores=[]),str(path),"exec"),namespace)
    function=namespace["build_inputs"];bound=MethodType(function,policy)
    expected=MappingProxyType(dict(bound=bound,function=function,code=function.__code__,defaults=function.__defaults__,
        kwdefaults=MappingProxyType(dict(function.__kwdefaults__)),genuine=genuine,genuine_code=genuine.__code__,
        genuine_defaults=genuine.__defaults__,genuine_kwdefaults=MappingProxyType(dict(genuine.__kwdefaults__)),
        globals=MappingProxyType(providers),**{"class":cls}))
    policy.build_inputs=bound;policy._hh_past_hit_binding=expected
    verify_past_hit_builder(policy,worker,expected)
    return expected


def finite_nested(value):
    if type(value) is dict: return all(finite_nested(item) for item in value.values())
    if type(value) is list: return all(finite_nested(item) for item in value)
    return type(value) in (int,float) and math.isfinite(value)


def publication_timing(begin,entered,finished=None):
    # HH_261007 - Report actual post-guard/serialization entry and completion, not an earlier gate timestamp.
    if not (type(begin) is int and type(entered) is int and 0<begin<=entered):raise ValueError("actual monotonic publication interval required")
    if finished is not None and not (type(finished) is int and entered<=finished):raise ValueError("actual publication completion chronology differs")
    return dict(publisher_bundle_started_ns=begin,actual_publish_entered_ns=entered,actual_publish_completed_ns=finished,
        entry_deadline_passed=entered-begin<=100_000_000,completion_deadline_passed=None if finished is None else finished-begin<=100_000_000,
        no_original_gate_substitution=True)


class AuditedAdapter:
    """HH_261007 - Observe authentic tensors/raw heads AFTER policy returns, without another inference."""
    def __init__(self, adapter, worker):
        self.adapter, self.worker = adapter, worker
        self.past_hit_binding=adapter._policy.__dict__.get("_hh_past_hit_binding")

    def reset(self, *args, **kwargs):
        return self.adapter.reset(*args, **kwargs)

    def accept(self, *args, **kwargs):
        policy = self.adapter._policy; model = policy.model; tensors = []; outputs = []
        verify_past_hit_builder(policy,self.worker,self.past_hit_binding)
        original_build, original_forward = policy.build_inputs, model.forward_cached
        build_owned = "build_inputs" in policy.__dict__; old_build = policy.__dict__.get("build_inputs")
        forward_owned = "forward_cached" in model.__dict__; old_forward = model.__dict__.get("forward_cached")
        def build(*a, **kw):
            value = original_build(*a, **kw); tensors.append(value[0]); return value
        def forward(*a, **kw):
            value = original_forward(*a, **kw); outputs.append(value); return value
        try:
            policy.build_inputs, model.forward_cached = build, forward
            result = self.adapter.accept(*args, **kwargs)
        finally:
            if build_owned: policy.build_inputs = old_build
            else: del policy.build_inputs
            if forward_owned: model.forward_cached = old_forward
            else: del model.forward_cached
            verify_past_hit_builder(policy,self.worker,self.past_hit_binding)
        try:
            self.worker.need(len(tensors) <= 1 and len(outputs) <= 1, "one actual tensor/NN event per anchor required")
            audit = dict(schema=self.worker.SCHEMA+".numerical_event", tensor_available=bool(tensors), NN_executed=bool(outputs),
                tensor_sha256=tensor_receipt(tensors[0]) if tensors else None, raw_output=None,
                frozen_policy_status=result.status, receipt_hashed_after_policy_return=True, another_forward_executed=False)
            if outputs:
                values = {name: value.detach().cpu().tolist() for name, value in outputs[0].items()}
                self.worker.need(finite_nested(values), "actual raw heads finite required")
                audit["raw_output"] = values
                audit["raw_tensor_sha256"] = tensor_receipt(outputs[0])
            result.diagnostics["owned_recorded_numerical_event"] = audit
            return result
        finally:
            # HH_261007 - Never accumulate 918 image tensors/raw tensor references in RAM.
            tensors.clear(); outputs.clear()


def tensor_identity_snapshot(mapping):
    # HH_261007 - Only cheap identities and version counters are captured before publication, never tensor values.
    if type(mapping) is not dict or any(type(name) is not str for name in mapping):
        raise ValueError("exact tensor dictionary required for deferred audit")
    names=tuple(sorted(mapping));values={name:mapping[name] for name in names};versions={}
    for name,value in values.items():
        version=value._version
        if type(version) is not int or version<0:raise ValueError("tracked tensor in-place version required")
        versions[name]=version
    return dict(mapping=mapping,names=names,values=values,versions=versions)


def verify_tensor_identity(snapshot):
    mapping=snapshot["mapping"]
    if tuple(sorted(mapping))!=snapshot["names"]:raise ValueError("pending tensor dictionary keys changed")
    for name,value in snapshot["values"].items():
        if mapping[name] is not value or type(value._version) is not int or value._version!=snapshot["versions"][name]:
            raise ValueError("pending tensor identity/in-place version changed: "+name)


class DeferredAuditedAdapter(AuditedAdapter):
    """HH_261007 - Exactly one original inference event; version-bound audit is materialized at the owned ACK seam."""
    def __init__(self,adapter,worker,*,before_accept):
        super().__init__(adapter,worker)
        worker.need(callable(before_accept),"mandatory source-owned pre-accept graph guard required")
        self.before_accept=before_accept
        self._pending=None;self.failure_tail=None;self.last_materialized_result=None;self.last_timing=None

    def accept(self,*args,**kwargs):
        self.worker.need(self._pending is None and self.failure_tail is None,"prior audit must flush successfully before next anchor")
        # HH_261007 - Deny an unknown current graph before touching original tensor/NN providers, not merely at an earlier callback.
        self.before_accept()
        policy=self.adapter._policy;model=policy.model;tensors=[];outputs=[];capture_ns=0
        verify_past_hit_builder(policy,self.worker,self.past_hit_binding)
        original_build,original_forward=policy.build_inputs,model.forward_cached
        build_owned="build_inputs" in policy.__dict__;old_build=policy.__dict__.get("build_inputs")
        forward_owned="forward_cached" in model.__dict__;old_forward=model.__dict__.get("forward_cached")
        def build(*a,**kw):
            nonlocal capture_ns
            value=original_build(*a,**kw);begun=time.perf_counter_ns()
            tensors.append(tensor_identity_snapshot(value[0]));capture_ns+=time.perf_counter_ns()-begun
            return value
        def forward(*a,**kw):
            nonlocal capture_ns
            value=original_forward(*a,**kw);begun=time.perf_counter_ns()
            outputs.append(tensor_identity_snapshot(value));capture_ns+=time.perf_counter_ns()-begun
            return value
        try:
            policy.build_inputs,model.forward_cached=build,forward
            result=self.adapter.accept(*args,**kwargs)
            attachment_started=time.perf_counter_ns()
            self.worker.need(len(tensors)<=1 and len(outputs)<=1,"one actual tensor/NN event per anchor required")
            for snapshot in tensors+outputs:verify_tensor_identity(snapshot)
            captured=time.perf_counter_ns()
            self._pending=dict(result=result,tensors=tensors,outputs=outputs,captured_ns=captured)
            result.diagnostics["owned_recorded_numerical_event"]=dict(schema=self.worker.SCHEMA+".numerical_event",
                tensor_available=bool(tensors),NN_executed=bool(outputs),tensor_sha256=None,raw_output=None,
                frozen_policy_status=result.status,deferred_audit_pending=True,another_forward_executed=False)
            capture_ns+=time.perf_counter_ns()-attachment_started
            result.diagnostics["owned_recorded_deferred_capture"]=dict(defer_capture_seconds=capture_ns/1e9,
                measured_scope="tensor_identity_version_snapshots_and_after_policy_validation_attachment_only",
                excludes_original_build_forward_policy_and_method_hook_restore=True,hash_work_measured_separately_at_flush=True)
            return result
        except BaseException as failure:
            self.failure_tail=dict(stage="policy_or_pending_capture",error=repr(failure),tensor_events=len(tensors),NN_events=len(outputs),
                tensor_hashes_materialized=False,raw_heads_materialized=False,another_forward_executed=False,partial_forward_execution_may_have_occurred=True)
            # HH_261007 - Failed diagnostic attachment cannot leave an empty pending event that later invents a no-NN receipt.
            self._pending=None;tensors.clear();outputs.clear();raise
        finally:
            if build_owned:policy.build_inputs=old_build
            else:del policy.build_inputs
            if forward_owned:model.forward_cached=old_forward
            else:del model.forward_cached
            try:verify_past_hit_builder(policy,self.worker,self.past_hit_binding)
            except BaseException as failure:
                # HH_261008 - Preserve an earlier failure/counts even when that handler already cleared the live event lists.
                prior=deepcopy(self.failure_tail)
                self.failure_tail=dict(stage="private_builder_restore_guard",error=repr(failure),restore_guard_error=repr(failure),
                    original_exception_context=None if failure.__context__ is None else repr(failure.__context__),prior_failure_tail=prior,
                    tensor_events=len(tensors) if prior is None else prior["tensor_events"],NN_events=len(outputs) if prior is None else prior["NN_events"],
                    live_event_lists_already_cleared=prior is not None,tensor_hashes_materialized=False,raw_heads_materialized=False,
                    another_forward_executed=False,partial_forward_execution_may_have_occurred=True)
                self._pending=None;tensors.clear();outputs.clear();raise

    def flush_pending(self,timestamp_ns=None,context="ACK"):
        if self._pending is None:
            self.worker.need(self.failure_tail is None,"prior deferred audit failed; no fabricated receipt")
            return None
        pending=self._pending;result=pending["result"];tensors=pending["tensors"];outputs=pending["outputs"]
        begun=time.perf_counter_ns()
        try:
            self.worker.need(timestamp_ns is None or result.anchor_ns==timestamp_ns,"pending numerical event belongs to another ACK anchor")
            for snapshot in tensors+outputs:verify_tensor_identity(snapshot)
            audit=dict(schema=self.worker.SCHEMA+".numerical_event",tensor_available=bool(tensors),NN_executed=bool(outputs),
                tensor_sha256=tensor_receipt(tensors[0]["mapping"]) if tensors else None,raw_output=None,
                frozen_policy_status=result.status,receipt_hashed_after_policy_return=True,another_forward_executed=False)
            if outputs:
                values={name:value.detach().cpu().tolist() for name,value in outputs[0]["mapping"].items()}
                self.worker.need(finite_nested(values),"actual raw heads finite required")
                audit["raw_output"]=values;audit["raw_tensor_sha256"]=tensor_receipt(outputs[0]["mapping"])
            for snapshot in tensors+outputs:verify_tensor_identity(snapshot)
            result.diagnostics["owned_recorded_numerical_event"]=audit
            finished=time.perf_counter_ns()
            self.last_timing=dict(captured_ns=pending["captured_ns"],materialization_started_ns=begun,
                materialization_finished_ns=finished,audit_flush_started_ns=begun,audit_flush_finished_ns=finished,
                audit_flush_seconds=(finished-begun)/1e9,context=context,
                after_policy_return=True,subscriber_audit_deferred=True,ACK_completion_includes_audit_cost=True)
            result.diagnostics["owned_recorded_deferred_audit_timing"]=self.last_timing
            self.last_materialized_result=result
            return audit
        except BaseException as failure:
            self.failure_tail=dict(stage="deferred_materialization",anchor_ns=getattr(result,"anchor_ns",None),
                frozen_policy_status=result.status,error=repr(failure),tensor_events=len(tensors),NN_events=len(outputs),
                captured_ns=pending["captured_ns"],materialization_started_ns=begun,
                tensor_hashes_materialized=False,raw_heads_materialized=False,another_forward_executed=False)
            raise
        finally:
            tensors.clear();outputs.clear();self._pending=None

    def reset(self,*args,**kwargs):
        try:
            if self._pending is not None:self.flush_pending(context="before_original_reset")
        finally:
            # HH_261007 - Preserve genuine failure/reset semantics even if the audit itself fails closed.
            reset_result=self.adapter.reset(*args,**kwargs)
        return reset_result

    def close_audit(self):
        if self._pending is not None:self.flush_pending(context="closing_without_ACK")
        return dict(pending_reference_count=0,failure_tail=self.failure_tail,last_materialization=self.last_timing,
            whole_interval_audit_claimed=False,another_forward_executed=False)


def materialize_owned_ACK(value,audited):
    # HH_261007 - Modify only new owned diagnostic fields, never status, selection, geometry or original control contents.
    actual=deepcopy(value);audit=audited.flush_pending(timestamp_ns=value["timestamp_ns"],context="before_ACK_serialization")
    result=audited.last_materialized_result
    if audit is None and result is not None and result.anchor_ns==value["timestamp_ns"]:
        audit=result.diagnostics.get("owned_recorded_numerical_event")
    if audit is not None:
        diagnostics=actual.setdefault("policy_diagnostics",{})
        diagnostics["owned_recorded_numerical_event"]=audit
        diagnostics["owned_recorded_deferred_audit_timing"]=audited.last_timing
        diagnostics["owned_recorded_deferred_capture"]=result.diagnostics["owned_recorded_deferred_capture"]
    actual["owned_subscriber_numerical_audit_deferred_until_ACK"]=True
    actual["owned_original_status_message_may_have_pending_audit"]=True
    actual["owned_ACK_completion_includes_deferred_audit"]=True
    return actual


def topic_sets(pure):
    incoming = {pure.PROVENANCE_TOPIC, pure.INPUT_PREFIX+"/navigation", *(pure.INPUT_PREFIX+"/"+k for k in ("odometry", "acceleration", "steering", "clock", "tf")),
        *(pure.INPUT_PREFIX+"/"+role+suffix for role in pure.ROLES for suffix in ("/compressed", "/camera_info"))}
    outgoing = {pure.ACK_TOPIC, *("/planning/portable_e2e/"+suffix for suffix in ("shadow_trajectory", "shadow_path", "status", "latency_ms", "selected_candidate"))}
    return incoming, outgoing


def graph_guard(node, allowed, expected_names):
    names = node.get_node_names_and_namespaces()
    if len(names) != len(set(names)) or any(namespace != "/" or name not in expected_names for name, namespace in names):
        raise ValueError("unowned/duplicate ROS graph node detected")
    topics = {name for name, _ in node.get_topic_names_and_types()}
    if not topics <= allowed | {"/parameter_events"}: raise ValueError("nonallowlisted ROS graph topic detected")
    return dict(nodes=names, topics=sorted(topics), DDS_security_identity_claimed=False)


def receive_graph_guard(node,topic,provenance_topic,allowed,expected_names):
    # HH_261007 - The first provenance input retains a full census; each other message retains true peer/GID/type/CDR checks.
    if topic==provenance_topic:return graph_guard(node,allowed,expected_names)
    return None


def active_shadow_graph_guard(nodes,allowed,expected_names,owned_node_type,owner_token):
    # HH_261007 - A destroyed startup probe is not a valid pre-NN graph source; use exactly one live source-owned shadow object.
    selected=[value for value in nodes if getattr(value,"_hh_graph_active",False) is True
        and getattr(value,"owned_name",None)=="portable_temporal_recorded_shadow"]
    if len(selected)!=1:raise ValueError("exactly one active owned shadow node required before inference")
    actual=selected[0]
    if not isinstance(actual,owned_node_type) or getattr(actual,"_hh_graph_owner_token",None) is not owner_token:
        raise ValueError("actual shadow node is not the source-owned registry instance")
    if actual.get_name()!="portable_temporal_recorded_shadow" or actual.get_namespace()!="/":
        raise ValueError("actual shadow node name/namespace differs before inference")
    actual.peer()
    return graph_guard(actual,allowed,expected_names)


def authenticated_gid(node, topic, info, expected_name):
    # HH_261007 - Reject absent actual transport metadata before consulting the owned graph endpoint.
    if not hasattr(info, "publisher_gid"):
        raise ValueError("actual DDS MessageInfo publisher GID is missing")
    endpoints = node.get_publishers_info_by_topic(topic)
    if len(endpoints) != 1: raise ValueError("exactly one declared DDS publisher required")
    endpoint = endpoints[0]
    if endpoint.node_name != expected_name or endpoint.node_namespace != "/" or bytes(endpoint.endpoint_gid) != bytes(info.publisher_gid):
        raise ValueError("actual DDS MessageInfo publisher GID differs from owned graph peer")
    return bytes(info.publisher_gid).hex()


def load_native_bridge(view, worker):
    # HH_261007 - Load only the separately admitted native module; never patch installed rclpy.
    declared = view["native_dds_bridge"]
    path = Path(declared["module_path"])
    worker.need(worker.digest(path) == declared["module_sha256"], "native DDS bridge module SHA differs")
    name = "_hh_owned_dds_info_261007"
    worker.need(name not in sys.modules, "native DDS bridge must be freshly loaded from the sealed path")
    bridge = worker.load_module(path, name)
    worker.need(Path(bridge.__file__).resolve(strict=True) == path and bridge.__name__ == name
        and callable(getattr(bridge, "take_owned", None)), "exact native DDS bridge origin/API required")
    return bridge


def native_metadata_tuple(value):
    # HH_261007 - Missing transport identity is a failure, not a graph-derived or synthesized GID.
    if type(value) is not tuple or len(value) != 2 or type(value[1]) is not dict:
        raise ValueError("actual native DDS message/metadata tuple required")
    message, metadata = value
    if message is None or any(type(key) is not str for key in metadata):
        raise ValueError("actual native DDS message and named metadata required")
    gid = metadata.get("publisher_gid")
    if type(gid) is not bytes or len(gid) != 24:
        raise ValueError("actual native 24-byte DDS publisher GID is missing or malformed")
    return message, metadata


def native_transport_witness(info, topic, message_type):
    # HH_261007 - Retain true taken handles and public-DDS GUID normalization, never replace sender identity from the graph.
    metadata = vars(info)
    expected = {"source_timestamp", "received_timestamp", "publication_sequence_number", "reception_sequence_number",
        "publisher_gid", "implementation_identifier", "from_intra_process", "raw_rmw_publisher_gid",
        "actual_publication_handle", "dds_reader", "dds_participant", "dds_reader_guid", "dds_topic_name",
        "dds_type_name", "dds_local_entity_count", "publisher_gid_provenance"}
    if set(metadata) != expected or metadata["implementation_identifier"] != "rmw_cyclonedds_cpp":
        raise ValueError("exact sealed Cyclone native metadata required")
    for key, size in (("publisher_gid",24),("raw_rmw_publisher_gid",24),("dds_reader_guid",16)):
        if type(metadata[key]) is not bytes or len(metadata[key]) != size:
            raise ValueError("exact native GID/reader GUID bytes required")
    raw = metadata["raw_rmw_publisher_gid"]; handle = metadata["actual_publication_handle"]
    if (type(handle) is not int or not 0 < handle < 2**64 or raw[8:] != bytes(16)
        or int.from_bytes(raw[:8], sys.byteorder) != handle or metadata["publisher_gid"][16:] != bytes(8)):
        raise ValueError("unknown actual Cyclone handle/GUID ABI denied")
    if (metadata["publisher_gid_provenance"] != "public_dds_matched_publication_data_for_actual_taken_handle"
        or metadata["dds_topic_name"] != "rt"+topic or not isinstance(message_type, type)):
        raise ValueError("actual matched endpoint provenance/topic required")
    expected_type = message_type.__module__.split(".")[0]+"::msg::dds_::"+message_type.__name__+"_"
    if metadata["dds_type_name"] != expected_type:
        raise ValueError("actual matched endpoint differs from immutable generated message type")
    for key in ("dds_reader","dds_participant","dds_local_entity_count"):
        if type(metadata[key]) is not int or metadata[key] <= 0:
            raise ValueError("actual native reader/participant/entity count required")
    if metadata["dds_local_entity_count"] > 512 or metadata["from_intra_process"] is not False:
        raise ValueError("bounded cross-process DDS receipt required")
    for key in ("source_timestamp","received_timestamp"):
        if type(metadata[key]) is not int or not 0 <= metadata[key] < 2**63:
            raise ValueError("native timestamp integer required; zero unsupported receipt timestamp retained")
    for key in ("publication_sequence_number","reception_sequence_number"):
        if type(metadata[key]) is not int or not 0 <= metadata[key] < 2**64:
            raise ValueError("native uint64 sequence required; unsupported UINT64_MAX retained")
    return {key:value.hex() if type(value) is bytes else value for key,value in metadata.items()}


def typed_subscription_identity(subscription):
    # HH_261007 - Native rcl_take requires the original typed allocation; retain identities before any take.
    message_type = subscription.msg_type
    raw = subscription.raw
    if not isinstance(message_type, type) or raw is not False or not callable(subscription.callback):
        raise ValueError("registered genuine typed subscription contract required")
    metaclass = type(message_type)
    capsules = {name:getattr(metaclass, name, None) for name in
        ("_TYPE_SUPPORT", "_CREATE_ROS_MESSAGE", "_DESTROY_ROS_MESSAGE", "_CONVERT_TO_PY")}
    if any(value is None for value in capsules.values()):
        raise ValueError("registered generated message capsules must already be initialized")
    return dict(subscription=subscription, handle=subscription.handle, message_type=message_type,
        metaclass=metaclass, raw=raw, callback=subscription.callback, capsules=capsules)


RECEIVE_STAGE_NAMES=("peer","provenance_graph","publisher_gid","native_witness","CDR","CDR_hash","callback")
NATIVE_STAGE_NAMES=("native_C_take_and_matched_endpoint_lookup","native_take_tuple_validation","native_execute_tuple_validation")


def measured_local_stage(stages,name,callback,*args,**kwargs):
    # HH_261007 - These are local monotonic measurements, never timestamps asserted to originate from DDS.
    if type(stages) is not list or len(stages)>=8 or name not in RECEIVE_STAGE_NAMES+NATIVE_STAGE_NAMES:
        raise ValueError("bounded declared local timing stages required")
    stage=dict(stage=name,begun_ns=None,finished_ns=None,duration_ns=None,failed_at_ns=None,failed_elapsed_ns=None,error=None)
    stages.append(stage)
    try:begun=time.perf_counter_ns()
    except BaseException as clock_failure:
        stage["clock_error"]="start read failed: "+repr(clock_failure);raise
    if type(begun) is not int or begun<=0:
        stage["clock_error"]="invalid actual start value: "+repr(begun)
        raise ValueError("actual integer local timing start required")
    stage["begun_ns"]=begun
    try:value=callback(*args,**kwargs)
    except BaseException as failure:
        stage["error"]=repr(failure)
        try:failed=time.perf_counter_ns()
        except BaseException as clock_failure:
            stage["clock_error"]="failure clock read failed: "+repr(clock_failure)
            raise failure
        if type(failed) is int and failed>=begun:
            stage.update(failed_at_ns=failed,failed_elapsed_ns=failed-begun)
        else:stage["clock_error"]="invalid actual failure clock; no fabricated ending"
        raise
    try:finished=time.perf_counter_ns()
    except BaseException as clock_failure:
        stage["clock_error"]="completion read failed: "+repr(clock_failure);raise
    if type(finished) is not int or finished<begun:
        stage["clock_error"]="invalid actual completion clock; no fabricated ending"
        raise ValueError("actual integer local timing completion required")
    stage.update(finished_ns=finished,duration_ns=finished-begun)
    return value


def compact_local_stages(stages,names):
    # HH_261007 - Keep actual starts, ends and integer durations while bounding diagnostic serialization overhead.
    compact=[]
    for stage in stages:
        if stage["stage"] not in names:raise ValueError("timing stage schema differs")
        begun,finished,duration=stage["begun_ns"],stage["finished_ns"],stage["duration_ns"]
        if type(begun) is not int or begun<=0:raise ValueError("actual integer timing start required")
        if finished is not None and (type(finished) is not int or finished<begun or type(duration) is not int or duration!=finished-begun):
            raise ValueError("actual integer timing end/duration required")
        if finished is None and duration is not None:raise ValueError("unfinished stage has no successful duration")
        failed,elapsed=stage["failed_at_ns"],stage["failed_elapsed_ns"]
        if failed is not None and (type(failed) is not int or failed<begun or type(elapsed) is not int or elapsed!=failed-begun):
            raise ValueError("actual integer failure timing required")
        if failed is None and elapsed is not None:raise ValueError("unknown failure clock cannot imply elapsed time")
        values=[names.index(stage["stage"]),stage["begun_ns"],stage["finished_ns"],stage["duration_ns"]]
        if stage["error"] is not None or "clock_error" in stage:
            values.extend((stage["failed_at_ns"],stage["failed_elapsed_ns"],stage["error"],stage.get("clock_error")))
        compact.append(values)
    return compact


def compact_receive_timing(rows,receipt,input_cdr):
    # HH_261007 - An original admitted receipt may bind observed input hashes/stamps; a failed ACK alone is not input-admission proof.
    provenance=receipt.get("source_selected_callback_provenance")
    if provenance is not None:
        if type(provenance) is not dict or type(provenance.get("ordinal")) is not int or type(provenance.get("timestamp_ns")) is not int \
            or provenance.get("ordinal")!=receipt.get("ordinal") \
            or provenance.get("timestamp_ns")!=receipt.get("timestamp_ns"):
            raise ValueError("authenticated original receipt provenance must bind its actual anchor")
    actual=[]
    for row in rows:
        if any(type(stamp) is not int or stamp<0 for stamp in row["observed_stamps_ns"]):
            raise ValueError("actual observed input stamp integers required")
        bound=provenance is not None and row["observed_provenance"]==provenance \
            and row["CDR_sha256"] is not None and input_cdr.get(row["topic"])==row["CDR_sha256"]
        if bound and any(stamp!=receipt["timestamp_ns"] for stamp in row["observed_stamps_ns"]):
            raise ValueError("observed input topic stamps differ from original admitted anchor")
        actual.append(dict(topic=row["topic"],observed_stamps_ns=list(row["observed_stamps_ns"]),
            provenance_graph_full_census_performed=row["provenance_graph_full_census_performed"],
            CDR_sha256=row["CDR_sha256"],anchor_binding="original_admission_and_actual_CDR_and_observed_stamps" if bound else "UNKNOWN",
            ordinal=receipt["ordinal"] if bound else None,
            native_stages=compact_local_stages(row["native_stages"],NATIVE_STAGE_NAMES),
            receive_stages=compact_local_stages(row["stages"],RECEIVE_STAGE_NAMES)))
    return dict(native_stage_names=list(NATIVE_STAGE_NAMES),receive_stage_names=list(RECEIVE_STAGE_NAMES),
        stage_columns=["stage_index","begun_ns","finished_ns","duration_ns","failed_at_ns","failed_elapsed_ns","error","clock_error"],
        trailing_success_null_fields_omitted=True,messages=actual,observed_count=len(actual),
        original_authenticated_provenance=deepcopy(provenance),padded_missing_messages=False,
        local_clock_only=True,pure_DDS_network_delay_claimed=False,stage_sum_is_wall_latency_claimed=False,
        unmeasured_residual_includes_sender_work_scheduling_and_original_ROS_admission=True)


def retain_unfinished_receive_rows(rows):
    # HH_261007 - A genuine failure ACK can occur inside an input callback; retain that active stage until its real return/closing.
    rows[:]=[row for row in rows if any(stage["finished_ns"] is None and stage["failed_at_ns"] is None
        and stage["error"] is None and "clock_error" not in stage for stage in row["stages"])]


def raw_timing_diagnostic(value):
    # HH_261007 - Retain invalid raw scalar evidence as explicit representations, never as accepted timing or fabricated anchor data.
    if type(value) in (str,int,bool,type(None)):return value
    if type(value) is float:
        return value if math.isfinite(value) else dict(raw_type="float",raw_repr=repr(value),accepted_as_timing=False)
    if type(value) in (list,tuple):return [raw_timing_diagnostic(item) for item in value]
    if type(value) is dict:return {str(key):raw_timing_diagnostic(item) for key,item in value.items()}
    return dict(raw_type=type(value).__name__,raw_repr=repr(value),accepted_as_timing=False)


def receive_timing_closing_tail(rows):
    try:return dict(timing=compact_receive_timing(rows,{},{}),compaction_error=None)
    except BaseException as failure:
        return dict(timing=None,compaction_error=repr(failure),raw_observed_timing_rows=raw_timing_diagnostic(rows),
            raw_only_not_valid_timing=True,anchor_binding="UNKNOWN",whole_interval_profile_claimed=False)


def metadata_executor_type(base_executor, bridge, await_or_execute,*,native_timing_enabled=False):
    # HH_261007 - Override only explicitly owned subscription takes; all other executor paths remain genuine.
    if type(native_timing_enabled) is not bool:raise ValueError("explicit local timing mode required")
    class MetadataExecutor(base_executor):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._hh_owned_subscriptions = {}
            self._hh_take_timing={};self._hh_take_failures=[];self._hh_take_overflow=None

        def native_timing_tail(self):
            return dict(pending=[dict(topic=value["topic"],native_stages=deepcopy(value["stages"]),
                actual_message_taken=True,native_info_delivered=value["info"] is not None,
                unconsumed=True) for value in self._hh_take_timing.values()],failures=deepcopy(self._hh_take_failures),
                overflow=deepcopy(self._hh_take_overflow),empty_native_takes_profiled=False,native_fields_modified=False)

        def release_native_timing(self):
            self._hh_take_timing.clear()

        def consume_native_timing(self,subscription,message,info,callback):
            if not native_timing_enabled:raise ValueError("native local timing is not enabled for this owned executor")
            key=id(subscription);value=self._hh_take_timing.get(key)
            if value is None or value["subscription"] is not subscription or value["message"] is not message \
                or value["info"] is not info or value["callback"] is not callback or subscription.callback is not callback \
                or vars(info)!=value["native_metadata"]:
                raise ValueError("native local timing must bind the exact actual subscription/message/info/callback once")
            self.verify_owned_subscription(subscription)
            self._hh_take_timing.pop(key)
            return deepcopy(value["stages"])

        def register_owned_subscription(self, subscription):
            if id(subscription) in self._hh_owned_subscriptions:
                raise ValueError("owned subscription is already registered; typed identity cannot be rebound")
            self._hh_owned_subscriptions[id(subscription)] = typed_subscription_identity(subscription)

        def owns_subscription(self, subscription):
            declared = self._hh_owned_subscriptions.get(id(subscription))
            return declared is not None and declared["subscription"] is subscription

        def verify_owned_subscription(self, subscription):
            # HH_261007 - Deny mutable type/handle/raw/callback/capsule drift BEFORE calling the native bridge.
            declared = self._hh_owned_subscriptions[id(subscription)]
            actual = typed_subscription_identity(subscription)
            for name in ("subscription", "handle", "message_type", "metaclass", "raw", "callback"):
                if actual[name] is not declared[name]:
                    raise ValueError("registered subscription identity changed: "+name)
            if any(actual["capsules"][name] is not value for name,value in declared["capsules"].items()):
                raise ValueError("registered subscription generated capsule identity changed")

        def _take_subscription(self, subscription):
            if not self.owns_subscription(subscription):
                return super()._take_subscription(subscription)
            self.verify_owned_subscription(subscription)
            # HH_261007 - Native take owns the real handle enter/exit; do not nest handle guards.
            if native_timing_enabled:
                key=id(subscription);stages=[]
                if key in self._hh_take_timing or len(self._hh_take_timing)>=32:
                    self._hh_take_overflow=dict(topic=subscription.topic_name,stage="before_native_take",error="native pending bound or duplicate",anchor_unknown=True)
                    raise ValueError("unconsumed native timing cannot be overwritten or silently truncated")
                try:
                    value=measured_local_stage(stages,NATIVE_STAGE_NAMES[0],bridge.take_owned,subscription)
                    if value is None:return None
                    message,metadata=measured_local_stage(stages,NATIVE_STAGE_NAMES[1],native_metadata_tuple,value)
                    self._hh_take_timing[key]=dict(subscription=subscription,message=message,native_metadata=metadata,
                        callback=subscription.callback,info=None,topic=subscription.topic_name,stages=stages)
                    return value
                except BaseException as failure:
                    if len(self._hh_take_failures)>=8:
                        self._hh_take_overflow=dict(topic=subscription.topic_name,stages=stages,error=repr(failure),anchor_unknown=True)
                        raise ValueError("native timing failure receipt bound exceeded; prior8 and overflow retained") from failure
                    self._hh_take_failures.append(dict(topic=subscription.topic_name,stages=stages,error=repr(failure),anchor_unknown=True))
                    raise
            value = bridge.take_owned(subscription)
            return None if value is None else native_metadata_tuple(value)

        async def _execute_subscription(self, subscription, value):
            if not self.owns_subscription(subscription):
                return await super()._execute_subscription(subscription, value)
            self.verify_owned_subscription(subscription)
            if value is None:
                return None
            if native_timing_enabled:
                pending=self._hh_take_timing.get(id(subscription))
                if pending is None or pending["info"] is not None:raise ValueError("one actual native take timing required before owned callback")
                message,metadata=measured_local_stage(pending["stages"],NATIVE_STAGE_NAMES[2],native_metadata_tuple,value)
                if pending["subscription"] is not subscription or pending["message"] is not message \
                    or pending["native_metadata"] is not metadata or pending["callback"] is not subscription.callback:
                    raise ValueError("actual taken message/native metadata/callback identity changed")
                info=SimpleNamespace(**metadata);pending["info"]=info
                return await await_or_execute(subscription.callback,message,info)
            message, metadata = native_metadata_tuple(value)
            return await await_or_execute(subscription.callback, message, SimpleNamespace(**metadata))

    return MetadataExecutor


def detach_private_node(node, executors):
    # HH_261008 - Genuine public Node.executor=None clears its weakref; remove_node/destroy_node alone do not.
    current=node.executor
    if current is not None:
        if not any(current is value for value in executors):raise ValueError("foreign executor must not be detached or mutated")
        node.executor=None
    if node.executor is not None or any(any(value is node for value in executor.get_nodes()) for executor in executors):
        raise ValueError("owned public executor detach did not complete")


class OwnedExecutorMembership:
    """HH_261008 - One run-token/exact-object registration for the actual live subscriber; no private executor field mutation."""
    def __init__(self,executor,owned_type,token,owned_nodes,live_guard):
        if not isinstance(owned_type,type) or type(owned_nodes) is not list or not callable(live_guard):raise ValueError("source-owned membership scope required")
        self.executor=executor;self.owned_type=owned_type;self.token=token;self.owned_nodes=owned_nodes;self.live_guard=live_guard
        self.registered={};self.closed=False;self.actual_node=None;self.actual_type=None
        self.verify_registry()
    def identity(self,node,*,live):
        if not isinstance(node,self.owned_type) or not any(node is value for value in self.owned_nodes) \
            or getattr(node,"_hh_graph_owner_token",None) is not self.token or getattr(node,"owned_name",None)!="portable_temporal_recorded_shadow":
            raise ValueError("exact source-owned node/object/run token required")
        if live and (self.closed or getattr(node,"_hh_graph_active",None) is not True):raise ValueError("destroyed node or closed private membership denied")
    def bind_actual_node(self,node):
        # HH_261008 - The genuine frozen factory returns ShadowNode(OwnedNode); bind its exact returned object AND concrete type.
        self.live_guard();self.identity(node,live=True);self.verify_registry()
        if self.actual_node is not None:raise ValueError("actual subscriber factory object already bound")
        if node.executor is not None:raise ValueError("actual subscriber factory returned foreign/stale membership")
        self.actual_node=node;self.actual_type=type(node)
    def verify_registry(self):
        nodes=self.executor.get_nodes()
        if type(nodes) is not list or len(nodes)!=len(self.registered) or len({id(node) for node in nodes})!=len(nodes) \
            or any(self.registered.get(id(node)) is not node for node in nodes):raise ValueError("foreign or duplicate private executor membership denied")
        for node in nodes:
            if node.executor is not self.executor:raise ValueError("public executor identity changed")
    def spin_once(self,node,*,timeout_sec=None):
        self.live_guard();self.identity(node,live=True);self.verify_registry()
        if node is not self.actual_node or type(node) is not self.actual_type:raise ValueError("exact genuine factory-returned subscriber object/type required")
        previous=self.registered.get(id(node))
        if previous is None:
            if self.registered or node.executor is not None:raise ValueError("one live subscriber and no foreign/stale executor required")
            if self.executor.add_node(node) is not True:raise ValueError("first private registration was duplicate or failed")
            self.registered[id(node)]=node;self.verify_registry()
        elif previous is not node or node.executor is not self.executor:raise ValueError("registered node identity changed")
        return self.executor.spin_once(timeout_sec=timeout_sec)
    def detach(self,node):
        self.identity(node,live=False);detach_private_node(node,(self.executor,));self.registered.pop(id(node),None);self.verify_registry()
    def finish(self):
        try:
            self.verify_registry()
            if self.registered:raise ValueError("owned persistent registrations remain at role close")
        finally:self.closed=True


def close_attempt(records,event,call):
    try:
        result=call();records.append(dict(event=event,success=True,error=None));return result
    except BaseException as failure:
        records.append(dict(event=event,success=False,error=repr(failure)));return None


def shutdown_private(executor,closed_executors,records,event):
    def close():
        if executor.shutdown(timeout_sec=0) is not True:raise ValueError("public executor shutdown did not finish")
        closed_executors.append(executor)
    close_attempt(records,event,close)


def close_owned_runtime(owned_nodes,outer_node,owned_type,token,executors,closed_executors,membership,shutdown,records):
    # HH_261008 - Close every tracked object, not only outer_node (publisher delegates leave that local as None).
    seen=set()
    for node in owned_nodes:
        if id(node) in seen:
            records.append(dict(event="duplicate_owned_node_tracking",success=False,error="duplicate exact object"));continue
        seen.add(id(node))
        if not isinstance(node,owned_type) or getattr(node,"_hh_graph_owner_token",None) is not token:
            records.append(dict(event="foreign_owned_node_tracking",success=False,error="source type/run token differs"));continue
        detach=lambda node=node: membership.detach(node) if membership is not None else detach_private_node(node,executors)
        close_attempt(records,"owned_node_detach:"+node.owned_name,detach)
        if node._hh_graph_active:close_attempt(records,"owned_node_destroy:"+node.owned_name,node.destroy_node)
    if outer_node is not None and id(outer_node) not in seen:
        close_attempt(records,"direct_outer_node_detach",lambda:detach_private_node(outer_node,executors))
        close_attempt(records,"direct_outer_node_destroy",outer_node.destroy_node)
    if membership is not None:close_attempt(records,"persistent_membership_finish",membership.finish)
    for executor in executors:
        if not any(executor is value for value in closed_executors):shutdown_private(executor,closed_executors,records,"private_executor_shutdown")
    close_attempt(records,"rclpy_context_shutdown",shutdown)


def private_rclpy_view(original, executor,*,membership=None):
    # HH_261007 - Frozen delegates use this local view; the installed module/global executor is untouched.
    private = SimpleNamespace(**vars(original))
    if membership is not None and (type(membership) is not OwnedExecutorMembership or membership.executor is not executor):raise ValueError("exact private persistent membership required")
    def spin_once(node, *, timeout_sec=None):
        if membership is not None:return membership.spin_once(node,timeout_sec=timeout_sec)
        executor.add_node(node)
        try:
            return executor.spin_once(timeout_sec=timeout_sec)
        finally:
            executor.remove_node(node)
    private.spin_once = spin_once
    return private


class OriginalConsoleWitness:
    """HH_261008 - Measure only actual original-console return; never replace or extend the original ACK clock/gates."""
    def __init__(self,worker,pure,index,view,mode):
        self.worker,self.pure,self.index,self.view,self.mode=worker,pure,index,view,mode
        self._ledger=None;self.rows=[]

    @property
    def ledger(self):
        # HH_261008 - No open codec stream can be orphaned by an early dependency/startup failure before an original print occurs.
        if self._ledger is None:self._ledger=self.worker.ledger(self.worker.OUTPUT/(self.mode+".stdout"))
        return self._ledger

    def print(self,*values,**kwargs):
        result=self.ledger.print(*values,**kwargs)
        clock_error=None
        try:finished=time.perf_counter_ns()
        except BaseException as failure:finished=None;clock_error=repr(failure)[:128]
        # HH_261008 - Parsing, binding and closing serialization happen after this mark and are explicitly excluded.
        self.worker.need(len(self.rows)<919,"complete original console denominator exceeded")
        row=dict(console_call=len(self.rows),original_console_returned_ns=finished,binding="UNKNOWN",
            ordinal=None,native_frame=None,timestamp_ns=None,receipt_arg_byte_sha256=None,
            actual_until_original_console_log_complete_ms=None,error=None,clock_error=clock_error)
        try:
            self.worker.need(len(values)==1 and type(values[0]) is str and kwargs==dict(flush=True),"exact genuine bounded one-argument print ABI required")
            raw=values[0].encode();row["receipt_arg_byte_sha256"]=hashlib.sha256(raw).hexdigest()
            value=self.pure.decode(raw,512*1024)
            if "ordinal" not in value:
                self.worker.need(self.mode=="publisher" and value.get("expected")==918,"unbound actual console summary")
                row["binding"]="ORIGINAL_CLOSING_SUMMARY_NOT_ANCHOR"
            else:
                ordinal=value["ordinal"]
                self.worker.need(type(ordinal) is int and 0<=ordinal<len(self.index.anchors),"actual printed ordinal absent/invalid")
                anchor=self.index.anchors[ordinal]
                self.worker.need(type(value.get("native_frame")) is int and value["native_frame"]==anchor.native_frame
                    and type(value.get("timestamp_ns")) is int and value["timestamp_ns"]==anchor.timestamp_ns,"actual printed frame/stamp join differs")
                status=value.get("status")
                self.worker.need(type(status) is str and status in ("PREFILL","REJECTED","TRAJECTORY","REJECTED_ROS_END_TO_END_DEADLINE","FAILED_RETAINED"),"actual original printed status unavailable/invalid")
                row.update(ordinal=ordinal,native_frame=value["native_frame"],timestamp_ns=value["timestamp_ns"],receipt_status=status)
                self.worker.need(value.get("schema")==self.pure.SCHEMA+".shadow_receipt"
                    and value.get("replay_id")==self.view["replay_id"]
                    and value.get("authority_sha256")==self.view["admitted_authority_sha256"],"actual printed receipt authority differs")
                provenance=value.get("source_selected_callback_provenance")
                self.worker.need(type(provenance) is dict and self.pure.encode(provenance)==self.pure.encode(anchor.provenance),"printed original source provenance unavailable/different")
                stages=value.get("stages",{});begin=stages.get("publisher_bundle_started_ns")
                self.worker.need(type(finished) is int and type(begin) is int and 0<begin<=finished,"original actual publisher-begin stage unavailable/invalid")
                end=stages.get("receipt_publication_finished_ns",stages.get("receipt_publication_entered_ns"))
                self.worker.need(type(end) is int and begin<=end<=finished,"original receipt stage chronology unavailable/invalid")
                self.worker.need(not any(previous.get("ordinal")==ordinal for previous in self.rows),"duplicate actual anchor console print retained")
                row.update(binding="ACTUAL_RECEIPT_SOURCE_JOIN",publisher_bundle_started_ns=begin,
                    source_provenance_sha256=hashlib.sha256(self.pure.encode(provenance)).hexdigest(),
                    original_receipt_stage_ns=end,actual_until_original_console_log_complete_ms=(finished-begin)/1e6)
        except BaseException as failure:
            error=repr(failure);row.update(error=error[:128],error_truncated=len(error)>128,error_sha256=hashlib.sha256(error.encode()).hexdigest())
        self.rows.append(row)
        return result

    def closing(self):
        ordinals={row["ordinal"] for row in self.rows if row["ordinal"] is not None}
        return dict(schema=self.worker.SCHEMA+".original_console_completion",mode=self.mode,expected_anchors=918,
            observed_original_console_calls=len(self.rows),actual_anchor_identities=len(ordinals),UNEXECUTED=918-len(ordinals),
            bound_anchor_measurements=sum(row["binding"]=="ACTUAL_RECEIPT_SOURCE_JOIN" for row in self.rows),
            UNKNOWN=sum(row["binding"]=="UNKNOWN" for row in self.rows),rows=self.rows,
            original_ACK_clock_unchanged=True,original_ACK_clock_includes_later_journal_or_console=False,
            witness_binding_and_ROLE_CLOSING_serialization_excluded_from_per_anchor_clock=True,
            additional_safety_gate_or_performance_PASS=False)


def run(mode, view):
    worker = bindings(); pure = worker.pure_proxy(view); ns = compile_original(worker)
    index = pure.load_index(view)
    console=OriginalConsoleWitness(worker,pure,index,view,mode) if mode in ("subscriber","publisher") else None
    if console is not None:ns["print"]=console.print
    ros = ns["dependencies"](view, pure)
    from rclpy.serialization import serialize_message
    from autoware_planning_msgs.msg import Trajectory
    from nav_msgs.msg import Path as PathMessage
    from rclpy.executors import SingleThreadedExecutor, await_or_execute
    # HH_261008 - Bind the source-reviewed installed PUBLIC lifetime API before any private executor construction.
    for name in ("rclpy.executors","rclpy.node"):
        path=Path(sys.modules[name].__file__).resolve(strict=True)
        worker.need(str(path) in worker.EXECUTOR_API_PINS and worker.digest(path)==worker.EXECUTOR_API_PINS[str(path)],"actual public executor API origin/source differs")
    import importlib
    import torch
    origins={};versions={}
    for name,expected_origin in view["expected_environment_origins"].items():
        module=importlib.import_module(name);path=Path(module.__file__);actual=path.resolve(strict=True)
        origin=dict(imported_path=str(path),resolved_path=str(actual),source_sha256=worker.digest(actual))
        worker.need(origin==expected_origin,"clean ROS/framework actual origin differs: "+name)
        origins[name]=origin;versions[name]=getattr(module,"__version__","not_exported")
    worker.need(versions==view["expected_environment_versions"] and not torch.cuda.is_initialized(),"clean CPU framework versions/CUDA state changed")
    worker.new_json(worker.OUTPUT/(mode+"_environment.json"),dict(origins=origins,versions=versions,CUDA_initialized=False,historical_V5_versions_verified=False))
    bridge = load_native_bridge(view, worker)
    worker.verify_project_modules(view, sys.modules.values())
    original_rclpy = ros.rclpy
    original_rclpy.init(args=[])
    MetadataExecutor = metadata_executor_type(SingleThreadedExecutor, bridge, await_or_execute,native_timing_enabled=mode=="subscriber")
    executor = MetadataExecutor(context=original_rclpy.get_default_context())
    owned_executors=[executor];closed_executors=[];membership=None;runtime_close_records=[]
    ros.rclpy = private_rclpy_view(original_rclpy, executor)
    incoming, outgoing = topic_sets(pure); all_topics = incoming | outgoing
    expected = {"portable_temporal_recorded_shadow", "portable_temporal_recorded_publisher"}
    completed = 0; error = None; node = None; records = worker.OUTPUT/(mode+"_anchors.jsonl")
    genuine_load = ns["load_runtime"]
    audited_adapters=[]; owned_nodes=[];owned_node_token=object()
    def before_subscriber_accept():
        worker.live_owner()
        return active_shadow_graph_guard(owned_nodes,all_topics,expected,OwnedNode,owned_node_token)
    def audited_load(*a):
        nonlocal executor,membership
        if mode == "subscriber":
            worker.need(membership is None,"subscriber startup probe/model load may occur only once")
            probe=None;probe_error=None;close_begin=len(runtime_close_records)
            try:
                probe = ros.Node("portable_temporal_recorded_shadow")
                until = time.monotonic()+.5
                while time.monotonic() < until:
                    graph_guard(probe, all_topics, {"portable_temporal_recorded_shadow"}); ros.rclpy.spin_once(probe, timeout_sec=.02)
            except BaseException as failure:probe_error=failure;raise
            finally:
                if probe is not None:close_attempt(runtime_close_records,"startup_probe_destroy",probe.destroy_node)
                shutdown_private(executor,closed_executors,runtime_close_records,"startup_probe_executor_shutdown")
                if probe_error is None and any(row["success"] is False for row in runtime_close_records[close_begin:]):raise ValueError("startup probe cleanup failed; no actual executor/model created")
            # HH_261008 - Public shutdown clears the probe iterator; construct a fresh actual private executor, never edit its private fields.
            executor=MetadataExecutor(context=original_rclpy.get_default_context());owned_executors.append(executor)
            membership=OwnedExecutorMembership(executor,OwnedNode,owned_node_token,owned_nodes,worker.live_owner)
            # HH_261008 - V14 fixes persistent membership; every other logging/scientific/lifetime statement matches V13.
            ros.rclpy=private_rclpy_view(original_rclpy,executor,membership=membership if worker.SUBSCRIBER_PERSISTENT_MEMBERSHIP else None)
        adapter, rig, contract, execution = genuine_load(*a)
        import torch
        worker.need(not torch.cuda.is_initialized() and torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1,
            "actual frozen CPU4/inter1/no CUDA execution required")
        # HH_261008 - Admit exactly this original-policy instance after strict loading, before audit wrappers capture its builder.
        install_past_hit_builder(adapter,worker)
        audited=DeferredAuditedAdapter(adapter,worker,before_accept=before_subscriber_accept) if mode=="subscriber" else AuditedAdapter(adapter,worker)
        audited_adapters.append(audited)
        return audited, rig, contract, execution
    ns["load_runtime"] = audited_load
    real_node = ros.Node
    class OwnedNode(real_node):
        def __init__(self, name):
            worker.need(name in expected, "exact declared node name required")
            super().__init__(name, enable_rosout=False, start_parameter_services=False)
            self._hh_graph_active=True;self._hh_graph_owner_token=owned_node_token
            self.owned_name = name; self.delayed = []; self.outputs = {}; self.graph_errors = []
            self.owned_input_hashes = {}; self.publish_hashes = {}; self.peer_generation = None
            self.owned_input_transport = {}; self.latest_transport = {}
            self.actual_output_calls=[];self.publisher_begin=None
            self._hh_receive_timing=[];self._hh_receive_overflow=None;self._hh_observed_provenance=None;self._hh_receive_journaled=False
            owned_nodes.append(self)
            graph_guard(self, all_topics, expected if mode != "direct" else {name})
            if name == "portable_temporal_recorded_publisher":
                qos = ros.QoSProfile(depth=16, reliability=ros.ReliabilityPolicy.RELIABLE)
                for cls, topic in ((Trajectory, "/planning/portable_e2e/shadow_trajectory"), (PathMessage, "/planning/portable_e2e/shadow_path")):
                    self.create_subscription(cls, topic, lambda msg, topic=topic: self.output_received(topic, msg), qos)
                self.create_timer(.005, self.deliver_receipts)

        def destroy_node(self):
            # HH_261008 - Detach through the genuine public setter at actual destruction; retain BOTH detach and destroy errors.
            failures=[];result=None
            try:
                try:
                    if membership is not None:membership.detach(self)
                    else:detach_private_node(self,owned_executors)
                except BaseException as failure:
                    runtime_close_records.append(dict(event="destroy_node_detach:"+self.owned_name,success=False,error=repr(failure)));failures.append(failure)
                try:result=super().destroy_node()
                except BaseException as failure:
                    runtime_close_records.append(dict(event="genuine_destroy_node:"+self.owned_name,success=False,error=repr(failure)));failures.append(failure)
            finally:self._hh_graph_active=False
            if failures:raise failures[0]
            return result

        def peer(self):
            worker.live_owner()
            role = "publisher" if self.owned_name.endswith("shadow") else "subscriber"
            receipt = worker.bounded_json(worker.OUTPUT/(role+"_started.json"))
            worker.same_generation(receipt["identity"])
            worker.need(receipt["plan_sha256"] == view["plan_sha256"] and receipt["authority_sha256"] == view["admitted_authority_sha256"], "peer plan/authority differs")

        def create_subscription(self, cls, topic, callback, qos, **kwargs):
            worker.need(topic in all_topics and self.resolve_topic_name(topic) == topic, "exact topic subscription allowlist required")
            def receive(msg, info):
                row=None
                if mode=="subscriber" and topic in incoming:
                    native=executor.consume_native_timing(subscription,msg,info,receive)
                    if len(self._hh_receive_timing)>=64:
                        self._hh_receive_overflow=dict(topic=topic,native_stages=native,error="receive timing bound exceeded",anchor_binding="UNKNOWN")
                        raise ValueError("all prior receive timing rows and overflow retained; no truncation")
                    row=dict(topic=topic,native_stages=native,stages=[],CDR_sha256=None,observed_stamps_ns=[],
                        observed_provenance=self._hh_observed_provenance,provenance_graph_full_census_performed=False)
                    self._hh_receive_timing.append(row)
                def invoke(name,call,*args):
                    return measured_local_stage(row["stages"],name,call,*args) if row is not None else call(*args)
                invoke("peer",self.peer)
                invoke("provenance_graph",receive_graph_guard,self,topic,pure.PROVENANCE_TOPIC,all_topics,expected)
                if row is not None:row["provenance_graph_full_census_performed"]=topic==pure.PROVENANCE_TOPIC
                gid = invoke("publisher_gid",authenticated_gid,self,topic,info,"portable_temporal_recorded_publisher" if topic in incoming else "portable_temporal_recorded_shadow")
                witness = invoke("native_witness",native_transport_witness,info,topic,cls)
                self.latest_transport[topic] = witness
                raw = invoke("CDR",serialize_message,msg); now = time.perf_counter_ns()
                if topic in incoming:
                    cdr_hash=invoke("CDR_hash",lambda:hashlib.sha256(raw).hexdigest())
                    self.owned_input_hashes[topic] = cdr_hash
                    self.owned_input_transport[topic] = witness
                    if row is not None:
                        row["CDR_sha256"]=cdr_hash
                        if hasattr(msg,"header"):row["observed_stamps_ns"]=[ns["stamp_ns"](msg.header.stamp)]
                        elif hasattr(msg,"clock"):row["observed_stamps_ns"]=[ns["stamp_ns"](msg.clock)]
                        elif hasattr(msg,"transforms"):row["observed_stamps_ns"]=[ns["stamp_ns"](item.header.stamp) for item in msg.transforms]
                    if topic==pure.PROVENANCE_TOPIC:
                        packet=pure.decode(msg.data.encode(),64*1024);self.publisher_begin=packet["publisher_bundle_started_ns"]
                        self.current_ordinal=packet["provenance"]["ordinal"]
                        if row is not None:
                            row["observed_stamps_ns"]=[packet["provenance"]["timestamp_ns"]]
                            # HH_261007 - This is only the observed packet until the frozen original admission authenticates it in a receipt.
                            self._hh_observed_provenance=packet["provenance"];row["observed_provenance"]=self._hh_observed_provenance
                if topic == pure.ACK_TOPIC:
                    value = pure.decode(msg.data.encode(), 128*1024)
                    self.delayed.append((msg, value, callback, now, gid, witness)); self.deliver_receipts(); return
                invoke("callback",callback,msg)
            subscription = super().create_subscription(cls, topic, receive, qos, **kwargs)
            executor.register_owned_subscription(subscription)
            return subscription

        def output_received(self, topic, msg):
            key = ns["stamp_ns"](msg.header.stamp)
            worker.need(hasattr(self,"current_ordinal") and key==index.anchors[self.current_ordinal].timestamp_ns,"DDS output is not the actual current anchor")
            worker.need((key, topic) not in self.outputs, "duplicate actual DDS output stamp/topic")
            self.outputs[key, topic] = dict(sha256=hashlib.sha256(serialize_message(msg)).hexdigest(),
                received_ns=time.perf_counter_ns(), actual_native_transport=self.latest_transport[topic])
            worker.append(worker.OUTPUT/"publisher_received_outputs.jsonl",dict(ordinal=self.current_ordinal,timestamp_ns=key,topic=topic,**self.outputs[key,topic]))

        def deliver_receipts(self):
            if not self.delayed: return
            msg, value, callback, received, gid, witness = self.delayed[0]
            stamp = value["timestamp_ns"]; selected_outputs = {}
            if value["status"] == "TRAJECTORY":
                keys = [(stamp, topic) for topic in ("/planning/portable_e2e/shadow_trajectory", "/planning/portable_e2e/shadow_path")]
                if not all(key in self.outputs for key in keys):
                    worker.need(time.perf_counter_ns()-received <= 5_000_000_000, "actual DDS trajectory/path absent after ACK; no invented output")
                    return
                selected_outputs = {topic: self.outputs.pop((stamp, topic)) for _, topic in keys}
            row = dict(value, actual_DDS_receipt_received_ns=received, actual_DDS_ACK_gid=gid,
                actual_DDS_ACK_transport=witness, actual_DDS_outputs=selected_outputs,
                published_input_CDR_sha256=dict(self.publish_hashes))
            worker.append(records, row); self.publish_hashes.clear()
            self.delayed.pop(0); callback(msg)

        def create_publisher(self, cls, topic, qos, **kwargs):
            if topic == "/parameter_events":
                # HH_261007 - rclpy Node creates this standard-only endpoint during its own constructor.
                return super().create_publisher(cls, topic, qos, **kwargs)
            worker.need(topic in all_topics and self.resolve_topic_name(topic) == topic, "exact publisher allowlist required")
            publisher = super().create_publisher(cls, topic, qos, **kwargs)
            real_publish = publisher.publish
            def publish(msg):
                worker.live_owner();graph_guard(self,all_topics,expected)
                if topic==pure.ACK_TOPIC:
                    worker.need(mode=="subscriber" and audited_adapters and isinstance(audited_adapters[-1],DeferredAuditedAdapter),"owned subscriber deferred ACK seam required")
                    value=pure.decode(msg.data.encode(),128*1024)
                    value=materialize_owned_ACK(value,audited_adapters[-1])
                    # HH_261007 - Timing copies/compaction/encoding stay inside the original full ACK completion clock.
                    value["owned_received_input_stage_timing"]=compact_receive_timing(self._hh_receive_timing,value,self.owned_input_hashes)
                    msg.data=pure.encode(value).decode()
                raw = serialize_message(msg); entered = time.perf_counter_ns()
                if topic in ("/planning/portable_e2e/shadow_trajectory","/planning/portable_e2e/shadow_path"):
                    timing=publication_timing(self.publisher_begin,entered)
                    event=dict(timing,ordinal=self.current_ordinal,topic=topic,CDR_sha256=hashlib.sha256(raw).hexdigest(),actually_published=False)
                    if not timing["entry_deadline_passed"]:
                        self.actual_output_calls.append(event)
                        worker.append(worker.OUTPUT/"subscriber_output_calls.jsonl",event)
                        raise ValueError("actual post-guard trajectory/path publication entry exceeds unchanged100ms")
                real_publish(msg); finished = time.perf_counter_ns()
                if topic in ("/planning/portable_e2e/shadow_trajectory","/planning/portable_e2e/shadow_path"):
                    event.update(publication_timing(self.publisher_begin,entered,finished),actually_published=True)
                    self.actual_output_calls.append(event);worker.append(worker.OUTPUT/"subscriber_output_calls.jsonl",event)
                    worker.need(event["completion_deadline_passed"],"actual trajectory/path publish completed late; already-emitted partial retained")
                if topic==pure.PROVENANCE_TOPIC:
                    packet=pure.decode(msg.data.encode(),64*1024)
                    self.current_ordinal=packet["provenance"]["ordinal"]
                    worker.need(not self.outputs and not self.delayed,"prior actual DDS output/ACK tail not consumed")
                if topic in incoming: self.publish_hashes[topic] = hashlib.sha256(raw).hexdigest()
                if topic == pure.ACK_TOPIC:
                    value = pure.decode(msg.data.encode(), 128*1024)
                    last=audited_adapters[-1].adapter.last_result if audited_adapters else None
                    actual_before_failure=asdict(last) if value["status"]=="FAILED_RETAINED" and last is not None and last.anchor_ns==value["timestamp_ns"] else None
                    worker.append(records, dict(value, actual_received_input_CDR_sha256=dict(self.owned_input_hashes),
                        actual_received_input_native_transport=dict(self.owned_input_transport),
                        actual_ACK_publish_entered_ns=entered, actual_ACK_publish_completed_ns=finished,
                        actual_output_publication_calls=list(self.actual_output_calls),actual_policy_result_before_transport_failure=actual_before_failure,
                        actual_full_publication_seconds=(finished-self.publisher_begin)/1e9 if self.publisher_begin is not None else None,
                        actual_full_publication_deadline_passed=finished-self.publisher_begin<=100_000_000 if self.publisher_begin is not None else False))
                    self._hh_receive_journaled=True
                    self.actual_output_calls.clear()
                    self.owned_input_hashes.clear(); worker.budget(); worker.same_generation(worker.bounded_json(worker.OUTPUT/"owner_started.json")["identity"])
                    self.owned_input_transport.clear()
                    retain_unfinished_receive_rows(self._hh_receive_timing)
                    self._hh_observed_provenance=None
                    if not self._hh_receive_timing:self._hh_receive_journaled=False
                return None
            # HH_261007 - A private genuine Publisher instance delegate, never a fake DDS success receipt.
            publisher.publish = publish
            return publisher
    ros.Node = OwnedNode
    try:
        if mode == "direct":
            node = real_node("portable_temporal_recorded_direct", enable_rosout=False, start_parameter_services=False)
            until = time.monotonic()+.5
            while time.monotonic() < until:
                graph_guard(node, set(), {"portable_temporal_recorded_direct"}); ros.rclpy.spin_once(node, timeout_sec=.02)
            adapter, rig, contract, execution = audited_load(view, index, pure); admission = pure.BundleAdmission(index)
            for anchor in index.anchors:
                worker.live_owner();graph_guard(node,set(),{"portable_temporal_recorded_direct"})
                worker.budget(); odom, acc, steer, infos, tf, clock = ns["message_set"](anchor, rig, index.context, ros)
                payloads = {source["role"]: dict(timestamp_ns=anchor.timestamp_ns, frame_id=index.context["optical_frames"][source["role"]], format="jpeg",
                    data=pure.read_sealed(Path(index.context["episode_root"])/source["jpeg_path"], source["jpeg_sha256"], 2*1024**2)) for source in anchor.cameras}
                observed = deepcopy(anchor.state); observed["steering_tire_angle_rad"] = steer.steering_tire_angle
                anchor, history, mask = admission.admit(anchor.provenance, payloads, observed, rig.features, tf_verified_rig_id=index.context["rig_id"])
                frame = ns["build_observed_frame"](anchor, payloads, history, mask, rig, contract, index)
                began = time.perf_counter_ns(); result = adapter.accept(frame, now_ns=anchor.timestamp_ns, ego_stamp_ns=anchor.timestamp_ns, route_stamp_ns=anchor.timestamp_ns)
                output_hashes = {}
                if result.status == "TRAJECTORY":
                    trajectory = ros.old.make_shadow_trajectory(result, odom, anchor_ns=anchor.timestamp_ns)
                    path = ros.old.make_shadow_path(trajectory)
                    output_hashes = {"/planning/portable_e2e/shadow_trajectory": hashlib.sha256(serialize_message(trajectory)).hexdigest(),
                        "/planning/portable_e2e/shadow_path": hashlib.sha256(serialize_message(path)).hexdigest()}
                input_hashes = {pure.INPUT_PREFIX+"/"+key: hashlib.sha256(serialize_message(value)).hexdigest() for key, value in
                    (("odometry", odom), ("acceleration", acc), ("steering", steer), ("tf", tf), ("clock", clock))}
                for role in pure.ROLES:
                    image = ros.CompressedImage(); ns["stamp"](image, anchor.timestamp_ns); image.header.frame_id=payloads[role]["frame_id"]; image.format="jpeg"; image.data=payloads[role]["data"]
                    input_hashes[pure.INPUT_PREFIX+"/"+role+"/compressed"] = hashlib.sha256(serialize_message(image)).hexdigest()
                    input_hashes[pure.INPUT_PREFIX+"/"+role+"/camera_info"] = hashlib.sha256(serialize_message(infos[role])).hexdigest()
                worker.append(records, dict(ordinal=anchor.ordinal, native_frame=anchor.native_frame, timestamp_ns=anchor.timestamp_ns,
                    status=result.status, selected=asdict(result.selected) if result.selected else None, policy_diagnostics=result.diagnostics,
                    input_CDR_sha256=input_hashes, converted_output_CDR_sha256=output_hashes, policy_plus_audit_seconds=(time.perf_counter_ns()-began)/1e9,
                    direct_outputs_published=False, execution=execution))
                completed += 1
        elif mode == "publisher":
            ns["recorded_publisher"](view, index, pure, ros)
            completed = worker.count_records(records)
        else:
            node = ns["recorded_node"](view, index, pure, ros)
            membership.bind_actual_node(node)
            worker.new_json(worker.OUTPUT/"subscriber_ready.json", dict(identity=worker.process_identity(__import__("os").getpid()),
                graph=graph_guard(node, all_topics, expected), model_loaded=True, CPU_threads=4, interop_threads=1))
            while node.last+1 < 918 and node.failed is None:
                worker.live_owner();graph_guard(node, all_topics, expected); ros.rclpy.spin_once(node, timeout_sec=.02)
                # HH_261008 - The native/receive callback has fully returned; original poll retains every guard and the existing 10ms timer.
                node.poll()
            completed = node.last+1
            worker.need(node.failed is None, "original subscriber admission failed: "+str(node.failed))
        worker.need(completed == 918, "whole declared918 interval incomplete")
    except BaseException as failure:
        error = repr(failure)
    finally:
        # HH_261007 - Always close the private executor/context and retain closing failures with the prefix.
        try:
            timing_nodes=[dict(name=value.owned_name,**receive_timing_closing_tail(value._hh_receive_timing),
                overflow=value._hh_receive_overflow,timing_already_in_real_ACK_journal=value._hh_receive_journaled) for value in owned_nodes]
            for tail in timing_nodes:
                if tail["compaction_error"] is not None:error=(error+"; " if error else "")+"timing tail validation: "+tail["compaction_error"]
            worker.new_json(worker.OUTPUT/(mode+"_receive_stage_tail.json"),dict(
                provenance="HH_261007 - Retain only actually measured unconsumed native/receive stages; never pad missing input or invent anchor identity.",
                native=executor.native_timing_tail(),nodes=timing_nodes,
                full_ACK_includes_instrumentation=True,whole_interval_profile_claimed=False))
        except BaseException as failure:error=(error+"; " if error else "")+"receive stage tail close: "+repr(failure)
        finally:
            # HH_261008 - A timing-release exception must not bypass all-owned-node detach/destruction/context cleanup.
            close_attempt(runtime_close_records,"native_timing_release",executor.release_native_timing)
            for value in owned_nodes:close_attempt(runtime_close_records,"receive_timing_release:"+value.owned_name,value._hh_receive_timing.clear)
        deferred_tails=[]
        for audited in audited_adapters:
            if isinstance(audited,DeferredAuditedAdapter):
                try:deferred_tails.append(audited.close_audit())
                except BaseException as failure:
                    deferred_tails.append(dict(pending_reference_count=0,failure_tail=audited.failure_tail,error=repr(failure)))
                    error=(error+"; " if error else "")+"deferred audit close: "+repr(failure)
        try:
            worker.new_json(worker.OUTPUT/(mode+"_deferred_numerical_audit_tail.json"),dict(
                provenance="HH_261007 - Preserve pending or failed numerical audits without rerunning inference or inventing hashes.",
                tails=deferred_tails,ACK_completion_excludes_audit_cost=False,whole_interval_audit_claimed=False))
        except BaseException as failure:
            error=(error+"; " if error else "")+"deferred tail receipt close: "+repr(failure)
        try:
            worker.new_json(worker.OUTPUT/(mode+"_native_transport_tail.json"), dict(
                provenance="HH_261007 - Preserve latest actually received native metadata even when no final ACK is emitted.",
                complete_transport_history_claimed=False,
                nodes=[dict(name=value.owned_name, latest_by_topic=dict(value.latest_transport),
                    unacknowledged_input_by_topic=dict(value.owned_input_transport)) for value in owned_nodes]))
        except BaseException as failure:
            error = (error+"; " if error else "")+"native transport tail close: "+repr(failure)
        try:
            completed=worker.count_records(records)
        except BaseException as failure:
            error = (error+"; " if error else "")+"record prefix close: "+repr(failure)
        close_owned_runtime(owned_nodes,node,OwnedNode,owned_node_token,owned_executors,closed_executors,membership,original_rclpy.shutdown,runtime_close_records)
        if any(row["success"] is False for row in runtime_close_records):error=(error+"; " if error else "")+"runtime close failures retained"
        # HH_261008 - Closing receipts are bounded metadata, not additional per-callback logs or a future ACK timestamp.
        if console is not None:
            try:
                receipt=console.closing();worker.budget(additional=len(worker.encode(receipt)),closing=True)
                worker.new_json(worker.OUTPUT/(mode+"_original_console_completion.json"),receipt)
            except BaseException as failure:error=(error+"; " if error else "")+"console witness close: "+repr(failure)
        try:
            closing=worker.close_ledgers()
            worker.budget(additional=len(worker.encode(closing)),closing=True)
            worker.new_json(worker.OUTPUT/(mode+"_codec_closing.json"),closing)
            if closing["codec_failure"]:error=(error+"; " if error else "")+"lossless codec failure retained"
        except BaseException as failure:error=(error+"; " if error else "")+"codec close: "+repr(failure)
    return dict(schema=worker.SCHEMA+".role_result", mode=mode, status="COMPLETE_RECORDED_ROLE_NOT_FEATURE_APPROVAL" if error is None else "FAILED_RETAINED",
        completed=completed, expected=918, UNEXECUTED=918-completed, error=error,runtime_close_records=runtime_close_records, no_retry=True, no_actuation=True, P3_approved=False)


