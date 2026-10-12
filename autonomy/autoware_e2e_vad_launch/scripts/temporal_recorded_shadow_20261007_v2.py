"""HH_261007 - Execute genuine direct/DDS recorded shadow paths after external admission.

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
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
WORKER = ROOT / "tools/e2e/temporal_recorded_ros_shadow_worker_20261007_v2.py"
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
        result[name] = dict(dtype=str(array.dtype), shape=list(array.shape), sha256=hashlib.sha256(array.tobytes()).hexdigest())
    return result


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

    def reset(self, *args, **kwargs):
        return self.adapter.reset(*args, **kwargs)

    def accept(self, *args, **kwargs):
        policy = self.adapter._policy; model = policy.model; tensors = []; outputs = []
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


def authenticated_gid(node, topic, info, expected_name):
    endpoints = node.get_publishers_info_by_topic(topic)
    if len(endpoints) != 1: raise ValueError("exactly one declared DDS publisher required")
    endpoint = endpoints[0]
    if endpoint.node_name != expected_name or endpoint.node_namespace != "/" or bytes(endpoint.endpoint_gid) != bytes(info.publisher_gid):
        raise ValueError("actual DDS MessageInfo publisher GID differs from owned graph peer")
    return bytes(info.publisher_gid).hex()


def run(mode, view):
    worker = bindings(); pure = worker.pure_proxy(view); ns = compile_original(worker)
    index = pure.load_index(view); ros = ns["dependencies"](view, pure)
    from rclpy.serialization import serialize_message
    from autoware_planning_msgs.msg import Trajectory
    from nav_msgs.msg import Path as PathMessage
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
    worker.verify_project_modules(view, sys.modules.values())
    ros.rclpy.init(args=[]); incoming, outgoing = topic_sets(pure); all_topics = incoming | outgoing
    expected = {"portable_temporal_recorded_shadow", "portable_temporal_recorded_publisher"}
    completed = 0; error = None; node = None; records = worker.OUTPUT/(mode+"_anchors.jsonl")
    genuine_load = ns["load_runtime"]
    audited_adapters=[]
    def audited_load(*a):
        if mode == "subscriber":
            probe = ros.Node("portable_temporal_recorded_shadow")
            try:
                until = time.monotonic()+.5
                while time.monotonic() < until:
                    graph_guard(probe, all_topics, {"portable_temporal_recorded_shadow"}); ros.rclpy.spin_once(probe, timeout_sec=.02)
            finally: probe.destroy_node()
        adapter, rig, contract, execution = genuine_load(*a)
        import torch
        worker.need(not torch.cuda.is_initialized() and torch.get_num_threads() == 4 and torch.get_num_interop_threads() == 1,
            "actual frozen CPU4/inter1/no CUDA execution required")
        audited=AuditedAdapter(adapter,worker);audited_adapters.append(audited)
        return audited, rig, contract, execution
    ns["load_runtime"] = audited_load
    real_node = ros.Node
    class OwnedNode(real_node):
        def __init__(self, name):
            worker.need(name in expected, "exact declared node name required")
            super().__init__(name, enable_rosout=False, start_parameter_services=False)
            self.owned_name = name; self.delayed = []; self.outputs = {}; self.graph_errors = []
            self.owned_input_hashes = {}; self.publish_hashes = {}; self.peer_generation = None
            self.actual_output_calls=[];self.publisher_begin=None
            graph_guard(self, all_topics, expected if mode != "direct" else {name})
            if name == "portable_temporal_recorded_publisher":
                qos = ros.QoSProfile(depth=16, reliability=ros.ReliabilityPolicy.RELIABLE)
                for cls, topic in ((Trajectory, "/planning/portable_e2e/shadow_trajectory"), (PathMessage, "/planning/portable_e2e/shadow_path")):
                    self.create_subscription(cls, topic, lambda msg, topic=topic: self.output_received(topic, msg), qos)
                self.create_timer(.005, self.deliver_receipts)

        def peer(self):
            worker.live_owner()
            role = "publisher" if self.owned_name.endswith("shadow") else "subscriber"
            receipt = worker.bounded_json(worker.OUTPUT/(role+"_started.json"))
            worker.same_generation(receipt["identity"])
            worker.need(receipt["plan_sha256"] == view["plan_sha256"] and receipt["authority_sha256"] == view["admitted_authority_sha256"], "peer plan/authority differs")

        def create_subscription(self, cls, topic, callback, qos, **kwargs):
            worker.need(topic in all_topics and self.resolve_topic_name(topic) == topic, "exact topic subscription allowlist required")
            def receive(msg, info):
                self.peer(); graph_guard(self, all_topics, expected)
                gid = authenticated_gid(self, topic, info, "portable_temporal_recorded_publisher" if topic in incoming else "portable_temporal_recorded_shadow")
                raw = serialize_message(msg); now = time.perf_counter_ns()
                if topic in incoming:
                    self.owned_input_hashes[topic] = hashlib.sha256(raw).hexdigest()
                    if topic==pure.PROVENANCE_TOPIC:
                        packet=pure.decode(msg.data.encode(),64*1024);self.publisher_begin=packet["publisher_bundle_started_ns"]
                        self.current_ordinal=packet["provenance"]["ordinal"]
                if topic == pure.ACK_TOPIC:
                    value = pure.decode(msg.data.encode(), 128*1024)
                    self.delayed.append((msg, value, callback, now, gid)); self.deliver_receipts(); return
                callback(msg)
            return super().create_subscription(cls, topic, receive, qos, **kwargs)

        def output_received(self, topic, msg):
            key = ns["stamp_ns"](msg.header.stamp)
            worker.need(hasattr(self,"current_ordinal") and key==index.anchors[self.current_ordinal].timestamp_ns,"DDS output is not the actual current anchor")
            worker.need((key, topic) not in self.outputs, "duplicate actual DDS output stamp/topic")
            self.outputs[key, topic] = dict(sha256=hashlib.sha256(serialize_message(msg)).hexdigest(), received_ns=time.perf_counter_ns())
            worker.append(worker.OUTPUT/"publisher_received_outputs.jsonl",dict(ordinal=self.current_ordinal,timestamp_ns=key,topic=topic,**self.outputs[key,topic]))

        def deliver_receipts(self):
            if not self.delayed: return
            msg, value, callback, received, gid = self.delayed[0]
            stamp = value["timestamp_ns"]; selected_outputs = {}
            if value["status"] == "TRAJECTORY":
                keys = [(stamp, topic) for topic in ("/planning/portable_e2e/shadow_trajectory", "/planning/portable_e2e/shadow_path")]
                if not all(key in self.outputs for key in keys):
                    worker.need(time.perf_counter_ns()-received <= 5_000_000_000, "actual DDS trajectory/path absent after ACK; no invented output")
                    return
                selected_outputs = {topic: self.outputs.pop((stamp, topic)) for _, topic in keys}
            row = dict(value, actual_DDS_receipt_received_ns=received, actual_DDS_ACK_gid=gid,
                actual_DDS_outputs=selected_outputs, published_input_CDR_sha256=dict(self.publish_hashes))
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
                        actual_ACK_publish_entered_ns=entered, actual_ACK_publish_completed_ns=finished,
                        actual_output_publication_calls=list(self.actual_output_calls),actual_policy_result_before_transport_failure=actual_before_failure,
                        actual_full_publication_seconds=(finished-self.publisher_begin)/1e9 if self.publisher_begin is not None else None,
                        actual_full_publication_deadline_passed=finished-self.publisher_begin<=100_000_000 if self.publisher_begin is not None else False))
                    self.actual_output_calls.clear()
                    self.owned_input_hashes.clear(); worker.budget(); worker.same_generation(worker.bounded_json(worker.OUTPUT/"owner_started.json")["identity"])
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
            completed = sum(1 for _ in records.open("rb")) if records.exists() else 0
        else:
            node = ns["recorded_node"](view, index, pure, ros)
            worker.new_json(worker.OUTPUT/"subscriber_ready.json", dict(identity=worker.process_identity(__import__("os").getpid()),
                graph=graph_guard(node, all_topics, expected), model_loaded=True, CPU_threads=4, interop_threads=1))
            while node.last+1 < 918 and node.failed is None:
                worker.live_owner();graph_guard(node, all_topics, expected); ros.rclpy.spin_once(node, timeout_sec=.02)
            completed = node.last+1
            worker.need(node.failed is None, "original subscriber admission failed: "+str(node.failed))
        worker.need(completed == 918, "whole declared918 interval incomplete")
    except BaseException as failure:
        error = repr(failure)
    finally:
        if records.exists(): completed=sum(1 for _ in records.open("rb"))
        if node is not None: node.destroy_node()
        ros.rclpy.shutdown()
    return dict(schema=worker.SCHEMA+".role_result", mode=mode, status="COMPLETE_RECORDED_ROLE_NOT_FEATURE_APPROVAL" if error is None else "FAILED_RETAINED",
        completed=completed, expected=918, UNEXECUTED=918-completed, error=error, no_retry=True, no_actuation=True, P3_approved=False)
