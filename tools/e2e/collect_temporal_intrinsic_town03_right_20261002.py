"""HH_261002 - NEW fixed intrinsic-pair research adapter; actual complete-camera recording must follow its ACK."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
import inspect
from pathlib import Path

from portable_e2e.temporal_intrinsic_curve_export_research_20261002 import OWNER_SCHEMA as NUMERIC_OWNER_SCHEMA, SCHEMA as LOADER_SCHEMA, verify_scope
from . import collect_temporal_route_joint_town03_right_v3_20261002 as parent
from .defer_exact_jpeg_until_ACK_research_20261001 import ExactJPEGAfterACK

frozen, collector, ROOT = parent.frozen, parent.collector, parent.ROOT
previous = parent.previous
SEED, ROUTE_SHA, ROUTE_LENGTH = parent.SEED, parent.ROUTE_SHA, parent.ROUTE_LENGTH
ENTRY, EXIT, EXPECTED_TURN_DEG = parent.ENTRY, parent.EXIT, parent.EXPECTED_TURN_DEG
PREVIEW_SCHEMA, THREAD_SCHEMA = parent.PREVIEW_SCHEMA, parent.THREAD_SCHEMA
RightTurnObserver = parent.RightTurnObserver
SCHEMA = "portable_e2e.actual_intrinsic_pair_town03_known_dev_right.20261002.v1"
OWNER_SCHEMA = "portable_e2e.owned_intrinsic_pair_town03_known_dev_right.20261002.v1"
PARENT_SHA = "da212cdaf265ae7e94555e9cea80bfa8da49677def14657460fa694c9fddfdfc"
_runtime, _capture = parent.runtime_factory, parent.capture_factory
_event_capture = previous.previous.run_event_capture
_joint = previous.previous.previous.previous.previous
_model_binding = _joint.validate_model_binding
_active = False


def verify_frozen():
    frozen._require(hashlib.sha256(Path(parent.__file__).read_bytes()).hexdigest() == PARENT_SHA,
                    "frozen real-NN V3 collector must remain exact")
    parent.verify_frozen()


def validate_capture(args, route):
    parent.validate_capture(args, route)


def validate_model_binding(model, args):
    description = getattr(model, "_research_temporal_export_metadata", {})
    verify_scope(description)
    binding = getattr(model, "_research_recovery_owner_binding", {})
    frozen._require(getattr(model, "_research_recovery_numeric_loader_schema", None) == LOADER_SCHEMA
        and binding.get("schema") == NUMERIC_OWNER_SCHEMA and binding.get("arm") == description.get("arm")
        and binding.get("arm") in ("control", "intrinsic_curve")
        and binding.get("fixed_endpoint_updates") == 6000 and binding.get("purpose") == "owned_carla_research_only"
        and binding.get("simulator_control_authorized") is True and binding.get("simulation_only") is True
        and getattr(model, "_research_recovery_simulator_control_authorized", False) is True
        and binding.get("owner_plan_sha256") == getattr(args, "research_source_sha256", None)
        and binding.get("completed_pair_authority_sha256") == getattr(model, "_research_intrinsic_completed_pair_authority_sha256", None)
        and isinstance(binding.get("completed_pair_authority_sha256"), str) and len(binding["completed_pair_authority_sha256"]) == 64
        and binding.get("vehicle_control_approved") is False and binding.get("model_promoted") is False,
        "exact newly authenticated completed-pair endpoint/private plan required; numeric-only cannot actuate")


def runtime_factory():
    # HH_261002 - Add ONLY the actual deferred JPEG callback to the frozen six-startup/runtime namespace bindings.
    verify_frozen(); original = next(n for n in ast.parse(Path(parent.__file__).read_bytes()).body
        if isinstance(n, ast.FunctionDef) and n.name == "runtime_factory")
    changed = copy.deepcopy(original); counts = dict(add=0, upper=0, inverse=0, match=0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.value.elts[3:3]":
            frozen._require(ast.unparse(node.value) == "[ast.Constant('AcknowledgedControlTransport'), ast.Constant('exact_camera_bundle')]",
                            "exact frozen callback insertion required")
            node.value.elts.append(ast.Call(func=ast.Attribute(value=ast.Name(id="ast", ctx=ast.Load()), attr="Constant", ctx=ast.Load()),
                args=[ast.Constant("_save_jpeg")], keywords=[])); counts["add"] += 1
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.iter.slice.upper" \
                and ast.unparse(node.value) == "ast.Constant(5)":
            node.value.args[0].value = 6; counts["upper"] += 1
        if isinstance(node, ast.Delete) and ast.unparse(node.targets[0]) == "node.value.elts[3:5]":
            node.targets[0].slice.upper.value = 6; counts["inverse"] += 1
        if isinstance(node, ast.Constant) and node.value == "names[:5]": node.value = "names[:6]"; counts["match"] += 1
    frozen._require(counts == dict(add=1, upper=1, inverse=1, match=1), "exact four copied-callback transform anchors required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.value.elts[3:3]": node.value.elts.pop()
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.iter.slice.upper" \
                and ast.unparse(node.value) == "ast.Constant(6)": node.value.args[0].value = 5
        if isinstance(node, ast.Delete) and ast.unparse(node.targets[0]) == "node.value.elts[3:6]": node.targets[0].slice.upper.value = 5
        if isinstance(node, ast.Constant) and node.value == "names[:6]": node.value = "names[:5]"
    frozen._require(ast.dump(restored) == ast.dump(original), "another event/fixture/native/ACK statement changed")
    namespace = dict(vars(parent))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])), __file__, "exec"), namespace)
    return namespace["runtime_factory"]()


def _recorder():
    save = collector._save_jpeg; recorder = getattr(save, "__self__", None)
    frozen._require(type(recorder) is ExactJPEGAfterACK and getattr(save, "__func__", None) is ExactJPEGAfterACK.enqueue
        and getattr(collector, "_exact_jpeg_after_ACK_active", False) is True,
        "actual immutable source JPEG enqueue scope must precede capture")
    transport = [cls for cls in collector.AcknowledgedControlTransport.__mro__ if cls.__name__ == "RecordingAfterACKTransport"]
    frozen._require(len(transport) == 1 and inspect.getclosurevars(transport[0].send).nonlocals.get("recorder") is recorder,
                    "the exact enqueue and ACK transport must own the same bounded recorder")
    return recorder


def checked_event_capture(function, *args, **kwargs):
    # HH_261002 - Verify ALL six copied mutable providers after deferred activation, before the first CARLA callback.
    recorder = _recorder(); namespace = function.__globals__
    names = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
             "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")
    frozen._require(all(namespace.get(name) is getattr(collector, name) for name in names),
                    "compiled capture retains a stale startup/observation/ACK/JPEG provider")
    partial = args[3]; proof = dict(schema=SCHEMA+".actual_recording_binding", copied_provider_names=list(names),
        all_copied_mutable_providers_current=True, enqueue_and_ACK_share_exact_recorder=True,
        source_image_transformed=False, exposure_to_ACK_latency_measured=False)
    try:
        return _event_capture(function, *args, **kwargs)
    finally:
        proof.update(actual_recording_bundle_count=len(recorder.events), pending_source_camera_count=len(recorder.pending),
            actual_after_ACK_receipt_count=sum(row["acknowledged_before_recording"] for row in recorder.events),
            every_recording_event_preserved=True)
        frozen._write_new(partial/"actual_recording_binding.json", proof)


def capture_factory():
    namespace = dict(vars(parent), runtime_factory=runtime_factory, SCHEMA=SCHEMA)
    node = next(n for n in ast.parse(Path(parent.__file__).read_bytes()).body if isinstance(n, ast.FunctionDef) and n.name == "capture_factory")
    exec(compile(ast.Module(body=[node], type_ignores=[]), __file__, "exec"), namespace)
    return namespace["capture_factory"]()


@contextmanager
def activated(model, args):
    global _active
    frozen._require(not _active and not parent._active and parent.capture_factory is _capture
        and parent.runtime_factory is _runtime and _joint.validate_model_binding is _model_binding
        and previous.previous.run_event_capture is _event_capture, "nested or unknown intrinsic research bindings")
    verify_frozen(); validate_model_binding(model, args)
    saved = parent.capture_factory, parent.SCHEMA, parent.OWNER_SCHEMA
    _active = True
    try:
        parent.capture_factory, parent.SCHEMA, parent.OWNER_SCHEMA = capture_factory, SCHEMA, OWNER_SCHEMA
        _joint.validate_model_binding = validate_model_binding
        previous.previous.run_event_capture = checked_event_capture
        with parent.activated(model, args): yield
    finally:
        previous.previous.run_event_capture = _event_capture
        _joint.validate_model_binding = _model_binding
        parent.capture_factory, parent.SCHEMA, parent.OWNER_SCHEMA = saved
        _active = False
