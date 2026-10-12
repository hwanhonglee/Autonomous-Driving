"""HH_261004 - Bind exact Town03 straight JPEG writes after their real ACK without changing driving math."""

from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
import inspect
from pathlib import Path

from . import collect_temporal_route_joint_town03_straight_20261004 as straight
from .defer_exact_jpeg_until_ACK_research_20261001 import ExactJPEGAfterACK


# HH_261004 - Keep the familiar right-v3 aliases because its compiled capture factory is the late-binding seam.
parent = straight.runtime
previous = parent.previous
frozen, collector, ROOT = straight.frozen, straight.collector, straight.ROOT
SEED, ROUTE_SHA, ROUTE_LENGTH = straight.SEED, straight.ROUTE_SHA, straight.ROUTE_LENGTH
ROUTE_POINTS = straight.ROUTE_POINTS
ENTRY, EXIT, EXPECTED_TURN_DEG = straight.ENTRY, straight.EXIT, straight.EXPECTED_TURN_DEG
PREVIEW_SCHEMA, THREAD_SCHEMA = straight.PREVIEW_SCHEMA, straight.THREAD_SCHEMA
RightTurnObserver = straight.StraightIntervalObserver
StraightIntervalObserver = straight.StraightIntervalObserver
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_fixed45.20261004.v2"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.v2"
PARENT_SHA = "fcccba488224a94552edde59e19fe8f577a735d4d27aa6ec9579b8d4a09cbaeb"
_runtime_factory = parent.runtime_factory
_capture_factory = parent.capture_factory
_straight_capture = straight.run_straight_capture
_active = False


def verify_frozen():
    """HH_261004 - Preserve the failed v1 source and every inherited controller/model/runtime implementation."""
    frozen._require(hashlib.sha256(Path(straight.__file__).read_bytes()).hexdigest() == PARENT_SHA,
                    "archived Town03 straight v1 collector changed")
    straight.verify_frozen()


def validate_capture(args, route):
    straight.validate_capture(args, route)


def runtime_factory():
    """HH_261004 - Add only `_save_jpeg` to right-v3's inverse-proven actual-provider late binding."""
    verify_frozen()
    original = next(node for node in ast.parse(Path(parent.__file__).read_bytes()).body
                    if isinstance(node, ast.FunctionDef) and node.name == "runtime_factory")
    changed = copy.deepcopy(original)
    counts = dict(add=0, upper=0, inverse=0, match=0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.value.elts[3:3]":
            frozen._require(ast.unparse(node.value) ==
                "[ast.Constant('AcknowledgedControlTransport'), ast.Constant('exact_camera_bundle')]",
                "exact frozen runtime-provider insertion required")
            node.value.elts.append(ast.Call(
                func=ast.Attribute(value=ast.Name(id="ast", ctx=ast.Load()), attr="Constant", ctx=ast.Load()),
                args=[ast.Constant("_save_jpeg")], keywords=[]))
            counts["add"] += 1
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.iter.slice.upper" \
                and ast.unparse(node.value) == "ast.Constant(5)":
            node.value.args[0].value = 6
            counts["upper"] += 1
        if isinstance(node, ast.Delete) and ast.unparse(node.targets[0]) == "node.value.elts[3:5]":
            node.targets[0].slice.upper.value = 6
            counts["inverse"] += 1
        if isinstance(node, ast.Constant) and node.value == "names[:5]":
            node.value = "names[:6]"
            counts["match"] += 1
    frozen._require(counts == dict(add=1, upper=1, inverse=1, match=1),
                    "exact four `_save_jpeg` late-binding AST anchors required")

    # HH_261004 - Prove that deleting this one provider produces the byte-equivalent parent AST.
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.value.elts[3:3]":
            node.value.elts.pop()
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "node.iter.slice.upper" \
                and ast.unparse(node.value) == "ast.Constant(6)":
            node.value.args[0].value = 5
        if isinstance(node, ast.Delete) and ast.unparse(node.targets[0]) == "node.value.elts[3:6]":
            node.targets[0].slice.upper.value = 5
        if isinstance(node, ast.Constant) and node.value == "names[:6]":
            node.value = "names[:5]"
    frozen._require(ast.dump(restored) == ast.dump(original),
                    "another camera, ACK, fixture, policy, or control statement changed")
    namespace = dict(vars(parent))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__)) + ":six_actual_capture_callbacks", "exec"), namespace)
    return namespace["runtime_factory"]()


def _recorder():
    """HH_261004 - Require one recorder to own both the copied JPEG callback and actual ACK transport."""
    save = collector._save_jpeg
    recorder = getattr(save, "__self__", None)
    frozen._require(type(recorder) is ExactJPEGAfterACK
        and getattr(save, "__func__", None) is ExactJPEGAfterACK.enqueue
        and getattr(collector, "_exact_jpeg_after_ACK_active", False) is True,
        "actual exact-JPEG enqueue scope must precede the compiled capture")
    transports = [kind for kind in collector.AcknowledgedControlTransport.__mro__
                  if kind.__name__ == "RecordingAfterACKTransport"]
    frozen._require(len(transports) == 1
        and inspect.getclosurevars(transports[0].send).nonlocals.get("recorder") is recorder,
        "the exact enqueue callback and ACK transport must share one bounded recorder")
    return recorder


def checked_straight_capture(function, *args, **kwargs):
    """HH_261004 - Verify all copied providers after activation and retain a create-only recording proof."""
    recorder = _recorder()
    namespace = function.__globals__
    names = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
             "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")
    frozen._require(all(namespace.get(name) is getattr(collector, name) for name in names),
                    "compiled straight capture retained a stale startup, camera, ACK, or JPEG provider")
    partial = args[3]
    proof = dict(schema=SCHEMA + ".actual_recording_binding", copied_provider_names=list(names),
        all_copied_mutable_providers_current=True, enqueue_and_ACK_share_exact_recorder=True,
        source_image_transformed=False, source_timestamp_changed=False,
        recording_wall_time_added_to_bundle_to_ACK_metric=False,
        exposure_to_ACK_latency_measured=False, controller_or_policy_math_changed=False,
        full_fixed45_success_not_claimed_by_binding_alone=True)
    try:
        return _straight_capture(function, *args, **kwargs)
    finally:
        proof.update(actual_recording_bundle_count=len(recorder.events),
            actual_after_ACK_receipt_count=sum(row["acknowledged_before_recording"] is True
                                               for row in recorder.events),
            pending_source_camera_count=len(recorder.pending), every_recording_event_preserved=True,
            complete_450_bundle_requirement_enforced_by_owner_diagnostic=True,
            complete_904_ACK_requirement_enforced_by_owner_diagnostic=True)
        frozen._write_new(partial / "actual_recording_binding.json", proof)


def capture_factory():
    """HH_261004 - Rebuild right-v3's wrapper with the six-provider runtime factory and no control changes."""
    namespace = dict(vars(parent), runtime_factory=runtime_factory, SCHEMA=SCHEMA)
    node = next(item for item in ast.parse(Path(parent.__file__).read_bytes()).body
                if isinstance(item, ast.FunctionDef) and item.name == "capture_factory")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 str(Path(__file__)) + ":actual_post_ACK_capture_factory", "exec"), namespace)
    return namespace["capture_factory"]()


@contextmanager
def activated(model, args):
    """HH_261004 - Bind the compiled writer to the inherited active recorder without nesting its owner scope."""
    global _active
    frozen._require(not _active and not straight._active and not parent._active
        and parent.capture_factory is _capture_factory and parent.runtime_factory is _runtime_factory
        and straight.run_straight_capture is _straight_capture,
        "fresh Town03 straight v2 activation cannot nest or replace unknown runtime bindings")
    verify_frozen()
    validate_capture(args, collector.load_route(args.route_file))
    saved = parent.capture_factory, parent.SCHEMA, parent.OWNER_SCHEMA, straight.run_straight_capture
    _active = True
    try:
        parent.capture_factory, parent.SCHEMA, parent.OWNER_SCHEMA = capture_factory, SCHEMA, OWNER_SCHEMA
        straight.run_straight_capture = checked_straight_capture
        with straight.activated(model, args):
            # HH_261004 - The recovery owner already created the exact output-scoped recorder; v1 missed only this copied global.
            _recorder()
            yield
    finally:
        parent.capture_factory, parent.SCHEMA, parent.OWNER_SCHEMA, straight.run_straight_capture = saved
        _active = False
