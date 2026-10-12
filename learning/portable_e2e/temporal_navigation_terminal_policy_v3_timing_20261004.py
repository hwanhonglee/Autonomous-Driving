"""HH_261004 - Measure authentic terminal bridge stages without changing its math.

The source-pinned V2 bridge is compiled once with removable scalar timing
statements.  Every original statement, check, return and exception remains in
the original order.  The inherited policy retains its original 100 ms gate.
These observations are research instrumentation, not a capability approval.
"""

from __future__ import annotations

import ast
import copy
import gc
import hashlib
from pathlib import Path
import threading
import time

from . import temporal_navigation_terminal_policy_v2_20261004 as frozen


SCHEMA = "portable_e2e.actual_temporal_navigation_terminal_stage_timing.20261004.v3"
FROZEN_BRIDGE_SHA256 = "e4c0118de9d9e420890d1c0f11df4fe469cf9cdb7e969f1cd96d2448798c40ab"
TERMINAL_STAGES = ("entry", "source_binding", "provenancehash", "canonicalgeometry",
    "missionconstruction", "observationserialization", "witnessevaluation",
    "receiptasdict", "terminaldecoder", "outputattachment")
ORDINARY_STAGES = ("entry", "ordinary_exact_delegate")
MAX_GC_EVENTS = 32


def _source_tree():
    # HH_261004 - Source hashing and compilation happen once, never on each inference.
    path = Path(frozen.__file__).absolute()
    frozen.require(path.resolve(strict=True) == path and path.is_file(), "timing bridge source alias denied")
    payload = path.read_bytes()
    frozen.require(hashlib.sha256(payload).hexdigest() == FROZEN_BRIDGE_SHA256,
                   "timing bridge frozen V2 source differs")
    return ast.parse(payload, filename=str(path)), str(path)


def _call(name, *arguments):
    return ast.Expr(ast.Call(ast.Attribute(ast.Name("self", ast.Load()), name, ast.Load()),
                             [ast.Constant(argument) for argument in arguments], []))


def _assigned_names(node):
    return {item.id for target in getattr(node, "targets", ()) for item in ast.walk(target)
            if isinstance(item, ast.Name)}


def _instrument(original):
    changed = copy.deepcopy(original)
    boundaries = {
        "pose_provenance": "source_binding", "pose_sha": "provenancehash",
        "provided": "canonicalgeometry", "observation": "observationserialization",
        "witness": "witnessevaluation", "distance": "receiptasdict", "result": "terminaldecoder"}
    counts = {name: 0 for name in TERMINAL_STAGES[1:] + ORDINARY_STAGES[1:]}
    body = []
    for node in changed.body:
        stage = next((stage for name, stage in boundaries.items() if name in _assigned_names(node)), None)
        if isinstance(node, ast.If) and ast.dump(node.test) == ast.dump(ast.parse("self.mission_payload is None", mode="eval").body):
            stage = "missionconstruction"
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Attribute)
                and target.attr == "previous_witness" for target in node.targets):
            stage = "outputattachment"
        if stage:
            counts[stage] += 1
            body.append(_call("_hh_timing_stage", stage))
        if isinstance(node, ast.If):
            for index, child in enumerate(node.body):
                if (isinstance(child, ast.Return) and isinstance(child.value, ast.Call) and isinstance(child.value.func, ast.Attribute)
                        and child.value.func.attr == "ordinary_decoder"):
                    node.body.insert(index, _call("_hh_timing_stage", "ordinary_exact_delegate"))
                    counts["ordinary_exact_delegate"] += 1
                    break
        body.append(node)
    frozen.require(all(count == 1 for count in counts.values()), "timing bridge AST boundary count differs")
    changed.body = [_call("_hh_timing_enter"), ast.Try(body=body,
        handlers=[ast.ExceptHandler(type=ast.Name("BaseException", ast.Load()), name="_hh_timing_error",
            body=[ast.Expr(ast.Call(ast.Attribute(ast.Name("self", ast.Load()), "_hh_timing_failed", ast.Load()),
                [ast.Name("_hh_timing_error", ast.Load())], [])), ast.Raise()])], orelse=[],
        finalbody=[_call("_hh_timing_finish")])]
    frozen.require(ast.dump(_restore(changed)) == ast.dump(original),
                   "timing instrumentation changed another original bridge statement")
    return changed


def _restore(changed):
    # HH_261004 - Strip only this revision's exact additions and prove inverse equality.
    restored = copy.deepcopy(changed)
    frozen.require(len(restored.body) == 2 and isinstance(restored.body[1], ast.Try), "timing wrapper differs")
    wrapped = restored.body[1]
    frozen.require(ast.dump(restored.body[0]) == ast.dump(_call("_hh_timing_enter"))
                   and ast.dump(wrapped.finalbody[0]) == ast.dump(_call("_hh_timing_finish"))
                   and len(wrapped.finalbody) == len(wrapped.handlers) == 1 and not wrapped.orelse,
                   "timing enter/finally shape differs")
    expected_handler = ast.ExceptHandler(type=ast.Name("BaseException", ast.Load()), name="_hh_timing_error",
        body=[ast.Expr(ast.Call(ast.Attribute(ast.Name("self", ast.Load()), "_hh_timing_failed", ast.Load()),
            [ast.Name("_hh_timing_error", ast.Load())], [])), ast.Raise()])
    frozen.require(ast.dump(wrapped.handlers[0]) == ast.dump(expected_handler), "timing exception passthrough differs")
    class RemoveMarks(ast.NodeTransformer):
        def visit_Expr(self, node):
            if (isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Attribute)
                    and isinstance(node.value.func.value, ast.Name) and node.value.func.value.id == "self"
                    and node.value.func.attr == "_hh_timing_stage"):
                frozen.require(len(node.value.args) == 1 and isinstance(node.value.args[0], ast.Constant)
                               and node.value.args[0].value in TERMINAL_STAGES[1:] + ORDINARY_STAGES[1:]
                               and not node.value.keywords, "unexpected timing mark")
                return None
            return self.generic_visit(node)
    restored.body = wrapped.body
    return RemoveMarks().visit(restored)


_TREE, _ORIGINAL_PATH = _source_tree()
_CLASS = next(node for node in _TREE.body if isinstance(node, ast.ClassDef)
              and node.name == "ActualNavigationTerminalDecoderBridge")
_ORIGINAL_CALL_AST = next(node for node in _CLASS.body if isinstance(node, ast.FunctionDef) and node.name == "__call__")
_TIMED_CALL_AST = _instrument(_ORIGINAL_CALL_AST)
_NAMESPACE = dict(vars(frozen))
exec(compile(ast.fix_missing_locations(ast.Module(body=[_TIMED_CALL_AST], type_ignores=[])),
             _ORIGINAL_PATH + ":timing_v3_inverse_verified", "exec"), _NAMESPACE)
_COMPILED_CALL = _NAMESPACE["__call__"]


class _ScopedGCObservation:
    """HH_261004 - Observe same-thread GC only; never collect, disable or change thresholds."""

    def __init__(self):
        self.thread_id = threading.get_ident()
        self.events, self.active = [], {}
        self.start_count = self.stop_count = self.overflow_count = 0
        self.callback = self.observe
        gc.callbacks.append(self.callback)

    def observe(self, phase, info):
        if threading.get_ident() != self.thread_id:
            return
        stamp, generation = time.perf_counter_ns(), info.get("generation")
        if phase == "start":
            self.start_count += 1
            if len(self.events) < MAX_GC_EVENTS:
                row = dict(generation=generation, start_monotonic_ns=stamp, finish_monotonic_ns=None,
                           collected=None, uncollectable=None)
                self.events.append(row)
                self.active[generation] = row
            else:
                self.overflow_count += 1
        elif phase == "stop":
            self.stop_count += 1
            row = self.active.pop(generation, None)
            if row is not None:
                row.update(finish_monotonic_ns=stamp, collected=info.get("collected"),
                           uncollectable=info.get("uncollectable"))

    def finish(self):
        for index, callback in enumerate(gc.callbacks):
            if callback is self.callback:
                del gc.callbacks[index]
                break
        return dict(same_thread_only=True, events=self.events, maximum_events=MAX_GC_EVENTS,
            actual_start_count=self.start_count, actual_stop_count=self.stop_count,
            overflow_start_count=self.overflow_count, bounded_event_overflow_disclosed=True,
            unmatched_start_count=len(self.active), collection_behavior_changed=False,
            collection_forced=False, collection_disabled=False, thresholds_changed=False)


class TimedNavigationTerminalDecoderBridge(frozen.ActualNavigationTerminalDecoderBridge):
    """HH_261004 - Preserve original numerical behavior and expose scalar observations separately."""

    __call__ = _COMPILED_CALL

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.last_timing = None
        self._hh_timing_active = False

    def bind_current(self, *args, **kwargs):
        super().bind_current(*args, **kwargs)
        self.last_timing = None

    def _hh_timing_enter(self):
        frozen.require(not self._hh_timing_active, "serial timing bridge call required")
        self._hh_timing_active = True
        self.last_timing = None
        self._hh_started = self._hh_stage_started = time.perf_counter_ns()
        self._hh_stage_name = "entry"
        self._hh_stages = []
        self._hh_error_type = None
        self._hh_gc = _ScopedGCObservation()

    def _hh_timing_stage(self, stage):
        stamp = time.perf_counter_ns()
        self._hh_stages.append(dict(stage=self._hh_stage_name, start_monotonic_ns=self._hh_stage_started,
            finish_monotonic_ns=stamp, elapsed_ns=stamp-self._hh_stage_started))
        self._hh_stage_name, self._hh_stage_started = stage, stamp

    def _hh_timing_failed(self, error):
        self._hh_error_type = type(error).__name__

    def _hh_timing_finish(self):
        gc_observation = self._hh_gc.finish()
        stamp = time.perf_counter_ns()
        # HH_261004 - Missing/malformed bindings must retain the original exception, not fail while formatting its timing.
        frame = self.current[0] if type(self.current) is tuple and self.current else None
        self._hh_stages.append(dict(stage=self._hh_stage_name, start_monotonic_ns=self._hh_stage_started,
            finish_monotonic_ns=stamp, elapsed_ns=stamp-self._hh_stage_started))
        self.last_timing = dict(schema=SCHEMA, frozen_bridge_source_sha256=FROZEN_BRIDGE_SHA256,
            status="FAILED" if self._hh_error_type else "COMPLETED", exception_type=self._hh_error_type,
            frame_timestamp_ns=getattr(frame, "timestamp_ns", None),
            call_started_monotonic_ns=self._hh_started, call_finished_monotonic_ns=stamp,
            total_observed_ns=stamp-self._hh_started, stages=self._hh_stages, gc_observation=gc_observation,
            original_bridge_AST_inverse_verified=True, source_rehashed_during_call=False,
            original_policy_deadline_seconds=.1, original_policy_deadline_unchanged=True,
            stage_work_inside_original_policy_deadline=True, scalar_receipt_finalization_also_inside_policy_deadline=True,
            stage_sum_excludes_final_scalar_receipt_attachment_overhead=True,
            original_inputs_math_checks_order_exceptions_unchanged=True,
            timing_only_research=True, numerical_outputs_changed=False, weights_changed=False,
            learned_STOP_claim=False, vehicle_control_approved=False, main_functions_approved=0, subfunctions_approved=0)
        self._hh_timing_active = False


def navigation_terminal_policy_class(base_policy, route_payload, source_binding):
    """HH_261004 - Inherit authentic build/step and attach research timing outside their timer."""
    original_class = frozen.navigation_terminal_policy_class(base_policy, route_payload, source_binding)
    class TimedActualNavigationTerminalPolicy(original_class):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.navigation_terminal_bridge = TimedNavigationTerminalDecoderBridge(self, route_payload, source_binding)
            # HH_261004 - The prior wrapper is not a decoder; recover its authentic ordinary delegate.
            self.navigation_terminal_bridge.ordinary_decoder = self.decoder.ordinary_decoder
            self.decoder = self.navigation_terminal_bridge

        def step(self, *args, **kwargs):
            self.navigation_terminal_bridge.last_timing = None
            result = super().step(*args, **kwargs)
            result["navigation_terminal_timing_research"] = self.navigation_terminal_bridge.last_timing
            result["navigation_terminal_timing_attachment_outside_original_policy_timer"] = True
            return result
    return TimedActualNavigationTerminalPolicy
