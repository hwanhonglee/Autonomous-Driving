"""HH_261004 - Observe actual camera callback to decoded-plan/ACK timing without replacing driving math."""

from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
import inspect
import json
import math
from pathlib import Path
import threading
import time
import sys

from portable_e2e import temporal_simulation_research_v2_20261001 as policy_runtime
from . import collect_temporal_route_joint_town03_straight_v3_20261004 as previous


frozen, collector, ROOT = previous.frozen, previous.collector, previous.ROOT
straight, fixture = previous.straight, previous.fixture
SEED, ROUTE_SHA, ROUTE_LENGTH = previous.SEED, previous.ROUTE_SHA, previous.ROUTE_LENGTH
ROUTE_POINTS = previous.ROUTE_POINTS
ENTRY, EXIT, EXPECTED_TURN_DEG = previous.ENTRY, previous.EXIT, previous.EXPECTED_TURN_DEG
PREVIEW_SCHEMA, THREAD_SCHEMA = previous.PREVIEW_SCHEMA, previous.THREAD_SCHEMA
RightTurnObserver, StraightIntervalObserver = previous.RightTurnObserver, previous.StraightIntervalObserver
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_fixed45.20261004.callback45_v2"
OWNER_SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_straight_fixed45.20261004.callback45_v2"
PARENT_SHA = "ff90ff73dfcff08d67fd8a9291f2e44c716697610b5129fa69b3952c127c68f2"
POLICY_SHA = "772275dd9e452c70bd9d5364bbacb32d8bf08438ce8bf3526f7ebcf3d64884dc"
_original_factory = straight.cloned_collect_episode
_original_step = policy_runtime.TemporalSimulationResearchPolicyV2.step
_active, _ledger = False, None
STAGES = ("input_ready", "raw_nn_finished", "decoded_plan_ready")


def verify_frozen():
    """HH_261004 - Authenticate the complete frozen predecessor and unchanged actual base-policy implementation."""
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PARENT_SHA,
                    "archived Town03 straight v3 collector changed")
    frozen._require(hashlib.sha256(Path(policy_runtime.__file__).read_bytes()).hexdigest() == POLICY_SHA,
                    "archived actual temporal policy changed")
    previous.verify_frozen()


def validate_capture(args, route):
    previous.validate_capture(args, route)


class CallbackLedger:
    """HH_261004 - Retain bounded scalar receipts, never camera payloads, queues, tensors or synthesized clocks."""

    def __init__(self, model, output, clock=time.perf_counter_ns):
        self.model, self.output, self.clock = model, Path(output), clock
        self.condition = threading.Condition()
        self.callbacks, self.bindings, self.bundles, self.plans = [], [], [], []
        self.errors, self.inflight, self.closed = [], 0, False
        self.policy_ids = set()
        self.capture_started = False

    def callback(self, role, actor_id, original_put):
        frozen._require(role in collector.MODEL_CAMERA_ORDER and type(actor_id) is int and actor_id > 0
            and callable(original_put), "one actual canonical camera actor and original queue callback required")
        with self.condition:
            frozen._require(not self.closed and role not in {r["role"] for r in self.bindings},
                            "camera timing registration cannot repeat or occur after close")
            self.bindings.append(dict(role=role, actor_id=actor_id, original_put_id=id(original_put)))

        def observed(image):
            entered = self.clock()  # HH_261004 - The actual registered callback's first executable instruction.
            row = dict(role=role, actor_id=actor_id, image_object_id=id(image),
                callback_enter_monotonic_ns=entered, original_queue_put_calls=0,
                original_queue_put_returned=False, original_queue_put_error=None, receipt_error=None)
            with self.condition:
                self.inflight += 1
                row["receipt_sequence"] = len(self.callbacks) + 1
                self.callbacks.append(row)
                if self.closed:
                    self.errors.append("actual callback entered after capture-close census")
            try:
                with self.condition:
                    row["original_queue_put_calls"] = 1
                    row["queue_put_started_monotonic_ns"] = self.clock()
                try:
                    result = original_put(image)
                except BaseException as error:
                    with self.condition:
                        row["original_queue_put_error"] = type(error).__name__ + ": " + str(error)
                    raise
                with self.condition:
                    row["original_queue_put_returned"] = True
                return result
            finally:
                with self.condition:
                    row["queue_put_finished_monotonic_ns"] = self.clock()
                try:
                    frame, stamp = image.frame, image.timestamp
                    frozen._require(type(frame) is int and frame >= 0 and type(stamp) in (int, float)
                        and math.isfinite(stamp) and stamp >= 0 and image.width == 640 and image.height == 360,
                        "actual callback source frame, simulation stamp and BGRA geometry required")
                    payload = image.raw_data
                    frozen._require(isinstance(payload, (bytes, bytearray, memoryview))
                        and len(payload) == 360 * 640 * 4, "actual callback source BGRA byte count differs")
                    digest = hashlib.sha256(b"bgra8")
                    digest.update(payload)
                    source_record = dict(native_frame=frame, source_image_timestamp_simulation_ns=int(round(stamp * 1.e9)),
                        source_BGRA_encoding_prefixed_sha256=digest.hexdigest(), source_byte_count=len(payload),
                        source_SHA_finished_monotonic_ns=self.clock(), source_pixels_changed=False,
                        source_timestamp_changed=False, source_image_strong_reference_retained=False)
                    with self.condition:
                        row.update(source_record)
                except BaseException as error:
                    # HH_261004 - A diagnostic failure cannot replace the original queue return/exception or control.
                    with self.condition:
                        row["receipt_error"] = type(error).__name__ + ": " + str(error)
                finally:
                    with self.condition:
                        self.inflight -= 1
                        self.condition.notify_all()
        return observed

    def observe_bundle(self, bundle, frame):
        """HH_261004 - Associate the exact returned objects without another queue read or camera-byte copy."""
        row = dict(native_frame=frame, bundle_binding_finished_monotonic_ns=self.clock(), cameras=[])
        for role in collector.MODEL_CAMERA_ORDER:
            image = bundle[role]
            row["cameras"].append(dict(role=role, image_object_id=id(image), native_frame=image.frame,
                source_image_timestamp_simulation_ns=int(round(float(image.timestamp) * 1.e9))))
        self.bundles.append(row)

    def mark(self, policy, frame, stage):
        """HH_261004 - Mark actual executed policy statements; do not evaluate or replace any prediction."""
        stamp = self.clock()
        frozen._require(stage in STAGES and policy.model is self.model,
                        "timing marker reached a foreign policy/model")
        self.policy_ids.add(id(policy))
        self.plans.append(dict(policy_object_id=id(policy), frame_timestamp_simulation_ns=frame.timestamp_ns,
            stage=stage, stage_finished_monotonic_ns=stamp, original_policy_math_changed=False))

    def finish(self):
        """HH_261004 - Join callback bookkeeping only AFTER the original collector destroyed its actors."""
        if not self.capture_started:
            # HH_261004 - A CPU activation/preflight without actual capture must not create an episode directory.
            return None
        with self.condition:
            drained = self.condition.wait_for(lambda: self.inflight == 0, timeout=5.0)
            self.closed = True
            if not drained:
                self.errors.append("callback receipts did not finish after original capture cleanup")
            value = dict(schema=SCHEMA + ".callback_receipts", clock="time.perf_counter_ns_monotonic",
                hardware_exposure_clock_observed=False, hardware_exposure_to_ACK_proven=False,
                ten_hz_wall_rate_proven=False, source_simulation_clock="CARLA_simulation_seconds",
                callbacks=copy.deepcopy(self.callbacks), camera_bindings=copy.deepcopy(self.bindings),
                bundles=copy.deepcopy(self.bundles), policy_stage_marks=copy.deepcopy(self.plans),
                errors=list(self.errors), inflight_callbacks=self.inflight,
                callback_count=len(self.callbacks), scalar_receipt_limit=12000,
                scalar_receipt_limit_exceeded=len(self.callbacks) > 12000,
                all_callbacks_including_bootstrap_or_unconsumed_preserved=drained,
                source_camera_payloads_retained_by_ledger=False,
                extra_queue_or_image_buffer_installed=False, callback_SHA_after_original_queue_put=True,
                callback_SHA_inside_observed_runtime_overhead=True,
                callback_file_IO_during_capture=False, original_put_object_and_arguments_preserved=True,
                policy_stage_statement_count=3, private_policy_object_count=len(self.policy_ids),
                receipt_close_after_original_capture_cleanup=True, cleanup_only_wait_maximum_seconds=5.0,
                physics_or_control_wait_added=False, source_math_inverse_AST_verified=True)
        # HH_261004 - Do not recreate a finalized partial tree; a successful original collector may rename it.
        target = self.output if self.output.is_dir() else self.output.with_name(self.output.name.removesuffix(".partial"))
        frozen._require(target.is_dir() and not target.is_symlink(),
                        "actual existing output directory required for callback receipts")
        frozen._write_new(target / "callback_timing_receipts.json", value)
        return value


def timed_camera_callback(role, actor_id, original_put):
    frozen._require(_active and _ledger is not None, "camera timing requires the active private callback owner")
    return _ledger.callback(role, actor_id, original_put)


def callback_capture_started(partial):
    """HH_261004 - Keep the inherited factory closure intact and admit exactly one actual output-scoped capture."""
    frozen._require(_active and _ledger is not None and not _ledger.capture_started
        and Path(partial) == _ledger.output, "one actual output-scoped callback capture required")
    _ledger.capture_started = True


def callback_bundle_selected(bundle, frame):
    """HH_261004 - Observe the returned originals without replacing the recorder-owned bundle provider."""
    frozen._require(_active and _ledger is not None, "bundle receipt requires the active private callback owner")
    _ledger.observe_bundle(bundle, frame)


def policy_marker(policy, frame, stage):
    frozen._require(_active and _ledger is not None, "policy timing requires the active private callback owner")
    _ledger.mark(policy, frame, stage)


def instrument_callback_AST(original):
    """HH_261004 - Observe original callbacks and selected bundles; inverse equality protects all provider calls."""
    changed, count = copy.deepcopy(original), 0
    for node in ast.walk(changed):
        if isinstance(node, ast.Call) and ast.unparse(node) == "sensor.listen(queues[spec.name].put)":
            node.args[0] = ast.parse("timed_camera_callback(spec.name, sensor.id, queues[spec.name].put)", mode="eval").body
            count += 1
    frozen._require(count == 1, "exact original six-camera listen loop anchor required")
    class BundleMark(ast.NodeTransformer):
        count = 0

        def visit_Assign(self, node):
            node = self.generic_visit(node)
            if len(node.targets) == 1 and ast.unparse(node.targets[0]) == "bundle" \
                    and isinstance(node.value, ast.Call) and ast.unparse(node.value.func) == "exact_camera_bundle":
                self.count += 1
                return [node, ast.parse("callback_bundle_selected(bundle, frame)").body[0]]
            return node
    mark = BundleMark()
    changed = mark.visit(changed)
    frozen._require(mark.count == 2, "both original timed/untimed exact-bundle assignments required")
    index = int(bool(changed.body and isinstance(changed.body[0], ast.Expr)
                     and isinstance(changed.body[0].value, ast.Constant)
                     and isinstance(changed.body[0].value.value, str)))
    changed.body.insert(index, ast.parse("callback_capture_started(partial)").body[0])
    restored = copy.deepcopy(changed)
    restored.body = [n for n in restored.body if ast.unparse(n) != "callback_capture_started(partial)"]
    class UndoBundleMark(ast.NodeTransformer):
        def visit_Expr(self, node):
            if ast.unparse(node) == "callback_bundle_selected(bundle, frame)":
                return None
            return self.generic_visit(node)
    restored = UndoBundleMark().visit(restored)
    for node in ast.walk(restored):
        if isinstance(node, ast.Call) and ast.unparse(node) == \
                "sensor.listen(timed_camera_callback(spec.name, sensor.id, queues[spec.name].put))":
            node.args[0] = ast.parse("queues[spec.name].put", mode="eval").body
    frozen._require(ast.dump(restored) == ast.dump(original), "another capture, queue, sensor or tick statement changed")
    return changed


def callback_capture_factory():
    """HH_261004 - Extend the frozen inverse-proven straight factory only after its own math/fixture proof."""
    original = next(node for node in ast.parse(Path(straight.__file__).read_bytes()).body
                    if isinstance(node, ast.FunctionDef) and node.name == "cloned_collect_episode")
    changed, count = copy.deepcopy(original), 0
    for index, node in enumerate(changed.body):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "namespace" for t in node.targets):
            changed.body.insert(index, ast.parse("changed = instrument_callback_AST(changed)").body[0])
            node.value.keywords.append(ast.keyword(arg="timed_camera_callback", value=ast.Name(id="timed_camera_callback", ctx=ast.Load())))
            node.value.keywords.append(ast.keyword(arg="callback_capture_started", value=ast.Name(id="callback_capture_started", ctx=ast.Load())))
            node.value.keywords.append(ast.keyword(arg="callback_bundle_selected", value=ast.Name(id="callback_bundle_selected", ctx=ast.Load())))
            count += 1
            break
    frozen._require(count == 1, "exact original straight namespace anchor required")
    restored = copy.deepcopy(changed)
    restored.body = [node for node in restored.body if ast.unparse(node) != "changed = instrument_callback_AST(changed)"]
    for node in restored.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "namespace" for t in node.targets):
            node.value.keywords = [k for k in node.value.keywords if k.arg not in
                ("timed_camera_callback", "callback_capture_started", "callback_bundle_selected")]
    frozen._require(ast.dump(restored) == ast.dump(original), "another straight factory statement changed")
    namespace = dict(vars(straight), instrument_callback_AST=instrument_callback_AST,
                     timed_camera_callback=timed_camera_callback, callback_capture_started=callback_capture_started,
                     callback_bundle_selected=callback_bundle_selected)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__)) + ":actual_camera_first_callback", "exec"), namespace)
    capture = namespace["cloned_collect_episode"]()
    function = inspect.getclosurevars(capture).nonlocals["function"]
    frozen._require(function.__globals__.get("timed_camera_callback") is timed_camera_callback,
                    "compiled actual camera registration retained a foreign callback helper")
    frozen._require(function.__globals__.get("callback_capture_started") is callback_capture_started,
                    "compiled actual capture-start receipt retained a foreign helper")
    frozen._require(function.__globals__.get("callback_bundle_selected") is callback_bundle_selected,
                    "compiled actual selected-bundle receipt retained a foreign helper")
    return capture


def timed_policy_step():
    """HH_261004 - Keep the actual base step AST identical after removing exactly three observation-only marks."""
    cls = next(n for n in ast.parse(Path(policy_runtime.__file__).read_bytes()).body
               if isinstance(n, ast.ClassDef) and n.name == "TemporalSimulationResearchPolicyV2")
    original = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "step")
    changed, counts = copy.deepcopy(original), {s: 0 for s in STAGES}
    class Insert(ast.NodeTransformer):
        def visit_Assign(self, node):
            node = self.generic_visit(node)
            target = ast.unparse(node.targets[0])
            stage = None
            if target == "(tensors, window)" and isinstance(node.value, ast.Call) \
                    and ast.unparse(node.value.func) == "self.build_inputs":
                stage = "input_ready"
            elif target == "raw" and isinstance(node.value, ast.Call) \
                    and ast.unparse(node.value.func) == "self.model.forward_cached":
                stage = "raw_nn_finished"
            elif target == "decision['decoded_output']":
                stage = "decoded_plan_ready"
            if stage is None:
                return node
            counts[stage] += 1
            return [node, ast.parse("policy_marker(self, frame, " + repr(stage) + ")").body[0]]
    changed = Insert().visit(changed)
    frozen._require(counts == {s: 1 for s in STAGES}, "exact actual input/NN/decoded-plan AST seams required")
    class Undo(ast.NodeTransformer):
        def visit_Expr(self, node):
            if isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name) \
                    and node.value.func.id == "policy_marker":
                return None
            return self.generic_visit(node)
    frozen._require(ast.dump(Undo().visit(copy.deepcopy(changed))) == ast.dump(original),
                    "another policy, prediction, decoder, gate, tracker or deadline statement changed")
    namespace = dict(vars(policy_runtime), policy_marker=policy_marker)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__)) + ":three_actual_policy_marks_only", "exec"), namespace)
    return namespace["step"]


@contextmanager
def activated(model, args):
    """HH_261004 - Scope fresh owner aliases and first-callback marks, restoring all delegates on every exit."""
    global _active, _ledger
    frozen._require(not _active and _ledger is None and not previous._active and not straight._active
        and straight.cloned_collect_episode is _original_factory
        and policy_runtime.TemporalSimulationResearchPolicyV2.step is _original_step,
        "callback timing cannot nest or replace an unknown factory/policy")
    verify_frozen()
    validate_capture(args, collector.load_route(args.route_file))
    clock_info = time.get_clock_info("perf_counter")
    frozen._require(clock_info.monotonic and not clock_info.adjustable, "actual monotonic nonadjustable clock required")
    saved = previous.SCHEMA, previous.OWNER_SCHEMA, straight.cloned_collect_episode, policy_runtime.TemporalSimulationResearchPolicyV2.step
    _active = True
    _ledger = CallbackLedger(model, args.output.with_name(args.output.name + ".partial"))
    try:
        previous.SCHEMA, previous.OWNER_SCHEMA = SCHEMA, OWNER_SCHEMA
        straight.cloned_collect_episode = callback_capture_factory
        with previous.activated(model, args):
            frozen._require(previous._actual_fixture_owner_schema() == OWNER_SCHEMA,
                            "fresh callback owner did not reach the actual fixture guard")
            # HH_261004 - Compile only after inherited preview/PI activation; v1 snapshotted the uncalibrated tracker.
            policy_runtime.TemporalSimulationResearchPolicyV2.step = timed_policy_step()
            frozen._require(policy_runtime.TemporalSimulationResearchPolicyV2.step.__globals__["trajectory_control"]
                is policy_runtime.trajectory_control, "timed step retained a stale preview tracker")
            bundle, transport, writer = collector.exact_camera_bundle, collector.AcknowledgedControlTransport, collector._save_jpeg
            previous.previous._recorder()
            try:
                yield
            finally:
                frozen._require(collector.exact_camera_bundle is bundle
                    and collector.AcknowledgedControlTransport is transport and collector._save_jpeg is writer,
                    "callback instrumentation replaced an original same-recorder provider")
                policy_runtime.TemporalSimulationResearchPolicyV2.step = saved[3]
    finally:
        # HH_261004 - No clock wait is inserted into the original capture; drain receipts after its normal cleanup.
        ledger = _ledger
        original_error = sys.exc_info()[1]
        try:
            if ledger is not None:
                try:
                    ledger.finish()
                except BaseException as error:
                    if original_error is None:
                        raise
                    # HH_261004 - Retain the original driving/setup exception; absent receipts force the owner's FAIL.
                    print("callback receipt finalization also failed: " + type(error).__name__ + ": " + str(error),
                          file=sys.stderr)
        finally:
            previous.SCHEMA, previous.OWNER_SCHEMA, straight.cloned_collect_episode, policy_runtime.TemporalSimulationResearchPolicyV2.step = saved
            _ledger, _active = None, False


def _stats(values):
    ordered = sorted(values)
    if not ordered:
        return dict(count=0, minimum=None, p95=None, maximum=None)
    location = .95 * (len(ordered) - 1)
    lo = int(location)
    hi = min(lo + 1, len(ordered) - 1)
    return dict(count=len(ordered), minimum=ordered[0],
        p95=ordered[lo] + (ordered[hi] - ordered[lo]) * (location - lo), maximum=ordered[-1])


def audit_receipts(value, cameras, commands, timings, counts):
    """HH_261004 - Join ALL declared latest/past source cameras to actual fresh plans and accepted ACKs."""
    require = frozen._require
    require(value.get("schema") == SCHEMA + ".callback_receipts"
        and value.get("clock") == "time.perf_counter_ns_monotonic"
        and value.get("hardware_exposure_clock_observed") is False
        and value.get("hardware_exposure_to_ACK_proven") is False
        and value.get("ten_hz_wall_rate_proven") is False
        and value.get("inflight_callbacks") == 0 and value.get("errors") == []
        and value.get("scalar_receipt_limit_exceeded") is False
        and value.get("all_callbacks_including_bootstrap_or_unconsumed_preserved") is True
        and value.get("source_math_inverse_AST_verified") is True,
        "complete genuine callback receipt closure required")
    roles = tuple(collector.MODEL_CAMERA_ORDER)
    callbacks, bindings, bundles, marks = (value[k] for k in ("callbacks", "camera_bindings", "bundles", "policy_stage_marks"))
    require(len(bindings) == 6 and {r["role"] for r in bindings} == set(roles)
        and len({r["actor_id"] for r in bindings}) == 6
        and len(callbacks) == value["callback_count"] <= 12000,
        "six distinct actual owned camera callback registrations required")
    actors = {r["role"]: r["actor_id"] for r in bindings}
    by_key, by_stamp = {}, {}
    for index, row in enumerate(callbacks, 1):
        key = row["role"], row["native_frame"]
        stamp_key = row["role"], row["source_image_timestamp_simulation_ns"]
        require(row["receipt_sequence"] == index and key not in by_key and stamp_key not in by_stamp
            and row["role"] in actors and row["actor_id"] == actors[row["role"]]
            and row.get("original_queue_put_calls") == 1 and row.get("original_queue_put_returned") is True
            and row.get("original_queue_put_error") is None and row.get("receipt_error") is None
            and row.get("source_byte_count") == 360 * 640 * 4
            and len(row.get("source_BGRA_encoding_prefixed_sha256", "")) == 64
            and 0 < row["callback_enter_monotonic_ns"] <= row["queue_put_started_monotonic_ns"]
                <= row["queue_put_finished_monotonic_ns"] <= row["source_SHA_finished_monotonic_ns"],
            "duplicate, incomplete or malformed actual callback receipt")
        by_key[key], by_stamp[stamp_key] = row, row
    selected, arrival = {}, []
    require(len(cameras) == len(bundles) == 450, "all 450 recorded camera bundles required")
    for camera, bound in zip(cameras, bundles):
        frame, stamp = camera["frame"], int(round(camera["timestamp"] * 1.e9))
        require(frame == bound["native_frame"] and frame not in selected
            and tuple(r["role"] for r in bound["cameras"]) == roles, "exact original bundle order and frame required")
        receipts = []
        for source in bound["cameras"]:
            row = by_key[(source["role"], frame)]
            require(source["native_frame"] == frame
                and source["source_image_timestamp_simulation_ns"] == row["source_image_timestamp_simulation_ns"] == stamp
                and source["image_object_id"] == row["image_object_id"]
                and row["callback_enter_monotonic_ns"] <= bound["bundle_binding_finished_monotonic_ns"],
                "actual selected image object/frame/stamp is not its recorded callback")
            receipts.append(row)
        selected[frame] = receipts
        arrival.append(max(r["callback_enter_monotonic_ns"] for r in receipts)
                       - min(r["callback_enter_monotonic_ns"] for r in receipts))
    stage_groups = {}
    for row in marks:
        require(row["stage"] in STAGES and row.get("original_policy_math_changed") is False,
                "genuine policy mark required")
        group = stage_groups.setdefault(row["frame_timestamp_simulation_ns"], {})
        require(row["stage"] not in group, "duplicate plan-stage mark")
        group[row["stage"]] = row
    fresh = [r for r in commands if r.get("learned_control_proposed") is True
             and r.get("held_between_camera_frames") is False]
    require(len(fresh) == counts["inference_count"] > 0 and len(stage_groups) == len(fresh),
            "all actual fresh NN inferences, including any failed stages, must be retained")
    by_ack = {r["ACK_sequence"]: r for r in timings}
    require(len(by_ack) == len(timings) == 904, "complete original ACK wall receipts required")
    results, per_role = [], {role: [] for role in roles}
    SHA_checks = 0
    for decision in fresh:
        frame, sequence = decision["inference_frame"], decision["ack_receipt_sequence"]
        require(frame == decision["command_frame"] and frame in selected,
                "fresh inference frame must be its actual latest source")
        receipts, ack = selected[frame], by_ack[sequence]
        stamp = receipts[0]["source_image_timestamp_simulation_ns"]
        group = stage_groups[stamp]
        require(set(group) == set(STAGES) and len({r["policy_object_id"] for r in group.values()}) == 1,
                "all three actual statements must belong to one private policy")
        inputs, nn, plan = (group[s]["stage_finished_monotonic_ns"] for s in STAGES)
        require(ack["inference_frame"] == ack["command_frame"] == frame
            and ack["learned_control_proposed"] is True and ack["held_between_camera_frames"] is False
            and ack["ACK_server_accepted"] is True and ack["control_source"] == "learned_trajectory_tracker"
            and ack["proposal_started_monotonic_ns"] <= inputs <= nn <= plan
                <= ack["proposal_finished_monotonic_ns"] <= ack["ACK_started_monotonic_ns"] <= ack["ACK_finished_monotonic_ns"]
            and max(r["callback_enter_monotonic_ns"] for r in receipts)
                <= ack["observed_bundle"]["exact_bundle_delivery_monotonic_ns"] <= inputs,
            "actual callback/input/NN/decoded-plan/proposal/accepted-ACK chronology differs")
        history = decision["owned_camera_history"]["source_camera_frames"]
        require(history and history[-1]["timestamp_ns"] == stamp,
                "actual temporal latest SHA identity differs from the selected callback bundle")
        for source in history:
            if source is None:
                continue
            require(len(source["camera_sha256"]) == 6 and source["timestamp_ns"] <= stamp,
                    "all causal history source SHAs required")
            for role, sha in zip(roles, source["camera_sha256"]):
                require(by_stamp[(role, source["timestamp_ns"])]["source_BGRA_encoding_prefixed_sha256"] == sha,
                        "actual original callback BGRA SHA differs from learned policy input")
                SHA_checks += 1
        first = min(r["callback_enter_monotonic_ns"] for r in receipts)
        last = max(r["callback_enter_monotonic_ns"] for r in receipts)
        row = dict(inference_frame=frame, ACK_sequence=sequence, source_image_timestamp_simulation_ns=stamp,
            first_callback_monotonic_ns=first, last_callback_monotonic_ns=last,
            input_ready_monotonic_ns=inputs, raw_nn_finished_monotonic_ns=nn,
            decoded_plan_ready_monotonic_ns=plan, ACK_finished_monotonic_ns=ack["ACK_finished_monotonic_ns"],
            callback_first_to_plan_seconds=(plan - first) / 1.e9,
            callback_last_to_plan_seconds=(plan - last) / 1.e9,
            callback_first_to_ACK_seconds=(ack["ACK_finished_monotonic_ns"] - first) / 1.e9,
            callback_last_to_ACK_seconds=(ack["ACK_finished_monotonic_ns"] - last) / 1.e9,
            decoded_plan_to_ACK_seconds=(ack["ACK_finished_monotonic_ns"] - plan) / 1.e9,
            source_SHA_history_checked=True)
        results.append(row)
        for item in receipts:
            per_role[item["role"]].append((plan - item["callback_enter_monotonic_ns"]) / 1.e9)
    metric_names = ("callback_first_to_plan_seconds", "callback_last_to_plan_seconds",
                    "callback_first_to_ACK_seconds", "callback_last_to_ACK_seconds", "decoded_plan_to_ACK_seconds")
    interval = [(b["decoded_plan_ready_monotonic_ns"] - a["decoded_plan_ready_monotonic_ns"]) / 1.e9
                for a, b in zip(results, results[1:])]
    require(all(v > 0 for v in interval), "actual decoded-plan wall clock reversed")
    elapsed = (results[-1]["decoded_plan_ready_monotonic_ns"] - results[0]["decoded_plan_ready_monotonic_ns"]) / 1.e9
    return dict(callback_receipt_identity_and_SHA_passed=True, callback_plan_ACK_chain_passed=True,
        actual_six_camera_callback_observed=True, callback_count=len(callbacks), selected_callback_count=450 * 6,
        extra_unselected_callback_count=len(callbacks) - 450 * 6, fresh_NN_plan_count=len(results),
        complete_all_callback_receipts_retained=True, complete_all_fresh_plan_receipts_retained=True,
        callback_to_temporal_input_SHA_comparison_count=SHA_checks,
        metrics={name: _stats([row[name] for row in results]) for name in metric_names},
        per_camera_callback_to_plan_seconds={role: _stats(vals) for role, vals in per_role.items()},
        latest_six_camera_callback_arrival_skew_seconds=_stats([v / 1.e9 for v in arrival]),
        decoded_plan_wall_interval_seconds=_stats(interval),
        observed_decoded_plan_wall_rate_hz=(len(results) - 1) / elapsed if elapsed > 0 else None,
        fresh_callback_to_plan_wall100ms_passed=all(r["callback_first_to_plan_seconds"] <= .1 for r in results),
        fresh_callback_to_ACK_wall100ms_passed=all(r["callback_first_to_ACK_seconds"] <= .1 for r in results),
        callback_to_plan_deadline_seconds=.1, callback_to_ACK_deadline_seconds=.1,
        hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
        CARLA_simulation_timestamp_used_as_wall_exposure=False,
        held_or_teacher_commands_counted_as_fresh_NN=False, independent_validation=False,
        training_data_approved=False, full_RTE01_approved=False, model_promoted=False, rows=results)
