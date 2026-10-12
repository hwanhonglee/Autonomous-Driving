"""HH_261002 - Calibrate only causal learned speed preview, preserving original STOP and signed45 guards."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import time

from portable_e2e import temporal_simulation_research_v2_20261001 as runtime
from . import collect_temporal_route_joint_thread4_signed45_20261002 as previous

frozen, collector, task, manual = previous.frozen, previous.collector, previous.task, previous.manual
PROFILES, profile, validate_seed, validate_capture = previous.PROFILES, previous.profile, previous.validate_seed, previous.validate_capture
THREAD_SCHEMA, thread_readback = previous.THREAD_SCHEMA, previous.thread_readback
SCHEMA = "portable_e2e.actual_route_joint_thread4_preview08_signed45.20261002.v1"
PREVIEW_SCHEMA = "portable_e2e.causal_selected_speed_preview08_calibration.20261002.v1"
FROZEN_COLLECTOR_SHA = "c4620641216713be49213edcefbf6a2cb5c57978fbcbd4b5f4e529f6d8500e15"
FROZEN_RUNTIME_SHA = "772275dd9e452c70bd9d5364bbacb32d8bf08438ce8bf3526f7ebcf3d64884dc"
_original_session = previous.Thread4ObservedSession
_actor_config = frozen.original.actor_controller_config
_trajectory_control = runtime.trajectory_control
_active, _proposal_session = False, None


def verify_frozen():
    for module, expected in ((previous, FROZEN_COLLECTOR_SHA), (runtime, FROZEN_RUNTIME_SHA)):
        frozen._require(hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() == expected,
                        "reviewed preview calibration dependency changed")
    previous.verify_frozen()


def actor_controller_config(physics, speed_cap_mps):
    # HH_261002 - Keep all actual measured geometry/gains; only the declared speed-preview field changes.
    original = _actor_config(physics, speed_cap_mps)
    frozen._require(original.speed_preview_s == .3, "original speed preview must remain0.3s")
    changed = replace(original, speed_preview_s=.8)
    changed.validate()
    frozen._require({k: v for k, v in asdict(changed).items() if k != "speed_preview_s"}
        == {k: v for k, v in asdict(original).items() if k != "speed_preview_s"},
        "preview calibration changed another controller parameter")
    return changed


def preview_selection(selected, config):
    """HH_261002 - Sample exactly the original round-index formula; never replace a trajectory or introduce a floor."""
    config.validate()
    frozen._require(config.speed_preview_s == .8 and len(selected.speed_mps) == len(selected.xy_base_m) == 64
        and not isinstance(selected.timestep_s, bool) and selected.timestep_s == .1
        and all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) and v >= 0
            for v in selected.speed_mps), "validated64-knot selected speed ABI/preview differs")
    index = lambda seconds: min(len(selected.speed_mps)-1, max(0, round(seconds/selected.timestep_s)-1))
    original_index, proposed_index = index(.3), index(.8)
    original_target = min(config.speed_cap_mps, selected.speed_mps[original_index])
    proposed_target = min(config.speed_cap_mps, selected.speed_mps[proposed_index])
    # HH_261002 - Current selected ABI has no intent; any future explicit non-GO intent keeps the entire old tracker.
    intent = getattr(selected, "intent", 0)
    go = type(intent) is int and intent == 0
    guard = ("non_GO_selected_intent" if not go else
             "either_selected_preview_target_at_or_below_original_STOP_threshold" if min(original_target, proposed_target) <= .05 else None)
    chosen = .3 if guard is not None else .8
    return dict(schema=PREVIEW_SCHEMA, configured_speed_preview_s=.8, effective_speed_preview_s=chosen,
        original_speed_preview_s=.3, original_preview_index=original_index, proposed_preview_index=proposed_index,
        original_selected_target_mps=original_target, proposed_selected_target_mps=proposed_target,
        effective_selected_target_mps=original_target if guard is not None else proposed_target,
        original_STOP_threshold_mps=.05, original_STOP_tracker_preserved=guard is not None,
        fallback_reason=guard, selected_intent_available=hasattr(selected, "intent"),
        explicit_cruise_GO_not_trained_STOP=True, selected_trajectory_changed=False,
        target_is_only_a_causal_selected_NN_speed_knot=True, actual_measured_speed_clipped=False,
        controller_calibration_not_model_improvement=True)


def trajectory_control(selected, current_speed_mps, config):
    # HH_261002 - Invoke the frozen tracker exactly once, followed by its unchanged single PI update in policy.step.
    frozen._require(_active and _proposal_session is not None, "exclusive observed preview proposal scope required")
    frozen._require(_proposal_session._preview_tracker_receipt is None, "tracker or PI must not be called twice per fresh decision")
    audit = preview_selection(selected, config)
    result = _trajectory_control(selected, current_speed_mps, replace(config, speed_preview_s=audit["effective_speed_preview_s"]))
    frozen._require(result["desired_speed_mps"] == audit["effective_selected_target_mps"], "frozen tracker preview target differs")
    _proposal_session._preview_tracker_receipt = dict(audit, original_tracker_control=dict(result))
    return result


class Preview08ObservedSession(_original_session):
    """HH_261002 - Record complete actual controller readbacks; preserve thread4/startup/prefill/held/emergency behavior."""

    def propose(self, transport, teacher, reason):
        global _proposal_session
        normal = reason in ("initial_drive_control", "next_driving_control") and bool(self.states)
        if not normal:
            return super().propose(transport, teacher, reason)
        frozen._require(_active and _proposal_session is None, "preview session proposal cannot overlap")
        if self.policy is not None:
            frozen._require(self.policy.controller.speed_preview_s == .8, "actual configured speed preview drifted")
        self._preview_tracker_receipt = None
        started = time.perf_counter_ns(); _proposal_session = self
        try:
            command, decision = super().propose(transport, teacher, reason)
        finally:
            _proposal_session = None
        controller = asdict(self.policy.controller) if self.policy is not None else None
        frozen._require(controller is not None and controller["speed_preview_s"] == .8,
                        "actual constructed preview0.8 controller missing")
        receipt = self._preview_tracker_receipt
        if decision is not None:
            # HH_261002 - Supersede the inherited tracker-unchanged label without changing any raw prediction or control.
            decision.update(decoder_gate_tracker_math_unchanged_except_disclosed_headroom=False,
                decoder_gate_tracker_math_unchanged_except_disclosed_headroom_and_speed_preview=True,
                causal_selected_speed_preview_calibration_schema=PREVIEW_SCHEMA)
        record = dict(schema=PREVIEW_SCHEMA, command_frame=self.states[-1]["frame"], command_reason=reason,
            proposal_started_monotonic_ns=started, proposal_finished_monotonic_ns=time.perf_counter_ns(),
            actual_policy_controller=controller, tracker_receipt=receipt,
            control_source=decision.get("control_source") if decision else None,
            inference_frame=decision.get("inference_frame") if decision else None,
            learned_control_proposed=bool(decision and decision.get("learned_control_proposed")),
            held_between_camera_frames=bool(decision and decision.get("held_between_camera_frames")),
            actual_returned_control=decision.get("control") if decision else None,
            guard_and_tracker_within_unchanged_policy100ms=True,
            journal_outside_policy100ms_inside_original_proposal_to_ACK=True)
        with (self.output/"speed_preview_configuration.jsonl").open("a") as stream:
            stream.write(json.dumps(record, allow_nan=False)+"\n")
        return command, decision


@contextmanager
def activated(model, args):
    """HH_261002 - Scoped configuration/control binding only; restore every frozen delegate after success or failure."""
    global _active
    frozen._require(not _active and not previous._active and _proposal_session is None, "preview collector cannot nest")
    verify_frozen(); previous.previous.validate_model_binding(model, args)
    frozen._require(getattr(model, "_research_speed_preview_calibration", None) == PREVIEW_SCHEMA,
                    "explicit preview owner binding required")
    frozen._require(previous.Thread4ObservedSession is _original_session
        and frozen.original.actor_controller_config is _actor_config and runtime.trajectory_control is _trajectory_control,
        "unknown original session/geometry/tracker binding")
    saved = (previous.SCHEMA, previous.Thread4ObservedSession, frozen.original.actor_controller_config,
             runtime.trajectory_control, collector.collect_episode)
    collect = collector.collect_episode

    def capture(*positional, **kwargs):
        positional[6].setdefault("capture_contract", {})["causal_selected_speed_preview_calibration"] = dict(
            schema=PREVIEW_SCHEMA, original_speed_preview_s=.3, configured_speed_preview_s=.8,
            either_preview_STOP_or_non_GO_preserves_entire_original_tracker=True,
            readback_journal="speed_preview_configuration.jsonl", unchanged_PI_target_cap_mps=2.4,
            unchanged_policy_deadline_seconds=.1, unchanged_observed_bundle_to_ACK_deadline_seconds=.1,
            NN_raw_decoded_selected_and_measured_speed_unchanged=True, model_promoted=False, vehicle_control_approved=False)
        return collect(*positional, **kwargs)

    _active = True
    previous.SCHEMA = SCHEMA; previous.Thread4ObservedSession = Preview08ObservedSession
    frozen.original.actor_controller_config = actor_controller_config; runtime.trajectory_control = trajectory_control
    collector.collect_episode = capture
    try:
        with previous.activated(model, args):
            yield
    finally:
        previous.SCHEMA, previous.Thread4ObservedSession, frozen.original.actor_controller_config, runtime.trajectory_control, collector.collect_episode = saved
        _active = False
