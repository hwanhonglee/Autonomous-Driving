"""HH_261008 - Connect provided ownership and native sensor headers through injectable callbacks.

This new nontraining adapter does not implement a live owner, a kernel observer,
a first RPC, CARLA registration, file permissions, actuation or TRAIN admission.
The separately sealed owner supplies the declaration, original first-RPC receipt,
fresh read-only metadata, actual callback objects and artifact permissions. The
metadata provider MUST NOT perform RPCs or mutate the world while obtaining its
observations. Checking caller metadata cannot authenticate that provider or the
receipt seal. This module imports no simulator, image, numeric or ML packages.

Every effect callback has a fresh ownership boundary immediately before it.
Creation, tick, each receive, artifact write/read and each stop/destroy are distinct
boundaries. Callback-supplied saved hashes remain claims about storage identity;
even a matching provided read buffer is not proof of a regular file or its path.
The caller must seal actual sources/assets/output paths and separately record its
kernel/lease/storage checks. The declared limits here are bounds, not authority.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json

from . import mapping_capture_ownership_20261008_v2 as ownership
from . import mapping_sensor_provenance_20261008_v2 as provenance

SCHEMA = "portable_e2e.mapping_capture_adapter.20261008.v1"
MAX_METADATA_BYTES = 8 * 1024 * 1024
MAX_FRAMES = 1209
FAILURE_RESERVE_BYTES = 64 * 1024
HELPER_PINS = {
    "mapping_capture_ownership_20261008_v2.py": "dfb37e3ee6970a76393d593675539c21393eed72cbb1068163347aa5f1fc6c43",
    "mapping_sensor_provenance_20261008_v2.py": "29683378b1d6aa4562bbaecdcba50444f68e420b2751d048c1dcdb957d6a4f50",
}


@dataclass(frozen=True)
class Limits:
    """HH_261008 - Caller-declared resource bounds never confer execution permission."""
    frames: int
    metadata_bytes: int = MAX_METADATA_BYTES
    artifact_bytes: int = MAX_METADATA_BYTES

    def validate(self):
        if not ownership.integer(self.frames, 1) or self.frames > MAX_FRAMES:
            raise ValueError("declared frames must be a strict integer in 1..1209")
        if (not ownership.integer(self.metadata_bytes, 1)
                or not FAILURE_RESERVE_BYTES * 2 <= self.metadata_bytes <= MAX_METADATA_BYTES):
            raise ValueError("metadata bound must be 128KiB..8MiB with a retained failure reserve")
        if not ownership.integer(self.artifact_bytes, 1) or self.artifact_bytes > MAX_METADATA_BYTES:
            raise ValueError("per-artifact provided read buffer bound must be 1 byte..8MiB")


@dataclass(frozen=True)
class CreatedActor:
    """Actual callback object plus its owner-provided, independently checked identity."""
    actor: object
    identity: dict


@dataclass(frozen=True)
class TickResult:
    """Actual tick return and snapshot already received by the guarded tick callback."""
    required_frame: int
    snapshot: object


@dataclass(frozen=True)
class Callbacks:
    create_actor: object
    tick: object
    receive: object
    write_artifacts: object
    destroy_actor: object
    stop_sensor: object = None
    read_artifact: object = None

    def validate(self):
        for name in ("create_actor", "tick", "receive", "write_artifacts", "destroy_actor"):
            if not callable(getattr(self, name)):
                raise ValueError(name + " must be an explicitly injected callback")
        for name in ("stop_sensor", "read_artifact"):
            value = getattr(self, name)
            if value is not None and not callable(value):
                raise ValueError(name + " must be None or an explicitly injected callback")


class ArtifactCallbackFailure(RuntimeError):
    """HH_261008 - A writer may retain actual partial artifact receipts when it fails."""
    def __init__(self, message, *, artifacts):
        super().__init__(message)
        self.artifacts = artifacts


class _Denied(RuntimeError):
    def __init__(self, stage, failures):
        self.stage = stage
        self.failures = failures
        super().__init__(", ".join(str(value) for value in failures))


def _encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _sha(value):
    return hashlib.sha256(_encoded(value)).hexdigest()


def _bounded(value):
    """Reject oversized scalar input without duplicating arbitrary callback metadata."""
    retained = ownership.retain(value)
    if retained != value:
        raise ValueError("callback metadata exceeds the unchanged helper scalar contract")
    return retained


def _error(failure):
    # HH_261008 - Keep a bounded explicit exception representation, not a successful substitute.
    rendered = repr(failure)
    return dict(type=type(failure).__name__, repr=rendered[:4096], repr_truncated=len(rendered) > 4096)


def _snapshot(snapshot):
    return ownership.retain(dict(frame=snapshot.frame, elapsed_seconds=snapshot.timestamp.elapsed_seconds,
                                  delta_seconds=snapshot.timestamp.delta_seconds))


def _received_header(image):
    return _bounded(dict(frame=image.frame, timestamp_s=image.timestamp))


class _Run:
    def __init__(self, declared, first, fresh_provider, callbacks, limits):
        self.limits, self.provider, self.callbacks = limits, fresh_provider, callbacks
        self.declared, self.first = _bounded(declared), _bounded(first)
        self.events, self.handles = [], {}
        self.metadata_used = 0
        self.result = dict(schema=SCHEMA,
            provenance="HH_261008 - Nontraining callback integration; no live-owner or storage authentication.",
            declaration=self.declared, first_rpc_receipt=self.first, helper_source_pins=HELPER_PINS.copy(),
            declared_limits=dict(frames=limits.frames, metadata_bytes=limits.metadata_bytes,
                                 per_artifact_read_buffer_bytes=limits.artifact_bytes),
            observations={}, boundaries=[], creation_events=self.events, native_created_actor_witnesses=[],
            frames=[], cleanup_actions=[],
            primary_failure=None, cleanup_errors=[], provided_actual_created_ids=[], uncleaned_actor_ids=[],
            expected_frames=limits.frames, attempted_frame_slots=0, passed_frames=0, failed_frames=0,
            UNEXECUTED_frames=limits.frames, actual_tick_callback_attempts=0,
            actual_receive_callback_attempts=0, actual_artifact_write_callback_attempts=0,
            actual_artifact_read_callback_attempts=0, actual_stop_callback_attempts=0,
            actual_destroy_callback_attempts=0,
            expected_camera_slots=limits.frames*3, attempted_camera_slots=0,
            received_camera_headers=0, failed_camera_attempts=0, UNEXECUTED_camera_slots=limits.frames*3,
            callback_effects_attempted=False, first_RPC_performed_by_adapter=False,
            evidence_origin="caller_provided_objects_and_callbacks_not_authenticated_by_adapter",
            provider_freshness_authenticated=False, first_receipt_seal_authenticated=False,
            kernel_ownership_or_file_identity_authenticated=False,
            artifact_storage_identity_authenticated=False, callback_permissions_authorized_by_adapter=False,
            actual_CARLA_capture_claimed=False, TRAIN_admitted=False, learned_control=False,
            failed_raw_capture_newly_TRAIN_eligible=False, hd_map_approved=False)
        self.metadata_used = len(_encoded(self.result))
        if self.metadata_used > limits.metadata_bytes - FAILURE_RESERVE_BYTES:
            raise ValueError("initial metadata exceeds the declared bound")

    def _reserve(self, value, *, final=False):
        # HH_261008 - Incremental accounting avoids repeatedly serializing every earlier frame.
        amount = len(_encoded(value)) + 128
        limit = self.limits.metadata_bytes if final else self.limits.metadata_bytes - FAILURE_RESERVE_BYTES
        if self.metadata_used + amount > limit:
            raise _Denied("metadata", ["declared_metadata_budget_exceeded"])
        self.metadata_used += amount

    def fail(self, stage, failure, *, ordinal=None):
        value = dict(stage=stage, ordinal=ordinal, error=_error(failure))
        if isinstance(failure, _Denied):
            value["failures"] = list(failure.failures)
            value["boundary_stage"] = failure.stage
        if self.result["primary_failure"] is None:
            self.result["primary_failure"] = value
        return value

    def guard(self, action, stage):
        """Fresh read-only owner metadata is the only callback preceding this guard."""
        supplied = self.provider(action)
        if type(supplied) is not dict or set(supplied) != {"observed", "live_actors"}:
            raise _Denied(action, ["exact_fresh_provider_observation_and_census_required"])
        supplied = _bounded(supplied)
        receipt = ownership.assess_boundary(self.declared, supplied["observed"], stage=stage,
            first_rpc_receipt=self.first, creation_events=self.events, live_actors=supplied["live_actors"])
        reference = _sha(supplied)
        evidence = dict(action=action, stage=stage, observation_ref=reference,
                        creation_event_count=len(self.events), receipt_sha256=_sha(receipt),
                        failures=list(receipt["failures"]), native_object_failures=[])
        # HH_261008 - A callback object cannot silently differ from an otherwise consistent claimed actor registry.
        for event in self.events:
            handle = self.handles.get(event.get("role"))
            identity = event.get("actor")
            if event.get("status") != "created" or handle is None:
                continue
            try:
                if (type(identity) is not dict or type(handle.actor.id) is not type(identity.get("id"))
                        or handle.actor.id != identity.get("id") or handle.actor.type_id != identity.get("type_id")):
                    evidence["native_object_failures"].append("provided_native_actor_identity_differs:"+event["role"])
            except Exception as failure:
                evidence["native_object_failures"].append("provided_native_actor_identity_unreadable:"+event["role"])
        if reference not in self.result["observations"]:
            self._reserve(supplied, final=stage == "before_cleanup")
            self.result["observations"][reference] = supplied
        self._reserve(evidence, final=stage == "before_cleanup")
        self.result["boundaries"].append(evidence)
        if evidence["failures"] or evidence["native_object_failures"]:
            raise _Denied(action, evidence["failures"]+evidence["native_object_failures"])
        return receipt

    def effect(self, counter, callback, *args):
        self.result[counter] += 1
        self.result["callback_effects_attempted"] = True
        return callback(*args)

    def create(self):
        for ordinal, spec in enumerate(self.declared["actor_specs"]):
            self.guard("create:"+spec["role"], "before_actor_creation")
            event = dict(ordinal=ordinal, role=spec["role"], status="failed", actor=None, error=None)
            try:
                self.result["callback_effects_attempted"] = True
                parent = self.handles.get(spec["parent_role"])
                handle = self.callbacks.create_actor(_bounded(spec), None if parent is None else parent.actor)
                if not isinstance(handle, CreatedActor):
                    raise ValueError("create callback must return its actual CreatedActor object and identity")
                # HH_261008 - Retain supplied created-ID witnesses even when the callback identity is malformed.
                event.update(status="created", actor=ownership.retain(handle.identity))
                self.handles[spec["role"]] = handle
                identity = _bounded(handle.identity)
                self.result["native_created_actor_witnesses"].append(ownership.retain(dict(
                    ordinal=ordinal, role=spec["role"], id=handle.actor.id, type_id=handle.actor.type_id)))
                # HH_261008 - Compare native ID/type directly; parent/role claims still require the sealed owner.
                if (type(handle.actor.id) is not type(identity.get("id")) or handle.actor.id != identity.get("id")
                        or handle.actor.type_id != identity.get("type_id")):
                    raise ValueError("created native object ID/type differs from provided identity")
                self.events.append(event)
                # HH_261008 - The next check retains a malformed actual created ID rather than recasting it as absent.
                self.guard("after_create:"+spec["role"], "before_actor_creation")
            except Exception as failure:
                event["error"] = repr(failure)[:4096] if event["status"] == "failed" else None
                if event not in self.events:
                    self.events.append(event)
                raise

    def _read_artifacts(self, frame, artifacts):
        if self.callbacks.read_artifact is None:
            frame["artifact_read_verification"] = "NOT_REQUESTED_CALLBACK_RECEIPTS_ONLY"
            return
        frame["artifact_read_verification"] = []
        for artifact in artifacts:
            self.guard("read_artifact:"+artifact["kind"], "before_tick")
            raw = self.effect("actual_artifact_read_callback_attempts", self.callbacks.read_artifact, _bounded(artifact))
            view = memoryview(raw)
            try:
                if not view.c_contiguous or view.ndim != 1 or view.itemsize != 1:
                    raise ValueError("provided artifact read must return a contiguous one-dimensional byte buffer")
                if not 0 < view.nbytes <= self.limits.artifact_bytes:
                    raise ValueError("provided artifact read buffer exceeds its declared bound")
                observed_sha = hashlib.sha256(view).hexdigest()
                check = dict(kind=artifact["kind"], path=artifact["path"], actual_provided_bytes=view.nbytes,
                    actual_provided_sha256=observed_sha,
                    matches_callback_receipt=view.nbytes == artifact["size_bytes"] and observed_sha == artifact["sha256"],
                    storage_identity_authenticated=False)
                frame["artifact_read_verification"].append(check)
                if not check["matches_callback_receipt"]:
                    raise _Denied("read_artifact", ["provided_saved_artifact_bytes_or_hash_differ"])
            finally:
                view.release()

    def capture_frame(self, ordinal):
        frame = dict(ordinal=ordinal, status="FAILED_RETAINED", required_frame=None, world_snapshot=None,
                     receive_events=[], receive_receipt=None, header_receipt=None, saved_artifacts=[],
                     artifact_callback_partial_outputs_unknown=False, failures=[])
        self.result["attempted_frame_slots"] += 1
        self.result["UNEXECUTED_frames"] -= 1
        try:
            tick_boundary = self.guard("tick:"+str(ordinal), "before_tick")
            tick = self.effect("actual_tick_callback_attempts", self.callbacks.tick)
            if not isinstance(tick, TickResult) or not ownership.integer(tick.required_frame):
                raise ValueError("tick callback must return its native frame and actual snapshot")
            frame["required_frame"] = tick.required_frame
            frame["world_snapshot"] = _snapshot(tick.snapshot)
            images = []
            for role in ownership.CAMERAS:
                handle = self.handles[role]
                self.guard("receive:"+str(ordinal)+":"+role, "before_tick")
                event = dict(ordinal=ordinal, role=role, actor_id=handle.identity["id"],
                    required_frame=tick.required_frame, status="error", observed_header=None, error=None)
                self.result["attempted_camera_slots"] += 1
                self.result["UNEXECUTED_camera_slots"] -= 1
                try:
                    image = self.effect("actual_receive_callback_attempts", self.callbacks.receive,
                                        role, handle.actor, tick.required_frame)
                    event["observed_header"] = _received_header(image)
                    event["status"] = "received"
                    images.append(image)
                    self.result["received_camera_headers"] += 1
                except Exception as failure:
                    event["status"] = "timeout" if isinstance(failure, TimeoutError) else "error"
                    event["error"] = repr(failure)[:4096]
                    self.result["failed_camera_attempts"] += 1
                    raise
                finally:
                    frame["receive_events"].append(event)
            self.guard("headers:"+str(ordinal), "before_tick")
            actors = tuple(self.handles[role].actor for role in ownership.CAMERAS)
            observed = provenance.observe_headers(tick.snapshot, actors, tuple(images),
                                                  ordinal=ordinal, required_frame=tick.required_frame)
            # HH_261008 - Only bounded extracted metadata reaches assessment or persistent result records.
            observed = _bounded(observed)
            headers = provenance.assess_headers(observed)
            frame["header_receipt"] = headers
            if headers["failures"]:
                raise _Denied("headers", headers["failures"])
            self.guard("write_artifacts:"+str(ordinal), "before_tick")
            try:
                artifacts = self.effect("actual_artifact_write_callback_attempts", self.callbacks.write_artifacts,
                                        ordinal, tick.snapshot, actors, tuple(images), _bounded(headers))
                artifacts = _bounded(artifacts)
                frame["saved_artifacts"] = artifacts
                bound = provenance.bind_saved_artifacts(headers, artifacts)
                frame["header_receipt"] = bound
            except Exception as failure:
                if isinstance(failure, ArtifactCallbackFailure):
                    frame["saved_artifacts"] = ownership.retain(failure.artifacts)
                else:
                    frame["artifact_callback_partial_outputs_unknown"] = not bool(frame["saved_artifacts"])
                raise
            self._read_artifacts(frame, artifacts)
            self.guard("after_artifacts:"+str(ordinal), "before_tick")
            after = _bounded(provenance.observe_headers(tick.snapshot, actors, tuple(images),
                                                        ordinal=ordinal, required_frame=tick.required_frame))
            frame["provided_headers_unchanged_after_artifact_callbacks"] = after == observed
            if after != observed:
                frame["changed_header_evidence_after_artifact_callbacks"] = after
                raise _Denied("after_artifacts", ["native_headers_or_raw_buffers_changed_during_artifact_callbacks"])
            frame["status"] = "COMPLETE_PROVIDED_OBJECTS_AND_ARTIFACT_CALLBACKS_NONTRAINING"
        except Exception as failure:
            frame["failures"].append(self.fail("frame", failure, ordinal=ordinal))
        finally:
            if frame["world_snapshot"] is not None:
                frame["receive_receipt"] = ownership.assess_receive(self.declared, tick_boundary, self.events,
                    ordinal=ordinal, required_frame=frame["required_frame"], world_snapshot=frame["world_snapshot"],
                    receive_events=frame["receive_events"])
                # HH_261008 - Avoid repeating immutable creation IDs/witnesses and owner flags in every frame.
                full = frame["receive_receipt"]
                frame["receive_receipt"] = {key: full[key] for key in (
                    "failures", "expected_camera_attempts", "attempted_camera_slots", "received_header_records",
                    "failed_camera_attempts", "UNEXECUTED_camera_attempts", "actual_provided_camera_attempt_count",
                    "extra_camera_attempt_count", "missing_headers_fabricated", "partial_raw_training_eligible")}
                if full["failures"] and not frame["failures"]:
                    failure = _Denied("after_receive", full["failures"])
                    frame["failures"].append(self.fail("after_receive", failure, ordinal=ordinal))
                    frame["status"] = "FAILED_RETAINED"
            try:
                self._reserve(frame)
            except _Denied as failure:
                frame["status"] = "FAILED_RETAINED"
                frame["failures"].append(self.fail("metadata", failure, ordinal=ordinal))
                self._reserve(frame, final=True)
            self.result["frames"].append(frame)
            self.result["passed_frames" if not frame["failures"] else "failed_frames"] += 1
        return not frame["failures"]

    def cleanup(self):
        created = [event for event in self.events if event["status"] == "created" and type(event["actor"]) is dict]
        ids = [event["actor"].get("id") for event in created]
        self.result["provided_actual_created_ids"] = ids.copy()
        self.result["uncleaned_actor_ids"] = ids.copy()
        for event in reversed(created):
            role, actor_id = event["role"], event["actor"].get("id")
            handle = self.handles.get(role)
            actions = ["stop", "destroy"] if role != "ego" and self.callbacks.stop_sensor is not None else ["destroy"]
            for action in actions:
                outcome = dict(role=role, actor_id=actor_id, action=action, status="DENIED_RETAINED", error=None)
                try:
                    receipt = self.guard("cleanup:"+action+":"+role, "before_cleanup")
                    if actor_id not in receipt["cleanup_candidate_ids"]:
                        outcome["status"] = "MISSING_FROM_PROVIDED_CENSUS_DESTRUCTION_NOT_CLAIMED"
                        raise _Denied("cleanup", ["created_actor_missing_without_destruction_evidence"])
                    elif handle is None:
                        raise ValueError("actual created object unavailable for cleanup callback")
                    else:
                        callback = self.callbacks.stop_sensor if action == "stop" else self.callbacks.destroy_actor
                        value = self.effect("actual_"+action+"_callback_attempts", callback, role, handle.actor)
                        if value is not True:
                            raise ValueError("cleanup callback must explicitly report True after its actual action")
                        outcome["status"] = "CALLBACK_REPORTED_COMPLETE_NOT_AUTHENTICATED"
                        if action == "destroy" and actor_id in self.result["uncleaned_actor_ids"]:
                            self.result["uncleaned_actor_ids"].remove(actor_id)
                except Exception as failure:
                    outcome["error"] = _error(failure)
                    if isinstance(failure, _Denied):
                        outcome["failures"] = failure.failures
                    self.result["cleanup_errors"].append(outcome.copy())
                    self.fail("cleanup", failure)
                try:
                    self._reserve(outcome, final=True)
                    self.result["cleanup_actions"].append(outcome)
                except _Denied as failure:
                    self.fail("cleanup_metadata", failure)
                    return

    def execute(self):
        try:
            if ownership.declaration_failures(self.declared):
                raise _Denied("declaration", ownership.declaration_failures(self.declared))
            # HH_261008 - Recompute the entire first receipt contract without claiming the external seal is authenticated.
            expected = ownership.assess_boundary(self.declared, self.first.get("observed"), stage="before_first_rpc")
            if expected != self.first or expected["failures"]:
                raise _Denied("first_rpc_receipt", ["sealed_first_RPC_receipt_contract_differs_or_failed"])
            self.create()
            for ordinal in range(self.limits.frames):
                if not self.capture_frame(ordinal):
                    break
        except Exception as failure:
            self.fail("lifecycle", failure)
        finally:
            self.cleanup()
        self.result["status"] = ("COMPLETE_CALLBACK_INTEGRATION_NONTRAINING_NOT_CAPTURE_OR_MAP_APPROVAL"
                                 if self.result["primary_failure"] is None else "FAILED_RETAINED_NONTRAINING")
        # HH_261008 - Exact final serialization verifies the conservative incremental budget; no result is written here.
        self.result["metadata_bytes"] = 0
        final_bytes = len(_encoded(self.result))
        self.result["metadata_bytes"] = final_bytes
        final_bytes = len(_encoded(self.result))
        self.result["metadata_bytes"] = final_bytes
        final_bytes = len(_encoded(self.result))
        self.result["metadata_bytes"] = final_bytes
        if final_bytes > self.limits.metadata_bytes:
            raise RuntimeError("final scalar metadata exceeds its declared bound; owner must retain this failure")
        return self.result


def run_capture(declaration, first_rpc_receipt, fresh_provider, callbacks, *, limits):
    """Run only the owner's explicitly supplied callbacks after metadata validation.

    The adapter never obtains a first RPC or installs a live sensor listener. The
    owner provides that sealed receipt. A passed result measures callback wiring;
    it is neither actual capture authentication, learned control nor TRAIN authority.
    Exceptions from effects preserve the original primary failure and attempted
    denominator, with independently guarded cleanup outcomes in the returned result.
    """
    if not isinstance(limits, Limits) or not isinstance(callbacks, Callbacks) or not callable(fresh_provider):
        raise ValueError("explicit Limits, Callbacks and fresh read-only metadata provider required")
    limits.validate()
    callbacks.validate()
    return _Run(declaration, first_rpc_receipt, fresh_provider, callbacks, limits).execute()
