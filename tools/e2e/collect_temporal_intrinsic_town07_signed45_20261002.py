"""HH_261002 - Protect signed straight driving with both fixed intrinsic-pair endpoints; no Town03 fixture or new control law."""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path

from . import collect_temporal_route_joint_preview05_signed45_20261002 as previous
from . import collect_temporal_intrinsic_town03_right_20261002 as proof

frozen, collector, task, manual = previous.frozen, previous.collector, previous.task, previous.manual
PROFILES, profile, validate_capture = previous.PROFILES, previous.profile, previous.validate_capture
THREAD_SCHEMA, PREVIEW_SCHEMA = previous.THREAD_SCHEMA, previous.PREVIEW_SCHEMA
SCHEMA = "portable_e2e.actual_intrinsic_pair_town07_signed45.20261002.v1"
OWNER_SCHEMA = "portable_e2e.owned_intrinsic_pair_town07_signed45.20261002.v1"
PARENT_SHA = "e0b3f5aeec01ce035545ccc92dd26cba274463024486928a3ef72de099892085"
PROOF_SHA = "15d774caec28b663c2f28b9f0e701a5ad7fc9118489f592814a50fe8db4b0385"
_joint = previous.previous.previous
_binding = _joint.validate_model_binding
_original_capture = collector.collect_episode
_active = False
NAMES = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
         "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")


def verify_frozen():
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PARENT_SHA
        and hashlib.sha256(Path(proof.__file__).read_bytes()).hexdigest() == PROOF_SHA,
        "frozen preview05/actual recorder proof provider changed")
    previous.verify_frozen(); proof.verify_frozen()
    frozen._require(_original_capture.__globals__ is vars(collector),
                    "signed45 must use the original live-module capture, not copied mutable callback globals")


def capture_factory(delegated):
    def capture(*positional, **kwargs):
        # HH_261002 - These providers remain in ORIGINAL module globals; all signed/session wrappers delegate there.
        recorder = proof._recorder()
        frozen._require(_original_capture.__globals__ is vars(collector)
            and all(_original_capture.__globals__[name] is getattr(collector, name) for name in NAMES),
            "actual original camera/bootstrap/ACK/JPEG provider identity differs")
        frozen._require(collector.checked_agent_initialization_observation is manual.checked_first_gear_initialization_observation,
                        "actual manual1 bootstrap checker must precede signed capture")
        partial = positional[3]
        record = dict(schema=SCHEMA+".actual_recording_binding", original_capture_live_module_globals=True,
            copied_mutable_provider_names=list(NAMES), all_actual_providers_current=True,
            enqueue_and_ACK_share_exact_recorder=True, source_image_transformed=False,
            sensor_callback_entry_observed=False, exposure_to_ACK_latency_measured=False)
        try: return delegated(*positional, **kwargs)
        finally:
            record.update(actual_recording_bundle_count=len(recorder.events),
                pending_source_camera_count=len(recorder.pending),
                actual_after_ACK_receipt_count=sum(row["acknowledged_before_recording"] for row in recorder.events),
                every_recording_event_preserved=True)
            frozen._write_new(partial/"actual_recording_binding.json", record)
    return capture


@contextmanager
def activated(model, args):
    global _active
    frozen._require(not _active and not previous._active and _joint.validate_model_binding is _binding,
                    "unknown or nested signed45 intrinsic bindings")
    verify_frozen(); proof.validate_model_binding(model, args)
    saved_schema = previous.SCHEMA; _active = True
    try:
        previous.SCHEMA = SCHEMA; _joint.validate_model_binding = proof.validate_model_binding
        with previous.activated(model, args):
            delegated = collector.collect_episode
            collector.collect_episode = capture_factory(delegated)
            try: yield
            finally: collector.collect_episode = delegated
    finally:
        previous.SCHEMA, _joint.validate_model_binding = saved_schema, _binding
        _active = False
