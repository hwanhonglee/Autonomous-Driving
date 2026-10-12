"""HH_261002 - Observe an explicitly non-bit-identical four-thread final6000 signed45 preset."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import time

import torch

from . import collect_temporal_route_joint_signed45_20261002 as previous
from . import collect_temporal_v2_owned_camera_research_20261001 as owned

frozen, collector, task, manual = previous.frozen, previous.collector, previous.task, previous.manual
PROFILES, profile, validate_seed, validate_capture = previous.PROFILES, previous.profile, previous.validate_seed, previous.validate_capture
SCHEMA = "portable_e2e.actual_route_joint_thread4_signed45.20261002.v1"
THREAD_SCHEMA = "portable_e2e.route_joint_thread4_numerical_preset.20261002.v1"
FROZEN_COLLECTOR_SHA = "013f02277db87828d67800b88e0113c7552bf64a2f534b228b3d88657f0f98ab"
OWNED_COLLECTOR_SHA = "dc31fb87df83fc908324cc0f55d28ab0c3a5d26f7f9fdf185d9bd690a61b50ad"
_original_session = owned.OwnedCameraTemporalLiveResearchSessionV2
_active = False


def thread_readback():
    result = dict(intra_op_threads=torch.get_num_threads(), inter_op_threads=torch.get_num_interop_threads())
    frozen._require(result == dict(intra_op_threads=4, inter_op_threads=1), "actual Torch4/interop1 preset differs")
    return result


def verify_frozen():
    for module, expected in ((previous, FROZEN_COLLECTOR_SHA), (owned, OWNED_COLLECTOR_SHA)):
        frozen._require(hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() == expected,
                        "reviewed delegated thread4 collector source changed")
    previous.verify_frozen()


class Thread4ObservedSession(_original_session):
    """HH_261002 - Check runtime settings before startup prefills/handoff/held commands, without altering proposals."""

    def propose(self, transport, teacher, reason):
        if reason in ("initial_drive_control", "next_driving_control") and self.states:
            record = dict(schema=THREAD_SCHEMA, command_frame=self.states[-1]["frame"], command_reason=reason,
                observed_monotonic_ns=time.perf_counter_ns(),
                intra_op_threads=torch.get_num_threads(), inter_op_threads=torch.get_num_interop_threads(),
                before_original_propose=True, before_handoff=not self.handed_off,
                settings_readback_outside_policy_deadline_inside_bundle_to_ACK=True,
                original_policy_deadline_seconds=.1, numerical_bitwise_equivalence_claimed=False)
            with (self.output / "cpu_thread_readback.jsonl").open("a") as stream:
                stream.write(json.dumps(record, allow_nan=False) + "\n")
            thread_readback()
        return super().propose(transport, teacher, reason)


@contextmanager
def activated(model, args):
    """HH_261002 - Scope only session diagnostics/schema; preserve every original policy, ACK and hazard branch."""
    global _active
    frozen._require(not _active and not previous._active and not owned._active, "thread4 collector cannot nest")
    verify_frozen(); previous.validate_model_binding(model, args)
    frozen._require(getattr(model, "_research_cpu_thread_numerical_revision", None) == THREAD_SCHEMA,
                    "explicit thread4 numerical owner binding required")
    thread_readback()
    saved_schema, saved_session, saved_collect = previous.SCHEMA, owned.OwnedCameraTemporalLiveResearchSessionV2, collector.collect_episode

    def collect(*positional, **kwargs):
        manifest = positional[6]
        manifest.setdefault("capture_contract", {})["cpu_thread_numerical_preset"] = dict(
            schema=THREAD_SCHEMA, **thread_readback(), strict_original_offline_parity_passed=False,
            exact_bitwise_or_strict_decoded_equivalence_claimed=False,
            readback_journal="cpu_thread_readback.jsonl", every_normal_proposal_checked_before_handoff=True,
            hard_policy_deadline_seconds=.1, observed_bundle_to_ACK_deadline_seconds=.1,
            no_safety_gate_or_prediction_replacement=True, model_promoted=False, vehicle_control_approved=False)
        return saved_collect(*positional, **kwargs)

    _active = True
    previous.SCHEMA = SCHEMA; owned.OwnedCameraTemporalLiveResearchSessionV2 = Thread4ObservedSession
    collector.collect_episode = collect
    try:
        with previous.activated(model, args):
            yield
    finally:
        previous.SCHEMA = saved_schema; owned.OwnedCameraTemporalLiveResearchSessionV2 = saved_session
        collector.collect_episode = saved_collect
        _active = False
