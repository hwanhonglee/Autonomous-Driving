"""HH_261002 - Actual final6000 joint weights under unchanged corrected signed45 physical guards."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path

from portable_e2e.temporal_route_joint_export_research_20261002 import (
    EXPORT_SCHEMA, OWNER_SCHEMA, SCHEMA as LOADER_SCHEMA, verify_scope,
)
from . import collect_temporal_signed_recovery45_v3_20261002 as previous

frozen, collector, task, manual = previous.frozen, previous.collector, previous.task, previous.manual
PROFILES, profile, validate_seed, validate_capture = previous.PROFILES, previous.profile, previous.validate_seed, previous.validate_capture
SCHEMA = "portable_e2e.actual_route_joint_final6000_signed45.20261002.v1"
FROZEN_COLLECTOR_SHA = "d693e11422f7b60f5b16c687bab9df093b72712f1918ab736d8114727df1af33"
_active = False


def verify_frozen():
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == FROZEN_COLLECTOR_SHA,
                    "corrected signed45 baseline collector must remain byte-exact")
    previous.verify_frozen()


def validate_model_binding(model, args):
    """HH_261002 - Permit only the explicit NEW partial-backbone export, never relabel it as frozen97."""
    description = getattr(model, "_research_temporal_export_metadata", {})
    verify_scope(description)
    binding = getattr(model, "_research_recovery_owner_binding", {})
    frozen._require(description.get("schema") == EXPORT_SCHEMA
        and getattr(model, "_research_recovery_numeric_loader_schema", None) == LOADER_SCHEMA
        and binding.get("schema") == OWNER_SCHEMA and binding.get("fixed_endpoint_updates") == 6000
        and binding.get("purpose") == "owned_carla_research_only" and binding.get("simulation_only") is True
        and binding.get("simulator_control_authorized") is True
        and getattr(model, "_research_recovery_simulator_control_authorized", False) is True
        and binding.get("owner_plan_sha256") == getattr(args, "research_source_sha256", None)
        and binding.get("vehicle_control_approved") is False and binding.get("model_promoted") is False,
        "joint numeric-only or non-final6000 weights cannot actuate")


@contextmanager
def activated(model, args):
    """HH_261002 - Delegate the same class, actual spawn/gear/teacher/mux/PI/ACK/control math without output edits."""
    global _active
    frozen._require(not _active and not previous._active, "joint signed45 activation cannot nest")
    verify_frozen(); validate_capture(args, collector.load_route(args.route_file)); validate_model_binding(model, args)
    saved_schema = previous.SCHEMA
    _active = True; previous.SCHEMA = SCHEMA
    try:
        with previous.activated(model, args):
            yield
    finally:
        previous.SCHEMA = saved_schema
        _active = False
