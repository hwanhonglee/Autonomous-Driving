"""HH_260906 - Isolate a numerical CARLA substep comparison without changing admission or control."""

from __future__ import annotations

import math
from typing import Any

SCHEMA = "carla.physics_substep_experiment.v1"
PROFILES = {"reference_10ms": 0.01, "fine_5ms": 0.005}
ABSOLUTE_TOLERANCE = 1e-12
BOOL_FIELDS = ("synchronous_mode", "no_rendering_mode", "substepping", "deterministic_ragdolls", "spectator_as_ego")
FLOAT_FIELDS = ("fixed_delta_seconds", "max_substep_delta_time", "max_culling_distance", "tile_stream_distance", "actor_active_distance")


def validate_arguments(args: Any) -> str:
    # HH_260906 - No arbitrary timestep, controller, initialization, or rate may enter this two-arm experiment.
    profile = getattr(args, "physics_substep_profile", "inherited")
    if profile not in PROFILES:
        raise ValueError("unknown explicit physics substep profile")
    required = {"goal_stop_profile": "turn_launch_013_v1", "control_transport": "acknowledged_batch",
                "agent_initialization": "after_bootstrap", "physics_hz": 20.0, "capture_hz": 10.0}
    for key, expected in required.items():
        if getattr(args, key, None) != expected:
            raise ValueError(f"{profile} requires {key}={expected}")
    return profile


def new_record(profile: str) -> dict[str, Any]:
    if profile not in PROFILES:
        raise ValueError("unknown explicit physics substep profile")
    return {"schema": SCHEMA, "profile": profile, "status": "NOT_REACHED", "explicit_opt_in": True,
            "numerical_only": True, "training_data_approved": False, "development_only": True,
            "comparison_absolute_tolerance": ABSOLUTE_TOLERANCE,
            "required_original": {"substepping": True, "max_substep_delta_time": 0.01, "max_substeps": 10},
            "requested_max_substep_delta_time": PROFILES[profile],
            "before": None, "requested": None, "after": None,
            "notice": "Maximum numerical substep setting, not measured internal substeps or improved physics accuracy; no dataset admission."}


def settings_snapshot(settings: Any) -> dict[str, Any]:
    # HH_260906 - Record all eleven scalar properties exposed by the local CARLA 0.9.15 WorldSettings API.
    result = {}
    for name in BOOL_FIELDS:
        value = getattr(settings, name)
        if type(value) is not bool:
            raise ValueError(f"WorldSettings {name} must be boolean")
        result[name] = value
    for name in FLOAT_FIELDS:
        value = getattr(settings, name)
        if name == "fixed_delta_seconds" and value is None:
            result[name] = None
            continue
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f"WorldSettings {name} must be finite and nonnegative")
        result[name] = float(value)
    count = getattr(settings, "max_substeps")
    if type(count) is not int or not 1 <= count <= 16:
        raise ValueError("WorldSettings max_substeps must be an integer in [1,16]")
    result["max_substeps"] = count
    return result


def equal_settings(actual: dict[str, Any], expected: dict[str, Any]) -> bool:
    if actual.keys() != expected.keys():
        return False
    return all((type(actual[k]) is type(value) and actual[k] == value)
               if type(value) is not float else
               (type(actual[k]) is float and math.isclose(actual[k], value, rel_tol=0, abs_tol=ABSOLUTE_TOLERANCE))
               for k, value in expected.items())


def configure_world(world: Any, original_settings: Any, args: Any, record: dict[str, Any]) -> None:
    # HH_260906 - Readback must succeed before any actor creation; failures retain evidence and have no fallback.
    try:
        profile = validate_arguments(args)
        if record != new_record(profile):
            raise ValueError("physics substep record must be fresh and match the requested profile")
        before = settings_snapshot(original_settings)
        record["before"] = before
        reference = record["required_original"]
        if not equal_settings({k: before[k] for k in reference}, reference):
            raise ValueError("original CARLA substeps differ from reviewed true/0.01/10 reference")
        settings = world.get_settings()
        if not equal_settings(settings_snapshot(settings), before):
            raise ValueError("CARLA world settings changed before substep application")
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.max_substep_delta_time = PROFILES[profile]
        requested = dict(before, synchronous_mode=True, fixed_delta_seconds=0.05,
                         max_substep_delta_time=PROFILES[profile])
        record["requested"] = requested
        if not equal_settings(settings_snapshot(settings), requested):
            raise ValueError("requested CARLA world settings differ from the fixed experiment")
        world.apply_settings(settings)
        record["after"] = settings_snapshot(world.get_settings())
        if not equal_settings(record["after"], requested):
            raise ValueError("CARLA physics substep readback mismatch")
        record["status"] = "PASS"
    except Exception as error:
        record["status"] = "FAIL"
        record["error"] = f"{type(error).__name__}: {error}"
        raise
