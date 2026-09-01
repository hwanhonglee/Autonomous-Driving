#!/usr/bin/env python3
"""Build deterministic expert routes on one already-running CARLA server."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

import networkx as nx

try:
    from inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest
except ModuleNotFoundError:
    from scripts.e2e.inventory_carla_training_maps import (
        DEFAULT_MANIFEST,
        ManifestError,
        load_manifest,
    )

try:
    from physical_straight_geometry import (
        PhysicalStraightGeometryError,
        SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
        analyze_serialized_physical_straight,
    )
except ModuleNotFoundError:
    from scripts.e2e.physical_straight_geometry import (
        PhysicalStraightGeometryError,
        SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
        analyze_serialized_physical_straight,
    )

try:
    from physical_turn_geometry import (
        PhysicalTurnGeometryError,
        SPEED_30KPH_TURN_CONTRACT_PROVENANCE,
        SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
        analyze_serialized_physical_turn,
    )
except ModuleNotFoundError:
    from scripts.e2e.physical_turn_geometry import (
        PhysicalTurnGeometryError,
        SPEED_30KPH_TURN_CONTRACT_PROVENANCE,
        SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
        analyze_serialized_physical_turn,
    )

try:
    from serialized_custom_turn_geometry import (
        SerializedCustomTurnGeometryError,
        analyze_serialized_custom_turn,
    )
except ModuleNotFoundError:
    from scripts.e2e.serialized_custom_turn_geometry import (
        SerializedCustomTurnGeometryError,
        analyze_serialized_custom_turn,
    )


SCENARIOS = ("lane_follow", "straight", "left", "right")
RECOVERABLE_TRACE_EXCEPTIONS = (
    KeyError,
    RuntimeError,
    nx.NetworkXNoPath,
    nx.NodeNotFound,
)
MAX_TRACE_ERROR_SAMPLES = 10
TURN_MANEUVER_OPTIONS = frozenset(
    {
        "LEFT",
        "RIGHT",
        "STRAIGHT",
        "CHANGELANELEFT",
        "CHANGELANERIGHT",
    }
)
TURN_GEOMETRY_ARGUMENTS = (
    "minimum_turn_arc_length_m",
    "maximum_turn_arc_length_m",
    "minimum_turn_heading_change_deg",
    "maximum_turn_heading_change_deg",
    "maximum_turn_heading_excess_deg",
    "turn_alignment_heading_margin_deg",
    "maximum_turn_command_lead_m",
    "maximum_turn_command_tail_m",
    "maximum_turn_p95_curvature_per_m",
)
SPAWN_POINT_ENDPOINT_SOURCE = "spawn_points"
GENERATED_WAYPOINT_ENDPOINT_SOURCE = "generated_waypoints"
PHYSICAL_STRAIGHT_PROFILES = ("speed_30kph",)
PHYSICAL_TURN_PROFILES = ("speed_30kph",)
ENDPOINT_JUNCTION_POLICIES = ("include", "exclude")
CANDIDATE_ENUMERATION_POLICIES = (
    "all_pairs",
    "directed_topology_straight_v1",
)
STRAIGHT_CAPACITY_PROFILES = ("town10hd_opt_30kph_compact_v1",)
TOWN10HD_OPT_STRAIGHT_CAPACITY_PROVENANCE: dict[str, Any] = {
    "profile_id": "town10hd_opt_30kph_compact_v1",
    "scope": "CARLA-only Town10HD_Opt 30 kph straight capacity screening",
    "measurement_source": (
        "measured 30 kph threshold-entry, exposure, and stopping distances"
    ),
    "validation_threshold_reuse": False,
    "minimum_route_length_m": 170.0,
    "threshold_entry_distance_m": 145.865,
    "minimum_exposure_distance_m": 7.5,
    "minimum_stop_distance_m": 14.169,
    "derived_required_distance_m": 167.534,
    "minimum_route_margin_m": 2.466,
    "derivation": "145.865 + 7.5 + 14.169 = 167.534; 170.0 - 167.534 = 2.466",
    "unchanged_verdicts": [
        "exact_serialized_physical_straight",
        "30_kph_speed_exposure",
        "AEB",
        "MRM",
    ],
    "postfilter_authority": (
        "exact serialized physical-straight geometry remains mandatory"
    ),
    "real_vehicle_ready": False,
}
# The in-tree Autoware CARLA bridge applies this in
# InitializeInterface._parse_spawn_point before actor creation.  The route keeps
# the raw road/base_link Z and must not apply the same clearance twice.
BRIDGE_SPAWN_Z_OFFSET_M = 2.0
BRIDGE_SPAWN_Z_OFFSET_SOURCE = (
    "src/universe/autoware_universe/simulator/autoware_carla_interface/src/"
    "autoware_carla_interface/carla_autoware.py::"
    "InitializeInterface._parse_spawn_point"
)


class CatalogError(RuntimeError):
    pass


def parse_integer_list(text: str) -> tuple[int, ...]:
    try:
        values = tuple(int(item.strip()) for item in text.split(",") if item.strip())
    except ValueError as error:
        raise CatalogError("seeds must be comma-separated integers") from error
    if not values or len(set(values)) != len(values):
        raise CatalogError("seeds must be a non-empty unique list")
    if any(value < 0 for value in values):
        raise CatalogError("seeds must be non-negative")
    return values


def parse_scenarios(text: str) -> tuple[str, ...]:
    values = tuple(item.strip() for item in text.split(",") if item.strip())
    if not values or len(set(values)) != len(values):
        raise CatalogError("scenarios must be a non-empty unique comma-separated list")
    unsupported = tuple(value for value in values if value not in SCENARIOS)
    if unsupported:
        raise CatalogError(
            f"scenarios must be a subset of {','.join(SCENARIOS)}; "
            f"unsupported={list(unsupported)}"
        )
    return values


def parse_excluded_spawn_indices(text: str) -> tuple[int, ...]:
    if not text.strip():
        return ()
    items = text.split(",")
    if any(not item.strip() for item in items):
        raise CatalogError("excluded spawn indices must be comma-separated integers")
    try:
        values = tuple(int(item.strip()) for item in items)
    except ValueError as error:
        raise CatalogError(
            "excluded spawn indices must be comma-separated integers"
        ) from error
    if len(set(values)) != len(values):
        raise CatalogError("excluded spawn indices must be unique")
    if any(value < 0 for value in values):
        raise CatalogError("excluded spawn indices must be non-negative")
    return values


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000"))
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--map-load-settle-sec",
        type=float,
        default=10.0,
        help=(
            "pre-read delay for the externally started map; also applied after an "
            "explicitly allowed map load"
        ),
    )
    parser.add_argument(
        "--allow-map-load",
        action="store_true",
        help=(
            "UNSAFE opt-in: permit client.load_world when the current map mismatches; "
            "some packaged/custom CARLA maps crash during client map loading"
        ),
    )
    parser.add_argument(
        "--active-server-profile",
        help="explicit profile id of the already-running external CARLA server",
    )
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument(
        "--scenarios",
        default=",".join(SCENARIOS),
        help="unique comma-separated subset of lane_follow,straight,left,right",
    )
    parser.add_argument("--pairs-per-seed", type=int, default=1)
    parser.add_argument("--min-distance", type=float, default=20.0)
    parser.add_argument("--max-distance", type=float, default=120.0)
    parser.add_argument("--preferred-distance", type=float, default=60.0)
    parser.add_argument("--sampling-resolution", type=float, default=1.0)
    parser.add_argument(
        "--endpoint-waypoint-spacing-m",
        type=float,
        help=(
            "opt in to deterministic CARLA generate_waypoints endpoints at this "
            "spacing; omitted keeps the existing recommended spawn-point endpoints"
        ),
    )
    parser.add_argument(
        "--endpoint-junction-policy",
        choices=ENDPOINT_JUNCTION_POLICIES,
        default="include",
        help="whether generated-waypoint endpoints may themselves be junction points",
    )
    parser.add_argument(
        "--candidate-enumeration-policy",
        choices=CANDIDATE_ENUMERATION_POLICIES,
        default="all_pairs",
        help="deterministic route-pair enumeration policy",
    )
    parser.add_argument(
        "--straight-capacity-profile",
        choices=STRAIGHT_CAPACITY_PROFILES,
        help="fail-closed map/profile-specific straight capacity contract",
    )
    parser.add_argument(
        "--physical-straight-profile",
        choices=PHYSICAL_STRAIGHT_PROFILES,
        help=(
            "opt in to a pinned physical-straight admission gate applied to the "
            "exact serialized straight route"
        ),
    )
    parser.add_argument(
        "--physical-turn-profile",
        choices=PHYSICAL_TURN_PROFILES,
        help=(
            "opt in to a pinned physical-turn admission gate applied to exact "
            "serialized left/right routes"
        ),
    )
    parser.add_argument(
        "--max-endpoint-offset",
        type=float,
        default=2.0,
        help="maximum planar XY offset between a spawn and its route endpoint",
    )
    parser.add_argument(
        "--initial-approach-distance-m",
        type=float,
        default=0.0,
        help=(
            "optional route-start geometry preflight distance; zero disables the "
            "preflight"
        ),
    )
    parser.add_argument(
        "--maximum-initial-lateral-deviation-m",
        type=float,
        help=(
            "maximum route-centerline deviation from the initial tangent inside "
            "--initial-approach-distance-m"
        ),
    )
    parser.add_argument(
        "--maximum-initial-heading-change-deg",
        type=float,
        help=(
            "maximum absolute route heading change inside "
            "--initial-approach-distance-m"
        ),
    )
    parser.add_argument("--minimum-turn-arc-length-m", type=float)
    parser.add_argument("--maximum-turn-arc-length-m", type=float)
    parser.add_argument("--minimum-turn-heading-change-deg", type=float)
    parser.add_argument("--maximum-turn-heading-change-deg", type=float)
    parser.add_argument("--maximum-turn-heading-excess-deg", type=float)
    parser.add_argument("--turn-alignment-heading-margin-deg", type=float)
    parser.add_argument("--maximum-turn-command-lead-m", type=float)
    parser.add_argument("--maximum-turn-command-tail-m", type=float)
    parser.add_argument("--maximum-turn-p95-curvature-per-m", type=float)
    parser.add_argument("--max-traces", type=int, default=5000)
    parser.add_argument(
        "--exclude-spawn-indices",
        default="",
        help="comma-separated CARLA spawn indices excluded as starts and goals",
    )
    args = parser.parse_args(argv)
    try:
        args.seeds = parse_integer_list(args.seeds)
        args.scenarios = parse_scenarios(args.scenarios)
        args.exclude_spawn_indices = parse_excluded_spawn_indices(
            args.exclude_spawn_indices
        )
    except CatalogError as error:
        parser.error(str(error))
    if not 0 < args.port <= 65535 or args.timeout <= 0.0:
        parser.error("port and timeout must be positive")
    if not math.isfinite(args.map_load_settle_sec) or args.map_load_settle_sec < 0.0:
        parser.error("map-load-settle-sec must be finite and non-negative")
    if args.pairs_per_seed <= 0 or args.max_traces <= 0:
        parser.error("pairs-per-seed and max-traces must be positive")
    if not 0.0 < args.min_distance <= args.max_distance:
        parser.error("distance range is invalid")
    if args.preferred_distance <= 0.0 or args.sampling_resolution <= 0.0:
        parser.error("preferred distance and sampling resolution must be positive")
    try:
        validate_endpoint_waypoint_spacing(args.endpoint_waypoint_spacing_m)
    except CatalogError as error:
        parser.error(str(error))
    if not math.isfinite(args.max_endpoint_offset) or args.max_endpoint_offset <= 0.0:
        parser.error("max-endpoint-offset must be positive and finite")
    try:
        initial_approach_contract(args)
        turn_geometry_contract(args)
        physical_straight_contract(args)
        physical_turn_contract(args)
        straight_capacity_contract(args)
    except CatalogError as error:
        parser.error(str(error))
    return args


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _load_runtime_modules():
    try:
        carla = importlib.import_module("carla")
        helper = importlib.import_module("prepare_carla_route")
    except ModuleNotFoundError as error:
        raise CatalogError(
            "CARLA Python API and agents are required; source scripts/e2e/env.sh or "
            "prepend the selected profile's Python API to PYTHONPATH"
        ) from error
    return carla, helper


def _map_by_id(manifest: Mapping[str, Any], map_id: str) -> Mapping[str, Any]:
    for entry in manifest["maps"]:
        if entry["id"] == map_id:
            return entry
    raise CatalogError(f"unknown map id: {map_id}")


def validate_active_server_profile(
    manifest: Mapping[str, Any],
    map_entry: Mapping[str, Any],
    active_server_profile: str | None,
) -> None:
    profiles = manifest["server_profiles"]
    if active_server_profile is not None and active_server_profile not in profiles:
        raise CatalogError(f"unknown active server profile: {active_server_profile}")
    required_profile = map_entry.get("server_profile")
    if active_server_profile is not None and active_server_profile != required_profile:
        raise CatalogError(
            f"map {map_entry['id']} requires server profile {required_profile!r}, "
            f"not declared active profile {active_server_profile!r}"
        )
    if (
        map_entry.get("status") == "source_editor_required"
        and active_server_profile != required_profile
    ):
        raise CatalogError(
            f"map {map_entry['id']} requires explicit --active-server-profile "
            f"{required_profile}"
        )


def _location_distance(first: Any, second: Any) -> float:
    return float(first.location.distance(second.location))


def validate_endpoint_waypoint_spacing(value: Any) -> float | None:
    """Validate the opt-in waypoint endpoint spacing for CLI and direct callers."""
    if value is None:
        return None
    try:
        spacing_m = float(value)
    except (TypeError, ValueError) as error:
        raise CatalogError(
            "endpoint waypoint spacing must be positive and finite"
        ) from error
    if not math.isfinite(spacing_m) or spacing_m <= 0.0:
        raise CatalogError("endpoint waypoint spacing must be positive and finite")
    return spacing_m


def _transform_sort_key(transform: Any) -> tuple[float, ...]:
    """Return a complete finite transform key suitable for stable endpoint order."""
    try:
        values = (
            float(transform.location.x),
            float(transform.location.y),
            float(transform.location.z),
            float(transform.rotation.roll),
            float(transform.rotation.pitch),
            float(transform.rotation.yaw),
        )
    except (AttributeError, TypeError, ValueError) as error:
        raise CatalogError("generated waypoint has an invalid transform") from error
    if not all(math.isfinite(value) for value in values):
        raise CatalogError("generated waypoint transform must be finite")
    return values


def _generated_waypoint_endpoint_records(
    carla_map: Any, spacing_m: float, junction_policy: str = "include"
) -> tuple[list[Any], list[Any], dict[str, Any]]:
    """Build a unique, deterministically ordered endpoint transform pool.

    ``carla.Map.generate_waypoints`` ordering is not part of CARLA's public
    contract.  Sorting the complete transform makes endpoint indices stable
    even if the API returns the same unique waypoints in a different order.
    CARLA can return the same exact transform more than once for topology
    boundaries. Because the planner consumes only the location, retaining
    those duplicates would create ambiguous aliases. Collapse exact duplicate
    full transforms deterministically and retain the raw/deduplicated counts in
    catalog provenance.
    """
    spacing_m = validate_endpoint_waypoint_spacing(spacing_m)
    assert spacing_m is not None
    if junction_policy not in ENDPOINT_JUNCTION_POLICIES:
        raise CatalogError(
            f"unsupported endpoint junction policy: {junction_policy!r}"
        )
    try:
        waypoints = list(carla_map.generate_waypoints(spacing_m))
    except (AttributeError, TypeError, RuntimeError, ValueError) as error:
        raise CatalogError(
            "CARLA failed to generate waypoint route endpoints"
        ) from error
    unique_by_transform: dict[tuple[float, ...], tuple[int, Any, Any]] = {}
    junction_count = 0
    junction_excluded_count = 0
    for api_index, waypoint in enumerate(waypoints):
        try:
            transform = waypoint.transform
        except AttributeError as error:
            raise CatalogError(
                f"generated waypoint {api_index} has no transform"
            ) from error
        is_junction = getattr(waypoint, "is_junction", None)
        if junction_policy == "exclude" and not isinstance(is_junction, bool):
            raise CatalogError(
                f"generated waypoint {api_index} lacks boolean is_junction"
            )
        if bool(is_junction):
            junction_count += 1
            if junction_policy == "exclude":
                junction_excluded_count += 1
                continue
        key = _transform_sort_key(transform)
        unique_by_transform.setdefault(key, (api_index, transform, waypoint))
    keyed = [
        (key, api_index, transform, waypoint)
        for key, (api_index, transform, waypoint) in unique_by_transform.items()
    ]
    if len(keyed) < 2:
        raise CatalogError("CARLA generated fewer than two waypoint route endpoints")
    keyed.sort(key=lambda item: item[0])
    return (
        [transform for _key, _api_index, transform, _waypoint in keyed],
        [waypoint for _key, _api_index, _transform, waypoint in keyed],
        {
            "api_count": len(waypoints),
            "junction_policy": junction_policy,
            "junction_waypoint_count": junction_count,
            "junction_excluded_count": junction_excluded_count,
            "eligible_api_count": len(waypoints) - junction_excluded_count,
            "duplicate_transform_count": (
                len(waypoints) - junction_excluded_count - len(keyed)
            ),
        },
    )


def _generated_waypoint_endpoint_pool(
    carla_map: Any, spacing_m: float, junction_policy: str = "include"
) -> tuple[list[Any], dict[str, Any]]:
    endpoints, _waypoints, provenance = _generated_waypoint_endpoint_records(
        carla_map, spacing_m, junction_policy
    )
    return endpoints, provenance


def generated_waypoint_endpoint_transforms(
    carla_map: Any, spacing_m: float, junction_policy: str = "include"
) -> list[Any]:
    endpoints, _provenance = _generated_waypoint_endpoint_pool(
        carla_map, spacing_m, junction_policy
    )
    return endpoints


def waypoint_spawn_height_contract(start_transform: Any) -> dict[str, Any]:
    """Describe the single bridge-owned road-clearance offset without applying it."""
    source_z_m = float(start_transform.location.z)
    if not math.isfinite(source_z_m):
        raise CatalogError("waypoint endpoint start Z must be finite")
    return {
        "endpoint_transform_z_m": source_z_m,
        "catalog_z_offset_m": 0.0,
        "bridge_z_offset_m": BRIDGE_SPAWN_Z_OFFSET_M,
        "actor_spawn_z_before_base_link_to_center_shift_m": (
            source_z_m + BRIDGE_SPAWN_Z_OFFSET_M
        ),
        "base_link_to_center_shift": (
            "bridge applies wheelbase/2 along the local longitudinal axis after "
            "the Z clearance; pitched transforms may also change actor Z"
        ),
        "offset_owner": "autoware_carla_interface_bridge",
        "bridge_source": BRIDGE_SPAWN_Z_OFFSET_SOURCE,
    }


def initial_approach_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return and validate the optional route-start geometry contract."""
    distance_m = float(getattr(args, "initial_approach_distance_m", 0.0))
    maximum_lateral = getattr(args, "maximum_initial_lateral_deviation_m", None)
    maximum_heading = getattr(args, "maximum_initial_heading_change_deg", None)
    if not math.isfinite(distance_m) or distance_m < 0.0:
        raise CatalogError("initial approach distance must be finite and non-negative")
    if distance_m == 0.0:
        if maximum_lateral is not None or maximum_heading is not None:
            raise CatalogError(
                "initial approach limits require a positive "
                "--initial-approach-distance-m"
            )
        return {
            "enabled": False,
            "distance_m": 0.0,
            "maximum_lateral_deviation_m": None,
            "maximum_heading_change_deg": None,
        }
    if maximum_lateral is None or maximum_heading is None:
        raise CatalogError(
            "a positive initial approach distance requires both lateral-deviation "
            "and heading-change limits"
        )
    maximum_lateral = float(maximum_lateral)
    maximum_heading = float(maximum_heading)
    if not math.isfinite(maximum_lateral) or maximum_lateral <= 0.0:
        raise CatalogError(
            "maximum initial lateral deviation must be positive and finite"
        )
    if (
        not math.isfinite(maximum_heading)
        or maximum_heading <= 0.0
        or maximum_heading > 180.0
    ):
        raise CatalogError(
            "maximum initial heading change must be finite and in (0, 180] degrees"
        )
    return {
        "enabled": True,
        "distance_m": distance_m,
        "maximum_lateral_deviation_m": maximum_lateral,
        "maximum_heading_change_deg": maximum_heading,
    }


def _normalize_angle_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def turn_geometry_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return the optional isolated-turn geometry contract."""
    values = {name: getattr(args, name, None) for name in TURN_GEOMETRY_ARGUMENTS}
    if all(value is None for value in values.values()):
        return {"enabled": False}
    missing = [name for name, value in values.items() if value is None]
    if missing:
        raise CatalogError(
            "turn geometry preflight requires every limit; missing "
            + ", ".join(sorted(missing))
        )
    numeric = {name: float(value) for name, value in values.items()}
    if any(not math.isfinite(value) or value <= 0.0 for value in numeric.values()):
        raise CatalogError("turn geometry limits must be positive and finite")
    if numeric["minimum_turn_arc_length_m"] >= numeric[
        "maximum_turn_arc_length_m"
    ]:
        raise CatalogError("minimum turn arc length must be below its maximum")
    if numeric["minimum_turn_heading_change_deg"] >= numeric[
        "maximum_turn_heading_change_deg"
    ]:
        raise CatalogError("minimum turn heading change must be below its maximum")
    if numeric["maximum_turn_heading_change_deg"] > 180.0:
        raise CatalogError("maximum turn heading change cannot exceed 180 degrees")
    if numeric["turn_alignment_heading_margin_deg"] >= numeric[
        "minimum_turn_heading_change_deg"
    ]:
        raise CatalogError(
            "turn alignment heading margin must be below the minimum heading change"
        )
    return {
        "enabled": True,
        "require_single_directional_block": True,
        "forbid_additional_maneuvers": True,
        "minimum_arc_length_m": numeric["minimum_turn_arc_length_m"],
        "maximum_arc_length_m": numeric["maximum_turn_arc_length_m"],
        "minimum_heading_change_deg": numeric[
            "minimum_turn_heading_change_deg"
        ],
        "maximum_heading_change_deg": numeric[
            "maximum_turn_heading_change_deg"
        ],
        "maximum_heading_excess_deg": numeric["maximum_turn_heading_excess_deg"],
        "alignment_heading_margin_deg": numeric[
            "turn_alignment_heading_margin_deg"
        ],
        "maximum_command_lead_m": numeric["maximum_turn_command_lead_m"],
        "maximum_command_tail_m": numeric["maximum_turn_command_tail_m"],
        "curvature_percentile": 95.0,
        "maximum_p95_abs_curvature_per_m": numeric[
            "maximum_turn_p95_curvature_per_m"
        ],
    }


def physical_straight_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return the explicit serialized-route geometry admission contract."""
    profile = getattr(args, "physical_straight_profile", None)
    if profile is None:
        return {"enabled": False}
    if profile not in PHYSICAL_STRAIGHT_PROFILES:
        raise CatalogError(f"unsupported physical-straight profile: {profile!r}")
    if tuple(getattr(args, "scenarios", SCENARIOS)) != ("straight",):
        raise CatalogError(
            "physical-straight profile requires --scenarios straight"
        )
    return {
        "enabled": True,
        "profile_id": profile,
        "measurement_source": "exact_serialized_route_with_terminal_goal",
        "admission_policy": "reject_before_accepted_pair_quota",
        "limits": dict(SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT),
    }


def straight_capacity_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Validate the sole map/profile-specific compact straight generation mode."""
    profile = getattr(args, "straight_capacity_profile", None)
    junction_policy = getattr(args, "endpoint_junction_policy", "include")
    enumeration_policy = getattr(args, "candidate_enumeration_policy", "all_pairs")
    if junction_policy not in ENDPOINT_JUNCTION_POLICIES:
        raise CatalogError(f"unsupported endpoint junction policy: {junction_policy!r}")
    if enumeration_policy not in CANDIDATE_ENUMERATION_POLICIES:
        raise CatalogError(
            f"unsupported candidate enumeration policy: {enumeration_policy!r}"
        )
    if profile is None:
        if junction_policy != "include" or enumeration_policy != "all_pairs":
            raise CatalogError(
                "non-default endpoint/candidate policy requires an explicit "
                "straight-capacity profile"
            )
        return {"enabled": False}
    if profile not in STRAIGHT_CAPACITY_PROFILES:
        raise CatalogError(f"unsupported straight-capacity profile: {profile!r}")

    expected_integers = {
        "pairs_per_seed": 8,
        "max_traces": 20000,
    }
    expected_numbers = {
        "min_distance": 170.0,
        "max_distance": 182.0,
        "preferred_distance": 172.0,
        "sampling_resolution": 1.0,
        "endpoint_waypoint_spacing_m": 0.5,
        "max_endpoint_offset": 2.0,
    }
    mismatches = []
    for field, expected in expected_integers.items():
        value = getattr(args, field, None)
        if isinstance(value, bool) or not isinstance(value, int) or value != expected:
            mismatches.append(f"{field}={value!r} (expected {expected!r})")
    for field, expected in expected_numbers.items():
        value = getattr(args, field, None)
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(float(value))
            or not math.isclose(
                float(value), expected, rel_tol=0.0, abs_tol=1.0e-9
            )
        ):
            mismatches.append(f"{field}={value!r} (expected {expected!r})")
    expected_identity = {
        "map_id": "town10hd_opt",
        "scenarios": ("straight",),
        "seeds": (0,),
        "physical_straight_profile": "speed_30kph",
        "weather": "ClearNoon",
        "endpoint_junction_policy": "exclude",
        "candidate_enumeration_policy": "directed_topology_straight_v1",
    }
    for field, expected in expected_identity.items():
        value = getattr(args, field, None)
        if value != expected:
            mismatches.append(f"{field}={value!r} (expected {expected!r})")
    if mismatches:
        raise CatalogError(
            f"straight-capacity profile {profile!r} contract mismatch: "
            + "; ".join(mismatches)
        )
    return {
        "enabled": True,
        "profile_id": profile,
        "map_id": "town10hd_opt",
        "scenario": "straight",
        "endpoint_waypoint_spacing_m": 0.5,
        "endpoint_junction_policy": "exclude",
        "candidate_enumeration_policy": "directed_topology_straight_v1",
        "admission_policy": "prefilter_then_exact_serialized_physical_postfilter",
        "provenance": dict(TOWN10HD_OPT_STRAIGHT_CAPACITY_PROVENANCE),
    }


def physical_turn_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return the explicit serialized packaged-Town turn admission contract."""
    profile = getattr(args, "physical_turn_profile", None)
    if profile is None:
        return {"enabled": False}
    if profile not in PHYSICAL_TURN_PROFILES:
        raise CatalogError(f"unsupported physical-turn profile: {profile!r}")
    if tuple(getattr(args, "scenarios", SCENARIOS)) != ("left", "right"):
        raise CatalogError(
            "physical-turn profile requires --scenarios left,right"
        )
    if initial_approach_contract(args)["enabled"] or turn_geometry_contract(args)[
        "enabled"
    ]:
        raise CatalogError(
            "physical-turn profile cannot be combined with custom-map turn gates"
        )
    return {
        "enabled": True,
        "profile_id": profile,
        "applicability": "packaged_town_only",
        "measurement_source": "exact_serialized_3d_route_with_terminal_goal",
        "admission_policy": "reject_before_accepted_pair_quota",
        "limits": dict(SPEED_30KPH_TURN_GEOMETRY_CONTRACT),
        "provenance": dict(SPEED_30KPH_TURN_CONTRACT_PROVENANCE),
    }


def _road_option_name(option: Any) -> str:
    return str(getattr(option, "name", option))


def _linear_percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _first_progress_crossing(
    samples: Sequence[tuple[float, float]], threshold: float
) -> float | None:
    for (start_distance, start_value), (end_distance, end_value) in zip(
        samples, samples[1:]
    ):
        if start_value >= threshold:
            return start_distance
        if end_value < threshold:
            continue
        if end_value <= start_value:
            return end_distance
        ratio = (threshold - start_value) / (end_value - start_value)
        return start_distance + ratio * (end_distance - start_distance)
    if samples and samples[-1][1] >= threshold:
        return samples[-1][0]
    return None


def _turn_block_geometry(
    route: Sequence[Any], start_index: int, end_index: int
) -> dict[str, Any]:
    distances = [0.0]
    for index in range(1, len(route)):
        distances.append(
            distances[-1]
            + float(
                route[index - 1][0].transform.location.distance(
                    route[index][0].transform.location
                )
            )
        )
    support_start = max(0, start_index - 1)
    support_end = min(len(route) - 1, end_index + 1)
    block_start = (
        0.5 * (distances[start_index - 1] + distances[start_index])
        if start_index
        else distances[start_index]
    )
    block_end = (
        0.5 * (distances[end_index] + distances[end_index + 1])
        if end_index + 1 < len(route)
        else distances[end_index]
    )
    yaws = [
        float(route[index][0].transform.rotation.yaw)
        for index in range(support_start, support_end + 1)
    ]
    unwrapped = [yaws[0]]
    for yaw in yaws[1:]:
        unwrapped.append(
            unwrapped[-1] + _normalize_angle_degrees(yaw - unwrapped[-1])
        )
    net_heading = unwrapped[-1] - unwrapped[0]
    absolute_net_heading = abs(net_heading)
    cumulative_heading = sum(
        abs(unwrapped[index] - unwrapped[index - 1])
        for index in range(1, len(unwrapped))
    )
    direction = 1.0 if net_heading >= 0.0 else -1.0
    progress = [
        (
            distances[support_start + index],
            max(0.0, direction * (yaw - unwrapped[0])),
        )
        for index, yaw in enumerate(unwrapped)
    ]
    curvatures = []
    for index in range(support_start + 1, support_end + 1):
        segment_length = distances[index] - distances[index - 1]
        if segment_length <= 1.0e-6:
            continue
        yaw_delta = _normalize_angle_degrees(
            float(route[index][0].transform.rotation.yaw)
            - float(route[index - 1][0].transform.rotation.yaw)
        )
        curvatures.append(abs(math.radians(yaw_delta)) / segment_length)
    arc_length = block_end - block_start
    return {
        "start_index": start_index,
        "end_index": end_index,
        "command_arc_length_m": arc_length,
        "absolute_net_heading_change_deg": absolute_net_heading,
        "cumulative_absolute_heading_change_deg": cumulative_heading,
        "heading_excess_deg": max(0.0, cumulative_heading - absolute_net_heading),
        "mean_absolute_curvature_per_m": (
            math.radians(cumulative_heading) / arc_length
            if arc_length > 0.0
            else math.inf
        ),
        "p95_absolute_curvature_per_m": _linear_percentile(curvatures, 95.0),
        "block_start_distance_m": block_start,
        "block_end_distance_m": block_end,
        "_heading_progress": progress,
    }


def analyze_turn_geometry(
    route: Sequence[Any], scenario: str, contract: Mapping[str, Any]
) -> dict[str, Any]:
    """Verify that a turn trial contains one compact, command-aligned turn."""
    if not contract.get("enabled"):
        raise CatalogError("turn geometry analysis requires an enabled contract")
    if scenario not in ("left", "right"):
        raise CatalogError("turn geometry analysis requires left or right scenario")
    if len(route) < 2:
        raise CatalogError("turn geometry analysis requires at least two points")

    target = scenario.upper()
    option_names = [_road_option_name(option) for _waypoint, option in route]
    indices = [index for index, option in enumerate(option_names) if option == target]
    blocks: list[list[int]] = []
    for index in indices:
        if not blocks or index != blocks[-1][-1] + 1:
            blocks.append([index])
        else:
            blocks[-1].append(index)
    additional = sorted((set(option_names) & TURN_MANEUVER_OPTIONS) - {target})
    reasons = []
    if len(blocks) != 1:
        reasons.append(
            f"expected exactly one contiguous {target} block, found {len(blocks)}"
        )
    if additional:
        reasons.append("additional maneuver commands present: " + ", ".join(additional))

    block_metrics = [
        _turn_block_geometry(route, block[0], block[-1]) for block in blocks
    ]
    selected = block_metrics[0] if len(block_metrics) == 1 else None
    if selected is not None:
        margin = float(contract["alignment_heading_margin_deg"])
        net_heading = float(selected["absolute_net_heading_change_deg"])
        lead_crossing = _first_progress_crossing(
            selected["_heading_progress"], margin
        )
        tail_crossing = _first_progress_crossing(
            selected["_heading_progress"], max(0.0, net_heading - margin)
        )
        command_lead = (
            math.inf
            if lead_crossing is None
            else max(
                0.0, lead_crossing - float(selected["block_start_distance_m"])
            )
        )
        command_tail = (
            math.inf
            if tail_crossing is None
            else max(0.0, float(selected["block_end_distance_m"]) - tail_crossing)
        )
        selected["command_lead_distance_m"] = command_lead
        selected["command_tail_distance_m"] = command_tail

        checks = (
            (
                float(selected["command_arc_length_m"])
                < float(contract["minimum_arc_length_m"]),
                "turn command arc length is below the minimum",
            ),
            (
                float(selected["command_arc_length_m"])
                > float(contract["maximum_arc_length_m"]),
                "turn command arc length exceeds the maximum",
            ),
            (
                net_heading < float(contract["minimum_heading_change_deg"]),
                "turn heading change is below the minimum",
            ),
            (
                net_heading > float(contract["maximum_heading_change_deg"]),
                "turn heading change exceeds the maximum",
            ),
            (
                float(selected["heading_excess_deg"])
                > float(contract["maximum_heading_excess_deg"]),
                "turn heading excess exceeds the maximum",
            ),
            (
                command_lead > float(contract["maximum_command_lead_m"]),
                "turn command lead distance exceeds the maximum",
            ),
            (
                command_tail > float(contract["maximum_command_tail_m"]),
                "turn command tail distance exceeds the maximum",
            ),
            (
                float(selected["p95_absolute_curvature_per_m"])
                > float(contract["maximum_p95_abs_curvature_per_m"]),
                "turn p95 absolute curvature exceeds the maximum",
            ),
        )
        reasons.extend(message for failed, message in checks if failed)

    for metrics in block_metrics:
        metrics.pop("_heading_progress", None)
    return {
        "status": "FAIL" if reasons else "PASS",
        "scenario": scenario,
        "directional_command": target,
        "directional_block_count": len(blocks),
        "additional_maneuver_commands": additional,
        "selected_block": selected,
        "directional_blocks": block_metrics,
        "limits": dict(contract),
        "failure_reasons": reasons,
    }


def analyze_initial_approach(
    route: Sequence[Any], contract: Mapping[str, Any]
) -> dict[str, Any]:
    """Measure route-start curvature without relying on RoadOption labels.

    CARLA marks non-junction bends as ``LANEFOLLOW``.  A route can therefore
    satisfy a maneuver-command contract while presenting an immediate sharp
    bend under command 3.  This preflight measures the actual centerline against
    its starting tangent, including an interpolated sample at the exact distance
    boundary.
    """
    if not contract.get("enabled"):
        raise CatalogError("initial approach analysis requires an enabled contract")
    if len(route) < 2:
        raise CatalogError("initial approach analysis requires at least two points")

    distance_limit = float(contract["distance_m"])
    first_transform = route[0][0].transform
    first_location = first_transform.location
    first_yaw_deg = float(first_transform.rotation.yaw)
    first_yaw_rad = math.radians(first_yaw_deg)
    normal_x = -math.sin(first_yaw_rad)
    normal_y = math.cos(first_yaw_rad)
    maximum_lateral = 0.0
    maximum_heading = 0.0
    covered_distance = 0.0

    def measure(x: float, y: float, yaw_deg: float) -> None:
        nonlocal maximum_lateral, maximum_heading
        dx = x - float(first_location.x)
        dy = y - float(first_location.y)
        maximum_lateral = max(maximum_lateral, abs(dx * normal_x + dy * normal_y))
        maximum_heading = max(
            maximum_heading,
            abs(_normalize_angle_degrees(yaw_deg - first_yaw_deg)),
        )

    previous = first_transform
    for waypoint, _option in route[1:]:
        current = waypoint.transform
        segment_length = float(previous.location.distance(current.location))
        if segment_length <= 1.0e-9:
            previous = current
            continue
        remaining = distance_limit - covered_distance
        if segment_length >= remaining:
            ratio = max(0.0, min(1.0, remaining / segment_length))
            yaw_delta = _normalize_angle_degrees(
                float(current.rotation.yaw) - float(previous.rotation.yaw)
            )
            measure(
                float(previous.location.x)
                + ratio * (float(current.location.x) - float(previous.location.x)),
                float(previous.location.y)
                + ratio * (float(current.location.y) - float(previous.location.y)),
                float(previous.rotation.yaw) + ratio * yaw_delta,
            )
            covered_distance = distance_limit
            break
        covered_distance += segment_length
        measure(
            float(current.location.x),
            float(current.location.y),
            float(current.rotation.yaw),
        )
        previous = current

    reasons = []
    if covered_distance < distance_limit - 1.0e-6:
        reasons.append(
            f"route covers only {covered_distance:.3f} m of the required "
            f"{distance_limit:.3f} m approach"
        )
    if maximum_lateral > float(contract["maximum_lateral_deviation_m"]):
        reasons.append(
            f"initial lateral deviation {maximum_lateral:.3f} m exceeds "
            f"{float(contract['maximum_lateral_deviation_m']):.3f} m"
        )
    if maximum_heading > float(contract["maximum_heading_change_deg"]):
        reasons.append(
            f"initial heading change {maximum_heading:.3f} deg exceeds "
            f"{float(contract['maximum_heading_change_deg']):.3f} deg"
        )
    return {
        "status": "FAIL" if reasons else "PASS",
        "distance_m": distance_limit,
        "covered_distance_m": covered_distance,
        "maximum_lateral_deviation_m": maximum_lateral,
        "maximum_heading_change_deg": maximum_heading,
        "limits": {
            "maximum_lateral_deviation_m": float(
                contract["maximum_lateral_deviation_m"]
            ),
            "maximum_heading_change_deg": float(
                contract["maximum_heading_change_deg"]
            ),
        },
        "failure_reasons": reasons,
    }


def deterministic_pairs(
    spawn_points: Sequence[Any],
    seed: int,
    minimum_distance: float,
    maximum_distance: float,
    preferred_distance: float,
    sampling_resolution: float,
    excluded_indices: Sequence[int] = (),
) -> list[tuple[int, int]]:
    excluded = frozenset(excluded_indices)
    ranked = []
    for start_index, start in enumerate(spawn_points):
        if start_index in excluded:
            continue
        for goal_index, goal in enumerate(spawn_points):
            if start_index == goal_index or goal_index in excluded:
                continue
            distance = _location_distance(start, goal)
            if not minimum_distance * 0.6 <= distance <= maximum_distance * 1.2:
                continue
            bucket = int(abs(distance - preferred_distance) / sampling_resolution)
            token = f"{seed}:{start_index}:{goal_index}".encode("ascii")
            tie_break = hashlib.sha256(token).digest()
            ranked.append((bucket, tie_break, start_index, goal_index))
    ranked.sort()
    return [(start, goal) for _, _, start, goal in ranked]


def _heading_difference_deg(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def deterministic_directed_topology_straight_pairs(
    endpoints: Sequence[Any],
    planner: Any,
    seed: int,
    minimum_distance: float,
    maximum_distance: float,
    preferred_distance: float,
    sampling_resolution: float,
    physical_straight_limits: Mapping[str, Any],
) -> tuple[list[tuple[int, int]], dict[str, Any]]:
    """Prefilter physically possible straight pairs on the directed road graph.

    Chord and endpoint-heading checks are mathematical necessary conditions of
    the unchanged physical-straight contract. Directed graph reachability then
    removes reverse-lane pairs without calling ``trace_route``. This is only an
    enumeration optimization: every retained pair still passes through the
    exact serialized-route distance, road-option, and physical postfilters.
    """
    graph = getattr(planner, "_graph", None)
    localize = getattr(planner, "_localize", None)
    if (
        graph is None
        or not callable(localize)
        or not hasattr(graph, "is_directed")
        or not graph.is_directed()
    ):
        raise CatalogError(
            "directed topology enumeration requires the GlobalRoutePlanner "
            "directed graph and localization API"
        )
    try:
        maximum_arc_ratio = float(
            physical_straight_limits["maximum_arc_to_direct_ratio"]
        )
        maximum_heading = float(
            physical_straight_limits["maximum_endpoint_tangent_to_chord_deg"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise CatalogError(
            "directed topology enumeration lacks physical-straight bounds"
        ) from error
    if (
        not math.isfinite(maximum_arc_ratio)
        or maximum_arc_ratio < 1.0
        or not math.isfinite(maximum_heading)
        or maximum_heading <= 0.0
    ):
        raise CatalogError(
            "directed topology enumeration has invalid physical-straight bounds"
        )

    direct_minimum = minimum_distance / maximum_arc_ratio
    cell_size = maximum_distance
    cells: dict[tuple[int, int], list[int]] = {}
    xy_yaw: list[tuple[float, float, float]] = []
    for index, endpoint in enumerate(endpoints):
        try:
            x = float(endpoint.location.x)
            y = float(endpoint.location.y)
            yaw = float(endpoint.rotation.yaw)
        except (AttributeError, TypeError, ValueError) as error:
            raise CatalogError(
                f"generated endpoint {index} lacks finite x/y/yaw"
            ) from error
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise CatalogError(f"generated endpoint {index} lacks finite x/y/yaw")
        xy_yaw.append((x, y, yaw))
        cell = (math.floor(x / cell_size), math.floor(y / cell_size))
        cells.setdefault(cell, []).append(index)

    geometric_by_start: dict[int, list[tuple[int, float]]] = {}
    geometric_count = 0
    for start_index, (start_x, start_y, start_yaw) in enumerate(xy_yaw):
        start_cell = (
            math.floor(start_x / cell_size),
            math.floor(start_y / cell_size),
        )
        for cell_x in range(start_cell[0] - 1, start_cell[0] + 2):
            for cell_y in range(start_cell[1] - 1, start_cell[1] + 2):
                for goal_index in cells.get((cell_x, cell_y), ()):
                    if goal_index == start_index:
                        continue
                    goal_x, goal_y, goal_yaw = xy_yaw[goal_index]
                    dx = goal_x - start_x
                    dy = goal_y - start_y
                    direct_distance = math.hypot(dx, dy)
                    if not (
                        direct_minimum - 1.0e-9
                        <= direct_distance
                        <= maximum_distance + 1.0e-9
                    ):
                        continue
                    chord_heading = math.degrees(math.atan2(dy, dx))
                    if (
                        _heading_difference_deg(start_yaw, chord_heading)
                        > maximum_heading + 1.0e-9
                        or _heading_difference_deg(goal_yaw, chord_heading)
                        > maximum_heading + 1.0e-9
                    ):
                        continue
                    geometric_by_start.setdefault(start_index, []).append(
                        (goal_index, direct_distance)
                    )
                    geometric_count += 1

    localized: dict[int, tuple[Any, Any]] = {}

    def localized_edge(index: int) -> tuple[Any, Any]:
        if index not in localized:
            try:
                edge = localize(endpoints[index].location)
            except Exception as error:
                raise CatalogError(
                    f"directed topology localization failed for endpoint {index}"
                ) from error
            if not isinstance(edge, tuple) or len(edge) != 2:
                raise CatalogError(
                    f"directed topology localization returned no edge for endpoint {index}"
                )
            if edge[0] not in graph or edge[1] not in graph:
                raise CatalogError(
                    f"directed topology localization returned an unknown edge for "
                    f"endpoint {index}"
                )
            localized[index] = edge
        return localized[index]

    ranked: list[tuple[int, bytes, int, int]] = []
    for start_index, candidates in geometric_by_start.items():
        source = localized_edge(start_index)[0]
        reachable = nx.descendants(graph, source)
        reachable.add(source)
        for goal_index, direct_distance in candidates:
            target = localized_edge(goal_index)[0]
            if target not in reachable:
                continue
            bucket = int(
                abs(direct_distance - preferred_distance) / sampling_resolution
            )
            token = f"{seed}:{start_index}:{goal_index}".encode("ascii")
            ranked.append(
                (bucket, hashlib.sha256(token).digest(), start_index, goal_index)
            )
    ranked.sort()
    pairs = [(start, goal) for _bucket, _tie, start, goal in ranked]
    return pairs, {
        "policy": "directed_topology_straight_v1",
        "input_endpoint_count": len(endpoints),
        "planar_chord_heading_candidate_count": geometric_count,
        "directed_reachable_candidate_count": len(pairs),
        "directed_graph_node_count": graph.number_of_nodes(),
        "directed_graph_edge_count": graph.number_of_edges(),
        "necessary_chord_minimum_m": direct_minimum,
        "necessary_chord_maximum_m": maximum_distance,
        "maximum_endpoint_heading_to_chord_deg": maximum_heading,
        "ranking": "preferred_chord_distance_bucket_then_seeded_sha256",
        "postfilter_authority": (
            "exact serialized route distance and physical-straight analysis"
        ),
        "real_vehicle_ready": False,
    }


def validate_excluded_spawn_indices(
    excluded_indices: Sequence[int], spawn_point_count: int
) -> tuple[int, ...]:
    excluded = tuple(excluded_indices)
    if len(set(excluded)) != len(excluded):
        raise CatalogError("excluded spawn indices must be unique")
    if any(index < 0 for index in excluded):
        raise CatalogError("excluded spawn indices must be non-negative")
    out_of_range = tuple(index for index in excluded if index >= spawn_point_count)
    if out_of_range:
        raise CatalogError(
            "excluded spawn indices are out of range for "
            f"{spawn_point_count} spawn points: {list(out_of_range)}"
        )
    if spawn_point_count - len(excluded) < 2:
        raise CatalogError(
            "exclude-spawn-indices leaves fewer than two start/goal candidates"
        )
    return excluded


def _reset_planner(planner: Any, helper: Any) -> None:
    if hasattr(planner, "_intersection_end_node"):
        planner._intersection_end_node = -1
    if hasattr(planner, "_previous_decision") and hasattr(helper, "RoadOption"):
        planner._previous_decision = helper.RoadOption.VOID


def trace_route_candidate(planner: Any, start: Any, goal: Any) -> tuple[Any, Exception | None]:
    """Return one route or a recoverable pair-specific planner error."""
    try:
        return planner.trace_route(start, goal), None
    except RECOVERABLE_TRACE_EXCEPTIONS as error:
        return None, error


def _trace_error_sample(
    start_index: int,
    goal_index: int,
    error: Exception,
    endpoint_source: str = SPAWN_POINT_ENDPOINT_SOURCE,
) -> dict[str, Any]:
    sample = {
        "error_type": type(error).__name__,
        "message": str(error)[:240],
    }
    if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE:
        return {
            "start_spawn_index": start_index,
            "goal_spawn_index": goal_index,
            **sample,
        }
    sample.update(
        {
            "endpoint_source": endpoint_source,
            "start_endpoint_index": start_index,
            "goal_endpoint_index": goal_index,
        }
    )
    return sample


def serialized_route_length(route_points: Sequence[Mapping[str, Any]]) -> float:
    """Return the exact serialized terminal distance used in route evidence."""
    if not route_points or not isinstance(route_points[-1], Mapping):
        raise CatalogError("serialized route has no terminal point")
    value = route_points[-1].get("distance_m")
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise CatalogError("serialized route terminal distance_m is invalid")
    return float(value)


def _physical_straight_rejection_sample(
    start_index: int,
    goal_index: int,
    result: Mapping[str, Any],
    endpoint_source: str,
) -> dict[str, Any]:
    def finite_metric(name: str) -> float | None:
        value = result.get(name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            return None
        return float(value)

    sample = {
        "failure_reasons": list(result.get("failure_reasons", [])),
        "route_length_m": finite_metric("route_length_m"),
        "direct_distance_m": finite_metric("direct_distance_m"),
        "arc_to_direct_ratio": finite_metric("arc_to_direct_ratio"),
        "maximum_chord_deviation_m": finite_metric(
            "maximum_chord_deviation_m"
        ),
        "maximum_endpoint_tangent_to_chord_deg": finite_metric(
            "maximum_endpoint_tangent_to_chord_deg"
        ),
        "absolute_net_heading_change_deg": finite_metric(
            "absolute_net_heading_change_deg"
        ),
        "cumulative_absolute_heading_change_deg": finite_metric(
            "cumulative_absolute_heading_change_deg"
        ),
        "p95_absolute_curvature_per_m": finite_metric(
            "p95_absolute_curvature_per_m"
        ),
        "maximum_absolute_curvature_per_m": finite_metric(
            "maximum_absolute_curvature_per_m"
        ),
    }
    if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE:
        return {
            "start_spawn_index": start_index,
            "goal_spawn_index": goal_index,
            **sample,
        }
    return {
        "endpoint_source": endpoint_source,
        "start_endpoint_index": start_index,
        "goal_endpoint_index": goal_index,
        **sample,
    }


def _physical_turn_rejection_sample(
    start_index: int,
    goal_index: int,
    result: Mapping[str, Any],
    endpoint_source: str,
) -> dict[str, Any]:
    def finite_or_none(value: Any) -> float | None:
        return (
            float(value)
            if isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            else None
        )

    block = result.get("selected_block")
    block_metrics = (
        {
            field: finite_or_none(block.get(field))
            for field in (
                "route_lead_distance_m",
                "route_tail_distance_m",
                "command_arc_length_m",
                "signed_net_heading_change_deg",
                "heading_excess_deg",
                "command_lead_distance_m",
                "command_tail_distance_m",
                "p95_absolute_curvature_per_m",
                "maximum_absolute_curvature_per_m",
            )
        }
        if isinstance(block, Mapping)
        else None
    )
    initial = result.get("initial_approach")
    initial_metrics = (
        {
            field: finite_or_none(initial.get(field))
            for field in (
                "covered_distance_m",
                "maximum_lateral_deviation_m",
                "maximum_heading_change_deg",
            )
        }
        if isinstance(initial, Mapping)
        else None
    )
    grade = result.get("longitudinal_grade")
    grade_metrics = (
        {
            field: finite_or_none(grade.get(field))
            for field in (
                "window_length_m",
                "maximum_absolute_grade_ratio",
                "maximum_absolute_grade_percent",
                "maximum_allowed_absolute_grade_ratio",
            )
        }
        if isinstance(grade, Mapping)
        else None
    )
    sample = {
        "scenario": result.get("scenario"),
        "failure_reasons": list(result.get("failure_reasons", [])),
        "route_length_m": finite_or_none(result.get("route_length_m")),
        "directional_block_count": result.get("directional_block_count"),
        "additional_maneuver_commands": list(
            result.get("additional_maneuver_commands", [])
        ),
        "initial_approach": initial_metrics,
        "longitudinal_grade": grade_metrics,
        "selected_block": block_metrics,
    }
    if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE:
        return {
            "start_spawn_index": start_index,
            "goal_spawn_index": goal_index,
            **sample,
        }
    return {
        "endpoint_source": endpoint_source,
        "start_endpoint_index": start_index,
        "goal_endpoint_index": goal_index,
        **sample,
    }


def _physical_turn_error_rejection_sample(
    start_index: int,
    goal_index: int,
    scenario: str,
    route_length_m: float,
    error: PhysicalTurnGeometryError,
    endpoint_source: str,
) -> dict[str, Any]:
    sample = {
        "scenario": scenario,
        "failure_reasons": [str(error)],
        "route_length_m": route_length_m,
        "directional_block_count": None,
        "additional_maneuver_commands": [],
        "initial_approach": None,
        "longitudinal_grade": None,
        "selected_block": None,
        "analysis_error": error.evidence(),
    }
    if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE:
        return {
            "start_spawn_index": start_index,
            "goal_spawn_index": goal_index,
            **sample,
        }
    return {
        "endpoint_source": endpoint_source,
        "start_endpoint_index": start_index,
        "goal_endpoint_index": goal_index,
        **sample,
    }


def _normalized_goal_metadata(
    helper: Any,
    goal_transform: Any,
    route: Sequence[Any],
    route_points: Sequence[Mapping[str, Any]],
    endpoint_source: str,
    goal_index: int,
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Normalize runtime/serialized goal Z while retaining the raw endpoint."""
    if not route or not route_points:
        raise CatalogError("goal normalization requires a non-empty route")
    original_carla = dict(helper.transform_dict(goal_transform))
    original_ros = dict(helper.ros_pose_dict(goal_transform))
    try:
        original_z = float(goal_transform.location.z)
        road_z = float(route[-1][0].transform.location.z)
    except (AttributeError, TypeError, ValueError) as error:
        raise CatalogError("goal endpoint or road waypoint Z is invalid") from error
    if not math.isfinite(original_z) or not math.isfinite(road_z):
        raise CatalogError("goal endpoint and road waypoint Z must be finite")
    original_carla.setdefault("z", original_z)
    original_ros.setdefault("z", original_z)
    normalized_route = [dict(point) for point in route_points]
    normalized_route[-1]["z"] = road_z
    normalized_carla = {**original_carla, "z": road_z}
    normalized_ros = {**original_ros, "z": road_z}
    provenance = {
        "endpoint_source": endpoint_source,
        "endpoint_index": int(goal_index),
        "original_goal_carla_transform": original_carla,
        "original_goal_ros_pose": original_ros,
        "terminal_z_normalization": {
            "policy": "last_road_waypoint_z",
            "original_endpoint_z_m": original_z,
            "last_road_waypoint_z_m": road_z,
            "runtime_goal_z_m": road_z,
            "serialized_terminal_z_m": road_z,
            "applied_offset_m": road_z - original_z,
        },
    }
    return normalized_route, normalized_carla, normalized_ros, provenance


def _route_payload(
    helper: Any,
    map_entry: Mapping[str, Any],
    weather: str,
    scenario: str,
    sampling_resolution: float,
    start_index: int,
    goal_index: int,
    endpoints: Sequence[Any],
    route: Sequence[Any],
    initial_approach: Mapping[str, Any] | None = None,
    turn_geometry: Mapping[str, Any] | None = None,
    *,
    endpoint_source: str = SPAWN_POINT_ENDPOINT_SOURCE,
    endpoint_waypoint_spacing_m: float | None = None,
    serialized_route: Sequence[Mapping[str, Any]] | None = None,
    physical_straight: Mapping[str, Any] | None = None,
    physical_turn: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if endpoint_source == GENERATED_WAYPOINT_ENDPOINT_SOURCE:
        endpoint_waypoint_spacing_m = validate_endpoint_waypoint_spacing(
            endpoint_waypoint_spacing_m
        )
        if endpoint_waypoint_spacing_m is None:
            raise CatalogError(
                "generated waypoint endpoint payload requires its spacing"
            )
    elif endpoint_source != SPAWN_POINT_ENDPOINT_SOURCE:
        raise CatalogError(f"unsupported endpoint source: {endpoint_source!r}")
    start_transform = endpoints[start_index]
    goal_transform = endpoints[goal_index]
    raw_route_points = (
        list(serialized_route)
        if serialized_route is not None
        else helper.serialize_route(route, goal_transform)
    )
    (
        route_points,
        goal_carla_transform,
        goal_ros_pose,
        goal_endpoint_provenance,
    ) = _normalized_goal_metadata(
        helper,
        goal_transform,
        route,
        raw_route_points,
        endpoint_source,
        goal_index,
    )
    option_counts: dict[str, int] = {}
    for point in route_points:
        option = point["road_option"]
        option_counts[option] = option_counts.get(option, 0) + 1
    payload = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "spawn_point_reference": "base_link",
        "town": map_entry["canonical_name"],
        "weather": weather,
        "scenario": scenario,
        "sampling_resolution_m": sampling_resolution,
        "route_length_m": route_points[-1]["distance_m"],
        **(
            {
                "start_spawn_index": start_index,
                "goal_spawn_index": goal_index,
            }
            if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE
            else {
                "endpoint_source": endpoint_source,
                "endpoint_waypoint_spacing_m": endpoint_waypoint_spacing_m,
                "start_endpoint_index": start_index,
                "goal_endpoint_index": goal_index,
            }
        ),
        "start_carla_transform": helper.transform_dict(start_transform),
        "start_ros_pose": helper.ros_pose_dict(start_transform),
        "goal_carla_transform": goal_carla_transform,
        "goal_ros_pose": goal_ros_pose,
        "goal_endpoint_provenance": goal_endpoint_provenance,
        "spawn_point": ",".join(
            f"{value:.6f}"
            for value in (
                start_transform.location.x,
                start_transform.location.y,
                start_transform.location.z,
                start_transform.rotation.roll,
                start_transform.rotation.pitch,
                start_transform.rotation.yaw,
            )
        ),
        "option_counts": option_counts,
        "route": route_points,
    }
    if endpoint_source == GENERATED_WAYPOINT_ENDPOINT_SOURCE:
        payload["spawn_height_contract"] = waypoint_spawn_height_contract(
            start_transform
        )
    if initial_approach is not None:
        payload["initial_approach_preflight"] = dict(initial_approach)
    if turn_geometry is not None:
        payload["turn_geometry_preflight"] = dict(turn_geometry)
    if physical_straight is not None:
        payload["physical_straight_preflight"] = dict(physical_straight)
    if physical_turn is not None:
        payload["physical_turn_preflight"] = dict(physical_turn)
    return payload


def _planar_location_distance(first: Any, second: Any) -> float:
    return math.hypot(
        float(first.x) - float(second.x),
        float(first.y) - float(second.y),
    )


def normalize_route_endpoints(
    route: Sequence[Any], start_location: Any, goal_location: Any
) -> list[Any]:
    """Trim planner snapping and overshoot using planar endpoint proximity."""
    if not route:
        return []
    start_index = min(
        range(len(route)),
        key=lambda index: (
            _planar_location_distance(
                route[index][0].transform.location, start_location
            ),
            index,
        ),
    )
    goal_index = min(
        range(start_index, len(route)),
        key=lambda index: (
            _planar_location_distance(
                route[index][0].transform.location, goal_location
            ),
            -index,
        ),
    )
    return list(route[start_index : goal_index + 1])


def route_endpoint_offsets(
    route: Sequence[Any], start_location: Any, goal_location: Any
) -> tuple[float, float]:
    """Measure endpoint snapping in the map plane, independent of elevation."""
    first_location = route[0][0].transform.location
    last_location = route[-1][0].transform.location
    return (
        _planar_location_distance(first_location, start_location),
        _planar_location_distance(last_location, goal_location),
    )


def _world_map_matches(world: Any, map_entry: Mapping[str, Any]) -> bool:
    name = str(world.get_map().name).rstrip("/")
    return name == map_entry["load_name"] or name.rsplit("/", 1)[-1] == map_entry[
        "canonical_name"
    ]


def activate_map(
    client: Any,
    world: Any,
    map_entry: Mapping[str, Any],
    settle_sec: float,
    sleep: Any = time.sleep,
    *,
    allow_map_load: bool = False,
) -> tuple[Any, bool]:
    """Verify an external map, loading it only after an explicit unsafe opt-in."""
    if settle_sec > 0.0:
        sleep(settle_sec)
    loaded = False
    if not _world_map_matches(world, map_entry):
        if not allow_map_load:
            raise CatalogError(
                f"current CARLA map {world.get_map().name!r} does not match "
                f"required map {map_entry['load_name']!r}; automatic map loading is "
                "disabled, so start CARLA on the required map or pass the explicit "
                "unsafe --allow-map-load opt-in"
            )
        world = client.load_world(map_entry["load_name"])
        loaded = True
        if settle_sec > 0.0:
            sleep(settle_sec)
    if not _world_map_matches(world, map_entry):
        raise CatalogError(
            f"server did not activate {map_entry['load_name']!r}; "
            f"current={world.get_map().name!r}"
        )
    return world, loaded


def build_catalog(
    args: argparse.Namespace,
    manifest: Mapping[str, Any],
    carla: Any,
    helper: Any,
) -> dict[str, Any]:
    map_entry = _map_by_id(manifest, args.map_id)
    if map_entry["status"] == "unavailable":
        raise CatalogError(f"map {args.map_id} is unavailable: {map_entry['reason']}")
    validate_active_server_profile(
        manifest, map_entry, getattr(args, "active_server_profile", None)
    )

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    initial_map_name = str(world.get_map().name)
    world, map_load_performed = activate_map(
        client,
        world,
        map_entry,
        getattr(args, "map_load_settle_sec", 10.0),
        allow_map_load=getattr(args, "allow_map_load", False),
    )

    carla_map = world.get_map()
    spawn_points = list(carla_map.get_spawn_points())
    capacity_contract = straight_capacity_contract(args)
    endpoint_junction_policy = getattr(args, "endpoint_junction_policy", "include")
    candidate_enumeration_policy = getattr(
        args, "candidate_enumeration_policy", "all_pairs"
    )
    endpoint_waypoint_spacing_m = validate_endpoint_waypoint_spacing(
        getattr(args, "endpoint_waypoint_spacing_m", None)
    )
    if endpoint_waypoint_spacing_m is None:
        endpoint_source = SPAWN_POINT_ENDPOINT_SOURCE
        endpoints = spawn_points
        endpoint_waypoints: list[Any] = []
        if len(endpoints) < 2:
            raise CatalogError(f"map {args.map_id} exposes fewer than two spawn points")
        excluded_endpoint_indices = validate_excluded_spawn_indices(
            getattr(args, "exclude_spawn_indices", ()), len(endpoints)
        )
    else:
        endpoint_source = GENERATED_WAYPOINT_ENDPOINT_SOURCE
        if getattr(args, "exclude_spawn_indices", ()):
            raise CatalogError(
                "exclude-spawn-indices cannot be used with generated waypoint "
                "route endpoints"
            )
        (
            endpoints,
            endpoint_waypoints,
            endpoint_pool_provenance,
        ) = _generated_waypoint_endpoint_records(
            carla_map,
            endpoint_waypoint_spacing_m,
            endpoint_junction_policy,
        )
        excluded_endpoint_indices = ()
    eligible_endpoint_count = len(endpoints) - len(excluded_endpoint_indices)
    planner = helper.GlobalRoutePlanner(carla_map, args.sampling_resolution)
    output_root = args.output_root.expanduser().resolve()
    route_root = output_root / "routes" / args.map_id
    catalog_routes = []
    scenario_results = []
    used_pairs: set[tuple[int, int]] = set()
    total_coverage = Counter()
    total_error_types: Counter[str] = Counter()
    total_error_samples: list[dict[str, Any]] = []
    total_physical_straight_rejection_samples: list[dict[str, Any]] = []
    total_physical_turn_rejection_samples: list[dict[str, Any]] = []
    approach_contract = initial_approach_contract(args)
    geometry_contract = turn_geometry_contract(args)
    selected_scenarios = tuple(getattr(args, "scenarios", SCENARIOS))
    straight_contract = physical_straight_contract(args)
    packaged_turn_contract = physical_turn_contract(args)

    for scenario in selected_scenarios:
        created_before = len(catalog_routes)
        traces = 0
        coverage = Counter()
        error_types: Counter[str] = Counter()
        error_samples: list[dict[str, Any]] = []
        candidate_enumeration_reports: list[dict[str, Any]] = []
        physical_straight_rejection_samples: list[dict[str, Any]] = []
        physical_turn_rejection_samples: list[dict[str, Any]] = []
        for seed in args.seeds:
            created_for_seed = 0
            if candidate_enumeration_policy == "directed_topology_straight_v1":
                if scenario != "straight" or not straight_contract["enabled"]:
                    raise CatalogError(
                        "directed topology candidate enumeration is restricted "
                        "to physical-straight generation"
                    )
                if len(endpoint_waypoints) != len(endpoints):
                    raise CatalogError(
                        "directed topology candidate enumeration lacks waypoint "
                        "endpoint provenance"
                    )
                pairs, enumeration_report = (
                    deterministic_directed_topology_straight_pairs(
                        endpoints,
                        planner,
                        seed,
                        args.min_distance,
                        args.max_distance,
                        args.preferred_distance,
                        args.sampling_resolution,
                        straight_contract["limits"],
                    )
                )
            else:
                pairs = deterministic_pairs(
                    endpoints,
                    seed,
                    args.min_distance,
                    args.max_distance,
                    args.preferred_distance,
                    args.sampling_resolution,
                    excluded_endpoint_indices,
                )
                enumeration_report = {
                    "policy": "all_pairs",
                    "input_endpoint_count": len(endpoints),
                    "ranked_candidate_count": len(pairs),
                    "ranking": (
                        "preferred_chord_distance_bucket_then_seeded_sha256"
                    ),
                    "postfilter_authority": (
                        "exact serialized route scenario/distance and enabled "
                        "physical geometry analysis"
                    ),
                }
            candidate_enumeration_reports.append(
                {"seed": seed, **enumeration_report}
            )
            for start_index, goal_index in pairs:
                if traces >= args.max_traces or created_for_seed >= args.pairs_per_seed:
                    break
                if (start_index, goal_index) in used_pairs:
                    continue
                traces += 1
                coverage["attempted"] += 1
                _reset_planner(planner, helper)
                route, trace_error = trace_route_candidate(
                    planner,
                    endpoints[start_index].location,
                    endpoints[goal_index].location,
                )
                if trace_error is not None:
                    coverage["planner_error"] += 1
                    error_types[type(trace_error).__name__] += 1
                    if len(error_samples) < MAX_TRACE_ERROR_SAMPLES:
                        error_samples.append(
                            _trace_error_sample(
                                start_index,
                                goal_index,
                                trace_error,
                                endpoint_source,
                            )
                        )
                    continue
                if not route or len(route) < 2:
                    coverage["no_route"] += 1
                    continue
                route = normalize_route_endpoints(
                    route,
                    endpoints[start_index].location,
                    endpoints[goal_index].location,
                )
                start_offset, goal_offset = route_endpoint_offsets(
                    route,
                    endpoints[start_index].location,
                    endpoints[goal_index].location,
                )
                maximum_endpoint_offset = float(
                    getattr(args, "max_endpoint_offset", 2.0)
                )
                if max(start_offset, goal_offset) > maximum_endpoint_offset:
                    coverage["endpoint_rejected"] += 1
                    continue
                if len(route) < 2:
                    coverage["no_route"] += 1
                    continue
                if not helper.route_matches(route, scenario):
                    coverage["scenario_mismatch"] += 1
                    continue
                serialized_route = helper.serialize_route(
                    route, endpoints[goal_index]
                )
                length = serialized_route_length(serialized_route)
                if not args.min_distance <= length <= args.max_distance:
                    coverage["distance_rejected"] += 1
                    continue
                physical_straight = None
                if straight_contract["enabled"] and scenario == "straight":
                    try:
                        physical_straight = analyze_serialized_physical_straight(
                            {"route": serialized_route},
                            straight_contract["limits"],
                        )
                    except PhysicalStraightGeometryError as error:
                        raise CatalogError(
                            f"serialized physical-straight analysis failed: {error}"
                        ) from error
                    if physical_straight["status"] != "PASS":
                        coverage["physical_straight_rejected"] += 1
                        if (
                            len(physical_straight_rejection_samples)
                            < MAX_TRACE_ERROR_SAMPLES
                        ):
                            physical_straight_rejection_samples.append(
                                _physical_straight_rejection_sample(
                                    start_index,
                                    goal_index,
                                    physical_straight,
                                    endpoint_source,
                                )
                            )
                        continue
                physical_turn = None
                if packaged_turn_contract["enabled"] and scenario in (
                    "left",
                    "right",
                ):
                    try:
                        physical_turn = analyze_serialized_physical_turn(
                            {
                                "scenario": scenario,
                                "route_length_m": length,
                                "route": serialized_route,
                            },
                            packaged_turn_contract["limits"],
                        )
                    except PhysicalTurnGeometryError as error:
                        if error.fatal:
                            raise CatalogError(
                                "serialized physical-turn analysis failed "
                                f"fatally ({error.error_scope}): {error}"
                            ) from error
                        coverage["physical_turn_rejected"] += 1
                        if (
                            len(physical_turn_rejection_samples)
                            < MAX_TRACE_ERROR_SAMPLES
                        ):
                            physical_turn_rejection_samples.append(
                                _physical_turn_error_rejection_sample(
                                    start_index,
                                    goal_index,
                                    scenario,
                                    length,
                                    error,
                                    endpoint_source,
                                )
                            )
                        continue
                    if physical_turn["status"] != "PASS":
                        coverage["physical_turn_rejected"] += 1
                        if (
                            len(physical_turn_rejection_samples)
                            < MAX_TRACE_ERROR_SAMPLES
                        ):
                            physical_turn_rejection_samples.append(
                                _physical_turn_rejection_sample(
                                    start_index,
                                    goal_index,
                                    physical_turn,
                                    endpoint_source,
                                )
                            )
                        continue
                initial_approach = None
                if approach_contract["enabled"] and scenario != "lane_follow":
                    initial_approach = analyze_initial_approach(
                        route, approach_contract
                    )
                    if initial_approach["status"] != "PASS":
                        coverage["initial_approach_rejected"] += 1
                        continue
                turn_geometry = None
                if geometry_contract["enabled"] and scenario in ("left", "right"):
                    try:
                        turn_geometry = analyze_serialized_custom_turn(
                            serialized_route, scenario, geometry_contract
                        )
                    except SerializedCustomTurnGeometryError as error:
                        raise CatalogError(
                            f"serialized custom-turn analysis failed: {error}"
                        ) from error
                    if turn_geometry["status"] != "PASS":
                        coverage["turn_geometry_rejected"] += 1
                        continue
                pair_index = created_for_seed
                route_id = f"{args.map_id}_{scenario}_s{seed:04d}_p{pair_index:02d}"
                route_path = route_root / scenario / f"{route_id}.json"
                payload = _route_payload(
                    helper,
                    map_entry,
                    args.weather,
                    scenario,
                    args.sampling_resolution,
                    start_index,
                    goal_index,
                    endpoints,
                    route,
                    initial_approach,
                    turn_geometry,
                    endpoint_source=endpoint_source,
                    endpoint_waypoint_spacing_m=endpoint_waypoint_spacing_m,
                    serialized_route=serialized_route,
                    physical_straight=physical_straight,
                    physical_turn=physical_turn,
                )
                atomic_write_json(route_path, payload)
                used_pairs.add((start_index, goal_index))
                created_for_seed += 1
                coverage["accepted"] += 1
                catalog_routes.append(
                    {
                        "id": route_id,
                        "status": "ready",
                        "scenario": scenario,
                        "seed": seed,
                        "pair_index": pair_index,
                        **(
                            {
                                "start_spawn_index": start_index,
                                "goal_spawn_index": goal_index,
                            }
                            if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE
                            else {
                                "endpoint_source": endpoint_source,
                                "start_endpoint_index": start_index,
                                "goal_endpoint_index": goal_index,
                            }
                        ),
                        "route_length_m": payload["route_length_m"],
                        "initial_approach_preflight": initial_approach,
                        **(
                            {"physical_straight_preflight": physical_straight}
                            if physical_straight is not None
                            else {}
                        ),
                        **(
                            {"physical_turn_preflight": physical_turn}
                            if physical_turn is not None
                            else {}
                        ),
                        **(
                            {"turn_geometry_preflight": turn_geometry}
                            if turn_geometry is not None
                            else {}
                        ),
                        "path": route_path.relative_to(output_root).as_posix(),
                        "sha256": sha256_file(route_path),
                    }
                )
            if capacity_contract["enabled"] and created_for_seed != (
                args.pairs_per_seed
            ):
                raise CatalogError(
                    "straight-capacity profile could not fill the exact accepted "
                    f"pair quota for seed {seed}: accepted={created_for_seed} "
                    f"required={args.pairs_per_seed} traces={traces} "
                    f"maximum={args.max_traces}"
                )
        count = len(catalog_routes) - created_before
        skipped = (
            coverage["planner_error"]
            + coverage["no_route"]
            + coverage["endpoint_rejected"]
            + coverage["scenario_mismatch"]
            + coverage["distance_rejected"]
            + coverage["physical_straight_rejected"]
            + coverage["physical_turn_rejected"]
            + coverage["initial_approach_rejected"]
            + coverage["turn_geometry_rejected"]
        )
        total_coverage.update(coverage)
        total_error_types.update(error_types)
        remaining_samples = MAX_TRACE_ERROR_SAMPLES - len(total_error_samples)
        if remaining_samples > 0:
            total_error_samples.extend(error_samples[:remaining_samples])
        remaining_physical_samples = (
            MAX_TRACE_ERROR_SAMPLES
            - len(total_physical_straight_rejection_samples)
        )
        if remaining_physical_samples > 0:
            total_physical_straight_rejection_samples.extend(
                physical_straight_rejection_samples[:remaining_physical_samples]
            )
        remaining_turn_samples = (
            MAX_TRACE_ERROR_SAMPLES - len(total_physical_turn_rejection_samples)
        )
        if remaining_turn_samples > 0:
            total_physical_turn_rejection_samples.extend(
                physical_turn_rejection_samples[:remaining_turn_samples]
            )
        scenario_results.append(
            {
                "scenario": scenario,
                "status": "READY" if count else "SKIP",
                "route_count": count,
                "traces": traces,
                "candidate_enumeration": candidate_enumeration_reports,
                "trace_coverage": {
                    "attempted": coverage["attempted"],
                    "accepted": coverage["accepted"],
                    "skipped": skipped,
                    "planner_errors": coverage["planner_error"],
                    "no_route": coverage["no_route"],
                    "endpoint_rejected": coverage["endpoint_rejected"],
                    "scenario_mismatch": coverage["scenario_mismatch"],
                    "distance_rejected": coverage["distance_rejected"],
                    **(
                        {
                            "physical_straight_rejected": coverage[
                                "physical_straight_rejected"
                            ],
                            "physical_straight_rejection_samples": (
                                physical_straight_rejection_samples
                            ),
                        }
                        if straight_contract["enabled"]
                        else {}
                    ),
                    **(
                        {
                            "physical_turn_rejected": coverage[
                                "physical_turn_rejected"
                            ],
                            "physical_turn_rejection_samples": (
                                physical_turn_rejection_samples
                            ),
                        }
                        if packaged_turn_contract["enabled"]
                        else {}
                    ),
                    "initial_approach_rejected": coverage[
                        "initial_approach_rejected"
                    ],
                    **(
                        {
                            "turn_geometry_rejected": coverage[
                                "turn_geometry_rejected"
                            ]
                        }
                        if geometry_contract["enabled"]
                        else {}
                    ),
                    "planner_error_types": dict(sorted(error_types.items())),
                    "planner_error_samples": error_samples,
                },
                "reason": None if count else "no matching route in the requested constraints",
            }
        )

    catalog = {
        "schema_version": 1,
        "status": "complete" if catalog_routes else "blocked",
        "map_id": args.map_id,
        "canonical_name": map_entry["canonical_name"],
        "load_name": map_entry["load_name"],
        "map_status": map_entry["status"],
        "server_profile": map_entry["server_profile"],
        "server": {
            "host": args.host,
            "port": args.port,
            "client_version": client.get_client_version(),
            "server_version": client.get_server_version(),
            "active_server_profile": getattr(args, "active_server_profile", None),
            "initial_map_name": initial_map_name,
            "active_map_name": str(carla_map.name),
            "map_load_allowed": getattr(args, "allow_map_load", False),
            "map_load_performed": map_load_performed,
            "map_lifecycle_managed": map_load_performed,
            "pre_read_settle_sec": getattr(args, "map_load_settle_sec", 10.0),
            "map_load_settle_sec": (
                getattr(args, "map_load_settle_sec", 10.0)
                if map_load_performed
                else 0.0
            ),
            "map_load_warning": (
                "client.load_world was explicitly enabled and may crash packaged or "
                "custom CARLA maps"
                if getattr(args, "allow_map_load", False)
                else None
            ),
            "server_lifecycle_managed": False,
        },
        "generation": {
            "weather": args.weather,
            "scenarios": list(selected_scenarios),
            "seeds": list(args.seeds),
            "pairs_per_seed": args.pairs_per_seed,
            "minimum_distance_m": args.min_distance,
            "maximum_distance_m": args.max_distance,
            "preferred_distance_m": args.preferred_distance,
            "sampling_resolution_m": args.sampling_resolution,
            "maximum_endpoint_offset_m": float(
                getattr(args, "max_endpoint_offset", 2.0)
            ),
            "initial_approach_contract": approach_contract,
            **(
                {"turn_geometry_contract": geometry_contract}
                if geometry_contract["enabled"]
                else {}
            ),
            **(
                {"physical_straight_contract": straight_contract}
                if straight_contract["enabled"]
                else {}
            ),
            **(
                {"physical_turn_contract": packaged_turn_contract}
                if packaged_turn_contract["enabled"]
                else {}
            ),
            "max_traces_per_scenario": args.max_traces,
            "endpoint_source": endpoint_source,
            "endpoint_waypoint_spacing_m": endpoint_waypoint_spacing_m,
            "endpoint_junction_policy": endpoint_junction_policy,
            "candidate_enumeration_policy": candidate_enumeration_policy,
            "straight_capacity_contract": capacity_contract,
            "endpoint_count": len(endpoints),
            "eligible_endpoint_count": eligible_endpoint_count,
            "endpoint_ordering": (
                "carla_spawn_point_index"
                if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE
                else "deduplicated_lexicographic_transform_x_y_z_roll_pitch_yaw"
            ),
            "spawn_point_count": len(spawn_points),
            **(
                {
                    "excluded_spawn_indices": list(excluded_endpoint_indices),
                    "excluded_spawn_point_count": len(excluded_endpoint_indices),
                    "eligible_spawn_point_count": eligible_endpoint_count,
                }
                if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE
                else {
                    "endpoint_api_count": endpoint_pool_provenance["api_count"],
                    "endpoint_eligible_api_count": endpoint_pool_provenance[
                        "eligible_api_count"
                    ],
                    "endpoint_junction_waypoint_count": endpoint_pool_provenance[
                        "junction_waypoint_count"
                    ],
                    "endpoint_junction_excluded_count": endpoint_pool_provenance[
                        "junction_excluded_count"
                    ],
                    "endpoint_duplicate_transform_count": endpoint_pool_provenance[
                        "duplicate_transform_count"
                    ],
                    "endpoint_deduplication": "exact_full_transform_keep_first_api_occurrence",
                    "spawn_height_contract": {
                        "catalog_z_offset_m": 0.0,
                        "bridge_z_offset_m": BRIDGE_SPAWN_Z_OFFSET_M,
                        "offset_owner": "autoware_carla_interface_bridge",
                        "bridge_source": BRIDGE_SPAWN_Z_OFFSET_SOURCE,
                    }
                }
            ),
            "trace_coverage": {
                "attempted": total_coverage["attempted"],
                "accepted": total_coverage["accepted"],
                "skipped": (
                    total_coverage["planner_error"]
                    + total_coverage["no_route"]
                    + total_coverage["endpoint_rejected"]
                    + total_coverage["scenario_mismatch"]
                    + total_coverage["distance_rejected"]
                    + total_coverage["physical_straight_rejected"]
                    + total_coverage["physical_turn_rejected"]
                    + total_coverage["initial_approach_rejected"]
                    + total_coverage["turn_geometry_rejected"]
                ),
                "planner_errors": total_coverage["planner_error"],
                "no_route": total_coverage["no_route"],
                "endpoint_rejected": total_coverage["endpoint_rejected"],
                "scenario_mismatch": total_coverage["scenario_mismatch"],
                "distance_rejected": total_coverage["distance_rejected"],
                **(
                    {
                        "physical_straight_rejected": total_coverage[
                            "physical_straight_rejected"
                        ],
                        "physical_straight_rejection_samples": (
                            total_physical_straight_rejection_samples
                        ),
                    }
                    if straight_contract["enabled"]
                    else {}
                ),
                **(
                    {
                        "physical_turn_rejected": total_coverage[
                            "physical_turn_rejected"
                        ],
                        "physical_turn_rejection_samples": (
                            total_physical_turn_rejection_samples
                        ),
                    }
                    if packaged_turn_contract["enabled"]
                    else {}
                ),
                "initial_approach_rejected": total_coverage[
                    "initial_approach_rejected"
                ],
                **(
                    {
                        "turn_geometry_rejected": total_coverage[
                            "turn_geometry_rejected"
                        ]
                    }
                    if geometry_contract["enabled"]
                    else {}
                ),
                "planner_error_types": dict(sorted(total_error_types.items())),
                "planner_error_samples": total_error_samples,
                **(
                    {
                        "excluded_spawn_point_count": len(
                            excluded_endpoint_indices
                        ),
                        "eligible_spawn_point_count": eligible_endpoint_count,
                    }
                    if endpoint_source == SPAWN_POINT_ENDPOINT_SOURCE
                    else {
                        "endpoint_count": len(endpoints),
                        "eligible_endpoint_count": eligible_endpoint_count,
                    }
                ),
            },
        },
        "scenario_results": scenario_results,
        "routes": catalog_routes,
    }
    atomic_write_json(output_root / "route_catalog.json", catalog)
    return catalog


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest, _ = load_manifest(args.manifest)
        carla, helper = _load_runtime_modules()
        catalog = build_catalog(args, manifest, carla, helper)
    except (CatalogError, ManifestError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"catalog={args.output_root.expanduser().resolve() / 'route_catalog.json'} "
        f"map={catalog['map_id']} routes={len(catalog['routes'])} status={catalog['status']}"
    )
    return 0 if catalog["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
