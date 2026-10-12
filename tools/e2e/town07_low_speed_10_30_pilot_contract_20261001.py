"""HH_261001 - Prospective native-only Town07 10–30 m low-speed feasibility gate."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-01/town07_low_speed_10_30_native_pilot_v1"
PLAN = ROOT / "storage/experiments/validation/2026-10-01/town07_low_speed_10_30_native_pilot_plan_v1.json"
ROUTE = ROOT / "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json"
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
XODR = CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town07.xodr"
ROUTE_SHA256 = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
XODR_SHA256 = "be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf"
EGG_SHA256 = "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a"
FLOOR_BYTES = 5 * 1024**3
STARTUP_RESERVE_BYTES = 128 * 1024**2
CLOSING_RESERVE_BYTES = 16 * 1024**2
MAX_TICKS = 1200
WARMUP_TICKS = 70
WARMUP_REFERENCE_PROGRESS_M = 0.3180999016452199
WARMUP_PROGRESS_TOLERANCE_M = 0.1
WARMUP_MAX_PLANAR_DISPLACEMENT_M = 0.05
TICK_SECONDS = 0.05
TARGET_KMH = 4.0
WINDOW_LO_M = 10.0
WINDOW_HI_M = 30.0
TARGET_SPEED_LO_MPS = 0.7
TARGET_SPEED_HI_MPS = 1.5
MAX_ABS_RATE_MPS2 = 2.9
GLOBAL_EMERGENCY_RATE_MPS2 = 8.0
MIN_WINDOW_SPAN_S = 12.0
SOURCE_PATHS = (
    "tools/e2e/town07_low_speed_10_30_pilot_contract_20261001.py",
    "tools/e2e/collect_town07_low_speed_10_30_pilot_20261001.py",
    "tools/e2e/run_owned_town07_low_speed_10_30_pilot_20261001.py",
    "tools/e2e/collect_carla_vad_expert.py",
    "tools/e2e/carla_moving_capture.py",
    "tools/e2e/carla_goal_stop_profile.py",
    "tools/e2e/collect_town07_throttle_isolation_20261001.py",
    "tools/e2e/town07_throttle_isolation_contract_20261001.py",
    "tools/e2e/run_owned_ah_town07_research_20261001.py",
    "tools/e2e/run_owned_town13_route_catalog.py",
    "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py",
    "tools/e2e/prepare_ctrack_isolated_probe_20261001.py",
    "tools/e2e/audit_town13_source_editor_preflight.py",
    "tools/e2e/inventory_carla_training_maps.py",
    "tools/e2e/prepare_carla_expert_route_catalog.py",
    "tools/e2e/physical_straight_geometry.py",
    "tools/e2e/physical_turn_geometry.py",
    "tools/e2e/serialized_custom_turn_geometry.py",
    "tools/e2e/run_carla_map.sh",
    "tools/e2e/env.sh",
    "tools/workspace_python.py",
    "tools/canonical_workspace.py",
)
AGENT_PATHS = tuple(CARLA_ROOT / "PythonAPI/carla/agents" / name for name in (
    "navigation/basic_agent.py", "navigation/local_planner.py", "navigation/controller.py",
    "navigation/global_route_planner.py", "tools/misc.py"))


def sha(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise ValueError("regular pinned source required: " + str(path))
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def free_bytes() -> int:
    return shutil.disk_usage(ROOT).free


def fixed_contract() -> dict:
    """HH_261001 - One no-camera no-training pilot, not a replacement for full 10 Hz capture."""
    return dict(schema="carla.town07_low_speed_10_30_native_pilot.20261001.v1",
        purpose="Prospective BasicAgent 4 km/h feasibility only; no six-camera data or training labels",
        route_sha256=ROUTE_SHA256, active_opendrive_sha256=XODR_SHA256,
        town="Town07", weather="ClearNoon", seed=2026100101,
        vehicle="vehicle.toyota.prius", quality="Epic", port_triplet=[2440, 2441, 2442],
        physics_hz=20, fixed_delta_seconds=TICK_SECONDS, physics_substeps="inherited",
        target_speed_kmh=TARGET_KMH, basic_agent_emergency_unchanged=True,
        normal_brake_cap=0.0, normal_throttle_cap=0.20,
        speed_reference="immutable CARLA actor-snapshot world_velocity_carla planar hypot; not base_link yaw-corrected speed",
        progress_reference="unclamped nearest-segment arc from immutable recorded rear-reference pose",
        selected_route_progress_m=[WINDOW_LO_M, WINDOW_HI_M],
        speed_feasibility_mps=[TARGET_SPEED_LO_MPS, TARGET_SPEED_HI_MPS],
        maximum_absolute_scalar_rate_mps2=MAX_ABS_RATE_MPS2,
        global_emergency_scalar_rate_mps2=GLOBAL_EMERGENCY_RATE_MPS2,
        minimum_selected_span_seconds=MIN_WINDOW_SPAN_S,
        max_native_ticks=MAX_TICKS, maximum_attempts=1, automatic_retry=False,
        stationary_warmup_ticks=WARMUP_TICKS,
        stationary_warmup_reference_route_progress_m=WARMUP_REFERENCE_PROGRESS_M,
        stationary_warmup_progress_tolerance_m=WARMUP_PROGRESS_TOLERANCE_M,
        stationary_warmup_max_planar_displacement_m=WARMUP_MAX_PLANAR_DISPLACEMENT_M,
        fixed_membership="Every consecutive native state at 10–30 m inclusive, with first >30 m closing state; no trimming",
        safety_abort=dict(max_abs_route_cte_m=1.0, maximum_speed_mps=2.9,
            collision_or_lane_invasion=True, red_traffic_light=True,
            ack_snapshot_or_frame_mismatch=True,
            minimum_free_bytes=FLOOR_BYTES + CLOSING_RESERVE_BYTES),
        resource_guard=dict(startup_free_bytes=FLOOR_BYTES + STARTUP_RESERVE_BYTES,
            running_free_bytes=FLOOR_BYTES + CLOSING_RESERVE_BYTES,
            closing_floor_bytes=FLOOR_BYTES),
        camera_count=0, model_inferences=0, training_gpu_use=False,
        whole_body_lane_clearance_proven=False,
        bounded_sweep_clearance_proven=False,
        training_data_approved=False, vehicle_control_approved=False,
        full_six_camera_capture_authorized_by_this_plan=False,
        outcome_dependent_window_selection=False)


def build_plan() -> dict:
    if PLAN.exists() or PLAN.is_symlink() or RUN.exists() or RUN.is_symlink():
        raise ValueError("pilot declaration and output must be create-only")
    assets = {str(ROUTE): ROUTE_SHA256, str(XODR): XODR_SHA256, str(EGG): EGG_SHA256}
    if any(sha(Path(name)) != digest for name, digest in assets.items()):
        raise ValueError("packaged route, map, or Python API changed")
    return dict(fixed_contract(), declared_at_utc=datetime.now(timezone.utc).isoformat(),
        asset_sha256=assets,
        source_sha256={name: sha(ROOT / name) for name in SOURCE_PATHS},
        external_agent_sha256={str(path): sha(path) for path in AGENT_PATHS})


def verify_plan(plan: dict, expected_sha256: str) -> None:
    if sha(PLAN) != expected_sha256:
        raise ValueError("caller-pinned prospective pilot plan differs")
    fixed = fixed_contract()
    if set(plan) != set(fixed) | {"declared_at_utc", "asset_sha256", "source_sha256", "external_agent_sha256"}:
        raise ValueError("pilot plan key closure differs")
    if any(plan[name] != expected for name, expected in fixed.items()):
        raise ValueError("pilot protocol differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    if stamp.tzinfo is None or stamp.utcoffset().total_seconds() != 0 or stamp > datetime.now(timezone.utc):
        raise ValueError("valid prior UTC declaration required")
    if plan["asset_sha256"] != {str(ROUTE): ROUTE_SHA256, str(XODR): XODR_SHA256, str(EGG): EGG_SHA256}:
        raise ValueError("pilot asset closure differs")
    if set(plan["source_sha256"]) != set(SOURCE_PATHS):
        raise ValueError("pilot local source closure differs")
    if set(plan["external_agent_sha256"]) != {str(path) for path in AGENT_PATHS}:
        raise ValueError("pilot external source closure differs")
    for name, digest in {**plan["asset_sha256"], **plan["external_agent_sha256"]}.items():
        if sha(Path(name)) != digest:
            raise ValueError("pilot external byte identity differs: " + name)
    for name, digest in plan["source_sha256"].items():
        if sha(ROOT / name) != digest:
            raise ValueError("pilot local byte identity differs: " + name)


def assess_native_window(rows: list[dict]) -> dict:
    """HH_261001 - Pure fail-closed feasibility check over the entire fixed native window."""
    failures = []
    if not isinstance(rows, list) or not rows:
        return dict(status="FAIL", failures=["no_native_states"], selected_count=0,
            training_data_approved=False)
    selected = []
    closed = False
    previous = None
    outside_selected_2p9_exceedances = []
    warmup = []
    warmup_origin_xy = None
    for index, row in enumerate(rows):
        try:
            if not isinstance(row, dict):
                raise ValueError("native_state_not_mapping")
            frame = row["frame"]
            stamp = row["timestamp"]
            progress = row["route_progress_m"]
            speed = row["speed_mps"]
            cte = row["route_cte_m"]
            x, y = row["x"], row["y"]
            aligned = row["control_alignment"]["status"] == "PASS"
            collisions, lane_invasions = row["collision_count"], row["lane_invasion_count"]
            hazard = row["traffic_light_hazard"]["hazard"]
            free = row["observed_free_disk_bytes"]
            phase = row["phase"]
            # HH_261001 - Reject malformed native evidence as FAIL before cadence or geometry arithmetic.
            if (type(frame) is not int or frame < 0
                    or any(type(value) not in (int, float) or not math.isfinite(value)
                           for value in (stamp, progress, speed, cte, x, y))
                    or any(type(value) is not int or value < 0
                           for value in (collisions, lane_invasions, free))
                    or type(hazard) is not bool):
                raise ValueError("nonfinite_or_wrong_type")
            events = collisions + lane_invasions
            if phase not in ("stationary_warmup", "driving"):
                raise ValueError("unknown_phase")
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            failures.append(f"malformed_native_state_{index}:{error}")
            continue
        rate = None
        if previous is not None:
            if frame != previous["frame"] + 1 or abs(stamp - previous["timestamp"] - TICK_SECONDS) > 1e-6:
                failures.append(f"native_cadence_{index}")
            if progress + 1e-8 < previous["route_progress_m"]:
                failures.append(f"route_progress_reversal_{index}")
            rate = (speed - previous["speed_mps"]) / (stamp - previous["timestamp"]) if stamp > previous["timestamp"] else math.inf
            if not math.isfinite(rate) or abs(rate) > GLOBAL_EMERGENCY_RATE_MPS2 + 1e-9:
                failures.append(f"global_emergency_scalar_rate_{index}")
        if not aligned or events or hazard or abs(cte) > 1.0 or speed > 2.9 or free < FLOOR_BYTES + CLOSING_RESERVE_BYTES:
            failures.append(f"global_safety_{index}")
        if phase == "stationary_warmup":
            warmup.append(row)
            if warmup_origin_xy is None:
                warmup_origin_xy = (x, y)
            if index != len(warmup) - 1:
                failures.append(f"nonprefix_warmup_{index}")
            if abs(progress - WARMUP_REFERENCE_PROGRESS_M) > WARMUP_PROGRESS_TOLERANCE_M or speed > .05:
                failures.append(f"warmup_settle_{index}")
            if math.hypot(x - warmup_origin_xy[0], y - warmup_origin_xy[1]) > WARMUP_MAX_PLANAR_DISPLACEMENT_M:
                failures.append(f"warmup_vehicle_drift_{index}")
        elif len(warmup) != WARMUP_TICKS:
            failures.append(f"driving_before_complete_70_tick_warmup_{index}")
        if closed:
            # HH_261001 - The first >30 m sample is the sole closing state, never a trim boundary.
            failures.append(f"postclosure_extra_state_{index}")
        in_window = WINDOW_LO_M <= progress <= WINDOW_HI_M
        entry_or_close_guard = (in_window and not closed) or (previous is not None
            and WINDOW_LO_M <= previous["route_progress_m"] <= WINDOW_HI_M and progress > WINDOW_HI_M)
        if rate is not None and math.isfinite(rate) and abs(rate) > MAX_ABS_RATE_MPS2 + 1e-9:
            if entry_or_close_guard:
                failures.append(f"selected_or_closing_scalar_rate_{index}")
            else:
                outside_selected_2p9_exceedances.append(index)
        if in_window and not closed and phase == "driving":
            selected.append(row)
            if not TARGET_SPEED_LO_MPS <= speed <= TARGET_SPEED_HI_MPS:
                failures.append(f"speed_window_{index}")
        elif progress > WINDOW_HI_M and phase == "driving":
            closed = True
        elif closed and progress <= WINDOW_HI_M:
            failures.append(f"postclosure_reentry_{index}")
        elif selected and progress < WINDOW_LO_M:
            failures.append(f"window_reentry_{index}")
        previous = row
    if not selected:
        failures.append("no_fixed_window_states")
    if len(warmup) != WARMUP_TICKS:
        failures.append("not_exactly_70_warmup_states")
    if not closed:
        failures.append("no_first_closing_state_above_30m")
    span = selected[-1]["timestamp"] - selected[0]["timestamp"] if selected else None
    if span is None or span < MIN_WINDOW_SPAN_S:
        failures.append("selected_span_under_12_seconds")
    if selected and selected[0]["route_progress_m"] > WINDOW_LO_M + 0.1:
        failures.append("first_window_state_missed_10m_entry")
    if selected and selected[-1]["route_progress_m"] < WINDOW_HI_M - 0.1:
        failures.append("last_window_state_missed_30m_exit")
    return dict(schema="carla.town07_low_speed_10_30_pilot_assessment.v1",
        status="PASS_FEASIBILITY_ONLY" if not failures else "FAIL", failures=failures,
        selected_count=len(selected), selected_span_seconds=span,
        observed_warmup_state_count=len(warmup),
        first_selected_frame=selected[0]["frame"] if selected else None,
        last_selected_frame=selected[-1]["frame"] if selected else None,
        full_window_speed_bounds_mps=[TARGET_SPEED_LO_MPS, TARGET_SPEED_HI_MPS],
        full_window_scalar_rate_limit_mps2=MAX_ABS_RATE_MPS2,
        global_emergency_rate_limit_mps2=GLOBAL_EMERGENCY_RATE_MPS2,
        prewindow_or_postclosure_2p9_exceedance_indices=outside_selected_2p9_exceedances,
        whole_body_lane_clearance_proven=False,
        bounded_sweep_clearance_proven=False,
        full_six_camera_capture_ready_for_separate_declaration=not failures,
        training_data_approved=False, vehicle_control_approved=False)


def encoded(value: dict) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
