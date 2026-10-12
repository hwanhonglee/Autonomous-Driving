"""HH_261001 - V3 Town07 throttle protocol with bootstrap-relative warmup drift."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil


ROOT = Path(__file__).resolve().parents[2]
RUN_ROOT = ROOT / "storage/experiments/validation/2026-10-01/town07_throttle_isolation_v3"
PLAN = ROOT / "storage/experiments/validation/2026-10-01/town07_throttle_isolation_plan_v3.json"
ROUTE = ROOT / "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json"
COMPARATOR_STATES = ROOT / "storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1/episode/states.jsonl"
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
XODR = CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town07.xodr"
ROUTE_SHA256 = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
COMPARATOR_STATES_SHA256 = "c0e6f48268d654cb83bd5608ee2829ea7e3f813c49860b2659c0cac22da5fc62"
XODR_SHA256 = "be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf"
EGG_SHA256 = "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a"
FLOOR_BYTES = 5 * 1024**3
TRIAL_RESERVE_BYTES = 128 * 1024**2
CLOSING_RESERVE_BYTES = 16 * 1024**2
PHYSICS_HZ = 20
TICK_SECONDS = 0.05
INTERVENTION_TICKS = 60
WARMUP_TICKS = 70
PREFIX_MAX_TICKS = 800
BRANCH_SPEED_MPS = (2.35, 2.55)
BRANCH_PROGRESS_OVERSHOOT_M = 0.25
THROTTLES = {"A": 0.01, "B": 0.20}
SOURCE_PATHS = (
    "tools/e2e/town07_throttle_isolation_contract_v3_20261001.py",
    "tools/e2e/collect_town07_throttle_isolation_v3_20261001.py",
    "tools/e2e/run_owned_town07_throttle_isolation_v3_20261001.py",
    "tools/e2e/collect_carla_vad_expert.py",
    "tools/e2e/run_owned_ah_town07_research_20261001.py",
    "tools/e2e/run_owned_town13_route_catalog.py",
    "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py",
    "tools/e2e/run_carla_map.sh",
    "tools/workspace_python.py",
    "learning/tests/test_town07_throttle_isolation_v3_20261001.py",
    "docs/assets/validation/2026-10-01/selector_phase_ab/DRIVING_NEXT_QUALITY_GATE.md",
    "docs/assets/validation/2026-10-01/selector_phase_ab/TOWN07_THROTTLE_ISOLATION_V3.md",
    "docs/assets/validation/2026-10-01/selector_phase_ab/TOWN07_THROTTLE_ISOLATION_V2_WITHDRAWN.md",
)
AGENT_SOURCE_PATHS = tuple(
    CARLA_ROOT / "PythonAPI/carla/agents" / name for name in (
        "navigation/basic_agent.py", "navigation/local_planner.py",
        "navigation/controller.py", "navigation/global_route_planner.py", "tools/misc.py"
    )
)


def sha(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise ValueError("regular pinned file required: " + str(path))
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def trial_specs() -> tuple[dict, ...]:
    """HH_261001 - Site-major ABBA order is fixed before any simulator outcome."""
    rows = []
    for site_progress in (16, 25):
        seen = {"A": 0, "B": 0}
        for arm in ("A", "B", "B", "A"):
            seen[arm] += 1
            trial_id = f"site{site_progress:02d}_{arm}{seen[arm]}"
            rows.append(dict(index=len(rows), trial_id=trial_id, site_progress_m=site_progress,
                             arm=arm, throttle=THROTTLES[arm], output=str(RUN_ROOT / trial_id)))
    return tuple(rows)


def fixed_contract() -> dict:
    return dict(schema="carla.town07_native_throttle_isolation.20261001.v3",
        intent="simulation-only source-pinned low-throttle physics diagnostic; no model, camera, dataset or training",
        route_sha256=ROUTE_SHA256, active_opendrive_sha256=XODR_SHA256,
        stationary_comparator_states_sha256=COMPARATOR_STATES_SHA256,
        stationary_comparator_frame_1147_progress_m=0.31809989301818486,
        stationary_comparator_warmup_rows=70,
        town="Town07", vehicle="vehicle.toyota.prius", weather="ClearNoon", quality="Epic",
        seed=2026100101, port_triplet=[2440, 2441, 2442], physics_hz=PHYSICS_HZ,
        world_fixed_delta_seconds=TICK_SECONDS, physics_substep_profile="inherited",
        camera_count=0, model_inferences=0, training_gpu_use=False,
        carla_server_gpu_use_not_independently_excluded=True,
        setup_bootstrap_ticks=1, stationary_warmup_ticks=WARMUP_TICKS,
        stationary_warmup_seconds=WARMUP_TICKS*TICK_SECONDS,
        stationary_warmup_control="ACKed full service brake at every native tick",
        warmup_final_max_abs_vx_mps=0.05,
        warmup_absolute_start_corridor_m=[0.0, 0.5],
        warmup_max_abs_progress_drift_from_bootstrap_m=0.25,
        prefix="BasicAgent target 9 km/h, unchanged emergency, normal throttle at most 0.20 and normal brake 0",
        prefix_max_ticks=PREFIX_MAX_TICKS, branch_speed_mps=list(BRANCH_SPEED_MPS),
        branch_progress_overshoot_m=BRANCH_PROGRESS_OVERSHOOT_M,
        intervention_ticks=INTERVENTION_TICKS, branch_steer=0.0, branch_brake=0.0,
        branch_throttles=THROTTLES, trial_order=list(trial_specs()),
        safety_abort=dict(max_abs_route_cte_m=1.0, maximum_speed_mps=2.7,
            collision_or_lane_event=True, red_traffic_light=True,
            control_ack_or_snapshot_mismatch=True,
            native_frame_discontinuity=True, minimum_free_bytes=FLOOR_BYTES+CLOSING_RESERVE_BYTES),
        warmup_final_guard=dict(max_abs_vx_mps=0.05,
            absolute_bootstrap_and_final_progress_corridor_m=[0.0, 0.5],
            max_abs_progress_drift_from_bootstrap_m=0.25,
            max_planar_displacement_from_bootstrap_m=0.25),
        resource_guard=dict(startup_free_bytes=FLOOR_BYTES+TRIAL_RESERVE_BYTES,
            running_free_bytes=FLOOR_BYTES+CLOSING_RESERVE_BYTES, floor_bytes=FLOOR_BYTES),
        maximum_attempts_per_trial=1, automatic_retry=False, outcome_dependent_selection=False,
        no_rewrite_of_prior_A_B_or_V3=True, training_data_approved=False,
        vehicle_control_approved=False, model_promoted=False)


def build_plan() -> dict:
    if (sha(ROUTE) != ROUTE_SHA256 or sha(XODR) != XODR_SHA256
            or sha(EGG) != EGG_SHA256 or sha(COMPARATOR_STATES) != COMPARATOR_STATES_SHA256):
        raise ValueError("packaged CARLA route/API/map or original V3 comparison pin differs")
    return dict(fixed_contract(), declared_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256={name: sha(ROOT / name) for name in SOURCE_PATHS},
        external_agent_sha256={str(path): sha(path) for path in AGENT_SOURCE_PATHS},
        asset_sha256={str(ROUTE): ROUTE_SHA256, str(XODR): XODR_SHA256,
                      str(EGG): EGG_SHA256, str(COMPARATOR_STATES): COMPARATOR_STATES_SHA256})


def verify_plan(plan: dict, *, expected_sha256: str) -> None:
    if sha(PLAN) != expected_sha256:
        raise ValueError("declared plan bytes differ")
    fixed = fixed_contract()
    if set(plan) != set(fixed) | {"declared_at_utc", "source_sha256", "external_agent_sha256", "asset_sha256"}:
        raise ValueError("declared plan keys differ")
    if any(plan[key] != value for key, value in fixed.items()):
        raise ValueError("declared fixed protocol differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"])
    if stamp.tzinfo is None or stamp.utcoffset().total_seconds() != 0 or stamp > datetime.now(timezone.utc):
        raise ValueError("valid prior UTC declaration required")
    if set(plan["source_sha256"]) != set(SOURCE_PATHS):
        raise ValueError("declared local source closure differs")
    if set(plan["external_agent_sha256"]) != {str(path) for path in AGENT_SOURCE_PATHS}:
        raise ValueError("declared CARLA agent source closure differs")
    if plan["asset_sha256"] != {str(ROUTE): ROUTE_SHA256, str(XODR): XODR_SHA256,
                                 str(EGG): EGG_SHA256,
                                 str(COMPARATOR_STATES): COMPARATOR_STATES_SHA256}:
        raise ValueError("declared packaged asset identities differ")
    for name, expected in plan["source_sha256"].items():
        if sha(ROOT / name) != expected:
            raise ValueError("declared local source changed: " + name)
    for name, expected in plan["external_agent_sha256"].items():
        if sha(Path(name)) != expected:
            raise ValueError("declared CARLA agent source changed: " + name)
    for name, expected in plan["asset_sha256"].items():
        if sha(Path(name)) != expected:
            raise ValueError("declared packaged asset changed: " + name)


def trial(index: int) -> dict:
    if type(index) is not int or not 0 <= index < len(trial_specs()):
        raise ValueError("trial index must be one fixed member of the eight-trial ABBA order")
    return trial_specs()[index]


def branch_status(progress_m: float, speed_mps: float, site_progress_m: int) -> str:
    if not all(math.isfinite(value) for value in (progress_m, speed_mps)):
        return "FAIL_NONFINITE_BRANCH_STATE"
    if progress_m < site_progress_m:
        return "BEFORE_SITE"
    if progress_m > site_progress_m + BRANCH_PROGRESS_OVERSHOOT_M:
        return "FAIL_BRANCH_PROGRESS_OVERSHOOT"
    if not BRANCH_SPEED_MPS[0] <= speed_mps <= BRANCH_SPEED_MPS[1]:
        return "FAIL_BRANCH_SPEED_OUTSIDE_MATCHED_WINDOW"
    return "START"


def safety_failures(*, frame: int, previous_frame: int | None, vx: float,
                    cte_m: float, collision_count: int, lane_event_count: int,
                    traffic_light_hazard: bool, alignment_ok: bool, free_bytes: int) -> list[str]:
    failures = []
    if previous_frame is not None and frame != previous_frame + 1:
        failures.append("native_frame_discontinuity")
    if not math.isfinite(vx) or not math.isfinite(cte_m):
        failures.append("nonfinite_state")
    if math.isfinite(vx) and abs(vx) > 2.7:
        failures.append("maximum_speed")
    if math.isfinite(cte_m) and abs(cte_m) > 1.0:
        failures.append("route_cte")
    if collision_count or lane_event_count:
        failures.append("observed_event")
    if traffic_light_hazard:
        failures.append("traffic_light_hazard")
    if not alignment_ok:
        failures.append("control_alignment")
    if type(free_bytes) is not int or free_bytes < FLOOR_BYTES + CLOSING_RESERVE_BYTES:
        failures.append("disk_guard")
    return failures


def warmup_final_failures(*, vx: float, bootstrap_progress_m: float,
                          route_progress_m: float, planar_displacement_m: float) -> list[str]:
    """HH_261001 - Separate absolute route-start corridor from relative stationary drift."""
    values = (vx, bootstrap_progress_m, route_progress_m, planar_displacement_m)
    if any(not math.isfinite(value) for value in values):
        return ["nonfinite_warmup_final_state"]
    failures = []
    if abs(vx) > 0.05:
        failures.append("warmup_not_stationary")
    if not 0.0 <= bootstrap_progress_m <= 0.5:
        failures.append("warmup_bootstrap_outside_start_corridor")
    if not 0.0 <= route_progress_m <= 0.5:
        failures.append("warmup_final_outside_start_corridor")
    if abs(route_progress_m - bootstrap_progress_m) > 0.25:
        failures.append("warmup_progress_drift")
    if not 0.0 <= planar_displacement_m <= 0.25:
        failures.append("warmup_planar_displacement")
    return failures


def free_bytes() -> int:
    return shutil.disk_usage(ROOT).free
