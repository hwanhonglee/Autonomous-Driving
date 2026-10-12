"""HH_260906 - Declare a separate Town03 DEV moving task while preserving the sealed Town07 implementation."""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from scripts.e2e import carla_moving_capture as legacy

ROOT = Path(__file__).resolve().parents[2]
CAPTURE_PATH = "artifacts/training/2026-09-16/moving_dev_capture_v2/01_town03_straight/episode"
PLAN_PATH = "artifacts/training/2026-09-16/moving_dev_capture_v2/dev_plan.json"
ROUTE_PATH = "artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town03/catalog/straight/routes/town03/straight/town03_straight_s0000_p07.json"
ROUTE_SHA = "8fc442ed6cc6f6e40a4f346c2cfbda26105bd5a53c0992f7c6eb78964838cf06"
SELECTION_PATH = "artifacts/training/2026-09-16/moving_dev_route_resolution_v1/route_resolution.json"
SELECTION_SHA = "52a07d0f25e2ff8ab0ad13956dfbe9e403235ae87b3673712f5ee46daf3d929f"
SOURCE_PATHS = (
    "scripts/e2e/collect_carla_vad_expert.py", "scripts/e2e/collect_carla_moving_dev_v2.py",
    "scripts/e2e/carla_moving_capture.py", "scripts/e2e/carla_moving_capture_dev_v2.py",
    "scripts/e2e/carla_goal_stop_profile.py", "scripts/e2e/carla_physics_substeps.py",
    "scripts/e2e/run_owned_carla_moving_dev_v2.sh",
)
FIXED_ARGS = dict(legacy.FIXED_ARGS, seed=201)
require, canonical, sha, read, encoded = legacy.require, legacy.canonical, legacy.sha, legacy.read, legacy.encoded
CAMERAS = legacy.CAMERAS

# HH_260906 - Reuse the unchanged native observation, projection, physics and pending-QA semantics, not old plan dispatch.
MovingTask = legacy.MovingTask
pending_scope = legacy.pending_scope
physics_snapshot = legacy.physics_snapshot
validate_original_physics = legacy.validate_original_physics
validate_applied_physics = legacy.validate_applied_physics
measured_progress = legacy.measured_progress
reconstruct = legacy.reconstruct


def plan_contract():
    return legacy.plan_contract() | dict(schema="carla.moving_task_capture_plan.v2",
        task_id="v03_s01_moving_15kph_dev_v2", case_id="01_town03_straight", town="Town03",
        route_sha256=ROUTE_SHA, fixed_arguments=FIXED_ARGS, dataset_split_intent="val",
        cohort_slot_id="V03-S01", route_id="town03_straight_s0000_p07",
        source_session_intent="session:carla_dev_v03_s01_p07_20260916_v2",
        release_test=False, independence_claim=False,
        route_selection_evidence=dict(path=SELECTION_PATH, sha256=SELECTION_SHA),
        implementation="Versioned DEV driver delegates native capture to byte-pinned legacy code with an explicit process-local plan dispatcher.")


def selection_evidence():
    path = canonical(ROOT/SELECTION_PATH)
    require(path.is_file() and path.stat().st_size <= 16*1024*1024 and sha(path) == SELECTION_SHA,
            "DEV route-selection receipt changed or missing")
    report = read(path)
    require(report.get("schema") == "carla.dev_route_overlap_resolution.v1"
            and report.get("status") == "GEOMETRY_ROUTE_SELECTED_NOT_CAPTURE_AUTHORIZED"
            and report.get("proposed_split") == "val", "DEV geometry selection required")
    selected = report.get("selected_candidate", {})
    require(selected.get("route_id") == "town03_straight_s0000_p07"
            and selected.get("route_sha256") == ROUTE_SHA
            and canonical(selected.get("route_path", "")) == ROOT/ROUTE_PATH, "DEV selected route identity differs")
    require(report.get("execution_allowed") is False and report.get("dataset_admission_approved") is False,
            "geometry receipt must not claim capture or admission")
    return report


def build_plan(output, route_path, mapping_path, calibration_path, declared_at_utc=None):
    selection_evidence()
    paths = {key: canonical(value) for key, value in dict(route_file=route_path, mapping=mapping_path, calibration=calibration_path).items()}
    target = canonical(output)
    require(target == ROOT/CAPTURE_PATH, "DEV output must match declared case")
    require(paths["route_file"] == ROOT/ROUTE_PATH and sha(paths["route_file"]) == ROUTE_SHA, "DEV route pin differs")
    route = read(paths["route_file"])
    require(route.get("town") == "Town03" and route.get("scenario") == "straight", "DEV route identity differs")
    require(all(path.is_relative_to(ROOT) for path in paths.values()), "DEV inputs must be contained in workspace")
    return plan_contract() | dict(declared_at_utc=declared_at_utc or datetime.now(timezone.utc).isoformat(),
        output=target.relative_to(ROOT).as_posix(), input_paths={key: path.relative_to(ROOT).as_posix() for key, path in paths.items()},
        input_sha256={key: sha(path) for key, path in paths.items()}, source_sha256={name: sha(ROOT/name) for name in SOURCE_PATHS})


def configuration_from_args(args):
    # HH_260906 - The DEV entrypoint has no flagless fallback to the historical TRAIN collector.
    path, expected = getattr(args, "moving_task_plan", None), getattr(args, "moving_task_plan_sha256", None)
    require(path is not None and type(expected) is str and len(expected) == 64
            and set(expected) <= set("0123456789abcdef"), "DEV requires paired prospective plan and SHA flags")
    path = canonical(path)
    require(path.stat().st_size <= 16*1024*1024 and sha(path) == expected, "DEV prospective plan SHA differs")
    plan, fixed = read(path), plan_contract()
    require(set(plan) == set(fixed) | {"declared_at_utc", "output", "input_paths", "input_sha256", "source_sha256"}, "DEV plan fields differ")
    require(encoded({key: plan[key] for key in fixed}) == encoded(fixed), "DEV fixed plan contract differs")
    stamp = datetime.fromisoformat(plan["declared_at_utc"].replace("Z", "+00:00"))
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0 and stamp <= datetime.now(timezone.utc),
            "prospective DEV UTC declaration required")
    for key, value in FIXED_ARGS.items():
        actual = getattr(args, key, None)
        require(actual == value and (type(value) is not bool or type(actual) is bool), f"DEV requires {key}={value}")
    require(plan["output"] == CAPTURE_PATH and canonical(args.output) == ROOT/CAPTURE_PATH, "DEV output differs from plan")
    require(not path.is_relative_to(canonical(args.output)) and not path.is_relative_to(Path(str(canonical(args.output))+".partial")),
            "DEV plan must be outside native capture output")
    require(set(plan["source_sha256"]) == set(SOURCE_PATHS), "DEV executable source closure differs")
    for name, digest in plan["source_sha256"].items():
        require(sha(ROOT/name) == digest, "DEV executable source changed: "+name)
    require(set(plan["input_paths"]) == set(plan["input_sha256"]) == {"route_file", "mapping", "calibration"}, "DEV input closure differs")
    for key, relative in plan["input_paths"].items():
        source = canonical(ROOT/relative)
        require(source.is_relative_to(ROOT) and canonical(getattr(args, key)) == source
                and sha(source) == plan["input_sha256"][key], "DEV input differs: "+key)
    require(canonical(args.route_file) == ROOT/ROUTE_PATH and plan["input_sha256"]["route_file"] == ROUTE_SHA, "DEV route pin differs")
    selection_evidence()
    return plan


def collector_options(plan_path):
    # HH_260906 - Owner supplies host/port; serialize every other non-null fixed option without shell evaluation.
    options = ["--moving-task-plan", str(canonical(plan_path)), "--moving-task-plan-sha256", sha(plan_path)]
    for name, value in FIXED_ARGS.items():
        if name in ("host", "port") or value is None or value is False:
            continue
        options.append("--"+name.replace("_", "-"))
        if value is not True:
            options.append(str(value))
    return options


def main(argv=None):
    import argparse
    import json
    import os
    from scripts.e2e import collect_carla_vad_expert as collector
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--declare", required=True, type=Path)
    parser.add_argument("--capture-output", type=Path, default=ROOT/CAPTURE_PATH)
    parser.add_argument("--route-file", type=Path, default=ROOT/ROUTE_PATH)
    parser.add_argument("--mapping", type=Path, default=collector.DEFAULT_MAPPING)
    parser.add_argument("--calibration", type=Path, default=collector.DEFAULT_CALIBRATION)
    args = parser.parse_args(argv)
    target, output = canonical(args.declare), canonical(args.capture_output)
    require(target == ROOT/PLAN_PATH and not target.exists(), "fresh fixed DEV declaration required")
    require(not output.parent.exists() and not output.exists() and not Path(str(output)+".partial").exists(),
            "one fresh DEV owner directory required before declaration")
    plan = build_plan(output, args.route_file, args.mapping, args.calibration)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as stream:
        stream.write(encoded(plan)); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(dict(plan=str(target), sha256=sha(target), output=str(output),
        schema=plan["schema"], split_intent="val", capture_started=False, dataset_admission_approved=False)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
