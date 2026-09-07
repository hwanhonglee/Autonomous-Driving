#!/usr/bin/env python3
"""HH_260906 - Compare recorded wheel locations with the declared virtual base frame without changing calibration."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from scripts.e2e import summarize_carla_low_speed_response as pedal
from scripts.e2e import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
PRIMARY_FILES = {
    "carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp": "1244cb0c6d64e619cb34898a4b8976bc2e28c8c6748efabb7ec5eac46c341c44",
    "carla/LibCarla/source/carla/rpc/WheelPhysicsControl.h": "84415ec37c01fe647a1e2b5e852afe21534cda9ef1ccb6a1fcceea2481478a8c",
    "carla/LibCarla/source/carla/rpc/VehiclePhysicsControl.h": "cebdb6ee0ac7bfb6d4924379cdd9260cd2d867a82069e7126794daa3e4689ee0",
    "carla/LibCarla/source/carla/geom/Location.h": "d147b89ce788fcba306a38fc5db94089e218d366f0a1fa868623621bef42a36b",
    "carla/LibCarla/source/carla/geom/Rotation.h": "7d373ddb222fb448faf7570825cf0ec29133e1ccbe3e2fa626e2bc6f13417357",
    "unreal/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Private/VehicleWheel.cpp": "4d2b36e5d897e47ff416df15d07e7ab7fe4ecf22f64c6ae1d8cc1953d51ebdce",
    "unreal/Engine/Plugins/Runtime/PhysXVehicles/Source/PhysXVehicles/Public/VehicleWheel.h": "9b9a8cd14e1d529781952f93e24a7eec7c8bd58ac94297412e090657b2b790b6",
}


def inverse_rotate(vector, pose):
    """HH_260906 - Apply the transpose of the documented CARLA rotation; input and output retain metres."""
    x, y, z = [base.number(value) for value in vector]
    pitch, yaw, roll = [math.radians(base.number(pose[name])) for name in ("pitch", "yaw", "roll")]
    cp, sp, cy, sy, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    return [cp * cy * x + cp * sy * y + sp * z,
            (cy * sp * sr - sy * cr) * x + (sy * sp * sr + cy * cr) * y - cp * sr * z,
            (-cy * sp * cr - sy * sr) * x + (-sy * sp * cr + cy * sr) * y + cp * cr * z]


def analyze_case(report):
    """HH_260906 - Wheel positions are world centimetres in inspected code, not local metre offsets or a COM measurement."""
    pose = report["actor_center_spawn_carla"]
    for name in ("x", "y", "z", "pitch", "yaw", "roll"):
        base.number(pose[name])
    values = report["vehicle_physics"]["values"]
    wheels = values["wheels"]
    base.require(len(wheels) == 4, "four recorded wheel observations required")
    local = [inverse_rotate([base.number(wheel["position"][name]) / 100.0 - pose[name]
                             for name in ("x", "y", "z")], pose) for wheel in wheels]
    base.require(all(base.number(wheel["max_steer_angle"]) > 0 for wheel in wheels[:2])
                 and all(base.number(wheel["max_steer_angle"]) == 0 for wheel in wheels[2:]),
                 "recorded steering configuration does not identify the reviewed front/rear index pairs")
    base.require(all(0 < point[0] < 3 for point in local[:2]) and all(-3 < point[0] < 0 for point in local[2:])
                 and all(abs(point[1]) < 2 and abs(point[2]) < 2 for point in local),
                 "wheel units, origin or front/rear ordering inconsistent with reviewed spawn geometry")
    front, rear = ([sum(point[axis] for point in pair) / 2 for axis in range(3)]
                   for pair in (local[:2], local[2:]))
    declared_base_x = -2.85 / 2
    return {"case_id": report["case"]["case_id"], "actor_id": report["actor_id"],
        "declared_actor_spawn_carla": pose, "wheel_positions_world_cm": [wheel["position"] for wheel in wheels],
        "wheel_positions_relative_to_declared_spawn_m": local,
        "front_pair_midpoint_relative_to_declared_spawn_m": front,
        "rear_pair_midpoint_relative_to_declared_spawn_m": rear,
        "front_to_rear_midpoint_distance_m": math.dist(front, rear),
        "longitudinal_front_to_rear_separation_m": front[0] - rear[0],
        "front_track_distance_m": math.dist(local[0], local[1]),
        "rear_track_distance_m": math.dist(local[2], local[3]),
        "declared_virtual_base_x_m": declared_base_x,
        "declared_base_minus_rear_midpoint_x_m": declared_base_x - rear[0],
        "declared_wheelbase_minus_observed_separation_m": 2.85 - (front[0] - rear[0]),
        "reported_center_of_mass_parameter": values["center_of_mass"],
        "measurement_notice": "Physics was queried immediately after spawn, before the first recorded tick. Declared spawn is not an independently observed same-frame actor pose; midpoint is not a ground-contact/no-slip point or measured COM."}


def reference_identity(cases, physics_identical):
    """HH_260906 - Verify spawn equality separately; equal physics never implies an equal declared origin."""
    base.require(cases and type(physics_identical) is bool, "nonempty cases and explicit physics identity required")
    same_spawn = all(case["declared_actor_spawn_carla"] == cases[0]["declared_actor_spawn_carla"] for case in cases)
    return {"all_cases_same_declared_spawn": same_spawn, "all_cases_same_physics": physics_identical,
            "all_cases_same_spawn_and_physics": same_spawn and physics_identical}


def run(trial_root, output, carla_root, unreal_root):
    """HH_260906 - Verify all twelve existing measurements and source bytes before writing a separate diagnostic."""
    trial_root, output = Path(trial_root).resolve(), Path(output)
    base.require(not output.exists() and not output.is_symlink()
                 and not output.resolve().is_relative_to(trial_root), "fresh output must be outside source trial")
    roots = {"carla": Path(carla_root), "unreal": Path(unreal_root)}
    primary = {}
    for name, expected in PRIMARY_FILES.items():
        group, relative = name.split("/", 1)
        path = roots[group] / relative
        base.require(path.is_file() and not path.is_symlink() and base.sha(path) == expected, "inspected primary source changed")
        primary[name] = path
    code = {name: base.sha(ROOT / name) for name in (
        "scripts/e2e/audit_carla_wheel_reference.py", "scripts/e2e/summarize_carla_low_speed_response.py",
        "scripts/e2e/summarize_carla_goal_stop_trials.py")}
    verified, _ = pedal.verify_trial(trial_root)
    base.require(verified["planned_and_included_case_count"] == 12, "this review is fixed to all twelve original pedal cases")
    cases, ledger = [], []
    for case in verified["cases"]:
        report = base.read_file(trial_root, f"actuation/{case['case_id']}/report.json", ledger)
        cases.append(analyze_case(report))
    for entry in verified["source_manifest"] + ledger:
        base.require(base.sha(trial_root / entry["path"]) == entry["sha256"], "raw source changed during wheel review")
    for name, path in primary.items():
        base.require(base.sha(path) == PRIMARY_FILES[name], "primary source changed during wheel review")
    base.require(all(base.sha(ROOT / name) == value for name, value in code.items()), "diagnostic source changed")
    result = {"schema": "carla.declared_wheel_reference_diagnostic.v1", "status": "DIAGNOSED_CALIBRATION_UNCHANGED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "cases": cases,
        "included_case_count": 12, **reference_identity(cases, verified["vehicle_physics_identical_across_all_cases"]),
        "source_sha256": code, "inspected_primary_source_sha256": PRIMARY_FILES,
        "source_manifest": verified["source_manifest"], "original_measurement_status": verified["status"],
        "scope": {"new_simulator_access": False, "model_loaded": False, "training": False, "calibration_changed": False,
            "dataset_admission": False, "executed_carla_or_unreal_binary_source_identity_proven": False,
            "same_frame_post_settle_wheel_pose_measured": False, "physical_velocity_reference_point_proven": False},
        "findings": [
            "Inspected CARLA source returns UE wheel world-space Location unchanged through Vector3D; unlike geom::Location, this field has no centimetre-to-metre conversion.",
            "Inspected VehiclePhysicsControl center_of_mass is COMNudge converted through geom::Location; this parameter alone is not an observed absolute world-space COM.",
            "The declared virtual base shift is a coordinate convention and is not automatically the exact physical rear wheel midpoint.",
            "Twelve identical spawn/physics records are repeated observations of one condition, not twelve independently calibrated vehicles."],
        "next_required_measurement": "Measure immutable actor pose and wheel geometry after settling in a stopped owned world, bind the RPC bracket to one frame, and verify velocity/acceleration reference semantics before proposing any coordinated TF/label/runtime change."}
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text(f"{base.sha(output / 'summary.json')}  summary.json\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("trial_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--carla-source-root", type=Path, required=True)
    parser.add_argument("--unreal-source-root", type=Path, required=True)
    args = parser.parse_args(argv)
    result = run(args.trial_root, args.output_dir, args.carla_source_root, args.unreal_source_root)
    print(json.dumps({"status": result["status"], "included_case_count": result["included_case_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
