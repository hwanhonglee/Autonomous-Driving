"""HH_261001 - New smaller actual recovery cohort; preserve every frozen v1 outcome and guard."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import math
from pathlib import Path
import sys

from scripts.e2e import run_owned_town07_signed_recovery_20261001 as previous
from scripts.e2e import town07_signed_recovery_task_20261001 as task

ROOT = previous.ROOT
MODULE = "scripts.e2e.run_owned_town07_signed_recovery_v2_20261001"
TASK_SCHEMA = "carla.actual_signed_teacher_recovery.20261001.v2"
SCHEMA = TASK_SCHEMA + ".owned_batch"
CASES = tuple(dict(case_id=f"train_{i:02d}", split="train", seed=2026100310+i,
    signed_lateral_offset_ros_m=lateral, signed_yaw_offset_ros_deg=yaw)
    for i, (lateral, yaw) in enumerate((lateral, yaw) for lateral in (-.20, 0., .20) for yaw in (-1.5, 0., 1.5))) + (
    dict(case_id="holdout_00", split="dev", seed=2026100410, signed_lateral_offset_ros_m=-.15, signed_yaw_offset_ros_deg=1.),
    dict(case_id="holdout_01", split="dev", seed=2026100411, signed_lateral_offset_ros_m=.15, signed_yaw_offset_ros_deg=-1.))
CENTER_MANIFEST = ROOT / "storage/experiments/validation/2026-10-01/town07_signed_teacher_recovery_v1/train_04/episode/manifest.json"
CENTER_MANIFEST_SHA = "999fa2a7f639521aebd005f2107f3047ccb6a305ac75f0ee3e35b308036d414b"
FROZEN_V1 = {
    "tools/e2e/town07_signed_recovery_task_20261001.py": "fddd47336e5a914f89db60dc7dccfa1dd613c3917125b213754de86a83958ecb",
    "tools/e2e/collect_town07_signed_recovery_20261001.py": "8260f04a704394f1406cbfc5956fac678766ff20932b0621e35f4162f4883c3e",
    "tools/e2e/run_owned_town07_signed_recovery_20261001.py": "d452e3901e42ced1af894e1ef329b6142ed10860de02cc0a4a2512f0022f500b"}
_fixed_contract, _runtime_sources = task.fixed_contract, previous.runtime_sources
_declare_batch = previous.declare_batch
_active = False
_capture_manifest = None
LAUNCH_THROTTLE, POST_HANDOFF_SLEW, POST_HANDOFF_CAP = .11, .025, .20


class SmoothRecoveryLaunchControl(previous.worker_module.v2.CombinedControl):
    """HH_261001 - Prospectively lower only teacher normal launch/ramp; retain measured state and hazard authority."""
    def apply(self, control, *, speed, timestamp, frame, normal_proposal):
        if float(control.brake) != 0.:
            raise ValueError("normal planner braking must already be capped to zero")
        if not math.isfinite(speed) or not math.isfinite(timestamp) or speed < 0:
            raise ValueError("finite nonnegative actual planar speed and native time required")
        if self.previous_time is not None and timestamp < self.previous_time:
            raise ValueError("native launch clock moved backwards")
        self.previous_time = timestamp
        if self.started_at is None: self.started_at = timestamp
        if self.handoff_at is None and speed >= .5: self.handoff_at = timestamp
        if self.handoff_at is None and timestamp-self.started_at >= 8.:
            raise ValueError("fixed low-speed launch did not reach 0.5 m/s within 8 seconds")
        before = previous.worker_module.base.control_dict(control)
        cap = LAUNCH_THROTTLE if self.handoff_at is None else min(POST_HANDOFF_CAP,
            LAUNCH_THROTTLE+POST_HANDOFF_SLEW*(timestamp-self.handoff_at+.05))
        if self.handoff_at is None: control.throttle, control.brake = LAUNCH_THROTTLE, 0.
        else: control.throttle = min(float(control.throttle), cap)
        before_steering = previous.worker_module.base.control_dict(control)
        low_speed_steering = speed <= .75
        if low_speed_steering: control.steer = 0.
        self.last = dict(schema=TASK_SCHEMA+".smooth_normal_teacher_control", source_frame=int(frame),
            source_timestamp=timestamp, measured_planar_speed_mps=speed, handoff_latched=self.handoff_at is not None,
            launch_started_at=self.started_at, handoff_at=self.handoff_at, normal_throttle_cap=cap,
            raw_normal_planner_proposal=normal_proposal, revised_normal_capped_proposal=before,
            launch_control_before_steering_intervention=before_steering,
            low_speed_steering_intervention=low_speed_steering, low_speed_zero_steering_mps=.75,
            profile_normal_control=previous.worker_module.base.control_dict(control), normal_brake_cap=0.,
            insertion="LocalPlanner normal request before unchanged BasicAgent emergency override",
            normal_launch_controls_inherited_v2=False, launch_throttle=LAUNCH_THROTTLE,
            post_handoff_throttle_ramp_per_second=POST_HANDOFF_SLEW, post_handoff_cap=POST_HANDOFF_CAP,
            actual_observed_speed_or_acceleration_clipped=False, native_acceleration_qa_limit_mps2=3.,
            training_data_approved=False, learned_control=False)
        annotate_manifest(_capture_manifest)
        return control


def annotate_manifest(manifest):
    # HH_261001 - Report the new teacher calibration truthfully before ACK and final write; no observed pixels/poses change.
    if manifest is None: return
    manifest["control_source"] = "Disclosed_BasicAgent_signed_recovery_teacher_smaller_grid_smooth_launch_v2"
    metadata = manifest.setdefault("capture_contract", {}).setdefault("teacher_controller", {})
    metadata.update(normal_launch_controls_inherited_v2=False, normal_launch_controller_revision=TASK_SCHEMA+".smooth_normal_teacher_control",
        launch_throttle=LAUNCH_THROTTLE, launch_handoff_mps=.5, maximum_launch_seconds=8.,
        post_handoff_throttle_ramp_per_second=POST_HANDOFF_SLEW, post_handoff_cap=POST_HANDOFF_CAP,
        normal_brake_cap=0., low_speed_zero_steering_mps=.75, inherited_BasicAgent_emergency_unchanged=True,
        measured_speed_clipped=False, measured_acceleration_clipped=False, native_acceleration_qa_limit_mps2=3.,
        launch_clock="actual world snapshot time", launch_speed_source="actual CARLA planar actor velocity",
        prospective_calibration_not_acceleration_admission_claim=True)


def verify_frozen_v1():
    for name, sha in FROZEN_V1.items():
        previous.require(previous.digest(ROOT/name) == sha, "frozen v1 recovery source changed: "+name)


def fixed_contract(run_id, case_id):
    previous.require(run_id == "v2", "this new cohort only owns fresh run-id v2")
    result = _fixed_contract(run_id, case_id)
    result["controller"] = dict(result["controller"], launch_throttle=LAUNCH_THROTTLE,
        post_handoff_throttle_ramp_per_second=POST_HANDOFF_SLEW, post_handoff_cap=POST_HANDOFF_CAP,
        normal_launch_controls_inherited_v2=False, native_acceleration_qa_limit_mps2=3.)
    result["teacher"] = "BasicAgent lateral/original hazard override; NEW disclosed constant0.11 launch and0.025/s normal post-handoff pedal cap; no native acceleration/speed clipping."
    result.update(recovery_cohort_revision="v2_smaller_actual_spawn_full_factorial_not_v1_subset",
        initial_body_preflight=dict(method="Pinned actual Prius box / archived Town07 route+XODR; two identical predicted static footprints and bounded sweep.",
            manifest_path=str(CENTER_MANIFEST), manifest_sha256=CENTER_MANIFEST_SHA,
            all_eleven_initial_boxes_raw_and_10cm_inset_required=True,
            simulated_or_observed_data=False, actual_full_capture_body_and_sweeps_still_required=True),
        previous_failed_grid_preserved=True, old_successful_subset_not_used_as_v2_training=True)
    return result


def runtime_sources():
    # HH_261001 - Pin deterministic import-only closure; no mutable directory glob or sys.modules enumeration.
    verify_frozen_v1()
    pins = _runtime_sources()
    for path in previous.lifecycle.imported_source_closure((Path(__file__).resolve(),)):
        pins[str(path.relative_to(ROOT))] = previous.digest(path)
    previous.require(all(pins.get(name) == sha for name, sha in FROZEN_V1.items()), "v1 dependency closure changed")
    return dict(sorted(pins.items()))


def initial_body_preflight():
    """HH_261001 - Check prospective static spawn geometry only; never make synthetic driving rows or labels."""
    previous.require(_active, "explicit v2 scoped context required")
    verify_frozen_v1()
    assets = dict(previous.ASSETS)
    previous.require(assets.get(CENTER_MANIFEST) == CENTER_MANIFEST_SHA
        and all(previous.digest(path) == sha for path, sha in assets.items()), "initial-body asset identity differs")
    manifest, route = previous.read_json(CENTER_MANIFEST), previous.read_json(task.ROUTE)
    previous.require(manifest["status"] == "complete" and manifest["coordinate_contract"]["wheelbase_m"] == 2.85,
        "complete same-rig center fixture required for recorded Prius box")
    nominal = previous.worker_module.base.shift_transform_local_x(route["start_carla_transform"], 2.85/2)
    original_spawn = manifest["capture_contract"]["actual_signed_spawn"]
    previous.require(original_spawn["nominal_actor_center_carla"] == nominal
        and original_spawn["signed_lateral_offset_ros_m"] == original_spawn["signed_yaw_offset_ros_deg"] == 0.,
        "reference fixture must be the actual centered original route/rig capture")
    actor = manifest["capture_contract"]["actor_body_geometry"]["actor_id"]
    xml = previous.XODR.read_bytes()
    saved_path, results = list(sys.path), []
    try:
        sys.path.insert(0, str(previous.EGG))
        import carla
        previous.require(str(carla.__file__).startswith(str(previous.EGG)+"/"), "foreign offline CARLA map API")
        for spec in CASES:
            center, rear = task.perturb_actor_center(nominal, wheelbase_m=2.85,
                signed_lateral_offset_ros_m=spec["signed_lateral_offset_ros_m"],
                signed_yaw_offset_ros_deg=spec["signed_yaw_offset_ros_deg"])
            spawn = previous.worker_module.base.apply_spawn_z_offset(center, .5)
            # HH_261001 - Identical predicted poses permit the existing stationary sweep guard; neither is observed TRAIN data.
            predicted = [dict(frame=i, timestamp=i*.05, capture_phase="stationary_warmup",
                actor_snapshot_transform_carla=dict(spawn)) for i in (0, 1)]
            rows, census = previous.geometry.body_census(manifest, route, predicted, xml, actor)
            passed = all(value["status"] == "PASS" and value["assessed_count"] == (1 if name.startswith("previous_sweep") else 2)
                for name, value in census.items())
            results.append(dict(**spec, status="PASS" if passed else "FAIL", nominal_actor_center_carla=nominal,
                predicted_rear_reference_carla=rear, predicted_actor_spawn_carla=spawn, census=census,
                predicted_polygon_carla_xy=rows[0]["body_polygon_carla_xy"], recorded_geometry_reference_actor_id=actor,
                actual_capture_actor_identity_claimed=False, synthetic_training_rows_created=False))
    finally:
        sys.path[:] = saved_path
    previous.require(all(previous.digest(path) == sha for path, sha in assets.items()), "body preflight asset changed during audit")
    previous.require(len(results) == 11 and all(row["status"] == "PASS" for row in results), "prospective v2 initial raw/inset bbox failed; do not declare")
    return dict(schema=SCHEMA+".predicted_initial_geometry", status="PASS", all_eleven_initial_boxes_clear=True,
        cases=results, source_asset_sha256={str(path): sha for path, sha in assets.items()},
        method="OFFLINE_PREDICTED_STATIC_FOOTPRINT_ONLY_NOT_ACTUAL_CAPTURE", CARLA_server_started=False,
        actual_903_native_boxes_and_902_sweeps_required_per_case=True, training_data_approved=False,
        source_v1_outcomes_rewritten=False)


def declare_batch(run_id):
    previous.require(run_id == "v2", "only the new v2 dataset may be declared")
    # HH_261001 - All initial geometries are checked before any case declaration or possible simulator launch.
    geometry = initial_body_preflight()
    result = _declare_batch(run_id)
    output, _ = task.paths(run_id)
    previous.write_new(output/"predicted_initial_geometry.json", geometry)
    return dict(result, predicted_initial_geometry_sha256=previous.digest(output/"predicted_initial_geometry.json"))


@contextmanager
def activated():
    """HH_261001 - Full restoration lets independent qualification replay the v2 task without changing frozen modules."""
    global _active, _capture_manifest
    previous.require(not _active and not previous._active, "v2 recovery activation cannot nest an owner/case")
    verify_frozen_v1()
    task_changes = dict(CASES=CASES, SCHEMA=TASK_SCHEMA, fixed_contract=fixed_contract)
    owner_changes = dict(MODULE=MODULE, SCHEMA=SCHEMA, CASE_IDS=tuple(row["case_id"] for row in CASES),
        runtime_sources=runtime_sources, declare_batch=declare_batch,
        ASSETS=dict(previous.ASSETS) | {CENTER_MANIFEST: CENTER_MANIFEST_SHA})
    old_task = {key: getattr(task, key) for key in task_changes}
    old_owner = {key: getattr(previous, key) for key in owner_changes}
    old_control = previous.worker_module.v2.CombinedControl
    old_collect = previous.worker_module.base.collect_episode
    old_manifest = _capture_manifest
    def collect(*positional, **kwargs):
        global _capture_manifest
        _capture_manifest = positional[6]
        annotate_manifest(_capture_manifest)
        try: return old_collect(*positional, **kwargs)
        finally:
            annotate_manifest(_capture_manifest)
            _capture_manifest = old_manifest
    _active = True
    try:
        for key, value in task_changes.items(): setattr(task, key, value)
        for key, value in owner_changes.items(): setattr(previous, key, value)
        previous.worker_module.v2.CombinedControl = SmoothRecoveryLaunchControl
        previous.worker_module.base.collect_episode = collect
        yield
    finally:
        for key, value in old_owner.items(): setattr(previous, key, value)
        for key, value in old_task.items(): setattr(task, key, value)
        previous.worker_module.v2.CombinedControl = old_control
        previous.worker_module.base.collect_episode = old_collect
        _capture_manifest = old_manifest
        _active = False


def main(argv=None):
    values = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--run-id", default="v2")
    ours, _ = parser.parse_known_args(values)
    previous.require(ours.run_id == "v2", "old v1 and arbitrary dataset paths must never be reused by this wrapper")
    if "--run-id" not in values and not any(value.startswith("--run-id=") for value in values):
        values += ["--run-id", "v2"]
    with activated(): return previous.main(values)


if __name__ == "__main__": raise SystemExit(main())
