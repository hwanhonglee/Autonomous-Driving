"""HH_261004 - Own one fresh fixed45 learned Town03 known-DEV straight screen without promotion."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import math
from pathlib import Path
import re
import sys

import numpy as np

from . import research_temporal_route_joint_town03_right_20261002 as previous
from . import research_temporal_route_joint_town03_right_v2_20261002 as fixture_owner
from . import research_temporal_route_joint_town03_right_v3_20261002 as runtime_owner
from . import collect_temporal_route_joint_town03_straight_20261004 as live


base, numeric, ROOT, CARLA_ROOT = previous.base, previous.numeric, previous.ROOT, previous.CARLA_ROOT
require, digest, read_json = previous.require, previous.digest, previous.read_json
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_20261004"
SCHEMA = live.OWNER_SCHEMA
ROUTE = ROOT / ("storage/experiments/validation/2026-09-01/"
    "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town03/catalog/straight/"
    "routes/town03/straight/town03_straight_s0000_p07.json")
XODR, UMAP = previous.XODR, previous.UMAP
XODR_SHA, UMAP_SHA = previous.XODR_SHA, previous.UMAP_SHA
RIGHT_V3_OWNER_SHA = "0a545281db1c54ec844347f95d3a7444a1ec4bd305eaacb132886f47ff655623"
_right_fixed = runtime_owner.fixed_contract
_right_sources = runtime_owner.runtime_sources
_right_diagnostic = runtime_owner.diagnostic_report
_active = False


def paths(run_id):
    require(run_id == "v1", "fresh Town03 straight one-shot requires exact v1")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v1"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    require(longitudinal_pi is True and previous._pins is not None,
            "externally pinned final6000/PI endpoint required")
    output, _ = paths(run_id)
    return ["--research-weights", str(numeric.WEIGHTS), "--research-metadata", str(numeric.METADATA),
        "--research-weights-sha256", previous._pins["weights_sha256"],
        "--research-metadata-sha256", previous._pins["metadata_sha256"],
        "--research-source-sha256", declaration_sha, "--research-longitudinal-pi",
        str(output / "episode"), str(ROUTE), "--host", "127.0.0.1", "--port", "2440",
        "--control-transport", "acknowledged_batch", "--agent-initialization", "after_bootstrap",
        "--stationary-warmup-sec", "1.2", "--stationary-tail-sec", "0",
        "--max-duration-sec", "45", "--target-speed-kmh", "9", "--seed", str(live.SEED),
        "--weather", "ClearNoon", "--mapping", str(base.MAPPING), "--calibration", str(base.CALIBRATION),
        "--spawn-z-offset-m", "0.5", "--wheelbase-m", "2.85",
        "--expected-live-xodr-sha256", XODR_SHA, "--goal-stop-profile", "disabled",
        "--observe-traffic-lights"]


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Preserve every GOOD6000 driving gate and replace only turn/event identity with fixed45 straight."""
    result = _right_fixed(run_id, longitudinal_pi)
    for name in ("event", "event_is_not_full_route_stop_RTE03_or_MAIN9_approval", "full_RTE03_approved",
                 "full_failed_tail_until_total110s_unless_original_capture_safety_abort",
                 "v1_output_preserved", "v1_hazard_failure_is_not_a_neural_turn_failure",
                 "v1_and_teacher_only_v2_failures_preserved"):
        result.pop(name, None)
    screen = result["precursor_motion_screen"]
    require(screen["net_learned_progress_minimum_m"] == 90.0
        and screen["unintended_stationary_dwell_strictly_below_s"] == 1.0
        and screen["cte_p95_maximum_m"] == 0.35 and screen["cte_absolute_maximum_m"] == 0.75
        and screen["collisions_maximum"] == screen["lane_invasions_maximum"] == 0,
        "inherited GOOD6000 motion thresholds changed")
    result.update(schema=SCHEMA, output=str(paths(run_id)[0]), declaration_path=str(paths(run_id)[1]),
        town="Town03", scenario="straight", seed=live.SEED, dataset_split="val",
        known_DEV_site_reused=True, independent_validation=False,
        experiment="One fixed45 centered learned straight interval on the known-DEV Town03 p07 route",
        total_capture_seconds=45, stationary_tail_seconds=0,
        maximum_native_state_count=900, maximum_camera_bundle_count=450,
        maximum_driving_native_ticks=876, maximum_warmup_native_ticks=24,
        full_original_navigation_points=live.ROUTE_POINTS,
        original_navigation_sha256=live.ROUTE_SHA, original_catalog_route_length_m=live.ROUTE_LENGTH,
        centered_lateral_offset_ros_m=0.0, centered_yaw_offset_ros_deg=0.0,
        matched_Town07_centered_seed=live.SEED,
        model_controller_sensor_timing_and_start_offset_unchanged=True,
        changed_site_bundle="Town03 map + p07 route + required owned Green signal fixture",
        fixed45_full_failed_budget_tail_required=True, early_metric_success_forbidden=True,
        event_completion_used_for_success=False, attempts=1, automatic_retry=False,
        capture_argv=capture_argv(run_id, True), owner_wall_timeout_s=600.0,
        server_command=["bash", "tools/e2e/run_carla_map.sh", "Town03", "--port", "2440",
            "--quality", "Epic", "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        worker_command=[sys.executable, "-B", str(ROOT / "tools/workspace_python.py"), "-m", MODULE, "--",
            "--mode", "worker", "--run-id", run_id, "--plan-sha256", base.PLAN_TOKEN]
            + [value for name in ("weights", "metadata", "checkpoint", "declaration")
               for value in ("--expected-" + name + "-sha256", previous._pins[name + "_sha256"])],
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    result["sensor_and_ACK_screen"].update(native_state_count=900, camera_bundle_count=450,
        maximum_native_state_count=900, maximum_camera_bundle_count=450)
    result["body_clearance_screen"] = dict(all_native_boxes="all actual900",
        all_snapshot_linear_sweeps="all actual899", lane_inset_m=0.1)
    result["additive_native_speed_rate_guard_mps2"] = dict(minimum=-6.0, maximum=3.0,
        warmup_to_first_driving_interval_included=True)
    fixture = result["owned_green_signal_fixture"]
    fixture.pop("closed_course_basic_turn_only_not_traffic_rule_performance", None)
    fixture.pop("known_v1_hazard_subtype_proven", None)
    fixture.update(fixture_native_ticks_are_outside_the_45s_capture_and_not_NN_performance=True,
        closed_course_basic_straight_only_not_traffic_rule_performance=True)
    fixture.pop("fixture_native_ticks_are_outside_the_110s_capture_and_not_NN_performance", None)
    result["execute_exit_status_interpretation"] = (
        "operational ownership plus complete retained fixed45 interval; see straight safety/policy/wall outcome")
    require(result["model_speed_cap_mps"] == 2.5 and result["native_actor_planar_speed_maximum_mps"] == 2.7
        and result["policy_deadline_seconds"] == 0.1 and result["controller_target_cap_mps"] == 2.4
        and result["speed_preview_s"] == 0.5 and result["cpu_thread_numerical_preset"]["intra_op_threads"] == 4
        and result["cpu_thread_numerical_preset"]["inter_op_threads"] == 1,
        "GOOD6000 controller/thread/speed/latency contract changed")
    return result


def runtime_sources():
    require(digest(Path(runtime_owner.__file__)) == RIGHT_V3_OWNER_SHA,
            "reviewed actual runtime owner changed")
    pins = _right_sources()
    live.verify_frozen()
    for path in base.imported_source_closure((Path(__file__).resolve(), Path(live.__file__).resolve())):
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in pins or pins[name] == sha, "new straight source closure conflict")
        pins[name] = sha
    return pins


def complete_interval(manifest, states, cameras, _event=None):
    """HH_261004 - Accept only the complete fixed45 failed-budget denominator, never a favorable early prefix."""
    if len(states) != 900 or len(cameras) != 450:
        return False
    native = [row["frame"] for row in states]
    chain = native == list(range(native[0], native[0] + 900)) and [row["frame"] for row in cameras] == native[::2]
    warmup = [row for row in states if row["capture_phase"] == "stationary_warmup"]
    driving = [row for row in states if row["capture_phase"] == "driving"]
    schedule = manifest.get("capture_contract", {}).get("capture_phase_schedule", {})
    phase = len(warmup) == 24 and len(driving) == 876 and states[:24] == warmup and states[24:] == driving \
        and schedule.get("stationary_tail", {}).get("scheduled_ticks") == 0
    result = manifest.get("result", {})
    budget = manifest.get("status") == "failed" \
        and manifest.get("error") == "CollectionError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail" \
        and "goal_reached" not in result and result.get("state_count") == 900 \
        and result.get("camera_anchor_count") == 450
    return bool(chain and phase and budget)


def recomputed_straight_interval(route, states, commands):
    """HH_261004 - Recompute every driving row from retained states and prior learned ACK receipts."""
    observer = live.StraightIntervalObserver(route)
    learned = {row["ack_receipt_sequence"] for row in commands
               if row.get("application", {}).get("learned_control_applied") is True}
    by_frame = {row["command_frame"]: row for row in commands if "command_frame" in row}
    require(len(by_frame) == sum("command_frame" in row for row in commands),
            "unique outgoing normal decision frame required")
    driving = [row for row in states if row.get("capture_phase") == "driving"]
    for state in driving:
        sequence = state.get("control_transport", {}).get("expected_receipt_sequence")
        learned_effect = sequence in learned and state.get("control_transport", {}).get("status") == "PASS"
        outgoing = by_frame.get(state["frame"], {})
        observer.observe(state, learned_effect=learned_effect,
                         outgoing_safe=bool(outgoing) and outgoing.get("control_source") != "emergency_brake")
    summary = observer.summary()
    summary.update(completed=bool(driving) and len(observer.records) == len(driving),
        full_fixed_interval_observed=bool(driving) and len(observer.records) == len(driving),
        completion_is_denominator_only_not_performance=True)
    return summary, observer.records


def diagnostic_report(output, plan, owner):
    """HH_261004 - Reuse generic body/policy/fixture/runtime audits while removing every turn success claim."""
    result = _right_diagnostic(output, plan, owner)
    interval = result.pop("independent_event", {})
    passed = bool(result.pop("joint_turn_motion_body_policy_wall100ms_passed", False))
    result.pop("full_RTE03_approved", None)
    result.update(schema=SCHEMA + ".diagnostic_report",
        recomputed_straight_interval=interval,
        fixed45_full_denominator_passed=bool(result.get("bounded_capture_complete")),
        straight_motion_body_policy_wall100ms_passed=passed,
        learned_driving_screen_passed=bool(result.get("learned_driving_screen_passed")) and passed,
        known_DEV_site_reused=True, independent_validation=False,
        full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    result["status"] = "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR" if result["learned_driving_screen_passed"] \
        else "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    return result


def private_worker(run_id, expected):
    require(_active and numeric._owner_plan_sha is None, "exclusive Town03 straight private worker required")
    plan = base.verify_plan(run_id, expected)
    base.assert_owned_worker(plan, expected)
    numeric._owner_plan_sha = expected
    try:
        return previous._original_worker(run_id, expected)
    finally:
        numeric._owner_plan_sha = None


@contextmanager
def activated(pins):
    """HH_261004 - Route the fresh owner through existing lock/PID/socket/map seals and restore every alias."""
    global _active
    require(not _active and digest(Path(runtime_owner.__file__)) == RIGHT_V3_OWNER_SHA,
            "fresh straight owner cannot nest or use changed runtime ownership")
    changes = {
        previous: dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
            runtime_sources=runtime_sources, diagnostic_report=diagnostic_report,
            independent_event=recomputed_straight_interval, complete_interval=complete_interval,
            private_worker=private_worker, live=live, MODULE=MODULE, SCHEMA=SCHEMA, ROUTE=ROUTE, XODR=XODR),
        fixture_owner: dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
            runtime_sources=runtime_sources, diagnostic_report=diagnostic_report,
            live=live, MODULE=MODULE, SCHEMA=SCHEMA),
        runtime_owner: dict(paths=paths, fixed_contract=fixed_contract, runtime_sources=runtime_sources,
            diagnostic_report=diagnostic_report, complete_interval=complete_interval,
            live=live, MODULE=MODULE, SCHEMA=SCHEMA),
    }
    saved = {module: {name: getattr(module, name) for name in values} for module, values in changes.items()}
    _active = True
    try:
        for module, values in changes.items():
            for name, value in values.items():
                setattr(module, name, value)
        with runtime_owner.activated(pins):
            yield
    finally:
        for module, values in reversed(tuple(saved.items())):
            for name, value in values.items():
                setattr(module, name, value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    args, remaining = parser.parse_known_args(argv)
    pins = {name + "_sha256": getattr(args, "expected_" + name + "_sha256")
            for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
