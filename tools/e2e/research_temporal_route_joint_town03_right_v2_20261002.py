"""HH_261002 - Own a fresh controlled-Green Town03 turn; v1 hazard failure remains immutable and unproven by subtype."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path

from . import research_temporal_route_joint_town03_right_20261002 as previous
from . import collect_temporal_route_joint_town03_right_v2_20261002 as live

base, numeric, ROOT = previous.base, previous.numeric, previous.ROOT
MODULE = "scripts.e2e.research_temporal_route_joint_town03_right_v2_20261002"
SCHEMA = live.OWNER_SCHEMA
V1_SHA = "ebfde24f322192b816584b10a52e7bf15a53090157d1753a37d08ec239286340"
_paths, _capture, _fixed, _sources, _diagnostic = previous.paths, previous.capture_argv, previous.fixed_contract, previous.runtime_sources, previous.diagnostic_report
_active = False


def paths(run_id):
    previous.require(run_id == "v2", "fresh controlled-Green v2 only")
    return _paths(run_id)


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    return _capture(run_id, longitudinal_pi, declaration_sha)+["--observe-traffic-lights"]


def fixed_contract(run_id, longitudinal_pi):
    result = _fixed(run_id, longitudinal_pi)
    result.update(owned_green_signal_fixture=dict(original_own_world_traffic_actors="ALL", state="Green", frozen=True,
        configured_after_exact_live_XODR_and_exclusive_world_proof_before_vehicle_spawn=True,
        original_light_state_frozen_flag_and_duration_restored_on_exit=True, elapsed_signal_phase_not_rewound=True,
        freeze_API_scope="world-global; exactly one call per install/restore", original_frozen_flags_must_be_uniform=True,
        pre_vehicle_fixture_native_ticks=1, post_vehicle_restoration_native_ticks=1,
        fixture_native_ticks_are_outside_the_110s_capture_and_not_NN_performance=True,
        fixture_tick_hz=20, original_physics_settings_restored_after_suffix_tick=True,
        all_original_hazard_detectors_and_emergency_returns_unchanged=True, ignore_traffic_lights=False,
        original_vehicle_and_light_detector_returns_logged_once=True, known_v1_hazard_subtype_proven=False,
        closed_course_basic_turn_only_not_traffic_rule_performance=True),
        actual_manual_first_gear_bootstrap_and_engagement_required=True,
        original_launch_throttle=.11, original_launch_handoff_mps=.5, original_launch_wall_limit_seconds=8.,
        v1_output_preserved=True, v1_hazard_failure_is_not_a_neural_turn_failure=True)
    return result


def runtime_sources():
    previous.require(previous.digest(Path(previous.__file__)) == V1_SHA, "frozen failed Town03 owner differs")
    values = _sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name, sha = str(path.relative_to(ROOT)), previous.digest(path)
        previous.require(name not in values or values[name] == sha, "new v2 imported source conflicts")
        values[name] = sha
    return values


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    result.update(schema=SCHEMA+".diagnostic_report", controlled_signal_fixture_passed=False,
                  actual_manual_first_gear_initialization_passed=False)
    try:
        episode = output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
        manifest = previous.read_json(episode/"manifest.json")
        fixture = previous.read_json(episode/"owned_green_signal_fixture.json")
        witness = previous.jsonl(episode/"original_BasicAgent_hazard_decisions.jsonl")
        states = previous.jsonl(episode/"states.jsonl"); ACK = previous.jsonl(episode/"control_receipts.jsonl")
        driving = [s for s in states if s["capture_phase"] == "driving"]
        expected_frames = [states[23]["frame"]]+[s["frame"] for s in driving]
        previous.require([w["frame"] for w in witness] == expected_frames, "every original driving decision witness required")
        previous.require(fixture["schema"] == live.SCHEMA+".owned_green_signal_fixture" and fixture["state"] == "RESTORED"
            and fixture["original_state_freeze_and_duration_restored"] is True and fixture["readback_count"] == len(witness)+1,
            "fixture install/per-decision/restoration evidence differs")
        by_sequence = {r["sequence"]: r for r in ACK}
        for state, row in zip(driving, witness[1:]):
            previous.require(row["schema"] == live.SCHEMA+".original_hazard_decisions"
                and [r["kind"] for r in row["original_decisions"]] == ["vehicle", "traffic_light"]
                and all(r["affected"] is False for r in row["original_decisions"])
                and row["original_hazard_braking_unchanged"] is True, "unexpected original hazard remains a failed attempt")
            receipt = by_sequence[state["control_transport"]["next_command_receipt_sequence"]]
            previous.require(receipt["before_frame"] == row["frame"] and receipt["status"] == "ACKNOWLEDGED",
                             "actual final command must bind the same original detector decision frame")
        initial = manifest["capture_contract"]["agent_initialization"]
        observed = [initial["construction"][key] for key in ("before", "after")]+[initial["engagement"]["before"]]
        previous.require(all(r["status"] == "PASS" and r["current_control"]["manual_gear_shift"] is True
            and type(r["current_control"]["gear"]) is int and r["current_control"]["gear"] == 1
            and r["current_control"]["throttle"] == r["current_control"]["steer"] == 0.
            and r["current_control"]["brake"] == 1. for r in observed), "actual strict manual1 full-brake witnesses absent")
        previous.verify_closure(output, plan)
        previous.require(all(previous.digest(name) == sha for name, sha in result["raw_source_sha256"].items()),
                         "raw fixture/hazard inputs changed during the new conjunctive diagnostic")
        result.update(controlled_signal_fixture_passed=True, actual_manual_first_gear_initialization_passed=True,
                      original_hazard_witness_count=len(witness), original_signal_actor_count=len(fixture["original_actor_ids"]))
    except Exception as error: result["new_fixture_diagnostic_error"] = type(error).__name__+": "+str(error)
    valid = result["controlled_signal_fixture_passed"] and result["actual_manual_first_gear_initialization_passed"]
    result["learned_driving_screen_passed"] = result.get("learned_driving_screen_passed", False) and valid
    result["joint_turn_motion_body_policy_wall100ms_passed"] = result.get("joint_turn_motion_body_policy_wall100ms_passed", False) and valid
    result["status"] = "PASS_KNOWN_DEV_RIGHT_TURN_PRECURSOR" if result["joint_turn_motion_body_policy_wall100ms_passed"] else "FAIL_KNOWN_DEV_RIGHT_TURN_PRECURSOR"
    return result


@contextmanager
def activated(pins):
    global _active
    previous.require(not _active, "new v2 owner cannot nest")
    previous.require(previous.digest(Path(previous.__file__)) == V1_SHA, "frozen v1 owner must not change")
    with previous.activated(pins):
        values = dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract, runtime_sources=runtime_sources,
                      diagnostic_report=diagnostic_report, live=live, MODULE=MODULE, SCHEMA=SCHEMA)
        saved_previous = {k: getattr(previous, k) for k in values}; saved_base = {k: getattr(base, k) for k in values}
        old_validate, old_numeric = live.frozen.validate_capture, numeric.MODULE
        _active = True
        try:
            for target in (previous, base):
                for name, value in values.items(): setattr(target, name, value)
            live.frozen.validate_capture = live.validate_capture; numeric.MODULE = MODULE
            yield
        finally:
            live.frozen.validate_capture = old_validate; numeric.MODULE = old_numeric
            for target, saved in ((base, saved_base), (previous, saved_previous)):
                for name, value in saved.items(): setattr(target, name, value)
            _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"): parser.add_argument("--expected-"+name+"-sha256", required=True)
    args, rest = parser.parse_known_args(argv)
    pins = {name+"_sha256": getattr(args, "expected_"+name+"_sha256") for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins): return base.main(rest)


if __name__ == "__main__": raise SystemExit(main())
