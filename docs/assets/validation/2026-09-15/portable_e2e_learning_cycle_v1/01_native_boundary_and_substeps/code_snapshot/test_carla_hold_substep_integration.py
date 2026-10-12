"""HH_260906 - Test the explicit Town07 numerical-only extension without CARLA, old-test edits or admission."""

from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as profiles
from scripts.e2e import carla_physics_substeps as substeps
from scripts.e2e import collect_carla_vad_expert as collector
from test_carla_physics_substeps import FakeWorld, settings

PROFILE = "comfortable_v7_hold_brake_ramp"
ROUTE = Path(__file__).resolve().parents[1] / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"


def arguments(profile="reference_10ms", **changes):
    values = dict(physics_substep_profile=profile, goal_stop_profile=PROFILE,
                  hold_brake_mode="immediate", low_speed_steering_mode="zero",
                  control_transport="acknowledged_batch", agent_initialization="after_bootstrap",
                  physics_hz=20., capture_hz=10.)
    return SimpleNamespace(**dict(values, **changes))


def flags(profile="reference_10ms", output="unused-hold-substep-output"):
    return [str(output), str(ROUTE), "--physics-substep-profile", profile,
            "--goal-stop-profile", PROFILE, "--hold-brake-mode", "immediate",
            "--low-speed-steering-mode", "zero", "--control-transport", "acknowledged_batch",
            "--agent-initialization", "after_bootstrap", "--target-speed-kmh", "28.8",
            "--physics-hz", "20", "--capture-hz", "10", "--goal-tolerance-m", "1",
            "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5"]


@pytest.mark.parametrize("profile,delta", substeps.PROFILES.items())
def test_explicit_v7_only_changes_numerical_settings_with_full_readback(profile, delta):
    args = arguments(profile)
    assert substeps.validate_arguments(args) == profile
    world = FakeWorld(); original = world.get_settings(); saved = deepcopy(vars(original))
    record = substeps.new_record(profile)
    substeps.configure_world(world, original, args, record)
    assert len(world.applied) == 1 and vars(original) == saved
    assert record["status"] == "PASS" and len(record["after"]) == 11
    assert record["after"] == record["requested"] == dict(record["before"],
        synchronous_mode=True, fixed_delta_seconds=.05, max_substep_delta_time=delta)
    assert record["after"]["substepping"] is True and record["after"]["max_substeps"] == 10
    assert record["numerical_only"] is True and record["training_data_approved"] is False


@pytest.mark.parametrize("key,value", [
    ("hold_brake_mode", "ramp"), ("hold_brake_mode", None), ("hold_brake_mode", True),
    ("low_speed_steering_mode", "native"), ("low_speed_steering_mode", None), ("low_speed_steering_mode", False),
    ("control_transport", "legacy_async"), ("agent_initialization", "before_bootstrap"),
    ("physics_hz", 100.), ("physics_hz", 40.), ("physics_hz", float("nan")),
    ("capture_hz", 20.), ("capture_hz", True), ("physics_substep_profile", "inherited"),
    ("physics_substep_profile", "other"), ("physics_substep_profile", None),
])
def test_incompatible_modes_or_rates_fail_before_settings_application(key, value):
    args = arguments(); setattr(args, key, value)
    world = FakeWorld(); record = substeps.new_record("reference_10ms")
    with pytest.raises(ValueError):
        substeps.configure_world(world, world.get_settings(), args, record)
    assert not world.applied and record["status"] == "FAIL" and record["before"] is None


@pytest.mark.parametrize("missing", ["hold_brake_mode", "low_speed_steering_mode", "control_transport",
    "agent_initialization", "physics_hz", "capture_hz"])
def test_required_fields_cannot_be_implicit(missing):
    args = arguments(); delattr(args, missing)
    with pytest.raises(ValueError): substeps.validate_arguments(args)


@pytest.mark.parametrize("profile", ["disabled", "comfortable_v1", "comfortable_v2", "comfortable_v3",
    "comfortable_v4", "comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering", "turn_low_v1",
    "turn_launch_014_v1", "turn_launch_015_v1", "unknown", None])
def test_other_control_profiles_do_not_enter_new_allowlist(profile):
    with pytest.raises(ValueError): substeps.validate_arguments(arguments(goal_stop_profile=profile))


@pytest.mark.parametrize("profile", substeps.PROFILES)
def test_historical_turn_optin_needs_no_new_hold_or_steering_flags(profile):
    args = arguments(profile, goal_stop_profile="turn_launch_013_v1")
    del args.hold_brake_mode, args.low_speed_steering_mode
    assert substeps.validate_arguments(args) == profile


def test_both_new_cli_arms_keep_identical_frozen_control_and_data_configuration():
    parsed = [collector.parse_args(flags(profile)) for profile in substeps.PROFILES]
    configs = [asdict(profiles.configuration_from_args(args)) for args in parsed]
    assert configs[0] == configs[1] == asdict(profiles.HoldBrakeRampDevelopmentGoalStopConfig())
    assert {key for key in vars(parsed[0]) if vars(parsed[0])[key] != vars(parsed[1])[key]} == {"physics_substep_profile"}
    assert all(collector.physics_substep_module(args) is substeps for args in parsed)
    assert configs[0]["hold_seconds"] == 2. and configs[0]["minimum_tail_seconds"] == 6.5
    assert configs[0]["stopped_speed_mps"] == .1 and configs[0]["training_data_approved"] is False


@pytest.mark.parametrize("flag,value", [("--hold-brake-mode", "ramp"), ("--low-speed-steering-mode", "native"),
    ("--control-transport", "legacy_async"), ("--agent-initialization", "before_bootstrap"),
    ("--physics-hz", "100"), ("--capture-hz", "20")])
def test_cli_refuses_changed_treatment_or_protocol(flag, value):
    command = flags(); command[command.index(flag)+1] = value
    with pytest.raises(SystemExit): collector.parse_args(command)


def test_changed_route_rejected_without_output(tmp_path):
    changed = tmp_path / "changed-route.json"; changed.write_bytes(ROUTE.read_bytes()+b"\n")
    command = flags(output=tmp_path/"episode"); command[1] = str(changed)
    with pytest.raises(SystemExit): collector.parse_args(command)
    assert not (tmp_path/"episode").exists() and not (tmp_path/"episode.partial").exists()


@pytest.mark.parametrize("name,value", [("no_rendering_mode", True), ("max_substeps", 11),
    ("max_substep_delta_time", .01), ("tile_stream_distance", 2000.)])
def test_new_optin_retains_all_setting_readback_failures(name, value):
    world = FakeWorld(drift={name:value}); record = substeps.new_record("fine_5ms")
    with pytest.raises(ValueError, match="readback"):
        substeps.configure_world(world, world.get_settings(), arguments("fine_5ms"), record)
    assert record["status"] == "FAIL" and record["after"][name] == value


def test_original_nonreference_physics_cannot_be_silently_replaced():
    world = FakeWorld(settings(max_substep_delta_time=.005)); record = substeps.new_record("fine_5ms")
    with pytest.raises(ValueError, match="original CARLA substeps"):
        substeps.configure_world(world, world.get_settings(), arguments("fine_5ms"), record)
    assert not world.applied


def test_failed_capture_preserves_helper_pin_and_not_reached_not_admitted(tmp_path, monkeypatch):
    output = tmp_path/"episode"
    def refuse(*args, **kwargs): raise collector.CollectionError("mocked no-server failure")
    monkeypatch.setattr(collector, "collect_episode", refuse)
    with pytest.raises(collector.CollectionError, match="mocked no-server"):
        collector.run(collector.parse_args(flags("fine_5ms", output)))
    manifest = json.loads((tmp_path/"episode.partial/manifest.json").read_text())
    assert manifest["status"] == "failed" and manifest["result"]["training_data_approved"] is False
    assert manifest["capture_contract"]["physics_substeps"] == substeps.new_record("fine_5ms")
    assert manifest["provenance"]["physics_substeps_helper_sha256"] == hashlib.sha256(Path(substeps.__file__).read_bytes()).hexdigest()
