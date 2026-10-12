"""HH_260906 - Check explicit terminal feedback integration without CARLA, installs or relaxed historical gates."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as profiles
from scripts.e2e import carla_physics_substeps as substeps
from scripts.e2e import collect_carla_vad_expert as collector
from test_carla_hold_substep_integration import flags as v7_flags
from test_carla_physics_substeps import FakeWorld, settings

PROFILE = "comfortable_v9_terminal_feedback"


def flags(mode="coast", output="unused-terminal-feedback-output"):
    command = v7_flags(output=output)
    command[command.index("--goal-stop-profile") + 1] = PROFILE
    return command + ["--terminal-approach-mode", mode]


@pytest.mark.parametrize("mode", ("coast", "feedback"))
def test_new_mode_is_explicit_and_changes_no_numerical_setting(mode):
    args = collector.parse_args(flags(mode))
    config = profiles.configuration_from_args(args)
    assert type(config) is profiles.TerminalApproachDevelopmentGoalStopConfig
    assert config.terminal_approach_mode == mode
    assert substeps.validate_arguments(args) == "reference_10ms"
    world = FakeWorld()
    record = substeps.new_record("reference_10ms")
    substeps.configure_world(world, world.get_settings(), args, record)
    assert record["status"] == "PASS" and len(record["after"]) == 11
    assert record["after"] == dict(record["before"], synchronous_mode=True, fixed_delta_seconds=.05)
    assert config.goal_tolerance_m == 1. and config.stopped_speed_mps == .1
    assert config.hold_seconds == 2. and config.minimum_tail_seconds == 6.5
    assert config.maximum_coast_seconds == 45. and config.training_data_approved is False


def test_commands_and_configurations_differ_only_in_terminal_mode():
    first, second = [collector.parse_args(flags(mode)) for mode in ("coast", "feedback")]
    assert {key for key in vars(first) if vars(first)[key] != vars(second)[key]} == {"terminal_approach_mode"}
    a, b = [asdict(profiles.configuration_from_args(args)) for args in (first, second)]
    assert {key for key in a if a[key] != b[key]} == {"terminal_approach_mode"}


@pytest.mark.parametrize("flag,value", [
    ("--physics-substep-profile", "inherited"), ("--physics-substep-profile", "fine_5ms"),
    ("--hold-brake-mode", "ramp"), ("--low-speed-steering-mode", "native"),
    ("--control-transport", "legacy_async"), ("--agent-initialization", "before_bootstrap"),
    ("--physics-hz", "100"), ("--capture-hz", "20"), ("--goal-tolerance-m", "2"),
    ("--stationary-tail-sec", "1"), ("--stationary-warmup-sec", "0"),
    ("--target-speed-kmh", "30"), ("--terminal-approach-mode", "other"),
])
def test_incompatible_cli_fails_before_outputs(tmp_path, flag, value):
    command = flags("feedback", tmp_path / "episode")
    command[command.index(flag) + 1] = value
    with pytest.raises(SystemExit):
        collector.parse_args(command)
    assert list(tmp_path.iterdir()) == []


def test_mode_cannot_be_omitted_or_used_by_old_commands():
    with pytest.raises(SystemExit): collector.parse_args(flags()[:-2])
    with pytest.raises(SystemExit): collector.parse_args(v7_flags() + ["--terminal-approach-mode", "feedback"])
    assert collector.parse_args(v7_flags()).terminal_approach_mode is None


def test_changed_route_cannot_enter_new_profile(tmp_path):
    command = flags(output=tmp_path / "episode")
    route = tmp_path / "changed-route.json"
    route.write_bytes(Path(command[1]).read_bytes() + b"\n")
    command[1] = str(route)
    with pytest.raises(SystemExit): collector.parse_args(command)
    assert not (tmp_path / "episode").exists()


@pytest.mark.parametrize("mode", ("coast", "feedback"))
def test_failed_capture_retains_explicit_scope_and_helper_hashes(tmp_path, monkeypatch, mode):
    def fail_without_server(*args, **kwargs):
        raise collector.CollectionError("mocked no-server failure")
    monkeypatch.setattr(collector, "collect_episode", fail_without_server)
    with pytest.raises(collector.CollectionError, match="mocked no-server"):
        collector.run(collector.parse_args(flags(mode, tmp_path / "episode")))
    manifest = json.loads((tmp_path / "episode.partial/manifest.json").read_text())
    contract = manifest["capture_contract"]
    profile = contract["goal_stop_profile"]
    assert manifest["status"] == "failed" and manifest["result"]["training_data_approved"] is False
    assert contract["physics_substeps"] == substeps.new_record("reference_10ms")
    assert profile["terminal_approach_mode"] == profile["terminal_approach_experiment"]["mode"] == mode
    assert profile["terminal_approach_experiment"]["goal_dwell_tail_or_timeout_criteria_changed"] is False
    assert profile["terminal_approach_experiment"]["original_future_or_native_gate_changes"] is False
    assert profile["hold_brake_experiment"]["emergency_ramp_allowed"] is False
    assert profile["attempt_ownership"].startswith("Two prospectively declared")
    assert manifest["provenance"]["goal_stop_helper_sha256"] == hashlib.sha256(Path(profiles.__file__).read_bytes()).hexdigest()
    assert manifest["provenance"]["physics_substeps_helper_sha256"] == hashlib.sha256(Path(substeps.__file__).read_bytes()).hexdigest()


@pytest.mark.parametrize("kind", ("collision", "lane_invasion"))
def test_v9_inherits_next_observed_event_abort(kind):
    config = profiles.configuration_from_args(collector.parse_args(flags("feedback")))
    governor = profiles.DevelopmentGoalStopGovernor(config, 8., 20.)
    events = ({123: [{}]}, {}) if kind == "collision" else ({}, {123: [{}]})
    record = {"measured_timestamp": 1.}
    collector.observe_low_speed_steering_events(governor, record, SimpleNamespace(snapshot=lambda: events))
    assert governor.failure_reason == PROFILE + "_observed_event_abort"
    assert record["low_speed_steering_event_monitor"]["observed_event_counts"][kind] == 1
    assert record["low_speed_steering_event_monitor"]["zero_latency_abort_proven"] is False


def test_nonreference_original_world_cannot_be_silently_replaced():
    args = collector.parse_args(flags("feedback"))
    world = FakeWorld(settings(max_substep_delta_time=.005))
    record = substeps.new_record("reference_10ms")
    with pytest.raises(ValueError, match="original CARLA substeps"):
        substeps.configure_world(world, world.get_settings(), args, record)
    assert not world.applied and record["status"] == "FAIL"
