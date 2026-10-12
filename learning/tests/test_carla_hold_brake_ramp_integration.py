# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Exercise the new collector opt-in and event boundary without contacting CARLA."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_carla_vad_expert as capture
from scripts.e2e import carla_goal_stop_profile as profiles


ROUTE = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists()) / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"


def argv(mode="immediate"):
    # HH_260906 - The checked-in exact TRAIN route is parsed locally; these tests do not open held-out data or sockets.
    return ["unused-v7-test-output", str(ROUTE), "--goal-stop-profile", "comfortable_v7_hold_brake_ramp",
            "--low-speed-steering-mode", "zero", "--hold-brake-mode", mode,
            "--target-speed-kmh", "28.8", "--physics-hz", "20", "--capture-hz", "10",
            "--goal-tolerance-m", "1", "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5",
            "--control-transport", "acknowledged_batch", "--agent-initialization", "after_bootstrap"]


@pytest.mark.parametrize("mode", ["immediate", "ramp"])
def test_explicit_v7_collector_arguments_keep_original_bounds(mode):
    args = capture.parse_args(argv(mode))
    config = profiles.configuration_from_args(args)
    assert type(config) is profiles.HoldBrakeRampDevelopmentGoalStopConfig
    assert config.hold_brake_mode == mode and config.low_speed_steering_mode == "zero"
    assert config.low_speed_steering_threshold_mps == .75
    assert config.hold_seconds == 2. and config.stopped_speed_mps == .1 and config.minimum_tail_seconds == 6.5


@pytest.mark.parametrize("change", ["missing_brake", "missing_steering", "native_steering", "before_bootstrap", "legacy_async"])
def test_v7_refuses_incompatible_or_implicit_modes_before_world_access(change):
    command = argv()
    if change.startswith("missing"):
        flag = "--hold-brake-mode" if change == "missing_brake" else "--low-speed-steering-mode"
        index = command.index(flag)
        del command[index:index + 2]
    else:
        flag, value = {"native_steering": ("--low-speed-steering-mode", "native"),
                       "before_bootstrap": ("--agent-initialization", "before_bootstrap"),
                       "legacy_async": ("--control-transport", "legacy_async")}[change]
        command[command.index(flag) + 1] = value
    with pytest.raises(SystemExit):
        capture.parse_args(command)


def test_v7_wrong_route_is_rejected_during_parser_preflight(tmp_path):
    # HH_260906 - An unrelated route cannot pass by selecting an inherited physics profile.
    route = tmp_path / "wrong-route.json"
    route.write_text(ROUTE.read_text() + "\n")
    command = argv()
    command[1] = str(route)
    with pytest.raises(SystemExit):
        capture.parse_args(command)


@pytest.mark.parametrize("mode", ["immediate", "ramp"])
def test_default_profile_rejects_new_hold_brake_flag(mode):
    with pytest.raises(SystemExit):
        capture.parse_args(["unused", str(ROUTE), "--hold-brake-mode", mode])


@pytest.mark.parametrize("mode", ["immediate", "ramp"])
@pytest.mark.parametrize("event", [None, "collision", "lane_invasion"])
def test_v7_inherits_observed_event_abort_without_claiming_instantaneous_delivery(mode, event):
    governor = profiles.DevelopmentGoalStopGovernor(
        profiles.HoldBrakeRampDevelopmentGoalStopConfig(hold_brake_mode=mode), 8., 20.)
    row = governor.update(100., 100., 0., -100., timestamp=1., longitudinal_speed_mps=0., driving=True)
    collision = {101: [{"event": "collision"}]} if event == "collision" else {}
    lane = {101: [{"event": "lane"}]} if event == "lane_invasion" else {}
    capture.observe_low_speed_steering_events(governor, row, SimpleNamespace(snapshot=lambda: (collision, lane)))
    monitor = row["low_speed_steering_event_monitor"]
    assert monitor["observed_event_counts"] == {"collision": int(event == "collision"), "lane_invasion": int(event == "lane_invasion")}
    assert monitor["asynchronous_event_delivery"] is True and monitor["zero_latency_abort_proven"] is False
    assert bool(governor.failure_reason) is (event is not None)
    if event:
        assert row["pilot_failure_reason"] == "comfortable_v7_hold_brake_ramp_observed_event_abort"
