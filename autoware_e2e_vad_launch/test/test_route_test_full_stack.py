import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[2] / "scripts" / "e2e"
sys.path.insert(0, str(SCRIPT_DIR))

import route_test
from route_test import (
    EvaluationFailure,
    RouteTestMonitor,
    goal_completion_failures,
    load_goal_ros_pose,
    make_result,
    parse_args,
    route_snapshot,
    update_goal_completion_claim,
)


class ReadyClient:
    def __init__(self, ready=True):
        self.ready = ready

    def service_is_ready(self):
        return self.ready


class FakeOperationModeService:
    class Request:
        pass


class DoneFuture:
    def __init__(self, response):
        self.response = response

    def done(self):
        return True

    def exception(self):
        return None

    def result(self):
        return self.response


class ScriptedOperationClient:
    def __init__(self, responses, on_call=None, ready=True):
        self.responses = list(responses)
        self.on_call = on_call
        self.ready = ready
        self.call_count = 0

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        self.call_count += 1
        if self.on_call is not None:
            self.on_call(self.call_count)
        index = min(self.call_count - 1, len(self.responses) - 1)
        return DoneFuture(self.responses[index])


def operation_response(success, code=0, message=""):
    return SimpleNamespace(
        status=SimpleNamespace(success=success, code=code, message=message)
    )


def operation_state(mode, control=False, transition=False):
    return SimpleNamespace(
        mode=mode,
        is_autoware_control_enabled=control,
        is_in_transition=transition,
        is_autonomous_mode_available=True,
        is_stop_mode_available=True,
    )


def make_monitor(full_stack):
    monitor = object.__new__(RouteTestMonitor)
    monitor.full_stack = full_stack
    monitor.position = (1.0, 2.0)
    monitor.speed_mps = 0.05
    monitor.sim_time = 3.0
    monitor.trajectory_points = 6
    monitor.trajectory_valid = True
    monitor.route_status = "ready"
    monitor.remaining_distance = 10.0
    monitor.cross_track_error = 0.1
    monitor.trajectory_correction = 0.0
    monitor.command = 2
    monitor.goal_reached = False
    monitor.engage_state = False
    monitor.aeb_configured = True
    monitor.message_counts = {"odometry": 1}
    monitor.stop_signal = None
    if full_stack:
        monitor.mrm_normal_value = 1
        monitor.mrm_none_value = 1
        monitor.mrm_state = SimpleNamespace(state=1, behavior=1)
    return monitor


def test_minimal_readiness_only_requires_legacy_engage_interface():
    monitor = make_monitor(full_stack=False)
    monitor.engage_client = ReadyClient()

    assert monitor.missing_ready_inputs() == []


def test_full_stack_readiness_reports_standard_state_and_service_gaps():
    monitor = make_monitor(full_stack=True)
    monitor.localization_initialized_value = 3
    monitor.localization_initialization_state = 2
    monitor.standard_route_aligned = False
    monitor.aeb_configured = False
    monitor.operation_mode_state = SimpleNamespace(
        is_in_transition=True,
        is_autonomous_mode_available=False,
    )
    monitor.mrm_state = SimpleNamespace(state=2, behavior=2)
    monitor.change_to_stop_client = ReadyClient()
    monitor.change_to_autonomous_client = ReadyClient(ready=False)
    monitor.enable_autoware_control_client = ReadyClient()

    missing = monitor.missing_ready_inputs()

    assert "initialized localization (current=2)" in missing
    assert "standard route alignment (current=False)" in missing
    assert "AEB VAD-object configuration (current=False)" in missing
    assert "stable operation mode" in missing
    assert "autonomous operation mode availability" in missing
    assert "normal MRM state (state=2, behavior=2)" in missing
    assert "change-to-autonomous service" in missing
    assert "engage service" not in missing


def test_full_stack_readiness_accepts_initialized_aligned_available_state():
    monitor = make_monitor(full_stack=True)
    monitor.localization_initialized_value = 3
    monitor.localization_initialization_state = 3
    monitor.standard_route_aligned = True
    monitor.operation_mode_state = SimpleNamespace(
        is_in_transition=False,
        is_autonomous_mode_available=True,
    )
    monitor.change_to_stop_client = ReadyClient()
    monitor.change_to_autonomous_client = ReadyClient()
    monitor.enable_autoware_control_client = ReadyClient()

    assert monitor.missing_ready_inputs() == []


def test_full_stack_drive_health_uses_mrm_not_reengage_availability():
    monitor = make_monitor(full_stack=True)
    monitor.localization_initialized_value = 3
    monitor.localization_initialization_state = 3
    monitor.standard_route_aligned = True
    monitor.operation_mode_state = SimpleNamespace(
        is_autonomous_mode_available=False,
    )

    monitor.validate_route_values()

    monitor.mrm_state = SimpleNamespace(state=2, behavior=2)
    with pytest.raises(EvaluationFailure, match="MRM left NORMAL/NONE"):
        monitor.validate_route_values()


def test_operation_mode_match_requires_stable_mode_and_requested_control_state():
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_state = SimpleNamespace(
        mode=2,
        is_in_transition=False,
        is_autoware_control_enabled=True,
    )

    assert monitor.operation_mode_matches(2)
    assert monitor.operation_mode_matches(2, require_control=True)
    assert not monitor.operation_mode_matches(2, require_control=False)
    assert not monitor.operation_mode_matches(1)

    monitor.operation_mode_state.is_in_transition = True
    assert not monitor.operation_mode_matches(2, require_control=True)


def test_operation_mode_transition_skips_service_when_target_is_already_stable():
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_service_type = FakeOperationModeService
    monitor.operation_mode_state = operation_state(mode=1)
    client = ScriptedOperationClient([operation_response(True)])
    report = {}

    returned = monitor.transition_operation_mode(
        client,
        "change-to-stop service",
        target_mode=1,
        target_name="STOP",
        overall_timeout=1.0,
        service_timeout=0.5,
        report=report,
    )

    assert returned is report
    assert client.call_count == 0
    assert report["already_at_target"] is True
    assert report["target_reached"] is True
    assert report["attempt_count"] == 0


def test_operation_mode_transition_accepts_target_after_transient_rejection(
    monkeypatch,
):
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_service_type = FakeOperationModeService
    monitor.operation_mode_state = operation_state(mode=1)
    client = ScriptedOperationClient(
        [operation_response(False, code=1, message="target diagnostics stale")]
    )
    report = {}

    monkeypatch.setattr(route_test.rclpy, "ok", lambda: True)

    def reach_target(_node, timeout_sec):
        assert timeout_sec >= 0.0
        monitor.operation_mode_state = operation_state(mode=2)

    monkeypatch.setattr(route_test.rclpy, "spin_once", reach_target)

    monitor.transition_operation_mode(
        client,
        "change-to-autonomous service",
        target_mode=2,
        target_name="AUTONOMOUS",
        overall_timeout=1.0,
        service_timeout=0.5,
        report=report,
        retry_interval=0.1,
    )

    assert client.call_count == 1
    assert report["target_reached"] is True
    assert report["attempt_count"] == 1
    assert report["attempts"][0]["response"] == {
        "success": False,
        "code": 1,
        "message": "target diagnostics stale",
    }
    assert "service rejected" in report["attempts"][0]["error"]
    assert report["last_error"] is None


def test_operation_mode_transition_retries_rejection_until_target(monkeypatch):
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_service_type = FakeOperationModeService
    monitor.operation_mode_state = operation_state(mode=1)

    def reach_target_on_second_call(call_count):
        if call_count == 2:
            monitor.operation_mode_state = operation_state(mode=2)

    client = ScriptedOperationClient(
        [
            operation_response(False, code=2, message="in transition"),
            operation_response(True),
        ],
        on_call=reach_target_on_second_call,
    )
    monkeypatch.setattr(route_test.rclpy, "ok", lambda: True)
    monkeypatch.setattr(route_test.rclpy, "spin_once", lambda *_args, **_kwargs: None)

    report = monitor.transition_operation_mode(
        client,
        "change-to-autonomous service",
        target_mode=2,
        target_name="AUTONOMOUS",
        overall_timeout=1.0,
        service_timeout=0.5,
        retry_interval=0.0,
    )

    assert client.call_count == 2
    assert report["target_reached"] is True
    assert report["attempt_count"] == 2
    assert report["attempts"][0]["response"]["code"] == 2


def test_operation_mode_transition_does_not_repeat_accepted_request(monkeypatch):
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_service_type = FakeOperationModeService
    monitor.operation_mode_state = operation_state(mode=1)
    client = ScriptedOperationClient([operation_response(True)])
    monkeypatch.setattr(route_test.rclpy, "ok", lambda: True)

    def reach_target(_node, timeout_sec):
        assert timeout_sec >= 0.0
        monitor.operation_mode_state = operation_state(mode=2)

    monkeypatch.setattr(route_test.rclpy, "spin_once", reach_target)

    report = monitor.transition_operation_mode(
        client,
        "change-to-autonomous service",
        target_mode=2,
        target_name="AUTONOMOUS",
        overall_timeout=1.0,
        service_timeout=0.5,
        retry_interval=0.0,
    )

    assert client.call_count == 1
    assert report["target_reached"] is True
    assert report["attempts"][0]["outcome"] == "accepted"
    assert report["attempts"][0]["error"] is None


def test_operation_mode_transition_does_not_retry_terminal_rejection(monkeypatch):
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_service_type = FakeOperationModeService
    monitor.operation_mode_state = operation_state(mode=1)
    client = ScriptedOperationClient(
        [operation_response(False, code=50000, message="internal error")]
    )
    report = {}
    monkeypatch.setattr(route_test.rclpy, "ok", lambda: True)

    with pytest.raises(EvaluationFailure, match="internal error"):
        monitor.transition_operation_mode(
            client,
            "change-to-autonomous service",
            target_mode=2,
            target_name="AUTONOMOUS",
            overall_timeout=1.0,
            service_timeout=0.5,
            report=report,
            retry_interval=0.0,
        )

    assert client.call_count == 1
    assert report["target_reached"] is False
    assert report["attempts"][0]["outcome"] == "terminal_rejection"


def test_operation_mode_transition_timeout_reports_last_rejection(monkeypatch):
    monitor = make_monitor(full_stack=True)
    monitor.operation_mode_service_type = FakeOperationModeService
    monitor.operation_mode_state = operation_state(mode=1)
    client = ScriptedOperationClient(
        [operation_response(False, code=1, message="diagnostics stale")]
    )
    report = {}
    clock = SimpleNamespace(now=0.0)

    def monotonic():
        clock.now += 0.05
        return clock.now

    monkeypatch.setattr(route_test.time, "monotonic", monotonic)
    monkeypatch.setattr(route_test.rclpy, "ok", lambda: True)
    monkeypatch.setattr(route_test.rclpy, "spin_once", lambda *_args, **_kwargs: None)

    with pytest.raises(EvaluationFailure, match=r"last error: .*diagnostics stale"):
        monitor.transition_operation_mode(
            client,
            "change-to-autonomous service",
            target_mode=2,
            target_name="AUTONOMOUS",
            overall_timeout=0.5,
            service_timeout=0.2,
            report=report,
            retry_interval=0.0,
        )

    assert report["target_reached"] is False
    assert report["attempt_count"] >= 1
    assert report["attempts"][-1]["response"]["code"] == 1
    assert "diagnostics stale" in report["last_error"]


def test_full_stack_snapshot_records_standard_state():
    monitor = make_monitor(full_stack=True)
    monitor.localization_initialization_state = 3
    monitor.standard_route_aligned = True
    monitor.operation_mode_state = SimpleNamespace(
        mode=2,
        is_autoware_control_enabled=True,
        is_in_transition=False,
        is_autonomous_mode_available=True,
        is_stop_mode_available=True,
    )

    snapshot = route_snapshot(
        monitor, {"x": 4.0, "y": 6.0, "z": 0.0, "yaw": 0.0}
    )

    assert snapshot["speed_mps"] == 0.05
    assert snapshot["direct_goal_distance_m"] == 5.0
    assert snapshot["route_status"] == "ready"
    assert snapshot["localization_initialization_state"] == 3
    assert snapshot["standard_route_aligned"] is True
    assert snapshot["aeb_configured_for_vad_objects"] is True
    assert snapshot["operation_mode"] == {
        "mode": 2,
        "is_autoware_control_enabled": True,
        "is_in_transition": False,
        "is_autonomous_mode_available": True,
        "is_stop_mode_available": True,
    }
    assert snapshot["mrm_state"] == {"state": 1, "behavior": 1}


def test_goal_completion_rejects_bool_only_false_positive():
    monitor = make_monitor(full_stack=False)
    monitor.goal_reached = True
    monitor.route_status = "ready"
    monitor.position = (4.0, 0.0)
    monitor.speed_mps = 1.0

    failures = goal_completion_failures(
        monitor,
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        max_goal_distance_m=1.5,
        max_stop_speed_mps=0.15,
    )

    assert "route status is 'ready'" in failures
    assert "direct goal distance 4.000 m exceeds 1.500 m" in failures
    assert "final speed 1.000 m/s exceeds 0.150 m/s" in failures


def test_goal_completion_requires_true_bool_even_with_other_evidence():
    monitor = make_monitor(full_stack=False)
    monitor.goal_reached = False
    monitor.route_status = "goal_reached"
    monitor.position = (0.5, 0.0)
    monitor.speed_mps = 0.05

    failures = goal_completion_failures(
        monitor,
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        max_goal_distance_m=1.5,
        max_stop_speed_mps=0.15,
    )

    assert failures == ["goal_reached Bool is False"]


def test_goal_completion_accepts_all_four_independent_checks():
    monitor = make_monitor(full_stack=False)
    monitor.goal_reached = True
    monitor.route_status = "goal_reached"
    monitor.position = (0.5, 0.0)
    monitor.speed_mps = 0.05

    assert goal_completion_failures(
        monitor,
        {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        max_goal_distance_m=1.5,
        max_stop_speed_mps=0.15,
    ) == []


def test_goal_completion_waits_for_odometry_newer_than_claim():
    monitor = make_monitor(full_stack=False)
    monitor.goal_reached = True
    monitor.route_status = "goal_reached"
    monitor.message_counts["odometry"] = 7

    claim_count, evidence_ready = update_goal_completion_claim(monitor, None)

    assert claim_count == 7
    assert evidence_ready is False
    assert update_goal_completion_claim(monitor, claim_count) == (7, False)

    monitor.message_counts["odometry"] += 1
    assert update_goal_completion_claim(monitor, claim_count) == (7, True)

    monitor.route_status = "ready"
    assert update_goal_completion_claim(monitor, claim_count) == (None, False)


def test_load_goal_ros_pose_requires_finite_complete_pose(tmp_path):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps(
            {
                "goal_ros_pose": {
                    "x": 4,
                    "y": 5.5,
                    "z": 0.3,
                    "yaw": -1.0,
                }
            }
        ),
        encoding="utf-8",
    )

    resolved, goal = load_goal_ros_pose(route_file)

    assert resolved == route_file.resolve()
    assert goal == {"x": 4.0, "y": 5.5, "z": 0.3, "yaw": -1.0}

    route_file.write_text(
        json.dumps(
            {
                "goal_ros_pose": {
                    "x": 4,
                    "y": 5.5,
                    "z": 0.3,
                    "yaw": float("inf"),
                }
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"goal_ros_pose\.yaw must be a finite"):
        load_goal_ros_pose(route_file)


def test_parse_args_loads_route_goal_and_completion_defaults(tmp_path, monkeypatch):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps(
            {
                "goal_ros_pose": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 0.0,
                    "yaw": 0.0,
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        sys, "argv", ["route_test.py", "--route-file", str(route_file)]
    )

    args = parse_args()

    assert args.route_file == route_file.resolve()
    assert args.goal_ros_pose["x"] == 1.0
    assert args.max_goal_distance == 1.5
    assert args.max_stop_speed == 0.15
    assert args.ready_stability == 4.0

    result = make_result(args)
    assert result["route_file"] == str(route_file.resolve())
    assert result["goal_ros_pose"] == args.goal_ros_pose
    assert result["limits"]["maximum_direct_goal_distance_m"] == 1.5
    assert result["limits"]["maximum_stop_speed_mps"] == 0.15
    assert result["limits"]["ready_stability_wall_sec"] == 4.0


def test_parse_args_rejects_nonpositive_completion_limits(tmp_path, monkeypatch):
    route_file = tmp_path / "route.json"
    route_file.write_text(
        json.dumps(
            {
                "goal_ros_pose": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 0.0,
                    "yaw": 0.0,
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "route_test.py",
            "--route-file",
            str(route_file),
            "--max-stop-speed",
            "0",
        ],
    )

    with pytest.raises(SystemExit, match="2"):
        parse_args()
