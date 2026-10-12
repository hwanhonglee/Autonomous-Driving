"""HH_261001 - Native-only throttle isolation checks that never start CARLA."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_town07_throttle_isolation_20261001 as worker
from scripts.e2e import run_owned_town07_throttle_isolation_20261001 as owner
from scripts.e2e import town07_throttle_isolation_contract_20261001 as protocol


def test_two_sites_have_fixed_abba_and_one_attempt_each():
    rows = protocol.trial_specs()
    assert len(rows) == 8
    assert [row["site_progress_m"] for row in rows] == [16] * 4 + [25] * 4
    assert [row["arm"] for row in rows] == list("ABBAABBA")
    assert [row["throttle"] for row in rows] == [.01, .20, .20, .01] * 2
    assert len({row["trial_id"] for row in rows}) == 8
    assert protocol.fixed_contract()["camera_count"] == 0
    assert protocol.fixed_contract()["model_inferences"] == 0
    assert protocol.fixed_contract()["automatic_retry"] is False
    with pytest.raises(ValueError):
        protocol.trial(8)


@pytest.mark.parametrize("progress,speed,expected", [
    (15.99, 2.45, "BEFORE_SITE"), (16., 2.35, "START"),
    (16.25, 2.55, "START"), (16.251, 2.45, "FAIL_BRANCH_PROGRESS_OVERSHOOT"),
    (16.1, 2.349, "FAIL_BRANCH_SPEED_OUTSIDE_MATCHED_WINDOW"),
    (16.1, 2.551, "FAIL_BRANCH_SPEED_OUTSIDE_MATCHED_WINDOW"),
    (float("nan"), 2.45, "FAIL_NONFINITE_BRANCH_STATE"),
])
def test_branch_uses_first_site_crossing_without_waiting_for_favorable_speed(progress, speed, expected):
    assert protocol.branch_status(progress, speed, 16) == expected


def test_safety_abort_rejects_events_cte_speed_ack_frame_and_disk():
    base = dict(frame=102, previous_frame=101, vx=2.45, cte_m=.1,
                collision_count=0, lane_event_count=0, traffic_light_hazard=False,
                alignment_ok=True,
                free_bytes=protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES)
    assert protocol.safety_failures(**base) == []
    cases = (("frame", 104, "native_frame_discontinuity"),
             ("vx", 2.701, "maximum_speed"), ("cte_m", 1.001, "route_cte"),
             ("collision_count", 1, "observed_event"),
             ("lane_event_count", 1, "observed_event"),
             ("traffic_light_hazard", True, "traffic_light_hazard"),
             ("alignment_ok", False, "control_alignment"),
             ("free_bytes", protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES - 1, "disk_guard"))
    for key, changed, reason in cases:
        assert reason in protocol.safety_failures(**(base | {key: changed}))


def test_red_light_guard_preserves_frame_and_does_not_advance_planner():
    class Agent:
        _base_tlight_threshold = 5.
        _speed_ratio = .2
        _lights_list = [object()]
        def _affected_by_traffic_light(self, lights, distance):
            assert lights is self._lights_list
            assert distance == pytest.approx(5.5)
            return True, SimpleNamespace(id=17, state="Red")
    world = SimpleNamespace(get_snapshot=lambda: SimpleNamespace(frame=101))
    record = worker.traffic_light_hazard_observation(world, Agent(), (1.5, 2., 0.), 101)
    assert record["hazard"] is True and record["light_actor_id"] == 17
    assert record["planner_advanced"] is False
    with pytest.raises(worker.base.CollectionError, match="different frame"):
        worker.traffic_light_hazard_observation(world, Agent(), (1.5, 2., 0.), 102)


def test_scripted_controls_keep_only_the_fixed_throttle_difference():
    class VehicleControl:
        def __init__(self, **values):
            self.__dict__.update(values)
    carla = SimpleNamespace(VehicleControl=VehicleControl)
    a, b = (worker.scripted_control(carla, throttle) for throttle in (.01, .20))
    assert a.throttle == .01 and b.throttle == .20
    assert (a.steer, a.brake, a.hand_brake, a.reverse) == (0., 0., False, False)
    assert (b.steer, b.brake, b.hand_brake, b.reverse) == (0., 0., False, False)
    with pytest.raises(ValueError):
        worker.scripted_control(carla, .10)


def test_native_summary_counts_all_sixty_ticks_and_discrete_drop():
    rows = [dict(phase="branch", frame=100, timestamp=5., vx=2.45, route_progress_m=16.)]
    for tick in range(1, 61):
        velocity = 2.45 - .02 * tick - (.5 if tick >= 20 else 0.)
        rows.append(dict(phase="intervention", intervention_tick=tick,
            frame=100 + tick, timestamp=5. + tick * .05, vx=velocity,
            route_progress_m=16. + .1 * tick))
    result = worker.summarize(rows, protocol.trial(0))
    assert result["observed_ticks"] == 60
    assert result["signed_rate_below_minus_8_mps2_count"] == 1
    assert result["minimum_signed_rate_mps2"] < -8
    assert result["net_route_progress_m"] == pytest.approx(6.)
    assert result["training_data_approved"] is False
    with pytest.raises(ValueError):
        worker.summarize(rows[:-1], protocol.trial(0))


def test_prior_order_allows_retained_physical_failure_but_not_owner_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(protocol, "RUN_ROOT", tmp_path / "campaign")
    first, second = protocol.trial(0), protocol.trial(1)
    owner.prior_ready(first)
    folder = Path(first["output"])
    folder.mkdir(parents=True)
    (folder / "owner_result.json").write_text(json.dumps(dict(cleanup_errors=[], ports_released=True,
        collector_exit_code=1)))
    (folder / "throttle_result.json").write_text(json.dumps(dict(trial_id=first["trial_id"],
        status="FAILED_RETAINED", source_hashes_match=True)))
    owner.prior_ready(second)
    (folder / "owner_result.json").write_text(json.dumps(dict(cleanup_errors=["port"], ports_released=False)))
    with pytest.raises(ValueError, match="release"):
        owner.prior_ready(second)


def test_worker_parser_rejects_output_redirection_before_simulator(tmp_path, monkeypatch):
    monkeypatch.setattr(protocol, "PLAN", tmp_path / "missing-plan.json")
    with pytest.raises(ValueError, match="worker output differs"):
        worker.parse_args(["--trial", "0", "--plan-sha256", "a" * 64,
                           "--output", str(tmp_path / "wrong")])


def test_owner_command_is_one_fixed_native_worker_without_camera_or_model():
    spec = protocol.trial(0)
    argv = owner.command(spec, "b" * 64)
    assert argv[3] == "scripts.e2e.collect_town07_throttle_isolation_20261001"
    assert argv[-1] == str(Path(spec["output"]) / "episode")
    assert not any("camera" in value or "research-weights" in value for value in argv)
