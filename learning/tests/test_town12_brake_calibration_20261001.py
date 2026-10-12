"""HH_261001 - Bound pulse timing, actual entry/exit, native ACK guards and exact ownership without CARLA."""

from types import SimpleNamespace

import pytest

from scripts.e2e import calibrate_town12_j15824_brake_pulses_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_20261001 as owner


def control(throttle=.20, brake=0., steer=.07):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=steer,
        hand_brake=False, reverse=False, manual_gear_shift=False, gear=1)


def enter(policy):
    policy.apply(control(), speed=0., timestamp=10., frame=100)
    policy.apply(control(), speed=.5, timestamp=10.05, frame=101)
    for index in range(10):
        value = policy.apply(control(), speed=2.4, timestamp=10.1 + index * .05, frame=102 + index)
    return value


@pytest.mark.parametrize("level", worker.LEVELS)
def test_each_declared_level_uses_actual_stable_entry_and_exact_pulse_budget(level):
    policy = worker.PulsePolicy(level)
    result = enter(policy)
    assert policy.entry == {"frame": 111, "timestamp": pytest.approx(10.55), "speed_mps": 2.4}
    assert policy.pulse_ticks == 1 and result.throttle == 0. and result.brake == level
    assert result.steer == .07
    for index in range(1, 40):
        assert policy.completion(2.) is None
        policy.apply(control(), speed=2., timestamp=10.55 + index * .05, frame=111 + index)
    assert policy.completion(2.) == "FIXED_PULSE_DURATION"
    with pytest.raises(worker.scout.ScoutError, match="bounded completion"):
        policy.apply(control(), speed=2., timestamp=12.55, frame=151)


def test_launch_is_gentle_and_a_measured_stop_is_not_invented():
    policy = worker.PulsePolicy(.001)
    first = policy.apply(control(), speed=0., timestamp=0., frame=1)
    assert first.throttle == .13 and first.brake == 0. and first.steer == .07
    handoff = policy.apply(control(), speed=.5, timestamp=.05, frame=2)
    assert handoff.throttle == pytest.approx(.1325)
    assert policy.completion(0.) is None
    for index in range(10):
        policy.apply(control(), speed=2.4, timestamp=.10 + index * .05, frame=3 + index)
    assert policy.completion(.501) is None
    assert policy.completion(.5) == "EARLY_MEASURED_SPEED_FLOOR"


def test_entry_band_must_be_consecutive_and_source_clock_native():
    policy = worker.PulsePolicy(0.)
    for index, speed in enumerate([.5, 2.4, 2.4, 2.46, 2.4]):
        policy.apply(control(), speed=speed, timestamp=index * .05, frame=100 + index)
    assert policy.stable_ticks == 1 and policy.entry is None
    with pytest.raises(worker.scout.ScoutError, match="cadence"):
        policy.apply(control(), speed=2.4, timestamp=.3, frame=106)


@pytest.mark.parametrize("value", [-1., 2.551, float("nan")])
def test_invalid_actual_speed_fails_before_pedal_change(value):
    original = control()
    with pytest.raises(worker.scout.ScoutError):
        worker.PulsePolicy(0.).apply(original, speed=value, timestamp=0., frame=1)
    assert original.throttle == .2 and original.brake == 0.


def test_no_unapproved_level_or_missing_normal_cap():
    with pytest.raises(worker.scout.ScoutError):
        worker.PulsePolicy(.02)
    with pytest.raises(worker.scout.ScoutError, match="capped"):
        worker.PulsePolicy(0.).apply(control(brake=.15), speed=.5, timestamp=0., frame=1)


def good_row():
    return {"speed_mps": 2.4, "front_bumper_stop_distance_m": 60., "signal_state": "Yellow",
        "route_projection": {"progress_m": 10.}, "control_alignment": {"status": "PASS"},
        "body_corners": [{"lane": [588, 0, 1], "lane_margin_m": .20}] * 8}


@pytest.mark.parametrize("key,value", [("speed_mps", 2.6), ("front_bumper_stop_distance_m", 39.99),
    ("front_bumper_stop_distance_m", float("nan")), ("route_projection", {"progress_m": float("nan")}),
    ("signal_state", "Off"), ("route_projection", {"progress_m": 30.01}),
    ("control_alignment", {"status": "FAIL"}), ("body_corners", [{"lane": [588, 0, 2], "lane_margin_m": 1.}]),
    ("body_corners", [{"lane": [588, 0, 1], "lane_margin_m": .099}])])
def test_native_measurement_guards_are_strict(key, value):
    row = good_row()
    assert worker.scalar_guards(row) == []
    row[key] = value
    assert worker.scalar_guards(row)


def test_ownership_cleanup_only_destroys_created_actors_and_verifies_absence():
    alive = {7, 8, 999}
    destroyed = []
    def actor(identity, kind):
        def destroy():
            destroyed.append(identity)
            alive.remove(identity)
            return True
        return SimpleNamespace(id=identity, type_id=kind, destroy=destroy, stop=lambda: None)
    snapshot = SimpleNamespace(frame=10, find=lambda identity: identity if identity in alive else None)
    result = worker.destroy_owned(SimpleNamespace(tick=lambda timeout: 10, get_snapshot=lambda: snapshot),
                                  [actor(7, "vehicle.toyota.prius"), actor(8, "sensor.other.collision")])
    assert destroyed == [8, 7] and alive == {999} and result["status"] == "PASS"


def test_failed_owned_destruction_is_not_reported_as_cleanup_pass():
    actor = SimpleNamespace(id=7, type_id="vehicle.toyota.prius", destroy=lambda: False)
    snapshot = SimpleNamespace(frame=10, find=lambda identity: actor)
    result = worker.destroy_owned(SimpleNamespace(tick=lambda timeout: 10, get_snapshot=lambda: snapshot), [actor])
    assert result["status"] == "FAIL" and result["remaining_owned_actor_ids"] == [7]


def test_options_retain_latest_approved_steering_factor_and_ratio():
    opts, _ = worker.collector.basic_agent_control_configuration(SimpleNamespace(target_speed_kmh=8.64,
        physics_hz=20., basic_agent_base_min_distance_m=1.5, basic_agent_distance_ratio=.5), 1.)
    assert opts["base_min_distance"] == 1.5 and opts["distance_ratio"] == .5
    assert opts["target_speed"] == 8.64


def test_protocol_never_claims_training_signal_stop_or_sixcam_capture():
    value = worker.protocol()
    assert not value["training_data_admitted"] and not value["future_labels_created"]
    assert not value["six_camera_capture"] and not value["traffic_light_states_written"]
    assert value["fresh_owned_actor_per_level"] and value["original_basic_agent_red_emergency_unchanged"]
    assert value["entry_band_mps"] == [2.35, 2.45] and value["pulse_ticks"] == 40


def test_new_owner_restores_all_shared_python_bindings_even_on_failure():
    inherited = owner.inherited
    before = inherited.OUTPUT, inherited.SOURCE_FILES, inherited.EXTERNAL_SOURCES
    original = inherited.diagnostic.owner
    callbacks = original.preflight, original._wait_ready, original._read_live_xodr, original._write_new_json
    with pytest.raises(RuntimeError):
        with owner.configured_owner() as configured:
            assert inherited.OUTPUT == owner.OUTPUT
            assert configured._read_live_xodr is not callbacks[2]
            assert all((owner.ROOT / name).is_file() for name in owner.SOURCE_FILES)
            raise RuntimeError("force restoration")
    assert (inherited.OUTPUT, inherited.SOURCE_FILES, inherited.EXTERNAL_SOURCES) == before
    assert (original.preflight, original._wait_ready, original._read_live_xodr, original._write_new_json) == callbacks
