"""HH_261001 - Do not mistake CARLA float32 storage for a controller change or tolerate metadata drift."""

from copy import deepcopy

from scripts.e2e import audit_town12_brake_calibration_v2_20261001 as audit


def test_only_existing_ack_control_tolerance_handles_float32():
    recorded = {"source_frame": 42, "source_timestamp": 1.25,
                "nested": {"throttle": .20000000298023224, "brake": 0., "steer": -.0001}}
    replay = deepcopy(recorded)
    replay["nested"]["throttle"] = .2
    assert audit.same_record(replay, recorded)
    replay["nested"]["throttle"] = .200002
    assert not audit.same_record(replay, recorded)


def test_frame_timestamp_schema_and_noncontrol_fields_remain_exact():
    original = {"source_frame": 42, "source_timestamp": 1.25, "entry": None, "stage": "launch"}
    for key, value in [("source_frame", 43), ("source_timestamp", 1.2500000001), ("stage", "pulse")]:
        changed = {**original, key: value}
        assert not audit.same_record(changed, original)
    assert not audit.same_record({**original, "extra": 0}, original)


def test_nonfinite_controls_cannot_pass_replay():
    assert not audit.same_record({"brake": float("nan")}, {"brake": 0.})
    assert not audit.same_record({"throttle": float("inf")}, {"throttle": float("inf")})
