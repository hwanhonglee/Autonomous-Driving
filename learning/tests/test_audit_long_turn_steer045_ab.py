"""CPU-only tests of the preregistered steering-cap comparison and denial path."""

import copy
import hashlib

import pytest

from scripts.e2e import audit_long_turn_steer045_ab as ab


def command(output, steering=None):
    result = [str(output), str(ab.prior.ROUTE), "--target-speed-kmh", "9",
              "--seed", "2026092702", "--goal-stop-profile", "disabled"]
    if steering is not None:
        result += ["--basic-agent-max-steering", steering]
    return result


def agent(cap):
    return {"normalized_max_steering": cap,
            "effective_opt_dict": {"max_steering": cap, "target_speed": 9.,
                                   "lateral_control_dict": {"K_P": 1.95, "K_I": .05, "K_D": .2}},
            "collector_cli": {"--basic-agent-max-steering": cap, "--basic-agent-lane-offset-m": 0.},
            "compatibility_defaults": {"max_steering": .8}}


def test_only_explicit_steering_cap_and_episode_path_may_change():
    a = command(ab.TRIAL_A / "episode")
    b = command(ab.TRIAL_B / "episode", "0.45")
    assert ab.one_factor_arguments(a, b)["all_other_collector_arguments_equal"]
    with pytest.raises(ab.prior.AuditError):
        ab.one_factor_arguments(a, b + ["--weather", "WetNoon"])
    with pytest.raises(ab.prior.AuditError):
        ab.one_factor_arguments(a, command(ab.TRIAL_B / "episode", "0.40"))


def test_effective_agent_options_must_match_except_normalized_cap():
    a, b = agent(.8), agent(.45)
    assert ab.one_factor_agent(a, b)["measured_tire_angle_is_not_normalized_command"]
    changed = copy.deepcopy(b)
    changed["effective_opt_dict"]["lateral_control_dict"]["K_P"] = 2.
    with pytest.raises(ab.prior.AuditError):
        ab.one_factor_agent(a, changed)


def test_geometry_provenance_binds_box_actor_and_raw_xml(tmp_path):
    episode = tmp_path / "episode"
    episode.mkdir()
    payload = b"<OpenDRIVE><road id='1'/></OpenDRIVE>"
    (episode / "active_map_opendrive.xodr").write_bytes(payload)
    manifest = {"files": {"active_map_opendrive": "active_map_opendrive.xodr"},
        "capture_contract": {"agent_initialization": {"construction": {
            "before": {"actor_id": 17}, "after": {"actor_id": 17}}},
            "actor_body_geometry": {"status": "RECORDED", "source": "carla.Vehicle.bounding_box",
                "actor_type_id": "vehicle.toyota.prius", "actor_id": 17,
                "reference_frame": "CARLA actor-local; not ROS rear-axle base_link",
                "lane_body_containment_verified": False,
                "bounding_box": {"location": {"x": 0., "y": 0., "z": .7},
                                 "extent": {"x": 2.2, "y": 1., "z": .8},
                                 "rotation": {"roll": 0., "pitch": 0., "yaw": 0.}}},
            "active_map_opendrive": {"status": "RECORDED", "source": "carla.Map.to_opendrive",
                "map_name": "Carla/Maps/Town01", "file": "active_map_opendrive.xodr",
                "sha256": hashlib.sha256(payload).hexdigest(), "size_bytes": len(payload),
                "lane_body_containment_verified": False}}}
    receipts = [{"actor_id": 17, "response_actor_id": 17}]
    assert ab.validate_geometry(tmp_path, manifest, receipts, {})["body_containment_verified"] is False
    bad = copy.deepcopy(manifest)
    bad["capture_contract"]["actor_body_geometry"]["actor_id"] = 18
    with pytest.raises(ab.prior.AuditError):
        ab.validate_geometry(tmp_path, bad, receipts, {})
    bad = copy.deepcopy(manifest)
    bad["capture_contract"]["active_map_opendrive"]["sha256"] = "0" * 64
    with pytest.raises(ab.prior.AuditError):
        ab.validate_geometry(tmp_path, bad, receipts, {})


def test_failed_owner_cannot_produce_a_b_metrics(monkeypatch):
    def failed_owner(ledger):
        ledger["owner_result.json"] = {"sha256": "0" * 64}
        return {"exit_code": 1}, {"stopped": {"status": "PASS"}}, {}

    monkeypatch.setattr(ab, "closed_owner", failed_owner)
    monkeypatch.setattr(ab, "audit_complete", lambda *_: pytest.fail("must not read B raw"))
    report, companions = ab.audit()
    assert report["status"] == "COLLECTION_FAILED"
    assert report["comparison"] is None and report["b_metrics"] is None and companions == {}
    assert report["dataset_admitted"] is False
