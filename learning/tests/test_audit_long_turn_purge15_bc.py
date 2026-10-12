"""HH_260906 - CPU-only fail-closed tests for the preregistered B/C purge audit."""

import copy

import pytest

from scripts.e2e import audit_long_turn_purge15_bc as bc


def command(output, purge=None):
    argv = [str(output), str(bc.prior.ROUTE), "--target-speed-kmh", "9",
            "--basic-agent-max-steering", "0.45", "--seed", "2026092702"]
    if purge is not None:
        argv += ["--basic-agent-base-min-distance-m", purge]
    return argv


def agent(purge):
    return {"effective_opt_dict": {"base_min_distance": purge,
                                   "max_steering": .45, "distance_ratio": .5,
                                   "lateral_control_dict": {"K_P": 1.95, "K_I": .05, "K_D": .2}},
            "waypoint_purge_lookahead": {"base_min_distance_m": purge,
                                          "distance_ratio_s": .5},
            "collector_cli": {"--basic-agent-base-min-distance-m": purge,
                              "--basic-agent-max-steering": .45},
            "normalized_max_steering": .45}


def test_only_purge_option_and_episode_path_may_change():
    b = command(bc.TRIAL_B / "episode")
    c = command(bc.EPISODE_C, "1.5")
    assert bc.one_factor_arguments(b, c)["all_other_collector_arguments_equal"]
    with pytest.raises(bc.prior.AuditError):
        bc.one_factor_arguments(b, command(bc.EPISODE_C, "1.4"))
    with pytest.raises(bc.prior.AuditError):
        bc.one_factor_arguments(b, c + ["--weather", "WetNoon"])
    with pytest.raises(bc.prior.AuditError):
        bc.one_factor_arguments(b, c + ["--basic-agent-base-min-distance-m", "1.5"])


def test_effective_agent_rejects_peer_factor_or_steering_cap_change():
    b, c = agent(3.), agent(1.5)
    assert bc.one_factor_agent(b, c)["all_other_effective_agent_options_equal"]
    changed = copy.deepcopy(c)
    changed["effective_opt_dict"]["distance_ratio"] = .6
    with pytest.raises(bc.prior.AuditError):
        bc.one_factor_agent(b, changed)
    changed = copy.deepcopy(c)
    changed["normalized_max_steering"] = .40
    with pytest.raises(bc.prior.AuditError):
        bc.one_factor_agent(b, changed)


def test_failed_owner_never_reads_c_episode_or_invents_comparison(monkeypatch):
    def failed_owner(ledger):
        ledger["owner_result.json"] = {"sha256": "0" * 64}
        return {"exit_code": 1}, {"stopped": {"status": "PASS"}}, {}, {}

    monkeypatch.setattr(bc, "closed_owner", failed_owner)
    monkeypatch.setattr(bc, "audit_complete", lambda *_: pytest.fail("must not read C episode"))
    report, companions = bc.audit()
    assert report["status"] == "COLLECTION_FAILED"
    assert report["comparison"] is None and report["c_metrics"] is None
    assert report["lane_gate"] is None and companions == {}
    assert report["dataset_admitted"] is False


def test_lane_exception_is_unsupported_never_pass(monkeypatch):
    def no_geometry(*_args, **_kwargs):
        raise bc.lane.Unsupported("missing lane binding")

    monkeypatch.setattr(bc.lane, "audit_episode", no_geometry)
    result = bc.run_lane_gate()
    assert result["status"] == "UNSUPPORTED"
    assert result["training_data_approved"] is False
    assert "missing lane binding" in result["reason"]
