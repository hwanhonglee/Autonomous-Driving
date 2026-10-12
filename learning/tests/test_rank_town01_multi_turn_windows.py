"""HH_260906 - Reject unowned or nominally unsafe multi-maneuver turn windows."""

from __future__ import annotations

import json

import pytest

from scripts.e2e import rank_town01_multi_turn_windows as ranker


def test_completed_owner_binds_exact_catalog_and_archived_sources() -> None:
    plan, result, catalog = ranker.completed_owner(ranker.OWNER)
    assert plan["search_mode"] == "multi_maneuver_right"
    assert plan["split_intent"] == "TRAIN_CANDIDATE"
    assert result["catalog_sha256"] == ranker.CATALOG_SHA256
    assert result["right_candidate_count"] == len(catalog["routes"]) == 16
    assert result["capture_executed"] is False


def test_wrong_owner_root_is_not_ranked(tmp_path) -> None:
    with pytest.raises(ranker.prior.InvalidEvidence, match="pinned multi-maneuver owner"):
        ranker.completed_owner(tmp_path)


def test_selected_site_uses_right_block_not_whole_route() -> None:
    _, _, catalog = ranker.completed_owner(ranker.OWNER)
    entry = next(item for item in catalog["routes"] if item["id"] == "town01_right_s0006_p03")
    route, _ = ranker.read(ranker.OWNER / entry["path"], entry["sha256"])
    selected = ranker.window.select_window(route)["selected"]
    site = ranker.fingerprint(route, selected)
    assert site[0] == ((279, 0, -1), (4, 0, 1))
    assert site[1][0] == pytest.approx(332.95578, abs=0.01)
    known, _, _ = ranker.prior.known_sites()
    assert all(not ranker.prior.same_site(site, old["fingerprint"]) for old in known)


def test_real_multi_rank_rejects_both_same_site_nominal_failures(tmp_path) -> None:
    output = tmp_path / "rank.json"
    report = ranker.audit(ranker.OWNER, output)
    assert json.loads(output.read_text()) == report
    assert report["right_route_count"] == 16
    assert report["static_window_candidate_count"] == 2
    assert report["selected_physical_site_count"] == 1
    assert report["provisional_site_count"] == 0
    assert report["status"] == "NO_NOMINAL_SAFE_NOVEL_CANDIDATE"
    assert report["capture_authorized"] is False
    assert report["dataset_admitted"] is False
    assert report["training_executed"] is False
    rows = {row["route_id"]: row for row in report["routes"]}
    assert sum(row["window"]["status"] == "UNSUPPORTED" for row in rows.values()) == 9
    assert sum(row["window"]["status"] == "NO_ELIGIBLE_WINDOW" for row in rows.values()) == 5
    for identity in ("town01_right_s0006_p03", "town01_right_s0007_p02"):
        row = rows[identity]
        assert row["known_site_matches"] == []
        assert row["window"]["selected"]["physical_turn_preflight"]["status"] == "PASS"
        assert row["nominal"]["status"] == "FAIL"
        assert row["nominal"]["production_lane_gate_supported"] is True
        assert row["nominal"]["raw_body_outside_count"] == 0
        assert row["nominal"]["inset_body_outside_count"] == 25
        assert row["nominal"]["raw_sweep_outside_count"] == 0
        assert row["nominal"]["inset_sweep_outside_count"] == 42
        assert row["nominal"]["maximum_inset_outside_m2"] < 0.025
        assert "nominal_body_or_sweep_outside_10cm_inner_lane" in row["exclusion_reasons"]
        assert row["capture_authorized"] is False


def test_missing_catalog_route_hash_is_rejected_before_ranking(monkeypatch, tmp_path) -> None:
    original = ranker.read

    def altered(path, expected_sha=None):
        data, digest = original(path, expected_sha)
        if path == ranker.OWNER / "route_catalog.json":
            data["routes"][0].pop("sha256")
        return data, digest

    monkeypatch.setattr(ranker, "read", altered)
    with pytest.raises(ranker.prior.InvalidEvidence, match="route identity, path or SHA invalid"):
        ranker.audit(ranker.OWNER, tmp_path / "rank.json")
    assert not (tmp_path / "rank.json").exists()


def test_explicit_window_default_preserves_v5_nominal_contract() -> None:
    _, manifest, _ = ranker.prior.known_sites()
    path = ranker.prior.C_EPISODE / "route.json"
    route, _ = ranker.read(path)
    import carla

    xodr = (ranker.prior.C_EPISODE / "active_map_opendrive.xodr").read_bytes()
    default = ranker.prior.provisional_nominal_clearance(route, manifest, xodr, carla)
    explicit = ranker.prior.provisional_nominal_clearance(route, manifest, xodr, carla,
                                                          start_m=30.0, end_m=120.0)
    assert default == explicit
    assert default["raw_body_outside_count"] == 0
