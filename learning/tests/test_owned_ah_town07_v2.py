"""HH_261001 - Distinguish the intended bounded diagnostic from arbitrary capture failures."""

from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as owner


def test_only_exact_budget_is_classified_as_diagnostic_completion():
    manifest = dict(status="failed", error=owner.BUDGET_ERROR,
                    capture_contract=dict(physics_hz=20., camera_hz=10., maximum_total_duration_sec=20.))
    assert owner.classify_bounded_capture(manifest, 400, 200)
    assert not owner.classify_bounded_capture(manifest, 399, 200)
    assert not owner.classify_bounded_capture(manifest, 400, 199)
    assert not owner.classify_bounded_capture(dict(manifest, error="CollectionError: camera timeout"), 400, 200)
    assert not owner.classify_bounded_capture(dict(manifest, status="complete"), 400, 200)
