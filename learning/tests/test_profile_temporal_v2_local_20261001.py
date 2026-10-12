"""HH_261001 - Keep local profiling create-only and keep CPU timings distinct from CUDA."""

import pytest
from scripts.e2e import profile_temporal_v2_local_20261001 as profile


def test_timing_statistics_retain_every_repetition():
    actual = profile.statistics([.004, .001, .002, .003])
    assert actual["count"] == 4
    assert actual["mean_ms"] == pytest.approx(2.5)
    assert actual["p95_ms"] == actual["maximum_ms"] == 4.
    assert actual["individual_ms"] == [4., 1., 2., 3.]


def test_profile_does_not_overwrite_previous_evidence(tmp_path):
    result = tmp_path / "existing.json"
    result.write_text("original")
    with pytest.raises(ValueError, match="new profile"):
        profile.main(["--checkpoint", str(tmp_path/"checkpoint"), "--data", str(tmp_path),
                      "--output", str(result), "--device", "cpu"])
    assert result.read_text() == "original"


def test_profile_requires_personal_workspace_result_scope(tmp_path):
    with pytest.raises(ValueError, match="local training diagnostics"):
        profile.main(["--checkpoint", str(tmp_path/"checkpoint"), "--data", str(tmp_path),
                      "--output", str(tmp_path/"new"), "--device", "cpu"])
