"""HH_261002 - Verify the identity-only V7 actual-media adapter without fabricating simulator or model success."""

import ast
from pathlib import Path

import pytest

from scripts.e2e import publish_town07_natural_goal_stop_teacher_v7_20261002 as publication


def test_all_frozen_identities_replaced_and_outputs_are_new():
    assert set(publication.V7_literal_replacement_counts) == set(publication.IDENTITIES)
    assert all(publication.V7_literal_replacement_counts.values())
    assert publication.BATCH.name == "town07_natural_goal_stop_teacher_brakefree_v7"
    assert publication.OUTPUT.name == "14_actual_goal_stop_teacher_v7"
    assert publication.TASK_SCHEMA.endswith(".v7")
    assert publication.FUTURE_QA_SCHEMA.endswith(".v7")
    assert publication.SCHEMA == "carla.actual_goal_stop_teacher_v7_publication.20261002.v1"
    assert publication.CALLER_SCHEMA == publication.SCHEMA + ".caller_pins"
    assert publication.digest(Path(publication.frozen_v6.__file__).resolve()) == publication.FROZEN_PUBLISHER_SHA256


def test_exact_literal_changes_keep_original_function_namespace_private():
    for name in ("render", "publish", "validate_records", "validate_pixels", "validate_future_qa", "main"):
        actual = getattr(publication, name)
        assert actual.__globals__ is vars(publication)
        assert actual is not getattr(publication.frozen_v6, name)
    assert publication.frozen_v6.BATCH.name.endswith("_v6")
    assert publication.frozen_v6.OUTPUT.name == "08_actual_goal_stop_teacher_v6"
    assert publication.frozen_v6.TASK_SCHEMA.endswith(".v6")


@pytest.mark.parametrize("count", [1, 2, 6, 11, 1782])
def test_whole_camera_membership_and_encoded_time_stay_identical(count):
    expected = publication.frozen_v6.sampled_indices(count)
    assert publication.sampled_indices(count) == expected
    cameras = [{"timestamp": index * .1} for index in range(count)]
    durations = publication.playback_durations(cameras, expected)
    assert durations == publication.frozen_v6.playback_durations(cameras, expected)
    assert sum(durations) == round((count - 1) * 100) + 100


@pytest.mark.parametrize("status,failed,source_failed,word", [
    ("PASS_ACTUAL_TEACHER_GOAL_CAPTURE_PENDING_FUTURE_QA", 0, False, "CHECKS_CLEAR_NOT_ADMITTED"),
    ("PASS_ACTUAL_TEACHER_GOAL_CAPTURE_PENDING_FUTURE_QA", 7, False, "FAIL"),
    ("FAIL_RETAINED", 0, True, "FAIL"),
])
def test_annotations_never_promote_teacher_or_remove_failed_qa(status, failed, source_failed, word):
    bundle = dict(diagnostic=dict(status=status), future=dict(
        failed_fixed_window_count=failed, source_capture_failed=source_failed, selected_anchor_count=1665))
    assert word in publication.outcome_annotation(bundle)
    assert f"{failed}/1665" in publication.outcome_annotation(bundle)
    assert "NOT_PROVEN" in publication.outcome_annotation(dict(diagnostic=dict(status=status), future=None))


def test_provider_drift_rejected_before_any_archive_read(monkeypatch, tmp_path):
    monkeypatch.setattr(publication, "digest", lambda path: "0" * 64)
    invoked = []
    monkeypatch.setattr(publication, "_identity_load_archive", lambda *args: invoked.append(args))
    with pytest.raises(Exception, match="renderer changed"):
        publication.load_archive(tmp_path / "not_read.json", "1" * 64)
    assert not invoked


def test_provider_pin_is_added_and_rechecked_after_original_full_validation(monkeypatch, tmp_path):
    provider = Path(publication.frozen_v6.__file__).resolve()
    monkeypatch.setattr(publication, "digest", lambda path: publication.FROZEN_PUBLISHER_SHA256)
    pins = {}
    monkeypatch.setattr(publication, "_identity_load_archive", lambda *args: dict(pins=pins))
    invocations = []
    monkeypatch.setattr(publication, "add_pin", lambda p, path, sha: p.update({str(path): sha}))
    monkeypatch.setattr(publication, "recheck", lambda p: invocations.append(dict(p)))
    assert publication.load_archive(tmp_path / "synthetic.json", "1" * 64)["pins"] is pins
    assert pins == {str(provider): publication.FROZEN_PUBLISHER_SHA256}
    assert invocations == [pins]


def test_no_optimizer_simulator_or_generated_imagery_entrypoints():
    text = Path(publication.__file__).read_text()
    tree = ast.parse(text)
    assert not any(isinstance(n, (ast.Import, ast.ImportFrom)) and "torch" in ast.unparse(n) for n in ast.walk(tree))
    assert "NO learned-model driving" in Path(publication.frozen_v6.__file__).read_text()
    assert publication.publish.__globals__["OUTPUT"] == publication.OUTPUT
