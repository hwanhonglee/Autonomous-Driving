"""HH_260906 - CPU-only contract checks for Town05 DEV scoring and non-promotion."""
import numpy as np
import pytest

from scripts.e2e import evaluate_map_town05_dev as town05


def test_native_semantic_table_and_wrong_shape_rejection():
    semantic = np.zeros((360, 640, 4), dtype=np.uint8)
    semantic[0, 0, 2] = 1
    semantic[0, 1, 2] = 24
    semantic[0, 2, 2] = 8
    semantic[0, 3, 2] = 255
    target = town05.semantic_target(semantic)
    assert tuple(target[0, :4]) == (1, 3, 5, 0)
    with pytest.raises(ValueError, match="native uint8"):
        town05.semantic_target(semantic[..., :3])


def test_confusion_iou_precision_recall_and_exact_denominator():
    target = np.zeros((360, 640), dtype=np.uint8)
    prediction = np.zeros_like(target)
    target[0, 0] = 1
    target[0, 1] = 1
    prediction[0, 0] = 1
    prediction[0, 2] = 1
    confusion = town05.confusion_for(target, prediction)
    road = town05.class_metrics(confusion)["classes"]["road"]
    assert int(confusion.sum()) == 360 * 640
    assert (road["true_positive_pixels"], road["false_positive_pixels"],
            road["false_negative_pixels"]) == (1, 1, 1)
    assert road["iou"] == pytest.approx(1 / 3)
    assert road["precision"] == road["recall"] == .5
    prediction[0, 3] = 7
    with pytest.raises(ValueError, match="seven-class"):
        town05.confusion_for(target, prediction)


def test_diagnostic_nonregression_never_promotes_candidate():
    parent = {"classes": {"road": {"iou": .80}, "lane_marking": {"iou": .50}}}
    candidate = {"classes": {"road": {"iou": .795}, "lane_marking": {"iou": .485}}}
    gate = town05.delta_and_gate(parent, candidate)
    assert gate["town05_directional_nonregression"] is True
    assert gate["road_iou_delta_c_minus_a_pp"] == pytest.approx(-.5)
    assert gate["lane_iou_delta_c_minus_a_pp"] == pytest.approx(-1.5)
    assert gate["automatic_promotion"] is False
    assert gate["deployment_approved"] is False
    assert gate["sign_used_for_gate"] is False
    candidate["classes"]["lane_marking"]["iou"] = .479
    assert town05.delta_and_gate(parent, candidate)["town05_directional_nonregression"] is False


def test_full_fov_panel_preserves_native_four_panels():
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    target = np.zeros((360, 640), dtype=np.uint8)
    panel = town05.full_fov_panel("frames/0123", rgb, target, target, target)
    assert panel.size == (1280, 816)
    with pytest.raises(ValueError, match="native full FOV"):
        town05.full_fov_panel("frames/0123", rgb[:100], target, target, target)


def test_preflight_rejects_changed_plan_before_any_model_load(tmp_path, monkeypatch):
    plan = tmp_path / "PLAN.md"
    plan.write_text("changed plan")
    monkeypatch.setattr(town05, "OUTPUT", tmp_path)
    monkeypatch.setattr(town05, "PLAN", plan)
    with pytest.raises(ValueError, match="PLAN changed"):
        town05.preflight()


def test_preflight_rejects_changed_source_audit_before_any_model_load(tmp_path, monkeypatch):
    audit = tmp_path / "audit.json"
    audit.write_text("{}")
    monkeypatch.setattr(town05, "OUTPUT", tmp_path)
    monkeypatch.setattr(town05, "AUDIT", audit)
    with pytest.raises(ValueError, match="Epic audit changed"):
        town05.preflight()
