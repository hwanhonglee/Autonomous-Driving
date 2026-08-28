import numpy as np

from scripts.e2e.vad_calibration_contract import analyze_tf_tree
from scripts.e2e.vad_calibration_contract import pose_error
from scripts.e2e.vad_calibration_contract import quaternion_transform


def test_tf_tree_rejects_two_parents_for_one_camera() -> None:
    identity = np.eye(4)
    report = analyze_tf_tree(
        [
            ("base_link", "camera", identity),
            ("sensor_kit", "camera", identity),
        ],
        "base_link",
        ["camera"],
    )

    assert report["valid"] is False
    assert report["conflicting_parent_children"] == ["camera"]


def test_tf_tree_rejects_a_cycle() -> None:
    identity = np.eye(4)
    report = analyze_tf_tree(
        [("base_link", "camera", identity), ("camera", "base_link", identity)],
        "base_link",
        ["camera"],
    )

    assert report["valid"] is False
    assert report["cycle"] is True


def test_tf_tree_accepts_equivalent_quaternion_signs() -> None:
    positive = quaternion_transform((1.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
    negative = quaternion_transform((1.0, 0.0, 1.0), (0.0, 0.0, 0.0, -1.0))
    report = analyze_tf_tree(
        [("base_link", "camera", positive), ("base_link", "camera", negative)],
        "base_link",
        ["camera"],
    )

    assert report["valid"] is True
    assert report["conflicting_value_edges"] == []


def test_tf_tree_ignores_an_unrelated_cycle() -> None:
    identity = np.eye(4)
    report = analyze_tf_tree(
        [
            ("base_link", "camera", identity),
            ("unrelated_a", "unrelated_b", identity),
            ("unrelated_b", "unrelated_a", identity),
        ],
        "base_link",
        ["camera"],
    )

    assert report["valid"] is True
    assert report["cycle"] is False


def test_pose_error_separates_translation_and_rotation() -> None:
    expected = quaternion_transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    actual = quaternion_transform((0.01, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    translation_error, rotation_error = pose_error(actual, expected)
    assert translation_error == 0.01
    assert rotation_error == 0.0
