"""HH_260906 - Verify packing and optical/UE axes independently of simulator data."""
import numpy as np
import pytest
from portable_e2e.mapping_projection import project_frame, project_predicted_frame


def fixture():
    raw = np.zeros((4, 4, 4), dtype=np.uint8)
    raw[..., 2] = 255  # optical depth = 255 / (2**24 - 1) * 1000
    sem = np.zeros_like(raw)
    sem[2, 2, 2] = 24
    return raw, sem


def call(raw, sem, matrix=None, **kw):
    return project_frame(raw, sem, np.eye(4) if matrix is None else matrix,
                         frame_id="carla_ue:Town01", timestamp_s=1,
                         fov_deg=90, label_table={24: "lane_marking"}, stride=1, **kw)


def test_bgra_depth_axes_and_translation():
    raw, sem = fixture()
    raw[..., 1] = 10  # 2815 in little-endian RGB packed depth
    matrix = np.eye(4)
    matrix[:3, 3] = [10, 20, 3]
    result = call(raw, sem, matrix)
    assert result.points[0][:2] == pytest.approx((10 + 2815 / 16777215 * 1000, 20))


def test_right_pixel_points_positive_ue_y():
    raw, sem = fixture()
    raw[..., 0] = 1
    sem[:] = 0
    sem[2, 3, 2] = 24
    x, y, *_ = call(raw, sem).points[0]
    assert y == pytest.approx(x / 2)


def test_zero_sky_and_unlabeled_excluded():
    raw, sem = fixture()
    raw[:] = 255
    assert call(raw, sem).points == ()
    raw[:] = 0
    assert call(raw, sem).points == ()


@pytest.mark.parametrize("matrix", [np.zeros((4, 4)), np.eye(3), np.diag([2, 1, 1, 1])])
def test_invalid_transform(matrix):
    with pytest.raises(ValueError):
        call(*fixture(), matrix)


def test_learned_projection_preserves_confidence_and_ue_axes():
    # HH_260929 - A learned semantic mask has no access to CARLA semantic bytes.
    depth = np.ones((4, 4), dtype=np.float32) * 2.0
    prediction = np.zeros((4, 4), dtype=np.uint8)
    prediction[2, 3] = 3
    certainty = np.ones((4, 4), dtype=np.float32)
    certainty[2, 3] = .75
    matrix = np.eye(4)
    matrix[:3, 3] = [10, 20, 3]
    result = project_predicted_frame(
        depth, prediction, certainty, matrix, frame_id="carla_ue:Town01",
        timestamp_s=1., fov_deg=90., label_table={3: "lane_marking"}, stride=1)
    assert result.source == "learned_prediction"
    assert len(result.points) == 1
    assert result.points[0] == pytest.approx((12.0, 21.0, "lane_marking", .75),
                                            abs=1e-6)


def test_learned_projection_keeps_unobserved_unknown():
    depth = np.ones((4, 4), dtype=np.float32) * 2.0
    prediction = np.zeros((4, 4), dtype=np.uint8)
    prediction[2, 2] = 3
    certainty = np.ones((4, 4), dtype=np.float32)
    certainty[2, 2] = .49
    result = project_predicted_frame(
        depth, prediction, certainty, np.eye(4), frame_id="map", timestamp_s=1.,
        fov_deg=90., label_table={3: "lane_marking"}, stride=1)
    assert result.points == ()


@pytest.mark.parametrize("bad_depth,bad_prediction,bad_confidence", [
    (np.zeros((4, 3)), np.zeros((4, 4), dtype=np.uint8), np.ones((4, 4))),
    (np.ones((4, 4)), np.ones((4, 4), dtype=np.float32), np.ones((4, 4))),
    (np.ones((4, 4)), np.zeros((4, 4), dtype=np.uint8), np.full((4, 4), 1.1)),
])
def test_learned_projection_rejects_invalid_sensor_contract(
        bad_depth, bad_prediction, bad_confidence):
    with pytest.raises(ValueError):
        project_predicted_frame(
            bad_depth, bad_prediction, bad_confidence, np.eye(4), frame_id="map",
            timestamp_s=1., fov_deg=90., label_table={3: "lane_marking"})
