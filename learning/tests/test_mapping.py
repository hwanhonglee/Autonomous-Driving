"""HH_260906 - Test fusion plumbing without claiming perception accuracy."""
import pytest
from portable_e2e.mapping import Observation, SemanticEvidenceMap, replay


def obs(t=1, points=((0.1, 0.1, "road", 0.8),), **kw):
    return Observation(t, kw.get("frame_id", "map"), kw.get("source", "oracle_sensor"), points)


def new(**kw):
    return SemanticEvidenceMap("map", "oracle_sensor", **kw)


def test_live_replay_equal():
    frames = [obs(1), obs(2)]
    live = new()
    for frame in frames:
        live.update(frame)
    assert live.snapshot() == replay(frames, frame_id="map", source="oracle_sensor").snapshot()


def test_dense_points_one_vote_and_negative_coordinates():
    m = new()
    m.update(obs(points=((-0.1, 0, "road", .5), (-0.1, 0, "road", .9))))
    assert m.snapshot()["cells"] == [{"ix": -1, "iy": 0, "labels": {
        "road": {"observed_frames": 1, "confidence_sum": .9}}}]


def test_dynamic_not_saved_and_expires_next_frame():
    m = new()
    m.update(obs(points=((0, 0, "vehicle", 1),)))
    assert not m.cells and m.dynamic
    m.update(obs(2, points=()))
    assert not m.dynamic


@pytest.mark.parametrize("frame", [obs(source="learned_prediction"), obs(frame_id="ego"),
    obs(points=((float("nan"), 0, "road", 1),)),
    obs(points=((0, 0, "unknown", 1),)), obs(points=((0, 0, "road", 2),))])
def test_invalid_frame_atomic(frame):
    m = new()
    before = m.snapshot()
    with pytest.raises(ValueError):
        m.update(frame)
    assert m.snapshot() == before


def test_duplicate_and_out_of_order_rejected():
    m = new()
    m.update(obs(2))
    for t in (1, 2):
        with pytest.raises(ValueError):
            m.update(obs(t))


def test_capacity_atomic():
    m = new(max_cells=1)
    m.update(obs())
    before = m.snapshot()
    with pytest.raises(ValueError):
        m.update(obs(2, ((1, 1, "road", 1),)))
    assert m.snapshot() == before


def test_conflicting_labels_preserved():
    m = new()
    m.update(obs(points=((0, 0, "road", 1), (0, 0, "sidewalk", .5))))
    assert set(m.snapshot()["cells"][0]["labels"]) == {"road", "sidewalk"}
    assert m.snapshot()["unobserved_is_unknown"]
