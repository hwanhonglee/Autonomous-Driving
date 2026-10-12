"""HH_260929 - Guard the frozen Town12 scope and exact connector identity."""

from types import SimpleNamespace
import hashlib

import pytest

from scripts.e2e import scout_town12_right_static_20260929 as scout
from scripts.e2e import scout_town12_right_static_20260929_v2 as v2
from scripts.e2e import scout_town12_right_static_20260929_v3 as v3


def test_preregistered_sources_and_six_scopes_are_present():
    manifest = scout.preregistered_inputs()
    assert scout.SCOPES_M == ((60, 100), (60, 120), (70, 100),
                              (70, 120), (80, 100), (80, 120))
    assert manifest["capture_contract"]["actor_body_geometry"]["actor_type_id"] == \
        "vehicle.toyota.prius"


def test_right_block_must_touch_exact_section_and_lane():
    entry = SimpleNamespace(road_id=9, section_id=1, lane_id=-1)
    points = [
        {"road_id": 9, "section_id": 0, "lane_id": -1},
        {"road_id": 9, "section_id": 1, "lane_id": -2},
        {"road_id": 9, "section_id": 1, "lane_id": -1},
    ]
    assert not scout.connector_on_block(points, (0, 1), entry)
    assert scout.connector_on_block(points, (1, 2), entry)


def test_v2_only_classifies_typed_unsupported_seam(monkeypatch):
    assert v2.preregistered_inputs()["coordinate_contract"]["wheelbase_m"] == 2.85

    def unsupported(*_args, **_kwargs):
        raise scout.nominal.lane.Unsupported("authorized route lanes have a gap wider than 5mm")

    monkeypatch.setattr(v2, "ORIGINAL_CLEARANCE", unsupported)
    assert v2.guarded_clearance()["status"] == "UNSUPPORTED"


def test_v3_reuses_only_identical_xodr_bytes():
    source = "<OpenDRIVE/>"
    constructed = []

    def make_map(name, xodr):
        result = object()
        constructed.append((name, xodr, result))
        return result

    proxy = v3.CachedCarlaMap(
        SimpleNamespace(Map=make_map), hashlib.sha256(source.encode()).hexdigest())
    first = proxy.Map("first", source)
    assert proxy.Map("second", source) is first
    assert proxy.requests == 2 and len(constructed) == 1
    with pytest.raises(scout.ScoutError, match="differs from pinned Town12 XODR"):
        proxy.Map("wrong", source + " ")
