"""HH_260929 - Guard the offline site's maneuver, identity, and signal scope."""

import xml.etree.ElementTree as ET

from scripts.e2e import scout_other_non_test_right_20260929 as scout


def point(option: str, road: int, lane: int, x: float) -> dict:
    return {"road_option": option, "road_id": road, "section_id": 0,
            "lane_id": lane, "x": x, "y": 0.0}


def test_one_contiguous_right_block_only():
    assert scout.right_block([point("LANEFOLLOW", 1, -1, 0),
                              point("RIGHT", 2, -1, 1),
                              point("RIGHT", 3, 1, 2),
                              point("LANEFOLLOW", 4, 1, 3)]) == (1, 2)
    assert scout.right_block([point("RIGHT", 2, -1, 1),
                              point("LANEFOLLOW", 4, 1, 2),
                              point("RIGHT", 5, 1, 3)]) is None
    assert scout.right_block([point("LEFT", 1, -1, 0),
                              point("RIGHT", 2, -1, 1)]) is None


def test_turn_must_be_on_enumerated_connector():
    points = [point("LANEFOLLOW", 2071, -1, 0),
              point("RIGHT", 2187, -1, 1),
              point("RIGHT", 30, -2, 2)]
    assert not scout.right_block_on_connector(points, (1, 2), 2071, -1)
    assert scout.right_block_on_connector(points, (1, 2), 2187, -1)


def test_site_fingerprint_preserves_connector_lane_changes():
    points = [point("RIGHT", 783, -1, -72.0),
              point("RIGHT", 783, -1, -73.0),
              point("RIGHT", 2, 3, -74.0)]
    assert scout.physical_site(points, (0, 2)) == {
        "lane_transitions": [[783, 0, -1], [2, 0, 3]],
        "center_xy_ros_m": [-73.0, 0.0],
    }


def test_direct_signal_is_not_mistaken_for_signal_free_route():
    roads = {
        45: ET.fromstring('<road id="45"><signals><signal id="7"/></signals></road>'),
        9: ET.fromstring('<road id="9"><signals/></road>'),
    }
    assert scout.signal_records(roads, [point("RIGHT", 45, 1, 0),
                                        point("RIGHT", 9, -2, 1)]) == {
        "direct_signal_road_ids": [45], "signal_reference_road_ids": [],
    }


def test_second_road_in_same_junction_is_not_second_site():
    roads = {
        639: ET.fromstring('<road id="639" junction="562"/>'),
        694: ET.fromstring('<road id="694" junction="562"/>'),
        777: ET.fromstring('<road id="777" junction="720"/>'),
    }
    points = [dict(point("RIGHT", road, 1, index), distance_m=50.0 + index)
              for index, road in enumerate((639, 694, 777))]
    assert scout.other_junction_ids(roads, points[:2], 562) == []
    assert scout.other_junction_ids(roads, points, 562) == [720]
