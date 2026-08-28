import argparse
import importlib.util
import math
from pathlib import Path

from lxml import etree
import numpy as np
import pytest
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/finalize_xodr_lanelet_map.py"


def load_module():
    spec = importlib.util.spec_from_file_location("finalize_xodr_lanelet_map", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def write_map_fixture(tmp_path: Path, turn_direction: str = "right"):
    source_osm = tmp_path / "source.osm"
    source_osm.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lon="1" lat="2"/>
  <node id="2" lon="2" lat="2"/>
  <node id="3" lon="2" lat="3"/>
  <node id="4" lon="1" lat="4"/>
  <node id="5" lon="2" lat="4"/>
  <node id="6" lon="2" lat="5"/>
  <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/></way>
  <way id="11"><nd ref="4"/><nd ref="5"/><nd ref="6"/></way>
  <relation id="100">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
  <relation id="101">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/>
  </relation>
  <relation id="102">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
    <tag k="turn_direction" v="{turn_direction}"/>
  </relation>
</osm>
""",
        encoding="utf-8",
    )
    source_xodr = tmp_path / "source.xodr"
    source_xodr.write_text(
        """<OpenDRIVE>
  <header west="-100" east="100" south="-100" north="100">
    <geoReference>+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs</geoReference>
  </header>
</OpenDRIVE>
""",
        encoding="utf-8",
    )
    output_osm = tmp_path / "output.osm"
    args = argparse.Namespace(
        source_osm=source_osm,
        source_xodr=source_xodr,
        output_osm=output_osm,
        translation_x_m=10.0,
        translation_y_m=20.0,
        translation_z_m=3.0,
        yaw_rad=math.pi / 2.0,
        waypoint_resolution_m=0.25,
        bounds_margin_m=0.0,
        speed_limit_kmh=30.0,
        map_version="fixture",
    )
    return args


def patch_surface(module, monkeypatch):
    class IdentityProjector:
        @staticmethod
        def transform(longitude, latitude):
            return longitude, latitude

    source_xy = np.asarray(
        [(1, 2), (2, 2), (2, 3), (1, 4), (2, 4), (2, 5)],
        dtype=np.float64,
    )
    elevation = np.asarray([0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    monkeypatch.setattr(module, "xodr_projection", lambda _root: IdentityProjector())
    monkeypatch.setattr(
        module,
        "xodr_waypoint_surface",
        lambda _path, _resolution: (cKDTree(source_xy), elevation, len(source_xy)),
    )


def test_prune_removes_dangling_references_and_relation_chains():
    module = load_module()
    root = etree.fromstring(
        b"""<osm>
  <node id="1"/><node id="2"/><node id="3"/>
  <way id="10"><nd ref="1"/><nd ref="2"/></way>
  <way id="11"><nd ref="1"/><nd ref="999"/></way>
  <way id="12"><nd ref="1"/><nd ref="3"/></way>
  <relation id="20">
    <member type="way" ref="12" role="left"/>
    <member type="way" ref="404" role="right"/>
    <tag k="type" v="lanelet"/>
  </relation>
  <relation id="21">
    <member type="way" ref="10" role="refers"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
  <relation id="22">
    <member type="relation" ref="21" role="refers"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
  <relation id="23">
    <member type="way" ref="12" role="left"/>
    <member type="way" ref="12" role="right"/>
    <member type="relation" ref="22" role="regulatory_element"/>
    <tag k="type" v="lanelet"/>
  </relation>
  <relation id="24">
    <member type="way" ref="12" role="refers"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
</osm>"""
    )
    nodes = root.findall("node")
    result = module.prune_out_of_bounds_geometry(
        root,
        nodes,
        np.asarray([(0.0, 0.0), (50.0, 50.0), (1.0, 1.0)]),
        (-10.0, 10.0, -10.0, 10.0),
    )

    assert result == {"node_count": 1, "way_count": 2, "relation_count": 3}
    assert {node.get("id") for node in root.findall("node")} == {"1", "3"}
    assert {way.get("id") for way in root.findall("way")} == {"12"}
    assert {relation.get("id") for relation in root.findall("relation")} == {
        "23",
        "24",
    }
    assert root.xpath("relation[@id='23']/member[@type='relation']") == []

    valid_ids = {
        "node": {node.get("id") for node in root.findall("node")},
        "way": {way.get("id") for way in root.findall("way")},
        "relation": {relation.get("id") for relation in root.findall("relation")},
    }
    for reference in root.xpath("way/nd"):
        assert reference.get("ref") in valid_ids["node"]
    for member in root.xpath("relation/member"):
        assert member.get("ref") in valid_ids[member.get("type")]


def test_prune_rejects_coordinate_count_mismatch():
    module = load_module()
    root = etree.fromstring(b'<osm><node id="1"/><node id="2"/></osm>')

    with pytest.raises(module.FinalizationError, match="must match OSM nodes"):
        module.prune_out_of_bounds_geometry(
            root,
            root.findall("node"),
            np.asarray([(0.0, 0.0)]),
            (-1.0, 1.0, -1.0, 1.0),
        )


def test_prune_collapsed_lanelets_removes_relation_references():
    module = load_module()
    root = etree.fromstring(
        b"""<osm>
  <way id="10"><nd ref="1"/><nd ref="2"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/></way>
  <relation id="20">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/>
  </relation>
  <relation id="21">
    <member type="relation" ref="20" role="refers"/>
    <tag k="type" v="regulatory_element"/>
  </relation>
</osm>"""
    )
    local_xy = {
        "1": (0.0, 0.0),
        "2": (0.0, 0.0),
        "3": (1.0, 0.0),
        "4": (1.0, 1.0),
    }

    result = module.prune_collapsed_lanelets(root, local_xy)

    assert result == {"relation_count": 1, "relation_ids": ["20"]}
    assert root.find("relation[@id='20']") is None
    assert root.xpath("relation[@id='21']/member") == []


def test_finalize_transforms_coordinates_and_classifies_untyped_shoulders(
    tmp_path, monkeypatch
):
    module = load_module()
    args = write_map_fixture(tmp_path)
    source_before = args.source_osm.read_bytes()
    patch_surface(module, monkeypatch)

    result = module.finalize(args)

    tree = etree.parse(args.output_osm)
    node = tree.find(".//node[@id='1']")
    assert float(tags(node)["local_x"]) == pytest.approx(8.0)
    assert float(tags(node)["local_y"]) == pytest.approx(21.0)
    assert float(tags(node)["ele"]) == pytest.approx(3.5)

    relations = {
        relation.get("id"): tags(relation)
        for relation in tree.findall(".//relation")
    }
    assert all(tags(way)["type"] == "virtual" for way in tree.findall(".//way"))
    assert "turn_direction" not in relations["100"]
    assert relations["101"]["subtype"] == "road_shoulder"
    assert relations["101"]["location"] == "urban"
    assert "turn_direction" not in relations["101"]
    assert relations["102"]["turn_direction"] == "right"
    assert result["road_lanelet_count"] == 2
    assert result["inferred_road_lanelet_count"] == 0
    assert result["inferred_road_shoulder_lanelet_count"] == 1
    assert result["virtual_boundary_count"] == 2
    assert result["pruned_collapsed_lanelets"] == {
        "relation_count": 0,
        "relation_ids": [],
    }
    assert result["turn_direction_counts"] == {
        "left": 0,
        "right": 1,
        "straight": 0,
    }
    assert args.source_osm.read_bytes() == source_before


def test_finalize_rejects_invalid_explicit_turn_direction(tmp_path, monkeypatch):
    module = load_module()
    args = write_map_fixture(tmp_path, turn_direction="uturn")
    patch_surface(module, monkeypatch)

    with pytest.raises(module.FinalizationError, match="invalid turn_direction"):
        module.finalize(args)

    assert not args.output_osm.exists()


def test_atomic_write_replaces_destination_and_cleans_temporary_file(tmp_path):
    module = load_module()
    destination = tmp_path / "map.osm"
    destination.write_text("old", encoding="ascii")
    tree = etree.ElementTree(etree.fromstring(b'<osm generator="fixture"/>'))

    module.atomic_write(tree, destination)

    assert etree.parse(destination).getroot().get("generator") == "fixture"
    assert list(tmp_path.glob(f".{destination.name}.*")) == []


def test_atomic_write_preserves_destination_when_replace_fails(tmp_path, monkeypatch):
    module = load_module()
    destination = tmp_path / "map.osm"
    destination.write_bytes(b"old")
    tree = etree.ElementTree(etree.fromstring(b"<osm/>"))

    def fail_replace(_source, _destination):
        raise OSError("replace failed")

    monkeypatch.setattr(module.os, "replace", fail_replace)
    with pytest.raises(OSError, match="replace failed"):
        module.atomic_write(tree, destination)

    assert destination.read_bytes() == b"old"
    assert list(tmp_path.glob(f".{destination.name}.*")) == []
