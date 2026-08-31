import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/setup_custom_full_map.py"
MANIFEST = ROOT / "scripts/e2e/custom_map_bundles.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("setup_custom_full_map", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_fixture_manifest(tmp_path: Path) -> Path:
    osm = tmp_path / "source.osm"
    osm.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
  <node id="2" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="0"/></node>
  <node id="3" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="2"/></node>
  <node id="4" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="2"/></node>
  <way id="10"><nd ref="1"/><nd ref="2"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/></way>
  <relation id="20">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
</osm>
""",
        encoding="utf-8",
    )
    pcd = tmp_path / "source.pcd"
    header = (
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH 1\n"
        "HEIGHT 1\n"
        "POINTS 1\n"
        "DATA binary\n"
    ).encode("ascii")
    pcd.write_bytes(header + struct.pack("<fff", 1.0, 2.0, 3.0))
    reference = tmp_path / "source.xodr"
    reference.write_text("<OpenDRIVE/>", encoding="ascii")

    def asset(path: Path, provenance: str) -> dict:
        return {
            "path": path.name,
            "sha256": sha256(path),
            "size_bytes": path.stat().st_size,
            "provenance": provenance,
        }

    document = {
        "schema_version": 1,
        "suite_id": "test_custom_maps",
        "description": "Temporary map fixture.",
        "profiles": {
            "fixture": {
                "display_name": "Fixture",
                "status": "test",
                "canonical_carla_map": "/Game/Test/Fixture",
                "target_name": "fixture_full",
                "selection_reason": "Unit-test source selection.",
                "projector": {"projector_type": "Local"},
                "carla_to_map_transform": {
                    "kind": "planar_rigid_with_vertical_offset",
                    "x_m": 1.0,
                    "y_m": 2.0,
                    "z_m": -3.0,
                    "yaw_rad": 0.1,
                    "confidence": "single_anchor_empirical_3d",
                    "source": "fixture",
                },
                "bundle_sources": {
                    "lanelet2_map": asset(osm, "Fixture Lanelet2."),
                    "pointcloud_map": asset(pcd, "Fixture point cloud."),
                },
                "lanelet2_derivation": None,
                "expected": {
                    "osm_nodes": 4,
                    "osm_ways": 2,
                    "osm_road_lanelets": 1,
                    "pcd_points": 1,
                },
                "reference_assets": {
                    "roadrunner_opendrive": asset(reference, "Fixture OpenDRIVE.")
                },
                "rejected_alternatives": [
                    {
                        "id": "old_fixture",
                        "paths": ["old.osm"],
                        "reason": "It belongs to another generation.",
                    }
                ],
            }
        },
    }
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return manifest


def inspect_fixture(module, tmp_path: Path):
    manifest = write_fixture_manifest(tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    return manifest_path, module.inspect_profile(document, "fixture", manifest_path)


def write_derived_fixture_manifest(module, tmp_path: Path) -> Path:
    manifest = write_fixture_manifest(tmp_path)
    osm = tmp_path / "source.osm"
    tree = ET.parse(osm)
    root = tree.getroot()
    relation = root.find("relation")
    assert relation is not None
    centerline = ET.Element("way", {"id": "12"})
    ET.SubElement(centerline, "nd", {"ref": "1"})
    ET.SubElement(centerline, "nd", {"ref": "2"})
    root.insert(list(root).index(relation), centerline)
    relation.insert(
        2,
        ET.Element(
            "member", {"type": "way", "ref": "12", "role": "centerline"}
        ),
    )
    ET.indent(tree, space="  ")
    tree.write(osm, encoding="utf-8", xml_declaration=True)

    document = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    profile = document["profiles"]["fixture"]
    source_asset = profile["bundle_sources"]["lanelet2_map"]
    source_asset["sha256"] = sha256(osm)
    source_asset["size_bytes"] = osm.stat().st_size
    profile["expected"]["osm_ways"] = 5
    derivation = {
        "schema_version": 1,
        "kind": "reverse_lanelet_boundary_ways",
        "relation_id": "20",
        "expected_centerline_way_id": "12",
        "boundary_way_clones": [
            {
                "role": "left",
                "source_way_id": "10",
                "derived_way_id": "30",
                "expected_source_first_node_id": "1",
                "expected_source_last_node_id": "2",
            },
            {
                "role": "right",
                "source_way_id": "11",
                "derived_way_id": "31",
                "expected_source_first_node_id": "3",
                "expected_source_last_node_id": "4",
            },
        ],
        "expected_output_sha256": "0" * 64,
        "expected_output_size_bytes": 1,
        "provenance": "Fixture-only boundary orientation repair.",
    }
    _, result, _ = module.derive_lanelet2_map(
        osm,
        derivation,
        source_sha256=source_asset["sha256"],
        verify_expected_output=False,
    )
    derivation["expected_output_sha256"] = result["output_sha256"]
    derivation["expected_output_size_bytes"] = result["output_size_bytes"]
    profile["lanelet2_derivation"] = derivation
    manifest.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return manifest


def inspect_derived_fixture(module, tmp_path: Path):
    manifest = write_derived_fixture_manifest(module, tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    return manifest_path, module.inspect_profile(document, "fixture", manifest_path)


def test_canonical_profiles_pin_the_selected_map_generations():
    module = load_module()
    document, path = module.load_manifest(MANIFEST)

    assert path == MANIFEST.resolve()
    assert set(document["profiles"]) == {
        "c_track_simulation",
        "c_track_simulation_xodr_current",
        "c_track_virtual_lanelet_only_reference",
        "woraksan_simulation_current",
    }
    c_track = document["profiles"]["c_track_simulation"]
    assert c_track["bundle_sources"]["lanelet2_map"]["path"].endswith(
        "lanelet2_maps_(c_track_test).osm"
    )
    assert "moved" not in c_track["bundle_sources"]["lanelet2_map"]["path"]
    assert c_track["lanelet2_derivation"] is None
    assert c_track["expected"]["pcd_points"] == 64_290_986
    assert c_track["carla_to_map_transform"] == {
        "kind": "xy_yaw_identity_with_vertical_offset",
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": -15.0,
        "yaw_rad": 0.0,
        "confidence": "empirical_settled_state_vertical_offset",
        "source": "Settled CARLA state to Lanelet elevation residual; median is approximately -14.943 m.",
    }

    current_c_track = document["profiles"]["c_track_simulation_xodr_current"]
    assert current_c_track["target_name"] == "C_track_1_0_7_xodr_full"
    assert current_c_track["bundle_sources"]["lanelet2_map"]["path"].endswith(
        "data/generated/xodr_lanelet/c_track_autoware.osm"
    )
    assert current_c_track["bundle_sources"]["pointcloud_map"]["path"].endswith(
        "pointcloud_map (CARLA_C_track_LIOSAM_transform_RoadRunner_C_track_v1_0_7).pcd"
    )
    assert current_c_track["expected"] == {
        "osm_nodes": 89_579,
        "osm_ways": 1_785,
        "osm_road_lanelets": 395,
        "pcd_points": 49_337_228,
    }
    assert current_c_track["carla_to_map_transform"]["x_m"] == 0.0
    assert current_c_track["carla_to_map_transform"]["y_m"] == 0.0
    assert current_c_track["carla_to_map_transform"]["z_m"] == -15.0
    assert current_c_track["carla_to_map_transform"]["yaw_rad"] == 0.0

    lanelet_only = document["profiles"]["c_track_virtual_lanelet_only_reference"]
    assert lanelet_only["status"] == (
        "reference_only_limited_xy_coverage_closed_loop_and_z_unvalidated"
    )
    assert lanelet_only["target_name"] == "C_track_virtual_lanelet_only_reference"
    assert lanelet_only["projector"] == {"projector_type": "Local"}
    assert lanelet_only["lanelet2_derivation"] is None
    assert lanelet_only["bundle_sources"]["lanelet2_map"] == {
        "path": "/home/hong/Downloads/Driving_Map_Set/Driving Map Set/Lanelet/Virtual/lanelet2_map (C_track_v1.0.8_tl_height_point_height)_52SCF0.osm",
        "sha256": "a366b9e7ff36ef900d7160f175e7e0a6edecdaa2177fbe633b2704f032ed4f60",
        "size_bytes": 2_936_540,
        "provenance": "Supplied Driving Map Set Virtual v1.0.8 Lanelet2 map in the _52SCF0 geographic frame; runtime uses its local_x/local_y coordinates through the Local projector.",
    }
    assert lanelet_only["bundle_sources"]["pointcloud_map"]["sha256"] == (
        "3eae5f4a1a6dd72d3753516d428bdb8d4a8e6d90d2d1ec459f2caa60faa52a75"
    )
    assert lanelet_only["bundle_sources"]["pointcloud_map"]["size_bytes"] == (
        592_046_914
    )
    assert lanelet_only["expected"] == {
        "osm_nodes": 13_172,
        "osm_ways": 1_209,
        "osm_road_lanelets": 518,
        "pcd_points": 49_337_228,
    }
    assert lanelet_only["carla_to_map_transform"] == {
        "kind": "xy_yaw_identity_reference_only_z_placeholder",
        "x_m": 0.0,
        "y_m": 0.0,
        "z_m": 0.0,
        "yaw_rad": 0.0,
        "confidence": "shared_coverage_xy_reference_only_z_unvalidated",
        "source": "Supplied local_x/local_y geometry and Virtual asset comparison; z=0 is an unvalidated placeholder and must not be used as a calibrated CARLA-to-map vertical transform.",
    }
    assert "reference-only alignment" in " ".join(
        module.alignment_warnings(lanelet_only["carla_to_map_transform"])
    )
    assert "91.4 m" in lanelet_only["selection_reason"]
    assert "no closed-loop or vertical alignment validation" in lanelet_only[
        "selection_reason"
    ]
    assert lanelet_only["reference_assets"]["same_local_geometry_52scf60"] == {
        "path": "/home/hong/Downloads/Driving_Map_Set/Driving Map Set/Lanelet/Virtual/lanelet2_map (C_track_v1.0.8_tl_height_point_height)_52SCF60.osm",
        "sha256": "1dd9ec1cd816ad231cfb7fca672a1367d8c74dd573cdc591c2228caee9f1a2ee",
        "size_bytes": 2_936_461,
        "provenance": "Alternate supplied geographic frame with the same local_x/local_y geometry and object counts as the selected _52SCF0 reference.",
    }
    assert lanelet_only["rejected_alternatives"] == []

    woraksan = document["profiles"]["woraksan_simulation_current"]
    assert woraksan["bundle_sources"]["lanelet2_map"]["path"] == (
        "/home/hong/camrod_ws/src/lanelet2_maps.osm"
    )
    assert woraksan["bundle_sources"]["pointcloud_map"]["path"].endswith(
        "georef_glim_worak_v2_binary.pcd"
    )
    assert woraksan["expected"]["pcd_points"] == 6_940_175
    assert woraksan["expected"]["osm_ways"] == 238
    derivation = woraksan["lanelet2_derivation"]
    assert derivation["relation_id"] == "4584"
    assert derivation["expected_centerline_way_id"] == "6568"
    assert {
        (item["role"], item["source_way_id"], item["derived_way_id"])
        for item in derivation["boundary_way_clones"]
    } == {("left", "2945", "7000"), ("right", "4583", "7001")}
    assert derivation["expected_output_sha256"] == (
        "90a9adbfe15c5cc4e8d8982cac2bda55a71367038301c4da75c265b644758903"
    )
    assert woraksan["carla_to_map_transform"]["x_m"] == pytest.approx(
        6.979673792021728
    )
    assert woraksan["carla_to_map_transform"]["y_m"] == pytest.approx(
        14.34904187519156
    )
    assert woraksan["carla_to_map_transform"]["z_m"] == pytest.approx(-33.0)
    assert woraksan["carla_to_map_transform"]["yaw_rad"] == pytest.approx(
        -0.0488802969776283
    )
    rejected_paths = {
        item
        for rejected in woraksan["rejected_alternatives"]
        for item in rejected["paths"]
    }
    assert "/home/hong/Documents/RoadRunner/Woraksan/Assets/Maps/lanelet2_maps.osm" in rejected_paths
    assert all(
        source["path"] not in rejected_paths
        for source in woraksan["bundle_sources"].values()
    )


def test_setup_uses_symlinks_and_writes_runtime_transform(tmp_path: Path):
    module = load_module()
    manifest_path, inspection = inspect_fixture(module, tmp_path)
    target = tmp_path / "target"

    result = module.prepare_bundle(inspection, manifest_path, target_dir=target)

    assert result["dry_run"] is False
    assert (target / "lanelet2_map.osm").is_symlink()
    assert (target / "pointcloud_map.pcd").is_symlink()
    assert (target / "lanelet2_map.osm").resolve() == tmp_path / "source.osm"
    assert (target / "pointcloud_map.pcd").resolve() == tmp_path / "source.pcd"
    assert (target / "map_projector_info.yaml").read_text() == "projector_type: Local\n"
    metadata = json.loads((target / "map_bundle.json").read_text())
    assert metadata["profile"] == "fixture"
    assert metadata["carla_to_map_transform"] == {
        "x_m": 1.0,
        "y_m": 2.0,
        "z_m": -3.0,
        "yaw_rad": 0.1,
    }
    assert metadata["bundle_sources"]["pointcloud_map"]["sha256"] == sha256(
        tmp_path / "source.pcd"
    )
    assert "3D preflight required" in " ".join(metadata["warnings"])

    repeated = module.prepare_bundle(inspection, manifest_path, target_dir=target)
    assert set(repeated["actions"].values()) == {"keep"}


def test_derived_lanelet_clones_reversed_bounds_without_mutating_source(tmp_path: Path):
    module = load_module()
    manifest_path, inspection = inspect_derived_fixture(module, tmp_path)
    target = tmp_path / "derived-target"
    source = tmp_path / "source.osm"
    source_before = source.read_bytes()

    result = module.prepare_bundle(inspection, manifest_path, target_dir=target)

    lanelet = target / "lanelet2_map.osm"
    assert result["actions"][str(lanelet)] == "create"
    assert lanelet.is_file()
    assert not lanelet.is_symlink()
    assert source.read_bytes() == source_before

    root = ET.parse(lanelet).getroot()
    ways = {way.attrib["id"]: way for way in root.findall("way")}
    refs = lambda way: [node.attrib["ref"] for node in way.findall("nd")]
    assert refs(ways["10"]) == ["1", "2"]
    assert refs(ways["11"]) == ["3", "4"]
    assert refs(ways["30"]) == ["2", "1"]
    assert refs(ways["31"]) == ["4", "3"]
    relation = next(item for item in root.findall("relation") if item.attrib["id"] == "20")
    members = {
        member.attrib["role"]: member.attrib["ref"]
        for member in relation.findall("member")
    }
    assert members == {"left": "30", "right": "31", "centerline": "12"}

    metadata = json.loads((target / "map_bundle.json").read_text(encoding="utf-8"))
    derivation = metadata["lanelet2_derivation"]["result"]
    assert derivation["source_sha256"] == sha256(source)
    assert derivation["output_sha256"] == sha256(lanelet)
    assert derivation["polygon_geometry_preserved"] is True
    assert metadata["structural_inspection"]["lanelet2_source"]["ways"] == 3
    assert metadata["structural_inspection"]["lanelet2"]["ways"] == 5

    repeated = module.prepare_bundle(inspection, manifest_path, target_dir=target)
    assert set(repeated["actions"].values()) == {"keep"}


def test_derivation_rejects_fresh_way_id_collision(tmp_path: Path):
    module = load_module()
    manifest = write_derived_fixture_manifest(module, tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    document["profiles"]["fixture"]["lanelet2_derivation"]["boundary_way_clones"][0][
        "derived_way_id"
    ] = "12"

    with pytest.raises(module.BundleError, match="derived way id collision: 12"):
        module.inspect_profile(document, "fixture", manifest_path)


def test_derivation_rejects_relation_boundary_mismatch(tmp_path: Path):
    module = load_module()
    manifest = write_derived_fixture_manifest(module, tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    clone = document["profiles"]["fixture"]["lanelet2_derivation"][
        "boundary_way_clones"
    ][0]
    clone["source_way_id"] = "12"

    with pytest.raises(module.BundleError, match="relation 20 left way mismatch"):
        module.inspect_profile(document, "fixture", manifest_path)


def test_derived_lanelet_symlink_migration_requires_explicit_refresh(tmp_path: Path):
    module = load_module()
    manifest_path, inspection = inspect_derived_fixture(module, tmp_path)
    target = tmp_path / "legacy-target"
    target.mkdir()
    lanelet = target / "lanelet2_map.osm"
    lanelet.symlink_to(tmp_path / "source.osm")

    with pytest.raises(module.BundleError, match="--refresh-generated"):
        module.prepare_bundle(inspection, manifest_path, target_dir=target)

    module.prepare_bundle(
        inspection,
        manifest_path,
        target_dir=target,
        refresh_generated=True,
    )
    assert lanelet.is_file()
    assert not lanelet.is_symlink()
    assert sha256(lanelet) == inspection["lanelet2_derivation"]["result"][
        "output_sha256"
    ]


def test_dry_run_does_not_create_target(tmp_path: Path):
    module = load_module()
    manifest_path, inspection = inspect_fixture(module, tmp_path)
    target = tmp_path / "dry-run-target"

    result = module.prepare_bundle(
        inspection, manifest_path, target_dir=target, dry_run=True
    )

    assert not target.exists()
    assert set(result["actions"].values()) == {"create"}


def test_source_hash_mismatch_is_actionable(tmp_path: Path):
    module = load_module()
    manifest = write_fixture_manifest(tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    pcd = tmp_path / "source.pcd"
    payload = bytearray(pcd.read_bytes())
    payload[-1] ^= 0x01
    pcd.write_bytes(payload)

    with pytest.raises(module.BundleError, match="SHA-256 mismatch.*Do not silently mix"):
        module.inspect_profile(document, "fixture", manifest_path)


def test_bundle_source_override_changes_only_location_and_keeps_pin(tmp_path: Path):
    module = load_module()
    manifest = write_fixture_manifest(tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    original = copy.deepcopy(document)
    alternate = tmp_path / "alternate.pcd"
    alternate.write_bytes((tmp_path / "source.pcd").read_bytes())

    overridden = module.apply_bundle_source_overrides(
        document, "fixture", [f"pointcloud_map={alternate}"]
    )
    inspection = module.inspect_profile(overridden, "fixture", manifest_path)

    assert document == original
    assert inspection["bundle_sources"]["pointcloud_map"]["resolved_path"] == str(
        alternate.resolve()
    )
    assert overridden["profiles"]["fixture"]["bundle_sources"]["pointcloud_map"][
        "sha256"
    ] == original["profiles"]["fixture"]["bundle_sources"]["pointcloud_map"][
        "sha256"
    ]


def test_bundle_source_override_rejects_unknown_or_duplicate_ids(tmp_path: Path):
    module = load_module()
    document, _ = module.load_manifest(write_fixture_manifest(tmp_path))

    with pytest.raises(module.BundleError, match="unknown bundle source"):
        module.apply_bundle_source_overrides(document, "fixture", ["unknown=/tmp/a"])
    with pytest.raises(module.BundleError, match="duplicate --source"):
        module.apply_bundle_source_overrides(
            document,
            "fixture",
            ["pointcloud_map=/tmp/a", "pointcloud_map=/tmp/b"],
        )


def test_runtime_bundle_can_skip_unavailable_provenance_references(tmp_path: Path):
    module = load_module()
    manifest = write_fixture_manifest(tmp_path)
    document, manifest_path = module.load_manifest(manifest)
    document["profiles"]["fixture"]["reference_assets"]["roadrunner_opendrive"][
        "path"
    ] = "missing-authoring-source.xodr"

    with pytest.raises(module.BundleError, match="source does not exist"):
        module.inspect_profile(document, "fixture", manifest_path)

    inspection = module.inspect_profile(
        document, "fixture", manifest_path, inspect_references=False
    )
    assert inspection["reference_assets"] == {}
    assert "reference assets were not inspected" in " ".join(inspection["warnings"])


def test_setup_refuses_to_clobber_regular_map_asset(tmp_path: Path):
    module = load_module()
    manifest_path, inspection = inspect_fixture(module, tmp_path)
    target = tmp_path / "target"
    target.mkdir()
    lanelet_target = target / "lanelet2_map.osm"
    lanelet_target.write_text("user-owned", encoding="utf-8")

    with pytest.raises(module.BundleError, match="refusing to replace existing non-symlink"):
        module.prepare_bundle(inspection, manifest_path, target_dir=target)
    assert lanelet_target.read_text(encoding="utf-8") == "user-owned"


def test_changed_generated_metadata_requires_explicit_refresh(tmp_path: Path):
    module = load_module()
    manifest_path, inspection = inspect_fixture(module, tmp_path)
    target = tmp_path / "target"
    module.prepare_bundle(inspection, manifest_path, target_dir=target)
    metadata = target / "map_bundle.json"
    metadata.write_text("{}\n", encoding="utf-8")

    with pytest.raises(module.BundleError, match="--refresh-generated"):
        module.prepare_bundle(inspection, manifest_path, target_dir=target)

    module.prepare_bundle(
        inspection,
        manifest_path,
        target_dir=target,
        refresh_generated=True,
    )
    assert json.loads(metadata.read_text())["profile"] == "fixture"


def test_manifest_requires_vertical_alignment_component(tmp_path: Path):
    module = load_module()
    manifest = write_fixture_manifest(tmp_path)
    document = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    changed = copy.deepcopy(document)
    del changed["profiles"]["fixture"]["carla_to_map_transform"]["z_m"]

    with pytest.raises(module.BundleError, match="carla_to_map_transform fields are invalid"):
        module.validate_manifest(changed)


@pytest.mark.parametrize("command", ["inspect", "setup"])
def test_cli_rejects_duplicate_target_names_before_asset_access(
    command: str, tmp_path: Path, capsys
):
    module = load_module()
    manifest = write_fixture_manifest(tmp_path)
    document = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    document["profiles"]["duplicate_fixture"] = copy.deepcopy(
        document["profiles"]["fixture"]
    )
    manifest.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    target_root = tmp_path / "targets"
    argv = ["--manifest", str(manifest), command, "fixture"]
    if command == "setup":
        argv.extend(["--target-root", str(target_root)])

    assert module.main(argv) == 2

    captured = capsys.readouterr()
    assert captured.out == ""
    assert (
        "profiles.duplicate_fixture.target_name duplicates "
        "profiles.fixture.target_name: 'fixture_full'"
    ) in captured.err
    assert "target_name values must be unique" in captured.err
    assert not target_root.exists()
