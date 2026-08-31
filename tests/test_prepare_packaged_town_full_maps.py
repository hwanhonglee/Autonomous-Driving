import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/prepare_packaged_town_full_maps.py"
MANIFEST = ROOT / "scripts/e2e/packaged_town_full_maps.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("prepare_packaged_town_full_maps", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_real_manifest_pins_all_packaged_town_candidates():
    module = load_module()
    document = module.load_manifest(MANIFEST)
    assert [entry["id"] for entry in document["maps"]] == [
        "town01",
        "town02_opt",
        "town03",
        "town04",
        "town05_opt",
        "town06",
        "town07",
        "town10hd_opt",
    ]
    assert tuple(document["pointcloud_transform"]["matrix_4x4_row_major"]) == (
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )
    ready = [entry for entry in document["maps"] if entry["pointcloud"] is not None]
    assert len(ready) == 7
    assert all(entry["pointcloud"]["transformed_sha256"] for entry in ready)
    blocked = document["maps"][-1]
    assert blocked["id"] == "town10hd_opt"
    assert blocked["pointcloud"] is None
    assert "point-cloud" in blocked["blocker"]


def test_manifest_rejects_a_transform_other_than_y_reflection(tmp_path):
    module = load_module()
    document = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    document["pointcloud_transform"]["matrix_4x4_row_major"][5] = 1.0
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(module.TownMapError, match="Y reflection"):
        module.load_manifest(path)


def test_source_root_environment_must_be_absolute(monkeypatch):
    module = load_module()
    contract = {
        "environment": "TEST_TOWN_MAP_ROOT",
        "candidates": ["/unused"],
        "provenance": "test",
    }
    monkeypatch.setenv("TEST_TOWN_MAP_ROOT", "relative")
    with pytest.raises(module.TownMapError, match="absolute"):
        module.resolve_source_root(contract, "marker")


def test_existing_regular_bundle_asset_is_never_replaced(tmp_path):
    module = load_module()
    source = tmp_path / "source"
    source.write_text("source", encoding="utf-8")
    target = tmp_path / "target"
    target.write_text("owned", encoding="utf-8")
    with pytest.raises(module.TownMapError, match="regular bundle asset"):
        module._ensure_symlink(target, source)
    assert target.read_text(encoding="utf-8") == "owned"


def test_route_catalog_requires_straight_and_turn(tmp_path, monkeypatch):
    module = load_module()
    map_id = "town01"
    route_dir = tmp_path / map_id / "catalog/routes" / map_id
    for scenario in ("straight", "left", "right"):
        scenario_dir = route_dir / scenario
        scenario_dir.mkdir(parents=True, exist_ok=True)
        (scenario_dir / f"{scenario}.json").write_text("{}", encoding="utf-8")

    class Validator:
        @staticmethod
        def validate_route_map(*_args, **_kwargs):
            return {
                "maximum_lanelet_distance_m": 0.0,
                "maximum_lanelet_vertical_distance_m": 0.1,
                "route_pose_count": 10,
            }

    monkeypatch.setattr(
        module,
        "validate_route_pcd_proximity",
        lambda paths, _map_path: {
            str(path): {"status": "PASS", "nearest_xy_m": {"max": 0.2}} for path in paths
        },
    )
    result = module.validate_catalog_routes(
        {"id": map_id, "bundle": {"path": str(tmp_path / "bundle")}},
        tmp_path,
        Validator,
    )
    assert result["status"] == "PASS"
    assert result["scenario_status"] == {
        "straight": "PASS",
        "left": "PASS",
        "right": "PASS",
    }


def test_route_pcd_correspondence_disambiguates_overlapping_heights():
    module = load_module()
    import numpy as np
    from scipy.spatial import cKDTree

    # The roof/bridge return is the XY-nearest point, but the road return is
    # the physically nearest point once route height is included.
    points = np.asarray(
        [
            [0.01, 0.01, 12.0],
            [0.20, 0.00, 0.02],
            [0.80, 0.00, 0.0],
        ],
        dtype=float,
    )
    route = np.asarray([[0.0, 0.0, 0.0]], dtype=float)
    xy_distances, z_distances, indices, unmatched = (
        module._height_aware_route_correspondence(
            route, points, cKDTree(points[:, :2]), 1.0
        )
    )

    assert unmatched == 0
    assert indices.tolist() == [1]
    assert xy_distances.tolist() == pytest.approx([0.2])
    assert z_distances.tolist() == pytest.approx([0.02])


def test_route_pcd_correspondence_fails_closed_without_xy_candidate():
    module = load_module()
    import numpy as np
    from scipy.spatial import cKDTree

    points = np.asarray([[1.01, 0.0, 0.0]], dtype=float)
    route = np.asarray([[0.0, 0.0, 0.0]], dtype=float)
    xy_distances, z_distances, indices, unmatched = (
        module._height_aware_route_correspondence(
            route, points, cKDTree(points[:, :2]), 1.0
        )
    )

    assert unmatched == 1
    assert indices.tolist() == [-1]
    assert np.isinf(xy_distances[0])
    assert np.isinf(z_distances[0])


def test_markdown_reports_town10_missing_pointcloud_as_blocked():
    module = load_module()
    report = {
        "maps": [
            {
                "id": "town10hd_opt",
                "status": "BLOCKED_MISSING_POINTCLOUD",
                "lanelet2": {"inspection": {"road_lanelets": 168}},
                "pointcloud_generated": None,
                "alignment": None,
                "errors": [],
                "blocker": "missing point cloud",
            }
        ]
    }
    markdown = module.render_markdown(report)
    assert "does **not** claim that VAD drove" in markdown
    assert "BLOCKED_MISSING_POINTCLOUD" in markdown
    assert "no Town10HD point cloud was found" in markdown
    assert "Town10HD_Opt stays blocked" in markdown
    assert "An audited semantic-LiDAR PCD is installed" not in markdown


def test_markdown_reports_recovered_town10_without_stale_blocker_text():
    module = load_module()
    report = {
        "maps": [
            {
                "id": "town10hd_opt",
                "status": "TEST_ROUTES_MAP_PREFLIGHT_PASS",
                "lanelet2": {"inspection": {"road_lanelets": 168}},
                "pointcloud_generated": {
                    "inspection": {"points": 6_818_935},
                },
                "alignment": {
                    "status": "PASS",
                    "nearest_xy_with_y_reflection_m": {
                        "p95": 0.15250933917077467,
                        "within_1m_fraction": 1.0,
                    },
                },
                "route_preflight": {
                    "status": "PASS",
                    "scenario_status": {
                        "straight": "PASS",
                        "left": "PASS",
                        "right": "PASS",
                    },
                },
                "errors": [],
                "blocker": None,
            }
        ]
    }

    markdown = module.render_markdown(report)

    assert "An audited semantic-LiDAR PCD is installed and pinned" in markdown
    assert "6,818,935 points" in markdown
    assert "collector provenance is `COMPLETE_QA_PASS`" in markdown
    assert "current readiness result is `TEST_ROUTES_MAP_PREFLIGHT_PASS`" in markdown
    assert "straight and turn route-to-Lanelet/PCD preflights also pass" in markdown
    assert "does not by itself claim that Autoware VAD drove" in markdown
    assert "no Town10HD point cloud was found" not in markdown
    assert "Town10HD_Opt stays blocked" not in markdown
