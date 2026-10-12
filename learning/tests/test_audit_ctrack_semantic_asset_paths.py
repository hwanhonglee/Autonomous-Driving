"""HH_260929 - Check path interpretation and fail-closed mesh evidence."""

from pathlib import Path

import pytest

from scripts.e2e.audit_ctrack_semantic_asset_paths import (
    classify_ctrack_paths, classify_town05_paths, printable_game_paths,
    tagger_folder,
)


def test_leading_slash_keeps_tagger_folder_position():
    assert tagger_folder("/Game/Carla/Static/Road/SM_Road") == "Road"
    assert tagger_folder("/Game/Carla/Maps/C_track_1_0_7/Roads_Road") == "C_track_1_0_7"


def test_readable_reference_classification_with_static_mesh_companions(tmp_path: Path):
    names = {
        "road": "C_track_1_0_7_Roads_Road_Layer0Node",
        "lane_marking": "C_track_1_0_7_Roads_Marking_Layer0Node",
        "sidewalk": "C_track_1_0_7_Roads_Sidewalk_Layer0Node",
    }
    package = tmp_path / "Carla/Maps/C_track_1_0_7"
    package.mkdir(parents=True)
    for name in names.values():
        (package / f"{name}.uasset").write_bytes(b"StaticMesh\x00")
    paths = {f"/Game/Carla/Maps/C_track_1_0_7/{name}" for name in names.values()}
    raw = b"\x00".join(path.encode() for path in paths) + b"\x00junk\x00"
    assert printable_game_paths(raw) == paths
    result = classify_ctrack_paths(paths, tmp_path)
    assert all(row["reference_count_unique"] == 1 and
               row["matching_tagger_folder_count"] == 0 and
               row["companion_static_mesh_assets_confirmed"] == 1
               for row in result.values())


def test_missing_packaged_static_mesh_fails_closed(tmp_path: Path):
    paths = {
        "/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Road_Layer0Node",
        "/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Marking_Layer0Node",
        "/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Sidewalk_Layer0Node",
    }
    with pytest.raises(ValueError, match="referenced packaged mesh missing"):
        classify_ctrack_paths(paths, tmp_path)


def test_builtin_town_path_has_expected_class_folders():
    paths = {
        "/Game/Carla/Static/Road/Town05_Road",
        "/Game/Carla/Static/RoadLine/Town05_Marking",
        "/Game/Carla/Static/SideWalk/Town05_SideWalk",
    }
    result = classify_town05_paths(paths)
    assert all(row["matching_tagger_folder_count"] == 1 for row in result.values())
