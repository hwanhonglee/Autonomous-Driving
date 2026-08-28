import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))

from align_carla_route_to_map import (  # noqa: E402
    AlignmentError,
    MapTransform,
    align_route_payload,
    format_ros_float,
    prepare_aligned_route,
    transform_pose,
)


def sample_route():
    return {
        "schema_version": 1,
        "town": "CustomMap",
        "coordinate_reference": "base_link",
        "spawn_point_reference": "base_link",
        "spawn_point": "1,2,3,0,0,90",
        "start_carla_transform": {"x": 1, "y": -2, "z": 3, "yaw": -90},
        "goal_carla_transform": {"x": 2, "y": -3, "z": 4, "yaw": -90},
        "start_ros_pose": {"x": 1.0, "y": 2.0, "z": 3.0, "yaw": 0.25},
        "goal_ros_pose": {"x": 4.0, "y": 5.0, "z": 6.0, "yaw": -3.0},
        "route": [
            {"x": 1.0, "y": 2.0, "z": 3.0, "yaw": 0.25, "road_id": 7},
            {"x": 4.0, "y": 5.0, "z": 6.0, "yaw": -3.0, "road_id": 8},
        ],
    }


def write_bundle(path, transform):
    path.mkdir(exist_ok=True)
    (path / "map_bundle.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "profile": "test_map",
                "canonical_carla_map": "/Game/Test/Town99",
                "carla_to_map_transform": transform,
            }
        ),
        encoding="utf-8",
    )


def test_transform_pose_applies_planar_rigid_transform_z_and_yaw():
    output = transform_pose(
        {"x": 1.0, "y": 2.0, "z": 3.0, "yaw": math.pi},
        MapTransform(10.0, -5.0, -2.0, math.pi / 2.0),
        "pose",
    )

    assert output["x"] == pytest.approx(8.0)
    assert output["y"] == pytest.approx(-4.0)
    assert output["z"] == pytest.approx(1.0)
    assert output["yaw"] == pytest.approx(-math.pi / 2.0)


@pytest.mark.parametrize(
    ("value", "expected"),
    [(0.0, "0.0"), (-15.0, "-15.0"), (0.125, "0.125"), (1.0e-8, "1e-08")],
)
def test_ros_float_format_never_turns_integral_values_into_integer_parameters(value, expected):
    assert format_ros_float(value) == expected


def test_alignment_transforms_only_ros_map_poses_and_records_provenance(tmp_path):
    route = sample_route()
    source = tmp_path / "route.json"
    source.write_text(json.dumps(route), encoding="utf-8")
    transform = MapTransform(10.0, -5.0, -2.0, math.pi / 2.0)

    output = align_route_payload(
        route,
        transform,
        source_path=source,
        source_sha256="abc",
        bundle={"profile": "test_map"},
    )

    assert output["spawn_point"] == route["spawn_point"]
    assert output["start_carla_transform"] == route["start_carla_transform"]
    assert output["goal_carla_transform"] == route["goal_carla_transform"]
    assert output["start_ros_pose"]["x"] == pytest.approx(8.0)
    assert output["route"][0]["road_id"] == 7
    assert output["coordinate_alignment"]["source_route_sha256"] == "abc"
    assert output["coordinate_alignment"]["map_bundle_profile"] == "test_map"


def test_prepare_writes_deterministic_route_and_reports_launch_values(tmp_path):
    source = tmp_path / "route.json"
    source.write_text(json.dumps(sample_route()), encoding="utf-8")
    bundle = tmp_path / "bundle"
    write_bundle(
        bundle,
        {"x_m": 1.0, "y_m": 2.0, "z_m": -33.0, "yaw_rad": -0.05},
    )
    runtime = tmp_path / "runtime"

    first = prepare_aligned_route(source, bundle, runtime_dir=runtime)
    second = prepare_aligned_route(source, bundle, runtime_dir=runtime)

    assert first == second
    assert Path(first["aligned_route"]).is_file()
    assert first["alignment_enabled"] is True
    aligned = json.loads(Path(first["aligned_route"]).read_text(encoding="utf-8"))
    assert aligned["goal_ros_pose"]["z"] == pytest.approx(-27.0)


def test_double_alignment_is_rejected(tmp_path):
    route = sample_route()
    route["coordinate_alignment"] = {"source_frame": "carla_map", "target_frame": "map"}
    with pytest.raises(AlignmentError, match="double transform"):
        align_route_payload(
            route,
            MapTransform(0.0, 0.0, 0.0, 0.0),
            source_path=tmp_path / "route.json",
            source_sha256="abc",
            bundle={"profile": "test"},
        )


def test_prepare_accepts_only_matching_existing_alignment(tmp_path):
    source = tmp_path / "route.json"
    bundle = tmp_path / "bundle"
    transform = {"x_m": 1.0, "y_m": 2.0, "z_m": -3.0, "yaw_rad": 0.1}
    write_bundle(bundle, transform)
    raw = tmp_path / "raw.json"
    raw.write_text(json.dumps(sample_route()), encoding="utf-8")
    first = prepare_aligned_route(raw, bundle, output_path=source)

    repeated = prepare_aligned_route(source, bundle, runtime_dir=tmp_path / "unused")

    assert first["already_aligned"] is False
    assert repeated["already_aligned"] is True
    assert repeated["aligned_route"] == str(source.resolve())

    changed_bundle = json.loads((bundle / "map_bundle.json").read_text(encoding="utf-8"))
    changed_bundle["carla_to_map_transform"]["z_m"] = -4.0
    (bundle / "map_bundle.json").write_text(json.dumps(changed_bundle), encoding="utf-8")
    with pytest.raises(AlignmentError, match="does not match"):
        prepare_aligned_route(source, bundle)


@pytest.mark.parametrize(
    "transform",
    [
        {"x_m": float("nan"), "y_m": 0.0, "z_m": 0.0, "yaw_rad": 0.0},
        {"x_m": 0.0, "y_m": 0.0, "z_m": 0.0},
    ],
)
def test_invalid_bundle_transform_is_rejected(tmp_path, transform):
    source = tmp_path / "route.json"
    source.write_text(json.dumps(sample_route()), encoding="utf-8")
    bundle = tmp_path / "bundle"
    write_bundle(bundle, transform)

    with pytest.raises(AlignmentError):
        prepare_aligned_route(source, bundle)


def test_full_wrapper_applies_bundle_to_route_state_and_launch(tmp_path):
    fixture = ROOT / "autoware_e2e_vad_launch/test/fixtures/route_map"
    map_path = tmp_path / "Town99_full"
    shutil.copytree(fixture / "Town99_full", map_path)
    write_bundle(
        map_path,
        {"x_m": 0.0, "y_m": 0.0, "z_m": -10.0, "yaw_rad": 0.0},
    )
    route = json.loads((fixture / "town99_route.json").read_text(encoding="utf-8"))
    route["spawn_point"] = "1.0,0.0,10.5,0.0,0.0,0.0"
    for key in ("start_ros_pose", "goal_ros_pose"):
        route[key]["z"] = 10.0
    for point in route["route"]:
        point["z"] = 10.0
    route_path = tmp_path / "route.json"
    route_path.write_text(json.dumps(route), encoding="utf-8")

    cuda_root = tmp_path / "cuda"
    bin_dir = cuda_root / "bin"
    bin_dir.mkdir(parents=True)
    for name, body in {
        "nvcc": "#!/usr/bin/env bash\nexit 0\n",
        "ros2": "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n",
    }.items():
        executable = bin_dir / name
        executable.write_text(body, encoding="utf-8")
        executable.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "AUTOWARE_E2E_CUDA_ROOT": str(cuda_root),
            "AUTOWARE_E2E_SKIP_INSTALL": "1",
            "AUTOWARE_E2E_FULL_MAP_PATH": str(map_path),
            "AUTOWARE_E2E_RUNTIME_DIR": str(tmp_path / "runtime"),
        }
    )

    completed = subprocess.run(
        [str(ROOT / "scripts/e2e/run_route_vad_full.sh"), str(route_path)],
        cwd=ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert "use_carla_map_alignment:=true" in arguments
    assert "carla_map_alignment_z_m:=-10.0" in arguments
    assert "truth_initial_pose:=[1.000000000,0.000000000,0.000000000,0.000000000,0.000000000,0.000000000,1.000000000]" in arguments
    route_argument = next(value for value in arguments if value.startswith("route_file:="))
    aligned_route = Path(route_argument.removeprefix("route_file:="))
    assert aligned_route.is_file()
    aligned = json.loads(aligned_route.read_text(encoding="utf-8"))
    assert aligned["spawn_point"] == route["spawn_point"]
    assert aligned["goal_ros_pose"]["z"] == pytest.approx(0.0)


def test_full_wrapper_rejects_manual_alignment_override(tmp_path):
    cuda_root = tmp_path / "cuda"
    (cuda_root / "bin").mkdir(parents=True)
    nvcc = cuda_root / "bin/nvcc"
    nvcc.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    nvcc.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "AUTOWARE_E2E_CUDA_ROOT": str(cuda_root),
            "AUTOWARE_E2E_SKIP_INSTALL": "1",
        }
    )
    route = ROOT / "autoware_e2e_vad_launch/test/fixtures/route_map/town99_route.json"

    completed = subprocess.run(
        [
            str(ROOT / "scripts/e2e/run_route_vad_full.sh"),
            str(route),
            "carla_map_alignment_z_m:=123",
        ],
        cwd=ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 2
    assert "controlled by map_bundle.json" in completed.stderr
