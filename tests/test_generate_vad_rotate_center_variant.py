import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto
import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/generate_vad_rotate_center_variant.py"
OVERLAY = (
    ROOT
    / "autoware_e2e_vad_launch/config/vad_carla_tiny_rotate_center_xy_100_53.param.yaml"
)
RECOMMENDED = (
    ROOT / "autoware_e2e_vad_launch/config/vad_carla_tiny_recommended.param.yaml"
)
INITIALIZER = "custom_op::RotatePlugin_554"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ros_parameters(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]


def make_synthetic_head(path: Path) -> None:
    bev = helper.make_tensor_value_info("bev", TensorProto.FLOAT, [2, 4, 6])
    angle = helper.make_tensor_value_info("angle", TensorProto.FLOAT, [])
    output = helper.make_tensor_value_info("rotated", TensorProto.FLOAT, [2, 4, 6])
    center = numpy_helper.from_array(
        np.asarray([3.0, 3.0], dtype=np.float32), name=INITIALIZER
    )
    rotate = helper.make_node(
        "RotatePlugin",
        ["bev", "angle", INITIALIZER],
        ["rotated"],
        name="/transformer/RotatePlugin",
        domain="custom_op",
        interpolation=1,
    )
    graph = helper.make_graph(
        [rotate], "synthetic_vad_head", [bev, angle], [output], [center]
    )
    model = helper.make_model(
        graph,
        producer_name="rotate-center-test",
        opset_imports=[helper.make_opsetid("", 15), helper.make_opsetid("custom_op", 1)],
    )
    onnx.checker.check_model(model)
    onnx.save(model, path)


def run_generator(source: Path, output: Path, *extra: str):
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--source",
            str(source),
            "--output",
            str(output),
            "--expected-source-sha256",
            sha256(source),
            "--expected-old-center",
            "3",
            "3",
            "--new-center",
            "3",
            "2",
            "--expected-bev-shape",
            "1",
            "2",
            "4",
            "6",
            *extra,
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_generates_idempotent_verified_variant_and_manifest(tmp_path):
    source = tmp_path / "source.onnx"
    output = tmp_path / "variants/rotate_center_xy_3_2/head.onnx"
    manifest_path = output.with_suffix(".manifest.json")
    engine_path = output.with_suffix(".engine")
    make_synthetic_head(source)
    source_bytes = source.read_bytes()

    first = run_generator(source, output)
    assert first.returncode == 0, first.stderr
    assert source.read_bytes() == source_bytes
    assert output.is_file()
    assert output.stat().st_size == source.stat().st_size
    assert manifest_path.is_file()
    assert not engine_path.exists()

    variant = onnx.load(output)
    center = next(item for item in variant.graph.initializer if item.name == INITIALIZER)
    assert numpy_helper.to_array(center).tolist() == [3.0, 2.0]

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["source"]["sha256"] == sha256(source)
    assert manifest["output"]["sha256"] == sha256(output)
    assert manifest["rotate_plugin"]["input_shape_chw"] == [2, 4, 6]
    assert manifest["rotate_plugin"]["bev_shape_nchw"] == [1, 2, 4, 6]
    assert manifest["rotate_plugin"]["geometric_center_xy"] == [3.0, 2.0]
    assert manifest["rotate_plugin"]["initializer"] == {
        "name": INITIALIZER,
        "dtype": "float32",
        "shape": [2],
        "old_center_xy": [3.0, 3.0],
        "new_center_xy": [3.0, 2.0],
    }
    assert manifest["validation"]["only_initializer_changed"] is True
    assert manifest["planned_engine"]["path"] == str(engine_path)
    assert manifest["planned_engine"]["generated_by_this_tool"] is False

    output_bytes = output.read_bytes()
    manifest_bytes = manifest_path.read_bytes()
    output_mtime = output.stat().st_mtime_ns
    manifest_mtime = manifest_path.stat().st_mtime_ns
    second = run_generator(source, output)
    assert second.returncode == 0, second.stderr
    assert output.read_bytes() == output_bytes
    assert manifest_path.read_bytes() == manifest_bytes
    assert output.stat().st_mtime_ns == output_mtime
    assert manifest_path.stat().st_mtime_ns == manifest_mtime

    validation = run_generator(source, output, "--validate-only")
    assert validation.returncode == 0, validation.stderr
    assert "Validated RotatePlugin variant" in validation.stdout


def test_rejects_non_geometric_center_without_creating_output(tmp_path):
    source = tmp_path / "source.onnx"
    output = tmp_path / "variant.onnx"
    make_synthetic_head(source)

    result = run_generator(source, output, "--new-center", "2", "2")
    assert result.returncode == 2
    assert "is not geometric" in result.stderr
    assert not output.exists()


def test_overlay_preserves_recommended_sync_and_stock_head_precision():
    overlay = ros_parameters(OVERLAY)
    recommended = ros_parameters(RECOMMENDED)

    assert overlay["sync_params"] == recommended["sync_params"]
    assert set(overlay["model_params"]["nets"]) == {"head"}
    head = overlay["model_params"]["nets"]["head"]
    variant_root = "$(var model_path)/v0.1/variants/rotate_center_xy_100_53"
    assert head == {
        "onnx_path": f"{variant_root}/vad-carla-tiny_head.onnx",
        "engine_path": f"{variant_root}/vad-carla-tiny_head.engine",
        "precision": "fp32",
    }
