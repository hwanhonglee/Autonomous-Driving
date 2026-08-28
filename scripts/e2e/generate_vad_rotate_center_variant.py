#!/usr/bin/env python3
"""Generate and validate a VAD temporal-head ONNX rotate-center variant."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Sequence

import numpy as np
import onnx
from onnx import numpy_helper


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "data/ml_models/vad/v0.1"
DEFAULT_SOURCE = MODEL_DIR / "vad-carla-tiny_head.onnx"
DEFAULT_VARIANT_DIR = MODEL_DIR / "variants/rotate_center_xy_100_53"
DEFAULT_OUTPUT = DEFAULT_VARIANT_DIR / "vad-carla-tiny_head.onnx"
OFFICIAL_SOURCE_SHA256 = (
    "31f49a592a764ce82bbe6e26d0bfc99dc8a9613628884dc73dd5e67521ff3e9e"
)
DEFAULT_INITIALIZER = "custom_op::RotatePlugin_554"
DEFAULT_OLD_CENTER = (100.0, 100.0)
DEFAULT_NEW_CENTER = (100.0, 53.0)
DEFAULT_BEV_SHAPE_NCHW = (1, 256, 106, 200)


class VariantError(RuntimeError):
    """Raised when the source or generated variant violates the artifact contract."""


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _record_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(resolved)


def _load_model(path: Path) -> onnx.ModelProto:
    try:
        model = onnx.load(str(path), load_external_data=True)
        onnx.checker.check_model(model)
    except Exception as error:  # onnx exposes several parser/checker exception types
        raise VariantError(f"invalid ONNX model {path}: {error}") from error
    return model


def _initializer(model: onnx.ModelProto, name: str) -> onnx.TensorProto:
    matches = [item for item in model.graph.initializer if item.name == name]
    if len(matches) != 1:
        raise VariantError(
            f"expected exactly one initializer named {name!r}, found {len(matches)}"
        )
    return matches[0]


def _static_tensor_shape(model: onnx.ModelProto, name: str) -> list[int]:
    values = (*model.graph.input, *model.graph.value_info, *model.graph.output)
    matches = [value for value in values if value.name == name]
    if len(matches) != 1:
        raise VariantError(
            f"expected one static shape declaration for tensor {name!r}, found {len(matches)}"
        )

    dimensions: list[int] = []
    for dimension in matches[0].type.tensor_type.shape.dim:
        if not dimension.HasField("dim_value") or dimension.dim_value <= 0:
            raise VariantError(f"tensor {name!r} has a dynamic or invalid dimension")
        dimensions.append(int(dimension.dim_value))
    return dimensions


def _center_values(initializer: onnx.TensorProto) -> tuple[np.ndarray, list[float]]:
    value = np.asarray(numpy_helper.to_array(initializer))
    if value.shape != (2,) or not np.issubdtype(value.dtype, np.floating):
        raise VariantError(
            "RotatePlugin center initializer must be a floating tensor with shape [2]"
        )
    return value, [float(item) for item in value.tolist()]


def _inspect_rotate_contract(
    model: onnx.ModelProto,
    initializer_name: str,
    expected_center: Sequence[float],
    expected_bev_shape_nchw: Sequence[int],
) -> dict[str, object]:
    initializer = _initializer(model, initializer_name)
    center_array, center = _center_values(initializer)
    expected_center_list = [float(item) for item in expected_center]
    if center != expected_center_list:
        raise VariantError(
            f"initializer {initializer_name!r} center is {center}, expected {expected_center_list}"
        )

    consumers = [
        node
        for node in model.graph.node
        if node.op_type == "RotatePlugin"
        and node.domain == "custom_op"
        and len(node.input) == 3
        and node.input[2] == initializer_name
    ]
    if len(consumers) != 1:
        raise VariantError(
            "expected exactly one custom_op::RotatePlugin with the center as its third input, "
            f"found {len(consumers)}"
        )
    node = consumers[0]

    shape_chw = _static_tensor_shape(model, node.input[0])
    if len(shape_chw) != 3:
        raise VariantError(
            f"RotatePlugin input must be [C,H,W], found shape {shape_chw}"
        )
    bev_shape_nchw = [1, *shape_chw]
    expected_shape = [int(item) for item in expected_bev_shape_nchw]
    if bev_shape_nchw != expected_shape:
        raise VariantError(
            f"RotatePlugin canonical BEV shape is {bev_shape_nchw}, expected {expected_shape}"
        )

    _, height, width = shape_chw
    geometric_center = [width / 2.0, height / 2.0]
    interpolation = None
    for attribute in node.attribute:
        if attribute.name == "interpolation":
            interpolation = int(onnx.helper.get_attribute_value(attribute))

    return {
        "node_name": node.name,
        "domain": node.domain,
        "op_type": node.op_type,
        "input_tensor": node.input[0],
        "input_shape_chw": shape_chw,
        "bev_shape_nchw": bev_shape_nchw,
        "geometric_center_xy": geometric_center,
        "interpolation": interpolation,
        "initializer": {
            "name": initializer_name,
            "dtype": str(center_array.dtype),
            "shape": list(center_array.shape),
            "center_xy": center,
        },
    }


def _replace_center(
    model: onnx.ModelProto, initializer_name: str, new_center: Sequence[float]
) -> None:
    initializer = _initializer(model, initializer_name)
    old_array, _ = _center_values(initializer)
    if not initializer.raw_data:
        raise VariantError(
            "RotatePlugin center initializer must use raw_data encoding for a byte-minimal patch"
        )
    replacement = onnx.TensorProto()
    replacement.CopyFrom(initializer)
    replacement.raw_data = np.asarray(new_center, dtype=old_array.dtype).tobytes()
    initializer.CopyFrom(replacement)


def _assert_expected_initializer_change(
    source: onnx.ModelProto,
    output: onnx.ModelProto,
    initializer_name: str,
    new_center: Sequence[float],
) -> None:
    expected = onnx.ModelProto()
    expected.CopyFrom(source)
    _replace_center(expected, initializer_name, new_center)
    expected_bytes = expected.SerializeToString(deterministic=True)
    output_bytes = output.SerializeToString(deterministic=True)
    if output_bytes != expected_bytes:
        raise VariantError(
            "output is not the byte-minimal expected center-initializer patch"
        )


def _atomic_save_model(model: onnx.ModelProto, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        onnx.save_model(model, str(temporary))
        os.chmod(temporary, 0o644)
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_write_json(path: Path, payload: dict[str, object]) -> bool:
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    if path.is_file() and path.read_bytes() == encoded:
        return False

    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return True


def generate_variant(
    *,
    source: Path,
    output: Path,
    manifest_path: Path,
    engine_path: Path,
    initializer_name: str,
    expected_source_sha256: str,
    expected_old_center: Sequence[float],
    new_center: Sequence[float],
    expected_bev_shape_nchw: Sequence[int],
    validate_only: bool = False,
) -> dict[str, object]:
    source = source.resolve()
    output = output.resolve()
    manifest_path = manifest_path.resolve()
    engine_path = engine_path.resolve()

    if not source.is_file():
        raise VariantError(f"source ONNX does not exist: {source}")
    if source == output:
        raise VariantError("output must not overwrite the source ONNX")
    if manifest_path in (source, output) or engine_path in (source, output, manifest_path):
        raise VariantError("source, output, manifest, and engine paths must be distinct")
    if len(expected_source_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in expected_source_sha256
    ):
        raise VariantError("expected source SHA256 must be 64 lowercase hexadecimal characters")

    source_sha256 = sha256_file(source)
    if source_sha256 != expected_source_sha256:
        raise VariantError(
            f"source SHA256 is {source_sha256}, expected {expected_source_sha256}"
        )

    source_model = _load_model(source)
    source_contract = _inspect_rotate_contract(
        source_model,
        initializer_name,
        expected_old_center,
        expected_bev_shape_nchw,
    )
    geometric_center = source_contract["geometric_center_xy"]
    requested_center = [float(item) for item in new_center]
    if requested_center != geometric_center:
        raise VariantError(
            f"new center {requested_center} is not geometric [W/2,H/2] center {geometric_center}"
        )

    generated = False
    if not output.exists():
        if validate_only:
            raise VariantError(f"variant ONNX does not exist: {output}")
        if engine_path.exists():
            raise VariantError(
                f"refusing to create a new ONNX beside an existing potentially stale engine: {engine_path}"
            )
        variant_model = onnx.ModelProto()
        variant_model.CopyFrom(source_model)
        _replace_center(variant_model, initializer_name, requested_center)
        onnx.checker.check_model(variant_model)
        _assert_expected_initializer_change(
            source_model, variant_model, initializer_name, requested_center
        )
        _atomic_save_model(variant_model, output)
        generated = True
    elif not output.is_file():
        raise VariantError(f"variant output is not a regular file: {output}")

    try:
        output_model = _load_model(output)
        output_contract = _inspect_rotate_contract(
            output_model,
            initializer_name,
            requested_center,
            expected_bev_shape_nchw,
        )
        _assert_expected_initializer_change(
            source_model, output_model, initializer_name, requested_center
        )
        if sha256_file(source) != source_sha256:
            raise VariantError("source ONNX changed while generating the variant")
    except Exception:
        if generated:
            output.unlink(missing_ok=True)
        raise

    source_initializer = source_contract["initializer"]
    output_initializer = output_contract["initializer"]
    manifest: dict[str, object] = {
        "schema_version": 1,
        "status": "valid",
        "variant": "rotate_center_xy_100_53",
        "source": {
            "path": _record_path(source),
            "sha256": source_sha256,
            "size_bytes": source.stat().st_size,
        },
        "output": {
            "path": _record_path(output),
            "sha256": sha256_file(output),
            "size_bytes": output.stat().st_size,
        },
        "planned_engine": {
            "path": _record_path(engine_path),
            "generated_by_this_tool": False,
        },
        "rotate_plugin": {
            "node_name": output_contract["node_name"],
            "domain": output_contract["domain"],
            "op_type": output_contract["op_type"],
            "input_tensor": output_contract["input_tensor"],
            "input_shape_chw": output_contract["input_shape_chw"],
            "bev_shape_nchw": output_contract["bev_shape_nchw"],
            "geometric_center_xy": output_contract["geometric_center_xy"],
            "interpolation": output_contract["interpolation"],
            "initializer": {
                "name": initializer_name,
                "dtype": output_initializer["dtype"],
                "shape": output_initializer["shape"],
                "old_center_xy": source_initializer["center_xy"],
                "new_center_xy": output_initializer["center_xy"],
            },
        },
        "validation": {
            "expected_source_sha256": expected_source_sha256,
            "source_sha256_matches": True,
            "onnx_checker": "passed",
            "only_initializer_changed": True,
        },
    }

    if validate_only:
        if not manifest_path.is_file():
            raise VariantError(f"manifest does not exist: {manifest_path}")
        try:
            recorded_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise VariantError(f"invalid manifest {manifest_path}: {error}") from error
        if recorded_manifest != manifest:
            raise VariantError("manifest does not match the validated source and output artifacts")
    else:
        _atomic_write_json(manifest_path, manifest)

    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a non-destructive VAD head ONNX variant whose RotatePlugin center is "
            "the verified geometric BEV center. No TensorRT engine is built."
        )
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--manifest",
        type=Path,
        help="default: OUTPUT with .manifest.json suffix",
    )
    parser.add_argument(
        "--engine-path",
        type=Path,
        help="planned engine path recorded in the manifest; no engine is generated",
    )
    parser.add_argument("--initializer", default=DEFAULT_INITIALIZER)
    parser.add_argument(
        "--expected-source-sha256", default=OFFICIAL_SOURCE_SHA256
    )
    parser.add_argument(
        "--expected-old-center",
        nargs=2,
        type=float,
        default=DEFAULT_OLD_CENTER,
        metavar=("X", "Y"),
    )
    parser.add_argument(
        "--new-center",
        nargs=2,
        type=float,
        default=DEFAULT_NEW_CENTER,
        metavar=("X", "Y"),
    )
    parser.add_argument(
        "--expected-bev-shape",
        nargs=4,
        type=int,
        default=DEFAULT_BEV_SHAPE_NCHW,
        metavar=("N", "C", "H", "W"),
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="require and validate an existing output and matching manifest",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output
    manifest_path = args.manifest or output.with_suffix(".manifest.json")
    engine_path = args.engine_path or output.with_suffix(".engine")
    try:
        manifest = generate_variant(
            source=args.source,
            output=output,
            manifest_path=manifest_path,
            engine_path=engine_path,
            initializer_name=args.initializer,
            expected_source_sha256=args.expected_source_sha256,
            expected_old_center=args.expected_old_center,
            new_center=args.new_center,
            expected_bev_shape_nchw=args.expected_bev_shape,
            validate_only=args.validate_only,
        )
    except VariantError as error:
        print(f"error: {error}", file=os.sys.stderr)
        return 2

    action = "Validated" if args.validate_only else "Generated or validated"
    print(f"{action} RotatePlugin variant: {manifest['output']['path']}")
    print(f"Manifest: {_record_path(manifest_path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
