#!/usr/bin/env python3
"""Capture a raw vehicle command converter config and its referenced maps."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

import yaml


MAP_PARAMETERS = {
    "accel_map": "csv_path_accel_map",
    "brake_map": "csv_path_brake_map",
    "steer_map": "csv_path_steer_map",
}
FIND_PACKAGE_SHARE = re.compile(
    r"^\$\(find-pkg-share\s+([a-zA-Z0-9_]+)\)(/.*)?$"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _package_share_directory(package: str) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:
        raise ValueError(
            "ament_index_python is required to resolve find-pkg-share references"
        ) from error
    try:
        return Path(get_package_share_directory(package))
    except Exception as error:
        raise ValueError(f"ROS package share not found: {package}") from error


def _is_within(path: Path, directory: Path) -> bool:
    try:
        path.relative_to(directory)
    except ValueError:
        return False
    return True


def resolve_map_reference(reference: object, config_directory: Path) -> dict[str, object]:
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("map reference must be a non-empty string")
    reference = reference.strip()
    match = FIND_PACKAGE_SHARE.fullmatch(reference)
    package = None
    if match:
        package = match.group(1)
        suffix = (match.group(2) or "").lstrip("/")
        if not suffix:
            raise ValueError(f"map reference must name a file inside package {package}")
        path = _package_share_directory(package) / suffix
        kind = "find-pkg-share"
    else:
        if "$(" in reference:
            raise ValueError(f"unsupported ROS substitution in map reference: {reference}")
        expanded = Path(reference).expanduser()
        if not expanded.is_absolute():
            raise ValueError(
                "relative map reference is ambiguous; use an absolute path or "
                f"$(find-pkg-share ...): {reference}"
            )
        path = expanded
        kind = "absolute"

    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"referenced actuation map is not a file: {path}")
    result: dict[str, object] = {
        "reference": reference,
        "reference_kind": kind,
        "source": str(path),
        "external_to_config_directory": not _is_within(path, config_directory),
    }
    if package is not None:
        result["package"] = package
    return result


def read_converter_map_references(config_path: Path) -> dict[str, dict[str, object]]:
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        parameters = payload["/**"]["ros__parameters"]
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as error:
        raise ValueError(f"invalid raw vehicle command converter YAML: {config_path}") from error
    if not isinstance(parameters, dict):
        raise ValueError(f"converter ros__parameters must be a mapping: {config_path}")

    missing = sorted(set(MAP_PARAMETERS.values()) - set(parameters))
    if missing:
        raise ValueError("converter config is missing map parameters: " + ", ".join(missing))
    return {
        name: resolve_map_reference(parameters[parameter], config_path.parent)
        for name, parameter in MAP_PARAMETERS.items()
    }


def capture_provenance(config_path: Path, output_directory: Path) -> dict[str, object]:
    config_path = config_path.expanduser().resolve()
    output_directory = output_directory.expanduser().resolve()
    if not config_path.is_file():
        raise ValueError(f"converter config is not a file: {config_path}")
    if output_directory.exists():
        raise FileExistsError(f"provenance output already exists: {output_directory}")

    references = read_converter_map_references(config_path)
    output_directory.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(
            prefix=f".{output_directory.name}.",
            dir=str(output_directory.parent),
        )
    )
    try:
        config_artifact = "raw_vehicle_cmd_converter.param.yaml"
        shutil.copy2(config_path, temporary / config_artifact)
        files: dict[str, dict[str, object]] = {
            "config": {
                "source": str(config_path),
                "artifact": config_artifact,
                "sha256": _sha256(config_path),
                "size_bytes": config_path.stat().st_size,
            }
        }
        for name, record in references.items():
            source = Path(str(record["source"]))
            artifact = f"{name}.csv"
            shutil.copy2(source, temporary / artifact)
            source_sha256 = _sha256(source)
            if _sha256(temporary / artifact) != source_sha256:
                raise RuntimeError(f"actuation provenance copy hash mismatch: {source}")
            files[name] = {
                **record,
                "parameter": MAP_PARAMETERS[name],
                "artifact": artifact,
                "sha256": source_sha256,
                "size_bytes": source.stat().st_size,
            }

        manifest: dict[str, object] = {
            "schema_version": 1,
            "execution": {
                "selected_config": str(config_path),
                "uses_original_selected_config": True,
                "uses_artifact_copy": False,
            },
            "files": files,
        }
        (temporary / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, output_directory)
        return manifest
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest = capture_provenance(args.config, args.output_dir)
    except (FileExistsError, ValueError, RuntimeError) as error:
        print(f"Actuation config provenance failed: {error}", file=sys.stderr)
        return 2
    print(
        "Captured raw vehicle command converter provenance: "
        f"{args.output_dir} ({len(manifest['files'])} files)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
