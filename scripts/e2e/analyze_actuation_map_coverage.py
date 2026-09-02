#!/usr/bin/env python3
"""Audit raw command-converter velocity-axis coverage for a speed profile.

The converter indexes its CSV maps by absolute current odometry speed, not by
the requested cruise target.  A target outside the map axis therefore exposes
a future coverage boundary; clamping occurs only if observed speed reaches that
region.  This tool records the preflight boundary and, optionally, the observed
runtime lookup range without treating either as calibration evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Sequence


ROOT = Path(__file__).resolve().parents[2]
CONVERTER_SOURCE_DIRECTORY = (
    ROOT
    / "src/universe/autoware_universe/vehicle/"
    "autoware_raw_vehicle_cmd_converter/src"
)
IMPLEMENTATION_FILES = ("accel_map.cpp", "brake_map.cpp")
CLAMP_EXPRESSION = "CSVLoader::clampValue(vel, vel_index_"
CURRENT_SPEED_SOURCE_EXPRESSIONS = (
    "const double vel = current_odometry_->twist.twist.linear.x;",
    "calculateAccelMap(vel, acc, accel_cmd_is_zero)",
    "calculateBrakeMap(vel, acc)",
)
MAP_LOOKUP_SPECS = (
    ("accel_map", "calculateAccelMap", "accel_map_", "getThrottle"),
    ("brake_map", "calculateBrakeMap", "brake_map_", "getBrake"),
)


class CoverageError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CoverageError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise CoverageError(f"{label} must contain a JSON object: {path}")
    return value


def _artifact_path(root: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise CoverageError(f"{label} artifact must be a relative file path")
    candidate = root / value
    if candidate.is_symlink() or not candidate.is_file():
        raise CoverageError(f"{label} artifact is missing or not regular: {candidate}")
    try:
        candidate.resolve().relative_to(root)
    except ValueError as error:
        raise CoverageError(f"{label} artifact escapes provenance root") from error
    return candidate


def _velocity_axis(path: Path, label: str) -> list[float]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            first = next(csv.reader(stream))
    except (OSError, UnicodeDecodeError, StopIteration, csv.Error) as error:
        raise CoverageError(f"cannot read {label} velocity axis: {error}") from error
    if len(first) < 3:
        raise CoverageError(f"{label} needs at least two velocity columns")
    try:
        axis = [float(value) for value in first[1:]]
    except ValueError as error:
        raise CoverageError(f"{label} velocity axis is not numeric") from error
    if not all(math.isfinite(value) for value in axis):
        raise CoverageError(f"{label} velocity axis contains a non-finite value")
    if any(right <= left for left, right in zip(axis, axis[1:])):
        raise CoverageError(f"{label} velocity axis must be strictly increasing")
    return axis


def _implementation_records(source_directory: Path) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for name in IMPLEMENTATION_FILES:
        path = source_directory / name
        if path.is_symlink() or not path.is_file():
            raise CoverageError(f"converter implementation is missing: {path}")
        try:
            source = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as error:
            raise CoverageError(f"cannot read converter implementation {path}: {error}") from error
        if CLAMP_EXPRESSION not in source:
            raise CoverageError(f"converter implementation lacks velocity clamping: {path}")
        records[name] = {
            "path": str(path.resolve()),
            "sha256": _sha256(path),
            "velocity_lookup": "clamp_to_csv_axis_then_linear_interpolation",
        }
    return records


def _cpp_code_without_comments_or_literals(source: str) -> str:
    """Preserve C++ structure while removing text that cannot execute."""
    output: list[str] = []
    index = 0
    state = "code"
    quote = ""
    while index < len(source):
        current = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            if current == "/" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if current == "/" and following == "*":
                output.extend((" ", " "))
                index += 2
                state = "block_comment"
                continue
            if current in {'"', "'"}:
                output.append(" ")
                quote = current
                index += 1
                state = "literal"
                continue
            output.append(current)
            index += 1
            continue
        if state == "line_comment":
            output.append("\n" if current == "\n" else " ")
            index += 1
            if current == "\n":
                state = "code"
            continue
        if state == "block_comment":
            if current == "*" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "code"
            else:
                output.append("\n" if current == "\n" else " ")
                index += 1
            continue
        if current == "\\" and following:
            output.extend((" ", "\n" if following == "\n" else " "))
            index += 2
            continue
        output.append("\n" if current == "\n" else " ")
        index += 1
        if current == quote:
            state = "code"
    return "".join(output)


def _matching_delimiter(
    source: str, opening_index: int, opening: str, closing: str
) -> int:
    depth = 0
    for index in range(opening_index, len(source)):
        if source[index] == opening:
            depth += 1
        elif source[index] == closing:
            depth -= 1
            if depth == 0:
                return index
    raise CoverageError(f"unterminated C++ delimiter after offset {opening_index}")


def _function_body(source: str, function_name: str) -> str:
    pattern = re.compile(
        r"\bRawVehicleCommandConverterNode\s*::\s*"
        + re.escape(function_name)
        + r"\s*\("
    )
    for match in pattern.finditer(source):
        opening_parenthesis = match.end() - 1
        closing_parenthesis = _matching_delimiter(
            source, opening_parenthesis, "(", ")"
        )
        opening_brace = source.find("{", closing_parenthesis + 1)
        declaration_end = source.find(";", closing_parenthesis + 1)
        if opening_brace < 0 or (
            declaration_end >= 0 and declaration_end < opening_brace
        ):
            continue
        closing_brace = _matching_delimiter(source, opening_brace, "{", "}")
        return source[opening_brace + 1 : closing_brace]
    raise CoverageError(
        f"converter node lacks {function_name} implementation body"
    )


def _split_cpp_arguments(arguments: str) -> list[str]:
    output: list[str] = []
    start = 0
    parentheses = 0
    brackets = 0
    braces = 0
    for index, character in enumerate(arguments):
        if character == "(":
            parentheses += 1
        elif character == ")":
            parentheses -= 1
        elif character == "[":
            brackets += 1
        elif character == "]":
            brackets -= 1
        elif character == "{":
            braces += 1
        elif character == "}":
            braces -= 1
        elif (
            character == ","
            and parentheses == 0
            and brackets == 0
            and braces == 0
        ):
            output.append(arguments[start:index].strip())
            start = index + 1
    output.append(arguments[start:].strip())
    return output


def _lookup_calls(
    body: str, receiver: str, method: str
) -> list[tuple[int, list[str]]]:
    pattern = re.compile(
        r"\b" + re.escape(receiver) + r"\s*\.\s*" + re.escape(method) + r"\s*\("
    )
    calls: list[tuple[int, list[str]]] = []
    for match in pattern.finditer(body):
        opening = match.end() - 1
        closing = _matching_delimiter(body, opening, "(", ")")
        calls.append((match.start(), _split_cpp_arguments(body[opening + 1 : closing])))
    return calls


def _absolute_current_velocity_expression(
    expression: str, body_prefix: str
) -> bool:
    compact = re.sub(r"\s+", "", expression)
    if compact in {
        "std::abs(current_velocity)",
        "std::fabs(current_velocity)",
    }:
        return True
    if not re.fullmatch(r"[A-Za-z_]\w*", compact):
        return False
    declaration = re.compile(
        r"\b(?:const\s+)?(?:double|auto)\s+"
        + re.escape(compact)
        + r"\s*=\s*std::(?:abs|fabs)\s*\(\s*current_velocity\s*\)\s*;"
    )
    matches = list(declaration.finditer(body_prefix))
    if not matches:
        return False
    following = body_prefix[matches[-1].end() :]
    mutation = re.compile(
        r"\b"
        + re.escape(compact)
        + r"\s*(?:\+\+|--|[+\-*/]?=(?!=))"
    )
    return mutation.search(following) is None


def _absolute_map_lookup_records(source: str) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for map_name, function_name, receiver, method in MAP_LOOKUP_SPECS:
        body = _function_body(source, function_name)
        calls = _lookup_calls(body, receiver, method)
        if not calls:
            raise CoverageError(
                f"converter node lacks {receiver}.{method} lookup in {function_name}"
            )
        velocity_expressions: list[str] = []
        for offset, arguments in calls:
            if len(arguments) < 2 or not _absolute_current_velocity_expression(
                arguments[1], body[:offset]
            ):
                observed = arguments[1] if len(arguments) >= 2 else "<missing>"
                raise CoverageError(
                    f"{receiver}.{method} does not use absolute current velocity: "
                    f"{observed}"
                )
            velocity_expressions.append(
                re.sub(r"\s+", " ", arguments[1]).strip()
            )
        records[map_name] = {
            "function": function_name,
            "lookup": f"{receiver}.{method}",
            "velocity_arguments": velocity_expressions,
            "absolute_current_velocity_verified": True,
        }
    return records


def _converter_node_record(source_directory: Path) -> dict[str, Any]:
    path = source_directory / "node.cpp"
    if path.is_symlink() or not path.is_file():
        raise CoverageError(f"converter node implementation is missing: {path}")
    try:
        source = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise CoverageError(f"cannot read converter node implementation {path}: {error}") from error
    code = _cpp_code_without_comments_or_literals(source)
    missing = [value for value in CURRENT_SPEED_SOURCE_EXPRESSIONS if value not in code]
    if missing:
        raise CoverageError(
            "converter node no longer proves current-odometry map indexing: "
            + ", ".join(missing)
        )
    lookup_records = _absolute_map_lookup_records(code)
    return {
        "path": str(path.resolve()),
        "sha256": _sha256(path),
        "map_velocity_input": "absolute_current_odometry_longitudinal_speed_mps",
        "target_cruise_speed_is_direct_lookup_input": False,
        "verified_map_lookups": lookup_records,
    }


def analyze_coverage(
    provenance_directory: Path,
    *,
    profile_id: str,
    target_speed_mps: float,
    allow_clamped_target: bool,
    observed_maximum_speed_mps: float | None = None,
    converter_source_directory: Path = CONVERTER_SOURCE_DIRECTORY,
) -> dict[str, Any]:
    if not profile_id:
        raise CoverageError("profile id must not be empty")
    if not math.isfinite(target_speed_mps) or target_speed_mps <= 0.0:
        raise CoverageError("target speed must be positive and finite")
    if observed_maximum_speed_mps is not None and (
        not math.isfinite(observed_maximum_speed_mps)
        or observed_maximum_speed_mps < 0.0
    ):
        raise CoverageError("observed maximum speed must be finite and non-negative")
    if provenance_directory.is_symlink():
        raise CoverageError("actuation provenance root must not be a symlink")
    root = provenance_directory.resolve()
    if not root.is_dir():
        raise CoverageError(f"actuation provenance root does not exist: {root}")
    manifest_path = root / "manifest.json"
    manifest = _read_object(manifest_path, "actuation provenance manifest")
    files = manifest.get("files")
    if not isinstance(files, dict):
        raise CoverageError("actuation provenance manifest has no files object")

    maps: dict[str, dict[str, Any]] = {}
    axes: dict[str, list[float]] = {}
    for name in ("accel_map", "brake_map"):
        record = files.get(name)
        if not isinstance(record, dict):
            raise CoverageError(f"actuation provenance is missing {name}")
        path = _artifact_path(root, record.get("artifact"), name)
        expected_sha = record.get("sha256")
        actual_sha = _sha256(path)
        if not isinstance(expected_sha, str) or actual_sha != expected_sha:
            raise CoverageError(f"{name} artifact SHA-256 does not match its manifest")
        axis = _velocity_axis(path, name)
        axes[name] = axis
        maps[name] = {
            "artifact": path.relative_to(root).as_posix(),
            "sha256": actual_sha,
            "velocity_axis_mps": axis,
            "minimum_velocity_mps": axis[0],
            "maximum_velocity_mps": axis[-1],
            "target_within_axis": axis[0] <= target_speed_mps <= axis[-1],
        }
    if axes["accel_map"] != axes["brake_map"]:
        raise CoverageError("accel and brake velocity axes differ")

    axis_minimum = axes["accel_map"][0]
    axis_maximum = axes["accel_map"][-1]
    target_within_axis = axis_minimum <= target_speed_mps <= axis_maximum
    if target_within_axis:
        status = "PASS"
        target_envelope_classification = "TARGET_ENVELOPE_COVERED_BY_MAP_AXIS"
    elif allow_clamped_target:
        status = "EXPLORATORY"
        target_envelope_classification = (
            "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
        )
    else:
        status = "BLOCKED"
        target_envelope_classification = (
            "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_NOT_AUTHORIZED"
        )

    runtime_observation: dict[str, Any]
    if observed_maximum_speed_mps is None:
        runtime_observation = {
            "available": False,
            "classification": "PREFLIGHT_ONLY_NO_OBSERVED_SPEED",
            "velocity_axis_clamping_observed": None,
        }
    else:
        observed_within_axis = (
            axis_minimum <= observed_maximum_speed_mps <= axis_maximum
        )
        runtime_observation = {
            "available": True,
            "maximum_absolute_current_speed_mps": observed_maximum_speed_mps,
            "maximum_absolute_current_speed_kph": observed_maximum_speed_mps * 3.6,
            "within_map_velocity_axis": observed_within_axis,
            "velocity_axis_clamping_observed": not observed_within_axis,
            "classification": (
                "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS"
                if observed_within_axis
                else "OBSERVED_SPEED_REACHED_CLAMPED_MAP_REGION"
            ),
        }

    return {
        "schema_version": 1,
        "analysis": "raw_vehicle_command_converter_velocity_coverage",
        "status": status,
        "profile_id": profile_id,
        "target_speed_mps": target_speed_mps,
        "target_speed_kph": target_speed_mps * 3.6,
        "map_velocity_axis_maximum_mps": axis_maximum,
        "map_velocity_axis_maximum_kph": axis_maximum * 3.6,
        "map_velocity_axis_minimum_mps": axis_minimum,
        "map_velocity_axis_minimum_kph": axis_minimum * 3.6,
        "target_excess_mps": max(0.0, target_speed_mps - axis_maximum),
        "target_shortfall_mps": max(0.0, axis_minimum - target_speed_mps),
        "target_within_map_velocity_axis": target_within_axis,
        "target_envelope_classification": target_envelope_classification,
        "target_envelope_extension_authorized_for_exploratory_simulation": bool(
            allow_clamped_target and not target_within_axis
        ),
        "runtime_lookup_observation": runtime_observation,
        "validation_boundary": {
            "simulation_only": True,
            "map_coverage_above_axis": False,
            "real_vehicle_ready": False,
            "route_result_pass_is_high_speed_actuation_calibration": False,
            "target_speed_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": (
                "absolute_current_odometry_longitudinal_speed_mps"
            ),
        },
        "provenance": {
            "root": str(root),
            "manifest": {
                "path": str(manifest_path),
                "sha256": _sha256(manifest_path),
            },
            "maps": maps,
            "converter_implementations": _implementation_records(
                converter_source_directory.resolve()
            ),
            "converter_node": _converter_node_record(
                converter_source_directory.resolve()
            ),
        },
    }


def _atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    staged = Path(staged_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--provenance-dir", required=True, type=Path)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--target-speed-mps", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--allow-target-envelope-beyond-axis",
        "--allow-clamped-target",
        dest="allow_target_envelope_beyond_axis",
        action="store_true",
        help=(
            "allow an exploratory CARLA target envelope beyond the CSV axis; "
            "the legacy --allow-clamped-target alias is retained, and runtime "
            "clamping still depends on observed current speed"
        ),
    )
    parser.add_argument(
        "--observed-maximum-speed-mps",
        type=float,
        help="post-run maximum absolute current speed used for runtime coverage",
    )
    parser.add_argument(
        "--converter-source-directory",
        type=Path,
        default=CONVERTER_SOURCE_DIRECTORY,
    )
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> int:
    try:
        payload = analyze_coverage(
            args.provenance_dir,
            profile_id=args.profile_id,
            target_speed_mps=args.target_speed_mps,
            allow_clamped_target=getattr(
                args,
                "allow_target_envelope_beyond_axis",
                getattr(args, "allow_clamped_target", False),
            ),
            observed_maximum_speed_mps=getattr(
                args, "observed_maximum_speed_mps", None
            ),
            converter_source_directory=args.converter_source_directory,
        )
    except CoverageError as error:
        payload = {
            "schema_version": 1,
            "analysis": "raw_vehicle_command_converter_velocity_coverage",
            "status": "ERROR",
            "error": f"{type(error).__name__}: {error}",
        }
        _atomic_write(args.output, payload)
        print(payload["error"])
        return 2
    _atomic_write(args.output, payload)
    print(
        "actuation map coverage: "
        f"status={payload['status']} target={payload['target_speed_kph']:.3f}kph "
        f"axis_max={payload['map_velocity_axis_maximum_kph']:.3f}kph"
    )
    return 1 if payload["status"] == "BLOCKED" else 0


def main(argv: Sequence[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
