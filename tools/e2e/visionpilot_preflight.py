#!/usr/bin/env python3
"""HH_260906 - Inspect VisionPilot comparison prerequisites without importing models or changing the environment."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import math
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.visionpilot_preflight.v1"
SOURCE_COMMIT = "f9fb99799e779ac602fde06175f2ec3ea3df3f6f"
MODEL_COMMITS = {
    "auto_speed": "b62e55b93ba728787cb1152347463030443634c4",
    "auto_steer": "727c8ae62b3e729913bd28596af768697a99c874",
    "auto_drive": "83cf0fa1a8e8ee760dd854e60ecee8a03ab9072d",
}
MODULES = {"onnxruntime": ("onnxruntime", "onnxruntime-gpu"),
           "cv2": ("opencv-python", "opencv-python-headless"), "onnx": ("onnx",)}
BASE_URL = f"https://github.com/autowarefoundation/vision_pilot/blob/{SOURCE_COMMIT}/"
REFERENCES = {
    "scope": BASE_URL + "README.md",
    "sensor": BASE_URL + "github-io/hardware.md",
    "output": BASE_URL + "VisionPilot/modules/middleware_interfaces/ros2_interface/vehicle_ros2_interface/src/vehicle_ros2_interface.cpp",
    "native_build": BASE_URL + "VisionPilot/CMakeLists.txt",
    "safety": BASE_URL + "Functional_Safety/SAFETY_ELEMENT_OUT_OF_CONTEXT.md",
}


class PreflightError(ValueError):
    """HH_260906 - Use public-safe fixed failure messages instead of exposing local paths or exception payloads."""


def require(condition, message):
    if not condition:
        raise PreflightError(message)


def canonical_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def parsed_version(value):
    """HH_260906 - Keep numeric version facts, not arbitrary metadata strings that might contain private identifiers."""
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+){0,3})(?:(a|b|rc)([0-9]+))?(?:\.post([0-9]+))?(?:\.dev([0-9]+))?(?:\+([A-Za-z0-9._-]+))?", value or "")
    if not match or len(value) > 100:
        return {"status": "UNRECOGNIZED_VERSION_FORMAT"}
    return {"status": "OBSERVED", "release": [int(x) for x in match[1].split(".")],
        "prerelease": {"kind": match[2], "number": int(match[3])} if match[2] else None,
        "post": int(match[4]) if match[4] else None, "dev": int(match[5]) if match[5] else None,
        "local_build_suffix_present_but_not_retained": match[6] is not None}


def discover_python():
    """HH_260906 - Search top-level module specs and package metadata only; never import cv2, ONNX Runtime, torch or CUDA."""
    observations = {}
    for module, distributions in MODULES.items():
        try:
            found = importlib.util.find_spec(module) is not None
            status = "FOUND" if found else "NOT_FOUND_IN_INTERPRETER_SEARCH_PATH"
        except Exception:
            status = "DISCOVERY_ERROR"
        packages = {}
        for package in distributions:
            try:
                packages[package] = parsed_version(importlib.metadata.version(package))
            except importlib.metadata.PackageNotFoundError:
                packages[package] = {"status": "DISTRIBUTION_METADATA_NOT_FOUND"}
            except Exception:
                packages[package] = {"status": "METADATA_DISCOVERY_ERROR"}
        observations[module] = {"module_spec_status": status, "distribution_metadata": packages}
    return {"python_release": list(sys.version_info[:3]), "modules": observations,
        "method": "Top-level importlib.util.find_spec and importlib.metadata.version; no module origins retained.",
        "candidate_modules_imported": False, "runtime_providers_queried": False}


def discover_native():
    """HH_260906 - Read only the system loader cache using a fixed command and discard every resolved library path."""
    executable = next((p for p in (Path("/sbin/ldconfig"), Path("/usr/sbin/ldconfig")) if p.is_file()), None)
    result = {"command": "ldconfig -p", "status": "COMMAND_NOT_AVAILABLE", "matched_sonames": [],
        "search_scope": "System dynamic-loader cache only; private SDKs, headers and other environments are not enumerated.",
        "candidate_library_loaded": False}
    if executable is None:
        return result
    try:
        completed = subprocess.run([str(executable), "-p"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=5, check=False, env={"LC_ALL": "C", "LANG": "C"})
    except subprocess.TimeoutExpired:
        result["status"] = "COMMAND_TIMED_OUT"
        return result
    except Exception:
        result["status"] = "COMMAND_FAILED"
        return result
    if completed.returncode != 0:
        result["status"] = "COMMAND_FAILED"
        return result
    if len(completed.stdout) > 8 * 1024 * 1024:
        result["status"] = "OUTPUT_TOO_LARGE"
        return result
    found = set()
    for line in completed.stdout.decode("utf-8", errors="replace").splitlines():
        match = re.match(r"\s*(lib(?:onnxruntime|opencv_core)\.so(?:\.[0-9]+)*(?:[a-z])?)\s+\([^\n]*\)\s+=>\s+", line)
        if match:
            found.add(match[1])
    result.update(status="CACHE_READ", matched_sonames=sorted(found))
    return result


def sensor_reference(path):
    """HH_260906 - Bind optional local source bytes without pretending to parse YAML or to verify declared sensor values."""
    if path is None:
        return None
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "sensor reference must be a regular non-symlink file")
    require(path.stat().st_size <= 1024 * 1024, "sensor reference is too large")
    try:
        payload = path.read_bytes()
    except OSError as error:
        raise PreflightError("sensor reference could not be read") from error
    require(len(payload) <= 1024 * 1024, "sensor reference is too large")
    return {"sha256": sha(payload), "size_bytes": len(payload), "contents_parsed": False,
        "declared_values_verified_against_source": False, "path_not_retained": True}


def declarations(args):
    require(type(args.camera_count) is int and 1 <= args.camera_count <= 32, "invalid declared camera count")
    for value in (args.front_width, args.front_height):
        require(type(value) is int and 1 <= value <= 16384, "invalid declared front-camera dimensions")
    require(isinstance(args.front_hfov_deg, (int, float)) and type(args.front_hfov_deg) is not bool
            and math.isfinite(args.front_hfov_deg) and 0 < args.front_hfov_deg < 180, "invalid declared front-camera field of view")
    require(isinstance(args.camera_hz, (int, float)) and type(args.camera_hz) is not bool
            and math.isfinite(args.camera_hz) and 0 < args.camera_hz <= 1000, "invalid declared camera cadence")
    require(args.output_kind in ("timed_trajectory", "steering_acceleration"), "unknown declared output kind")
    require(args.task in ("in_lane_adas", "route_conditioned_urban"), "unknown declared task")
    require(re.fullmatch(r"[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}", args.carla_version) is not None,
            "invalid declared simulator version")
    return {"schema": "portable_e2e.visionpilot_declared_inputs.v1", "values_origin": "CLI_DECLARATION_NOT_LIVE_OBSERVATION",
        "default_basis": "Current documented Portable comparison defaults; not a live sensor/config measurement.",
        "sensor": {"camera_count": args.camera_count, "front_width": args.front_width, "front_height": args.front_height,
            "front_horizontal_fov_deg": args.front_hfov_deg, "camera_hz": args.camera_hz},
        "output_kind": args.output_kind, "requested_task": args.task,
        "carla_release": [int(x) for x in args.carla_version.split(".")],
        "optional_sensor_source_bytes": sensor_reference(args.sensor_config)}


def assess(inputs, python, native):
    """HH_260906 - Module presence and a matching declared sensor never establish executable, timing or vehicle readiness."""
    sensor = inputs["sensor"]
    findings = []
    if sensor["camera_count"] != 1:
        findings.append("REQUIRES_EXPLICIT_FRONT_VIEW_SELECTION")
    if not (50 <= sensor["front_horizontal_fov_deg"] <= 55 and
            1_000_000 <= sensor["front_width"] * sensor["front_height"] <= 2_000_000):
        findings.append("REQUIRES_SENSOR_ADAPTER_OR_QUALIFIED_SENSOR_PROFILE")
    if not math.isclose(sensor["camera_hz"], 10., abs_tol=1e-9, rel_tol=0):
        findings.append("REQUIRES_TEMPORAL_INPUT_CONTRACT_REVIEW")
    if inputs["output_kind"] == "timed_trajectory":
        findings.append("REQUIRES_OUTPUT_ADAPTER")
    if inputs["requested_task"] != "in_lane_adas":
        findings.append("REQUESTED_TASK_EXCEEDS_REVIEWED_IN_LANE_SCOPE")
    if inputs["carla_release"] != [0, 9, 16]:
        findings.append("SIMULATOR_BRIDGE_VERSION_DIFFERENCE_UNVERIFIED")
    ort = python["modules"]["onnxruntime"]["module_spec_status"]
    if ort != "FOUND":
        findings.append("PYTHON_ONNX_RUNTIME_DISCOVERY_INCOMPLETE")
    native_ort = any(n.startswith("libonnxruntime.so") for n in native["matched_sonames"])
    if native["status"] != "CACHE_READ" or not native_ort:
        findings.append("NATIVE_ONNX_RUNTIME_SDK_UNVERIFIED")
    findings.extend(("CAMERA_CALIBRATION_AND_PREPROCESSING_UNVERIFIED", "OFFICIAL_WEIGHT_BYTES_NOT_VERIFIED",
        "NATIVE_HEADERS_AND_ABI_UNVERIFIED", "RUNTIME_INFERENCE_NOT_RUN", "TEN_HZ_DEADLINE_NOT_MEASURED",
        "NONACTUATING_ADAPTER_AND_FAILSAFE_NOT_VERIFIED"))
    return {"status": "NOT_READY", "execution_status": "NOT_RUN", "findings": findings,
        "reviewed_capability_contract": {"task": "single_front_camera_in_lane_L2_ADAS",
            "front_horizontal_fov_deg": [50, 55], "documented_nominal_resolution_megapixels": [1, 2],
            "resolution_check_policy": "Conservative literal pixel-count band; the official exact supported-grid list was not established. Nominal 2 MP formats just above 2000000 pixels need profile review, not an assertion of incompatibility.",
            "design_camera_hz": 10, "official_simulator_reference": [0, 9, 16],
            "reviewed_ros2_vehicle_outputs": ["std_msgs/Float64 steering", "std_msgs/Float64 acceleration"],
            "autoware_timed_trajectory_output_implemented_in_reviewed_interface": False,
            "model_outputs": {"auto_speed": "in-path/cutting-in-or-out/out-of-path object detections",
                "auto_steer": "image-coordinate in-lane spatial path", "auto_drive": "CIPO distance/presence and road curvature from two frames"}},
        "important_distinctions": [
            "Python ONNX Runtime is not the C++ SDK required by the official native stack.",
            "A missing module/cache entry does not prove absence from private SDKs or other virtual environments.",
            "A discovered spec, version or SONAME is not an import, ABI, provider, inference or performance test.",
            "Resize/crop cannot recreate missing scene detail; calibration and original sensor conditions require validation.",
            "One-camera in-lane L2 output is not the full route-conditioned multi-camera feature roadmap.",
            "Declared 10 Hz and official design cadence do not establish actual throughput, latency or GUI FPS."],
        "unknowns": ["usable official model files", "native SDK headers and runtime ABI", "camera calibration",
            "preprocessing parity", "end-to-end latency distribution", "vehicle command authority isolation",
            "closed-loop quality", "real-vehicle fitness"]}


def checked_output(path):
    path = Path(path)
    require(not path.exists() and not path.is_symlink(), "output directory must be new")
    # HH_260906 - Protect both the workspace spelling and the real target when datasets is a mounted symlink.
    dataset_roots = (ROOT / "datasets", (ROOT / "datasets").resolve())
    require(not any(path.resolve().is_relative_to(root) for root in dataset_roots), "output must stay outside datasets")
    require(not any(p.is_symlink() for p in path.absolute().parents), "output ancestors must not be symlinks")
    return path


def run_preflight(args):
    """HH_260906 - Produce an authenticated new report directory only; failures never leave a success checksum marker."""
    output = checked_output(args.output_dir)
    source_before = sha(Path(__file__).read_bytes())
    inputs = declarations(args)
    python, native = discover_python(), discover_native()
    report = {"schema": SCHEMA, "preflight_status": "COMPLETE", "model_execution_status": "NOT_RUN",
        "completion_proof_required": "The directory is complete only with matching final SHA256SUMS for declared_inputs.json and report.json; a partial report without this marker is not completed evidence.",
        "source_sha256": source_before, "input_contract_sha256": sha(canonical_bytes(inputs)),
        "reviewed_official_sources": {"vision_pilot": SOURCE_COMMIT, "models": MODEL_COMMITS, "references": REFERENCES},
        "assessment": assess(inputs, python, native), "observations": {"python": python, "native_loader_cache": native},
        "scope": {"read_only_discovery": True, "network_access": False, "remote_access": False,
            "model_download": False, "repository_clone": False, "installation": False, "native_build": False,
            "model_inference": False, "gpu_access": False, "live_simulator_access": False,
            "config_modified": False, "training": False, "training_data_approved": False,
            "vehicle_control_approved": False, "readiness_inferred_from_module_presence": False},
        "next_step": "Review the declared sensor/task/output contract and missing prerequisites before authorizing an isolated offline baseline; no installation is authorized by this report."}
    require(sensor_reference(args.sensor_config) == inputs["optional_sensor_source_bytes"], "sensor reference changed during discovery")
    require(sha(Path(__file__).read_bytes()) == source_before, "preflight source changed during discovery")
    output.mkdir(parents=True, exist_ok=False, mode=0o700)
    for name, value in (("declared_inputs.json", inputs), ("report.json", report)):
        with (output / name).open("xb") as handle:
            handle.write(canonical_bytes(value))
    require(sensor_reference(args.sensor_config) == inputs["optional_sensor_source_bytes"], "sensor reference changed during publication")
    require(sha(Path(__file__).read_bytes()) == source_before, "preflight source changed during publication")
    with (output / "SHA256SUMS").open("x") as handle:
        for name in ("declared_inputs.json", "report.json"):
            handle.write(f"{sha((output / name).read_bytes())}  {name}\n")
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sensor-config", type=Path, help="Optional bytes-only hash reference; values are still CLI declarations, not parsed observations.")
    parser.add_argument("--camera-count", type=int, default=6)
    parser.add_argument("--front-width", type=int, default=640)
    parser.add_argument("--front-height", type=int, default=360)
    parser.add_argument("--front-hfov-deg", type=float, default=70.)
    parser.add_argument("--camera-hz", type=float, default=10.)
    parser.add_argument("--output-kind", choices=("timed_trajectory", "steering_acceleration"), default="timed_trajectory")
    parser.add_argument("--task", choices=("in_lane_adas", "route_conditioned_urban"), default="route_conditioned_urban")
    parser.add_argument("--carla-version", default="0.9.15")
    return parser.parse_args(argv)


def main(argv=None):
    try:
        result = run_preflight(parse_args(argv))
    except (PreflightError, OSError):
        print("VisionPilot preflight failed; no readiness or completed evidence is established.", file=sys.stderr)
        return 2
    print(json.dumps({"preflight_status": "COMPLETE", "assessment_status": result["assessment"]["status"],
        "model_execution_status": "NOT_RUN"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
