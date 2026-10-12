"""HH_260906 - Verify read-only discovery, explicit declarations, public-safe evidence and fail-closed preflight output."""

import builtins
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import subprocess

import pytest

from scripts.e2e import visionpilot_preflight as module


def arguments(tmp_path, **values):
    args = module.parse_args(["--output-dir", str(tmp_path / "new-report")])
    for key, value in values.items():
        setattr(args, key, value)
    return args


def python_observation(found=True):
    return {"python_release": [3, 10, 12], "modules": {name: {
        "module_spec_status": "FOUND" if found else "NOT_FOUND_IN_INTERPRETER_SEARCH_PATH",
        "distribution_metadata": {}} for name in module.MODULES}, "candidate_modules_imported": False}


def native_observation(found=True):
    return {"status": "CACHE_READ", "matched_sonames": ["libonnxruntime.so.1"] if found else []}


@pytest.fixture
def discovery(monkeypatch):
    monkeypatch.setattr(module, "discover_python", lambda: python_observation())
    monkeypatch.setattr(module, "discover_native", lambda: native_observation())


def test_default_contract_is_declared_not_observed_and_mismatches_are_explicit(tmp_path):
    inputs = module.declarations(arguments(tmp_path))
    assert inputs["values_origin"] == "CLI_DECLARATION_NOT_LIVE_OBSERVATION"
    assert inputs["optional_sensor_source_bytes"] is None
    assert inputs["sensor"] == {"camera_count": 6, "front_width": 640, "front_height": 360,
        "front_horizontal_fov_deg": 70., "camera_hz": 10.}
    result = module.assess(inputs, python_observation(), native_observation())
    assert result["status"] == "NOT_READY"
    assert result["execution_status"] == "NOT_RUN"
    assert {"REQUIRES_EXPLICIT_FRONT_VIEW_SELECTION", "REQUIRES_SENSOR_ADAPTER_OR_QUALIFIED_SENSOR_PROFILE",
        "REQUIRES_OUTPUT_ADAPTER", "REQUESTED_TASK_EXCEEDS_REVIEWED_IN_LANE_SCOPE",
        "SIMULATOR_BRIDGE_VERSION_DIFFERENCE_UNVERIFIED", "TEN_HZ_DEADLINE_NOT_MEASURED"}.issubset(result["findings"])


def test_matching_declarations_and_all_libraries_found_never_mean_ready(tmp_path):
    inputs = module.declarations(arguments(tmp_path, camera_count=1, front_width=1920, front_height=1080,
        front_hfov_deg=52., output_kind="steering_acceleration", task="in_lane_adas", carla_version="0.9.16"))
    # HH_260906 - Keep this fixture inside the conservative literal band; nominal 1080p is not declared incompatible.
    inputs["sensor"]["front_width"] = 1600
    inputs["sensor"]["front_height"] = 1200
    result = module.assess(inputs, python_observation(), native_observation())
    assert "REQUIRES_SENSOR_ADAPTER_OR_QUALIFIED_SENSOR_PROFILE" not in result["findings"]
    assert "REQUIRES_OUTPUT_ADAPTER" not in result["findings"]
    assert "REQUESTED_TASK_EXCEEDS_REVIEWED_IN_LANE_SCOPE" not in result["findings"]
    assert result["status"] == "NOT_READY" and result["execution_status"] == "NOT_RUN"
    assert "CAMERA_CALIBRATION_AND_PREPROCESSING_UNVERIFIED" in result["findings"]
    assert "RUNTIME_INFERENCE_NOT_RUN" in result["findings"]


@pytest.mark.parametrize("field,value", [("camera_count", 0), ("camera_count", True), ("camera_count", 33),
    ("front_width", 0), ("front_height", 16385), ("front_width", 640.), ("front_hfov_deg", float("nan")),
    ("front_hfov_deg", float("inf")), ("front_hfov_deg", 180.), ("front_hfov_deg", True),
    ("camera_hz", -1.), ("camera_hz", float("nan")), ("camera_hz", True), ("camera_hz", 1001.),
    ("task", "driverless"), ("output_kind", "direct_can"), ("carla_version", "0.9.16/private"),
    ("carla_version", "10.20.30.40")])
def test_invalid_declarations_fail_before_discovery(tmp_path, monkeypatch, field, value):
    monkeypatch.setattr(module, "discover_python", lambda: pytest.fail("discovery must not run"))
    with pytest.raises(module.PreflightError):
        module.run_preflight(arguments(tmp_path, **{field: value}))
    assert not (tmp_path / "new-report").exists()


def test_discovery_never_imports_candidate_modules_or_reports_origin(monkeypatch):
    specs, packages = [], []
    def find(name):
        specs.append(name)
        return SimpleNamespace(origin="/home/private/person/secret") if name != "onnxruntime" else None
    def version(name):
        packages.append(name)
        if name == "onnx": return "1.18.0"
        if name == "onnxruntime-gpu": return "1.20.0+private.localbuild"
        raise module.importlib.metadata.PackageNotFoundError(name)
    original_import = builtins.__import__
    def guarded(name, *args, **kwargs):
        assert name.split(".")[0] not in {"torch", "onnxruntime", "cv2", "onnx", "carla"}
        return original_import(name, *args, **kwargs)
    monkeypatch.setattr(module.importlib.util, "find_spec", find)
    monkeypatch.setattr(module.importlib.metadata, "version", version)
    monkeypatch.setattr(builtins, "__import__", guarded)
    result = module.discover_python()
    assert specs == ["onnxruntime", "cv2", "onnx"]
    assert len(packages) == 5
    assert result["candidate_modules_imported"] is False
    assert result["modules"]["onnxruntime"]["module_spec_status"] == "NOT_FOUND_IN_INTERPRETER_SEARCH_PATH"
    assert result["modules"]["onnx"]["distribution_metadata"]["onnx"]["release"] == [1, 18, 0]
    payload = json.dumps(result)
    assert "/home/" not in payload and "secret" not in payload and "private.localbuild" not in payload


def test_discovery_errors_keep_unknowns_without_private_exception_text(monkeypatch):
    def fail(*args): raise RuntimeError("/home/private host-user@host 10.20.30.40")
    monkeypatch.setattr(module.importlib.util, "find_spec", fail)
    monkeypatch.setattr(module.importlib.metadata, "version", fail)
    result = module.discover_python()
    assert all(v["module_spec_status"] == "DISCOVERY_ERROR" for v in result["modules"].values())
    assert "private" not in json.dumps(result) and "10.20.30.40" not in json.dumps(result)


@pytest.mark.parametrize("value", ["/home/person/key", "user@host", "1.0+secret@host", "secret", "x" * 1000])
def test_arbitrary_version_metadata_is_not_echoed(value):
    assert module.parsed_version(value) == {"status": "UNRECOGNIZED_VERSION_FORMAT"}


def test_version_fields_cannot_look_like_a_private_network_address():
    assert module.parsed_version("10.20.30.40")["release"] == [10, 20, 30, 40]
    assert "10.20.30.40" not in json.dumps(module.parsed_version("10.20.30.40"))
    assert module.parsed_version("2.0.0rc2.post1.dev4+site.private")["prerelease"] == {"kind": "rc", "number": 2}


def test_native_command_is_fixed_sanitized_bounded_and_does_not_load_a_library(monkeypatch):
    monkeypatch.setattr(module.Path, "is_file", lambda p: str(p) == "/sbin/ldconfig")
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=b"libonnxruntime.so.1 (libc6) => /home/private/sdk/lib.so\n"
            b"libopencv_core.so.4.5d (libc6) => /lib/example\n"
            b"libonnxruntime_providers_cuda.so (libc6) => /home/private/gpu\n"
            b"secret@host 10.20.30.40\n", stderr=b"private error")
    monkeypatch.setattr(module.subprocess, "run", run)
    result = module.discover_native()
    assert result["status"] == "CACHE_READ"
    assert result["matched_sonames"] == ["libonnxruntime.so.1", "libopencv_core.so.4.5d"]
    assert result["candidate_library_loaded"] is False
    assert calls[0][0] == ["/sbin/ldconfig", "-p"]
    assert calls[0][1]["timeout"] == 5 and calls[0][1]["env"] == {"LC_ALL": "C", "LANG": "C"}
    assert "shell" not in calls[0][1]
    assert "/home/" not in json.dumps(result) and "10.20.30.40" not in json.dumps(result)


@pytest.mark.parametrize("fault,status", [("missing", "COMMAND_NOT_AVAILABLE"), ("timeout", "COMMAND_TIMED_OUT"),
    ("error", "COMMAND_FAILED"), ("nonzero", "COMMAND_FAILED"), ("large", "OUTPUT_TOO_LARGE")])
def test_native_failure_does_not_become_absence_or_readiness(monkeypatch, fault, status):
    monkeypatch.setattr(module.Path, "is_file", lambda p: fault != "missing")
    def run(*args, **kwargs):
        if fault == "timeout": raise subprocess.TimeoutExpired("private", 5)
        if fault == "error": raise OSError("/home/private")
        return SimpleNamespace(returncode=1 if fault == "nonzero" else 0,
            stdout=b"x" * (8 * 1024 * 1024 + 1) if fault == "large" else b"", stderr=b"")
    monkeypatch.setattr(module.subprocess, "run", run)
    result = module.discover_native()
    assert result["status"] == status and result["matched_sonames"] == []


def test_fresh_report_is_deterministic_hash_bound_and_never_ready(tmp_path, discovery):
    reference = tmp_path / "private_sensor.yaml"
    reference.write_text("camera: private-unparsed-text\n")
    first = arguments(tmp_path, sensor_config=reference)
    result = module.run_preflight(first)
    second = arguments(tmp_path, sensor_config=reference, output_dir=tmp_path / "second")
    assert module.run_preflight(second) == result
    assert result["assessment"]["status"] == "NOT_READY"
    assert result["scope"]["readiness_inferred_from_module_presence"] is False
    assert {p.name for p in first.output_dir.iterdir()} == {"declared_inputs.json", "report.json", "SHA256SUMS"}
    inputs = json.loads((first.output_dir / "declared_inputs.json").read_bytes())
    assert inputs["optional_sensor_source_bytes"]["sha256"] == hashlib.sha256(reference.read_bytes()).hexdigest()
    assert inputs["optional_sensor_source_bytes"]["contents_parsed"] is False
    assert inputs["optional_sensor_source_bytes"]["declared_values_verified_against_source"] is False
    assert result["input_contract_sha256"] == hashlib.sha256((first.output_dir / "declared_inputs.json").read_bytes()).hexdigest()
    for line in (first.output_dir / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert hashlib.sha256((first.output_dir / name).read_bytes()).hexdigest() == expected
        assert (first.output_dir / name).read_bytes() == (second.output_dir / name).read_bytes()
    assert "private-unparsed-text" not in "".join(p.read_text() for p in first.output_dir.iterdir())


@pytest.mark.parametrize("kind", ["existing", "symlink", "datasets", "symlink_ancestor"])
def test_output_rejected_before_discovery(tmp_path, monkeypatch, kind):
    monkeypatch.setattr(module, "ROOT", tmp_path)
    output = tmp_path / "new"
    if kind == "existing": output.mkdir()
    elif kind == "symlink": output.symlink_to(tmp_path / "missing")
    elif kind == "datasets": output = tmp_path / "datasets" / "new"
    else:
        (tmp_path / "link").symlink_to(tmp_path, target_is_directory=True)
        output = tmp_path / "link" / "new"
    monkeypatch.setattr(module, "discover_python", lambda: pytest.fail("unsafe destination must fail first"))
    with pytest.raises(module.PreflightError): module.run_preflight(arguments(tmp_path, output_dir=output))


def test_real_dataset_alias_target_is_protected(tmp_path, monkeypatch):
    workspace = tmp_path / "workspace"
    workspace.mkdir()
    dataset_target = tmp_path / "mounted_dataset"
    dataset_target.mkdir()
    (workspace / "datasets").symlink_to(dataset_target, target_is_directory=True)
    monkeypatch.setattr(module, "ROOT", workspace)
    monkeypatch.setattr(module, "discover_python", lambda: pytest.fail("dataset destination must fail first"))
    with pytest.raises(module.PreflightError, match="outside datasets"):
        module.run_preflight(arguments(tmp_path, output_dir=dataset_target / "new-output"))
    assert not (dataset_target / "new-output").exists()


@pytest.mark.parametrize("kind", ["missing", "symlink", "large"])
def test_sensor_reference_invalid(tmp_path, kind):
    source = tmp_path / "source"
    if kind == "symlink": source.symlink_to(tmp_path / "missing")
    if kind == "large": source.write_bytes(b"x" * (1024 * 1024 + 1))
    with pytest.raises(module.PreflightError): module.sensor_reference(source)


def test_source_change_during_discovery_is_not_published(tmp_path, monkeypatch):
    source = tmp_path / "sensor"
    source.write_text("old")
    def change():
        source.write_text("new")
        return python_observation()
    monkeypatch.setattr(module, "discover_python", change)
    monkeypatch.setattr(module, "discover_native", native_observation)
    with pytest.raises(module.PreflightError, match="changed during discovery"):
        module.run_preflight(arguments(tmp_path, sensor_config=source))
    assert not (tmp_path / "new-report").exists()


def test_persist_failure_leaves_no_completed_checksum_and_safe_cli_error(tmp_path, monkeypatch, discovery, capsys):
    original = Path.open
    def fail(path, *args, **kwargs):
        if path.name == "report.json": raise OSError("/home/private/error")
        return original(path, *args, **kwargs)
    monkeypatch.setattr(module.Path, "open", fail)
    assert module.main(["--output-dir", str(tmp_path / "new-report")]) == 2
    assert "private" not in capsys.readouterr().err
    assert not (tmp_path / "new-report" / "SHA256SUMS").exists()


def test_sensor_source_change_during_publication_leaves_no_checksum(tmp_path, monkeypatch, discovery):
    source = tmp_path / "sensor"
    source.write_text("old")
    original = Path.open
    def mutate(path, *args, **kwargs):
        if path.name == "report.json": source.write_text("changed-after-discovery")
        return original(path, *args, **kwargs)
    monkeypatch.setattr(module.Path, "open", mutate)
    with pytest.raises(module.PreflightError, match="during publication"):
        module.run_preflight(arguments(tmp_path, sensor_config=source))
    assert not (tmp_path / "new-report" / "SHA256SUMS").exists()


def test_preflight_source_change_is_not_authenticated(tmp_path, monkeypatch, discovery):
    source_bytes = Path(module.__file__).read_bytes()
    original = module.sha
    calls = []
    def changed(payload):
        if payload == source_bytes:
            calls.append(True)
            if len(calls) >= 3: return "f" * 64
        return original(payload)
    monkeypatch.setattr(module, "sha", changed)
    with pytest.raises(module.PreflightError, match="source changed during publication"):
        module.run_preflight(arguments(tmp_path))
    assert not (tmp_path / "new-report" / "SHA256SUMS").exists()


def test_cli_runs_read_only_phase_and_requires_no_implicit_install(tmp_path, discovery, capsys):
    assert module.main(["--output-dir", str(tmp_path / "new-report")]) == 0
    assert json.loads(capsys.readouterr().out) == {"preflight_status": "COMPLETE", "assessment_status": "NOT_READY", "model_execution_status": "NOT_RUN"}
    report = json.loads((tmp_path / "new-report" / "report.json").read_text())
    assert report["preflight_status"] == "COMPLETE" and report["model_execution_status"] == "NOT_RUN"
    assert "SHA256SUMS" in report["completion_proof_required"]
    assert report["reviewed_official_sources"]["vision_pilot"] == module.SOURCE_COMMIT
    for flag in ("network_access", "remote_access", "model_download", "installation", "native_build", "model_inference", "gpu_access", "live_simulator_access", "training", "vehicle_control_approved"):
        assert report["scope"][flag] is False


def test_cli_rejects_abbreviations(tmp_path):
    with pytest.raises(SystemExit): module.parse_args(["--output-d", str(tmp_path / "new")])
