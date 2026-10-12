"""HH_260906 - Exercise Town13 static blockers without starting CARLA or UE4Editor."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / ".git").exists())
SCRIPT = ROOT / "tools/e2e/audit_town13_source_editor_preflight.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("audit_town13_source_editor_preflight", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def fixture_manifest(tmp_path: Path):
    payloads = {name: name.encode("ascii") for name in audit.EXPECTED_SHA256}
    paths = {}
    for name, payload in payloads.items():
        path = tmp_path / name
        path.write_bytes(payload)
        paths[name] = str(path)
    expected = {name: hashlib.sha256(payload).hexdigest() for name, payload in payloads.items()}
    profile = {key: paths[key] for key in ("executable", "project", "python_api")}
    profile.update(kind="source_editor", python_api_sha256=expected["python_api"])
    map_entry = dict(audit.EXPECTED_MAP)
    map_entry.update({key: paths[key] for key in ("level_path", "opendrive_path")})
    manifest = {"maps": [map_entry], "server_profiles": {"source_editor_0915_4ws": profile}}
    return manifest, expected, paths


def fixture_route(tmp_path: Path):
    route = tmp_path / "town13_route.json"
    route.write_text(json.dumps({"town": "Town13", "coordinate_reference": "base_link",
                                 "start_spawn_index": 1, "goal_spawn_index": 2,
                                 "route": [{"x": 0}, {"x": 1}]}), encoding="utf-8")
    return route, hashlib.sha256(route.read_bytes()).hexdigest()


def test_all_static_inputs_never_authorize_source_editor_or_dev(tmp_path):
    manifest, expected, _ = fixture_manifest(tmp_path)
    route, digest = fixture_route(tmp_path)
    report = audit.audit_town13(manifest, route, digest, expected)
    assert all(asset["ok"] for asset in report["assets"].values())
    assert report["route"]["ok"] is True
    assert report["status"] == "STOP_NOT_EXECUTABLE"
    assert report["execution_authorized"] is False
    assert report["dataset_admitted"] is False
    assert report["map_model_validation_passed"] is False
    assert report["blockers"] == [
        "source_editor_catalog_owner_live_start_unverified",
        "town13_mapping_probe_scope_not_implemented",
        "owned_live_world_and_camera_visibility_unverified",
    ]


def test_missing_exact_route_is_a_separate_blocker(tmp_path):
    manifest, expected, _ = fixture_manifest(tmp_path)
    report = audit.audit_town13(manifest, expected_sha256=expected)
    assert "exact_route_and_sha256_not_declared" in report["blockers"]
    assert report["execution_authorized"] is False


def test_changed_asset_and_symlink_are_rejected(tmp_path):
    manifest, expected, paths = fixture_manifest(tmp_path)
    Path(paths["level_path"]).write_bytes(b"changed")
    link = tmp_path / "editor_link"
    link.symlink_to(paths["executable"])
    manifest["server_profiles"]["source_editor_0915_4ws"]["executable"] = str(link)
    report = audit.audit_town13(manifest, expected_sha256=expected)
    assert "level_path_sha256_mismatch" in report["blockers"]
    assert "executable_missing_nonregular_or_symlink" in report["blockers"]


def test_wrong_town_or_changed_route_bytes_are_rejected(tmp_path):
    route, digest = fixture_route(tmp_path)
    assert audit.inspect_route(route, "0" * 64)["reason"] == "route_sha256_mismatch"
    value = json.loads(route.read_text(encoding="utf-8"))
    value["town"] = "Town15"
    route.write_text(json.dumps(value), encoding="utf-8")
    new_digest = hashlib.sha256(route.read_bytes()).hexdigest()
    assert audit.inspect_route(route, new_digest)["reason"] == "route_town_or_coordinate_mismatch"
    assert audit.inspect_route(route, digest)["reason"] == "route_sha256_mismatch"


def test_suite_split_drift_never_uses_test_as_validation(tmp_path):
    manifest, expected, _ = fixture_manifest(tmp_path)
    manifest["maps"][0]["split"] = "test"
    report = audit.audit_town13(manifest, expected_sha256=expected)
    assert "town13_suite_binding_mismatch" in report["blockers"]
    assert report["execution_authorized"] is False
