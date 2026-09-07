"""HH_260906 - Verify exact historical bounds bytes without weakening limits or contacting a simulator."""

import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from scripts.e2e import summarize_carla_goal_stop_trials as base
from scripts.e2e import summarize_carla_low_speed_response as pedal


def put(path, raw):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(raw)


def git(repo, *args):
    return subprocess.run(["git", *args], cwd=repo, capture_output=True, check=True).stdout.decode().strip()


@pytest.fixture
def historical(tmp_path, monkeypatch):
    # HH_260906 - A temporary local repository makes changed-live-source and missing-history cases reproducible.
    repo = tmp_path / "repo"
    repo.mkdir()
    raw = {base.BOUNDS_SOURCE_PATHS[0]: b"PHYSICAL_MAXIMUM_ACCELERATION_MPS2 = 2.9\n",
           base.BOUNDS_SOURCE_PATHS[1]: b"class RuntimeGateConfig:\n    maximum_acceleration_mps2: float = 3.0\n    maximum_deceleration_mps2: float = 6.0\n"}
    for name, value in raw.items():
        put(repo / name, value)
    git(repo, "init", "--quiet")
    git(repo, "add", "portable_e2e")
    git(repo, "-c", "user.name=Bounds Test", "-c", "user.email=bounds@example.invalid", "commit", "--quiet", "-m", "Frozen bounds fixture")
    commit = git(repo, "rev-parse", "HEAD")
    monkeypatch.setattr(base, "ROOT", repo)
    bounds = base.source_bounds(raw)
    put(repo / base.BOUNDS_SOURCE_PATHS[0], raw[base.BOUNDS_SOURCE_PATHS[0]] + b"# HH_260906 - Later model architecture.\n")
    trial = tmp_path / "run_001"
    trial.mkdir()
    return {"repo": repo, "trial": trial, "raw": raw, "bounds": bounds,
            "plan": {"source_head_commit": commit}, "commit": commit}


def resolve(fixture, **kwargs):
    return base.resolve_bounds_source(fixture["trial"], fixture["bounds"], fixture["plan"], **kwargs)


def archive(fixture):
    fixture["plan"].update(bounds_source_bytes_archived=True,
                           source_sha256=copy.deepcopy(fixture["bounds"]["source_sha256"]))
    for name, raw in fixture["raw"].items():
        put(fixture["trial"] / "provenance" / name, raw)


def test_historical_changed_live_source_preserves_recorded_limits_and_hashes(historical):
    bounds, proof = resolve(historical)
    assert bounds == historical["bounds"]
    assert bounds["source_sha256"] != base.source_bounds()["source_sha256"]
    assert {item["proof_kind"] for item in proof["sources"].values()} == {"owner_plan_git_blob_hash_match"}
    assert {item["git_commit"] for item in proof["sources"].values()} == {historical["commit"]}
    assert proof["historical_execution_proven_by_this_check"] is False
    assert proof["git_fetch_attempted"] is False
    assert str(historical["repo"]) not in json.dumps(proof)


def test_current_exact_hash_match_is_explicit_not_historical_execution(historical, monkeypatch):
    historical["bounds"] = base.source_bounds()
    monkeypatch.setattr(base, "_historical_bounds_bytes", lambda _: pytest.fail("current exact match must not require Git"))
    bounds, proof = resolve(historical)
    assert bounds == historical["bounds"]
    assert {item["proof_kind"] for item in proof["sources"].values()} == {"current_hash_match"}
    assert proof["historical_execution_proven_by_this_check"] is False


def test_archive_wins_over_missing_git_and_changed_live(historical, monkeypatch):
    archive(historical)
    monkeypatch.setattr(base, "_historical_bounds_bytes", lambda _: pytest.fail("archive must not fall back"))
    bounds, proof = resolve(historical)
    assert bounds == historical["bounds"] and proof["bounds_archive_available"] is True
    assert {item["proof_kind"] for item in proof["sources"].values()} == {"recorded_source_archive"}
    base.recheck_bounds_source_archive(historical["trial"], proof)
    put(historical["trial"] / "provenance" / base.BOUNDS_SOURCE_PATHS[0], b"changed")
    with pytest.raises(base.EvidenceError, match="changed during analysis"):
        base.recheck_bounds_source_archive(historical["trial"], proof)


@pytest.mark.parametrize("mutation", ["bytes", "missing", "symlink", "directory_symlink", "plan_sha", "flag_type"])
def test_any_malformed_archive_fails_without_historical_fallback(historical, monkeypatch, mutation):
    archive(historical)
    path = historical["trial"] / "provenance" / base.BOUNDS_SOURCE_PATHS[0]
    if mutation == "bytes":
        path.write_bytes(path.read_bytes() + b"\n")
    elif mutation == "missing":
        path.unlink()
    elif mutation == "symlink":
        path.unlink()
        path.symlink_to(historical["repo"] / base.BOUNDS_SOURCE_PATHS[0])
    elif mutation == "directory_symlink":
        directory = path.parent
        renamed = directory.with_name("redirected")
        directory.rename(renamed)
        directory.symlink_to(renamed, target_is_directory=True)
    elif mutation == "plan_sha":
        historical["plan"]["source_sha256"][base.BOUNDS_SOURCE_PATHS[0]] = "0" * 64
    else:
        historical["plan"]["bounds_source_bytes_archived"] = 1
    monkeypatch.setattr(base, "_historical_bounds_bytes", lambda _: pytest.fail("malformed archive must not fall back"))
    with pytest.raises(base.EvidenceError):
        resolve(historical)


def test_unflagged_present_but_partial_archive_still_fails(historical):
    name = base.BOUNDS_SOURCE_PATHS[0]
    put(historical["trial"] / "provenance" / name, historical["raw"][name])
    with pytest.raises(base.EvidenceError, match="missing or unsafe"):
        resolve(historical)


@pytest.mark.parametrize("kind", ["empty_directory", "dangling_directory_symlink"])
def test_empty_or_dangling_bounds_archive_never_falls_back(historical, kind):
    path = historical["trial"] / "provenance/portable_e2e"
    path.parent.mkdir()
    if kind == "empty_directory":
        path.mkdir()
    else:
        path.symlink_to(path.parent / "missing", target_is_directory=True)
    with pytest.raises(base.EvidenceError, match="missing or unsafe"):
        resolve(historical)


def test_git_blob_reads_disable_lazy_fetch_and_all_transport_protocols(historical, monkeypatch):
    original = subprocess.run
    calls = []
    def guarded(*args, **kwargs):
        calls.append(args[0])
        assert kwargs["env"]["GIT_NO_LAZY_FETCH"] == "1"
        assert kwargs["env"]["GIT_ALLOW_PROTOCOL"] == ""
        assert kwargs["env"]["GIT_TERMINAL_PROMPT"] == "0"
        assert args[0][:3] == ["git", "-c", "protocol.allow=never"]
        assert "fetch" not in args[0]
        return original(*args, **kwargs)
    monkeypatch.setattr(base.subprocess, "run", guarded)
    resolve(historical)
    assert len(calls) == 3


@pytest.mark.parametrize("mutation", ["recorded_sha", "recorded_path", "recorded_constant", "source_constant", "duplicate_constant", "invalid_ast"])
def test_wrong_hash_path_or_relaxed_constants_are_rejected(historical, mutation):
    if mutation == "recorded_sha":
        historical["bounds"]["source_sha256"][base.BOUNDS_SOURCE_PATHS[0]] = "f" * 64
    elif mutation == "recorded_path":
        historical["bounds"]["source_sha256"]["../model.py"] = historical["bounds"]["source_sha256"].pop(base.BOUNDS_SOURCE_PATHS[0])
    elif mutation == "recorded_constant":
        historical["bounds"]["physical_decoder"]["maximum_acceleration_mps2"] = 3.0
    else:
        name = base.BOUNDS_SOURCE_PATHS[0]
        raw = {"source_constant": b"PHYSICAL_MAXIMUM_ACCELERATION_MPS2 = 3.0\n",
               "duplicate_constant": historical["raw"][name] * 2,
               "invalid_ast": b"PHYSICAL_MAXIMUM_ACCELERATION_MPS2 = float('nan')\n"}[mutation]
        historical["raw"][name] = raw
        historical["bounds"]["source_sha256"][name] = hashlib.sha256(raw).hexdigest()
        archive(historical)
    with pytest.raises(base.EvidenceError):
        resolve(historical)


@pytest.mark.parametrize("commit", ["", "HEAD", "-" * 40, "0" * 40, None])
def test_invalid_or_unavailable_history_fails_clearly_without_fetch(historical, commit):
    historical["plan"]["source_head_commit"] = commit
    with pytest.raises(base.EvidenceError, match="invalid bounds source_head_commit|no fetch was attempted"):
        resolve(historical)


def test_valid_but_wrong_historical_commit_does_not_fall_back(historical):
    git(historical["repo"], "add", "portable_e2e/model.py")
    git(historical["repo"], "-c", "user.name=Bounds Test", "-c", "user.email=bounds@example.invalid", "commit", "--quiet", "-m", "Different bytes")
    historical["plan"]["source_head_commit"] = git(historical["repo"], "rev-parse", "HEAD")
    with pytest.raises(base.EvidenceError, match="do not match recorded SHA256"):
        resolve(historical)


def test_legacy_reference_requires_explicit_opt_in_exact_digest_and_initial_trial(historical, monkeypatch):
    historical["plan"] = None
    monkeypatch.setattr(base, "REVIEWED_LEGACY_BOUNDS_COMMIT", historical["commit"])
    monkeypatch.setattr(base, "REVIEWED_LEGACY_BOUNDS_SHA256", copy.deepcopy(historical["bounds"]["source_sha256"]))
    with pytest.raises(base.EvidenceError, match="no reviewed"):
        resolve(historical)
    _, proof = resolve(historical, allow_reviewed_legacy=True)
    assert {item["proof_kind"] for item in proof["sources"].values()} == {"reviewed_initial_trial_git_blob_hash_match"}
    historical["trial"] = historical["trial"].with_name("run_002")
    historical["trial"].mkdir()
    with pytest.raises(base.EvidenceError, match="no reviewed"):
        resolve(historical, allow_reviewed_legacy=True)


@pytest.mark.parametrize("mutation", ["extra_path", "text_not_bytes", "runtime_bool", "runtime_limit", "duplicate_gate"])
def test_explicit_source_ast_input_remains_strict(historical, mutation):
    raw = historical["raw"]
    if mutation == "extra_path":
        raw["portable_e2e/extra.py"] = b"pass\n"
    elif mutation == "text_not_bytes":
        raw[base.BOUNDS_SOURCE_PATHS[0]] = raw[base.BOUNDS_SOURCE_PATHS[0]].decode()
    else:
        name = base.BOUNDS_SOURCE_PATHS[1]
        if mutation == "runtime_bool":
            raw[name] = raw[name].replace(b"3.0", b"True")
        elif mutation == "runtime_limit":
            raw[name] = raw[name].replace(b"6.0", b"7.0")
        else:
            raw[name] *= 2
    with pytest.raises(base.EvidenceError):
        base.source_bounds(raw)


@pytest.fixture
def actual_v2_new_archive(tmp_path):
    # HH_260906 - Upgrade only a temporary copy's provenance contract; original measured data remains untouched.
    source = Path(__file__).resolve().parents[1] / "artifacts/training/2026-09-08/low_speed_response_v2/run_001"
    if not (source / "owner_result.json").is_file():
        pytest.skip("optional local archived second calibration is absent")
    trial = tmp_path / "run_001"
    shutil.copytree(source, trial)
    plan = json.loads((trial / "owner_plan.json").read_text())
    owner = json.loads((trial / "owner_result.json").read_text())
    plan["bounds_source_bytes_archived"] = True
    for name, raw in base._historical_bounds_bytes(plan["source_head_commit"]).items():
        put(trial / "provenance" / name, raw)
        plan["source_sha256"][name] = hashlib.sha256(raw).hexdigest()
        owner["source_checks"][name] = True
    (trial / "owner_plan.json").write_text(json.dumps(plan))
    (trial / "owner_result.json").write_text(json.dumps(owner))
    return trial


def test_new_twelve_source_archive_preserves_v2_measurements_and_separate_bounds_proof(actual_v2_new_archive):
    result, _ = pedal.verify_trial(actual_v2_new_archive)
    assert len(result["executed_sources"]["source_sha256"]) == 12
    assert result["bounds_source_proof"]["bounds_archive_available"] is True
    assert sum(case["state_count"] for case in result["cases"]) == 2742
    assert not any(item["path"].startswith("provenance/portable_e2e/") for item in result["source_manifest"])


@pytest.mark.parametrize("mutation", ["unflagged", "missing_helper", "extra_pin", "extra_archive", "corrupt_bounds"])
def test_new_archive_does_not_weaken_exact_source_denominators(actual_v2_new_archive, mutation):
    trial = actual_v2_new_archive
    plan = json.loads((trial / "owner_plan.json").read_text())
    if mutation == "unflagged":
        plan.pop("bounds_source_bytes_archived")
    elif mutation == "missing_helper":
        plan["source_sha256"].pop("scripts/e2e/carla_low_speed_response_matrix.py")
    elif mutation == "extra_pin":
        plan["source_sha256"]["other.py"] = "0" * 64
    elif mutation == "extra_archive":
        put(trial / "provenance/other.py", b"pass\n")
    else:
        put(trial / "provenance" / base.BOUNDS_SOURCE_PATHS[0], b"changed")
    (trial / "owner_plan.json").write_text(json.dumps(plan))
    with pytest.raises(base.EvidenceError):
        pedal.verify_trial(trial)


def test_flagged_original_matrix_also_requires_twelve_sources_and_matrix_helper(tmp_path):
    # HH_260906 - This synthetic provenance upgrade is not a claim that the original trial used a newer wrapper.
    repo = Path(__file__).resolve().parents[1]
    source = repo / "artifacts/training/2026-09-08/low_speed_response_v1/run_001"
    if not (source / "owner_result.json").is_file():
        pytest.skip("optional local first calibration is absent")
    trial = tmp_path / "run_001"
    shutil.copytree(source, trial)
    plan = json.loads((trial / "owner_plan.json").read_text())
    owner = json.loads((trial / "owner_result.json").read_text())
    manifest_path = trial / "actuation/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    archived = base._historical_bounds_bytes(plan["source_head_commit"])
    helper = "scripts/e2e/carla_low_speed_response_matrix.py"
    archived[helper] = (repo / helper).read_bytes()
    plan["bounds_source_bytes_archived"] = True
    for name, raw in archived.items():
        put(trial / "provenance" / name, raw)
        plan["source_sha256"][name] = hashlib.sha256(raw).hexdigest()
        owner["source_checks"][name] = True
    manifest["source_sha256"][Path(helper).name] = plan["source_sha256"][helper]
    manifest_path.write_text(json.dumps(manifest))
    (trial / "owner_plan.json").write_text(json.dumps(plan))
    (trial / "owner_result.json").write_text(json.dumps(owner))
    result, _ = pedal.verify_trial(trial)
    assert result["planned_and_included_case_count"] == 12
    assert len(result["executed_sources"]["source_sha256"]) == 12
    assert result["bounds_source_proof"]["bounds_archive_available"] is True
    manifest["source_sha256"].pop(Path(helper).name)
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(base.EvidenceError, match="calibrator source denominator"):
        pedal.verify_trial(trial)
