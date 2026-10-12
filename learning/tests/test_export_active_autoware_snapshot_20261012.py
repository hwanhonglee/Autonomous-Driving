"""HH_261012 - Verify source publication using disposable repositories only."""
from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "hh_active_autoware_export", ROOT / "tools/maintenance/export_active_autoware_snapshot_20261012.py")
assert SPEC and SPEC.loader
EXPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORT)


def git(repo: Path, *args: str, input_bytes: bytes | None = None) -> bytes:
    env = dict(os.environ)
    for key in ("GIT_INDEX_FILE", "GIT_DIR", "GIT_WORK_TREE", "GIT_COMMON_DIR"):
        env.pop(key, None)
    env.update({"GIT_CONFIG_NOSYSTEM": "1", "GIT_CONFIG_GLOBAL": "/dev/null"})
    run = subprocess.run(["git", "-C", str(repo), *args], input=input_bytes,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env, check=False)
    assert run.returncode == 0, run.stderr.decode()
    return run.stdout


def fixture(tmp_path: Path, name: str = "core/tiny") -> tuple[Path, Path, Path]:
    workspace = tmp_path / "workspace"
    repo = workspace / "autonomy/src" / name
    repo.mkdir(parents=True)
    git(repo, "init", "-q")
    git(repo, "config", "user.email", "test@example.invalid")
    git(repo, "config", "user.name", "Disposable test fixture")
    git(repo, "remote", "add", "origin", "https://github.com/example/disposable.git")
    (repo / "source.py").write_text("# Base fixture\nanswer = 1\n", encoding="utf-8")
    (repo / "deleted.cpp").write_text("int base = 1;\n", encoding="utf-8")
    git(repo, "add", ".")
    git(repo, "commit", "-q", "-m", "Disposable base")
    output = workspace / "autonomy/source_snapshots/fixture_v1"
    return workspace, repo, output


def test_complete_patch_replays_staged_unstaged_deleted_and_new_source(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    base = git(repo, "rev-parse", "HEAD").decode().strip()
    (repo / "source.py").write_text("# Staged fixture\nanswer = 2\n", encoding="utf-8")
    git(repo, "add", "source.py")
    (repo / "source.py").write_text("# Current fixture\nanswer = 3\n", encoding="utf-8")
    (repo / "new.hpp").write_text("// New fixture\nconstexpr int answer = 3;\n", encoding="utf-8")
    (repo / "deleted.cpp").unlink()
    before_index = (repo / ".git/index").read_bytes()
    before_status = git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all")
    result = EXPORT.export_snapshot(workspace, output)
    assert result["repository_count"] == result["dirty_repository_count"] == 1
    assert result["untracked_source_count"] == 1
    assert result["tracked_status_count"] == 2
    assert (repo / ".git/index").read_bytes() == before_index
    assert git(repo, "status", "--porcelain=v1", "-z", "--untracked-files=all") == before_status
    assert git(repo, "rev-parse", "HEAD").decode().strip() == base
    clone = tmp_path / "disposable_replay"
    git(tmp_path, "clone", "--no-hardlinks", "-q", str(repo), str(clone))
    patch = output / result["repositories"][0]["patch"]
    git(clone, "apply", "--check", str(patch))
    git(clone, "apply", str(patch))
    assert (clone / "source.py").read_bytes() == (repo / "source.py").read_bytes()
    assert (clone / "new.hpp").read_bytes() == (repo / "new.hpp").read_bytes()
    assert not (clone / "deleted.cpp").exists()
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["source_before_after_equal"]
    assert manifest["original_git_index_unchanged"]
    assert not manifest["real_vehicle_actuation_authorized"]
    for line in (output / "SHA256SUMS").read_text().splitlines():
        sha, name = line.split("  ", 1)
        assert EXPORT.digest((output / name).read_bytes()) == sha
    assert base in (output / "exact.repos").read_text()


def test_existing_output_is_never_overwritten(tmp_path: Path) -> None:
    workspace, _, output = fixture(tmp_path)
    output.mkdir(parents=True)
    witness = output / "preserve.txt"
    witness.write_text("keep\n")
    with pytest.raises(EXPORT.SnapshotError, match="Existing"):
        EXPORT.export_snapshot(workspace, output)
    assert witness.read_text() == "keep\n"


@pytest.mark.parametrize("name,value,error", [
    ("new.npz", b"bytes", "Unreviewed"),
    ("new.py", b"binary\0payload", "binary"),
    ("new.py", b"\xff", "UTF8"),
    ("private_key.py", b"text", "Private filename"),
    ("new.py", b"-----BEGIN OPENSSH PRIVATE KEY-----\n", "credential marker"),
    ("new.py", b"api_key = 'ghp_" + b"a" * 30 + b"'\n", "credential marker"),
    ("new.py", b"password = 'actual-not-a-placeholder'\n", "credential literal"),
])
def test_untracked_publication_rejections(tmp_path: Path, name: str, value: bytes, error: str) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / name).write_bytes(value)
    with pytest.raises(EXPORT.SnapshotError, match=error):
        EXPORT.export_snapshot(workspace, output)
    assert not output.exists()


def test_untracked_symlink_is_rejected(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / "new.py").symlink_to(repo / "source.py")
    with pytest.raises(EXPORT.SnapshotError, match="Symlink"):
        EXPORT.export_snapshot(workspace, output)
    assert not output.exists()


@pytest.mark.parametrize("url", [
    "https://ghp_hidden@github.com/example/repo.git",
    "https://user:password@github.com/example/repo.git",
    "https://github.com/example/repo.git?token=hidden",
    "ssh://user:password@github.com/example/repo.git",
    "file:///private/repository",
])
def test_credential_or_unsupported_remote_is_rejected(tmp_path: Path, url: str) -> None:
    workspace, repo, output = fixture(tmp_path)
    git(repo, "remote", "set-url", "origin", url)
    with pytest.raises(EXPORT.SnapshotError, match="remote URL"):
        EXPORT.export_snapshot(workspace, output)
    assert not output.exists()


def test_clean_repository_is_declared_without_patch(tmp_path: Path) -> None:
    workspace, _, output = fixture(tmp_path)
    result = EXPORT.export_snapshot(workspace, output)
    assert result["repository_count"] == 1
    assert result["dirty_repository_count"] == result["untracked_source_count"] == 0
    assert result["repositories"][0]["patch"] is None
    assert result["repositories"][0]["cached_patch_check"] == "NOT_NEEDED_CLEAN_REPOSITORY"


@pytest.mark.parametrize("relative", ["../outside", ".git/output", "storage/experiments/test",
                                     "autonomy/src/new_output", "autonomy/install/output"])
def test_output_scope_is_rejected(tmp_path: Path, relative: str) -> None:
    workspace, _, _ = fixture(tmp_path)
    with pytest.raises((EXPORT.SnapshotError, ValueError)):
        EXPORT.export_snapshot(workspace, workspace / relative)


def test_source_change_between_captures_refuses_publication(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    workspace, repo, output = fixture(tmp_path)
    original = EXPORT.capture_repository
    calls = 0

    def mutate(current: Path, source_root: Path):
        nonlocal calls
        calls += 1
        if calls == 2:
            (repo / "source.py").write_text("# Concurrent change\nanswer = 99\n")
        return original(current, source_root)

    monkeypatch.setattr(EXPORT, "capture_repository", mutate)
    with pytest.raises(EXPORT.SnapshotError, match="changed before"):
        EXPORT.export_snapshot(workspace, output)
    assert not output.exists()


def test_staged_rename_is_rejected(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    git(repo, "mv", "source.py", "renamed.py")
    with pytest.raises(EXPORT.SnapshotError, match="Rename"):
        EXPORT.export_snapshot(workspace, output)
    assert not output.exists()


def test_saved_byte_verification_does_not_require_clean_live_repository(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / "new.py").write_text("# New source\nanswer = 4\n")
    result = EXPORT.export_snapshot(workspace, output)
    verified = EXPORT.verify_bundle(workspace, output)
    assert verified["status"] == "VERIFIED_SAVED_BYTES_ONLY"
    assert verified["checked_files"] == 3
    assert verified["source_apply_count"] == 0
    assert result["dirty_repository_count"] == 1


def test_clean_base_patch_check_has_no_source_apply(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / "new.py").write_text("# New source\nanswer = 4\n")
    EXPORT.export_snapshot(workspace, output)
    (repo / "new.py").unlink()
    before_index = (repo / ".git/index").read_bytes()
    result = EXPORT.verify_bundle(workspace, output, check_clean_bases=True)
    assert result["status"] == "VERIFIED_SAVED_BYTES_AND_CLEAN_BASE_CHECK"
    assert result["source_apply_count"] == 0
    assert not (repo / "new.py").exists()
    assert (repo / ".git/index").read_bytes() == before_index
    assert not git(repo, "status", "--porcelain=v1")


def test_dirty_replay_base_is_rejected(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / "new.py").write_text("# New source\nanswer = 4\n")
    EXPORT.export_snapshot(workspace, output)
    with pytest.raises(EXPORT.SnapshotError, match="base is dirty"):
        EXPORT.verify_bundle(workspace, output, check_clean_bases=True)


def test_wrong_replay_base_head_is_rejected(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    EXPORT.export_snapshot(workspace, output)
    git(repo, "commit", "--allow-empty", "-q", "-m", "Different disposable HEAD")
    with pytest.raises(EXPORT.SnapshotError, match="HEAD differs"):
        EXPORT.verify_bundle(workspace, output, check_clean_bases=True)


def test_saved_patch_corruption_is_rejected(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / "new.py").write_text("# New source\nanswer = 4\n")
    result = EXPORT.export_snapshot(workspace, output)
    patch = output / result["repositories"][0]["patch"]
    patch.write_bytes(patch.read_bytes() + b"\n# Changed bytes\n")
    with pytest.raises(EXPORT.SnapshotError, match="checksum mismatch"):
        EXPORT.verify_bundle(workspace, output)


def test_unlisted_bundle_file_is_rejected(tmp_path: Path) -> None:
    workspace, _, output = fixture(tmp_path)
    EXPORT.export_snapshot(workspace, output)
    (output / "unexpected.py").write_text("# Not declared\n")
    with pytest.raises(EXPORT.SnapshotError, match="Unexpected or missing"):
        EXPORT.verify_bundle(workspace, output)


def test_bundle_symlink_is_rejected(tmp_path: Path) -> None:
    workspace, _, output = fixture(tmp_path)
    EXPORT.export_snapshot(workspace, output)
    (output / "linked.txt").symlink_to(output / "exact.repos")
    with pytest.raises(EXPORT.SnapshotError, match="Symlink"):
        EXPORT.verify_bundle(workspace, output)


def test_script_check_only_has_no_apply_argument() -> None:
    source = (ROOT / "tools/maintenance/export_active_autoware_snapshot_20261012.py").read_text()
    assert 'parser.add_argument("--check-only"' in source
    assert 'parser.add_argument("--verify"' in source
    assert 'parser.add_argument("--apply"' not in source


def test_oversized_source_is_rejected(tmp_path: Path) -> None:
    workspace, repo, output = fixture(tmp_path)
    (repo / "oversized.py").write_bytes(b"#" * (EXPORT.MAX_SOURCE_BYTES + 1))
    with pytest.raises(EXPORT.SnapshotError, match="Oversized"):
        EXPORT.export_snapshot(workspace, output)


def test_source_root_symlink_is_rejected(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    (workspace / "autonomy").mkdir(parents=True)
    actual = tmp_path / "external_source"
    actual.mkdir()
    (workspace / "autonomy/src").symlink_to(actual, target_is_directory=True)
    with pytest.raises(EXPORT.SnapshotError, match="Symlink"):
        EXPORT.export_snapshot(workspace, workspace / "autonomy/source_snapshots/v1")
