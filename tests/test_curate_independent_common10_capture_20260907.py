"""HH_260906 - Verify create-only publication without touching captured evidence."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts/e2e/curate_independent_common10_capture_20260907.py"
SPEC = importlib.util.spec_from_file_location("independent_common10_curator", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
curator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(curator)


@pytest.mark.parametrize("kind", ["json", "text"])
def test_publication_writers_create_once_without_overwriting(tmp_path, kind):
    destination = tmp_path / f"report.{kind}"
    if kind == "json":
        curator.write_json(destination, {"status": "PASS", "samples": 309})
        assert json.loads(destination.read_text()) == {"status": "PASS", "samples": 309}
        replacement = {"status": "FAIL"}
        writer = curator.write_json
    else:
        curator.write_text(destination, "original evidence\n")
        assert destination.read_text() == "original evidence\n"
        replacement = "replacement evidence\n"
        writer = curator.write_text
    original = destination.read_bytes()

    with pytest.raises(FileExistsError):
        writer(destination, replacement)

    assert destination.read_bytes() == original


def test_copy_exact_preserves_binary_bytes_and_streamed_sha256(tmp_path):
    # HH_260906 - Cross the one-MiB boundary used by the streaming copy and hash helpers.
    payload = bytes(range(256)) * 4097 + b"\x00\xff\r\nfinal-frame"
    source = tmp_path / "source.gif"
    destination = tmp_path / "published.gif"
    source.write_bytes(payload)

    curator.copy_exact(source, destination)

    assert destination.read_bytes() == payload
    assert source.read_bytes() == payload
    assert curator.sha256(destination) == hashlib.sha256(payload).hexdigest()
    with pytest.raises(FileExistsError):
        curator.copy_exact(source, destination)
    assert destination.read_bytes() == payload


def test_copy_exact_rejects_a_corrupted_copy(tmp_path, monkeypatch):
    source = tmp_path / "source.png"
    destination = tmp_path / "published.png"
    source.write_bytes(b"original captured frame")

    def corrupt_copy(incoming, outgoing, length):
        # HH_260906 - Simulate a truncated copy to exercise the independent digest check.
        outgoing.write(incoming.read(4))

    monkeypatch.setattr(curator.shutil, "copyfileobj", corrupt_copy)

    with pytest.raises(RuntimeError, match="Copy digest mismatch"):
        curator.copy_exact(source, destination)

    assert source.read_bytes() == b"original captured frame"


@pytest.mark.parametrize("account,owner", [("fixture_local", "fixture_owner"), ("another-user", "team_member")])
def test_sanitize_replaces_nested_roots_without_mutating_the_input(account, owner):
    payload = {
        "route": f"/home/{account}/autoware_e2e/artifacts/route.json",
        "nested": [
            f"/home/{account}/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla",
            {"dataset": f"/home/{account}/personal/{owner}/dataset/prepared/corpus"},
            {"venv": f"/home/{account}/personal/{owner}/portable_e2e/venvs/py312"},
            None,
            False,
            309,
        ],
        "unrelated": "/data/another_project/evidence.json",
    }
    original = copy.deepcopy(payload)

    actual = curator.sanitize(payload)

    assert actual == {
        "route": "${REPO_ROOT}/artifacts/route.json",
        "nested": [
            "${CARLA_ROOT}/PythonAPI/carla",
            {"dataset": "${PERSONAL_DATASET_ROOT}/prepared/corpus"},
            {"venv": "${PORTABLE_E2E_ROOT}/venvs/py312"},
            None,
            False,
            309,
        ],
        "unrelated": "/data/another_project/evidence.json",
    }
    assert payload == original


def test_sanitize_covers_command_boundaries_dictionary_keys_and_unknown_home_paths():
    payload = {
        "/home/fixture_user/artifacts/private.json": (
            "cd '/home/fixture_user/personal/fixture_owner/portable_e2e' && "
            'read "/home/fixture_user/autoware_e2e"'
        ),
        "similar_name": "/home/fixture_user/autoware_e2e_backup/report.json",
        "placeholder": "${PORTABLE_E2E_ROOT}/runs/result.json",
    }

    assert curator.sanitize(payload) == {
        "${USER_HOME}/artifacts/private.json": (
            "cd '${PORTABLE_E2E_ROOT}' && read \"${REPO_ROOT}\""
        ),
        "similar_name": "${USER_HOME}/autoware_e2e_backup/report.json",
        "placeholder": "${PORTABLE_E2E_ROOT}/runs/result.json",
    }


def test_source_view_binds_original_bytes_before_path_sanitization(tmp_path, monkeypatch):
    monkeypatch.setattr(curator, "REPO", tmp_path)
    source = tmp_path / "raw_metadata.json"
    original_bytes = (
        b'{ "root": "/home/fixture_user/autoware_e2e/datasets",\n'
        b'  "status": "PASS", "sample_count": 534 }\n'
    )
    source.write_bytes(original_bytes)

    view = curator.source_view(source)

    assert view["raw_source_path"] == "raw_metadata.json"
    assert view["raw_source_sha256"] == hashlib.sha256(original_bytes).hexdigest()
    assert view["record"] == {
        "root": "${REPO_ROOT}/datasets", "status": "PASS", "sample_count": 534
    }
    assert "not a standalone training dataset" in view["publication_notice"]
    assert source.read_bytes() == original_bytes
    assert view["raw_source_sha256"] != hashlib.sha256(
        json.dumps(view["record"]).encode()
    ).hexdigest()


@pytest.mark.parametrize(
    "category",
    [
        "02_new_training_town01_right",
        "03_held_out_test_town04_straight",
        "05_dataset_transfer",
    ],
)
@pytest.mark.parametrize("existing_type", ["directory", "file", "dangling_symlink"])
def test_main_refuses_any_existing_owned_category_before_writing(
    tmp_path, monkeypatch, category, existing_type
):
    root = tmp_path / "capture"
    root.mkdir()
    (root / "campaign_summary.json").write_text("{}\n")
    output = tmp_path / "publication"
    output.mkdir()
    target = output / category
    if existing_type == "directory":
        target.mkdir()
        (target / "keep.txt").write_bytes(b"existing evidence")
    elif existing_type == "file":
        target.write_bytes(b"existing evidence")
    else:
        target.symlink_to(tmp_path / "missing-target")
    monkeypatch.setattr(curator, "ROOT", root)
    monkeypatch.setattr(curator, "OUTPUT", output)

    def reject_copy(*args):
        pytest.fail("Copying must not begin after an existing-category conflict")

    monkeypatch.setattr(curator, "copy_exact", reject_copy)

    with pytest.raises(RuntimeError, match="Owned output categories must be entirely new"):
        curator.main()

    assert set(output.iterdir()) == {target}
    if existing_type == "directory":
        assert (target / "keep.txt").read_bytes() == b"existing evidence"
    elif existing_type == "file":
        assert target.read_bytes() == b"existing evidence"
    else:
        assert target.is_symlink()
        assert target.readlink() == tmp_path / "missing-target"
        assert not (tmp_path / "missing-target").exists()


def test_category_checksum_manifest_covers_payloads_and_is_create_only(tmp_path):
    nested = tmp_path / "nested"
    nested.mkdir()
    (tmp_path / "README.md").write_bytes(b"observed expert evidence\n")
    (nested / "frame.png").write_bytes(b"\x00captured bytes\xff")

    curator.finish_category(tmp_path)

    checksum_file = tmp_path / "SHA256SUMS"
    original = checksum_file.read_bytes()
    records = dict(line.split("  ", 1)[::-1] for line in original.decode().splitlines())
    assert set(records) == {"README.md", "nested/frame.png"}
    for path, digest in records.items():
        assert hashlib.sha256((tmp_path / path).read_bytes()).hexdigest() == digest
    (nested / "frame.png").write_bytes(b"altered bytes")
    assert hashlib.sha256((nested / "frame.png").read_bytes()).hexdigest() != records[
        "nested/frame.png"
    ]
    with pytest.raises(FileExistsError):
        curator.finish_category(tmp_path)
    assert checksum_file.read_bytes() == original


def test_capture_runner_is_valid_bash_without_executing_or_launching_carla():
    # HH_260906 - Bash syntax-only mode parses the runner without sourcing its environment.
    runner = REPO / "scripts/e2e/run_independent_common10_capture_20260907.sh"
    result = subprocess.run(
        ["bash", "--noprofile", "--norc", "-n", str(runner)],
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout == ""
