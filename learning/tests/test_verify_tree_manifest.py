from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import pytest

from scripts.e2e import verify_tree_manifest as verifier
from scripts.e2e.verify_tree_manifest import ManifestLimits, build_tree_manifest


def _write_tree(root: Path, files: dict[str, bytes]) -> None:
    root.mkdir()
    for relative_path, payload in files.items():
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def test_manifest_is_sorted_and_fingerprint_is_reproducible(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    _write_tree(first, {"z.bin": b"last", "nested/a.bin": b"first"})
    _write_tree(second, {"nested/a.bin": b"first", "z.bin": b"last"})

    first_report = build_tree_manifest(first)
    second_report = build_tree_manifest(second)

    assert first_report["valid"] is True
    assert [entry["path"] for entry in first_report["files"]] == [
        "nested/a.bin",
        "z.bin",
    ]
    assert first_report["manifest_sha256"] == second_report["manifest_sha256"]
    assert first_report["file_count"] == 2
    assert first_report["total_size_bytes"] == len(b"firstlast")
    assert first_report["files"][0] == {
        "path": "nested/a.bin",
        "size": 5,
        "sha256": hashlib.sha256(b"first").hexdigest(),
    }


def test_exact_source_destination_comparison_passes(tmp_path: Path) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    files = {"a.txt": b"alpha", "nested/b.bin": b"beta"}
    _write_tree(source, files)
    _write_tree(destination, files)

    report = verifier.verify_tree_manifest(source, destination)

    assert report["valid"] is True
    assert report["status"] == "PASS"
    assert report["comparison"]["status"] == "PASS"
    assert report["comparison"]["matching"] is True
    assert report["source"]["manifest_sha256"] == report["destination"][
        "manifest_sha256"
    ]


def test_comparison_reports_missing_extra_and_different_files(tmp_path: Path) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    _write_tree(
        source,
        {"same.bin": b"same", "missing.bin": b"missing", "changed.bin": b"old"},
    )
    _write_tree(
        destination,
        {"same.bin": b"same", "extra.bin": b"extra", "changed.bin": b"new"},
    )

    report = verifier.verify_tree_manifest(source, destination)
    comparison = report["comparison"]

    assert report["valid"] is False
    assert comparison["status"] == "FAIL"
    assert comparison["missing_files"] == ["missing.bin"]
    assert comparison["extra_files"] == ["extra.bin"]
    assert comparison["different_file_count"] == 1
    assert comparison["different_files"][0]["path"] == "changed.bin"
    assert comparison["different_files"][0]["source_size"] == 3
    assert comparison["different_files"][0]["destination_size"] == 3


def test_root_symlink_regular_file_and_missing_path_fail_closed(tmp_path: Path) -> None:
    real = tmp_path / "real"
    _write_tree(real, {"data.bin": b"payload"})
    link = tmp_path / "link"
    link.symlink_to(real, target_is_directory=True)
    regular = tmp_path / "regular.bin"
    regular.write_bytes(b"payload")

    link_report = build_tree_manifest(link)
    regular_report = build_tree_manifest(regular)
    missing_report = build_tree_manifest(tmp_path / "missing")

    assert link_report["errors"] == ["tree root must not be a symbolic link"]
    assert regular_report["errors"] == ["tree root must be a directory"]
    assert missing_report["valid"] is False
    assert missing_report["errors"][0].startswith("cannot inspect tree root:")


def test_embedded_nul_root_returns_json_failure_instead_of_traceback() -> None:
    report = build_tree_manifest("invalid\0root")

    assert report["valid"] is False
    assert report["status"] == "FAIL"
    assert report["errors"][0].startswith("cannot inspect tree root:")
    json.dumps(report, allow_nan=False)


def test_symlink_and_fifo_inside_tree_are_rejected(tmp_path: Path) -> None:
    target = tmp_path / "target.bin"
    target.write_bytes(b"target")
    linked_tree = tmp_path / "linked"
    linked_tree.mkdir()
    (linked_tree / "link.bin").symlink_to(target)
    fifo_tree = tmp_path / "fifo"
    fifo_tree.mkdir()
    os.mkfifo(fifo_tree / "pipe")

    linked_report = build_tree_manifest(linked_tree)
    fifo_report = build_tree_manifest(fifo_tree)

    assert linked_report["valid"] is False
    assert "forbidden symbolic link" in linked_report["errors"][0]
    assert fifo_report["valid"] is False
    assert "forbidden FIFO" in fifo_report["errors"][0]


@pytest.mark.parametrize("name", ["bad\\name.bin", "bad\nname.bin", "e\u0301.bin"])
def test_surprising_or_non_normalized_names_are_rejected(
    tmp_path: Path, name: str
) -> None:
    tree = tmp_path / "tree"
    _write_tree(tree, {name: b"payload"})

    report = build_tree_manifest(tree)

    assert report["valid"] is False
    assert "path component" in report["errors"][0]


def test_file_path_replacement_during_hash_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    tree = tmp_path / "tree"
    _write_tree(tree, {"data.bin": b"original"})
    replacement = tmp_path / "replacement.bin"
    replacement.write_bytes(b"replacement")
    original_hash = verifier._hash_open_file

    def hash_then_replace(file_descriptor: int) -> str:
        digest = original_hash(file_descriptor)
        os.replace(replacement, tree / "data.bin")
        return digest

    monkeypatch.setattr(verifier, "_hash_open_file", hash_then_replace)
    report = build_tree_manifest(tree)

    assert report["valid"] is False
    assert any(
        "changed while it was hashed" in error
        or "different object after hashing" in error
        for error in report["errors"]
    )


def test_safe_opens_request_nofollow_when_supported(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    tree = tmp_path / "tree"
    _write_tree(tree, {"nested/data.bin": b"payload"})
    original_open = verifier.os.open
    flags_seen: list[int] = []

    def recording_open(
        path: os.PathLike[str] | str,
        flags: int,
        *args: object,
        **kwargs: object,
    ) -> int:
        flags_seen.append(flags)
        return original_open(path, flags, *args, **kwargs)

    monkeypatch.setattr(verifier.os, "open", recording_open)
    report = build_tree_manifest(tree)

    assert report["valid"] is True
    assert len(flags_seen) >= 3
    if hasattr(os, "O_NOFOLLOW"):
        assert all(flags & os.O_NOFOLLOW for flags in flags_seen)


def test_invalid_limits_fail_as_json_report(tmp_path: Path) -> None:
    tree = tmp_path / "tree"
    _write_tree(tree, {"data.bin": b"payload"})

    report = build_tree_manifest(tree, limits=ManifestLimits(max_files=0))

    assert report["valid"] is False
    assert report["errors"] == ["max_files must be a positive integer"]
    json.dumps(report, allow_nan=False)


def test_scandir_stops_streaming_as_soon_as_file_limit_is_exceeded(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    tree = tmp_path / "tree"
    _write_tree(
        tree,
        {
            "one.bin": b"one",
            "two.bin": b"two",
            "three.bin": b"three",
            "four.bin": b"four",
        },
    )
    original_scandir = verifier.os.scandir
    yielded: list[str] = []

    class CountingIterator:
        def __init__(self, inner: object) -> None:
            self.inner = inner

        def __enter__(self) -> "CountingIterator":
            return self

        def __exit__(self, *args: object) -> None:
            self.inner.close()  # type: ignore[attr-defined]

        def __iter__(self) -> "CountingIterator":
            return self

        def __next__(self) -> os.DirEntry[str]:
            entry = next(self.inner)  # type: ignore[arg-type]
            yielded.append(entry.name)
            return entry

    def counting_scandir(file_descriptor: int) -> CountingIterator:
        return CountingIterator(original_scandir(file_descriptor))

    monkeypatch.setattr(verifier.os, "scandir", counting_scandir)
    report = build_tree_manifest(tree, limits=ManifestLimits(max_files=1))

    assert report["valid"] is False
    assert any("file count exceeds" in error for error in report["errors"])
    assert len(yielded) == 2


def test_deep_tree_stops_at_configured_depth_and_cli_returns_json(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    tree = tmp_path / "tree"
    tree.mkdir()
    current = tree
    for index in range(10):
        current = current / f"d{index}"
        current.mkdir()
    (current / "payload.bin").write_bytes(b"payload")

    status = verifier.main([str(tree), "--max-depth", "8"])
    report = json.loads(capsys.readouterr().out)

    assert status == 2
    assert report["status"] == "FAIL"
    assert report["source"]["limits"]["max_depth"] == 8
    assert any("directory depth 9 exceeds" in error for error in report["errors"])


def test_depth_above_hard_safety_ceiling_is_rejected_without_traversal(
    tmp_path: Path
) -> None:
    tree = tmp_path / "tree"
    _write_tree(tree, {"payload.bin": b"payload"})

    report = build_tree_manifest(
        tree,
        limits=ManifestLimits(max_depth=verifier.MAX_SAFE_DEPTH + 1),
    )

    assert report["valid"] is False
    assert report["file_count"] == 0
    assert report["errors"] == [
        f"max_depth must not exceed safe limit {verifier.MAX_SAFE_DEPTH}"
    ]


def test_single_tree_cli_emits_json_and_comparison_exit_statuses(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    _write_tree(source, {"data.bin": b"payload"})
    _write_tree(destination, {"data.bin": b"payload"})

    single_status = verifier.main([str(source)])
    single_report = json.loads(capsys.readouterr().out)
    matching_status = verifier.main([str(source), str(destination)])
    matching_report = json.loads(capsys.readouterr().out)
    (destination / "data.bin").write_bytes(b"changed")
    mismatch_status = verifier.main([str(source), str(destination)])
    mismatch_report = json.loads(capsys.readouterr().out)

    assert single_status == 0
    assert single_report["comparison"]["status"] == "NOT_REQUESTED"
    assert matching_status == 0
    assert matching_report["status"] == "PASS"
    assert mismatch_status == 2
    assert mismatch_report["status"] == "FAIL"


def test_inventory_does_not_modify_file_content_or_metadata(tmp_path: Path) -> None:
    tree = tmp_path / "tree"
    _write_tree(tree, {"data.bin": b"payload"})
    path = tree / "data.bin"
    before = os.stat(path)
    before_payload = path.read_bytes()

    report = build_tree_manifest(tree)
    after = os.stat(path)

    assert report["valid"] is True
    assert path.read_bytes() == before_payload
    assert (after.st_size, after.st_mtime_ns, after.st_ctime_ns) == (
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    )
