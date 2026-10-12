"""HH_261007 - Check metadata-only thesis catalog without inspecting real evidence or starting external tools."""
from pathlib import Path

from scripts.maintenance import catalog_thesis_4pc_v1_20261007 as m


def test_file_catalog_preserves_bytes_and_generation(tmp_path):
    path = tmp_path / "raw.db3"
    path.write_bytes(b"test-not-a-real-bag")
    before = path.stat()
    value = m.inventory(tmp_path)
    assert value["file_count"] == 1
    assert value["logical_bytes"] == len(b"test-not-a-real-bag")
    assert value["files"][0]["generation_stable"]
    assert path.read_bytes() == b"test-not-a-real-bag"
    assert m.generation(before) == m.generation(path.stat())


def test_symlink_and_private_git_content_are_not_followed(tmp_path):
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir(); (elsewhere / "raw").write_bytes(b"outside")
    root = tmp_path / "evidence"; root.mkdir()
    (root / "link").symlink_to(elsewhere, target_is_directory=True)
    (root / ".git").mkdir(); (root / ".git/config").write_bytes(b"private")
    value = m.inventory(root)
    assert value["file_count"] == 0 and value["symlinks"] == [str(root / "link")]


def test_missing_root_is_not_silently_empty(tmp_path):
    value = m.inventory(tmp_path / "missing")
    assert value["exists"] is False and value["errors"]


def test_fixed_roots_do_not_catalog_credentials_or_models():
    assert all(path.name not in {".ssh", ".codex", ".cache"} for path in m.SOURCES.values())
    assert m.OUTPUT == m.ROOT / "storage/experiments/maintenance/2026-10-07/thesis_4pc_catalog_v1"
    assert m.MAX_OUTPUT_BYTES == 8 * 1024**2
