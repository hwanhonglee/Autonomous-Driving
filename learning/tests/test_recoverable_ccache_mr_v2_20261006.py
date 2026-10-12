# HH_261006 - Correct only canonical scripts namespace; preserve frozen V1 test and its first collection failure.
"""HH_261006 - Fake filesystem/authority/source regressions only; never operate on real ccache or SSH."""
import ast
from argparse import Namespace
import inspect
from pathlib import Path
import stat

import pytest

from scripts.maintenance import recoverable_ccache_mr_v1_20261006 as m

M = "a/b/" + "0" * 31 + "M"
R = "a/b/" + "1" * 31 + "R"


def fixture_cache(tmp_path):
    root = tmp_path / "fake_cache"
    (root / "a/b").mkdir(parents=True)
    for relative, payload in ((M, b"synthetic_manifest"), (R, b"synthetic_result"),
        ("a/CACHEDIR.TAG", b"tag"), ("a/stats", b"stats_a"), ("a/b/stats", b"stats_b")):
        (root / relative).write_bytes(payload)
    return root


@pytest.mark.parametrize("relative", [M, R, "0/f/" + "z" * 31 + "M"])
def test_scope_exact_current_shape(relative):
    assert m.scoped(relative)


@pytest.mark.parametrize("relative", ["a/b/stats", "a/CACHEDIR.TAG", "../" + M, "/" + M,
    "a/" + "0" * 31 + "M", "a/b/c/" + "0" * 31 + "R", "a/b/shortM",
    "g/b/" + "0" * 31 + "M", "a/b/" + "0" * 31 + "X", "a/b/" + "A" * 31 + "M",
    "CMakeFiles/f.o", "a//b/" + "0" * 31 + "M"])
def test_unknown_noncache_or_retained_file_never_scoped(relative):
    assert not m.scoped(relative)


def test_full_census_and_preserved_files(tmp_path):
    root = fixture_cache(tmp_path); result = m.census(root)
    assert set(result["objects"]) == {M, R}
    assert set(result["retained"]) == {"a/CACHEDIR.TAG", "a/stats", "a/b/stats"}
    assert set(result["directories"]) == {".", "a", "a/b"}
    assert result["object_count"] == 2 and result["logical_bytes"] == 34
    assert all(len(r["sha256"]) == 64 for r in result["objects"].values())
    assert m.preserved_audit(result)["no_unknown_or_object_reference"]


@pytest.mark.parametrize("fault", ["symlink", "hardlink", "unknown", "special", "directory", "owner"])
def test_census_denies_every_unsafe_entry(tmp_path, monkeypatch, fault):
    root = fixture_cache(tmp_path)
    if fault == "symlink": (root / "a/b/link").symlink_to(root / M)
    elif fault == "hardlink":
        import os
        os.link(root / M, root / (R + "bad"))
    elif fault == "unknown": (root / "a/b/notes.txt").write_text("not_cache")
    elif fault == "special":
        import os
        os.mkfifo(root / "a/b/fifo")
    elif fault == "directory": (root / "not_hex").mkdir()
    else: monkeypatch.setattr(m.base, "UID", m.base.UID + 100)
    with pytest.raises((ValueError, TypeError)):
        m.census(root)


def test_retained_stats_tag_or_added_consumer_change_denied(tmp_path):
    root = fixture_cache(tmp_path); manifest = m.census(root)
    (root / "a/b/stats").write_bytes(b"active_writer")
    with pytest.raises(ValueError, match="retained"):
        m.preserved_audit(manifest)


def test_additional_M_R_after_inspection_denied(tmp_path):
    root = fixture_cache(tmp_path); manifest = m.census(root)
    (root / ("a/b/" + "2" * 31 + "M")).write_bytes(b"new_compiler")
    with pytest.raises(ValueError, match="new or modified"):
        m.preserved_audit(manifest)


def test_original_per_file_fd_hash_intent_and_unlink_only_approved_M_R(tmp_path):
    root = fixture_cache(tmp_path); manifest = m.census(root); before=[]
    private = m.private_scope()
    receipt = private["verify_object_before_unlink"](M, manifest["objects"][M], manifest,
        do_unlink=True, before_unlink=before.append)
    assert receipt["removed"] is True and len(before) == 1 and before[0]["unlink_attempted"] is True
    assert not (root / M).exists() and (root / R).is_file() and (root / "a/b/stats").is_file()
    assert m.preserved_audit(manifest)["no_unknown_or_object_reference"]
    with pytest.raises(ValueError):
        private["opened_object"](root, "a/b/stats", manifest["directories"])


def test_replaced_inode_blocks_unlink_before_intent(tmp_path):
    root = fixture_cache(tmp_path); manifest = m.census(root); intentions=[]
    (root / M).unlink(); (root / M).write_bytes(b"replacement")
    with pytest.raises(ValueError, match="generation"):
        m.private_scope()["verify_object_before_unlink"](M, manifest["objects"][M], manifest,
            do_unlink=True, before_unlink=intentions.append)
    assert intentions == [] and (root / M).exists()


def test_private_scope_does_not_mutate_frozen_globals():
    before = (m.base.BUILD, m.base.scoped, m.base.process_audit, m.base.exact_system_service)
    main = m.prepared_main(); private = main.__globals__
    assert private["BUILD"] == m.CACHE and private["REMOTE"] == m.REMOTE
    assert private["SCHEMA"] == m.SCHEMA and private["AUTH_SCHEMA"] == m.AUTH_SCHEMA
    assert private["scoped"] is m.scoped and private["exact_system_service"] is m.known.exact_system_service
    assert {"ccache", "sccache", "distcc"} <= private["BUILD_TOOLS"]
    assert (m.base.BUILD, m.base.scoped, m.base.process_audit, m.base.exact_system_service) == before


def test_main_inverse_archive_seam_and_remote_scoped_source():
    original = inspect.getsource(m.base.main)
    old = "for p in (Path(__file__).resolve(), TEST)"
    changed = original.replace(old, "for p in ARCHIVE_SOURCES")
    assert original.count(old) == 1 and ast.dump(ast.parse(changed.replace("for p in ARCHIVE_SOURCES", old))) == ast.dump(ast.parse(original))
    code = m.private_scope()["remote_program"]("verify")
    assert inspect.getsource(m.scoped) in code
    assert str(m.REMOTE) in code and "all_object_hashes_verified" in code
    assert "v1" in m.SCHEMA and len(m.ARCHIVE_SOURCES) == 6


def test_fresh_cache_authority_only_not_old_object_approval(tmp_path, monkeypatch):
    monkeypatch.setattr(m, "OUT", tmp_path)
    monkeypatch.setattr(m.base, "read", lambda *a: {"schema":m.known.AUTH_SCHEMA, "explicit_user_approved_exact_o_scope":True})
    monkeypatch.setattr(m, "sources", lambda: {})
    args = Namespace(authority=str(tmp_path / "ROOT_remove_authority.json"), authority_sha256="a"*64,
        mode="remove", inspection_sha256="b"*64, backup_receipt_sha256="c"*64)
    with pytest.raises(ValueError, match="cache-specific"):
        m.checked_authority(args, {"manifest":{"objects_sha256":"d"*64}})


def test_backup_receipt_and_retained_scope_required(tmp_path, monkeypatch):
    monkeypatch.setattr(m, "OUT", tmp_path); monkeypatch.setattr(m, "sources", lambda: {})
    authority = dict(schema=m.AUTH_SCHEMA,mode="remove",approved_cache_root=str(m.CACHE),remote_archive=str(m.REMOTE),
        inspection_sha256="b"*64,objects_sha256="d"*64,source_sha256={},
        explicit_user_approved_exact_ccache_M_R_scope=True,keep_all_stats_tags_directories_and_other_artifacts=True,
        known_system_services_identified_but_privileged_FD_proof_not_claimed=True,
        backup_receipt_sha256="c"*64,fail_closed_unknown_process_and_preservation_risk=True)
    monkeypatch.setattr(m.base,"read",lambda *a:authority)
    args=Namespace(authority=str(tmp_path/"ROOT_remove_authority.json"),authority_sha256="a"*64,mode="remove",
        inspection_sha256="b"*64,backup_receipt_sha256="c"*64)
    assert m.checked_authority(args,{"manifest":{"objects_sha256":"d"*64}})==authority
    authority["backup_receipt_sha256"]="wrong"
    with pytest.raises(ValueError,match="backup-bound"):
        m.checked_authority(args,{"manifest":{"objects_sha256":"d"*64}})
