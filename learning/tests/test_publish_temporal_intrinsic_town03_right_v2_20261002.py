"""HH_261002 - Exact successful atomic-rename aliases only; no real archive, weights, inference, or simulator."""
import ast
import copy
import importlib.util
import json
from pathlib import Path

import pytest

from scripts.e2e import publish_temporal_intrinsic_town03_right_v2_20261002 as p

spec = importlib.util.spec_from_file_location("_intrinsic_frozen_publication_fixture",
    Path(__file__).with_name("test_publish_temporal_intrinsic_town03_right_20261002.py"))
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)


@pytest.fixture
def archive(tmp_path, monkeypatch):
    result = old.archive.__wrapped__(tmp_path, monkeypatch)
    monkeypatch.setattr(p.previous, "publication_sources", p._sources)
    monkeypatch.setattr(p, "publication_sources", lambda: set())
    return result


def load(archive):
    with p.activated(), p.previous.activated(): return p.previous.load_archive(*archive[:2])


def renamed_archive(archive):
    # HH_261002 - Write six genuine source files under partial, then perform one successful directory rename in this fixture.
    path, _, caller, run = archive; episode = run/"episode"; partial = run/"episode.partial"
    episode.rename(partial)
    rows = [json.loads(line) for line in (partial/"recording_after_ack.jsonl").read_bytes().splitlines()]
    for row in rows: row["written_files"] = [str(partial/Path(name).relative_to(episode)) for name in row["written_files"]]
    old.write(partial/"recording_after_ack.jsonl", b"\n".join(json.dumps(row).encode() for row in rows))
    manifest = json.loads((partial/"manifest.json").read_bytes()); manifest["status"] = "complete"
    old.write(partial/"manifest.json", manifest); partial.rename(episode)
    diag = json.loads((run/"diagnostic_report.json").read_bytes()); diag["original_capture_status"] = "complete"
    old.write(run/"diagnostic_report.json", diag)
    return path, old.refresh(path, caller, run), caller, run


@pytest.mark.parametrize("rename", [False, True])
def test_both_exact_literal_aliases_accept_same_original_sources_without_rewriting(archive, rename):
    value = renamed_archive(archive) if rename else archive
    source = value[3]/"episode/recording_after_ack.jsonl"; before = source.read_bytes()
    _, bundle, pins = load(value)
    assert len(bundle["cameras"]) == 3 and len(bundle["states"]) == 6
    assert bundle["actual_recording_census"]["after_ACK_count"] == 3
    assert bundle["diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
    assert source.read_bytes() == before and not (value[3]/"episode.partial").exists()
    pins.recheck()


@pytest.mark.parametrize("kind", ["mixed", "foreign_arm", "foreign_run", "basename", "escape", "duplicate", "other_relative", "symlink"])
def test_alias_matching_is_not_basename_or_mixed_foreign_path_fallback(archive, kind):
    value = renamed_archive(archive); run = value[3]
    def corrupt(rows):
        row = rows[-1]; target = Path(row["written_files"][0])
        if kind == "mixed": row["written_files"][0] = str(run/"episode"/target.relative_to(run/"episode.partial"))
        elif kind == "foreign_arm": row["written_files"] = [name.replace("/control/", "/intrinsic_curve/") for name in row["written_files"]]
        elif kind == "foreign_run": row["written_files"] = [name.replace(run.parent.name, "unowned_run") for name in row["written_files"]]
        elif kind == "basename": row["written_files"] = [Path(name).name for name in row["written_files"]]
        elif kind == "escape": row["written_files"][0] = str(run/"episode.partial/../episode"/target.relative_to(run/"episode.partial"))
        elif kind == "duplicate": row["written_files"][0] = row["written_files"][1]
        elif kind == "other_relative": row["written_files"][0] = str(target.with_name("unrecorded.jpg"))
        else:
            alias = run/"episode.partial"; alias.symlink_to(run/"episode", target_is_directory=True)
    with pytest.raises(Exception): load(old.edit_rows(value, corrupt))


def test_frozen_reader_still_rejects_partial_alias_while_new_version_accepts(archive):
    value = renamed_archive(archive)
    with p.previous.activated():
        with pytest.raises(Exception): p.previous.load_archive(*value[:2])
    assert load(value)[1]["actual_recording_census"]["after_ACK_count"] == 3


def test_new_function_changes_only_the_single_exact_literal_set_comparison(monkeypatch):
    trees = []
    def capture_compile(tree, filename, mode):
        trees.append(copy.deepcopy(tree)); return compile(tree, filename, mode)
    monkeypatch.setattr(p, "compile", capture_compile, raising=False)
    delegated = p.verification_function()
    original = next(n for n in ast.parse(Path(p.previous.__file__).read_bytes()).body
        if isinstance(n, ast.FunctionDef) and n.name == "verify_pair_and_recording")
    assert len(trees) == 1
    changed = trees[0].body[0]
    class Restore(ast.NodeTransformer):
        count = 0
        def visit_Call(self, node):
            if ast.unparse(node) == "written_files_match(row['written_files'], bundle['episode'], camera['images'])":
                self.count += 1; return ast.parse("set(row['written_files']) == expected_files", mode="eval").body
            return self.generic_visit(node)
    restore = Restore(); changed = restore.visit(changed)
    assert restore.count == 1 and ast.dump(changed, include_attributes=False) == ast.dump(original, include_attributes=False)
    assert delegated.__globals__ is not vars(p.previous)


def test_inverse_AST_rejects_another_guard_change(monkeypatch):
    real = p.copy.deepcopy; calls = 0
    def corrupt(node):
        nonlocal calls
        result = real(node); calls += 1
        if calls == 1: result.body.append(ast.parse("other_guard_changed = True").body[0])
        return result
    monkeypatch.setattr(p.copy, "deepcopy", corrupt)
    with pytest.raises(Exception, match="another pair"): p.verification_function()


def test_new_scope_restores_all_original_providers_and_rejects_nesting(archive):
    before = p.previous.verify_pair_and_recording, p.previous.publication_sources
    with pytest.raises(RuntimeError, match="fixture"):
        with p.activated():
            with pytest.raises(Exception):
                with p.activated(): pass
            raise RuntimeError("fixture")
    assert before == (p.previous.verify_pair_and_recording, p.previous.publication_sources)
    assert p._active is False and p.previous._active is False and p.previous._loaded is None


def test_validate_only_successful_rename_writes_no_media_or_companion(archive, capsys):
    value = renamed_archive(archive)
    assert p.main(["--caller-pins", str(value[0]), "--caller-pins-sha256", value[1], "--validate-only"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["validated"] and not result["output_created"] and not p.previous.OUTPUT.exists()
    assert p.previous._loaded is None and not p._active


def test_publication_source_closure_contains_both_frozen_readers_and_new_adapter():
    sources = p.publication_sources()
    assert {Path(p.__file__).resolve(), Path(p.previous.__file__).resolve(),
        Path(p.previous.previous.__file__).resolve()}.issubset(sources)


def test_root_frozen_source_pins_remain_exact():
    p.verify_frozen()
    assert p.previous.previous.prior.digest(p.previous.__file__) == p.FROZEN_SHA
    assert p.previous.previous.prior.digest(p.previous.previous.__file__) == p.BASE_SHA
