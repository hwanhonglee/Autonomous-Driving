"""HH_260906 - Validate link migration independently of live compatibility symlinks."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hh_document_links", ROOT / "tools/check_workspace_links.py")
links = importlib.util.module_from_spec(spec)
spec.loader.exec_module(links)


def test_relative_legacy_link_preserves_fragment_without_alias(tmp_path):
    target = tmp_path / "learning/portable_e2e/README.md"
    target.parent.mkdir(parents=True)
    target.write_text("# Model\n")
    result = links.inspect_link(tmp_path, tmp_path / "docs/a.md", "../portable_e2e/README.md#model")
    assert result["status"] == "legacy"
    assert result["replacement"] == "../learning/portable_e2e/README.md#model"
    assert not (tmp_path / "portable_e2e").exists()


def test_missing_destination_is_reported_not_rewritten(tmp_path):
    result = links.inspect_link(tmp_path, tmp_path / "README.md", "artifacts/missing.json")
    assert result["status"] == "missing"
    assert result["replacement"] == "artifacts/missing.json"


def test_fences_inline_code_and_external_urls_are_unchanged(tmp_path):
    text = '```md\n[x](scripts/fake.py)\n```\n`[y](scripts/fake.py)`\n[x](https://example.com/x)\n'
    targets = list(links.destinations(text))
    assert len(targets) == 1
    assert links.inspect_link(tmp_path, tmp_path / "README.md", targets[0][3]) is None


def test_reference_image_angle_title_and_parentheses(tmp_path):
    text = '![x](<data/a b.png> "title")\n[ref]: scripts/a.py "title"\n[x](docs/a(b).md)\n'
    assert [row[3] for row in links.destinations(text)] == ['<data/a b.png>', 'scripts/a.py', 'docs/a(b).md']
    target = tmp_path / 'storage/assets/a b.png'
    target.parent.mkdir(parents=True)
    target.write_bytes(b'png')
    result = links.inspect_link(tmp_path, tmp_path / 'README.md', '<data/a%20b.png>')
    assert result['replacement'] == '<storage/assets/a b.png>'


def test_audit_changes_only_destination(tmp_path):
    (tmp_path / 'tools').mkdir()
    (tmp_path / 'tools/run.py').write_text('pass\n')
    doc = tmp_path / 'README.md'
    before = '[scripts/run.py](scripts/run.py "keep")\nHistorical: `scripts/run.py`\n'
    doc.write_text(before)
    records, edits = links.audit(tmp_path, [doc])
    assert len(records) == 1
    assert edits[doc][1] == before.replace('](scripts/', '](tools/')
    assert doc.read_text() == before


def test_external_files_are_not_rebased(tmp_path):
    result = links.inspect_link(tmp_path, tmp_path / 'README.md', '../another/scripts/x.py')
    assert result['status'] == 'outside_workspace'


def test_immutable_remote_source_is_not_a_navigational_document():
    frozen = links.FROZEN_20260929_SOURCE
    assert links.is_immutable_source_snapshot(frozen / 'plan.md')
    assert links.is_immutable_source_snapshot(frozen / 'audit.md')
    assert not links.is_immutable_source_snapshot(frozen.parent.parent / 'README.md')
    assert not links.is_immutable_source_snapshot(frozen / 'other.md')
    # HH_260929 - The archived photometric preregistration keeps original link bytes.
    photo = links.FROZEN_PHOTOMETRIC_SOURCE
    assert links.is_immutable_source_snapshot(photo / 'plan.md')
    assert not links.is_immutable_source_snapshot(photo / 'README.md')
