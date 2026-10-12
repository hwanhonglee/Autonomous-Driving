"""HH_261007 - Preserve alias-debt fixtures and verify the user's completed canonical layout.

The historical layout test remains unchanged. This fresh revision requires
removed root aliases to stay absent, including dangling symlinks, rather than
requiring obsolete editor exclusions for paths that no longer exist.
"""
import importlib.util
import json
from pathlib import Path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / '.git').exists())
spec = importlib.util.spec_from_file_location('hh_layout_audit_v2', ROOT / 'tools/audit_legacy_layout.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
audit, MOVES = module.audit, module.MOVES

# HH_261007 - Fix the entire retired-root denominator so future registry edits cannot hide a restored alias.
RETIRED_ROOTS = (
    'src', 'autoware_e2e_vad_launch', 'build', 'install', 'log', 'cmake',
    'include', 'patches', 'portable_e2e', 'config', 'tests', 'scripts',
    'data', 'datasets', 'artifacts', 'workflows',
)
CANONICAL_ROOTS = ('autonomy', 'learning', 'tools', 'storage', 'docs')


def populate(root, links):
    for old, new in MOVES.items():
        target = root / new
        target.mkdir(parents=True, exist_ok=True)
        if links:
            (root / old).symlink_to(new, target_is_directory=True)


def test_temporary_links_are_not_completion(tmp_path):
    populate(tmp_path, True)
    result = audit(tmp_path)
    assert result['legacy_alias_count'] == 16
    assert not result['legacy_removal_complete']
    assert not result['path_errors']


def test_wrong_link_fails(tmp_path):
    populate(tmp_path, True)
    (tmp_path / 'scripts').unlink()
    (tmp_path / 'scripts').symlink_to('learning')
    assert audit(tmp_path)['path_errors']


def test_zero_links_is_not_runtime_approval(tmp_path):
    populate(tmp_path, False)
    result = audit(tmp_path)
    assert result['legacy_removal_complete']
    assert not result['canonical_only_runtime_verified']


def test_workflow_shortcuts_are_counted_separately(tmp_path):
    # HH_261007 - Preserve the original synthetic distinction between workflow links and root compatibility debt.
    populate(tmp_path, True)
    link = tmp_path / 'docs/workflows/model_source.py'
    link.symlink_to('../../learning/portable_e2e')
    result = audit(tmp_path)
    assert result['legacy_alias_count'] == 16
    assert result['workflow_shortcut_count'] == 1
    assert not result['legacy_removal_complete']


def test_current_workspace_keeps_all_removed_roots_absent_and_canonical_roots_visible():
    # HH_261007 - exists() alone misses dangling links; lexical symlink absence is part of the current user contract.
    assert set(MOVES) == set(RETIRED_ROOTS) and len(RETIRED_ROOTS) == 16
    assert all(not (ROOT / name).exists() and not (ROOT / name).is_symlink() for name in RETIRED_ROOTS)
    assert all((ROOT / name).is_dir() and not (ROOT / name).is_symlink() for name in CANONICAL_ROOTS)
    config = json.loads((ROOT / 'autoware_e2e.code-workspace').read_text())
    settings = config['settings']
    excludes = settings.get('files.exclude', {})
    assert not any(excludes.get(name) for name in CANONICAL_ROOTS)
    assert [folder['path'] for folder in config['folders']] == ['.']
    assert settings['python.analysis.extraPaths'] == ['./learning', '.']
    assert settings['search.followSymlinks'] is False
