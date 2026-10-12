"""HH_260906 - Test compatibility accounting without treating hidden aliases as removed."""
import json
import importlib.util
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'.git').exists())
spec=importlib.util.spec_from_file_location('hh_layout_audit',ROOT/'tools/audit_legacy_layout.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
audit,MOVES=module.audit,module.MOVES


def populate(root, links):
    for old, new in MOVES.items():
        target=root/new
        target.mkdir(parents=True,exist_ok=True)
        if links:(root/old).symlink_to(new,target_is_directory=True)


def test_temporary_links_are_not_completion(tmp_path):
    populate(tmp_path,True)
    result=audit(tmp_path)
    assert result['legacy_alias_count']==16
    assert not result['legacy_removal_complete']
    assert not result['path_errors']


def test_wrong_link_fails(tmp_path):
    populate(tmp_path,True)
    (tmp_path/'scripts').unlink()
    (tmp_path/'scripts').symlink_to('learning')
    assert audit(tmp_path)['path_errors']


def test_zero_links_is_not_runtime_approval(tmp_path):
    populate(tmp_path,False)
    result=audit(tmp_path)
    assert result['legacy_removal_complete']
    assert not result['canonical_only_runtime_verified']


def test_workflow_shortcuts_are_counted_separately(tmp_path):
    # HH_260906 - Removing duplicate navigation links does not erase remaining root compatibility debt.
    populate(tmp_path, True)
    link = tmp_path / 'docs/workflows/model_source.py'
    link.symlink_to('../../learning/portable_e2e')
    result = audit(tmp_path)
    assert result['legacy_alias_count'] == 16
    assert result['workflow_shortcut_count'] == 1
    assert not result['legacy_removal_complete']


def test_versioned_workspace_hides_only_legacy_roots():
    root=next(p for p in Path(__file__).resolve().parents if (p/'.git').exists())
    config=json.loads((root/'autoware_e2e.code-workspace').read_text())
    excludes=config['settings']['files.exclude']
    assert all(excludes.get(old) for old in MOVES)
    assert not any(excludes.get(name) for name in ('autonomy','learning','tools','storage','docs'))
