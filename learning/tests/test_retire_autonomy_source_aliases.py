"""HH_260906 - Verify source-only rewriting and recoverable alias removal."""
import importlib.util
from pathlib import Path
import pytest
from portable_e2e.workspace_paths import source_path

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('retire_sources',ROOT/'tools/retire_autonomy_source_aliases.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def populate(root):
    for old,new in module.TARGETS.items():
        (root/new).mkdir(parents=True)
        (root/new/'keep').write_text('preserved')
        (root/old).symlink_to(new,target_is_directory=True)


def test_rewrite_explicit_paths_not_manifest_identifiers():
    source='ROOT / "autonomy/src/core/x"\nROOT / "autonomy/include" / "header"\n${root}/autonomy/patches/a.patch\n"workspace_path": "src/core/x"'
    changed=module.rewrite_text(source)
    assert 'ROOT / "autonomy/src/core/x"' in changed
    assert 'ROOT / "autonomy/include"' in changed
    assert '${root}/autonomy/patches/a.patch' in changed
    assert '"workspace_path": "src/core/x"' in changed
    assert module.rewrite_text(changed)==changed
    assert module.rewrite_text('PACKAGE_ROOT / "include/x"')=='PACKAGE_ROOT / "include/x"'


def test_success_only_removes_links(tmp_path):
    populate(tmp_path)
    module.retire(tmp_path,lambda:None,tmp_path/'record.json')
    assert all(not (tmp_path/old).exists() for old in module.TARGETS)
    assert all((tmp_path/new/'keep').read_text()=='preserved' for new in module.TARGETS.values())


def test_failed_check_restores_every_alias(tmp_path):
    populate(tmp_path)
    def fail():raise RuntimeError('check failed')
    with pytest.raises(RuntimeError):module.retire(tmp_path,fail,tmp_path/'record.json')
    assert all((tmp_path/old).is_symlink() for old in module.TARGETS)


def test_real_directory_refused(tmp_path):
    populate(tmp_path);(tmp_path/'src').unlink();(tmp_path/'src').mkdir()
    with pytest.raises(ValueError):module.retire(tmp_path,lambda:None,tmp_path/'record.json')
    assert (tmp_path/'src').is_dir()


def test_manifest_resolution_without_alias(tmp_path):
    assert source_path(tmp_path,'patches/a.patch')==tmp_path/'autonomy/patches/a.patch'
    assert source_path(tmp_path,'autonomy/src/x')==tmp_path/'autonomy/src/x'
    for bad in ('../outside','/outside',''):
        with pytest.raises(ValueError):source_path(tmp_path,bad)
