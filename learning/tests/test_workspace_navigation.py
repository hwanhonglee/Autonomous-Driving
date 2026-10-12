# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Verify navigation coverage and cleanup exclusions without touching the workspace."""
import importlib.util
from pathlib import Path
import subprocess
import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT=next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())


def load(name):
    spec=importlib.util.spec_from_file_location(name,_hh_source_path(ROOT, 'scripts')/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def test_every_script_appears_once_in_catalog():
    w=load('workspace');text=w.catalog_text()
    scripts=[p for p in (ROOT/'tools').rglob('*') if p.is_file() and p.suffix in ('.py','.sh') and '__pycache__' not in p.parts]
    for path in scripts:assert text.count(f'](../../{path.relative_to(ROOT).as_posix()})')==1


def test_mapping_training_has_mapping_category():
    w=load('workspace')
    assert w.category(Path('train_map_perception_pilot.py'))==w.CATEGORIES[1]
    # HH_260906 - Expanded mapping data and remote training belong to mapping, not driving or maintenance.
    assert w.category(Path('run_map_expansion_ai.py'))==w.CATEGORIES[1]
    assert w.category(Path('prepare_map_expansion_ai.py'))==w.CATEGORIES[1]


def test_cleanup_does_not_include_data_or_logs():
    c=load('archive_workspace_caches')
    assert all('__pycache__' in p or p=='.pytest_cache' for p in c.TARGETS)


def test_symlink_cache_refused(tmp_path):
    c=load('archive_workspace_caches')
    target=tmp_path/'other';target.mkdir()
    (tmp_path/'.pytest_cache').symlink_to(target,target_is_directory=True)
    with pytest.raises(ValueError):c.inspect(tmp_path)


def test_tracked_cache_refused(tmp_path):
    c=load('archive_workspace_caches')
    subprocess.run(['git','init','-q',str(tmp_path)],check=True)
    cache=tmp_path/'.pytest_cache';cache.mkdir();(cache/'important').write_text('keep')
    subprocess.run(['git','add','.pytest_cache'],cwd=tmp_path,check=True)
    with pytest.raises(ValueError):c.inspect(tmp_path)
