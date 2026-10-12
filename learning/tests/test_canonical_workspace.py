"""HH_260906 - Prove scoped path relocation keeps source and filesystem safety checks intact."""
import hashlib
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('test_layout_adapter', ROOT/'tools/canonical_workspace.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def test_root_maps_only_repository_aliases(tmp_path):
    root = adapter.boundary(tmp_path)
    assert root/'datasets/a' == tmp_path/'storage/datasets/a'
    assert root.joinpath('scripts', 'e2e/a.py') == tmp_path/'tools/e2e/a.py'
    assert root/'portable_e2e/model.py' == tmp_path/'learning/portable_e2e/model.py'
    assert root/'arbitrary/datasets/a' == tmp_path/'arbitrary/datasets/a'
    assert root/'/foreign/datasets/a' == Path('/foreign/datasets/a')
    assert tmp_path/'datasets/a' != root/'datasets/a'  # No global pathlib patch.


def test_alias_traversal_rejected(tmp_path):
    with pytest.raises(ValueError, match='traversal'):
        adapter.boundary(tmp_path)/'scripts/../secret'


def test_external_paths_and_logical_identifiers_unchanged(tmp_path):
    namespace = dict(__file__=str(tmp_path/'tools/e2e/example.py'), ROOT=tmp_path,
                     IDS=['scripts/e2e/a.py'], FOREIGN=Path('/external/datasets'),
                     PATHS={'data':tmp_path/'datasets/train'})
    adapter.adapt(namespace, tmp_path)
    assert namespace['IDS'] == ['scripts/e2e/a.py']
    assert namespace['FOREIGN'] == Path('/external/datasets')
    assert namespace['PATHS']['data'] == tmp_path/'storage/datasets/train'
    assert namespace['__workspace_layout_adapter__']['source_bytes_modified'] is False
    assert len(namespace['__workspace_layout_adapter__']['sha256']) == 64


def test_foreign_module_not_adapted(tmp_path):
    namespace = dict(__file__='/foreign/tools/e2e/example.py', ROOT=tmp_path)
    adapter.adapt(namespace, tmp_path)
    assert type(namespace['ROOT']) is type(tmp_path)
    assert '__workspace_layout_adapter__' not in namespace


def test_dataset_output_guard_without_root_link(tmp_path):
    (tmp_path/'storage/datasets').mkdir(parents=True)
    root = adapter.boundary(tmp_path)
    output = tmp_path/'storage/datasets/new-output'
    protected = (root/'datasets').resolve()
    assert protected == tmp_path/'storage/datasets'
    assert output.resolve().is_relative_to(protected)
    assert not (tmp_path/'datasets').exists()


def test_mapping_does_not_resolve_or_hide_symlinks(tmp_path):
    (tmp_path/'tools').mkdir()
    (tmp_path/'tools/link').symlink_to('/foreign')
    path = adapter.boundary(tmp_path)/'scripts/link/file'
    assert path.parent.is_symlink()
    assert '..' not in path.parts
    assert path == tmp_path/'tools/link/file'


def test_main_dataclass_source_integrity_and_state_restore(tmp_path):
    path = tmp_path/'tools/e2e/example.py'
    path.parent.mkdir(parents=True)
    source = '''from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
@dataclass
class Config:
    value: int = 3
if __name__ == '__main__':
    assert Config().value == 3
    assert ROOT/'datasets' == Path(__file__).parents[2]/'storage/datasets'
    assert sys.argv[1:] == ['argument']
    raise SystemExit(7)
'''
    path.write_text(source)
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    previous, argv = sys.modules['__main__'], sys.argv
    with pytest.raises(SystemExit) as result:
        adapter.execute(path, tmp_path, ['argument'])
    assert result.value.code == 7
    assert sys.modules['__main__'] is previous and sys.argv is argv
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before


def test_entry_without_main_guard_retains_python_semantics(tmp_path):
    path = tmp_path/'example.py'
    path.write_text('raise SystemExit(19)')
    with pytest.raises(SystemExit) as result:
        adapter.execute(path, tmp_path, [])
    assert result.value.code == 19


def test_unrelated_constants_keep_identity_and_type(tmp_path):
    from collections import namedtuple
    point = namedtuple('Point', 'x y')(1, 2)
    values = {'point':point, 'list':[1, 'datasets/logical-id']}
    namespace = dict(__file__=str(tmp_path/'tools/e2e/example.py'), VALUES=values)
    adapter.adapt(namespace, tmp_path)
    assert namespace['VALUES'] is values
    assert namespace['VALUES']['point'] is point


def test_guard_reordering_is_rejected(tmp_path):
    path = tmp_path/'example.py'
    path.write_text("if __name__ == '__main__':\n    raise RuntimeError('not executed')\nvalue = 1\n")
    with pytest.raises(ValueError, match='refusing to reorder'):
        adapter.execute(path, tmp_path, [])
