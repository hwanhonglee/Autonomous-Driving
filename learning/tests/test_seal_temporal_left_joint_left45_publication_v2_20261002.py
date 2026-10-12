"""HH_261002 - Re-run every synthetic original guard with fresh V2 identity and reject symlink ancestors before payload or write."""
import ast
from pathlib import Path
import pytest
from scripts.e2e import seal_temporal_left_joint_left45_publication_v2_20261002 as s

original=ast.parse(Path(__file__).with_name('test_seal_temporal_left_joint_left45_publication_20261002.py').read_bytes())
hits=0
for node in ast.walk(original):
    if isinstance(node,ast.ImportFrom):
        for alias in node.names:
            if alias.name=='seal_temporal_left_joint_left45_publication_20261002':
                alias.name='seal_temporal_left_joint_left45_publication_v2_20261002';hits+=1
assert hits==1
exec(compile(ast.fix_missing_locations(original),__file__,'exec'),globals())


def test_output_symlink_ancestor_rejected_before_authority_read(tmp_path,monkeypatch):
    root=tmp_path.resolve();outside=root/'outside';outside.mkdir();alias=root/'alias';alias.symlink_to(outside,target_is_directory=True)
    monkeypatch.setattr(s,'ROOT',root);monkeypatch.setattr(s,'OUTPUT',alias/'caller.json')
    monkeypatch.setattr(s,'seal',lambda *args:pytest.fail('must reject before first authority payload'))
    with pytest.raises((ValueError,RuntimeError)):s.main(['--authority',str(root/'unused.json'),'--authority-sha256','a'*64])
    assert not (outside/'caller.json').exists()


def test_fresh_V2_identity_and_original_full_guards():
    assert s.SCHEMA.endswith('.v2') and s.OUTPUT.name=='ROOT_publication_caller_v2.json'
    assert s.previous.OUTPUT.name=='ROOT_publication_caller.json'
    assert s.seal.__code__.co_code==s.previous.seal.__code__.co_code
    assert s.seal.__code__.co_consts==s.previous.seal.__code__.co_consts
