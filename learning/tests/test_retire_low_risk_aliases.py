"""HH_260906 - Verify rollback and scope for link retirement without removing any real directory."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import pytest

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'.git').exists())
spec=importlib.util.spec_from_file_location('retirement',ROOT/'tools/retire_low_risk_aliases.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def populate(root):
    for old,new in m.TARGETS.items():
        target=root/new;target.mkdir(parents=True,exist_ok=True)
        (target/'keep.txt').write_text('evidence')
        (root/old).symlink_to(new,target_is_directory=True)


def test_failed_validation_restores_links(tmp_path,monkeypatch):
    populate(tmp_path);monkeypatch.setattr(m,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['retire','--remove'])
    def fail(*args,**kwargs):raise subprocess.CalledProcessError(1,['validation'])
    monkeypatch.setattr(m.subprocess,'run',fail)
    with pytest.raises(subprocess.CalledProcessError):m.main()
    assert all((tmp_path/p).is_symlink() for p in m.TARGETS)
    assert all((tmp_path/p/'keep.txt').read_text()=='evidence' for p in m.TARGETS.values())


def test_success_removes_only_links(tmp_path,monkeypatch):
    populate(tmp_path);monkeypatch.setattr(m,'ROOT',tmp_path)
    monkeypatch.setattr(sys,'argv',['retire','--remove'])
    monkeypatch.setattr(m.subprocess,'run',lambda *a,**kw:None)
    m.main()
    assert all(not (tmp_path/p).exists() for p in m.TARGETS)
    assert all((tmp_path/p/'keep.txt').read_text()=='evidence' for p in m.TARGETS.values())


def test_real_directory_is_refused(tmp_path,monkeypatch):
    populate(tmp_path);(tmp_path/'config').unlink();(tmp_path/'config').mkdir()
    monkeypatch.setattr(m,'ROOT',tmp_path);monkeypatch.setattr(sys,'argv',['retire','--remove'])
    with pytest.raises(ValueError,match='unexpected alias'):m.main()
    assert (tmp_path/'config').is_dir() and (tmp_path/'tests').is_symlink()
