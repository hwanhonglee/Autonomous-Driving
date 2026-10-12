# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Verify the fixed relocation scope and refuse linked or tracked evidence."""
import importlib.util
from pathlib import Path
import subprocess
import pytest

spec=importlib.util.spec_from_file_location('organizer',next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())/'tools/organize_workspace_root.py')
organizer=importlib.util.module_from_spec(spec);spec.loader.exec_module(organizer)


def fixture(root):
    subprocess.run(['git','init','-q',str(root)],check=True)
    for name in organizer.TARGETS:
        if name=='maps':
            (root/name).mkdir();(root/name/'status.json').write_text('{}')
        else:(root/name).write_text('{}')


def test_fixed_scope_excludes_real_data(tmp_path):
    fixture(tmp_path)
    rows=organizer.inventory(tmp_path)
    assert len(rows)==4 and 'maps/status.json' in rows
    assert not any(p.startswith(('src/','data/','datasets/','build/')) for p in rows)


def test_tracked_evidence_is_not_moved(tmp_path):
    fixture(tmp_path)
    subprocess.run(['git','add','SUMMARY.md'],cwd=tmp_path,check=True)
    with pytest.raises(ValueError,match='tracked'):organizer.inventory(tmp_path)


def test_linked_evidence_refused(tmp_path):
    fixture(tmp_path)
    (tmp_path/'maps/link').symlink_to(tmp_path/'aggregate.json')
    with pytest.raises(ValueError,match='linked'):organizer.inventory(tmp_path)
