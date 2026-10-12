"""HH_260906 - Verify rollback and target preservation without touching live aliases."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('hh_alias_probe', ROOT/'tools/probe_remaining_aliases.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def fixture(root):
    for name, target in probe.TARGETS.items():
        (root/target).mkdir(parents=True, exist_ok=True)
        (root/name).symlink_to(target)
    payload = root/'storage/assets/keep.txt'
    payload.write_text('original')
    return payload


def test_successful_probe_still_restores_every_link(tmp_path):
    payload = fixture(tmp_path)
    inode = payload.stat().st_ino
    def check():
        assert all(not (tmp_path/name).exists() for name in probe.TARGETS)
        assert payload.read_text() == 'original'
        return 42
    assert probe.probe(tmp_path, list(probe.TARGETS), check) == 42
    assert all((tmp_path/name).is_symlink() for name in probe.TARGETS)
    assert payload.stat().st_ino == inode


@pytest.mark.parametrize('exception', [RuntimeError, KeyboardInterrupt])
def test_failed_probe_restores_links(tmp_path, exception):
    fixture(tmp_path)
    def check():
        raise exception('test')
    with pytest.raises(exception):
        probe.probe(tmp_path, ['data', 'scripts'], check)
    assert all((tmp_path/name).is_symlink() for name in probe.TARGETS)


def test_real_directory_is_never_removed(tmp_path):
    fixture(tmp_path)
    (tmp_path/'data').unlink()
    (tmp_path/'data').mkdir()
    with pytest.raises(ValueError, match='unexpected alias'):
        probe.probe(tmp_path, list(probe.TARGETS), lambda: None)
    assert (tmp_path/'data').is_dir() and not (tmp_path/'data').is_symlink()
    assert (tmp_path/'scripts').is_symlink()
