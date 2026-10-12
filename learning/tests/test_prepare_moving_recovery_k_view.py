"""HH_260906 - Check create-only and manifest-pin rejection before any K view copy."""
from pathlib import Path
import pytest
from scripts.e2e.prepare_moving_recovery_k_view import build, write_new


def test_requires_five_sources(tmp_path):
    """HH_260906 - Refuse partial cohorts without creating outputs."""
    output = tmp_path / 'output'
    with pytest.raises(ValueError, match='four successful'):
        build([], output)
    assert not output.exists()


def test_existing_output_is_preserved(tmp_path):
    """HH_260906 - Never replace a previous experiment."""
    with pytest.raises(ValueError, match='fresh canonical'):
        build([], tmp_path)


def test_existing_partial_is_preserved(tmp_path):
    """HH_260906 - Keep interrupted preparation evidence."""
    (tmp_path / 'output.partial').mkdir()
    with pytest.raises(ValueError, match='partial already'):
        build([], tmp_path / 'output')


def test_manifest_pin_checked_before_copy(tmp_path):
    """HH_260906 - Reject untrusted source bytes without output creation."""
    source = tmp_path / 'input'
    source.mkdir()
    write_new(source / 'dataset.json', {})
    with pytest.raises(ValueError, match='SHA mismatch'):
        build([(str(source), '0' * 64)] * 5, tmp_path / 'output')
    assert not (tmp_path / 'output.partial').exists()


def test_write_is_create_only(tmp_path):
    """HH_260906 - Do not overwrite reports."""
    path = tmp_path / 'report.json'
    write_new(path, {'first': True})
    before = path.read_bytes()
    with pytest.raises(FileExistsError):
        write_new(path, {'second': True})
    assert path.read_bytes() == before
