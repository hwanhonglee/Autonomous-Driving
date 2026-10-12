"""HH_261007 - Test non-destructive organization only in temporary trees, never on real home data."""
import os
from pathlib import Path

import pytest

from scripts.maintenance import organize_home_thesis_v1_20261007 as subject


def test_declared_scope_and_category_cohesion():
    moves = subject.declared_moves()
    assert len(moves) == 26
    assert sum(Path(row['source']).parent.name == 'Documents' for row in moves) == 4
    assert all('Unreal Projects' not in row['source'] and 'AnyDesk' not in row['source'] for row in moves)
    names = ('C_track_1_0_7.fbx', 'C_track_1_0_7.fbm')
    destinations = [Path(row['destination']).parent for row in moves if Path(row['source']).name in names]
    assert len(set(destinations)) == 1


def test_same_bytes_and_identity_after_move(tmp_path):
    source = tmp_path / 'source'
    destination = tmp_path / 'destination'
    source.mkdir()
    (source / 'a.txt').write_bytes(b'original\n')
    before = subject.snapshot(source)
    subject.rename_no_replace(source, destination)
    assert not source.exists()
    assert subject.equivalent_after_rename(before, subject.snapshot(destination))


def test_no_overwrite_existing_file(tmp_path):
    source, destination = tmp_path / 'source', tmp_path / 'destination'
    source.write_bytes(b'original')
    destination.write_bytes(b'user edit')
    with pytest.raises(FileExistsError):
        subject.rename_no_replace(source, destination)
    assert source.read_bytes() == b'original'
    assert destination.read_bytes() == b'user edit'


@pytest.mark.parametrize('kind', ['symlink', 'broken_link', 'hardlink', 'fifo'])
def test_reject_unsafe_objects(tmp_path, kind):
    source = tmp_path / 'source'
    source.mkdir()
    if kind == 'fifo':
        os.mkfifo(source / 'unsafe')
    else:
        outside = tmp_path / 'outside'
        outside.write_bytes(b'protected')
        if kind == 'hardlink':
            os.link(outside, source / 'unsafe')
        else:
            (source / 'unsafe').symlink_to(outside if kind == 'symlink' else tmp_path / 'absent')
    with pytest.raises(RuntimeError):
        subject.snapshot(source)


def test_change_is_not_preserved(tmp_path):
    source = tmp_path / 'source'
    source.write_bytes(b'old')
    before = subject.snapshot(source)
    source.write_bytes(b'new')
    assert not subject.equivalent_after_rename(before, subject.snapshot(source))


def test_json_output_is_create_only(tmp_path):
    output = tmp_path / 'record.json'
    subject.write_json_new(output, {'a': 1})
    original = output.read_bytes()
    with pytest.raises(FileExistsError):
        subject.write_json_new(output, {'a': 2})
    assert output.read_bytes() == original


@pytest.mark.parametrize('problem', ['source_escape', 'destination_escape', 'collision', 'symbolic_parent', 'duplicate'])
def test_invalid_plan_is_refused_before_move(tmp_path, monkeypatch, problem):
    monkeypatch.setattr(subject, 'HOME_ROOT', tmp_path)
    monkeypatch.setattr(subject, 'DESTINATION', tmp_path / 'thesis_4pc')
    docs = tmp_path / 'Documents'
    docs.mkdir()
    source = docs / 'item.txt'
    source.write_bytes(b'kept')
    destination = subject.DESTINATION / 'category/item.txt'
    moves = [{'source': str(source), 'destination': str(destination)}]
    if problem == 'source_escape':
        other = tmp_path / 'Other'
        other.mkdir()
        source = other / 'item.txt'
        source.write_bytes(b'kept')
        moves[0]['source'] = str(source)
    elif problem == 'destination_escape':
        moves[0]['destination'] = str(tmp_path / 'Elsewhere/item.txt')
    elif problem == 'collision':
        destination.parent.mkdir(parents=True)
        destination.write_bytes(b'existing')
    elif problem == 'symbolic_parent':
        subject.DESTINATION.mkdir()
        other = tmp_path / 'Other'
        other.mkdir()
        (subject.DESTINATION / 'category').symlink_to(other, target_is_directory=True)
    elif problem == 'duplicate':
        moves.append(dict(moves[0]))
    with pytest.raises((RuntimeError, FileExistsError)):
        subject.validate_layout(moves)
    assert source.read_bytes() == b'kept'


def fixture_plan(tmp_path, monkeypatch):
    # HH_261007 - Full-cycle fixtures redirect every root and never reach real Documents or Downloads.
    monkeypatch.setattr(subject, 'HOME_ROOT', tmp_path)
    monkeypatch.setattr(subject, 'DESTINATION', tmp_path / 'thesis_4pc')
    monkeypatch.setattr(subject, 'RECORDS', tmp_path / 'thesis_4pc/records')
    monkeypatch.setattr(subject, 'active_path_check', lambda moves: {'matching_references': []})
    (tmp_path / 'Documents').mkdir()
    (tmp_path / 'Downloads').mkdir()
    moves = subject.declared_moves()
    for row in moves:
        source = Path(row['source'])
        if source.suffix == '.fbm' or source.name == 'Driving Set' or source.name.startswith('chrony_'):
            source.mkdir()
            (source / 'nested.txt').write_bytes(b'preserve nested input')
        else:
            source.write_bytes(b'preserve input')
    return moves


def test_full_inspect_execute_verify_and_rollback(tmp_path, monkeypatch):
    moves = fixture_plan(tmp_path, monkeypatch)
    digest = subject.inspect()['plan_sha256']
    receipt = subject.execute(digest)
    assert receipt['moved_top_level_objects'] == 26
    assert receipt['preserved_files'] == 26
    assert subject.verify(digest)['verified']
    assert all(not Path(row['source']).exists() for row in moves)
    assert subject.rollback(digest)['restored_top_level_objects'] == 26
    assert all(Path(row['source']).exists() and not Path(row['destination']).exists() for row in moves)


def test_changed_source_refused_before_execute(tmp_path, monkeypatch):
    moves = fixture_plan(tmp_path, monkeypatch)
    digest = subject.inspect()['plan_sha256']
    Path(moves[0]['source']).write_bytes(b'user new edit')
    with pytest.raises(RuntimeError):
        subject.execute(digest)
    assert all(Path(row['source']).exists() for row in moves)
    assert not (subject.RECORDS / 'move_journal.jsonl').exists()


@pytest.mark.parametrize('problem', ['new_original', 'changed_destination'])
def test_rollback_does_not_overwrite_user_changes(tmp_path, monkeypatch, problem):
    moves = fixture_plan(tmp_path, monkeypatch)
    digest = subject.inspect()['plan_sha256']
    subject.execute(digest)
    path = Path(moves[0]['source' if problem == 'new_original' else 'destination'])
    path.write_bytes(b'user new edit')
    with pytest.raises(RuntimeError):
        subject.rollback(digest)
    assert path.read_bytes() == b'user new edit'
    assert not (subject.RECORDS / 'rollback_journal.jsonl').exists()
