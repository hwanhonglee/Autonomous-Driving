# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Test the thin CPU transfer verifier without remote access or GPU imports."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import verify_moving_common10_transfer as transfer

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
SCRIPT = _hh_source_path(ROOT, 'scripts/e2e/verify_moving_common10_transfer.py')


def test_source_ids_resolve_without_root_symlinks_and_keep_frozen_layout(tmp_path):
    # HH_260906 - Canonical current repositories and real frozen repositories share byte identities, not paths.
    current, frozen = tmp_path / 'current', tmp_path / 'frozen'
    (current / 'learning/portable_e2e').mkdir(parents=True)
    (current / 'tools/e2e').mkdir(parents=True)
    frozen.mkdir()
    for name in transfer.SOURCE_PATHS:
        first, rest = name.split('/', 1)
        canonical = current / {'scripts': 'tools', 'portable_e2e': 'learning/portable_e2e'}[first] / rest
        assert transfer.source_path(current, name) == canonical
        assert transfer.source_path(frozen, name) == frozen / name
    with pytest.raises(ValueError, match='unknown'):
        transfer.source_path(current, '../foreign.py')


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.mark.parametrize('split', ['test', 'val', 'unknown', None])
def test_non_train_rejected_before_payload(tmp_path, monkeypatch, split):
    root = tmp_path / 'dataset'
    ep = root / 'episodes/one/episode.json'
    write(ep, dict(split=split, sample_count=328, map_id='Town07'))
    write(root / 'dataset.json', {'episodes': [{'manifest': 'episodes/one/episode.json', 'sha256': transfer.sha(ep)}]})
    calls = []
    original = transfer.read
    def observed(path):
        calls.append(Path(path).name)
        assert Path(path).name in ('dataset.json', 'episode.json'), 'payload was opened before split rejection'
        return original(path)
    monkeypatch.setattr(transfer, 'read', observed)
    with pytest.raises(ValueError, match='no VAL/TEST payload'):
        transfer.metadata_preflight(root, transfer.sha(root / 'dataset.json'))
    assert calls == ['dataset.json', 'episode.json']


def test_external_inventory_pin_precedes_dataset_access(tmp_path, monkeypatch):
    root = tmp_path / 'dataset'; root.mkdir()
    repo = tmp_path / 'repo'; repo.mkdir()
    inv = tmp_path / 'inventory.json'; write(inv, {})
    monkeypatch.setattr(transfer, 'metadata_preflight', lambda *_: pytest.fail('dataset accessed after wrong inventory pin'))
    args = SimpleNamespace(dataset_root=root, source_repo=repo, output=tmp_path / 'out.json',
        create_inventory=False, inventory=inv, inventory_sha256='a' * 64, manifest_sha256='b' * 64)
    with pytest.raises(ValueError, match='external inventory SHA'):
        transfer.execute(args)
    assert not args.output.exists()


@pytest.mark.parametrize('bad', ['{"x":NaN}', '{"x":1e999}', '{"x":0,"x":1}'])
def test_strict_json(tmp_path, bad):
    path = tmp_path / 'bad.json'; path.write_text(bad)
    with pytest.raises(ValueError):
        transfer.read(path)


def test_symlink_and_escape_metadata_rejected(tmp_path):
    root = tmp_path / 'dataset'; root.mkdir()
    other = tmp_path / 'other'; other.mkdir()
    (root / 'link').symlink_to(other, target_is_directory=True)
    for reference in ('../other/x', '/etc/passwd', 'link/x'):
        with pytest.raises(ValueError):
            transfer.relative_file(root, reference)


def test_checkpoint_filename_rejected_before_tree_hash(tmp_path):
    (tmp_path / 'latest.pt').write_bytes(b'not opened')
    tree = SimpleNamespace(build_tree_manifest=lambda *_: pytest.fail('tree hashes checkpoint'))
    with pytest.raises(ValueError, match='checkpoint payloads'):
        transfer.inventory(tree, tmp_path)


def test_report_inside_dataset_rejected(tmp_path):
    root = tmp_path / 'dataset'; root.mkdir()
    args = SimpleNamespace(dataset_root=root, source_repo=tmp_path, output=root / 'report.json')
    with pytest.raises(ValueError, match='fresh report outside'):
        transfer.execute(args)


def test_existing_report_never_overwritten(tmp_path):
    root = tmp_path / 'dataset'; root.mkdir()
    out = tmp_path / 'out.json'; out.write_text('preserve')
    args = SimpleNamespace(dataset_root=root, source_repo=tmp_path, output=out)
    with pytest.raises(ValueError, match='fresh report'):
        transfer.execute(args)
    assert out.read_text() == 'preserve'


def test_full_synthetic_local_inventory_remote_copy_and_corruption(tmp_path, monkeypatch):
    # HH_260906 - Generate only synthetic fixture data; the isolated verifier then uses actual frozen CPU core and tree helper.
    helper_path = Path(__file__).with_name('test_qualify_carla_moving_common10.py')
    spec = importlib.util.spec_from_file_location('moving_conversion_fixture', helper_path)
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    args, _grant, _states, _manifest = helper.make_synthetic_task(tmp_path, monkeypatch)
    local, _ = helper.adapter.prepare_dataset(args)
    manifest_sha = transfer.sha(local / 'dataset.json')
    environment = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONDONTWRITEBYTECODE='1')
    base = [sys.executable, '-I', '-B', str(SCRIPT), '--source-repo', str(ROOT), '--manifest-sha256', manifest_sha]
    local_inventory = tmp_path / 'local_inventory.json'
    first = subprocess.run(base + ['--dataset-root', str(local), '--create-inventory', '--output', str(local_inventory)],
        env=environment, capture_output=True, text=True)
    assert first.returncode == 0, first.stdout + first.stderr
    expected = json.loads(local_inventory.read_text())
    assert expected['train_examples'] == 328 and expected['selected_jpeg_count'] == 1968
    assert expected['torch_imported'] is False and expected['dataset_promoted'] is False
    copied = tmp_path / 'transported'; shutil.copytree(local, copied)
    remote_report = tmp_path / 'remote_report.json'
    command = base + ['--dataset-root', str(copied), '--inventory', str(local_inventory),
        '--inventory-sha256', transfer.sha(local_inventory)]
    second = subprocess.run(command + ['--output', str(remote_report)], env=environment, capture_output=True, text=True)
    assert second.returncode == 0, second.stdout + second.stderr
    report = json.loads(remote_report.read_text())
    assert report['tree'] == expected['tree'] and report['dataset_pre_post_unchanged'] is True
    assert report['train_loader_fingerprint_sha256'] == expected['train_loader_fingerprint_sha256']
    assert not list(local.rglob('__pycache__')) and not list(copied.rglob('__pycache__'))
    payload = next(copied.rglob('*.jpg')); payload.write_bytes(payload.read_bytes() + b'changed')
    rejected = tmp_path / 'rejected.json'
    third = subprocess.run(command + ['--output', str(rejected)], env=environment, capture_output=True, text=True)
    assert third.returncode != 0 and 'inventory differs' in third.stderr
    assert not rejected.exists()
