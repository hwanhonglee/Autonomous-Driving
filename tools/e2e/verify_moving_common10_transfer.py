#!/usr/bin/env python3
"""HH_260906 - Authenticate one TRAIN-only moving Common10 transfer; no promotion, Torch, or training."""
from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
SCHEMA = 'portable_e2e.moving_common10_transport_inventory.v1'
SOURCE_PATHS = ('scripts/e2e/verify_tree_manifest.py', 'portable_e2e/__init__.py',
    'portable_e2e/contract.py', 'portable_e2e/dataset.py', 'portable_e2e/config/common_10hz_v1.contract.json')
TREE_FIELDS = ('manifest_sha256', 'file_count', 'directory_count', 'total_size_bytes', 'files')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def safe(path):
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve()
            and not any(p.is_symlink() for p in (path, *path.parents)), 'canonical nonsymlink path required; use the real dataset directory')
    return path


def sha(path):
    path = safe(path)
    require(path.is_file(), 'regular file required')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def expected_sha(value):
    require(isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value), 'external SHA256 required')
    return value


def read(path):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    value = json.loads(safe(path).read_bytes(), object_pairs_hook=pairs,
        parse_constant=lambda _: require(False, 'nonfinite JSON'))
    json.dumps(value, allow_nan=False)
    require(isinstance(value, dict), 'JSON object required')
    return value


def relative_file(root, value):
    require(isinstance(value, str) and value and not Path(value).is_absolute() and '..' not in Path(value).parts,
            'metadata reference must stay inside the dataset')
    path = safe(root / value)
    require(path.is_relative_to(root), 'dataset metadata reference escaped')
    return path


def metadata_preflight(root, manifest_sha):
    # HH_260906 - Reject TEST/VAL metadata before inventory hashing, validation, sample JSONL or image payload reads.
    require(sha(root / 'dataset.json') == expected_sha(manifest_sha), 'external dataset manifest SHA differs')
    manifest = read(root / 'dataset.json')
    references = manifest.get('episodes')
    require(isinstance(references, list) and len(references) == 1, 'exactly one TRAIN episode required')
    reference = references[0]
    episode_path = relative_file(root, reference['manifest'])
    require(sha(episode_path) == expected_sha(reference['sha256']), 'episode manifest SHA differs')
    episode = read(episode_path)
    require(episode.get('split') == 'train' and episode.get('sample_count') == 328
            and episode.get('map_id') == 'Town07', 'only 328 Town07 TRAIN examples are permitted; no VAL/TEST payload may be opened')
    require(episode.get('source_provenance', {}).get('adapter_id') == 'carla_fixed_moving_task_to_common10_v1',
            'wrong moving conversion adapter')
    sidecar = read(root / 'moving_conversion_qualification.json')
    require(sidecar.get('status') == 'QUALIFIED_COMMON10_PLANNING_CONVERSION'
            and sidecar.get('qualified_for_common10_planning_conversion') is True
            and sidecar.get('dataset_manifest_sha256') == manifest_sha
            and sidecar.get('train_examples') == 328 and sidecar.get('val_examples') == sidecar.get('test_examples') == 0
            and sidecar.get('native_goal_reached') is False and sidecar.get('model_or_vehicle_release_approved') is False,
            'moving conversion qualification is incomplete or mismatched')
    return episode


def source_path(repo, name):
    # HH_260906 - Keep frozen source IDs and hashes; translate only this verifier's closed source list.
    require(name in SOURCE_PATHS, 'unknown transport source identifier')
    relative = Path(name)
    if (repo / 'learning/portable_e2e').is_dir() and (repo / 'tools/e2e').is_dir():
        roots = {'scripts': 'tools', 'portable_e2e': 'learning/portable_e2e'}
        return safe(repo / roots[relative.parts[0]] / Path(*relative.parts[1:]))
    return safe(repo / relative)


def load_core(repo, pins):
    # HH_260906 - Import only pre-pinned, framework-free native modules from the explicit source repository.
    require(not any(name == 'torch' or name.startswith(('torch.', 'portable_e2e')) for name in sys.modules),
            'fresh isolated CPU verifier process required')
    require(all(sha(source_path(repo, name)) == digest for name, digest in pins.items()), 'source changed before imports')
    sys.path.insert(0, str(repo / 'learning' if (repo / 'learning/portable_e2e').is_dir() else repo))
    contract = importlib.import_module('portable_e2e.contract')
    dataset = importlib.import_module('portable_e2e.dataset')
    for module in (contract, dataset):
        require(Path(module.__file__).resolve().is_relative_to(repo), 'native import escaped pinned repository')
    spec = importlib.util.spec_from_file_location('_moving_transfer_tree', source_path(repo, SOURCE_PATHS[0]))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    require('torch' not in sys.modules, 'Torch must not be imported by transfer verification')
    return contract, dataset, module


def inventory(tree, root):
    # HH_260906 - Reuse the Sep 7 no-follow, inode-checked tree utility; do not implement a competing file inventory.
    for path in root.rglob('*'):
        require(path.suffix.lower() not in ('.pt', '.pth', '.ckpt', '.safetensors'), 'checkpoint payloads are outside transport scope')
    result = tree.build_tree_manifest(root)
    require(result['status'] == 'PASS' and result['valid'] is True and result['error_count'] == 0,
            'tree manifest failed: ' + repr(result.get('errors')))
    require(sum(item['path'].endswith('.jpg') for item in result['files']) == 1968, 'exactly 1968 selected JPEG copies required')
    return {name: result[name] for name in TREE_FIELDS}


def execute(args):
    root, repo, output = safe(args.dataset_root), safe(args.source_repo), safe(args.output)
    require(not output.exists() and not output.is_relative_to(root) and not root.is_relative_to(output),
            'fresh report outside dataset required')
    require(root.is_dir() and repo.is_dir(), 'dataset and source repository directories required')
    expected = None
    if args.create_inventory:
        require(args.inventory is None and args.inventory_sha256 is None, 'inventory creation cannot consume another inventory')
    else:
        require(args.inventory is not None and args.inventory_sha256 is not None, 'external inventory and SHA required')
        inventory_path = safe(args.inventory)
        require(not inventory_path.is_relative_to(root), 'transport inventory must remain outside dataset')
        require(sha(inventory_path) == expected_sha(args.inventory_sha256), 'external inventory SHA differs')
        expected = read(inventory_path)
        require(expected['schema'] == SCHEMA and expected['dataset_manifest_sha256'] == expected_sha(args.manifest_sha256),
                'transport inventory schema/manifest differs')
    metadata_preflight(root, args.manifest_sha256)
    source = {name: sha(source_path(repo, name)) for name in SOURCE_PATHS}
    verifier_sha = sha(Path(__file__).resolve())
    if expected is not None:
        require(expected['source_sha256'] == source and expected['verifier_sha256'] == verifier_sha,
                'transport verification source bytes differ from local inventory')
    contract, dataset, tree = load_core(repo, source)
    before = inventory(tree, root)
    if expected is not None:
        require(expected['tree'] == before, 'transported file inventory differs from authenticated local dataset')
    validation = contract.validate_dataset(root, mode='planning', check_image_hashes=True)
    loaded = dataset.load_training_examples(root, split='train', mode='planning', check_image_hashes=True)
    require(validation['episode_count'] == 1 and validation['sample_count'] == len(loaded.examples) == 328,
            'unchanged native planning validator/TRAIN loader count differs')
    after = inventory(tree, root)
    require(before == after, 'dataset changed during CPU verification')
    require(source == {name: sha(source_path(repo, name)) for name in SOURCE_PATHS} and sha(Path(__file__).resolve()) == verifier_sha,
            'verification source changed during checks')
    if expected is not None:
        require(sha(args.inventory) == args.inventory_sha256, 'external inventory changed during checks')
        require(expected['train_loader_fingerprint_sha256'] == loaded.fingerprint_sha256,
                'TRAIN loader fingerprint differs from authenticated local inventory')
    require('torch' not in sys.modules, 'Torch was unexpectedly imported')
    result = dict(schema=SCHEMA if args.create_inventory else 'portable_e2e.moving_common10_transport_verification.v1',
        status='PASS', dataset_manifest_sha256=args.manifest_sha256, source_sha256=source,
        verifier_sha256=verifier_sha, tree=before, train_loader_fingerprint_sha256=loaded.fingerprint_sha256,
        validation=validation, train_examples=328, val_examples=0, test_examples=0, selected_jpeg_count=1968,
        dataset_root=str(root), source_repo=str(repo), read_only_dataset=True, dataset_pre_post_unchanged=True,
        checkpoint_payloads_loaded=False, torch_imported=False, gpu_used=False, training_run_performed=False,
        dataset_promoted=False, catalog_goal_success_claim=False, model_or_vehicle_release_approved=False,
        **({} if expected is None else {'external_inventory_sha256': args.inventory_sha256}))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        stream.write((json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode())
        stream.flush(); os.fsync(stream.fileno())
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--source-repo', type=Path, required=True)
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--create-inventory', action='store_true')
    parser.add_argument('--inventory', type=Path)
    parser.add_argument('--inventory-sha256')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    result = execute(args)
    print(json.dumps(dict(status=result['status'], output=str(args.output), output_sha256=sha(args.output),
                         train_examples=328, torch_imported=False, dataset_promoted=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
