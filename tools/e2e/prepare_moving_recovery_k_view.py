#!/usr/bin/env python3
"""HH_260906 - Build a create-only K1 view from five pinned, validated datasets."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from portable_e2e import contract, dataset


def require(value, message):
    """HH_260906 - Fail before promoting an incomplete or altered dataset."""
    if not value:
        raise ValueError(message)


def digest(path):
    """HH_260906 - Bind exact source and destination bytes."""
    return contract.sha256_file(path)


def write_new(path, value):
    """HH_260906 - Preserve any previous outputs rather than overwrite them."""
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def build(sources, output):
    """HH_260906 - Preserve episode bytes, labels and splits under source-specific prefixes."""
    output = output.absolute()
    require(output == output.resolve() and not output.exists(), 'fresh canonical output required')
    partial = output.with_name(output.name + '.partial')
    require(not partial.exists(), 'partial already exists; inspect rather than retry')
    require(len(sources) == 5, 'one baseline and four successful additions required')
    manifests, roots, inventories = [], [], []
    seen_episodes = set()
    for index, (name, expected) in enumerate(sources):
        root = Path(name).absolute()
        require(root == root.resolve() and not root.is_symlink(), 'canonical source required')
        require(not output.is_relative_to(root) and not root.is_relative_to(output), 'overlapping trees')
        require(digest(root / 'dataset.json') == expected, 'source manifest SHA mismatch')
        manifest = json.loads((root / 'dataset.json').read_text())
        require(len(manifest['episodes']) == (6 if index == 0 else 1), 'unexpected episode count')
        for ref in manifest['episodes']:
            relative = Path(ref['manifest'])
            require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe episode reference')
            episode = json.loads((root / relative).read_text())
            require(episode['split'] in (('train', 'val') if index == 0 else ('train',)), 'forbidden split')
            require('stationary' not in ref['episode_id'].lower(), 'stationary data forbidden')
            require(ref['episode_id'] not in seen_episodes, 'duplicate episode')
            seen_episodes.add(ref['episode_id'])
        report = contract.validate_dataset(root, mode='planning', check_image_hashes=True)
        require(report['status'] == 'PASS', 'source validation failed')
        expected_counts = {'train': 1475, 'val': 665} if index == 0 else {'train': 328}
        require({k:v for k,v in report['split_sample_counts'].items() if v} == expected_counts,
                'source split counts differ')
        if index:
            receipt = json.loads((root / 'moving_conversion_qualification.json').read_text())
            require(receipt.get('qualified_for_common10_planning_conversion') is True
                    and receipt.get('dataset_manifest_sha256') == expected, 'conversion receipt mismatch')
        inventory = {}
        for path in sorted(root.rglob('*')):
            require(not path.is_symlink(), 'source symlink forbidden')
            if path.is_file():
                inventory[str(path.relative_to(root))] = digest(path)
        roots.append(root)
        manifests.append(manifest)
        inventories.append(inventory)
    require(len(set(roots)) == 5, 'duplicate source roots')
    output.parent.mkdir(parents=True, exist_ok=True)
    partial.mkdir()
    merged = dict(manifests[0], dataset_id='carla-moving-recovery-k1-20260922-v1', episodes=[], rigs=[])
    rig_hashes = {}
    for index, (root, manifest, inventory) in enumerate(zip(roots, manifests, inventories)):
        prefix = f'source_{index}'
        shutil.copytree(root, partial / prefix)
        for relative, expected in inventory.items():
            require(digest(root / relative) == expected
                    and digest(partial / prefix / relative) == expected, 'copy mismatch/source changed')
        for ref in manifest['episodes']:
            merged['episodes'].append(dict(ref, manifest=prefix + '/' + ref['manifest']))
        for ref in manifest['rigs']:
            rig_id = ref['rig_id']
            require(rig_id not in rig_hashes or rig_hashes[rig_id] == ref['sha256'], 'rig identity collision')
            if rig_id not in rig_hashes:
                merged['rigs'].append(dict(ref, manifest=prefix + '/' + ref['manifest']))
                rig_hashes[rig_id] = ref['sha256']
    write_new(partial / 'dataset.json', merged)
    report = contract.validate_dataset(partial, mode='planning', check_image_hashes=True)
    require(report['status'] == 'PASS', 'merged validation failed')
    require({k:v for k,v in report['split_sample_counts'].items() if v} == {'train':2787,'val':665},
            'merged split counts differ')
    for split, count in [('train',2787),('val',665)]:
        loaded = dataset.load_training_examples(partial, split=split, mode='planning', check_image_hashes=True)
        require(len(loaded.examples) == count, 'loader count differs')
    write_new(partial / 'k_view_provenance.json', dict(
        schema='portable_e2e.k_view.v1', source_manifest_sha256=dict(sources),
        copied_file_sha256={str(root):inv for root,inv in zip(roots,inventories)},
        validation=report, labels_modified=False, splits_modified=False, training_executed=False))
    require(not output.exists(), 'output appeared during preparation')
    partial.rename(output)
    return {'output':str(output), 'dataset_sha256':digest(output/'dataset.json'), 'counts':report['split_sample_counts']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', action='append', nargs=2, metavar=('ROOT','SHA256'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output)))
