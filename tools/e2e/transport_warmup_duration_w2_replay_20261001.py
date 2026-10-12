"""HH_261001 - Preserve remote safe-export proof while explicitly transporting only its file paths."""

import argparse
from copy import deepcopy
import json
from pathlib import Path

from scripts.e2e import research_warmup_duration_w2_replay_20261001 as replay

ROOT = replay.ROOT
PLAN = ROOT / 'docs/assets/validation/2026-10-01/selector_phase_ab/W2_REPLAY_TRANSPORT_ADDENDUM.md'
PLAN_SHA = 'b66cd74cf6f3c3884f179f9b3792696b86521f51ccf3d5ddc0f57cc410ba4bef'
SCHEMA = 'portable_e2e.w2_export_path_transport.v1'
FILES = (Path(__file__), PLAN, ROOT / 'learning/tests/test_warmup_duration_w2_transport_20261001.py')
PINNED_V1 = {
    'tools/e2e/research_warmup_duration_w2_replay_20261001.py': '07dd532b9d193b39ab7b0987110c4ced024e5d07f041ae6b4be3758aa6a54528',
    'learning/tests/test_warmup_duration_w2_replay_20261001.py': 'edbab14fa48131df34e543ff41df91c360f1f34367abc9bccba9018f1fc9c95a',
    'docs/assets/validation/2026-10-01/selector_phase_ab/W2_TOWN07_PAIRED_REPLAY_PREREGISTRATION.md': replay.PLAN_SHA,
}


def sources():
    replay.require(replay.digest(PLAN) == PLAN_SHA, 'frozen prospective transport addendum differs')
    for name, expected in PINNED_V1.items():
        replay.require(replay.digest(ROOT / name) == expected, 'original frozen v1 preparation changed')


def absolute(value):
    path = Path(value)
    replay.require(path.is_absolute() and '..' not in path.parts, 'explicit absolute normalized path required')
    return path


def make_view(original, mapping):
    """HH_261001 - Pure path mapping; all non-path metadata and expected hashes remain untouched."""
    remote_root, local_root = absolute(mapping['remote_training_root']), absolute(mapping['local_training_root'])
    remote_parent, local_parent = absolute(mapping['remote_parent_checkpoint']), absolute(mapping['local_parent_checkpoint'])
    remote_exports, local_exports = absolute(mapping['remote_export_root']), absolute(mapping['local_export_root'])
    replay.require(not remote_parent.is_relative_to(remote_root), 'separate AH-parent mapping must be unambiguous')
    replay.require(remote_root != local_root and remote_exports != local_exports and remote_parent != local_parent,
                   'explicit remote-to-local path transport required')
    evidence = original.get('evidence_file_sha256', {})
    replay.require(len(evidence) == 35 and str(remote_parent) in evidence, 'exact 34 training files plus one AH parent required')
    result, records, translated = deepcopy(original), [], {}
    for name, expected in sorted(evidence.items()):
        old = absolute(name)
        if old == remote_parent:
            new = local_parent
        else:
            replay.require(old.is_relative_to(remote_root), 'undeclared remote evidence path: ' + name)
            new = local_root / old.relative_to(remote_root)
        replay.require(str(new) not in translated, 'two source identities collide at one destination')
        replay.exact_sha(expected)
        translated[str(new)] = expected
        records.append(dict(kind='training_evidence', remote=str(old), local=str(new), sha256=expected))
    result['evidence_file_sha256'] = translated
    for name in ('weights', 'metadata'):
        old = absolute(original[name])
        replay.require(old.parent == remote_exports, 'exact export-root mapping required')
        new = local_exports / old.name
        replay.require(str(new) not in translated, 'model export collides with training evidence')
        result[name] = str(new)
        expected = replay.exact_sha(original[name + '_sha256'])
        records.append(dict(kind=name, remote=str(old), local=str(new), sha256=expected))
    replay.require(result['weights'] != result['metadata'], 'separate model NPZ and metadata required')
    replay.require(len({row['local'] for row in records}) == 37, '37 distinct mapped data files required')
    return result, records


def verify_records(records):
    for row in records:
        replay.require(replay.digest(Path(row['local'])) == row['sha256'],
                       'copied file differs from exact remote export proof: ' + row['local'])


def relocate(args):
    sources()
    replay.exact_sha(args.remote_binding_sha256)
    replay.require(replay.digest(args.remote_binding) == args.remote_binding_sha256,
                   'original retained remote binding bytes differ')
    mapping = {name: str(absolute(getattr(args, name))) for name in
        ('remote_training_root', 'local_training_root', 'remote_parent_checkpoint',
         'local_parent_checkpoint', 'remote_export_root', 'local_export_root')}
    original = replay.read_json(args.remote_binding)
    view, records = make_view(original, mapping)
    verify_records(records)
    targets = [args.remote_binding.resolve(), args.binding.resolve(), args.receipt.resolve()]
    replay.require(len(set(targets)) == 3, 'original binding, new view and receipt must be distinct')
    for target in (args.binding, args.receipt):
        replay.require(not target.exists() and not target.is_symlink(), 'fresh exclusive transport outputs required')
    args.binding.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    replay.write_new(args.binding, view)
    binding_sha = replay.digest(args.binding)
    # HH_261001 - Local validation loads only strict numeric NPZ, never a checkpoint pickle.
    binding = replay.binding_read(args.binding, binding_sha)
    replay.load_endpoint('W2', binding)
    receipt = dict(schema=SCHEMA, status='VERIFIED_PATH_ONLY_TRANSPORT',
        original_binding=str(args.remote_binding.resolve()), original_binding_sha256=args.remote_binding_sha256,
        local_binding=str(args.binding.resolve()), local_binding_sha256=binding_sha,
        addendum_sha256=PLAN_SHA, original_replay_plan_sha256=replay.PLAN_SHA,
        explicit_mapping=mapping, files=records, copied_data_files=37,
        nonpath_payload_unchanged=True, checkpoint_decode_on_local=False, endpoint_reselected=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    replay.write_new(args.receipt, receipt)
    return dict(status=receipt['status'], binding=str(args.binding.resolve()), binding_sha256=binding_sha,
        receipt=str(args.receipt.resolve()), receipt_sha256=replay.digest(args.receipt),
        original_binding_sha256=args.remote_binding_sha256)


def verify_transport(binding_path, binding_sha, receipt_path, receipt_sha):
    sources()
    replay.exact_sha(receipt_sha)
    replay.require(replay.digest(receipt_path) == receipt_sha, 'externally pinned transport receipt required')
    receipt = replay.read_json(receipt_path)
    replay.require(receipt.get('schema') == SCHEMA and receipt.get('status') == 'VERIFIED_PATH_ONLY_TRANSPORT'
        and receipt.get('addendum_sha256') == PLAN_SHA
        and receipt.get('original_replay_plan_sha256') == replay.PLAN_SHA
        and receipt.get('local_binding') == str(binding_path.resolve())
        and receipt.get('local_binding_sha256') == binding_sha
        and receipt.get('copied_data_files') == 37 and receipt.get('nonpath_payload_unchanged') is True
        and receipt.get('checkpoint_decode_on_local') is False and receipt.get('endpoint_reselected') is False
        and receipt.get('training_data_approved') is False and receipt.get('vehicle_control_approved') is False
        and receipt.get('model_promoted') is False, 'transport receipt contract differs')
    original_path = absolute(receipt['original_binding'])
    replay.require(replay.digest(original_path) == replay.exact_sha(receipt['original_binding_sha256']),
                   'byte-identical original remote binding must remain present')
    view, records = make_view(replay.read_json(original_path), receipt['explicit_mapping'])
    replay.require(records == receipt['files'] and view == replay.read_json(binding_path),
                   'only explicitly declared paths may differ from the remote binding')
    verify_records(records)
    binding = replay.binding_read(binding_path, binding_sha)
    replay.load_endpoint('W2', binding)
    return receipt


def trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    receipt = verify_transport(args.binding, args.binding_sha256, args.receipt, args.receipt_sha256)
    original_binding = Path(receipt['original_binding'])
    extras = FILES + (original_binding, args.receipt.resolve())
    replay.require(all(path.is_relative_to(ROOT) for path in extras), 'local archived transport evidence must be inside workspace')
    old_sources, old_pins = replay.NEW_FILES, owner.PINS
    try:
        replay.NEW_FILES = old_sources + extras
        owner.PINS = dict(old_pins)
        owner.PINS.update({PLAN: PLAN_SHA, original_binding: receipt['original_binding_sha256'],
                           args.receipt.resolve(): args.receipt_sha256})
        # HH_261001 - The unchanged worker sees the exact local binding; the owner archives the full transport chain.
        return replay.trial(args)
    finally:
        replay.NEW_FILES, owner.PINS = old_sources, old_pins


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', required=True, choices=('contract', 'relocate', 'verify', 'preflight', 'execute'))
    parser.add_argument('--arm', choices=('W1', 'W2'))
    for name in ('remote-binding', 'remote-training-root', 'local-training-root', 'remote-parent-checkpoint',
                 'local-parent-checkpoint', 'remote-export-root', 'local-export-root', 'binding', 'receipt'):
        parser.add_argument('--' + name, type=Path)
    for name in ('remote-binding-sha256', 'binding-sha256', 'receipt-sha256'):
        parser.add_argument('--' + name)
    args = parser.parse_args(argv)
    if args.mode == 'contract':
        sources()
        result = dict(status='PATH_TRANSPORT_PREREGISTERED_NO_EXPORT_OR_SIMULATOR',
            addendum_sha256=PLAN_SHA, retained_v1_sha256=PINNED_V1, copied_data_files=37,
            checkpoint_decode_on_local=False, runtime_protocol_changed=False)
    elif args.mode == 'relocate':
        required = ('remote_binding', 'remote_binding_sha256', 'remote_training_root', 'local_training_root',
                    'remote_parent_checkpoint', 'local_parent_checkpoint', 'remote_export_root',
                    'local_export_root', 'binding', 'receipt')
        replay.require(all(getattr(args, name) is not None for name in required), 'all explicit transport paths and original SHA required')
        result = relocate(args)
    else:
        replay.require(all(getattr(args, name) is not None for name in
            ('binding', 'binding_sha256', 'receipt', 'receipt_sha256')), 'both exact local receipt identities required')
        if args.mode == 'verify':
            result = verify_transport(args.binding, args.binding_sha256, args.receipt, args.receipt_sha256)
        else:
            replay.require(args.arm is not None, 'fixed replay arm required')
            result = trial(args)
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0 if args.mode != 'execute' or result['status'] in ('DIAGNOSTIC_BUDGET_REACHED', 'DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
