"""HH_261001 - Secure CPU-only export of both fixed directional-gradient endpoints; never select a winner."""

import argparse
from contextlib import contextmanager
import inspect
import json
import os
from pathlib import Path

import torch
from portable_e2e import simulation_research as research
from scripts.e2e import run_ah_directional_lower_gradient_ab_20261001 as study

base = study.base
SCHEMA = 'portable_e2e.directional_gradient_export.v1'
CLOSURE_SHA = 'cec35999a624292318bb8b2640ea778730b16ca28ff3f99be7a291a105c0745c'
FILES = ('tools/e2e/export_directional_gradient_cpu_20261001.py',
         'learning/tests/test_directional_gradient_cpu_export_20261001.py')


@contextmanager
def endpoint_identity(checkpoint_sha):
    old = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    try:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = checkpoint_sha, study.SCHEMA
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = old


def read(path):
    base.require(path.is_file() and not path.is_symlink() and path.stat().st_size < 8 * 1024**2,
                 'bounded regular receipt required: ' + str(path))
    return json.loads(path.read_text())


def validate_payload(payload, arm, declaration, receipt, initial_state):
    base.require(arm in study.ARMS and payload.get('checkpoint_schema') == study.SCHEMA
        and payload.get('arm') == arm and payload.get('parent_checkpoint_sha256') == study.previous.AH_CHECKPOINT_SHA256
        and payload.get('training_plan_sha256') == study.PLAN_SHA256
        and payload.get('training_dataset_fingerprint_sha256') == study.reference.DATASET_FINGERPRINT
        and payload.get('exposure_order_sha256') == study.reference.ORDER_SHA
        and payload.get('warmup_mask_sha256') == study.reference.MASK_SHA
        and payload.get('fit') == study.COUNTS and payload.get('research_objective') == declaration
        and payload.get('initial_model_state_sha256') == initial_state
        and payload.get('inference_uses_original_model') is True
        and payload.get('training_data_approved') is False and payload.get('vehicle_control_approved') is False
        and payload.get('model_promoted') is False, 'fixed endpoint provenance differs')
    base.require(isinstance(payload.get('optimizer_state_dict'), dict)
        and bool(payload['optimizer_state_dict'].get('state')), 'actual final optimizer evidence missing')
    expected_recipe = 'native' if arm == study.ARMS[0] else study.directional.RECIPE
    base.require(payload.get('training_only_gradient_recipe') == expected_recipe, 'training recipe differs')
    actual_state = base.tensor_digest(payload['model_state_dict'])
    base.require(actual_state == payload.get('final_model_state_sha256') == receipt.get('final_model_state_sha256'),
                 'actual final tensor digest differs')
    if arm == study.ARMS[0]:
        base.require(actual_state == study.reference.EXPECTED_W1_STATE, 'native control did not reproduce W1')
    return actual_state


def verify_training_sources(root, source_root, sources):
    # HH_261001 - Bind the actual executing source snapshot as well as the archived training closure.
    for relative, expected in sources.items():
        item = Path(relative)
        base.require(not item.is_absolute() and '..' not in item.parts, 'unsafe source reference')
        archived, current = root / 'source' / item, source_root / item
        base.require(archived.is_file() and current.is_file() and not archived.is_symlink() and not current.is_symlink()
            and base.digest(archived) == expected == base.digest(current), 'current or archived source differs: ' + relative)


def validate_chain(root, parent_path):
    declaration = read(root / 'declaration.json')
    complete = read(root / 'both_final_checkpoints_complete.json')
    summary = read(root / 'summary.json')
    sources = declaration.get('source_sha256', {})
    base.require(declaration.get('schema') == study.SCHEMA and declaration.get('training_plan_sha256') == study.PLAN_SHA256
        and len(sources) == 34 and study.supervision.digest_json(sources) == CLOSURE_SHA,
        'fixed 34-file training closure differs')
    verify_training_sources(root, Path(__file__).resolve().parents[2], sources)
    evidence = {}
    paths = [root / name for name in ('declaration.json', 'both_final_checkpoints_complete.json', 'summary.json')]
    for relative, expected in sources.items():
        item = Path(relative)
        base.require(not item.is_absolute() and '..' not in item.parts, 'unsafe source reference')
        archived = root / 'source' / item
        base.require(base.digest(archived) == expected, 'archived training source differs: ' + relative)
        paths.append(archived)
    base.require(summary.get('schema') == study.SCHEMA and summary.get('status') == 'COMPLETE_NOT_PROMOTED'
        and summary.get('source_hashes_match') is True and summary.get('training') == complete
        and set(complete) == set(study.ARMS), 'both fixed final checkpoints required before export')
    base.require(base.digest(parent_path) == study.previous.AH_CHECKPOINT_SHA256, 'original AH parent bytes differ')
    parent, parent_sha = research._read_checkpoint_file(parent_path, torch.device('cpu'))
    base.require(parent_sha == study.previous.AH_CHECKPOINT_SHA256, 'parent changed while securely read')
    initial_state = base.tensor_digest(parent['model_state_dict'])
    endpoints = {}
    for arm in study.ARMS:
        base.require(not (root / arm / 'failure.json').exists(), 'failed endpoint is ineligible for export')
        receipt_path = root / arm / 'training_receipt.json'
        receipt = read(receipt_path)
        relative = arm + '/training/final_checkpoint.pt'
        base.require(receipt == complete[arm] and receipt.get('fit') == study.COUNTS
            and receipt.get('checkpoint') == relative, 'final fit receipt differs')
        checkpoint = root / relative
        base.require(base.digest(checkpoint) == receipt['checkpoint_sha256'], 'checkpoint bytes differ')
        payload, checkpoint_sha = research._read_checkpoint_file(checkpoint, torch.device('cpu'))
        base.require(checkpoint_sha == receipt['checkpoint_sha256'], 'checkpoint changed while securely read')
        state_sha = validate_payload(payload, arm, declaration, receipt, initial_state)
        paths.extend((receipt_path, checkpoint))
        if arm == study.ARMS[0]:
            native_path = root / arm / 'native_identity.json'
            native = read(native_path)
            base.require(native.get('status') == 'PASS_EXACT_W1_STATE'
                and native.get('actual_model_state_sha256') == native.get('expected_model_state_sha256') == state_sha
                and native.get('checkpoint_sha256') == checkpoint_sha and native.get('actual_fit') == study.COUNTS,
                'native identity barrier receipt differs')
            paths.append(native_path)
        endpoints[arm] = dict(checkpoint=relative, checkpoint_sha256=checkpoint_sha, model_state_sha256=state_sha)
    for path in paths:
        base.require(not path.is_symlink() and path.resolve().is_relative_to(root), 'redirected export evidence')
        evidence[str(path.relative_to(root))] = base.digest(path)
    return endpoints, evidence, initial_state


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', choices=('contract', 'export'), required=True)
    parser.add_argument('--training-output', type=Path)
    parser.add_argument('--parent-checkpoint', type=Path)
    args = parser.parse_args(argv)
    source_root = Path(__file__).resolve().parents[2]
    study.verify_new_sources(source_root)
    new_sources = {name: base.digest(source_root / name) for name in FILES}
    base.require(os.environ.get('CUDA_VISIBLE_DEVICES') == '', 'CPU-only visibility required')
    base.require('weights_only' in inspect.signature(torch.load).parameters, 'modern safe checkpoint reader required; no fallback')
    base.require('carla' not in __import__('sys').modules and 'yaml' not in __import__('sys').modules,
                 'simulator dependencies unexpectedly imported')
    if args.mode == 'contract':
        print(json.dumps(dict(status='SECURE_CPU_EXPORT_PREFLIGHT_PASS', source_sha256=new_sources)))
        return 0
    base.require(args.training_output is not None and args.parent_checkpoint is not None, 'exact training root and parent required')
    root = args.training_output.resolve()
    destination = root / 'exports'
    base.require(not destination.exists() and not destination.is_symlink(), 'fresh exports directory required')
    endpoints, evidence, initial_state = validate_chain(root, args.parent_checkpoint)
    destination.mkdir()
    for arm, endpoint in endpoints.items():
        weights = destination / (arm + '.npz')
        with endpoint_identity(endpoint['checkpoint_sha256']):
            exported = research.export_ah_weights(root / endpoint['checkpoint'], weights)
            model = research.load_ah_weights(weights, weights.with_suffix('.json'),
                weights_sha256=exported['weights_sha256'], metadata_sha256=exported['metadata_sha256'])
            base.require(base.tensor_digest(model.state_dict()) == endpoint['model_state_sha256'], 'numeric export differs from final state')
        endpoint.update(weights=str(weights.relative_to(root)), metadata=str(weights.with_suffix('.json').relative_to(root)),
            weights_sha256=exported['weights_sha256'], metadata_sha256=exported['metadata_sha256'])
    base.require(all(base.digest(root / name) == expected for name, expected in evidence.items()), 'evidence changed during export')
    base.require(all(base.digest(source_root / name) == expected for name, expected in new_sources.items()), 'export source changed')
    verify_training_sources(root, source_root, read(root / 'declaration.json')['source_sha256'])
    binding = dict(schema=SCHEMA, status='BOTH_FIXED_ENDPOINTS_EXPORTED_NOT_PROMOTED', endpoints=endpoints,
        training_schema=study.SCHEMA, parent_checkpoint_sha256=study.previous.AH_CHECKPOINT_SHA256,
        initial_model_state_sha256=initial_state, training_source_closure_sha256=CLOSURE_SHA,
        training_plan_sha256=study.PLAN_SHA256, fixed_fit_each_arm=study.COUNTS,
        evidence_file_sha256=evidence, export_source_sha256=new_sources,
        current_and_archived_training_sources_verified_before_and_after=True,
        original_inference_model=True, trained_surrogate_required_at_runtime=False,
        endpoint_selection_used_metric_values=False, optimizer_updates=0,
        vehicle_control_approved=False, model_promoted=False, simulator_launched=False)
    base.write_new(destination / 'endpoint_binding.json', binding)
    print(json.dumps(dict(status=binding['status'], endpoints=endpoints,
        binding_sha256=base.digest(destination / 'endpoint_binding.json')), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
