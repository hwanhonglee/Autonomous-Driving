#!/usr/bin/env python3
"""HH_261001 - Independently replay every actual W-pair forward from lossless tensor archives."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from portable_e2e import simulation_research as research
from scripts.e2e.capture_exact_simulation_inputs_20261001 import array_digest, output_digest
from scripts.e2e.research_warmup_supervision_candidate_20261001 import ARMS, identity


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def load_snapshot(folder, row):
    name = row['tensor_file']
    if Path(name).name != name or not name.endswith('.npz'):
        raise ValueError('archive filename must be a flat NPZ member')
    path = folder / name
    if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != row['tensor_file_sha256']:
        raise ValueError('exact input archive bytes differ')
    with np.load(path, allow_pickle=False) as archive:
        if set(archive.files) != set(research.FIELDS):
            raise ValueError('exact six input fields required')
        arrays = {key:archive[key].copy() for key in research.FIELDS}
    for key, value in arrays.items():
        expected_dtype = np.bool_ if key.endswith('_mask') else np.float32
        if (value.dtype != expected_dtype or not np.isfinite(value).all()
                or array_digest(value) != row['tensor_sha256'][key]):
            raise ValueError('exact input tensor ABI or hash differs: ' + key)
    return arrays


def replay(args):
    folder = args.trial / 'exact_runtime_inputs'
    journal = read_rows(folder / 'index.jsonl')
    episode = args.trial / 'episode.partial'
    if not episode.exists():
        episode = args.trial / 'episode'
    states = {row['frame']:row for row in read_rows(episode / 'states.jsonl')}
    published = {}
    for decision in read_rows(episode / 'learned_control.jsonl'):
        if decision.get('raw_output') is None or decision.get('held_between_camera_frames'):
            continue
        stamp = round(states[decision['command_frame']]['timestamp'] * 1e9)
        if stamp in published:
            raise ValueError('ambiguous published inference timestamp')
        published[stamp] = decision
    if not journal or len(journal) != len(published):
        raise ValueError('complete attempted-forward journal/publication coverage required')
    torch.set_num_threads(2)
    with identity(args.checkpoint_sha256):
        model = research.load_ah_weights(args.weights, args.metadata,
            weights_sha256=args.weights_sha256, metadata_sha256=args.metadata_sha256)
    results = []
    with torch.no_grad():
        for sequence, row in enumerate(journal):
            if row['sequence'] != sequence or row['source_checkpoint_sha256'] != args.checkpoint_sha256:
                raise ValueError('journal order or source model differs')
            decision = published.pop(row['timing']['now_ns'])
            if (decision['source_checkpoint_sha256'] != args.checkpoint_sha256
                    or output_digest(decision['raw_output']) != row['raw_output_sha256']):
                raise ValueError('journal output does not match published actual control evidence')
            arrays = load_snapshot(folder, row)
            xy, speed, logits = model(*(torch.from_numpy(arrays[key]) for key in research.FIELDS))
            actual = dict(candidate_xy=xy[0].tolist(), candidate_speed=speed[0].tolist(),
                          candidate_logits=logits[0].tolist())
            errors = {key:float(np.max(np.abs(np.asarray(actual[key]) - np.asarray(decision['raw_output'][key]))))
                      for key in actual}
            results.append(dict(sequence=sequence, inference_frame=decision['inference_frame'],
                exact_raw_output_hash_match=output_digest(actual) == row['raw_output_sha256'],
                max_absolute_errors=errors))
    if published:
        raise ValueError('unreplayed published forward')
    return dict(schema='portable_e2e.exact_warmup_input_replay.v1',
        status='PASS' if all(row['exact_raw_output_hash_match'] for row in results) else 'REPLAY_MISMATCH',
        arm=args.arm, trial=str(args.trial), checkpoint_sha256=args.checkpoint_sha256,
        journal_sha256=hashlib.sha256((folder / 'index.jsonl').read_bytes()).hexdigest(),
        forwards_replayed=len(results), exact_matches=sum(row['exact_raw_output_hash_match'] for row in results),
        results=results, device='cpu', compute_threads=2, feature_intervention=False,
        model_promotion=False, training_steps=0, feature_approval_count=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=ARMS, required=True)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--weights-sha256', required=True)
    parser.add_argument('--metadata-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay(args)
    with args.output.open('x') as stream:
        json.dump(result, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key:result[key] for key in ('status', 'arm', 'forwards_replayed', 'exact_matches')}))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
