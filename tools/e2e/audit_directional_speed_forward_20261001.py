"""HH_261001 - Replay all fixed W1 snapshots with original and surrogate-backward classes before any new fit."""

import argparse
import hashlib
import json
from pathlib import Path
import torch
from portable_e2e import simulation_research as research
from portable_e2e.directional_speed_gradient_research import DirectionalLowerSpeedGradientModel
from scripts.e2e.collect_w1_town07_long_20261001 import OUTPUT, CHECKPOINT, WEIGHTS_SHA, METADATA_SHA
from scripts.e2e.replay_exact_warmup_inputs_20261001 import read_rows, load_snapshot
from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity
from scripts.e2e.capture_exact_simulation_inputs_20261001 import output_digest


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('new evidence path required')
    root = Path(__file__).resolve().parents[2]
    archive = OUTPUT / 'sampled_exact_runtime_inputs'
    journal = read_rows(archive / 'index.jsonl')
    if len(journal) != 149:
        raise ValueError('complete fixed sparse snapshot set required')
    weights = root / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports/W1_warmup_anchor_off.npz'
    paths = [Path(__file__), root / 'learning/portable_e2e/directional_speed_gradient_research.py',
             root / 'learning/portable_e2e/model.py', archive / 'index.jsonl', weights, weights.with_suffix('.json')]
    before = {str(p): digest(p) for p in paths}
    torch.set_num_threads(2)
    with identity(CHECKPOINT):
        original = research.load_ah_weights(weights, weights.with_suffix('.json'), weights_sha256=WEIGHTS_SHA, metadata_sha256=METADATA_SHA)
    candidate = DirectionalLowerSpeedGradientModel(original.config).eval()
    candidate.load_state_dict(original.state_dict(), strict=True)
    candidate.requires_grad_(False)
    rows = []
    with torch.no_grad():
        for sequence, entry in enumerate(journal):
            if entry['sequence'] != sequence or entry['source_checkpoint_sha256'] != CHECKPOINT:
                raise ValueError('snapshot source or sequence differs')
            arrays = load_snapshot(archive, entry)
            inputs = [torch.from_numpy(arrays[key]) for key in research.FIELDS]
            outputs = [model(*inputs) for model in (original, candidate)]
            hashes = [output_digest(dict(candidate_xy=out[0][0].tolist(), candidate_speed=out[1][0].tolist(),
                                         candidate_logits=out[2][0].tolist())) for out in outputs]
            if hashes != [entry['raw_output_sha256']] * 2:
                raise ValueError('original/candidate forward does not exactly reproduce recorded output')
            rows.append(dict(sequence=sequence, actual_recorded_output_sha256=hashes[0], all_outputs_bit_exact=True))
    if before != {str(p): digest(p) for p in paths}:
        raise ValueError('source or archive changed')
    report = dict(schema='portable_e2e.directional_gradient_forward_identity.v1', status='PASS',
        source_sha256=before, checkpoint_sha256=CHECKPOINT, frames=149, exact_original_replays=149,
        exact_candidate_forward_replays=149, drive_xy_speed_logits_and_stop_unchanged=True,
        parameters_updated=0, model_promoted=False, actual_new_model_driving=False, rows=rows,
        limitation='These are prior W1 states and weights. Forward equality is not performance improvement or proof about candidate weights after training.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('rows', 'source_sha256')}, indent=2))


if __name__ == '__main__':
    main()
