"""HH_261001 - Test a decoder-gradient hypothesis on every retained long-run snapshot without fitting or control changes."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from portable_e2e import simulation_research as research
from scripts.e2e.capture_exact_simulation_inputs_20261001 import output_digest
from scripts.e2e.collect_w1_town07_long_20261001 import OUTPUT, CHECKPOINT, WEIGHTS_SHA, METADATA_SHA
from scripts.e2e.replay_exact_warmup_inputs_20261001 import load_snapshot, read_rows
from scripts.e2e.research_warmup_supervision_candidate_20261001 import identity


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def decoder_derivatives(model, raw, tensors):
    # HH_261001 - These are mathematical sensitivity probes, not invented expert targets or a new training objective.
    raw = raw.detach().reshape(1, 6, 64, 2).clone().requires_grad_(True)
    xy, speed = model._decode_physical_v1(raw, tensors['ego_history'], tensors['route_xy'], tensors['route_mask'])
    near = torch.autograd.grad(speed[:, :, :20].sum(), raw, retain_graph=True)[0]
    # HH_261001 - Inspect the unchanged tracker's actual 0.3-second preview, not a tuned time threshold.
    preview = torch.autograd.grad(speed[:, :, 2].sum(), raw, retain_graph=True)[0]
    full_speed = torch.autograd.grad(speed.sum(), raw, retain_graph=True)[0]
    full_xy = torch.autograd.grad(xy.sum(), raw)[0]
    rows = []
    for i in range(6):
        prefix = speed[0, i, :20].detach()
        row = dict(candidate=i, first20_speed_all_exact_zero=bool((prefix == 0).all()),
            controller_preview_speed_mps=float(speed[0, i, 2].detach()),
            max_abs_controller_preview_derivative_first3_accel_logits=float(preview[0, i, :3, 0].abs().max()),
            full_speed_sum_derivative_each_first3_accel_logit=full_speed[0, i, :3, 0].tolist(),
            first20_acceleration_logits_all_negative=bool((raw[0, i, :20, 0] < 0).all()),
            first20_speed_min=float(prefix.min()), first20_speed_max=float(prefix.max()),
            final_speed_mps=float(speed[0, i, -1].detach()),
            max_abs_near_speed_sum_derivative_first20_accel_logits=float(near[0, i, :20, 0].abs().max()),
            max_abs_full_speed_sum_derivative_first20_accel_logits=float(full_speed[0, i, :20, 0].abs().max()),
            max_abs_full_xy_sum_derivative_first20_accel_logits=float(full_xy[0, i, :20, 0].abs().max()))
        rows.append(row)
    return xy.detach(), speed.detach(), rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('new audit output required')
    folder = OUTPUT / 'sampled_exact_runtime_inputs'
    episode = OUTPUT / 'episode.partial'
    journal = read_rows(folder / 'index.jsonl')
    states = {row['frame']: row for row in read_rows(episode / 'states.jsonl')}
    published = {}
    for row in read_rows(episode / 'learned_control.jsonl'):
        if row.get('raw_output') is not None and not row.get('held_between_camera_frames'):
            stamp = round(states[row['command_frame']]['timestamp'] * 1e9)
            if stamp in published:
                raise ValueError('ambiguous published timestamp')
            published[stamp] = row
    if len(journal) != 149:
        raise ValueError('all 149 prospectively sampled snapshots required')
    root = Path(__file__).resolve().parents[2]
    bundle = root / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports'
    weights = bundle / 'W1_warmup_anchor_off.npz'
    metadata = weights.with_suffix('.json')
    sources = [Path(__file__), root / 'learning/portable_e2e/model.py', folder / 'index.jsonl',
               episode / 'states.jsonl', episode / 'learned_control.jsonl', weights, metadata]
    before = {str(path): sha(path) for path in sources}
    torch.set_num_threads(2)
    with identity(CHECKPOINT):
        model = research.load_ah_weights(weights, metadata, weights_sha256=WEIGHTS_SHA, metadata_sha256=METADATA_SHA)
    captured = []
    handle = model.trajectory_head.register_forward_hook(lambda module, inputs, output: captured.append(output.detach()))
    results = []
    try:
        for sequence, row in enumerate(journal):
            if row['sequence'] != sequence or row['source_checkpoint_sha256'] != CHECKPOINT:
                raise ValueError('sample order or endpoint differs')
            decision = published[row['timing']['now_ns']]
            if output_digest(decision['raw_output']) != row['raw_output_sha256']:
                raise ValueError('sample is not published actual output')
            arrays = load_snapshot(folder, row)
            tensors = {key: torch.from_numpy(arrays[key]) for key in research.FIELDS}
            captured.clear()
            with torch.no_grad():
                xy, speed, logits = model(*(tensors[key] for key in research.FIELDS))
            actual = dict(candidate_xy=xy[0].tolist(), candidate_speed=speed[0].tolist(), candidate_logits=logits[0].tolist())
            if output_digest(actual) != row['raw_output_sha256'] or len(captured) != 1:
                raise ValueError('exact original replay required before gradient probe')
            decoded_xy, decoded_speed, gradients = decoder_derivatives(model, captured[0], tensors)
            if not torch.equal(decoded_xy, xy[:, :6]) or not torch.equal(decoded_speed, speed[:, :6]):
                raise ValueError('gradient-enabled decoder forward differs')
            selected = decision.get('selected')
            candidate = selected['candidate_index'] if selected else None
            state = states[decision['inference_frame']]
            results.append(dict(sequence=sequence, inference_frame=decision['inference_frame'],
                timestamp_s=state['timestamp'], progress_m=state['route_progress_m'], actual_vx_mps=state['vx'],
                selected_candidate=candidate, exact_replay=True,
                selected_gradient=gradients[candidate] if candidate is not None and candidate < 6 else None,
                drive_candidate_gradients=gradients))
    finally:
        handle.remove()
    if before != {str(path): sha(path) for path in sources}:
        raise ValueError('immutable evidence or source changed')
    selected = [r['selected_gradient'] for r in results if r['selected_gradient'] is not None]
    dead = [r for r in selected if r['first20_speed_all_exact_zero']
            and r['max_abs_near_speed_sum_derivative_first20_accel_logits'] == 0]
    dead_preview = [r for r in selected if r['controller_preview_speed_mps'] == 0
                    and r['max_abs_controller_preview_derivative_first3_accel_logits'] == 0]
    report = dict(schema='portable_e2e.w1_prefix_gradient_diagnosis.v2', status='COMPLETE_DIAGNOSTIC_ONLY',
        checkpoint_sha256=CHECKPOINT, source_sha256=before, snapshots=len(results), exact_replay_count=len(results),
        selected_drive_count=len(selected), selected_zero_2s_prefix_and_zero_near_gradient_count=len(dead),
        selected_zero_controller_preview_and_zero_direct_gradient_count=len(dead_preview),
        zero_preview_with_all_first3_full_speed_sum_derivatives_zero_count=sum(all(v == 0 for v in r['full_speed_sum_derivative_each_first3_accel_logit']) for r in dead_preview),
        all_zero_prefix_also_zero_full_speed_derivative=all(r['max_abs_full_speed_sum_derivative_first20_accel_logits'] == 0 for r in dead) if dead else None,
        all_zero_prefix_also_zero_full_xy_derivative=all(r['max_abs_full_xy_sum_derivative_first20_accel_logits'] == 0 for r in dead) if dead else None,
        results=results, optimizer_updates=0, new_labels=False, control_changes=False, model_promoted=False,
        interpretation='The first audit found zero fully-flat 2-second prefixes; do not call that entire horizon gradient-dead. This follow-up additionally inspects the existing controller index2 (0.3s). Hard lower-speed clamp can block direct regression gradients at that preview. This does not isolate the only live failure cause; shared representation and other losses may still change predictions. No counterfactual expert labels or driving improvement are demonstrated.',
        sampling='All 149 retained prospectively fixed 1-in-10 forwards, not all 1483 live inferences.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('results', 'source_sha256')}, indent=2))


if __name__ == '__main__':
    main()
