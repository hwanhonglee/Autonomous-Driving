#!/usr/bin/env python3
"""HH_261001 - Distinguish sustained progress from near-zero velocity chatter in immutable trials."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def analyze(states, commands):
    # HH_261001 - Attribute only acknowledged learned commands observed by the next native state.
    receipts = {r['ack_receipt_sequence'] for r in commands
                if r.get('application', {}).get('learned_control_applied') is True}
    observed = [s for s in states if s.get('control_transport', {}).get('status') == 'PASS'
                and s['control_transport'].get('expected_receipt_sequence') in receipts]
    if not observed:
        raise ValueError('no acknowledged learned states')
    speeds = [float(s['vx']) for s in observed]
    if not all(math.isfinite(v) for v in speeds):
        raise ValueError('nonfinite measured speed')
    periods = [b['timestamp'] - a['timestamp'] for a, b in zip(states, states[1:])]
    if not periods or any(abs(dt - .05) > 1e-5 for dt in periods):
        raise ValueError('fixed 20 Hz native state contract required')
    dt = .05
    # HH_261001 - This added descriptive threshold does not replace any original pass/fail gate.
    near = [abs(v) <= .5 for v in speeds]
    tail = 0
    for flag in reversed(near):
        if not flag:
            break
        tail += 1
    by_frame = {s['frame']: s for s in states}
    unique = {r['inference_frame']: r for r in commands if r.get('raw_output') is not None}
    candidates = []
    for frame, row in unique.items():
        state = by_frame[frame]
        if abs(state['vx']) > .5:
            continue
        raw = row['raw_output']
        passed = {g['index'] for g in row['candidate_gates'] if g['passed']}
        available = [i for i in range(6) if i in passed
                     and isinstance(raw['candidate_speed'][i][2], (float, int))
                     and raw['candidate_speed'][i][2] > .5]
        selected = row.get('selected')
        candidates.append(dict(frame=frame, actual_vx=state['vx'],
            selected_index=selected['candidate_index'] if selected else None,
            selected_preview_mps=selected['speed_mps'][2] if selected else None,
            drive_candidates_passing_existing_gate_with_preview_above_0p5=available))
    return dict(observed_learned_state_count=len(observed),
        measured_learned_duration_s=len(observed) * dt,
        measured_time_abs_vx_at_most_0p5_s=sum(near) * dt,
        trailing_time_abs_vx_at_most_0p5_s=tail * dt,
        reverse_time_vx_below_minus_0p05_s=sum(v < -.05 for v in speeds) * dt,
        mean_signed_speed_mps=sum(speeds) / len(speeds),
        mean_absolute_speed_mps=sum(abs(v) for v in speeds) / len(speeds),
        low_speed_inferences=len(candidates),
        low_speed_frames_with_any_passing_drive_preview_above_0p5=sum(bool(r['drive_candidates_passing_existing_gate_with_preview_above_0p5']) for r in candidates),
        low_speed_frames_selecting_preview_at_most_0p05_despite_other_drive_above_0p5=sum(
            r['selected_preview_mps'] is not None and r['selected_preview_mps'] <= .05
            and bool(r['drive_candidates_passing_existing_gate_with_preview_above_0p5']) for r in candidates),
        raw_low_speed_candidate_rows=candidates,
        candidate_safety_caveat='Existing trajectory runtime gate only, not obstacle or lane containment proof; no fallback applied.',
        inference_caveat='No counterfactual expert future exists here; an available moving candidate is not an oracle label.',
        original_gates_changed=False, controls_changed=False, training_data_approved=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    episode = args.run / 'episode.partial'
    if not episode.exists():
        episode = args.run / 'episode'
    files = [episode / name for name in ('states.jsonl', 'learned_control.jsonl')]
    before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    result = analyze(*(read_rows(p) for p in files))
    after = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    if after != before:
        raise ValueError('raw source changed during audit')
    result.update(schema='portable_e2e.initial_acceleration_low_speed_audit.v1', run=str(args.run), source_sha256=before)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'raw_low_speed_candidate_rows'}, indent=2))


if __name__ == '__main__':
    main()
