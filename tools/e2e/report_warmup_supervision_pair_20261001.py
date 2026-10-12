#!/usr/bin/env python3
"""HH_261001 - Preserve complete paired driving evidence, including moving-to-stall transitions."""

import argparse
import json
from pathlib import Path
from scripts.e2e import report_initial_acceleration_pair_20261001 as report
from scripts.e2e.audit_initial_acceleration_low_speed_20261001 import analyze as low_speed
from scripts.e2e.research_warmup_supervision_candidate_20261001 import ARMS, SCHEMA


def transitions(states, commands):
    # HH_261001 - Use the observed inference-frame state, not the next acknowledged state.
    by_frame = {row['frame']: row for row in states}
    inferences = {}
    for row in commands:
        if row.get('raw_output') is not None and row.get('selected') is not None:
            inferences.setdefault(row['inference_frame'], row)
    changes, previous = [], None
    start = min(inferences) if inferences else None
    for frame, row in sorted(inferences.items()):
        if frame not in by_frame:
            raise ValueError('observed inference-frame state missing')
        selected = row['selected']['candidate_index']
        if selected != previous:
            state = by_frame[frame]
            changes.append(dict(inference_frame=frame, from_candidate=previous, to_candidate=selected,
                seconds_since_first_inference=state['timestamp'] - by_frame[start]['timestamp'],
                observed_vx_mps=state['vx'], selected_v0p3_mps=row['selected']['speed_mps'][2],
                applied_control=row['control'], learned_ack=row.get('application', {}).get('learned_control_applied') is True))
        previous = selected
    return dict(unique_inferences=len(inferences), candidate_transitions=changes,
                candidate_id_alone_is_not_stop_semantics=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training-summary', type=Path, required=True)
    parser.add_argument('--w0-run', type=Path, required=True)
    parser.add_argument('--w1-run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summary = report.read_json(args.training_summary)
    if summary['schema'] != SCHEMA or summary['status'] != 'COMPLETE_NOT_PROMOTED':
        raise ValueError('complete fixed warmup-pair training required')
    training = [summary['training'][arm] for arm in ARMS]
    for field in ('exposure_order_sha256', 'warmup_mask_sha256'):
        if training[0][field] != training[1][field]:
            raise ValueError('paired schedule or source phase mask differs')
    if any(row['fit']['global_step'] != 2999 or row['fit']['scheduled_steps'] != 3000 for row in training):
        raise ValueError('fixed final endpoints required')
    args.output.mkdir(parents=True, exist_ok=False)
    trials = {}
    for arm, run in zip(ARMS, (args.w0_run, args.w1_run)):
        bundle = report.analyze_trial(run, summary['training'][arm]['checkpoint_sha256'])
        result = bundle[-1]
        folder = args.output / arm
        folder.mkdir()
        result['visualization'] = report.render_trial(*bundle[:-1], result, folder, arm)
        result['low_speed_and_chatter'] = low_speed(bundle[1], bundle[3])
        result['candidate_transitions'] = transitions(bundle[1], bundle[3])
        result['schema'] = 'portable_e2e.warmup_supervision_actual_control.v1'
        with (folder / 'analysis.json').open('x') as stream:
            json.dump(result, stream, sort_keys=True, indent=2, allow_nan=False)
            stream.write('\n')
        trials[arm] = result
    result = dict(schema='portable_e2e.warmup_supervision_pair_report.v1',
        source_summary_sha256=report.sha(args.training_summary), trials=trials,
        pair_checks=report.pair_checks(*(trials[arm] for arm in ARMS)),
        final_dev_strict_gates=summary['strict_gates'], final_dev_W1_vs_W0=summary['W1_vs_W0'],
        training_data_approved=False, feature_approval_count=0, independent_validation=False,
        limitation='Known TRAIN-route diagnostics; all original owner failures retained; no route-completion claim.')
    with (args.output / 'pair_report.json').open('x') as stream:
        json.dump(result, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(pair_checks=result['pair_checks'], trials={arm:{k:row[k] for k in
        ('progress_m', 'trailing_stationary_seconds', 'maximum_abs_cte_m', 'collision_events', 'lane_invasion_events')}
        for arm, row in trials.items()}), indent=2))


if __name__ == '__main__':
    main()
