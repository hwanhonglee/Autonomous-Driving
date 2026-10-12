#!/usr/bin/env python3
"""HH_261001 - Reuse full raw driving analysis for the fixed selector pair without rewriting earlier evidence."""

import argparse
import json
from pathlib import Path
from scripts.e2e import report_initial_acceleration_pair_20261001 as report
from scripts.e2e.audit_initial_acceleration_low_speed_20261001 import analyze as low_speed
from scripts.e2e.research_selector_phase_candidate_20261001 import ARMS, SCHEMA


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training-summary', type=Path, required=True)
    parser.add_argument('--s0-run', type=Path, required=True)
    parser.add_argument('--s1-run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summary = report.read_json(args.training_summary)
    if summary['schema'] != SCHEMA or summary['status'] != 'COMPLETE_NOT_PROMOTED':
        raise ValueError('fixed complete selector training summary required')
    training = [summary['training'][arm] for arm in ARMS]
    for field in ('frozen_generator_state_sha256', 'frozen_train_candidate_cache_sha256', 'exposure_order_sha256'):
        if training[0][field] != training[1][field]:
            raise ValueError('paired frozen generation/exposure differs')
    if not all(row['generator_bit_identity'] is True and row['cache_bit_identity'] is True for row in training):
        raise ValueError('frozen generator/cache check failed')
    args.output.mkdir(parents=True, exist_ok=False)
    trials = {}
    for arm, run in zip(ARMS, (args.s0_run, args.s1_run)):
        bundle = report.analyze_trial(run, summary['training'][arm]['checkpoint_sha256'])
        result = bundle[-1]
        folder = args.output / arm
        folder.mkdir()
        result['visualization'] = report.render_trial(*bundle[:-1], result, folder, arm)
        result['low_speed_and_chatter'] = low_speed(bundle[1], bundle[3])
        result['schema'] = 'portable_e2e.selector_phase_actual_control.v1'
        with (folder / 'analysis.json').open('x') as stream:
            json.dump(result, stream, sort_keys=True, indent=2, allow_nan=False)
            stream.write('\n')
        trials[arm] = result
    result = dict(schema='portable_e2e.selector_phase_pair_report.v1',
        source_summary_sha256=report.sha(args.training_summary), trials=trials,
        pair_checks=report.pair_checks(*(trials[arm] for arm in ARMS)),
        frozen_generator_state_sha256=training[0]['frozen_generator_state_sha256'],
        final_dev_strict_gates=summary['strict_gates'], final_dev_S1_vs_S0=summary['S1_vs_S0'],
        training_data_approved=False, feature_approval_count=0, independent_validation=False,
        limitation='One known TRAIN-route run per model; original owner failures retained; no route-completion claim.')
    with (args.output / 'pair_report.json').open('x') as stream:
        json.dump(result, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(pair_checks=result['pair_checks'], trials={arm:{key: value for key,value in row.items()
        if key in ('progress_m','maximum_speed_kph','maximum_abs_cte_m','trailing_stationary_seconds',
                   'collision_events','lane_invasion_events')} for arm,row in trials.items()}), indent=2))


if __name__ == '__main__':
    main()
