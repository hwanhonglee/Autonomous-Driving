#!/usr/bin/env python3
"""HH_260906 - Verify six fresh fits from transported scalars; no model, TEST or checkpoint loading."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import sys
from types import FunctionType

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.e2e import run_stopmix_regret_research as runner
from portable_e2e.runtime_contract import RuntimeGateConfig

PLAN_SHA = '222286ee7409bd05b5b2b9fc53fce287ce9ef959c17e527b0679d1a20668e9a7'
WORKER_SHA = '170ddeea85d4a3330229cb0850ec919e8edf9b44a07947f76ebac2d7b1b66e7f'
COUNTS = {'train': 1147, 'val': 337}
LOCAL_PURE_PATHS = ('portable_e2e/__init__.py', 'portable_e2e/contract.py',
    'portable_e2e/dataset.py', 'portable_e2e/runtime_contract.py')
GROUPS = ('unavailable_masks', 'stationary_hold', 'moving_to_stop', 'continuing_motion', 'other_motion')
METRICS = ('selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps', 'oracle_ade_m', 'ade_selection_regret_m')
BEHAVIOR = ('terminal_speed_mps', 'terminal_exact_zero', 'terminal_at_or_below_0p1_mps',
    'first_exact_zero_future_index', 'reacceleration_after_exact_future_zero', 'nonincreasing_speed_exact')
IDENTITY = ('sample_id', 'episode_id', 'sequence_index', 'anchor_timestamp_ns', 'camera_sha256',
    'source_manifest_sha256', 'target_motion_group', 'valid_future_points', 'raw_current_vx_mps')
CODES = ('nonfinite_or_shape', 'spatial', 'speed', 'geometric_speed', 'speed_disagreement',
    'distance_disagreement', 'speed_rate', 'geometric_speed_rate', 'stationary_drift', 'first_distance',
    'first_behind', 'backward_step', 'step', 'heading', 'curvature', 'lateral_acceleration', 'extent', 'contract')
DENIALS = dict(training_data_approved=False, vehicle_control_approved=False, automatic_promotion=False,
    test_read_or_use=False, checkpoint_payloads_loaded=False, neural_inference=False,
    trajectory_math_replayed=False, target_groups_reclassified=False)
require, digest, read = runner.require, runner.digest, runner.read


def numeric(value):
    require(type(value) in (int, float) and math.isfinite(value), 'finite numeric scalar required')
    return value


def stamp(value):
    result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(result.tzinfo is not None and result.utcoffset().total_seconds() == 0, 'UTC timestamp required')
    return result


def json_rows(path):
    # HH_260906 - The transport inventory binds every byte; strict parsing rejects malformed/nonfinite rows.
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSONL key')
            result[key] = value
        return result
    with runner.regular(path).open() as stream:
        return [json.loads(line, object_pairs_hook=pairs,
            parse_constant=lambda _: require(False, 'nonfinite JSONL')) for line in stream]


def geometry(rows):
    count = len(rows)
    require(count > 0, 'geometry denominator must be positive')
    def result(values, index=None):
        passed = sum(item['geometry_pass'] for item in values)
        failures = Counter(code for item in values for code in set(item['failure_codes']))
        output = dict(sample_count=count, geometry_pass_count=passed, geometry_reject_count=count-passed,
            geometry_pass_rate=passed/count, failure_counts={code: failures[code] for code in CODES if failures[code]})
        if index is not None:
            output = dict(candidate_index=index, **output)
        return output
    audits = [row['runtime_geometry'] for row in rows]
    selection = [sum(row['selected_candidate_index'] == index for row in rows) for index in range(12)]
    dominant = max(selection)
    entropy = -sum((n/count)*math.log(n/count) for n in selection if n)
    all_count = sum(a['all_candidates_geometry_pass'] for a in audits)
    any_count = sum(a['any_candidate_geometry_pass'] for a in audits)
    selected = [dict(geometry_pass=a['selected_geometry_pass'], failure_codes=a['selected_failure_codes']) for a in audits]
    return dict(sample_count=count, all_candidates_geometry_pass_count=all_count,
        all_candidates_geometry_pass_rate=all_count/count, any_candidate_geometry_pass_count=any_count,
        any_candidate_geometry_pass_rate=any_count/count,
        candidate_results=[result([a['candidates'][index] for a in audits], index) for index in range(12)],
        selected_result=result(selected), selector=dict(selection_counts={str(i): n for i, n in enumerate(selection)},
            invalid_selection_count=0, distinct_selected_candidates=sum(n > 0 for n in selection),
            dominant_candidate_index=selection.index(dominant), dominant_candidate_fraction=dominant/count,
            normalized_selection_entropy=entropy/math.log(12)))


def aggregate(rows):
    if not rows:
        return dict(sample_count=0, metrics=None, geometry=None, stop_selection_count=None)
    return dict(sample_count=len(rows), metrics={key: math.fsum(row[key] for row in rows)/len(rows) for key in METRICS},
        geometry=geometry(rows), stop_selection_count=sum(row['selected_stop_candidate'] for row in rows),
        selected_terminal_exact_zero_count=sum(row['selected_speed_behavior']['terminal_exact_zero'] for row in rows),
        selected_terminal_at_or_below_0p1_count=sum(row['selected_speed_behavior']['terminal_at_or_below_0p1_mps'] for row in rows),
        selected_reacceleration_after_exact_future_zero_count=sum(row['selected_speed_behavior']['reacceleration_after_exact_future_zero'] for row in rows))


def compare_aggregates(stored, measured, path, entropy_differences):
    require(type(stored) is type(measured), 'aggregate type differs: '+path)
    if isinstance(stored, dict):
        require(set(stored) == set(measured), 'aggregate fields differ: '+path)
        for key in stored:
            compare_aggregates(stored[key], measured[key], path+'/'+key, entropy_differences)
    elif isinstance(stored, list):
        require(len(stored) == len(measured), 'aggregate list length differs: '+path)
        for index, (left, right) in enumerate(zip(stored, measured)):
            compare_aggregates(left, right, path+'/'+str(index), entropy_differences)
    elif stored != measured:
        # HH_260906 - Only derived selector entropy permits 1e-15 absolute roundoff; all counts/metrics/decisions remain exact.
        require(path.endswith('/geometry/selector/normalized_selection_entropy')
            and type(stored) is float and math.isfinite(stored) and math.isfinite(measured)
            and abs(stored-measured) <= 1e-15, 'aggregate scalar differs: '+path)
        entropy_differences.append(dict(path=path, stored=stored, recomputed=measured,
            recomputed_minus_stored=measured-stored, absolute_delta=abs(measured-stored)))


def verify_rows(rows, count):
    require(type(count) is int and count > 0 and len(rows) == count
        and [r['index'] for r in rows] == list(range(count)), 'complete ordered row denominator required')
    require(len({r['sample_id'] for r in rows}) == count, 'duplicate sample identity')
    for row in rows:
        require(row['candidate_count'] == 12 and row['target_motion_group'] in GROUPS, 'candidate/group ABI differs')
        require(type(row['valid_future_points']) is int and 1 <= row['valid_future_points'] <= 64, 'valid prefix length differs')
        require((row['target_motion_group'] == 'unavailable_masks') == (row['valid_future_points'] < 64), 'mask group contradicts prefix')
        numeric(row['raw_current_vx_mps'])
        index = row['selected_candidate_index']
        require(type(index) is int and 0 <= index < 12, 'selected index differs')
        require(row['selected_stop_candidate'] is (index >= 6), 'selected STOP family differs')
        require(len(row['camera_sha256']) == 6, 'six camera identities required')
        for value in [*row['camera_sha256'], row['source_manifest_sha256']]:
            runner.hash_value(value)
        candidates = row['candidates']
        require(len(candidates) == 12, 'twelve scalar candidates required')
        for ci, candidate in enumerate(candidates):
            require(candidate['candidate_index'] == ci and candidate['family'] == ('STOP' if ci >= 6 else 'DRIVE'), 'candidate family differs')
            for key in ('ade_m', 'fde_m', 'speed_mae_mps', 'terminal_speed_mps'):
                require(numeric(candidate[key]) >= 0, 'negative stored metric/speed')
            for key in ('terminal_exact_zero', 'terminal_at_or_below_0p1_mps', 'reacceleration_after_exact_future_zero', 'nonincreasing_speed_exact'):
                require(type(candidate[key]) is bool, 'speed behavior flag must be Boolean')
            first = candidate['first_exact_zero_future_index']
            require(first is None or (type(first) is int and 0 <= first < 64), 'future zero index differs')
            require(candidate['terminal_exact_zero'] is (candidate['terminal_speed_mps'] == 0), 'terminal zero scalar differs')
            require(candidate['terminal_at_or_below_0p1_mps'] is (candidate['terminal_speed_mps'] <= .1), 'terminal low-speed scalar differs')
            require(not candidate['terminal_exact_zero'] or first is not None, 'terminal zero requires a first future zero')
            require(not candidate['reacceleration_after_exact_future_zero'] or (first is not None and first < 63), 'impossible reacceleration flag')
        chosen = candidates[index]
        for selected, candidate_key in (('selected_ade_m', 'ade_m'), ('selected_fde_m', 'fde_m'), ('selected_speed_mae_mps', 'speed_mae_mps')):
            require(row[selected] == chosen[candidate_key], 'selected scalar differs from selected candidate')
        require(row['oracle_ade_m'] == min(c['ade_m'] for c in candidates)
            and row['ade_selection_regret_m'] == row['selected_ade_m']-row['oracle_ade_m'], 'oracle/regret scalar differs')
        require(row['selected_speed_behavior'] == {k: chosen[k] for k in BEHAVIOR}, 'selected behavior differs')
        gate = row['runtime_geometry']
        require(len(gate['candidates']) == 12 and gate['selected_candidate_index'] == index, 'gate selector ABI differs')
        for ci, item in enumerate(gate['candidates']):
            require(item['candidate_index'] == ci and type(item['geometry_pass']) is bool, 'gate candidate ABI differs')
            require(isinstance(item['failure_codes'], list) and len(set(item['failure_codes'])) == len(item['failure_codes'])
                and all(code in CODES for code in item['failure_codes']), 'invalid gate failure code')
            require(item['geometry_pass'] is (not item['failure_codes']), 'gate pass contradicts failure codes')
        require(gate['all_candidates_geometry_pass'] is all(c['geometry_pass'] for c in gate['candidates'])
            and gate['any_candidate_geometry_pass'] is any(c['geometry_pass'] for c in gate['candidates']), 'gate aggregate flags differ')
        require(gate['selected_geometry_pass'] is gate['candidates'][index]['geometry_pass']
            and gate['selected_failure_codes'] == gate['candidates'][index]['failure_codes'], 'selected gate differs')
    return dict(all_samples=aggregate(rows), target_motion_groups={group: aggregate([r for r in rows if r['target_motion_group'] == group]) for group in GROUPS})


def verify_history(rows, arm):
    require(len(rows) == 1540, 'all 1540 optimizer-history rows required')
    keys = {'global_step', 'epoch', 'samples_seen', 'batch_domain_sample_counts', 'domain_samples_seen',
        'loss', 'regression_loss', 'candidate_score_loss', 'selected_ade_m', 'selected_fde_m',
        'selected_speed_mae_mps', 'selected_yaw_mae_rad', 'selected_kinematic_speed_mae_mps', 'gradient_norm'}
    if arm == 'B_cost_aware':
        keys.add('candidate_regret_loss')
    for step, row in enumerate(rows, 1):
        require(set(row) == keys, 'optimizer history fields differ')
        seen = 4*step-step//287
        require(row['global_step'] == step and row['epoch'] == (step-1)//287
            and row['samples_seen'] == seen and row['domain_samples_seen'] == {'carla': seen}
            and row['batch_domain_sample_counts'] == {'carla': 3 if step % 287 == 0 else 4}, 'optimizer exposure/order differs')
        for key, value in row.items():
            if key not in ('batch_domain_sample_counts', 'domain_samples_seen'):
                numeric(value)
        require(('candidate_regret_loss' in row) is (arm == 'B_cost_aware'), 'history objective differs')


def pair_decision(a, b, limits):
    am, bm = a['native_metrics'], b['native_metrics']
    delta = {key: bm[key]-am[key] for key in am if key != 'loss'}
    checks = dict(selected_ade_strictly_lower=bm['selected_ade_m'] < am['selected_ade_m'],
        selected_fde_strictly_lower=bm['selected_fde_m'] < am['selected_fde_m'],
        geometry_count_nondecreasing=b['geometry_pass_count'] >= a['geometry_pass_count'],
        speed_mae_within_1p05=bm['selected_speed_mae_mps'] <= 1.05*am['selected_speed_mae_mps'])
    absolute = {key: dict(value=bm[key], limit=limit, pass_=bm[key] <= limit) for key, limit in limits.items()}
    absolute_a = {key: dict(value=am[key], limit=limit, pass_=am[key] <= limit) for key, limit in limits.items()}
    subgroups = {}
    for split in ('train', 'val'):
        subgroups[split] = {}
        for group in GROUPS:
            before = a['behavior'][split]['target_motion_groups'][group]
            after = b['behavior'][split]['target_motion_groups'][group]
            require(before['sample_count'] == after['sample_count'], 'paired subgroup denominator differs')
            subgroups[split][group] = dict(sample_count=before['sample_count'],
                metric_delta_B_minus_A=None if not before['sample_count'] else {k: after['metrics'][k]-before['metrics'][k] for k in METRICS},
                count_delta_B_minus_A=None if not before['sample_count'] else {k: after[k]-before[k] for k in before if k.endswith('_count') and k != 'sample_count'},
                geometry_pass_delta=None if not before['sample_count'] else after['geometry']['selected_result']['geometry_pass_count']-before['geometry']['selected_result']['geometry_pass_count'])
    return dict(metric_delta_B_minus_A=delta, paired_checks=checks, native_absolute_checks_A=absolute_a, native_absolute_checks_B=absolute,
        paired_comparison_pass=all(checks.values()), absolute_gate_pass=all(v['pass_'] for v in absolute.values()),
        subgroup_deltas=subgroups, total_loss_comparison='NOT_COMPARABLE_DIFFERENT_OBJECTIVES')


def verify_transport(root, receipt_path, receipt_sha):
    runner.hash_value(receipt_sha)
    require(digest(receipt_path) == receipt_sha, 'transport receipt SHA differs')
    transport = read(receipt_path)
    require(transport['schema'] == 'portable_e2e.stopmix_regret_transport.v1' and transport['status'] == 'COLLECTED'
        and transport['plan_sha256'] == PLAN_SHA and type(transport['ssh_exit_code']) is int and transport['ssh_exit_code'] == 0
        and transport['checkpoint_payloads_copied'] is False and transport['remote_inventory_unchanged'] is True, 'transport completion differs')
    stamp(transport['collected_at_utc'])
    remote, copied = transport['remote_file_sha256'], transport['copied_file_sha256']
    for mapping in (remote, copied):
        require(isinstance(mapping, dict) and mapping, 'nonempty file inventory required')
        for name, value in mapping.items():
            runner.relative(name); runner.hash_value(value)
    require(copied == {name: value for name, value in remote.items() if not name.endswith('.pt')}, 'every non-checkpoint file must be mirrored, and no checkpoint may be copied')
    files = set()
    for path in root.rglob('*'):
        require(not path.is_symlink(), 'symlink mirror entry forbidden')
        if path.is_file():
            name = path.relative_to(root).as_posix()
            require(name in copied and not name.endswith('.pt'), 'unexpected local mirror file; refuse to read')
            files.add(name)
    require(files == set(copied), 'mirror file inventory incomplete')
    require(all(digest(root/name) == value for name, value in copied.items()), 'mirror bytes differ')
    return transport


def verify_plan_archive(plan_path, archive_path):
    require(digest(plan_path) == PLAN_SHA, 'fixed prospective plan SHA differs')
    plan = read(plan_path)
    # HH_260906 - The frozen worker normalizes its archived JSON; preserve the external prospective byte pin separately.
    require(read(archive_path) == plan and digest(archive_path) == hashlib.sha256(runner.encoded(plan)).hexdigest(),
        'archived plan differs from the exact frozen-worker normalization')
    return plan


def summarize(root, plan_path, receipt_path, receipt_sha):
    root = Path(root).absolute()
    require(root.is_dir() and root == root.resolve() and not any(p.is_symlink() for p in (root, *root.parents)), 'canonical mirror directory required')
    own_sha = digest(Path(__file__))
    require(digest(Path(runner.__file__)) == WORKER_SHA, 'local pure worker schema source differs')
    plan = runner.validate_plan(verify_plan_archive(plan_path, root/'plan.json'), WORKER_SHA)
    transport = verify_transport(root, receipt_path, receipt_sha)
    for name, expected in plan['source_sha256'].items():
        require(digest(root/'source'/name) == expected, 'archived native source differs')
    require(digest(root/'source/run_stopmix_regret_research.py') == WORKER_SHA
        and digest(root/'source/profile_portable_training.py') == runner.OWNER_SHA, 'archived worker/owner differs')
    require(all(digest(ROOT/name) == plan['source_sha256'][name] for name in LOCAL_PURE_PATHS), 'local pure helper/threshold source differs')
    started, summary = read(root/'started.json'), read(root/'summary.json')
    require(started['schema'] == summary['schema'] == 'portable_e2e.stopmix_regret_research.v1'
        and started['status'] == 'RUNNING' and summary['status'] == 'COMPLETE_NOT_PROMOTED'
        and started['plan_sha256'] == summary['plan_sha256'] == PLAN_SHA
        and summary['source_and_view_postcheck_pass'] is True, 'runner completion missing')
    for document in (started, summary):
        require(all(document[key] is value for key, value in runner.DENIALS.items()), 'runner scope denial differs')
    require(summary['runs'] == {run: 'COMPLETE' for run in runner.RUNS}, 'all six fresh fits required')
    require(started['runs'] == {run: 'NOT_RUN' for run in runner.RUNS} and started['stages'] == [], 'initial execution ledger differs')
    idle = dict(physical_index=0, uuid=runner.GPU_UUID, pre_cuda_compute_processes=0)
    require(started['pre_cuda_gpu'] == summary['pre_cuda_gpu'] == summary['final_gpu'] == idle, 'GPU0 ownership boundary receipts differ')
    previous = stamp(started['started_at_utc'])
    require(stamp(plan['declared_at_utc']) <= previous and started['started_at_utc'] == summary['started_at_utc'], 'prospective execution order differs')
    first_behavior = read(root/runner.RUNS[0]/'behavior_started.json')['command']
    remote_workspace = Path(first_behavior[0]).parents[3]
    remote_output = remote_workspace/'runs/diagnostics/stopmix_regret_20260914_v1'
    remote_plan, remote_script = Path(first_behavior[3]), Path(first_behavior[1])
    for path in (remote_plan, remote_script):
        require(path.is_relative_to(remote_workspace) and '..' not in path.parts, 'external stage path escaped personal workspace')
    def attested_digest(path):
        path = Path(path)
        if path.suffix == '.pt':
            base = root if path.is_relative_to(root) else remote_output
            return transport['remote_file_sha256'][path.relative_to(base).as_posix()]
        return digest(path)
    # HH_260906 - Reuse only the frozen pure stage-schema checks; checkpoint SHA comes from the pinned remote receipt, never local payloads.
    stage_check = FunctionType(runner.verify_stage.__code__, dict(vars(runner), digest=attested_digest))
    stage_command = FunctionType(runner.command.__code__, dict(vars(runner), digest=attested_digest))
    receipts, results, identities, entropy_differences = [], {}, {}, []
    for run in runner.RUNS:
        item = root/run
        checkpoint_sha = transport['remote_file_sha256'][run+'/training/checkpoints/latest.pt']
        for stage, timeout in runner.TIMEOUTS.items():
            initial, completed = read(item/f'{stage}_started.json'), read(item/f'{stage}_completed.json')
            expected_command = stage_command(plan, remote_workspace, remote_workspace/'autoware_e2e', remote_output, run, stage, remote_plan, remote_script)
            require(initial == dict(run=run, stage=stage, command=expected_command, timeout_seconds=timeout,
                started_at_utc=initial['started_at_utc'], status='RUNNING'), 'stage command/start receipt differs')
            evidence = stage_check(plan, root, run, stage)
            require(completed == dict(initial, status='COMPLETE', ended_at_utc=completed['ended_at_utc'], **evidence)
                and completed['checkpoint_sha256'] == checkpoint_sha, 'stage completion/hash binding differs')
            begin, end = stamp(initial['started_at_utc']), stamp(completed['ended_at_utc'])
            require(previous <= begin <= end and (end-begin).total_seconds() <= timeout+30, 'stage chronology/timeout differs')
            previous = end; receipts.append(completed)
        train, evaluation, audit, behavior = [read(item/name) for name in ('training/run.json', 'evaluation/metrics.json', 'gate_v8.json', 'behavior/summary.json')]
        require(train['training_split'] == 'train' and len(set(train['training_episode_ids'])) == 3, 'training episode identity differs')
        require(evaluation['rendered_trajectory_png'] == 12 and evaluation['training_episode_count'] == 3
            and evaluation['evaluation_episode_count'] == 1, 'native evaluation episode/render denominator differs')
        history_rows = json_rows(item/'training/metrics.jsonl')
        verify_history(history_rows, run.split('/')[1])
        require(train['last_metrics'] == history_rows[-1], 'training final history and manifest differ')
        require(audit['gate']['thresholds'] == asdict(RuntimeGateConfig(candidate_count=12, future_points=64)), 'fixed v8 thresholds differ')
        measured = {}
        for split, count in COUNTS.items():
            rows = json_rows(item/'behavior'/f'{split}_samples.jsonl')
            measured[split] = verify_rows(rows, count)
            compare_aggregates(behavior['splits'][split],
                dict(measured[split], samples_sha256=digest(item/'behavior'/f'{split}_samples.jsonl')),
                run+'/behavior/'+split, entropy_differences)
            identity = [{key: row[key] for key in IDENTITY} for row in rows]
            require(split not in identities or identities[split] == identity, 'sample identity/groups differ across six fits')
            identities[split] = identity
            episode_ids = {row['episode_id'] for row in rows}
            require(episode_ids == set(train['training_episode_ids']) if split == 'train' else not episode_ids.intersection(train['training_episode_ids']), 'TRAIN/VAL episode overlap or identity differs')
            if split == 'val':
                expected_pngs = {f'{index:04d}_' + ''.join(c if c.isalnum() or c in '-_.' else '_' for c in row['sample_id'])[:120] + '.png'
                    for index, row in enumerate(rows[:12])}
                require({p.name for p in (item/'evaluation/trajectories').glob('*.png')} == expected_pngs, 'native first-twelve VAL trajectory artifacts differ')
                compare_aggregates(audit['geometry'], measured[split]['all_samples']['geometry'],
                    run+'/native_val/geometry', entropy_differences)
                for key in METRICS[:-1]:
                    require(math.isclose(measured[split]['all_samples']['metrics'][key], evaluation['metrics'][key], rel_tol=1e-5, abs_tol=1e-5), 'native/scalar metric reductions disagree')
                for horizon, index in (('1p0', 10), ('3p0', 30), ('6p4', 64)):
                    denominator = sum(row['valid_future_points'] >= index for row in rows)
                    for name in (f'ade_{horizon}s_m', f'fde_{horizon}s_m', f'speed_mae_{horizon}s_mps'):
                        require(evaluation['metric_counts'].get(name) == denominator and denominator > 0, 'native horizon metric denominator differs')
        results[run] = dict(checkpoint_sha256=checkpoint_sha, optimizer_steps=1540, training_samples_seen=6155,
            native_metrics=evaluation['metrics'], metric_counts=evaluation['metric_counts'],
            geometry_pass_count=audit['geometry']['selected_result']['geometry_pass_count'], behavior=measured,
            native_minus_scalar={key: evaluation['metrics'][key]-measured['val']['all_samples']['metrics'][key] for key in METRICS[:-1]})
    require(summary['stages'] == receipts and summary['paired_results'] == runner.paired_results(root), 'runner final stage/paired ledger differs')
    require(previous <= stamp(summary['ended_at_utc']) <= stamp(transport['collected_at_utc']), 'collection/finish chronology differs')
    require((stamp(summary['ended_at_utc'])-stamp(started['started_at_utc'])).total_seconds() <= plan['whole_timeout_seconds']+30,
        'whole execution deadline differs')
    pairs = [dict(seed=seed, arms={arm: results[f'seed_{seed}/{arm}'] for arm in runner.ARMS},
        **pair_decision(results[f'seed_{seed}/A_original_loss'], results[f'seed_{seed}/B_cost_aware'], plan['decision']['absolute_limits_m'])) for seed in runner.SEEDS]
    result = dict(schema='portable_e2e.stopmix_regret_paired_summary.v1', status='SIX_FRESH_FITS_VERIFIED_NOT_PROMOTED',
        plan_sha256=PLAN_SHA, seeds=pairs, all_three_paired_comparisons_pass=all(row['paired_comparison_pass'] for row in pairs),
        all_three_B_absolute_gates_pass=all(row['absolute_gate_pass'] for row in pairs), **DENIALS,
        interpretation='Stored per-candidate scalar metrics and v8 result flags were independently aggregated; raw XY, speed arrays, logits and targets were not saved, so trajectory errors, geometry and subgroup classification were not replayed. Native horizon values remain evaluation/source/checkpoint-bound. TRAIN is fitted evidence; empty VAL hold groups provide no stopping evidence. No closed-loop or vehicle-control approval.')
    verification = dict(schema='portable_e2e.stopmix_regret_verification.v1', status='PASS_NOT_PROMOTED',
        plan_sha256=PLAN_SHA, transport_receipt_sha256=receipt_sha, worker_sha256=WORKER_SHA,
        archived_plan_sha256=digest(root/'plan.json'), archived_plan_serialization='Exact frozen-worker compact normalization of the externally pinned prospective JSON; not byte-identical to the original plan file',
        summarizer_sha256=own_sha, copied_file_sha256=transport['copied_file_sha256'],
        local_pure_source_sha256={name: plan['source_sha256'][name] for name in LOCAL_PURE_PATHS},
        remote_checkpoint_sha256={run: results[run]['checkpoint_sha256'] for run in runner.RUNS},
        fitted_models=6, optimizer_history_rows=9240, behavior_rows=6*sum(COUNTS.values()),
        selector_entropy_roundoff=dict(absolute_tolerance=1e-15,
            maximum_observed_absolute_delta=max((row['absolute_delta'] for row in entropy_differences), default=0.0),
            affected_fields=entropy_differences,
            scope='Only derived normalized_selection_entropy permits roundoff; every other aggregate field is exact. Python/libm reduction differences are a possible cause, not established model or physical variance.'),
        numerical_comparison_tolerance=dict(relative=1e-5, absolute=1e-5, scope='GPU versus stored-scalar reduction verification only, not a decision threshold'), **DENIALS)
    verify_transport(root, receipt_path, receipt_sha)
    require(digest(Path(__file__)) == own_sha and digest(plan_path) == PLAN_SHA
        and digest(Path(runner.__file__)) == WORKER_SHA
        and all(digest(ROOT/name) == plan['source_sha256'][name] for name in LOCAL_PURE_PATHS),
        'verification source/plan changed')
    return verification, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('input-root', 'plan', 'transport-receipt', 'output-dir'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--transport-sha256', required=True)
    args = parser.parse_args(argv)
    output = args.output_dir.absolute()
    require(not output.exists() and output == output.resolve()
        and not any(p.is_symlink() for p in (output, *output.parents)), 'fresh nonsymlink output required')
    for path in (args.input_root.absolute(), args.plan.absolute(), args.transport_receipt.absolute()):
        require(not output.is_relative_to(path) and not path.is_relative_to(output), 'summary output must be disjoint from evidence')
    verification, result = summarize(args.input_root, args.plan, args.transport_receipt, args.transport_sha256)
    output.mkdir(parents=True, exist_ok=False)
    runner.write(output/'verification.json', verification)
    runner.write(output/'paired_summary.json', result)
    print(json.dumps(dict(status=result['status'], output_dir=str(output))))


if __name__ == '__main__':
    main()
