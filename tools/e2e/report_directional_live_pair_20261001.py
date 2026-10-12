#!/usr/bin/env python3
"""HH_261001 - Recount the fixed Town07 A/B pair and historical W1 without new driving."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

from scripts.e2e import audit_simulation_research_body_lane_20261001 as lane_audit


ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / 'storage/experiments/validation/2026-10-01'
DEFAULT_RUNS = {
    'A': RUNS / 'directional_gradient_A_town07_sparse20_v1',
    'B': RUNS / 'directional_gradient_B_town07_sparse20_v1',
    'W1_history': RUNS / 'warmup_supervision_W1_warmup_anchor_off_town07_v1',
}
EXPECTED = {
    'A': ('325e188ba0430bd4743d15857f8f44813f6b0de2db6b0fd780cd02bad03522b3',
          'ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f'),
    'B': ('a49ed9cfe0f4cfca970f21f9ac7cd9e20ebf8365b70330a3104b7714d988a37b',
          '6964bc82fbf4eb5742f732aaa05e01967f0b8e69a177652312fa5e251e7cf0ab'),
    'W1_history': ('b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76',
                   'ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f'),
}
PLAN_SHA = '57c0d77539702e49206427f701ba3181ea97b2218bbaf2a45e32068bfde9d0ba'
BINDING_SHA = 'a6b674a471f2bec5c1c153d912a61fac5ecffae72ef797f6cbc9a86574efac8e'
PHS = 20
CAMERAS = ('CAM_FRONT', 'CAM_BACK', 'CAM_FRONT_LEFT', 'CAM_BACK_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT')
CORE = ('manifest.json', 'route.json', 'active_map_opendrive.xodr', 'states.jsonl',
        'camera_frames.jsonl', 'learned_control.jsonl', 'control_receipts.jsonl', 'learned_summary.json')


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def finite(values):
    return [float(value) for value in values if isinstance(value, (int, float)) and math.isfinite(value)]


def pct(values, p):
    values = sorted(finite(values))
    require(values, 'empty numeric percentile source')
    rank = (len(values) - 1) * p / 100
    lower = math.floor(rank)
    upper = math.ceil(rank)
    return values[lower] + (values[upper] - values[lower]) * (rank - lower)


def verify_source_archive(run, plan):
    """HH_261001 - Bind all 95 frozen imports to their retained copy and exact owner pins."""
    pins = plan['source_sha256']
    require(len(pins) == 95, 'expected frozen 95-file runtime closure')
    for name, expected in pins.items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe source path')
        archive = run / 'source' / relative
        require(archive.is_file() and not archive.is_symlink() and digest(archive) == expected,
                f'archived runtime source differs: {archive}')
    return dict(file_count=len(pins), source_pin_sha256=hashlib.sha256(
        json.dumps(pins, sort_keys=True, separators=(',', ':')).encode()).hexdigest())


def validate_record(label, run):
    episode = run / 'episode.partial'
    require(episode.is_dir() and not (run / 'episode').exists(), 'expected one retained partial episode')
    source_hashes = {name: digest(episode / name) for name in CORE}
    states, cameras, commands, receipts = (rows(episode / name) for name in
        ('states.jsonl', 'camera_frames.jsonl', 'learned_control.jsonl', 'control_receipts.jsonl'))
    manifest, route = read(episode / 'manifest.json'), read(episode / 'route.json')
    owner, plan = read(run / 'owner_diagnostic_result.json'), read(run / 'owner_plan.json')
    expected_checkpoint, expected_state = EXPECTED[label]
    require(len(states) == 400 and len(cameras) == 200 and len(commands) == 377,
            f'{label}: full 400/200/377 denominator missing')
    require(len({row['frame'] for row in states}) == 400 and all(
        b['frame'] == a['frame'] + 1 and abs(b['timestamp'] - a['timestamp'] - .05) <= 1e-6
        for a, b in zip(states, states[1:])), f'{label}: native 20 Hz sequence differs')
    by_frame = {row['frame']: row for row in states}
    require(len({row['frame'] for row in cameras}) == 200 and
        [row['frame'] for row in cameras] == [row['frame'] for row in states[::2]],
        f'{label}: camera cadence differs')
    for camera in cameras:
        require(camera['timestamp'] == by_frame[camera['frame']]['timestamp'] and
                tuple(camera['camera_order']) == CAMERAS and set(camera['images']) == set(CAMERAS) and
                camera['timestamp_span_sec'] == 0, f'{label}: camera frame binding differs')
        for image in camera['images'].values():
            relative = Path(image)
            require(not relative.is_absolute() and '..' not in relative.parts,
                    f'{label}: unsafe original camera reference')
            path = episode / relative
            require(path.is_file() and not path.is_symlink() and path.stat().st_size > 0,
                    f'{label}: missing original JPEG {path}')
    require(manifest['status'] == 'failed' and 'driving budget' in manifest['error']
            and owner['status'] == 'DIAGNOSTIC_BUDGET_REACHED' and owner['goal_reached'] is False
            and owner['port_triplet_released'] is True and owner['source_hashes_match'] is True
            and owner['owner_cleanup_errors'] == [] and owner['port_check_errors'] == [],
            f'{label}: expected bounded failure/cleanup differs')
    capture = manifest['capture_contract']
    require(capture['physics_hz'] == PHS and capture['camera_hz'] == 10
            and capture['maximum_total_duration_sec'] == 20 and capture['target_speed_kmh'] == 9
            and capture['learned_research']['checkpoint_sha256'] == expected_checkpoint
            and plan['source_checkpoint_sha256'] == owner['source_checkpoint_sha256'] == expected_checkpoint
            and (plan.get('model_state_sha256', expected_state) == expected_state),
            f'{label}: fixed capture or endpoint identity differs')
    require(source_hashes['route.json'] == manifest['provenance']['route_sha256']
            and source_hashes['active_map_opendrive.xodr'] == capture['active_map_opendrive']['sha256']
            and source_hashes['active_map_opendrive.xodr'] == 'be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf',
            f'{label}: archived route or active map differs')
    require(all(row['source_checkpoint_sha256'] == expected_checkpoint for row in commands),
            f'{label}: command endpoint changed')
    inferred = {row['inference_frame']: row for row in commands if row.get('raw_output') is not None}
    applied = [row for row in commands if row.get('application', {}).get('learned_control_applied') is True]
    require(len(inferred) == 183 and len(applied) == 366 and
            sum(row['control_source'] == 'expert_startup' for row in commands) == 11 and
            not any(row.get('emergency_reason') for row in commands),
            f'{label}: inference/ACK/teacher/emergency counts differ')
    receipt_ids = {row['sequence'] for row in receipts if row['status'] == 'ACKNOWLEDGED'}
    aligned = [row for row in states if row.get('control_transport', {}).get('status') == 'PASS'
               and row['control_transport']['expected_receipt_sequence'] in
               {command['ack_receipt_sequence'] for command in applied}]
    require(len(aligned) == 365 and all(row['ack_receipt_sequence'] in receipt_ids for row in applied),
            f'{label}: native learned ACK alignment differs')
    for inference in inferred.values():
        selected = inference['selected']
        raw = inference['raw_output']
        index = selected['candidate_index']
        require(inference['selection_policy'] == 'raw_argmax_fail_closed' and
                index == max(range(12), key=lambda i: raw['candidate_logits'][i]) and
                inference['candidate_gates'][index]['passed'] is True and
                selected['speed_mps'] == raw['candidate_speed'][index],
                f'{label}: selected raw preview differs')
    source_archive = verify_source_archive(run, plan) if label != 'W1_history' else None
    if label == 'W1_history':
        historical_receipt = read(ROOT / 'storage/experiments/training/2026-10-01/'
            'ah_warmup_supervision_ab_20261001_v1/W1_warmup_anchor_off/training_receipt.json')
        require(historical_receipt['checkpoint_sha256'] == expected_checkpoint and
                historical_receipt['final_model_state_sha256'] == expected_state,
                'historical W1 training receipt differs')
    if label != 'W1_history':
        require(plan['replay_plan_sha256'] == owner['replay_plan_sha256'] == PLAN_SHA
                and plan['binding_sha256'] == owner['binding_sha256'] == BINDING_SHA
                and digest(run / 'endpoint_binding.json') == BINDING_SHA
                and plan['no_metric_based_endpoint_selection'] is True,
                f'{label}: fixed pair plan/export binding differs')
    body_rows, body_summary = lane_audit.body_census(manifest, route, states,
        (episode / 'active_map_opendrive.xodr').read_bytes(), receipts[0]['actor_id'])
    require(len(body_rows) == 400 and all(body_summary[k]['assessed_count'] ==
            (399 if k.startswith('previous_sweep') else 400) for k in body_summary),
            f'{label}: body/sweep denominator differs')
    require(source_hashes == {name: digest(episode / name) for name in CORE},
            f'{label}: raw episode changed during read')
    return dict(label=label, run=run, episode=episode, states=states, cameras=cameras,
                commands=commands, inferred=inferred, aligned=aligned, owner=owner, plan=plan,
                route=route, capture=capture, source_hashes=source_hashes, body_summary=body_summary,
                body_rows=body_rows, source_archive=source_archive)


def stopped_seconds(states, threshold):
    return round(sum(abs(row['vx']) <= threshold for row in states) / PHS, 3)


def trailing_seconds(states, threshold):
    count = 0
    for row in reversed(states):
        if abs(row['vx']) > threshold:
            break
        count += 1
    return round(count / PHS, 3)


def summarize(bundle):
    states, aligned = bundle['states'], bundle['aligned']
    t0 = states[0]['timestamp']
    by_frame = {row['frame']: row for row in states}
    inferences = list(bundle['inferred'].values())
    first_inference_time = by_frame[inferences[0]['inference_frame']]['timestamp']
    brake_rows = []
    preview_rows = []
    for row in inferences:
        state = by_frame[row['inference_frame']]
        preview = row['raw_output']['candidate_speed'][row['selected']['candidate_index']][2]
        item = dict(frame=state['frame'], t_capture_s=round(state['timestamp'] - t0, 3),
                    t_learned_s=round(state['timestamp'] - first_inference_time, 3),
                    observed_vx_mps=state['vx'], selected_raw_preview_0p3_mps=preview,
                    selected_candidate=row['selected']['candidate_index'],
                    tracker_desired_mps=row['control']['desired_speed_mps'],
                    tracker_brake=row['control']['brake'],
                    tracker_throttle=row['control']['throttle'],
                    selected_raw_2p0_mps=row['raw_output']['candidate_speed'][row['selected']['candidate_index']][19])
        preview_rows.append(item)
        if row['control']['brake'] > .05 and state['vx'] > 1.0:
            brake_rows.append(item)
    speeds = [abs(row['vx']) for row in states]
    signed = [row['vx'] for row in states]
    learned_speeds = [abs(row['vx']) for row in aligned]
    early = [row for row in preview_rows if row['t_capture_s'] < 10]
    require(len(early) == 83, f"{bundle['label']}: expected 83 distinct first-10s learned inferences")
    candidate_gates = [gate for inference in inferences for gate in inference['candidate_gates']]
    require(len(candidate_gates) == 183 * 12, f"{bundle['label']}: all 12 runtime gates per inference required")
    moved = False
    first_observed_stop = None
    for state in states:
        moved = moved or abs(state['vx']) > 1.0
        if moved and abs(state['vx']) <= .05:
            first_observed_stop = dict(frame=state['frame'], t_capture_s=round(state['timestamp'] - t0, 3),
                                       route_progress_m=state['route_progress_m'])
            break
    native_rates = [((later['vx'] - earlier['vx']) /
                     (later['timestamp'] - earlier['timestamp']), later['frame'])
                    for earlier, later in zip(states, states[1:])]
    worst_rate, worst_rate_frame = min(native_rates)
    body = bundle['body_summary']
    route_length = bundle['route']['route_length_m']
    result = dict(
        run=str(bundle['run']), episode=str(bundle['episode']),
        checkpoint_sha256=EXPECTED[bundle['label']][0], model_state_sha256=EXPECTED[bundle['label']][1],
        source_sha256=bundle['source_hashes'], source_archive=bundle['source_archive'],
        owner_plan_sha256=digest(bundle['run'] / 'owner_plan.json'),
        owner_diagnostic_sha256=digest(bundle['run'] / 'owner_diagnostic_result.json'),
        original_manifest_status='failed', original_owner_status=bundle['owner']['status'],
        original_error=bundle['owner']['original_capture_error'], cleanup_ok=True,
        native_states=len(states), camera_bundles=len(bundle['cameras']), original_jpegs=6 * len(bundle['cameras']),
        distinct_inferences=len(inferences), learned_commands_acknowledged=366,
        learned_ack_observed_states=len(aligned),
        expert_startup_commands=11, emergency_commands=0,
        native_control_alignment_pass_count=sum(row.get('control_transport', {}).get('status') == 'PASS' for row in states),
        route_length_m=route_length, first_arc_m=states[0]['route_progress_m'],
        final_arc_m=states[-1]['route_progress_m'],
        net_arc_progress_m=states[-1]['route_progress_m'] - states[0]['route_progress_m'],
        route_fraction_at_budget=states[-1]['route_progress_m'] / route_length,
        planar_travel_m=sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(states, states[1:])),
        speed_abs_mps_all_400=dict(min=min(speeds), median=statistics.median(speeds), max=max(speeds)),
        speed_signed_mps_all_400=dict(min=min(signed), median=statistics.median(signed), max=max(signed)),
        speed_abs_mps_learned_ack_365=dict(min=min(learned_speeds), median=statistics.median(learned_speeds),
                                           max=max(learned_speeds)),
        stopped_time_all_400_s=dict(abs_vx_at_most_0p05=stopped_seconds(states, .05),
                                    abs_vx_at_most_0p5=stopped_seconds(states, .5)),
        stopped_time_learned_ack_365_s=dict(abs_vx_at_most_0p05=stopped_seconds(aligned, .05),
                                            abs_vx_at_most_0p5=stopped_seconds(aligned, .5),
                                            trailing_abs_vx_at_most_0p5=trailing_seconds(aligned, .5)),
        reverse_time_vx_below_minus_0p05_s=round(sum(row['vx'] < -.05 for row in aligned) / PHS, 3),
        maximum_abs_route_cte_m=max(abs(row['route_cte_m']) for row in states),
        minimum_discrete_signed_vx_rate_mps2=worst_rate,
        minimum_discrete_signed_vx_rate_frame=worst_rate_frame,
        first_observed_stop_after_exceeding_1mps=first_observed_stop,
        collision_events=sum(len(row['collision']) for row in states),
        lane_invasion_events=sum(len(row['lane_invasion']) for row in states),
        body_and_bounded_sweep=body,
        inference_ms=dict(p50=pct([row['total_seconds'] * 1000 for row in inferences], 50),
                          p95=pct([row['total_seconds'] * 1000 for row in inferences], 95),
                          max=max(row['total_seconds'] * 1000 for row in inferences),
                          over_original_50ms_count=sum(row['total_seconds'] > .05 for row in inferences)),
        runtime_candidate_gates=dict(assessed=len(candidate_gates),
            passed=sum(gate['passed'] is True for gate in candidate_gates),
            failed=sum(gate['passed'] is False for gate in candidate_gates),
            failed_reason_counts=dict(Counter(gate['reason'] for gate in candidate_gates if not gate['passed']))),
        selected_candidate_histogram=dict(Counter(row['selected']['candidate_index'] for row in inferences)),
        first_10_capture_seconds_inference_census=dict(
            inference_count=len(early), candidate_1_count=sum(row['selected_candidate'] == 1 for row in early),
            tracker_brake_positive_count=sum(row['tracker_brake'] > 0 for row in early),
            mean_selected_raw_0p3_mps=statistics.mean(row['selected_raw_preview_0p3_mps'] for row in early),
            mean_selected_raw_2p0_mps=statistics.mean(row['selected_raw_2p0_mps'] for row in early),
            mean_tracker_throttle=statistics.mean(row['tracker_throttle'] for row in early)),
        first_candidate_1=next((row for row in preview_rows if row['selected_candidate'] == 1), None),
        first_selected_preview_at_most_0p05=next((row for row in preview_rows if
            row['selected_raw_preview_0p3_mps'] <= .05), None),
        raw_selected_preview=dict(
            observed_vx_above_1p0_preview_at_most_0p5=sum(
                row['observed_vx_mps'] > 1 and row['selected_raw_preview_0p3_mps'] <= .5 for row in preview_rows),
            observed_vx_above_0p5_preview_at_most_0p05=sum(
                row['observed_vx_mps'] > .5 and row['selected_raw_preview_0p3_mps'] <= .05 for row in preview_rows),
            selected_preview_at_most_0p05=sum(row['selected_raw_preview_0p3_mps'] <= .05 for row in preview_rows),
            first_5s_observed_vx_above_1p0_preview_at_most_0p5=sum(
                row['t_learned_s'] < 5 and row['observed_vx_mps'] > 1
                and row['selected_raw_preview_0p3_mps'] <= .5 for row in preview_rows)),
        moving_brake_over_0p05_inference_count=len(brake_rows),
        first_moving_brake_over_0p05=brake_rows[0] if brake_rows else None,
        first_moving_low_preview=next((row for row in preview_rows if
            row['observed_vx_mps'] > 1 and row['selected_raw_preview_0p3_mps'] <= .5), None),
        preview_rows=preview_rows,
        goal_reached=False, training_data_approved=False, vehicle_control_approved=False,
    )
    return result


def plot(results, bundles, output):
    """HH_261001 - Use recorded states and raw selected previews; no reconstructed counterfactual."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    colors = {'A': '#2176ae', 'B': '#d34e43', 'W1_history': '#519052'}
    names = {'A': 'A native W1-state replay', 'B': 'B directional-gradient replay',
             'W1_history': 'Historical W1 20 s'}
    fig, axes = plt.subplots(1, 2, figsize=(11, 7), gridspec_kw={'width_ratios': [1, 1.35]})
    route = bundles['A']['route']['route']
    initial = bundles['A']['states'][0]
    c, s = math.cos(initial['yaw']), math.sin(initial['yaw'])

    def centered(x, y):
        dx, dy = x - initial['x'], y - initial['y']
        return -(-s * dx + c * dy), c * dx + s * dy

    route_xy = np.asarray([centered(point['x'], point['y']) for point in route])
    axes[0].plot(route_xy[:, 0], route_xy[:, 1], '--', color='#555555', lw=1.25, label='Fixed route center')
    for label, bundle in bundles.items():
        track = np.asarray([centered(row['x'], row['y']) for row in bundle['states']])
        axes[0].plot(track[:, 0], track[:, 1], lw=1.9, color=colors[label], label=names[label])
        axes[0].scatter(track[-1, 0], track[-1, 1], s=30, color=colors[label])
    axes[0].set(xlim=(-2.3, 2.3), ylim=(-1, 33), xlabel='Right from starting vehicle (m)',
                ylabel='Forward from starting vehicle (m)', title='Observed full 20 s paths')
    axes[0].set_aspect('equal', adjustable='box')
    axes[0].grid(alpha=.2)
    axes[0].legend(fontsize=8, loc='upper left')
    for label, bundle in bundles.items():
        t0 = bundle['states'][0]['timestamp']
        times = [row['timestamp'] - t0 for row in bundle['states']]
        axes[1].plot(times, [abs(row['vx']) * 3.6 for row in bundle['states']],
                     lw=1.6, color=colors[label], label=names[label])
    axes[1].axhline(9, color='#777777', ls='--', lw=.8, label='9 km/h study target')
    axes[1].set(xlim=(0, 20), ylim=(-.2, 10), xlabel='Capture time (s)', ylabel='Observed speed (km/h)',
                title='Native 20 Hz measured speed')
    axes[1].grid(alpha=.2)
    axes[1].legend(fontsize=8, loc='upper right')
    fig.suptitle('HH_261001 | Same fixed Town07 route | one attempt per endpoint', fontsize=12)
    fig.tight_layout()
    fig.savefig(output / 'vehicle_centered_comparison.png', dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for label, bundle in bundles.items():
        t0 = bundle['states'][0]['timestamp']
        rows_ = results[label]['preview_rows']
        axes[0].plot([row['timestamp'] - t0 for row in bundle['states']],
                     [row['vx'] for row in bundle['states']], color=colors[label], lw=1.4, label=names[label])
        axes[1].plot([row['t_capture_s'] for row in rows_],
                     [row['selected_raw_preview_0p3_mps'] for row in rows_],
                     color=colors[label], lw=1.2, label=names[label])
        axes[2].plot([row['t_capture_s'] for row in rows_],
                     [row['tracker_brake'] for row in rows_], color=colors[label], lw=1.2, label=names[label])
    axes[0].set_ylabel('Measured signed vx (m/s)')
    axes[1].set_ylabel('Raw selected 0.3 s\npreview (m/s)')
    axes[2].set_ylabel('Tracker brake')
    axes[2].set_xlabel('Capture time (s)')
    axes[2].set_xlim(0, 20)
    axes[0].legend(fontsize=8, ncol=3, loc='upper center')
    for axis in axes:
        axis.grid(alpha=.2)
    fig.suptitle('Raw selected plan versus actual movement and commanded braking', fontsize=12)
    fig.tight_layout()
    fig.savefig(output / 'speed_preview_brake.png', dpi=150)
    plt.close(fig)


def camera_link(output, bundle, index, name='CAM_FRONT'):
    camera = bundle['cameras'][index]
    return (Path(__import__('os').path.relpath(bundle['episode'] / camera['images'][name], output)).as_posix(),
            camera['frame'])


def markdown(results, bundles, checks, output):
    def f(value, digits=2):
        return f'{value:.{digits}f}'

    lines = ['# HH_261001 — Town07 방향 경사 A/B 실제 주행 비교', '',
        '같은 TRAIN 직진 경로에서 미리 지정한 A 1회, B 1회를 각각 20초 실행했다. '
        'A는 W1과 모델 텐서가 같지만 새 물리 실행이다. 과거 W1 20초는 변동 폭을 보기 위한 별도 참고 기록이다.', '',
        '![동일 경로의 차체 시작점 기준 실제 궤적과 속도](vehicle_centered_comparison.png)', '',
        '![측정 속도, 선택된 원시 0.3초 속도 예측, 제동](speed_preview_brake.png)', '',
        '| 항목 | A 새 실행 | B 새 실행 | 과거 W1 20초 |',
        '| --- | ---: | ---: | ---: |']
    metrics = [
        ('상태 / 6카메라 묶음', lambda r: f"{r['native_states']} / {r['camera_bundles']}"),
        ('추론 / 학습 제어 ACK', lambda r: f"{r['distinct_inferences']} / {r['learned_commands_acknowledged']}"),
        ('순 경로 진행 (m)', lambda r: f(r['net_arc_progress_m'])),
        ('마지막 경로 위치 / 전체 길이 (m)', lambda r: f"{f(r['final_arc_m'])} / {f(r['route_length_m'])}"),
        ('실측 절대 속력 최소/중앙/최대 (m/s, 400상태)',
         lambda r: '/'.join(f(v) for v in r['speed_abs_mps_all_400'].values())),
        ('학습 ACK가 다음 상태에 연결된 365개 중 abs(vx)≤0.5m/s (초)',
         lambda r: f(r['stopped_time_learned_ack_365_s']['abs_vx_at_most_0p5'])),
        ('같은 365개 중 abs(vx)≤0.05m/s (초)',
         lambda r: f(r['stopped_time_learned_ack_365_s']['abs_vx_at_most_0p05'])),
        ('첫 실측 정지 시각 (초)', lambda r: f(r['first_observed_stop_after_exceeding_1mps']['t_capture_s'])),
        ('최저 20Hz 이산 vx 변화율 (m/s²)', lambda r: f(r['minimum_discrete_signed_vx_rate_mps2'])),
        ('최대 중심 경로 오차 (m)', lambda r: f(r['maximum_abs_route_cte_m'], 3)),
        ('추론 p95 (ms)', lambda r: f(r['inference_ms']['p95'])),
        ('50ms 추론 지연 초과 / 후보 runtime gate 실패',
         lambda r: f"{r['inference_ms']['over_original_50ms_count']} / {r['runtime_candidate_gates']['failed']}"),
        ('후보 runtime gate 통과 / 검사 (12×183)',
         lambda r: f"{r['runtime_candidate_gates']['passed']} / {r['runtime_candidate_gates']['assessed']}"),
        ('충돌 / 차선 침범 이벤트', lambda r: f"{r['collision_events']} / {r['lane_invasion_events']}"),
        ('차체 10cm inset 침범 (400)',
         lambda r: str(r['body_and_bounded_sweep']['sampled_body_inset_10cm_lane']['violation_count'])),
        ('bounded sweep 10cm inset 침범 (399)',
         lambda r: str(r['body_and_bounded_sweep']['previous_sweep_inset_10cm_lane']['violation_count'])),
        ('bounded sweep inset 최소 안쪽 여유 (m)',
         lambda r: f(r['body_and_bounded_sweep']['previous_sweep_inset_10cm_lane']['minimum_interior_clearance_m'], 3)),
    ]
    for title, value in metrics:
        lines.append('| ' + title + ' | ' + ' | '.join(value(results[k]) for k in ('A', 'B', 'W1_history')) + ' |')
    lines.extend(['',
        f"새 B−A 순 진행: {results['B']['net_arc_progress_m']-results['A']['net_arc_progress_m']:+.2f}m. "
        f"과거 W1 대비 A도 {results['A']['net_arc_progress_m']-results['W1_history']['net_arc_progress_m']:+.2f}m 차이가 난다. "
        '한 번씩의 물리 실행으로 학습 변경의 인과 효과를 단정할 수 없다.', '',
        '### 원시 예측과 초반 제동', '',
        '원시 선택 후보의 0.3초 속도는 추론 183회당 한 번만 집계했다. '
        '실제 속도는 같은 추론 프레임의 차량 상태다. 제동은 제어기 명령이며 실측 감속과 구분한다.', '',
        '| 측정값 | A | B | 과거 W1 |', '| --- | ---: | ---: | ---: |'])
    for title, fn in [
        ('후보 1 첫 선택: 캡처 시각 (초)', lambda r: f(r['first_candidate_1']['t_capture_s'])),
        ('초반 10초 선택 0.3초 속도 평균 (m/s)',
         lambda r: f(r['first_10_capture_seconds_inference_census']['mean_selected_raw_0p3_mps'], 3)),
        ('초반 10초 선택 2.0초 속도 평균 (m/s)',
         lambda r: f(r['first_10_capture_seconds_inference_census']['mean_selected_raw_2p0_mps'], 3)),
        ('초반 10초 평균 스로틀',
         lambda r: f(r['first_10_capture_seconds_inference_census']['mean_tracker_throttle'], 3)),
        ('실측 vx>1.0인데 선택 예측≤0.5m/s', lambda r: r['raw_selected_preview']['observed_vx_above_1p0_preview_at_most_0p5']),
        ('그중 첫 학습 5초', lambda r: r['raw_selected_preview']['first_5s_observed_vx_above_1p0_preview_at_most_0p5']),
        ('실측 vx>1.0에서 제동>0.05 추론', lambda r: r['moving_brake_over_0p05_inference_count']),
        ('첫 해당 제동: 캡처 시각 (초)', lambda r: None if r['first_moving_brake_over_0p05'] is None else f(r['first_moving_brake_over_0p05']['t_capture_s'])),
        ('첫 선택 0.3초 속도≤0.05: 시각 (초)',
         lambda r: f(r['first_selected_preview_at_most_0p05']['t_capture_s'])),
    ]:
        lines.append('| ' + title + ' | ' + ' | '.join(str(fn(results[k])) for k in ('A', 'B', 'W1_history')) + ' |')
    lines.extend(['',
        '첫 제동 사건과 선택 예측의 방향은 사건별로 다를 수 있다. '
        '선택 후보 번호만으로 정지 뜻을 부여하지 않았고, 미선택 후보를 실제 주행으로 해석하지 않았다.', '',
        '초반 10초(각 83회 추론)의 후보1 선택은 A/B/과거 W1 각각 '
        + '/'.join(str(results[k]['first_10_capture_seconds_inference_census']['candidate_1_count'])
                   for k in ('A', 'B', 'W1_history')) + '회다. 같은 구간 실제 양의 제동 명령은 '
        + '/'.join(str(results[k]['first_10_capture_seconds_inference_census']['tracker_brake_positive_count'])
                   for k in ('A', 'B', 'W1_history')) + '회다. '
        'B의 초반 진행 감소를 실제 제동 명령으로 설명할 수 없다. '
        '선택 후보의 낮은 목표속도와 스로틀이 관측되지만 이 단일 물리 시험만으로 원인을 확정하지 않는다.', '',
        '### 원본과 결과 해석', '',
        f"A/B 동일 경로 해시: `{checks['route_sha256']}`; 활성 OpenDRIVE 해시: `{checks['active_map_sha256']}`. "
        f"A/B 각 95개 실행 소스 보관본의 핀 일치: `{checks['runtime_sources_equal']}`. "
        f"고정 이중 export binding 해시: `{BINDING_SHA}`. "
        'A의 모델 텐서 해시는 과거 W1과 정확히 같다.', '',
        '두 실행 모두 원래 수집기 상태는 `failed`이고 소유자 결과는 `DIAGNOSTIC_BUDGET_REACHED`다. '
        '400상태/200카메라를 완전히 보관했으나 목표 경로는 완주하지 못했다. '
        '소유 프로세스·포트·소스 정리는 두 팔 모두 통과했다. '
        '학습 자료 편입, 모델 승격, 실차 제어 승인은 없다.', '',
        'A/B에서 최저 이산 vx 변화율은 각각 약 −18.56/−18.53m/s²다. '
        '이는 인접 20Hz 상태의 속도 차분이며 별도 승차감 기준을 통과했다는 뜻이 아니다. '
        '초반 둔화와 급정지 순간의 원인을 제동 명령 하나로 환원하지 않는다.', '',
        '차체 검사는 보관된 actor bounding box와 활성 OpenDRIVE를 이용한 20Hz 정적 다각형 및 '
        '두 상태 사이 bounded snapshot-linear sweep의 10cm 안쪽 차로 검사다. '
        '연속 시간의 실제 물리 충돌 부재를 증명하지는 않는다. '
        '충돌/차선 이벤트 0개와 차체 여유는 별도 근거다.', '',
        '원본 전면 카메라 JPEG (복사하지 않은 보관 파일):', ''])
    for label in ('A', 'B', 'W1_history'):
        entries = []
        for index, caption in ((0, '시작'), (60, '중간'), (199, '마지막')):
            path, frame = camera_link(output, bundles[label], index)
            entries.append(f'[{caption} frame {frame}]({path})')
        lines.append(f"- {label}: " + ' · '.join(entries))
    lines.extend(['', '전체 원본: `comparison.json`의 각 run 경로에서 `episode.partial/`을 확인한다. '
                  '읽기 전용 집계이며 시뮬레이터 실행, 새 추론 또는 제어 변경이 없다.', ''])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink(), 'fresh output directory required')
    runs = DEFAULT_RUNS
    pair = read(runs['B'] / 'paired_attempt_receipt.json')
    require(pair['status'] == 'BOTH_FIXED_ATTEMPTS_RETAINED_NOT_PROMOTED'
            and pair['two_full_20_second_captures'] is True and pair['both_cleanup_ok'] is True
            and pair['fixed_arm_order'] == ['A', 'B'] and pair['binding_sha256'] == BINDING_SHA
            and pair['replay_plan_sha256'] == PLAN_SHA and pair['model_promoted'] is False,
            'completed fixed pair receipt required')
    bundles = {label: validate_record(label, run) for label, run in runs.items()}
    for label in ('A', 'B'):
        receipt = pair['arms'][label]
        bundle = bundles[label]
        require(receipt['checkpoint_sha256'] == EXPECTED[label][0]
                and receipt['owner_plan_sha256'] == digest(bundle['run'] / 'owner_plan.json')
                and receipt['owner_diagnostic_sha256'] == digest(bundle['run'] / 'owner_diagnostic_result.json')
                and receipt['state_count'] == 400 and receipt['camera_count'] == 200
                and receipt['cleanup_ok'] is True, f'{label}: pair receipt/owner evidence differs')
    require(bundles['A']['plan']['source_sha256'] == bundles['B']['plan']['source_sha256']
            and bundles['A']['plan']['source_pins'] == bundles['B']['plan']['source_pins'],
            'A/B frozen runtime source chain differs')
    route_hashes = {b['source_hashes']['route.json'] for b in bundles.values()}
    map_hashes = {b['source_hashes']['active_map_opendrive.xodr'] for b in bundles.values()}
    require(len(route_hashes) == len(map_hashes) == 1, 'A/B/W1 route/map identity differs')
    capture_fields = ('physics_hz', 'camera_hz', 'maximum_total_duration_sec', 'target_speed_kmh',
                      'camera_order', 'seed', 'spawn_z_offset_m', 'actor_body_geometry')
    for field in capture_fields:
        require(bundles['A']['capture'][field] == bundles['B']['capture'][field],
                f'A/B capture {field} differs')
    summaries = {label: summarize(bundle) for label, bundle in bundles.items()}
    checks = dict(route_sha256=next(iter(route_hashes)), active_map_sha256=next(iter(map_hashes)),
                  runtime_sources_equal=True, A_B_capture_contract_fields_equal=list(capture_fields),
                  A_model_state_equals_historical_W1=True, fixed_arm_order=['A', 'B'],
                  no_favorable_arm_selection=True, all_400_200_denominators_verified=True)
    report = dict(schema='portable_e2e.directional_town07_live_pair_readonly.v1',
                  status='BOTH_FIXED_ATTEMPTS_RECOUNTED_NOT_PROMOTED',
                  pair_receipt_sha256=digest(runs['B'] / 'paired_attempt_receipt.json'),
                  checks=checks, trials=summaries,
                  B_minus_A_net_arc_progress_m=summaries['B']['net_arc_progress_m']-summaries['A']['net_arc_progress_m'],
                  limitations=['One new physical attempt per endpoint on a known TRAIN route.',
                               'A and historical W1 have identical model tensors but separate simulator runs.',
                               'Selected raw preview is a model hypothesis, not observed future motion.',
                               'ACK proves server acceptance and state control matching, not independent actuation measurement.',
                               'Bounded snapshot-linear body sweep is not continuous-time physical safety proof.'],
                  new_neural_inferences=0, simulator_launched=False, training_data_approved=False,
                  model_promoted=False, vehicle_control_approved=False)
    args.output.mkdir(parents=True)
    plot(summaries, bundles, args.output)
    (args.output / 'comparison.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n')
    (args.output / 'README.md').write_text(markdown(summaries, bundles, checks, args.output))
    size = sum(path.stat().st_size for path in args.output.iterdir() if path.is_file())
    require(size <= 10 * 1024 * 1024, 'report exceeds 10 MiB output budget')
    print(json.dumps(dict(output=str(args.output), bytes=size, checks=checks,
        summary={label: {key: result[key] for key in ('net_arc_progress_m', 'final_arc_m',
            'stopped_time_learned_ack_365_s', 'moving_brake_over_0p05_inference_count', 'inference_ms')}
            for label, result in summaries.items()}), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
