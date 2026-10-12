"""HH_261001 - Fixed149 exact W1 reconstruction and W2 same-input counterfactual; never a live rollout."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

import numpy as np
import torch
from portable_e2e.contract import ContractError
from portable_e2e import simulation_research as research
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
from scripts.e2e import transport_warmup_duration_w2_replay_20261001 as transport
from scripts.e2e.replay_exact_warmup_inputs_20261001 import load_snapshot, read_rows
from scripts.e2e.capture_exact_simulation_inputs_20261001 import output_digest

replay = transport.replay
ROOT = replay.ROOT
OUTPUT = ROOT / 'docs/assets/validation/2026-10-01/warmup_supervision_ab/w2_fixed149_replay_v1'
PLAN = OUTPUT / 'PROTOCOL.md'
PLAN_SHA = '1e2dc295e8f850ed4af7abb673ef118c7d66ebb784ad51cf990738980a061e91'
TRIAL = ROOT / 'storage/experiments/validation/2026-10-01/warmup_supervision_W1_town07_long_v1'
ARCHIVE = TRIAL / 'sampled_exact_runtime_inputs'
EPISODE = TRIAL / 'episode.partial'
EXPORTS = ROOT / 'storage/experiments/training/2026-10-01/ah_warmup_long_w2_20261001_v1/exports'
BINDING, RECEIPT = EXPORTS / 'local_endpoint_binding.json', EXPORTS / 'path_transport_receipt.json'
BINDING_SHA = 'dde8b7f3045f51d6e5b057ee9991db1a280690995a01fe49bf9df2f29140e7b9'
RECEIPT_SHA = '9bf27466d2456d0ca454385516f77ba6d74e46d0c04a0dc7291f785521d797aa'
PINS = {
    PLAN: PLAN_SHA,
    ARCHIVE / 'index.jsonl': '951fd7b4f79e6da8e12e5f4af5c82ebcba5b1e1e6dc067e568c23af2d62db8bb',
    EPISODE / 'states.jsonl': 'c3d92ae3ec97ab2914236093275d950356afb04f81475ac4a91e463764397058',
    EPISODE / 'learned_control.jsonl': '3c50214fa5da16898221a80ce6208cce89612060c6618174eaa69a02c9895105',
    ROOT / 'tools/e2e/replay_exact_warmup_inputs_20261001.py': 'c3809dea76bd5f5b478d0d2fdcbb68c49162a5825a72b8973e32563a63e531a7',
    ROOT / 'tools/e2e/capture_exact_simulation_inputs_20261001.py': 'e1b5a01fbed81814d820157b737474e114b05d76bc1d9b062989404f69bc9d2d',
    ROOT / 'learning/portable_e2e/runtime_contract.py': '38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3',
    ROOT / 'learning/portable_e2e/model.py': 'fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e',
    ROOT / 'learning/portable_e2e/simulation_research.py': '4a66e2d3b522272822ddb43c43edeed1ebc6a2ee6c20bd0cda6ce2b3ea6c9252',
    BINDING: BINDING_SHA, RECEIPT: RECEIPT_SHA,
}
CONTROLLER = dict(wheelbase_m=2.819186218917395, maximum_steer_angle_rad=1.2217303432379762,
    speed_cap_mps=2.5, minimum_lookahead_m=1.5, lookahead_time_s=.8, speed_preview_s=.3,
    throttle_gain=.7, brake_gain=.7, maximum_throttle=.5)
MAX_GENERATED_BYTES = 10 * 1024**2


def prefix_metrics(xy, speed, points):
    replay.require(points in (3,20,64) and len(speed) == len(xy) == 64, 'fixed64 output and3/20/64 prefixes required')
    selected = speed[:points]
    previous = (0.,0.)
    path_length = 0.
    for point in xy[:points]:
        path_length += math.dist(previous, point)
        previous = point
    return dict(speed_min_mps=min(selected), speed_max_mps=max(selected),
        speed_mean_mps=statistics.fmean(selected), last_speed_mps=selected[-1],
        exact_zero_speed_points=sum(v == 0 for v in selected), endpoint_xy_m=list(xy[points-1]),
        path_length_m=path_length, maximum_abs_y_m=max(abs(p[1]) for p in xy[:points]))


def gate_and_control(raw, current):
    xy, speed, logits = (raw[name] for name in ('candidate_xy','candidate_speed','candidate_logits'))
    replay.require(len(logits) == len(xy) == len(speed) == 12, 'exact12 candidate ABI required')
    raw_winner = max(range(12), key=logits.__getitem__)
    gates = []
    for index in range(12):
        try:
            validate_and_select_trajectory([xy[index]],[speed[index]],[logits[index]],
                RuntimeGateConfig(candidate_count=1),current_speed_mps=current)
            gates.append(dict(index=index,passed=True,reason=None))
        except ContractError as error:
            gates.append(dict(index=index,passed=False,reason=str(error)))
    selected, reason = None, None
    try:
        selected = validate_and_select_trajectory(xy,speed,logits,RuntimeGateConfig(candidate_count=12),current_speed_mps=current)
        control = research.trajectory_control(selected,current,research.ControllerConfig(**CONTROLLER))
    except ContractError as error:
        reason = str(error)
        control = dict(throttle=0.,brake=1.,steer=0.,reverse=False,hand_brake=False)
    summary = dict(raw_output_sha256=output_digest(raw), raw_argmax=raw_winner,
        raw_argmax_mode='DRIVE' if raw_winner < 6 else 'STOP', logits=logits,
        selected_trajectory_gate_pass=selected is not None, rejection_reason=reason,
        candidate_gates=gates, selected_speed_mps=speed[raw_winner], selected_xy_m=xy[raw_winner],
        prefixes={str(n):prefix_metrics(xy[raw_winner],speed[raw_winner],n) for n in (3,20,64)},
        raw_controller_preview_speed_mps=speed[raw_winner][2],
        tracker_proposal=control, counterfactual_control_acknowledged=False,
        all_candidate_preview_speeds_mps=[s[2] for s in speed])
    return summary, selected


def verify_w1_actual(summary, selected, decision):
    replay.require(summary['candidate_gates'] == decision['candidate_gates'], 'W1 actual candidate gates differ')
    replay.require(output_digest(asdict(selected) if selected else None) == output_digest(decision['selected']),
                   'W1 actual selected trajectory differs')
    if decision['control_source'] == 'learned_trajectory_tracker':
        replay.require(output_digest(summary['tracker_proposal']) == output_digest(decision['control']),
                       'W1 actual learned tracker control differs')
        return True
    return False


def summarize(rows):
    arms = {}
    for arm in ('W1','W2'):
        values = [row[arm] for row in rows]
        arms[arm] = dict(raw_argmax_counts=dict(Counter(str(v['raw_argmax']) for v in values)),
            raw_mode_counts=dict(Counter(v['raw_argmax_mode'] for v in values)),
            selected_gate_pass_count=sum(v['selected_trajectory_gate_pass'] for v in values),
            all_candidate_gate_pass_count=sum(g['passed'] for v in values for g in v['candidate_gates']),
            selected_gate_failures=dict(Counter(v['rejection_reason'] for v in values if v['rejection_reason'])),
            exact_zero_preview_count=sum(v['raw_controller_preview_speed_mps'] == 0 for v in values),
            preview_at_or_below_hold_threshold_count=sum(v['raw_controller_preview_speed_mps'] <= .05 for v in values),
            preview_mean_mps=statistics.fmean(v['raw_controller_preview_speed_mps'] for v in values),
            prefixes={str(n):dict(mean_selected_speed_mps=statistics.fmean(v['prefixes'][str(n)]['speed_mean_mps'] for v in values),
                mean_selected_path_length_m=statistics.fmean(v['prefixes'][str(n)]['path_length_m'] for v in values),
                mean_selected_max_abs_y_m=statistics.fmean(v['prefixes'][str(n)]['maximum_abs_y_m'] for v in values)) for n in (3,20,64)},
            throttle_positive_proposals=sum(v['tracker_proposal']['throttle'] > 0 for v in values),
            brake_positive_proposals=sum(v['tracker_proposal']['brake'] > 0 for v in values))
    zero = [r for r in rows if r['W1']['raw_controller_preview_speed_mps'] == 0]
    stationary = [r for r in rows if abs(r['observed_W1_vx_mps']) <= .1]
    return dict(arms=arms, changed_argmax_count=sum(r['W1']['raw_argmax'] != r['W2']['raw_argmax'] for r in rows),
        mode_transition_counts=dict(Counter(r['W1']['raw_argmax_mode']+'->'+r['W2']['raw_argmax_mode'] for r in rows)),
        exact_W1_zero_preview_subset=dict(count=len(zero),
            W2_still_exact_zero_count=sum(r['W2']['raw_controller_preview_speed_mps'] == 0 for r in zero),
            W2_above_hold_threshold_and_gate_pass_count=sum(r['W2']['raw_controller_preview_speed_mps'] > .05
                and r['W2']['selected_trajectory_gate_pass'] for r in zero)),
        near_stationary_W1_state_subset=dict(count=len(stationary),
            W1_zero_preview_count=sum(r['W1']['raw_controller_preview_speed_mps'] == 0 for r in stationary),
            W2_zero_preview_count=sum(r['W2']['raw_controller_preview_speed_mps'] == 0 for r in stationary)),
        mean_signed_W2_minus_W1_preview_mps=statistics.fmean(r['W2']['raw_controller_preview_speed_mps']
            -r['W1']['raw_controller_preview_speed_mps'] for r in rows))


def run():
    replay.require(os.environ.get('CUDA_VISIBLE_DEVICES') == '' and all(os.environ.get(k) == '2'
        for k in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS')), 'explicit CPU/three two-thread environment required')
    for path, sha in PINS.items():
        replay.require(replay.digest(path) == sha, 'fixed counterfactual source/evidence differs: '+str(path))
    for filename in ('records.jsonl','summary.json','failure.json'):
        replay.require(not (OUTPUT/filename).exists(), 'new counterfactual evidence required')
    torch.set_num_threads(2)
    transport.verify_transport(BINDING,BINDING_SHA,RECEIPT,RECEIPT_SHA)
    binding = replay.binding_read(BINDING,BINDING_SHA)
    models = {arm:replay.load_endpoint(arm,replay.endpoint(arm,binding)) for arm in ('W1','W2')}
    journal = read_rows(ARCHIVE/'index.jsonl')
    replay.require(len(journal) == 149, 'complete fixed149 archive required')
    states = {row['frame']:row for row in read_rows(EPISODE/'states.jsonl')}
    published = {}
    for decision in read_rows(EPISODE/'learned_control.jsonl'):
        if decision.get('raw_output') is None or decision.get('held_between_camera_frames'):
            continue
        stamp = round(states[decision['command_frame']]['timestamp']*1e9)
        replay.require(stamp not in published, 'ambiguous original publication timestamp')
        published[stamp] = decision
    imported = {Path(m.__file__).resolve() for m in list(sys.modules.values()) if getattr(m,'__file__',None)
        and Path(m.__file__).is_file() and Path(m.__file__).resolve().is_relative_to(ROOT)}
    before = {str(path):replay.digest(path) for path in imported | set(PINS) | {Path(__file__),
        ROOT/'learning/tests/test_w2_fixed149_counterfactual_20261001.py'}}
    before.update({str(Path(binding[key])):binding[key+'_sha256'] for key in ('weights','metadata')})
    before.update({str(replay.W1_EXPORT):replay.w1.WEIGHTS_SHA,
                   str(replay.W1_EXPORT.with_suffix('.json')):replay.w1.METADATA_SHA})
    rows, generated, started = [], 0, time.monotonic()
    try:
        with (OUTPUT/'records.jsonl').open('x') as stream, torch.no_grad():
            for sequence, entry in enumerate(journal):
                replay.require(entry['sequence'] == sequence and entry['source_checkpoint_sha256'] == replay.w1.CHECKPOINT,
                               'fixed W1 archive sequence/identity differs')
                decision = published[entry['timing']['now_ns']]
                replay.require(decision['source_checkpoint_sha256'] == replay.w1.CHECKPOINT
                    and output_digest(decision['raw_output']) == entry['raw_output_sha256']
                    and decision['controller'] == CONTROLLER, 'published W1 output/controller differs')
                arrays = load_snapshot(ARCHIVE,entry)
                state = states[decision['inference_frame']]
                current = float(state['vx'])
                replay.require(np.float32(current) == arrays['ego_history'][0,-1,1], 'archived/model and actual current speeds disagree')
                inputs = [torch.from_numpy(arrays[key]) for key in research.FIELDS]
                results, actual_control_match = {}, False
                for arm in ('W1','W2'):
                    xy, speed, logits = models[arm](*inputs)
                    raw = dict(candidate_xy=xy[0].tolist(),candidate_speed=speed[0].tolist(),candidate_logits=logits[0].tolist())
                    if arm == 'W1':
                        replay.require(output_digest(raw) == entry['raw_output_sha256'], 'W1 raw output is not bit-exact')
                    results[arm], selected = gate_and_control(raw,current)
                    if arm == 'W1':
                        actual_control_match = verify_w1_actual(results[arm],selected,decision)
                row = dict(sequence=sequence,inference_frame=decision['inference_frame'],timestamp_ns=entry['timing']['now_ns'],
                    tensor_file=entry['tensor_file'],tensor_file_sha256=entry['tensor_file_sha256'],
                    observed_W1_vx_mps=current,observed_W1_progress_m=state['route_progress_m'],
                    W1_bit_exact=True,W1_actual_tracker_reconstructed=actual_control_match,
                    actual_W1_control_source=decision['control_source'],actual_W1_application=decision.get('application'),
                    W2_has_new_actor_state=False,W2_actuator_acknowledged=False,**results)
                encoded=json.dumps(row,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n'
                generated += len(encoded.encode())
                replay.require(generated < MAX_GENERATED_BYTES-1024**2, 'bounded output limit exceeded; failure retained')
                stream.write(encoded);stream.flush();rows.append(row)
                if (sequence+1)%25 == 0:
                    print(json.dumps(dict(completed=sequence+1,total=149,W1_bit_exact=True)),flush=True)
        replay.require(before == {path:replay.digest(Path(path)) for path in before}, 'source/endpoint/evidence changed')
        for entry in journal:
            replay.require(replay.digest(ARCHIVE/entry['tensor_file']) == entry['tensor_file_sha256'], 'snapshot changed during replay')
        result = dict(schema='portable_e2e.w2_fixed149_counterfactual.v1',status='PASS_OFFLINE_COUNTERFACTUAL_ONLY',
            frames=149,W1_exact_raw_reconstructions=sum(r['W1_bit_exact'] for r in rows),
            W1_exact_tracker_reconstructions=sum(r['W1_actual_tracker_reconstructed'] for r in rows),
            source_sha256=before,binding_sha256=BINDING_SHA,transport_receipt_sha256=RECEIPT_SHA,
            W1_checkpoint_sha256=replay.w1.CHECKPOINT,W2_checkpoint_sha256=binding['checkpoint_sha256'],
            W2_model_state_sha256=binding['model_state_sha256'],controller=CONTROLLER,
            trajectory_gate=asdict(RuntimeGateConfig(candidate_count=12)),device='cpu',threads=2,
            records_sha256=replay.digest(OUTPUT/'records.jsonl'),records_bytes=generated,elapsed_wall_s=time.monotonic()-started,
            optimizer_updates=0,simulator_launches=0,SSH_calls=0,W2_live_driving=False,new_physical_body_sweeps=False,
            live_deadlines_tested=False,training_data_approved=False,model_promoted=False,
            interpretation='W2 inference on fixed observed W1 states only; output change is not driving improvement, safety, comfort or generalization proof.',
            **summarize(rows))
        replay.write_new(OUTPUT/'summary.json',result)
        replay.require(sum(p.stat().st_size for p in OUTPUT.iterdir() if p.is_file()) < MAX_GENERATED_BYTES, 'total generated evidence exceeded10MiB')
        print(json.dumps({k:result[k] for k in ('status','frames','W1_exact_raw_reconstructions','W1_exact_tracker_reconstructions',
            'arms','changed_argmax_count','exact_W1_zero_preview_subset','near_stationary_W1_state_subset','records_bytes')},indent=2),flush=True)
        return result
    except BaseException as error:
        replay.write_new(OUTPUT/'failure.json',dict(status='FAIL_RETAINED',error=f'{type(error).__name__}: {error}',
            completed_rows=len(rows),source_sha256=before,optimizer_updates=0,simulator_launches=0))
        raise


def main():
    argparse.ArgumentParser(description=__doc__,allow_abbrev=False).parse_args()
    run()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
