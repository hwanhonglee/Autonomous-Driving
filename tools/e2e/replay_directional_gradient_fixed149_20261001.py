"""HH_261001 - Released directional A/B endpoints on fixed149 W1 inputs, never a physical rollout."""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import time

import numpy as np
import torch
from portable_e2e import simulation_research as research
from scripts.e2e import replay_w2_fixed149_counterfactual_20261001 as prior
from scripts.e2e import report_directional_lower_gradient_ab_20261001 as audit

ROOT=audit.ROOT
OUTPUT=ROOT/'docs/assets/validation/2026-10-01/warmup_supervision_ab/directional_gradient_fixed149_v1'
PLAN=OUTPUT/'PROTOCOL.md'
PLAN_SHA='919f42ed1d9fd68f9658eac971470bd64148fa2556b5a5b3a3409450ff174aec'
PINS={path:sha for path,sha in prior.PINS.items() if path not in (prior.BINDING,prior.RECEIPT,prior.PLAN)}
PINS.update({PLAN:PLAN_SHA,Path(prior.__file__):'436ad5c7b7e20d2ca6722e8ddc54cbaaa9a9b536a142922b1bb6e439cfbcc479',
    Path(audit.__file__):'02a52a6267484302c8743ca6b1db3a9631e593039029568f10e1a5d3c218221b'})


@contextmanager
def endpoint_identity(sha):
    prior.replay.exact_sha(sha)
    old=research.AH_CHECKPOINT_SHA256,research.CHECKPOINT_SCHEMA
    try:
        research.AH_CHECKPOINT_SHA256,research.CHECKPOINT_SCHEMA=sha,audit.SCHEMA
        yield
    finally:
        research.AH_CHECKPOINT_SHA256,research.CHECKPOINT_SCHEMA=old


def load_released(args,ledger):
    audit.require(audit.sha(args.binding)==args.binding_sha256,'explicit final dual-export binding SHA required')
    binding=audit.read(args.binding,ledger)
    complete=audit.read(args.training/'both_final_checkpoints_complete.json',ledger)
    summary=audit.read(args.training/'summary.json',ledger)
    audit.require(summary.get('status')=='COMPLETE_NOT_PROMOTED' and summary.get('schema')==audit.SCHEMA
        and summary.get('training')==complete and summary.get('source_hashes_match') is True,
        'completed released pair required; no partial fit mode')
    audit.validate_export_binding(binding,complete)
    audit.require(len(binding['evidence_file_sha256'])==42,'full42-file restricted-export proof required')
    for name,expected in binding['evidence_file_sha256'].items():
        path=audit.evidence_relative(args.training,name)
        audit.require(audit.sha(path)==expected,'restricted-export evidence bytes differ')
        ledger[str(path)]=expected
    models={}
    for label,arm in zip(audit.LABELS,audit.ARMS):
        endpoint=binding['endpoints'][arm]
        audit.require(not (args.training/arm/'failure.json').exists(),'failed endpoint is not eligible')
        weights=audit.evidence_relative(args.training,endpoint['weights'])
        metadata_path=audit.evidence_relative(args.training,endpoint['metadata'])
        metadata=audit.read(metadata_path,ledger)
        audit.require(ledger[str(metadata_path)]==endpoint['metadata_sha256']
            and audit.original.npz_state_digest(weights,metadata)==endpoint['model_state_sha256'],
            'independent numeric model identity differs')
        ledger[str(weights)]=audit.sha(weights)
        with endpoint_identity(endpoint['checkpoint_sha256']):
            models[label]=research.load_ah_weights(weights,metadata_path,
                weights_sha256=endpoint['weights_sha256'],metadata_sha256=endpoint['metadata_sha256'])
    return binding,models


def summarize(rows):
    aliases=[dict(W1=r['A'],W2=r['B'],observed_W1_vx_mps=r['historical_W1_vx_mps']) for r in rows]
    raw=prior.summarize(aliases)
    resolved=[r for r in rows if r['A']['raw_controller_preview_speed_mps']==0
        and r['B']['raw_controller_preview_speed_mps']>.05 and r['B']['selected_trajectory_gate_pass']]
    return dict(arms={'A':raw['arms']['W1'],'B':raw['arms']['W2']},
        changed_argmax_count=raw['changed_argmax_count'],mode_transition_counts=raw['mode_transition_counts'],
        exact_A_zero_preview_subset=audit.relabel(raw['exact_W1_zero_preview_subset']),
        near_stationary_historical_W1_state_subset=audit.relabel(raw['near_stationary_W1_state_subset']),
        mean_signed_B_minus_A_preview_mps=raw['mean_signed_W2_minus_W1_preview_mps'],
        resolved_zero_previews=dict(count=len(resolved),
            raw_argmax_changed=sum(r['A']['raw_argmax']!=r['B']['raw_argmax'] for r in resolved),
            old_A_candidate_index_still_exact_zero_under_B=sum(r['B']['all_candidate_preview_speeds_mps'][r['A']['raw_argmax']]==0 for r in resolved)),
        actual_B_control_ACK_count=0,new_simulator_states=0)


def run(args):
    audit.require(os.environ.get('CUDA_VISIBLE_DEVICES')=='' and all(os.environ.get(k)=='2'
        for k in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS')),'explicit CPU/two-thread environment required')
    for path,sha in PINS.items():
        audit.require(audit.sha(path)==sha,'pinned replay source/input/protocol differs')
    for name in ('records.jsonl','summary.json','failure.json'):
        audit.require(not (OUTPUT/name).exists(),'fresh fixed149 directional evidence required')
    torch.set_num_threads(2)
    ledger={str(path):sha for path,sha in PINS.items()}
    ledger[str(Path(__file__))]=audit.sha(Path(__file__))
    test=ROOT/'learning/tests/test_replay_directional_gradient_fixed149_20261001.py'
    ledger[str(test)]=audit.sha(test)
    binding,models=load_released(args,ledger)
    journal=prior.read_rows(prior.ARCHIVE/'index.jsonl')
    audit.require(len(journal)==149,'all fixed149 snapshots required')
    states={r['frame']:r for r in prior.read_rows(prior.EPISODE/'states.jsonl')}
    published={}
    for row in prior.read_rows(prior.EPISODE/'learned_control.jsonl'):
        if row.get('raw_output') is None or row.get('held_between_camera_frames'):continue
        stamp=round(states[row['command_frame']]['timestamp']*1e9)
        audit.require(stamp not in published,'ambiguous original publication stamp')
        published[stamp]=row
    rows=[];generated=0;started=time.monotonic()
    try:
        with (OUTPUT/'records.jsonl').open('x') as stream,torch.no_grad():
            for sequence,entry in enumerate(journal):
                audit.require(entry['sequence']==sequence and entry['source_checkpoint_sha256']==prior.replay.w1.CHECKPOINT,
                    'exact W1 snapshot source/sequence required')
                decision=published[entry['timing']['now_ns']]
                audit.require(prior.output_digest(decision['raw_output'])==entry['raw_output_sha256']
                    and decision['controller']==prior.CONTROLLER,'original output/controller evidence differs')
                arrays=prior.load_snapshot(prior.ARCHIVE,entry)
                state=states[decision['inference_frame']];current=float(state['vx'])
                audit.require(np.float32(current)==arrays['ego_history'][0,-1,1],'same archived/model current speed required')
                inputs=[torch.from_numpy(arrays[name]) for name in research.FIELDS]
                results={};exact_control=False
                for label in ('A','B'):
                    xy,speed,logits=models[label](*inputs)
                    raw=dict(candidate_xy=xy[0].tolist(),candidate_speed=speed[0].tolist(),candidate_logits=logits[0].tolist())
                    if label=='A':
                        audit.require(prior.output_digest(raw)==entry['raw_output_sha256'],'A must bit-reproduce historical W1')
                    results[label],selected=prior.gate_and_control(raw,current)
                    if label=='A':exact_control=prior.verify_w1_actual(results[label],selected,decision)
                row=dict(sequence=sequence,historical_W1_inference_frame=decision['inference_frame'],
                    timestamp_ns=entry['timing']['now_ns'],tensor_file=entry['tensor_file'],tensor_file_sha256=entry['tensor_file_sha256'],
                    historical_W1_vx_mps=current,historical_W1_progress_m=state['route_progress_m'],
                    A_bit_exact_to_historical_W1=True,historical_W1_tracker_reconstructed=exact_control,
                    actual_historical_W1_application=decision.get('application'),new_A_or_B_control_ACK=False,**results)
                encoded=json.dumps(row,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n'
                generated+=len(encoded.encode())
                audit.require(generated<prior.MAX_GENERATED_BYTES-1024**2,'bounded evidence size exhausted')
                stream.write(encoded);stream.flush();rows.append(row)
                if (sequence+1)%25==0:print(json.dumps(dict(completed=sequence+1,total=149,A_exact=True)),flush=True)
        audit.require(all(audit.sha(Path(path))==sha for path,sha in ledger.items()),'replay source or released evidence changed')
        for entry in journal:
            audit.require(audit.sha(prior.ARCHIVE/entry['tensor_file'])==entry['tensor_file_sha256'],'snapshot changed during replay')
        result=dict(schema='portable_e2e.directional_gradient_fixed149_counterfactual.v1',
            status='PASS_OFFLINE_COUNTERFACTUAL_ONLY',frames=149,
            A_exact_historical_W1_raw_reconstructions=sum(r['A_bit_exact_to_historical_W1'] for r in rows),
            exact_historical_W1_tracker_reconstructions=sum(r['historical_W1_tracker_reconstructed'] for r in rows),
            source_sha256=ledger,export_binding_sha256=args.binding_sha256,
            endpoints=binding['endpoints'],records_sha256=audit.sha(OUTPUT/'records.jsonl'),records_bytes=generated,
            elapsed_wall_s=time.monotonic()-started,device='cpu',threads=2,optimizer_updates=0,simulator_launches=0,
            live_A_or_B_driving=False,physical_body_sweeps=False,live_deadline_tested=False,
            model_promoted=False,training_data_approved=False,
            interpretation='Both new endpoints on historical W1 states only. No new actor state, actuator ACK or rollout improvement proof.',
            **summarize(rows))
        prior.replay.write_new(OUTPUT/'summary.json',result)
        audit.require(sum(p.stat().st_size for p in OUTPUT.iterdir() if p.is_file())<prior.MAX_GENERATED_BYTES,'output exceeded10MiB')
        print(json.dumps({k:result[k] for k in ('status','frames','A_exact_historical_W1_raw_reconstructions',
            'arms','changed_argmax_count','resolved_zero_previews','records_bytes')},indent=2),flush=True)
        return result
    except BaseException as error:
        prior.replay.write_new(OUTPUT/'failure.json',dict(status='FAIL_RETAINED',completed_rows=len(rows),
            error=f'{type(error).__name__}: {error}',source_sha256=ledger,new_optimizer_steps=0,simulator_launches=0))
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--training',type=Path,required=True)
    parser.add_argument('--binding',type=Path,required=True)
    parser.add_argument('--binding-sha256',required=True)
    run(parser.parse_args())
    return 0


if __name__=='__main__':raise SystemExit(main())
