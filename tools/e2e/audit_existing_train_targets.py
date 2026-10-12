#!/usr/bin/env python3
"""HH_260906 - Audit explicitly selected TRAIN labels without opening held-out episode payloads."""
import argparse
from collections import Counter
import json
import math
import re
from pathlib import Path
from portable_e2e.contract import ContractError,sha256_file
from portable_e2e.runtime_contract import RuntimeGateConfig,validate_and_select_trajectory


def gate_failure_details(error):
    """HH_260906 - Read whitelisted numeric locals from the unchanged failing gate, not a reimplemented formula."""
    trace=error.__traceback__;details={}
    while trace is not None:
        frame=trace.tb_frame
        if frame.f_code is validate_and_select_trajectory.__code__:
            for name in ('index','speed_mps','speed_rate_mps2','step_m','geometric_speed_mps',
                         'heading_delta','curvature_path_m','implied_curvature_rad_per_m',
                         'lateral_speed_mps','lateral_speed_limit_mps'):
                value=frame.f_locals.get(name)
                if type(value) in (float,int) and math.isfinite(value):details[name]=value
        trace=trace.tb_next
    return details


def audit(root,ids):
    """HH_260906 - Authenticate selected episode and sample bytes and preserve all rejected targets."""
    document=json.loads((root/'dataset.json').read_text());refs={r['episode_id']:r for r in document['episodes']}
    if len(refs)!=len(document['episodes']) or len(ids)!=len(set(ids)):raise ValueError('duplicate episode identity')
    result=[]
    for identity in ids:
        if not identity.startswith(('carla_town01_','carla_town07_','carla_c_track_1_0_7_')):
            raise ValueError('episode not on active TRAIN map allowlist')
        ref=refs[identity];relative=Path(ref['manifest'])
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('unsafe episode path')
        ep=root/relative
        if ep.is_symlink() or sha256_file(ep)!=ref['sha256']:raise ValueError('episode checksum mismatch')
        meta=json.loads(ep.read_text())
        if meta['split']!='train':raise ValueError('selected episode is not TRAIN')
        sample=Path(meta['sample_jsonl'])
        if sample.is_absolute() or '..' in sample.parts:raise ValueError('unsafe sample path')
        sample=ep.parent/sample
        if sample.is_symlink() or sha256_file(sample)!=meta['sample_jsonl_sha256']:raise ValueError('sample checksum mismatch')
        total=0;failures=[];passing_ids=[];config=RuntimeGateConfig(candidate_count=1);strata={}
        for line in sample.read_text().splitlines():
            row=json.loads(line);target=row['labels']['planning'];total+=1
            # HH_260906 - Label-side strata are diagnostic only and must never become future inference inputs.
            moving=max(target['speed_mps'])>=.5
            turning=moving and max(abs(yaw) for yaw in target['yaw_rad'])>=math.radians(15)
            group=('future_turn_ge15deg' if turning else 'future_moving_nonturn' if moving else 'future_low_motion')
            stat=strata.setdefault(group,dict(total=0,passing=0));stat['total']+=1
            if len(target['valid'])!=64 or not all(target['valid']):
                failures.append(dict(sample_id=row['sample_id'],reason='incomplete_future',category='incomplete_future'));continue
            try:
                validate_and_select_trajectory([target['positions_base_xy_m']],[target['speed_mps']],[0.],config,
                                              current_speed_mps=row['ego']['linear_velocity_base_mps'][0])
            except ContractError as error:
                reason=str(error);category=re.sub(r'\d+','N',reason)
                failures.append(dict(sample_id=row['sample_id'],reason=reason,category=category,
                    diagnostic_group=group,gate_locals=gate_failure_details(error)))
            else:
                passing_ids.append(row['sample_id']);stat['passing']+=1
        if total!=meta['sample_count']:raise ValueError('sample count mismatch')
        result.append(dict(episode_id=identity,sample_count=total,passing=len(passing_ids),
                           first_failure_categories=dict(Counter(x['category'] for x in failures)),
                           failures=failures,passing_ids=passing_ids,
                           diagnostic_strata=strata,episode_sha256=ref['sha256'],samples_sha256=meta['sample_jsonl_sha256']))
    return dict(dataset=str(root),dataset_sha256=sha256_file(root/'dataset.json'),episodes=result,
                labels_modified=False,filtering_applied=False,training_executed=False,
                held_out_payloads_opened=False,
                limitations=['First failure only, not all violations.',
                             'Runtime feasibility is not full decoder representability.',
                             'Diagnostic only; passing IDs are not automatic data admission or a new split.'])


def main():
    """HH_260906 - Save a create-only report with per-episode results and unchanged source hashes."""
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--episode-id',action='append',required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();report=audit(args.dataset,args.episode_id)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)
    for episode in report['episodes']:
        print(json.dumps({k:v for k,v in episode.items() if k not in ('failures','passing_ids')}))


if __name__=='__main__':main()
