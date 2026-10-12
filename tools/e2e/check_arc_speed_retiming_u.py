"""HH_260906 - Check bounded slowdown from measured initial speed; no synthetic label admission."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from portable_e2e.contract import ContractError
from portable_e2e.runtime_contract import RuntimeGateConfig,validate_and_select_trajectory
from portable_e2e.torch_dataset import _linear_route_samples


def bounded_speeds(desired,current,factor):
    """HH_260906 - Preserve initial-state continuity instead of multiplying the initial speed away."""
    if not 0<factor<=1 or not math.isfinite(current):raise ValueError('invalid retiming request')
    v=max(0.,current);out=[]
    for target in desired:
        if not math.isfinite(target) or target<0:raise ValueError('invalid desired speed')
        v=max(0.,max(v-.2,min(v+.2,target*factor)));out.append(v)
    return out


def main():
    """HH_260906 - Keep runtime rejection and incomplete-turn coverage visible for every slowdown factor."""
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--smooth',action='store_true');a=p.parse_args()
    result=json.loads((a.run/'results.json').read_text());meta=json.loads((a.run/'input_metadata.json').read_text())
    reports=[]
    for row in result['rows']:
        info=meta[row['token']];route=np.asarray(_linear_route_samples(info['route'],limit=128),dtype=float)
        arc=np.concatenate(([0.],np.linalg.norm(np.diff(route,axis=0),axis=1).cumsum()))
        fit=min((f for f in row['fits'] if f['objective']=='joint'),key=lambda f:f['objective_value'])
        for factor in (1.,.75,.5,.25):
            speeds=bounded_speeds(fit['speed'],info['current_speed'],factor)
            distance=np.cumsum(speeds)*.1
            xy=np.stack([np.interp(distance,arc,route[:,d]) for d in (0,1)],axis=-1)
            if a.smooth:
                # HH_260906 - C1 route interpolation and a disclosed ego-origin join; retain the original route data.
                tangent=np.gradient(route,arc,axis=0)
                s=np.minimum(distance,arc[-1]);ix=np.clip(np.searchsorted(arc,s,side='right')-1,0,len(arc)-2)
                h=arc[ix+1]-arc[ix];t=((s-arc[ix])/h)[:,None]
                xy=(2*t**3-3*t**2+1)*route[ix]+(t**3-2*t**2+t)*h[:,None]*tangent[ix]
                xy+=(-2*t**3+3*t**2)*route[ix+1]+(t**3-t**2)*h[:,None]*tangent[ix+1]
                join=np.clip(s/5.,0.,1.)[:,None]
                xy-=route[0]*(1.-3*join**2+2*join**3)
            try:
                validate_and_select_trajectory([xy.tolist()],[speeds],[0.],RuntimeGateConfig(candidate_count=1),
                    current_speed_mps=info['current_speed'])
                verdict='PASS'
            except ContractError as error:verdict=str(error)
            delta=xy[-1]-xy[-2]
            reports.append(dict(token=row['token'],factor=factor,current_speed_mps=info['current_speed'],
                gate=verdict,progress_m=float(distance[-1]),final_speed_mps=speeds[-1],
                final_segment_heading_deg=math.degrees(math.atan2(delta[1],delta[0])),
                smoothed_route=a.smooth,learned_model=False,turn_completion_proven=False))
    with a.output.open('x') as stream:json.dump(dict(rows=reports,training_admitted=False),stream,indent=2)
    for row in reports: print(json.dumps(row))


if __name__=='__main__':main()
