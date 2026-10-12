"""HH_260906 - Create an explicitly goal-changing research view; never overwrite collected labels."""
import json
from pathlib import Path
import numpy as np


def align_terminal(points):
    """HH_260906 - Extend a verified straight approach through the last three metres."""
    points=np.asarray(points,float);s=np.r_[0.,np.linalg.norm(np.diff(points,axis=0),axis=1).cumsum()]
    fit=points[(s>=s[-1]-9.) & (s<=s[-1]-3.)]
    if len(fit)<4:raise ValueError('insufficient approach')
    center=fit.mean(axis=0);_,_,axes=np.linalg.svd(fit-center);direction=axes[0]
    residual=np.linalg.norm((fit-center)-np.outer((fit-center)@direction,direction),axis=1).max()
    if residual>.02:raise ValueError('approach not straight')
    mask=s>s[-1]-3.;result=points.copy()
    result[mask]=center+np.outer((points[mask]-center)@direction,direction)
    return result.tolist(),dict(approach_fit_max_error_m=float(residual),
        goal_shift_m=float(np.linalg.norm(result[-1]-points[-1])),goal_changed=True)


if __name__=='__main__':
    root=Path('artifacts/training/2026-09-22');out=root/'terminal_alignment_ab_v1'
    metadata=json.loads((root/'arc_progress_u_v1/input_metadata.json').read_text());changes={}
    for token,item in metadata.items():
        if 'town01' in token and token[-6:] in ('000440','000464','000487'):
            item['route'],changes[token]=align_terminal(item['route'])
    out.mkdir(exist_ok=False)
    (out/'metadata.json').write_text(json.dumps(metadata))
    (out/'changes.json').write_text(json.dumps(changes,indent=2))
    print(json.dumps(changes))
