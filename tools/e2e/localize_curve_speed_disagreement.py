"""HH_260906 - Separate endpoint-speed semantics from measured braking without modifying gates."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re


def interval_metrics(a, b):
    """HH_260906 - Compare interval displacement with endpoint and trapezoidal speeds explicitly."""
    dt=b['timestamp']-a['timestamp']
    if dt<=0:raise ValueError('nonpositive interval')
    va=math.hypot(a['vx'],a['vy']);vb=math.hypot(b['vx'],b['vy'])
    geometric=math.hypot(b['x']-a['x'],b['y']-a['y'])/dt
    return dict(from_frame=a['frame'],to_frame=b['frame'],dt_s=dt,
        entry_speed_mps=va,endpoint_speed_mps=vb,geometric_speed_mps=geometric,
        endpoint_error_mps=geometric-vb,trapezoid_error_mps=geometric-(va+vb)/2,
        measured_speed_rate_mps2=(vb-va)/dt)


def analyze(trial, audit_path):
    """HH_260906 - Authenticate audit source and collapse overlapping first failures into physical intervals."""
    episode=trial/'episode'
    if not episode.exists():episode=trial/'episode.partial'
    matches=[r for r in json.loads(audit_path.read_text()) if Path(r['trial']).resolve()==trial.resolve()]
    if len(matches)!=1:raise ValueError('one matching raw audit required')
    report=matches[0]
    for name,digest in report['source_sha256'].items():
        if hashlib.sha256((episode/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('raw source differs from audit')
    states=[json.loads(line) for line in (episode/'states.jsonl').read_text().splitlines()]
    by_frame={r['frame']:r for r in states}
    rows=[by_frame[c['frame']] for c in map(json.loads,(episode/'camera_frames.jsonl').read_text().splitlines())]
    index={r['frame']:i for i,r in enumerate(rows)}
    counts=Counter()
    for failure in report['failures']:
        match=re.fullmatch(r'selected trajectory speed (\d+) disagrees with its displacement',failure['reason'])
        if match:counts[index[failure['frame']]+int(match[1])+1]+=1
    intervals=[]
    for i,count in sorted(counts.items()):
        a,b=rows[i-1:i+1]
        record=interval_metrics(a,b)
        native=[s for s in states if a['frame']<=s['frame']<=b['frame']]
        record.update(overlapping_first_failures=count,phase=b['capture_phase'],
            native_intervals=[interval_metrics(x,y) for x,y in zip(native,native[1:])],
            native_controls=[dict(frame=s['frame'],current=s['current_control'],next=s['next_control'],
                ack=s.get('control_transport')) for s in native])
        intervals.append(record)
    all_intervals=[interval_metrics(a,b) for a,b in zip(rows,rows[1:])]
    return dict(trial=str(trial),first_failure_windows=sum(counts.values()),
        distinct_first_failure_intervals=len(counts),intervals=intervals,
        all_camera_intervals=len(all_intervals),
        all_endpoint_disagreements=sum(abs(r['endpoint_error_mps'])>.25 for r in all_intervals),
        diagnostic_trapezoid_disagreements=sum(abs(r['trapezoid_error_mps'])>.25 for r in all_intervals),
        all_deceleration_violations=sum(r['measured_speed_rate_mps2'] < -6 for r in all_intervals),
        source_sha256=report['source_sha256'],audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),
        labels_modified=False,runtime_gate_modified=False,training_admitted=False,
        limitations=['Trapezoidal speed is a diagnostic, not replacement labels or a relaxed gate.',
            'A chord differs from traveled arc length on turns.',
            'First-failure intervals do not enumerate every violation hidden by earlier gates.',
            'ACK frame binding is not proof of physical actuator response.'])


def main():
    """HH_260906 - Write a new immutable diagnostic report from existing local recordings."""
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();report=analyze(args.trial,args.audit)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k not in ('intervals','source_sha256','limitations')}))


if __name__=='__main__':main()
