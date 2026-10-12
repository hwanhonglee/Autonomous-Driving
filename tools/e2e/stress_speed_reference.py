#!/usr/bin/env python3
"""HH_260906 - Bounded single-process randomized offline reference stress tests, not training."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import random
import time
from scripts.e2e.offline_turn_speed_envelope import EnvelopeConfig
from scripts.e2e.verify_speed_reference_scenarios import traversal


def main():
    """HH_260906 - Preserve deterministic failures and keep resource bounds explicit."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',type=int,default=1000);parser.add_argument('--seed',type=int,default=20260922)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if not 1<=args.cases<=5000:raise ValueError('bounded case count required')
    rng=random.Random(args.seed);started=time.monotonic();failures=[];counts=Counter();max_rate=0.;min_rate=0.
    for index in range(args.cases):
        points=[];arc=0.;yaw=rng.uniform(-math.pi,math.pi)
        count=rng.randint(60,160);turn_start=count//3;turn_end=2*count//3
        turn=rng.uniform(-2.1,2.1)
        for point in range(count):
            if point:arc+=rng.uniform(.4,1.6)
            if turn_start<=point<turn_end:yaw+=turn/(turn_end-turn_start)
            points.append(dict(distance_m=arc,yaw=math.atan2(math.sin(yaw),math.cos(yaw))))
        config=EnvelopeConfig(cruise_mps=rng.uniform(3,8),lateral_mps2=rng.uniform(1,2.5),
                              acceleration_mps2=rng.uniform(.5,1.5),deceleration_mps2=rng.uniform(.5,1.5))
        dt=rng.choice((.025,.05,.1,.2));result=traversal(points,config,dt);counts[result['status']]+=1
        if result['status']!='PASS_IDEAL_ALGEBRA_ONLY':
            failures.append(dict(case_index=index,points=points,config=config.__dict__,dt_s=dt,
                                 result={k:v for k,v in result.items() if k!='trace'}))
        else:
            max_rate=max(max_rate,result['maximum_rate_mps2']);min_rate=min(min_rate,result['minimum_rate_mps2'])
            if result['maximum_rate_mps2']>config.acceleration_mps2+1e-7 or result['minimum_rate_mps2'] < -config.deceleration_mps2-1e-7:
                raise AssertionError('reported feasible trace violates temporal rate bounds')
        if (index+1)%100==0:print(json.dumps(dict(completed=index+1,counts=dict(counts))),flush=True)
    report=dict(seed=args.seed,cases=args.cases,counts=dict(counts),failures=failures,
                maximum_rate_mps2=max_rate,minimum_rate_mps2=min_rate,wall_seconds=time.monotonic()-started,
                synthetic_only=True,training_executed=False,carla_executed=False,
                source_sha256={name:hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in (
                    'scripts/e2e/stress_speed_reference.py','scripts/e2e/verify_speed_reference_scenarios.py',
                    'scripts/e2e/research_speed_reference.py','scripts/e2e/offline_turn_speed_envelope.py')})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(status='STRESS_COMPLETE',wall_seconds=report['wall_seconds'],counts=dict(counts))))


if __name__=='__main__':main()
