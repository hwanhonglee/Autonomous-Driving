#!/usr/bin/env python3
"""HH_260906 - Exercise synthetic ideal tracking and stop alerts without simulator or GPU use."""
import argparse
from dataclasses import replace
import hashlib
import itertools
import json
from pathlib import Path
from scripts.e2e.offline_turn_speed_envelope import EnvelopeConfig,plan
from scripts.e2e.research_speed_reference import ReferenceRequest,SpeedReference


def traversal(points,config,dt):
    """HH_260906 - Check only the ideal point-mass reference algebra, not actuator behavior."""
    envelope=plan(points,config);ref=SpeedReference(envelope)
    progress=envelope['distance_m'][0];speed=0.;elapsed=0.;rates=[];trace=[]
    while elapsed<180:
        response=ref.update(ReferenceRequest(progress,speed,dt))
        if not response['normal_reference_allowed']:
            return dict(status='FAIL_REFERENCE_INFEASIBLE',time_s=elapsed,progress_m=progress,
                        speed_mps=speed,response=response,trace=trace)
        target=response['target_speed_mps'];rates.append((target-speed)/dt)
        progress+=response['ideal_step_distance_m'];speed=target;elapsed+=dt
        trace.append([elapsed,progress,speed])
        if envelope['distance_m'][-1]-progress<.01 and speed<.15:
            return dict(status='PASS_IDEAL_ALGEBRA_ONLY',time_s=elapsed,remaining_m=envelope['distance_m'][-1]-progress,
                        terminal_speed_mps=speed,maximum_rate_mps2=max(rates),minimum_rate_mps2=min(rates),trace=trace)
    return dict(status='FAIL_TIMEOUT',time_s=elapsed,progress_m=progress,speed_mps=speed,trace=trace)


def main():
    """HH_260906 - Save every case including failures; never convert synthetic traces into training data."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    cases=[];source_hashes={}
    for route_path in args.route:
        content=route_path.read_bytes();route=json.loads(content)
        if route['town'] not in ('Town01','Town07') or route['scenario']!='right':raise ValueError('TRAIN-only route')
        source_hashes[str(route_path)]=hashlib.sha256(content).hexdigest()
        for dt,lateral,accel in itertools.product((.025,.05,.1,.2),(1.5,2.0,2.5),(.5,1.,1.5)):
            config=replace(EnvelopeConfig(),lateral_mps2=lateral,acceleration_mps2=accel,deceleration_mps2=accel)
            result=traversal(route['route'],config,dt)
            result.update(route=str(route_path),dt_s=dt,lateral_mps2=lateral,acceleration_mps2=accel)
            cases.append(result)
    ref=SpeedReference(plan([dict(distance_m=float(i),yaw=0.) for i in range(101)]))
    stops=[]
    for speed,dist in itertools.product((0.,2.,4.,8.),(0.,1.,5.,10.,40.)):
        result=ref.update(ReferenceRequest(40,speed,.1,stop_distance_m=dist,stop_required=True))
        impossible=speed*speed/2>dist
        if impossible and result['normal_reference_allowed']:raise AssertionError('late stop incorrectly permitted')
        emergency=ref.update(ReferenceRequest(40,speed,.1,stop_distance_m=dist,emergency=True))
        if emergency['target_speed_mps']!=0 or emergency['normal_reference_allowed']:raise AssertionError('emergency modified')
        stops.append(dict(speed_mps=speed,distance_m=dist,comfort_stopping_distance_exceeds_available=impossible,
                          reference_status=result['status'],emergency_preserved=True))
    args.output.mkdir(parents=True,exist_ok=False)
    with (args.output/'cases.json').open('x') as stream:json.dump(cases,stream,indent=2,allow_nan=False)
    summary=dict(cases=len(cases),passed=sum(c['status']=='PASS_IDEAL_ALGEBRA_ONLY' for c in cases),
        failed=[{k:v for k,v in c.items() if k!='trace'} for c in cases if c['status']!='PASS_IDEAL_ALGEBRA_ONLY'],
        stop_emergency_cases=stops,source_sha256=source_hashes,
        code_sha256={name:hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in (
            'scripts/e2e/offline_turn_speed_envelope.py','scripts/e2e/research_speed_reference.py',
            'scripts/e2e/verify_speed_reference_scenarios.py')},
        carla_executed=False,training_executed=False,
        notice='Ideal reference algebra only: no calibrated dynamics, sensors, steering, collisions, traffic perception or learned policy.')
    with (args.output/'summary.json').open('x') as stream:json.dump(summary,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('failed','stop_emergency_cases','source_sha256','code_sha256')}))


if __name__=='__main__':main()
