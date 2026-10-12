"""HH_260906 - Audit all native AJ observations and full measured futures; never admit or rewrite training labels."""
import argparse
import hashlib
import gzip
import inspect
import json
import shutil
from pathlib import Path
import numpy as np
from scripts.e2e import audit_carla_raw_pre_admission as raw


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(path):
    """HH_260906 - Check owned source archives, all images, aligned commands and unchanged future geometry rules."""
    owner=json.loads((path/'owner_result.json').read_text())
    plan=json.loads((path/'owner_plan.json').read_text())
    if not owner['source_bytes_unchanged_and_archived']:raise ValueError('capture source integrity failed')
    for name,digest in plan['source_sha256'].items():
        if sha(path/'provenance'/name)!=digest:raise ValueError('archived source changed')
    episode=path/'episode'
    if not episode.exists():episode=path/'episode.partial'
    manifest=json.loads((episode/'manifest.json').read_text())
    if manifest['cleanup']['completed'] is not True:raise ValueError('cleanup incomplete')
    states=[json.loads(line) for line in (episode/'states.jsonl').read_text().splitlines()]
    cameras=[json.loads(line) for line in (episode/'camera_frames.jsonl').read_text().splitlines()]
    route=json.loads((episode/'route.json').read_text())
    if sha(episode/'route.json')!=plan['route_sha256']:raise ValueError('route differs')
    timeline=raw.measured_timeline(states)
    pixels,images,anchors=raw.camera_audit(episode,manifest,cameras,timeline)
    try:
        future,windows=raw.measured_future_audit(timeline,cameras,route,anchors)
    except raw.contract.ContractError as error:
        # HH_260906 - Preserve an overshoot as rejected/unassessable, never clip an endpoint to manufacture future labels.
        if 'no route remaining after the anchor' not in str(error):raise
        future=dict(status='REJECTED_ROUTE_END',error=str(error),full_future_audit_completed=False,
            notice='The unchanged navigation contract rejects at least one anchor; no partial metrics are presented as a full audit.')
        windows=[dict(disposition='whole_trial_future_audit_incomplete',error=str(error))]
    speed=np.array([np.hypot(r['vx'],r['vy']) for r in states]);time=np.array([r['timestamp'] for r in states])
    by_frame={r['frame']:i for i,r in enumerate(states)}
    metrics={}
    for cadence,indices in [('native20',list(range(len(states)))),('camera10',[by_frame[r['frame']] for r in cameras])]:
        v=speed[indices];t=time[indices];rate=np.diff(v)/np.diff(t)
        metrics[cadence]=dict(acceleration_max=float(rate.max()),deceleration_min=float(rate.min()),
            decoder_violations=int(((rate>2.9)|(rate< -2.9)).sum()),
            runtime_rate_violations=int(((rate>3)|(rate< -6)).sum()))
    fields=('throttle','brake','steer','hand_brake','reverse','manual_gear_shift')
    alignment=sum(any(abs(float(b['current_control'][k])-float(a['next_control'][k]))>1e-6 for k in fields)
        for a,b in zip(states,states[1:]))
    result=manifest['result']
    cap=manifest['capture_contract']['goal_stop_profile']['normal_brake_cap']
    # HH_260906 - Separate above-normal-cap controls from terminal normal braking; hazard reason is not recorded.
    above=[i for i,r in enumerate(states) if r['capture_phase']=='driving' and r['current_control']['brake']>cap+1e-6]
    brake_witnesses=[dict(frame=states[i]['frame'],elapsed_seconds=float(time[i]-time[0]),
        phase=states[i].get('goal_stop',{}).get('phase'),current_control=states[i]['current_control'],
        remaining_m=states[i].get('goal_stop',{}).get('remaining_route_arc_m')) for i in above]
    summary=dict(path=str(path),owner_exit=owner['exit_code'],capture_status=manifest['status'],error=manifest.get('error'),
        goal_reached=result.get('goal_reached'),remaining_route_m=result.get('final_remaining_route_m'),
        duration_seconds=float(time[-1]-time[0]),max_speed_kmh=float(speed.max()*3.6),
        collision_events=result.get('collision_event_count'),lane_sensor_events=result.get('lane_invasion_event_count'),
        body_lane_containment_verified=False,measured_speed_rate=metrics,command_alignment_mismatches=alignment,
        pixels=pixels,measured_future=future,configuration=manifest['capture_contract']['goal_stop_profile'],
        above_normal_brake_cap_frames=brake_witnesses,hazard_reason_verified=False,
        raw_sha256={name:sha(episode/name) for name in ('manifest.json','states.jsonl','camera_frames.jsonl','route.json')},
        owner_sha256=sha(path/'owner_plan.json'),learned_model_control=False,training_data_approved=False,
        notice='Read-only necessary-condition diagnostic, not a replacement for source-bound admission, lane containment or closed-loop approval.')
    return summary,states,cameras,episode,windows


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--trial',action='append',required=True,type=Path)
    # HH_260906 - Reuse unchanged raw/future checks for the subsequent AK control comparison with honest figure labeling.
    parser.add_argument('--campaign',choices=('AJ','AK','AL'),default='AJ')
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    # HH_260906 - Archive the exact diagnostic code before reading evidence, including unchanged geometry helpers.
    root=Path(__file__).resolve().parents[2]
    source_files=raw.source_identity()['files']
    source_files[str(Path(__file__).resolve().relative_to(root))]=sha(Path(__file__))
    for relative,digest in source_files.items():
        src=root/relative;dst=args.output/'source'/relative
        if sha(src)!=digest:raise ValueError('diagnostic source changed before archival')
        dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    summaries=[];traces=[]
    for path in args.trial:
        summary,states,cameras,episode,windows=audit(path.resolve())
        summaries.append(summary);traces.append((path.name,states,episode,cameras))
        with (args.output/(path.name+'.json')).open('x') as stream:json.dump(summary,stream,indent=2)
        # HH_260906 - Preserve every assessed or unavailable future, including rejected warmup and stop windows.
        with gzip.open(args.output/(path.name+'.future.jsonl.gz'),'xt') as stream:
            for row in windows:stream.write(json.dumps(row)+'\n')
        print(path.name,json.dumps(summary['measured_speed_rate']),flush=True)
    sources={str(Path(inspect.getsourcefile(f)).resolve()):sha(Path(inspect.getsourcefile(f)))
        for f in (raw.measured_timeline,raw.measured_future_audit,raw.targets.audit_sample,raw.adapter._interpolate_state)}
    with (args.output/'summary.json').open('x') as stream:
        json.dump(dict(trials=summaries,diagnostic_source_sha256=sources,script_sha256=sha(Path(__file__)),
            training_executed=False,automatic_promotion=False),stream,indent=2)
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(14,9),constrained_layout=True)
    for name,states,_,_ in traces:
        t=np.array([r['timestamp'] for r in states]);t-=t[0]
        v=np.array([np.hypot(r['vx'],r['vy']) for r in states])
        axes[0,0].plot(t,v*3.6,label=name)
        axes[0,1].plot(t[1:],np.diff(v)/np.diff(t),label=name)
        axes[1,0].plot([r['x'] for r in states],[r['y'] for r in states],label=name)
        end=np.array([r.get('goal_stop',{}).get('remaining_route_arc_m',np.nan) for r in states])
        axes[1,1].plot(end,v,label=name)
    axes[0,0].set_title('Measured speed (km/h) / elapsed seconds')
    axes[0,1].set_title('Native speed rate (m/s²); unchanged decoder bounds')
    for bound in (-2.9,2.9):axes[0,1].axhline(bound,color='red',ls='--')
    axes[1,0].set_title('Measured ROS-map path (m)');axes[1,0].axis('equal')
    axes[1,1].set_title('Terminal speed (m/s) / remaining route arc (m)');axes[1,1].set_xlim(5,0)
    for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=7)
    fig.suptitle(f'{args.campaign} expert control comparison — no learned-model driving / no data admission')
    fig.savefig(args.output/'control_comparison.png',dpi=130);plt.close(fig)
    # HH_260906 - Show actual six-camera recordings and an ego-centered path, never synthetic driving evidence.
    from PIL import Image,ImageDraw
    for name,states,episode,cameras in traces:
        route=json.loads((episode/'route.json').read_text())['route'];lookup={r['frame']:i for i,r in enumerate(states)}
        pictures=[]
        for camera in cameras[::max(1,len(cameras)//32)]:
            i=lookup[camera['frame']];r=states[i];canvas=Image.new('RGB',(960,760),(20,24,32));draw=ImageDraw.Draw(canvas)
            draw.text((10,8),name+' | EXPERT ONLY | NOT TRAINING APPROVED',fill='white')
            for j,cam in enumerate(camera['camera_order']):
                x=(j%3)*320;y=35+(j//3)*205
                draw.text((x+5,y),cam,fill='white')
                canvas.paste(Image.open(episode/camera['images'][cam]).convert('RGB').resize((320,180)),(x,y+20))
            panel=Image.new('RGB',(600,300),(20,24,32));pd=ImageDraw.Draw(panel)
            def point(x,y):return (int(300+(x-r['x'])*5),int(150-(y-r['y'])*5))
            pd.line([point(p['x'],p['y']) for p in route],fill=(70,140,255),width=3)
            trail=states[max(0,i-200):i+1]
            if len(trail)>1:pd.line([point(p['x'],p['y']) for p in trail],fill=(255,190,70),width=3)
            pd.ellipse((295,145,305,155),fill='white');canvas.paste(panel,(0,455))
            draw.text((605,480),f"speed: {np.hypot(r['vx'],r['vy'])*3.6:.2f} km/h",fill='white')
            draw.text((605,510),f"throttle: {r['current_control']['throttle']:.3f}",fill='white')
            draw.text((605,540),f"brake: {r['current_control']['brake']:.3f}",fill='white')
            draw.text((605,580),'White ego / Blue route / Gold trail',fill='white')
            draw.text((605,610),'Sampled replay; not GUI FPS',fill='white');pictures.append(canvas)
        pictures[len(pictures)//2].save(args.output/(name+'.png'))
        pictures[0].save(args.output/(name+'.gif'),save_all=True,append_images=pictures[1:],duration=200,loop=0)


if __name__=='__main__':main()
