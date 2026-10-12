"""HH_260906 - Publish actual AI mapping results and rejected expert-control diagnostics, never approval."""
import json
import argparse
from pathlib import Path
import numpy as np
from scripts.e2e.collect_dual_learning_results import fetch, render_mapping

ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/'docs/assets/validation/2026-09-26/dual_data_ai'
CAPTURE=ROOT/'storage/experiments/training/2026-09-26/dual_data_ai_v1'
DATA=ROOT/'storage/datasets/prepared/map_expanded_20260926_ai_v1'
REMOTE='/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics'


def write_new(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2)


def main():
    # HH_260906 - Keep both mapping experiments distinct; do not republish old driving diagnostics as a new run.
    parser=argparse.ArgumentParser()
    parser.add_argument('--revision',choices=('v1','v2'),default='v1')
    revision=parser.parse_args().revision
    data=DATA if revision=='v1' else DATA.with_name('map_expanded_20260926_ai_v2')
    REPORT.mkdir(parents=True,exist_ok=True)
    mapping=REPORT/('results/mapping' if revision=='v1' else 'results/mapping_ai_v2');mapping.mkdir(parents=True,exist_ok=True)
    for name in ('summary.json','started.json','data_manifest.json','loss_contract.json','history.jsonl','dev_predictions.npz'):
        if not (mapping/name).exists():fetch(REMOTE+f'/map_expanded_20260926_ai_{revision}/'+name,mapping/name)
    if (mapping/'data_manifest.json').read_bytes()!=(data/'manifest.json').read_bytes():
        raise ValueError('training manifest differs')
    summary=json.loads((mapping/'summary.json').read_text())
    if summary['status']!='PILOT_COMPLETE_NOT_PROMOTED':raise ValueError('mapping did not finish')
    render_mapping(data,mapping/'dev_predictions.npz',mapping)
    receipt=REPORT/'receipts';receipt.mkdir(exist_ok=True)
    exitpath=receipt/('mapping_exit.json' if revision=='v1' else 'mapping_v2_exit.json')
    if not exitpath.exists():fetch(REMOTE+f'/mapping_20260926_ai_{revision}.exit.json',exitpath)
    checkpoint=CAPTURE/('mapping_ai_checkpoint' if revision=='v1' else 'mapping_ai_v2_checkpoint');checkpoint.mkdir(exist_ok=True)
    if not (checkpoint/'checkpoint.pt').exists():fetch(REMOTE+f'/map_expanded_20260926_ai_{revision}/checkpoint.pt',checkpoint/'checkpoint.pt')
    if revision=='v2':
        print(json.dumps(dict(mapping_miou=summary['mean_iou_present'],revision=revision,report=str(mapping))),flush=True)
        return
    # HH_260906 - Draw native measured motion without retiming, trimming or converting failed data for training.
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    episode=CAPTURE/'driving_town01_p03_9kph_v2/episode.partial'
    manifest=json.loads((episode/'manifest.json').read_text());result=manifest['result']
    rows=[json.loads(line) for line in (episode/'states.jsonl').read_text().splitlines()]
    route=json.loads((episode/'route.json').read_text())['route']
    times=np.array([r['timestamp'] for r in rows]);times-=times[0]
    speed=np.array([np.hypot(r['vx'],r['vy']) for r in rows])
    rates=np.diff(speed)/np.diff(times)
    xy=np.array([[r['x'],r['y']] for r in rows])
    pedals=np.array([[r['current_control'][key] for key in ('throttle','brake')] for r in rows])
    controls=result['control_transport'];quality=result['goal_stop_quality']
    violations=[]
    for i in np.flatnonzero((rates>2.9)|(rates< -2.9)):
        violations.append(dict(from_frame=rows[i]['frame'],to_frame=rows[i+1]['frame'],
            elapsed_seconds=float(times[i+1]),speed_rate_mps2=float(rates[i]),
            from_speed_mps=float(speed[i]),to_speed_mps=float(speed[i+1]),
            observed_control=rows[i+1]['current_control'],goal_stop=rows[i+1]['goal_stop']))
    report=dict(status='REJECTED_EXPERT_DATA_NOT_MODEL_DRIVING',capture_status=manifest['status'],
        error=manifest.get('error'),goal_reached=result['goal_reached'],route_progress_m=result['final_route_progress_m'],
        collision_events=result['collision_event_count'],lane_sensor_events=result['lane_invasion_event_count'],
        lane_body_containment_verified=False,learned_model_control=False,training_executed=False,
        state_count=len(rows),camera_bundles=result['camera_anchor_count'],simulation_seconds=result['duration_sec'],
        native_acceleration_max=float(rates.max()),native_deceleration_min=float(rates.min()),
        scalar_quality=quality['status'],control_alignment_failures=controls['control_alignment_failure_count'],
        command_failures=controls['failed_command_count'],violations=violations,
        cte_absolute_max=float(max(abs(r['route_cte_m']) for r in rows)),
        conclusion='Native launch speed jump and terminal brake-associated speed drops violate existing bounds. '
                   'Control ACKs match but do not independently prove physical actuation or isolate tire/physics effects. '
                   'No clipping, event removal, gate relaxation or training admission.')
    driving=REPORT/'results/driving_capture';driving.mkdir(parents=True,exist_ok=True)
    if not (driving/'summary.json').exists():write_new(driving/'summary.json',report)
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    axes[0,0].plot([r['x'] for r in route],[r['y'] for r in route],'--',label='Catalog route')
    axes[0,0].plot(xy[:,0],xy[:,1],label='Measured expert path');axes[0,0].axis('equal')
    axes[0,0].set_title('Right turn completed; NOT learned-model driving');axes[0,0].legend()
    axes[0,1].plot(times,speed*3.6);axes[0,1].set_title('Measured speed (km/h)')
    axes[1,0].plot(times[1:],rates);axes[1,0].axhline(2.9,color='red',ls='--');axes[1,0].axhline(-2.9,color='red',ls='--')
    axes[1,0].set_title('Native 20 Hz speed rate (m/s²); fixed decoder bounds')
    axes[1,1].plot(times,pedals[:,0],label='Throttle');axes[1,1].plot(times,pedals[:,1],label='Brake')
    axes[1,1].set_xlim(times[-1]-14,times[-1]-5);axes[1,1].set_title('Terminal pedal oscillation');axes[1,1].legend()
    for ax in axes.flat:ax.grid(alpha=.25)
    fig.suptitle('AI expert capture REJECTED: preserve bad launch/stop evidence; no new driving weights')
    fig.savefig(driving/'motion_and_rejection.png',dpi=130);plt.close(fig)
    # HH_260906 - Keep the vehicle centered in the measured trajectory panel; frames are real camera recordings.
    from PIL import Image, ImageDraw
    frames=[json.loads(line) for line in (episode/'camera_frames.jsonl').read_text().splitlines()]
    lookup={r['frame']:i for i,r in enumerate(rows)};pictures=[]
    for f in frames[::15]:
        i=lookup[f['frame']];cx,cy=xy[i];canvas=Image.new('RGB',(1120,460),(20,24,32));draw=ImageDraw.Draw(canvas)
        draw.text((12,10),f'EXPERT CAPTURE ONLY | REJECTED TRAIN DATA | t={times[i]:.1f}s | {speed[i]*3.6:.1f} km/h',fill='white')
        canvas.paste(Image.open(episode/f['images']['CAM_FRONT']).convert('RGB').resize((640,360)),(0,55))
        # HH_260906 - Clip the ego-centered map inside its panel, never draw long route lines over the camera image.
        panel=Image.new('RGB',(480,360),(20,24,32));pd=ImageDraw.Draw(panel)
        def point(x,y):return (int(240+(x-cx)*6),int(180-(y-cy)*6))
        pd.line([point(r['x'],r['y']) for r in route],fill=(90,150,255),width=3)
        if i>0:pd.line([point(x,y) for x,y in xy[max(0,i-200):i+1]],fill=(255,200,70),width=3)
        pd.ellipse((234,174,246,186),fill='white');canvas.paste(panel,(640,55))
        draw.text((665,430),'Blue route | Gold measured trail | White ego',fill='white')
        pictures.append(canvas)
    pictures[len(pictures)//2].save(driving/'expert_turn.png')
    pictures[0].save(driving/'expert_turn.gif',save_all=True,append_images=pictures[1:],duration=150,loop=0)
    print(json.dumps(dict(mapping_miou=summary['mean_iou_present'],driving=report['status'],report=str(REPORT))),flush=True)


if __name__=='__main__':main()
