"""HH_260906 - Bounded read-only SSH monitoring and local result publication; no promotion."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import re
import subprocess
import time

REMOTE="/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics"
JOBS={"driving":"full_recovery_ae_20260922_v1", "mapping":"map_perception_pilot_20260922_v2"}


def ssh(command):
    return subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','pro6000-training',command],
                           check=True,capture_output=True,text=True,timeout=20).stdout


def fetch(remote,local):
    # HH_260906 - Transfer into a private partial file and verify remote bytes before publishing.
    digest=ssh('sha256sum '+shlex.quote(remote)).split()[0]
    temp=local.with_suffix(local.suffix+'.partial')
    subprocess.run(['scp','-q','-o','BatchMode=yes','-o','ConnectTimeout=8',
                    'pro6000-training:'+remote,str(temp)],check=True,timeout=60)
    if hashlib.sha256(temp.read_bytes()).hexdigest()!=digest:raise ValueError('transfer checksum mismatch')
    if local.exists():raise ValueError('refuse result overwrite')
    temp.rename(local)


def render_mapping(data,predictions,output):
    import numpy as np
    from PIL import Image,ImageDraw,ImageFont
    colors=np.array([[30,30,40],[67,92,123],[141,88,147],[246,222,90],
                     [246,100,70],[60,220,180],[179,190,199]],dtype=np.uint8)
    with np.load(data/'dev.npz',allow_pickle=False) as d:rgb=d['rgb'];target=d['target']
    with np.load(predictions,allow_pickle=False) as d:pred=d['prediction']
    if pred.shape!=target.shape or pred.dtype!=np.uint8 or pred.max()>=7:raise ValueError('prediction shape/labels differ')
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    pictures=[]
    resampling=getattr(Image,'Resampling',Image)
    for i in sorted(set(range(0,len(pred),max(1,len(pred)//30)))|{len(pred)//2,len(pred)-1}):
        canvas=Image.new('RGB',(1280,260),(16,23,33));draw=ImageDraw.Draw(canvas)
        draw.text((16,10),'LEARNED RGB PERCEPTION | separate-route DEV | NO HD MAP / DRIVING APPROVAL',font=font,fill='white')
        error=np.zeros_like(rgb[i]);error[pred[i]!=target[i]]=[255,70,70]
        for j,(name,arr) in enumerate([('RGB only input',rgb[i]),('CARLA label target',colors[target[i]]),
                                      ('Learned prediction',colors[pred[i]]),('Red: incorrect pixels',error)]):
            draw.text((j*320+8,50),name,font=font,fill='white')
            canvas.paste(Image.fromarray(arr).resize((320,180),resampling.NEAREST),(j*320,78))
        pictures.append(canvas)
        if i in {0,len(pred)//2,len(pred)-1}:canvas.save(output/f'perception_{i:04d}.png')
    pictures[0].save(output/'learned_perception.gif',save_all=True,append_images=pictures[1:],duration=150,loop=0)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--seconds',type=int,default=7200)
    # HH_260906 - Select fresh named runs without rewriting historical monitor defaults.
    p.add_argument('--driving-job',default=JOBS['driving'])
    p.add_argument('--mapping-job',default=JOBS['mapping'])
    # HH_260906 - A driving-only extension must not relabel an old mapping run as new training.
    p.add_argument('--driving-only',action='store_true')
    a=p.parse_args()
    jobs={'driving':a.driving_job}
    if not a.driving_only:jobs['mapping']=a.mapping_job
    if not all(re.fullmatch(r'[a-z0-9_]+',name) for name in jobs.values()):
        raise ValueError('simple diagnostic run names required')
    root=Path(__file__).resolve().parents[2]
    if not a.output.resolve().is_relative_to(root/'docs/assets/validation') or not 1<=a.seconds<=7200:
        raise ValueError('bounded publication directory and monitoring duration required')
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'started.json').write_text(json.dumps(dict(started_at_utc=datetime.now(timezone.utc).isoformat(),
        jobs=jobs,read_only_remote=True,automatic_promotion=False,monitor_seconds=a.seconds),indent=2))
    deadline=time.monotonic()+a.seconds;done={}
    while time.monotonic()<deadline:
        for role,name in jobs.items():
            if role in done:continue
            remote=REMOTE+'/'+name
            try:
                ready=ssh('if test -f '+shlex.quote(remote+'/summary.json')+'; then printf READY; else printf WAIT; fi')
                if ready!='READY':continue
                folder=a.output/role;folder.mkdir(exist_ok=True)
                if not (folder/'summary.json').exists():fetch(remote+'/summary.json',folder/'summary.json')
                summary=json.loads((folder/'summary.json').read_text())
                if role=='mapping':
                    if summary['status']!='PILOT_COMPLETE_NOT_PROMOTED':raise ValueError('mapping incomplete')
                    if not (folder/'data_manifest.json').exists():fetch(remote+'/data_manifest.json',folder/'data_manifest.json')
                    if (folder/'data_manifest.json').read_bytes()!=(a.data/'manifest.json').read_bytes():
                        raise ValueError('publication dataset differs from trained dataset')
                    if not (folder/'dev_predictions.npz').exists():fetch(remote+'/dev_predictions.npz',folder/'dev_predictions.npz')
                    render_mapping(a.data,folder/'dev_predictions.npz',folder)
                elif summary['status']!='COMPLETE_NOT_PROMOTED':raise ValueError('driving incomplete')
                done[role]=summary
                print(role+' RESULTS_PUBLISHED',flush=True)
            except (subprocess.SubprocessError,OSError,ValueError,KeyError) as error:
                print(role+' RETRY '+str(error),flush=True)
        if len(done)==len(jobs):
            drive=done['driving']
            rows=drive['evaluation']['subgroups']
            report=['# 모델 학습 완료 결과', '',
                '<!-- HH_260906 - Auto-publish verified completed summaries without deployment or feature approval. -->','',
                ('주행 추가 학습과 평가가 끝났습니다. 지도 학습은 이번 실행에 포함되지 않았습니다.'
                 if a.driving_only else '두 학습과 평가가 끝났습니다.')+' 자동 채택·배포는 하지 않았습니다.','',
                '## 주행 모델','',
                f"- 우회전 평균 궤적 오차: {rows['town03_right_turn_dev']['metrics']['selected_ade_m']:.4f} m.",
                f"- 직진 평균 궤적 오차: {rows['town03_straight_dev']['metrics']['selected_ade_m']:.4f} m.",
                f"- 기존 K1 대비 비퇴행: {drive['versus_original_k1']['pass_count']}/9. Strict gate: {drive['strict_gate']['metric_pass_count']}/9.",
                '- 기존에 반복 관측한 DEV 평가이며 실차·폐루프 승인 아님.','']
            if 'versus_parent_af' in drive:
                report += [f"- 부모 AF 대비 비퇴행: {drive['versus_parent_af']['pass_count']}/9.",'']
            if not a.driving_only:
                mapping=done['mapping']
                report += ['## RGB 지도 인식 모델','',
                f"- 평가 mIoU: {mapping['mean_iou_present']:.4f}; 픽셀 정확도: {mapping['pixel_accuracy']:.4f}.",
                '- 같은 Town/날씨의 별도 경로 파일럿. 독립 TEST 또는 완성된 정밀지도 품질 아님.',
                '- [실제 학습 모델 예측 GIF](mapping/learned_perception.gif)','',
                '| 분류 | IoU |','|---|---:|']
                report += [f'| {name} | {value:.4f} |' if value is not None else f'| {name} | 평가 없음 |' for name,value in mapping['iou'].items()]
            (a.output/'README.md').write_text('\n'.join(report)+'\n')
            (a.output/'complete.json').write_text(json.dumps(dict(completed_at_utc=datetime.now(timezone.utc).isoformat(),
               status='DRIVING_RESULT_PUBLISHED_NOT_PROMOTED' if a.driving_only else 'BOTH_RESULTS_PUBLISHED_NOT_PROMOTED'),indent=2))
            return
        time.sleep(30)
    (a.output/'timeout.json').write_text(json.dumps(dict(status='MONITOR_TIMEOUT',completed_jobs=list(done)),indent=2))
    raise TimeoutError('remote training left untouched; monitoring deadline reached')


if __name__=='__main__':main()
