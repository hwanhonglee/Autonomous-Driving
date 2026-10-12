"""HH_260906 - Explain completed model errors without training, threshold changes or promotion."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/'docs/assets/validation/2026-09-26/dual_model_recovery_ah'
DATA=ROOT/'storage/datasets/prepared/map_native_20260926_ah_v1'
BUCKETS=((1,4),(5,16),(17,64),(65,None))


def digest(path):
    """HH_260906 - Bind this analysis to the exact already-completed evidence."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def region_stats(target,prediction,class_id):
    """HH_260906 - GT connected-region pixel recall is not tracked object detection recall."""
    from scipy.ndimage import label
    if target.shape!=prediction.shape or target.ndim!=3:
        raise ValueError('equal N,H,W semantic arrays required')
    rows={str(lo)+'-'+(str(hi) if hi else 'inf'):dict(regions=0,target_pixels=0,correct_pixels=0,zero_hit_regions=0)
          for lo,hi in BUCKETS}
    for truth,pred in zip(target,prediction):
        components,count=label(truth==class_id,structure=np.ones((3,3),dtype=int))
        size=np.bincount(components.ravel(),minlength=count+1)
        correct=np.bincount(components[pred==class_id].ravel(),minlength=count+1)
        for region in range(1,count+1):
            pixels=int(size[region]);hits=int(correct[region])
            for lo,hi in BUCKETS:
                if pixels>=lo and (hi is None or pixels<=hi):
                    row=rows[str(lo)+'-'+(str(hi) if hi else 'inf')]
                    row['regions']+=1;row['target_pixels']+=pixels;row['correct_pixels']+=hits
                    row['zero_hit_regions']+=int(hits==0);break
    for row in rows.values():
        row['pixel_recall']=row['correct_pixels']/row['target_pixels'] if row['target_pixels'] else None
    return rows


def driving_breakdown(evaluation):
    """HH_260906 - Separate candidate lower bound from selector gap, without causal attribution."""
    groups={}
    for name,group in evaluation['subgroups'].items():
        m=group['metrics'];selected=m['selected_ade_m'];oracle=m['oracle_ade_m']
        if not np.isfinite([selected,oracle]).all() or selected+1e-6<oracle:
            raise ValueError('invalid selected/oracle metric relation')
        groups[name]=dict(selected_ade_m=selected,oracle_ade_m=oracle,selection_gap_m=selected-oracle,
            horizon_ade_m={key:m[key] for key in ('ade_1p0s_m','ade_3p0s_m','ade_6p4s_m')})
    return groups


def mapping_analysis():
    """HH_260906 - Native-resolution DEV only; no TRAIN edits or class-weight fitting."""
    output=REPORT/'diagnostics';output.mkdir(exist_ok=True)
    manifest_path=DATA/'manifest.json';manifest=json.loads(manifest_path.read_text())
    data_path=DATA/'dev.npz';ref=manifest['splits']['dev']
    if digest(data_path)!=ref['sha256']:raise ValueError('DEV bytes changed')
    prediction_path=REPORT/'results/mapping/dev_predictions.npz'
    summary_path=REPORT/'results/mapping/summary.json';summary=json.loads(summary_path.read_text())
    with np.load(data_path,allow_pickle=False) as d:rgb,target=d['rgb'],d['target']
    with np.load(prediction_path,allow_pickle=False) as d:pred=d['prediction']
    if pred.shape!=target.shape or pred.dtype!=np.uint8 or pred.max()>=len(manifest['labels']):
        raise ValueError('invalid predictions')
    labels=manifest['labels'];confusion=np.bincount((target.astype(np.int64)*len(labels)+pred).ravel(),
        minlength=len(labels)**2).reshape(len(labels),len(labels))
    if confusion.tolist()!=summary['confusion']:raise ValueError('prediction/summary confusion differs')
    classes={}
    for class_id,name in enumerate(labels):
        tp=int(confusion[class_id,class_id]);gt=int(confusion[class_id].sum());guessed=int(confusion[:,class_id].sum())
        classes[name]=dict(target_pixels=gt,predicted_pixels=guessed,true_positive_pixels=tp,
            precision=tp/guessed if guessed else None,recall=tp/gt if gt else None,iou=summary['iou'][name])
        if name in ('traffic_light_fixture','traffic_sign','lane_marking'):
            classes[name]['gt_connected_region_size_px']=region_stats(target,pred,class_id)
    payload=dict(classes=classes,input_sha256={str(p.relative_to(ROOT)):digest(p) for p in (manifest_path,data_path,prediction_path,summary_path)},
        source_sha256=digest(Path(__file__)),resolution_wh=[640,360],split='repeated_DEV',
        trained=False,thresholds_changed=False,deployment_approved=False,
        limitations='8-connected GT semantic regions per frame; not instances/tracks; repeated frames are correlated; no parent size-stratified prediction archive')
    with (output/'mapping_errors.json').open('x') as f:json.dump(payload,f,indent=2)
    # HH_260906 - Choose panels by GT object area, not by whether the prediction looks successful.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(12,7))
    for row,name in enumerate(('traffic_light_fixture','traffic_sign')):
        class_id=labels.index(name);areas=(target==class_id).sum((1,2));index=int(areas.argmax())
        axes[row,0].imshow(rgb[index]);axes[row,0].set_title(f'{name}: largest GT area, frame {index}')
        axes[row,1].imshow(target[index]==class_id,vmin=0,vmax=1,cmap='gray');axes[row,1].set_title('Ground-truth class pixels')
        axes[row,2].imshow(pred[index]==class_id,vmin=0,vmax=1,cmap='gray');axes[row,2].set_title('Learned class pixels')
        for axis in axes[row]:axis.axis('off')
    fig.suptitle('Native 640x360 repeated DEV | semantic pixels, NOT HD-map approval')
    fig.tight_layout();fig.savefig(output/'small_object_failures.png',dpi=160);plt.close(fig)
    print('MAPPING_FAILURE_ANALYSIS_COMPLETE',flush=True)


def driving_analysis():
    """HH_260906 - Reuse sealed full-horizon evaluations; do not rerun or select checkpoints on DEV."""
    old=ROOT/'docs/assets/validation/2026-09-22/portable_e2e_learning_cycle_v1'
    paths={'K1':old/'21_full_continuation_ad/original_k1_evaluation.json',
           'AD':old/'21_full_continuation_ad/completed_summary.json',
           'AF':old/'24_root_cleanup_and_controlled_learning/results/driving/summary.json',
           'AG':old/'27_driving_continuation_ag/results/driving/summary.json',
           'AH':REPORT/'results/driving/summary.json'}
    raw={name:json.loads(path.read_text()) for name,path in paths.items()}
    reports={name:driving_breakdown(doc if name=='K1' else doc['evaluation']) for name,doc in raw.items()}
    current=raw['AH']
    quality=current['strict_gate']['pass'] and current['versus_original_k1']['pass']
    next_action=('ELIGIBLE_FOR_SEPARATE_EXPORT_AND_LOCAL_LATENCY_VALIDATION_NOT_DEPLOYMENT' if quality
                 else 'DO_NOT_DEPLOY_REVIEW_GENERATION_SELECTION_AND_COLLECT_VALID_MOVING_TURNS')
    payload=dict(models=reports,source_sha256=digest(Path(__file__)),
        input_sha256={name:digest(path) for name,path in paths.items()},
        existing_quality_and_nonregression_pass=quality,next_action=next_action,
        strict_gate=current['strict_gate'],nonregression=current['versus_original_k1'],
        oracle_uses_future_truth=True,causal_claim=False,trained=False,deployment_approved=False)
    output=REPORT/'diagnostics';output.mkdir(exist_ok=True)
    with (output/'driving_errors.json').open('x') as f:json.dump(payload,f,indent=2)
    lines=['# AH 학습 다음 단계: 오류 분해와 진행 판단','',
        '<!-- HH_260906 - Do not equate optimization completion or oracle predictions with safe driving. -->','',
        f"기존 절대 기준: {current['strict_gate']['metric_pass_count']}/9; K1 대비 비퇴행: {current['versus_original_k1']['pass_count']}/9.",'',
        '다음 단계: '+('별도 export 출력 일치와 로컬 지연 검증 대상. 배포 승인 아님.' if quality else
        '**운용 모델 교체하지 않음. 후보 생성/선택 오차와 유효 이동 회전 자료를 보강하는 단계.**'),'',
        '| 모델/구간 | 실제 선택 ADE | 최선 후보 ADE | 선택 gap | 1초 ADE | 3초 ADE |',
        '|---|---:|---:|---:|---:|---:|']
    for model,groups in reports.items():
        for name,label in (('town03_right_turn_dev','우회전'),('town03_straight_dev','직진')):
            g=groups[name];h=g['horizon_ade_m']
            lines.append(f"| {model}/{label} | {g['selected_ade_m']:.4f} | {g['oracle_ade_m']:.4f} | {g['selection_gap_m']:.4f} | {h['ade_1p0s_m']:.4f} | {h['ade_3p0s_m']:.4f} |")
    lines+=['','단위 m. 최선 후보는 미래 정답을 본 분석용 하한이며 실제 운전에서 사용할 수 없습니다.',
        '선택 gap은 집계 분해이며 인과 증명이 아닙니다. 앞 2초 보조 손실을 학습했지만 기존 보고서가 제공하는',
        '1초/3초/6.4초 지표를 그대로 비교합니다. 측정하지 않은 2초 평가 수치를 만들어내지 않습니다.','',
        '[지도 작은 객체 오류 수치](mapping_errors.json) · [지도 실패 장면](small_object_failures.png)','',
        '다음 데이터는 TRAIN 전용 경로에서 이동 회전·근접 신호등/표지판을 수집하고, 기존 DEV/TEST는 학습에 넣지 않습니다.',
        '0.5m/s 이하 표본만 통과했던 기존 266개를 이동 회전 보강으로 세지 않습니다.',
        '새 데이터 없이 학습 횟수만 늘리는 다음 실험은 자동 실행하지 않습니다. 기능 헤드 추가보다 기본 성능을 우선합니다.']
    with (output/'README.md').open('x') as f:f.write('\n'.join(lines)+'\n')
    print('DRIVING_ERROR_ANALYSIS_COMPLETE '+next_action,flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--wait-seconds',type=int,default=0);a=p.parse_args()
    if not 0<=a.wait_seconds<=7200:raise ValueError('bounded wait required')
    mapping_analysis()
    deadline=time.monotonic()+a.wait_seconds
    while not (REPORT/'results/driving/summary.json').exists():
        if time.monotonic()>=deadline:raise TimeoutError('Driving summary unavailable; no training changed')
        if (REPORT/'results/timeout.json').exists():raise TimeoutError('Upstream collector timed out')
        time.sleep(min(30,max(.01,deadline-time.monotonic())))
    driving_analysis()


if __name__=='__main__':main()
