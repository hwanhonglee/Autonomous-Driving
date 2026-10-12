"""HH_260906 - Publish fixed historical comparisons after the owned dual-learning monitor finishes."""
import json
from pathlib import Path
import sys
import time


ROOT=Path(__file__).resolve().parents[2]
RELATIVE='docs/assets/validation/2026-09-26/dual_model_recovery_ah'


def comparison_table(reports):
    """HH_260906 - Expose straight regression alongside turn improvements; never promote."""
    lines=['| 주행 모델 | 우회전 ADE (m) | 직진 ADE (m) | K1 비퇴행 | 절대 기준 |',
           '|---|---:|---:|---:|---:|']
    for name,report in reports.items():
        groups=report['evaluation']['subgroups']
        turn=groups['town03_right_turn_dev']['metrics']['selected_ade_m']
        straight=groups['town03_straight_dev']['metrics']['selected_ade_m']
        lines.append(f"| {name} | {turn:.4f} | {straight:.4f} | {report['versus_original_k1']['pass_count']}/9 | {report['strict_gate']['metric_pass_count']}/9 |")
    return '\n'.join(lines)


def main():
    # HH_260906 - No remote execution or GPU use: wait for local evidence, then copy the owned checkpoint.
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    from collect_dual_learning_results import fetch
    import hashlib
    root=ROOT/RELATIVE;results=root/'results';deadline=time.monotonic()+7200
    while not (results/'complete.json').exists():
        if (results/'timeout.json').exists() or time.monotonic()>=deadline:
            raise TimeoutError('No completed dual-model evidence; remote jobs left unchanged')
        time.sleep(30)
    previous=ROOT/'docs/assets/validation/2026-09-22/portable_e2e_learning_cycle_v1'
    references={
        'AD':previous/'21_full_continuation_ad/completed_summary.json',
        'AF':previous/'24_root_cleanup_and_controlled_learning/results/driving/summary.json',
        'AG':previous/'27_driving_continuation_ag/results/driving/summary.json',
        'AH':results/'driving/summary.json'}
    reports={name:json.loads(path.read_text()) for name,path in references.items()}
    mapping=json.loads((results/'mapping/summary.json').read_text())
    lines=['# AH 주행·지도 최종 비교','',
        '<!-- HH_260906 - Complete offline evaluations do not approve model deployment. -->','',
        '두 학습 및 DEV 평가 완료. 자체 모델 폐루프 기능 승인 0/30은 변경하지 않습니다.','',
        comparison_table(reports),'',
        'ADE는 낮을수록 좋습니다. 과거 K1은 우회전 3.8325 m / 직진 0.5113 m였습니다.',
        '각 항목의 기준·원문은 results/driving/summary.json에 보존됩니다. 새 독립 TEST 결과가 아닙니다.','',
        '## 지도','',
        f"동일 640×360 입력 mIoU: {mapping['parent_native']['mean_iou_present']:.4f} → {mapping['mean_iou_present']:.4f}.",
        f"동일 320×180 입력 mIoU: {mapping['parent_common_320']['mean_iou_present']:.4f} → {mapping['candidate_common_320']['mean_iou_present']:.4f}.",
        '320×180 차선 표시 퇴행과 작은 신호등/표지판의 낮은 절대 성능도 함께 보아야 합니다.',
        '지도 회수 완료는 정밀지도/교통규칙/실차 품질 승인이 아닙니다.','',
        '[상세 설정과 해석](README.md) · [예측 GIF](results/mapping/learned_perception.gif)','',
        '후속 우선순위: 회전/직진/속도의 비퇴행 여부 검토 → 유망 후보만 별도 로컬 지연/폐루프 검증.',
        '실패한 후보를 자동 배포하거나 학습 데이터/안전 기준을 바꾸지 않습니다.','']
    # HH_260906 - Fetch only this fixed owned run, checking both transfer hash and declared checkpoint hash.
    checkpoint=ROOT/'storage/experiments/training/2026-09-26/dual_recovery_ah/driving/final_checkpoint.pt'
    checkpoint.parent.mkdir(parents=True,exist_ok=True)
    remote='/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/driving_near_20260926_ah_v1'
    fetch(remote+'/K1_current_plus_successful4/training/final_checkpoint.pt',checkpoint)
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=reports['AH']['training']['checkpoint_sha256']:
        raise ValueError('Downloaded driving checkpoint differs from evaluated checkpoint')
    with (root/'comparison.md').open('x') as f:f.write('\n'.join(lines))
    print('AH_COMPARISON_AND_CHECKPOINT_PUBLISHED_NOT_PROMOTED',flush=True)


if __name__=='__main__':main()
