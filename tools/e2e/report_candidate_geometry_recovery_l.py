#!/usr/bin/env python3
"""HH_260906 - Verify complete L training, paired sampling and final gates before publication."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from scripts.e2e import run_moving_multiroute_recovery_k as r
from scripts.e2e import run_candidate_geometry_recovery_l as l
from scripts.e2e.report_moving_recovery_k import plot_results


def history(path):
    """HH_260906 - Read the actual completed epoch journal, not estimated progress."""
    return [json.loads(line) for line in path.read_text().splitlines()]


def publish(root,baseline,output):
    """HH_260906 - Independently recheck the one-variable whole-model comparison without forward inference."""
    plan=r.read_json(root/'plan.json'); summary=r.read_json(root/'summary.json'); decision=r.read_json(root/'decision.json')
    old_plan=r.read_json(baseline/'plan.json'); old_summary=r.read_json(baseline/'summary.json')
    r.require(not (root/'failed.json').exists() and summary['status']=='COMPLETE_NOT_PROMOTED','L run incomplete')
    r.require(summary['schema']==l.SCHEMA,'L schema differs')
    l.validate_experiment_contract(plan,old_plan)
    for name,expected in plan['source_sha256'].items():
        r.require(r.digest(root/'source'/name)==expected and r.digest(ROOT/name)==expected,'source differs: '+name)
    r.require(r.digest(root/'plan.json')==summary['plan_sha256'],'plan digest differs')
    r.require(r.digest(root/'decision.json')==summary['decision_sha256'],'decision digest differs')
    r.require(r.digest(baseline/'summary.json')==plan['baseline_summary_sha256'],'baseline differs')
    train=summary['training']
    r.require(train==r.read_json(root/l.ARM/'training/run.json'),'training record differs')
    r.require(train['global_step']==15000 and train['samples_seen']==60000 and train['dev_evaluation_count_during_fit']==0,
              'fit budget or evaluation phase differs')
    r.require(train['initial_model_state_sha256']==plan['initial_model_state_sha256'],'initial model differs')
    r.require(plan['base_initial_state_sha256']==old_plan['model']['seeded_initial_model_state_sha256'],'base init differs')
    r.require(r.digest(root/train['checkpoint'])==train['checkpoint_sha256'],'checkpoint differs')
    evaluation=r.read_json(root/l.ARM/'final_dev_evaluation.json')
    r.require(r.digest(root/l.ARM/'final_dev_evaluation.json')==summary['evaluation_sha256'],'evaluation differs')
    r.require(evaluation['sample_count']==665 and evaluation['checkpoint_sha256']==train['checkpoint_sha256'],'evaluated scope differs')
    old_ref=old_summary['evaluations'][l.ARM]
    r.require(r.digest(baseline/old_ref['path'])==old_ref['sha256'],'baseline evaluation digest differs')
    old_eval=r.read_json(baseline/old_ref['path'])
    strict=r.strict_gate(evaluation); relative=r.paired_nonregression(old_eval,evaluation)
    r.require(strict==decision['strict'] and relative==decision['relative_to_K1'],'independent gate mismatch')
    r.require(decision['pass_all']==(strict['pass'] and relative['pass']) and decision['promotion_eligible'] is False,'decision differs')
    old_history=history(baseline/l.ARM/'training/epoch_history.jsonl')
    new_history=history(root/l.ARM/'training/epoch_history.jsonl')
    r.require(len(old_history)==len(new_history)==20,'history length differs')
    for index,(old,new) in enumerate(zip(old_history,new_history)):
        r.require(old['schedule_sha256']==new['schedule_sha256'],'paired sampling differs')
        r.require(new['completed_steps']==(index+1)*750 and new['dev_observed'] is False,'epoch accounting differs')
    r.require(not output.exists(),'create-only report output required')
    output.mkdir(parents=True)
    r.write_new(output/'verification.json',dict(status='VERIFIED_L_COMPLETE_NOT_PROMOTED',decision=decision,
        paired_epoch_schedules_equal=20,summary_sha256=r.digest(root/'summary.json'),training=train))
    lines=['# L 후보 형상 인지 — 전체 모델 학습 결과','',
        '<!-- HH_260906 - Report both gains and regressions without promoting the experimental checkpoint. -->','',
        '판정: `'+decision['status']+'`','',
        '기존 K1과 동일 데이터·초기 기본 가중치·20개 epoch 샘플링 순서·손실·15,000 step을 확인했다.',
        '모든 파라미터를 새로 학습했고 최종 DEV 665개를 한 번 평가했다. TEST·폐루프·실차는 미실행이다.','',
        '| 범위 | 지표 | K1 기준 | L | L−K1 |','| --- | --- | ---: | ---: | ---: |']
    for row in relative['checks']:
        lines.append(f"| {row['group']} | {row['metric']} | {row['K0']:.6f} | {row['K1']:.6f} | {row['delta_K1_minus_K0']:+.6f} |")
    lines+=['',f"고정 지표 {strict['metric_pass_count']}/9, 형상 {strict['geometry_pass_count']}/3, K1 대비 비퇴행 {relative['pass_count']}/9.",
        f"학습 시간 {train['wall_seconds']/60:.1f}분, 15,000 step·60,000 sample exposure, 파라미터 {l.PARAMETERS:,}개.",'',
        '기본 모델 대비 용량이 57,601개 증가했다. 이 실험만으로 형상 입력 효과와 추가 용량 효과를 분리하지 않는다.',
        '반복 관측 DEV·단일 seed 결과이므로 통과하더라도 자동 배포하지 않는다.','']
    (output/'README.md').write_text('\n'.join(lines))
    plot_results(dict(histories={r.ARMS[0]:old_history,r.ARMS[1]:new_history},
        evaluations={r.ARMS[0]:old_eval,r.ARMS[1]:evaluation}),output,labels=('K1 baseline','L geometry'),
        training_title='Full-model architecture comparison — same TRAIN, loss and sample order')
    return dict(status='VERIFIED_L_COMPLETE_NOT_PROMOTED',decision=decision['status'],output=str(output))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    parser.add_argument('--baseline-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(publish(args.run_root,args.baseline_root,args.output)))
