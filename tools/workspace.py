#!/usr/bin/env python3
"""HH_260906 - Safe workspace navigation, complete script index and read-only job status."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
# HH_260906 - Driving remains completed AH; mapping has a separate additive-data AI experiment.
RESULTS=ROOT/'docs/assets/validation/2026-09-26/dual_model_recovery_ah/results'
MAPPING_RESULTS=ROOT/'docs/assets/validation/2026-09-26/dual_data_ai/results/mapping_ai_v2'
REMOTE='/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics'
JOBS={'주행':'driving_near_20260926_ah_v1','지도':'map_expanded_20260926_ai_v2'}
LOGS={'주행':'driving_20260926_ah_v1.launch.log','지도':'mapping_20260926_ai_v2.launch.log'}
CATEGORIES=('01 주행·학습', '02 지도·인식', '03 시뮬레이터·수집·제어', '04 평가·분석·자료', '05 환경·운영·기타')


def category(path):
    name=path.name.lower()
    if any(s in name for s in ('mapping','map_perception','map_native','map_expansion','map_bundle','lanelet','opendrive')):
        return CATEGORIES[1]
    if name.startswith(('analyze','audit','diagnose','evaluate','summarize','render','plot','publish','verify','compare','inspect','aggregate','organize')):
        return CATEGORIES[3]
    if any(s in name for s in ('train','learning','recovery','selector','recalibrat','replay_seed','continuation')):
        return CATEGORIES[0]
    if any(s in name for s in ('carla','collect','route','scenario','actuation','brak','steer','curve','speed','sensor','capture')):
        return CATEGORIES[2]
    return CATEGORIES[4]


def catalog_text(root=ROOT):
    files=sorted(p for p in (root/'tools').rglob('*') if p.is_file() and p.suffix in ('.py','.sh') and '__pycache__' not in p.parts)
    rows=['# 실행·분석 파일 전체 목록','',
          '<!-- HH_260906 - Discover every script without moving hashed source files or declaring old code unused. -->','',
          f'대상: tools/ 아래 Python·Shell {len(files)}개. 파일명 기반 용도 분류이며 실행 권한·검증 완료 표시가 아니다.',
          '일상 작업은 [작업별 안내](README.md)를 먼저 이용하세요. 원격 전용 학습 스크립트를 로컬에서 실행하지 마세요.','']
    for group in CATEGORIES:
        matches=[p for p in files if category(p)==group]
        rows += [f'## {group} ({len(matches)}개)','']
        rows += [f'- [{p.name}](../../{p.relative_to(root).as_posix()}) — `{p.parent.relative_to(root)}`' for p in matches]
        rows.append('')
    return '\n'.join(rows)+'\n'


def describe(role,payload):
    if role=='주행':
        groups=payload['evaluation']['subgroups']
        return (f"완료·미배포 | 우회전 ADE {groups['town03_right_turn_dev']['metrics']['selected_ade_m']:.3f}m"
                f" | 직진 ADE {groups['town03_straight_dev']['metrics']['selected_ade_m']:.3f}m"
                f" | 비퇴행 {payload['versus_original_k1']['pass_count']}/9")
    return f"파일럿 완료·미배포 | mIoU {payload['mean_iou_present']:.3f} | 완성된 HD 지도 아님"


def status(remote=False):
    print('조회 시각:',datetime.now().astimezone().isoformat(timespec='seconds'))
    print('주행 기능 승인: 0/30 (2026-09-22 문서 기준, 실시간 기능 판정 아님)')
    for role,name in JOBS.items():
        if remote:
            directory=REMOTE+'/'+name
            logfile=REMOTE+'/'+LOGS[role]
            command=f'if test -f {directory}/summary.json; then cat {directory}/summary.json; elif test -f {logfile}; then tail -n 1 {logfile}; else printf NOT_STARTED_OR_QUEUED; fi'
            try:
                result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','pro6000-training',command],
                    capture_output=True,text=True,check=True,timeout=15).stdout.strip()
                try:
                    data=json.loads(result)
                    message=describe(role,data) if 'status' in data else f'마지막 로그 step={data.get("step","미확인")} (프로세스 생존 여부와 별개)'
                except (ValueError,KeyError):message=result[:400]+' (로그 기록 기준)'
            except (OSError,subprocess.SubprocessError) as error:message='원격 조회 실패: '+str(error)
        else:
            file=(RESULTS/'driving' if role=='주행' else MAPPING_RESULTS)/'summary.json'
            message=describe(role,json.loads(file.read_text())) if file.exists() else '완료 결과 아직 로컬에 없음 — 현재 상태는 --remote로 확인'
        print(f'{role}: {message}')
    print('주행 결과 폴더:',RESULTS)
    print('지도 결과 폴더:',MAPPING_RESULTS)
    # HH_260906 - A completed teacher-control experiment must not be presented as new driving-model training.
    followup=ROOT/'docs/assets/validation/2026-09-27/drive_control_aj/README.md'
    if followup.exists():print('주행 후속: AJ 출발·정지 제어 비교 — 학습 자료 미승인·재학습 미실행:',followup)


def main():
    p=argparse.ArgumentParser(description='HH workspace: read-only status and categorized navigation; never starts training')
    sub=p.add_subparsers(dest='command',required=True)
    s=sub.add_parser('status',help='학습 상태 조회 (읽기 전용)');s.add_argument('--remote',action='store_true')
    sub.add_parser('guide',help='작업별 진입점 표시')
    sub.add_parser('catalog',help='전체 스크립트 색인 갱신')
    a=p.parse_args()
    if a.command=='status':status(a.remote)
    elif a.command=='catalog':
        (ROOT/'docs/workflows/EXECUTABLES.md').write_text(catalog_text(),encoding='utf-8')
        print('Updated docs/workflows/EXECUTABLES.md')
    else:
        for name in ('01_driving','02_mapping','03_simulation','04_results','05_maintenance'):
            print(ROOT/'docs/workflows'/name/'README.md')


if __name__=='__main__':main()
