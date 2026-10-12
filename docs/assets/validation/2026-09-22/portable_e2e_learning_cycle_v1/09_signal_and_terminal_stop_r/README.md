# R — 신호등 사전 감속과 종료 전 정지 개선 실험

<!-- HH_260906 - Keep observed stop improvement separate from full data acceptance and model performance. -->

## 결과 요약

2026-09-22 로컬 CARLA에서 연구용 설정 4개를 실제 실행했다. **R03/R04에서 목표 창 안의 저속 정지 확인 2초 + 정지 유지 6.5초를 완료**했다. 종료 시 이동 상태에서 full-brake tail로 넘어가던 문제는 이 두 실행에서 나타나지 않았다.

그러나 신호등 긴급 제동에 따른 감속 초과 1구간이 남았고, 전체 raw 미래 궤적 검증은 저속 곡률·거리 일관성 문제 때문에 미통과다. **모든 R 실행은 실패/연구 자료로 보존하며 학습 승인하지 않는다. 기본 수집기나 기존 모델을 교체하지 않았다.**

## 구현

- 별도 `collect_research_stop_preview.py`와 `run_owned_stop_preview_trial.sh`를 추가했다. 공용 collector 및 runtime gate는 수정하지 않았다.
- CARLA native stop waypoint에서 같은 road/lane, 같은 진행 방향, 경로 횡오차 2m 이내, 전방 60m 이내의 Red/Yellow 신호를 선정한다. BasicAgent의 신호등 판정 함수를 별도로 호출하거나 cache를 수정하지 않는다.
- 정지 여유 거리를 제외한 거리로 정상 요청 속도를 제한한다. 긴급 제동은 원래 BasicAgent가 정상 planner 출력 이후 그대로 적용한다.
- 정상 제동은 0.04 상한, 초당 0.02 증가율, throttle 0.20 상한을 시험했다. R02부터 실제 속도 2m/s 이하에서는 정상 제동을 0으로 제한하여 타력 감속을 시험했다. 이는 연구용 변경이며 제동 성능 보장이 아니다.
- 기존 comfortable_v2의 goal window, 정지 속도, 2초 dwell, overshoot 실패 및 실측 QA는 유지했다. 단순 위치 도달로 성공 처리하지 않는다.
- R02부터 목표 접근 속도 상한 `gain * max(remaining - 0.75, 0)`을 추가했다. R02 gain=0.2/s, R03/R04=0.1/s.

## 실행별 실측

Town01 right p02 / seed 2026092203 / ClearNoon / Prius / Epic / 지면 스폰 / 물리20Hz / 6카메라10Hz / ACK 제어. 최대 요청 속도는 28.8km/h이지만 **실제 최고 속도는 아래와 같아 30km/h 주행 검증이 아니다.**

| 실행 | 주요 차이 | 목표 정지 | 실제 최고 km/h | 카메라 최저 감속 m/s² | -6 초과 감속 구간 | raw 미래 통과 |
|---|---|---|---:|---:|---:|---:|
| Q 기준 | 곡선 속도 제한, 기존 종료 | 이동 후 강제 정지 | 별도 Q 기록 | -24.17 | 5 | 342/524 |
| R01 | 신호 여유8m, 정상 brake 상한/ramp, comfortable_v2 | overshoot 실패, 마지막 2.33m/s | 13.92 | -11.00 | 1 | 335/549 |
| R02 | 저속 타력 + terminal gain0.2 | overshoot 실패, 마지막 0.52m/s | 12.39 | -8.83 | 1 | 425/604 |
| R03 | terminal gain만0.1로 감소 | 정지·유지 완료, 전체 QA 실패 | 11.53 | -8.59 | 1 | 481/949 |
| R04 | 신호 여유만8→12m | 정지·유지 완료, 전체 QA 실패 | 10.93 | -7.19 | 1 | 468/960 |

R02 중간 telemetry에서 약0.48m/s가 보였지만 마지막 native 상태는0.52m/s였다. 표는 최종 native 상태 기준이다. R01/R02의 collector 오류 문구는 `BasicAgent reported done before ...`이지만 실제 goal record의 실패 이유는 `comfortable_goal_endpoint_overshoot`이다. 이를 단순 planner 완료 조건 문제로 오해해 우회하지 않았다.

네 실행 모두 기록된 충돌/차선 침범은 0. R03/R04는 각각 목표 약0.920/0.929m 앞에서 dwell 완료 후 tail 130 native state를 기록했다. R04 마지막 driving 속도는0.0442m/s, tail 종료 속도는0이다. **정지 완료와 전체 QA 합격은 별개**다.

## 신호등 효과와 한계

R01 긴급 제동 호출은0회였지만 정상 brake0.04에서도 저속 급감속이 발생했다. R02 이후 저속 타력 감속을 적용하자 신호등 긴급 제동 영역에 들어가 다시 개입했다. R04의 남은 -7.19m/s² 구간에서도 actor17 신호등 위험 판단 및 긴급 제동 호출이 기록됐다.

긴급 제동 tick 수 R02=369, R03=370, R04=363은 정지 중 반복 호출도 포함한다. 독립 급제동 사건 수나 실패 구간 수가 아니다. R04는 정상 감속 여유를 늘렸지만 문제를 완전히 해결하지 못했다. 보수적인 조기 정지로 인해 실제 속도와 처리 시간이 낮아지는 대가도 있다.

## 전체 데이터 품질은 개선됐다고 볼 수 없음

R04 raw 실패 첫 원인은 곡률244, 가감속64, 적분 거리 불일치183, 최소 이동 범위1개다. raw 통과율은 48.8%로 Q의65.3%보다 낮다. 각 실행 시간과 완전 미래 창 개수가 다르고 정지/저속 비중도 달라 동일 분모의 모델 A/B 점수가 아니다. 일부 급제동 개선만으로 전체 성능 향상을 주장하지 않는다.

현재 완료한 것은 **연구용 개선 구현 + 로컬 시험 + 실패 분리**다. 기존 9영역/30기능 로드맵에서 속도 추종·신호등 정지·목표 정지 관련 데이터 품질을 다루고 있다. 자체 모델 승인 closed-loop 기능은0/30으로 변함없다. 모델 학습이나 실차 평가를 수행한 결과가 아니다.

![Q 대비 R03/R04 실측 경로·속도·가감속](final_pilot_comparison.png)

## 다음 작업

1. R03의 목표 정지 절차를 보존하고, 신호등 접근을 별도로 분리한다. 일괄 저속 brake=0은 부족했다. 실제 감속 피드백과 저속 제동 응답을 사용한 정상 제동을 검증하되 emergency strength는 유지한다.
2. 저속 곡률/적분 거리 실패를 native snapshot, base_link 변환, 제어 및 속도 정의와 대조한다. 미래 창 중복을 독립 사건으로 세지 않는다. label clipping/기준 완화로 통과시키지 않는다.
3. 한 경로가 전체 수집 검증을 통과한 후 다른 TRAIN 경로/seed로 확대한다. 현재는 전체 Town 테스트나30km/h 성능 검증 단계가 아니다.
4. 유효 데이터 확보 후 고정 DEV/TEST를 유지하며 원격 개인 venv·GPU0에서 동등 학습량 재학습하고 로컬 learned closed-loop 검증으로 이어간다. 이번에는 원격 학습을 시작하지 않았다.

다음 단위는 저속 원시 사건 분석과 정상 제동의 한 조건 대조 실험으로 약30~60분 작업 범위를 예상한다. 재학습 시간/비용은 데이터 승인 후 산정한다. 청구 토큰 사용량/잔액은 이 실행 기록에서 확인할 수 없다.

## 재현 및 보존

- 전체 원본: `artifacts/training/2026-09-22/stop_preview_r_v1/01_town01_preview` ~ `04_town01_signal_margin`의 `episode.partial`.
- `owner_plan.json`: 정확한 CLI, 지도/route hash. `provenance`: 해당 실행의 코드. 실행마다 소스 원본을 보존하므로 현재 R04 코드로 이전 revision 결과를 재현했다고 주장하지 않는다.
- `owner_result.json`: 네 실행 모두 소스 보존/실행 중 불변 확인. 소유 CARLA 프로세스만 정리했다.
- `summary.json`, `summary_v4.json`, `raw_audit*.json`, `hazard_audit*.json`: 분석 결과.
- `tests_v3.xml`: 관련214개 통과. 신호 lane/direction/state 필터와 원래 긴급 제동 반환 유지 테스트 포함.
- 분석: `python3 -m scripts.e2e.summarize_stop_preview_trials --trial TRIAL_DIR --output NEW_SUMMARY.json`

설치, 원격 GPU 작업, 다른 테스트 종료, 자료 삭제, commit/push는 하지 않았다. 이번 화면 자료는 실측 그래프와 원본 6카메라 영상이며 Autoware/RViz 화면으로 표시하지 않는다.
