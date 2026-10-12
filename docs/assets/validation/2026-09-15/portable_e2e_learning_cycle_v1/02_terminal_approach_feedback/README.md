# 02 — 저속 종점 접근 제어 구현과 실제 A/B 검증

<!-- HH_260906 - Distinguish successful collection from worsened trajectory quality; preserve every original failure and do not promote the candidate. -->

## 결론: 두 회차 모두 정지 완료, 피드백 B는 품질 악화로 미채택

남은 거리와 실측 속도를 이용하는 **저속 종점 접근 제어(v9)**를 구현하고,
로컬 CARLA Town07 직진에서 기존 무페달 감속 A와 피드백 B를 각각 한 번 실행했다.
두 회차 모두 원래 목표 창 안에서 정지·2초 유지·6.5초 후속 관측을 완료했다.
그러나 B는 **곡률 실패 증가와 새로운 횡가속도 기준 실패**가 있어 채택하지 않는다.
이번 결과를 새 학습에 넣거나 모든 맵에서 성공했다고 판정하지 않았다.

| 항목 | A — 기존 무페달 감속 | B — 저속 피드백 |
| --- | ---: | ---: |
| 실행·카메라·제어·원시 속도 검사 | 통과 | 통과 |
| 마지막 목표 거리 | 0.996705 m | 0.293478 m |
| 마지막 보고 속도 / 정지 유지 | 0 m/s / 2초 | 0 m/s / 2초 |
| 전체 원시 상태 / 6카메라 묶음 | 1,472 / 736 | 1,511 / 756 |
| 완전한 64점 미래 창 | 671 | 691 |
| 곡률 기준 실패 창 | **115 / 671 (17.14%)** | **201 / 691 (29.09%)** |
| 겹치는 곡률 실패 / 고유 실패 시점 | 115 / 3 | 938 / 16 |
| 최대 평가 곡률 | 18.444801 rad/m | **573.865933 rad/m** |
| XY 기반 횡가속도 기준 실패 창 | 0 | **64** |
| 최대 XY 기반 횡가속도 지표 | 0.414636 m/s² | **5.503501 m/s²** |
| 나머지 미래 지표 7개 실패 | 0 | 0 |
| 학습 데이터 편입 / 모델 채택 | 안 함 | **안 함** |

여기서 곡률·횡가속도는 원래 미래 XY 궤적 검사에서 계산한 지표다. 차량 IMU에서
같은 크기의 횡가속도를 측정했다는 뜻은 아니다. 원시 속도 변화율 검사의 통과와도 구분한다.
겹치는 미래 창들은 독립적인 938번의 주행 실패가 아니다.

최종 비교 상태는 `BOTH_COMPLETED_CONDITION_SPECIFIC_NOT_ADMITTED`이다.
이는 **두 번의 진단 수집 완료**이며, 개선 성공이나 안전한 모델 주행 승인이 아니다.

## 무엇을 구현했나

두 모드 모두 물리 계산 간격을 **최대 10 ms**로 고정했다. 이전에 목표 밖에서 정지한
5 ms 설정은 이번에 사용하지 않았다. 순항 목표는 **28.8 km/h(30 km/h급)**이며,
0.12~0.35 m/s라는 작은 목표 속도는 마지막 저속 접근 구간에만 적용된다.

- 공통: Town07 TRAIN 직진 210.598 m, Prius, ClearNoon, seed 0, Epic,
  원시 상태·제어 20 Hz, 원본 6카메라 10 Hz, 기존 PID 게인.
- A: 기존처럼 감속 구간에서 가속·제동 페달을 모두 0으로 둔다.
- B: 기존 감속에 진입한 뒤 실측 속도 ≤0.75 m/s에서 피드백 상태를 활성화한다.
  남은 거리에 따라 저속 목표를 낮추고, 실측 속도가 목표 이상이면 가속 요청은 0이다.
  정상 가속 페달은 최대 0.15, 증가율은 초당 0.10으로 제한한다.
- 목표 창 안에서는 가속 요청을 0으로 한다. 기존 실측 정지 조건이 충족되면
  원래 즉시 유지 제동을 적용한다. 비상 제동의 우선권은 변경하지 않았다.

**목표 거리 1 m, 정지 속도 ≤0.1 m/s, 유지 2초, 감속 제한 45초, 후속 관측 6.5초와
원래 품질 기준은 그대로다.** 상세 수식·위험·사전 중단 정책은 [실행 전 설계](design.md),
정확한 명령과 소스 해시는 [고정 계획](plan.json)에 있다. 두 회차 모두 자동 재시도 없이 한 번씩 실행했다.

기존 PID의 게인·업데이트 방법을 고치거나 내부 상태를 초기화하지 않았다. 다만 B의 목표 속도 입력이
바뀌므로 피드백 활성화 뒤 **PID 내부 이력까지 A와 같다고 주장하지 않는다.**
실측 최고 속도를 0.35 m/s 이하로 보장한 것도 아니다. 이것은 목표 값이며 기존 실측 안전 검사를 유지했다.

## 실제 경로·속도·페달 분석

![실제 감속부터 마지막 관측까지의 거리·속도·페달·측방 변위·yaw](motion_comparison/coast_to_stop_motion.png)

그림은 두 회차의 감속 시작부터 마지막까지 **모든 원시 관측**을 사용한다.
각 감속 시작을 표시상의 0초로 두었으며, 원시 시각이나 학습 정답은 바꾸지 않았다.
[그래프 수치·원본 해시](motion_comparison/manifest.json)를 함께 보존했다.

1. 새 A는 이전 10 ms 기준 회차와 **1,472개 상태 × 26개 물리·제어 필드가 정확히 일치**했다.
2. 새 A/B도 첫 양의 피드백 페달 요청을 만드는 관측까지 **1,114개 상태가 정확히 일치**했다.
   해당 관측의 다음 명령만 A는 0, B는 약 0.000106이었다.
3. B는 양의 페달을 55개 연속 명령에서 요청했다. 첫/마지막 명령 원본 시각 차이는 2.70초,
   실제 요청 최대값은 약 0.062851이었다. 감속 타이머를 새로 시작하지 않았다.
4. B는 A보다 목표에 약 0.703 m 더 가까이 정지했지만 전체 관측이 1.95초 길어졌고,
   궤적 품질은 악화됐다. **목표에 가까워졌다는 이유만으로 개선안으로 채택하지 않는다.**

이번 고정 10 ms 조건에서는 A도 이미 정지에 성공했다. 따라서 이전 5 ms의 조기 정지 문제를
B가 복구했다고 입증한 실험은 아니다.

피드백 상태 활성화와 실제 양의 페달 요청은 다르다. B의 활성화 관측은 native index 1008,
첫 양의 요청 관측은 index 1113이다. 활성화 화면에서 페달이 0인 것은 누락이 아니다.

출발의 원래 곡률 실패는 남았고, B에는 피드백 구간의 고유 실패 시점 13개가 추가됐다.
이 13개는 약 30~34 mm씩 움직이는 구간에도 존재하므로 모두 정지 상태의 미세 잡음으로 치부하지 않는다.
정지 경계의 최대 곡률은 극저속·작은 변위와 카메라 표본 위치의 영향을 함께 받는다.
특히 B 최대값은 해당 정지 관측을 첫 미래점으로 삼는 계산이다. 특정 내부 물리 원인을 확정하거나
검사 예외를 적용하지 않았다. [프레임별 독립 분석](residual_analysis.md)에 구분해 기록했다.

## 차량 중심 실제 화면 — PNG 14장·GIF 2개

아래 화면은 **원본 6카메라 전체 화각 + 차량 중심 ±20 m 경로 + 과거 10초 실측 궤적 + 속도·제어**다.
Autoware UI나 자체 학습 모델의 직접 운전 화면은 아니다. CARLA BasicAgent 기반 데이터 수집 장면이다.
카메라 PNG 13장과 위 그래프 1장, 실제 프레임 GIF 2개를 보존했다.

| 장면 | A — 무페달 감속 | B — 피드백 |
| --- | --- | --- |
| 첫 원본 카메라 | [PNG](visuals/01_coast_A/01_first_saved_camera_anchor.png) | [PNG](visuals/02_feedback_B/01_first_saved_camera_anchor.png) |
| 정상 PID 진입 | [PNG](visuals/01_coast_A/02_launch_to_normal_pid.png) | [PNG](visuals/02_feedback_B/02_launch_to_normal_pid.png) |
| 무페달 감속 진입 | [PNG](visuals/01_coast_A/03_normal_pid_to_zero_pedal_coast.png) | [PNG](visuals/02_feedback_B/03_normal_pid_to_zero_pedal_coast.png) |
| 피드백 상태 활성화 | **해당 없음·미관측** | [PNG — 아직 페달 0](visuals/02_feedback_B/04_zero_pedal_coast_to_terminal_feedback.png) |
| 정상 유지 제동 요청 시점 | [PNG](visuals/01_coast_A/05_zero_pedal_coast_to_service_brake_hold.png) | [PNG](visuals/02_feedback_B/05_zero_pedal_coast_to_service_brake_hold.png) |
| 완전 유지 제동 읽기 | [PNG](visuals/01_coast_A/06_first_full_hold_brake_readback.png) | [PNG](visuals/02_feedback_B/06_first_full_hold_brake_readback.png) |
| 마지막 원본 카메라 | [PNG](visuals/01_coast_A/07_last_saved_camera_anchor.png) | [PNG](visuals/02_feedback_B/07_last_saved_camera_anchor.png) |
| 실제 프레임 요약 | [GIF](visuals/01_coast_A/sampled_actual_camera_timelapse.gif) | [GIF](visuals/02_feedback_B/sampled_actual_camera_timelapse.gif) |

각 GIF는 균등하게 고른 실제 24프레임을 400 ms씩 표시하는 9.6초 요약이다.
A는 약 73.5초, B는 약 75.5초의 시뮬레이터 시간을 담는다. 실시간 10 Hz 재생이나 화면 FPS 측정이 아니다.
제동 요청의 원시 시점과 실제 선택한 카메라의 0/50 ms 차이는 각 manifest에 그대로 기록했다.
존재하지 않는 A 피드백 화면을 다른 장면으로 대체하지 않았다.

## 데이터·검사·보존 범위

새 수집 합계는 **2,983개 상태, 1,492개 6카메라 묶음, 8,952개 JPEG**다.
완전한 미래 창은 1,362개이며 후속 정지 문맥용 묶음 130개도 보존했다.
원시 간격 2,981개와 제어 기록 2,983개를 검산했고, 총 2,991개 제어 ACK가 수락됐다.
ACK 수락은 물리 작동 시각이나 학습 적합성을 증명하지 않는다.

[전체 검사 요약](analysis/summary.json) · [A/B 비교](analysis/paired_comparisons.json) ·
[모든 미래 창·무손실 압축](analysis/future_windows.jsonl.gz) · [원시 간격 분석](analysis/native_residuals.jsonl) ·
[제어 결정·명령·실제 상태](analysis/control_policy_witnesses.jsonl) · [원본 카메라 검사 목록](analysis/image_inventory.jsonl)

관련 기존 검사 34개 파일에 새 검사를 포함해 **1,300 passed / 1 skipped (57.51초)**다.
[JUnit 기록](regression.xml)을 보존했다. 기존 안전한 weights-only exact-resume 조건의 skip을 우회하지 않았다.
이는 코드 검사 통과이지 B의 궤적 품질 통과가 아니다. 별도 읽기 전용 전체 재검산도 완료했다.
두 회차의 CARLA 준비·수집 후·종료 확인은 모두 통과했고, 기록된 소유 프로세스만 정리했다.

이전 공개 자료 350개를 변경하지 않았다. 전체 원시 JPEG/상태와 이전 실패 기록은 비공개 artifacts에 남겼다.
공개 사본의 큰 미래 창 파일만 무손실 압축했으며 [복사·압축 원본 해시](publication_manifest.json)를 제공한다.
Git clone에는 선택한 화면과 분석 결과가 포함되지만 비공개 전체 원시 JPEG는 포함되지 않는다.
[소스 사본](code_snapshot/run_case.py)은 원래 저장소 경로를 전제로 한 실행 증거이며 독립 설치 도구가 아니다.

원본 데이터와 고정 소스가 있는 이 저장소에서 CARLA를 새로 켜지 않고 재검산하려면:

```bash
python3 scripts/e2e/summarize_carla_terminal_approach.py \
  --campaign-root artifacts/training/2026-09-15/terminal_approach_v9 \
  --plan artifacts/training/2026-09-15/terminal_approach_v9/plan.json \
  --plan-sha256 11fd19f89db1dac0aaab203bbec845528e6e4be4c26101ce4069be9590b94087 \
  --verify-only
```

## 다음 단계와 두 환경

로컬: 이번 저속 접근 기능 구현과 실제 A/B를 완료했다. **B는 미채택**이며 기존 기본 설정을 자동 변경하지 않았다.
다음은 피드백 구간의 페달 변동·작은 위치/yaw 변동과 정지 경계의 곡률 실패를 분리해,
정상 저속 페달 응답을 더 안정적으로 만드는 새 후보를 별도로 설계·검증하는 순서다.
카메라 시점을 유리하게 옮기거나 정답·기준을 바꿔 통과시키지 않는다.

PRO6000: **이번에는 SSH·새 학습·설치를 실행하지 않았다.** 원격 사용량도 새로 확인하지 않았다.
데이터 품질과 독립 TRAIN/VAL 검증이 준비된 뒤 GPU 0·hwanhong 개인 venv에서 다음 학습을 진행한다.
현재 학습 모델 직접 운전으로 검증 완료한 기능은 여전히 **0/30**이다.
실차 운용 승인, 모든 Town 검증, Git push, 원격 Autoware 빌드를 수행했다는 뜻이 아니다.

[9월 15일 최신 목록](../CURRENT.md) · [이전 물리 계산 간격 비교](../01_native_boundary_and_substeps/README.md)
