# 02 — 정지 상태 미세 움직임: 서비스·주차 브레이크 ABBA 실측

<!-- HH_260906 - Separate completed stationary diagnostics, initialization failures, and future training decisions. -->

**결과: 4회 계측 완료, 학습 데이터 편입·주행 기능 승인은 아님.** 2026-09-14 로컬
CARLA에서 A1 → B1 → B2 → A2를 실행했다. PRO6000에는 접속하거나 학습을 실행하지 않았다.
기존 정답의 작은 변위·곡률 문제를 “가만히 있어도 계속 흔들리는 현상”으로 설명할 수 있는지
먼저 분리한 검사다. METEOR 설치·모델 교체 실험이 아니다.

## 실제 결과

| 순서 | 제동 조건 | 상태값 / 6카메라 묶음 / JPEG | 정지 유지 400표본의 XY 변위·보고 속도 | 종료 |
| --- | --- | ---: | --- | --- |
| A1 | 서비스 브레이크 1, 주차 브레이크 꺼짐 | 470 / 235 / 1,410 | 모두 0 | 정상 |
| B1 | 서비스 브레이크 1, 주차 브레이크 켜짐 | 470 / 235 / 1,410 | 모두 0 | 정상 |
| B2 | B1과 동일 | 470 / 235 / 1,410 | 모두 0 | 정상 |
| A2 | A1과 동일 | 470 / 235 / 1,410 | 모두 0 | 정상 |
| 합계 | 동일 spawn의 반복; 독립 경로 4개가 아님 | **1,880 / 940 / 5,640** | 학습/주행 성공 수에 포함하지 않음 | 4/4 완료 |

모든 회차의 470개 위치·속도·가속도·각속도 관측값은 프레임 ID·시각을 제외하면 일치한다.
실제 제어 readback에서 주차 브레이크 상태의 차이는 확인됐지만, 이 조건에서는 측정 가능한
움직임 차이가 없었다. 따라서 **이미 서비스 브레이크를 끝까지 밟은 정지 상태에서 주차
브레이크를 추가하는 개선 효과는 관찰되지 않았다.** 자동으로 B를 채택하지 않는다.

초기 안착 70표본은 삭제하지 않았다. 네 회차 모두 Actor 기준 첫 관측→마지막 XY 변위는
약 **0.164342 mm**, 안착 중 누적 XY 이동은 **1.019049 mm**, 최대 평면 속도는
**0.01735685 m/s**였다. 이후 hold에서는 Actor와 기존 virtual-base 모두 변위·보고 속도·
위치 변화와 속도 적분의 차이가 0이다. spawn 높이 오프셋 때문에 초기 수직 낙하도 존재한다.
“전체 470표본에서 전혀 움직이지 않았다”는 결과가 아니다.

위치 변화와 속도 적분 차이의 초기 최대값은 50 ms에서 Actor 약 0.568786 mm,
virtual-base 약 0.570242 mm다. 100 ms 두 시작 parity에서도 함께 계산했으며, 작은 값 자체를
허용 오차나 센서 노이즈 상한으로 채택하지 않는다. 시각의 미세한 차이 때문에 적분 통계의
마지막 소수 자리는 회차마다 다를 수 있다. 원시 위치/속도 일치와 전체 JSON 동일성은 다르다.

[전체 수치·5,761개 입력 파일 해시](evidence/summary.json) ·
[실행 전 고정 계획](provenance/stationary_motion_abba_v3_plan.json) ·
[코드·환경 검증서](verification.json)

## 실제 카메라와 위치 분석 그림

아래는 새로 촬영한 센서 영상과 원시 상태값으로 그린 **PNG 8장**이다. 이미지 생성이나
보기 좋은 프레임 선별을 하지 않았다. 각 카메라 패널은 사전에 정한 마지막 tick 470의
640×360 전체 화각을 유지한다. 아래 위치 그래프는 최종 virtual-base를 (0,0) 중앙에 둔다.
앞쪽 약 1.425 m의 점은 Actor 기준점이며, 차가 그 거리만큼 움직였다는 뜻이 아니다.
정지 변위가 작아 경로가 점처럼 보이므로 별도의 mm 단위 시간 그래프를 함께 제공한다.

| 회차 | 변위·속도·방향·적분 차이 | 6카메라 전체 화각 + 차량 중심 위치 |
| --- | --- | --- |
| A1 | [계측 그래프](evidence/01_service_A1_measured_traces.png) | [카메라·위치 패널](evidence/01_service_A1_six_camera_trace.png) |
| B1 | [계측 그래프](evidence/02_parking_B1_measured_traces.png) | [카메라·위치 패널](evidence/02_parking_B1_six_camera_trace.png) |
| B2 | [계측 그래프](evidence/03_parking_B2_measured_traces.png) | [카메라·위치 패널](evidence/03_parking_B2_six_camera_trace.png) |
| A2 | [계측 그래프](evidence/04_service_A2_measured_traces.png) | [카메라·위치 패널](evidence/04_service_A2_six_camera_trace.png) |

![A1 실제 변위와 속도](evidence/01_service_A1_measured_traces.png)

![B1 6카메라와 차량 중심 위치](evidence/02_parking_B1_six_camera_trace.png)

이것은 Autoware/RViz 전체 데스크톱 화면이나 모델의 예측 궤적·자율주행 영상이 아니다.
이번에는 Autoware를 실행하지 않았고 새 주행 GIF도 만들지 않았다. 카메라 패널에는 밝은
하늘과 어두운 도로·수목이 그대로 남아 있으며 노출/색을 보정하지 않았다. JPEG 해시·
디코딩·크기 검증은 학습용 시각 품질 승인을 의미하지 않는다.

## 동일하게 고정한 설정과 계측 범위

- CARLA 0.9.15, C_track_1_0_7, Prius, Epic, ClearNoon, 같은 좌회전 route의 시작점.
  route는 spawn 선택에만 사용하며 이번에 회전 주행을 수행하지 않는다.
- 매번 새 서버·차량. world tick 20 Hz(50 ms), substep 최대 10 ms × 10.
- 동일 명령: throttle 0, steer 0, 서비스 brake 1, reverse/manual-gear false.
  B에서만 hand_brake true. 자동 기어 readback을 기록하며 다른 값으로 강제하지 않는다.
- 안착 70 tick + 정지 유지 400 tick. 두 phase의 제어는 동일하며 phase 경계가 제동 전환은 아니다.
- 6카메라는 매 native tick 관측·수신하고 짝수 tick 235묶음을 저장한다(저장 10 Hz).
  이것은 GUI FPS, 모델 추론 속도 또는 sensor-to-actuator 10 Hz 보장이 아니다.
- 매 명령 ACK와 명령 전 journal, 프레임 고정 상태 snapshot, 앞/뒤 frame을 확인한 제어 readback,
  카메라 frame·시간·이미지 해시, 설정·날씨 복원, 소유 actor 8개 삭제 확인을 기록했다.
- 50 ms 간격은 전체 469개, 안착 69개, hold 끝점 400개다. hold의 1개는 경계를 가로지른다.
  100 ms는 각 parity 234개(안착 34, hold 200, 경계 1)이고 중간 50 ms 속도도 적분에 사용한다.
  phase별 sample 통계는 원래 native 표본을 사용하며 parity마다 새로운 표본 수로 합산하지 않는다.
- 충돌 또는 tick ≥70에서 평면 속도 >0.1 m/s / 요청 spawn에서 XY 거리 >0.5 m면 중단한다.
  위치·속도 주입, physics 끄기, 데이터 잘라내기, label 수정, runtime/decoder gate 완화는 없다.

## 실패·대기 기록도 구분해서 보존

| 기록 | 실제 관측 | 원인·처리 |
| --- | --- | --- |
| [v1 초기화 실패](provenance/stationary_motion_abba_v1_execution_status.json) | 차량 생성 전 실패; 상태·ACK·이미지 0; 나머지 3회 미실행 | 날씨 객체의 preset을 재귀 직렬화한 코드 오류. 날씨 14개 수치 필드만 기록하도록 수정 후 새 v2로 선언 |
| [v2 기준점 실패](provenance/stationary_motion_abba_v2_execution_status.json) | 안착 70개·카메라 35묶음·JPEG 210장; hold 없음; 나머지 3회 미실행 | 첫 tick 전 API pose가 0으로 반환되어 196 m 이동으로 오판. v3에서는 이미 계산한 정확한 요청 spawn을 기준점으로 사용 |
| [B1 실행 전 포트 거부](provenance/stationary_motion_abba_v3_b1_preflight_note.json) | 서버·차량·출력 폴더 생성 전 거부 | 2101 해제 확인 후 같은 B1을 시작. 당시 일시 점유의 정확한 원인은 미확정 |
| [B2 종료 후 A2 대기](provenance/stationary_motion_abba_v3_a2_port_cooldown_note.json) | B2 정상 완료; 다음 서버 미시작 | 2101의 서버 측 TIME-WAIT와 bind 거부를 직접 관측. 자연 해제 후 동일 A2 실행 |

v1은 world 설정 복원은 확인했지만 **날씨 복원 검증은 같은 직렬화 오류로 실패**했다.
v2는 설정·날씨·소유 actor 정리와 종료가 확인됐다. v2와 v3의 첫 70개 물리 관측값은 일치하며,
기준점 계산만 바뀌었다. 실패 원본은 PASS로 다시 쓰지 않았다. 추가 tick·삭제·기준 완화 없이
별도 계획과 소스 해시로 수정 이력을 남겼다. v3 정상 4회 외에 실패 2회를 성공 수에 더하지 않는다.
다른 작업을 종료하거나 시스템·Conda·패키지 설치·재부팅·GPU 1 변경은 하지 않았다.

## 파일 위치와 재검증

저장소 루트 기준 원본은 `artifacts/training/2026-09-14/stationary_motion_abba_v{1,2,3}`에 있다.
각 회차에 owner 계획/결과, server/collector 로그, lifecycle 검사, 실행 소스 archive와
`motion_probe` 또는 `motion_probe.partial`의 상태·ACK·카메라·이미지가 있다. 대용량 원본은
Git clone만으로 생기지 않는다. 이 공개 폴더에는 분석 결과·PNG·계획/실패 기록을 게시한다.
[v1 계획](provenance/stationary_motion_abba_v1_plan.json)과
[v2 계획](provenance/stationary_motion_abba_v2_plan.json)도 변경 없이 복사했다.

실행 소스는 당시 HEAD `3cb676ac`에 더해 검토한 **미커밋 worker/wrapper**를 포함한다.
따라서 HEAD만 체크아웃해서 동일 소스라고 판단하면 안 된다. 계획의 15개 source hash와
각 회차의 provenance archive가 기준이다. 이번 턴에 commit/push는 하지 않았다.

기존 로컬 의존성과 원본을 가진 상태에서, 저장소 루트에서 아래 명령으로 분석만 재실행한다.
출력은 반드시 기존에 없는 폴더여야 한다. 명령은 CARLA나 학습을 시작하지 않는다.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 python3 \
  scripts/e2e/summarize_carla_stationary_motion.py \
  --campaign-root artifacts/training/2026-09-14/stationary_motion_abba_v3 \
  --output-dir artifacts/analysis/stationary_motion_v3_recheck_01
```

관련 코드 테스트는 **303 passed / 37.37초**였다. 실제 실행 4회와 코드 테스트 303개는 별도다.
최종 12:40 KST 검사에서 15개 실행 소스, CARLA 실행 파일·맵·기본 Engine 설정 해시가
계획과 같았고 포트 2100–2102는 모두 bind 가능했다. 기존 9월 9일 seal과 오늘 01번 데이터
감사 자료의 해시도 유지했다. [SHA256SUMS](SHA256SUMS)로 이 하위 게시 자료를 검증할 수 있다.

## 다음에 진행할 순서

1. **움직이다 정지하는 전환을 별도 계측:** 기존 실패 구간의 무가속 coast와 제동 전환을
   Actor 3D 상태·wheel angle·ACK와 함께 고정 조건으로 관측한다. 동일 정지 실험을 계속
   반복하거나 handbrake를 해결책으로 채택하지 않는다. 이번 측정의 결과를 moving-coast에 대입하지 않는다.
2. **데이터 품질·범위 확보:** 원본 운동/정답 계약과 카메라 품질을 통과한 독립 직진·좌/우회전·
   자연 감속·정지 유지 episode를 TRAIN/VAL에 분리한다. 현재 VAL hold 0개와 차로·깊이·신호
   정답 부족은 이 정지 진단으로 채워진 것이 아니다. 기존 TEST는 계속 잠근다.
3. **PRO6000 GPU 0 학습:** 데이터 버전·split을 고정하고 소수 TRAIN 학습 점검 → 같은 조건
   3-seed A/B → 경로/속도/정지 subgroup 평가. 개인 hwanhong venv만 사용한다.
4. **로컬 검증:** export 동등성 → Autoware 10 Hz shadow → fallback → 제한된 CARLA
   learned closed-loop. 이후 ACC·회피·차선변경으로 확장한다.

**현재 판정:** 정상 정지 상태의 지속 미세 움직임은 이번 조건에서 관찰되지 않았다.
초기 안착 오차의 존재는 확인했지만 수치 노이즈의 보편적 상한, COM 위치, 주행 정답의
곡률/운동 적합성, 감속 문제 해결 또는 실차 안전성을 입증하지 않는다.
전체 learned closed-loop 기능 승인 수는 여전히 **0/30**이다.

[9월 14일 전체 목차](../README.md) · [현재 목표·학습 계획](../../../../../portable-e2e-status-2026-09-14.md)
