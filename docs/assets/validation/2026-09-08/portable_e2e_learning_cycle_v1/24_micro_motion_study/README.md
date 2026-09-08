# 미세 이동: 위치·속도 적분과 시간 간격 민감도

<!-- HH_260906 - Separate raw-motion diagnostics from label replacement, model training and vehicle-control qualification. -->

기존 C-track 8회 원본에 대한 CPU 전용 분석입니다. 새 주행 촬영·모델 학습·라벨 보정은 하지 않습니다.
이 분석은 앞선 곡률 실패를 이미 알고 설계했으므로, 새 독립 검증 데이터나 눈가림 실험이 아닙니다.

## 분석 질문

1. 같은 기준점에서 실제 위치 변화와 기록 속도를 적분한 이동량은 얼마나 다른가?
2. 한 미래 지점을 기준으로 뒤쪽 0.1·0.2·0.5초를 보면 변위 방향이 얼마나 달라지는가?
3. 차이가 준비·출발·주행·정차와 미속/정상 이동 중 어디에 집중되는가?

[실행 전 선언한 계획](plan.json)은 원본 파일 SHA와 계산 범위를 고정한 역사적 기록입니다.
계획의 `DECLARED_NOT_EXECUTED`는 선언 당시 상태이며 최종 실행 상태와 혼동하지 않습니다.

## 실제 실행 결과

<!-- HH_260906 - Report complete overlapping denominators and measured magnitudes without promoting raw data or attributing unmeasured causes. -->

**2026-09-08 15:28:29–15:28:47 KST에 완료**했습니다. 실행 종료 코드는 0이며,
[실행 상태](execution_status.json)의 `COMPLETE`와 분석 판정 `MEASURED_NOT_ADMITTED`는
각각 계산 완료와 학습 미승인을 뜻합니다. [실행 전 소스 연결 기록](execution_binding.json)에
커밋·소스 SHA·실제 명령을 남겼습니다. 원본 5개 파일과 실행 소스의 전후 해시가 일치합니다.

| 전체 분석 범위 | 실제 수 |
|---|---:|
| 원본 20 Hz 차량 상태 | 15,466 |
| 50 ms 구간 | 15,458 |
| 100 ms 구간: 시작 오프셋 0 / 1 | 7,728 / 7,722 |
| 전체 카메라 앵커 / 완전한 6.4초 미래 창 | 7,736 / 7,216 |
| 미래 지점: 모든 창의 64개 인덱스 | 461,824 |
| 기존 곡률 초과 창 / 초과 지점-창 조합 | 1,560 / 29,382 |
| 초과 지점 중 첫 인덱스 / 나머지 인덱스 | 380 / 29,002 |

고유한 정확한 target timestamp는 461,824개, 100 ms로 반올림한 tick은 7,720개입니다.
초과 부분은 각각 29,382개·526개입니다. 나노초 단위 격자 차이와 서로 겹치는 미래 창
때문에 정확한 timestamp가 달라도 독립적인 주행 사건은 아닙니다.

### 위치와 속도 적분의 차이

8회 각각 전체 phase를 포함한 **사다리꼴 적분 XY 잔차 P95**의 범위입니다.
아래는 실행별 P95의 최솟값–최댓값이며, 모든 실행을 합친 하나의 P95가 아닙니다.

| 기준점 | 50 ms | 100 ms 오프셋 0 | 100 ms 오프셋 1 |
|---|---:|---:|---:|
| ActorSnapshot API 위치 | 2.904–2.948 mm | 5.030–5.072 mm | 4.973–5.057 mm |
| 기존 평면 변환 후륜 위치 | 2.371–2.425 mm | 4.425–4.531 mm | 4.417–4.474 mm |

이 숫자가 작다고 물리 기준점·라벨의 정확성을 승인하지 않습니다. 준비·미속·주행·꼬리와
속도 구간을 분리한 상세 통계는 JSON에 모두 남겼습니다. `후륜 위치 변화 − API 위치 변화`
와 두 기준점 offset 변화의 대수적 항등식 잔차는 0이지만, API가 무게중심이라는 증거는 아닙니다.

### 시간을 길게 보면 방향 차이가 없어지는가?

같은 끝점에서 200/500 ms 뒤쪽 변위 방향을 원래 100 ms 방향과 비교했습니다.
아래 속도는 원래 100 ms XY 변위의 평균속도이며 차량 속도 라벨과 구별합니다.

| 범위 | 200 ms 대 100 ms: P95 / 최대 | 500 ms 대 100 ms: P95 / 최대 |
|---|---:|---:|
| 0.1 m/s 미만, 평가 가능한 전체 | 0.252728° / 147.350142° | 0.220325° / 153.556752° |
| 1 m/s 이상, 평가 가능한 전체 | P95 0.777630° | P95 3.316519° |

미속 비교의 대부분은 작지만 일부 극단값은 큽니다. 반대로 회전 중 긴 구간의 방향 차이는
정상 곡선의 시간 범위 차이도 포함하므로, 500 ms가 항상 더 정확하거나 문제가 해결됐다는
의미가 아닙니다. **긴 구간의 곡률을 새 PASS 기준으로 계산한 실험도 아닙니다.**

| 뒤쪽 시간 범위 | 방향 평가 가능 | 창 내부 앞부분 부족 | 변위가 너무 작아 미평가 |
|---|---:|---:|---:|
| 100 ms | 435,210 | 0 | 26,614 |
| 200 ms | 428,932 | 7,216 | 25,676 |
| 500 ms | 409,954 | 28,864 | 23,006 |

각 행의 합계는 461,824입니다. 두 방향의 차이를 계산하려면 양쪽 모두 평가 가능해야 하므로,
그래프의 비교 분모는 위 단일 방향 평가 수보다 작을 수 있습니다. 미평가는 0 오차나 성공이 아닙니다.

## 실제 수치 그래프와 원본 연결

<!-- HH_260906 - Link numerical plots, preserve the initial display-only publication, and distinguish reproduction from a new driving run. -->

PNG 3장은 원본 계측 수치를 그린 것으로 새 CARLA/Autoware 화면이나 모델 주행 영상이 아닙니다.
기존 차량 중심 경로·6카메라·GIF는 [21 실제 주행 자료](../21_turn_launch_matrix/README.md)에 있습니다.

![모든 8회 위치 변화와 속도 적분 잔차](analysis_readable_v2/01_position_velocity_residuals.png)

![모든 미래 지점의 시간 간격별 방향 차이](analysis_readable_v2/02_direction_interval_sensitivity.png)

두 방향 비교 패널은 같은 축을 사용하며 P95와 분모를 표시합니다. 최대 방향 차이는 위 표에
별도로 남겼으므로 이 P95 막대를 모든 값의 최댓값으로 읽지 않습니다.

![페달 0.13 두 반복의 준비·출발·정지 잔차](analysis_readable_v2/03_both_013_repetitions_trace.png)

마지막 그래프 왼쪽은 첫 관측부터 8초로 warmup도 포함합니다. 오른쪽은 마지막 주행 8초와
전체 정지 후 꼬리입니다. 같은 조건의 두 반복이지 독립적인 도로·환경 검증은 아닙니다.

- [전체 그룹별 분석 JSON](analysis_readable_v2/summary.json): 원본 sidecar의 정확한 복사본.
- [그래프 수치와 분모](analysis_readable_v2/plot_metrics.json): 모든 행을 합쳐 계산한 통계, 그룹 P95의 평균이 아님.
- [소스·입력·기존 패키지 버전](analysis_readable_v2/provenance.json), [출력 SHA256](analysis_readable_v2/SHA256SUMS).

최초 그래프는 방향 축이 0–215°여서 작은 P95를 읽기 어려웠습니다. **값을 바꾸지 않고 축과
수치 표기만 개선한 v2**를 공개합니다. 첫 출력 7개 파일은 private
`artifacts/training/2026-09-08/micro_motion_study_v1/publication_initial_axis_v1`에 원형 보존합니다.
새 주행·원본 분석 재실행·새 데이터 버전으로 세지 않습니다.

### 결과 확인과 재실행

저장소를 받은 뒤 이 README와 PNG·JSON을 열면 결과를 확인할 수 있습니다. PNG가 Git LFS
포인터로만 보이면 저장소의 [초보자 가이드](../../../../../BEGINNER_QUICKSTART_KO.md)를 먼저 확인합니다.
대용량 private 원본 JSONL은 Git에 넣지 않았으므로 **git clone만으로 원본 분석을 재실행할 수는 없습니다.**
재실행에는 계획에 고정한 5개 원본 파일과 동일 SHA가 필요합니다. 분석은 CPU 전용 표준
Python이고, 그래프는 이미 있는 matplotlib/numpy/Pillow를 사용했습니다. 새 설치는 하지 않았습니다.

저장소 루트에서 공개본 무결성 확인:

```bash
(cd docs/assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/24_micro_motion_study && sha256sum -c SHA256SUMS)
```

원본을 보유한 환경에서만, 기존 출력을 덮어쓰지 않는 새 폴더로 계산:

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' python3 -m scripts.e2e.analyze_carla_micro_motion \
  --input-root artifacts/training/2026-09-08/turn_launch_raw_geometry_v1 \
  --output-dir artifacts/training/2026-09-08/micro_motion_reproduction_01
```

같은 출력 폴더가 이미 있거나 입력 SHA·파일 구성이 다르면 거절합니다. 기존 결과를 삭제해서
억지로 실행하지 않습니다. 분석 소스 커밋은 `927531e`, 최종 표시 소스 커밋은 `78fb168`입니다.

## 측정값 해석

위치 잔차는 `실제 위치 변화 − 기록 속도의 시간 적분`입니다. 20 Hz 속도의 왼쪽 값·오른쪽 값·사다리꼴 적분을 각각 남깁니다.
50 ms 전체 간격과 100 ms 간격의 두 시작 오프셋을 포함하고, 준비/주행/꼬리 구간의 경계도 제외하지 않습니다.
잔차를 시간으로 나눈 값은 평균 속도 차이이며 순간 속도 오차와 동일하지 않습니다.

ActorSnapshot API 위치와 원래 변환한 후륜 위치는 별도로 비교합니다. API 위치가 물리 무게중심이라는 가정은 하지 않습니다.
후륜 속도는 원래 평면 변환의 `vx/vy`를 기록 yaw로 회전한 값으로, pitch/roll까지 포함한 완전한 강체 운동의 정답이 아닙니다.
수직 낙하/초기 접지와 평면 XY 이동도 구분합니다. 두 기준점의 위치 차분 관계가 수학적으로 일치해도 물리 기준점 승인은 아닙니다.

미래 64점은 첫 점부터 끝까지 모두 포함합니다. 0.2·0.5초에 필요한 앞쪽 점이 없는 경우에는 미평가로 남기며,
앵커 이전 자료를 몰래 보충하거나 같은 점을 복사하지 않습니다. 변위가 기존 0.0001 m 이하인 방향도 미평가이며 PASS가 아닙니다.
긴 구간에서 방향이 안정되더라도 기존 0.1초 곡률 실패·라벨·안전 기준을 바꾸지 않습니다.
겹치는 창의 점 수, 정확한 target timestamp 수, 반올림한 100 ms tick 수는 서로 구분하며 독립적인 물리 사건 수로 해석하지 않습니다.

## 다음 판단의 경계

이 분석만으로 센서 노이즈, 물리 엔진 오류 또는 실제 제어 지연을 확정하지 않습니다.
원본을 정지로 치환하거나 오차 허용폭을 기존 실패에 맞춰 정하지 않습니다.
별도의 궤적/불확실성 표현을 개발한다면 독립적인 보정 자료와 새 버전이 필요합니다.
8회 원본의 학습 미승인 표시와 기존 모델·차량 TF·물리/런타임 게이트는 유지합니다.

## 코드와 결과 검증

<!-- HH_260906 - Keep code-regression success separate from raw-label admission and preserve the earlier rendering version. -->

표시 보완을 포함한 소스 커밋 `78fb168`에서 최종 검사했습니다.

| 검사 | 실제 결과 | 시간 |
|---|---|---:|
| 일반 테스트 | 4,204 passed / 6 skipped | 204.46초 |
| Autoware 런치 패키지 테스트 | 373 passed / 0 skipped | 13.48초 |
| 합계 | **4,577 passed / 6 skipped** | 두 실행 시간은 각각 기록 |

[검사 명령·판정·독립 검토 범위](verification.json), [일반 원본 로그](tests/main.log),
[런치 원본 로그](tests/launch.log), [확인한 소스 477개 SHA](tests/source_sha256.json)를 제공합니다.
5개 skip은 기존 로컬 PyTorch의 안전한 `torch.load(weights_only=...)` 기능 부재,
1개는 고정된 Woraksan 지도 bundle 부재 때문입니다. 패키지 설치나 안전 검사 우회는 하지 않았습니다.

이전 표시 소스 `3c91cad`에서도 일반 4,204 passed / 6 skipped였으며 그 로그는 private
`artifacts/training/2026-09-08/micro_motion_study_v1/regression_before_display_fix.log`에 보존합니다.
두 번 검사한 같은 테스트를 서로 다른 통과 항목으로 더하지 않습니다. 집중 59개 테스트도
위 전체 검사에 포함되므로 합계에 다시 더하지 않습니다.

독립 검토는 원본에서 native 대표 72구간(phase 경계 48개 포함), 미래 11앵커의
704지점·2,112개 시간 범위 요청을 별도로 재계산했습니다. 전체 공개 통계 158개 그룹도
행 단위로 다시 집계하여 정확히 일치했습니다. 모든 미래점의 전체 분모 검증과 대표점의
수식 재계산 범위를 구별합니다. PNG 3장도 실제 열어 확인했습니다.

코드 테스트가 통과했다는 뜻이며 새 학습 모델의 주행·실차 안전·데이터 승인이 아닙니다.
