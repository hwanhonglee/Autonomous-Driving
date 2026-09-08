# 정지 출력 구조 연구 — 재가속 없는 10 Hz 후보 궤적

<!-- HH_260906 - Develop a separate stop representation without implying learned intent, road following or deployment qualification. -->

## 무엇을 개선하는가?

[17 디코더 표현 가능성 시험](../17_decoder_representability/README.md)에서 최종 출력 실패
34개는 정지 직전의 짧은 경로에서 속도가 다시 증가해 기존 검증기에 거절된 경우였습니다.
기존 검증기는 실제로 단조 감속하며 0으로 끝나는 짧은 경로를 이미 허용합니다.
따라서 검증 기준을 바꾸지 않고 **정지하기로 한 뒤의 속도·궤적 출력 구조**를 별도로 연구합니다.

이 코드는 신경망 전체나 새 checkpoint가 아닙니다. 정지를 선택하는 모델도 아직 없습니다.
지속 주행·신호 판단·앞차 추종·장애물 회피·차선변경을 수행하지 않으며, 곡선 출력이 가능하다는
것과 실제 도로를 따라간다는 것은 다릅니다. 현재 모델·학습 CLI·배포 허용 목록에는 연결하지 않았습니다.

원본 8회의 저속 곡률 실패를 다룬 [24 분석](../24_micro_motion_study/README.md)과도 별개입니다.
원본 라벨·phase·실패 표시를 수정하거나 데이터를 새 학습용으로 승인하지 않습니다.

## 실제 실행 결과

<!-- HH_260906 - Report the full synthetic grid and finite-precision differences without interpreting geometry checks as learned driving performance. -->

**2026-09-08 18:27:17–18:27:18 KST**에 기존 로컬 CPU 환경에서 완료했습니다.
소스와 [고정 계획](../../../../../../config/portable_e2e_stop_primitive_probe_20260908.json)을
커밋 `f409ab2`에 넣은 후 전체 시험을 실행했습니다. [실행 연결 기록](execution_binding.json)에
선언·실행 시각과 실제 명령을 남겼습니다. 계획 파일의 `DECLARED_NOT_EXECUTED`는 선언 당시
상태이며, 실제 결과의 `COMPLETE_NOT_ADMITTED`와 구별합니다.

| 검사 | float32 | float64 | 전체 |
|---|---:|---:|---:|
| 고정 조건 | 50 | 50 | 100 |
| 후보 궤적 | 300 | 300 | 600 |
| 기존 runtime gate 통과 | 300 | 300 | **600 / 600** |
| 독립 수식 대조 일치 | 300 | 300 | **600 / 600** |
| 수치 불변 조건 검사 | 300 | 300 | **600 / 600** |
| 마지막 속도가 정확히 0 | 300 | 300 | **600 / 600** |

미래 지점 38,400개, 수식 대조 scalar 154,200개를 처리했습니다. 누락·중단된 행은 0개이며
실행 전후 계획·소스 해시가 일치했습니다. 최고 속도 변화율은 두 자료형 모두 0이어서
시험 후보에서 재가속이 발생하지 않았습니다.

| 수치 차이 | float32 | float64 |
|---|---:|---:|
| 최소 속도 변화율 | −2.900004387 m/s² | −2.899999993 m/s² |
| 독립 수식과 최대 XY 성분 차이 | 0.000001601 m | 0.000000000000001776 m |
| 독립 수식과 최대 속도 차이 | 0.0000007592 m/s | 0 |
| 연속 제동거리와 이산 경로 길이의 최대 차이 | 0.4166675 m | 0.4166667 m |

float32의 약 0.000004387 m/s² 차이는 그대로 기록했습니다. 수식 대조는 자료형 epsilon에
따른 허용량을 사용하지만, 기존 runtime gate의 감속 한계나 오차 허용량은 바꾸지 않았습니다.
연속 제동거리와의 약 0.417 m 차이는 작은 수식 재계산 오차와 **다른 이산화 문제**이며,
정확한 목표 정지·실차 제동거리를 검증하지 못한 이유 중 하나입니다.

여기서 10 Hz는 예측점 사이의 100 ms 시간 격자입니다. 카메라·모델의 실제 추론 처리량이나
CARLA 화면 FPS를 측정한 시험이 아닙니다. 실행 시간도 신경망 추론 성능으로 환산하지 않습니다.

## 실제 수치로 그린 그림과 전체 결과

<!-- HH_260906 - Publish every synthetic candidate and describe fixed plot selections without presenting them as captured driving scenes. -->

아래는 **합성 계산 결과 그래프**이며 CARLA·Autoware 화면 촬영이 아닙니다. 속도 그림은
두 자료형의 초기 속도 0.4·4 m/s에서 정지 시간 입력 5개를 모두 보여줍니다. XY 그림은
초기 속도 4 m/s·시간 입력 0에서 후보 6개를 모두 표시합니다. 검은 사각형이 현재 차량
기준점이며, 두 XY 축은 동일 비율이고 기준점이 중앙입니다. 실제 카메라 영상·GIF는 이번에
새로 생성하지 않았습니다.

![초기 속도와 정지 시간별 합성 감속 곡선](results_readable_v2/synthetic_stop_speed.png)

![차량 기준점 중심의 직진·곡선 정지 후보](results_readable_v2/synthetic_stop_xy.png)

그림은 고정한 일부 입력 조건을 보여주지만, [전체 100개 조건·600개 후보 원본](results_readable_v2/rows.jsonl)에는
모든 XY·속도·방향·개별 판정이 있습니다. [실행 요약](results_readable_v2/summary.json),
[출처·전체 후보 재집계·그림 선택 조건](results_readable_v2/provenance.json),
[게시 파일 SHA](results_readable_v2/SHA256SUMS)도 함께 보존합니다.

첫 게시본에서 XY 축 제목과 하단 설명이 겹쳐, 그림 여백만 수정한 `results_readable_v2`를
게시했습니다. 전체 행·요약·속도 PNG는 첫 게시본과 바이트 단위로 동일합니다. 첫 게시본은
개인 보관 `artifacts/training/2026-09-08/stop_primitive_publication_initial_layout_v1`에 이동해
보존했으며 원본 시험 결과는 재실행하거나 덮어쓰지 않았습니다. 실제 그림에서 글자 영역이
겹치지 않는지 검사하는 회귀 테스트도 추가했습니다.

## 구현과 입력

<!-- HH_260906 - Keep the research identifier and tensor ABI outside all existing trained-model and runtime-bundle allowlists. -->

코드: [stop_primitive_research.py](../../../../../../portable_e2e/stop_primitive_research.py).
연구 ID: `portable_e2e.stop_primitive.research.v1`.

| 입력 | 의미 | 모양 |
|---|---|---|
| `current_speed_mps` | 현재 전진 속도, 0–30 km/h | `[batch]` |
| `duration_logits` | 정지할 때까지의 시간을 정하는 연구 변수 | `[batch, 6]` |
| `curvature_logits` | 각 구간에서 좌우로 휘는 정도를 정하는 연구 변수 | `[batch, 6, 64]` |

출력은 6개 후보의 XY·속도·방향과 정지 시간입니다. 현재는 연구자가 고정한 입력을 사용하며
카메라로부터 이 변수를 예측하는 학습 모델은 연결하지 않았습니다. 미래 정답·정지 phase·
checkpoint를 받는 입력도 없습니다. 음수 속도, 30 km/h 초과, NaN/Inf, 잘못된 차원·자료형은
조용히 잘라 맞추지 않고 거절합니다. float32와 float64를 구분합니다.

## 출력이 만들어지는 방식

<!-- HH_260906 - State the discrete output convention and its limits instead of claiming exact continuous braking distance. -->

현재 속도를 `v0`, 간격을 `dt=0.1초`, 감속 한계를 `a=2.9 m/s²`라고 할 때:

```text
최소 정지 시간 Tmin = max(dt, v0 / a)
정지 시간 T = Tmin + sigmoid(duration_logit) × (6.4초 − Tmin)
예측 속도 v(t) = max(0, v0 × (1 − t/T))
t ≥ T이면 속도는 정확히 0
```

속도는 단조롭게 감소하고 예측 끝은 0입니다. 이 수학적 성질과 실제 float32/64 계산 및 기존
출력 검증 결과는 따로 확인합니다. 감속 2.9·곡률 0.2·횡가속 2.8은 기존 물리 디코더의
제한을 사용하며, 새 실험 결과를 보고 안전 기준을 완화하지 않습니다.

XY 이동량은 기존 출력 계약과 같은 `구간 끝 속도 × 0.1초`입니다. 연속 시간에서의
정확한 제동거리 `v0 × T / 2`와 같지는 않습니다. 끝 속도로 적분하는 이산화 차이는
남아 있으므로, 이 결과를 목표 위치에 정확히 정지한다는 증거로 사용하지 않습니다.
곡률은 구간 진입 속도로 제한하고, 이동거리가 0이면 방향도 바뀌지 않습니다.
비상 제동·승차감 jerk·실제 차량 제어 지연은 검증 범위에 없습니다.

## 시험 범위와 해석

<!-- HH_260906 - Identify synthetic grid evidence distinctly from captured data, training runs and independent environment validation. -->

고정한 합성 속도 10개 × 정지 시간 변수 5개 × 자료형 2개, 총 100개 조건입니다.
각 조건의 6개 곡률 후보를 모두 검사하며 좋은 후보만 선택해 통과 수를 세지 않습니다.
총 600개 후보는 실제 촬영이나 600회 주행이 아닙니다. 선행 실패와 단위 시험을 알고
정한 개발 시험으로, 독립적인 실환경 검증이나 눈가림 실험이 아닙니다.

검사 항목은 최종 속도 0, 재가속 없음, 감속 한계, 속도–XY 일치, 기존 runtime gate,
독립 수식 재계산입니다. 수식 대조와 수치 불변 조건 검사에는 자료형별 epsilon 허용량을
사용합니다. 기존 디코더의 물리 상수나 runtime gate의 기준을 수정한 것은 아닙니다.

이후 학습에 연결하려면 정지 판단에 사용할 **현재 관측·경로 입력**, 정지 의도 정답,
승인된 데이터와 별도 모델 버전을 먼저 정의해야 합니다. 학습 정답으로 미래 정보를
사용하는 것과 실제 추론 입력에 미래 정답을 넣는 것은 구별합니다.
원본 수집의 warmup/tail 표시를 온라인 정지 판단 입력처럼 숨겨 사용하지 않습니다.

## 로컬 CPU 재실행

<!-- HH_260906 - Reproduce only the fixed synthetic study in an existing environment and never overwrite archived evidence. -->

이 저장소 루트의 **기존 Torch·NumPy 환경**에서 아래 명령을 실행합니다. CARLA 서버,
Autoware 실행, 실제 데이터셋, checkpoint 또는 새 설치는 필요하지 않습니다. 새로 clone한
임의의 환경에 Torch가 없으면 그대로 실행되지는 않습니다. 이 안내가 설치 권한을 뜻하지 않습니다.

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  python3 -m scripts.e2e.probe_portable_stop_primitive \
  --plan config/portable_e2e_stop_primitive_probe_20260908.json \
  --output-dir artifacts/training/2026-09-08/stop_primitive_reproduction_01
```

출력 폴더는 아직 없는 이름이어야 합니다. 완료 후 `summary.json`의 전체 수·실패 수와
`SHA256SUMS`를 확인합니다. 파일을 수정해 기존 선언 계획의 소스 해시가 달라졌다면 거절되며,
새 코드 시험에는 별도 계획이 필요합니다. 재실행 시각이 달라지므로 새 summary의 해시는
이번 보관본과 달라질 수 있습니다. 게시 도구의 기대 해시를 이번 값으로 무조건 재사용하지 마세요.

그래프 생성기는 [curate_stop_primitive_probe.py](../../../../../../scripts/e2e/curate_stop_primitive_probe.py)이며
기존 Matplotlib이 필요합니다. 원본 결과의 summary·manifest 해시를 명시적으로 확인한 후
새 출력 폴더에 게시하며, 숫자나 runtime 판정을 다시 계산해 바꾸지 않습니다.

## 최종 코드 회귀 검사

<!-- HH_260906 - Count the final two complete suites once and disclose skipped dependencies and source scope. -->

그림 여백·표시 검사까지 반영한 소스 `15127c4`에서 두 전체 검사를 다시 실행했습니다.
최종 결과는 **4,795 passed / 6 skipped / 0 failed**입니다.

| 범위 | PASS | SKIP | pytest 수행 시간 | 원본 출력 |
|---|---:|---:|---:|---|
| `tests/` 전체 | 4,422 | 6 | 204.70초 | [main.log](tests/main.log) |
| `autoware_e2e_vad_launch/test/` 전체 | 373 | 0 | 13.59초 | [launch.log](tests/launch.log) |

VisionPilot 전용 49개와 게시 도구 전용 53개도 위 일반 테스트에 포함되므로 합계에 다시
더하지 않습니다. [실제 명령·요청/완료 관찰 시각·범위](verification.json)와
[검사 전후 소스 488개 SHA](tests/source_sha256.json)를 남겼고 모두 동일했습니다.
488개는 소스 파일 수이며, 외부 라이브러리·가중치·전체 데이터셋을 해시 검증한 수는 아닙니다.

건너뛴 6개 중 5개는 기존 로컬 Torch에 안전한 `weights_only` 로딩 기능이 없어서,
1개는 고정된 Woraksan 경로·맵 묶음이 없어서입니다. 설치나 검사 우회는 하지 않았습니다.
런치 검사는 mock ROS·임시 가짜 명령과 코드 함수 검사이며 실제 Autoware를 새로 구동한
것이 아닙니다. 작은 합성 텐서 학습 검사가 포함되지만 실제 데이터셋의 새 모델 학습은 아닙니다.

이미 의존성이 준비된 로컬 저장소 루트에서 같은 두 검사를 실행하려면:

```bash
source scripts/e2e/env.sh
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  python3 -m pytest -q -rs -p no:cacheprovider tests
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  python3 -m pytest -q -rs -p no:cacheprovider autoware_e2e_vad_launch/test
```

이는 ROS·메시지 등 기존 로컬 의존성을 사용하는 코드 검사입니다. clone만 한 임의의 PC에서
의존성 없이 실행된다는 안내나 서버에 Autoware를 빌드하라는 명령이 아닙니다.
