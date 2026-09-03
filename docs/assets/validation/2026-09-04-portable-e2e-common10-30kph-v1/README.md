# CARLA Common10 30 km/h 데이터 증거 — 2026-09-04

이 묶음은 **CARLA `BasicAgent` expert가 생성한 Town07 직진 주행을 Common10
학습 계약으로 변환한 데이터와 중앙 고정 검토 화면**이다. 현재 PNG/GIF는
Autoware VAD가 주행하는 RViz 화면도, 학습된 모델의 추론·폐루프 성능 증거도
아니다. 서로 다른 실행 계층을 혼동하지 않도록 과거 Autoware VAD 화면은 아래
별도 절에서만 연결한다.

## 진행 상태

| 범위 | Common10 정식 데이터 상태 | 설명 |
|---|---:|---|
| Town07 / straight / 30 km/h | **PASS** | 6-camera 10 Hz 계약과 planning 검증 통과 |
| `c_track_1_0_7` / turn / 30 km/h | **PENDING** | 충분한 길이의 회전 경로로 새로 수집·검증해야 함 |
| Town03 / turn / 30 km/h | **PENDING** | 충분한 길이의 회전 경로로 새로 수집·검증해야 함 |
| 지원하는 모든 Town / straight + turn | **PENDING** | Town별 정식 Common10 행렬은 아직 완성되지 않음 |

따라서 이 묶음의 `PASS`는 **완료된 Town07 직진 에피소드 범위**에만 적용된다.
전체 Town 캠페인의 완료 판정은 아니다.

## 화면 자료

- [차량 중앙 고정 전체 화면 PNG](town07_straight/town07_straight_30kph_common10_centered_overview.png)
  — 1600×900, 6개 카메라와 heading-up 경로·이력·future trajectory 및 전체
  경로 inset을 한 화면에 표시한다.
- [차량 중앙 고정 주행 GIF](town07_straight/town07_straight_30kph_common10_centered_drive.gif)
  — 1600×900, 60 frame, 리뷰용 5 fps, 12초이다.

GIF의 5 fps는 문서 열람 용량을 줄이기 위한 재생 속도다. 센서 원본 및 준비된
학습 샘플의 검증된 cadence는 약 10 Hz이므로 두 값을 같은 의미로 해석하면 안
된다.

## 검증 결과

| 항목 | 결과 |
|---|---:|
| 데이터셋 | `carla-common10-30kph-20260904-v1` |
| 계약 | `common_10hz_v1` |
| 소스 | `carla_basicagent_expert` / `carla-0.9.15-native-expert-v1` |
| 맵·경로·날씨 | `Town07` / `town07_straight_s0000_p00` / `ClearNoon` |
| 목표 속도 | 30 km/h |
| 학습 split / 샘플 | `train` / 309 |
| 실효 sample rate | 9.999999850974028 Hz |
| median / p99 / maximum period | 100.000001 / 100.000002 / 100.000002 ms |
| 준비된 주행 구간 | 30.800000459 s / 208.4440665985935 m |
| 카메라 bundle / future 64-point coverage | 100% / 100% |
| 최대 bundle skew / state delta / command age | 0 / 0 / 0 ms |
| 고유 JPEG scan 최소 비율 | 100% |
| 충돌 / 차선 침범 | 0 / 0 |
| catalog goal | 2.3683169969315316 m 잔여, 2.5 m tolerance로 도달 |

원본 수집은 20 Hz physics, 10 Hz camera, 3.5초 정지 warm-up, BasicAgent 주행,
6.5초 정지 tail 순서로 총 747 state와 카메라별 374 frame을 기록했다. 미래
trajectory 64점을 끝까지 확보할 수 있는 anchor만 선택해 309개 bundle을
준비했고, tail의 65개 anchor는 label context로만 사용했다.

planning 검증은 schema, Common10 cadence, JPEG SHA-256, native frame identity,
source-manifest binding, 카메라 timestamp, causal route reconstruction, offline
1 ms bundle readiness를 통과했다. 상세한 기계 판독 값은
[`validation_summary.json`](validation_summary.json)에 있다.

## 학습 파이프라인 smoke 결과

검증된 데이터는 개인 프로젝트 venv에서 CPU 1-step과 physical GPU 0만 UUID로
노출한 CUDA 1-step을 각각 통과했다. 이어서 별도 새 run으로 32개 고정 sample,
batch 4, 5-step smoke를 실행해 20 sample 처리와 checkpoint 저장까지 확인했다.
GPU 실행 종료 뒤 physical GPU 0은 0 MiB, 0%이며 compute process가 남지 않았다.

이는 decode, forward, loss, backward, optimizer, checkpoint 연결에 대한
**pipeline smoke PASS**다. 이 데이터에는 별도 `val` split이 없으므로 평가를
억지로 수행하지 않았고, step별 loss를 성능 개선으로 해석하지 않는다. 세 run의
설정·최종 상태·산출물 SHA-256은
[`reports/training_smoke_summary.json`](reports/training_smoke_summary.json)에 있다.

## 판정 한계

- raw source artifact byte-level 내용 검토는 `NOT_RUN_REQUIRES_RAW_SOURCE_REVIEW`다.
- image pixel decode 검사는 `NOT_RUN_REQUIRES_APPROVED_IMAGE_LIBRARY`다.
- 모델 추론 및 폐루프 runtime 실행은 수행하지 않았다.
- 별도 `val` split 평가도 수행하지 않았다.
- trajectory geometry를 raw source state에서 독립 재계산한 검증은 수행하지 않았다.
- 수동 release review가 여전히 필요하다.
- 이 결과만으로 실제 차량 적용 가능성이나 Autoware VAD 성능을 주장할 수 없다.

## 별도의 Autoware VAD 30 km/h 실행 화면

아래 자료는 2026-09-02 Autoware VAD runtime/control 캠페인의 Town07 직진
selected-baseline regression이다. **이번 CARLA Common10 데이터 수집과는 다른
실행**이며, 현재 자료를 Autoware 화면으로 오인하지 않도록 분리해 둔다.

- [Autoware 차량 중앙 전체 화면](../2026-09-02-runtime-control-campaign-v1/30kph/town07_straight/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png)
- [Autoware 주행 GIF](../2026-09-02-runtime-control-campaign-v1/30kph/town07_straight/C_selected_baseline_regression/02_autoware_drive.gif)
- [Autoware runtime/control 캠페인 설명](../2026-09-02-runtime-control-campaign-v1/README.md)

수집부터 검증·학습 smoke test까지의 재현 절차는
[`docs/portable-e2e-training.md`](../../../portable-e2e-training.md)를 따른다.

## 무결성 식별자

- dataset manifest SHA-256: `f399423c40939e6799e1a66ad3b618f4165b70d0afc38d2e3de4f65179858c4c`
- dataset fingerprint SHA-256: `eb1900c327a4e33dae2f49391d27069861591fcdac2327896fc41d37f4863605`
- prepared tree manifest SHA-256: `990bade233bc037bc6892cdf9d51e877b70c4669e11424ee59eb5cbab7f7c016`
- Common10 contract SHA-256: `eb8f44c98a5bee6e560f4983c6ecbc67122c4fe043d4d50fe2ea3737bda9c722`
- 데이터 생성 코드 commit: `0eb9b1bf50d29e362020ac33265e10ff6aea6c23`
