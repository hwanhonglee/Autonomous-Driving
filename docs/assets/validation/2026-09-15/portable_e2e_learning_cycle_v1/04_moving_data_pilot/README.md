<!-- HH_260906 - Publish exact recorded moving-pilot visuals and bounded diagnostic facts without granting dataset admission. -->
# Town07 직진 moving-data pilot — 실제 수집 완료, 데이터셋 편입 전

2026-09-15에 새로 수집한 **CARLA BasicAgent expert** 1회 기록이다. 사전에 선언한 이동 구간에서 328개 camera anchor, 32.700000487초를 확보했고, 마지막 anchor 뒤의 실제 6.400000095초도 수집했다. 선택 구간의 기존 9개 필요조건 검사에서는 모두 위반 0이다.

그러나 이 묶음의 판정은 `DIAGNOSED_NOT_ADMITTED`이다. 원본은 `PENDING_QA`이며, 이 공개 작업은 Common10 변환·학습 편입·차량 제어 승인을 하지 않았다. 후속 명시적 qualification/변환의 결과는 별도 증거로 확인해야 한다. **학습 모델의 주행, 현재 Autoware 주행, 정지 기능 통과, 30 km/h 통과가 아니다.**

## 이번에 실제로 한 일

| 항목 | 실제 선언·기록 |
|---|---|
| 사례 | `01_moving_t07_s01`, 1회 시도, 자동 재시도 없음 |
| 맵·경로 | Town07 / 직진 `town07_straight_s0000_p00`, 원본 경로 길이 210.597591 m |
| 차량·환경 | Toyota Prius, wheelbase 2.85 m, ClearNoon, Epic, seed **0** |
| 제어 | CARLA PythonAPI BasicAgent, 목표 15 km/h, 기존 lateral PID Kp=1.95 / Ki=.05 / Kd=.2 |
| 시계·카메라 | 동기 world/native 20 Hz, 저장 카메라 10 Hz, 고정 6-camera rig, JPEG 640×360 / quality 95 |
| 물리 설정 | `physics_substep_profile=inherited`; 실제 readback은 synchronous=true, fixed delta=.05 s, substepping=true, max substep delta=.01 s, max substeps=10. 11개 필드의 before/requested/after는 JSON에 보존하며, 이 설정이 물리 원인을 입증한 것은 아님 |
| 명령 증거 | `acknowledged_batch`, `after_bootstrap`, ACK 1,153/1,153; 실패·정렬 불일치 0 |
| 사전 선택 | 실제 XY 경로 투영 진행거리 [30, 165] m의 모든 저장된 driving anchor. 첫 native 관측 >165 m에서 영구 닫음 |
| 종료 조건 | 선택 camera 시각 범위 ≥30초, 마지막 anchor 뒤 실제 ≥6.4초 확보 후에만 cleanup brake 및 6.5초 tail |
| 실제 종료 | owner exit 0, capture complete, `task_complete=true`, collision 0 / lane invasion 0, cleanup 완료·오류 0 |
| 실제 목표 도달 | `goal_reached=false`, `basic_agent_done=false`. **주행 구간 종료 시** 남은 경로 19.495503 m; goal tolerance 2.5 m |

목표 정지 profile은 `disabled`였다. Warmup과 cleanup tail은 throttle=0 / steer=0 / service brake=1이며 hand brake는 전 구간 false였다. Driving의 실제 readback 범위는 throttle 0–0.75, brake 0–0.198528, steer −0.034210–+0.041789이다. 즉, 전체 주행을 무브레이크·고정 throttle 실험으로 해석하면 안 된다. ACK PASS는 API 수락과 기록된 frame/control의 일관성이지, 물리 actuator 적용 시각의 증명은 아니다.

이 **seed 0 수집 파이프라인 pilot**은 cohort planner의 **seed 101 슬롯 완료가 아니다**. 새로운 TRAIN/VAL/TEST split을 선언하거나 기존 split을 변경하지 않았다. 다른 Town, 회전, 정지·재출발, 실제 도로 데이터 범위를 채운 것도 아니다.

## 원본 분모와 선택 구간

| 원본 phase | native states | camera anchors | 6-camera JPEG | 예정 tick 시간 / 첫–끝 관측 간격 |
|---|---:|---:|---:|---|
| stationary_warmup | 70 | 35 | 210 | 3.50 s / 3.450000051 s |
| driving | 949 | 475 | 2,850 | 47.45 s / 47.400000706 s |
| stationary_tail | 130 | 65 | 390 | 6.50 s / 6.450000096 s |
| 전체 | **1,149** | **575** | **3,450** | **57.45 s / 57.400000855 s** |

3,450개 원본 JPEG를 모두 decode했고, 깨진 이미지·참조 누락·camera/state timestamp 또는 phase 불일치는 0이다. 1,153 ACK와 1,149 state의 분모 차이는 bootstrap/초기화/다음 명령 등을 포함하는 명령 journal과 tick 관측의 차이이며, 누락 state 4개를 뜻하지 않는다. ACK reason은 bootstrap 1, warmup 시작 1, 다음 warmup 70, 최초 driving 1, 다음 driving 949, driving 종료 brake 1, 다음 tail 130이다.

| 경계 | frame | simulation timestamp (s) | 독립 재계산 진행거리 |
|---|---:|---:|---:|
| 첫 선택 camera | 1935 | 16.059247260 | 30.098333415 m |
| 마지막 선택 camera | 2589 | 48.759247747 | 164.750216624 m |
| 선택을 닫은 첫 native 관측 | 2591 | 48.859247748 | 165.161959817 m |
| 마지막 선택 anchor의 실제 future support | 2717 | 55.159247842 | 원본 마지막 driving 관측 |

328개 선택 anchor는 32.700000487초이고, 각각 실제 원본으로 뒷받침되는 64개 미래 point를 갖는다: 총 20,992 point, unavailable point 0. 마지막 support 뒤 **다음 native frame 2718**부터 cleanup brake readback이 시작된다. 첫 저장 tail camera는 frame 2719이므로, 그림은 brake가 시작된 native 관측보다 50 ms 뒤다. 선택을 닫은 뒤 다시 구간 안으로 돌아온 anchor를 추가하거나, 검사 결과를 보고 anchor를 제거하지 않았다.

필요조건 계산에는 기존 방식대로 관측된 20 Hz native 상태 사이의 100 ms 라벨 시각 선형 보간이 사용된다. 없는 미래를 외삽하거나 정지 pose·속도를 주입한 것이 아니다. 그림의 future 선은 실제 기록의 시각화이며, 학습 모델의 예측 선이 아니다.

## 선택 구간 통과와 전체 원본 실패를 함께 공개

전체 575개 camera 중 원본 full-64-point 검사 대상은 **510개**(warmup 35 + driving 475), 나머지 **tail 65개는 context-only**다. 선택 구간 328개는 이 510개 중 사전 공간 규칙으로 정해진 부분집합이다. 선택 밖 247개 camera와 모든 native/tail 원본은 그대로 남아 있다.

| 변경하지 않은 검사 | 선택 anchor 위반 /328 | 전체 full-horizon anchor 위반 /510 | 전체 위반 step 수 | 고유 100 ms 위반 tick 수 |
|---|---:|---:|---:|---:|
| speed_limit | 0 | 0 | 0 | 0 |
| speed_acceleration | 0 | 40 | 79 | 2 |
| speed_deceleration | 0 | 63 | 246 | 4 |
| xy_speed_limit | 0 | 0 | 0 | 0 |
| xy_acceleration | 0 | 38 | 38 | 1 |
| xy_deceleration | 0 | 63 | 305 | 5 |
| xy_curvature | 0 | 97 | 222 | 8 |
| xy_lateral_acceleration | 0 | 0 | 0 | 0 |
| yaw_label_heading_envelope | 0 | 0 | 0 | 0 |

6.4초 미래 창들이 겹치므로 anchor·step 수를 독립 사건 수로 더하면 안 된다. 전체 원본의 launch/cleanup 가감속·곡률 실패를 삭제하거나 성공으로 바꾸지 않았다. 선택 구간의 9개 필요조건 통과도 충분한 주행 안전성, 정지 성능, 데이터셋 admission을 뜻하지 않는다. 관측된 선택 future speed는 약 4.121870–4.124441 m/s(14.839–14.848 km/h)다.

## 실제 장면 — 고정 경계 6장

모든 PNG는 기존 renderer가 만든 **1600×900 원본 파일을 byte-exact 복사**했다. 6개 카메라 전체 FOV를 비율 유지 축소·letterbox로 표시한다. 확대·crop·생성형 보정 없이, 공개 PNG 안의 36개 camera tile을 해당 원본 JPEG의 동일 축소 픽셀과 대조했다. 오른쪽은 ego 중심·차량 heading-up 경로와 관측 궤적이며, 전체 경로 inset도 포함한다.

이미지 제목의 `scalar QA UNASSESSED`는 기존 goal-stop scalar 항목이다. 이번 moving 선택 구간의 별도 9개 필요조건 감사와 다른 필드이므로 그림 문구를 소급 수정하지 않았다. 정확한 moving 판정은 [summary.json](summary.json)을 따른다.

| 이미지 | camera index / frame | 첫 원본 관측부터 경과 | 실제 phase / speed / brake |
|---|---|---|---|
| [01 첫 원본](01_first_raw_camera.png) | 0 / 1699 | 0.0 s | warmup / 0 km/h / 1 |
| [02 첫 선택 anchor](02_first_selected_moving_anchor.png) | 118 / 1935 | 11.800000176 s | driving / 14.8431 km/h / 0 |
| [03 마지막 선택 anchor](03_last_selected_moving_anchor.png) | 445 / 2589 | 44.500000663 s | driving / 14.8470 km/h / 0 |
| [04 cleanup brake 전 마지막 camera](04_last_camera_before_cleanup_braking.png) | 509 / 2717 | 50.900000758 s | driving / 14.8470 km/h / 0 |
| [05 첫 tail camera](05_first_tail_camera.png) | 510 / 2719 | 51.000000760 s | tail / 14.1247 km/h / 1 |
| [06 마지막 원본](06_last_raw_camera.png) | 574 / 2847 | 57.400000855 s | tail / 0 km/h / 1 |

![첫 원본 — warmup](01_first_raw_camera.png)

![첫 선택 moving anchor](02_first_selected_moving_anchor.png)

![마지막 선택 moving anchor](03_last_selected_moving_anchor.png)

![cleanup brake 전 마지막 camera — 실제 future support](04_last_camera_before_cleanup_braking.png)

![첫 tail camera — 실제 brake readback](05_first_tail_camera.png)

![마지막 원본 — cleanup tail이며 목표 정지 성공이 아님](06_last_raw_camera.png)

## 전체 기록 요약 GIF — 실시간 화면이 아님

![실제 전체 기록의 가속 요약](whole_recording_accelerated.gif)

원본 camera index `0, 10, 20, …, 570, 574`의 **59개 실제 장면**을 사용했다. 일반 간격은 simulation 약 1초, 마지막 간격은 약 0.4초다. 표시 10 fps / frame당 100 ms이므로 **57.400000855초 원본을 5.9초로 요약**한다. 마지막 짧은 원본 간격에도 표시 시간 100 ms를 사용하므로 정확히 일정한 배속 영상은 아니다. 실시간 10 Hz Autoware UI나 모델 실행 속도 증거로 해석하지 않는다. GIF의 palette 변환은 기존 renderer 출력 그대로이며, 공개 시 다시 인코딩하지 않았다.

## 증거·해시·보존 범위

- [사전 task plan 원본](task_plan.json): SHA256 `f48132acb3e4da42767384d2e18b2ff04d2f719288c01071ef04ce7c17d23877`.
- [실제 compact summary](summary.json): 원본 결과·phase/control/선택·전체 실패 분모와 source pin을 보존한다. 큰 per-state 배열만 중복하지 않는다.
- [경계 PNG 원본 provenance](boundary_visual_provenance.json), [GIF renderer 원본 provenance](recording_visual_provenance.json): 원본 camera/metadata SHA, renderer SHA, 선택 index를 그대로 복사했다. 후자에는 비공개에 남긴 예전 PNG 3장의 entry도 포함되어 있으며, 이 PNG들을 여기 중복 복사하지 않았다. 그 파일의 `training_data_approved=false`는 시각화 승인 부인이지, 원본 `PENDING_QA` metadata를 수정한 값이 아니다.
- [공개 복사·재검증 provenance](publication_provenance.json): 공개 source/copy SHA, 원본 3,450 JPEG의 전체 inventory fingerprint, 56개 source/metadata pin 재확인, 36개 PNG tile 대조, GIF 59-frame decode 결과.
- [이 폴더 SHA256SUMS](SHA256SUMS): 이 폴더의 모든 다른 파일을 봉인한다. 이전 공개 폴더, 상위 seal, `CURRENT`는 수정하지 않았다.

원본 저장 위치는 repository 상대 경로 `artifacts/training/2026-09-15/moving_cohort_v1/` 아래 `01_moving_t07_s01/`, `01_moving_t07_s01_analysis/`, `01_moving_t07_s01_boundaries/`, `01_moving_t07_s01_visuals/`이다. 원본 capture는 2026-09-15 11:16:07 UTC(20:16:07 KST)에 owner 종료가 기록됐다. 모든 raw JPEG·state·ACK·실패 future JSONL과 기존 provenance를 보존했다. 대용량 원본 JPEG와 future/projection JSONL은 이 공개 폴더에 중복하지 않았다.

원본 diagnostic summary SHA256는 `345bef0f0847465cbc1b809e0ac49ac1d02ff7cdd1b07aa734e028fa5a6ad353`, 원본 episode manifest SHA256는 `cee242e67edd4d2850625af350f8f979ebe3924f8bacae1985e50abbe9706599`, 경로 SHA256는 `804dc69cec5ebbc2edba70946694f312c77328b5cb5d7260b096af0775f34848`이다. 향후 admission 상태가 달라져도 이 원본 진단 시점의 판정과 실패 기록은 소급 변경하지 않는다.
