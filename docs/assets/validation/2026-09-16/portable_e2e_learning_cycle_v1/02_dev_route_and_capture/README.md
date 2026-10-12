# Town03 직진 DEV — 경로 분리·실제 수집·Common10 변환 완료

<!-- HH_260906 - Publish the bounded Town03 DEV pilot with its unchanged raw failures and explicit non-training scope. -->

**이번 목표를 완료했다.** 기존 DEV 경로와 가까웠던 p04 대신 p07을 수집 전에 고정하고,
로컬 CARLA 전문가 주행 1회 → 원본 검사 → Common10 검증용 데이터 변환 → 실제 로더
검사를 마쳤다. **새 모델 학습이나 모델 직접 운전은 하지 않았다.**

| 구분 | 이번에 실제로 한 일 |
| --- | --- |
| 로컬 PC | CPU 경로 분석·검사, 로컬 GPU로 CARLA 1회 수집, DEV 변환, 실제 영상 PNG/GIF 작성 |
| SSH PRO6000 | 접속·설치·전송·학습 모두 하지 않음. 현재 서버 작업 상태도 확인하지 않음 |
| 새 데이터 | Town03 직진 **VAL 328개 / TRAIN 0 / TEST 0**; 기존 학습 데이터에 아직 결합하지 않음 |
| 모델/기능 승인 | 새 학습 0회, 모델 채택 없음, 자체 모델 직접 주행 승인 **0/30 유지** |

## 1. 경로를 어떻게 바꿨나

기존 Town03 우회전 DEV와 후보 직진 p04는 최소 **3.499997 m**까지 가까웠다.
road 19 / section 0에서 서로 반대 방향의 인접 차선이며, 방향 차이는 약 180도다.
연속 선분으로 계산한 5 m 이내 근접 범위는 후보 arc 약 **130.430–183.025 m**다.
선택 구간 30–165 m 중 **34.570 m(25.61%)**, 명목상 미래 지원 구간 중 **18.025 m**가
가까웠다. 앞선 waypoint 53/213이라는 개수와 이번 길이 비율은 다른 측정이다.

8개 후보를 비교하기 전에 다음 규칙을 고정했다: 전체 경로 최소 거리 >5 m,
공유 road/section 없음, 15 km/h에서 30–165 m 선택 구간과 명목 6.4초 미래 이후
경로 여유 ≥2.5 m. 통과 후보 중 최소 거리가 가장 큰 경로를 선택한다.
**5/8개가 이 기하 규칙을 통과했고 p07을 선택했다.**

- 새 경로: `town03_straight_s0000_p07`, 전체 평면 길이 **209.370 m**.
- 기존 Town03 DEV와 전체 경로 최소 거리 **269.636 m**, 공유 road/section 없음.
- 명목 미래 지원 이후 여유 **17.703 m**. 실제 시간 지원은 별도 원본 검사로 확인했다.
- 같은 맵의 검토 대상 경로에 대한 분리다. 통계적 독립성·전체 과거 데이터의 무중복을 보장하지 않는다.
- 비교한 두 경로 모두 DEV다. TRAIN–DEV 유출을 발견하거나 해결했다는 주장이 아니다.

[경로 비교 상세 JSON](01_route_analysis/route_resolution.json) ·
[경로 비교 PNG](01_route_analysis/route_comparison.png)

![수집 전 실제 경로 기하 비교](01_route_analysis/route_comparison.png)

위 그림은 파일의 실제 경로 좌표를 그린 **수집 전 기하 분석**이지 주행 촬영 화면이 아니다.
기존 p04 계획과 Town07 증거를 수정하지 않고 새 DEV v2 실행 계약을 사용했다.

## 2. 실제 수집·변환 결과

2026-09-16 **13:37:11–13:37:44 KST** owner 실행. CARLA 0.9.15 / Town03 / ClearNoon /
Prius / Epic / seed 201 / 목표 15 km/h. 카메라 6개 저장 10 Hz, 상태·physics 20 Hz,
최대 물리 substep 10 ms × 10, 초기 3.5초와 종료 6.5초 기록을 보존했다.

| 항목 | 실측 또는 검사 결과 |
| --- | --- |
| 원본 | 상태 1,151개, 카메라 묶음 576개, JPEG 3,456장; 약 57.5초 |
| 선택 구간 | 사전 고정 arc 30–165 m; 실제 30.276–164.916 m, **32.700000487초** |
| 선택 표본 | frame 1261부터 1915까지 2 frame 간격 **328개** |
| 영상 | 실제 선택 영상 1,968장 독립 복사, 선택 bundle 누락 0, 카메라 간 시각 차이 0 |
| 시간 간격 | 선택 영상 약 10 Hz; 최대 간격 100.000002 ms |
| 속도 | 선택 구간 미래 label 기준 약 **14.82 km/h**; 목표 15 km/h와 실측을 구별 |
| 미래 지원 | 각 anchor 이후 **6.4초/64점**, 총 **20,992점** 모두 실제 원본 상태로 재구성 |
| 제어·이벤트 | ACK 1,155/1,155, 실패 0, collision 0, lane invasion 0 |
| 선택 미래 품질 | 기존 9개 필요조건의 위반 anchor **모두 0**; 기준 완화 없음 |
| Common10 | 기존 planning validator PASS, 실제 VAL loader **328개**, TRAIN/TEST loader는 표본 없음 확인 |
| 정리 | 소유 CARLA 종료 확인, 종료 후 2100–2102 TCP/UDP 3회 관측 모두 사용 가능 |

328개는 서로 시간이 겹치는 한 주행의 표본이다. 독립 주행 328회나 328개 기능 검증으로 세지 않는다.
10 Hz는 시뮬레이션 시각 기준 영상 저장 주기이며 GUI 부드러움·실시간 추론·제어 10 Hz 보장이 아니다.
새 DEV 데이터셋의 빈 TEST split을 검사했을 뿐, 기존 잠긴 TEST 데이터 payload는 열지 않았다.

[원본 진단](02_actual_capture/raw_audit.json) ·
[변환 승인 근거](03_common10_dev/conversion_receipt.json) ·
[최종 변환 검증](03_common10_dev/conversion_result.json) ·
[planning 검사](03_common10_dev/validation_report.json) ·
[최종 경로에서 로더 재검사](03_common10_dev/postrename_verification.json)

## 3. 전체 원본의 문제는 그대로 남아 있다

**선택한 정속 구간의 합격이지 전체 출발·제동 문제 해결이 아니다.**
원본에서 6.4초 미래가 있는 511개 anchor 전체를 검사하면 아래 위반이 남는다.
나머지 종료 65개 영상은 label 지원용 문맥이다.

| 기존 필요조건 | 원본 전체 위반 anchor 수 | 선택 328개 위반 anchor 수 |
| --- | ---: | ---: |
| 보고 속도 가속 | 40 | 0 |
| 보고 속도 감속 | 63 | 0 |
| 위치 기반 가속 | 39 | 0 |
| 위치 기반 감속 | 63 | 0 |
| 위치 기반 곡률 | 67 | 0 |
| yaw/경로 heading envelope | 60 | 0 |
| 보고/위치 속도 상한·횡가속도 | 각각 0 | 각각 0 |

이 수치들은 같은 시점이 겹칠 수 있으므로 더해서 실패 장면 수로 사용하지 않는다.
출발 및 정리용 제동의 실패는 원본·진단·변환 결과에 보존했다. 품질을 보고 유리한 구간을
다시 고른 것이 아니라, 수집 전에 고정한 moving-task 구간에 한해서 변환했다.
전체 576개 중 248개는 문맥으로 남기며 원본 영상을 삭제하지 않았다.
`goal_reached=false`, 원본 `PENDING_QA` 값도 그대로다. 별도 변환 검증서가 한정된 승인을 기록한다.
자연 감속·정지 유지 데이터 부족과 제동 label 문제는 다음 단계에서도 별도로 해결해야 한다.

## 4. 실제 6카메라·차량 중심 경로 화면

**BasicAgent 전문가가 운전했다. Portable E2E, VAD 또는 Autoware가 운전한 화면이 아니다.**
차량 위치는 오른쪽 패널 중앙에 고정하며 경로·주행 이력·실제로 관측한 미래를 표시한다.
미래 선은 모델 예측이 아니다. 왼쪽 6개 카메라의 전체 화각을 유지했고, 어두운 구간도
색/노출 보정이나 생성 이미지 대체 없이 남겼다. RViz/전체 데스크톱 촬영은 이번 범위가 아니다.

[전체 기록 요약 GIF](02_actual_capture/preview/whole_recording_accelerated.gif) ·
[첫 선택 표본 PNG](02_actual_capture/boundaries/02_first_selected_moving_anchor.png) ·
[마지막 선택 표본 PNG](02_actual_capture/boundaries/03_last_selected_moving_anchor.png)

![실제 6카메라와 차량 중심 경로](02_actual_capture/boundaries/03_last_selected_moving_anchor.png)

GIF는 원본 약 57.5초를 59장·10 fps·약 5.9초로 요약한 **가속 미리보기**다.
실제 속도 판단용 실시간 재생이 아니며, 표시가 띄엄띄엄 보이는 것과 원본 영상 누락을 혼동하면 안 된다.
`boundaries/`에 원본 시작·선택 시작/끝·정리 제동 직전·tail 시작/끝의 **PNG 6장**을 모두 제공한다.
화면의 `NOT ADMITTED`는 변환 전 원본 진단 당시 표기이며, 이후 제한된 DEV 변환 결과는
`03_common10_dev/`에 별도로 기록했다. 과거 화면 문구를 사후 PASS로 바꾸지 않았다.

## 5. 코드·증거 보존과 재검증

관련 신규/기존 11개 테스트 모듈 **311 passed / 40.18초**.
기존 Town07 owner 소스 **12개**, 이번 Town03 owner 소스 **14개**가 현재 파일·당시 archive와
각각 동일함을 재확인했다. DEV 변환 receipt는 별도로 **소스 13개·입력 3,534개**를 고정한다.
기존 collector/owner/수학 계약/기존 qualifier는 이번 작업에서 수정하지 않았다.
새 코드와 주석에는 `HH_260906 - ...` 영어 표기를 사용했다.

[게시 자료·소스 검증 기록](verification.json) · [이 폴더 체크섬](SHA256SUMS).
preview의 원본 provenance에는 원본 폴더에 남긴 별도 PNG 기록도 포함되며, 게시본은 중복을
줄이기 위해 GIF와 경계 PNG 6장을 제공한다. 실제 게시 파일 대응은 검증 기록에 명시했다.

새 진입점은 `carla_moving_capture_dev_v2.py`, `collect_carla_moving_dev_v2.py`,
`run_owned_carla_moving_dev_v2.sh`, `audit_carla_moving_dev_v2.py`,
`qualify_carla_moving_dev_common10_v2.py`, `prepare_carla_common10_dev_v2.py`다.
이들은 **이번 정확한 Town03 사례**를 위한 계약이다. 임의 맵/seed용 범용 수집 완료로 표시하지 않는다.

- 실행 기반 HEAD: `3cb676acbd2be4511be156d43ec002c554b438f3` + 미커밋 소스.
  HEAD만으로 실행 코드 전체를 재현할 수 없다. receipt/owner의 소스 SHA와 보관본이 기준이다.
- native 및 owner 원본: `artifacts/training/2026-09-16/moving_dev_capture_v2/01_town03_straight/`.
- 준비 데이터: `artifacts/training/2026-09-16/moving_dev_capture_v2/prepared_moving_dev_v2/`.
- 이 게시 폴더에는 분석·화면·검증서·변환 소스 snapshot을 모았다. 전체 원시 데이터는 Git clone에 포함되지 않는다.
- 초기 `validation_report.json`의 경로가 `.partial`인 것은 rename 전 검사한 기록이다.
  `postrename_verification.json`은 최종 경로에서 별도로 재검사한 결과다. 과거 보고서는 수정하지 않았다.
- 데이터의 `project-generated-carla-internal-review` 표기는 기존 내부 연구 분류이며 공개 재배포 허가를 뜻하지 않는다.
- 시스템/venv 설치, 재부팅, 원격 작업, commit/push, 기존 증거 삭제를 하지 않았다.

기존 로컬 환경과 데이터가 있을 때 아래 명령은 **검증만** 한다. 학습·CARLA를 시작하지 않는다.

```bash
cd /home/a/autoware_e2e
PYTHONDONTWRITEBYTECODE=1 python3 -B -c '
from portable_e2e import contract, dataset
p = "artifacts/training/2026-09-16/moving_dev_capture_v2/prepared_moving_dev_v2"
r = contract.validate_dataset(p, mode="planning", check_image_hashes=True)
d = dataset.load_training_examples(p, split="val", mode="planning", check_image_hashes=True)
print(r["status"], r["split_sample_counts"], len(d.examples), d.fingerprint_sha256)
'
```

## 6. 다음에는 무엇을 하나

이번 단계는 **학습할 모델을 평가할 직진 DEV 자료를 확보하는 단계**였다. 새 표본이 TRAIN이
아니므로 이번 작업 자체가 모델 가중치나 성능을 높이지는 않는다.

1. 이미 확보한 Town07 TRAIN 328개와 이번 Town03 DEV 328개를 기존 TRAIN/DEV에 결합한
   새 데이터 버전의 중복·split·기능별 분포를 고정한다. 기존 TEST는 계속 잠근다.
2. **직진 보강 효과를 보는 학습 A/B**를 진행한다. 기존 고정 데이터 대비 새 TRAIN 추가 효과를
   같은 모델 구조·학습 예산·seed로 비교하며 기존 회전 DEV와 새 직진 DEV의 성능을 따로 보고한다.
   학습 위치는 PRO6000 물리 GPU 0, 패키지는 개인 `hwanhong` venv만 사용한다.
   서버 상태와 데이터 수신 해시를 확인한 뒤 실행하며, 이번에는 예약하거나 시작하지 않았다.
3. 자연 감속·정지 유지·회전 등 부족한 TRAIN/DEV 데이터를 별도 수집한다. 새 직진 episode 하나로
   ACC·회피·차선변경·정지 기능 검증을 대신하지 않는다. 정지 유지 DEV 부족도 아직 남아 있다.
4. 개선 모델이 offline 기준을 통과하면 로컬 export 동등성 → Autoware 10 Hz shadow →
   fallback 확인 → 제한된 CARLA 모델 직접 주행 순으로 진행한다. 지금 실차 적용 승인은 아니다.

다음 학습의 소요 시간은 데이터 버전과 서버 부하 확인 후 추정한다. 이전 6모델·20 epoch 실행이
약 2시간이었던 것은 참고치일 뿐 이번 완료 시간 보장이 아니다. VAD/METEOR 등과 같은 조건의
벤치마크가 없어 동등·우월 성능이라고 비교할 수도 없다.

[현재 목차](../CURRENT.md) · [직전 경로 사전검사](../01_dev_route_preflight/README.md) ·
[직전 실제 학습 결과](../../../2026-09-15/portable_e2e_learning_cycle_v1/CURRENT.md)
