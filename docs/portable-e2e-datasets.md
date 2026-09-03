# Portable E2E 데이터셋 선택·반입 가이드

작성 기준일: 2026-09-03

대상: CARLA와 실차 데이터를 같은 6-camera + ego trajectory + route 형식으로 학습하려는 초보 운영자

## 먼저 결론

이 프로젝트에서 데이터 준비와 학습은 서로 다른 단계다.

1. 원본 파일을 내려받는다.
2. 압축 파일의 무결성과 구조를 확인한다.
3. 원본 timestamp·calibration·route를 보존한 채 프로젝트 공통 형식으로 변환한다.
4. 변환 결과가 `common_10hz_v1` 계약을 만족하는지 검증한다.
5. 검증을 통과한 뒤에만 작은 학습 smoke test를 시작한다.

현재 추천 순서는 다음과 같다.

1. **Bench2Drive Mini**: CARLA 파서와 변환 파이프라인만 검증한다. Mini 자체로 의미 있는 모델을 학습하려고 하지 않는다.
2. **nuScenes mini schema/adapter smoke**: 실제 카메라·calibration·timestamp·CAN route를
   처음 연결한다. 현재 planning cadence gate의 학습 자격 자료로는 쓰지 않는다.
3. **nuPlan 제한 subset**: 장시간 10 Hz 실차 데이터와 mission route를 본격적으로 연결한다.
4. 위 세 단계의 리포트가 모두 통과한 뒤에만 더 큰 데이터를 승인받아 확장한다.

Waymo Open Dataset E2E와 Argoverse 2는 기술적으로 참고할 가치는 있지만, 현재 **실차에 올릴 수 있는 가중치의 학습 원천에서는 제외**한다. 두 데이터셋의 표준 이용 조건은 차량 운용 또는 시험장 prototype 사용과 충돌한다.

## 현재 상태: 다운로드, 데이터 준비, 학습을 혼동하지 않기

2026-09-03 원격 학습 서버 검증 결과를 반영한 실제 상태는 다음과 같다. 실제 계정 경로는
공개 문서에 기록하지 않고 `$PORTABLE_E2E_ROOT`를 기준으로 표시한다.

| 단계 | Bench2Drive Mini 상태 | 의미 |
|---|---|---|
| remote 임시 폴더로 전송 | **완료** | `$PORTABLE_E2E_ROOT/tmp/downloads/bench2drive/legacy-mini-10-research-only/archives`에 저장됐다. |
| 예상 archive 전체 존재 확인 | **PASS: 10/10** | tag `0.0.3` 공식 Mini manifest의 정확한 10개 파일이 있다. |
| 파일 크기·SHA-256 확인 | **PASS** | 내장 공식 size/SHA verifier가 2,812,733,742 / 2,812,733,742 bytes를 검증했다. |
| 예상 밖 파일·partial·symlink 검사 | **PASS** | fail-closed verifier 전체 결과가 PASS다. |
| 압축 해제 | **아직 안 함** | 검증된 `.tar.gz` archive만 있으며 dataset tree는 아직 없다. |
| common10 변환 | 아직 안 함 | camera 순서, 좌표계, trajectory, route를 아직 맞추지 않았다. |
| 변환 데이터 검증 | 아직 안 함 | 10 Hz, timestamp gap, bundle skew, horizon을 아직 판정하지 않았다. |
| 학습 | **아직 시작하지 않음** | checkpoint나 학습 결과가 생성된 상태가 아니다. |

따라서 현재 상태를 한 문장으로 표현하면 다음과 같다.

> Bench2Drive legacy Mini 10개 archive의 다운로드와 공식 size/SHA 검증은 완료됐고, 압축 해제·common10 변환·학습은 아직 수행되지 않았다.

현재 분류는 **다운로드/무결성 검증 완료, 미해제·미변환·미학습**이다.

따라서 archive download가 끝난 사실을 “dataset 준비 완료” 또는 “학습 시작”으로 표현하면 안 된다. 현재 단계에는 실행 중이거나 완료된 학습 process와 checkpoint가 없다.

nuScenes v1.0-mini 원본 archive도 공식 tutorial URL에서 별도 격리 경로로 다운로드했다.

| 항목 | 확인 결과 |
|---|---|
| 원격 원본 | `$PORTABLE_E2E_ROOT/datasets/raw/nuscenes/v1.0-mini/research-only-pending-terms-review/v1.0-mini.tgz` |
| byte | **4,167,696,325** |
| SHA-256 | `943037abbb3b26b3070dc76504a43eb440503b00baf9ac2f1538d9c03fc9298f` |
| streaming tar audit | **PASS**, 31,252 entries = regular 31,224 + directory 28 |
| link/기타/위험 경로/중복 이름 | 모두 0 |
| 현재 상태 | **미해제·미변환·미학습, terms review pending** |

이 검사는 gzip/tar stream 전체를 읽어 container 손상과 기본 경로 안전성만 확인했다. CAN bus
expansion은 아직 받지 않았고 camera cadence, calibration, route와 label 적합성도 아직
검증하지 않았다. 따라서 이 archive는 데이터 준비나 실차 사용 승인을 뜻하지 않는다.

nuPlan v1.1 mini도 전체 mini가 아니라 custom camera adapter smoke에 필요한 제한 subset만
격리 경로로 반입하고 있다.

원격 root:
`$PORTABLE_E2E_ROOT/datasets/raw/nuplan/v1.1-mini/research-only-pending-terms-review`

| 원본 | 공식 object byte | 현재 상태 | 검증 범위 |
|---|---:|---|---|
| `nuplan-v1.1_mini.zip` | 8,550,100,030 | **다운로드 완료, 미해제** | exact byte 일치, local SHA-256 `a3fe40afd81cc634884f8d0b7ea3604f2e617e365d5c258c61cfdd833c8d987b`, 전체 ZIP payload CRC PASS |
| `nuplan-maps-v1.0.zip` | 971,557,640 | **다운로드 완료, 미해제** | exact byte 일치, local SHA-256 `d0310009fa9e8dd88014038336538aca678842c009fbf03fae76ed28f702ffc6`, 전체 ZIP payload CRC PASS |
| `nuplan-v1.1_mini_camera_0.zip` | 52,219,710,368 | **분할 다운로드 진행 중** | final archive 조립 후 exact byte, local SHA-256, 전체 payload CRC를 검증해야 완료 |

세 파일의 계획 압축 합계는 **61,741,368,038 bytes**다. 완료된 ZIP은 원본을 추출하지
않고 모든 member payload를 읽어 CRC를 확인했다. 위 SHA-256은 이번에 받은 파일의 local
provenance 값이며 공개된 공식 checksum이라고 표현하지 않는다. camera group 0까지 최종
검증을 통과해도 압축 해제·common10 변환·학습은 별도 단계다. package 설치와 GPU 사용도
수행하지 않았다.

`research-only-pending-terms-review`는 데이터 이용조건이 적용되지 않는다는 뜻이 아니다.
프로젝트의 intended use, 변환본·checkpoint 처리와 실차 사용 범위를 내부 승인하기 전이라는
상태 표시다.

## 이 프로젝트가 요구하는 `common10` 형식

정식 기준은 [`portable_e2e/config/common_10hz_v1.contract.json`](../portable_e2e/config/common_10hz_v1.contract.json)이다. 핵심만 풀어 쓰면 다음과 같다.

| 항목 | 프로젝트 기준 |
|---|---|
| camera 수와 순서 | `FRONT, BACK, FRONT_LEFT, BACK_LEFT, FRONT_RIGHT, BACK_RIGHT` |
| 저장/학습 영상 | 640×360 JPEG, decode 후 RGB8, rectified |
| 기준 rate | nominal 10 Hz, 실효 rate 최소 9.5 Hz |
| episode 길이 | 최소 30초 |
| camera 간 timestamp 차이 | offline dataset bundle 최대 20 ms |
| native frame 결합 | camera timestamp는 원본 timestamp와 같고, hashed source manifest의 frame ID/index/hash와 일치 |
| trajectory | anchor 시점의 `base_link` 기준, 0.1초 간격, 0.1~6.4초, 64점 |
| route input | hashed global route + anchor ego pose에서 1 m 간격, 최대 120 m로 validator가 재생성 |
| trajectory 유효률 | 최소 95% |
| 명령 | left, right, straight, lane_follow, change_left, change_right |
| 누락값 | 0으로 채우지 않고 `available`과 `valid_mask`로 표시 |
| split | route/site/day가 train과 val/test에 겹치지 않게 분리 |

`10 Hz 형식으로 저장했다`와 `실제로 10 Hz로 관측했다`는 다르다. 4 Hz 이미지를 복사하여 10개로 늘리거나, 같은 JPG에 새 timestamp만 붙이는 것은 10 Hz 데이터가 아니다.

### 원본을 변환본에 묶는 `source-manifest.v1`

각 episode의 hashed source manifest는 최소한 다음을 보존한다.

- `source_dataset_id`, 정확한 version, `license_id`
- 원본 archive/object의 URI와 SHA-256
- 여섯 camera 각각의 원본 frame ID, 원본 timestamp, 원본 sequence index
- 원본 payload SHA-256, 변환된 JPEG SHA-256, 승인된 image transform ID
- 선택된 sample ID와 그 sample에 들어간 여섯 원본 frame ID
- mission route가 anchor 전에 이용 가능했던 시각, 원본 route payload와 episode route hash

validator는 selected sample과 이 manifest를 다시 대조하고, camera의 저장 timestamp가 native
source timestamp와 정확히 같은지도 확인한다. 정지 장면은 실제로 같은 JPEG payload가 반복될
수 있으므로 JPEG scan 반복률은 진단값으로 남기되 단독 fail 조건으로 쓰지 않는다. 대신 ID,
timestamp, sequence, prepared hash와 selected bundle을 함께 묶는다.

이 검사는 변환 도중의 실수·재타이밍·manifest 불일치를 막는 장치다. 외부 archive의 출처가
진짜인지 또는 해당 약관으로 학습해도 되는지까지 암호학적으로 증명하지는 않는다. 따라서
report의 `raw_source_artifact_content`는 raw source 검토 전 `NOT_RUN`이고, release에는 원본
SHA/다운로드 기록/약관/intended-use 수동 검토가 계속 필요하다.

실세계 rig도 두 종류를 구분한다. 공개·기록 데이터에는 source calibration과 rectification
evidence를 고정하는 `recorded_dataset` profile을 쓰고, 실제 차량 live input에만 만료일/PTP
offset/commissioning evidence가 있는 `live_vehicle` profile을 쓴다. 공개 과거 데이터에 현재
차량 commissioning 승인을 가짜로 붙이지 않는다.

### route 정답 누설 방지

sample이 임의 `route_polyline_base_m`를 선언하는 방식은 허용하지 않는다. validator가 hashed
map-frame route에 ego anchor pose를 투영하고, 가장 가까운 segment가 교차할 때는 현재 heading과
segment index로 결정적으로 분기를 정한 뒤, 진행 방향으로 1 m마다 최대 120 m를 다시 만든다.
sample route의 점 개수·좌표·anchor arc와 mission endpoint가 이 결과와 허용 오차 5 cm 안에서
같아야 한다. 따라서 역순 route, 분기 점프, 미래 ego 위치 간격을 route spacing으로 넣는 입력은
planning 자격을 받지 못한다.

## 로컬 확인 및 원격 전송을 마친 legacy 데이터

로컬 source 수치는 2026-09-03에 파일을 수정하지 않고 metadata와 manifest만 읽어 확인했다. legacy export manifest에서 `status: validated`인 것은 정확히 11개다. 이후 이 11쌍을 원격 학습 서버로 전송하고 checksum dry-run으로 다시 대조했다.

여기서 **legacy validated**는 과거 exporter의 안전성·경로·label 검사에 통과했다는 뜻이다. `common_10hz_v1` planning 검사에 통과했다는 뜻은 아니다.

아래 3~11번에서 “같은 `seed_0000` 아래 `export`”는 raw의 마지막 `/episode`를 `/export`로 바꾼 정확한 형제 경로를 뜻한다.

| 번호 | raw episode | 대응 export | camera / anchor | export sample |
|---:|---|---|---:|---:|
| 1 | `artifacts/validation/2026-08-31/c_track_394_to_290_expert` | `artifacts/validation/2026-08-31/c_track_394_to_290_export` | 4 Hz / 73 | 61 |
| 2 | `artifacts/validation/2026-08-31/town01_left_expert` | `artifacts/validation/2026-08-31/town01_left_export` | 4 Hz / 102 | 90 |
| 3 | `artifacts/validation/2026-08-31/maps/c_track_1_0_7/smoke/c_track_1_0_7/c_track_1_0_7_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 120 | 105 |
| 4 | `artifacts/validation/2026-08-31/maps/town01/smoke/town01/town01_lane_follow_s0001_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 121 | 106 |
| 5 | `artifacts/validation/2026-08-31/maps/town02_opt/smoke/town02_opt/town02_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 129 | 114 |
| 6 | `artifacts/validation/2026-08-31/maps/town03/smoke/town03/town03_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 148 | 133 |
| 7 | `artifacts/validation/2026-08-31/maps/town04/smoke/town04/town04_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 122 | 107 |
| 8 | `artifacts/validation/2026-08-31/maps/town05_opt/smoke/town05_opt/town05_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 110 | 95 |
| 9 | `artifacts/validation/2026-08-31/maps/town06/smoke/town06/town06_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 121 | 106 |
| 10 | `artifacts/validation/2026-08-31/maps/town07/smoke/town07/town07_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 121 | 106 |
| 11 | `artifacts/validation/2026-08-31/maps/town10hd_opt/smoke/town10hd_opt/town10hd_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/episode` | 같은 `seed_0000` 아래 `export` | 5 Hz / 121 | 106 |

11개 raw+export 쌍의 합계는 다음과 같다.

- **regular-file payload 646,642,112 bytes, 약 616.686 MiB**
- camera anchor 1,288개
- JPEG 7,728개: anchor마다 6 cameras
- legacy export sample 1,129개
- 6 cameras, 640×360 JPEG, route JSON, 20 Hz ego state/dynamics/control
- legacy future label은 0.5초 간격의 0.5~3.0초, 6점

앞서 로컬 `du -sb`로 얻은 647,047,616 bytes, 약 617.073 MiB는 directory entry의 apparent size까지 더한 tree 합계다. 실제 전송 payload를 말할 때는 regular file만 합산한 **646,642,112 bytes**를 사용한다. 두 값의 차이 405,504 bytes는 dataset file 누락이 아니다.

원격 학습 서버 전송도 완료됐다.

| 검사항목 | 원격 결과 |
|---|---|
| destination | `$PORTABLE_E2E_ROOT/datasets/legacy/carla_2026-08-31/source_tree` |
| regular files | **7,805** |
| regular-file payload | **646,642,112 bytes** |
| symlink | **0** |
| `rsync --checksum --dry-run` 변경 대상 | **0** |

7,805개는 JPG 7,728개, raw metadata 44개, export metadata 33개의 합계다. checksum dry-run의 변경 대상 0은 검증 시점에 source와 destination의 regular-file 내용이 같았다는 뜻이며, dry-run 자체는 원격 파일을 수정하지 않는다.

export 폴더만 복사하면 안 되는 조건도 지켜졌다. `samples.jsonl`의 `images/...` 경로는 대응 raw episode의 image를 가리키므로, 원격 `source_tree`에는 위 표의 raw와 export 쌍이 함께 보존돼 있다. 다음 adapter도 이 쌍을 입력으로 받아 새 common10 tree를 별도로 만들어야 한다.

11개 모두 common10 정식 학습 corpus에는 아직 들어갈 수 없다.

- camera가 4 Hz 또는 5 Hz여서 실효 9.5 Hz 기준에 미달한다.
- 가장 긴 raw도 30초 최소 episode 기준에 미달한다.
- legacy trajectory가 3.0초/6점이라 6.4초/64점 기준에 미달한다.
- 4/5 Hz image를 반복하거나 timestamp만 바꿔 10 Hz로 표시하면 안 된다.
- 대부분 lane-follow 표본이다. 두 번째 `town01_left` export만 left 17 sample과 lane-follow 73 sample을 포함한다.

따라서 이 11쌍의 목적은 legacy adapter, schema, camera permutation, route/command 변환, 작은 overfit smoke다. 본 학습용 10 Hz CARLA corpus는 36.4초 이상의 새로운 직진·회전 episode로 별도 수집해야 한다.

제외 대상을 혼동하지 않는다.

- `.../town01_lane_follow_s0000_p00/.../episode`는 raw는 있지만 대응 export가 없어 11쌍에서 제외한다.
- `90_quarantine` 아래 실패 episode와 `pytest-of-a` fixture는 전송·학습 대상이 아니다.
- 화면 GIF, RViz PNG, log만 있는 validation trial은 expert dataset으로 간주하지 않는다.

상위 폴더의 현재 크기는 `docs/assets` 약 1.3 GiB, `artifacts/validation` 약 20 GiB, `data` 약 15 GiB였다. 이 크기 전체가 학습 데이터라는 뜻은 아니다.

- `docs/assets`는 주로 PNG/GIF와 공개용 실행 증거다.
- `artifacts/validation`은 log, rosbag, 화면 녹화, 결과 JSON 등이 혼재한 검증 산출물이다.
- `data`의 대부분은 vendor library, CUDA/TensorRT, map, 기존 model이다.

화면 GIF나 RViz PNG는 사람이 결과를 확인하는 evidence이지, camera-calibration-trajectory-route가 동기화된 학습 sample은 아니다.

## 후보 데이터셋 비교

용량은 서로 다른 공개 방식 때문에 모두 같은 기준은 아니다. 아래에는 공식 표기값 또는 공식 object metadata를 합산한 **압축 다운로드 기준**을 적고, 추정치와 전체 원시 용량을 구분했다.

| 데이터셋 | cameras / 원본 rate | ego trajectory | route/command | 확인 용량 | common10과 주요 차이 | 현재 용도 |
|---|---|---|---|---:|---|---|
| Bench2Drive Mini | RGB 6개, 수집 10 Hz | 연속 ego pose에서 생성 가능 | near/far command·target, HD map | 공식 약 4 GB; Mini 10개 archive 합계 2,812,733,742 bytes | camera 배열 순서 변환, 6.4초 trajectory와 route polyline 파생 필요 | **1순위 parser smoke** |
| Bench2Drive v0.0.4 | RGB 6개, 10 Hz | pose 기반 생성 | command·target·HD map | no-depth 약 400 GB, depth 포함 7.3 TB | version/format 차이, 라이선스 제약 | Mini 통과 후 별도 승인 |
| nuScenes v1.0-mini | camera 6개, 12 Hz | ego pose/CAN에서 생성 | CAN ideal route 존재, 약 3% scene 누락 | mini archive 약 4.168 GB; full CAN 약 0.781 GB | scene 20초; 10 Hz thinning 시 약 166.7 ms gap으로 현재 p99 150 ms gate 불통과; camera skew audit 필요 | **2순위 schema/real-adapter smoke 전용** |
| nuScenes trainval cameras | camera 6개, 12 Hz | 동일 | 동일 | camera archive 합계 약 177.288 GB | 30초 episode 불가, 현 planning cadence gate 불통과, non-commercial | 별도 profile/contract 검토 전 정식 corpus 제외 |
| nuPlan raw | camera 8개, 10 Hz | 10 Hz ego pose와 dynamics | route roadblock IDs, mission goal, HD map | 최소 mini DB+map+camera group 0 약 61.741 GB; 전체 mini camera 약 450.668 GB | 8→6 선택, 2000×1200 보정 resize, skew audit | **3순위 본격 real source** |
| NAVSIM/OpenScene | nuPlan 8 cameras, 보통 2 Hz history | 출력 trajectory 4초/10 Hz | left/straight/right/unknown | trainval logs 14 GB, sensors 2 TB 초과; navtrain sensor 300~445 GB | 입력 2 Hz, horizon 4초, command가 너무 거침 | framework/evaluation 참고 |
| Waymo E2E | camera 8개, 10 Hz | 미래 5초/4 Hz | left/straight/right | 공식 페이지에 총 byte 미표기 | 12/20초, 5초 horizon, 표준 license의 vehicle-use 금지 | deployable 학습에서 제외 |
| Argoverse 2 Sensor | ring 7 + stereo 2, 20 Hz | 6DoF ego pose | mission route 없음 | 약 1 TB | 15초, route 부재, 시험장 prototype 이용조건 충돌 | deployable 학습에서 제외 |

접근·라이선스를 빠르게 비교하면 다음과 같다.

| 데이터셋 | 공식 접근 | 현재 프로젝트의 권리 판단 |
|---|---|---|
| Bench2Drive | GitHub 문서와 Hugging Face archive | repository의 CC BY-NC-ND 계열 표기와 dataset host metadata를 함께 확인해야 한다. 수정 dataset/checkpoint 배포와 상업·차량 사용은 서면 확인 전 보류한다. |
| nuScenes | 공식 계정 download 또는 AWS Open Data | 무료 공개판은 비상업 조건이다. 상업·산업용은 Motional commercial license를 확인한다. |
| nuPlan | 공식 사이트/조건 동의 또는 AWS Open Data | download 경로가 공개 S3여도 dataset 조건이 없어지는 것은 아니다. 이용 시점의 약관과 version을 manifest에 보존한다. |
| NAVSIM/OpenScene | 공식 repository 안내의 split/archive | code license와 data license가 다르다. OpenScene/nuPlan의 비상업·share-alike 조건을 각각 확인한다. |
| Waymo E2E | 공식 신청/Google 계정과 GCS | 표준 조건으로 학습한 weight를 차량 운용·운전 보조에 사용하지 않는다. deployable branch와 완전히 분리한다. |
| Argoverse 2 | 공식 사이트에서 조건 동의 후 download | 비상업/share-alike 조건과 AV test-track prototype 제한 때문에 deployable branch에서 제외한다. |

AWS Open Data나 Hugging Face에서 로그인 없이 byte를 받을 수 있다는 사실은 학습·재배포·차량 사용 권한을 자동으로 주지 않는다. 각 원본의 license/terms URL, 확인 날짜, intended use를 dataset manifest에 기록한다. 이 문서의 판단은 법률 자문이 아니다.

## 1순위: Bench2Drive Mini

### 무엇이 들어 있는가

Bench2Drive는 CARLA에서 expert가 주행한 대규모 closed-loop benchmark다. 공식 논문과 annotation 문서가 설명하는 학습 자료에는 다음이 포함된다.

- 6개 RGB camera와 camera별 intrinsic/extrinsic
- ego 위치, 방향, 속도, 가속도, angular velocity
- throttle, steer, brake
- near/far navigation command와 target waypoint
- 객체 annotation과 HD map/topology
- weather와 scenario metadata

원 데이터는 10 Hz로 수집됐다. 공개 benchmark의 원래 학습 자료는 약 200만 annotated frame, 13,638개 short clip, 44개 interactive scenario, 23개 weather, 12개 town 규모다.

Mini는 이 전체를 대표하지 않는다. 공식 Mini 목록은 Base dataset에서 고른 10개 scene/archive로, town과 파서가 정상인지 확인하기 위한 작은 표본이다.

### 중요한 version 주의사항

현재 공식 repository의 최신 문서는 v0.0.4를 설명하지만, 공식 `download_mini.sh`와 `bench2drive_mini_10.json`이 가리키는 Mini는 legacy Base/v0.0.3 계열 archive다.

즉 다음을 동일한 것으로 취급하면 안 된다.

- 현재 다운로드·SHA 검증을 마친 **Bench2Drive legacy Mini 10 archives**
- v0.0.4의 **1,100 routes / no-depth 약 400 GB** 학습 세트

Mini로 작성한 parser는 v0.0.4 전체 자료에서 다시 schema/version 검사를 받아야 한다.

### common10으로 바꿀 때 필요한 일

Bench2Drive 계열의 공개 camera order는 보통 다음 순서다.

```text
FRONT, FRONT_LEFT, FRONT_RIGHT, BACK, BACK_LEFT, BACK_RIGHT
```

프로젝트 common10 순서는 다음과 같다.

```text
FRONT, BACK, FRONT_LEFT, BACK_LEFT, FRONT_RIGHT, BACK_RIGHT
```

파일을 읽을 때 이름으로 매핑해야 하며, 배열 index를 그대로 사용하면 뒤/좌/우 image가 잘못 연결될 수 있다.

또한 raw annotation에 이미 common10 형식의 `64 × (x, y, yaw)` tensor가 들어 있다고 가정하면 안 된다. 연속 ego pose와 원본 timestamp를 이용해 각 anchor 이후 6.4초 trajectory를 anchor의 `base_link` 좌표로 변환하고, scene 끝에서 부족한 점은 `valid_mask=false`로 표시해야 한다.

near/far target은 route 조건의 한 형태이지만, 미래 ego trajectory와 route polyline은 다르다. route는 HD map과 route metadata에서 만들고, expert가 실제로 간 미래 위치를 route인 것처럼 누설하지 않는다.

### 접근과 라이선스

- [Bench2Drive 공식 repository](https://github.com/Thinklab-SJTU/Bench2Drive)
- [공식 annotation 설명](https://github.com/Thinklab-SJTU/Bench2Drive/blob/0.0.4/docs/anno.md)
- [공식 Mini download script](https://github.com/Thinklab-SJTU/Bench2Drive/blob/main/tools/download_mini.sh)
- [검증에 사용한 공식 Mini 10 archive manifest — tag 0.0.3](https://github.com/Thinklab-SJTU/Bench2Drive/blob/0.0.3/docs/bench2drive_mini_10.json)
- [Bench2Drive Base dataset](https://huggingface.co/datasets/rethinklab/Bench2Drive)
- [Bench2Drive v0.0.4 dataset](https://huggingface.co/datasets/rethinklab/Bench2Drive-V0.0.4)
- [Bench2DriveZoo VAD branch](https://github.com/Thinklab-SJTU/Bench2DriveZoo/tree/uniad/vad)
- [NeurIPS 2024 공식 논문](https://papers.nips.cc/paper_files/paper/2024/file/017761f94a1cd66d01c041aff85492c4-Paper-Datasets_and_Benchmarks_Track.pdf)

공식 repository는 별도 표기가 없는 asset/code에 CC BY-NC-ND 계열 조건을 제시한다. Hugging Face metadata의 표기와 충돌해 보이는 부분도 있으므로, 수정 데이터나 fine-tuned weight를 배포하거나 상업·실차 목적으로 쓰기 전에는 권리자에게 서면으로 범위를 확인한다. 이 문서는 법률 자문이 아니다.

## 2순위: nuScenes mini schema/adapter smoke

### 왜 두 번째인가

nuScenes는 common10과 똑같은 이름의 6-camera surround rig를 제공한다.

```text
CAM_FRONT
CAM_BACK
CAM_FRONT_LEFT
CAM_BACK_LEFT
CAM_FRONT_RIGHT
CAM_BACK_RIGHT
```

1,000개 scene, 각 약 20초, camera 약 12 Hz이며 전체 camera image는 약 140만 장이다. GPS/IMU와 calibrated sensor pose가 있고, CAN bus expansion에는 pose와 ideal route polyline이 있다.

따라서 Mini는 다음을 처음 검증하기 좋다.

- 실세계 camera의 exposure/timestamp 차이
- camera별 calibration과 image rectification
- CAN ego pose와 camera anchor 결합
- ideal route와 ego future를 서로 다른 label로 유지
- scene 경계에서 trajectory mask 처리

### common10에 바로 합격하지 않는 이유

- scene이 20초라서 common10 최소 30초를 만족하지 않는다.
- 12 Hz camera에서 image를 복제·합성하지 않고 10 Hz만 고르면 일부 연속 frame 사이가
  약 166.7 ms가 된다. 이는 현재 p99 gap 150 ms planning cadence gate를 통과하지 못한다.
- camera가 lidar sweep을 기준으로 서로 다른 시점에 trigger될 수 있어, 6장이 20 ms 이내인지 실제 파일별로 측정해야 한다.
- CAN route는 일부 scene에서 누락된다. 공식 CAN 문서는 약 3% 누락을 언급한다.
- 20초 scene의 끝을 넘어 다른 scene의 pose를 연결하면 안 된다.

따라서 nuScenes mini는 현재 **schema/adapter smoke 전용**이며 `common_10hz_v1` planning
validation 대상이 아니다. 실제 timestamp를 모두 측정한 뒤 `source_profile=nuscenes_20s`
같은 별도 profile/contract를 설계·검토·고정하기 전에는 정식 common10 학습 corpus에서
제외한다. 현재 validator에는 이 예외 profile이 구현되어 있지 않다. 20초 scene을 억지로
이어 붙여 30초로 만들지 않는다.

### 용량과 접근

- official mini archive: object metadata 기준 약 4.168 GB compressed
- full trainval camera archives: 합계 약 177.288 GB compressed
- test camera archives: 합계 약 33.066 GB compressed
- CAN bus expansion archive: 약 0.781 GB compressed

첫 단계에는 full trainval을 받지 않는다. official `v1.0-mini`와 그 version에 맞는 metadata/map/CAN만 준비해 adapter smoke를 수행한다.

- [공식 tutorial의 익명 v1.0-mini archive](https://www.nuscenes.org/data/v1.0-mini.tgz)
- [공식 nuScenes tutorial](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/tutorials/nuscenes_tutorial.ipynb)
- [nuScenes 공식 페이지](https://www.nuscenes.org/nuscenes)
- [공식 schema 문서](https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/schema_nuscenes.md)
- [공식 CAN bus 문서](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/can_bus/README.md)
- [공식 AWS Open Data registry](https://registry.opendata.aws/motional-nuscenes/)
- [비상업 이용 조건](https://www.nuscenes.org/terms-of-use)
- [상업 이용 안내](https://www.nuscenes.org/terms-of-use-commercial)

nuScenes의 일반 공개 조건은 비상업 사용을 전제로 한다. 향후 수익 목적 산업 R&D, 배포 또는 차량 사용은 Motional의 별도 commercial license 범위를 확인한다.

## 3순위: nuPlan 제한 subset

### 왜 본격 실차 source로 가장 적합한가

nuPlan은 네 도시에서 수집한 약 1,200시간의 사람 운전 기록을 제공하고, 그중 약 120시간은 raw sensor 자료다. 주요 sensor 구성은 다음과 같다.

- camera 8개, 10 Hz, 원본 2000×1200
- lidar 5개, 20 Hz
- IMU 100 Hz
- GPS 20 Hz
- calibrated ego pose, velocity, acceleration, angular rate
- mission goal, route roadblock IDs, HD map

입력이 원래 10 Hz이고 장시간 log와 mission route가 있어, 실제 6-camera trajectory planner의 본격 자료로는 가장 가깝다.

### 왜 전체 mini camera 9개 group이 아니라 제한 subset만 받는가

nuPlan raw sensor는 전체 20 TB가 넘는다. 공식 object metadata를 기준으로 camera archive만 보아도 대략 다음과 같다.

- train cameras: 약 5.389 TB compressed
- validation cameras: 약 0.867 TB compressed
- train + validation cameras: 약 6.256 TB compressed
- mini cameras: 합계 약 0.451 TB compressed

전체 mini camera 9개 group을 한꺼번에 받으면 약 450.7 GB다. 첫 실용 subset은 공식 mini
DB 8.550 GB, map v1.0 0.972 GB와 camera group 0 52.220 GB, 합계 약 **61.741 GB**다.
group 0은 공식 manifest상 7개 log를 포함한다. 사용자가 원본 반입을 승인해 이 세 archive는
adapter보다 먼저 내려받았지만, 이는 CPU/network staging을 앞당긴 것뿐이다. 약관·용도 gate,
압축 해제, adapter 검증과 학습 순서는 건너뛰지 않는다. 이 구성은 공식 devkit 전체 설치가
아니라 프로젝트 custom camera adapter smoke용 제한 subset이다.

### common10 adapter의 핵심

- 8개 camera 중 6개를 이름만 보고 추측하지 않고, calibration yaw와 FoV를 확인해 선택한다.
- 2000×1200을 640×360으로 찌그러뜨리지 않는다. 먼저 rectification/crop 정책을 정하고, resize 후 intrinsic `K`를 다시 계산한다.
- route roadblock IDs를 HD map 위의 polyline과 command로 변환한다.
- 연속 10 Hz ego pose로 6.4초 future trajectory를 생성한다.
- camera 6개의 실제 timestamp skew와 ego-state delta를 sample마다 기록한다.
- 전체를 받기 전에 선택한 소수 log 또는 한정 archive로 검증한다.

### 접근과 라이선스

- [nuPlan 공식 사이트](https://nuplan.org/)
- [nuPlan devkit](https://github.com/motional/nuplan-devkit)
- [공식 database schema](https://github.com/motional/nuplan-devkit/blob/master/docs/nuplan_schema.md)
- [공식 dataset setup](https://github.com/motional/nuplan-devkit/blob/master/docs/dataset_setup.md)
- [공식 AWS Open Data registry](https://registry.opendata.aws/motional-nuplan/)
- [Motional Dataset Terms](https://www.nuscenes.org/terms-of-use)
- [nuPlan devkit code license](https://github.com/motional/nuplan-devkit/blob/master/LICENSE.txt)
- [공식 mini DB archive](https://d1qinkmu0ju04f.cloudfront.net/public/nuplan-v1.1/nuplan-v1.1_mini.zip)
- [공식 map v1.0 archive](https://d1qinkmu0ju04f.cloudfront.net/public/nuplan-v1.1/nuplan-maps-v1.0.zip)
- [공식 mini camera group 0](https://d1qinkmu0ju04f.cloudfront.net/public/nuplan-v1.1/sensor_blobs/mini_set/nuplan-v1.1_mini_camera_0.zip)

nuPlan dataset은 별도 표기가 없는 한 CC BY-NC-SA 4.0과 Motional Dataset Terms가 함께
적용된다. Terms는 웹사이트 밖에서 내려받아 사용하는 경우에도 적용되며 derived data도
범위에 포함한다고 명시한다. 공식 사이트 경로는 account 생성과 Terms 동의를 요구하고,
AWS Open Data는 AWS account 없는 anonymous read를 제공한다. 이는 접근 방식의 차이일 뿐
사용 권리의 차이가 아니다. 상업적 이용은 Motional의 별도 license를 확인한다.

nuPlan devkit의 Apache-2.0은 code license이며 dataset license가 아니다. 변환본이나
checkpoint의 배포, 상업적 이용과 실차 사용 범위는 임의로 단정하지 않고 Motional에
서면 확인한다. 어느 경로로 받든 확인한 이용조건 URL·날짜·dataset version을 manifest에
기록한다. 이 문서는 법률 자문이 아니다.

## NAVSIM/OpenScene은 무엇에 쓰는가

NAVSIM/OpenScene은 nuPlan 기반의 end-to-end planning framework와 evaluation을 배우기 좋다. 하지만 public split의 camera history는 보통 2 Hz이고, route command는 left/straight/right/unknown으로 거칠며, 평가 trajectory는 4초/10 Hz다.

common10은 camera 10 Hz와 trajectory 6.4초/64점을 요구한다. NAVSIM 4초 trajectory만 쓰면 64점 중 40점, 즉 62.5%만 유효해 최소 95% 기준에 미달한다.

따라서 현재 역할은 다음과 같다.

- planner agent interface 참고
- PDM 기반 평가 아이디어 참고
- trajectory metric과 benchmark 구조 참고
- 10 Hz 6-camera 원천 dataset의 대체품으로는 사용하지 않음

- [NAVSIM 공식 repository](https://github.com/autonomousvision/navsim)
- [공식 agent 문서](https://github.com/autonomousvision/navsim/blob/main/docs/agents.md)
- [공식 split 문서](https://github.com/autonomousvision/navsim/blob/main/docs/splits.md)
- [OpenScene 공식 repository](https://github.com/OpenDriveLab/OpenScene)

OpenScene data에는 비상업/share-alike 조건과 nuPlan 조건이 연결될 수 있으므로, code의 Apache-2.0 표기만 보고 data와 checkpoint도 같은 권리라고 가정하지 않는다.

## Waymo E2E를 현재 제외하는 이유

Waymo E2E 자체는 기술적으로 강하다.

- 8 cameras, 10 Hz
- left/straight/right high-level intent
- 과거 4초와 미래 5초 trajectory
- 4,021개 상세 segment split, train/validation은 주로 20초, test는 12초

그러나 common10 관점에서도 다음 차이가 있다.

- 미래 trajectory가 5초/4 Hz이므로 6.4초 64점 중 실제 horizon은 최대 50점, 78.125%뿐이다.
- interpolation을 10 Hz로 해도 새 관측 정보가 생기지 않는다.
- 12초와 20초 segment는 30초 기준에 미달한다.
- 8→6 camera 선택과 3-class→6-class command 확장이 필요하다.

더 중요한 이유는 이용 조건이다. Waymo Open Dataset 표준 조건은 dataset으로 학습·정제한 model/weight를 derivative IP 범위에 포함하고, 이를 차량 운용이나 운전 보조에 사용하는 것을 제한한다. 공식 FAQ도 test track prototype 사용을 허용되는 예로 보지 않는다.

따라서 현재 정책은 다음과 같다.

- Waymo data를 deployable checkpoint의 train/validation mixture에 한 sample도 섞지 않는다.
- 필요하면 network와 weight가 완전히 분리된 offline benchmark에서만 검토한다.
- 차량 사용이 필요하면 Waymo로부터 별도 license를 먼저 받는다.

- [Waymo Open Dataset 공식 소개](https://waymo.com/open/about/)
- [공식 E2E Driving Dataset](https://waymo.com/open/data/e2e/)
- [공식 E2E protobuf schema](https://github.com/waymo-research/waymo-open-dataset/blob/master/src/waymo_open_dataset/protos/end_to_end_driving_data.proto)
- [공식 이용 조건](https://waymo.com/open/terms/)
- [공식 FAQ](https://waymo.com/open/faq/)

## Argoverse 2를 현재 제외하는 이유

Argoverse 2 Sensor Dataset은 약 1 TB, 1,000개 scenario, scenario당 약 15초이고, 7개 ring camera와 2개 stereo camera를 20 Hz로 제공한다. 6DoF ego pose와 HD map도 있다.

그러나 현재 planning model에는 다음 문제가 있다.

- 15초 scenario라 common10 30초 기준을 만족하지 않는다.
- 9개 camera에서 6개를 다시 선정해야 한다.
- 목적지까지의 mission route 또는 high-level route condition이 없다.
- 별도 Motion Forecasting Dataset은 10 Hz trajectory를 제공하지만 raw 6-camera input과 같은 형태로 짝지을 수 있는 planner dataset이 아니다.
- 공식 조건의 금지 사례에는 AV test track prototype을 위한 model 학습이 포함된다.

따라서 perception/offline 연구에는 참고할 수 있지만, 현재 실차 planner checkpoint를 위한 download 대상에서는 제외한다.

- [Argoverse 2 공식 페이지](https://www.argoverse.org/av2.html)
- [Argoverse 공식 이용 조건](https://www.argoverse.org/about.html)

## 권장 다운로드·검증 순서

### Stage 0 — 완료: Mini archive 다운로드와 공식 SHA 검증

이 단계는 2026-09-03에 완료됐다.

- remote archive 경로: `$PORTABLE_E2E_ROOT/tmp/downloads/bench2drive/legacy-mini-10-research-only/archives`
- 공식 기준: [Bench2Drive tag 0.0.3 Mini 10 manifest](https://github.com/Thinklab-SJTU/Bench2Drive/blob/0.0.3/docs/bench2drive_mini_10.json)
- 결과: **PASS, 10/10 archives**
- 검증 byte: **2,812,733,742 / 2,812,733,742**
- 검증 범위: 정확한 파일명, 파일별 byte, 파일별 SHA-256, missing/extra/partial, symlink와 non-regular entry
- 변경 범위: archive를 읽었을 뿐 download·extract·repair·rename·delete를 하지 않음

실행한 것과 같은 verifier 명령은 다음과 같다. 이 script는 tag `0.0.3`의 공식 10개 size/SHA-256을 code 안에 고정한 fail-closed 읽기 전용 검사기다.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
export REPO_ROOT="$PORTABLE_E2E_ROOT/autoware_e2e"
B2D_ARCHIVE_DIR="$PORTABLE_E2E_ROOT/tmp/downloads/bench2drive/legacy-mini-10-research-only/archives"

source "$PORTABLE_E2E_ROOT/venvs/py312/bin/activate"
cd "$REPO_ROOT"

CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python scripts/e2e/verify_bench2drive_mini.py "$B2D_ARCHIVE_DIR"
```

정상 요약은 다음과 같다.

```text
Bench2Drive legacy Mini archive verification: PASS
Verified: 10/10 archives (2812733742 / 2812733742 bytes)
```

검사 중에는 2.8 GB 전체의 SHA-256을 계산하므로 마지막 요약이 나올 때까지 화면 출력이 없을 수 있다. 이것은 학습 멈춤이 아니다. 이 PASS는 official archive byte의 무결성까지만 보장하며, 압축 해제 후 file structure나 common10 label 품질을 아직 보장하지 않는다.

### Stage 1 — Bench2Drive Mini parser smoke

hash는 모두 맞았다. 다음 단계에서 먼저 archive 내부 목록과 필요한 여유 공간을 확인한 뒤 별도 staging 경로에만 압축을 푼다. 현재 문서 갱신 시점에는 **아직 압축을 풀지 않았다**. 원본 archive는 수정하지 않는다.

아래 목록 명령은 archive를 추출하지 않는다. 실제 10개 모두에 대해 구조가 같은지 확인한 다음에 extraction을 승인한다.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
B2D_ARCHIVE_DIR="$PORTABLE_E2E_ROOT/tmp/downloads/bench2drive/legacy-mini-10-research-only/archives"
ARCHIVE_NAME='<exact-archive-name.tar.gz>'
tar -tzf "$B2D_ARCHIVE_DIR/$ARCHIVE_NAME" | sed -n '1,80p'
```

검사 순서는 다음과 같다.

1. 10개 archive와 scene 수를 센다.
2. 각 scene의 첫·중간·마지막 frame을 연다.
3. frame마다 6 camera가 모두 있는지 센다.
4. source timestamp가 단조 증가하는지, 중앙 period가 약 100 ms인지 계산한다.
5. camera intrinsic/extrinsic과 해상도를 읽는다.
6. ego pose, speed, command, target, HD map 참조를 확인한다.
7. 이름 기반 camera permutation을 적용한다.
8. common10 staging sample을 만든다.
9. 6.4초 trajectory와 route를 각각 검증한다.
10. validation report가 통과한 뒤 dataloader 1 batch와 model forward 1회를 실행한다.

Mini 단계의 완료 조건은 “학습 loss가 좋아졌다”가 아니라 **같은 원본을 반복 변환했을 때 동일한 sample manifest와 checksum이 나온다**는 것이다.

### Stage 2 — nuScenes schema/real-adapter smoke

official `v1.0-mini` archive 다운로드와 streaming 구조 검사는 완료됐다. 약관·용도 검토 후
별도 staging에만 풀고 다음 adapter 검사를 진행한다. CAN route 검증은 version에 맞는 별도
CAN bus expansion을 받기 전까지 완료로 표시하지 않는다. 이 단계는 현재 planning
validation이나 정식 common10 학습 자격을 만들지 않는다.

확인 항목:

- 정확한 6-camera 이름 매핑
- camera별 실제 timestamp와 bundle skew
- calibration 적용 전/후 image와 갱신된 intrinsic
- ego pose와 CAN pose의 timestamp 차이
- ideal route 존재 여부와 누락 flag
- native timestamp를 보존한 cadence report와 10 Hz thinning 시 약 166.7 ms gap 확인
- 20초 scene 경계의 trajectory valid mask

이 단계에서도 full trainval download나 장기 학습은 하지 않는다. 측정 결과에 맞는 별도
profile/contract가 검토·고정되기 전에는 변환 결과를 정식 common10 학습 corpus에 넣지 않는다.

### Stage 3A — 진행 중: nuPlan 제한 subset 원본 반입

사용자 승인 아래 official mini DB, map v1.0과 camera group 0만 골라 격리 경로에 받고 있다.
DB와 map은 exact byte, local SHA-256 기록과 전체 ZIP payload CRC까지 통과했다. camera group
0은 최종 archive의 같은 검사가 끝나기 전까지 진행 중이다. 원본 반입 순서는 앞당겼지만
아직 Terms/intended-use review, 압축 해제, adapter 변환과 학습 승인은 아니다.

### Stage 3B — archive·약관·용도 gate

- 세 archive의 exact byte, local SHA-256과 전체 ZIP payload CRC 리포트 보존
- license/terms URL, 확인 날짜, intended use와 재배포 정책 기록
- group 0의 7개 log 이름을 mini DB의 정확한 log record와 join할 수 있는지 확인
- 압축 해제 후 원본과 변환물에 필요한 저장량 재확인

### Stage 3C — 미해제 staging과 adapter smoke

Stage 3B 승인 뒤에 새 staging 경로로 필요한 member만 안전하게 풀고 다음을 확인한다.

확인 항목:

- calibration 기반 8→6 camera 선택
- 10 Hz camera timestamp continuity와 6-camera skew
- route roadblock→polyline/command 변환
- 6.4초 ego future 생성
- log/route/site/day 단위 split
- source log에 속하지 않는 camera member의 fail-closed 거부

가장 작은 이름의 archive도 수십~수백 GB일 수 있으므로, URL 이름만 보고 “작은 subset”이라고
가정하지 않는다. 이후 camera group이나 full split은 object byte size, 필요한 압축 해제 공간과
운영자 승인을 먼저 확인한다.

### Stage 4 — 학습 확대

다음 gate를 모두 통과한 후에만 여러 dataset을 섞는다.

- 원본 license와 intended use 검토 완료
- source별 adapter unit test 통과
- camera/ego/route timestamp report 통과
- calibration overlay 육안 검증 통과
- source별 train/val/test leakage 없음
- dataloader 1-batch와 forward/backward smoke 통과
- GPU와 disk quota 승인
- source별 sampling ratio와 domain label 고정

CARLA와 실제 자료를 섞는 것은 가능하지만, 단순히 모든 파일을 한 폴더에 복사하는 방식은 피한다. sample마다 `domain`, `dataset`, `dataset_version`, `rig_id`, `town_or_site`, `route_id`, `weather_or_day`, `license_id`를 유지해야 domain별 실패를 분석할 수 있다.

## 원본 timestamp 보존 원칙

timestamp는 image와 차량 상태를 정확히 연결하는 핵심 label이다. 다음 규칙은 예외 없이 적용한다.

### 1. 원본 시간을 덮어쓰지 않는다

각 record에 최소한 다음 항목을 남긴다.

```text
source_timestamp_ns
source_rate_hz
camera_timestamp_ns[6]
ego_timestamp_ns
selected_anchor_timestamp_ns
camera_to_anchor_delta_ns[6]
ego_to_anchor_delta_ns
bundle_skew_ns
availability
valid_mask
resample_method
source_dataset / source_version / source_path / source_hash
```

frame 번호를 가짜 nanosecond timestamp로 바꾸거나, resample 후 원본 timestamp 열을 새 시간으로 덮어쓰지 않는다.

### 2. camera image를 보간하지 않는다

camera rate를 줄이는 모든 실험은 미래 frame을 보지 않는 causal/latest 규칙과 원본
timestamp를 보존한다. 그러나 downsampling 가능 여부와 common10 planning 자격은 다르다.
특히 nuScenes의 12 Hz image를 복제·합성 없이 10 Hz로 thinning하면 약 166.7 ms gap이 생겨
현재 p99 150 ms gate를 통과하지 못한다. 측정 후 별도 profile/contract가 고정되기 전에는
schema/adapter smoke에만 사용한다. 4 Hz→10 Hz처럼 늘려야 하는 경우에도 JPG를 반복하거나
합성하여 common10 정식 data로 만들지 않는다.

pose나 trajectory 같은 연속 수치는 필요한 경우 timestamp 기반으로 보간할 수 있지만, 반드시 원본 두 점과 보간 방식, 유효 구간을 기록한다. image와 누락 label을 0으로 채우지 않는다.

### 3. scene 경계를 넘지 않는다

20초 scene의 마지막 anchor에 6.4초 미래가 없으면 다른 scene의 시작을 이어 붙이지 않는다. 부족한 점은 `valid_mask=false`다. common10의 최소 유효률을 못 넘으면 그 anchor는 training에서 제외하거나 dataset-specific evaluation profile로만 사용한다.

### 4. calibration도 provenance다

camera extrinsic은 vehicle/rig/log별로 보존한다. crop, rectification, resize를 했다면 변환 행렬과 최종 intrinsic을 같이 저장한다. “TF만 대충 맞추면 된다”가 아니라 image pixel과 3D ray가 실제로 맞는지 overlay로 확인해야 한다.

### 5. route와 미래 주행을 분리한다

route는 운전자 또는 planner가 anchor 시점에 알 수 있었던 mission 정보다. 미래 ego trajectory를 보고 route를 만들어 input으로 넣으면 정답 누설이 된다. route metadata/HD map으로 route를 만들고, 실제 future ego는 supervision label로만 둔다.

### 6. split을 frame 단위로 무작위 생성하지 않는다

같은 route의 인접 frame이 train과 validation에 나뉘면 거의 같은 장면을 외운 모델이 좋은 점수를 받는다. 먼저 route/site/day 또는 town/scenario를 나눈 뒤 각 split에서 sample을 생성한다.

## 다운로드 이후 남겨야 할 증거

각 dataset마다 아래 파일을 한 폴더에 남겨야 “받았다”가 아니라 “재현 가능하게 준비했다”고 말할 수 있다.

- 원본 download URL과 접근 날짜
- license/terms URL과 확인 날짜
- 원본 archive 이름, byte, SHA-256 manifest
- 압축 해제된 file count와 byte count
- dataset version/schema version
- camera 이름, 해상도, nominal/측정 Hz
- camera별 count와 timestamp 통계
- calibration 요약과 overlay PNG
- ego/trajectory/route availability 통계
- common10 validation JSON/Markdown
- 제외된 sample 수와 정확한 이유
- train/val/test split manifest와 hash
- adapter source commit hash

원본과 변환물은 분리한다.

```text
raw/          # 다운로드 원본, 읽기 전용
staging/      # 압축 해제 또는 임시 구조
common10/     # 변환된 sample
reports/      # 통계, validation, PNG
manifests/    # URL, version, license, SHA-256, split
```

`raw/`를 변환 script가 수정하지 못하도록 운영하고, 변환물을 다시 만들 수 있는 manifest를 남긴다.

## 최종 선택

현재 첫 학습을 곧바로 시작하는 것이 다음 행동은 아니다. Bench2Drive Mini 다운로드와 공식 SHA 검증은 완료됐으므로, 다음은 archive 구조 확인·별도 staging 압축 해제·parser/common10 변환 smoke다.

- **CARLA 시작점**: Bench2Drive Mini — pipeline 검증 전용
- **실세계 빠른 시작점**: nuScenes mini — exact 6-camera와 CAN 연결을 보는
  schema/adapter smoke 전용; 현 planning gate 불통과
- **실세계 planner 연구·adapter 후보**: nuPlan 제한 subset — 10 Hz, 장시간 log, mission route
- **평가 framework 참고**: NAVSIM/OpenScene
- **현재 deployable 학습에서 제외**: Waymo E2E, Argoverse 2

Mini parser와 real-data adapter가 모두 통과한 뒤에도 planning gate를 통과한 source만 정식
common10 학습 corpus에 섞는다. nuScenes mini는 별도 profile/contract 승인 전까지
schema/adapter smoke에 남긴다. 학습 자격을 얻은 CARLA와 실차 sample은 source/domain별
sampling 비율과 평가 결과를 분리해 관리한다.
