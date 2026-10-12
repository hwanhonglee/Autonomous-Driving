# 새 촬영 없는 지도 독립 DEV/TEST 원본 재고 조사

<!-- HH_261001 - Inventory only local and personal-remote dataset metadata without unsealing reserved routes. -->

**판정: 지도 의미분할에 바로 사용할, 과거 미노출·품질 합격·분할 고정된 독립 경로 2개를 찾지 못했다(0/2).** 따라서 [차선 후보 학습 관문](README.md)의 GPU fit NO-GO를 유지한다. 이 문서는 파일명·manifest·분할·과거 노출 기록의 읽기 전용 조사다. 새 수집·모델 추론·GPU·자료 복제·설치·역할 변경은 없었다. 파일 시스템 밖에 다른 자료가 *없다*는 전역 명제는 아니다.

## 실제 검사 범위

| 위치 | 읽은 범위 / 분류 |
| --- | --- |
| 로컬 `storage/datasets` | 하위 파일명·깊이2 폴더·map manifest·prepared `episode.json`의 역할. 전체 약4.0GB; `prepared/map_native_20260926_ah_v1`, `map_expanded_20260926_ai_v1/v2`의 map NPZ는 **6개**(train/dev 각3). CARLA common10 prepared의 episode metadata는 **15개 항목이나 중복 포장된 물리 episode 5종**이다. 예약 TEST 영상 픽셀은 열지 않았다. |
| 원격 `/home/pro6000/personal/hwanhong/dataset` | 개인 경로만 SSH read-only `find/stat` 및 map manifest/기존 Bench2Drive 구조 감사 영수증. NPZ 총**870**개: Oct1 구동 exact-input 백업366, 이미 학습한 p02/p04 원본480, map package/cache24. 원격 공용 사용자 경로는 조사하지 않았다. |
| 근거 문서 | [기존 새 DEV 적격성 감사](../../2026-09-29/map_new_dev_feasibility/README.md), [통합 분할 계획](../../2026-09-27/map_next_training_design/README.md), [현재 지도 TRAIN/평가 노출](../map_unused_route_inventory/README.md). 이미 본 Town을 새 TEST로 다시 부르지 않았다. |

## 후보별 판정

| 후보 | 현재 raw/라벨·역할 | 새 지도 독립 DEV/TEST 여부 |
| --- | --- | --- |
| 로컬·원격 `map_native`, `map_expanded v1/v2` | Town01 TRAIN114→301→497의 누적 사본; 세 `dev.npz`는 같은 Town01 DEV118 SHA `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3` | **아니오.** 별도 세 경로가 아닌 기존 누적/반복 DEV. |
| 원격 `map_generalization_20260927_v1` | TRAIN913에 Town03가 포함되고 DEV118은 같은 Town01; manifest가 `Town07 sealed` 명시 | **아니오.** Town03는 과거 모델 C 학습/주행 DEV와 충돌; Town07 봉인 원본을 열지 않는다. |
| 원격 `map_road_lane_ab`, `map_town02_p00_route_holdout`, `map_appearance_matched`, `map_route_geometry_matched` | 기존 Town01/Town02 TRAIN, Town05 DEV232, Town02 p02 holdout232, p01/p00 TRUE480 및 이미 학습한 p02/p04 FALSE480 | **아니오.** 후보 학습·점수 비교에 이미 노출; 뒤에 포장된 동일 원본은 새로운 장소가 아니다. p02/p04 새 OFF/ON 캡처는 미실행. |
| 로컬·원격 CARLA `common10`/exact backup/legacy | 주행용 다중 카메라 JPEG·상태·경로; 로컬 prepared15 metadata 항목의 split은 Town07 train4, CTrack train4, Town03 val4, Town01 train2, **Town04 test1**. 원격 exact backup366 NPZ는 이미 수행한 Town07 구동 입력 | **아니오.** 현재 지도 모델의 같은 틱 CARLA semantic 픽셀 정답/rig QA가 없고, Town04는 예약 TEST. CTrack/Town03는 노출·역할 충돌. 단순 RGB를 oracle label로 간주하지 않는다. |
| 원격 Bench2Drive mini tar.gz 10개 | 개인 `raw/bench2drive/legacy-mini-10-research-only/archives`; 보관 구조 감사는 RGB surround/top-down과 annotation을 기록하지만 **지도 7클래스 픽셀 semantic GT 또는 해당 지도 rig/depth ABI를 제공한다는 영수증은 없다**. 기존 conversion readiness `BLOCKED_FAIL_CLOSED`: source timestamp10Hz 근거 없음, 비표준 JSON 숫자 정책 미정, prepared validation 미실행. 폴더도 terms review 보류 역할 | **현재는 아니오.** Town04/Town10HD/Town15 등 예약 장소 이름이 archive에 있다고 지도 분할을 변경하지 않는다. 추출·변환·TEST 픽셀 열람 없음. |
| Town13, 기타 Town | 이전 Town13 240장은 전량 skydome 경고, 재촬영 시도 충돌. Town04/06/07/10HD/15는 보호/예약; Town05·Town03는 과거 노출 | **아니오.** 이름만 다른 영상/과거 실패를 품질 합격 두 경로로 세지 않는다. |

로컬 common10의 실제 Town04 episode metadata는 `split=test`, 선별 카메라 bundle309개지만 지도 semantic 정답 파일은 없었다. 이 309장을 새 DEV로 열지 않았다. 원격 개인 `raw/carla`는 빈 폴더였다. 원격 map package24 NPZ의 소스 역할은 위 manifest와 일치하며, 깊은 raw NPZ480은 기존 p02/p04 geometry 학습 원본이었다. `find`로 검사한 파일 형식 외에 tar 내부 임의 숨은 모달리티를 전수 증명한 것은 아니므로 Bench2Drive에 대해서는 **현재 자격 불충분**이라고만 판정한다.

원격의 `map_native`, `map_expanded_v1`, `map_expanded_v2`, `map_generalization` **네 `dev.npz` 실제 bytes SHA-256도 모두** `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3`로 일치했다. 이름이 다른 네 DEV가 아니라 정확히 같은 Town01 DEV118 원본이다.

## 다음 관문

새 촬영을 생략할 수 있다는 근거는 없다. 먼저 주행·지도 통합 분할 원장으로 **미사용 두 경로의 site/route/렌더 조건을 봉인**하고, 원본 같은 틱 RGB/semantic/depth/pose, owner/lifecycle, 해시·전체 픽셀·시각 QA를 확보해야 한다. 기존 예약 TEST의 해제나 Bench2Drive 새 라벨 제작은 별도 프로젝트 판단이 필요하다. 두 경로가 생겨도 모델을 학습하기 전에 evaluation script·지표·한 번의 열람 규칙을 고정해야 한다.
