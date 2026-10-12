# Town02 추가 자료의 도로·차선 일반화 A/B — 실행 전 계약

<!-- HH_260929 - Preregister a diagnostic data-composition experiment without claiming sign or HD-map approval. -->

**목적:** 고정 RGB 의미분할 `MapPerception`의 Town05_Opt 도로·차선 성능을 새 TRAIN 장소로 개선할 수 있는지 확인한다. Town02_Opt p02 원본은 앞선 *1–4픽셀 소형 표지판* 사전 기준에서 0/232장으로 실패했다. 그 실패를 취소하거나 표지판 성능 개선 근거로 사용하지 않는다. 이번 A/B는 별도의 도로·차선 탐색 실험이며, 결과가 좋아도 모델 승격·자동 정밀지도 완성·실차 승인으로 세지 않는다.

## 입력과 분할

- 공통 D0 TRAIN: AI v2의 고정 Town01 497장. 공통 Town01 DEV 118장은 학습·샘플러·하이퍼파라미터 선택에 넣지 않는다. 원본 manifest/배열 SHA와 순서가 바뀌면 중단한다.
- D1 TRAIN: 동일한 Town01 497장 뒤에 [Town02_Opt p02 소유 원본](../../2026-09-27/map_town02_p02_train_scout/README.md)의 0.25m 위치 중복 제거 232장을 추가한다. Town02 프레임의 RGB 640×360과 CARLA semantic 태그를 전수 검사하고, owner·route·센서 해시 및 서로 다른 Town을 명시한다. 기존 소형 표지판 거부 판정은 보존한다.
- 평가는 Town01 DEV 118장과 [Town05_Opt 직진 DEV](../../2026-09-27/map_town05_dev_capture/README.md)의 보존 232장에만 한다. Town03 B/C TRAIN, Town07/04/06/10HD/15 TEST 또는 Town13 결함 영상은 사용하지 않는다. Town05는 이전 후보 분석에 이미 사용됐으므로 새 맹검 TEST가 아니다.

## 동일 조건과 판정

- 두 팔 모두 같은 새 훈련기·`MapPerception` 7클래스·640×360·scratch 초기화·seed `20260922`·AdamW `3e-4`, weight decay `1e-4`·batch 8·2,000 step·TRAIN-only 동일 crop을 사용한다. 클래스 `sqrt_frequency` 가중치는 **D0 Town01 TRAIN에서 한 번 계산해 두 팔에 동일하게 고정**한다. D0/D1 사이에서 바뀌는 것은 TRAIN 프레임 출처 구성뿐이다.
- D0의 Town01 DEV **7개 클래스 IoU가 보관 AI v2와 각각 절대 0.5pp 이내**로 재현되지 않으면 D1을 시작하지 않고 코드·환경 차이를 조사한다. D1의 개발 목표는 Town05에서 D0 대비 road/lane 각각 +5pp 이상이면서 절대 road ≥80%, lane ≥50%이고, Town01에서 보관 AI v2 기준 road ≥97.751%, lane ≥89.233%를 보존하는 것이다. 다른 5개 클래스의 혼동행렬과 IoU를 모두 보고하며 **한 클래스라도 퇴행하면 7클래스 모델 승격 없음**으로 남긴다.
- 이 실험은 1 seed 진단이다. 통과해도 별도 site/날씨의 미사용 검증과 최소 3 seed 재현, 온디바이스 지연 및 기하 누적 지도가 없으므로 배포하지 않는다. 실패하면 같은 DEV 결과를 보며 기준이나 데이터를 사후 수정하지 않는다.

## 자원·중단 조건

로컬에서 원본 전수 감사와 fresh dataset/코드 해시를 먼저 완료한다. 그 뒤에만 `/home/pro6000/personal/hwanhong` 개인 영역으로 복사하고 기존 개인 `venvs/py312`에서 물리 GPU 0만 순차 실행한다. GPU 0이 점유 중이거나 사전 검사가 하나라도 실패하면 학습을 시작하지 않는다. GPU 1·공용 패키지·다른 사용자 프로세스에는 접근하거나 설치하지 않는다. 이전 같은 계열의 2,000-step 학습은 팔당 약 75–77초였으나, 새 A/B의 준비·전송·평가 시간은 별도다.

**현재 상태:** 이 문서 작성 시점은 사전 등록이며, 새 D0/D1 학습·모델 성능 결과는 아직 없다.
