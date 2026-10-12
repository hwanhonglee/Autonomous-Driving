# 다음 학습 전 점검 — 정지·회전 장면과 보조 정답의 부족분

<!-- HH_260906 - Report real TRAIN/VAL coverage without treating overlapping anchors, metadata or unknown labels as qualified driving evidence. -->

실행: **2026-09-14 11:55:29 KST**, 로컬 CPU. 판정: `AUDITED_NOT_TRAINING_APPROVED`.
이는 기존 CARLA Common10 v3의 **제한된 coverage 감사**이며 전체 데이터 검증기·물리 품질 판정기·
학습 허가 도구가 아니다. 모델 추론·학습·CARLA 실행·데이터 변경은 하지 않았다.

## 실제 집계

| 항목 | TRAIN | VAL |
| --- | ---: | ---: |
| 시간 앵커 | 1,147 | 337 |
| episode / route-group | 3 / 3 | 1 / 1 |
| 정지 유지 `stationary_hold` | 118 | **0** |
| 이동 → 정지 `moving_to_stop` | 202 | 51 |
| 계속 이동 `continuing_motion` | 611 | 208 |
| 그 외 이동 `other_motion` | 216 | 78 |
| 불완전한 미래 mask | 0 | 0 |
| warmup 앵커 | 105 | 35 |
| 기록된 날씨 종류 | ClearNoon 1종 | ClearNoon 1종 |

표본은 겹치는 미래 6.4초 창이다. **정지 유지 118개는 모두 Town01 우회전 episode 한 개에서
나왔다.** 서로 독립적인 118번의 정지 시험이 아니다. TRAIN은 Town07 직진·C-track 좌회전·
Town01 우회전이고 VAL은 Town03 우회전 한 episode뿐이다. VAL에 별도의 직진·좌회전
episode는 없다. 회전 episode 안에 직선 구간이 없다는 뜻은 아니다.

분류는 기존 정지 행동 검사와 같은 규칙이다. 정지 유지는 현재 속도의 절댓값이 0.1 m/s 이하이고
64개 미래 속도가 모두 0.1 m/s 이하인 경우다. 이동→정지는 현재 전방 속도가 0.1 m/s를 넘고
마지막 10개 미래 속도가 모두 0.1 m/s 이하인 경우다. 계속 이동은 미래 속도가 모두 0.5 m/s
이상인 경우다. 이들은 **정답 기반 진단 분류**이지 모델의 runtime 입력·정지 이유·제어 성공 판정이 아니다.

## METEOR 아이디어를 적용하기 전 필요한 정답

| 정답 종류 | 현재 TRAIN/VAL 상태 | 해석 |
| --- | --- | --- |
| 경로·속도 | 유효한 미래 배열 있음 | 기존 물리 품질·다양성 문제의 해결을 뜻하지 않음 |
| 객체·점유공간 | `available=false` | 객체가 없다는 음성 정답이 아님 |
| 차로·깊이·교통신호 | `NOT_RECORDED` | 현재 정답을 재사용해 보조 loss를 바로 학습할 수 없음 |

기계 판독 보고서는 VAL 정지 유지 부재 1개, VAL 직진/좌회전 독립 episode 부재 2개,
보조 정답 부재 5개를 **8개 gap 항목**으로 기록한다. 이는 모든 데이터 위험을 망라한 수가 아니다.
단일 날씨, 적은 episode 수, 정지 이유 부재, 원본 가감속·저속 곡률 문제도 별도로 해결해야 한다.
기존 실패 수집의 승인 상태와 학습/runtime gate는 바꾸지 않았다.

## TEST와 검증 범위

TEST는 episode 메타데이터의 **1 episode / 선언된 309개**와 hash만 기록했다.
TEST의 표본 JSONL·정답·route·collection config·이미지는 열지 않았다. 보고서의 TEST sample
hash는 선언값이지 이번 실행에서 payload bytes를 검산한 값이 아니다. TEST를 다음 모델의 선택이나
coverage 분석에 사용하지 않는다.

읽은 메타데이터와 TRAIN/VAL 표본 파일 **22개**의 SHA를 실행 끝에 다시 대조했고, 감사기·기존
계약/reader·기존 행동 분류 코드·contract JSON **5개**의 SHA도 전후 비교했다. 영상 bytes와
센서 전체 유효성은 검사하지 않았다. route-group은 TRAIN/VAL 실제 route의 의미 hash를 대조하고,
TEST는 선언된 그룹과의 메타데이터 충돌만 검사했다. 따라서 이 결과만으로 전체 split 독립성을
인증하거나 새 데이터를 승인하지 않는다.

기존 Common10·모델·학습·기존 결과 요약 코드는 수정하지 않았다. 새 감사기는 다음을 검사한다.

- 참조 hash 누락·불일치, 명시적인 admission 거절, 부적절한 경로·symlink.
- 중복 source sample, route-group의 split 충돌, 순서와 미래 배열의 시각.
- 유한 수치, bool 혼입, 연속 prefix mask, 무효 정답의 null 보존.
- 현재 navigation command의 인과성과 freshness. command·시나리오 태그를 실제 회전 성공으로 세지 않음.

새 도구 **47개**와 기존 관련 도구 **96개**, 합계 **143개 CPU 테스트**가 5.04초에 통과했다.
코드 테스트 수는 모델 성능·주행 기능 통과 수가 아니다.

## 재현 방법

기존 패키지가 준비된 로컬 저장소에서 실행한다. 설치 명령이나 SSH 학습 명령이 아니다.
출력 파일은 새 경로여야 하며 데이터셋 내부 출력과 기존 파일 덮어쓰기를 거부한다.

```bash
cd /home/a/autoware_e2e
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 python3 \
  scripts/e2e/audit_portable_data_readiness.py \
  --dataset datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3 \
  --output artifacts/training/2026-09-14/data_readiness_recheck_01.json

CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  tests/test_audit_portable_data_readiness.py \
  tests/test_portable_e2e_dataset_metadata.py \
  tests/test_portable_e2e_contract.py \
  tests/test_audit_portable_stopmix_behavior.py
```

다른 PC에서는 첫 줄을 자신의 저장소 경로로 바꾼다. dataset은 Git 코드와 별도로 해당 버전이
필요하다. `created_at_utc` 때문에 재실행 결과 전체 SHA는 달라지므로 입력/소스 pin과 집계를 대조한다.

## 근거와 다음 단계

- [실제 JSON](data_readiness.json): `e9d419e6f8172d432261828a7eed27a8319a079109c3c1fd78911c4841bb1144`.
- 입력 dataset manifest: `18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c`.
- [감사 코드](../../../../../../tools/e2e/audit_portable_data_readiness.py): `70ef71eaf5865f630568c566ea8d32288abee25fdf0e96714c10e5fa67c6042d`.
- [테스트](../../../../../../learning/tests/test_audit_portable_data_readiness.py): `81c0e7ea4455ccdcfd7b76f1ed9e9a077736445df07c634137a9fefb024960d0`.

다음은 원본 물리 품질을 충족하는 정답 수집을 복구하고, 기존 TRAIN/VAL route-group과 구분되는
정지 유지·직진·좌/우회전 episode 및 날씨를 확보하는 것이다. 새 VAL은 추가 실험 전에 고정하고
기존 개발 VAL은 반복 사용한 사실을 보존한다. 객체·차로·깊이 등은 정답 출처·좌표·시간·mask를
명시한 버전별 계약을 설계한 뒤 편입한다. 없는 정답을 0으로 채우지 않는다.

이후 자체 모델의 공간 구조 변경과 보조 loss를 **하나씩** 비교한다. 검증한 새 데이터로 소수 TRAIN
overfit → 원격 개인 venv/GPU 0에서 3-seed A/B → 로컬 export/10 Hz shadow → 제한된 CARLA
폐루프 순서다. 이 보고서 생성으로 새 모델 학습이나 직접 주행이 시작된 것은 아니다.

[METEOR 검토](../../../../../portable-e2e-meteor-review-2026-09-14.md) · [오늘의 결과 목록](../README.md)
