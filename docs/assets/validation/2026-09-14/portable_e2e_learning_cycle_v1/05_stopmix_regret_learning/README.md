# 05 — STOPMIX 전체 모델: 후보 비용 보조 손실 A/B 학습

<!-- HH_260906 - Continue through real whole-model training while keeping legacy-data research separate from newly qualified driving data. -->

**전체 모델 6회 학습·후속 검사 18회·결과 회수·독립 검산 완료. 운용 모델로는 미채택.**
데이터 검증 → PRO6000 GPU 0 학습 → 결과 검산까지 수행했다. 새 데이터 수집과 병행한
기존 자료 연구이며, 새 CARLA 진단과 기존 미승인 수집은 학습에 넣지 않았다.

<!-- HH_260906 - Record actual launch separately from the prospective declaration and final completion. -->
실행 계획은 06:28:48 UTC에 선언했고, **15:29:30~15:58:47 KST(29분 16초)** 동안 실행했다.
원격 실행기와 SSH 모두 종료 코드 0이며, 총 9,240 optimizer step·36,930회 표본 노출을 확인했다.
각 모델의 1,056,362개 파라미터 전체를 새로 학습했다. 설치나 원격 Autoware 빌드는 없다.
데이터 뷰는 원본과 동일 바이트인 TRAIN/VAL 1,484표본·8,904 JPEG를 포함하며,
전체 8,926파일·865,140,787바이트를 검증·전송했다. 서버 결과 293파일의 전송 전후 해시가 같았고,
체크포인트 6개는 서버에 유지한 채 나머지 287파일·65,797,287바이트를 회수·검증했다.

## 먼저 볼 결과

<!-- HH_260906 - Retain every seed and subgroup regression; execution success is not a performance or driving pass. -->
ADE는 미래 경로의 평균 위치 오차, FDE는 6.4초 끝점 오차다. 낮을수록 좋다.
VAL은 동일한 Town03 우회전 337표본이며, 독립적인 337회 주행을 의미하지 않는다.

| seed | 평균 경로 ADE, A→B (m) | 끝점 FDE, A→B (m) | 속도 MAE, A→B (m/s) | 동일 seed 상대 비교 |
| --- | ---: | ---: | ---: | --- |
| 20260903 | 6.218 → 3.645 | 15.032 → 8.047 | 2.341 → 1.024 | 통과 |
| 20260904 | 5.165 → 5.219 | 12.467 → 11.196 | 1.976 → 1.741 | ADE 악화로 실패 |
| 20260905 | 4.916 → 4.798 | 12.133 → 10.717 | 1.708 → 1.664 | 통과 |

세 쌍 모두 선택 경로 기하 검사 336/337로 동일하다. 그러나 세 seed 전부 개선이라는 조건은
충족하지 못했다. B의 1초 ADE는 모두 0.5 m 이하였지만, **3초 ADE 1 m·6.4초 ADE 2 m·
6.4초 FDE 4 m 한계는 세 seed 모두 초과**했다. 따라서 B가 일관되게 더 낫다거나
실제 운용에 적합하다고 판정하지 않는다. 기존 shadow/운용 모델은 교체하지 않았다.

정지 관련 결과도 함께 확인했다.

- TRAIN 정지 유지 118개는 여섯 모델 모두 STOP 후보·미래 끝속도 0을 118/118 유지했다.
  **VAL 정지 유지 표본은 0개**여서 독립적인 정지 기능 통과 근거가 아니다.
- seed03의 VAL 감속→정지 51개에서는 STOP 선택이 6→46개로 증가하고 ADE 12.200→7.087 m로
  감소했다. 다만 이것도 정지 위치/의도나 실제 운전 성공의 증명은 아니다.
- 같은 seed03의 TRAIN 지속 주행 611개에서는 ADE 0.904→1.264 m, FDE 1.731→2.354 m로 악화했다.
  TRAIN 기타 움직임 216개의 미래 0속도 이후 재가속 지표도 38→115개로 증가했다.
  정지·재출발 의도 정답이 없으므로 이 지표만으로 모든 재가속을 위험 행동으로 단정하지 않는다.
- 모든 seed·TRAIN/VAL·하위 그룹의 개선과 악화는 [전체 비교 JSON](paired_summary.json)에 남겼다.
  A/B의 총 loss는 목적함수가 달라 직접 비교하지 않는다.

## 경로 그림과 검증 자료

예정했던 VAL 12개 인덱스를 전 모델에 동일하게 적용한 **ego 중심 원본 경로 PNG 72장**이다.
영상 입력의 모델 예측이며 Autoware 화면·학습 모델 직접 주행 영상은 아니다.

- [seed03 A/B 경로 24장](galleries/seed_20260903.md)
- [seed04 A/B 경로 24장](galleries/seed_20260904.md)
- [seed05 A/B 경로 24장](galleries/seed_20260905.md)
- [실제 로컬 CARLA 카메라·경로·제동 화면](../06_brake_hold_pilot/README.md): 별도 BasicAgent 파일럿, 학습 미편입.
- [사전 실행 계획](declared_plan.json), [데이터 뷰 검증 요약](dataset_view_summary.json),
  [전송 검증서](transport_receipt.json), [독립 검산](verification.json),
  [539 passed / 1 skipped 코드 검증](code_verification.json).

독립 검산은 학습 이력 9,240행과 저장된 행동 지표 8,904행을 확인했다. 저장된 후보별 scalar와
기하 판정 결과를 재집계한 것이며, 보관되지 않은 전체 XY/속도/logit/정답 배열을 다시 추론하거나
기하 수학을 재실행한 검산은 아니다. CPU/GPU 지표 집계 대조 허용치는 1e-5이고,
선택 분포 entropy 9개 필드의 최대 차이 1.11e-16에는 해당 파생 필드만 1e-15를 허용했다.
이는 판정 기준 완화가 아니며, 나머지 집계 필드·모든 표본 수는 정확히 일치해야 한다.
로컬 exact-resume 테스트 1개는 보안 weights-only 로딩 조건 때문에 건너뛰었다.

원문 보고서·로그·명령·소스·native 평가 PNG까지 포함한 전체 회수본은
`artifacts/training/2026-09-14/stopmix_regret_research_v1/remote_results/`에 있다.
공개 검증서의 파일 목록은 이 전체 회수본을 가리킨다. 이 문서 폴더에는 비교 요약과
고정 경로 갤러리를 제공하며, checkpoint는 아래 서버 경로에 유지한다.

```text
$PORTABLE_E2E_ROOT/runs/diagnostics/stopmix_regret_20260914_v1/
  seed_20260903|20260904|20260905/
    A_original_loss|B_cost_aware/training/checkpoints/latest.pt
```

실제 사용 명령은 회수본의 각 `train_started.json` 등에 보관되어 있다. 실행 후 보관된
`plan.json`은 실행기가 JSON 공백을 정규화한 사본이며, 원본 사전 계획과 내용이 동일함을
검증했다. [declared_plan.json](declared_plan.json)은 정규화 전 원본 바이트다.
완료 폴더에 같은 명령을 다시 실행하면 덮어쓰기/재개를 거부한다.

## 무엇을 비교하는가

동일한 자체 Portable E2E STOPMIX 전체 모델(주행 6개 + 정지 6개 후보)에 대해 다음을 비교한다.

| 항목 | A 기존 손실 | B 후보 비용 보조 손실 |
| --- | --- | --- |
| 모델·초기화·데이터·학습 순서 | 동일 | 동일 |
| 기존 회귀·후보 정답 분류 손실 | 유지 | 유지 |
| candidate_regret_weight | 0 | 0.1 |
| candidate_score_weight | 0.1 | 0.1 |
| 학습 | seed 20260903/04/05, 각 1,540 step, batch 4, LR 0.0001 | 동일 |

기존 후보 오차가 큰 선택에 더 큰 손실을 주면 DRIVE/STOP 선택이 나아지는지가 가설이다.
9월 9일에는 6개 DRIVE 모델의 보조 손실 실험과 12개 STOPMIX의 기본 손실 실험을 따로 했지만,
이 조합의 동일 모델 A/B는 이번에 새로 수행했다. 이전 선택기만 이어학습한 결과와도 구분한다.
이번은 **모든 모델 parameter를 처음부터 학습한 6회 fresh fit**이며 과거 결과를 A로 대체하지 않았다.

`20260903/04/05`는 난수 초기화 번호(seed)이며 실행 날짜가 아니다. 여섯 회 모두 9월 14일에
새로 실행했다. 각 회의 1,540 step은 6,155표본 노출(전체 TRAIN 약 5.37회)에 해당한다.
12개 후보는 기능 12개가 아니라 동일 장면에서 선택할 주행 6개·정지 6개 경로다.

## 데이터 범위

기존 v3의 TRAIN 1,147개·VAL 337개를 그대로 복사한 별도 연구용 데이터 뷰를 만들었다.
episode·split·정답·타임스탬프·원본 이미지 바이트는 변경하지 않으며 TEST는 포함하지 않는다.
기존 기본 검증기가 전체 manifest의 모든 episode를 검사하므로, TEST payload가 있는 원본 v3에
직접 일반 학습 명령을 실행하지 않는다. 새 뷰 생성 시 TEST는 split을 구분하는 episode 메타데이터
외의 sample/route/provenance/image를 열지 않는다. 새 뷰에서 native validator를 그대로 실행한다.

원본 승인 거부 표시를 지우지 않고, 이미 거부된 수집을 재포장하지 않는다. 새 뷰는 **새 품질 승인이나
독립 데이터 수집이 아니다.** TRAIN 정지 유지 118개는 Town01 한 episode, VAL 정지 유지는 0개이며,
기존 급감속 정답·warmup·작은 데이터·단일 개발 VAL의 한계를 그대로 유지한다. 뷰 fingerprint가
바뀌었지만 TRAIN/VAL의 tensor fingerprint는 원본과 같다. 과거 checkpoint의 exact resume으로 표시하지 않는다.

## 실행 제한과 판정

- PRO6000 physical GPU 0의 기존 UUID만 노출하고 개인 hwanhong venv만 사용한다.
- 설치·시스템/Conda·GPU 1·다른 작업 종료·원격 Autoware 빌드는 하지 않는다.
- 실행 전 계획·코드·데이터 SHA를 고정하고 모든 seed의 final checkpoint와 전체 결과를 남긴다.
- 각 학습 뒤 VAL 경로/속도·고정 runtime geometry·TRAIN/VAL 정지 subgroup·재가속을 확인한다.
- 실패 seed를 빼거나 가장 좋은 epoch를 고르지 않으며 TEST는 모델 추론·학습·선택에 사용하지 않는다.
- 모델 자동 채택·운용 bundle 변경·차량 제어·학습 기능 승인은 없다.

새로운 자연 감속·제동 유지·재출발 수집 프로파일과 독립 TRAIN/VAL 확보는 별도 데이터 개선 작업으로
남아 있다. 그 작업이 완료됐다고 바꿔 쓰거나 이번 기존 자료 연구를 새 데이터 학습으로 설명하지 않는다.

## 무엇이 바뀌고, 무엇은 아직 아닌가

<!-- HH_260906 - Separate actual optimizer work, held-out development evaluation, and vehicle operation. -->
입력은 6카메라·보정값·ego 이력·예정 경로이고 출력은 미래 6.4초의 후보 경로·속도·점수다.
CARLA expert의 기록을 정답으로 사용한 지도 모방학습이다. VAD나 METEOR의 가중치를 내려받아
교체하는 작업, 실제 차량 데이터와의 혼합 학습, 모델이 직접 운전하는 강화학습은 아니다.
평가 PNG는 저장된 VAL 입력의 예측 경로 비교이며 Autoware 실행 화면이나 실제 모델 주행이 아니다.

학습이 정상 종료돼도 성능 기준·데이터 범위·폐루프 시험은 별개다. 상위 9영역·하위 30개 요구사항의
직접 모델 주행 통과 수를 학습 횟수만큼 올리지 않는다. 현 운용/shadow 체크포인트는 그대로 유지한다.

## 다음 큰 단계

구간 삭제·라벨 보정 없이 수집 자료의 준비/출발/저속 곡률 문제를 먼저 해결하고,
독립 TRAIN/VAL의 정지 유지·자연 감속·재출발·회전 범위를 확보한다. 정지/재출발의 원인을
구분할 입력·정답도 함께 준비한다. 그 다음 새 데이터 버전을 고정하고 이번 보조 손실의
주행 오차/재가속 부작용까지 포함해 전체 모델 A/B를 재검증한다.
성능·export 동등성·10 Hz shadow·안전 전제 검사를 통과한 뒤에야 제한된 CARLA 모델 직접 제어로 간다.

[9월 14일 전체 목차](../README.md) · [직전 제동 진단](../04_coast_brake_transition/README.md)
