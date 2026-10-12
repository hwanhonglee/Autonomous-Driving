# 지도 모델 C: Town01 복습 7 : Town03 신규 1, 학습 전 계약

<!-- HH_260906 - Predeclare one TRAIN-only sampling change; never tune on the sealed Town07 holdout. -->

**상태: 코드·합성 테스트 완료, 원격 staging·학습 미실행.** Town03 416장을 더한 지도 후보 B는 기존 Town01 DEV에서 큰 퇴행을 보였으므로 **채택하지 않는다. Town07 평가 원본은 봉인 상태**다. 다음 후보 C에서는 B와 완전히 같은 913장 TRAIN/118장 DEV·7-class RGB 모델·640×360 해상도·초기화(seed 20260922)·2,000 step·batch 8·optimizer·TRAIN crop·전체 TRAIN에서 계산한 class weights를 유지하고, **TRAIN 배치의 Town 노출 비율만** 바꾼다. Head 추가, 라벨 변경, DEV 재배치, 데이터 추가, 사후 임계값 조정은 없다.

| 반복 확인한 Town01 DEV 원본 640×360 | AI v2 부모 | Town03 추가 후 B | 기존 보호 기준 |
|---|---:|---:|---|
| 도로 IoU | 98.751% | 98.565% | 부모 대비 -1%p 이내 |
| 차선 IoU | 91.233% | **77.089%** | 부모 대비 -2%p 이내 |
| 표지판 IoU | 3.157% | **0.856%** | 부모보다 낮지 않을 것 |
| 표지판 정밀도 | 3.293% | **0.886%** | 부모보다 낮지 않을 것 |
| 7-class 평균 IoU | 60.121% | **55.967%** | 부모 대비 -2%p 이내 |

이 DEV는 반복 확인한 같은 Town01 자료이며 독립 TEST가 아니다. 이 실패가 추가 데이터의 양 자체 때문이라고 확정하지 않는다. 모델 구조·loss 가중치·도메인 외관·한 seed의 영향이 함께 있을 수 있다. 다만 B는 매 장을 균등 추출하므로 배치 한 장 기준 Town01 노출 **497/913=54.44%**, Town03 **416/913=45.56%**이다. Town03은 차선 라벨 약 **2,546,611픽셀**을 보탰지만 표지판 라벨은 **140픽셀**만 보탰다. 전체 913장으로 다시 계산한 lane loss 가중치는 AI v2의 **1.462→0.802**, sign 가중치는 **2.172→2.651**이다. B의 실패를 단순히 “학습 step이 적어서”라고 해석할 수 없다.

| 한 장 기준 TRAIN 기대 라벨 픽셀, crop 전 | Town01만 | B 균등 913장 | C 7:1 배치 |
|---|---:|---:|---:|
| 차선 | 578.8 | 3,104.3 | 1,271.7 |
| 표지판 | 136.4 | 74.4 | 119.4 |

계산은 고정 class pixel 합계를 각 Town 프레임 수로 나눈 뒤 샘플링 확률을 적용한 **crop 전 기대값**이다. C는 동일한 913장 loss weight를 유지하면서 Town01의 희소 표지판 노출을 B보다 높이고, Town03의 촘촘한 차선 편향을 낮춘다. B 가중치를 곱한 기대 차선/표지판 기여는 대략 B **2,488/197**, C **1,019/317**로 바뀐다. 기존 AI v2의 약 **846/296**에 가깝게 하려는 한 요인 시험이지 성능 회복을 보장하는 계산은 아니다. 실제 crop이 표지판을 잘라낼 수 있고 Town03 노출이 B의 45.56%에서 12.5%로 줄어 교차-Town 일반화가 약해질 수도 있다.

## 고정 입력·단일 변경

- B와 정확히 같은 `storage/experiments/training/2026-09-27/map_generalization_town03_train_package_v1/`을 사용한다. Manifest SHA256 `e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d`, `train.npz` SHA256 `1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440`, 고정 DEV `dev.npz` SHA256 `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3`. 부모 AI v2 checkpoint SHA256 `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380`.
- B의 trainer는 수정하지 않고 별도 `tools/e2e/train_map_town_rehearsal_c.py`를 사용한다. 매 batch는 기존 Town01 index `[0,497)`에서 7장, 추가 Town03 index `[497,913)`에서 1장을 **TRAIN 전용 RNG seed 20260922로 복원 가능하게 중복 허용 추출한 뒤 permutation**한다. 2,000 step 동안 기대 추출은 Town01 14,000회, Town03 2,000회다. DEV/TEST 라벨·예측은 샘플러 입력이 아니다.
- 새 trainer는 GPU 획득 **전** 같은 manifest/배열 SHA, parent AI v2, B의 `started.json`·`loss_contract.json`·archived trainer 및 4개 지원 소스 SHA를 검사한다. B와 class pixel counts·class weights가 정확히 같아야 한다. 새 체크포인트는 scratch 초기화로 하나만 만들며 B 가중치를 이어 학습하지 않는다.

## 평가·중단 조건

1. 개인 venv에서 CPU `--validate-only`와 합성 단위 검사에 통과해야 한다. 원격 작업은 `/home/pro6000/personal/hwanhong/portable_e2e` 하위 새 source/run/log/receipt에만 남긴다. **물리 GPU 0 고정 UUID만** 사용하고, 사용 중이면 중단한다. GPU 1·다른 작업·시스템 패키지를 건드리거나 설치하지 않는다.
2. 후보 C의 학습 종료 후 **같은 Town01 DEV**에서 부모 AI v2 대비 도로 IoU 하락 ≤1%p, 차선 ≤2%p, 7-class mIoU ≤2%p, 표지판 IoU·정밀도 비퇴행을 **모두** 충족해야 한다. 이 기준은 기존 [교차-Town 계획](../map_next_factor/PLAN.md)의 DEV 보호 관문과 같다. B와 C의 DEV 비교도 함께 보관하지만, B를 기준으로 임계값을 느슨하게 바꾸지 않는다.
3. 하나라도 실패하면 C를 탈락시키고 **Town07를 열지 않는다.** C가 DEV 보호를 통과하더라도 곧바로 배포하지 않는다. 먼저 평가기의 후보 trainer 소스 inventory를 C 전용으로 변경·합성 테스트·SHA 재동결하고, 그 후에만 사전 선언된 Town07 단회 상대/절대 관문을 적용한다. 점수를 보고 Town07 경로나 관문을 바꾸지 않는다.

예상 GPU 학습 자체는 B의 실측 **75.62초**와 같은 2,000 step 규모지만, 전송·CPU 검사·GPU 대기·결과 회수 시간은 별도이며 총시간/토큰 비용은 확정할 수 없다. 1회 미통과 시 자동 반복하지 않는다. 지도 분할 개선이 확인돼도 정밀지도 완성, 10Hz 전체 파이프라인, 실차 또는 자율주행 기능 승인은 아니다.

로컬 합성·계약 회귀 검증: `test_train_map_town_rehearsal_c.py`, `test_run_map_town_rehearsal_20260927.py`, 기존 map expansion/class-weight/B runner 테스트를 함께 실행해 **57 passed**. 이는 Town07의 이미지를 열거나 모델 성능을 재측정한 결과가 아니다.

## 원격 staging 및 검증 명령 초안 — 실행 전 해시 재확인

아래는 **명령 예시**이며 이 계획을 작성하면서 실행하지 않았다. 비밀번호·비밀키 내용을 명령에 쓰지 않는다. 사용자 전용 경로 외로 설치하거나 복사하지 않는다. 고정 C trainer SHA256은 `e1808cb77df8bc7c28d5019eb46c2d40d7e756d2f45ce59d01b43d944ab04de2`, one-shot runner SHA256은 `444a4359a3b6419d03af4c8a4daad8f2e4a22b42f3bcb2249c92eca0f0b35419`이다. 전송 직전 로컬과 원격 파일 해시가 일치하는지 확인한다. runner는 원격 개인 venv, 패키지/부모/B 체크포인트·소스 해시, 새 output/log/receipt를 CPU 사전검사에서 확인한다.

```bash
ssh -p 6000 -i /home/a/.ssh/pro6000_ed25519_v2 pro6000@220.90.5.14 'mkdir -p /home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_town_rehearsal_20260927_v1'
rsync -av -e 'ssh -p 6000 -i /home/a/.ssh/pro6000_ed25519_v2' -- tools/e2e/train_map_town_rehearsal_c.py tools/e2e/map_expansion_contract.py learning/portable_e2e/map_perception.py learning/portable_e2e/map_learning_balance.py tools/e2e/dual_recovery_objectives.py tools/e2e/run_map_town_rehearsal_20260927.py pro6000@220.90.5.14:/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_town_rehearsal_20260927_v1/
ssh -p 6000 -i /home/a/.ssh/pro6000_ed25519_v2 pro6000@220.90.5.14 'cd /home/pro6000/personal/hwanhong/portable_e2e && /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python source_worktrees/map_town_rehearsal_20260927_v1/run_map_town_rehearsal_20260927.py'
```

실제 실행은 한 번만 하고 먼저 remote source/data/parent/B SHA를 runner의 CPU `--validate-only`로 확인한다. 이 단계의 `CUDA_VISIBLE_DEVICES`는 빈 값이다. 그 뒤 runner가 고정 UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`의 **물리 GPU0** 메모리 사용량·utilization 모두 0인지 확인한다. 사용 중이면 로그/종료 기록을 남기고 학습하지 않는다. 승인/재시도 자동화는 없다.
