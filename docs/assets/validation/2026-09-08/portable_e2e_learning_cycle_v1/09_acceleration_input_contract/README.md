# 09 — 가속도 입력 기준점 진단과 연구용 입력 제외 모델

<!-- HH_260906 - Separate an unchanged-data diagnostic, an untrained research ablation and unsupported runtime deployment. -->

기존 데이터의 **위치·횡속도와 가속도가 같은 기준점을 사용하지 않는 문제**를 확인했습니다. 이를 수집기에서만 고치면 실시간 입력과 또 다른 불일치가 생길 수 있어, 데이터나 런타임을 바꾸지 않았습니다. 대신 가속도 두 숫자만 내부적으로 제외하는 **별도 연구 모델 코드**를 추가했고 합성 입력 단위 테스트를 완료했습니다. 이 새 ID의 실제 데이터 학습·주행·성능 개선은 아직 확인하지 않았습니다.

## 1. 기존 데이터에서 확인한 사실

`base_link_state`는 가정한 CARLA actor 기준점에서 위치를 뒤로 `wheelbase/2`만큼 옮기고 `vy`에서 `L × yaw_rate`를 뺍니다. 그러나 `ax/ay`는 입력 가속도를 회전만 해 저장합니다. Common10 변환기는 이 값을 그대로 복사하고, 학습 모델의 13개 ego feature 중 인덱스 **3, 4**로 사용합니다. 따라서 입력 정의의 불일치이며, XY/속도 정답 자체를 바꿔야 한다는 결론은 아닙니다.

조사한 함수의 바이트는 해당 데이터 변환의 소스 commit `081a71f7fc014d2864b790b0b0cae7378ae18e4c`와 동일했습니다. 원본 21개 파일을 SHA로 연결하고 train 3개 episode·val 1개 episode만 읽었습니다. 데이터셋 상위 목록에 test 참조는 있으나 test episode의 metadata·상태·sample·이미지·예측은 열지 않았으며 전체 corpus validator도 호출하지 않았습니다.

## 2. 평면 강체 가정의 조건부 보정량

평면 ROS 좌표계에서 같은 원점의 속도/가속도를 받는다고 **가정**하고, 뒤쪽 기준점 offset을 `r=(−L,0,0)`, `L=1.425 m`로 놓으면:

```text
a_rear = a_source + alpha × r + omega × (omega × r)
delta_ax = +L × yaw_rate²
delta_ay = −L × yaw_acceleration
```

`yaw_acceleration`은 실제 원시 20 Hz의 현재·직전 yaw rate와 **실제 timestamp 차이**로 계산했습니다. 미래 표본은 사용하지 않았습니다. 각 episode 첫 표본은 이전 기록이 없어 계산 불가로 남겼습니다. 전체 **train 1,147 / val 337** 중 평가 가능 수는 **1,144 / 336**이며, 첫 표본을 정상 가속도 0으로 꾸미지 않았습니다.

| 구분 | 조건부 보정 벡터 크기 RMS / 최대 (m/s²) | 기존 가속도와 rear 속도 차분의 잔차 RMS | 조건부 보정 후 잔차 RMS |
|---|---:|---:|---:|
| train, 1,144/1,147 | 2.210797 / 9.180823 | 2.209539 | 0.039934 |
| val, 336/337 | 0.668965 / 5.299538 | 0.666413 | 0.036717 |

![기존 train/val 가속도 기준점의 조건부 보정량과 내부 일관성 잔차](acceleration_reference_rms.png)

이 잔차는 **모델 예측 오차가 아닙니다.** 뒤쪽 기준점 속도를 world 좌표로 옮겨 차분한 내부 비교값입니다. 가정한 원점의 속도를 역변환해 차분하면 기록된 가속도와 RMS 약 `4.84e−8 / 3.12e−8 m/s²`로 일치합니다. 이는 코드 내부 기준점 불일치를 강하게 뒷받침하지만, 정답 물리 가속도를 독립 계측했다는 뜻은 아닙니다.

원래 로그에는 actor 중심의 전체 raw pose/pitch/roll, world velocity/acceleration이 없습니다. 위치 변환은 pitch를 쓰지만 동역학은 yaw만 사용합니다. 따라서 **실제 CARLA 물리 원점/질량중심, 완전한 3D 가속도 변환은 검증되지 않았습니다.** 로컬 CARLA 소스의 world 속도 차분 구현도 실행된 dirty server의 정확한 build 증명은 아닙니다. 자세한 분모·phase·최대값 사례·소스 해시는 [원본 감사 JSON](acceleration_reference_audit.json)에 있습니다.

## 3. 실시간 입력도 정의를 맞춰야 한다

로컬 full launch 소스에서는 `/localization/acceleration`을 `twist2accel`이 만들고 portable shadow가 이를 직접 사용합니다. 해당 estimator는 **body 축 속도 성분의 차분을 lowpass gain 0.5로 필터링**합니다. 단순한 body 속도 차분은 `omega × v` 항을 포함하는 관성 가속도를 body 축으로 표현한 것과 다릅니다. 같은 `base_link` 이름만으로 의미가 같아지지 않습니다.

이는 소스 설정 검토이며 현재 live ROS graph를 계측한 결과는 아닙니다. 별도의 VAD IMU adapter를 portable의 기본 가속도 입력으로 혼동하지 않았습니다. [런타임 소스 기록](runtime_boundary_source_evidence.json)과 [CARLA 후보 소스 기록](local_carla_source_evidence.json)을 함께 보관합니다.

수집기의 가속도만 바꾸는 방안은 채택하지 않았습니다. 향후 일관된 가속도 feature를 만들려면 학습·런타임 **동일한** 10 Hz causal ego timeline, 실제 dt, 첫 history·중단·초기화 규칙, 좌표 정의, 필터 규칙을 새 policy/모델/자료 식별자에 고정해야 합니다. 기존 데이터와 checkpoint는 유지해야 합니다.

## 4. 구현된 것은 ‘가속도 값 제외’ 연구 모델뿐

새 ID는 `portable_e2e.perspective_trajectory.physical_no_accel.v1`입니다. 공유 `forward`에서 raw 유한성 검사를 먼저 한 뒤, 복사한 ego history의 인덱스 3, 4를 **모든 시점에서** 0으로 만듭니다. 나머지 13개 입력 구조, 954,590개 파라미터, 같은 seed 초기 가중치, physical decoder·loss·gate는 그대로입니다. 이는 잘못될 수 있는 가속도 수치에 대한 의존성을 비교하는 실험이지, 올바른 가속도를 복원한 모델이 아닙니다.

**현재 secure runtime bundle 허용 목록에는 새 ID가 없습니다. 내보내기·현재 ROS 런타임 배포가 지원되지 않습니다.** 또한 가속도 메시지 구독·유한성·stale/동기화 검사도 제거하지 않았습니다. 숫자 영향만 제외하는 것과 센서/topic 없이 실행할 수 있는 것은 다릅니다.

새 모델 33개 및 기존 candidate-rank 22개 합성 CPU 테스트가 통과했습니다. 관련 모델·bundle·CLI 확장 회귀는 **131 passed, 5 skipped**였고, skip은 기존 로컬 PyTorch의 secure checkpoint load 기능 제한에 따른 것입니다. 기존 세 ID의 이전 forward 대비 bit-exact 결과, 동일 파라미터/decoder, 모든 history의 가속도 입력 불변성·해당 gradient 0·다른 gradient 유지, NaN/Inf 거부, ID가 다른 checkpoint resume 및 bundle 경계를 확인했습니다.

구현과 제한은 [연구 모델 가이드](../../../../../portable-e2e-no-acceleration-ablation.md), 코드/설정/테스트의 정확한 SHA와 미학습 상태는 [연구 모델 설계 기록](ablation_design.json)에 있습니다. 아직 해당 새 모델로 실제 데이터 학습을 시작하지 않았고, 기존 C/D/E 실행을 새 실험으로 재분류하지 않았습니다.

## 출처와 다음 단계

원본 감사 JSON은 바이트 그대로 복사했습니다. 원본 해시는 `efa34fc56d6de603d8e72c08ff993a6a6ece7207d363ecbb4410f2712814ba34`이며, 수행한 비공개 감사 스크립트의 SHA도 JSON과 [provenance.json](provenance.json)에 유지했습니다. 계정 절대 경로·비밀 값은 공개하지 않으며 대량 원시 상태/이미지를 중복 복사하지 않았습니다. PNG는 원래 감사 수치로 그린 그래프이지 주행 화면·GIF·카메라 자료가 아닙니다.

우선 수집기의 경로·속도·정지 품질을 개선합니다. 이후 **같은 새 검증된 corpus에서** 기존 physical-v1 대 새 ID를 같은 3개 seed와 optimizer budget/loss/gate로 비교하는 계획을 먼저 고정해야 합니다. 이 자료는 새 학습 실행·test 설정 선택·모델 승격·주행 승인이나 물리 한계 완화를 의미하지 않습니다.

[체크섬 목록](SHA256SUMS) · [전체 묶음으로 돌아가기](../README.md)
