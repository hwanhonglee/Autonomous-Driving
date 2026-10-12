# Town07 제동 비교 첫 버전의 실행 오류 기록

<!-- HH_261001 - Retain the failed handoff attempt and never report infrastructure failure as model performance. -->

**첫 A는 비교 코드의 자료형 검사 오류로 중단됐다. 모델 주행 성능을 판단할 수 없으며 B는 실행하지 않는다.** 실패 파일을 덮어쓰지 않고, 실제 반환 자료형까지 검증하는 별도 v2에서 수정 후 다시 선언한다.

## 실제로 실행한 범위

CARLA는 기동했고 준비·전문가 출발 동안 상태35개와6카메라 묶음18개를 남겼다. 모델 제어로 전환하려는 시점에 `returned research control differs from the scoped tracker` 오류가 발생했다. 모델 제어가 ACK된 명령은0개다. 정책 내부의 계산 시도와 성공적으로 기록된 추론은 구분해야 하므로, 이 종료 요약만으로 모델 forward 호출도0회였다고 말하지 않는다.

소유 프로세스 종료와 포트 해제는 통과했고, 종료 시 소스 일치도 true다. 실행 후 별도의 읽기 전용 대조에서도 보관된 로컬 소스79개와 현재 파일이 모두 일치했다. 원본 실패 파일과 명령 영수증은 보존했다.

## 왜 실패했나

기존 제어기는 추종할 좌표를 `tuple`로 만든다. 기존 정책은 결과를 JSON에 저장할 수 있도록 같은 좌표를 `list`로 바꿔 반환한다. 새 비교 검사는 이 변환 뒤의 결과와 변환 전의 결과를 그대로 비교했다. 숫자가 같아도 자료형이 달라 거절한 것이다.

처음 단위 시험은 실제 JSON 변환을 하지 않는 단순한 정책 모형을 사용했으므로 이 결함을 놓쳤다. 실제 주행에서 발견한 구현 오류이며, 작은 제동의 효과나 모델 가중치 품질을 보여주는 결과가 아니다.

## 수정과 재시험 원칙

새 v2는 기존 정책의 실제 JSON 변환을 적용한 값으로 동등성을 확인한다. **브레이크만 바꾼다는 검사 자체는 유지**하고 조향·스로틀·목표속도·정지·긴급 권한을 느슨하게 하지 않는다. 실제 자료형 변환을 포함하는 회귀 시험을 추가한 뒤 새 소스·출력 경로·사전 계획으로 실행한다. v1의 재시도와 B 실행은 금지한다.

- [실패 원본](../../../../../storage/experiments/validation/2026-10-01/town07_w1_moving_brake_exploratory_v1/A1_original/).
- [소유자 종료 결과](../../../../../storage/experiments/validation/2026-10-01/town07_w1_moving_brake_exploratory_v1/A1_original/owner_diagnostic_result.json).
- [원본 사전 선언](../../../../../storage/experiments/validation/2026-10-01/town07_w1_moving_brake_exploratory_plan_v1.json).

학습 업데이트·모델 승격·기능 승인은 없다. 기본 주행5개를 위한 원인 판별 단계이며 전체 승인 기능 수는0/30이다.
