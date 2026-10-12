# Autoware 코드 아키텍처 스캐폴드 개발

<!-- HH_261012 - Isolate prospective architecture refactoring from vehicle commissioning and simulation experiments. -->

이 브랜치는 `autoware-architecture/v1.9.0-scaffold`입니다. 공통 Autoware 소스 snapshot에서 코드 의존 관계·어댑터·인터페이스·검사 구조를 후속 개편하기 위한 개발 기준선입니다. 이번 브랜치 생성 자체로 새 아키텍처 변환이나 새 자율주행 기능이 구현된 것은 아닙니다.

실제 작업공간의 `autonomy/learning/tools/storage/docs` 구조를 유지합니다. 제거한 레거시 경로·심볼릭 링크를 다시 만들지 않으며, 다른 브랜치에서 사용 중인 runtime과 동결된 실험 원본을 이동하거나 덮어쓰지 않습니다.

후속 변경에서는 자체 주행 모델과 자동 지도 모델을 독립된 인터페이스로 유지하고, Autoware 및 일반적인 소비자 어댑터를 분리합니다. 학습, 추론, 센서·좌표 계약, 차량 제어와 독립 평가를 혼동하지 않습니다. 자동 HD 지도는 보조 입력이며 지도 없이 동작하는 주행 경로의 검증도 유지합니다.

리팩터링 전후의 메시지·좌표·시간·출력 동등성 및 기존 회귀 검사를 우선합니다. 구조를 정리했다고 주행 성능이 개선되거나 차량 구동이 승인된 것으로 표시하지 않습니다. 모듈 이동·새 head·제어 로직 변경은 각각 별도의 명시적 변경과 새 평가가 필요합니다.

[세 브랜치와 재현 방법](docs/autoware-three-branch-guide.md) · [작업공간](WORKSPACE.md) · [자체 모델 현황](docs/assets/validation/2026-10-12/README.md)
