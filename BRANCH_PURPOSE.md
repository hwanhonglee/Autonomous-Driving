# fixmoving 플랫폼 개발

<!-- HH_261012 - Define fixmoving as the user's future platform without inferring a vehicle model or hardware specification. -->

이 브랜치는 `autoware-platform/v1.9.0-fixmoving`입니다. **fixmoving은 앞으로 개발할 플랫폼 이름이며 차량 모델명이 아닙니다.** 공통 Autoware 소스 snapshot을 그 플랫폼에 연계·확장하는 개발 기준선으로 사용합니다.

플랫폼에서 Autoware가 맡을 역할과 입력·출력, 구성 요소 사이의 연결 범위와 검증 기준을 후속 설계에서 정합니다. 이름만으로 하드웨어·센서·구동 방식이나 기존 vehicle interface의 존재를 가정하지 않으며, 브랜치 목적을 정하는 데 차량 모델명은 필요하지 않습니다.

CARLA 실행·보정·연구 코드는 공통 개발 이력으로 유지합니다. 이 브랜치 생성만으로 fixmoving의 전체 구현, 새 모델의 성능 또는 차량 구동 검증이 완료된 것은 아닙니다. 자체 portable E2E 주행 모델과 지도 모델은 별도의 학습·평가를 유지합니다.

향후 실제 차량을 연결한다면 해당 차량의 인터페이스·보정·비구동 검증·안전 조건을 별도로 확인하고 구동은 별도 승인 아래 진행합니다. 이 안내는 플랫폼의 목적을 정의한 것이며 안전 인터페이스 구현이나 소프트웨어 interlock을 대신하지 않습니다.

[세 브랜치와 재현 방법](docs/autoware-three-branch-guide.md) · [작업공간](WORKSPACE.md) · [자체 모델 현황](docs/assets/validation/2026-10-12/README.md)
