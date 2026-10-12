# 데이터·모델·원시 실험 보관

<!-- HH_260906 - Distinguish large operational assets from reviewed report publications. -->

- `assets/`: 이전 `data/`. 실행 지도, 모델 등 의존 자산.
- `datasets/`: 이전 `datasets/`. 원시·준비된 학습 데이터.
- `experiments/`: 이전 `artifacts/`. 원시 센서·로그·실험 결과와 실패 근거.

세 폴더는 실제 이동했고 Git에서 제외합니다. 공개 보고서·PNG/GIF는 `docs/assets/`에 그대로 있습니다.
기존 manifest 경로를 보존하기 위해 루트 호환 링크가 있습니다. 과거 해시 고정 데이터 계약은
임의 수정하지 말고 새 실행부터 새 경로로 선언해야 합니다.

원격 개인 dataset 폴더는 이번 로컬 구조 변경의 대상이 아닙니다.
