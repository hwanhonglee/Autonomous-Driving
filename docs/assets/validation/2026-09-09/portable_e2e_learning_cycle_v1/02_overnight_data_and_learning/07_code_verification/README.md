# 코드 전체 회귀 검사 — 새 검사 통과와 이전 실패 보존

<!-- HH_260906 - Report only the frozen tested revision and keep skipped security/runtime checks explicit. -->

후속 전체 검사는 [commit 7f1141b 결과](frozen_7f1141/README.md)와
[commit cf3bfc2 결과](frozen_cf3bfc2/README.md)에 각각 보존했습니다.
최신 cf3bfc2 검사는 **6,095 passed / 6 skipped / 0 failed**, 소스·설정 573개 전후 동일이며
TRAIN oracle·K12 고정 선택기·원격 실행기 테스트가 포함됩니다. 앞선 7f1141b의
5,960개 또는 아래 99902f3 결과를 더해 하나의 검사 수로 세지 않습니다.

로컬에서 **2026-09-09 03:48–03:52 KST**, commit `99902f3d604543b62e56facb3b32303a3a89aa77`의
소스·설정 553개를 검사 전후 대조했습니다. 변경된 파일 0개이며, GPU는 사용하지 않았습니다.

| 범위 | 통과 | 건너뜀 | 실패 |
|---|---:|---:|---:|
| 저장소 tests: 추적된 Python 파일 205개 | 5,234 | 6 | 0 |
| Autoware launch: 추적된 Python 파일 14개 | 373 | 0 | 0 |
| 이번 전체 실행 합계 | **5,607** | **6** | **0** |

부분 검사의 통과 수를 여기에 다시 더하지 않습니다. 이후에 수정한 코드, 새 학습 결과,
실시간 10 Hz 추론, 차량 주행 성공 또는 실제 차량 안전을 증명하는 수치가 아닙니다.

## 어떤 검사는 건너뛰었나요?

- 5개는 로컬의 기존 PyTorch가 안전한 `weights_only` checkpoint 로딩을 지원하지 않아 건너뛰었습니다.
  우회 로딩이나 패키지 설치는 하지 않았습니다. 이번 원격 계측 학습의 별도 안전 로딩 검증과
  이 다섯 로컬 테스트의 미실행을 혼동하면 안 됩니다.
- 1개는 고정된 Woraksan 지도·경로 묶음이 없어 건너뛰었습니다.

[현재 main 원본 로그](current/main.log) · [launch 로그](current/launch.log) ·
[전체 결과](current/summary.json) · [검사 전 소스 SHA 목록](current/started.json)

## 이전 실패를 지우지 않았습니다

이전 b478 고정 검사는 **5,375 통과 / 1 실패 / 6 건너뜀**이었습니다.
과거 decoder 분석 도구는 의도적으로 당시 모델 해시만 허용합니다. 새 연구 모델이 추가된 뒤에도
그 과거 도구가 현재 모델을 허용해야 한다고 가정한 테스트가 실패했습니다.
테스트만 수정하여 과거 소스 상수와 설정을 확인하고, 달라진 현재 모델은 올바르게 거부하는지
검사하도록 했습니다. 과거 도구의 해시 허용 목록·모델·판정 기준은 바꾸지 않았습니다.
합성 양성 fixture는 과거 수치 실험의 재실행을 의미하지 않습니다.

[이전 실패 로그](historical/main.log) · [이전 launch 로그](historical/launch.log) ·
[이전 전체 결과](historical/summary.json)

## 재현과 무결성

기존 의존성이 갖춰진 로컬 저장소에서, 검증한 commit의 코드가 모두 같은지 먼저 확인한 뒤
다음처럼 실행할 수 있습니다. 환경 설치나 원격 학습 명령이 아닙니다.

```bash
source scripts/e2e/env.sh
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 python3 -m pytest -q -rs -p no:cacheprovider tests
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 python3 -m pytest -q -rs -p no:cacheprovider autoware_e2e_vad_launch/test
```

원래 실행의 정확한 파일 목록·명령은 current/main.json 및 current/launch.json에 있습니다.
후속 코드나 새 테스트가 추가된 작업 트리에서 실행하면 위의 고정 분모와 달라질 수 있습니다.
[publication_manifest.json](publication_manifest.json)은 원본과 공개 파일 SHA를 연결합니다.
historical 로그의 개인 작업 경로만 치환할 수 있으며, 원본은 private artifacts에 보존합니다.

이 폴더에서 `sha256sum -c SHA256SUMS`로 공개 자료를 확인할 수 있습니다.
current/SHA256SUMS는 당시 private 원본의 체크섬도 보존한 것이므로, 공개 변환이 있다면
그 체크섬과 공개 폴더용 체크섬의 역할을 구분해야 합니다.
