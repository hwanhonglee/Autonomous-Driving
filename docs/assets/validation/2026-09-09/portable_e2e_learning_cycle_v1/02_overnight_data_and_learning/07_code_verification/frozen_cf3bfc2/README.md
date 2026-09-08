# 고정 cf3bfc2 코드 회귀 — 6,095 통과 / 6 건너뜀

<!-- HH_260906 - Keep this exact frozen tracked-test denominator separate from private publishers and later untracked tests. -->

2026-09-09 **05:36:26–05:40:56 KST**(2026-09-08 20:36:26–20:40:56 UTC),
commit `cf3bfc26df38c2c61b47095cf3bd08bc82eba196`의 추적 소스·설정 **573개**를 검사 전후 대조했고 변경은 0개였습니다.
발행 시에도 당시 Git 객체 573개의 SHA와 정확한 테스트 파일 목록을 다시 확인합니다.

| 범위 | 추적 Python 파일 | 통과 | 건너뜀 | 실패 | pytest 시간 | 프로세스 전체 시간 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| main | 215 | 5,722 | 6 | 0 | 268.45s | 269.522256s |
| launch | 14 | 373 | 0 | 0 | 13.53s | 14.092859s |
| 두 실행 합계 | 229 | **6,095** | **6** | **0** | 별도 실행 | 병렬 실행: 시간을 합산하지 않음 |

launch의 **13.53s는 pytest 시간**, 약 **14.09s는 프로세스 시작→종료 시간**입니다.
main과 launch는 동시에 실행했으므로 두 시간을 더해 전체 벽시계 시간이라고 부르지 않습니다.
main 파일 목록에는 수집 대상 보조 Python 파일도 포함되며, 파일 수와 테스트 사례 수는 다릅니다.

건너뛴 6개는 안전한 `weights_only` 로딩을 지원하지 않는 기존 로컬 PyTorch 관련 5개,
고정 Woraksan 지도·경로가 없는 1개입니다. 우회 로딩이나 패키지 설치는 하지 않았습니다.
이번 고정 커밋에 들어간 TRAIN oracle·K12 고정 선택기·실행 worker 테스트는 위 분모에 이미 포함되어 있습니다.
개인 artifacts의 발행 도구·설명 그림 테스트와 이 커밋 이후의 미추적/새 테스트는 포함하지 않았습니다.
부분 테스트 수·이전 회귀 수를 다시 더하지 않습니다.

[main 원본 로그](main.log) · [launch 원본 로그](launch.log) · [전체 결과](summary.json) ·
[검사 전 573개 소스 SHA](started.json) · [정확한 main 명령](main.json) · [정확한 launch 명령](launch.json)

## 이전 기록과 범위

[부모 회귀 기록](../README.md), [7f1141 기록](../frozen_7f1141/README.md)의 **5,960 통과 / 6 건너뜀**과 이전 실패 원본은 변경하지 않았습니다.
이번 결과는 고정된 로컬 CPU 코드 검사이며 GPU 학습, 실시간 추론 FPS, Autoware 자율주행,
CARLA 새 주행, 실제 차량 안전·기능 승인 또는 모델 승격을 증명하지 않습니다.
현재 작업 트리 전체를 재실행하면 이후 추가 코드 때문에 분모가 달라질 수 있습니다.

## 원본과 재현

두 `.log` 파일은 원본 바이트 그대로입니다. JSON은 원본 SHA를 덧붙인 메타데이터 뷰이며
필요한 개인 경로만 치환합니다. [실행 driver 소스 뷰](driver_source_view.json)는 당시 driver의
개인 경로를 가린 **참고 자료**이며 실행 파일이 아닙니다. 원본 driver는 비공개로 그대로 보존합니다.
정확한 실행 명령과 215/14개 파일 목록은 main.json/launch.json에 있습니다.
재현에는 고정 commit과 기존 ROS/PyTorch 환경이 필요하며, 이 공개 폴더만으로 설치·실행하지 않습니다.

[원본/공개 SHA 연결](publication_manifest.json) · [공개 체크섬](SHA256SUMS) ·
[원본 private 체크섬 참조](original_private_SHA256SUMS.txt)

이 폴더에서 `sha256sum -c SHA256SUMS`로 공개 파일을 확인할 수 있습니다.
원본 private 체크섬은 변환 전 원본용이므로 공개 JSON의 체크섬과 구분합니다.
