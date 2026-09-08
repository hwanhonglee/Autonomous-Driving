# nuPlan 실차 데이터 준비 — DB 한 개의 읽기 전용 검사

<!-- HH_260906 - Separate native metadata availability from legal approval, calibrated six-camera inputs and model training. -->

결론은 **메타데이터를 읽을 수 있지만, 현재 common10 학습 데이터로는 준비되지 않았다**입니다.
2026-09-09 03:09 KST에 기존 archive 안의 DB 한 개만 메모리에서 읽었습니다.
디스크로 압축 해제·설치·GPU 사용·이미지/지도 payload 읽기·pickle 실행·변환·학습·이용조건 동의는 하지 않았습니다.
선정한 DB 한 개는 압축된 내용을 메모리에서 풀어 CRC·SHA와 SQLite 메타데이터를 검사했습니다.
원본 보고서의 `extracted: false`는 디스크에 DB를 추출하지 않았다는 뜻이며, 압축 payload를 읽지 않았다는 뜻이 아닙니다.

## 확인 범위

camera group 0에 포함된 7개 log 중 `(압축 전 크기, member 이름)` 순서의 첫 DB를
payload 열기 전에 고정했습니다. 주행 성적에 따라 고른 log가 아닙니다.

- DB: `2021.06.03.12.02.06_veh-35_00233_00609.db`
- 압축 전 173,629,440 bytes, 압축된 member 100,266,146 bytes; member CRC·SHA 검증.
- 개인 venv의 Python 3.12.3 / SQLite 3.45.1; 보고서 구간 약 0.857초, 최대 RSS 약 445 MiB.
- 상한: 메모리 2 GiB / CPU 60초 / 외부 wall timeout 90초. DB는 `:memory:`로만 열었습니다.
- 원본 archive의 실행 전후 크기·mtime·ctime·inode·device 및 기존 검증 보고서 해시를 대조했습니다.
  **전체 8.55 GB archive를 이번에 다시 해시한 것은 아닙니다.** 이전 full CRC/SHA 근거와 이번 단일 member 검증을 구분합니다.

[사전 선택 계획](original_execution/selection_plan.json) · [원본 검사 보고서](original_execution/report.json) ·
[실행 소스 원본](source_executed/inspect_nuplan_single_db.py) · [게시 파일 SHA-256](SHA256SUMS).

## 실제로 있던 데이터

| 항목 | 이 DB에서 확인한 값 |
|---|---|
| 카메라 | 8개 × 3,760 records = 30,080개; 각 약 10 Hz |
| 카메라 연속 timestamp 간격 | p99 100.436–100.438 ms, 중복 timestamp 0 |
| Ego pose | 37,800개, 약 100 Hz |
| LiDAR timestamp | 7,520개, 약 20 Hz |
| 로그 길이 | 카메라 약 375.9초 |
| 연결 무결성 | image→camera/ego, lidar→ego/scene 누락 0 |
| native scene route | 20/20 scene에 roadblock ID text 존재; 각 20–40개 |
| mission goal pose 연결 | **8/20만 이 DB 안에서 조회 가능**, 나머지 12개는 미확보 |
| 카메라 크기 | DB 선언은 모두 1920×1080; JPEG를 열지 않아 실제 픽셀 크기는 이번에 검증하지 않음 |
| 보정값 | translation/rotation/intrinsic/distortion 총 32개 BLOB의 타입·길이·SHA만 확인 |

## 아직 해결되지 않은 입력 조건

![원본 카메라 timestamp의 관측 간격](01_native_camera_timestamp_offsets.png)

F0의 각 원본 timestamp에 가장 가까운 다른 채널 timestamp를 선택한 **8-camera 진단**에서는
3,760개 모두 최대·최소 간격이 **42.328–42.473 ms**였습니다. 현 계약의 20 ms보다 큽니다.
F0와 B0만 비교해도 B0가 **24.901–25.021 ms** 늦었습니다. 이것은 선택된 6-camera rig의
합격/불합격 시험이 아니며, 보정·화각을 확인하지 않은 채 8개 중 6개를 임의 채택하지 않습니다.
카메라가 각각 10 Hz라는 사실만으로 동시 관측 조건을 만족하는 것은 아닙니다.

연결된 record라도 `image timestamp − ego timestamp`는 −5.556~+5.508 ms,
`lidar timestamp − ego timestamp`는 −5.377~+5.516 ms였습니다.
원본 시간을 보존했으며 같은 시각으로 덮어쓰지 않았습니다. 공식 devkit도 연결된 ego/lidar
시각이 다를 수 있음을 명시하고 일부 API에서 호환성 때문에 lidar 시각을 사용하므로,
이를 그대로 현재 시각의 입력이라고 가정하면 안 됩니다.
[공식 native query 구현](https://github.com/motional/nuplan-devkit/blob/master/nuplan/database/nuplan_db/nuplan_scenario_queries.py).

보정 필드는 공식 구현에서 pickle 역직렬화를 사용합니다. 이번 검사는 이를 실행하지 않았으므로
숫자 보정값·광학 프레임·왜곡 보정·화각은 아직 미검증입니다.
[공식 calibration serializer](https://github.com/motional/nuplan-devkit/blob/master/nuplan/database/common/sql_types.py),
[공식 camera schema](https://github.com/motional/nuplan-devkit/blob/master/nuplan/database/nuplan_db_orm/camera.py).

roadblock text와 mission goal 연결이 있다는 사실은 **그 경로가 anchor 이전에 차량에 제공됐다는
시각 증거**가 아닙니다. 지도 중심선도 열지 않았고, 정지 의도 정답도 정의하지 않았습니다.
카메라 DB의 JPEG 경로와 실제 camera ZIP member의 정확한 일대일 대응은 이번 범위 밖입니다.
서버의 원본·보정 BLOB·이미지는 게시하지 않고 크기와 해시·집계만 남겼습니다.

다음 단계는 이용 범위 검토, 실행 코드가 없는 숫자 보정 파서 설계, 명시적인 6-camera 선택과
비동기 관측 처리 규격 검토입니다. timestamp를 수정하거나 20 ms 기준을 임의로 완화해
현재 corpus에 넣지 않습니다. 신규 C-track 실패 데이터와 마찬가지로 **데이터 승인·실차 사용 승인은 없습니다.**

## 실행 코드와 사후 보강을 구분

실제 검사는 보관된 `8285ac1b…` 소스와 사전 단위 테스트 36개 통과 후 한 번만 실행했습니다.
이후 원본 파일/기존 검증 보고서가 사라지거나 변할 때 실패 보고서 저장이 생략될 수 있는
정리 경로를 보강하고, 공백·주석이 있는 virtual-table 선언도 거부하도록 테스트했습니다.
보강판 테스트는 43개 통과했고, **DB payload를 다시 읽지는 않았습니다.**
[소스 버전·전송·보강 기록](execution_and_hardening.json)이 원래 실행과 사후 수정을 구분합니다.
