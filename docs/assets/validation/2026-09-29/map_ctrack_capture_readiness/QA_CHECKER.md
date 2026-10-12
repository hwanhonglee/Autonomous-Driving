# C-track 지도 촬영 후 CPU 점검 도구

<!-- HH_260929 - Define technical post-capture QA without opening the map split or approving data. -->

[사전 촬영 점검](README.md)의 두 경로에 대해, 새 RGB/semantic/depth/pose 수집본이 만들어졌을 때 [점검 코드](../../../../../tools/e2e/audit_ctrack_mapping_capture.py)를 실행한다. 이 점검 도구의 단위 시험은 CARLA·GPU·SSH 없이 수행했으며, 단위 시험만으로 실제 C-track 수집 성공이나 데이터 편입을 주장하지 않는다.

```bash
cd /home/a/autoware_e2e
python3 tools/workspace_python.py tools/e2e/audit_ctrack_mapping_capture.py \
  --scenario straight \
  --source /absolute/path/to/owned_capture/mapping \
  --output /absolute/path/to/fresh_straight_qa.json

# HH_260929 - Use a separate fresh output file for the archived left route.
python3 tools/workspace_python.py tools/e2e/audit_ctrack_mapping_capture.py \
  --scenario left \
  --source /absolute/path/to/owned_left_capture/mapping \
  --output /absolute/path/to/fresh_left_qa.json
```

`--scenario straight`는 직진 p03 route SHA-256 `cb8632fa48598389a948fe552826ab05a0d2c737126dd3f39e4dde12d5e3b70e`, `left`는 확장 좌회전 route SHA-256 `2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b`를 요구한다. 다른 route나 Town05 원본을 C-track으로 검사하면 `FAIL`이다. `--output`을 빼면 원본 수정 없이 간단 요약만 출력한다. 지정한 결과 파일은 새 파일이어야 하고 수집 원본 폴더 밖에 있어야 한다. `FAIL`의 종료 코드는 2다.

도구는 원본 `summary.json`, `records.json`, 240개의 RGB PNG와 BGRA semantic/depth·카메라 pose NPZ를 전수 읽고 SHA-256을 보고서에 남긴다. 수집 완료·원본 replay 일치·충돌 0, 연속 CARLA frame과 10 Hz 기록, 640×360 3센서 payload, rigid pose와 카메라 전방 1.5 m mount, 유효 깊이(0.1–40 m), road/lane 정답 태그, 실제 주행 위치를 확인한다. 위치는 0.25 m 간격으로 중복 제거하고 pinned route에 투영해 직진·좌회전 구간의 **전/중/후**를 실제로 지나갔는지 측정한다. 기술적 위생 하한은 서로 다른 위치 120개, road 픽셀 1%, lane 픽셀 0.01%와 lane 양성 프레임 24개, 프레임마다 유효 깊이 5%, 노선 이탈 5 m 이하, 구간 안 프레임 3개다. 구간 안에서도 서로 다른 위치 3개와 route 진행 거리 5 m가 필요하다. 연속 프레임 사이 route 진행 거리가 2 m 넘게 뒤로 가거나 이전 최대 진행 거리보다 3 m 넘게 뒤로 떨어지면 실패한다. 이 값들은 실제 C-track 새 촬영 전에 정한 보수적 수집 위생 조건이며 학습 성능 목표가 아니다.

결과는 구체적인 `fail_reasons`와 측정치를 제공한다. 모두 충족돼도 이름은 `QA_CONDITIONAL`이며 `dataset_admitted=false`다. 점검기는 수집본 부모 폴더의 `owner_plan.json`, `owner_started.json`, `lifecycle/ready.json`, `owner_result.json`과 수집본의 `pre_actor_provenance.json`을 직접 읽고 해시를 기록한다. 정확한 `mapping-ctrack-v2`·Epic·route·소유 세대·live OpenDRIVE 해시·종료 코드 0·학습 미편입 상태를 비교하고, 계획에 적힌 모든 보관 소스의 바이트 해시도 확인한다. 하나라도 없거나 달라지면 `FAIL`이다. 이는 보관 파일 간의 자기 일관성 검증이지 외부 서명·독립 증명은 아니다.

촬영기의 파일에는 세 카메라의 **개별** frame ID·timestamp·pose가 저장되지 않는다. 점검기는 수집 당시 동기화 거절 로직과 `live_replay_exact_match=true`, 저장된 단일 카메라 pose와 차량 위치의 관계만 재확인할 수 있다. Epic 전체 화각 렌더 검토, 통합 사이트 원장의 지도 TRAIN 역할 검토, 별도 미사용 DEV는 후속 관문이다. 이 도구는 그 권한을 만들지 않는다.

[단위 시험](../../../../../learning/tests/test_audit_ctrack_mapping_capture.py) 10개는 합성 10프레임에서 조건 충족 시 `QA_CONDITIONAL`, 충돌·pose 불일치·다른 route/Town, 타임스탬프·깊이·차선·구간 후반 누락과 **첫 통과 후 3 m 역행** 시 명시적 `FAIL`을 확인했다. 소유 실행 결과 실패·누락, 보관 소스 변조, 수집 전 provenance 불일치, C-track 고정 QA 기준 변경도 거부한다. 작은 역행을 여러 번 쌓아 최대 진행 위치에서 4 m 뒤처지는 경우를 계산했고, 직진·좌회전 고정 route의 바이트와 구간을 재확인했다. 실제 C-track 원본의 점수나 성공은 별도 실행 결과로 판단해야 한다.
