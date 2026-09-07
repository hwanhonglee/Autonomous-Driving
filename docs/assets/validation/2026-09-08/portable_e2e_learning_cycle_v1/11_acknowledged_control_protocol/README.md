# 11. 제어 명령 수신 확인·동일 프레임 관측 A/B — 정합성 개선, 데이터 미승인

<!-- HH_260906 - Publish all four observed attempts without equating transport consistency with physical actuation or data admission. -->

기존 비동기 방식 2회와 새 `acknowledged_batch` 방식 2회를 모두 비교했습니다. 새 방식은 **명령 수신 확인과 동일 프레임 차량 관측을 함께 변경한 프로토콜**입니다. 순수한 액추에이터 지연만 바꾼 실험은 아닙니다. Town07/straight, Low, Prius, ClearNoon, seed 0, 목표 28.8 km/h, 물리 20 Hz·카메라 10 Hz, comfortable_v3 제어·품질 기준은 동일합니다.

| 방식 / 실행 | 인접 상태 제어 불일치 | 수신·관측 프로토콜 | 수집기 종료 코드 | 학습 데이터 승인 |
|---|---:|---|---:|---|
| 기존 async 001 | 79 / 1,467 | 엄격한 인접 정합성 FAIL | 1 | 아니요 |
| 기존 async 002 | 85 / 1,477 | 엄격한 인접 정합성 FAIL | 0 | 아니요 |
| 새 ACK 001 | 0 / 1,466 | PASS: 1,472개 수신, 1,467개 상태 + 초기 관측 | 1 | 아니요 |
| 새 ACK 002 | 0 / 1,467 | PASS: 1,473개 수신, 1,468개 상태 + 초기 관측 | 1 | 아니요 |

기존 164건은 모두 두 행 전 요청과 일치했지만, 이것만으로 물리적인 제어 지연을 단정하지 않습니다. 새 방식의 모든 기록된 상태는 원시 ActorSnapshot 벡터에서 다시 계산한 기존 좌표 변환과 정확히 일치했고, 카메라는 각 실행 734개 앵커·6방향 총 4,404개 이미지의 바이트 해시와 프레임·시간을 확인했습니다. 자동 기어는 요청값과의 일치 판정에서 제외합니다. 다른 제어값은 `1e-6` 기준이며, 이전 행으로 라벨을 옮겨 PASS로 만들지 않았습니다.

## 남은 실패: 실제 속도 변화율

| 새 실행 | 20 Hz 최소 속도 변화율 | 카메라 10 Hz 최소 속도 변화율 | 목표 거리 오차 | 연속 순항 |
|---|---:|---:|---:|---:|
| ACK 001 | −8.86046 m/s² | −4.75127 m/s² | 0.83199 m | 8.60 s |
| ACK 002 | −7.73085 m/s² | −3.79393 m/s² | 0.83650 m | 8.65 s |

각 실행에서 20 Hz·10 Hz 모두 디코더의 ±2.9 m/s² 기준을 위반한 구간이 1개씩 남았습니다. 20 Hz는 런타임의 감속 −6 m/s² 기준도 초과합니다. 정차 후 꼬리 구간 문제가 아니라 주행 중 남은 경로 약 47 m 지점입니다. 두 실행 모두 목표 정차·2초 유지·6.5초 정차 꼬리 구간을 완료했고 충돌·차선 침범 기록은 없지만, 스칼라 품질은 FAIL입니다. 종료 코드 0이었던 기존 async 002도 제어 정합성 FAIL이므로 승인하지 않았습니다.

`current_control`과 화면의 `Reported brake`는 **CARLA API가 보고한 제어값**입니다. 물리 페달·토크·실제 적용 시점 증거가 아닙니다. 원본 요약의 `applied`라는 기존 필드명도 같은 제한으로 읽어야 합니다. ACK 수신은 서버가 명령을 받아들였다는 증거이며 물리 작동 시점을 증명하지 않습니다.

## 실제 카메라·차량 중심 경로 화면

- ACK 001: [전체 구간 가속 GIF](visuals/run_001/whole_recording_accelerated.gif), [순항 PNG](visuals/run_001/02_measured_cruise.png), [최대 감속 PNG](visuals/run_001/06_maximum_observed_deceleration.png)
- ACK 002: [전체 구간 가속 GIF](visuals/run_002/whole_recording_accelerated.gif), [순항 PNG](visuals/run_002/02_measured_cruise.png), [최대 감속 PNG](visuals/run_002/06_maximum_observed_deceleration.png)
- [기존 async 두 실행 화면·보고서](../10_comfortable_v3_pilot/README.md)

각 실행에는 시작·순항·관성 주행 진입·목표 정차·마지막 관측·최대 감속 PNG 6장이 있습니다. GIF는 실제 10 Hz 카메라 앵커를 5개마다 선택하고 마지막 앵커까지 포함한 148프레임, 약 73.3초를 14.8초로 표시하는 가속 미리보기입니다. 전체 원본 프레임을 재생하는 실시간 영상이 아닙니다. 경로 화면은 차량 중심·차량 진행 방향 위쪽이며, 주황색 미래 경로는 실제로 뒤에 관측한 궤적입니다. 모델 예측·학습 모델 주행·Autoware 실행 화면이 아닙니다.

Low 원본 화면에는 바둑판 모양의 도로 재질이 보입니다. 이 자료는 실패 진단용 공개이며 시각 품질 또는 학습 데이터 승인을 뜻하지 않습니다. 새 주행 캡처·픽셀 합성·실패 프레임 삭제·라벨 보간은 하지 않았습니다.

## 증거와 재현

[독립 전체 보고서](summary.json), [CARLA 16개 기본 소스 조사](carla_source_evidence.json), [공개 파일 출처](provenance.json), [전체 파일 SHA256](SHA256SUMS)를 함께 제공합니다. 각 실행의 `run_001_image_hashes.json`·`run_002_image_hashes.json`은 모든 카메라 이미지 해시입니다. 두 번째 실행 전 검토는 첫 실행의 상태·수신 기록 해시와 실제 품질 수치에 묶였고, 사전 계획·소스 커밋·최대 2회·종료 시간 조건을 확인했습니다. 기록된 시각이 실제 파일 생성 시각 또는 사람 신원까지 증명하지는 않습니다.

```bash
python3 scripts/e2e/audit_carla_acknowledged_control.py \
  artifacts/training/2026-09-08/acknowledged_control_v1 \
  --campaign \
  --output-dir artifacts/training/2026-09-08/acknowledged_control_audit_recheck
```

원본 대용량 아티팩트와 기록된 Git 이력이 있어야 재현됩니다. 기존 출력 폴더에는 덮어쓰지 않습니다. 조사한 CARLA 0.9.15 로컬 소스와 실제 실행 바이너리의 빌드 동일성은 입증되지 않았습니다. 이 프로토콜의 사전 제한 2회는 모두 사용했으며, 결과는 `COMPLETE_NOT_ADMITTED`입니다. 새 모델 학습·데이터 편입·자동 승격은 없습니다.
