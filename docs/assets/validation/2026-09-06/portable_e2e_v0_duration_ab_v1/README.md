# Portable E2E v0 duration A/B evidence

이 폴더는 동일한 CARLA Common10 corpus, `perspective_trajectory.v0`, seed, batch,
optimizer 설정을 유지하고 학습 길이만 154 step(1 epoch)에서 1,540 step(10 epoch)으로
늘린 duration A/B의 정리본이다. 원격 호스트 주소, 사용자 홈 절대경로 및 GPU UUID는 포함하지
않았다.

## 결론

- Town03 우회전 `val` 337개에서 1/3/6.4초 ADE와 FDE, speed/yaw 오차는 모두 감소했다.
- 보존된 최초 `runtime_gate_audit.json`은 이전 gate 정의에서 learned-selected
  trajectory `16/337`을 기록한 **historical 비교 자료**다.
- 최신 고정 runtime geometry gate v6 재감사에서는 여섯 후보 각각과
  learned-selected trajectory가 모두 `0/337`이며, selector는 candidate 1을 `337/337`
  선택했다. selected failure는 geometric-speed·speed-disagreement·step·reported/geometric
  speed-rate·distance-disagreement·curvature·lateral-acceleration이 각각 `326`, heading
  `310`, backward-step `94`, speed `47`이다.
- 따라서 이 결과는 학습 시간 증가가 open-loop 회귀 오차를 줄였다는 증거일 뿐, shadow
  runtime 승격이나 차량 제어 승인이 아니다.

## 파일

- `run_summary.json`: 10-epoch 학습 설정, provenance 및 마지막 batch 지표의 sanitized 요약
- `duration_ab_comparison.json`: 보존된 1-epoch baseline과 10-epoch 결과의 정량 비교
- `town03_val_metrics.json`: Town03 전체 `val` open-loop 평가 결과
- `runtime_gate_audit.json`: 이전 gate 정의의 historical 감사 원본; 최신 판정에 사용 금지
- `local_cpu_runtime_smoke_gate_v6.json`: 안전한 bundle strict-load, 30개 CPU 호출과 최신
  gate v6 전체 Town03 `val` 337개 감사의 sanitized 결과
- `runtime_bundle_export_summary.json`: executable checkpoint를 runtime에 직접 읽히지 않기
  위한 private `.runtime.npz` export 및 양쪽 strict-load 검증 요약
- `local_ros_graph_startup_smoke.json`: 설치본 ROS 2 shadow node의 provenance heartbeat,
  전용 output namespace, 제어 publisher 부재 및 clean SIGINT 종료 확인. live TF extrinsic
  parity 미구현을 명시하며 전체 `healthy_now=false`를 유지함
- `trajectory_png_manifest.json`: deterministic evaluation order의 앞 12개 PNG 크기와 SHA-256
- `trajectory_png/`: 900x700 RGB trajectory plot 12장
- `SHA256SUMS`: 이 evidence bundle의 파일 무결성 목록

각 PNG는 회색 route, 초록 expert trajectory, 굵은 색상의 selected trajectory와 ego-frame
원점을 보여주는 open-loop 분석 그림이다. Autoware/CARLA 전체 화면 캡처로 간주하면 안 된다.
private weight bundle 자체는 Git에 포함하지 않는다. 입력이 없는 ROS graph startup smoke만
완료했으며 실제 CARLA shadow 화면, trajectory output 및 sensor-to-plan 10 Hz 증거는 아직 이
폴더에 없다. 이 자료들은 후속 live run에서 별도 카테고리로 수집한다.

> RESEARCH SHADOW EVIDENCE ONLY — NOT APPROVED FOR VEHICLE CONTROL
