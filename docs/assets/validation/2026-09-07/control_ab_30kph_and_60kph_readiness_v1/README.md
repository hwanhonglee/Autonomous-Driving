# 2026-09-07 control validation review set

이 폴더는 30 km/h strict six-camera 10 Hz A/B 결과, 과거 60 km/h readiness 근거,
Town06 strict 10 Hz 60 km/h 1차 실행 결과를 한곳에 모은 발행본이다. 원시 rosbag,
MKV, 실행 로그와 임시 파일은 포함하지 않았다.

- `00_summary`: 기계 판독 결과표, 파일 manifest와 SHA-256
- `01_town07_straight`: baseline과 longitudinal recovery 2.0 후보
- `02_c_track_turn`: 선택 baseline, 5 m 후보, 독립 safety screen과 비교
- `03_town03_turn`: 선택 baseline과 10 m 후보 비교
- `04_rejected_and_runtime_diagnostics`: C-track 10 m geometry fail과 host stall 근거
- `05_60kph_readiness`: 5 Hz historical pilot, strict 10 Hz live v1과 blocker 분석

차량 제어 주체는 모든 최신 30 km/h 실행에서 Autoware VAD였다. Portable E2E는
10 Hz shadow-only였고 actuator를 제어하지 않았다. 후보는 어느 것도 승격되지 않았으며
strict 10 Hz live v1은 Town06 경로를 완주했지만 최고 36.44 km/h였고, 전체 구간
카메라 p95 지연과 첫 녹화 경계의 1프레임 비대칭 때문에 최종 `NO_GO`다.

manifest의 source-relative 경로와 JSON 내부 `/home/a/...` 값은 실행 당시 원본
provenance를 보존한 기록이며 clone 후 실행 경로가 아니다. 발행본 검증은 상대 경로의 `SHA256SUMS`와
`publication_manifest.json`만 사용한다. rosbag, MKV, stack/runtime 로그는 제외했고,
host stall 판정에 필요한 route 구간 vmstat 표본만 파생 JSON으로 보존했다.
