## 캠페인 결론

`speed_30kph_camera_source_5hz` 프로필로 실행 가능한 9개 맵의 직진·회전 18개를
동일한 cold-start full-stack 절차로 검증했다. 최종 matrix는 **COMPLETE 9/9**이고,
선택된 직진·회전 trial은 **18/18 PASS**다. 19개 canonical map 중 실행 자산이 없는
10개는 기존과 같이 `BLOCKED`로 보존했으며 다른 맵의 결과로 대체하지 않았다.

- 실행 맵: `town01`, `town02_opt`, `town03`, `town04`, `town05_opt`, `town06`,
  `town07`, `town10hd_opt`, `c_track_1_0_7`.
- 회전 방향: 7개 좌회전, `town06`·`town10hd_opt` 2개 우회전. 직진과 회전은 각기
  독립 route·result·bag·PNG·GIF·분석 자료를 가진다.
- 여섯 camera의 simulation stamp rate는 모든 선택 trial에서
  `4.999999925 Hz`, 최악 stamp gap은 `0.200000003 s`였다. bundle coverage는
  `99.543-100%`, 선택 trial의 superseded/pruned/coalesced drop은 모두 0이고
  published/mailbox-taken count가 모두 일치했다.
- 직진 최고속도는 `27.65-27.87 kph`로 27 kph 이상 지속 노출 gate를 통과했다.
  회전은 곡률·횡가속 제한 프로필이므로 `16.98-24.23 kph`였고 30 kph 유지 조건을
  적용하지 않았다.
- 최대 CTE는 `0.391-0.626 m`, 최대 횡가속도는 `0.142-1.114 m/s²`였다.
- owned RViz window의 `base_link`, X/Y `0/0`, scale `10` 계약과 실제 판독을 모두
  확인했다. 1920×1080 대표 PNG는 route-midpoint MKV frame과 pixel-exact이고,
  9개 맵 × 2개 장면의 차량·기준 경로·최종 궤적·VAD 후보 궤적이 **18/18 PASS**다.

`town02_opt/straight/attempt_001`은 차량 주행과 goal result 자체는 성공했지만 recorder
시작 뒤 VAD queue `superseded`가 0에서 1로 증가해 엄격 camera-source gate에서
실패했다. 이 최초 시도는 삭제하거나 PASS로 바꾸지 않았다. 동일 route와 설정의
`attempt_002`는 superseded 0, bundle 100%, candidate/front `232/232`,
published/mailbox-taken `277/277`로 PASS했고 최종 선택 시도가 되었다. 첫 시도의
result·stack·recorder·route·runtime과 두 campaign console은
`campaign_diagnostics/`에 별도 보존했다.

## 18개 선택 trial 측정치

| Map | 구간 | 방향 | 시도 | 길이 m | 최고 kph | 최대 CTE m | 최대 횡가속 m/s² | RTF | Bundle % | 최악 gap s | Candidate/front | Pub/taken/drop |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `town01` | 직진 | straight | 001 | 210.2 | 27.68 | 0.547 | 0.583 | 0.998 | 100.00 | 0.200 | 253/253 | 301/301/0 |
| `town01` | 회전 | left | 001 | 81.3 | 20.83 | 0.536 | 1.114 | 0.998 | 100.00 | 0.200 | 237/236 | 282/282/0 |
| `town02_opt` | 직진 | straight | 002 | 177.1 | 27.87 | 0.543 | 0.398 | 0.999 | 100.00 | 0.200 | 232/232 | 277/277/0 |
| `town02_opt` | 회전 | left | 001 | 83.6 | 20.95 | 0.510 | 0.663 | 0.998 | 100.00 | 0.200 | 223/223 | 270/270/0 |
| `town03` | 직진 | straight | 001 | 209.5 | 27.66 | 0.506 | 0.693 | 0.998 | 99.62 | 0.200 | 260/260 | 300/300/0 |
| `town03` | 회전 | left | 001 | 73.1 | 18.28 | 0.548 | 1.042 | 0.255 | 100.00 | 0.200 | 204/203 | 227/227/0 |
| `town04` | 직진 | straight | 001 | 210.0 | 27.76 | 0.391 | 0.199 | 0.999 | 100.00 | 0.200 | 249/248 | 286/286/0 |
| `town04` | 회전 | left | 001 | 70.4 | 17.71 | 0.521 | 0.605 | 0.999 | 99.54 | 0.200 | 219/219 | 265/265/0 |
| `town05_opt` | 직진 | straight | 001 | 209.8 | 27.76 | 0.523 | 0.561 | 0.998 | 100.00 | 0.200 | 251/251 | 297/297/0 |
| `town05_opt` | 회전 | left | 001 | 75.0 | 18.08 | 0.567 | 0.743 | 0.998 | 100.00 | 0.200 | 222/221 | 265/265/0 |
| `town06` | 직진 | straight | 001 | 210.5 | 27.76 | 0.496 | 0.142 | 0.998 | 100.00 | 0.200 | 250/250 | 296/296/0 |
| `town06` | 회전 | right | 001 | 88.1 | 24.23 | 0.478 | 0.865 | 0.999 | 100.00 | 0.200 | 211/212 | 259/259/0 |
| `town07` | 직진 | straight | 001 | 210.6 | 27.65 | 0.531 | 0.712 | 0.998 | 100.00 | 0.200 | 279/278 | 323/323/0 |
| `town07` | 회전 | left | 001 | 82.6 | 19.22 | 0.446 | 0.697 | 0.999 | 100.00 | 0.200 | 218/217 | 262/262/0 |
| `town10hd_opt` | 직진 | straight | 001 | 171.2 | 27.76 | 0.511 | 0.269 | 0.998 | 100.00 | 0.200 | 225/226 | 271/271/0 |
| `town10hd_opt` | 회전 | right | 001 | 79.8 | 17.16 | 0.626 | 0.972 | 0.998 | 100.00 | 0.200 | 215/215 | 261/261/0 |
| `c_track_1_0_7` | 직진 | straight | 001 | 209.5 | 27.86 | 0.550 | 0.443 | 0.998 | 100.00 | 0.200 | 271/271 | 312/312/0 |
| `c_track_1_0_7` | 회전 | left | 001 | 73.9 | 16.98 | 0.513 | 0.707 | 0.999 | 100.00 | 0.200 | 246/245 | 293/293/0 |

Candidate/front가 1 차이인 행은 whole-bag 시작·종료 경계의 한 frame 허용치다. 각
선택 시도의 실제 VAD queue에서는 capacity pruning, runtime supersession,
coalescing이 없었고, 이 조건을 raw stack/recorder log에서 발행 시점에 다시 계산했다.

## 화면 끊김 원인

두 현상을 구분해야 한다. 설정된 5 Hz camera 자체는 새 frame이 0.2초마다 오므로
RTF가 1이어도 20/30/60 fps 영상보다 계단식으로 보인다. 여기에 CARLA RTF가 낮아지면
simulation의 0.2초가 더 긴 wall time으로 늘어나 훨씬 심한 정지·슬로모션처럼 보인다.

18개 중 17개는 RTF `0.998-0.999`였지만 `town03/turn`만 `0.255`였다. 이 시도에서도
여섯 stamp rate는 5 Hz, 최악 gap은 0.2초, bundle은 100%, queue drop은 0이므로
camera message 누락이 원인이 아니다. 벽시계에서 보이는 유효 갱신은 대략
`5 × 0.255 = 1.27 fps`가 되어 같은 simulation frame이 오래 유지된 것이다.

해당 `town03/turn`의 `started_at`부터 `finished_at`까지를 KST 초 단위로 내린 폐구간
envelope에서 raw row(중복 timestamp 유지)를 계산했다. 24 CPU 기준 idle은
`12436/138 = 90.1%`(최저 `79%`), GPU SM은 `1891/132 = 14.3%`(최대 `44%`),
I/O wait는 평균·최대 모두 `0%`였다. 원본 vmstat·GPU log와 이 구간 excerpt도
`campaign_diagnostics/`에 보존했다. 따라서 PC 전체의 지속적인 CPU/GPU 포화보다는
CARLA renderer/scheduler·cold-start/cache 상태의 일시적 실시간성 저하가 더 잘 맞는다.
다만 이 측정은 single-core, sub-second spike 또는 renderer synchronization 병목을
배제하지 않는다. 이전에 확인된 `vad_route_manager` OpenBLAS 24-thread 과구독은 해당
노드에만 BLAS thread 1개 제한을 적용해 제거했으며, 선택 18개에서는 camera gap 또는
queue loss로 재발하지 않았다.

## 해석 경계

이 캠페인은 `vad_route_manager_hybrid`에서 VAD geometry를 평가하고 종방향 cruise
속도는 explicit CARLA simulation overlay가 공급하는 simulation screening이다. raw VAD
cruise velocity 검증이나 실제 차량 준비 완료를 뜻하지 않으며
`real_vehicle_ready=false`다. 다음 60 kph 단계는 이 30 kph 기준을 그대로 유지하면서
정지거리·경로 길이·곡률별 제한·제어기 saturation gate를 별도 프로필로 먼저 정의해야
한다.
