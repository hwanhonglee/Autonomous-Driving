# Town06 60 km/h v3 속도 병목 분석

> 분석 대상: `town06_straight_60kph_pilot_best_effort_image_depth1_v3/trial/attempt_001`
> 방식: 저장된 rosbag/JSON/설정/로그만 읽은 오프라인 분석. 추가 주행이나 ROS 상태 변경 없음.
> 보존 결론: **탐색적 FAIL로 그대로 보존하고 재시도하지 않음**. 실제 차량 적용 가능 상태가 아님.

## 결론

60 km/h 목표값 자체는 정상 전달됐다. 플래닝과 게이트 모두 최대·p95가 `16.666666 m/s`였다. 그러나 물리적으로 거의 완전한 직선인 route에서도 VAD-conditioned 궤적의 순간 곡률이 커지며 4~9 m/s 수준의 안전 속도 제한이 반복 적용됐다. 이때 궤적의 `-2.0 m/s²` feed-forward가 속도 오차 PID 출력을 상쇄하여 중간 구간에서 실제 감속까지 발생했다.

가속이 허용된 구간에는 controller/command gate의 `1.5 m/s²` 상한이 37.65초(42.69%) 동안 걸렸고, stock raw converter의 `max_throttle=0.4`도 6.20초(7.03%) 동안 걸렸다. 다만 현재 맵으로 `1.5 m/s²`를 요청할 때 필요한 무제한 throttle은 최대 약 `0.41545`뿐이다. 따라서 `max_throttle=0.5` 단독 변경은 클램프 기여도를 확인하는 작은 simulation-only A/B로는 타당하지만, 15 m/s 도달을 보장하거나 충분하다고 볼 근거는 없다.

최종 결과는 목표점 도달 자체는 성공했으나 최대 속도 `10.1004 m/s`(36.36 km/h), 15 m/s 이상 지속시간 `0.0 s`로 속도 계약이 실패했다.

## 병목 판정

| 항목 | v3의 직접 병목 | 근거 |
|---|---|---|
| 목표/모델 속도 전달 | 아니오 | explicit planning 및 gated target 최대·p95 모두 16.666666 m/s. raw VAD 2.5 m/s는 이 프로파일의 cruise source가 아님 |
| Conditioned 궤적 곡률 | 예, 주된 순간 병목 | 직선 route보다 conditioned trajectory 곡률이 수만 배 큼. 곡률 속도 cap과 horizon 최저속도가 일치 |
| Controller/command gate | 예, 지속 포화 | positive 1.5 m/s² gate limit 37.65 s / 42.69% |
| Throttle 0.4 상한 | 예, 부차적 | 6.20 s / 7.03% near-saturation. 무제한 역산 명령 최대 0.41545 |
| Actuation-map 속도 축 | 이번 v3에서는 아니오 | 실제 최대 10.10 m/s < 축 끝 13.89 m/s, runtime clamp 없음 |
| Route 길이 | 의도 모델에서는 아니오 | 445.88 m는 1.5 m/s² 가속·2.0 m/s² 제동 모델에 충분. 다만 관측 plant 응답에서는 결합 제약이 됨 |

## 직선 route와 곡률 cap 교차 검증

Route preflight는 PASS였다.

- 길이: `445.87995 m`
- chord 최대 편차: `0.00000205 m`
- 누적 절대 heading 변화: `0.000061°`
- route 최대 곡률: `4.80e-7 1/m`

반면 conditioned final trajectory의 snapshot peak curvature는 p95 `0.01905 1/m`, 최대 `0.06313 1/m`였다. 저장 rosbag의 `/planning/trajectory`를 header stamp 기준으로 직접 역직렬화해 세 점 곡률과 `sqrt(1.0/|kappa|)`를 계산하면 다음과 같다.

| 상대 시각 | 남은 거리 | Peak curvature | 계산 cap | Planning horizon 최저속도 | 첫 점 가속도 |
|---:|---:|---:|---:|---:|---:|
| 16.0 s | 418.4 m | 0.0225 1/m | 6.67 m/s | 6.67 m/s | -2.0 m/s² |
| 41.6 s | 215.0 m | 0.0333 1/m | 5.48 m/s | 5.48 m/s | -2.0 m/s² |
| 58.2 s | 93.8 m | 0.0599 1/m | 4.09 m/s | 4.09 m/s | 0.0 m/s² |

중간 감속은 목표점 제동이 아니다. 예를 들어 남은 거리 93.8 m에서 설정된 `2.0 m/s²` 목표점 제동 cap은 약 19 m/s라서 16.67 m/s cruise를 제한하지 않는다. 반면 측정된 horizon 최저속도는 곡률 cap 공식과 일치한다.

Controller debug도 같은 인과를 보여준다.

- 약 16초: target 약 7.51 m/s, trajectory accel -2.0 m/s². 포화된 P+I 요구가 음의 FF에 상쇄되어 약 0.16 m/s²만 남음.
- 약 42초: 실제 약 8.17 m/s인데 target 약 10.48 m/s, trajectory accel -2.0 m/s². 음의 FF 때문에 gate 전 가속 요구가 약 -0.15 m/s²가 됨.

곡률 안전 제한을 꺼서 해결하면 안 된다. 15 m/s에서 곡률 `0.02 1/m`는 약 `4.5 m/s²` 횡가속을 뜻하며 시험 상한 `1.2 m/s²`를 크게 넘는다.

## 종방향 응답과 0.5 throttle 후보

안정적으로 가속한 약 20~40초 구간에서 gated acceleration은 주로 `1.5 m/s²`였고 throttle은 `0.364 → 0.400`으로 증가했지만, robust measured acceleration은 약 `0.32 → 0.13 m/s²`로 감소했다.

| 실제 속도 bin | 표본 | 중앙 throttle | 맵 예상 가속도 중앙값 | 관측 robust 가속도 중앙값 |
|---|---:|---:|---:|---:|
| 6–8 m/s | 301 | 0.375 | 1.501 m/s² | 0.271 m/s² |
| 8–9 m/s | 153 | 0.391 | 1.501 m/s² | 0.232 m/s² |
| 9–10 m/s | 145 | 0.400 | 1.460 m/s² | 0.165 m/s² |
| 10–11 m/s | 16 | 0.400 | 1.419 m/s² | 0.132 m/s² |

이 표는 전용 actuation calibration을 대체하지 않는 mismatch 신호다. header stamp 최근접 정렬을 썼고 응답 지연 보정은 하지 않았다. 다만 command→status RMSE가 `0.01845`, p95 절대오차가 `0.03868`이므로 명령 전달 유실만으로 속도 부족을 설명할 수는 없다.

CSV와 converter 구현을 그대로 모사해 역산하면, `max_throttle=0.4`가 실제로 잘라낸 표본은 117/1765개(6.63%, 33.15~39.60초)이며 무제한 필요값 최대가 `0.41545`다. 즉 0.5행을 열어도 현재 `1.5 m/s²` envelope에서는 0.5까지 명령하지 않는다.

## 맵 축과 route 길이

Actuation map 속도 축은 `13.89 m/s`(50.004 km/h)까지다. v3는 최대 10.10 m/s라 축 clamp가 없었지만, 이후 60 km/h에 가까워지면 마지막 열로 clamp된다. 그러므로 속도를 달성하더라도 축을 16.67 m/s 이상 여유 있게 재계측·확장하기 전에는 60 km/h actuation readiness를 주장할 수 없다.

길이 자체는 원인이 아니다. 이상적인 상수 가속 기준으로 0→15 m/s를 1.5 m/s²로 올리는 데 75 m, 15→0을 2.0 m/s²로 줄이는 데 56.25 m, 합계 131.25 m면 된다. 하지만 약 10 m/s에서 관측 중앙 가속도 0.165 m/s²를 단순 유지한다고 가정하면 10→15 m/s에만 약 379 m가 추가로 필요하다. 이는 plant 모델이 아니라 길이 결합 제약을 설명하는 보수적 예시이며, route 연장으로 근본 문제를 숨기면 안 된다.

## 당시 제안한 다음 단계 (후속 결과 반영)

이 목록은 2026-09-02 분석 당시 제안이다. 1번 corridor A/B는 이후 실행되어 HOLD였고 [후속 결과](geometry_corridor_ab.md)를 함께 봐야 한다.

1. 먼저 기존 `--tight-corridor`만 켠 geometry 단일 A/B를 수행한다. `route_corridor_half_width_m`을 0.5→0.2 m로 줄이고 speed/controller/gate/map/throttle은 모두 고정한다. 직선에서 conditioned trajectory peak curvature와 4~9 m/s mid-route cap이 줄어드는지 확인한다. clipping kink나 correction 증가 시 기각한다.
2. Geometry가 격리된 뒤 `max_throttle=0.5`만 바꾼 simulation-only raw-converter A/B로 0.4 clamp의 실제 기여도를 측정한다. 이는 진단 실험이며 60 km/h 달성 후보로 간주하지 않는다.
3. CARLA 전용 정속 pedal/speed/acceleration sweep을 수행해 종방향 맵을 재계측한다. 속도 축을 16.67 m/s 이상으로 확장하고, 필요 시 controller/gate envelope 변경도 별도 버전의 exploratory profile로 함께 검증한다.

실제 차량에는 이 설정이나 CARLA 맵을 재사용하지 않는다. 별도의 실차 캘리브레이션과 안전 검증이 필요하다.

## 계산 방법과 원본

- 발행용 기계 판독 보고서: [`speed_limit_analysis.json`](speed_limit_analysis.json)
- 발행한 historical pilot 근거: [`historical_pilot/evidence`](../historical_pilot/evidence/)
- 발행한 핵심 근거: `result.json`, `diagnosis.json`, `actuation_map_coverage.json`, `actuation_map_runtime_coverage.json`, `camera_source_5hz_validation.json`, `runtime_load_analysis.json`
- 원시 rosbag, stack log와 대용량 설정 스냅샷은 이 경량 발행본에서 제외했으며 원본 SHA-256은 JSON 보고서에 보존했다.

곡률은 보존된 `trajectory_planar_curvatures()`와 동일한 세 점 Menger 공식을 사용했고, 속도 cap은 `sqrt(maximum_lateral_acceleration / abs(curvature))`로 계산했다. Throttle 역산은 각 accel-map 행을 절대 현재 속도로 선형 보간한 뒤 요청 가속도 축으로 다시 선형 보간하는 `AccelMap::getThrottle()` 절차를 재현했다. 사용한 핵심 파일의 SHA-256은 JSON 보고서에 기록했다.
