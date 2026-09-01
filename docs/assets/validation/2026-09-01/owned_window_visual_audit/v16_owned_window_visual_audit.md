# Centered Autoware VAD visual audit

- Overall status: **PASS**
- Mechanical audit: **PASS** (18/18 PASS)
- Visual review: **PASS** (18/18 PASS, 0 flagged, 0 pending)
- Contact sheet: `v16_owned_window_contact_sheet.png` (1920×2412)

Every selected rerun is the highest-numbered strict result-PASS attempt for its map/trial. `source_route.json` is byte-for-byte equal to the selected original named by the existing publication manifest.

| Map | Trial | Attempt | Result | Same source route | Midpoint frame | Centered contract | Visual review | Notes |
|---|---|---:|---|---|---|---|---|---|
| `c_track_1_0_7` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+94.630s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 구간 맵 뷰 중앙에 있고 기준 경로, 최종 궤적, VAD 후보 궤적과 전방 카메라 및 Autonomous 상태가 함께 보인다. |
| `c_track_1_0_7` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+76.099s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 좌회전 진입부 중앙에 있고 회전 기준 경로, 최종 궤적과 VAD 후보군이 식별되며 셸 오염이 없다. |
| `town01` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+85.665s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직진 차로 중앙에 있고 기준 경로, 최종 궤적, 후보 궤적과 전방 카메라가 동시에 보인다. |
| `town01` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+76.665s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 회전 구간 중앙에 있고 꺾이는 기준 경로와 최종·후보 궤적이 구분된다. |
| `town02_opt` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+76.643s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 도로 중앙에 있고 경로·후보 궤적·카메라·Autonomous 상태가 함께 보인다. |
| `town02_opt` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+73.794s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 코너 중앙에 있고 회전 기준 경로, 최종 궤적과 다색 VAD 후보군이 판독된다. |
| `town03` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+87.546s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직진 경로 중앙에 유지되고 기준·최종·후보 궤적과 도로 지도가 모두 보인다. |
| `town03` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+83.068s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 교차로 중앙에 있고 회전 방향 기준 경로와 최종·후보 궤적이 확인된다. |
| `town04` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+85.555s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직진 차로 중앙에 있고 기준 경로와 최종·후보 궤적, 전방 카메라가 동시에 표시된다. |
| `town04` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+76.011s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 교차로 중심에 있고 회전 기준 경로와 최종·후보 궤적이 선명하며 외부 창 오염이 없다. |
| `town05_opt` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+87.562s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 맵 뷰 중앙에 있고 직진 기준 경로, 최종 궤적과 투명한 후보 궤적이 함께 보인다. |
| `town05_opt` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+79.428s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 회전부 중앙에 있고 진입·출구 경로와 최종·후보 궤적이 식별된다. |
| `town06` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+87.913s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 도로 중앙에 있고 기준·최종·VAD 후보 경로와 전방 카메라가 보인다. |
| `town06` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+70.099s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 우회전 구간 중앙에 있고 목표 방향 경로와 최종·후보 궤적이 모두 표시된다. |
| `town07` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+94.822s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직진 경로 중앙에 있고 기준 경로, 최종 궤적, 후보 궤적과 상태 패널이 함께 보인다. |
| `town07` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+73.822s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 좌회전 교차로 중앙에 있고 회전 기준 경로와 최종·후보 궤적이 판독된다. |
| `town10hd_opt` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+78.253s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 구간 중앙에 있고 기준·최종·후보 궤적과 카메라·Autonomous 상태가 명확하다. |
| `town10hd_opt` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+69.394s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 우회전 중 맵 뷰 중앙에 있고 진입·회전·출구 경로와 최종·후보 궤적이 보인다. |

## Reproduce

```bash
python3 scripts/e2e/summarize_centered_vad_visuals.py --centered-root /home/a/autoware_e2e/artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final --publication-manifest /home/a/autoware_e2e/docs/assets/validation/2026-09-01/publication_manifest.json --visual-review /home/a/autoware_e2e/artifacts/validation/2026-09-01/v16_owned_window_visual_review.json --output-json /home/a/autoware_e2e/artifacts/validation/2026-09-01/v16_owned_window_visual_audit.json --output-markdown /home/a/autoware_e2e/artifacts/validation/2026-09-01/v16_owned_window_visual_audit.md --contact-sheet /home/a/autoware_e2e/artifacts/validation/2026-09-01/v16_owned_window_contact_sheet.png
```
