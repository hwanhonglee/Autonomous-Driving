# Centered Autoware VAD visual audit

- Overall status: **PASS**
- Mechanical audit: **PASS** (18/18 PASS)
- Visual review: **PASS** (18/18 PASS, 0 flagged, 0 pending)
- Contact sheet: `v16_owned_window_contact_sheet.png` (1920×2412)
- Publication selection binding: `9f30a6629f72e10ea2f0d2d75838308a814c3756b7bd59a5b05bf4ff61932bcb` (18 stable records; not a full-file manifest digest)

Every selected rerun is the highest-numbered full-stack result-PASS attempt for its map/trial; this result-level label is separate from the camera-source matrix gate. `source_route.json` is byte-for-byte equal to the selected original named by the publication selection.

| Map | Trial | Attempt | Result | Same source route | Midpoint frame | Centered contract | Visual review | Notes |
|---|---|---:|---|---|---|---|---|---|
| `c_track_1_0_7` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+26.833s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 구간 중앙에 있고 기준 경로, 최종 궤적, VAD 후보 궤적과 전방 카메라가 함께 식별된다. |
| `c_track_1_0_7` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+24.428s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 회전 진입부 중앙에 있고 굽은 기준 경로, 최종 궤적과 VAD 후보군이 명확하다. |
| `town01` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+24.898s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직진 차로 중앙에 있고 기준·최종·후보 궤적과 Autoware 상태를 함께 확인할 수 있다. |
| `town01` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+23.473s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 교차로 회전 구간 중앙에 있고 굽은 경로와 궤적 계층이 구분된다. |
| `town02_opt` | `straight` | `attempt_002` | PASS | EXACT_MATCH | PASS (+22.670s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 재시도 PASS 화면에서 차량이 직선 차로 중앙에 있고 경로와 VAD 후보 궤적이 선명하다. |
| `town02_opt` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+22.244s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 코너 중앙에 있고 회전 기준 경로, 최종 궤적과 다색 VAD 후보군이 판독된다. |
| `town03` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+25.837s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직진 경로 중앙에 유지되고 기준·최종·후보 궤적과 도로 지도가 모두 보인다. |
| `town03` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+71.376s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 복합 교차로 중심에 있고 회전 경로와 여러 VAD 후보 궤적이 함께 식별된다. |
| `town04` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+24.732s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 다차로 뷰 중앙에 있고 기준 경로와 최종·후보 궤적을 판독할 수 있다. |
| `town04` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+21.704s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 교차로 회전부 중앙에 있고 굽은 기준 경로와 VAD 후보군이 전체 화면에 보인다. |
| `town05_opt` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+24.830s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 경로 중앙에 있고 교차로 전후 기준 경로와 최종·후보 궤적이 보인다. |
| `town05_opt` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+22.028s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 복합 회전 구간 중앙에 있고 경로, 최종 궤적과 후보 궤적 분포가 확인된다. |
| `town06` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+24.915s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 수평 직선 경로 중앙에 있고 기준·최종·후보 궤적의 겹침을 판독할 수 있다. |
| `town06` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+21.108s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 우회전 궤적 중앙에 있고 회전 기준 경로, 최종 궤적과 후보군이 보인다. |
| `town07` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+27.729s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 직선 도로 중앙에 있고 기준 경로와 최종·후보 궤적이 차량 주위에 정렬된다. |
| `town07` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+21.591s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 회전 교차로 중앙에 있고 굽은 기준 경로와 최종·후보 궤적이 명확하다. |
| `town10hd_opt` | `straight` | `attempt_001` | PASS | EXACT_MATCH | PASS (+22.408s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 수평 직선 구간 중앙에 있고 기준 경로와 최종·VAD 후보 궤적을 확인할 수 있다. |
| `town10hd_opt` | `turn` | `attempt_001` | PASS | EXACT_MATCH | PASS (+21.433s) | PASS (`base_link`, X/Y 0, scale 10) | PASS | 차량이 우회전 구간 중앙에 있고 회전 기준 경로, 최종 궤적과 VAD 후보군이 선명하다. |

## Reproduce

```bash
python3 scripts/e2e/summarize_centered_vad_visuals.py --centered-root /home/a/autoware_e2e/artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1 --publication-manifest /home/a/autoware_e2e/docs/assets/validation/2026-09-02-all-towns-camera-source-5hz/publication_manifest.json --visual-review /home/a/autoware_e2e/artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/owned_window_visual_audit_camera_source_5hz/v16_owned_window_visual_review.json --output-json /home/a/autoware_e2e/artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/owned_window_visual_audit_camera_source_5hz/v16_owned_window_visual_audit.json --output-markdown /home/a/autoware_e2e/artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/owned_window_visual_audit_camera_source_5hz/v16_owned_window_visual_audit.md --contact-sheet /home/a/autoware_e2e/artifacts/validation/2026-09-02/autoware_vad_town_matrix_30kph_camera_source_5hz_v1/owned_window_visual_audit_camera_source_5hz/v16_owned_window_contact_sheet.png
```
