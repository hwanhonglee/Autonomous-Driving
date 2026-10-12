# Failed Town07 capture: useful cruise evidence, no admission

<!-- HH_261001 - Preserve the failed episode and every candidate-window denominator. -->

The read-only [census](window_census_v1/summary.json) retains all 945 camera anchors. Of these, 881 have a complete 6.4 s future and 817 clear the recorded-window physical screen, camera indices 64–880 (anchor progress 0.909–196.515 m). No Common10 data was created. All 945 remain admission DENIED; the original episode failure, +0.071 m terminal overshoot at nonzero speed, and native launch violation remain unchanged.

795 clear anchors have current longitudinal speed 1.5–2.5 m/s; all 795 have |v(+0.3 s)−vx|≤0.1 m/s. This is a descriptive coverage count, not a selection criterion. These heavily overlapping windows from one straight route are not 795 independent maneuvers. They provide measured low-speed cruise evidence absent from the 16 retained TRAIN anchors in that speed band, whose immediate futures predominantly accelerate.

The screen covers ten camera observations (0.9 s), 64 actual future targets, native interpolation brackets, and a preceding native guard. It retains the original nine 10 Hz physical metrics, applies their first-step checks to history edges, and additionally screens native scalar speed/rate, frame clocks, full raw-state coordinate reconstruction, six-camera pixel/frame/timestamp checks, acknowledged control chains, events, phases, and terminal-side geometry. The guard observation's incoming scalar-rate interval is conservatively screened too. There is no future extrapolation, clipping, model-score selection, or label change.

The +3.193852 m/s² native pulse ending at frame 1347 excludes 64 overlapping windows. The last 64 anchors lack full future support and their available support touches the terminal-overrun state. Other overlapping reasons include 9 incomplete histories and 45 warmup/phase-crossing spans. The single `nine_discrete_future_bounds` entry is the final anchor's unavailable diagnostic (zero valid points), not an additional observed motion violation. All 881 full futures passed the nine discrete bounds.

All 1,890 native actor bodies and 1,889 bounded snapshot-linear sweeps fit the same 10 cm inset intended-lane corridor, maximum outside area zero. Geometry uses the archived active OpenDRIVE and actual eight-vertex Prius box with full recorded roll/pitch/yaw; every native polygon is retained in the compressed ledger. It is not proof of continuous physical motion. Full original ACK reconstruction is PASS, establishing server acceptance and observed-control agreement, not independent physical actuation. Pixel decoding/calibration binding alone does not establish visual asset/semantic quality.

## Exact admission blockers

`prepare_carla_common10_dataset.py:351–426` rejects explicit `training_data_approved=false`, `status != complete`, and absent true catalog-goal completion. The existing moving exception is not arbitrary post-hoc filtering: `qualify_carla_moving_recovery_common10.py:115–156,216–236` requires a sealed prospective slot/plan, truthful completed moving task, unchanged PENDING_QA metadata, 70 warmup and 130 tail states, and exact membership replay. Its converter selects a fixed geometric interval, not a favorable physical subset (`prepare_carla_common10_dataset.py:1148`). The old complete-episode body validator also correctly rejects this failed episode; the present report independently invokes only its pure geometry routines and never fabricates a complete manifest.

## Fastest defensible new collection proposal — not launched

Declare one NEW low-speed moving-cruise task on this already TRAIN Town07 straight route, with an independently source-bound owner and unchanged physical gates. A fixed geometric anchor interval [30,120] m offers at least 36 s at the 2.5 m/s cap. Require full 0.9 s history plus native guard before the first anchor, at least 30 s actual selected-camera span, and a complete measured 6.4 s future after the last anchor. At the cap, final support is near 136 m, leaving about 74 m of the 210.6 m route for a separate safely recorded termination; this is a geometric budget, not a guaranteed stopping distance.

Use actual expert control, not any learned rollout relabeled as expert; retain all startup and shutdown records. Preserve BasicAgent emergency intervention and abort evidence. Select every anchor in the predeclared interval or fail the whole moving-window task—do not trim bad windows after capture. The complete protected history/window/future must pass all existing numerical/sensor/ACK/body checks; independently report whole-recording failures outside that task scope. `task_complete` must remain distinct from `goal_reached=false`, route success, and training admission. A fresh schema/qualification receipt and explicit authorization are required before any conversion; old denial markers cannot be removed or overridden. The current Low-render capture does not satisfy the existing moving qualifier's Epic configuration, another reason not to reuse its receipt path.

A prospective scope-specific quality record should include the exact task-plan/source/route/map/actor/sensor hashes, every raw native and camera identity, fixed membership and protected support boundaries, original whole-run outcome, separate per-scope gate results, and immutable exclusions outside the declared label scope. Fresh PENDING_QA status is not approval. This task would supply cruise context, not a start/stop demonstration or new turn capability. W1's actual 20 s improvement supports prioritizing longer unchanged-model validation before another fit.

## Reproduction and hashes

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.audit_town07_failed_episode_windows_20261001 -- --run storage/experiments/validation/2026-10-01/town07_low_speed_start_expert_v2 --raw-audit docs/assets/validation/2026-10-01/town07_low_speed_start_expert_v2/raw_audit --output /new/nonexistent/window_census_output
```

Tool SHA256 `f25c410ca3e43126167d04f32e7e4dbc5297eae2e409a401ea6b6e61d2fc8c92`; summary SHA256 `9bae8faa7d00d91feda6b5b2035b901739feabb1dd705a844a9de87a46b1256d`. Two focused tests pass. Summary binds original metadata, helper sources, and all 5,670 JPEG hashes. `all_windows.jsonl.gz` preserves every candidate; `all_native_geometry_and_bounds.jsonl.gz` preserves every native geometry/gate result.
