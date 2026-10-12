# Typed unsupported-seam rerun

<!-- HH_260929 - Preserve the first failed attempt and freeze only its exception classification. -->

The first preregistered execution stopped without writing `report.json`. The archived nominal-body helper raised `audit_future_actor_lane_containment.Unsupported` when two authorized route lanes had a gap wider than its existing **5 mm** endpoint tolerance. The v1 scanner caught `ValueError`, `RuntimeError`, and `KeyError`, but `Unsupported` inherits directly from `Exception`.

This v2 run retains the exact Town12 XODR, source API, planner, six endpoint scopes, original physical profile, fixed window, 146 m length, archived Prius dimensions, and 10 cm inset. Its only change is to record that typed exception as `UNSUPPORTED` for the affected route and continue auditing the remaining physical-profile PASS routes. No seam tolerance or geometry threshold changes. The original [preregistration](preregister.json) and v1 script remain intact. [V2 preregistration](preregister_v2.json) pins the wrapper and the original inputs before a new `report_v2.json` is written. Static results do not authorize collection or dataset admission.
