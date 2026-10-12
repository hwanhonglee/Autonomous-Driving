# Same-map reuse during the six-scope search

<!-- HH_260929 - Preserve V1/V2 and freeze a performance-only rerun. -->

V2 remains running with the exact source, split, scopes and thresholds in its preregistration. At the 45-minute read-only review it was CPU-active with no report or error. Its nominal-body helper constructs a new `carla.Map` from the same 71 MB Town12 XODR for each physical-profile PASS route. This v3 run supplies one cached `carla.Map` instance to those calls, rejecting every request unless its UTF-8 XODR bytes hash to the pinned source SHA256. The map's geometric content and all helper code are unchanged. V3 writes a separate `report_v3.json`; the v2 process and its output path remain untouched.

The [v3 preregistration](preregister_v3.json) pins the wrapper and prior code before execution. This is an execution optimization only: six scopes, 1 m planner, original shape profile, 30–120 m body window, 10 cm inset and 5 mm lane-seam tolerance are identical. On completion, compare counts and every physical-pass route SHA256 with v2 when v2 also completes. Do not treat a v3 result as an independent site or as capture/dataset approval.
