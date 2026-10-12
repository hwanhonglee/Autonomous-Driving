# Turn-centered raw QA window binding — preparation only

This change prepares the raw camera-anchor selector for a future multi-maneuver
route. **It does not audit, admit, capture, train, or approve a new episode.**
The existing Town01 p06 and B/C call sites pass no new arguments, so their
original independently projected **30–120 m** selection, first native
`>120 m` closure, and all downstream gates remain unchanged.

The optional `select_window(..., route_source_bytes=...,
expected_route_sha256=..., window_report_bytes=...,
expected_window_report_sha256=...)` path requires all four inputs. The two
expected SHA256 values must come from an independently verified owner/catalog
and preregistered selector-report record; a caller must not calculate a pin
from the bytes it is about to inspect and treat that as external provenance.
The function verifies both byte hashes, the decoded raw route against the
in-memory right/base_link route, selector status and route binding, the complete indexed RIGHT
command block, a fixed 90 m window, 30 m approach/post-turn context, and the
9 km/h × 6.4 s future distance plus 10 m route-end guard. It also independently
rejects another maneuver in the protected window/future and a left turn or
lane change anywhere on this right-route search. Any partial,
unbound, changed, or geometrically inconsistent input raises `AuditError`.

On a valid bound window, every driving camera anchor inside the fixed
progress interval is selected. The first native state beyond the specified end
closes selection permanently, even if the vehicle later backtracks. The same
native/camera continuity, 30 s span, >0.5 m/s speed, progress-monotonicity,
turn entry/exit, RIGHT command, and observed yaw gates run on that entire
selection. No post-hoc anchor exclusion was introduced. The future-point,
nine-bound, ACK, RGB, collision, and intended-lane/body gates remain in the
full auditors and have **not** been declared passed by this selector work.

The optional output uses `first_over_window_end_closes_selection` and
`first_progress_over_window_end_m`; the original p06 output keys remain
`first_over_120_closes_selection` and `first_progress_over_120_m`.

Before this can become a complete new raw QA, a separate owner-bound auditor
must pin the new trial, route, source archives, map, sensor settings, selector
report, and output directory; verify the full native episode and all future
points; measure intended-lane body and sweep containment; and run the actual
moving-time and motion-bound gates. The current full `audit()` and CLI remain
hard-pinned to Town01 p06 and must **not** be run with an alternate window as
if they represented a multi-route audit.

CPU-only regression: `python3 tools/workspace_python.py -m pytest -- -q
learning/tests/test_audit_long_turn_9kph_raw.py
learning/tests/test_audit_long_turn_steer045_ab.py
learning/tests/test_audit_long_turn_purge15_bc.py` — **22 passed** on
2026-09-27. No CARLA server, GPU, SSH, package install, or training was used.
