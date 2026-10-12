# Fixed signal-record correction and nominal witness

<!-- HH_260929 - Preserve v1 evidence and correct the OpenDRIVE signal screen before final audit. -->

The first immutable [report](report.json) counts only OpenDRIVE `signalReference` elements. A read-only inspection of its Town05_Opt junction 396 route found a direct `<signal>` on road 45, which that column missed. The v1 column means **reference-free**, not signal-free, and cannot authorize collection. This v2 rerun keeps the same maps, 30 endpoint scopes, route length, 30–120 m window, unchanged physical-turn profile, future/goal criteria, and no-server boundary. It adds direct `<signal>` records to each physical-profile PASS route and writes a new `report_v2.json`, leaving v1 intact.

The v1 route at Town05_Opt junction 396, road 444/lane +1, `previous(70m)[0] → next(100m)[0]` was the only distinct connector with no signal **reference** and no other junction in the protected span outside Town03's existing DEV junction. V2 additionally runs the existing nominal Prius body and 10 cm inset sweep calculation on exactly that route, using the pinned Town01 archived Prius bounding box as a cross-map geometry hypothesis. That is not a live Town05 actor or XODR match and cannot replace driven-body QA. A failure remains a failure; no geometry threshold is changed or failing arc removed.
