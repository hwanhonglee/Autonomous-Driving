# Proposed bounded V4 only if V3 stops without a complete report

<!-- HH_260929 - Identify repeat work without changing the running V3 or its static gates. -->

Status: `DESIGN_ONLY_NOT_RUN`. The pinned V3 process continues. This proposal does not authorize stopping it, executing V4, or using a partial progress log as a route result.

## Observed call path

For every route that passes the unchanged physical shape profile, `rank_town01_v5_long_turns.provisional_nominal_clearance` calls `audit_future_actor_lane_containment.lane_corridors`. That function unconditionally calls `_section_bounds(xml)`, which runs `ElementTree.fromstring` over the entire 71,350,074-byte Town12 XODR and builds all road/section bounds. V3 reuses the `carla.Map` object, but does not cache this XML parse. The same helper also builds 0.25 m centerline search banks for requested sections, 0.05 m lane corridor tessellation, and a 0.10 m body/sweep grid over 30–120 m (901 body poses, 900 sweeps). Only the repeated pure `_section_bounds` result is proposed for caching; route-dependent banks, corridors, bodies and sweeps remain recalculated.

## Frozen V4 scope if a rerun is needed

Pin a new script, preregistration and output path before execution. Use the same source/packaged XODR SHA256 `26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f`, source CARLA egg, 1 m planner, all right-heading driving connectors, and exactly `(60,100), (60,120), (70,100), (70,120), (80,100), (80,120)` m endpoint scopes. Keep the original 146 m route minimum, one RIGHT block bound to the exact connector, 30–120 m window, original `carla_vad_30kph_v2` shape profile, nominal Prius dimensions, 10 cm lane inset, and 5 mm seam tolerance. Continue to classify typed `Unsupported` as `UNSUPPORTED`. Do not read TEST maps, contact CARLA/UE4, capture, or train.

The proposed cache accepts only the pinned XODR bytes and stores the unchanged `_section_bounds` mapping for reuse. It must not change the `lane_corridors` body calculation or permit a different XML payload. A focused test should prove one parse for repeated identical bytes, rejection of changed bytes, and identical section bounds to the original function on the pinned Town12 XODR. A dedicated V4 scanner must emit a separate append-only progress ledger after each **completed** junction with junction ID, completed/total junction count, cumulative stage counts and a digest of accumulated physical-pass route hashes. A final `report_v4.json` may be written only after all junctions complete and every physical-profile PASS receives PASS/FAIL/UNSUPPORTED body status. Partial ledger rows are operational progress only, never evidence of route qualification.

The V4 preregistration should pin the V1–V3 scripts/plans, V2 stop evidence, exact V3 terminal state, physical/nominal helper hashes, source body manifest, split references, new code and new output names. If V3 completes, compare stage counts, physical-pass route SHA256 sets and every nominal status/metric before using V4. If V3 stops without a report, preserve its PID/exit/log/output evidence first. The old suite's `train` label remains a provisional scout role; the integrated site split guard permits Town12 only `offline_train_scout`, with capture, dataset admission and independent DEV credit false.
