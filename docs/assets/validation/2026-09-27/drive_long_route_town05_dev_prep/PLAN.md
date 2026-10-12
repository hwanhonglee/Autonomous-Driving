# Town05_Opt long-turn provisional DEV route search

<!-- HH_260906 - Pre-register a catalog-only site search before opening an owned CARLA process. -->

**Status: SEARCH COMPLETED, NO MATCH, NO DATASET ADMISSION.** The next driving evaluation needs a turn at a site other than the Town01 expert TRAIN candidate. Town03's new long right route was geometrically identical to its existing DEV route. The archived Town05_Opt right-turn catalog contains eight 77.6–83.9 m routes, all below the fixed 9 km/h 30 s moving-anchor plus 6.4 s future distance budget of 121 m with a 30 m pre-turn lead. The 200–230 m search below tested whether a suitable new *candidate* exists; it did not certify a driving episode or a learned model.

**Observed result:** [owner result](../../../../../storage/experiments/validation/2026-09-27/drive_long_route_town05_opt_dev_catalog_v1/owner_result.json) recorded exit code 1 and `catalog_status=blocked`; the [catalog](../../../../../storage/experiments/validation/2026-09-27/drive_long_route_town05_opt_dev_catalog_v1/route_catalog.json) has **0 left and 0 right accepted routes** after 199,536 attempted traces (99,768 per scenario). There were 0 planner errors; 146,372 scenario mismatches, 31,912 rejected endpoints, 19,038 route-length rejections and 2,206 physical-turn rejections across both scenarios. All 13 archived source hashes stayed equal and ready/after_catalog/stopped receipts passed with ports released. The 20-minute worker limit was not an authorization to repeat failed search. No CARLA drive, camera frames, dataset admission or learned training occurred. The prepared [offline ranking gate](../drive_town05_opt_long_turn_rank/README.md) correctly refuses to publish a rank for this incomplete/blocked catalog.

Town05_Opt is **provisional integrated DEV only** for this search. `tools/e2e/inventory_carla_training_maps.py` still records the older global `town05_opt: train` split, while the newer integrated-map plan designates Town05_Opt as DEV. No dataset split is changed by this catalog. Before any candidate can be used for a held-out model evaluation, audit all driving and map TRAIN inputs/checkpoint provenance against Town05_Opt and lock the integrated site split in a machine-readable contract. If exposure cannot be ruled out, this catalog cannot be called independent DEV.

## Fixed owned search

- Entry point: `bash tools/e2e/run_owned_carla_turn_route_catalog.sh town05_opt`, after a separate free-port/process/runtime-lock preflight.
- Only packaged `Town05_Opt` on 127.0.0.1:2100, quality `Epic`, no `client.load_world`, no existing CARLA reuse, no GPU/SSH training or system installation.
- New output: `storage/experiments/validation/2026-09-27/drive_long_route_town05_opt_dev_catalog_v1`, reject if it exists; preserve all failure evidence and source bytes.
- Fixed generator: left/right, seeds 1 and 2, 8 pairs/seed, route length 200–230 m, preferred 210 m, 1 m sampling, `speed_30kph` physical-turn profile, 100000 trace cap. The profile is geometric and **does not authorize 30 km/h driving**; any eventual raw episode starts at the separate 9 km/h expert-control contract.
- Required owner checks: source archive/hash, ready/after_catalog/stopped lifecycle, `route_catalog.json` complete, at least one right route, no collection/training, ports released. Failure means no route candidate approved.

## Candidate admission beyond the catalog

Do not count seed variants as independent sites. Compare road/lane turn transition and coordinates with all older Town05 and existing TRAIN/DEV routes; reject loops, extra turns/lane changes, unsafe endpoints, insufficient 30 m pre-turn/6.4 s future tail, and site leakage. Only after a specific route and episode plan is frozen may an owned BasicAgent capture occur. That raw episode must separately pass 10 Hz six-camera synchronization, 20 Hz native-state and bbox/OpenDRIVE provenance, 30 s uninterrupted moving anchors with full measured 6.4 s future, unchanged motion bounds, collision 0, and 10 cm inset intended-lane body/sweep checks. **Catalog success is not raw QA success, raw QA success is not learned closed-loop success.**
