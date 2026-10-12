# Town05_Opt right p00 Epic: frozen map A/C DEV comparison plan

Status at registration: **no inference or score from this right-turn capture has been opened**. This plan fixes the inputs, denominators, metrics, selected images, and interpretation before running either checkpoint. The output directory is new and must never replace capture or earlier evaluation assets.

## Pinned inputs

| Role | Exact source and SHA-256 |
|---|---|
| Right p00 Epic capture | `storage/experiments/training/2026-09-27/map_town05_right_sign_dev_scout_epic_v1/mapping/`, audited as `docs/assets/validation/2026-09-27/map_town05_sign_dev_scout/summary.json` `201d9e3dc6a7ecff46da485682afcc73682cdb3d045b7579920d6b94670d5050` |
| Route | `town05_opt_right_s0000_p00.json` `2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3` |
| Capture owner plan / mapping summary | `caf6c1d24819b08d3829e80d893e03f4ebe2903245336044d6b3aa1f57afd8a7` / `9c6b66385164a7df5dbb85ba973047e89ccb7d44cbfea1ba84f28952674686f8` |
| A: frozen AI v2 | `storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_ai_v2_checkpoint/checkpoint.pt` `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380` |
| C: rejected rehearsal candidate | `docs/assets/validation/2026-09-27/map_town_rehearsal_c/remote_run_v2/checkpoint.pt` `864de725655ef7219218f2b4a9da059630e6e3c30a18a010f74238938aa4a7f6` |
| C run receipts / archived model source | `started.json` `7e11781ab16240543e93a475a39e571fd0062279cadf447c0eaecb87712811be` / `summary.json` `37d6ab4efcf05ba8bd604bf888314dd07d9d1a8d5d3ee3fed7f632d9fa82eac3` / `source/map_perception.py` `1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b` |
| Model / reusable evaluator | `learning/portable_e2e/map_perception.py` `1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b` / `tools/e2e/evaluate_map_town05_dev.py` `ca06d9add59695e5e668147fc3600e9e068635d6599b9249a6d6c4b6d06462ef` |
| Capture audit code / shared audit helper | `tools/e2e/audit_map_town05_sign_dev_scout.py` `8ab683faf85c096724f803c7dec28e0391a18a22468d1c2490af9d9152b63363` / `tools/e2e/audit_map_generalization_town03.py` `7d2d50ccc9191da44c50162f924c1cbbccfbf9eb06dbac88a7e124fd7e5dc81a` |

The audit reports a complete 240-pair native 640×360 RGB/semantic capture, 195 frames retained by distance ≥0.25 m from every earlier retained ego XY, Town05_Opt/Epic/right p00/ClearNoon provenance, zero collisions, exact live/replay match, source archive checks, and a clean owner shutdown. Before inference, recompute the complete audit and require exact equality with its pinned JSON. Check every frame SHA during inference. Both checkpoint bytes and model source must match the hashes above; load only on CPU and strictly check the seven-label ABI, input shape, 2,000 steps, `deployable=false`, and parameter count 130,503. Run with `CUDA_VISIBLE_DEVICES=''`, two Torch CPU threads, no crop, resize, augmentation, threshold, or postprocessing. Semantic tags are used only for truth: `{1:road, 24:lane_marking, 2:sidewalk, 7:traffic_light_fixture, 8:traffic_sign, 6:pole}`; all other tags become `other`.

## Frozen measurement

The **primary denominator is the 195 retained frames**; report all 240 raw frames separately. Sum seven-class native-pixel confusion matrices with truth on rows and prediction on columns. Report road and lane-marking IoU, precision, recall, TP/FP/FN and C−A IoU in percentage points. Include road truth predicted as `other` or sidewalk and lane truth predicted as `other` or road. Seven-class mean IoU and pixel accuracy are context only. The same continuous route is one correlated development sequence, not 195 independent trials or a blind TEST.

Select retained ordinal **0, 97, 194** before predictions: `frames/0000`, `frames/0105`, `frames/0203`. Each full-FOV figure must show unaltered 640×360 RGB, CARLA semantic oracle, A prediction, and C prediction. Save both models' native masks for all 240 frames plus the retained indices; independently reconstruct confusion matrices from that artifact and the pinned source after inference.

The scout **failed its sign-capable DEV gate**: 57 tag-8 truth pixels across retained frames and at most 3 pixels in any retained whole frame. Sign pixels may appear as a clearly labeled incidental diagnostic, but **sign IoU, precision, recall, or false positives cannot enter this comparison's gate or rank a model**. They are not distinct object-detection counts. This evaluation is for road/lane diagnostics only.

The directional road/lane diagnostic flag is true only if C road IoU ≥ A−1.0 pp **and** C lane-marking IoU ≥ A−2.0 pp on the 195 retained frames. It has no promotion effect. C already failed the pre-existing Town01 DEV protection gate (lane −2.528 pp and sign IoU/precision regressions), so `automatic_promotion=false` and `deployment_approved=false` regardless of Town05 outcome. This route also cannot approve an HD map, learned-control driving, or a real vehicle. No CARLA, GPU, SSH, installation, training, or held-out Town07/other TEST access is part of this work.
