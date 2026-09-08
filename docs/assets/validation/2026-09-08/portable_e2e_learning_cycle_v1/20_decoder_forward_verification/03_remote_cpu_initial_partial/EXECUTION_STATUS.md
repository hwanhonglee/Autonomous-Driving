# Partial CLI execution: numerical audit retained, rendering unavailable

<!-- HH_260906 - Preserve the remote CPU publication failure without installing packages or reclassifying numerical disagreements. -->

The remote existing personal py312 venv ran the exact released independent auditor at repository commit dfccaa8ed9f00295ebd2014f4191671b7fa98ddb with CUDA hidden and four CPU threads. Numerical audit output was produced at 2026-09-07T20:41:14.390588+00:00 using Torch 2.13.0+cu130.

The CLI exited 1 during `write_result -> render -> import matplotlib`, with `ModuleNotFoundError: No module named 'matplotlib'`. The already written summary.json, per_anchor_numeric.jsonl and input_verification.json were retained and copied byte-for-byte. README, PNGs and the normal output SHA256SUMS were not generated remotely. This is not a completed publication command. No installation or automatic retry was performed.

All six initial latent SHA values match; 22 initial objective comparisons remain outside the unchanged absolute/relative 1e-5 tolerance, maximum absolute difference 8.252372483585901e-05 m². Matching Torch versions did not remove these recorded differences and does not establish a physical or numerical cause. The original GPU candidate failures, source bytes and input data are unchanged.
