# V2 paired capture: raw-target attribute support

<!-- HH_261001 - Preserve the actual v1 failure; do not invent unsupported camera state. -->

The first Town01 paired v1 owner exited 1 after the collector found a missing
enable_postprocess_effects attribute on a non-RGB camera. Both RGB cameras
were successfully spawned with exact false/true settings, but zero frames
were captured. Actor/world cleanup and the owner lifecycle completed; sources
and original failed receipts are unchanged.

V2 still requires the attribute on RGB and sets false/true explicitly. For
depth/semantic sensors it reproduces the archived TRAIN collector's conditional
behavior: set false only when the attribute is exposed. Otherwise leave it
unset, record supported=false/applied_value=null, and archive actual blueprint
and actor attributes. This is not a claim that an unsupported setting is false.
Raw depth/semantic bytes are shared identically between both RGB arms.

The geometric calibration, route scope, four-camera synchronization, 240 frames,
hashes, replay, lifecycle and admission gates remain unchanged. New worker,
owner, auditor and tests carry v2 filenames. The failed v1 files and its unused
pre-prediction evaluator registration remain historical records.
