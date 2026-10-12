# Class-weight wording correction — before geometry fitting

<!-- HH_261001 - Preserve immutable plans while correcting a mistaken label, not the executed class weights. -->

The earlier inventory/proposal and frozen `reference_plan.json` describe the weights as “original Town01.” That wording is inaccurate for the actual preceding appearance and photometric runs. Inspection of the immutable appearance package builder, manifest, and photometric `fit/started.json` confirms their weights were computed once from **original729 + paired480 targets**, then fixed identically in both arms.

The geometry study retains those **actual preceding values unchanged**, as parent explicitly confirmed before any geometry fit:

| Class | Exact float32 value represented in JSON |
| --- | ---: |
| other | 0.13594788312911987 |
| road | 0.13577374815940857 |
| sidewalk | 0.2083463817834854 |
| lane_marking | 1.4355568885803223 |
| traffic_light_fixture | 1.9548143148422241 |
| traffic_sign | 2.183657646179199 |
| pole | 0.9459031224250793 |

Evidence: base manifest `32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8`; new actual remote manifest `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d` copies the same values. Frozen remote packager already used these exact base values. Neither the historical artifacts nor frozen plan/packager is modified. The new trainer and separate fit authorization pin this vector explicitly and **never recompute it from geometry480 labels**. This is a provenance wording correction, not another experimental factor.
