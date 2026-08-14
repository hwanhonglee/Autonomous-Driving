# PC1–PC3 shared-run linkage

The shared run ID was assigned retrospectively from serialized-payload evidence,
not from wall-clock overlap alone. The files were not concatenated or merged.

## Compared streams

| Item | PC1 | PC3 |
|---|---|---|
| Topic | `/perception/pc4/virtual_obstacles/tracked_objects` | same |
| Type | `autoware_perception_msgs/msg/TrackedObjects` | same |
| Input | validated SQLite DB0–DB13 | `pc3_crosslink_20260814T0924Z_0.mcap`–`_5.mcap` |
| Total messages | 4,415 | 3,204 |

The comparison used the raw serialized CDR bytes stored by rosbag. It found the
following longest contiguous equality:

```text
PC1 PC4-topic suffix[1306..4415]
  byte-for-byte ==
PC3 PC4-topic prefix[1..3110]
```

All 3,110 common payloads were byte-identical and in the same order. Duplicate,
reorder, and mismatch counts were all zero. PC1 had 1,305 earlier non-common
payloads and no later payload; PC3 had no earlier non-common payload and 94 later
payloads. Every per-message CDR SHA-256 was unique within each stream. The common
serialized payload size was 2,459,004 bytes and the PC1 portion spans DB8–DB13.

## Aggregate digests

```text
SHA256(concat(binary_SHA256(each_CDR)))
= 5a9c206fd41bb6af913fb27f39b642f263fdd86bcb4be0644998973b9671591a

SHA256(concat(uint64_be(CDR_length) || CDR))
= c3156261d4ffaadace5eb09c5fd4c06b186c85e4ee55e17ffab036aa8cc2ebca
```

## Boundary observations

| Boundary | PC1 record time | PC3 record time | CDR SHA-256 |
|---|---|---|---|
| First common payload | `2026-08-14T18:24:30.028564455+09:00` | `2026-08-14T18:24:30.030118368+09:00` | `1c28902dd8098a3faa844d9194b63c890ec07f5e0f7ca2aa844f74168e7d1eaa` |
| Last common payload | `2026-08-14T18:29:42.068291005+09:00` | `2026-08-14T18:29:42.070603148+09:00` | `78b2eec0860d1f15ca4d1090e5bccea520e2eb526bc5a62869ffcf9ee6bf2606` |

The PC1 PC4 stream began at `18:22:18.264469964 KST`; its final payload was the
last common payload. The PC3 stream continued for 94 messages and ended at
`18:29:51.570444666 KST`.

## Read-only comparison method

1. Open PC1 split SQLite files in `metadata.yaml` order and read the selected topic
   with `ORDER BY timestamp,id`.
2. Read the PC3 topic through rosbag2's MCAP reader in metadata file order
   (`_0`–`_5`). MCAP lacked a message index, so the reader used chunk/file order.
3. Compute SHA-256 for every raw CDR and find the maximum `PC1 suffix == PC3
   prefix` length.
4. Compare the raw CDR bytes again at every aligned position and compute the two
   aggregate digests above.

This proves payload-level linkage between the PC1 and PC3 result packages and
supports their common run ID. It does not authenticate the original PC4 host,
prove PC2 acceptance, or establish cross-host one-way latency. Although the first
and last PC3 record times are 1.553913 ms and 2.312143 ms later than PC1,
respectively, no accepted common clock was established, so those differences must
not be reported as transport latency.
