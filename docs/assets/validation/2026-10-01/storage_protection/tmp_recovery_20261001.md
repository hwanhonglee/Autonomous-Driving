# HH_261001 — Reversible cleanup of two September `/tmp` files

<!-- HH_261001 - Record exact reversible temporary-file actions separately from research evidence. -->

No experiment source, result, model checkpoint, dataset, or unreviewed `/tmp`
clone was removed in this operation. These were old temporary artifacts owned
by the local user and not opened by a live process when checked.

| Old local target | Preservation / action | Reversal |
| --- | --- | --- |
| `/tmp/tmp.xTOkyRkkoN/torch-2.13.0+cu130-cp312-cp312-manylinux_2_28_x86_64.whl` (526,482,318 bytes) | Copied to the personal PRO6000 `~/personal/hwanhong/portable_e2e/cache/wheels/` path with matching source/remote SHA256 `8db7338e6895c3d4bd89a02ff4209507d1f0cf2ffeb3b898538b5a07d1ea8c1e`; then removed only this local wheel and its empty temp directory. No package was installed. | Copy the exact remote wheel back to a new local directory and compare SHA256. |
| `/tmp/autoware-preflight.strace` (219,334,940 bytes) | Compressed in place to `/tmp/autoware-preflight.strace.gz` (4,821,703 bytes). `gzip -t` passed; decompressed SHA256 remained `2a23b89aaa0bfddeb2959e853a21a8829d540542b37e757a2fed2ea0f7dba853`. | `gzip -cd` into a new filename and compare SHA256; leave the compressed original until no longer needed. |

The four modified `/tmp/carla_contract*` repository copies were **not**
removed. They are not interchangeable with either artifact above. A separate
user choice is required before backing up/removing those copies for further
simulator disk capacity.
