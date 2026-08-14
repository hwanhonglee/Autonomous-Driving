# PC1 bag recovery provenance

## Failure

Power loss interrupted rosbag2. DB0–DB12 were complete. DB13 declared 12,002 pages of 4,096 bytes but contained only 10,431 pages, so 1,571 pages (6,434,816 bytes) were never present in the file. There was no WAL, SHM, or journal. The corrupt DB13 SHA-256 was `edc3e616b87c345a70ceca49624236f94f1c255a008ba4e504e39f0889b216dd`. DB14 was empty.

## Recovery boundary

SQLite `.recover` produced 111,309 normal messages plus `lost_and_found` records. Exactly 7,899 records met all of these conditions:

- four-field message rows with a non-null ID;
- IDs 111310 through 119208, unique and contiguous;
- valid topic IDs referencing the recovered `topics` table;
- integer timestamps;
- non-empty BLOB payloads.

Another 840 two-field records were timestamp-index cells, not messages, and were not promoted.

## Validation

The publication DB13 is a separate recovered candidate, not an in-place repair of the corrupt original. After promoting the 7,899 valid continuation rows:

- message rows: 119,208;
- IDs: 1 through 119208, with no gap or duplicate;
- invalid topic references: 0;
- timestamp-index rows: 119,208;
- `PRAGMA quick_check`: `ok`;
- selected-message semantic deserialization errors: 0.

All 14 publication DB files subsequently returned `quick_check: ok`, and rosbag2 metadata was rebuilt. The final extended bag contains 2,184,011 messages over 825.258834 s.

## Irrecoverable tail

The final approximately 14.7 seconds and roughly 39,000–40,000 messages never reached persistent SQLite pages and cannot be recovered from this PC1 disk image. Sidecar logs also end without a graceful stop marker.

## Publication policy

The publication includes the validated recovered bag. To avoid duplicating large and unusable artifacts, it does not include the corrupt original, the empty DB14, the conservative intermediate DB13, or the textual `.recover` dump. Those originals remain preserved locally on PC1.

For primary paper statistics, DB0–DB12 are the conservative window. DB13-derived results may be used only with this recovery disclosure and the published integrity checks.
