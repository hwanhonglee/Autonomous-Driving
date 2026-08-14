<!-- HH_260810 - Explain the curated PC4 evidence subset committed to Git. -->
# PC4 source-evidence subset

<!-- HH_260810 - Preserve source provenance while excluding oversized rosbag databases. -->
This directory contains review copies of small files from the read-only PC4 evidence root
identified in the parent `README.md`. The original `SOURCE_ARTIFACTS.sha256` manifest covers 65
files, including the two rosbag databases that are too large for ordinary GitHub storage.

<!-- HH_260810 - Distinguish Git review copies from the authoritative byte-level originals. -->
The original evidence root and manifest remain authoritative for byte-level verification. Most
copies here are byte-identical. Git-oriented text creation added one final newline to the two
`metadata.yaml` files and normalized CRLF to LF in the two metric CSV files; `diff
--strip-trailing-cr` shows no textual row difference. No scientific values were changed.

<!-- HH_260810 - Index the curated evidence used to audit the summary. -->
Included here:

- capture and completion manifests
- Domain 10 and Domain 42 rosbag metadata
- the original 65-file SHA-256 manifest
- periodic Chrony, NIC, and host metrics
- adapter snapshot metadata
- active endpoint audits for both domains
- CARLA pre-capture inventory
- scope limitations
- five managed-process lifecycle records

<!-- HH_260810 - Prevent omitted raw data from being mistaken for missing provenance. -->
The omitted databases remain on PC4 and are identified by filename, size, and SHA-256 in the
parent report and `SOURCE_ARTIFACTS.sha256`. This Git review subset supports topic counts,
QoS, process ownership, clock state, link behavior, and artifact provenance; deserialized and CDR
statistics in `summary.json` and `metrics.csv` were computed from the preserved local databases.
