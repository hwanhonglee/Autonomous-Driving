# VILS field-result index

Store reviewable result summaries under:

```text
results/<PC>/<UTC-start>-<run-id>/
```

Each run directory must state whether it is a completed acceptance run, partial evidence, or a
failed/aborted attempt. A `COMPLETE` marker from an individual recorder is not, by itself, a
four-PC acceptance PASS.

Large rosbag, pcap, imagery, and raw runtime logs remain in the owner-controlled evidence store.
Git retains small manifests, parsed metrics, and checksums that identify those external artifacts.
