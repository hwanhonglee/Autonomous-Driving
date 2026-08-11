# 06 — Duplicate node, publisher, and TF analysis

Evidence: `source-derived` plus `runtime-verified-PC1-local-five-cycles`; four-PC counts unverified

The highest-risk paths are the separate PC1 planning simulator and SocketCAN aliases; historical
PC2/PC3 overlapping sensing; sensor-model launch plus manual driver launch; multi-PC RViz; PC2 and
PC4 simultaneous predicted-object publishers; multiple bridge instances; and duplicate static TF.

`/system/operation_mode/state` must not be assigned to PC3 merely to fill the current gap. Source
ownership is PC1 control. The isolated PC1 graph measured publisher count 0 and subscriber count
6 while control was intentionally absent in the fail-closed phase.

PC1's five isolated runs each had 47 nodes and zero duplicate FQNs. This does not clear
simultaneous cross-PC duplicate publishers, renamed duplicate functionality, or static TF risks.

Detailed local risks: [PC1_duplicate_analysis.md](PC1_duplicate_analysis.md).
Runtime tables: [duplicate_nodes.md](duplicate_nodes.md),
[duplicate_publishers.csv](duplicate_publishers.csv), and [duplicate_tf.md](duplicate_tf.md).
