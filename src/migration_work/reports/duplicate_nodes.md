# Duplicate node audit

Evidence: `runtime-verified-PC1-local-five-cycles`; `runtime-unverified-4pc`

The isolated PC1 fail-closed profile reported 47 nodes and zero duplicate fully qualified node
names in each of five cycles. Forbidden control, SocketCAN, adapter, parking, and RViz-adaptor
nodes were also zero. Post-stop node count was zero in every cycle.

No simultaneous four-PC node snapshot has been taken. Required integration method:

```bash
ros2 node list | sort | uniq -d
ros2 component list
```

Also correlate process command lines and component containers; identical functionality under a
renamed node still violates ownership. Critical expected singleton roles are PC1 planning/API,
PC2 perception, PC3 map/localization/TF/system/common sensing, and PC4 bridge.
