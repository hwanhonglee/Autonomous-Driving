# 09 — Chrony

Evidence: `runtime-verified-PC1-only`

Chrony was active and synchronized on PC1. Configuration includes `server 192.168.9.7 iburst`
plus public Ubuntu pools, but the selected source at audit time was public `211.108.117.211`, not
`192.168.9.7`. PC1 is a client; no `allow`/`local` server role was proven.

The observed clock offset was small, but source selection and topology are not deterministic for
the four-PC experiment. Decide whether PC3 is the internal server candidate, then record
`chronyc -n tracking` and `chronyc -n sources -v` on all PCs. This is a readiness blocker, not a
reason to change time configuration during this local patch.
